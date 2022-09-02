/**
 * @file channel_sim.cc
 * @brief Implementation file for the channel simulator class
 */
#include "channel_sim.h"

#include <atomic>
#include <utility>

#include "datatype_conversion.h"
#include "gettime.h"
#include "logger.h"
#include "message.h"
#include "signal_handler.h"

static std::atomic<bool> running = true;
static constexpr bool kPrintChannelOutput = false;
static const size_t kDefaultQueueSize = 36;
static const bool kPrintDebugTxUser = false;
static const bool kPrintDebugTxBs = false;

static constexpr size_t kUdpMTU = 9000;
static constexpr size_t kDequeueBulkSize = 5;
static constexpr size_t kSockBufSize = (1024 * 1024 * 64 * 8) - 1;

//#define CHSIM_DEBUG_MEMORY
/* Helper classes */
class SocketRxBuffer {
 public:
  SocketRxBuffer(size_t num_bytes) : data_size_(0), data_(num_bytes) {}
  ~SocketRxBuffer() = default;

  inline size_t DataSize() const { return data_size_; }
  inline size_t StorageSize() const { return data_.size(); }
  inline std::byte* Rx() { return &data_.at(data_size_); }
  inline const std::byte* At(size_t location) const {
    return &data_.at(location);
  }
  inline void AddData(size_t data) {
    data_size_ = data_size_ + data;
    if (data_size_ > data_.size()) {
      throw std::runtime_error("SocketRxBuffer overflow");
    }
  }
  inline void RemoveData(size_t remove) {
    if (remove > 0) {
      if (remove > data_size_) {
        throw std::runtime_error("Requested removal of too many bytes");
      }
      data_size_ = data_size_ - remove;
      if (data_size_ > 0) {
        std::memmove(&data_.at(0), &data_.at(remove), data_size_);
      }
    }
  }

 private:
  size_t data_size_;
  SimdAlignByteVector data_;
};

ChannelSim::ChannelSim(const Config* const config, size_t bs_thread_num,
                       size_t user_thread_num, size_t worker_thread_num,
                       size_t in_core_offset, std::string in_chan_type,
                       double in_chan_snr)
    : cfg_(config),
      bs_thread_num_(bs_thread_num),
      user_thread_num_(user_thread_num),
      bs_socket_num_(config->NumRadios()),
      user_socket_num_(config->UeNum()),
      worker_thread_num_(worker_thread_num),
      core_offset_(in_core_offset),
      channel_type_(std::move(in_chan_type)),
      channel_snr_(in_chan_snr) {
  // initialize parameters from config
  ::srand(time(nullptr));
  dl_data_plus_beacon_symbols_ =
      cfg_->Frame().NumDLSyms() + cfg_->Frame().NumBeaconSyms();
  ul_data_plus_pilot_symbols_ =
      cfg_->Frame().NumULSyms() + cfg_->Frame().NumPilotSyms();

  rx_buffer_bs_ = std::make_unique<ChSimRxBuffer>(
      ChSimRxBuffer::ChSimRxType::kRxTypeBeaconDl, cfg_, kFrameWnd,
      dl_data_plus_beacon_symbols_, cfg_->BsAntNum(), cfg_->PacketLength());

  rx_buffer_ue_ = std::make_unique<ChSimRxBuffer>(
      ChSimRxBuffer::ChSimRxType::kRxTypePilotUl, cfg_, kFrameWnd,
      ul_data_plus_pilot_symbols_, cfg_->UeAntNum(), cfg_->PacketLength());

  bs_comm_.resize(bs_socket_num_);
  ue_comm_.resize(user_socket_num_);

  task_queue_bs_ = moodycamel::ConcurrentQueue<EventData>(
      kFrameWnd * dl_data_plus_beacon_symbols_ * cfg_->BsAntNum() *
      kDefaultQueueSize);
  task_queue_user_ = moodycamel::ConcurrentQueue<EventData>(
      kFrameWnd * ul_data_plus_pilot_symbols_ * cfg_->UeAntNum() *
      kDefaultQueueSize);
  message_queue_ = moodycamel::ConcurrentQueue<EventData>(
      kFrameWnd * cfg_->Frame().NumTotalSyms() *
      (cfg_->BsAntNum() + cfg_->UeAntNum()) * kDefaultQueueSize);
  payload_length_ = cfg_->PacketLength() - Packet::kOffsetOfData;

  // Initialize channel
  channel_ = std::make_unique<Channel>(cfg_, channel_type_, channel_snr_);

  for (size_t i = 0; i < worker_thread_num; i++) {
    task_ptok_.at(i) =
        std::make_unique<moodycamel::ProducerToken>(message_queue_);
  }
  task_threads_.resize(worker_thread_num);

  // create task threads (transmit to base station and client antennas)
  for (size_t i = 0; i < worker_thread_num; i++) {
    task_threads_.at(i) = std::thread(&ChannelSim::TaskThread, this, i);
  }

  ue_rx_.Init(ul_data_plus_pilot_symbols_, cfg_->UeAntNum());
  ue_tx_.Init(dl_data_plus_beacon_symbols_);
  bs_rx_.Init(dl_data_plus_beacon_symbols_, cfg_->BsAntNum());
  bs_tx_.Init(ul_data_plus_pilot_symbols_);
}

ChannelSim::~ChannelSim() {
  AGORA_LOG_INFO("Destroying channel simulator\n");
  running.store(false);
  for (auto& join_thread : task_threads_) {
    if (join_thread.joinable()) {
      join_thread.join();
    }
  }
}

void ChannelSim::ScheduleTask(EventData do_task,
                              moodycamel::ConcurrentQueue<EventData>* in_queue,
                              moodycamel::ProducerToken const& ptok) {
  if (in_queue->try_enqueue(ptok, do_task) == false) {
    AGORA_LOG_WARN("need more memory\n");
    if (in_queue->enqueue(ptok, do_task) == false) {
      AGORA_LOG_ERROR("task enqueue failed\n");
      throw std::runtime_error("ChannelSim: task enqueue failed");
    }
  }
}

void ChannelSim::Run() {
  constexpr size_t kChanSimMasterTid = 0;
  AGORA_LOG_INFO("Starting Channel Simulator ...\n");
  PinToCoreWithOffset(ThreadType::kMaster, core_offset_, kChanSimMasterTid);

  moodycamel::ProducerToken ptok_bs(task_queue_bs_);
  moodycamel::ProducerToken ptok_user(task_queue_user_);
  moodycamel::ConsumerToken ctok(message_queue_);

  //Setup the rx threads
  //RxLoop();
  std::vector<std::thread> rec_threads;
  std::vector<ChSimRxStorage> rec_thread_storage;
  //Base station rx
  {
    // initialize bs-facing sockets
    const size_t total_sockets = cfg_->BsAntNum();
    const size_t num_threads = bs_thread_num_;
    for (size_t socket_id = 0; socket_id < total_sockets; socket_id++) {
      const size_t local_port_id = cfg_->BsRruPort() + socket_id;
      const size_t remote_port_id = cfg_->BsServerPort() + socket_id;
      bs_comm_.emplace_back(std::make_unique<UDPComm>(
          cfg_->BsRruAddr(), local_port_id, kSockBufSize, 0));
      //Create a 1:1 connection
      bs_comm_.back()->Connect(cfg_->BsServerAddr(), remote_port_id);
      AGORA_LOG_INFO(
          "ChannelSim set up UDP socket server listening to port %s:%zu with "
          "remote address %s:%zu\n",
          cfg_->BsRruAddr().c_str(), local_port_id,
          cfg_->BsServerAddr().c_str(), remote_port_id);
    }

    size_t socket_offset = 0;
    size_t sockets_per_thread = total_sockets / num_threads;
    if ((total_sockets % num_threads) > 0) {
      sockets_per_thread++;
    }
    for (size_t i = 0; i < num_threads; i++) {
      size_t sockets_this_thread = sockets_per_thread;
      if (socket_offset + sockets_per_thread > total_sockets) {
        //Grab the remainder
        sockets_this_thread = total_sockets - socket_offset;
      }

      if (sockets_this_thread > 0) {
        auto storage = rec_thread_storage.emplace_back(
            i, core_offset_ + 1, cfg_->PacketLength(), socket_offset,
            sockets_this_thread, &bs_comm_, rx_buffer_bs_.get(),
            &message_queue_);
        rec_threads.emplace_back(std::thread(&ChannelSim::RxLoop, &storage));
        socket_offset += sockets_per_thread;
      } else {
        AGORA_LOG_WARN("Not launching Bs Rx Thread %zu\n", i);
        break;
      }
    }
  }

  //User rx
  {
    // initialize client-facing sockets
    const size_t total_sockets = cfg_->UeAntNum();
    const size_t num_threads = user_thread_num_;
    for (size_t socket_id = 0; socket_id < total_sockets; socket_id++) {
      const size_t local_port_id = cfg_->UeRruPort() + socket_id;
      const size_t remote_port_id = cfg_->UeServerPort() + socket_id;
      ue_comm_.emplace_back(std::make_unique<UDPComm>(
          cfg_->UeRruAddr(), local_port_id, kSockBufSize, 0));
      ue_comm_.back()->Connect(cfg_->UeServerAddr(), remote_port_id);

      AGORA_LOG_INFO(
          "ChannelSim set up UDP socket server listening to port %zu with "
          "remote %s:%zu with remote address %s:%zu\n",
          cfg_->UeRruAddr().c_str(), local_port_id,
          cfg_->UeServerAddr().c_str(), remote_port_id);
    }

    size_t socket_offset = 0;
    size_t sockets_per_thread = total_sockets / num_threads;
    if ((total_sockets % num_threads) > 0) {
      sockets_per_thread++;
    }
    for (size_t i = 0; i < num_threads; i++) {
      size_t sockets_this_thread = sockets_per_thread;
      if (socket_offset + sockets_per_thread > total_sockets) {
        //Grab the remainder
        sockets_this_thread = total_sockets - socket_offset;
      }

      if (sockets_this_thread > 0) {
        auto storage = rec_thread_storage.emplace_back(
            i, core_offset_ + 1 + bs_thread_num_, cfg_->PacketLength(),
            socket_offset, sockets_this_thread, &ue_comm_, rx_buffer_ue_.get(),
            &message_queue_);
        rec_threads.emplace_back(std::thread(&ChannelSim::RxLoop, &storage));
        socket_offset += sockets_per_thread;
      } else {
        AGORA_LOG_WARN("Not launching User Rx Thread %zu\n", i);
      }
    }
  }

  //Give time for all the threads launch
  std::this_thread::sleep_for(std::chrono::seconds(1));

  std::array<EventData, kDequeueBulkSize> events_list;
  while ((running.load()) && (SignalHandler::GotExitSignal() == false)) {
    const auto dequeued_events = message_queue_.try_dequeue_bulk(
        ctok, events_list.data(), events_list.size());

    for (size_t current_event = 0; current_event < dequeued_events;
         current_event++) {
      EventData& event = events_list.at(current_event);

      switch (event.event_type_) {
        case EventType::kPacketRX: {
          const size_t frame_id = gen_tag_t(event.tags_[0u]).frame_id_;
          const size_t symbol_id = gen_tag_t(event.tags_[0u]).symbol_id_;
          const gen_tag_t::TagType type = gen_tag_t(event.tags_[0u]).tag_type_;
          // received a packet from a client antenna
          if (type == gen_tag_t::TagType::kUsers) {
            const size_t ue_symbol_idx = cfg_->GetPilotUlIdx(symbol_id);
            const bool last_antenna =
                ue_rx_.CompleteTask(frame_id, ue_symbol_idx);
            // when received all client antennas on this symbol, kick-off BS TX
            if (last_antenna) {
              if (kDebugPrintPerSymbolDone) {
                AGORA_LOG_INFO(
                    "Scheduling uplink transmission of frame %zu, symbol %zu, "
                    "from %zu user to %zu BS antennas, in %.3fuS\n",
                    frame_id, symbol_id, cfg_->UeAntNum(), cfg_->BsAntNum(),
                    ue_rx_.GetTaskTimeUs(frame_id, ue_symbol_idx));
              }
              ScheduleTask(EventData(EventType::kPacketTX, event.tags_[0]),
                           &task_queue_bs_, ptok_bs);
              const bool last_symbol = ue_rx_.CompleteSymbol(frame_id);
              if (last_symbol) {
                ue_rx_.Reset(frame_id);
                AGORA_LOG_INFO(
                    "Frame %zu: Finished uplink reception of %zu symbols in "
                    "%.3fuS\n",
                    frame_id, ul_data_plus_pilot_symbols_,
                    ue_rx_.GetTaskTimeUs(frame_id));
              }
            }
          } else if (type == gen_tag_t::TagType::kAntennas) {
            const size_t dl_symbol_id = cfg_->GetBeaconDlIdx(symbol_id);
            const bool last_antenna =
                bs_rx_.CompleteTask(frame_id, dl_symbol_id);
            AGORA_LOG_TRACE("Rx downlink frame %zu, symbol %zu\n", frame_id,
                            symbol_id);

            // when received all BS antennas on this symbol, kick-off client TX
            if (last_antenna) {
              if (kDebugPrintPerSymbolDone) {
                AGORA_LOG_INFO(
                    "Scheduling downlink transmission in frame %zu, symbol "
                    "%zu, from %zu BS to %zu user antennas, in %.3fuS\n",
                    frame_id, symbol_id, cfg_->BsAntNum(), cfg_->UeAntNum(),
                    bs_rx_.GetTaskTimeUs(frame_id, dl_symbol_id));
              }
              ScheduleTask(EventData(EventType::kPacketTX, event.tags_[0]),
                           &task_queue_user_, ptok_user);
              const bool last_symbol = bs_rx_.CompleteSymbol(frame_id);
              if (last_symbol) {
                bs_rx_.Reset(frame_id);
                AGORA_LOG_INFO(
                    "Frame %zu: Finished downlink reception of %zu symbols "
                    "in %.3fuS\n",
                    frame_id, dl_data_plus_beacon_symbols_,
                    bs_rx_.GetTaskTimeUs(frame_id));
              }
            }
          } else {
            throw std::runtime_error("Invalid kPacketRx type");
          }
        } break;

        case EventType::kPacketTX: {
          const size_t frame_id = gen_tag_t(event.tags_[0u]).frame_id_;
          if (gen_tag_t(event.tags_[0u]).tag_type_ ==
              gen_tag_t::TagType::kUsers) {
            const bool last_symbol = ue_tx_.CompleteTask(frame_id);
            if (last_symbol) {
              if (kDebugPrintPerFrameDone) {
                AGORA_LOG_INFO(
                    "Frame %zu: Finished downlink transmission of %zu "
                    "symbols in %.3fuS\n",
                    frame_id, dl_data_plus_beacon_symbols_,
                    ue_tx_.GetTaskTimeUs(frame_id));
              }
              ue_tx_.Reset(frame_id);
            }
          } else if (gen_tag_t(event.tags_[0]).tag_type_ ==
                     gen_tag_t::TagType::kAntennas) {
            const bool last_symbol = bs_tx_.CompleteTask(frame_id);
            if (last_symbol) {
              if (kDebugPrintPerFrameDone) {
                AGORA_LOG_INFO(
                    "Frame %zu: Finished uplink transmission of %zu symbols "
                    "in "
                    "%.3fuS\n",
                    frame_id, ul_data_plus_pilot_symbols_,
                    bs_tx_.GetTaskTimeUs(frame_id));
              }
              bs_tx_.Reset(frame_id);
            }
          }
        } break;
        default:
          AGORA_LOG_ERROR("Invalid Event Type!\n");
          break;
      }
    }
  }
  running.store(false);

  // Join the joinable threads
  for (auto& join_thread : rec_threads) {
    if (join_thread.joinable()) {
      join_thread.join();
    }
  }
}

void* ChannelSim::TaskThread(size_t tid) {
  PinToCoreWithOffset(ThreadType::kWorker,
                      core_offset_ + bs_thread_num_ + 1 + user_thread_num_,
                      tid);

  // Set Up thread local storage here
  moodycamel::ConsumerToken bs_consumer_token(task_queue_bs_);
  moodycamel::ConsumerToken ue_consumer_token(task_queue_user_);

  ChSimWorkerStorage thread_store(tid, cfg_->UeAntNum(), cfg_->BsAntNum(),
                                  cfg_->SampsPerSymbol(), cfg_->PacketLength());

  EventData event;
  while (running) {
    if (task_queue_bs_.try_dequeue(bs_consumer_token, event)) {
      DoTxBs(&thread_store, event.tags_[0u]);
    } else if (task_queue_user_.try_dequeue(ue_consumer_token, event)) {
      DoTxUser(&thread_store, event.tags_[0u]);
    }
  }
  return nullptr;
}

void* ChannelSim::RxLoop(ChSimRxStorage* rx_storage) {
  const size_t socket_lo = rx_storage->SocketOffset();
  const size_t total_sockets = rx_storage->SocketNumber();
  const size_t socket_hi = socket_lo + total_sockets;

  moodycamel::ProducerToken local_ptok(rx_storage->ResponseQueue());
  PinToCoreWithOffset(ThreadType::kWorkerTXRX, rx_storage->CoreId(),
                      rx_storage->Id());

  const size_t rx_packet_size = rx_storage->PacketLength();
  const size_t buffer_size = kUdpMTU;
  RtAssert(rx_packet_size < buffer_size,
           "Rx Buffer must be larger than the packet size");
  SimdAlignByteVector thread_rx_buffer(buffer_size);

  AGORA_LOG_INFO(
      "RxLoop[%zu]: handling sockets %zu from %zu to %zu rx packet bytes "
      "%zu, max udp rx %zu \n",
      rx_storage->Id(), total_sockets, socket_lo, socket_hi, rx_packet_size,
      buffer_size);

  size_t socket_id = socket_lo;
  while (running) {
    const size_t rx_req_size = buffer_size;
    const int rx_bytes = rx_storage->Socket(socket_id)->Recv(
        thread_rx_buffer.data(), rx_req_size);

    if (0 > rx_bytes) {
      AGORA_LOG_WARN("RxLoop[%zu]: socket %zu receive failed\n",
                     rx_storage->Id(), socket_id);
      throw std::runtime_error("ChannelSim: socket receive failed");
    } else if (rx_bytes > 0) {
      const size_t data_rx = static_cast<size_t>(rx_bytes);
      RtAssert(data_rx == rx_packet_size,
               "Must recv exactly rx_packet_size bytes");
      const Packet* pkt =
          reinterpret_cast<const Packet*>(thread_rx_buffer.data());

      const size_t frame_id = pkt->frame_id_;
      const size_t symbol_id = pkt->symbol_id_;
      const size_t ant_id = pkt->ant_id_;

      rx_storage->TransferRxData(frame_id, symbol_id, ant_id, pkt->data_,
                                 rx_bytes);
      if (kDebugPrintInTask) {
        AGORA_LOG_TRACE(
            "RxLoop[%zu]: Received packet for frame %zu, symbol %zu, "
            "ant %zu from socket %zu \n",
            rx_storage->Id(), frame_id, symbol_id, ant_id, socket_id);
      }

      RtAssert(
          rx_storage->ResponseQueue().enqueue(
              local_ptok,
              EventData(EventType::kPacketRX,
                        gen_tag_t::FrmSymUe(frame_id, symbol_id, ant_id).tag_)),
          "kPacketRX message enqueue failed!");

      // Move to the next socket
      if (socket_id == socket_hi) {
        socket_id = socket_lo;
      } else {
        socket_id++;
      }
    }
  }  // running
  return nullptr;
}

/// Warning: Threads are sharing these sender sockets.
void ChannelSim::DoTx(size_t frame_id, size_t symbol_id, size_t max_ant,
                      size_t ant_per_socket, const arma::cx_float* source_data,
                      SimdAlignByteVector* udp_pkt_buf,
                      std::vector<std::unique_ptr<UDPComm>>& udp_senders) {
  // The 2 is from complex float -> float
  const size_t convert_length = (2 * cfg_->SampsPerSymbol());

#if defined(CHSIM_DEBUG_MEMORY)
  RtAssert(((convert_length % 16) == 0) &&
               ((reinterpret_cast<intptr_t>(source_data) % 64) == 0),
           "Data Alignment not correct before calling into AVX optimizations");
#endif
  size_t source_idx = 0;
  auto* pkt = reinterpret_cast<Packet*>(&udp_pkt_buf->at(0));
  for (size_t ant_id = 0u; ant_id < max_ant; ant_id++) {
    const size_t socket = ant_id / ant_per_socket;
    pkt->frame_id_ = frame_id;
    pkt->symbol_id_ = symbol_id;
    pkt->ant_id_ = ant_id;
    pkt->cell_id_ = 0;

    RtAssert(udp_pkt_buf->size() >
                 (convert_length + Packet::kOffsetOfData + sizeof(short)),
             "TX UDP Buffer Overflow " + std::to_string(udp_pkt_buf->size()) +
                 " : " +
                 std::to_string(
                     (convert_length + Packet::kOffsetOfData + sizeof(short))));

    //inplace conversion to tx buffer
    SimdConvertFloatToShort(
        &reinterpret_cast<const float*>(source_data)[source_idx], pkt->data_,
        convert_length);

    udp_senders.at(socket)->Send(udp_pkt_buf->data(), cfg_->PacketLength());
    source_idx += convert_length;
  }
}

void ChannelSim::DoTxBs(ChSimWorkerStorage* local, size_t tag) {
  const size_t frame_id = gen_tag_t(tag).frame_id_;
  const size_t symbol_id = gen_tag_t(tag).symbol_id_;
  const size_t ue_ul_symbol_idx = cfg_->GetPilotUlIdx(symbol_id);
  if (kPrintDebugTxBs) {
    AGORA_LOG_INFO(
        "Channel Sim[%zu]: DoTxBs processing frame %zu, symbol %zu, at %f ms\n",
        local->Id(), frame_id, symbol_id, GetTime::GetTimeUs() / 1000);
  }

  auto* fmat_src = local->UeInput();
  const size_t convert_length = (2 * cfg_->SampsPerSymbol());
  for (size_t in_ant = 0; in_ant < cfg_->UeAntNum(); in_ant++) {
    const auto* src_ptr = reinterpret_cast<const short*>(
        rx_buffer_ue_->Read(frame_id, symbol_id, in_ant));

    AGORA_LOG_FRAME(
        "Channel Sim[%zu]: DoTxBs processing frame %zu, symbol %zu, ant "
        "%zu, samples per symbol %zu ue ant num %zu ue plus %zu "
        "location %zu\n",
        local->Id(), frame_id, symbol_id, in_ant, cfg_->SampsPerSymbol(),
        cfg_->UeAntNum(), ul_data_plus_pilot_symbols_,
        reinterpret_cast<intptr_t>(src_ptr));

#if defined(CHSIM_DEBUG_MEMORY)
    RtAssert(
        ((convert_length % 16) == 0) &&
            ((reinterpret_cast<intptr_t>(src_ptr) % 64) == 0) &&
            ((reinterpret_cast<intptr_t>(&(*fmat_src)(0, in_ant)) % 64) == 0),
        "Data Alignment not correct before calling into AVX optimizations");
#endif
    // convert received data to complex float,
    // apply channel, convert back to complex short to TX
    SimdConvertShortToFloat(src_ptr,
                            reinterpret_cast<float*>(&(*fmat_src)(0, in_ant)),
                            convert_length);
  }

  AGORA_LOG_TRACE(
      "Channel Sim[%zu]:  DoTxBs Length %lld samps %lld ue ants %lld data size "
      "%zu\n",
      local->Id(), fmat_src->n_elem, fmat_src->n_cols, fmat_src->n_rows,
      sizeof(arma::cx_float));

  auto* fmat_noisy = local->UeOutput();
  const bool is_downlink = false;
  bool is_new_frame;
  if (ue_ul_symbol_idx == 0) {
    is_new_frame = true;
  } else {
    is_new_frame = false;
  }
  // Apply Channel
  channel_->ApplyChan(*fmat_src, *fmat_noisy, is_downlink, is_new_frame);
  AGORA_LOG_TRACE("Noisy dimensions %lld x %lld : %lld\n", fmat_noisy->n_rows,
                  fmat_noisy->n_cols, fmat_noisy->n_elem);

  if (kPrintChannelOutput) {
    Utils::PrintMat(*fmat_noisy, "rx_ul");
  }

  DoTx(frame_id, symbol_id, cfg_->BsAntNum(), cfg_->NumChannels(),
       fmat_noisy->memptr(), &local->TxBuffer(), bs_comm_);

  RtAssert(message_queue_.enqueue(
               *task_ptok_.at(local->Id()),
               EventData(EventType::kPacketTX,
                         gen_tag_t::FrmSymAnt(frame_id, symbol_id, 0).tag_)),
           "BS TX message enqueue failed!\n");
}

void ChannelSim::DoTxUser(ChSimWorkerStorage* local, size_t tag) {
  const size_t frame_id = gen_tag_t(tag).frame_id_;
  const size_t symbol_id = gen_tag_t(tag).symbol_id_;
  const size_t bs_dl_symbol_idx = cfg_->GetBeaconDlIdx(symbol_id);

  if (kPrintDebugTxUser) {
    AGORA_LOG_INFO(
        "Channel Sim[%zu]: DoTxUser processing frame %zu, symbol %zu, dl "
        "symbol %zu, at %f ms\n",
        local->Id(), frame_id, symbol_id, bs_dl_symbol_idx,
        GetTime::GetTimeUs() / 1000);
  }

  auto* fmat_src = local->BsInput();
  const size_t convert_length = (2 * cfg_->SampsPerSymbol());
  for (size_t in_ant = 0; in_ant < cfg_->BsAntNum(); in_ant++) {
    const auto* src_ptr = reinterpret_cast<const short*>(
        rx_buffer_bs_->Read(frame_id, symbol_id, in_ant));

    AGORA_LOG_FRAME(
        "Channel Sim[%zu]: DoTxUser processing frame %zu, symbol %zu, dl "
        "symbol %zu, samples per symbol %zu bs ant num %zu location %zu\n",
        local->Id(), frame_id, symbol_id, bs_dl_symbol_idx,
        cfg_->SampsPerSymbol(), cfg_->BsAntNum(),
        reinterpret_cast<intptr_t>(src_ptr));

#if defined(CHSIM_DEBUG_MEMORY)
    RtAssert(
        ((convert_length % 16) == 0) &&
            ((reinterpret_cast<intptr_t>(src_ptr) % 64) == 0) &&
            ((reinterpret_cast<intptr_t>(&(*fmat_src)(0, in_ant)) % 64) == 0),
        "Data Alignment not correct before calling into AVX optimizations");
#endif
    // convert received data to complex float,
    // apply channel, convert back to complex short to TX
    SimdConvertShortToFloat(src_ptr,
                            reinterpret_cast<float*>(&(*fmat_src)(0, in_ant)),
                            convert_length);
  }

  AGORA_LOG_TRACE(
      "Channel Sim[%zu]: SimdConvertShortToFloat: DoTxUser Length %lld samps "
      "%lld bs ants %lld data size %zu\n",
      local->Id(), fmat_src->n_elem, fmat_src->n_cols, fmat_src->n_rows,
      sizeof(arma::cx_float));

  auto* fmat_noisy = local->BsOutput();
  // Apply Channel
  const bool is_downlink = true;
  bool is_new_frame;
  if (bs_dl_symbol_idx == 0) {
    is_new_frame = true;
  } else {
    is_new_frame = false;
  }
  channel_->ApplyChan(*fmat_src, *fmat_noisy, is_downlink, is_new_frame);

  AGORA_LOG_TRACE("Noisy dimensions %lld x %lld : %lld\n", fmat_noisy->n_rows,
                  fmat_noisy->n_cols, fmat_noisy->n_elem);

  if (kPrintChannelOutput) {
    Utils::PrintMat(*fmat_noisy, "rx_dl");
  }

  DoTx(frame_id, symbol_id, cfg_->UeAntNum(), cfg_->NumUeChannels(),
       fmat_noisy->memptr(), &local->TxBuffer(), ue_comm_);

  RtAssert(message_queue_.enqueue(
               *task_ptok_.at(local->Id()),
               EventData(EventType::kPacketTX,
                         gen_tag_t::FrmSymUe(frame_id, symbol_id, 0).tag_)),
           "UE TX message enqueue failed!\n");
}
