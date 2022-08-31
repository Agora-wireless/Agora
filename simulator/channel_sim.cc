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
static const bool kPrintDebugTxBs = true;

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

  // initialize bs-facing and client-facing data buffers
  const size_t rx_buffer_ue_size = kFrameWnd * ul_data_plus_pilot_symbols_ *
                                   cfg_->UeAntNum() * payload_length_;
  rx_buffer_ue_.resize(rx_buffer_ue_size);

  const size_t rx_buffer_bs_size = kFrameWnd * dl_data_plus_beacon_symbols_ *
                                   cfg_->BsAntNum() * payload_length_;
  rx_buffer_bs_.resize(rx_buffer_bs_size);

  // initilize rx and tx counters
  bs_rx_counter_ =
      std::make_unique<size_t[]>(dl_data_plus_beacon_symbols_ * kFrameWnd);
  std::memset(bs_rx_counter_.get(), 0u,
              sizeof(size_t) * dl_data_plus_beacon_symbols_ * kFrameWnd);

  user_rx_counter_ =
      std::make_unique<size_t[]>(ul_data_plus_pilot_symbols_ * kFrameWnd);
  std::memset(user_rx_counter_.get(), 0u,
              sizeof(size_t) * ul_data_plus_pilot_symbols_ * kFrameWnd);

  bs_tx_counter_.fill(0);
  user_tx_counter_.fill(0);

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

  std::vector<std::thread> base_station_rec_threads;
  base_station_rec_threads.resize(bs_thread_num_);

  for (size_t i = 0; i < bs_thread_num_; i++) {
    base_station_rec_threads.at(i) =
        std::thread(&ChannelSim::BsRxLoop, this, i);
  }

  std::vector<std::thread> user_rx_threads;
  user_rx_threads.resize(user_thread_num_);
  for (size_t i = 0; i < user_thread_num_; i++) {
    user_rx_threads.at(i) = std::thread(&ChannelSim::UeRxLoop, this, i);
  }

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
          const size_t frame_idx = (frame_id % kFrameWnd);
          // received a packet from a client antenna
          if (type == gen_tag_t::TagType::kUsers) {
            const size_t ue_symbol_idx = GetUeUlIdx(symbol_id);
            const size_t frame_offset =
                (frame_idx * ul_data_plus_pilot_symbols_) + ue_symbol_idx;
            size_t& ue_rx_symbol_cnt = user_rx_counter_[frame_offset];
            ue_rx_symbol_cnt++;
            // when received all client antennas on this symbol, kick-off BS TX
            if (ue_rx_symbol_cnt == cfg_->UeAntNum()) {
              ue_rx_symbol_cnt = 0;
              if (kDebugPrintPerSymbolDone) {
                AGORA_LOG_SYMBOL(
                    "Scheduling uplink transmission of frame %zu, symbol %zu, "
                    "from %zu user to %zu BS antennas\n",
                    frame_id, symbol_id, cfg_->UeAntNum(), cfg_->BsAntNum());
              }
              ScheduleTask(EventData(EventType::kPacketTX, event.tags_[0]),
                           &task_queue_bs_, ptok_bs);
            } else if (ue_rx_symbol_cnt > cfg_->UeAntNum()) {
              throw std::runtime_error("Invalid user rx count");
            }
            // received a packet from a BS antenna
          } else if (type == gen_tag_t::TagType::kAntennas) {
            const size_t dl_symbol_id = GetBsDlIdx(symbol_id);
            const size_t frame_offset =
                (frame_idx * dl_data_plus_beacon_symbols_) + dl_symbol_id;
            size_t& bs_rx_symbol_cnt = bs_rx_counter_[frame_offset];
            bs_rx_symbol_cnt++;

            AGORA_LOG_TRACE("Rx downlink frame %zu, symbol %zu, %zu\n",
                            frame_id, symbol_id, bs_rx_symbol_cnt);

            // when received all BS antennas on this symbol, kick-off client TX
            if (bs_rx_symbol_cnt == cfg_->BsAntNum()) {
              bs_rx_symbol_cnt = 0;
              if (kDebugPrintPerSymbolDone) {
                AGORA_LOG_SYMBOL(
                    "Scheduling downlink transmission in frame %zu, symbol "
                    "%zu, from %zu BS to %zu user antennas\n",
                    frame_id, symbol_id, cfg_->BsAntNum(), cfg_->UeAntNum());
              }
              ScheduleTask(EventData(EventType::kPacketTX, event.tags_[0]),
                           &task_queue_user_, ptok_user);
            } else if (bs_rx_symbol_cnt > cfg_->UeAntNum()) {
              throw std::runtime_error("Invalid bs rx count");
            }
          }
        } break;

        case EventType::kPacketTX: {
          const size_t frame_id = gen_tag_t(event.tags_[0u]).frame_id_;
          const size_t frame_idx = (frame_id % kFrameWnd);
          if (gen_tag_t(event.tags_[0u]).tag_type_ ==
              gen_tag_t::TagType::kUsers) {
            size_t& tx_counter = user_tx_counter_.at(frame_idx);
            tx_counter++;
            if (tx_counter == dl_data_plus_beacon_symbols_) {
              if (kDebugPrintPerFrameDone) {
                AGORA_LOG_INFO(
                    "Finished downlink transmission %zu symbols in frame %zu\n",
                    dl_data_plus_beacon_symbols_, frame_id);
              }
              tx_counter = 0;
            } else if (tx_counter > dl_data_plus_beacon_symbols_) {
              throw std::runtime_error("Downlink transmission error");
            }
          } else if (gen_tag_t(event.tags_[0]).tag_type_ ==
                     gen_tag_t::TagType::kAntennas) {
            size_t& tx_counter = bs_tx_counter_.at(frame_idx);
            tx_counter++;
            if (tx_counter == ul_data_plus_pilot_symbols_) {
              if (kDebugPrintPerFrameDone) {
                AGORA_LOG_INFO(
                    "Finished uplink transmission of %zu symbols in frame "
                    "%zu\n",
                    ul_data_plus_pilot_symbols_, frame_id);
              }
              tx_counter = 0;
            } else if (tx_counter > ul_data_plus_pilot_symbols_) {
              throw std::runtime_error("Uplink transmission error");
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
  for (auto& join_thread : base_station_rec_threads) {
    if (join_thread.joinable()) {
      join_thread.join();
    }
  }
  for (auto& join_thread : user_rx_threads) {
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

  const size_t tx_buffer_ue_size =
      dl_data_plus_beacon_symbols_ * cfg_->UeAntNum() * payload_length_;

  const size_t tx_buffer_bs_size =
      ul_data_plus_pilot_symbols_ * cfg_->BsAntNum() * payload_length_;

  WorkerThreadStorage thread_store(
      tid, tx_buffer_ue_size, cfg_->UeAntNum(), tx_buffer_bs_size,
      cfg_->BsAntNum(), cfg_->SampsPerSymbol(), cfg_->PacketLength());

  EventData event;
  while (running) {
    if (task_queue_bs_.try_dequeue(bs_consumer_token, event)) {
      DoTxBs(thread_store, event.tags_[0]);
    } else if (task_queue_user_.try_dequeue(ue_consumer_token, event)) {
      DoTxUser(thread_store, event.tags_[0]);
    }
  }
  return nullptr;
}

void* ChannelSim::BsRxLoop(size_t tid) {
  const size_t socket_lo = tid * bs_socket_num_ / bs_thread_num_;
  const size_t socket_hi = (tid + 1) * bs_socket_num_ / bs_thread_num_;
  const size_t total_sockets = socket_hi - socket_lo;

  moodycamel::ProducerToken local_ptok(message_queue_);
  PinToCoreWithOffset(ThreadType::kWorkerTXRX, core_offset_ + 1, tid);

  // initialize bs-facing sockets
  for (size_t socket_id = socket_lo; socket_id < socket_hi; socket_id++) {
    const size_t local_port_id = cfg_->BsRruPort() + socket_id;
    const size_t remote_port_id = cfg_->BsServerPort() + socket_id;
    bs_comm_.at(socket_id) = std::make_unique<UDPComm>(
        cfg_->BsRruAddr(), local_port_id, kSockBufSize, 0);
    //Create a 1:1 connection
    bs_comm_.at(socket_id)->Connect(cfg_->BsServerAddr(), remote_port_id);
    AGORA_LOG_INFO(
        "ChannelSim::BsRxLoop[%zu]: set up UDP socket server listening to port "
        "%zu with remote address %s:%zu\n",
        tid, local_port_id, cfg_->BsServerAddr().c_str(), remote_port_id);
  }

  const size_t rx_packet_size = cfg_->PacketLength();
  const size_t buffer_size = (rx_packet_size) + kUdpMTU;
  std::vector<SocketRxBuffer> thread_rx_buffers(total_sockets,
                                                SocketRxBuffer(buffer_size));

  AGORA_LOG_INFO(
      "BsRxLoop[%zu]: handling sockets %zu from %zu to %zu rx packet bytes "
      "%zu, max udp rx %zu \n",
      tid, total_sockets, socket_lo, socket_hi, rx_packet_size, buffer_size);

  size_t socket_id = socket_lo;
  while (running) {
    SocketRxBuffer& rx_buffer = thread_rx_buffers.at(socket_id - socket_lo);
    const size_t rx_req_size = rx_packet_size;
    const int rx_bytes =
        bs_comm_.at(socket_id)->Recv(rx_buffer.Rx(), rx_req_size);
    if (0 > rx_bytes) {
      AGORA_LOG_WARN("BsRxLoop[%zu] socket %zu receive failed\n", tid,
                     socket_id);
      throw std::runtime_error("ChannelSim: BS socket receive failed");
    } else if (rx_bytes > 0) {
      const size_t data_rx = static_cast<size_t>(rx_bytes);
      rx_buffer.AddData(data_rx);

      // Process all the full packets
      const size_t packets_to_process = rx_buffer.DataSize() / rx_packet_size;
      RtAssert(packets_to_process == 1, "Multipacket receive not supported");
      size_t processed_packets = 0;
      size_t data_offset = 0;
      while (processed_packets < packets_to_process) {
        const Packet* pkt =
            reinterpret_cast<const Packet*>(rx_buffer.At(data_offset));

        const size_t frame_id = pkt->frame_id_;
        const size_t symbol_id = pkt->symbol_id_;
        const size_t ant_id = pkt->ant_id_;
        const size_t frame_idx = frame_id % kFrameWnd;
        if (kDebugPrintInTask) {
          AGORA_LOG_INFO(
              "BsRxLoop[%zu]: Received BS packet for frame %zu, symbol %zu, "
              "ant %zu from socket %zu\n",
              tid, frame_id, symbol_id, ant_id, socket_id);
        }

        const size_t bs_dl_symbol_id = GetBsDlIdx(symbol_id);
        const size_t symbol_offset =
            (frame_idx * dl_data_plus_beacon_symbols_) + bs_dl_symbol_id;
        const size_t dest_offset = (symbol_offset * cfg_->BsAntNum()) + ant_id;
        //Do fast memcpy
        std::memcpy(&rx_buffer_bs_.at(dest_offset * payload_length_),
                    pkt->data_, payload_length_);

        RtAssert(
            message_queue_.enqueue(
                local_ptok,
                EventData(
                    EventType::kPacketRX,
                    gen_tag_t::FrmSymAnt(frame_id, symbol_id, ant_id).tag_)),
            "BS socket message enqueue failed!");

        data_offset += rx_packet_size;
        processed_packets++;
      }
      // Shift over any processed data
      rx_buffer.RemoveData(data_offset);

      AGORA_LOG_TRACE(
          "BsRxLoop[%zu]: handling socket %zu data buffered %zu processed "
          "%zu\n",
          tid, socket_id, rx_buffer.data_size_, processed_packets);
    }  // bytes available

    // Move to the next socket
    if (++socket_id == socket_hi) {
      socket_id = socket_lo;
    }
  }  // running
  return nullptr;
}

void* ChannelSim::UeRxLoop(size_t tid) {
  const size_t socket_lo = tid * user_socket_num_ / user_thread_num_;
  const size_t socket_hi = (tid + 1) * user_socket_num_ / user_thread_num_;
  const size_t total_sockets = socket_hi - socket_lo;

  moodycamel::ProducerToken local_ptok(message_queue_);
  PinToCoreWithOffset(ThreadType::kWorkerTXRX,
                      core_offset_ + 1 + bs_thread_num_, tid);

  // initialize client-facing sockets
  for (size_t socket_id = socket_lo; socket_id < socket_hi; socket_id++) {
    const size_t local_port_id = cfg_->UeRruPort() + socket_id;
    const size_t remote_port_id = cfg_->UeServerPort() + socket_id;
    ue_comm_.at(socket_id) = std::make_unique<UDPComm>(
        cfg_->UeRruAddr(), local_port_id, kSockBufSize, 0);
    ue_comm_.at(socket_id)->Connect(cfg_->UeServerAddr(), remote_port_id);

    AGORA_LOG_INFO(
        "ChannelSim::UeRxLoop[%zu]: set up UDP socket server listening to "
        "%s:%zu with remote address %s:%zu\n",
        tid, cfg_->UeRruAddr().c_str(), local_port_id,
        cfg_->UeServerAddr().c_str(), remote_port_id);
  }

  const size_t rx_packet_size = cfg_->PacketLength();
  const size_t buffer_size = (rx_packet_size) + kUdpMTU;
  std::vector<SocketRxBuffer> thread_rx_buffers(total_sockets,
                                                SocketRxBuffer(buffer_size));

  AGORA_LOG_INFO(
      "UeRxLoop[%zu]: handling sockets %zu from %zu to %zu rx packet bytes "
      "%zu, max udp rx %zu \n",
      tid, total_sockets, socket_lo, socket_hi, rx_packet_size, buffer_size);

  size_t socket_id = socket_lo;
  while (running) {
    SocketRxBuffer& rx_buffer = thread_rx_buffers.at(socket_id - socket_lo);
    const size_t rx_req_size = rx_packet_size;
    const int rx_bytes =
        ue_comm_.at(socket_id)->Recv(rx_buffer.Rx(), rx_req_size);

    if (0 > rx_bytes) {
      AGORA_LOG_WARN("UeRxLoop[%zu]: socket %zu receive failed\n", tid,
                     socket_id);
      throw std::runtime_error("ChannelSim: UE socket receive failed");
    } else if (rx_bytes > 0) {
      const size_t data_rx = static_cast<size_t>(rx_bytes);
      rx_buffer.AddData(data_rx);

      // Process all the full packets
      const size_t packets_to_process = rx_buffer.DataSize() / rx_packet_size;
      RtAssert(packets_to_process == 1, "Multipacket receive not supported");
      size_t processed_packets = 0;
      size_t data_offset = 0;
      while (processed_packets < packets_to_process) {
        const Packet* pkt =
            reinterpret_cast<const Packet*>(rx_buffer.At(data_offset));

        const size_t frame_id = pkt->frame_id_;
        const size_t symbol_id = pkt->symbol_id_;
        const size_t ant_id = pkt->ant_id_;
        const size_t frame_idx = frame_id % kFrameWnd;

        const size_t ue_ul_symbol_id = GetUeUlIdx(symbol_id);
        const size_t symbol_offset =
            (frame_idx * ul_data_plus_pilot_symbols_) + ue_ul_symbol_id;
        const size_t offset = symbol_offset * cfg_->UeAntNum() + ant_id;
        auto* rx_data_destination = &rx_buffer_ue_.at(offset * payload_length_);
        std::memcpy(rx_data_destination, pkt->data_, payload_length_);

        if (kDebugPrintInTask) {
          AGORA_LOG_TRACE(
              "UeRxLoop[%zu]: Received UE packet for frame %zu, symbol %zu, "
              "ant %zu from socket %zu copying data to location %zu\n",
              tid, frame_id, symbol_id, ant_id, socket_id,
              reinterpret_cast<size_t>(rx_data_destination));
        }

        RtAssert(
            message_queue_.enqueue(
                local_ptok,
                EventData(
                    EventType::kPacketRX,
                    gen_tag_t::FrmSymUe(frame_id, symbol_id, ant_id).tag_)),
            "UE Socket message enqueue failed!");

        data_offset += rx_packet_size;
        processed_packets++;
      }
      // Shift over any processed data
      rx_buffer.RemoveData(data_offset);

      AGORA_LOG_TRACE(
          "UeRxLoop[%zu]: handling socket %zu data buffered %zu processed "
          "%zu\n",
          tid, socket_id, rx_buffer.data_size_, processed_packets);
    }  // bytes available

    // Move to the next socket
    if (++socket_id == socket_hi) {
      socket_id = socket_lo;
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
  //const size_t convert_length = (2 * cfg_->SampsPerSymbol() * max_ant);
  const size_t convert_length = (2 * cfg_->SampsPerSymbol());

#if defined(CHSIM_DEBUG_MEMORY)
  RtAssert(((convert_length % 16) == 0) &&
               ((reinterpret_cast<intptr_t>(dst_ptr) % 64) == 0) &&
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

    RtAssert(udp_pkt_buf->size() < (convert_length + Packet::kOffsetOfData),
             "TX UDP Buffer Overflow");

    //Modify this to use SIMD.
    ConvertFloatToShort(
        reinterpret_cast<const float*>(&source_data[source_idx]), pkt->data_,
        convert_length);

    // Can remove this with some changes
    //std::memcpy(pkt->data_, &tx_buffer[ant_id * payload_length_],
    //            payload_length_);
    //Destination already set by "connect"
    udp_senders.at(socket)->Send(udp_pkt_buf->data(), udp_pkt_buf->size());
    source_idx += convert_length;
  }
}

void ChannelSim::DoTxBs(WorkerThreadStorage& local, size_t tag) {
  const size_t frame_id = gen_tag_t(tag).frame_id_;
  const size_t symbol_id = gen_tag_t(tag).symbol_id_;
  const size_t frame_idx = frame_id % kFrameWnd;
  const size_t ue_ul_symbol_idx = GetUeUlIdx(symbol_id);

  if (kPrintDebugTxBs) {
    AGORA_LOG_INFO(
        "Channel Sim[%zu]: DoTxBs processing frame %zu, symbol %zu, ul "
        "symbol %zu, at %f ms\n",
        local.Id(), frame_id, symbol_id, ue_ul_symbol_idx,
        GetTime::GetTimeUs() / 1000);
  }

  const size_t symbol_offset =
      (frame_idx * ul_data_plus_pilot_symbols_) + ue_ul_symbol_idx;
  const size_t total_offset_ue =
      symbol_offset * payload_length_ * cfg_->UeAntNum();

  const auto* src_ptr =
      reinterpret_cast<const short*>(&rx_buffer_ue_.at(total_offset_ue));

  arma::cx_fmat& fmat_src = local.UeInput();
  // 2 for complex type
  const size_t convert_length = (2 * fmat_src.n_rows * fmat_src.n_cols);

  AGORA_LOG_FRAME(
      "Channel Sim[%zu]: DoTxBs processing frame %zu, symbol %zu, ul symbol "
      "%zu, samples per symbol %zu ue ant num %zu offset %zu ue plus %zu "
      "location %zu\n",
      local.Id(), frame_id, symbol_id, ue_ul_symbol_idx, cfg_->SampsPerSymbol(),
      cfg_->UeAntNum(), total_offset_ue, ul_data_plus_pilot_symbols_,
      reinterpret_cast<intptr_t>(src_ptr));

  AGORA_LOG_TRACE(
      "Channel Sim[%zu]: SimdConvertShortToFloat: DoTxBs Length %lld samps "
      "%lld ue ants %lld data size %zu\n",
      local.Id, fmat_src.n_elem, fmat_src.n_cols, fmat_src.n_rows,
      sizeof(arma::cx_float));

#if defined(CHSIM_DEBUG_MEMORY)
  RtAssert(((convert_length % 16) == 0) &&
               ((reinterpret_cast<intptr_t>(src_ptr) % 64) == 0) &&
               ((reinterpret_cast<intptr_t>(fmat_src.memptr()) % 64) == 0),
           "Data Alignment not correct before calling into AVX optimizations");
#endif
  // convert received data to complex float,
  // apply channel, convert back to complex short to TX
  SimdConvertShortToFloat(src_ptr, reinterpret_cast<float*>(fmat_src.memptr()),
                          convert_length);

  arma::cx_fmat& fmat_noisy = local.UeOutput();

  const bool is_downlink = false;
  bool is_new_frame;
  if (ue_ul_symbol_idx == 0) {
    is_new_frame = true;
  } else {
    is_new_frame = false;
  }
  // Apply Channel
  channel_->ApplyChan(fmat_src, fmat_noisy, is_downlink, is_new_frame);

  AGORA_LOG_TRACE("Noisy dimensions %lld x %lld : %lld\n", fmat_noisy.n_rows,
                  fmat_noisy.n_cols, fmat_noisy.n_elem);

  if (kPrintChannelOutput) {
    Utils::PrintMat(fmat_noisy, "rx_ul");
  }

  DoTx(frame_id, symbol_id, cfg_->BsAntNum(), cfg_->NumChannels(),
       fmat_noisy.memptr(), &local.TxBuffer(), bs_comm_);

  RtAssert(message_queue_.enqueue(
               *task_ptok_.at(local.Id()),
               EventData(EventType::kPacketTX,
                         gen_tag_t::FrmSymAnt(frame_id, symbol_id, 0).tag_)),
           "BS TX message enqueue failed!\n");
}

void ChannelSim::DoTxUser(WorkerThreadStorage& local, size_t tag) {
  const size_t frame_id = gen_tag_t(tag).frame_id_;
  const size_t symbol_id = gen_tag_t(tag).symbol_id_;
  const size_t frame_idx = frame_id % kFrameWnd;
  const size_t bs_dl_symbol_idx = GetBsDlIdx(symbol_id);

  if (kPrintDebugTxUser) {
    AGORA_LOG_INFO(
        "Channel Sim[%zu]: DoTxUser processing frame %zu, symbol %zu, dl "
        "symbol %zu, at %f ms\n",
        local.Id(), frame_id, symbol_id, bs_dl_symbol_idx,
        GetTime::GetTimeUs() / 1000);
  }

  const size_t symbol_offset =
      (frame_idx * dl_data_plus_beacon_symbols_) + bs_dl_symbol_idx;
  const size_t total_offset_bs =
      symbol_offset * payload_length_ * cfg_->BsAntNum();

  const auto* src_ptr =
      reinterpret_cast<const short*>(&rx_buffer_bs_.at(total_offset_bs));

  arma::cx_fmat& fmat_src = local.BsInput();
  // 2 for complex type
  const size_t convert_length = (2 * fmat_src.n_rows * fmat_src.n_cols);

  AGORA_LOG_FRAME(
      "Channel Sim[%zu]: DoTxUser processing frame %zu, symbol %zu, dl symbol "
      "%zu, samples per symbol %zu bs ant num %zu offset %zu location %zu\n",
      local.Id(), frame_id, symbol_id, bs_dl_symbol_idx, cfg_->SampsPerSymbol(),
      cfg_->BsAntNum(), total_offset_bs, reinterpret_cast<intptr_t>(src_ptr));

  AGORA_LOG_TRACE(
      "Channel Sim[%zu]: SimdConvertShortToFloat: DoTxUser Length %lld samps "
      "%lld bs ants %lld data size %zu\n",
      local.Id(), fmat_src.n_elem, fmat_src.n_cols, fmat_src.n_rows,
      sizeof(arma::cx_float));

#if defined(CHSIM_DEBUG_MEMORY)
  RtAssert(((convert_length % 16) == 0) &&
               ((reinterpret_cast<intptr_t>(src_ptr) % 64) == 0) &&
               ((reinterpret_cast<intptr_t>(fmat_src.memptr()) % 64) == 0),
           "Data Alignment not correct before calling into AVX optimizations");
#endif
  // convert received data to complex float,
  // apply channel, convert back to complex short to TX
  SimdConvertShortToFloat(src_ptr, reinterpret_cast<float*>(fmat_src.memptr()),
                          convert_length);

  arma::cx_fmat& fmat_noisy = local.BsOutput();
  // Apply Channel
  const bool is_downlink = true;
  bool is_new_frame;
  if (bs_dl_symbol_idx == 0) {
    is_new_frame = true;
  } else {
    is_new_frame = false;
  }
  channel_->ApplyChan(fmat_src, fmat_noisy, is_downlink, is_new_frame);

  AGORA_LOG_TRACE("Noisy dimensions %lld x %lld : %lld\n", fmat_noisy.n_rows,
                  fmat_noisy.n_cols, fmat_noisy.n_elem);

  if (kPrintChannelOutput) {
    Utils::PrintMat(fmat_noisy, "rx_dl");
  }

  DoTx(frame_id, symbol_id, cfg_->UeAntNum(), cfg_->NumUeChannels(),
       fmat_noisy.memptr(), &local.TxBuffer(), ue_comm_);

  RtAssert(message_queue_.enqueue(
               *task_ptok_.at(local.Id()),
               EventData(EventType::kPacketTX,
                         gen_tag_t::FrmSymUe(frame_id, symbol_id, 0).tag_)),
           "UE TX message enqueue failed!\n");
}
