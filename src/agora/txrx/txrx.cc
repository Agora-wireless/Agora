/**
 * @file txrx.cc
 * @brief Implementation of PacketTXRX initialization functions, and datapath
 * functions for communicating with simulators.
 */

#include "txrx.h"

#include <chrono>

#include "logger.h"

static constexpr bool kEnableSlowStart = true;
static constexpr bool kEnableSlowSending = false;
static constexpr bool kDebugPrintBeacon = false;

static constexpr size_t kSlowStartMulStage1 = 32;
static constexpr size_t kSlowStartMulStage2 = 8;

PacketTXRX::PacketTXRX(Config* cfg, size_t core_offset)
    : cfg_(cfg),
      core_offset_(core_offset),
      ant_per_cell_(cfg->BsAntNum() / cfg->NumCells()),
      socket_thread_num_(cfg->SocketThreadNum()) {
  if ((kUseArgos == false) && (kUseUHD == false)) {
    udp_servers_.resize(cfg->NumRadios());
    udp_clients_.resize(cfg->NumRadios());
  } else {
    radioconfig_ = std::make_unique<RadioConfig>(cfg);
  }
}

PacketTXRX::PacketTXRX(Config* cfg, size_t core_offset,
                       moodycamel::ConcurrentQueue<EventData>* queue_message,
                       moodycamel::ConcurrentQueue<EventData>* queue_task,
                       moodycamel::ProducerToken** rx_ptoks,
                       moodycamel::ProducerToken** tx_ptoks)
    : PacketTXRX(cfg, core_offset) {
  message_queue_ = queue_message;
  task_queue_ = queue_task;
  rx_ptoks_ = rx_ptoks;
  tx_ptoks_ = tx_ptoks;
}

PacketTXRX::~PacketTXRX() {
  if (kUseArgos || kUseUHD) {
    radioconfig_->RadioStop();
  }

  for (auto& worker : socket_std_threads_) {
    if (worker.joinable() == true) {
      worker.join();
    }
  }
}

bool PacketTXRX::StartTxRx(Table<char>& buffer, size_t packet_num_in_buffer,
                           Table<size_t>& frame_start, char* tx_buffer,
                           Table<complex_float>& calib_dl_buffer,
                           Table<complex_float>& calib_ul_buffer) {
  frame_start_ = &frame_start;
  tx_buffer_ = tx_buffer;
  threads_started_ = 0;

  if ((kUseArgos == true) || (kUseUHD == true)) {
    if (radioconfig_->RadioStart() == false) {
      std::fprintf(stderr, "Failed to start radio\n");
      return false;
    }

    if (cfg_->Frame().NumDLSyms() > 0) {
      std::memcpy(
          calib_dl_buffer[kFrameWnd - 1], radioconfig_->GetCalibDl(),
          cfg_->OfdmDataNum() * cfg_->BfAntNum() * sizeof(arma::cx_float));
      std::memcpy(
          calib_ul_buffer[kFrameWnd - 1], radioconfig_->GetCalibUl(),
          cfg_->OfdmDataNum() * cfg_->BfAntNum() * sizeof(arma::cx_float));
    }
  }

  std::printf("PacketTXRX: rx threads %zu, packet buffers %zu\n",
              socket_thread_num_, packet_num_in_buffer);

  buffers_per_socket_ = packet_num_in_buffer / socket_thread_num_;
  /// Make sure we can fit each channel in the tread buffer without rollover
  assert(buffers_per_socket_ % cfg_->NumChannels() == 0);

  rx_packets_.resize(socket_thread_num_);
  for (size_t i = 0; i < socket_thread_num_; i++) {
    rx_packets_.at(i).reserve(buffers_per_socket_);
    for (size_t number_packets = 0; number_packets < buffers_per_socket_;
         number_packets++) {
      auto* pkt_loc = reinterpret_cast<Packet*>(
          buffer[i] + (number_packets * cfg_->PacketLength()));
      rx_packets_.at(i).emplace_back(pkt_loc);
    }

    if (kUseArgos == true) {
      socket_std_threads_.emplace_back(&PacketTXRX::LoopTxRxArgos, this, i);
    } else if (kUseUHD == true) {
      socket_std_threads_.emplace_back(&PacketTXRX::LoopTxRxUsrp, this, i);
    } else {
      MLPD_SYMBOL("LoopTXRX: Starting thread %zu\n", i);
      socket_std_threads_.emplace_back(&PacketTXRX::LoopTxRx, this, i);
    }
  }
  //Need to wait for all the threads to have started......
  while (threads_started_.load() != socket_std_threads_.size()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    std::printf("%zu : %zu Threads have started \n", threads_started_.load(),
                socket_std_threads_.size());
  }
  MLPD_INFO("LoopTXRX: socket threads are waiting for events\n");

  if ((kUseArgos == true && cfg_->HwFramer() == true) || (kUseUHD == true)) {
    radioconfig_->Go();
  }
  return true;
}

void PacketTXRX::SendBeacon(int tid, size_t frame_id) {
  static double send_time = 0;
  double time_now = GetTime::GetTimeUs() / 1000;
  size_t radio_lo = tid * cfg_->NumRadios() / socket_thread_num_;
  size_t radio_hi = (tid + 1) * cfg_->NumRadios() / socket_thread_num_;

  // Send a beacon packet in the downlink to trigger user pilot
  std::vector<uint8_t> udp_pkt_buf(cfg_->PacketLength(), 0);
  auto* pkt = reinterpret_cast<Packet*>(&udp_pkt_buf[0]);

  if (kDebugPrintBeacon) {
    std::printf("TXRX [%d]: Sending beacon for frame %zu tx delta %f ms\n", tid,
                frame_id, time_now - send_time);
  }
  send_time = time_now;

  for (size_t beacon_sym = 0; beacon_sym < cfg_->Frame().NumBeaconSyms();
       beacon_sym++) {
    for (size_t ant_id = radio_lo; ant_id < radio_hi; ant_id++) {
      new (pkt) Packet(frame_id, cfg_->Frame().GetBeaconSymbol(beacon_sym),
                       0 /* cell_id */, ant_id);

      udp_clients_.at(ant_id)->Send(cfg_->BsRruAddr(),
                                    cfg_->BsRruPort() + ant_id,
                                    udp_pkt_buf.data(), cfg_->PacketLength());
    }
  }
}

void PacketTXRX::LoopTxRx(size_t tid) {
  PinToCoreWithOffset(ThreadType::kWorkerTXRX, core_offset_, tid);

  const double rdtsc_freq = GetTime::MeasureRdtscFreq();
  const size_t frame_tsc_delta =
      cfg_->GetFrameDurationSec() * 1e9f * rdtsc_freq;
  const size_t two_hundred_ms_ticks = (0.2f /* 200 ms */ * 1e9f * rdtsc_freq);

  // Slow start variables (Start with no less than 200 ms)
  const size_t slow_start_tsc1 =
      std::max(kSlowStartMulStage1 * frame_tsc_delta, two_hundred_ms_ticks);

  const size_t slow_start_thresh1 = kFrameWnd;
  const size_t slow_start_tsc2 = kSlowStartMulStage2 * frame_tsc_delta;
  const size_t slow_start_thresh2 = kFrameWnd * 4;
  size_t delay_tsc = frame_tsc_delta;

  if (kEnableSlowStart) {
    delay_tsc = slow_start_tsc1;
  }

  size_t* const rx_frame_start = (*frame_start_)[tid];
  size_t rx_slot = 0;
  size_t radios_per_thread = (cfg_->NumRadios() / socket_thread_num_);
  if (cfg_->NumRadios() % socket_thread_num_ > 0) {
    radios_per_thread++;
  }
  const size_t radio_lo = tid * radios_per_thread;
  const size_t radio_hi =
      std::min((radio_lo + radios_per_thread), cfg_->BsAntNum()) - 1;

  static constexpr size_t sock_buf_size = (1024 * 1024 * 64 * 8) - 1;
  for (size_t radio_id = radio_lo; radio_id <= radio_hi; ++radio_id) {
    size_t local_port_id = cfg_->BsServerPort() + radio_id;

    udp_servers_.at(radio_id) =
        std::make_unique<UDPServer>(local_port_id, sock_buf_size);
    udp_clients_.at(radio_id) = std::make_unique<UDPClient>();
    MLPD_FRAME(
        "TXRX thread %d: set up UDP socket server listening to port %d"
        " with remote address %s:%d \n",
        tid, local_port_id, cfg_->BsRruAddr().c_str(),
        cfg_->BsRruPort() + radio_id);
  }

  MLPD_INFO("LoopTxRx[%zu] has %zu:%zu total radios %zu\n", tid, radio_lo,
            radio_hi, (radio_hi - radio_lo) + 1);
  // \todo Sync the start of the threads
  threads_started_.fetch_add(1);

  int prev_frame_id = -1;
  size_t radio_id = radio_lo;
  size_t tx_frame_start = GetTime::Rdtsc();
  size_t tx_frame_id = 0;
  size_t send_time = delay_tsc + tx_frame_start;
  // Send Beacons for the first time to kick off sim
  // SendBeacon(tid, tx_frame_id++);
  while (cfg_->Running() == true) {
    size_t rdtsc_now = GetTime::Rdtsc();

    if (rdtsc_now > send_time) {
      SendBeacon(tid, tx_frame_id++);

      if (kEnableSlowStart) {
        if (tx_frame_id == slow_start_thresh1) {
          delay_tsc = slow_start_tsc2;
        } else if (tx_frame_id == slow_start_thresh2) {
          delay_tsc = frame_tsc_delta;
          if (kEnableSlowSending) {
            // Temp for historic reasons
            delay_tsc = frame_tsc_delta * 4;
          }
        }
      }
      tx_frame_start = send_time;
      send_time += delay_tsc;
    }

    const size_t send_result = DequeueSend(tid);
    if (0 == send_result) {
      // receive data

      Packet* pkt = RecvEnqueue(tid, radio_id, rx_slot);
      if (pkt != nullptr) {
        rx_slot = (rx_slot + 1) % buffers_per_socket_;

        if (kIsWorkerTimingEnabled) {
          int frame_id = pkt->frame_id_;
          if (frame_id > prev_frame_id) {
            rx_frame_start[frame_id % kNumStatsFrames] = GetTime::Rdtsc();
            prev_frame_id = frame_id;
          }
        }

        if (++radio_id == (radio_hi + 1)) {
          radio_id = radio_lo;
        }
      }
    }  // end if -1 == send_result
  }    // end while
}

Packet* PacketTXRX::RecvEnqueue(size_t tid, size_t radio_id, size_t rx_slot) {
  moodycamel::ProducerToken* local_ptok = rx_ptoks_[tid];
  size_t packet_length = cfg_->PacketLength();
  RxPacket& rx = rx_packets_.at(tid).at(rx_slot);

  // if rx_buffer is full, exit
  if (rx.Empty() == false) {
    MLPD_ERROR("TXRX thread %zu rx_buffer full, offset: %zu\n", tid, rx_slot);
    cfg_->Running(false);
    return (nullptr);
  }
  Packet* pkt = rx.RawPacket();

  ssize_t rx_bytes = udp_servers_.at(radio_id)->Recv(
      reinterpret_cast<uint8_t*>(pkt), packet_length);
  if (0 > rx_bytes) {
    MLPD_ERROR("RecvEnqueue: Udp Recv failed with error\n");
    throw std::runtime_error("PacketTXRX: recv failed");
  } else if (rx_bytes == 0) {
    pkt = nullptr;
  } else if (static_cast<size_t>(rx_bytes) == packet_length) {
    if (kDebugPrintInTask) {
      std::printf("In TXRX thread %zu: Received frame %d, symbol %d, ant %d\n",
                  tid, pkt->frame_id_, pkt->symbol_id_, pkt->ant_id_);
    }
    if (kDebugMulticell) {
      std::printf(
          "Before packet combining: receiving data stream from the "
          "antenna %d in cell %d,\n",
          pkt->ant_id_, pkt->cell_id_);
    }
    pkt->ant_id_ += pkt->cell_id_ * ant_per_cell_;
    if (kDebugMulticell) {
      std::printf(
          "After packet combining: the combined antenna ID is %d, it "
          "comes from the cell %d\n",
          pkt->ant_id_, pkt->cell_id_);
    }

    // Push kPacketRX event into the queue.
    rx.Use();
    EventData rx_message(EventType::kPacketRX, rx_tag_t(rx).tag_);
    if (message_queue_->enqueue(*local_ptok, rx_message) == false) {
      MLPD_ERROR("socket message enqueue failed\n");
      throw std::runtime_error("PacketTXRX: socket message enqueue failed");
    }
  } else {
    MLPD_ERROR("RecvEnqueue: Udp Recv failed to receive all expected bytes");
    throw std::runtime_error(
        "PacketTXRX::RecvEnqueue: Udp Recv failed to receive all expected "
        "bytes");
  }
  return pkt;
}

size_t PacketTXRX::DequeueSend(int tid) {
  const size_t max_dequeue_items =
      (cfg_->BsAntNum() / cfg_->SocketThreadNum()) + 1;
  // can pull this into the class
  std::vector<EventData> events(max_dequeue_items);

  // Single producer ordering in q is preserved
  const size_t dequeued_items = task_queue_->try_dequeue_bulk_from_producer(
      *tx_ptoks_[tid], events.data(), events.size());

  for (size_t item = 0; item < dequeued_items; item++) {
    EventData& current_event = events.at(item);

    // std::printf("tx queue length: %d\n", task_queue_->size_approx());
    assert(current_event.event_type_ == EventType::kPacketTX);

    const size_t ant_id = gen_tag_t(current_event.tags_[0]).ant_id_;
    const size_t frame_id = gen_tag_t(current_event.tags_[0]).frame_id_;
    const size_t symbol_id = gen_tag_t(current_event.tags_[0]).symbol_id_;

    const size_t data_symbol_idx_dl = cfg_->Frame().GetDLSymbolIdx(symbol_id);
    const size_t offset =
        (cfg_->GetTotalDataSymbolIdxDl(frame_id, data_symbol_idx_dl) *
         cfg_->BsAntNum()) +
        ant_id;

    if (kDebugPrintInTask) {
      std::printf(
          "PacketTXRX[%d]: Transmitted frame %zu, symbol %zu, ant %zu, tag "
          "%zu, offset: %zu, item %zu:%zu, msg_queue_length: %zu\n",
          tid, frame_id, symbol_id, ant_id,
          gen_tag_t(current_event.tags_[0]).tag_, offset, item, dequeued_items,
          message_queue_->size_approx());
    }

    auto* pkt =
        reinterpret_cast<Packet*>(&tx_buffer_[offset * cfg_->DlPacketLength()]);
    new (pkt) Packet(frame_id, symbol_id, 0 /* cell_id */, ant_id);

    // Send data (one OFDM symbol)
    udp_clients_.at(ant_id)->Send(cfg_->BsRruAddr(), cfg_->BsRruPort() + ant_id,
                                  reinterpret_cast<uint8_t*>(pkt),
                                  cfg_->DlPacketLength());

    RtAssert(message_queue_->enqueue(
                 *rx_ptoks_[tid],
                 EventData(EventType::kPacketTX, current_event.tags_[0])),
             "Socket message enqueue failed\n");
  }
  return dequeued_items;
}
