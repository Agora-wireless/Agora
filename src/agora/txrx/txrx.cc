/**
 * @file txrx.cc
 * @brief Implementation of PacketTXRX initialization functions, and datapath
 * functions for communicating with simulators.
 */

#include "txrx.h"

#include "logger.h"

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
  for (size_t i = 0; i < cfg_->SocketThreadNum(); i++) {
    socket_std_threads_.at(i).join();
  }
}

bool PacketTXRX::StartTxRx(Table<char>& buffer, size_t packet_num_in_buffer,
                           Table<size_t>& frame_start, char* tx_buffer,
                           Table<complex_float>& calib_dl_buffer,
                           Table<complex_float>& calib_ul_buffer) {
  frame_start_ = &frame_start;
  tx_buffer_ = tx_buffer;

  if ((kUseArgos == true) || (kUseUHD == true)) {
    if (radioconfig_->RadioStart() == false) {
      std::fprintf(stderr, "Failed to start radio\n");
      return false;
    }

    if (cfg_->Frame().NumDLSyms() > 0) {
      std::memcpy(
          calib_dl_buffer[kFrameWnd - 1], radioconfig_->GetCalibDl(),
          cfg_->OfdmDataNum() * cfg_->BfAntNum() * sizeof(arma::cx_float));
    }
    if (cfg_->Frame().NumULSyms() > 0) {
      std::memcpy(
          calib_ul_buffer[kFrameWnd - 1], radioconfig_->GetCalibUl(),
          cfg_->OfdmDataNum() * cfg_->BfAntNum() * sizeof(arma::cx_float));
    }
  }

  std::printf("PacketTXRX: rx threads %zu, packet buffers %zu\n",
              socket_thread_num_, packet_num_in_buffer);

  buffers_per_socket_ = packet_num_in_buffer / socket_thread_num_;
  /// Make sure we can fit each channel in the tread buffer without rollover
  assert(buffers_per_thread_ % config_->NumChannels() == 0);

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
      socket_std_threads_.at(i) =
          std::thread(&PacketTXRX::LoopTxRxArgos, this, i);
    } else if (kUseUHD == true) {
      socket_std_threads_.at(i) =
          std::thread(&PacketTXRX::LoopTxRxUsrp, this, i);
    } else {
      MLPD_SYMBOL("LoopTXRX: Starting thread %zu\n", i);
      socket_std_threads_.at(i) = std::thread(&PacketTXRX::LoopTxRx, this, i);
    }
  }

  if ((kUseArgos == true) || (kUseUHD == true)) {
    radioconfig_->Go();
  }
  return true;
}

void PacketTXRX::SendBeacon(int tid, size_t frame_id) {
  size_t radio_lo = tid * cfg_->NumRadios() / socket_thread_num_;
  size_t radio_hi = (tid + 1) * cfg_->NumRadios() / socket_thread_num_;

  // Send a beacon packet in the downlink to trigger user pilot
  std::vector<uint8_t> udp_pkt_buf(cfg_->PacketLength(), 0);
  auto* pkt = reinterpret_cast<Packet*>(&udp_pkt_buf[0]);

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
  size_t* rx_frame_start = (*frame_start_)[tid];
  size_t rx_slot = 0;
  size_t radio_lo = tid * cfg_->NumRadios() / socket_thread_num_;
  size_t radio_hi = (tid + 1) * cfg_->NumRadios() / socket_thread_num_;

  const size_t sock_buf_size = (1024 * 1024 * 64 * 8) - 1;
  for (size_t radio_id = radio_lo; radio_id < radio_hi; ++radio_id) {
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

  size_t frame_tsc_delta(cfg_->GetFrameDurationSec() * 1e9 *
                         GetTime::MeasureRdtscFreq());
  int prev_frame_id = -1;
  size_t radio_id = radio_lo;
  size_t tx_frame_start = GetTime::Rdtsc();
  size_t tx_frame_id = 0;
  size_t slow_start_factor = 10;
  // Send Beacons for the first time to kick off sim
  SendBeacon(tid, tx_frame_id++);
  while (cfg_->Running() == true) {
    if (GetTime::Rdtsc() - tx_frame_start >
        frame_tsc_delta * slow_start_factor) {
      tx_frame_start = GetTime::Rdtsc();
      SendBeacon(tid, tx_frame_id++);
      if (tx_frame_id > 5) {
        slow_start_factor = 5;
      } else if (tx_frame_id > 100) {
        slow_start_factor = 4;
      } else if (tx_frame_id > 200) {
        slow_start_factor = 2;
      } else if (tx_frame_id > 500) {
        slow_start_factor = 1;
      }
    }
    if (-1 != DequeueSend(tid)) {
      continue;
    }
    // receive data

    struct Packet* pkt = RecvEnqueue(tid, radio_id, rx_slot);
    if (pkt == nullptr) {
      continue;
    }

    rx_slot = (rx_slot + 1) % buffers_per_socket_;

    if (kIsWorkerTimingEnabled) {
      int frame_id = pkt->frame_id_;
      if (frame_id > prev_frame_id) {
        rx_frame_start[frame_id % kNumStatsFrames] = GetTime::Rdtsc();
        prev_frame_id = frame_id;
      }
    }

    if (++radio_id == radio_hi) {
      radio_id = radio_lo;
    }
  }
}

struct Packet* PacketTXRX::RecvEnqueue(size_t tid, size_t radio_id,
                                       size_t rx_slot) {
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

int PacketTXRX::DequeueSend(int tid) {
  auto& c = cfg_;
  EventData event;
  if (!task_queue_->try_dequeue_from_producer(*tx_ptoks_[tid], event)) {
    return -1;
  }

  // std::printf("tx queue length: %d\n", task_queue_->size_approx());
  assert(event.event_type_ == EventType::kPacketTX);

  size_t ant_id = gen_tag_t(event.tags_[0]).ant_id_;
  size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
  size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;

  size_t data_symbol_idx_dl = cfg_->Frame().GetDLSymbolIdx(symbol_id);
  size_t offset = (c->GetTotalDataSymbolIdxDl(frame_id, data_symbol_idx_dl) *
                   c->BsAntNum()) +
                  ant_id;

  if (kDebugPrintInTask) {
    std::printf(
        "In TXRX thread %d: Transmitted frame %zu, symbol %zu, "
        "ant %zu, tag %zu, offset: %zu, msg_queue_length: %zu\n",
        tid, frame_id, symbol_id, ant_id, gen_tag_t(event.tags_[0]).tag_,
        offset, message_queue_->size_approx());
  }

  char* cur_buffer_ptr = tx_buffer_ + offset * c->DlPacketLength();
  auto* pkt = reinterpret_cast<Packet*>(cur_buffer_ptr);
  new (pkt) Packet(frame_id, symbol_id, 0 /* cell_id */, ant_id);

  // Send data (one OFDM symbol)
  udp_clients_.at(ant_id)->Send(cfg_->BsRruAddr(), cfg_->BsRruPort() + ant_id,
                                reinterpret_cast<uint8_t*>(cur_buffer_ptr),
                                c->DlPacketLength());

  RtAssert(
      message_queue_->enqueue(*rx_ptoks_[tid],
                              EventData(EventType::kPacketTX, event.tags_[0])),
      "Socket message enqueue failed\n");
  return event.tags_[0];
}
