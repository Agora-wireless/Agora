/**
 * @file txrx_client.cc
 * @brief Implementation file for the radio txrx class
 */
#include "txrx_client.h"

#include <condition_variable>

#include "config.h"
#include "logger.h"

static constexpr size_t kFrameSync = 1000u;

RadioTxRx::RadioTxRx(Config* const cfg, int n_threads, int in_core_id)
    : config_(cfg), thread_num_(n_threads), core_id_(in_core_id) {
  if ((kUseArgos == false) && (kUseUHD == false)) {
    udp_servers_.resize(config_->UeAntNum());
    udp_clients_.resize(config_->UeAntNum());
  } else {
    radioconfig_ = std::make_unique<ClientRadioConfig>(config_);
  }

  /* initialize random seed: */
  srand(time(nullptr));
  thread_sync_ = false;
}

RadioTxRx::RadioTxRx(Config* const config, int n_threads, int in_core_id,
                     moodycamel::ConcurrentQueue<EventData>* in_message_queue,
                     moodycamel::ConcurrentQueue<EventData>* in_task_queue,
                     moodycamel::ProducerToken** in_rx_ptoks,
                     moodycamel::ProducerToken** in_tx_ptoks)
    : RadioTxRx(config, n_threads, in_core_id) {
  message_queue_ = in_message_queue;
  task_queue_ = in_task_queue;
  rx_ptoks_ = in_rx_ptoks;
  tx_ptoks_ = in_tx_ptoks;
}

RadioTxRx::~RadioTxRx() {
  if (kUseArgos || kUseUHD) {
    radioconfig_->RadioStop();
  }

  for (auto& join_thread : txrx_threads_) {
    join_thread.join();
  }
}

bool RadioTxRx::StartTxRx(Table<char>& in_buffer, size_t in_buffer_length,
                          char* in_tx_buffer, int* in_tx_buffer_status,
                          int in_tx_buffer_frame_num, int in_tx_buffer_length) {
  tx_buffer_frame_num_ = in_tx_buffer_frame_num;
  tx_buffer_length_ = in_tx_buffer_length;
  tx_buffer_ = in_tx_buffer;
  tx_buffer_status_ = in_tx_buffer_status;

  if ((kUseArgos == true) || (kUseUHD == true)) {
    if (radioconfig_->RadioStart() == false) {
      return false;
    }
  }

  txrx_threads_.resize(thread_num_);
  buffers_per_thread_ = in_buffer_length / thread_num_;
  /// Make sure we can fit each channel in the tread buffer without rollover
  assert(buffers_per_thread_ % config_->NumUeChannels() == 0);
  rx_packets_.resize(thread_num_);
  for (size_t i = 0; i < thread_num_; i++) {
    rx_packets_.at(i).reserve(buffers_per_thread_);
    for (size_t number_packets = 0; number_packets < buffers_per_thread_;
         number_packets++) {
      auto* pkt_loc = reinterpret_cast<Packet*>(
          in_buffer[i] + (number_packets * config_->PacketLength()));
      rx_packets_.at(i).emplace_back(pkt_loc);
    }

    // start socket thread
    if ((kUseArgos == true) && (config_->UeHwFramer() == true)) {
      txrx_threads_.at(i) = std::thread(&RadioTxRx::LoopTxRxArgos, this, i);
    } else if (kUseArgos || kUseUHD) {
      txrx_threads_.at(i) = std::thread(&RadioTxRx::LoopTxRxArgosSync, this, i);
    } else {
      txrx_threads_.at(i) = std::thread(&RadioTxRx::LoopTxRx, this, i);
    }
  }

  // give time for all threads to lock
  sleep(1);
  {
    std::unique_lock<std::mutex> locker(mutex_);
    this->thread_sync_ = true;
    this->cond_.notify_all();
  }
  return true;
}

Packet* RadioTxRx::RecvEnqueue(size_t tid, size_t ant_id, size_t rx_slot) {
  moodycamel::ProducerToken* local_ptok = rx_ptoks_[tid];
  const size_t packet_length = config_->PacketLength();
  RxPacket& rx = rx_packets_.at(tid).at(rx_slot);

  // if rx_buffer is full, exit
  if (rx.Empty() == false) {
    std::printf("Receive thread %zu rx_buffer full, offset: %zu \n", tid,
                rx_slot);
    config_->Running(false);
    return (nullptr);
  }
  Packet* pkt = rx.RawPacket();

  ssize_t rx_bytes = udp_servers_.at(ant_id)->Recv(
      reinterpret_cast<uint8_t*>(pkt), packet_length);

  if (0 > rx_bytes) {
    std::printf("Receive thread %zu rx_buffer %zu, location: %zu \n", tid,
                rx_slot, (size_t)pkt);
    std::printf("RadioTxRx: receive failed\n");
    throw std::runtime_error("RadioTxRx: receive failed");
  } else if (static_cast<size_t>(rx_bytes) == packet_length) {
    // Push kPacketRX event into the queue.
    rx.Use();
    EventData rx_message(EventType::kPacketRX, rx_tag_t(rx).tag_);
    if (message_queue_->enqueue(*local_ptok, rx_message) == false) {
      std::printf("socket message enqueue failed\n");
      throw std::runtime_error("RadioTxRx: socket message enqueue failed");
    }
    return pkt;
  } else if (rx_bytes != 0) {
    std::printf(
        "RadioTxRx: receive failed on %zu with less than full packet %zu : "
        "%zu\n",
        ant_id, rx_bytes, packet_length);
    throw std::runtime_error(
        "RadioTxRx: receive failed with less than full packet");
  }
  return (nullptr);
}

int RadioTxRx::DequeueSend(int tid) {
  const auto& c = config_;
  EventData event;
  if (task_queue_->try_dequeue_from_producer(*tx_ptoks_[tid], event) == false) {
    return -1;
  }

  RtAssert((event.event_type_ == EventType::kPacketTX) ||
               (event.event_type_ == EventType::kPacketPilotTX),
           "RadioTxRx: Wrong Event Type in TX Queue!");

  // std::printf("tx queue length: %d\n", task_queue_->size_approx());
  const size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
  const size_t ue_id = gen_tag_t(event.tags_[0]).ue_id_;
  std::vector<char> zeros(c->PacketLength(), 0);
  std::vector<char> pilot(c->PacketLength(), 0);
  std::memcpy(&pilot[Packet::kOffsetOfData], c->PilotCi16().data(),
              c->PacketLength() - Packet::kOffsetOfData);

  // Transmit pilot symbols on each UE channel
  for (size_t channel = 0; channel < c->NumUeChannels(); channel++) {
    const size_t ant_id = (ue_id * c->NumUeChannels()) + channel;
    for (size_t symbol_idx = 0; symbol_idx < c->Frame().NumPilotSyms();
         symbol_idx++) {
      if (kDebugPrintInTask) {
        std::printf(
            "In TX thread %d: Transmitted pilot in frame %zu, "
            "symbol %zu, ant %zu\n",
            tid, frame_id, c->Frame().GetPilotSymbol(symbol_idx), ant_id);
      }

      auto* pkt = (symbol_idx == ant_id) ? (Packet*)pilot.data()
                                         : (Packet*)zeros.data();
      new (pkt) Packet(frame_id, c->Frame().GetPilotSymbol(symbol_idx),
                       0 /* cell_id */, ant_id);

      udp_clients_.at(ant_id)->Send(
          config_->BsRruAddr(), config_->UeRruPort() + ant_id,
          reinterpret_cast<uint8_t*>(pkt), c->PacketLength());
    }

    if (event.event_type_ == EventType::kPacketTX) {
      for (size_t symbol_id = 0; symbol_id < c->Frame().NumULSyms();
           symbol_id++) {
        size_t offset =
            (c->GetTotalDataSymbolIdxUl(frame_id, symbol_id) * c->UeAntNum()) +
            ant_id;

        if (kDebugPrintInTask) {
          std::printf(
              "In TX thread %d: Transmitted frame %zu, data symbol %zu, "
              "ant %zu, tag %zu, offset: %zu, msg_queue_length: %zu\n",
              tid, frame_id, c->Frame().GetULSymbol(symbol_id), ant_id,
              gen_tag_t(event.tags_[0]).tag_, offset,
              message_queue_->size_approx());
        }

        auto* pkt = (Packet*)(tx_buffer_ + offset * c->PacketLength());
        new (pkt) Packet(frame_id, c->Frame().GetULSymbol(symbol_id),
                         0 /* cell_id */, ant_id);

        // Send data (one OFDM symbol)
        udp_clients_.at(ant_id)->Send(
            config_->BsRruAddr(), config_->UeRruPort() + ant_id,
            reinterpret_cast<uint8_t*>(pkt), c->PacketLength());
      }
    }  // event.event_type_ == EventType::kPacketTX
  }    // foreach channel

  if (event.event_type_ == EventType::kPacketPilotTX) {
    RtAssert(message_queue_->enqueue(
                 *rx_ptoks_[tid],
                 EventData(EventType::kPacketPilotTX, event.tags_[0])),
             "Socket message enqueue failed\n");
  } else if (event.event_type_ == EventType::kPacketTX) {
    RtAssert(
        message_queue_->enqueue(
            *rx_ptoks_[tid], EventData(EventType::kPacketTX, event.tags_[0])),
        "Socket message enqueue failed\n");
  }
  return event.tags_[0];
}

void* RadioTxRx::LoopTxRx(size_t tid) {
  PinToCoreWithOffset(ThreadType::kWorkerTXRX, core_id_, tid);
  size_t rx_slot = 0;
  const size_t ant_lo = (tid * config_->UeAntNum()) / thread_num_;
  const size_t ant_hi = ((tid + 1) * config_->UeAntNum()) / thread_num_;
  std::printf("Receiver thread %zu has %zu antennas\n", tid, ant_hi - ant_lo);

  size_t sock_buf_size = (1024 * 1024 * 64 * 8) - 1;
  for (size_t ant_id = ant_lo; ant_id < ant_hi; ++ant_id) {
    size_t local_port_id = config_->UeServerPort() + ant_id;
    udp_servers_.at(ant_id) =
        std::make_unique<UDPServer>(local_port_id, sock_buf_size);
    udp_clients_.at(ant_id) = std::make_unique<UDPClient>();
    MLPD_FRAME(
        "TXRX thread %zu: set up UDP socket server listening to port %d"
        " with remote address %s:%d \n",
        tid, local_port_id, config_->BsRruAddr().c_str(),
        config_->UeRruPort() + ant_id);
  }

  size_t ant_id = ant_lo;
  while (config_->Running() == true) {
    if (-1 != DequeueSend(tid)) {
      continue;
    }
    // receive data
    Packet* pkt = RecvEnqueue(tid, ant_id, rx_slot);
    if (pkt == nullptr) {
      continue;
    }
    rx_slot = (rx_slot + 1) % buffers_per_thread_;

    if (++ant_id == ant_hi) {
      ant_id = ant_lo;
    }
  }
  return nullptr;
}

// dequeue_send_sdr
int RadioTxRx::DequeueSendArgos(size_t tid, const long long time0) {
  const auto& c = config_;
  auto& radio = radioconfig_;
  const size_t num_channels = c->NumUeChannels();
  const size_t samples_per_symbol = c->SampsPerSymbol();
  const size_t samples_per_frame =
      samples_per_symbol * c->Frame().NumTotalSyms();

  EventData event;
  const bool tx_ready =
      task_queue_->try_dequeue_from_producer(*tx_ptoks_[tid], event);
  if (tx_ready == false) {
    return -1;
  }

  RtAssert((event.event_type_ == EventType::kPacketTX) ||
               (event.event_type_ == EventType::kPacketPilotTX),
           "Wrong Event Type in TX Queue!");

  //Assuming the 1 message per radio per frame
  const size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
  //ue_id = radio_id
  const size_t ue_id = gen_tag_t(event.tags_[0]).ue_id_;
  const size_t ue_ant_start = ue_id * c->NumUeChannels();
  const size_t tx_frame_id = (frame_id + TX_FRAME_DELTA);
  long long tx_time = 0;

  std::vector<std::complex<int16_t>> zeros(samples_per_symbol,
                                           std::complex<int16_t>(0, 0));
  //Place zeros in all dimensions
  std::vector<void*> tx_data(num_channels, zeros.data());

  MLPD_INFO("DequeueSendArgos[%zu]: Transmitted frame %zu, ue %zu\n", tid,
            frame_id, ue_id);
  RtAssert(ue_id == tid, "DequeueSendArgos Ue id was not the expected values");

  // Transmit pilot(s)
  // For UHD devices, first pilot should not be with the END_BURST flag
  // 1: HAS_TIME, 2: HAS_TIME | END_BURST
  int flags_tx = 1;

  if (c->UeHwFramer() == false) {
    for (size_t ch = 0; ch < num_channels; ch++) {
      const size_t ant_pilot = ue_ant_start + ch;
      const size_t pilot_symbol_id = c->Frame().GetPilotSymbol(ant_pilot);

      //See if we need to set end burst for the last channel
      // (see if the next symbol is an uplink symbol)
      if ((ch + 1) == num_channels) {
        if (c->Frame().NumULSyms() > 0) {
          const size_t first_ul_symbol = c->Frame().GetULSymbol(0);
          if ((pilot_symbol_id + 1) == (first_ul_symbol)) {
            flags_tx = 2;
          }
        }
      }

      //Add the pilot to the correct index
      tx_data.at(ch) = c->PilotCi16().data();

      tx_time = time0 + (tx_frame_id * samples_per_frame) +
                (pilot_symbol_id * samples_per_symbol) -
                c->ClTxAdvance().at(ue_id);

      const int tx_status = radio->RadioTx(
          ue_id, tx_data.data(), samples_per_symbol, flags_tx, tx_time);

      if (tx_status < 0) {
        std::cout << "BAD Radio Tx: (PILOT)" << tx_status << "For Ue Radio "
                  << ue_id << "/" << samples_per_symbol << std::endl;
      } else if (tx_status != static_cast<int>(samples_per_symbol)) {
        std::cout << "BAD Write: (PILOT)" << tx_status << "For Ue Radio "
                  << ue_id << "/" << samples_per_symbol << std::endl;
      }

      if (kDebugPrintInTask) {
        std::printf(
            "In TX thread %zu: Transmitted frame %zu, pilot symbol %zu, ue "
            "%zu\n",
            tid, frame_id, pilot_symbol_id, ue_id);
      }
      //Replace the pilot with zeros for next channel pilot
      tx_data.at(ch) = zeros.data();
    }
  }
  if (event.event_type_ == EventType::kPacketPilotTX) {
    RtAssert(message_queue_->enqueue(
                 *rx_ptoks_[tid],
                 EventData(EventType::kPacketPilotTX, event.tags_[0])),
             "Socket message enqueue failed\n");
    // kPacketPilotTX is scheduled when no Uplink data is present,
    // so return here!
    return event.tags_[0];
  }

  // Transmit data for all symbols
  const size_t packet_length = c->PacketLength();
  flags_tx = 1;
  for (size_t symbol_id = 0; symbol_id < c->Frame().NumULSyms(); symbol_id++) {
    const size_t tx_symbol_id = c->Frame().GetULSymbol(symbol_id);
    const size_t offset =
        (c->GetTotalDataSymbolIdxUl(frame_id, symbol_id) * c->UeAntNum()) +
        ue_ant_start;

    for (size_t ch = 0; ch < num_channels; ch++) {
      auto* tx_pkt =
          reinterpret_cast<Packet*>(tx_buffer_[(offset + ch) * packet_length]);
      tx_data.at(ch) = static_cast<void*>(tx_pkt->data_);
      tx_buffer_status_[offset + ch] = 0;
    }
    if (c->UeHwFramer()) {
      tx_time = ((long long)tx_frame_id << 32) | (tx_symbol_id << 16);
    } else {
      tx_time = time0 + (tx_frame_id * samples_per_frame) +
                (tx_symbol_id * samples_per_symbol) -
                c->ClTxAdvance().at(ue_id);
    }

    if (tx_symbol_id == c->Frame().GetULSymbolLast()) {
      flags_tx = 2;  // HAS_TIME & END_BURST, fixme
    }
    const int tx_status = radio->RadioTx(ue_id, tx_data.data(),
                                         samples_per_symbol, flags_tx, tx_time);
    if (tx_status < static_cast<int>(samples_per_symbol)) {
      std::cout << "BAD Write (UL): For Ue " << ue_id << " " << tx_status << "/"
                << samples_per_symbol << std::endl;
    }

    if (kDebugPrintInTask) {
      std::printf(
          "DequeueSendArgos[%zu]: Transmitted frame %zu, data symbol %zu, "
          "radio %zu, tag %zu, offset: %zu, msg_queue_length: %zu\n",
          tid, frame_id, tx_symbol_id, ue_id, gen_tag_t(event.tags_[0]).tag_,
          offset, message_queue_->size_approx());
    }
  }

  RtAssert(
      message_queue_->enqueue(*rx_ptoks_[tid],
                              EventData(EventType::kPacketTX, event.tags_[0])),
      "Socket message enqueue failed\n");
  return event.tags_[0];
}

std::vector<Packet*> RadioTxRx::RecvEnqueueArgos(size_t tid, size_t radio_id,
                                                 size_t& frame_id,
                                                 size_t& symbol_id,
                                                 size_t rx_slot,
                                                 bool dummy_enqueue) {
  std::vector<Packet*> rx_info;
  const auto& c = config_;
  moodycamel::ProducerToken* local_ptok = rx_ptoks_[tid];

  const size_t num_rx_samps = c->SampsPerSymbol();
  long long rx_time = 0;

  std::vector<void*> samp(c->NumUeChannels());
  for (size_t ch = 0; ch < c->NumUeChannels(); ch++) {
    RxPacket& rx = rx_packets_.at(tid).at(rx_slot + ch);
    if (rx.Empty() == false) {
      std::printf("RX [%zu] at rx_offset %zu buffer full\n", tid, rx_slot);
      c->Running(false);
      return rx_info;
    }
    samp.at(ch) = rx.RawPacket()->data_;
  }

  if (dummy_enqueue == false) {
    ClientRadioConfig* radio = radioconfig_.get();
    const int rx_status =
        radio->RadioRx(radio_id, samp.data(), num_rx_samps, rx_time);
    if (rx_status < 0) {
      std::cerr << "RX [" << tid << "]: Receive error! Stopping... "
                << std::endl;
      c->Running(false);
      return rx_info;
    } else if (static_cast<size_t>(rx_status) != num_rx_samps) {
      std::cerr << "RX [" << tid << "]: BAD Receive(" << rx_status << "/"
                << num_rx_samps << ") at Time " << rx_time << std::endl;
    }

    if (c->UeHwFramer()) {
      frame_id = static_cast<size_t>(rx_time >> 32);
      symbol_id = static_cast<size_t>((rx_time >> 16) & 0xFFFF);
    } else {
      // assert(c->UeHwFramer()
      //    || (((rxTime - time0) / (num_rx_samps * c->frame().NumTotalSyms()))
      //           == frame_id));
    }
  }
  if (kDebugPrintInTask) {
    std::printf(
        "RX [%zu]: frame_id %zu, symbol_id %zu, radio_id %zu rxtime %llx\n",
        tid, frame_id, symbol_id, radio_id, rx_time);
  }
  const size_t ant_id = radio_id * c->NumUeChannels();
  for (size_t ch = 0; ch < c->NumUeChannels(); ch++) {
    RxPacket& rx = rx_packets_.at(tid).at(rx_slot + ch);
    new (rx.RawPacket())
        Packet(frame_id, symbol_id, 0 /* cell_id */, ant_id + ch);

    rx.Use();
    EventData rx_message(EventType::kPacketRX, rx_tag_t(rx).tag_);

    const bool enqueue_status =
        message_queue_->enqueue(*local_ptok, rx_message);

    RtAssert(enqueue_status == true, "receive message enqueue failed");
    rx_info.push_back(rx.RawPacket());
  }
  return rx_info;
}

void* RadioTxRx::LoopTxRxArgos(size_t tid) {
  PinToCoreWithOffset(ThreadType::kWorkerTXRX, core_id_, tid);
  const auto& c = config_;
  const size_t num_radios = c->UeNum();
  const size_t radio_lo = tid * num_radios / thread_num_;
  const size_t radio_hi = (tid + 1) * num_radios / thread_num_;
  std::printf("RadioTxRx thread %zu has radios %zu to %zu (%zu)\n", tid,
              radio_lo, radio_hi - 1, radio_hi - radio_lo);

  // Use mutex to sychronize data receiving across threads
  {
    std::unique_lock<std::mutex> locker(mutex_);
    std::printf("RadioTxRx [%zu]: waiting for release\n", tid);
    cond_.wait(locker, [this] { return thread_sync_.load(); });
  }
  std::printf("RadioTxRx [%zu]: released\n", tid);

  ClientRadioConfig* radio = radioconfig_.get();

  std::vector<int> all_trigs(radio_hi - radio_lo, 0);
  struct timespec tv;
  struct timespec tv2;
  clock_gettime(CLOCK_MONOTONIC, &tv);

  size_t rx_slot = 0;
  size_t frame_id(0);
  size_t symbol_id(0);
  size_t radio_id = radio_lo;
  while (c->Running() == true) {
    clock_gettime(CLOCK_MONOTONIC, &tv2);
    double diff =
        ((tv2.tv_sec - tv.tv_sec) * 1e9 + (tv2.tv_nsec - tv.tv_nsec)) / 1e9;
    if (diff > 2) {
      for (size_t it = radio_lo; it < radio_hi; it++) {
        int total_trigs = radio->Triggers(it);
        std::cout << "radio: " << it << ", new triggers: "
                  << total_trigs - all_trigs[it - radio_lo]
                  << ", total: " << total_trigs << std::endl;
        all_trigs[it - radio_lo] = total_trigs;
      }
      tv = tv2;
    }
    // transmit data
    if (-1 != DequeueSendArgos(tid, 0)) {
      continue;
    }

    const auto rx_pkts =
        RecvEnqueueArgos(tid, radio_id, frame_id, symbol_id, rx_slot, false);
    if (rx_pkts.size() != c->NumUeChannels()) {
      throw std::runtime_error(
          "Received data but it was not the correct dimension");
      continue;
    }

    rx_slot = (rx_slot + c->NumUeChannels()) % buffers_per_thread_;
    if (++radio_id == radio_hi) {
      radio_id = radio_lo;
    }
  }
  return nullptr;
}

void* RadioTxRx::LoopTxRxArgosSync(size_t tid) {
  static constexpr size_t kSyncDetectChannel = 0;
  ///\todo FIXME: This only works when there is 1 radio per thread.
  PinToCoreWithOffset(ThreadType::kWorkerTXRX, core_id_, tid);
  const auto& c = config_;
  const size_t num_channels = c->NumUeChannels();
  const size_t samples_per_symbol = c->SampsPerSymbol();
  const size_t samples_per_frame =
      samples_per_symbol * c->Frame().NumTotalSyms();
  ClientRadioConfig* radio = radioconfig_.get();

  std::vector<std::vector<std::complex<int16_t>>> frm_buff(
      num_channels, std::vector<std::complex<int16_t>>(
                        samples_per_frame, std::complex<int16_t>(0, 0)));

  const std::vector<std::vector<std::complex<int16_t>>> zeros(
      num_channels, std::vector<std::complex<int16_t>>(
                        samples_per_frame, std::complex<int16_t>(0, 0)));

  std::vector<std::complex<float>> sync_buff(samples_per_frame,
                                             std::complex<float>(0.0f, 0.0f));

  //Setup rx memory
  std::vector<void*> frm_rx_buff(num_channels);
  for (size_t ch = 0; ch < num_channels; ch++) {
    frm_rx_buff.at(ch) = frm_buff.at(ch).data();
  }

  const size_t radio_id = tid;
  // Use mutex to sychronize data receiving across threads
  {
    std::unique_lock<std::mutex> locker(mutex_);

    //Every thread does this....
    MLPD_INFO("RadioTxRx [%zu]: waiting for release\n", tid);
    cond_.wait(locker, [this] { return thread_sync_.load(); });
  }
  MLPD_INFO("RadioTxRx [%zu]: released\n", tid);

  // Keep receiving one frame of data until a beacon is found
  // Perform initial beacon detection every kBeaconDetectInterval frames
  ssize_t sync_index = -1;
  int rx_offset = 0;
  long long rx_time = 0;
  while ((c->Running() == true) && (sync_index < 0)) {
    int rx_status = 0;
    for (size_t find_beacon_retry = 0;
         find_beacon_retry < kBeaconDetectInterval; find_beacon_retry++) {
      rx_status = radio->RadioRx(radio_id, frm_rx_buff.data(),
                                 samples_per_frame, rx_time);

      if (rx_status != static_cast<int>(samples_per_frame)) {
        std::cerr << "RadioTxRx [" << radio_id << "]: BAD SYNC Receive("
                  << rx_status << "/" << samples_per_frame << ") at Time "
                  << rx_time << std::endl;
      }
    }

    //Received frame data, inspect for beacon
    if (rx_status == static_cast<int>(samples_per_frame)) {
      // convert data to complex float for sync detection
      for (size_t i = 0; i < samples_per_frame; i++) {
        sync_buff.at(i) = (std::complex<float>(
            static_cast<float>(static_cast<std::complex<int16_t>*>(
                                   frm_rx_buff.at(kSyncDetectChannel))[i]
                                   .real()) /
                32768.0f,
            static_cast<float>(static_cast<std::complex<int16_t>*>(
                                   frm_rx_buff.at(kSyncDetectChannel))[i]
                                   .imag()) /
                32768.0f));
      }
      sync_index = CommsLib::FindBeaconAvx(sync_buff, c->GoldCf32());
      if (sync_index >= 0) {
        rx_offset = sync_index - c->BeaconLen() - c->OfdmTxZeroPrefix();
        MLPD_INFO(
            "RadioTxRx [%zu]: Beacon detected at Time %lld, sync_index: %ld, "
            "rx sample offset: %d\n",
            radio_id, rx_time, sync_index, rx_offset);
      }
    }
  }  // end while sync_index < 0

  // Read rx_offset to align with the begining of a frame
  if (rx_offset > 0) {
    radio->RadioRx(radio_id, frm_rx_buff.data(), rx_offset, rx_time);
    rx_offset = 0;
  } else if (rx_offset < 0) {
    throw std::runtime_error("rx sample offset is less than 0");
  }

  long long time0 = 0;
  size_t rx_frame_id = 0;
  size_t symbol_id = 0;

  bool resync = false;
  size_t resync_retry_cnt = 0;
  size_t resync_retry_max = 100;
  size_t resync_success = 0;

  std::stringstream sout;
  size_t rx_slot = 0;

  //Beacon sync detected run main rx routines
  while (c->Running() == true) {
    if ((c->FramesToTest() > 0) && (rx_frame_id > c->FramesToTest())) {
      c->Running(false);
      break;
    }

    // recv symbol  0  = (Beacon)
    const size_t rx_samples = samples_per_symbol + rx_offset;
    const int rx_beacon_status =
        radio->RadioRx(radio_id, frm_rx_buff.data(), rx_samples, rx_time);
    //Reset offset to 0 after it has been read, resync could change this
    rx_offset = 0;

    if (rx_beacon_status < 0) {
      std::cerr << "RadioTxRx [" << radio_id << "]: Receive error! Stopping... "
                << std::endl;
      c->Running(false);
      break;
    } else if (static_cast<size_t>(rx_beacon_status) != rx_samples) {
      std::cerr << "RadioTxRx [" << radio_id << "]: BAD Beacon Receive("
                << rx_beacon_status << "/" << rx_samples << ") at Time "
                << rx_time << std::endl;
    }

    if (rx_frame_id == 0) {
      time0 = rx_time;
    }

    // Enqueue the rx'd beacon symbol = 0
    const auto rx_pkts =
        RecvEnqueueArgos(tid, radio_id, rx_frame_id, symbol_id, rx_slot, true);
    if (rx_pkts.size() == c->NumUeChannels()) {
      throw std::runtime_error(
          "Received data but it was not the correct dimension");
    }
    symbol_id++;
    //Increment the rx buffer space.
    rx_slot = (rx_slot + c->NumUeChannels()) % buffers_per_thread_;

    // resync every kFrameSync frames:
    ///\todo: kFrameSync should be a function of sample rate and max CFO
    if (((rx_frame_id / kFrameSync) > 0) && ((rx_frame_id % kFrameSync) == 0)) {
      resync = true;
    }

    if (resync) {
      sync_buff.resize(samples_per_symbol);
      // convert data to complex float for sync detection
      for (size_t i = 0; i < samples_per_symbol; i++) {
        sync_buff.at(i) = (std::complex<float>(
            static_cast<float>(static_cast<std::complex<int16_t>*>(
                                   frm_rx_buff.at(kSyncDetectChannel))[i]
                                   .real()) /
                32768.0f,
            static_cast<float>(static_cast<std::complex<int16_t>*>(
                                   frm_rx_buff.at(kSyncDetectChannel))[i]
                                   .imag()) /
                32768.0f));
      }
      size_t resync_index = CommsLib::FindBeaconAvx(sync_buff, c->GoldCf32());
      if (resync_index >= 0) {
        rx_offset = resync_index - c->BeaconLen() - c->OfdmTxZeroPrefix();
        time0 += rx_offset;
        resync = false;
        resync_success++;
        MLPD_INFO(
            "RadioTxRx [%zu]: Re-syncing with offset %d, after %zu tries, "
            "index: %ld\n",
            radio_id, rx_offset, resync_retry_cnt + 1, resync_index);
        resync_retry_cnt = 0;
      } else {
        resync_retry_cnt++;
      }
    }
    if (resync && (resync_retry_cnt > resync_retry_max)) {
      MLPD_ERROR(
          "RadioTxRx [%zu]: Exceeded resync retry limit (%zu) for client %zu "
          "reached after %zu resync successes at frame: %zu.  Stopping!\n",
          radio_id, resync_retry_max, tid, resync_success, rx_frame_id);
      c->Running(false);
      break;
    }

    // Received the beacon, Schedule transmit pilots and symbols
    while (-1 != DequeueSendArgos(tid, time0)) {
      ;
    }
    //resyncing could have changed the rx_offset..

    // receive the remaining of the frame
    for (; symbol_id < c->Frame().NumTotalSyms(); symbol_id++) {
      // Attempt a Tx (After every Rx Symbol)
      while (-1 != DequeueSendArgos(tid, time0)) {
        ;
      }

      if (config_->IsDownlink(rx_frame_id, symbol_id) == true) {
        const auto rx_pkts = RecvEnqueueArgos(tid, radio_id, rx_frame_id,
                                              symbol_id, rx_slot, false);
        if (rx_pkts.size() != c->NumUeChannels()) {
          throw std::runtime_error(
              "Received data but it was not the correct dimension");
        }
        rx_slot = (rx_slot + c->NumUeChannels()) % buffers_per_thread_;
      } else {
        // Otherwise throw away the symbol.
        const int rx_status = radio->RadioRx(radio_id, frm_rx_buff.data(),
                                             samples_per_symbol, rx_time);

        if (rx_status < 0) {
          std::cerr << "RadioTxRx [" << radio_id
                    << "]: Receive error !Stopping... " << std::endl;
          c->Running(false);
          break;
        } else if (static_cast<size_t>(rx_status) != samples_per_frame) {
          std::cerr << "RadioTxRx [" << radio_id << "]: BAD Receive("
                    << rx_status << " / " << samples_per_symbol << ") at Time "
                    << rx_time << std::endl;
        }

        if (kDebugPrintInTask) {
          std::printf(
              "RadioTxRx [%zu]: receive thread %zu, frame_id %zu, "
              "symbol_id %zu, radio_id %zu rxtime %llx\n",
              radio_id, tid, rx_frame_id, symbol_id, radio_id, rx_time);
        }
      }
    }  // Rx Rest of the frame
    //incrememt frame and reset the symbol to start
    rx_frame_id++;
    symbol_id = 0;
  }
  return nullptr;
}
