/**
 * @file txrx_client.cc
 * @brief Implementation file for the radio txrx class
 */
#include "txrx_client.h"

#include "config.h"

RadioTxRx::RadioTxRx(Config* cfg, int n_threads, int in_core_id)
    : config_(cfg), thread_num_(n_threads), core_id_(in_core_id) {
  if (!kUseArgos && !kUseUHD) {
    socket_.resize(config_->NumRadios());
    servaddr_.resize(config_->NumRadios());
  } else {
    radioconfig_ = new ClientRadioConfig(config_);
  }

  /* initialize random seed: */
  srand(time(NULL));
}

RadioTxRx::RadioTxRx(Config* config, int n_threads, int in_core_id,
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
    delete radioconfig_;
  }
  delete config_;
}

bool RadioTxRx::StartTxRx(Table<char>& in_buffer, Table<int>& in_buffer_status,
                          size_t in_buffer_frame_num, size_t in_buffer_length,
                          char* in_tx_buffer, int* in_tx_buffer_status,
                          int in_tx_buffer_frame_num, int in_tx_buffer_length) {
  buffer_frame_num_ = in_buffer_frame_num;
  assert(in_buffer_length ==
         config_->PacketLength() * buffer_frame_num_);  // should be integer
  buffer_length_ = in_buffer_length;
  buffer_ = &in_buffer;                // for save data
  buffer_status_ = &in_buffer_status;  // for save status

  tx_buffer_frame_num_ = in_tx_buffer_frame_num;
  tx_buffer_length_ = in_tx_buffer_length;
  tx_buffer_ = in_tx_buffer;
  tx_buffer_status_ = in_tx_buffer_status;

  if (kUseArgos || kUseUHD) {
    if (!radioconfig_->RadioStart()) {
      return false;
    }
  }

  for (int i = 0; i < thread_num_; i++) {
    pthread_t txrx_thread;
    // record the thread id
    auto* context = new EventHandlerContext<RadioTxRx>;
    context->obj_ptr_ = this;
    context->id_ = i;
    // start socket thread
    if (kUseArgos && config_->HwFramer()) {
      if (pthread_create(
              &txrx_thread, NULL,
              PthreadFunWrapper<RadioTxRx, &RadioTxRx::LoopTxRxArgos>,
              context) != 0) {
        perror("socket thread create failed");
        std::exit(0);
      }
    } else if (kUseArgos || kUseUHD) {
      if (pthread_create(
              &txrx_thread, NULL,
              PthreadFunWrapper<RadioTxRx, &RadioTxRx::LoopTxRxArgosSync>,
              context) != 0) {
        perror("socket thread create failed");
        std::exit(0);
      }
    } else {
      if (pthread_create(&txrx_thread, NULL,
                         PthreadFunWrapper<RadioTxRx, &RadioTxRx::LoopTxRx>,
                         context) != 0) {
        perror("socket thread create failed");
        std::exit(0);
      }
    }
  }

  // give time for all threads to lock
  sleep(1);
  pthread_cond_broadcast(&cond_);

  return true;
}

struct Packet* RadioTxRx::RecvEnqueue(int tid, int radio_id, int rx_offset) {
  moodycamel::ProducerToken* local_ptok = rx_ptoks_[tid];
  char* rx_buffer = (*buffer_)[tid];
  int* rx_buffer_status = (*buffer_status_)[tid];
  int packet_length = config_->PacketLength();

  // if rx_buffer is full, exit
  if (rx_buffer_status[rx_offset] == 1) {
    std::printf("Receive thread %d rx_buffer full, offset: %d\n", tid,
                rx_offset);
    config_->Running(false);
    return (NULL);
  }
  struct Packet* pkt = (struct Packet*)&rx_buffer[rx_offset * packet_length];
  if (-1 == recv(socket_[radio_id], (char*)pkt, packet_length, 0)) {
    if (errno != EAGAIN && config_->Running()) {
      perror("recv failed");
      std::exit(0);
    }
    return (NULL);
  }

  // get the position in rx_buffer
  // move ptr & set status to full
  rx_buffer_status[rx_offset] = 1;

  // Push kPacketRX event into the queue.
  EventData rx_message(EventType::kPacketRX, rx_tag_t(tid, rx_offset).tag_);
  if (!message_queue_->enqueue(*local_ptok, rx_message)) {
    std::printf("socket message enqueue failed\n");
    std::exit(0);
  }
  return pkt;
}

int RadioTxRx::DequeueSend(int tid) {
  auto& c = config_;
  EventData event;
  if (!task_queue_->try_dequeue_from_producer(*tx_ptoks_[tid], event)) {
    return -1;
  }

  // std::printf("tx queue length: %d\n", task_queue_->size_approx());
  size_t ant_id = gen_tag_t(event.tags_[0]).ant_id_;
  size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
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

      auto* pkt = (struct Packet*)(tx_buffer_ + offset * c->PacketLength());
      new (pkt) Packet(frame_id, c->Frame().GetULSymbol(symbol_id),
                       0 /* cell_id */, ant_id);

      // Send data (one OFDM symbol)
      ssize_t ret = sendto(socket_[ant_id], (char*)pkt, c->PacketLength(), 0,
                           (struct sockaddr*)&servaddr_[ant_id],
                           sizeof(servaddr_[ant_id]));
      RtAssert(ret > 0, "sendto() failed");
    }
    RtAssert(
        message_queue_->enqueue(
            *rx_ptoks_[tid], EventData(EventType::kPacketTX, event.tags_[0])),
        "Socket message enqueue failed\n");
  } else if (event.event_type_ == EventType::kPacketPilotTX) {
    std::vector<char> zeros(c->PacketLength(), 0);
    std::vector<char> pilot(c->PacketLength(), 0);
    std::memcpy(&pilot[Packet::kOffsetOfData], c->PilotCi16().data(),
                c->PacketLength() - Packet::kOffsetOfData);
    for (size_t symbol_idx = 0; symbol_idx < c->Frame().NumPilotSyms();
         symbol_idx++) {
      if (kDebugPrintInTask) {
        std::printf(
            "In TX thread %d: Transmitted pilot in frame %zu, "
            "symbol %zu, "
            "ant %zu\n",
            tid, frame_id, c->Frame().GetPilotSymbol(symbol_idx), ant_id);
      }

      auto* pkt = (symbol_idx == ant_id) ? (struct Packet*)pilot.data()
                                         : (struct Packet*)zeros.data();
      new (pkt) Packet(frame_id, c->Frame().GetPilotSymbol(symbol_idx),
                       0 /* cell_id */, ant_id);
      // Send pilots
      ssize_t ret = sendto(socket_[ant_id], (char*)pkt, c->PacketLength(), 0,
                           (struct sockaddr*)&servaddr_[ant_id],
                           sizeof(servaddr_[ant_id]));
      RtAssert(ret > 0, "sendto() failed");
    }
    RtAssert(message_queue_->enqueue(
                 *rx_ptoks_[tid],
                 EventData(EventType::kPacketPilotTX, event.tags_[0])),
             "Socket message enqueue failed\n");
  } else {
    std::printf("Wrong event type in tx queue!");
    std::exit(0);
  }
  return event.tags_[0];
}

void* RadioTxRx::LoopTxRx(int tid) {
  PinToCoreWithOffset(ThreadType::kWorkerTXRX, core_id_, tid);
  size_t rx_offset = 0;
  int radio_lo = tid * config_->NumRadios() / thread_num_;
  int radio_hi = (tid + 1) * config_->NumRadios() / thread_num_;
  std::printf("Receiver thread %d has %d radios\n", tid, radio_hi - radio_lo);

  int sock_buf_size = 1024 * 1024 * 64 * 8 - 1;
  for (int radio_id = radio_lo; radio_id < radio_hi; ++radio_id) {
    int local_port_id = config_->UeServerPort() + radio_id;
    socket_[radio_id] = SetupSocketIpv4(local_port_id, true, sock_buf_size);
    SetupSockaddrRemoteIpv4(&servaddr_[radio_id],
                            config_->UeRruPort() + radio_id,
                            config_->BsRruAddr().c_str());
    std::printf(
        "TXRX thread %d: set up UDP socket server listening to port %d"
        " with remote address %s:%d \n",
        tid, local_port_id, config_->BsRruAddr().c_str(),
        config_->UeRruPort() + radio_id);
    fcntl(socket_[radio_id], F_SETFL, O_NONBLOCK);
  }

  int radio_id = radio_lo;
  while (config_->Running()) {
    if (-1 != DequeueSend(tid)) {
      continue;
    }
    // receive data
    struct Packet* pkt = RecvEnqueue(tid, radio_id, rx_offset);
    if (pkt == NULL) {
      continue;
    }
    rx_offset = (rx_offset + 1) % buffer_frame_num_;

    if (++radio_id == radio_hi) {
      radio_id = radio_lo;
    }
  }
  return 0;
}

// dequeue_send_sdr
int RadioTxRx::DequeueSendArgos(int tid, long long time0) {
  auto& c = config_;
  auto& radio = radioconfig_;
  int packet_length = c->PacketLength();
  int num_samps = c->SampsPerSymbol();
  int frm_num_samps = num_samps * c->Frame().NumTotalSyms();

  // For UHD devices, first pilot should not be with the END_BURST flag
  // 1: HAS_TIME, 2: HAS_TIME | END_BURST
  int flags_tx_pilot = (kUseUHD && c->NumChannels() == 2) ? 1 : 2;

  EventData event;
  if (!task_queue_->try_dequeue_from_producer(*tx_ptoks_[tid], event)) {
    return -1;
  }

  assert(event.event_type_ == EventType::kPacketTX);

  size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
  size_t ue_id = gen_tag_t(event.tags_[0]).ant_id_;
  size_t tx_frame_id = frame_id + TX_FRAME_DELTA;
  size_t ant_id = ue_id * c->NumChannels();
  long long tx_time(0);
  int r;

  // Transmit pilot
  if (!c->HwFramer()) {
    size_t pilot_symbol_id = c->Frame().GetPilotSymbol(ant_id);

    tx_time = time0 + tx_frame_id * frm_num_samps +
              pilot_symbol_id * num_samps - c->ClTxAdvance()[ue_id];
    r = radio->RadioTx(ue_id, pilot_buff0_.data(), num_samps, flags_tx_pilot,
                       tx_time);
    if (r < num_samps) {
      std::cout << "BAD Write: (PILOT)" << r << "/" << num_samps << std::endl;
    }
    if (c->NumChannels() == 2) {
      pilot_symbol_id = c->Frame().GetPilotSymbol(ant_id + 1);
      tx_time = time0 + tx_frame_id * frm_num_samps +
                pilot_symbol_id * num_samps - c->ClTxAdvance()[ue_id];
      r = radio->RadioTx(ue_id, pilot_buff1_.data(), num_samps, 2, tx_time);
      if (r < num_samps) {
        std::cout << "BAD Write (PILOT): " << r << "/" << num_samps
                  << std::endl;
      }
    }
  }

  // Transmit data
  for (size_t symbol_id = 0; symbol_id < c->Frame().NumULSyms(); symbol_id++) {
    size_t tx_symbol_id = c->Frame().GetULSymbol(symbol_id);
    size_t offset =
        (c->GetTotalDataSymbolIdxUl(frame_id, symbol_id) * c->UeAntNum()) +
        ant_id;

    void* txbuf[2];
    for (size_t ch = 0; ch < c->NumChannels(); ++ch) {
      struct Packet* pkt =
          (struct Packet*)(tx_buffer_ + (offset + ch) * packet_length);
      txbuf[ch] = (void*)pkt->data_;
      tx_buffer_status_[offset + ch] = 0;
    }
    tx_time = c->HwFramer()
                  ? ((long long)tx_frame_id << 32) | (tx_symbol_id << 16)
                  : time0 + tx_frame_id * frm_num_samps +
                        tx_symbol_id * num_samps - c->ClTxAdvance()[ue_id];
    int flags_tx_symbol = 1;  // HAS_TIME
    if (tx_symbol_id == c->Frame().GetULSymbolLast()) {
      flags_tx_symbol = 2;  // HAS_TIME & END_BURST, fixme
    }
    r = radio->RadioTx(ue_id, txbuf, num_samps, flags_tx_symbol, tx_time);
    if (r < num_samps) {
      std::cout << "BAD Write (UL): " << r << "/" << num_samps << std::endl;
    }
  }

  RtAssert(
      message_queue_->enqueue(*rx_ptoks_[tid],
                              EventData(EventType::kPacketTX, event.tags_[0])),
      "Socket message enqueue failed\n");
  return event.tags_[0];
}

struct Packet* RadioTxRx::RecvEnqueueArgos(int tid, size_t radio_id,
                                           size_t& frame_id, size_t& symbol_id,
                                           size_t rx_offset) {
  auto& c = config_;
  moodycamel::ProducerToken* local_ptok = rx_ptoks_[tid];
  char* rx_buffer = (*buffer_)[tid];
  int* rx_buffer_status = (*buffer_status_)[tid];
  int num_samps = c->SampsPerSymbol();
  int packet_length = c->PacketLength();
  ClientRadioConfig* radio = radioconfig_;

  // if buffer is full, exit
  if (rx_buffer_status[rx_offset] == 1) {
    std::printf("RX thread %d at rx_offset %zu buffer full\n", tid, rx_offset);
    c->Running(false);
    return NULL;
  }

  long long rx_time(0);
  struct Packet* pkt[c->NumChannels()];
  void* samp[c->NumChannels()];
  for (size_t ch = 0; ch < c->NumChannels(); ++ch) {
    pkt[ch] =
        (struct Packet*)&rx_buffer[((rx_offset + ch) % buffer_frame_num_) *
                                   packet_length];
    samp[ch] = pkt[ch]->data_;
  }
  int r = radio->RadioRx(radio_id, samp, num_samps, rx_time);
  if (r < num_samps) {
    std::cerr << "BAD Receive(" << r << "/" << num_samps << ") at Time "
              << rx_time << std::endl;
  }
  if (r < 0) {
    std::cerr << "Receive error! Stopping... " << std::endl;
    c->Running(false);
    return NULL;
  }
  if (c->HwFramer()) {
    frame_id = (size_t)(rx_time >> 32);
    symbol_id = (size_t)((rx_time >> 16) & 0xFFFF);
  } else {
    // assert(c->hw_framer
    //    || (((rxTime - time0) / (num_samps * c->symbol_num_perframe))
    //           == frame_id));
  }
  if (kDebugPrintInTask) {
    std::printf(
        "downlink receive: thread %d, frame_id %zu, symbol_id "
        "%zu, radio_id %zu "
        "rxtime %llx\n",
        tid, frame_id, symbol_id, radio_id, rx_time);
  }
  size_t ant_id = radio_id * c->NumChannels();
  for (size_t ch = 0; ch < c->NumChannels(); ++ch) {
    new (pkt[ch]) Packet(frame_id, symbol_id, 0 /* cell_id */, ant_id + ch);

    // get the position in rx_buffer
    // move ptr & set status to full
    rx_buffer_status[rx_offset + ch] = 1;

    EventData rx_message(EventType::kPacketRX,
                         rx_tag_t(tid, rx_offset + ch).tag_);

    RtAssert(message_queue_->enqueue(*local_ptok, rx_message),
             "socket message enqueue failed");
  }
  return pkt[0];
}

void* RadioTxRx::LoopTxRxArgos(int tid) {
  PinToCoreWithOffset(ThreadType::kWorkerTXRX, core_id_, tid);
  auto& c = config_;
  size_t num_radios = c->NumRadios();
  size_t radio_lo = tid * num_radios / thread_num_;
  size_t radio_hi = (tid + 1) * num_radios / thread_num_;
  std::printf("receiver thread %d has radios %zu to %zu (%zu)\n", tid, radio_lo,
              radio_hi - 1, radio_hi - radio_lo);

  // Use mutex to sychronize data receiving across threads
  pthread_mutex_lock(&mutex_);
  std::printf("Thread %d: waiting for release\n", tid);

  pthread_cond_wait(&cond_, &mutex_);
  pthread_mutex_unlock(&mutex_);  // unlocking for all other threads

  ClientRadioConfig* radio = radioconfig_;

  std::vector<int> all_trigs(radio_hi - radio_lo, 0);
  struct timespec tv;
  struct timespec tv2;
  clock_gettime(CLOCK_MONOTONIC, &tv);

  size_t rx_offset = 0;
  size_t frame_id(0);
  size_t symbol_id(0);
  size_t radio_id = radio_lo;
  while (c->Running()) {
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

    struct Packet* pkt =
        RecvEnqueueArgos(tid, radio_id, frame_id, symbol_id, rx_offset);
    if (pkt == NULL) {
      continue;
    }

    rx_offset += c->NumChannels();
    rx_offset %= buffer_frame_num_;
    if (++radio_id == radio_hi) {
      radio_id = radio_lo;
    }
  }
  return 0;
}

void* RadioTxRx::LoopTxRxArgosSync(int tid) {
  // FIXME: This only works when there is 1 radio per thread.
  PinToCoreWithOffset(ThreadType::kWorkerTXRX, core_id_, tid);
  auto& c = config_;

  // Use mutex to sychronize data receiving across threads
  pthread_mutex_lock(&mutex_);
  std::printf("Thread %d: waiting for release\n", tid);

  ClientRadioConfig* radio = radioconfig_;

  int num_samps = c->SampsPerSymbol();
  int frm_num_samps = num_samps * c->Frame().NumTotalSyms();
  std::vector<std::complex<int16_t>> frm_buff0(frm_num_samps, 0);
  std::vector<std::complex<int16_t>> frm_buff1(frm_num_samps, 0);
  std::vector<void*> frm_rx_buff(2);
  frm_rx_buff.at(0) = frm_buff0.data();

  std::vector<std::complex<int16_t>> zeros0(c->SampsPerSymbol(), 0);
  std::vector<std::complex<int16_t>> zeros1(c->SampsPerSymbol(), 0);
  pilot_buff0_.resize(2);
  pilot_buff1_.resize(2);
  pilot_buff0_.at(0) = c->PilotCi16().data();
  if (c->NumChannels() == 2) {
    pilot_buff0_.at(1) = zeros0.data();
    pilot_buff1_.at(0) = zeros1.data();
    pilot_buff1_.at(1) = c->PilotCi16().data();
    frm_rx_buff.at(1) = frm_buff1.data();
  }

  long long rx_time(0);
  int radio_id = tid;
  int sync_index(-1);
  int rx_offset(0);
  size_t cursor(0);
  std::stringstream sout;

  pthread_cond_wait(&cond_, &mutex_);
  pthread_mutex_unlock(&mutex_);  // unlocking for all other threads

  // Keep receiving one frame of data until a beacon is found
  // Perform initial beacon detection every kBeaconDetectInterval frames
  while (c->Running() && sync_index < 0) {
    int r;
    for (size_t find_beacon_retry = 0;
         find_beacon_retry < kBeaconDetectInterval; find_beacon_retry++) {
      r = radio->RadioRx(radio_id, frm_rx_buff.data(), frm_num_samps, rx_time);

      if (r != frm_num_samps) {
        std::cerr << "BAD SYNC Receive(" << r << "/" << frm_num_samps
                  << ") at Time " << rx_time << std::endl;
        continue;
      }
    }

    // convert data to complex float for sync detection
    std::vector<std::complex<float>> sync_buff(frm_num_samps, 0);
    for (int i = 0; i < frm_num_samps; i++) {
      sync_buff[i] = (std::complex<float>(frm_buff0[i].real() / 32768.0,
                                          frm_buff0[i].imag() / 32768.0));
    }
    sync_index = CommsLib::FindBeaconAvx(sync_buff, c->GoldCf32());
    if (sync_index < 0) {
      continue;
    }
    sout << "Client " << radio_id << ": Beacon detected at Time " << rx_time
         << ", sync_index: " << sync_index << std::endl;
    std::cout << sout.str();
    sout.str(std::string());  // clear stringstream after print
    rx_offset = sync_index - c->BeaconLen() - c->OfdmTxZeroPrefix();
  }

  // Read rx_offset to align with the begining of a frame
  radio->RadioRx(radio_id, frm_rx_buff.data(), rx_offset, rx_time);

  long long time0(0);
  size_t frame_id(0);

  bool resync = false;
  int resync_retry_cnt(0);
  int resync_retry_max(100);
  rx_offset = 0;
  while (c->Running()) {
    if (c->FramesToTest() > 0 && frame_id > c->FramesToTest()) {
      c->Running(false);
      break;
    }

    // recv corresponding to symbol_id = 0 (Beacon)
    int r = radio->RadioRx(radio_id, frm_rx_buff.data(), num_samps + rx_offset,
                           rx_time);
    if (r != num_samps + rx_offset) {
      std::cerr << "BAD Beacon Receive(" << r << "/" << num_samps
                << ") at Time " << rx_time << std::endl;
    }
    if (r < 0) {
      std::cerr << "Receive error! Stopping... " << std::endl;
      c->Running(false);
      break;
    }
    if (frame_id == 0) {
      time0 = rx_time;
    }

    // resync every X=1000 frames:
    // TODO: X should be a function of sample rate and max CFO
    if (frame_id / 1000 > 0 && frame_id % 1000 == 0) {
      resync = true;
    }
    rx_offset = 0;
    if (resync) {
      // convert data to complex float for sync detection
      std::vector<std::complex<float>> sync_buff;
      for (int i = 0; i < num_samps; i++) {
        sync_buff.push_back(std::complex<float>(frm_buff0[i].real() / 32768.0,
                                                frm_buff0[i].imag() / 32768.0));
      }
      sync_index = CommsLib::FindBeaconAvx(sync_buff, c->GoldCf32());
      if (sync_index >= 0) {
        rx_offset = sync_index - c->BeaconLen() - c->OfdmTxZeroPrefix();
        time0 += rx_offset;
        resync = false;
        resync_retry_cnt = 0;
        sout << "Client " << radio_id << ": Re-syncing with offset "
             << rx_offset << " after " << resync_retry_cnt + 1 << " tries\n";
        std::cout << sout.str();
        sout.str(std::string());  // clear stringstream after print
      } else {
        resync_retry_cnt++;
      }
    }
    if (resync && resync_retry_cnt > resync_retry_max) {
      sout << "Client " << radio_id << ": Exceeded resync retry limit ("
           << resync_retry_max << "). Stopping..." << std::endl;
      std::cerr << sout.str();
      sout.str(std::string());  // clear stringstream after print
      c->Running(false);
      break;
    }

    // schedule transmit pilots and symbols
    while (-1 != DequeueSendArgos(tid, time0)) {
      ;
    }

    // receive the remaining of the frame
    for (size_t symbol_id = 1; symbol_id < c->Frame().NumTotalSyms();
         symbol_id++) {
      if (!config_->IsPilot(frame_id, symbol_id) &&
          !(config_->IsDownlink(frame_id, symbol_id))) {
        radio->RadioRx(radio_id, frm_rx_buff.data(), num_samps, rx_time);
        if (r < num_samps) {
          std::cerr << "BAD Receive(" << r << "/" << num_samps << ") at Time "
                    << rx_time << std::endl;
        }
        if (r < 0) {
          std::cerr << "Receive error! Stopping... " << std::endl;
          c->Running(false);
          break;
        }
        if (kDebugPrintInTask) {
          std::printf(
              "idle receive: thread %d, frame_id %zu, symbol_id %zu, "
              "radio_id %d "
              "rxtime %llx\n",
              tid, frame_id, symbol_id, radio_id, rx_time);
        }
      } else {
        struct Packet* pkt =
            RecvEnqueueArgos(tid, radio_id, frame_id, symbol_id, cursor);
        if (pkt == NULL) {
          break;
        }

        cursor += c->NumChannels();
        cursor %= buffer_frame_num_;
      }
    }
    frame_id++;
  }
  return 0;
}
