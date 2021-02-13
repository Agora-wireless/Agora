/**
 * @file txrx_usrp.cc
 * @brief Implementation of PacketTXRX datapath functions for communicating
 * with USRP hardware
 */

#include "txrx.h"

static constexpr bool kDebugDownlink = false;

void PacketTXRX::LoopTxRxUsrp(int tid) {
  PinToCoreWithOffset(ThreadType::kWorkerTXRX, core_offset_, tid);
  size_t* rx_frame_start = (*frame_start_)[tid];
  int rx_offset = 0;
  int radio_lo = tid * cfg_->NumRadios() / socket_thread_num_;
  int radio_hi = (tid + 1) * cfg_->NumRadios() / socket_thread_num_;
  std::printf("receiver thread %d has %d radios\n", tid, radio_hi - radio_lo);

  // prepare BS beacon in host buffer
  std::vector<void*> beaconbuff(2);
  std::vector<void*> zeros(2);
  zeros[0] = calloc(cfg_->SampsPerSymbol(), sizeof(int16_t) * 2);
  zeros[1] = calloc(cfg_->SampsPerSymbol(), sizeof(int16_t) * 2);
  beaconbuff[0] = cfg_->BeaconCi16().data();
  beaconbuff[1] = zeros[0];

  std::vector<std::complex<int16_t>> samp_buffer0(cfg_->SampsPerSymbol() * 14,
                                                  0);
  std::vector<std::complex<int16_t>> samp_buffer1(cfg_->SampsPerSymbol() * 14,
                                                  0);
  std::vector<void*> samp_buffer(2);
  samp_buffer[0] = samp_buffer0.data();
  if (true) {
    samp_buffer[1] = samp_buffer1.data();
  }

  // long long rxTimeBs(0);
  // long long txTimeBs(0);
  rx_time_bs_ = 0;
  tx_time_bs_ = 0;

  std::cout << "Sync BS host and FGPA timestamp..." << std::endl;
  radioconfig_->RadioRx(0, samp_buffer.data(), rx_time_bs_);
  // Schedule the first beacon in the future
  tx_time_bs_ =
      rx_time_bs_ + cfg_->SampsPerSymbol() * cfg_->Frame().NumTotalSyms() * 40;
  radioconfig_->RadioTx(0, beaconbuff.data(), 2, tx_time_bs_);
  long long bs_init_rx_offset = tx_time_bs_ - rx_time_bs_;
  for (int it = 0; it < std::floor(bs_init_rx_offset / cfg_->SampsPerSymbol());
       it++) {
    radioconfig_->RadioRx(0, samp_buffer.data(), rx_time_bs_);
  }

  std::cout << std::endl;
  std::cout << "Init BS sync done..." << std::endl;
  std::cout << "Start BS main recv loop..." << std::endl;

  int global_frame_id = 0;
  int global_symbol_id = 0;

  int prev_frame_id = -1;
  int radio_id = radio_lo;
  while (cfg_->Running()) {
    // transmit data
    // if (-1 != dequeue_send_usrp(tid))
    //   continue;
    // receive data
    // struct Packet* pkt = recv_enqueue_usrp(tid, radio_id, rx_offset);
    struct Packet* pkt = RecvEnqueueUsrp(tid, radio_id, rx_offset,
                                         global_frame_id, global_symbol_id);

    // Schedule beacon in the future
    if (global_symbol_id == 0) {
      tx_time_bs_ = rx_time_bs_ +
                    cfg_->SampsPerSymbol() * cfg_->Frame().NumTotalSyms() * 20;
      int tx_ret = radioconfig_->RadioTx(0, beaconbuff.data(), 2, tx_time_bs_);
      if (tx_ret != (int)cfg_->SampsPerSymbol()) {
        std::cerr << "BAD Transmit(" << tx_ret << "/" << cfg_->SampsPerSymbol()
                  << ") at Time " << tx_time_bs_ << ", frame count "
                  << global_frame_id << std::endl;
      }
    }

    if (++radio_id == radio_hi) {
      radio_id = radio_lo;
    }

    // Update global frame_id and symbol_id
    global_symbol_id++;
    if (global_symbol_id == (int)cfg_->Frame().NumTotalSyms()) {
      global_symbol_id = 0;
      global_frame_id++;
    }

    if (pkt == nullptr) {
      continue;
    }
    rx_offset = (rx_offset + cfg_->NumChannels()) % packet_num_in_buffer_;

    if (kIsWorkerTimingEnabled) {
      int frame_id = pkt->frame_id_;
      // int symbol_id = pkt->symbol_id;
      if (frame_id > prev_frame_id) {
        rx_frame_start[frame_id % kNumStatsFrames] = Rdtsc();
        prev_frame_id = frame_id;
      }
    }
  }
}

struct Packet* PacketTXRX::RecvEnqueueUsrp(int tid, int radio_id, int rx_offset,
                                           int frame_id, int symbol_id) {
  moodycamel::ProducerToken* local_ptok = rx_ptoks_[tid];
  char* rx_buffer = (*buffer_)[tid];
  int* rx_buffer_status = (*buffer_status_)[tid];
  int packet_length = cfg_->PacketLength();

  // init samp_buffer for dummy read
  std::vector<std::complex<int16_t>> samp_buffer0(
      cfg_->SampsPerSymbol() * cfg_->Frame().NumTotalSyms(), 0);
  std::vector<std::complex<int16_t>> samp_buffer1(
      cfg_->SampsPerSymbol() * cfg_->Frame().NumTotalSyms(), 0);
  std::vector<void*> samp_buffer(2);
  samp_buffer[0] = samp_buffer0.data();
  if (cfg_->NumChannels() == 2) {
    samp_buffer[1] = samp_buffer1.data();
  }

  // if rx_buffer is full, exit
  int n_channels = cfg_->NumChannels();
  struct Packet* pkt[n_channels];
  void* samp[n_channels];
  for (int ch = 0; ch < n_channels; ++ch) {
    // if rx_buffer is full, exit
    if (rx_buffer_status[rx_offset + ch] == 1) {
      std::printf("Receive thread %d rx_buffer full, offset: %d\n", tid,
                  rx_offset);
      cfg_->Running(false);
      break;
    }
    pkt[ch] = (struct Packet*)&rx_buffer[(rx_offset + ch) * packet_length];
    samp[ch] = pkt[ch]->data_;
  }

  int tmp_ret;

  if (cfg_->IsPilot(frame_id, symbol_id) ||
      cfg_->IsUplink(frame_id, symbol_id)) {
    tmp_ret = radioconfig_->RadioRx(radio_id, samp, rx_time_bs_);
  } else {
    tmp_ret = radioconfig_->RadioRx(radio_id, samp_buffer.data(), rx_time_bs_);
  }

  if (!cfg_->Running() || tmp_ret <= 0 ||
      (!cfg_->IsPilot(frame_id, symbol_id) &&
       !cfg_->IsUplink(frame_id, symbol_id))) {
    return nullptr;
  }

  int ant_id = radio_id * n_channels;
  if (cfg_->IsPilot(frame_id, symbol_id) ||
      cfg_->IsUplink(frame_id, symbol_id)) {
    for (int ch = 0; ch < n_channels; ++ch) {
      new (pkt[ch]) Packet(frame_id, symbol_id, 0, ant_id + ch);
      // move ptr & set status to full
      rx_buffer_status[rx_offset + ch] =
          1;  // has data, after it is read, it is set to 0

      // Push kPacketRX event into the queue
      EventData rx_message(EventType::kPacketRX,
                           rx_tag_t(tid, rx_offset + ch).tag_);

      if (!message_queue_->enqueue(*local_ptok, rx_message)) {
        std::printf("socket message enqueue failed\n");
        std::exit(0);
      }
    }
  }

  return pkt[0];
}

int PacketTXRX::DequeueSendUsrp(int tid) {
  auto& c = cfg_;
  EventData event;
  if (!task_queue_->try_dequeue_from_producer(*tx_ptoks_[tid], event)) {
    return -1;
  }

  std::printf("tx queue length: %zu\n", task_queue_->size_approx());
  assert(event.event_type_ == EventType::kPacketTX);

  size_t ant_id = gen_tag_t(event.tags_[0]).ant_id_;
  size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
  size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;

  size_t offset =
      (c->GetTotalDataSymbolIdx(frame_id, symbol_id) * c->BsAntNum()) + ant_id;

  symbol_id += c->UeAntNum();
  frame_id += TX_FRAME_DELTA;

  void* txbuf[2];
  int n_channels = c->NumChannels();
  int ch = ant_id % n_channels;

  if (kDebugDownlink) {
    std::vector<std::complex<int16_t>> zeros(c->SampsPerSymbol());
    size_t dl_symbol_idx = c->Frame().GetDLSymbolIdx(symbol_id);
    if (ant_id != c->RefAnt()) {
      txbuf[ch] = zeros.data();
    } else if (dl_symbol_idx < c->Frame().ClientDlPilotSymbols()) {
      txbuf[ch] = (void*)c->UeSpecificPilotT()[0];
    } else {
      txbuf[ch] =
          (void*)c->DlIqT()[dl_symbol_idx - c->Frame().ClientDlPilotSymbols()];
    }
  } else {
    char* cur_buffer_ptr = tx_buffer_ + offset * c->PacketLength();
    struct Packet* pkt = (struct Packet*)cur_buffer_ptr;
    txbuf[ch] = (void*)pkt->data_;
  }

  size_t last = c->Frame().GetDLSymbolLast();
  int flags = (symbol_id != last) ? 1   // HAS_TIME
                                  : 2;  // HAS_TIME & END_BURST, fixme
  long long frame_time = ((long long)frame_id << 32) | (symbol_id << 16);
  radioconfig_->RadioTx(ant_id / n_channels, txbuf, flags, frame_time);

  if (kDebugPrintInTask) {
    std::printf(
        "In TX thread %d: Transmitted frame %zu, symbol %zu, "
        "ant %zu, offset: %zu, msg_queue_length: %zu\n",
        tid, frame_id, symbol_id, ant_id, offset,
        message_queue_->size_approx());
  }

  RtAssert(
      message_queue_->enqueue(*rx_ptoks_[tid],
                              EventData(EventType::kPacketTX, event.tags_[0])),
      "Socket message enqueue failed\n");
  return event.tags_[0];
}

int PacketTXRX::DequeueSendUsrp(int tid, int frame_id, int symbol_id) {
  auto& c = cfg_;
  EventData event;
  if (!task_queue_->try_dequeue_from_producer(*tx_ptoks_[tid], event)) {
    return -1;
  }
  std::cout << "DDD" << std::endl;

  std::printf("tx queue length: %zu\n", task_queue_->size_approx());
  assert(event.event_type_ == EventType::kPacketTX);

  size_t ant_id = gen_tag_t(event.tags_[0]).ant_id_;
  // size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
  // size_t symbol_id = gen_tag_t(event.tags[0]).symbol_id;

  size_t offset =
      (c->GetTotalDataSymbolIdx(frame_id, symbol_id) * c->BsAntNum()) + ant_id;

  // symbol_id += c->UeAntNum();
  // frame_id += TX_FRAME_DELTA;

  std::cout << "ant_id: " << ant_id << ", frame_id: " << frame_id
            << ", symbol_id: " << symbol_id;
  std::cout << "offset: " << offset << std::endl;

  void* txbuf[2];
  int n_channels = c->NumChannels();
  int ch = ant_id % n_channels;

  if (kDebugDownlink) {
    std::vector<std::complex<int16_t>> zeros(c->SampsPerSymbol());
    size_t dl_symbol_idx = c->Frame().GetDLSymbolIdx(symbol_id);
    if (ant_id != c->RefAnt()) {
      txbuf[ch] = zeros.data();
    } else if (dl_symbol_idx < c->Frame().ClientDlPilotSymbols()) {
      txbuf[ch] = (void*)c->UeSpecificPilotT()[0];
    } else {
      txbuf[ch] =
          (void*)c->DlIqT()[dl_symbol_idx - c->Frame().ClientDlPilotSymbols()];
    }
  } else {
    char* cur_buffer_ptr = tx_buffer_ + offset * c->PacketLength();
    struct Packet* pkt = (struct Packet*)cur_buffer_ptr;
    txbuf[ch] = (void*)pkt->data_;
  }

  size_t last = c->Frame().GetDLSymbolLast();
  int flags = (symbol_id != (int)last) ? 1   // HAS_TIME
                                       : 2;  // HAS_TIME & END_BURST, fixme
  long long frame_time = ((long long)frame_id << 32) | (symbol_id << 16);
  radioconfig_->RadioTx(ant_id / n_channels, txbuf, flags, frame_time);

  if (kDebugPrintInTask) {
    std::printf(
        "In TX thread %d: Transmitted frame %d, symbol %d, "
        "ant %zu, offset: %zu, msg_queue_length: %zu\n",
        tid, frame_id, symbol_id, ant_id, offset,
        message_queue_->size_approx());
  }

  RtAssert(
      message_queue_->enqueue(*rx_ptoks_[tid],
                              EventData(EventType::kPacketTX, event.tags_[0])),
      "Socket message enqueue failed\n");
  return event.tags_[0];
}
