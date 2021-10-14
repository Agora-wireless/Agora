/**
 * @file txrx_usrp.cc
 * @brief Implementation of PacketTXRX datapath functions for communicating
 * with USRP hardware
 */

#include "txrx.h"

static constexpr bool kDebugDownlink = false;

void PacketTXRX::LoopTxRxUsrp(size_t tid) {
  PinToCoreWithOffset(ThreadType::kWorkerTXRX, core_offset_, tid);
  size_t* rx_frame_start = (*frame_start_)[tid];
  size_t rx_slot = 0;
  size_t radio_lo = tid * cfg_->NumRadios() / socket_thread_num_;
  size_t radio_hi = (tid + 1) * cfg_->NumRadios() / socket_thread_num_;
  std::printf("LoopTxRxUsrp: TxRx thread %zu has %zu radios\n", tid,
              radio_hi - radio_lo);

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

  size_t global_frame_id = 0;
  size_t global_symbol_id = 0;

  int prev_frame_id = -1;
  size_t radio_id = radio_lo;
  while (cfg_->Running() == true) {
    // transmit data
    // if (-1 != dequeue_send_usrp(tid))
    //   continue;
    // receive data
    struct Packet* pkt = RecvEnqueueUsrp(tid, radio_id, rx_slot,
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
    if (global_symbol_id == cfg_->Frame().NumTotalSyms()) {
      global_symbol_id = 0;
      global_frame_id++;
    }

    if (pkt == nullptr) {
      continue;
    }
    rx_slot = (rx_slot + cfg_->NumChannels()) % buffers_per_socket_;

    if (kIsWorkerTimingEnabled) {
      int frame_id = pkt->frame_id_;
      if (frame_id > prev_frame_id) {
        rx_frame_start[frame_id % kNumStatsFrames] = GetTime::Rdtsc();
        prev_frame_id = frame_id;
      }
    }
  }
}

struct Packet* PacketTXRX::RecvEnqueueUsrp(size_t tid, size_t radio_id,
                                           size_t rx_slot, size_t frame_id,
                                           size_t symbol_id) {
  moodycamel::ProducerToken* local_ptok = rx_ptoks_[tid];

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

  size_t n_channels = cfg_->NumChannels();
  std::vector<void*> samp(n_channels);
  for (size_t ch = 0; ch < n_channels; ++ch) {
    RxPacket& rx = rx_packets_.at(tid).at(rx_slot + ch);
    // if rx_buffer is full, exit
    if (rx.Empty() == false) {
      std::printf("Receive thread %zu rx_buffer full, offset: %zu\n", tid,
                  rx_slot);
      cfg_->Running(false);
      break;
    }
    samp.at(ch) = rx.RawPacket();
  }

  int tmp_ret;
  if (cfg_->IsPilot(frame_id, symbol_id) ||
      cfg_->IsUplink(frame_id, symbol_id)) {
    tmp_ret = radioconfig_->RadioRx(radio_id, samp.data(), rx_time_bs_);
  } else {
    tmp_ret = radioconfig_->RadioRx(radio_id, samp_buffer.data(), rx_time_bs_);
  }

  if ((cfg_->Running() == false) || tmp_ret <= 0 ||
      (!cfg_->IsPilot(frame_id, symbol_id) &&
       !cfg_->IsUplink(frame_id, symbol_id))) {
    return nullptr;
  }

  size_t ant_id = radio_id * n_channels;
  if (cfg_->IsPilot(frame_id, symbol_id) ||
      cfg_->IsUplink(frame_id, symbol_id)) {
    for (size_t ch = 0; ch < n_channels; ++ch) {
      RxPacket& rx = rx_packets_.at(tid).at(rx_slot + ch);
      new (rx.RawPacket()) Packet(frame_id, symbol_id, 0, ant_id + ch);
      rx.Use();
      // Push kPacketRX event into the queue
      EventData rx_message(EventType::kPacketRX, rx_tag_t(rx).tag_);

      if (message_queue_->enqueue(*local_ptok, rx_message) == false) {
        std::printf("socket message enqueue failed\n");
        throw std::runtime_error("PacketTXRX: socket message enqueue failed");
      }
    }
  }
  return rx_packets_.at(tid).at(rx_slot).RawPacket();
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

  symbol_id += c->UeAntTotal();
  frame_id += TX_FRAME_DELTA;

  void* txbuf[2];
  int n_channels = c->NumChannels();
  int ch = ant_id % n_channels;

  if (kDebugDownlink == true) {
    std::vector<std::complex<int16_t>> zeros(c->SampsPerSymbol());
    size_t dl_symbol_idx = c->Frame().GetDLSymbolIdx(symbol_id);
    if (ant_id != c->RefAnt()) {
      txbuf[ch] = zeros.data();
    } else if (dl_symbol_idx < c->Frame().ClientDlPilotSymbols()) {
      txbuf[ch] = reinterpret_cast<void*>(c->UeSpecificPilotT()[0]);
    } else {
      txbuf[ch] = reinterpret_cast<void*>(
          c->DlIqT()[dl_symbol_idx - c->Frame().ClientDlPilotSymbols()]);
    }
  } else {
    char* cur_buffer_ptr = tx_buffer_ + offset * c->PacketLength();
    auto* pkt = reinterpret_cast<struct Packet*>(cur_buffer_ptr);
    txbuf[ch] = reinterpret_cast<void*>(pkt->data_);
  }

  size_t last = c->Frame().GetDLSymbolLast();
  int flags = (symbol_id != last) ? 1   // HAS_TIME
                                  : 2;  // HAS_TIME & END_BURST, fixme
  long long frame_time = ((long long)frame_id << 32) | (symbol_id << 16);
  radioconfig_->RadioTx(ant_id / n_channels, txbuf, flags, frame_time);

  if (kDebugPrintInTask == true) {
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

  // symbol_id += c->UeAntTotal();
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
      txbuf[ch] = reinterpret_cast<void*>(c->UeSpecificPilotT()[0]);
    } else {
      txbuf[ch] = reinterpret_cast<void*>(
          c->DlIqT()[dl_symbol_idx - c->Frame().ClientDlPilotSymbols()]);
    }
  } else {
    char* cur_buffer_ptr = tx_buffer_ + offset * c->PacketLength();
    auto* pkt = reinterpret_cast<struct Packet*>(cur_buffer_ptr);
    txbuf[ch] = reinterpret_cast<void*>(pkt->data_);
  }

  size_t last = c->Frame().GetDLSymbolLast();
  int flags = (symbol_id != static_cast<int>(last))
                  ? 1   // HAS_TIME
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
