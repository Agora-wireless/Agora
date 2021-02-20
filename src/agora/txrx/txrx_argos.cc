/**
 * @file txrx_argos.cc
 * @brief Implementation of PacketTXRX datapath functions for communicating
 * with real Argos hardware
 */
#include "logger.h"
#include "txrx.h"

static constexpr bool kDebugDownlink = false;

void PacketTXRX::LoopTxRxArgos(int tid) {
  PinToCoreWithOffset(ThreadType::kWorkerTXRX, core_offset_, tid);
  size_t* rx_frame_start = (*frame_start_)[tid];
  int rx_offset = 0;
  int radio_lo = tid * cfg_->NumRadios() / socket_thread_num_;
  int radio_hi = (tid + 1) * cfg_->NumRadios() / socket_thread_num_;
  MLPD_INFO("TXRX thread %d has %d radios\n", tid, radio_hi - radio_lo);

  int prev_frame_id = -1;
  int radio_id = radio_lo;
  while (cfg_->Running() == true) {
    if (-1 != DequeueSendArgos(tid)) {
      continue;
    }
    // receive data
    struct Packet* pkt = RecvEnqueueArgos(tid, radio_id, rx_offset);
    if (pkt == nullptr) {
      continue;
    }
    rx_offset = (rx_offset + cfg_->NumChannels()) % packet_num_in_buffer_;

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

struct Packet* PacketTXRX::RecvEnqueueArgos(int tid, int radio_id,
                                            int rx_offset) {
  moodycamel::ProducerToken* local_ptok = rx_ptoks_[tid];
  char* rx_buffer = (*buffer_)[tid];
  int* rx_buffer_status = (*buffer_status_)[tid];
  int packet_length = cfg_->PacketLength();

  // if rx_buffer is full, exit
  int n_channels = cfg_->NumChannels();
  struct Packet* pkt[n_channels];
  void* samp[n_channels];
  for (int ch = 0; ch < n_channels; ++ch) {
    // if rx_buffer is full, exit
    if (rx_buffer_status[rx_offset + ch] == 1) {
      MLPD_ERROR("TXRX thread %d rx_buffer full, offset: %d\n", tid, rx_offset);
      cfg_->Running(false);
      break;
    }
    pkt[ch] = (struct Packet*)&rx_buffer[(rx_offset + ch) * packet_length];
    samp[ch] = pkt[ch]->data_;
  }

  long long frame_time;
  if ((cfg_->Running() == false) ||
      radioconfig_->RadioRx(radio_id, samp, frame_time) <= 0) {
    return nullptr;
  }

  int frame_id = (int)(frame_time >> 32);
  int symbol_id = (int)((frame_time >> 16) & 0xFFFF);
  int ant_id = radio_id * n_channels;
  for (int ch = 0; ch < n_channels; ++ch) {
    if (radio_id == (int)cfg_->RefAnt() / n_channels &&
        ant_id + ch !=
            (int)cfg_->RefAnt())  // ignore second channel in ref radio
      continue;
    new (pkt[ch]) Packet(frame_id, symbol_id, 0 /* cell_id */, ant_id + ch);
    // move ptr & set status to full
    rx_buffer_status[rx_offset + ch] =
        1;  // has data, after it is read, it is set to 0

    // Push kPacketRX event into the queue.
    EventData rx_message(EventType::kPacketRX,
                         rx_tag_t(tid, rx_offset + ch).tag_);

    if (message_queue_->enqueue(*local_ptok, rx_message) == false) {
      std::printf("socket message enqueue failed\n");
      throw std::runtime_error("PacketTXRX: socket message enqueue failed");
    }
  }
  return pkt[0];
}

int PacketTXRX::DequeueSendArgos(int tid) {
  auto& c = cfg_;
  std::array<EventData, 4> event;
  if (task_queue_->try_dequeue_bulk_from_producer(*tx_ptoks_[tid], event.data(),
                                                  c->NumChannels()) == 0) {
    return -1;
  }

  // std::printf("tx queue length: %d\n", task_queue_->size_approx());
  assert(event.event_type_ == EventType::kPacketTX);

  size_t frame_id = gen_tag_t(event.at(0).tags_[0]).frame_id_;
  size_t symbol_id = gen_tag_t(event.at(0).tags_[0]).symbol_id_;
  size_t ant_id = gen_tag_t(event.at(0).tags_[0]).ant_id_;

  size_t dl_symbol_idx = cfg_->Frame().GetDLSymbolIdx(symbol_id);
  size_t offset =
      (c->GetTotalDataSymbolIdxDl(frame_id, dl_symbol_idx) * c->BsAntNum()) +
      ant_id;

  // TODO: support nChannels > 1
  void* caltxbuf[2];
  auto channels = Utils::StrToChannels(c->Channel());

  if ((c->Frame().IsRecCalEnabled() == true) &&
      (symbol_id == c->Frame().GetDLSymbol(0)) && (ant_id != c->RefAnt())) {
    std::vector<std::complex<int16_t>> zeros(c->SampsPerSymbol(),
                                             std::complex<int16_t>(0, 0));
    for (size_t s = 0; s < c->AntPerGroup(); s++) {
      size_t ch = channels[s % c->NumChannels()];
      caltxbuf[ch] =
          (frame_id % c->AntGroupNum() == ant_id / c->AntPerGroup() &&
           s == ant_id % c->AntPerGroup())
              ? c->PilotCi16().data()
              : zeros.data();
      if (channels.size() > 1) caltxbuf[1 - ch] = zeros.data();
      long long frame_time = ((long long)(frame_id + TX_FRAME_DELTA) << 32) |
                             (c->Frame().GetDLCalSymbol(s) << 16);
      radioconfig_->RadioTx(ant_id / c->NumChannels(), caltxbuf, 1, frame_time);
    }
  }

  void* txbuf[2];
  if (kDebugDownlink == true) {
    std::vector<std::complex<int16_t>> zeros(c->SampsPerSymbol());
    size_t ch = channels[0];
    if (ant_id != 0) {
      txbuf[ch] = zeros.data();
    } else if (dl_symbol_idx < c->Frame().ClientDlPilotSymbols()) {
      txbuf[ch] = reinterpret_cast<void*>(c->UeSpecificPilotT()[0]);
    } else {
      txbuf[ch] = reinterpret_cast<void*>(c->DlIqT()[dl_symbol_idx]);
    }
  } else {
    for (size_t ch = 0; ch < c->NumChannels(); ch++) {
      char* cur_buffer_ptr = tx_buffer_ + (offset + ch) * c->DlPacketLength();
      auto* pkt = reinterpret_cast<struct Packet*>(cur_buffer_ptr);
      txbuf[ch] = (void*)pkt->data_;
    }
  }

  size_t last = c->Frame().GetDLSymbolLast();
  int flags = (symbol_id != last) ? 1   // HAS_TIME
                                  : 2;  // HAS_TIME & END_BURST, fixme
  frame_id += TX_FRAME_DELTA;
  long long frame_time = ((long long)frame_id << 32) | (symbol_id << 16);
  radioconfig_->RadioTx(ant_id / c->NumChannels(), txbuf, flags, frame_time);

  if (kDebugPrintInTask == true) {
    std::printf(
        "In TX thread %d: Transmitted frame %zu, symbol %zu, "
        "ant %zu, offset: %zu, msg_queue_length: %zu\n",
        tid, frame_id, symbol_id, ant_id, offset,
        message_queue_->size_approx());
  }

  for (size_t i = 0; i < c->NumChannels(); i++) {
    RtAssert(message_queue_->enqueue(
                 *rx_ptoks_[tid],
                 EventData(EventType::kPacketTX, event.at(i).tags_[0])),
             "Socket message enqueue failed\n");
  }
  return event.at(0).tags_[0];
}
