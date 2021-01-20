/**
 * @file txrx_argos.cpp
 * @brief Implementation of PacketTXRX datapath functions for communicating
 * with real Argos hardware
 */

#include "txrx.hpp"

static constexpr bool kDebugDownlink = false;

void PacketTXRX::loop_tx_rx_argos(int tid) {
  pin_to_core_with_offset(ThreadType::kWorkerTXRX, kCoreOffset, tid);
  size_t* rx_frame_start = (*frame_start_)[tid];
  int rx_offset = 0;
  int radio_lo = tid * cfg_->num_radios() / kSocketThreadNum;
  int radio_hi = (tid + 1) * cfg_->num_radios() / kSocketThreadNum;
  std::printf("TXRX thread %d has %d radios\n", tid, radio_hi - radio_lo);

  int prev_frame_id = -1;
  int radio_id = radio_lo;
  while (cfg_->running() == true) {
    if (-1 != dequeue_send_argos(tid)) continue;
    // receive data
    struct Packet* pkt = recv_enqueue_argos(tid, radio_id, rx_offset);
    if (pkt == NULL) continue;
    rx_offset = (rx_offset + cfg_->num_channels()) % packet_num_in_buffer_;

    if (kIsWorkerTimingEnabled) {
      int frame_id = pkt->frame_id_;
      if (frame_id > prev_frame_id) {
        rx_frame_start[frame_id % kNumStatsFrames] = rdtsc();
        prev_frame_id = frame_id;
      }
    }

    if (++radio_id == radio_hi) radio_id = radio_lo;
  }
}

struct Packet* PacketTXRX::recv_enqueue_argos(int tid, int radio_id,
                                              int rx_offset) {
  moodycamel::ProducerToken* local_ptok = rx_ptoks_[tid];
  char* rx_buffer = (*buffer_)[tid];
  int* rx_buffer_status = (*buffer_status_)[tid];
  int packet_length = cfg_->packet_length();

  // if rx_buffer is full, exit
  int n_channels = cfg_->num_channels();
  struct Packet* pkt[n_channels];
  void* samp[n_channels];
  for (int ch = 0; ch < n_channels; ++ch) {
    // if rx_buffer is full, exit
    if (rx_buffer_status[rx_offset + ch] == 1) {
      std::printf("TXRX thread %d rx_buffer full, offset: %d\n", tid,
                  rx_offset);
      cfg_->running(false);
      break;
    }
    pkt[ch] = (struct Packet*)&rx_buffer[(rx_offset + ch) * packet_length];
    samp[ch] = pkt[ch]->data_;
  }

  long long frame_time;
  if ((cfg_->running() == false) ||
      radioconfig_->radioRx(radio_id, samp, frame_time) <= 0) {
    return NULL;
  }

  int frame_id = (int)(frame_time >> 32);
  int symbol_id = (int)((frame_time >> 16) & 0xFFFF);
  int ant_id = radio_id * n_channels;
  for (int ch = 0; ch < n_channels; ++ch) {
    new (pkt[ch]) Packet(frame_id, symbol_id, 0 /* cell_id */, ant_id + ch);
    // move ptr & set status to full
    rx_buffer_status[rx_offset + ch] =
        1;  // has data, after it is read, it is set to 0

    // Push kPacketRX event into the queue.
    EventData rx_message(EventType::kPacketRX,
                          rx_tag_t(tid, rx_offset + ch).tag_);

    if (!message_queue_->enqueue(*local_ptok, rx_message)) {
      std::printf("socket message enqueue failed\n");
      std::exit(0);
    }
  }
  return pkt[0];
}

int PacketTXRX::dequeue_send_argos(int tid) {
  auto& c = cfg_;
  EventData event;
  if (task_queue_->try_dequeue_from_producer(*tx_ptoks_[tid], event) == false) {
    return -1;
  }

  // std::printf("tx queue length: %d\n", task_queue_->size_approx());
  assert(event.event_type_ == EventType::kPacketTX);

  size_t ant_id = gen_tag_t(event.tags_[0]).ant_id_;
  size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
  size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;

  size_t dl_symbol_idx = cfg_->GetDLSymbolIdx(frame_id, symbol_id);
  size_t offset =
      (c->GetTotalDataSymbolIdxDl(frame_id, dl_symbol_idx) * c->bs_ant_num()) +
      ant_id;

  // TODO: support nChannels > 1
  void* txbuf[2];
  int n_channels = c->num_channels();
  int ch = ant_id % n_channels;

  if ((c->frame().IsRecCalEnabled() == true) &&
      (symbol_id == c->frame().GetDLSymbol(0))) {
    std::vector<std::complex<int16_t>> zeros(c->samps_per_symbol(),
                                             std::complex<int16_t>(0, 0));
    for (size_t s = 0; s < c->ant_per_group(); s++) {
      txbuf[ch] =
          (frame_id % c->ant_group_num() == ant_id / c->ant_per_group() &&
           s == ant_id % c->ant_per_group())
              ? c->pilot_ci16().data()
              : zeros.data();
      long long frame_time = ((long long)(frame_id + TX_FRAME_DELTA) << 32) |
                            (c->frame().GetDLCalSymbol(s) << 16);
      radioconfig_->radioTx(ant_id / n_channels, txbuf, 1, frame_time);
    }
  }

  if (kDebugDownlink == true) {
    std::vector<std::complex<int16_t>> zeros(c->samps_per_symbol());
    if (ant_id != c->ref_ant()) {
      txbuf[ch] = zeros.data();
    } else if (dl_symbol_idx < c->frame().client_dl_pilot_symbols()) {
      txbuf[ch] = reinterpret_cast<void*>(c->ue_specific_pilot_t()[0]);
    } else {
      txbuf[ch] = reinterpret_cast<void*>(c->dl_iq_t()[dl_symbol_idx]);
    }
  } else {
    char* cur_buffer_ptr = tx_buffer_ + offset * c->dl_packet_length();
    struct Packet* pkt = (struct Packet*)cur_buffer_ptr;
    txbuf[ch] = (void*)pkt->data_;
  }

  size_t last = c->frame().GetDLSymbolLast();
  int flags = (symbol_id != last) ? 1   // HAS_TIME
                                  : 2;  // HAS_TIME & END_BURST, fixme
  frame_id += TX_FRAME_DELTA;
  long long frame_time = ((long long)frame_id << 32) | (symbol_id << 16);
  radioconfig_->radioTx(ant_id / n_channels, txbuf, flags, frame_time);

  if (kDebugPrintInTask == true) {
    std::printf(
        "In TX thread %d: Transmitted frame %zu, symbol %zu, "
        "ant %zu, offset: %zu, msg_queue_length: %zu\n",
        tid, frame_id, symbol_id, ant_id, offset,
        message_queue_->size_approx());
  }

  rt_assert(
      message_queue_->enqueue(*rx_ptoks_[tid],
                              EventData(EventType::kPacketTX, event.tags_[0])),
      "Socket message enqueue failed\n");
  return event.tags_[0];
}
