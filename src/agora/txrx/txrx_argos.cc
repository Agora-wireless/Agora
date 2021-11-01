/**
 * @file txrx_argos.cc
 * @brief Implementation of PacketTXRX datapath functions for communicating
 * with real Argos hardware
 */
#include "logger.h"
#include "txrx.h"

static constexpr bool kDebugDownlink = false;

void PacketTXRX::LoopTxRxArgos(size_t tid) {
  PinToCoreWithOffset(ThreadType::kWorkerTXRX, core_offset_, tid);

  size_t* const rx_frame_start = (*frame_start_)[tid];
  size_t rx_slot = 0;
  size_t radios_per_thread = (cfg_->NumRadios() / socket_thread_num_);
  if (cfg_->NumRadios() % socket_thread_num_ > 0) {
    radios_per_thread++;
  }
  const size_t radio_lo = tid * radios_per_thread;
  const size_t radio_hi =
      std::min((radio_lo + radios_per_thread), cfg_->BsAntNum()) - 1;
  MLPD_INFO("LoopTxRxArgos[%zu] has %zu:%zu total radios %zu\n", tid, radio_lo,
            radio_hi - 1, radio_hi - radio_lo);

  ssize_t prev_frame_id = -1;
  size_t radio_id = radio_lo;
  while (cfg_->Running() == true) {
    if (0 == DequeueSendArgos(tid)) {
      // receive data
      auto pkts = RecvEnqueueArgos(tid, radio_id, rx_slot);
      if (pkts.size() > 0) {
        rx_slot = (rx_slot + pkts.size()) % buffers_per_socket_;
        RtAssert(pkts.size() == cfg_->NumChannels(),
                 "Received data but it was the wrong dimension");

        if (kIsWorkerTimingEnabled) {
          const int frame_id = pkts.front()->frame_id_;
          if (frame_id > prev_frame_id) {
            rx_frame_start[frame_id % kNumStatsFrames] = GetTime::Rdtsc();
            prev_frame_id = frame_id;
          }
        }
        if (++radio_id == (radio_hi + 1)) {
          radio_id = radio_lo;
        }
      }  // if (pkt.size() > 0)
    }    // DequeueSendArgos(tid) == 0
  }      // cfg_->Running() == true
}

std::vector<Packet*> PacketTXRX::RecvEnqueueArgos(size_t tid, size_t radio_id,
                                                  size_t rx_slot) {
  moodycamel::ProducerToken* local_ptok = rx_ptoks_[tid];
  size_t packet_length = cfg_->PacketLength();

  size_t ant_id = radio_id * cfg_->NumChannels();
  std::vector<size_t> ant_ids(cfg_->NumChannels());
  std::vector<void*> samp(cfg_->NumChannels());

  for (size_t ch = 0; ch < cfg_->NumChannels(); ++ch) {
    RxPacket& rx = rx_packets_.at(tid).at(rx_slot + ch);

    // if rx_buffer is full, exit
    if (rx.Empty() == false) {
      MLPD_ERROR("TXRX thread %zu rx_buffer full, rx slot: %zu\n", tid,
                 rx_slot);
      cfg_->Running(false);
      break;
    }
    ant_ids.at(ch) = ant_id + ch;
    samp.at(ch) = rx.RawPacket()->data_;
  }

  long long frame_time;
  if ((cfg_->Running() == false) ||
      radioconfig_->RadioRx(radio_id, samp.data(), frame_time) <= 0) {
    std::vector<Packet*> empty_pkt;
    return empty_pkt;
  }

  size_t frame_id = (size_t)(frame_time >> 32);
  size_t symbol_id = (size_t)((frame_time >> 16) & 0xFFFF);
  std::vector<size_t> symbol_ids(cfg_->NumChannels(), symbol_id);

  /// \TODO: What if ref_ant is set to the second channel?
  if ((cfg_->Frame().IsRecCalEnabled() == true) &&
      (cfg_->IsCalDlPilot(frame_id, symbol_id) == true) &&
      (radio_id == cfg_->RefRadio()) && (cfg_->AntPerGroup() > 1)) {
    if (cfg_->AntPerGroup() > cfg_->NumChannels()) {
      symbol_ids.resize(cfg_->AntPerGroup());
      ant_ids.resize(cfg_->AntPerGroup());
    }
    for (size_t s = 1; s < cfg_->AntPerGroup(); s++) {
      RxPacket& rx = rx_packets_.at(tid).at(rx_slot + s);

      std::vector<void*> tmp_samp(cfg_->NumChannels());
      std::vector<char> dummy_buff(packet_length);
      tmp_samp.at(0) = rx.RawPacket()->data_;
      tmp_samp.at(1) = dummy_buff.data();
      if ((cfg_->Running() == false) ||
          radioconfig_->RadioRx(radio_id, tmp_samp.data(), frame_time) <= 0) {
        std::vector<Packet*> empty_pkt;
        return empty_pkt;
      }
      symbol_ids.at(s) = cfg_->Frame().GetDLCalSymbol(s);
      ant_ids.at(s) = ant_id;
    }
  }

  std::vector<Packet*> pkt;
  for (size_t ch = 0; ch < symbol_ids.size(); ++ch) {
    RxPacket& rx = rx_packets_.at(tid).at(rx_slot + ch);
    pkt.push_back(rx.RawPacket());
    new (rx.RawPacket())
        Packet(frame_id, symbol_ids.at(ch), 0 /* cell_id */, ant_ids.at(ch));

    rx.Use();
    // Push kPacketRX event into the queue.
    EventData rx_message(EventType::kPacketRX, rx_tag_t(rx).tag_);

    if (message_queue_->enqueue(*local_ptok, rx_message) == false) {
      std::printf("socket message enqueue failed\n");
      throw std::runtime_error("PacketTXRX: socket message enqueue failed");
    }
  }
  return pkt;
}

size_t PacketTXRX::DequeueSendArgos(int tid) {
  const size_t max_dequeue_items =
      (cfg_->BsAntNum() / cfg_->SocketThreadNum()) + 1;

  std::vector<EventData> events(max_dequeue_items);

  // Single producer ordering in q is preserved
  const size_t dequeued_items = task_queue_->try_dequeue_bulk_from_producer(
      *tx_ptoks_[tid], events.data(), events.size());

  for (size_t item = 0; item < dequeued_items; item++) {
    EventData& current_event = events.at(item);

    // std::printf("tx queue length: %d\n", task_queue_->size_approx());
    assert(current_event.event_type_ == EventType::kPacketTX);

    size_t frame_id = gen_tag_t(current_event.tags_[0u]).frame_id_;
    const size_t symbol_id = gen_tag_t(current_event.tags_[0u]).symbol_id_;
    const size_t ant_id = gen_tag_t(current_event.tags_[0u]).ant_id_;
    const size_t radio_id = ant_id / cfg_->NumChannels();

    // See if this is the last antenna on the radio.  Assume that we receive the
    // last one
    // last (and all the others came before).  No explicit tracking
    const bool last_antenna =
        ((ant_id % cfg_->NumChannels()) + 1) == (cfg_->NumChannels());

    //std::printf("PacketTXRX[%d]: tx antenna %zu radio %zu is last %d\n", tid,
    //            ant_id, radio_id, last_antenna);

    const size_t dl_symbol_idx = cfg_->Frame().GetDLSymbolIdx(symbol_id);
    const size_t offset =
        (cfg_->GetTotalDataSymbolIdxDl(frame_id, dl_symbol_idx) *
         cfg_->BsAntNum()) +
        ant_id;

    if (last_antenna) {
      // Transmit downlink calibration (array to ref) pilot
      std::vector<void*> caltxbuf(cfg_->NumChannels());
      if ((cfg_->Frame().IsRecCalEnabled() == true) &&
          (symbol_id == cfg_->Frame().GetDLSymbol(0)) &&
          (radio_id != cfg_->RefRadio())) {
        std::vector<std::complex<int16_t>> zeros(cfg_->SampsPerSymbol(),
                                                 std::complex<int16_t>(0, 0));
        for (size_t s = 0; s < cfg_->RadioPerGroup(); s++) {
          bool calib_turn = (frame_id % cfg_->RadioGroupNum() ==
                                 radio_id / cfg_->RadioPerGroup() &&
                             s == radio_id % cfg_->RadioPerGroup());
          for (size_t ch = 0; ch < cfg_->NumChannels(); ch++) {
            caltxbuf.at(ch) =
                calib_turn ? cfg_->PilotCi16().data() : zeros.data();
            if (cfg_->NumChannels() > 1) caltxbuf.at(1 - ch) = zeros.data();
            long long frame_time =
                ((long long)(frame_id + TX_FRAME_DELTA) << 32) |
                (cfg_->Frame().GetDLCalSymbol(s * cfg_->NumChannels() + ch)
                 << 16);
            radioconfig_->RadioTx(radio_id, caltxbuf.data(), 1, frame_time);
          }
        }
      }

      std::vector<void*> txbuf(cfg_->NumChannels());
      if (kDebugDownlink == true) {
        std::vector<std::complex<int16_t>> zeros(cfg_->SampsPerSymbol());
        for (size_t ch = 0; ch < cfg_->NumChannels(); ch++) {
          if (ant_id != 0) {
            txbuf.at(ch) = zeros.data();
          } else if (dl_symbol_idx < cfg_->Frame().ClientDlPilotSymbols()) {
            txbuf.at(ch) = reinterpret_cast<void*>(cfg_->UeSpecificPilotT()[0]);
          } else {
            txbuf.at(ch) =
                reinterpret_cast<void*>(cfg_->DlIqT()[dl_symbol_idx]);
          }
          if (cfg_->NumChannels() > 1) {
            txbuf.at(1 - ch) = zeros.data();
          }
        }
      } else {
        for (size_t ch = 0; ch < cfg_->NumChannels(); ch++) {
          char* cur_buffer_ptr =
              tx_buffer_ + (offset + ch) * cfg_->DlPacketLength();
          auto* pkt = reinterpret_cast<Packet*>(cur_buffer_ptr);
          txbuf.at(ch) = reinterpret_cast<void*>(pkt->data_);
        }
      }

      const size_t last = cfg_->Frame().GetDLSymbolLast();
      const int flags = (symbol_id != last) ? 1   // HAS_TIME
                                            : 2;  // HAS_TIME & END_BURST, fixme
      frame_id += TX_FRAME_DELTA;
      long long frame_time = ((long long)frame_id << 32) | (symbol_id << 16);
      radioconfig_->RadioTx(radio_id, txbuf.data(), flags, frame_time);
    }

    if (kDebugPrintInTask == true) {
      std::printf(
          "In TX thread %d: Transmitted frame %zu, symbol %zu, ant %zu, "
          "offset: %zu, msg_queue_length: %zu\n",
          tid, frame_id, symbol_id, ant_id, offset,
          message_queue_->size_approx());
    }

    RtAssert(message_queue_->enqueue(
                 *rx_ptoks_[tid],
                 EventData(EventType::kPacketTX, current_event.tags_[0])),
             "Socket message enqueue failed\n");
  }
  return dequeued_items;
}
