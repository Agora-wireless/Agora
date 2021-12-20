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
      std::min((radio_lo + radios_per_thread), cfg_->NumRadios()) - 1;

  threads_started_.fetch_add(1);

  long long time0 = 0;
  if (cfg_->HwFramer() == false) {
    // prepare BS beacon in host buffer
    std::vector<void*> beaconbuff(2);
    std::vector<void*> zeros(2);
    zeros[0] = calloc(cfg_->SampsPerSymbol(), sizeof(int16_t) * 2);
    zeros[1] = calloc(cfg_->SampsPerSymbol(), sizeof(int16_t) * 2);
    beaconbuff[0] = cfg_->BeaconCi16().data();
    beaconbuff[1] = zeros[0];

    std::vector<std::complex<int16_t>> samp_buffer0(cfg_->SampsPerSymbol(), 0);
    std::vector<std::complex<int16_t>> samp_buffer1(cfg_->SampsPerSymbol(), 0);
    std::vector<void*> samp_buffer(2);
    samp_buffer[0] = samp_buffer0.data();
    if (cfg_->NumChannels() > 1) {
      samp_buffer[1] = samp_buffer1.data();
    }

    long long rx_time_bs = 0;
    long long tx_time_bs = 0;
    size_t frame_time = cfg_->SampsPerSymbol() * cfg_->Frame().NumTotalSyms();

    std::cout << "Sync BS host and FGPA timestamp..." << std::endl;
    int rx_ret = 0;
    int tx_ret = 0;
    while ((rx_ret = radioconfig_->RadioRx(radio_lo, samp_buffer.data(),
                                           rx_time_bs)) < 0)
      ;
    tx_time_bs = rx_time_bs + frame_time * TX_FRAME_DELTA;
    time0 = tx_time_bs;
    std::cout << "Received first data at time " << rx_time_bs << " on thread "
              << tid << std::endl;
    std::cout << "Transmit first beacon at time " << tx_time_bs << " on thread "
              << tid << std::endl;
    for (auto radio = radio_lo + 1; radio < radio_hi; radio++)
      radioconfig_->RadioRx(radio_lo, samp_buffer.data(), rx_time_bs);
    // Schedule the first beacon in the future
    size_t beacon_radio = cfg_->BeaconAnt() / cfg_->NumChannels();
    for (auto radio = radio_lo; radio < radio_hi; radio++) {
      if (radio == beacon_radio)
        tx_ret = radioconfig_->RadioTx(radio, beaconbuff.data(), 2, tx_time_bs);
      else
        tx_ret = radioconfig_->RadioTx(radio, zeros.data(), 2, tx_time_bs);
      if (tx_ret != (int)cfg_->SampsPerSymbol()) {
        std::cerr << "BAD Transmit(" << tx_ret << "/" << cfg_->SampsPerSymbol()
                  << ") at Time " << tx_time_bs << std::endl;
      }
    }
    for (size_t sym = 1; sym < cfg_->Frame().NumTotalSyms(); sym++) {
      for (auto radio = radio_lo; radio < radio_hi; radio++)
        radioconfig_->RadioRx(radio, samp_buffer.data(), rx_time_bs);
    }
    for (size_t frm = 1; frm < TX_FRAME_DELTA; frm++) {
      tx_time_bs += frame_time;
      for (auto radio = radio_lo; radio < radio_hi; radio++) {
        if (radio == beacon_radio)
          tx_ret =
              radioconfig_->RadioTx(radio, beaconbuff.data(), 2, tx_time_bs);
        else
          tx_ret = radioconfig_->RadioTx(radio, zeros.data(), 2, tx_time_bs);
        if (tx_ret != (int)cfg_->SampsPerSymbol()) {
          std::cerr << "BAD Transmit(" << tx_ret << "/"
                    << cfg_->SampsPerSymbol() << ") at Time " << tx_time_bs
                    << std::endl;
        }
      }
      for (size_t sym = 0; sym < cfg_->Frame().NumTotalSyms(); sym++) {
        for (auto radio = radio_lo; radio < radio_hi; radio++) {
          radioconfig_->RadioRx(radio, samp_buffer.data(), rx_time_bs);
        }
      }
    }

    std::cout << "Start BS main recv loop..." << std::endl;
  }

  if (radio_lo > radio_hi) {
    MLPD_INFO("LoopTxRxArgos[%zu] has no radios, exiting\n", tid);
    return;
  }
  MLPD_INFO("LoopTxRxArgos[%zu] has %zu:%zu total radios %zu\n", tid, radio_lo,
            radio_hi, (radio_hi - radio_lo) + 1);

  ssize_t prev_frame_id = -1;
  size_t radio_id = radio_lo;
  size_t global_frame_id = 0;
  size_t global_symbol_id = 0;
  while (cfg_->Running() == true) {
    if (0 == DequeueSendArgos(tid, time0)) {
      // receive data
      auto pkts = RecvEnqueueArgos(tid, radio_id, rx_slot, global_frame_id,
                                   global_symbol_id);
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
      }  // if (pkt.size() > 0)
      if (radio_id == radio_hi) {
        if (cfg_->HwFramer() == false) {
          // Update global frame_id and symbol_id
          global_symbol_id++;
          if (global_symbol_id == cfg_->Frame().NumTotalSyms()) {
            global_symbol_id = 0;
            global_frame_id++;
            if (cfg_->Frame().NumDLSyms() == 0) {
              for (size_t radio = radio_lo; radio <= radio_hi; radio++)
                this->SendBeaconHW(global_frame_id, radio, time0);
            }
          }
        }
        radio_id = radio_lo;
      } else {
        radio_id++;
      }
    }  // DequeueSendArgos(tid) == 0
  }    // cfg_->Running() == true
}

std::vector<Packet*> PacketTXRX::RecvEnqueueArgos(size_t tid, size_t radio_id,
                                                  size_t rx_slot,
                                                  size_t global_frame_id,
                                                  size_t global_symbol_id) {
  moodycamel::ProducerToken* local_ptok = rx_ptoks_[tid];
  size_t packet_length = cfg_->PacketLength();

  size_t ant_id = radio_id * cfg_->NumChannels();
  size_t cell_id = cfg_->CellId().at(radio_id);
  std::vector<size_t> ant_ids(cfg_->NumChannels());
  std::vector<size_t> symbol_ids(cfg_->NumChannels(), global_symbol_id);
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
  bool cal_rx = (radio_id != cfg_->RefRadio(cell_id) &&
                 cfg_->IsCalUlPilot(global_frame_id, global_symbol_id)) ||
                (radio_id == cfg_->RefRadio(cell_id) &&
                 cfg_->IsCalDlPilot(global_frame_id, global_symbol_id));
  bool dummy_read =
      (cfg_->HwFramer() == false) &&
      (!cfg_->IsPilot(global_frame_id, global_symbol_id) &&
       !cfg_->IsUplink(global_frame_id, global_symbol_id) && !cal_rx);
  if (dummy_read) {
    // init samp_buffer for dummy read
    std::vector<std::complex<int16_t>> samp_buffer0(cfg_->SampsPerSymbol(), 0);
    std::vector<std::complex<int16_t>> samp_buffer1(cfg_->SampsPerSymbol(), 0);
    std::vector<void*> samp_buffer(2);
    samp_buffer[0] = samp_buffer0.data();
    if (cfg_->NumChannels() == 2) {
      samp_buffer[1] = samp_buffer1.data();
    }
    if (cfg_->Running() == true)
      radioconfig_->RadioRx(radio_id, samp_buffer.data(), frame_time);
    std::vector<Packet*> empty_pkt;
    return empty_pkt;
  } else {
    if ((cfg_->Running() == false) ||
        radioconfig_->RadioRx(radio_id, samp.data(), frame_time) <= 0) {
      std::vector<Packet*> empty_pkt;
      return empty_pkt;
    }
    size_t frame_id = global_frame_id;
    size_t symbol_id = global_symbol_id;
    if (cfg_->HwFramer() == true) {
      frame_id = (size_t)(frame_time >> 32);
      symbol_id = (size_t)((frame_time >> 16) & 0xFFFF);
      symbol_ids.clear();
      symbol_ids.resize(cfg_->NumChannels(), symbol_id);

      // additional read for DL Calibration in two-chan case
      /// \TODO: What if ref_ant is set to the second channel?
      if ((cfg_->Frame().IsRecCalEnabled() == true) &&
          (cfg_->IsCalDlPilot(frame_id, symbol_id) == true) &&
          (radio_id == cfg_->RefRadio(cell_id)) && (cfg_->AntPerGroup() > 1)) {
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
              radioconfig_->RadioRx(radio_id, tmp_samp.data(), frame_time) <=
                  0) {
            std::vector<Packet*> empty_pkt;
            return empty_pkt;
          }
          symbol_ids.at(s) = cfg_->Frame().GetDLCalSymbol(s);
          ant_ids.at(s) = ant_id;
        }
      }
    }
    std::vector<Packet*> pkt;
    for (size_t ch = 0; ch < symbol_ids.size(); ++ch) {
      RxPacket& rx = rx_packets_.at(tid).at(rx_slot + ch);
      pkt.push_back(rx.RawPacket());
      new (rx.RawPacket())
          Packet(frame_id, symbol_ids.at(ch), cell_id, ant_ids.at(ch));

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
}

void PacketTXRX::SendBeaconHW(size_t frame_id, size_t radio_id,
                              long long time0) {
  std::vector<void*> beaconbuff(2);
  std::vector<void*> zeros(2);
  zeros[0] = calloc(cfg_->SampsPerSymbol(), sizeof(int16_t) * 2);
  zeros[1] = calloc(cfg_->SampsPerSymbol(), sizeof(int16_t) * 2);
  size_t beacon_radio = cfg_->BeaconAnt() / cfg_->NumChannels();
  size_t beacon_ch = cfg_->BeaconAnt() % cfg_->NumChannels();
  if (radio_id == beacon_radio) {
    beaconbuff[beacon_ch] = cfg_->BeaconCi16().data();
    beaconbuff[1 - beacon_ch] = zeros[0];
  } else {
    beaconbuff[0] = zeros[0];
    beaconbuff[1] = zeros[1];
  }
  if (cfg_->NumChannels() > 1) beaconbuff[1] = zeros[0];
  // assuming beacon is first symbol
  long long frame_time = time0 + (frame_id + TX_FRAME_DELTA) *
                                     cfg_->SampsPerSymbol() *
                                     cfg_->Frame().NumTotalSyms();
  int tx_ret =
      radioconfig_->RadioTx(radio_id, beaconbuff.data(), 2, frame_time);
  if (tx_ret != (int)cfg_->SampsPerSymbol()) {
    std::cerr << "BAD BEACON Transmit(" << tx_ret << "/"
              << cfg_->SampsPerSymbol() << ") at Time " << tx_time_bs_
              << ", frame count " << frame_id << std::endl;
  }
}

size_t PacketTXRX::DequeueSendArgos(int tid, long long time0) {
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
    size_t cell_id = cfg_->CellId().at(radio_id);

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
      // Schedule beacon in the future
      if (cfg_->HwFramer() == false && symbol_id == 0) {
        this->SendBeaconHW(frame_id, radio_id, time0);
      }

      // Transmit downlink calibration (array to ref) pilot
      std::vector<void*> caltxbuf(cfg_->NumChannels());
      if ((cfg_->Frame().IsRecCalEnabled() == true) &&
          (symbol_id == cfg_->Frame().GetDLSymbol(0))) {
        std::vector<std::complex<int16_t>> zeros(cfg_->SampsPerSymbol(),
                                                 std::complex<int16_t>(0, 0));
        for (size_t s = 0; s < cfg_->RadioPerGroup(); s++) {
          if (radio_id != cfg_->RefRadio(cell_id)) {
            bool calib_turn = (frame_id % cfg_->RadioGroupNum() ==
                                   radio_id / cfg_->RadioPerGroup() &&
                               s == radio_id % cfg_->RadioPerGroup());
            for (size_t ch = 0; ch < cfg_->NumChannels(); ch++) {
              caltxbuf.at(ch) =
                  calib_turn ? cfg_->PilotCi16().data() : zeros.data();
              if (cfg_->NumChannels() > 1) caltxbuf.at(1 - ch) = zeros.data();
              long long frame_time = 0;
              if (cfg_->HwFramer() == false)
                frame_time = time0 + cfg_->SampsPerSymbol() *
                                         ((frame_id + TX_FRAME_DELTA) *
                                              cfg_->Frame().NumTotalSyms() +
                                          cfg_->Frame().GetDLCalSymbol(
                                              s * cfg_->NumChannels() + ch));
              else
                frame_time =
                    ((long long)(frame_id + TX_FRAME_DELTA) << 32) |
                    (cfg_->Frame().GetDLCalSymbol(s * cfg_->NumChannels() + ch)
                     << 16);
              radioconfig_->RadioTx(radio_id, caltxbuf.data(), 1, frame_time);
            }
          } else {
            caltxbuf.at(0) = cfg_->PilotCi16().data();
            if (cfg_->NumChannels() > 1) caltxbuf.at(1) = zeros.data();
            long long frame_time = 0;
            if (cfg_->HwFramer() == false)
              frame_time = time0 + cfg_->SampsPerSymbol() *
                                       ((frame_id + TX_FRAME_DELTA) *
                                            cfg_->Frame().NumTotalSyms() +
                                        cfg_->Frame().GetULCalSymbol(0));
            else
              frame_time = ((long long)(frame_id + TX_FRAME_DELTA) << 32) |
                           (cfg_->Frame().GetULCalSymbol(0) << 16);
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
      long long frame_time = 0;
      if (cfg_->HwFramer() == false)
        frame_time =
            time0 + cfg_->SampsPerSymbol() * ((frame_id + TX_FRAME_DELTA) *
                                                  cfg_->Frame().NumTotalSyms() +
                                              symbol_id);
      else
        frame_time =
            ((long long)(frame_id + TX_FRAME_DELTA) << 32) | (symbol_id << 16);
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
