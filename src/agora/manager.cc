/**
 * @file manager.cc
 * @brief Implementation file for the main manager class
 */

#include "manager.h"

// file specific debugger variables
static const bool kDebugPrintPacketsFromMac = false;
static const bool kDebugDeferral = true;

Manager::Manager(Config* cfg, Stats* stats, PhyStats* phy_stats,
                 MessageInfo* queues, Buffer* buffers, FrameInfo* frame_info)
    : base_worker_core_offset_(cfg->CoreOffset() + 1 + cfg->SocketThreadNum()),
      config_(cfg),
      stats_(stats),
      phy_stats_(phy_stats),
      queues_(queues),
      buffers_(buffers),
      frame_info_(frame_info) {
  PinToCoreWithOffset(ThreadType::kMaster, config_->CoreOffset(), 0,
                      kEnableCoreReuse, false /* quiet */);
  CheckIncrementScheduleFrame(0, ScheduleProcessingFlags::kProcessingComplete);
}

Manager::~Manager() {}

// Increments the cur_sche_frame_id when all ScheduleProcessingFlags have
// been acheived.
void Manager::CheckIncrementScheduleFrame(size_t frame_id,
                                          ScheduleProcessingFlags completed) {
  schedule_process_flags_ += completed;
  assert(frame_info_->cur_sche_frame_id == frame_id);
  unused(frame_id);

  if (schedule_process_flags_ ==
      static_cast<uint8_t>(ScheduleProcessingFlags::kProcessingComplete)) {
    frame_info_->cur_sche_frame_id++;
    schedule_process_flags_ = ScheduleProcessingFlags::kNone;
    if (config_->Frame().NumULSyms() == 0) {
      schedule_process_flags_ += ScheduleProcessingFlags::kUplinkComplete;
    }
    if (config_->Frame().NumDLSyms() == 0) {
      schedule_process_flags_ += ScheduleProcessingFlags::kDownlinkComplete;
    }
  }
}

size_t Manager::FetchEvent(EventData events_list[],
                           bool is_turn_to_dequeue_from_io) {
  const size_t max_events_needed = std::max(
      kDequeueBulkSizeTXRX * (config_->SocketThreadNum() + 1 /* MAC */),
      kDequeueBulkSizeWorker * config_->WorkerThreadNum());
  size_t num_events = 0;
  if (is_turn_to_dequeue_from_io) {
    for (size_t i = 0; i < config_->SocketThreadNum(); i++) {
      num_events += queues_->message_queue.try_dequeue_bulk_from_producer(
          *(queues_->rx_ptoks_ptr[i]), events_list + num_events,
          kDequeueBulkSizeTXRX);
    }

    if (kEnableMac == true) {
      num_events += queues_->mac_response_queue.try_dequeue_bulk(
          events_list + num_events, kDequeueBulkSizeTXRX);
    }
  } else {
    num_events +=
        queues_->complete_task_queue[(frame_info_->cur_proc_frame_id & 0x1)]
            .try_dequeue_bulk(events_list + num_events, max_events_needed);
  }
  return num_events;
}

void Manager::HandleEventPacketRx(size_t tag) {
  Packet* pkt = rx_tag_t(tag).rx_packet_->RawPacket();

  if (pkt->frame_id_ >= ((frame_info_->cur_sche_frame_id + kFrameWnd))) {
    AGORA_LOG_ERROR(
        "Error: Received packet for future frame %u beyond "
        "frame window (= %zu + %zu). This can happen if "
        "Agora is running slowly, e.g., in debug mode\n",
        pkt->frame_id_, frame_info_->cur_sche_frame_id, kFrameWnd);
    config_->Running(false);
    return;
  }

  // UpdateRxCounters(pkt->frame_id_, pkt->symbol_id_, config, stats_,
  //                  counters_->rx_counters);
  const size_t frame_slot = pkt->frame_id_ % kFrameWnd;
  if (config_->IsPilot(pkt->frame_id_, pkt->symbol_id_)) {
    counters_->rx_counters.num_pilot_pkts_[frame_slot]++;
    if (counters_->rx_counters.num_pilot_pkts_.at(frame_slot) ==
        counters_->rx_counters.num_pilot_pkts_per_frame_) {
      counters_->rx_counters.num_pilot_pkts_.at(frame_slot) = 0;
      stats_->MasterSetTsc(TsType::kPilotAllRX, pkt->frame_id_);
      stats_->PrintPerFrameDone(PrintType::kPacketRXPilots, pkt->frame_id_);
    }
  } else if (config_->IsCalDlPilot(pkt->frame_id_, pkt->symbol_id_) ||
             config_->IsCalUlPilot(pkt->frame_id_, pkt->symbol_id_)) {
    counters_->rx_counters.num_reciprocity_pkts_.at(frame_slot)++;
    if (counters_->rx_counters.num_reciprocity_pkts_.at(frame_slot) ==
        counters_->rx_counters.num_reciprocity_pkts_per_frame_) {
      counters_->rx_counters.num_reciprocity_pkts_.at(frame_slot) = 0;
      stats_->MasterSetTsc(TsType::kRCAllRX, pkt->frame_id_);
    }
  }
  // Receive first packet in a frame
  if (counters_->rx_counters.num_pkts_.at(frame_slot) == 0) {
    if (kEnableMac == false) {
      // schedule this frame's encoding
      // Defer the schedule.  If frames are already deferred or the current
      // received frame is too far off
      if ((encode_deferral_.empty() == false) ||
          (pkt->frame_id_ >=
           (frame_info_->cur_proc_frame_id + kScheduleQueues))) {
        if (kDebugDeferral) {
          AGORA_LOG_INFO("   +++ Deferring encoding of frame %zu\n",
                         pkt->frame_id_);
        }
        encode_deferral_.push(pkt->frame_id_);
      } else {
        ScheduleDownlinkProcessing(pkt->frame_id_);
      }
    }
    stats_->MasterSetTsc(TsType::kFirstSymbolRX, pkt->frame_id_);
    if (kDebugPrintPerFrameStart) {
      const size_t prev_frame_slot = (frame_slot + kFrameWnd - 1) % kFrameWnd;
      AGORA_LOG_INFO(
          "Main [frame %zu + %.2f ms since last frame]: Received "
          "first packet. Remaining packets in prev frame: %zu\n",
          pkt->frame_id_,
          stats_->MasterGetDeltaMs(TsType::kFirstSymbolRX, pkt->frame_id_,
                                   pkt->frame_id_ - 1),
          counters_->rx_counters.num_pkts_[prev_frame_slot]);
    }
  }

  counters_->rx_counters.num_pkts_.at(frame_slot)++;
  if (counters_->rx_counters.num_pkts_.at(frame_slot) ==
      counters_->rx_counters.num_rx_pkts_per_frame_) {
    stats_->MasterSetTsc(TsType::kRXDone, pkt->frame_id_);
    stats_->PrintPerFrameDone(PrintType::kPacketRX, pkt->frame_id_);
    counters_->rx_counters.num_pkts_.at(frame_slot) = 0;
  }  // end of updateRxCounter
  fft_queue_arr_.at(pkt->frame_id_ % kFrameWnd).push(fft_req_tag_t(tag));
}

/*
void Manager::HandleEventFft(size_t tag) {
  const size_t frame_id = gen_tag_t(tag).frame_id_;
  const size_t symbol_id = gen_tag_t(tag).symbol_id_;
  const SymbolType sym_type = config->GetSymbolType(symbol_id);

  if (sym_type == SymbolType::kPilot) {
    const bool last_fft_task =
        counters_->pilot_fft_counters.CompleteTask(frame_id, symbol_id);
    if (last_fft_task == true) {
      stats_->PrintPerSymbolDone(PrintType::kFFTPilots, frame_id, symbol_id,
                                 counters_->pilot_fft_counters);

      if ((config->Frame().IsRecCalEnabled() == false) ||
          ((config->Frame().IsRecCalEnabled() == true) &&
           (rc_last_frame_ == frame_id))) {
        // If CSI of all UEs is ready, schedule ZF/prediction
        const bool last_pilot_fft =
            counters_->pilot_fft_counters.CompleteSymbol(frame_id);
        if (last_pilot_fft == true) {
          stats_->MasterSetTsc(TsType::kFFTPilotsDone, frame_id);
          stats_->PrintPerFrameDone(PrintType::kFFTPilots, frame_id);
          counters_->pilot_fft_counters.Reset(frame_id);
          if (kPrintPhyStats == true) {
            phy_stats_->PrintSnrStats(frame_id);
          }
          if (kEnableMac == true) {
            SendSnrReport(EventType::kSNRReport, frame_id, symbol_id);
          }
          ScheduleSubcarriers(EventType::kZF, frame_id, 0);
        }
      }
    }
  } else if (sym_type == SymbolType::kUL) {
    const size_t symbol_idx_ul = config->Frame().GetULSymbolIdx(symbol_id);

    const bool last_fft_per_symbol =
        uplink_fft_counters_.CompleteTask(frame_id, symbol_id);

    if (last_fft_per_symbol == true) {
      fft_cur_frame_for_symbol_.at(symbol_idx_ul) = frame_id;

      stats_->PrintPerSymbolDone(PrintType::kFFTData, frame_id, symbol_id,
                                 uplink_fft_counters_);
      // If precoder exist, schedule demodulation
      if (zf_last_frame_ == frame_id) {
        ScheduleSubcarriers(EventType::kDemul, frame_id, symbol_id);
      }
      const bool last_uplink_fft =
          uplink_fft_counters_.CompleteSymbol(frame_id);
      if (last_uplink_fft == true) {
        uplink_fft_counters_.Reset(frame_id);
      }
    }
  } else if ((sym_type == SymbolType::kCalDL) ||
             (sym_type == SymbolType::kCalUL)) {
    stats_->PrintPerSymbolDone(PrintType::kFFTCal, frame_id, symbol_id,
                               rc_counters_);

    const bool last_rc_task = rc_counters_.CompleteTask(frame_id);
    if (last_rc_task == true) {
      stats_->PrintPerFrameDone(PrintType::kFFTCal, frame_id);
      rc_counters_.Reset(frame_id);
      stats_->MasterSetTsc(TsType::kRCDone, frame_id);
      rc_last_frame_ = frame_id;

      // See if the calibration has completed
      if (kPrintPhyStats) {
        const size_t frames_for_cal = config->RecipCalFrameCnt();

        if ((frame_id % frames_for_cal) == 0 && (frame_id > 0)) {
          const size_t previous_cal_slot =
              config->ModifyRecCalIndex(config->RecipCalIndex(frame_id), -1);
          //Print the previous index
          phy_stats_->PrintCalibSnrStats(previous_cal_slot);
        }
      }  // kPrintPhyStats
    }    // last_rc_task
  }      // kCaLDL || kCalUl
}

void Manager::HandleEventZf(size_t tag) {
  const size_t frame_id = gen_tag_t(tag).frame_id_;
  stats_->PrintPerTaskDone(PrintType::kZF, frame_id, 0,
                           zf_counters_.GetTaskCount(frame_id), zf_counters_);
  const bool last_zf_task = zf_counters_.CompleteTask(frame_id);
  if (last_zf_task == true) {
    stats_->MasterSetTsc(TsType::kZFDone, frame_id);
    zf_last_frame_ = frame_id;
    stats_->PrintPerFrameDone(PrintType::kZF, frame_id);
    zf_counters_.Reset(frame_id);
    if (kPrintZfStats) {
      phy_stats_->PrintZfStats(frame_id);
    }

    for (size_t i = 0; i < config_->Frame().NumULSyms(); i++) {
      if (fft_cur_frame_for_symbol_.at(i) == frame_id) {
        ScheduleSubcarriers(EventType::kDemul, frame_id,
                            config_->Frame().GetULSymbol(i));
      }
    }
    // Schedule precoding for downlink symbols
    for (size_t i = 0; i < config_->Frame().NumDLSyms(); i++) {
      const size_t last_encoded_frame = encode_cur_frame_for_symbol_.at(i);
      if ((last_encoded_frame != SIZE_MAX) &&
          (last_encoded_frame >= frame_id)) {
        ScheduleSubcarriers(EventType::kPrecode, frame_id,
                            config_->Frame().GetDLSymbol(i));
      }
    }
  }  // end if (zf_counters_.last_task(frame_id) == true)
}

void Manager::HandleEventDemul(size_t tag) {
  const size_t frame_id = gen_tag_t(tag).frame_id_;
  const size_t symbol_id = gen_tag_t(tag).symbol_id_;
  const size_t base_sc_id = gen_tag_t(tag).sc_id_;

  stats_->PrintPerTaskDone(PrintType::kDemul, frame_id, symbol_id, base_sc_id,
                           demul_counters_);
  const bool last_demul_task =
      demul_counters_.CompleteTask(frame_id, symbol_id);

  if (last_demul_task == true) {
    if (kUplinkHardDemod == false) {
      ScheduleCodeblocks(EventType::kDecode, Direction::kUplink, frame_id,
                         symbol_id);
    }
    stats_->PrintPerSymbolDone(PrintType::kDemul, frame_id, symbol_id,
                               demul_counters_);
    const bool last_demul_symbol = demul_counters_.CompleteSymbol(frame_id);
    if (last_demul_symbol == true) {
      max_equaled_frame_ = frame_id;
      stats_->MasterSetTsc(TsType::kDemulDone, frame_id);
      stats_->PrintPerFrameDone(PrintType::kDemul, frame_id);
      // skip Decode when hard demod is enabled
      if (kUplinkHardDemod == true) {
        assert(cur_proc_frame_id == frame_id);
        CheckIncrementScheduleFrame(frame_id, kUplinkComplete);
        const bool work_finished = CheckFrameComplete(frame_id);
        if (work_finished == true) {
          goto finish;
        }
      } else {
        demul_counters_.Reset(frame_id);
        if (config_->BigstationMode() == false) {
          assert(cur_sche_frame_id_ == frame_id);
          CheckIncrementScheduleFrame(frame_id, kUplinkComplete);
        } else {
          ScheduleCodeblocks(EventType::kDecode, Direction::kUplink, frame_id,
                             symbol_id);
        }
      }
    }
  }
}

void Manager::HandleEventDecode(size_t tag) {
  const size_t frame_id = gen_tag_t(tag).frame_id_;
  const size_t symbol_id = gen_tag_t(tag).symbol_id_;

  const bool last_decode_task =
      decode_counters_.CompleteTask(frame_id, symbol_id);
  if (last_decode_task == true) {
    if (kEnableMac == true) {
      ScheduleUsers(EventType::kPacketToMac, frame_id, symbol_id);
    }
    stats_->PrintPerSymbolDone(PrintType::kDecode, frame_id, symbol_id,
                               decode_counters_);
    const bool last_decode_symbol = decode_counters_.CompleteSymbol(frame_id);
    if (last_decode_symbol == true) {
      stats_->MasterSetTsc(TsType::kDecodeDone, frame_id);
      stats_->PrintPerFrameDone(PrintType::kDecode, frame_id);
      if (kEnableMac == false) {
        assert(cur_proc_frame_id == frame_id);
        const bool work_finished = CheckFrameComplete(frame_id);
        if (work_finished == true) {
          goto finish;
        }
      }
    }
  }
}

void Manager::HandleEventRanUpdate(size_t tag1, size_t tag2, size_t tag3) {
  RanConfig rc;
  rc.n_antennas_ = tag1;
  rc.mod_order_bits_ = tag2;
  rc.frame_id_ = tag3;
  // UpdateRanConfig(rc);
  nlohmann::json msc_params = config_->MCSParams(Direction::kUplink);
  msc_params["modulation"] = MapModToStr(rc.mod_order_bits_);
  config_->UpdateUlMCS(msc_params);
}

void Manager::HandleEventPacketToMac(size_t tag) {
  const size_t frame_id = gen_tag_t(tag).frame_id_;
  const size_t symbol_id = gen_tag_t(tag).symbol_id_;

  const bool last_tomac_task =
      tomac_counters_.CompleteTask(frame_id, symbol_id);
  if (last_tomac_task == true) {
    stats_->PrintPerSymbolDone(PrintType::kPacketToMac, frame_id, symbol_id,
                               tomac_counters_);

    const bool last_tomac_symbol = tomac_counters_.CompleteSymbol(frame_id);
    if (last_tomac_symbol == true) {
      assert(cur_proc_frame_id == frame_id);
      // stats_->MasterSetTsc(TsType::kMacTXDone, frame_id);
      stats_->PrintPerFrameDone(PrintType::kPacketToMac, frame_id);
      const bool work_finished = CheckFrameComplete(frame_id);
      if (work_finished == true) {
        goto finish;
      }
    }
  }
}

void Manager::HandleEventPacketFromMac(size_t tag) {
  // This is an entire frame (multiple mac packets)
  const size_t ue_id = rx_mac_tag_t(tag).tid_;
  const size_t radio_buf_id = rx_mac_tag_t(tag).offset_;
  const auto* pkt = reinterpret_cast<const MacPacketPacked*>(
      &dl_bits_buffer_[ue_id][radio_buf_id * config_->MacBytesNumPerframe(
                                                 Direction::kDownlink)]);

  AGORA_LOG_INFO("Agora: frame %d @ offset %zu %zu @ location %zu\n",
                 pkt->Frame(), ue_id, radio_buf_id,
                 reinterpret_cast<intptr_t>(pkt));

  if (kDebugPrintPacketsFromMac) {
    std::stringstream ss;

    for (size_t dl_data_symbol = 0;
         dl_data_symbol < config_->Frame().NumDlDataSyms(); dl_data_symbol++) {
      ss << "Agora: kPacketFromMac, frame " << pkt->Frame() << ", symbol "
         << std::to_string(pkt->Symbol()) << " crc "
         << std::to_string(pkt->Crc()) << " bytes: ";
      for (size_t i = 0; i < pkt->PayloadLength(); i++) {
        ss << std::to_string((pkt->Data()[i])) << ", ";
      }
      ss << std::endl;
      pkt = reinterpret_cast<const MacPacketPacked*>(
          reinterpret_cast<const uint8_t*>(pkt) +
          config_->MacPacketLength(Direction::kDownlink));
    }
    AGORA_LOG_INFO("%s\n", ss.str().c_str());
  }

  const size_t frame_id = pkt->Frame();
  const bool last_ue = mac_to_phy_counters_.CompleteTask(frame_id, 0);
  if (last_ue == true) {
    // schedule this frame's encoding
    // Defer the schedule.  If frames are already deferred or the
    // current received frame is too far off
    if ((encode_deferral_.empty() == false) ||
        (frame_id >= (cur_proc_frame_id + kScheduleQueues))) {
      if (kDebugDeferral) {
        AGORA_LOG_INFO("   +++ Deferring encoding of frame %zu\n", frame_id);
      }
      encode_deferral_.push(frame_id);
    } else {
      ScheduleDownlinkProcessing(frame_id);
    }
    mac_to_phy_counters_.Reset(frame_id);
    stats_->PrintPerFrameDone(PrintType::kPacketFromMac, frame_id);
  }
}

void Manager::HandleEventEncode(size_t tag) {
  const size_t frame_id = gen_tag_t(tag).frame_id_;
  const size_t symbol_id = gen_tag_t(tag).symbol_id_;

  const bool last_encode_task =
      encode_counters_.CompleteTask(frame_id, symbol_id);
  if (last_encode_task == true) {
    encode_cur_frame_for_symbol_.at(
        config_->Frame().GetDLSymbolIdx(symbol_id)) = frame_id;
    // If precoder of the current frame exists
    if (zf_last_frame_ == frame_id) {
      ScheduleSubcarriers(EventType::kPrecode, frame_id, symbol_id);
    }
    stats_->PrintPerSymbolDone(PrintType::kEncode, frame_id, symbol_id,
                               encode_counters_);

    const bool last_encode_symbol = encode_counters_.CompleteSymbol(frame_id);
    if (last_encode_symbol == true) {
      encode_counters_.Reset(frame_id);
      stats_->MasterSetTsc(TsType::kEncodeDone, frame_id);
      stats_->PrintPerFrameDone(PrintType::kEncode, frame_id);
    }
  }
}

void Manager::HandleEventPrecode(size_t tag) {
  // Precoding is done, schedule ifft
  const size_t sc_id = gen_tag_t(tag).sc_id_;
  const size_t frame_id = gen_tag_t(tag).frame_id_;
  const size_t symbol_id = gen_tag_t(tag).symbol_id_;
  stats_->PrintPerTaskDone(PrintType::kPrecode, frame_id, symbol_id, sc_id,
                           precode_counters_);
  const bool last_precode_task =
      precode_counters_.CompleteTask(frame_id, symbol_id);

  if (last_precode_task == true) {
    // precode_cur_frame_for_symbol_.at(
    //    config_->Frame().GetDLSymbolIdx(symbol_id)) = frame_id;
    ScheduleAntennas(EventType::kIFFT, frame_id, symbol_id);
    stats_->PrintPerSymbolDone(PrintType::kPrecode, frame_id, symbol_id,
                               precode_counters_);

    const bool last_precode_symbol = precode_counters_.CompleteSymbol(frame_id);
    if (last_precode_symbol == true) {
      precode_counters_.Reset(frame_id);
      stats_->MasterSetTsc(TsType::kPrecodeDone, frame_id);
      stats_->PrintPerFrameDone(PrintType::kPrecode, frame_id);
    }
  }
}

void Manager::HandleEventIfft(size_t tag) {
  // IFFT is done, schedule data transmission
const size_t ant_id = gen_tag_t(tag).ant_id_;
const size_t frame_id = gen_tag_t(tag).frame_id_;
const size_t symbol_id = gen_tag_t(tag).symbol_id_;
const size_t symbol_idx_dl = config_->Frame().GetDLSymbolIdx(symbol_id);
stats_->PrintPerTaskDone(PrintType::kIFFT, frame_id, symbol_id, ant_id,
                         ifft_counters_);

const bool last_ifft_task = ifft_counters_.CompleteTask(frame_id, symbol_id);
if (last_ifft_task == true) {
  ifft_cur_frame_for_symbol_.at(symbol_idx_dl) = frame_id;
  if (symbol_idx_dl == ifft_next_symbol_) {
    // Check the available symbols starting from the current symbol
    // Only schedule symbols that are continuously available
    for (size_t sym_id = symbol_idx_dl;
         sym_id <= ifft_counters_.GetSymbolCount(frame_id); sym_id++) {
      const size_t symbol_ifft_frame = ifft_cur_frame_for_symbol_.at(sym_id);
      if (symbol_ifft_frame == frame_id) {
        ScheduleAntennasTX(frame_id, config_->Frame().GetDLSymbol(sym_id));
        ifft_next_symbol_++;
      } else {
        break;
      }
    }
  }
  stats_->PrintPerSymbolDone(PrintType::kIFFT, frame_id, symbol_id,
                             ifft_counters_);

  const bool last_ifft_symbol = ifft_counters_.CompleteSymbol(frame_id);
  if (last_ifft_symbol == true) {
    ifft_next_symbol_ = 0;
    stats_->MasterSetTsc(TsType::kIFFTDone, frame_id);
    stats_->PrintPerFrameDone(PrintType::kIFFT, frame_id);
    assert(frame_id == cur_proc_frame_id);
    CheckIncrementScheduleFrame(frame_id, kDownlinkComplete);
    const bool work_finished = CheckFrameComplete(frame_id);
    if (work_finished == true) {
      goto finish;
    }
  }
}
}

    void Manager::HandleEventPacketTx(size_t tag) {
  // Data is sent
  const size_t ant_id = gen_tag_t(tag).ant_id_;
  const size_t frame_id = gen_tag_t(tag).frame_id_;
  const size_t symbol_id = gen_tag_t(tag).symbol_id_;
  stats_->PrintPerTaskDone(PrintType::kPacketTX, frame_id, symbol_id, ant_id,
                           tx_counters_);

  const bool last_tx_task = tx_counters_.CompleteTask(frame_id, symbol_id);
  if (last_tx_task == true) {
    stats_->PrintPerSymbolDone(PrintType::kPacketTX, frame_id, symbol_id,
                               tx_counters_);
    // If tx of the first symbol is done
    if (symbol_id == config_->Frame().GetDLSymbol(0)) {
      stats_->MasterSetTsc(TsType::kTXProcessedFirst, frame_id);
      stats_->PrintPerFrameDone(PrintType::kPacketTXFirst, frame_id);
    }

    const bool last_tx_symbol = tx_counters_.CompleteSymbol(frame_id);
    if (last_tx_symbol == true) {
      stats_->MasterSetTsc(TsType::kTXDone, frame_id);
      stats_->PrintPerFrameDone(PrintType::kPacketTX, frame_id);

      const bool work_finished = CheckFrameComplete(frame_id);
      if (work_finished == true) {
        goto finish;
      }
    }

    tx_count++;
    if (tx_count == tx_counters_.MaxSymbolCount() * 9000) {
      tx_count = 0;

      const double diff = GetTime::GetTimeUs() - tx_begin;
      const int samples_num_per_ue =
          config_->OfdmDataNum() * tx_counters_.MaxSymbolCount() * 1000;

      AGORA_LOG_INFO(
          "TX %d samples (per-client) to %zu clients in %f secs, "
          "throughtput %f bps per-client (16QAM), current tx queue "
          "length %zu\n",
          samples_num_per_ue, config_->UeAntNum(), diff,
          samples_num_per_ue * std::log2(16.0f) / diff,
          GetConq(sched_info_arr, EventType::kPacketTX, 0)->size_approx());
      unused(diff);
      unused(samples_num_per_ue);
      tx_begin = GetTime::GetTimeUs();
    }
  }
}
*/

void Manager::Start() {
  const bool start_status = packet_tx_rx_->StartTxRx(buffer_->calib_dl_buffer,
                                                     buffer_->calib_ul_buffer);
  // Start packet I/O
  if (start_status == false) {
    // this->Stop();  // TODO: stop everything?
    StopAgora();
    return;
  }

  // Counters for printing summary
  size_t tx_count = 0;
  double tx_begin = GetTime::GetTimeUs();

  bool is_turn_to_dequeue_from_io = true;
  const size_t max_events_needed = std::max(
      kDequeueBulkSizeTXRX * (config_->SocketThreadNum() + 1 /* MAC */),
      kDequeueBulkSizeWorker * config_->WorkerThreadNum());
  EventData events_list[max_events_needed];

  while ((config_->Running() == true) &&
         (SignalHandler::GotExitSignal() == false)) {
    // Get a batch of events
    size_t num_events = FetchEvent(events_list, is_turn_to_dequeue_from_io);
    is_turn_to_dequeue_from_io = !is_turn_to_dequeue_from_io;

    // Handle each event
    for (size_t ev_i = 0; ev_i < num_events; ev_i++) {
      EventData& event = events_list[ev_i];

      // FFT processing is scheduled after falling through the switch
      switch (event.event_type_) {
        case EventType::kPacketRX: {
          HandleEventPacketRx(event.tags_[0]);
        } break;

        // case EventType::kFFT: {
        //   for (size_t i = 0; i < event.num_tags_; i++) {
        //     HandleEventFft(event.tags_[i]);
        //   }
        // } break;

        // case EventType::kZF: {
        //   for (size_t i = 0; (i < event.num_tags_); i++) {
        //     HandleEventZf(event.tags_[i]);
        //   }
        // } break;

        // case EventType::kDemul: {
        //   HandleEventDemul(event.tags_[0]);
        // } break;

        // case EventType::kDecode: {
        //   HandleEventDecode(event.tags_[0]);
        // } break;

        // case EventType::kRANUpdate: {
        //   HandleEventRanUpdate(event.tags_[0], event.tags_[1], event.tags_[2]);
        // } break;

        // case EventType::kPacketToMac: {
        //   HandleEventPacketToMac(event.tags_[0]);
        // } break;

        // case EventType::kPacketFromMac: {
        //   HandleEventFromMac(event.tags_[0u]);
        // } break;

        // case EventType::kEncode: {
        //   for (size_t i = 0u; i < event.num_tags_; i++) {
        //     HandleEventEncode(event.tags_[i]);
        //   }
        // } break;

        // case EventType::kPrecode: {
        //   HandleEventPrecode(event.tags_[0]);
        // } break;

        // case EventType::kIFFT: {
        //   for (size_t i = 0; i < event.num_tags_; i++) {
        //     HandleEventIfft(event.tags_[i]);
        //   }
        // } break;

        // case EventType::kPacketTX: {
        //   HandleEventPacketTx(event.tags_[0]);
        // } break;
        default:
          AGORA_LOG_ERROR("Wrong event type in message queue!");
          std::exit(0);
      } /* End of switch */

      // We schedule FFT processing if the event handling above results in
      // either (a) sufficient packets received for the current frame,
      // or (b) the current frame being updated.
      std::queue<fft_req_tag_t>& cur_fftq =
          fft_queue_arr_.at(cur_sche_frame_id_ % kFrameWnd);
      const size_t qid = cur_sche_frame_id_ & 0x1;
      if (cur_fftq.size() >= config_->FftBlockSize()) {
        const size_t num_fft_blocks = cur_fftq.size() / config_->FftBlockSize();
        for (size_t i = 0; i < num_fft_blocks; i++) {
          EventData do_fft_task;
          do_fft_task.num_tags_ = config_->FftBlockSize();
          do_fft_task.event_type_ = EventType::kFFT;

          for (size_t j = 0; j < config_->FftBlockSize(); j++) {
            RtAssert(!cur_fftq.empty(),
                     "Using front element cur_fftq when it is empty");
            do_fft_task.tags_[j] = cur_fftq.front().tag_;
            cur_fftq.pop();

            if (fft_created_count_ == 0) {
              stats_->MasterSetTsc(TsType::kProcessingStarted,
                                   cur_sche_frame_id_);
            }
            fft_created_count_++;
            if (fft_created_count_ ==
                counters_->rx_counters.num_rx_pkts_per_frame_) {
              fft_created_count_ = 0;
              //   if (config_->BigstationMode() == true) {
              //     CheckIncrementScheduleFrame(cur_sche_frame_id_,
              //                                 kUplinkComplete);
              //   }
            }
          }
          TryEnqueueFallback(GetConq(sched_info_arr, EventType::kFFT, qid),
                             GetPtok(sched_info_arr, EventType::kFFT, qid),
                             do_fft_task);
        }
      }
    } /* End of for */
  }   /* End of while */

finish:
  AGORA_LOG_INFO("Agora: printing stats and saving to file\n");
  stats_->PrintSummary();
  stats_->SaveToFile();
  if (flags_.enable_save_decode_data_to_file_ == true) {
    SaveDecodeDataToFile(stats_->LastFrameId(), config, decoded_buffer_);
  }
  if (flags_.enable_save_tx_data_to_file_ == true) {
    SaveTxDataToFile(stats_->LastFrameId(), config, dl_socket_buffer_);
  }

  // Calculate and print per-user BER
  if ((kEnableMac == false) && (kPrintPhyStats == true)) {
    phy_stats_->PrintPhyStats();
  }
  this->Stop();
}

// Scheduler
/**
   * @brief Schedule LDPC decoding or encoding over code blocks
   * @param task_type Either LDPC decoding or LDPC encoding
   * @param frame_id The monotonically increasing frame ID
   * @param symbol_idx The index of the symbol among uplink symbols for LDPC
   * decoding, and among downlink symbols for LDPC encoding
   */
void ScheduleSubcarriers(EventType event_type, size_t frame_id,
                         size_t symbol_id) {
  auto base_tag = gen_tag_t::FrmSymSc(frame_id, symbol_id, 0);
  size_t num_events = SIZE_MAX;
  size_t block_size = SIZE_MAX;

  switch (event_type) {
    case EventType::kDemul:
    case EventType::kPrecode:
      num_events = config->DemulEventsPerSymbol();
      block_size = config->DemulBlockSize();
      break;
    case EventType::kZF:
      num_events = config->ZfEventsPerSymbol();
      block_size = config->ZfBlockSize();
      break;
    default:
      assert(false);
  }

  size_t qid = (frame_id & 0x1);
  if (event_type == EventType::kZF) {
    EventData event;
    event.event_type_ = event_type;
    event.num_tags_ = config->ZfBatchSize();
    size_t num_blocks = num_events / event.num_tags_;
    size_t num_remainder = num_events % event.num_tags_;
    if (num_remainder > 0) {
      num_blocks++;
    }
    for (size_t i = 0; i < num_blocks; i++) {
      if ((i == num_blocks - 1) && num_remainder > 0) {
        event.num_tags_ = num_remainder;
      }
      for (size_t j = 0; j < event.num_tags_; j++) {
        event.tags_[j] =
            gen_tag_t::FrmSymSc(frame_id, symbol_id,
                                block_size * (i * event.num_tags_ + j))
                .tag_;
      }
      TryEnqueueFallback(GetConq(sched_info_arr, event_type, qid),
                         GetPtok(sched_info_arr, event_type, qid), event);
    }
  } else {
    for (size_t i = 0; i < num_events; i++) {
      TryEnqueueFallback(GetConq(sched_info_arr, event_type, qid),
                         GetPtok(sched_info_arr, event_type, qid),
                         EventData(event_type, base_tag.tag_));
      base_tag.sc_id_ += block_size;
    }
  }
}

void ScheduleCodeblocks(EventType event_type, Direction dir, size_t frame_id,
                        size_t symbol_idx) {
  auto base_tag = gen_tag_t::FrmSymCb(frame_id, symbol_idx, 0);
  const size_t num_tasks =
      config->UeAntNum() * config->LdpcConfig(dir).NumBlocksInSymbol();
  size_t num_blocks = num_tasks / config->EncodeBlockSize();
  const size_t num_remainder = num_tasks % config->EncodeBlockSize();
  if (num_remainder > 0) {
    num_blocks++;
  }
  EventData event;
  event.num_tags_ = config->EncodeBlockSize();
  event.event_type_ = event_type;
  size_t qid = frame_id & 0x1;
  for (size_t i = 0; i < num_blocks; i++) {
    if ((i == num_blocks - 1) && num_remainder > 0) {
      event.num_tags_ = num_remainder;
    }
    for (size_t j = 0; j < event.num_tags_; j++) {
      event.tags_[j] = base_tag.tag_;
      base_tag.cb_id_++;
    }
    TryEnqueueFallback(GetConq(sched_info_arr, event_type, qid),
                       GetPtok(sched_info_arr, event_type, qid), event);
  }
}

void SendSnrReport(EventType event_type, size_t frame_id, size_t symbol_id) {
  assert(event_type == EventType::kSNRReport);
  unused(event_type);
  auto base_tag = gen_tag_t::FrmSymUe(frame_id, symbol_id, 0);
  for (size_t i = 0; i < config->UeAntNum(); i++) {
    EventData snr_report(EventType::kSNRReport, base_tag.tag_);
    snr_report.num_tags_ = 2;
    float snr = phy_stats_->GetEvmSnr(frame_id, i);
    std::memcpy(&snr_report.tags_[1], &snr, sizeof(float));
    TryEnqueueFallback(&mac_request_queue, snr_report);
    base_tag.ue_id_++;
  }
}

void ScheduleDownlinkProcessing(size_t frame_id) {
  size_t num_pilot_symbols = config->Frame().ClientDlPilotSymbols();

  for (size_t i = 0; i < num_pilot_symbols; i++) {
    if (zf_last_frame_ == frame_id) {
      ScheduleSubcarriers(EventType::kPrecode, frame_id,
                          config->Frame().GetDLSymbol(i));
    } else {
      encode_cur_frame_for_symbol_.at(i) = frame_id;
    }
  }

  for (size_t i = num_pilot_symbols; i < config->Frame().NumDLSyms(); i++) {
    ScheduleCodeblocks(EventType::kEncode, Direction::kDownlink, frame_id,
                       config->Frame().GetDLSymbol(i));
  }
}

void ScheduleAntennas(EventType event_type, size_t frame_id, size_t symbol_id) {
  assert(event_type == EventType::kFFT or event_type == EventType::kIFFT);
  auto base_tag = gen_tag_t::FrmSymAnt(frame_id, symbol_id, 0);

  size_t num_blocks = config->BsAntNum() / config->FftBlockSize();
  size_t num_remainder = config->BsAntNum() % config->FftBlockSize();
  if (num_remainder > 0) {
    num_blocks++;
  }
  EventData event;
  event.num_tags_ = config->FftBlockSize();
  event.event_type_ = event_type;
  size_t qid = frame_id & 0x1;
  for (size_t i = 0; i < num_blocks; i++) {
    if ((i == num_blocks - 1) && num_remainder > 0) {
      event.num_tags_ = num_remainder;
    }
    for (size_t j = 0; j < event.num_tags_; j++) {
      event.tags_[j] = base_tag.tag_;
      base_tag.ant_id_++;
    }
    TryEnqueueFallback(GetConq(sched_info_arr, event_type, qid),
                       GetPtok(sched_info_arr, event_type, qid), event);
  }
}

void ScheduleAntennasTX(size_t frame_id, size_t symbol_id) {
  const size_t total_antennas = config->BsAntNum();
  const size_t handler_threads = config->SocketThreadNum();

  // Build the worker event lists
  std::vector<std::vector<EventData>> worker_events(handler_threads);
  for (size_t antenna = 0u; antenna < total_antennas; antenna++) {
    const size_t enqueue_worker_id = packet_tx_rx_->AntNumToWorkerId(antenna);
    EventData tx_data;
    tx_data.num_tags_ = 1;
    tx_data.event_type_ = EventType::kPacketTX;
    tx_data.tags_.at(0) =
        gen_tag_t::FrmSymAnt(frame_id, symbol_id, antenna).tag_;
    worker_events.at(enqueue_worker_id).push_back(tx_data);

    AGORA_LOG_TRACE(
        "ScheduleAntennasTX: (Frame %zu, Symbol %zu, Ant %zu) - tx event added "
        "to worker %zu : %zu\n",
        frame_id, symbol_id, antenna, enqueue_worker_id, worker_events.size());
  }

  //Enqueue all events for all workers
  size_t enqueue_worker_id = 0;
  for (const auto& worker : worker_events) {
    if (!worker.empty()) {
      AGORA_LOG_TRACE(
          "ScheduleAntennasTX: (Frame %zu, Symbol %zu) - adding %zu "
          "event(s) to worker %zu transmit queue\n",
          frame_id, symbol_id, worker.size(), enqueue_worker_id);

      TryEnqueueBulkFallback(GetConq(sched_info_arr, EventType::kPacketTX, 0),
                             tx_ptoks_ptr[enqueue_worker_id], worker.data(),
                             worker.size());
    }
    enqueue_worker_id++;
  }
}

void ScheduleUsers(EventType event_type, size_t frame_id, size_t symbol_id) {
  assert(event_type == EventType::kPacketToMac);
  unused(event_type);
  auto base_tag = gen_tag_t::FrmSymUe(frame_id, symbol_id, 0);

  for (size_t i = 0; i < config->UeAntNum(); i++) {
    TryEnqueueFallback(&mac_request_queue,
                       EventData(EventType::kPacketToMac, base_tag.tag_));
    base_tag.ue_id_++;
  }
}

// Increments the cur_sche_frame_id when all ScheduleProcessingFlags have
// been acheived.
void CheckIncrementScheduleFrame(size_t frame_id,
                                 ScheduleProcessingFlags completed) {
  schedule_process_flags_ += completed;
  assert(cur_sche_frame_id_ == frame_id);
  unused(frame_id);

  if (schedule_process_flags_ ==
      static_cast<uint8_t>(ScheduleProcessingFlags::kProcessingComplete)) {
    cur_sche_frame_id_++;
    schedule_process_flags_ = ScheduleProcessingFlags::kNone;
    if (config->Frame().NumULSyms() == 0) {
      schedule_process_flags_ += ScheduleProcessingFlags::kUplinkComplete;
    }
    if (config->Frame().NumDLSyms() == 0) {
      schedule_process_flags_ += ScheduleProcessingFlags::kDownlinkComplete;
    }
  }
}

/// Determines if all the work has been completed on the frame_id
/// Completion is determined based on the ifft, tx, decode, and tomac
/// counters. If frame processing is complete.  All of the work counters are
/// reset and the cur_proc_frame_id_ is incremented.  Returns true if all
/// processing is complete AND the frame_id is the last frame to test. False
/// otherwise.
bool CheckFrameComplete(size_t frame_id) {
  bool finished = false;

  AGORA_LOG_TRACE(
      "Checking work complete %zu, ifft %d, tx %d, decode %d, tomac %d, tx "
      "%d\n",
      frame_id, static_cast<int>(ifft_counters_.IsLastSymbol(frame_id)),
      static_cast<int>(tx_counters_.IsLastSymbol(frame_id)),
      static_cast<int>(decode_counters_.IsLastSymbol(frame_id)),
      static_cast<int>(tomac_counters_.IsLastSymbol(frame_id)),
      static_cast<int>(tx_counters_.IsLastSymbol(frame_id)));

  // Complete if last frame and ifft / decode complete
  if ((true == ifft_counters_.IsLastSymbol(frame_id)) &&
      (true == tx_counters_.IsLastSymbol(frame_id)) &&
      (((false == kEnableMac) &&
        (true == decode_counters_.IsLastSymbol(frame_id))) ||
       ((true == kUplinkHardDemod) &&
        (true == demul_counters_.IsLastSymbol(frame_id))) ||
       ((true == kEnableMac) &&
        (true == tomac_counters_.IsLastSymbol(frame_id))))) {
    stats_->UpdateStats(frame_id);
    assert(frame_id == cur_proc_frame_id);
    if (true == kUplinkHardDemod) {
      demul_counters_.Reset(frame_id);
    }
    decode_counters_.Reset(frame_id);
    tomac_counters_.Reset(frame_id);
    ifft_counters_.Reset(frame_id);
    tx_counters_.Reset(frame_id);
    if (config->Frame().NumDLSyms() > 0) {
      for (size_t ue_id = 0; ue_id < config->UeAntNum(); ue_id++) {
        dl_bits_buffer_status_[ue_id][frame_id % kFrameWnd] = 0;
      }
    }
    cur_proc_frame_id++;

    if (frame_id == (config->FramesToTest() - 1)) {
      finished = true;
    } else {
      // Only schedule up to kScheduleQueues so we don't flood the queues
      // Cannot access the front() element if the queue is empty
      for (size_t encode = 0;
           (encode < kScheduleQueues) && (!encode_deferral_.empty());
           encode++) {
        const size_t deferred_frame = encode_deferral_.front();
        if (deferred_frame < (cur_proc_frame_id + kScheduleQueues)) {
          if (kDebugDeferral) {
            AGORA_LOG_INFO("   +++ Scheduling deferred frame %zu : %zu \n",
                           deferred_frame, cur_proc_frame_id);
          }
          RtAssert(deferred_frame >= cur_proc_frame_id,
                   "Error scheduling encoding because deferral frame is less "
                   "than current frame");
          ScheduleDownlinkProcessing(deferred_frame);
          encode_deferral_.pop();
        } else {
          // No need to check the next frame because it is too large
          break;
        }
      }  // for each encodable frames in kScheduleQueues
    }    // !finished
  }
  return finished;
}