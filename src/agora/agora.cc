/**
 * @file agora.cc
 * @brief Implementation file for the main agora class
 */

#include "agora.h"

#include <cmath>
#include <memory>

#if defined(USE_DPDK)
#include "packet_txrx_dpdk.h"
#endif
#include "gettime.h"
#include "logger.h"
#include "message.h"
#include "modulation.h"
#include "packet_txrx_radio.h"
#include "packet_txrx_sim.h"
#include "signal_handler.h"

static const bool kDebugPrintPacketsFromMac = false;
static const bool kDebugDeferral = true;

static const std::string kProjectDirectory = TOSTRING(PROJECT_DIRECTORY);
static const std::string kOutputFilepath =
    kProjectDirectory + "/files/experiment/";
static const std::string kTxDataFilename = kOutputFilepath + "tx_data.bin";
static const std::string kDecodeDataFilename =
    kOutputFilepath + "decode_data.bin";

Agora::Agora(Config* const cfg)
    : base_worker_core_offset_(cfg->CoreOffset() + 1 + cfg->SocketThreadNum()),
      config_(cfg),
      stats_(std::make_unique<Stats>(cfg)),
      phy_stats_(std::make_unique<PhyStats>(cfg, Direction::kUplink)),
      buffer_(std::make_unique<AgoraBuffer>(cfg)) {
  std::string directory = TOSTRING(PROJECT_DIRECTORY);
  AGORA_LOG_INFO("Agora: project directory [%s], RDTSC frequency = %.2f GHz\n",
                 kProjectDirectory.c_str(), cfg->FreqGhz());

  PinToCoreWithOffset(ThreadType::kMaster, cfg->CoreOffset(), 0,
                      kEnableCoreReuse, false /* quiet */);
  CheckIncrementScheduleFrame(0, ScheduleProcessingFlags::kProcessingComplete);
  // Important to set frame_.cur_sche_frame_id_ after the call to
  // CheckIncrementScheduleFrame because it will be incremented however,
  // CheckIncrementScheduleFrame will initialize the schedule tracking variable
  // correctly.
  frame_.cur_sche_frame_id_ = 0;
  frame_.cur_proc_frame_id_ = 0;

  InitializeQueues();
  InitializeCounters();
  InitializeThreads();
}

Agora::~Agora() {
  if (kEnableMac == true) {
    mac_std_thread_.join();
  }

  stats_.reset();
  phy_stats_.reset();
  worker_.reset();
  FreeQueues();
}

void Agora::Stop() {
  AGORA_LOG_INFO("Agora: terminating\n");
  config_->Running(false);
  usleep(1000);
  packet_tx_rx_.reset();
}

void Agora::SendSnrReport(EventType event_type, size_t frame_id,
                          size_t symbol_id) {
  assert(event_type == EventType::kSNRReport);
  unused(event_type);
  auto base_tag = gen_tag_t::FrmSymUe(frame_id, symbol_id, 0);
  for (size_t i = 0; i < config_->UeAntNum(); i++) {
    EventData snr_report(EventType::kSNRReport, base_tag.tag_);
    snr_report.num_tags_ = 2;
    float snr = this->phy_stats_->GetEvmSnr(frame_id, i);
    std::memcpy(&snr_report.tags_[1], &snr, sizeof(float));
    TryEnqueueFallback(&mac_request_queue_, snr_report);
    base_tag.ue_id_++;
  }
}

void Agora::ScheduleDownlinkProcessing(size_t frame_id) {
  size_t num_pilot_symbols = config_->Frame().ClientDlPilotSymbols();

  for (size_t i = 0; i < num_pilot_symbols; i++) {
    if (beam_last_frame_ == frame_id) {
      ScheduleSubcarriers(EventType::kPrecode, frame_id,
                          config_->Frame().GetDLSymbol(i));
    } else {
      encode_cur_frame_for_symbol_.at(i) = frame_id;
    }
  }

  for (size_t i = num_pilot_symbols; i < config_->Frame().NumDLSyms(); i++) {
    ScheduleCodeblocks(EventType::kEncode, Direction::kDownlink, frame_id,
                       config_->Frame().GetDLSymbol(i));
  }
}

void Agora::ScheduleAntennas(EventType event_type, size_t frame_id,
                             size_t symbol_id) {
  assert(event_type == EventType::kFFT or event_type == EventType::kIFFT);
  auto base_tag = gen_tag_t::FrmSymAnt(frame_id, symbol_id, 0);

  size_t num_blocks = config_->BsAntNum() / config_->FftBlockSize();
  size_t num_remainder = config_->BsAntNum() % config_->FftBlockSize();
  if (num_remainder > 0) {
    num_blocks++;
  }
  EventData event;
  event.num_tags_ = config_->FftBlockSize();
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
    TryEnqueueFallback(GetConq(message_.sched_info_arr_, event_type, qid),
                       GetPtok(message_.sched_info_arr_, event_type, qid),
                       event);
  }
}

void Agora::ScheduleAntennasTX(size_t frame_id, size_t symbol_id) {
  const size_t total_antennas = config_->BsAntNum();
  const size_t handler_threads = config_->SocketThreadNum();

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

      TryEnqueueBulkFallback(
          GetConq(message_.sched_info_arr_, EventType::kPacketTX, 0),
          tx_ptoks_ptr_[enqueue_worker_id], worker.data(), worker.size());
    }
    enqueue_worker_id++;
  }
}

void Agora::ScheduleSubcarriers(EventType event_type, size_t frame_id,
                                size_t symbol_id) {
  auto base_tag = gen_tag_t::FrmSymSc(frame_id, symbol_id, 0);
  size_t num_events = SIZE_MAX;
  size_t block_size = SIZE_MAX;

  switch (event_type) {
    case EventType::kDemul:
    case EventType::kPrecode:
      num_events = config_->DemulEventsPerSymbol();
      block_size = config_->DemulBlockSize();
      break;
    case EventType::kBeam:
      num_events = config_->BeamEventsPerSymbol();
      block_size = config_->BeamBlockSize();
      break;
    default:
      assert(false);
  }

  size_t qid = (frame_id & 0x1);
  if (event_type == EventType::kBeam) {
    EventData event;
    event.event_type_ = event_type;
    event.num_tags_ = config_->BeamBatchSize();
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
      TryEnqueueFallback(GetConq(message_.sched_info_arr_, event_type, qid),
                         GetPtok(message_.sched_info_arr_, event_type, qid),
                         event);
    }
  } else {
    for (size_t i = 0; i < num_events; i++) {
      TryEnqueueFallback(GetConq(message_.sched_info_arr_, event_type, qid),
                         GetPtok(message_.sched_info_arr_, event_type, qid),
                         EventData(event_type, base_tag.tag_));
      base_tag.sc_id_ += block_size;
    }
  }
}

void Agora::ScheduleCodeblocks(EventType event_type, Direction dir,
                               size_t frame_id, size_t symbol_idx) {
  auto base_tag = gen_tag_t::FrmSymCb(frame_id, symbol_idx, 0);
  const size_t num_tasks =
      config_->UeAntNum() * config_->LdpcConfig(dir).NumBlocksInSymbol();
  size_t num_blocks = num_tasks / config_->EncodeBlockSize();
  const size_t num_remainder = num_tasks % config_->EncodeBlockSize();
  if (num_remainder > 0) {
    num_blocks++;
  }
  EventData event;
  event.num_tags_ = config_->EncodeBlockSize();
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
    TryEnqueueFallback(GetConq(message_.sched_info_arr_, event_type, qid),
                       GetPtok(message_.sched_info_arr_, event_type, qid),
                       event);
  }
}

void Agora::ScheduleUsers(EventType event_type, size_t frame_id,
                          size_t symbol_id) {
  assert(event_type == EventType::kPacketToMac);
  unused(event_type);
  auto base_tag = gen_tag_t::FrmSymUe(frame_id, symbol_id, 0);

  for (size_t i = 0; i < config_->UeAntNum(); i++) {
    TryEnqueueFallback(&mac_request_queue_,
                       EventData(EventType::kPacketToMac, base_tag.tag_));
    base_tag.ue_id_++;
  }
}

size_t Agora::FetchEvent(EventData events_list[],
                         bool is_turn_to_dequeue_from_io) {
  const size_t max_events_needed = std::max(
      kDequeueBulkSizeTXRX * (config_->SocketThreadNum() + 1 /* MAC */),
      kDequeueBulkSizeWorker * config_->WorkerThreadNum());
  size_t num_events = 0;
  if (is_turn_to_dequeue_from_io) {
    for (size_t i = 0; i < config_->SocketThreadNum(); i++) {
      num_events += message_queue_.try_dequeue_bulk_from_producer(
          *(rx_ptoks_ptr_[i]), events_list + num_events, kDequeueBulkSizeTXRX);
    }

    if (kEnableMac == true) {
      num_events += mac_response_queue_.try_dequeue_bulk(
          events_list + num_events, kDequeueBulkSizeTXRX);
    }
  } else {
    num_events +=
        message_.complete_task_queue_[(frame_.cur_proc_frame_id_ & 0x1)]
            .try_dequeue_bulk(events_list + num_events, max_events_needed);
  }
  return num_events;
}

void Agora::Start() {
  const auto& cfg = this->config_;

  const bool start_status = packet_tx_rx_->StartTxRx(buffer_->calib_dl_buffer_,
                                                     buffer_->calib_ul_buffer_);
  // Start packet I/O
  if (start_status == false) {
    this->Stop();
    return;
  }

  // Counters for printing summary
  size_t tx_count = 0;
  double tx_begin = GetTime::GetTimeUs();

  bool is_turn_to_dequeue_from_io = true;
  const size_t max_events_needed =
      std::max(kDequeueBulkSizeTXRX * (cfg->SocketThreadNum() + 1 /* MAC */),
               kDequeueBulkSizeWorker * cfg->WorkerThreadNum());
  EventData events_list[max_events_needed];

  while ((config_->Running() == true) &&
         (SignalHandler::GotExitSignal() == false)) {
    // Get a batch of events
    size_t const num_events =
        FetchEvent(events_list, is_turn_to_dequeue_from_io);
    is_turn_to_dequeue_from_io = !is_turn_to_dequeue_from_io;

    // Handle each event
    for (size_t ev_i = 0; ev_i < num_events; ev_i++) {
      EventData& event = events_list[ev_i];

      // FFT processing is scheduled after falling through the switch
      switch (event.event_type_) {
        case EventType::kPacketRX: {
          Packet* pkt = rx_tag_t(event.tags_[0]).rx_packet_->RawPacket();

          if (pkt->frame_id_ >=
              ((this->frame_.cur_sche_frame_id_ + kFrameWnd))) {
            AGORA_LOG_ERROR(
                "Error: Received packet for future frame %u beyond "
                "frame window (= %zu + %zu). This can happen if "
                "Agora is running slowly, e.g., in debug mode\n",
                pkt->frame_id_, this->frame_.cur_sche_frame_id_, kFrameWnd);
            cfg->Running(false);
            break;
          }

          UpdateRxCounters(pkt->frame_id_, pkt->symbol_id_);
          fft_queue_arr_.at(pkt->frame_id_ % kFrameWnd)
              .push(fft_req_tag_t(event.tags_[0]));
        } break;

        case EventType::kFFT: {
          for (size_t i = 0; i < event.num_tags_; i++) {
            HandleEventFft(event.tags_[i]);
          }
        } break;

        case EventType::kBeam: {
          for (size_t tag_id = 0; (tag_id < event.num_tags_); tag_id++) {
            const size_t frame_id = gen_tag_t(event.tags_[tag_id]).frame_id_;
            stats_->PrintPerTaskDone(PrintType::kBeam, frame_id, 0,
                                     beam_counters_.GetTaskCount(frame_id),
                                     this->beam_counters_);
            const bool last_beam_task =
                this->beam_counters_.CompleteTask(frame_id);
            if (last_beam_task == true) {
              this->stats_->MasterSetTsc(TsType::kBeamDone, frame_id);
              beam_last_frame_ = frame_id;
              stats_->PrintPerFrameDone(PrintType::kBeam, frame_id);
              this->beam_counters_.Reset(frame_id);
              if (kPrintBeamStats) {
                this->phy_stats_->PrintBeamStats(frame_id);
              }

              for (size_t i = 0; i < cfg->Frame().NumULSyms(); i++) {
                if (this->fft_cur_frame_for_symbol_.at(i) == frame_id) {
                  ScheduleSubcarriers(EventType::kDemul, frame_id,
                                      cfg->Frame().GetULSymbol(i));
                }
              }
              // Schedule precoding for downlink symbols
              for (size_t i = 0; i < cfg->Frame().NumDLSyms(); i++) {
                const size_t last_encoded_frame =
                    this->encode_cur_frame_for_symbol_.at(i);
                if ((last_encoded_frame != SIZE_MAX) &&
                    (last_encoded_frame >= frame_id)) {
                  ScheduleSubcarriers(EventType::kPrecode, frame_id,
                                      cfg->Frame().GetDLSymbol(i));
                }
              }
            }  // end if (beam_counters_.last_task(frame_id) == true)
          }
        } break;

        case EventType::kDemul: {
          const size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          const size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;
          const size_t base_sc_id = gen_tag_t(event.tags_[0]).sc_id_;

          stats_->PrintPerTaskDone(PrintType::kDemul, frame_id, symbol_id,
                                   base_sc_id, this->demul_counters_);
          const bool last_demul_task =
              this->demul_counters_.CompleteTask(frame_id, symbol_id);

          if (last_demul_task == true) {
            if (kUplinkHardDemod == false) {
              ScheduleCodeblocks(EventType::kDecode, Direction::kUplink,
                                 frame_id, symbol_id);
            }
            stats_->PrintPerSymbolDone(PrintType::kDemul, frame_id, symbol_id,
                                       this->demul_counters_);
            const bool last_demul_symbol =
                this->demul_counters_.CompleteSymbol(frame_id);
            if (last_demul_symbol == true) {
              max_equaled_frame_ = frame_id;
              this->stats_->MasterSetTsc(TsType::kDemulDone, frame_id);
              stats_->PrintPerFrameDone(PrintType::kDemul, frame_id);
              if (kPrintPhyStats) {
                this->phy_stats_->PrintEvmStats(frame_id);
              }
              this->phy_stats_->RecordEvm(frame_id);
              this->phy_stats_->RecordEvmSnr(frame_id);
              if (kUplinkHardDemod) {
                this->phy_stats_->RecordBer(frame_id);
                this->phy_stats_->RecordSer(frame_id);
              }
              this->phy_stats_->ClearEvmBuffer(frame_id);

              // skip Decode when hard demod is enabled
              if (kUplinkHardDemod == true) {
                assert(this->frame_.cur_proc_frame_id_ == frame_id);
                CheckIncrementScheduleFrame(frame_id, kUplinkComplete);
                const bool work_finished = this->CheckFrameComplete(frame_id);
                if (work_finished == true) {
                  goto finish;
                }
              } else {
                this->demul_counters_.Reset(frame_id);
                if (cfg->BigstationMode() == false) {
                  assert(frame_.cur_sche_frame_id_ == frame_id);
                  CheckIncrementScheduleFrame(frame_id, kUplinkComplete);
                } else {
                  ScheduleCodeblocks(EventType::kDecode, Direction::kUplink,
                                     frame_id, symbol_id);
                }
              }
            }
          }
        } break;

        case EventType::kDecode: {
          const size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          const size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;

          const bool last_decode_task =
              this->decode_counters_.CompleteTask(frame_id, symbol_id);
          if (last_decode_task == true) {
            if (kEnableMac == true) {
              ScheduleUsers(EventType::kPacketToMac, frame_id, symbol_id);
            }
            stats_->PrintPerSymbolDone(PrintType::kDecode, frame_id, symbol_id,
                                       this->decode_counters_);
            const bool last_decode_symbol =
                this->decode_counters_.CompleteSymbol(frame_id);
            if (last_decode_symbol == true) {
              this->stats_->MasterSetTsc(TsType::kDecodeDone, frame_id);
              stats_->PrintPerFrameDone(PrintType::kDecode, frame_id);
              this->phy_stats_->RecordBer(frame_id);
              this->phy_stats_->RecordSer(frame_id);
              if (kEnableMac == false) {
                assert(this->frame_.cur_proc_frame_id_ == frame_id);
                const bool work_finished = this->CheckFrameComplete(frame_id);
                if (work_finished == true) {
                  goto finish;
                }
              }
            }
          }
        } break;

        case EventType::kRANUpdate: {
          RanConfig rc;
          rc.n_antennas_ = event.tags_[0];
          rc.mod_order_bits_ = event.tags_[1];
          rc.frame_id_ = event.tags_[2];
          UpdateRanConfig(rc);
        } break;

        case EventType::kPacketToMac: {
          const size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          const size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;

          const bool last_tomac_task =
              this->tomac_counters_.CompleteTask(frame_id, symbol_id);
          if (last_tomac_task == true) {
            stats_->PrintPerSymbolDone(PrintType::kPacketToMac, frame_id,
                                       symbol_id, this->tomac_counters_);

            const bool last_tomac_symbol =
                this->tomac_counters_.CompleteSymbol(frame_id);
            if (last_tomac_symbol == true) {
              assert(this->frame_.cur_proc_frame_id_ == frame_id);
              // this->stats_->MasterSetTsc(TsType::kMacTXDone, frame_id);
              stats_->PrintPerFrameDone(PrintType::kPacketToMac, frame_id);
              const bool work_finished = this->CheckFrameComplete(frame_id);
              if (work_finished == true) {
                goto finish;
              }
            }
          }
        } break;

        case EventType::kPacketFromMac: {
          // This is an entire frame (multiple mac packets)
          const size_t ue_id = rx_mac_tag_t(event.tags_[0u]).tid_;
          const size_t radio_buf_id = rx_mac_tag_t(event.tags_[0u]).offset_;
          const auto* pkt = reinterpret_cast<const MacPacketPacked*>(
              &buffer_->GetDlBitsBuffer(
                  ue_id)[radio_buf_id *
                         config_->MacBytesNumPerframe(Direction::kDownlink)]);

          AGORA_LOG_INFO("Agora: frame %d @ offset %zu %zu @ location %zu\n",
                         pkt->Frame(), ue_id, radio_buf_id,
                         reinterpret_cast<intptr_t>(pkt));

          if (kDebugPrintPacketsFromMac) {
            std::stringstream ss;

            for (size_t dl_data_symbol = 0;
                 dl_data_symbol < config_->Frame().NumDlDataSyms();
                 dl_data_symbol++) {
              ss << "Agora: kPacketFromMac, frame " << pkt->Frame()
                 << ", symbol " << std::to_string(pkt->Symbol()) << " crc "
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
          const bool last_ue =
              this->mac_to_phy_counters_.CompleteTask(frame_id, 0);
          if (last_ue == true) {
            // schedule this frame's encoding
            // Defer the schedule.  If frames are already deferred or the
            // current received frame is too far off
            if ((this->encode_deferral_.empty() == false) ||
                (frame_id >=
                 (this->frame_.cur_proc_frame_id_ + kScheduleQueues))) {
              if (kDebugDeferral) {
                AGORA_LOG_INFO("   +++ Deferring encoding of frame %zu\n",
                               frame_id);
              }
              this->encode_deferral_.push(frame_id);
            } else {
              ScheduleDownlinkProcessing(frame_id);
            }
            this->mac_to_phy_counters_.Reset(frame_id);
            stats_->PrintPerFrameDone(PrintType::kPacketFromMac, frame_id);
          }
        } break;

        case EventType::kEncode: {
          for (size_t i = 0u; i < event.num_tags_; i++) {
            const size_t frame_id = gen_tag_t(event.tags_[i]).frame_id_;
            const size_t symbol_id = gen_tag_t(event.tags_[i]).symbol_id_;

            const bool last_encode_task =
                encode_counters_.CompleteTask(frame_id, symbol_id);
            if (last_encode_task == true) {
              this->encode_cur_frame_for_symbol_.at(
                  cfg->Frame().GetDLSymbolIdx(symbol_id)) = frame_id;
              // If precoder of the current frame exists
              if (beam_last_frame_ == frame_id) {
                ScheduleSubcarriers(EventType::kPrecode, frame_id, symbol_id);
              }
              stats_->PrintPerSymbolDone(PrintType::kEncode, frame_id,
                                         symbol_id, this->encode_counters_);

              const bool last_encode_symbol =
                  this->encode_counters_.CompleteSymbol(frame_id);
              if (last_encode_symbol == true) {
                this->encode_counters_.Reset(frame_id);
                this->stats_->MasterSetTsc(TsType::kEncodeDone, frame_id);
                stats_->PrintPerFrameDone(PrintType::kEncode, frame_id);
              }
            }
          }
        } break;

        case EventType::kPrecode: {
          // Precoding is done, schedule ifft
          const size_t sc_id = gen_tag_t(event.tags_[0]).sc_id_;
          const size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          const size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;
          stats_->PrintPerTaskDone(PrintType::kPrecode, frame_id, symbol_id,
                                   sc_id, this->precode_counters_);
          const bool last_precode_task =
              this->precode_counters_.CompleteTask(frame_id, symbol_id);

          if (last_precode_task == true) {
            // precode_cur_frame_for_symbol_.at(
            //    this->config_->Frame().GetDLSymbolIdx(symbol_id)) = frame_id;
            ScheduleAntennas(EventType::kIFFT, frame_id, symbol_id);
            stats_->PrintPerSymbolDone(PrintType::kPrecode, frame_id, symbol_id,
                                       this->precode_counters_);

            const bool last_precode_symbol =
                this->precode_counters_.CompleteSymbol(frame_id);
            if (last_precode_symbol == true) {
              this->precode_counters_.Reset(frame_id);
              this->stats_->MasterSetTsc(TsType::kPrecodeDone, frame_id);
              stats_->PrintPerFrameDone(PrintType::kPrecode, frame_id);
            }
          }
        } break;

        case EventType::kIFFT: {
          for (size_t i = 0; i < event.num_tags_; i++) {
            /* IFFT is done, schedule data transmission */
            const size_t ant_id = gen_tag_t(event.tags_[i]).ant_id_;
            const size_t frame_id = gen_tag_t(event.tags_[i]).frame_id_;
            const size_t symbol_id = gen_tag_t(event.tags_[i]).symbol_id_;
            const size_t symbol_idx_dl = cfg->Frame().GetDLSymbolIdx(symbol_id);
            stats_->PrintPerTaskDone(PrintType::kIFFT, frame_id, symbol_id,
                                     ant_id, this->ifft_counters_);

            const bool last_ifft_task =
                this->ifft_counters_.CompleteTask(frame_id, symbol_id);
            if (last_ifft_task == true) {
              ifft_cur_frame_for_symbol_.at(symbol_idx_dl) = frame_id;
              if (symbol_idx_dl == ifft_next_symbol_) {
                // Check the available symbols starting from the current symbol
                // Only schedule symbols that are continuously available
                for (size_t sym_id = symbol_idx_dl;
                     sym_id <= ifft_counters_.GetSymbolCount(frame_id);
                     sym_id++) {
                  const size_t symbol_ifft_frame =
                      ifft_cur_frame_for_symbol_.at(sym_id);
                  if (symbol_ifft_frame == frame_id) {
                    ScheduleAntennasTX(frame_id,
                                       cfg->Frame().GetDLSymbol(sym_id));
                    ifft_next_symbol_++;
                  } else {
                    break;
                  }
                }
              }
              stats_->PrintPerSymbolDone(PrintType::kIFFT, frame_id, symbol_id,
                                         this->ifft_counters_);

              const bool last_ifft_symbol =
                  this->ifft_counters_.CompleteSymbol(frame_id);
              if (last_ifft_symbol == true) {
                ifft_next_symbol_ = 0;
                this->stats_->MasterSetTsc(TsType::kIFFTDone, frame_id);
                stats_->PrintPerFrameDone(PrintType::kIFFT, frame_id);
                assert(frame_id == this->frame_.cur_proc_frame_id_);
                this->CheckIncrementScheduleFrame(frame_id, kDownlinkComplete);
                const bool work_finished = this->CheckFrameComplete(frame_id);
                if (work_finished == true) {
                  goto finish;
                }
              }
            }
          }
        } break;

        case EventType::kPacketTX: {
          // Data is sent
          const size_t ant_id = gen_tag_t(event.tags_[0]).ant_id_;
          const size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          const size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;
          stats_->PrintPerTaskDone(PrintType::kPacketTX, frame_id, symbol_id,
                                   ant_id, this->tx_counters_);

          const bool last_tx_task =
              this->tx_counters_.CompleteTask(frame_id, symbol_id);
          if (last_tx_task == true) {
            stats_->PrintPerSymbolDone(PrintType::kPacketTX, frame_id,
                                       symbol_id, this->tx_counters_);
            // If tx of the first symbol is done
            if (symbol_id == cfg->Frame().GetDLSymbol(0)) {
              this->stats_->MasterSetTsc(TsType::kTXProcessedFirst, frame_id);
              stats_->PrintPerFrameDone(PrintType::kPacketTXFirst, frame_id);
            }

            const bool last_tx_symbol =
                this->tx_counters_.CompleteSymbol(frame_id);
            if (last_tx_symbol == true) {
              this->stats_->MasterSetTsc(TsType::kTXDone, frame_id);
              stats_->PrintPerFrameDone(PrintType::kPacketTX, frame_id);

              const bool work_finished = this->CheckFrameComplete(frame_id);
              if (work_finished == true) {
                goto finish;
              }
            }

            tx_count++;
            if (tx_count == tx_counters_.MaxSymbolCount() * 9000) {
              tx_count = 0;

              const double diff = GetTime::GetTimeUs() - tx_begin;
              const int samples_num_per_ue =
                  cfg->OfdmDataNum() * tx_counters_.MaxSymbolCount() * 1000;

              AGORA_LOG_INFO(
                  "TX %d samples (per-client) to %zu clients in %f secs, "
                  "throughtput %f bps per-client (16QAM), current tx queue "
                  "length %zu\n",
                  samples_num_per_ue, cfg->UeAntNum(), diff,
                  samples_num_per_ue * std::log2(16.0f) / diff,
                  GetConq(message_.sched_info_arr_, EventType::kPacketTX, 0)
                      ->size_approx());
              unused(diff);
              unused(samples_num_per_ue);
              tx_begin = GetTime::GetTimeUs();
            }
          }
        } break;
        default:
          AGORA_LOG_ERROR("Wrong event type in message queue!");
          std::exit(0);
      } /* End of switch */

      // We schedule FFT processing if the event handling above results in
      // either (a) sufficient packets received for the current frame,
      // or (b) the current frame being updated.
      std::queue<fft_req_tag_t>& cur_fftq =
          fft_queue_arr_.at(this->frame_.cur_sche_frame_id_ % kFrameWnd);
      const size_t qid = this->frame_.cur_sche_frame_id_ & 0x1;
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

            if (this->fft_created_count_ == 0) {
              this->stats_->MasterSetTsc(TsType::kProcessingStarted,
                                         this->frame_.cur_sche_frame_id_);
            }
            this->fft_created_count_++;
            if (this->fft_created_count_ ==
                rx_counters_.num_rx_pkts_per_frame_) {
              this->fft_created_count_ = 0;
              if (cfg->BigstationMode() == true) {
                this->CheckIncrementScheduleFrame(frame_.cur_sche_frame_id_,
                                                  kUplinkComplete);
              }
            }
          }
          TryEnqueueFallback(
              GetConq(message_.sched_info_arr_, EventType::kFFT, qid),
              GetPtok(message_.sched_info_arr_, EventType::kFFT, qid),
              do_fft_task);
        }
      }
    } /* End of for */
  }   /* End of while */

finish:
  AGORA_LOG_INFO("Agora: printing stats and saving to file\n");
  this->stats_->PrintSummary();
  this->stats_->SaveToFile();
  if (flags_.enable_save_decode_data_to_file_ == true) {
    SaveDecodeDataToFile(this->stats_->LastFrameId());
  }
  if (flags_.enable_save_tx_data_to_file_ == true) {
    SaveTxDataToFile(this->stats_->LastFrameId());
  }

  // Calculate and print per-user BER
  if ((kEnableMac == false) && (kPrintPhyStats == true)) {
    this->phy_stats_->PrintPhyStats();
  }
  this->Stop();
}

void Agora::InitializeQueues() {
  using mt_queue_t = moodycamel::ConcurrentQueue<EventData>;

  int data_symbol_num_perframe = config_->Frame().NumDataSyms();
  message_queue_ =
      mt_queue_t(kDefaultMessageQueueSize * data_symbol_num_perframe);
  for (auto& c : message_.complete_task_queue_) {
    c = mt_queue_t(kDefaultWorkerQueueSize * data_symbol_num_perframe);
  }
  // Create concurrent queues for each Doer
  for (auto& vec : message_.sched_info_arr_) {
    for (auto& s : vec) {
      s.concurrent_q_ =
          mt_queue_t(kDefaultWorkerQueueSize * data_symbol_num_perframe);
      s.ptok_ = new moodycamel::ProducerToken(s.concurrent_q_);
    }
  }

  for (size_t i = 0; i < config_->SocketThreadNum(); i++) {
    rx_ptoks_ptr_[i] = new moodycamel::ProducerToken(message_queue_);
    tx_ptoks_ptr_[i] = new moodycamel::ProducerToken(
        *GetConq(message_.sched_info_arr_, EventType::kPacketTX, 0));
  }

  for (size_t i = 0; i < config_->WorkerThreadNum(); i++) {
    for (size_t j = 0; j < kScheduleQueues; j++) {
      message_.worker_ptoks_ptr_[i][j] =
          new moodycamel::ProducerToken(message_.complete_task_queue_[j]);
    }
  }
}

void Agora::FreeQueues() {
  // remove tokens for each doer
  for (auto& vec : message_.sched_info_arr_) {
    for (auto& s : vec) {
      delete s.ptok_;
    }
  }

  for (size_t i = 0; i < config_->SocketThreadNum(); i++) {
    delete rx_ptoks_ptr_[i];
    delete tx_ptoks_ptr_[i];
  }

  for (size_t i = 0; i < config_->WorkerThreadNum(); i++) {
    for (size_t j = 0; j < kScheduleQueues; j++) {
      delete message_.worker_ptoks_ptr_[i][j];
    }
  }
}

void Agora::InitializeCounters() {
  const auto& cfg = config_;

  rx_counters_.num_pilot_pkts_per_frame_ =
      cfg->BsAntNum() * cfg->Frame().NumPilotSyms();
  // BfAntNum() for each 'L' symbol (no ref node)
  // RefRadio * NumChannels() for each 'C'.
  //rx_counters_.num_reciprocity_pkts_per_frame_ = cfg->BsAntNum();
  const size_t num_rx_ul_cal_antennas = cfg->BfAntNum();
  // Same as the number of rx reference antennas (ref ant + other channels)
  const size_t num_rx_dl_cal_antennas = cfg->BsAntNum() - cfg->BfAntNum();

  rx_counters_.num_reciprocity_pkts_per_frame_ =
      (cfg->Frame().NumULCalSyms() * num_rx_ul_cal_antennas) +
      (cfg->Frame().NumDLCalSyms() * num_rx_dl_cal_antennas);

  AGORA_LOG_INFO("Agora: Total recip cal receive symbols per frame: %zu\n",
                 rx_counters_.num_reciprocity_pkts_per_frame_);

  rx_counters_.num_rx_pkts_per_frame_ =
      rx_counters_.num_pilot_pkts_per_frame_ +
      rx_counters_.num_reciprocity_pkts_per_frame_ +
      (cfg->BsAntNum() * cfg->Frame().NumULSyms());

  fft_created_count_ = 0;
  pilot_fft_counters_.Init(cfg->Frame().NumPilotSyms(), cfg->BsAntNum());
  uplink_fft_counters_.Init(cfg->Frame().NumULSyms(), cfg->BsAntNum());
  fft_cur_frame_for_symbol_ =
      std::vector<size_t>(cfg->Frame().NumULSyms(), SIZE_MAX);

  rc_counters_.Init(cfg->BsAntNum());

  beam_counters_.Init(cfg->BeamEventsPerSymbol());

  demul_counters_.Init(cfg->Frame().NumULSyms(), cfg->DemulEventsPerSymbol());

  decode_counters_.Init(
      cfg->Frame().NumULSyms(),
      cfg->LdpcConfig(Direction::kUplink).NumBlocksInSymbol() *
          cfg->UeAntNum());

  tomac_counters_.Init(cfg->Frame().NumULSyms(), cfg->UeAntNum());

  // Downlink
  if (config_->Frame().NumDLSyms() > 0) {
    encode_counters_.Init(
        config_->Frame().NumDlDataSyms(),
        config_->LdpcConfig(Direction::kDownlink).NumBlocksInSymbol() *
            config_->UeAntNum());
    encode_cur_frame_for_symbol_ =
        std::vector<size_t>(config_->Frame().NumDLSyms(), SIZE_MAX);
    ifft_cur_frame_for_symbol_ =
        std::vector<size_t>(config_->Frame().NumDLSyms(), SIZE_MAX);
    precode_counters_.Init(config_->Frame().NumDLSyms(),
                           config_->DemulEventsPerSymbol());
    // precode_cur_frame_for_symbol_ =
    //    std::vector<size_t>(config_->Frame().NumDLSyms(), SIZE_MAX);
    ifft_counters_.Init(config_->Frame().NumDLSyms(), config_->BsAntNum());
    tx_counters_.Init(config_->Frame().NumDLSyms(), config_->BsAntNum());
    // mac data is sent per frame, so we set max symbol to 1
    mac_to_phy_counters_.Init(1, config_->UeAntNum());
  }
}

void Agora::InitializeThreads() {
  const auto& cfg = config_;
  /* Initialize TXRX threads */
  if (kUseArgos || kUseUHD) {
    packet_tx_rx_ = std::make_unique<PacketTxRxRadio>(
        cfg, cfg->CoreOffset() + 1, &message_queue_,
        GetConq(message_.sched_info_arr_, EventType::kPacketTX, 0),
        rx_ptoks_ptr_, tx_ptoks_ptr_, buffer_->socket_buffer_,
        buffer_->GetSocketBufferSize() / cfg->PacketLength(),
        this->stats_->FrameStart(), buffer_->dl_socket_buffer_);
#if defined(USE_DPDK)
  } else if (kUseDPDK) {
    packet_tx_rx_ = std::make_unique<PacketTxRxDpdk>(
        cfg, cfg->CoreOffset() + 1, &message_queue_,
        GetConq(message_.sched_info_arr_, EventType::kPacketTX, 0),
        rx_ptoks_ptr_, tx_ptoks_ptr_, buffer_->socket_buffer_,
        buffer_->GetSocketBufferSize() / cfg->PacketLength(),
        this->stats_->FrameStart(), buffer_->dl_socket_buffer_);
#endif
  } else {
    /* Default to the simulator */
    packet_tx_rx_ = std::make_unique<PacketTxRxSim>(
        cfg, cfg->CoreOffset() + 1, &message_queue_,
        GetConq(message_.sched_info_arr_, EventType::kPacketTX, 0),
        rx_ptoks_ptr_, tx_ptoks_ptr_, buffer_->socket_buffer_,
        buffer_->GetSocketBufferSize() / cfg->PacketLength(),
        this->stats_->FrameStart(), buffer_->dl_socket_buffer_);
  }

  if (kEnableMac == true) {
    const size_t mac_cpu_core =
        cfg->CoreOffset() + cfg->SocketThreadNum() + cfg->WorkerThreadNum() + 1;
    mac_thread_ = std::make_unique<MacThreadBaseStation>(
        cfg, mac_cpu_core, buffer_.get(), &mac_request_queue_,
        &mac_response_queue_);

    mac_std_thread_ =
        std::thread(&MacThreadBaseStation::RunEventLoop, mac_thread_.get());
  }

  // Create workers
  worker_ = std::make_unique<AgoraWorker>(cfg, stats_.get(), phy_stats_.get(),
                                          &message_, buffer_.get(), &frame_);

  AGORA_LOG_INFO(
      "Master thread core %zu, TX/RX thread cores %zu--%zu, worker thread "
      "cores %zu--%zu\n",
      cfg->CoreOffset(), cfg->CoreOffset() + 1,
      cfg->CoreOffset() + 1 + cfg->SocketThreadNum() - 1,
      base_worker_core_offset_,
      base_worker_core_offset_ + cfg->WorkerThreadNum() - 1);
}

void Agora::HandleEventFft(size_t tag) {
  const size_t frame_id = gen_tag_t(tag).frame_id_;
  const size_t symbol_id = gen_tag_t(tag).symbol_id_;
  const SymbolType sym_type = config_->GetSymbolType(symbol_id);

  if (sym_type == SymbolType::kPilot) {
    const bool last_fft_task =
        pilot_fft_counters_.CompleteTask(frame_id, symbol_id);
    if (last_fft_task == true) {
      stats_->PrintPerSymbolDone(PrintType::kFFTPilots, frame_id, symbol_id,
                                 pilot_fft_counters_);

      if ((config_->Frame().IsRecCalEnabled() == false) ||
          ((config_->Frame().IsRecCalEnabled() == true) &&
           (this->rc_last_frame_ == frame_id))) {
        // If CSI of all UEs is ready, schedule Beam/prediction
        const bool last_pilot_fft =
            pilot_fft_counters_.CompleteSymbol(frame_id);
        if (last_pilot_fft == true) {
          this->stats_->MasterSetTsc(TsType::kFFTPilotsDone, frame_id);
          stats_->PrintPerFrameDone(PrintType::kFFTPilots, frame_id);
          this->pilot_fft_counters_.Reset(frame_id);
          if (kPrintPhyStats == true) {
            this->phy_stats_->PrintUlSnrStats(frame_id);
          }
          this->phy_stats_->RecordUlPilotSnr(frame_id);
          if (kEnableMac == true) {
            SendSnrReport(EventType::kSNRReport, frame_id, symbol_id);
          }
          ScheduleSubcarriers(EventType::kBeam, frame_id, 0);
        }
      }
    }
  } else if (sym_type == SymbolType::kUL) {
    const size_t symbol_idx_ul = config_->Frame().GetULSymbolIdx(symbol_id);

    const bool last_fft_per_symbol =
        uplink_fft_counters_.CompleteTask(frame_id, symbol_id);

    if (last_fft_per_symbol == true) {
      fft_cur_frame_for_symbol_.at(symbol_idx_ul) = frame_id;

      stats_->PrintPerSymbolDone(PrintType::kFFTData, frame_id, symbol_id,
                                 uplink_fft_counters_);
      // If precoder exist, schedule demodulation
      if (beam_last_frame_ == frame_id) {
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
                               this->rc_counters_);

    const bool last_rc_task = this->rc_counters_.CompleteTask(frame_id);
    if (last_rc_task == true) {
      stats_->PrintPerFrameDone(PrintType::kFFTCal, frame_id);
      this->rc_counters_.Reset(frame_id);
      this->stats_->MasterSetTsc(TsType::kRCDone, frame_id);
      this->rc_last_frame_ = frame_id;

      // See if the calibration has completed
      if (kPrintPhyStats) {
        const size_t frames_for_cal = config_->RecipCalFrameCnt();

        if ((frame_id % frames_for_cal) == 0 && (frame_id > 0)) {
          const size_t previous_cal_slot =
              config_->ModifyRecCalIndex(config_->RecipCalIndex(frame_id), -1);
          //Print the previous index
          phy_stats_->PrintCalibSnrStats(previous_cal_slot);
        }
      }  // kPrintPhyStats
    }    // last_rc_task
  }      // kCaLDL || kCalUl
}

void Agora::UpdateRanConfig(RanConfig rc) {
  nlohmann::json msc_params = config_->MCSParams(Direction::kUplink);
  msc_params["modulation"] = MapModToStr(rc.mod_order_bits_);
  config_->UpdateUlMCS(msc_params);
}

void Agora::UpdateRxCounters(size_t frame_id, size_t symbol_id) {
  const size_t frame_slot = frame_id % kFrameWnd;
  if (config_->IsPilot(frame_id, symbol_id)) {
    rx_counters_.num_pilot_pkts_[frame_slot]++;
    if (rx_counters_.num_pilot_pkts_.at(frame_slot) ==
        rx_counters_.num_pilot_pkts_per_frame_) {
      rx_counters_.num_pilot_pkts_.at(frame_slot) = 0;
      this->stats_->MasterSetTsc(TsType::kPilotAllRX, frame_id);
      this->stats_->PrintPerFrameDone(PrintType::kPacketRXPilots, frame_id);
    }
  } else if (config_->IsCalDlPilot(frame_id, symbol_id) ||
             config_->IsCalUlPilot(frame_id, symbol_id)) {
    rx_counters_.num_reciprocity_pkts_.at(frame_slot)++;
    if (rx_counters_.num_reciprocity_pkts_.at(frame_slot) ==
        rx_counters_.num_reciprocity_pkts_per_frame_) {
      rx_counters_.num_reciprocity_pkts_.at(frame_slot) = 0;
      this->stats_->MasterSetTsc(TsType::kRCAllRX, frame_id);
    }
  }
  // Receive first packet in a frame
  if (rx_counters_.num_pkts_.at(frame_slot) == 0) {
    if (kEnableMac == false) {
      // schedule this frame's encoding
      // Defer the schedule.  If frames are already deferred or the current
      // received frame is too far off
      if ((encode_deferral_.empty() == false) ||
          (frame_id >= (this->frame_.cur_proc_frame_id_ + kScheduleQueues))) {
        if (kDebugDeferral) {
          AGORA_LOG_INFO("   +++ Deferring encoding of frame %zu\n", frame_id);
        }
        encode_deferral_.push(frame_id);
      } else {
        ScheduleDownlinkProcessing(frame_id);
      }
    }
    this->stats_->MasterSetTsc(TsType::kFirstSymbolRX, frame_id);
    if (kDebugPrintPerFrameStart) {
      const size_t prev_frame_slot = (frame_slot + kFrameWnd - 1) % kFrameWnd;
      AGORA_LOG_INFO(
          "Main [frame %zu + %.2f ms since last frame]: Received "
          "first packet. Remaining packets in prev frame: %zu\n",
          frame_id,
          this->stats_->MasterGetDeltaMs(TsType::kFirstSymbolRX, frame_id,
                                         frame_id - 1),
          rx_counters_.num_pkts_[prev_frame_slot]);
    }
  }

  rx_counters_.num_pkts_.at(frame_slot)++;
  if (rx_counters_.num_pkts_.at(frame_slot) ==
      rx_counters_.num_rx_pkts_per_frame_) {
    this->stats_->MasterSetTsc(TsType::kRXDone, frame_id);
    this->stats_->PrintPerFrameDone(PrintType::kPacketRX, frame_id);
    rx_counters_.num_pkts_.at(frame_slot) = 0;
  }
}

void Agora::SaveDecodeDataToFile(int frame_id) {
  const auto& cfg = config_;
  const size_t num_decoded_bytes =
      cfg->NumBytesPerCb(Direction::kUplink) *
      cfg->LdpcConfig(Direction::kUplink).NumBlocksInSymbol();

  AGORA_LOG_INFO("Saving decode data to %s\n", kDecodeDataFilename.c_str());
  FILE* fp = std::fopen(kDecodeDataFilename.c_str(), "wb");

  for (size_t i = 0; i < cfg->Frame().NumULSyms(); i++) {
    for (size_t j = 0; j < cfg->UeAntNum(); j++) {
      int8_t* ptr = buffer_->GetDecodedBuffer(frame_id % kFrameWnd, i, j);
      std::fwrite(ptr, num_decoded_bytes, sizeof(uint8_t), fp);
    }
  }
  std::fclose(fp);
}

void Agora::SaveTxDataToFile(UNUSED int frame_id) {
  const auto& cfg = config_;
  AGORA_LOG_INFO("Saving Frame %d TX data to %s\n", frame_id,
                 kTxDataFilename.c_str());
  FILE* fp = std::fopen(kTxDataFilename.c_str(), "wb");

  for (size_t i = 0; i < cfg->Frame().NumDLSyms(); i++) {
    size_t total_data_symbol_id = cfg->GetTotalDataSymbolIdxDl(frame_id, i);

    for (size_t ant_id = 0; ant_id < cfg->BsAntNum(); ant_id++) {
      size_t offset = total_data_symbol_id * cfg->BsAntNum() + ant_id;
      auto* pkt = reinterpret_cast<Packet*>(
          &buffer_->GetDlSocketBuffer()[offset * cfg->DlPacketLength()]);
      //dl_socket_buffer_[offset * cfg->DlPacketLength()]);
      short* socket_ptr = pkt->data_;
      std::fwrite(socket_ptr, cfg->SampsPerSymbol() * 2, sizeof(short), fp);
    }
  }
  std::fclose(fp);
}

void Agora::GetEqualData(float** ptr, int* size) {
  const auto& cfg = config_;
  auto offset = cfg->GetTotalDataSymbolIdxUl(
      max_equaled_frame_, cfg->Frame().ClientUlPilotSymbols());
  *ptr = (float*)&buffer_->GetEqualBuffer(offset)[0];
  *size = cfg->UeAntNum() * cfg->OfdmDataNum() * 2;
}
void Agora::CheckIncrementScheduleFrame(size_t frame_id,
                                        ScheduleProcessingFlags completed) {
  this->schedule_process_flags_ += completed;
  assert(this->frame_.cur_sche_frame_id_ == frame_id);
  unused(frame_id);

  if (this->schedule_process_flags_ ==
      static_cast<uint8_t>(ScheduleProcessingFlags::kProcessingComplete)) {
    this->frame_.cur_sche_frame_id_++;
    this->schedule_process_flags_ = ScheduleProcessingFlags::kNone;
    if (this->config_->Frame().NumULSyms() == 0) {
      this->schedule_process_flags_ += ScheduleProcessingFlags::kUplinkComplete;
    }
    if (this->config_->Frame().NumDLSyms() == 0) {
      this->schedule_process_flags_ +=
          ScheduleProcessingFlags::kDownlinkComplete;
    }
  }
}

bool Agora::CheckFrameComplete(size_t frame_id) {
  bool finished = false;

  AGORA_LOG_TRACE(
      "Checking work complete %zu, ifft %d, tx %d, decode %d, tomac %d, tx "
      "%d\n",
      frame_id, static_cast<int>(this->ifft_counters_.IsLastSymbol(frame_id)),
      static_cast<int>(this->tx_counters_.IsLastSymbol(frame_id)),
      static_cast<int>(this->decode_counters_.IsLastSymbol(frame_id)),
      static_cast<int>(this->tomac_counters_.IsLastSymbol(frame_id)),
      static_cast<int>(this->tx_counters_.IsLastSymbol(frame_id)));

  // Complete if last frame and ifft / decode complete
  if ((true == this->ifft_counters_.IsLastSymbol(frame_id)) &&
      (true == this->tx_counters_.IsLastSymbol(frame_id)) &&
      (((false == kEnableMac) &&
        (true == this->decode_counters_.IsLastSymbol(frame_id))) ||
       ((true == kUplinkHardDemod) &&
        (true == this->demul_counters_.IsLastSymbol(frame_id))) ||
       ((true == kEnableMac) &&
        (true == this->tomac_counters_.IsLastSymbol(frame_id))))) {
    this->stats_->UpdateStats(frame_id);
    assert(frame_id == this->frame_.cur_proc_frame_id_);
    if (true == kUplinkHardDemod) {
      this->demul_counters_.Reset(frame_id);
    }
    this->decode_counters_.Reset(frame_id);
    this->tomac_counters_.Reset(frame_id);
    this->ifft_counters_.Reset(frame_id);
    this->tx_counters_.Reset(frame_id);
    if (config_->Frame().NumDLSyms() > 0) {
      for (size_t ue_id = 0; ue_id < config_->UeAntNum(); ue_id++) {
        this->buffer_->SetDlBitsBufferStatus(0, ue_id, frame_id % kFrameWnd);
      }
    }
    this->frame_.cur_proc_frame_id_++;

    if (frame_id == (this->config_->FramesToTest() - 1)) {
      finished = true;
    } else {
      // Only schedule up to kScheduleQueues so we don't flood the queues
      // Cannot access the front() element if the queue is empty
      for (size_t encode = 0;
           (encode < kScheduleQueues) && (!encode_deferral_.empty());
           encode++) {
        const size_t deferred_frame = this->encode_deferral_.front();
        if (deferred_frame <
            (this->frame_.cur_proc_frame_id_ + kScheduleQueues)) {
          if (kDebugDeferral) {
            AGORA_LOG_INFO("   +++ Scheduling deferred frame %zu : %zu \n",
                           deferred_frame, frame_.cur_proc_frame_id_);
          }
          RtAssert(deferred_frame >= this->frame_.cur_proc_frame_id_,
                   "Error scheduling encoding because deferral frame is less "
                   "than current frame");
          ScheduleDownlinkProcessing(deferred_frame);
          this->encode_deferral_.pop();
        } else {
          // No need to check the next frame because it is too large
          break;
        }
      }  // for each encodable frames in kScheduleQueues
    }    // !finished
  }
  return finished;
}

extern "C" {
EXPORT Agora* AgoraNew(Config* cfg) {
  AGORA_LOG_TRACE("Size of Agora: %zu\n", sizeof(Agora*));
  auto* agora = new Agora(cfg);

  return agora;
}
EXPORT void AgoraStart(Agora* agora) { agora->Start(); }
EXPORT void AgoraStop(/*Agora *agora*/) {
  SignalHandler::SetExitSignal(true); /*agora->stop();*/
}
EXPORT void AgoraDestroy(Agora* agora) { delete agora; }
EXPORT void AgoraGetEqualData(Agora* agora, float** ptr, int* size) {
  return agora->GetEqualData(ptr, size);
}
}
