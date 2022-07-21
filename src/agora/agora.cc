/**
 * @file agora.cc
 * @brief Implementation file for the main agora class
 */

#include "agora.h"

#include <cmath>
#include <memory>

#include "agora_utils.h"

#if defined(USE_DPDK)
#include "packet_txrx_dpdk.h"
#endif
#include "packet_txrx_radio.h"
#include "packet_txrx_sim.h"

Agora::Agora(Config* const cfg) {
  csi_buffers_.Alloc(kFrameWnd, cfg->UeAntNum(),
                     cfg->BsAntNum() * cfg->OfdmDataNum());
  demod_buffers_.Alloc(kFrameWnd, cfg->Frame().NumULSyms(), cfg->UeAntNum(),
                       kMaxModType * cfg->OfdmDataNum());
  decoded_buffer_.Alloc(
      kFrameWnd, cfg->Frame().NumULSyms(), cfg->UeAntNum(),
      cfg->LdpcConfig(Direction::kUplink).NumBlocksInSymbol() *
          Roundup<64>(cfg->NumBytesPerCb(Direction::kUplink)));
  ul_zf_matrices_.Alloc(kFrameWnd, cfg->OfdmDataNum(),
                        cfg->BsAntNum() * cfg->UeAntNum());
  dl_zf_matrices_.Alloc(kFrameWnd, cfg->OfdmDataNum(),
                        cfg->UeAntNum() * cfg->BsAntNum());
  config = cfg;
  base_worker_core_offset_ = cfg->CoreOffset() + 1 + cfg->SocketThreadNum();
  stats_ = std::make_unique<Stats>(cfg);
  phy_stats_ = std::make_unique<PhyStats>(cfg, Direction::kUplink);

  std::string directory = TOSTRING(PROJECT_DIRECTORY);
  AGORA_LOG_INFO("Agora: project directory [%s], RDTSC frequency = %.2f GHz\n",
                 directory.c_str(), config->FreqGhz());

  PinToCoreWithOffset(ThreadType::kMaster, config->CoreOffset(), 0,
                      kEnableCoreReuse, false /* quiet */);
  CheckIncrementScheduleFrame(0, ScheduleProcessingFlags::kProcessingComplete);
  // Important to set cur_sche_frame_id_ after the call to
  // CheckIncrementScheduleFrame because it will be incremented however,
  // CheckIncrementScheduleFrame will initialize the schedule tracking variable
  // correctly.
  cur_sche_frame_id_ = 0;
  cur_proc_frame_id = 0;

  InitializeQueues();
  InitializeUplinkBuffers();
  InitializeDownlinkBuffers();

  /* Initialize TXRX threads */
  if (kUseArgos || kUseUHD) {
    packet_tx_rx_ = std::make_unique<PacketTxRxRadio>(
        config, config->CoreOffset() + 1, &message_queue,
        GetConq(EventType::kPacketTX, 0), rx_ptoks_ptr, tx_ptoks_ptr,
        socket_buffer_, socket_buffer_size_ / config->PacketLength(),
        stats_->FrameStart(), dl_socket_buffer_);
#if defined(USE_DPDK)
  } else if (kUseDPDK) {
    packet_tx_rx_ = std::make_unique<PacketTxRxDpdk>(
        config, config->CoreOffset() + 1, &message_queue,
        GetConq(EventType::kPacketTX, 0), rx_ptoks_ptr, tx_ptoks_ptr,
        socket_buffer_, socket_buffer_size_ / config->PacketLength(),
        stats_->FrameStart(), dl_socket_buffer_);
#endif
  } else {
    /* Default to the simulator */
    packet_tx_rx_ = std::make_unique<PacketTxRxSim>(
        config, config->CoreOffset() + 1, &message_queue,
        GetConq(EventType::kPacketTX, 0), rx_ptoks_ptr, tx_ptoks_ptr,
        socket_buffer_, socket_buffer_size_ / config->PacketLength(),
        stats_->FrameStart(), dl_socket_buffer_);
  }

  if (kEnableMac == true) {
    const size_t mac_cpu_core = config->CoreOffset() +
                                config->SocketThreadNum() +
                                config->WorkerThreadNum() + 1;
    mac_thread_ = std::make_unique<MacThreadBaseStation>(
        config, mac_cpu_core, decoded_buffer_, &dl_bits_buffer_,
        &dl_bits_buffer_status_, &mac_request_queue, &mac_response_queue);

    mac_std_thread_ =
        std::thread(&MacThreadBaseStation::RunEventLoop, mac_thread_.get());
  }

  if (kEnableMatLog) {
    for (size_t i = 0; i < mat_loggers_.size(); i++) {
      if ((i != CsvLog::kMatDLZF) || (config->Frame().NumDLSyms() > 0)) {
        mat_loggers_.at(i) =
            std::make_shared<CsvLog::MatLogger>(config->RadioId().at(0), i);
      }
    }
  }

  // Create worker threads
  CreateThreads();

  AGORA_LOG_INFO(
      "Master thread core %zu, TX/RX thread cores %zu--%zu, worker thread "
      "cores %zu--%zu\n",
      config->CoreOffset(), config->CoreOffset() + 1,
      config->CoreOffset() + 1 + config->SocketThreadNum() - 1,
      base_worker_core_offset_,
      base_worker_core_offset_ + config->WorkerThreadNum() - 1);
}

Agora::~Agora() {
  if (kEnableMac == true) {
    mac_std_thread_.join();
  }

  for (auto& worker_thread : workers_) {
    AGORA_LOG_SYMBOL("Agora: Joining worker thread\n");
    worker_thread.join();
  }
  FreeUplinkBuffers();
  FreeDownlinkBuffers();
  stats_.reset();
  phy_stats_.reset();
  FreeQueues();
}

void Agora::Stop() {
  AGORA_LOG_INFO("Agora: terminating\n");
  config->Running(false);
  usleep(1000);
  packet_tx_rx_.reset();
}

void Agora::Start() {
  const bool start_status =
      packet_tx_rx_->StartTxRx(calib_dl_buffer_, calib_ul_buffer_);
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
      std::max(kDequeueBulkSizeTXRX * (config->SocketThreadNum() + 1 /* MAC */),
               kDequeueBulkSizeWorker * config->WorkerThreadNum());
  EventData events_list[max_events_needed];

  while ((config->Running() == true) &&
         (SignalHandler::GotExitSignal() == false)) {
    // Get a batch of events
    size_t num_events = fetch_event(events_list, is_turn_to_dequeue_from_io);
    is_turn_to_dequeue_from_io = !is_turn_to_dequeue_from_io;

    // Handle each event
    for (size_t ev_i = 0; ev_i < num_events; ev_i++) {
      EventData& event = events_list[ev_i];

      // FFT processing is scheduled after falling through the switch
      switch (event.event_type_) {
        case EventType::kPacketRX: {
          Packet* pkt = rx_tag_t(event.tags_[0]).rx_packet_->RawPacket();

          if (pkt->frame_id_ >= ((cur_sche_frame_id_ + kFrameWnd))) {
            AGORA_LOG_ERROR(
                "Error: Received packet for future frame %u beyond "
                "frame window (= %zu + %zu). This can happen if "
                "Agora is running slowly, e.g., in debug mode\n",
                pkt->frame_id_, cur_sche_frame_id_, kFrameWnd);
            config->Running(false);
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

        case EventType::kZF: {
          for (size_t tag_id = 0; (tag_id < event.num_tags_); tag_id++) {
            const size_t frame_id = gen_tag_t(event.tags_[tag_id]).frame_id_;
            PrintPerTaskDone(PrintType::kZF, frame_id, 0,
                             zf_counters_.GetTaskCount(frame_id));
            const bool last_zf_task = zf_counters_.CompleteTask(frame_id);
            if (last_zf_task == true) {
              stats_->MasterSetTsc(TsType::kZFDone, frame_id);
              zf_last_frame_ = frame_id;
              PrintPerFrameDone(PrintType::kZF, frame_id);
              zf_counters_.Reset(frame_id);
              if (kPrintZfStats) {
                phy_stats_->PrintZfStats(frame_id);
              }

              for (size_t i = 0; i < config->Frame().NumULSyms(); i++) {
                if (fft_cur_frame_for_symbol_.at(i) == frame_id) {
                  ScheduleSubcarriers(EventType::kDemul, frame_id,
                                      config->Frame().GetULSymbol(i));
                }
              }
              // Schedule precoding for downlink symbols
              for (size_t i = 0; i < config->Frame().NumDLSyms(); i++) {
                const size_t last_encoded_frame =
                    encode_cur_frame_for_symbol_.at(i);
                if ((last_encoded_frame != SIZE_MAX) &&
                    (last_encoded_frame >= frame_id)) {
                  ScheduleSubcarriers(EventType::kPrecode, frame_id,
                                      config->Frame().GetDLSymbol(i));
                }
              }
            }  // end if (zf_counters_.last_task(frame_id) == true)
          }
        } break;

        case EventType::kDemul: {
          const size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          const size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;
          const size_t base_sc_id = gen_tag_t(event.tags_[0]).sc_id_;

          PrintPerTaskDone(PrintType::kDemul, frame_id, symbol_id, base_sc_id);
          const bool last_demul_task =
              demul_counters_.CompleteTask(frame_id, symbol_id);

          if (last_demul_task == true) {
            if (kUplinkHardDemod == false) {
              ScheduleCodeblocks(EventType::kDecode, Direction::kUplink,
                                 frame_id, symbol_id);
            }
            PrintPerSymbolDone(PrintType::kDemul, frame_id, symbol_id);
            const bool last_demul_symbol =
                demul_counters_.CompleteSymbol(frame_id);
            if (last_demul_symbol == true) {
              max_equaled_frame_ = frame_id;
              stats_->MasterSetTsc(TsType::kDemulDone, frame_id);
              PrintPerFrameDone(PrintType::kDemul, frame_id);
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
                if (config->BigstationMode() == false) {
                  assert(cur_sche_frame_id_ == frame_id);
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
              decode_counters_.CompleteTask(frame_id, symbol_id);
          if (last_decode_task == true) {
            if (kEnableMac == true) {
              ScheduleUsers(EventType::kPacketToMac, frame_id, symbol_id);
            }
            PrintPerSymbolDone(PrintType::kDecode, frame_id, symbol_id);
            const bool last_decode_symbol =
                decode_counters_.CompleteSymbol(frame_id);
            if (last_decode_symbol == true) {
              stats_->MasterSetTsc(TsType::kDecodeDone, frame_id);
              PrintPerFrameDone(PrintType::kDecode, frame_id);
              if (kEnableMac == false) {
                assert(cur_proc_frame_id == frame_id);
                const bool work_finished = CheckFrameComplete(frame_id);
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
              tomac_counters_.CompleteTask(frame_id, symbol_id);
          if (last_tomac_task == true) {
            PrintPerSymbolDone(PrintType::kPacketToMac, frame_id, symbol_id);

            const bool last_tomac_symbol =
                tomac_counters_.CompleteSymbol(frame_id);
            if (last_tomac_symbol == true) {
              assert(cur_proc_frame_id == frame_id);
              // stats_->MasterSetTsc(TsType::kMacTXDone, frame_id);
              PrintPerFrameDone(PrintType::kPacketToMac, frame_id);
              const bool work_finished = CheckFrameComplete(frame_id);
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
              &dl_bits_buffer_[ue_id]
                              [radio_buf_id * config->MacBytesNumPerframe(
                                                  Direction::kDownlink)]);

          AGORA_LOG_INFO("Agora: frame %d @ offset %zu %zu @ location %zu\n",
                         pkt->Frame(), ue_id, radio_buf_id,
                         reinterpret_cast<intptr_t>(pkt));

          if (kDebugPrintPacketsFromMac) {
            std::stringstream ss;

            for (size_t dl_data_symbol = 0;
                 dl_data_symbol < config->Frame().NumDlDataSyms();
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
                  config->MacPacketLength(Direction::kDownlink));
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
                AGORA_LOG_INFO("   +++ Deferring encoding of frame %zu\n",
                               frame_id);
              }
              encode_deferral_.push(frame_id);
            } else {
              ScheduleDownlinkProcessing(frame_id);
            }
            mac_to_phy_counters_.Reset(frame_id);
            PrintPerFrameDone(PrintType::kPacketFromMac, frame_id);
          }
        } break;

        case EventType::kEncode: {
          for (size_t i = 0u; i < event.num_tags_; i++) {
            const size_t frame_id = gen_tag_t(event.tags_[i]).frame_id_;
            const size_t symbol_id = gen_tag_t(event.tags_[i]).symbol_id_;

            const bool last_encode_task =
                encode_counters_.CompleteTask(frame_id, symbol_id);
            if (last_encode_task == true) {
              encode_cur_frame_for_symbol_.at(
                  config->Frame().GetDLSymbolIdx(symbol_id)) = frame_id;
              // If precoder of the current frame exists
              if (zf_last_frame_ == frame_id) {
                ScheduleSubcarriers(EventType::kPrecode, frame_id, symbol_id);
              }
              PrintPerSymbolDone(PrintType::kEncode, frame_id, symbol_id);

              const bool last_encode_symbol =
                  encode_counters_.CompleteSymbol(frame_id);
              if (last_encode_symbol == true) {
                encode_counters_.Reset(frame_id);
                stats_->MasterSetTsc(TsType::kEncodeDone, frame_id);
                PrintPerFrameDone(PrintType::kEncode, frame_id);
              }
            }
          }
        } break;

        case EventType::kPrecode: {
          // Precoding is done, schedule ifft
          const size_t sc_id = gen_tag_t(event.tags_[0]).sc_id_;
          const size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          const size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;
          PrintPerTaskDone(PrintType::kPrecode, frame_id, symbol_id, sc_id);
          const bool last_precode_task =
              precode_counters_.CompleteTask(frame_id, symbol_id);

          if (last_precode_task == true) {
            // precode_cur_frame_for_symbol_.at(
            //    config->Frame().GetDLSymbolIdx(symbol_id)) = frame_id;
            ScheduleAntennas(EventType::kIFFT, frame_id, symbol_id);
            PrintPerSymbolDone(PrintType::kPrecode, frame_id, symbol_id);

            const bool last_precode_symbol =
                precode_counters_.CompleteSymbol(frame_id);
            if (last_precode_symbol == true) {
              precode_counters_.Reset(frame_id);
              stats_->MasterSetTsc(TsType::kPrecodeDone, frame_id);
              PrintPerFrameDone(PrintType::kPrecode, frame_id);
            }
          }
        } break;

        case EventType::kIFFT: {
          for (size_t i = 0; i < event.num_tags_; i++) {
            /* IFFT is done, schedule data transmission */
            const size_t ant_id = gen_tag_t(event.tags_[i]).ant_id_;
            const size_t frame_id = gen_tag_t(event.tags_[i]).frame_id_;
            const size_t symbol_id = gen_tag_t(event.tags_[i]).symbol_id_;
            const size_t symbol_idx_dl =
                config->Frame().GetDLSymbolIdx(symbol_id);
            PrintPerTaskDone(PrintType::kIFFT, frame_id, symbol_id, ant_id);

            const bool last_ifft_task =
                ifft_counters_.CompleteTask(frame_id, symbol_id);
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
                                       config->Frame().GetDLSymbol(sym_id));
                    ifft_next_symbol_++;
                  } else {
                    break;
                  }
                }
              }
              PrintPerSymbolDone(PrintType::kIFFT, frame_id, symbol_id);

              const bool last_ifft_symbol =
                  ifft_counters_.CompleteSymbol(frame_id);
              if (last_ifft_symbol == true) {
                ifft_next_symbol_ = 0;
                stats_->MasterSetTsc(TsType::kIFFTDone, frame_id);
                PrintPerFrameDone(PrintType::kIFFT, frame_id);
                assert(frame_id == cur_proc_frame_id);
                CheckIncrementScheduleFrame(frame_id, kDownlinkComplete);
                const bool work_finished = CheckFrameComplete(frame_id);
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
          PrintPerTaskDone(PrintType::kPacketTX, frame_id, symbol_id, ant_id);

          const bool last_tx_task =
              tx_counters_.CompleteTask(frame_id, symbol_id);
          if (last_tx_task == true) {
            PrintPerSymbolDone(PrintType::kPacketTX, frame_id, symbol_id);
            // If tx of the first symbol is done
            if (symbol_id == config->Frame().GetDLSymbol(0)) {
              stats_->MasterSetTsc(TsType::kTXProcessedFirst, frame_id);
              PrintPerFrameDone(PrintType::kPacketTXFirst, frame_id);
            }

            const bool last_tx_symbol = tx_counters_.CompleteSymbol(frame_id);
            if (last_tx_symbol == true) {
              stats_->MasterSetTsc(TsType::kTXDone, frame_id);
              PrintPerFrameDone(PrintType::kPacketTX, frame_id);

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
                  config->OfdmDataNum() * tx_counters_.MaxSymbolCount() * 1000;

              AGORA_LOG_INFO(
                  "TX %d samples (per-client) to %zu clients in %f secs, "
                  "throughtput %f bps per-client (16QAM), current tx queue "
                  "length %zu\n",
                  samples_num_per_ue, config->UeAntNum(), diff,
                  samples_num_per_ue * std::log2(16.0f) / diff,
                  GetConq(EventType::kPacketTX, 0)->size_approx());
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
          fft_queue_arr_.at(cur_sche_frame_id_ % kFrameWnd);
      const size_t qid = cur_sche_frame_id_ & 0x1;
      if (cur_fftq.size() >= config->FftBlockSize()) {
        const size_t num_fft_blocks = cur_fftq.size() / config->FftBlockSize();
        for (size_t i = 0; i < num_fft_blocks; i++) {
          EventData do_fft_task;
          do_fft_task.num_tags_ = config->FftBlockSize();
          do_fft_task.event_type_ = EventType::kFFT;

          for (size_t j = 0; j < config->FftBlockSize(); j++) {
            RtAssert(!cur_fftq.empty(),
                     "Using front element cur_fftq when it is empty");
            do_fft_task.tags_[j] = cur_fftq.front().tag_;
            cur_fftq.pop();

            if (fft_created_count_ == 0) {
              stats_->MasterSetTsc(TsType::kProcessingStarted,
                                   cur_sche_frame_id_);
            }
            fft_created_count_++;
            if (fft_created_count_ == rx_counters_.num_rx_pkts_per_frame_) {
              fft_created_count_ = 0;
              if (config->BigstationMode() == true) {
                CheckIncrementScheduleFrame(cur_sche_frame_id_,
                                            kUplinkComplete);
              }
            }
          }
          TryEnqueueFallback(GetConq(EventType::kFFT, qid),
                             GetPtok(EventType::kFFT, qid), do_fft_task);
        }
      }
    } /* End of for */
  }   /* End of while */

finish:
  AGORA_LOG_INFO("Agora: printing stats and saving to file\n");
  stats_->PrintSummary();
  stats_->SaveToFile();
  if (flags_.enable_save_decode_data_to_file_ == true) {
    SaveDecodeDataToFile(stats_->LastFrameId());
  }
  if (flags_.enable_save_tx_data_to_file_ == true) {
    SaveTxDataToFile(stats_->LastFrameId());
  }

  // Calculate and print per-user BER
  if ((kEnableMac == false) && (kPrintPhyStats == true)) {
    phy_stats_->PrintPhyStats();
  }
  this->Stop();
}

void Agora::GetEqualData(float** ptr, int* size) {
  auto offset = config->GetTotalDataSymbolIdxUl(
      max_equaled_frame_, config->Frame().ClientUlPilotSymbols());
  *ptr = (float*)&equal_buffer_[offset][0];
  *size = config->UeAntNum() * config->OfdmDataNum() * 2;
}

void Agora::Worker(int tid) {
  PinToCoreWithOffset(ThreadType::kWorker, base_worker_core_offset_, tid);

  /* Initialize operators */
  auto compute_zf = std::make_unique<DoZF>(
      config, tid, csi_buffers_, calib_dl_buffer_, calib_ul_buffer_,
      calib_dl_msum_buffer_, calib_ul_msum_buffer_, ul_zf_matrices_,
      dl_zf_matrices_, phy_stats_.get(), stats_.get(),
      mat_loggers_.at(CsvLog::kMatCSI), mat_loggers_.at(CsvLog::kMatDLZF));

  auto compute_fft = std::make_unique<DoFFT>(
      config, tid, data_buffer_, csi_buffers_, calib_dl_buffer_,
      calib_ul_buffer_, phy_stats_.get(), stats_.get());

  // Downlink workers
  auto compute_ifft = std::make_unique<DoIFFT>(config, tid, dl_ifft_buffer_,
                                               dl_socket_buffer_, stats_.get());

  auto compute_precode =
      std::make_unique<DoPrecode>(config, tid, dl_zf_matrices_, dl_ifft_buffer_,
                                  dl_mod_bits_buffer_, stats_.get());

  auto compute_encoding = std::make_unique<DoEncode>(
      config, tid, Direction::kDownlink,
      (kEnableMac == true) ? dl_bits_buffer_ : config->DlBits(),
      (kEnableMac == true) ? kFrameWnd : 1, dl_mod_bits_buffer_, stats_.get());

  // Uplink workers
  auto compute_decoding =
      std::make_unique<DoDecode>(config, tid, demod_buffers_, decoded_buffer_,
                                 phy_stats_.get(), stats_.get());

  auto compute_demul = std::make_unique<DoDemul>(
      config, tid, data_buffer_, ul_zf_matrices_, ue_spec_pilot_buffer_,
      equal_buffer_, demod_buffers_, phy_stats_.get(), stats_.get());

  std::vector<Doer*> computers_vec;
  std::vector<EventType> events_vec;
  ///*************************
  computers_vec.push_back(compute_zf.get());
  computers_vec.push_back(compute_fft.get());
  events_vec.push_back(EventType::kZF);
  events_vec.push_back(EventType::kFFT);

  if (config->Frame().NumULSyms() > 0) {
    computers_vec.push_back(compute_decoding.get());
    computers_vec.push_back(compute_demul.get());
    events_vec.push_back(EventType::kDecode);
    events_vec.push_back(EventType::kDemul);
  }

  if (config->Frame().NumDLSyms() > 0) {
    computers_vec.push_back(compute_ifft.get());
    computers_vec.push_back(compute_precode.get());
    computers_vec.push_back(compute_encoding.get());
    events_vec.push_back(EventType::kIFFT);
    events_vec.push_back(EventType::kPrecode);
    events_vec.push_back(EventType::kEncode);
  }

  size_t cur_qid = 0;
  size_t empty_queue_itrs = 0;
  bool empty_queue = true;
  while (config->Running() == true) {
    for (size_t i = 0; i < computers_vec.size(); i++) {
      if (computers_vec.at(i)->TryLaunch(*GetConq(events_vec.at(i), cur_qid),
                                         complete_task_queue[cur_qid],
                                         worker_ptoks_ptr[tid][cur_qid])) {
        empty_queue = false;
        break;
      }
    }
    // If all queues in this set are empty for 5 iterations,
    // check the other set of queues
    if (empty_queue == true) {
      empty_queue_itrs++;
      if (empty_queue_itrs == 5) {
        if (cur_sche_frame_id_ != cur_proc_frame_id) {
          cur_qid ^= 0x1;
        } else {
          cur_qid = (cur_sche_frame_id_ & 0x1);
        }
        empty_queue_itrs = 0;
      }
    } else {
      empty_queue = true;
    }
  }
  AGORA_LOG_SYMBOL("Agora worker %d exit\n", tid);
}

void Agora::WorkerFft(int tid) {
  PinToCoreWithOffset(ThreadType::kWorkerFFT, base_worker_core_offset_, tid);

  /* Initialize FFT operator */
  std::unique_ptr<DoFFT> compute_fft(
      new DoFFT(config, tid, data_buffer_, csi_buffers_, calib_dl_buffer_,
                calib_ul_buffer_, phy_stats_.get(), stats_.get()));
  std::unique_ptr<DoIFFT> compute_ifft(new DoIFFT(
      config, tid, dl_ifft_buffer_, dl_socket_buffer_, stats_.get()));

  while (config->Running() == true) {
    // TODO refactor the if / else
    if (compute_fft->TryLaunch(*GetConq(EventType::kFFT, 0),
                               complete_task_queue[0],
                               worker_ptoks_ptr[tid][0]) == true) {
      // Do nothing
    } else if ((config->Frame().NumDLSyms() > 0) &&
               (compute_ifft->TryLaunch(*GetConq(EventType::kIFFT, 0),
                                        complete_task_queue[0],
                                        worker_ptoks_ptr[tid][0]) == true)) {
      // Do nothing
    }
  }
}

void Agora::WorkerZf(int tid) {
  PinToCoreWithOffset(ThreadType::kWorkerZF, base_worker_core_offset_, tid);

  /* Initialize ZF operator */
  auto compute_zf(std::make_unique<DoZF>(
      config, tid, csi_buffers_, calib_dl_buffer_, calib_ul_buffer_,
      calib_dl_msum_buffer_, calib_ul_msum_buffer_, ul_zf_matrices_,
      dl_zf_matrices_, phy_stats_.get(), stats_.get(),
      mat_loggers_.at(CsvLog::kMatCSI), mat_loggers_.at(CsvLog::kMatDLZF)));

  while (config->Running() == true) {
    compute_zf->TryLaunch(*GetConq(EventType::kZF, 0), complete_task_queue[0],
                          worker_ptoks_ptr[tid][0]);
  }
}

void Agora::WorkerDemul(int tid) {
  PinToCoreWithOffset(ThreadType::kWorkerDemul, base_worker_core_offset_, tid);

  std::unique_ptr<DoDemul> compute_demul(new DoDemul(
      config, tid, data_buffer_, ul_zf_matrices_, ue_spec_pilot_buffer_,
      equal_buffer_, demod_buffers_, phy_stats_.get(), stats_.get()));

  /* Initialize Precode operator */
  std::unique_ptr<DoPrecode> compute_precode(
      new DoPrecode(config, tid, dl_zf_matrices_, dl_ifft_buffer_,
                    dl_mod_bits_buffer_, stats_.get()));

  assert(false);

  while (config->Running() == true) {
    if (config->Frame().NumDLSyms() > 0) {
      compute_precode->TryLaunch(*GetConq(EventType::kDemul, 0),
                                 complete_task_queue[0],
                                 worker_ptoks_ptr[tid][0]);
    } else {
      compute_demul->TryLaunch(*GetConq(EventType::kPrecode, 0),
                               complete_task_queue[0],
                               worker_ptoks_ptr[tid][0]);
    }
  }
}

void Agora::WorkerDecode(int tid) {
  PinToCoreWithOffset(ThreadType::kWorkerDecode, base_worker_core_offset_, tid);

  std::unique_ptr<DoEncode> compute_encoding(new DoEncode(
      config, tid, Direction::kDownlink,
      (kEnableMac == true) ? dl_bits_buffer_ : config->DlBits(),
      (kEnableMac == true) ? kFrameWnd : 1, dl_mod_bits_buffer_, stats_.get()));

  std::unique_ptr<DoDecode> compute_decoding(
      new DoDecode(config, tid, demod_buffers_, decoded_buffer_,
                   phy_stats_.get(), stats_.get()));

  while (config->Running() == true) {
    if (config->Frame().NumDLSyms() > 0) {
      compute_encoding->TryLaunch(*GetConq(EventType::kEncode, 0),
                                  complete_task_queue[0],
                                  worker_ptoks_ptr[tid][0]);
    } else {
      compute_decoding->TryLaunch(*GetConq(EventType::kDecode, 0),
                                  complete_task_queue[0],
                                  worker_ptoks_ptr[tid][0]);
    }
  }
}

void Agora::CreateThreads() {
  if (config->BigstationMode() == true) {
    for (size_t i = 0; i < config->FftThreadNum(); i++) {
      workers_.emplace_back(&Agora::WorkerFft, this, i);
    }
    for (size_t i = config->FftThreadNum();
         i < config->FftThreadNum() + config->ZfThreadNum(); i++) {
      workers_.emplace_back(&Agora::WorkerZf, this, i);
    }
    for (size_t i = config->FftThreadNum() + config->ZfThreadNum();
         i < config->FftThreadNum() + config->ZfThreadNum() +
                 config->DemulThreadNum();
         i++) {
      workers_.emplace_back(&Agora::WorkerDemul, this, i);
    }
    for (size_t i = config->FftThreadNum() + config->ZfThreadNum() +
                    config->DemulThreadNum();
         i < config->WorkerThreadNum(); i++) {
      workers_.emplace_back(&Agora::WorkerDecode, this, i);
    }
  } else {
    AGORA_LOG_SYMBOL("Agora: creating %zu workers\n",
                     config->WorkerThreadNum());
    for (size_t i = 0; i < config->WorkerThreadNum(); i++) {
      workers_.emplace_back(&Agora::Worker, this, i);
    }
  }
}

extern "C" {
EXPORT Agora* AgoraNew(Config* config) {
  AGORA_LOG_TRACE("Size of Agora: %zu\n", sizeof(Agora*));
  auto* agora = new Agora(config);

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