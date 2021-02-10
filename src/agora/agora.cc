/**
 * @file agora.cc
 * @brief Implementation file for the main agora class
 */

#include "agora.h"

Agora::Agora(Config* cfg)
    : base_worker_core_offset_(cfg->CoreOffset() + 1 + cfg->SocketThreadNum()),
      csi_buffers_(kFrameWnd, cfg->UeNum(),
                   cfg->BsAntNum() * cfg->OfdmDataNum()),
      ul_zf_matrices_(kFrameWnd, cfg->OfdmDataNum(),
                      cfg->BsAntNum() * cfg->UeNum()),
      demod_buffers_(kFrameWnd, cfg->Frame().NumTotalSyms(), cfg->UeNum(),
                     kMaxModType * cfg->OfdmDataNum()),
      decoded_buffer_(kFrameWnd, cfg->Frame().NumTotalSyms(), cfg->UeNum(),
                      cfg->LdpcConfig().NumBlocksInSymbol() *
                          Roundup<64>(cfg->NumBytesPerCb())),
      dl_zf_matrices_(kFrameWnd, cfg->OfdmDataNum(),
                      cfg->UeNum() * cfg->BsAntNum()) {
  std::string directory = TOSTRING(PROJECT_DIRECTORY);
  std::printf("Agora: project directory [%s], RDTSC frequency = %.2f GHz\n",
              directory.c_str(), cfg->FreqGhz());

  this->config_ = cfg;

  PinToCoreWithOffset(ThreadType::kMaster, cfg->CoreOffset(), 0,
                      false /* quiet */);
  InitializeQueues();
  InitializeUplinkBuffers();

  if (config_->Frame().NumDLSyms() > 0) {
    std::printf("Agora: Initializing downlink buffers\n");
    InitializeDownlinkBuffers();
  }

  stats_ = new Stats(cfg);
  phy_stats_ = new PhyStats(cfg);

  /* Initialize TXRX threads */
  packet_tx_rx_.reset(new PacketTXRX(
      cfg, cfg->CoreOffset() + 1, &message_queue_,
      GetConq(EventType::kPacketTX, 0), rx_ptoks_ptr_, tx_ptoks_ptr_));

  if (kEnableMac) {
    const size_t mac_cpu_core =
        cfg->CoreOffset() + cfg->SocketThreadNum() + cfg->WorkerThreadNum() + 1;
    mac_thread_ = new MacThread(
        MacThread::Mode::kServer, cfg, mac_cpu_core, decoded_buffer_,
        nullptr /* ul bits */, nullptr /* ul bits status */, &dl_bits_buffer_,
        &dl_bits_buffer_status_, &mac_request_queue_, &mac_response_queue_);

    mac_std_thread_ = std::thread(&MacThread::RunEventLoop, mac_thread_);
  }

  /* Create worker threads */
  CreateThreads();

  std::printf(
      "Master thread core %zu, TX/RX thread cores %zu--%zu, worker thread "
      "cores %zu--%zu\n",
      cfg->CoreOffset(), cfg->CoreOffset() + 1,
      cfg->CoreOffset() + 1 + cfg->SocketThreadNum() - 1,
      base_worker_core_offset_,
      base_worker_core_offset_ + cfg->WorkerThreadNum() - 1);
}

Agora::~Agora() {
  FreeUplinkBuffers();
  /* Downlink */
  if (config_->Frame().NumDLSyms() > 0) {
    FreeDownlinkBuffers();
  }

  if (kEnableMac) {
    mac_std_thread_.join();
  }
  for (size_t i = 0; i < config_->WorkerThreadNum(); i++) {
    worker_std_threads_[i].join();
  }
  delete mac_thread_;
}

void Agora::Stop() {
  std::cout << "Agora: stopping threads" << std::endl;
  config_->Running(false);
  usleep(1000);
  packet_tx_rx_.reset();
}

void Agora::SendSnrReport(EventType event_type, size_t frame_id,
                          size_t symbol_id) {
  assert(event_type == EventType::kSNRReport);
  auto base_tag = gen_tag_t::FrmSymUe(frame_id, symbol_id, 0);
  for (size_t i = 0; i < config_->UeNum(); i++) {
    EventData snr_report(EventType::kSNRReport, base_tag.tag_);
    snr_report.num_tags_ = 2;
    float snr = phy_stats_->GetEvmSnr(frame_id, i);
    std::memcpy(&snr_report.tags_[1], &snr, sizeof(float));
    TryEnqueueFallback(&mac_request_queue_, snr_report);
    base_tag.ue_id_++;
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
    TryEnqueueFallback(GetConq(event_type, qid), GetPtok(event_type, qid),
                       event);
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
    case EventType::kZF:
      num_events = config_->ZfEventsPerSymbol();
      block_size = config_->ZfBlockSize();
      break;
    default:
      assert(false);
  }

  size_t qid = frame_id & 0x1;
  if (event_type == EventType::kZF) {
    EventData event;
    event.event_type_ = event_type;
    event.num_tags_ = config_->ZfBatchSize();
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
      TryEnqueueFallback(GetConq(event_type, qid), GetPtok(event_type, qid),
                         event);
    }
  } else {
    for (size_t i = 0; i < num_events; i++) {
      TryEnqueueFallback(GetConq(event_type, qid), GetPtok(event_type, qid),
                         EventData(event_type, base_tag.tag_));
      base_tag.sc_id_ += block_size;
    }
  }
}

void Agora::ScheduleCodeblocks(EventType event_type, size_t frame_id,
                               size_t symbol_idx) {
  auto base_tag = gen_tag_t::FrmSymCb(frame_id, symbol_idx, 0);

  // for (size_t i = 0;
  //      i < config_->UE_NUM * config_->LDPC_config.nblocksInSymbol; i++) {
  //     try_enqueue_fallback(get_conq(event_type), get_ptok(event_type),
  //         Event_data(event_type, base_tag._tag));
  //     base_tag.cb_id++;
  // }
  size_t num_tasks =
      config_->UeNum() * config_->LdpcConfig().NumBlocksInSymbol();
  size_t num_blocks = num_tasks / config_->EncodeBlockSize();
  size_t num_remainder = num_tasks % config_->EncodeBlockSize();
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
    TryEnqueueFallback(GetConq(event_type, qid), GetPtok(event_type, qid),
                       event);
  }
}

void Agora::ScheduleUsers(EventType event_type, size_t frame_id,
                          size_t symbol_id) {
  assert(event_type == EventType::kPacketToMac);
  auto base_tag = gen_tag_t::FrmSymUe(frame_id, symbol_id, 0);

  for (size_t i = 0; i < config_->UeNum(); i++) {
    TryEnqueueFallback(&mac_request_queue_,
                       EventData(EventType::kPacketToMac, base_tag.tag_));
    base_tag.ue_id_++;
  }
}

void Agora::Start() {
  auto& cfg = config_;

  // Start packet I/O
  if (!packet_tx_rx_->StartTxRx(socket_buffer_, socket_buffer_status_,
                                socket_buffer_status_size_,
                                stats_->frame_start_, dl_socket_buffer_,
                                calib_dl_buffer_, calib_ul_buffer_)) {
    this->Stop();
    return;
  }

  PinToCoreWithOffset(ThreadType::kMaster, cfg->CoreOffset(), 0,
                      false /* quiet */);

  // Counters for printing summary
  size_t tx_count = 0;
  double tx_begin = GetTimeUs();

  bool is_turn_to_dequeue_from_io = true;
  const size_t max_events_needed =
      std::max(kDequeueBulkSizeTXRX * (cfg->SocketThreadNum() + 1 /* MAC */),
               kDequeueBulkSizeWorker * cfg->WorkerThreadNum());
  EventData events_list[max_events_needed];

  while (config_->Running() && !SignalHandler::GotExitSignal()) {
    // Get a batch of events
    size_t num_events = 0;
    if (is_turn_to_dequeue_from_io) {
      for (size_t i = 0; i < cfg->SocketThreadNum(); i++) {
        num_events += message_queue_.try_dequeue_bulk_from_producer(
            *(rx_ptoks_ptr_[i]), events_list + num_events,
            kDequeueBulkSizeTXRX);
      }
      num_events += mac_response_queue_.try_dequeue_bulk(
          events_list + num_events, kDequeueBulkSizeTXRX);
    } else {
      num_events +=
          complete_task_queue_[cur_proc_frame_id_ & 0x1].try_dequeue_bulk(
              events_list + num_events, max_events_needed);
    }
    is_turn_to_dequeue_from_io = !is_turn_to_dequeue_from_io;

    // Handle each event
    for (size_t ev_i = 0; ev_i < num_events; ev_i++) {
      EventData& event = events_list[ev_i];

      // FFT processing is scheduled after falling through the switch
      switch (event.event_type_) {
        case EventType::kPacketRX: {
          size_t socket_thread_id = rx_tag_t(event.tags_[0]).tid_;
          size_t sock_buf_offset = rx_tag_t(event.tags_[0]).offset_;
          auto* pkt = (Packet*)(socket_buffer_[socket_thread_id] +
                                (sock_buf_offset * cfg->PacketLength()));

          if (pkt->frame_id_ >= cur_sche_frame_id_ + kFrameWnd) {
            std::printf(
                "Error: Received packet for future frame %u beyond "
                "frame window (= %zu + %zu). This can happen if "
                "Agora is running slowly, e.g., in debug mode\n",
                pkt->frame_id_, cur_sche_frame_id_, kFrameWnd);
            cfg->Running(false);
            break;
          }

          UpdateRxCounters(pkt->frame_id_, pkt->symbol_id_);
          fft_queue_arr_[pkt->frame_id_ % kFrameWnd].push(
              fft_req_tag_t(event.tags_[0]));
        } break;

        case EventType::kFFT: {
          for (size_t i = 0; i < event.num_tags_; i++) {
            HandleEventFft(event.tags_[i]);
          }
        } break;

        case EventType::kZF: {
          for (size_t tag_id = 0; tag_id < event.num_tags_; tag_id++) {
            size_t frame_id = gen_tag_t(event.tags_[tag_id]).frame_id_;
            PrintPerTaskDone(PrintType::kZF, frame_id, 0,
                             zf_counters_.GetTaskCount(frame_id));
            if (zf_counters_.LastTask(frame_id)) {
              stats_->MasterSetTsc(TsType::kZFDone, frame_id);
              zf_last_frame_ = frame_id;
              PrintPerFrameDone(PrintType::kZF, frame_id);

              // If all the data in a frame has arrived when ZF is done
              for (size_t i = 0; i < cfg->Frame().NumULSyms(); i++) {
                if (fft_cur_frame_for_symbol_[i] == frame_id) {
                  ScheduleSubcarriers(EventType::kDemul, frame_id, i);
                }
              }
              // Schedule precoding for downlink symbols
              for (size_t i = 0; i < cfg->Frame().NumDLSyms(); i++) {
                if (encode_cur_frame_for_symbol_[i] == frame_id) {
                  ScheduleSubcarriers(EventType::kPrecode, frame_id,
                                      cfg->Frame().GetDLSymbol(i));
                }
              }
            }
          }
        } break;

        case EventType::kDemul: {
          size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          size_t symbol_idx_ul = gen_tag_t(event.tags_[0]).symbol_id_;
          size_t base_sc_id = gen_tag_t(event.tags_[0]).sc_id_;

          PrintPerTaskDone(PrintType::kDemul, frame_id, symbol_idx_ul,
                           base_sc_id);
          if (demul_counters_.LastTask(frame_id, symbol_idx_ul)) {
            ScheduleCodeblocks(EventType::kDecode, frame_id, symbol_idx_ul);
            PrintPerSymbolDone(PrintType::kDemul, frame_id, symbol_idx_ul);
            if (demul_counters_.LastSymbol(frame_id)) {
              max_equaled_frame_ = frame_id;
              if (!cfg->BigstationMode()) {
                assert(cur_sche_frame_id_ == frame_id);
                cur_sche_frame_id_++;
              } else {
                ScheduleCodeblocks(EventType::kDecode, frame_id, symbol_idx_ul);
              }
              stats_->MasterSetTsc(TsType::kDemulDone, frame_id);
              PrintPerFrameDone(PrintType::kDemul, frame_id);
            }
          }
        } break;

        case EventType::kDecode: {
          size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          size_t symbol_idx_ul = gen_tag_t(event.tags_[0]).symbol_id_;

          if (decode_counters_.LastTask(frame_id, symbol_idx_ul)) {
            if (kEnableMac) {
              ScheduleUsers(EventType::kPacketToMac, frame_id, symbol_idx_ul);
            }
            PrintPerSymbolDone(PrintType::kDecode, frame_id, symbol_idx_ul);
            if (decode_counters_.LastSymbol(frame_id)) {
              stats_->MasterSetTsc(TsType::kDecodeDone, frame_id);
              PrintPerFrameDone(PrintType::kDecode, frame_id);
              if (!kEnableMac) {
                assert(cur_proc_frame_id_ == frame_id);
                cur_proc_frame_id_++;
                stats_->UpdateStatsInFunctionsUplink(frame_id);
                if (stats_->last_frame_id_ == cfg->FramesToTest() - 1) {
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
          size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          size_t symbol_idx_ul = gen_tag_t(event.tags_[0]).symbol_id_;
          if (tomac_counters_.LastTask(frame_id, symbol_idx_ul)) {
            PrintPerSymbolDone(PrintType::kPacketToMac, frame_id,
                               symbol_idx_ul);
            if (tomac_counters_.LastSymbol(frame_id)) {
              assert(cur_proc_frame_id_ == frame_id);
              cur_proc_frame_id_++;
              // stats->master_set_tsc(TsType::kMacTXDone, frame_id);
              PrintPerFrameDone(PrintType::kPacketToMac, frame_id);
              stats_->UpdateStatsInFunctionsUplink(frame_id);
              if (stats_->last_frame_id_ == cfg->FramesToTest() - 1) {
                goto finish;
              }
            }
          }

        } break;

        case EventType::kEncode: {
          for (size_t i = 0; i < event.num_tags_; i++) {
            size_t frame_id = gen_tag_t(event.tags_[i]).frame_id_;
            size_t symbol_id = gen_tag_t(event.tags_[i]).symbol_id_;
            size_t symbol_idx_dl = cfg->Frame().GetDLSymbolIdx(symbol_id);
            if (encode_counters_.LastTask(frame_id, symbol_idx_dl)) {
              encode_cur_frame_for_symbol_[symbol_idx_dl] = frame_id;
              // If precoder of the current frame exists
              if (zf_last_frame_ == frame_id) {
                ScheduleSubcarriers(EventType::kPrecode, frame_id, symbol_id);
              }
              PrintPerSymbolDone(PrintType::kEncode, frame_id, symbol_idx_dl);
              if (encode_counters_.LastSymbol(frame_id)) {
                stats_->MasterSetTsc(TsType::kEncodeDone, frame_id);
                PrintPerFrameDone(PrintType::kEncode, frame_id);
              }
            }
          }
        } break;

        case EventType::kPrecode: {
          /* Precoding is done, schedule ifft */
          size_t sc_id = gen_tag_t(event.tags_[0]).sc_id_;
          size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;
          size_t symbol_idx_dl = cfg->Frame().GetDLSymbolIdx(symbol_id);
          PrintPerTaskDone(PrintType::kPrecode, frame_id, symbol_idx_dl, sc_id);
          if (precode_counters_.LastTask(frame_id, symbol_idx_dl)) {
            ScheduleAntennas(EventType::kIFFT, frame_id, symbol_id);
            PrintPerSymbolDone(PrintType::kPrecode, frame_id, symbol_idx_dl);
            if (precode_counters_.LastSymbol(frame_id)) {
              stats_->MasterSetTsc(TsType::kPrecodeDone, frame_id);
              PrintPerFrameDone(PrintType::kPrecode, frame_id);
            }
          }
        } break;

        case EventType::kIFFT: {
          for (size_t i = 0; i < event.num_tags_; i++) {
            /* IFFT is done, schedule data transmission */
            size_t ant_id = gen_tag_t(event.tags_[i]).ant_id_;
            size_t frame_id = gen_tag_t(event.tags_[i]).frame_id_;
            size_t symbol_id = gen_tag_t(event.tags_[i]).symbol_id_;
            size_t symbol_idx_dl = cfg->Frame().GetDLSymbolIdx(symbol_id);
            TryEnqueueFallback(GetConq(EventType::kPacketTX, 0),
                               tx_ptoks_ptr_[ant_id % cfg->SocketThreadNum()],
                               EventData(EventType::kPacketTX, event.tags_[0]));
            PrintPerTaskDone(PrintType::kIFFT, frame_id, symbol_idx_dl, ant_id);

            if (ifft_counters_.LastTask(frame_id, symbol_idx_dl)) {
              PrintPerSymbolDone(PrintType::kIFFT, frame_id, symbol_idx_dl);
              if (ifft_counters_.LastSymbol(frame_id)) {
                stats_->MasterSetTsc(TsType::kIFFTDone, frame_id);
                PrintPerFrameDone(PrintType::kIFFT, frame_id);
                assert(frame_id == cur_proc_frame_id_);
                cur_proc_frame_id_++;
                cur_sche_frame_id_++;
                stats_->UpdateStatsInFunctionsDownlink(frame_id);
                if (stats_->last_frame_id_ == cfg->FramesToTest() - 1) {
                  goto finish;
                }
              }
            }
          }
        } break;

        case EventType::kPacketTX: {
          /* Data is sent */
          size_t ant_id = gen_tag_t(event.tags_[0]).ant_id_;
          size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;
          size_t symbol_idx_dl = cfg->Frame().GetDLSymbolIdx(symbol_id);
          PrintPerTaskDone(PrintType::kPacketTX, frame_id, symbol_idx_dl,
                           ant_id);
          if (tx_counters_.LastTask(frame_id, symbol_idx_dl)) {
            PrintPerSymbolDone(PrintType::kPacketTX, frame_id, symbol_idx_dl);
            /* If tx of the first symbol is done */
            if (symbol_id == cfg->Frame().GetDLSymbol(0)) {
              stats_->MasterSetTsc(TsType::kTXProcessedFirst, frame_id);
              PrintPerFrameDone(PrintType::kPacketTXFirst, frame_id);
            }
            if (tx_counters_.LastSymbol(frame_id)) {
              stats_->MasterSetTsc(TsType::kTXDone, frame_id);
              PrintPerFrameDone(PrintType::kPacketTX, frame_id);
              if (stats_->last_frame_id_ == cfg->FramesToTest() - 1) {
                goto finish;
              }
            }

            tx_count++;
            if (tx_count == tx_counters_.max_symbol_count_ * 9000) {
              tx_count = 0;
              double diff = GetTimeUs() - tx_begin;
              int samples_num_per_ue =
                  cfg->OfdmDataNum() * tx_counters_.max_symbol_count_ * 1000;

              std::printf(
                  "TX %d samples (per-client) to %zu clients "
                  "in %f secs, throughtput %f bps per-client "
                  "(16QAM), current tx queue length %zu\n",
                  samples_num_per_ue, cfg->UeNum(), diff,
                  samples_num_per_ue * log2(16.0f) / diff,
                  GetConq(EventType::kPacketTX, 0)->size_approx());
              tx_begin = GetTimeUs();
            }
          }
        } break;
        default:
          std::printf("Wrong event type in message queue!");
          std::exit(0);
      } /* End of switch */

      // We schedule FFT processing if the event handling above results in
      // either (a) sufficient packets received for the current frame,
      // or (b) the current frame being updated.
      std::queue<fft_req_tag_t>& cur_fftq =
          fft_queue_arr_[cur_sche_frame_id_ % kFrameWnd];
      size_t qid = cur_sche_frame_id_ & 0x1;
      if (cur_fftq.size() >= config_->FftBlockSize()) {
        size_t num_fft_blocks = cur_fftq.size() / config_->FftBlockSize();
        for (size_t i = 0; i < num_fft_blocks; i++) {
          EventData do_fft_task;
          do_fft_task.num_tags_ = config_->FftBlockSize();
          do_fft_task.event_type_ = EventType::kFFT;

          for (size_t j = 0; j < config_->FftBlockSize(); j++) {
            do_fft_task.tags_[j] = cur_fftq.front().tag_;
            cur_fftq.pop();
            if (fft_created_count_++ == 0) {
              stats_->MasterSetTsc(TsType::kProcessingStarted,
                                   cur_sche_frame_id_);
            } else if (fft_created_count_ == rx_counters_.num_pkts_per_frame_) {
              fft_created_count_ = 0;
              if (cfg->BigstationMode()) {
                cur_sche_frame_id_++;
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

  std::printf("Agora: printing stats and saving to file\n");
  stats_->PrintSummary();
  stats_->SaveToFile();
  if (flags_.enable_save_decode_data_to_file_) {
    SaveDecodeDataToFile(stats_->last_frame_id_);
  }
  if (flags_.enable_save_tx_data_to_file_) {
    SaveTxDataToFile(stats_->last_frame_id_);
  }

  // Calculate and print per-user BER
  if (!kEnableMac && kPrintPhyStats) {
    phy_stats_->PrintPhyStats();
  }
  this->Stop();
}

void Agora::HandleEventFft(size_t tag) {
  size_t frame_id = gen_tag_t(tag).frame_id_;
  size_t symbol_id = gen_tag_t(tag).symbol_id_;
  SymbolType sym_type = config_->GetSymbolType(symbol_id);

  if (sym_type == SymbolType::kPilot) {
    if (fft_counters_.LastTask(frame_id, symbol_id)) {
      PrintPerSymbolDone(PrintType::kFFTPilots, frame_id, symbol_id);
      if (!config_->DownlinkMode() ||
          (config_->DownlinkMode() && !config_->Frame().IsRecCalEnabled()) ||
          (config_->DownlinkMode() && config_->Frame().IsRecCalEnabled() &&
           rc_last_frame_ == frame_id)) {
        /* If CSI of all UEs is ready, schedule ZF/prediction */
        if (fft_counters_.LastSymbol(frame_id)) {
          stats_->MasterSetTsc(TsType::kFFTPilotsDone, frame_id);
          PrintPerFrameDone(PrintType::kFFTPilots, frame_id);
          if (kPrintPhyStats) {
            phy_stats_->PrintSnrStats(frame_id);
          }
          if (kEnableMac) {
            SendSnrReport(EventType::kSNRReport, frame_id, symbol_id);
          }
          ScheduleSubcarriers(EventType::kZF, frame_id, 0);
        }
      }
    }
  } else if (sym_type == SymbolType::kUL) {
    if (fft_counters_.LastTask(frame_id, symbol_id)) {
      size_t symbol_idx_ul = config_->Frame().GetULSymbolIdx(symbol_id);
      fft_cur_frame_for_symbol_[symbol_idx_ul] = frame_id;
      PrintPerSymbolDone(PrintType::kFFTData, frame_id, symbol_id);
      /* If precoder exist, schedule demodulation */
      if (zf_last_frame_ == frame_id) {
        ScheduleSubcarriers(EventType::kDemul, frame_id, symbol_idx_ul);
      }
    }
  } else if (sym_type == SymbolType::kCalDL or sym_type == SymbolType::kCalUL) {
    PrintPerSymbolDone(PrintType::kFFTCal, frame_id, symbol_id);
    if (rc_counters_.LastTask(frame_id)) {
      PrintPerFrameDone(PrintType::kFFTCal, frame_id);
      stats_->MasterSetTsc(TsType::kRCDone, frame_id);
      rc_last_frame_ = frame_id;
    }
  }
}

void Agora::Worker(int tid) {
  PinToCoreWithOffset(ThreadType::kWorker, base_worker_core_offset_, tid,
                      false /* quiet */);

  /* Initialize operators */
  auto* compute_fft = new DoFFT(
      config_, tid, socket_buffer_, socket_buffer_status_, data_buffer_,
      csi_buffers_, calib_dl_buffer_, calib_ul_buffer_, phy_stats_, stats_);

  auto* compute_ifft =
      new DoIFFT(config_, tid, dl_ifft_buffer_, dl_socket_buffer_, stats_);

  auto* compute_zf =
      new DoZF(config_, tid, csi_buffers_, calib_dl_buffer_, calib_ul_buffer_,
               ul_zf_matrices_, dl_zf_matrices_, stats_);

  auto* compute_demul = new DoDemul(config_, tid, data_buffer_, ul_zf_matrices_,
                                    ue_spec_pilot_buffer_, equal_buffer_,
                                    demod_buffers_, phy_stats_, stats_);

  auto* compute_precode =
      new DoPrecode(config_, tid, dl_zf_matrices_, dl_ifft_buffer_,
                    dl_encoded_buffer_, stats_);

  auto* compute_encoding =
      new DoEncode(config_, tid, config_->DlBits(), dl_encoded_buffer_, stats_);

  auto* compute_decoding = new DoDecode(config_, tid, demod_buffers_,
                                        decoded_buffer_, phy_stats_, stats_);

  std::vector<Doer*> computers_vec;
  std::vector<EventType> events_vec;
  if (config_->Frame().NumDLSyms() > 0) {
    computers_vec = {compute_zf, compute_fft, compute_ifft, compute_precode,
                     compute_encoding};
    events_vec = {EventType::kZF, EventType::kFFT, EventType::kIFFT,
                  EventType::kPrecode, EventType::kEncode};
  } else {
    computers_vec = {compute_zf, compute_fft, compute_decoding, compute_demul};
    events_vec = {EventType::kZF, EventType::kFFT, EventType::kDecode,
                  EventType::kDemul};
  }

  size_t cur_qid = 0;
  size_t empty_qeueu_itrs = 0;
  bool empty_queue = true;
  while (true) {
    for (size_t i = 0; i < computers_vec.size(); i++) {
      if (computers_vec[i]->TryLaunch(*GetConq(events_vec[i], cur_qid),
                                      complete_task_queue_[cur_qid],
                                      worker_ptoks_ptr_[tid][cur_qid])) {
        empty_queue = false;
        break;
      }
    }
    // If all queues in this set are empty for 5 iterations,
    // check the other set of qeueus
    if (empty_queue) {
      empty_qeueu_itrs++;
      if (empty_qeueu_itrs == 5) {
        if (cur_sche_frame_id_ != cur_proc_frame_id_) {
          cur_qid ^= 0x1;
        } else {
          cur_qid = cur_sche_frame_id_ & 0x1;
        }
        empty_qeueu_itrs = 0;
      }
    } else {
      empty_queue = true;
    }
  }
}

void Agora::WorkerFft(int tid) {
  PinToCoreWithOffset(ThreadType::kWorkerFFT, base_worker_core_offset_, tid);

  /* Initialize IFFT operator */
  auto* compute_fft = new DoFFT(
      config_, tid, socket_buffer_, socket_buffer_status_, data_buffer_,
      csi_buffers_, calib_dl_buffer_, calib_ul_buffer_, phy_stats_, stats_);
  auto* compute_ifft =
      new DoIFFT(config_, tid, dl_ifft_buffer_, dl_socket_buffer_, stats_);

  while (true) {
    if (compute_fft->TryLaunch(*GetConq(EventType::kFFT, 0),
                               complete_task_queue_[0],
                               worker_ptoks_ptr_[tid][0])) {
    } else if (config_->Frame().NumDLSyms() > 0 &&
               compute_ifft->TryLaunch(*GetConq(EventType::kIFFT, 0),
                                       complete_task_queue_[0],
                                       worker_ptoks_ptr_[tid][0])) {
    }
  }
}

void Agora::WorkerZf(int tid) {
  PinToCoreWithOffset(ThreadType::kWorkerZF, base_worker_core_offset_, tid);

  /* Initialize ZF operator */
  auto* compute_zf =
      new DoZF(config_, tid, csi_buffers_, calib_dl_buffer_, calib_ul_buffer_,
               ul_zf_matrices_, dl_zf_matrices_, stats_);

  while (true) {
    compute_zf->TryLaunch(*GetConq(EventType::kZF, 0), complete_task_queue_[0],
                          worker_ptoks_ptr_[tid][0]);
  }
}

void Agora::WorkerDemul(int tid) {
  PinToCoreWithOffset(ThreadType::kWorkerDemul, base_worker_core_offset_, tid);

  auto* compute_demul = new DoDemul(config_, tid, data_buffer_, ul_zf_matrices_,
                                    ue_spec_pilot_buffer_, equal_buffer_,
                                    demod_buffers_, phy_stats_, stats_);

  /* Initialize Precode operator */
  auto* compute_precode =
      new DoPrecode(config_, tid, dl_zf_matrices_, dl_ifft_buffer_,
                    dl_encoded_buffer_, stats_);

  while (true) {
    if (config_->Frame().NumDLSyms() > 0) {
      compute_precode->TryLaunch(*GetConq(EventType::kDemul, 0),
                                 complete_task_queue_[0],
                                 worker_ptoks_ptr_[tid][0]);
    } else {
      compute_demul->TryLaunch(*GetConq(EventType::kPrecode, 0),
                               complete_task_queue_[0],
                               worker_ptoks_ptr_[tid][0]);
    }
  }
}

void Agora::WorkerDecode(int tid) {
  PinToCoreWithOffset(ThreadType::kWorkerDecode, base_worker_core_offset_, tid);

  auto* compute_encoding =
      new DoEncode(config_, tid, config_->DlBits(), dl_encoded_buffer_, stats_);

  auto* compute_decoding = new DoDecode(config_, tid, demod_buffers_,
                                        decoded_buffer_, phy_stats_, stats_);

  while (true) {
    if (config_->Frame().NumDLSyms() > 0) {
      compute_encoding->TryLaunch(*GetConq(EventType::kEncode, 0),
                                  complete_task_queue_[0],
                                  worker_ptoks_ptr_[tid][0]);
    } else {
      compute_decoding->TryLaunch(*GetConq(EventType::kDecode, 0),
                                  complete_task_queue_[0],
                                  worker_ptoks_ptr_[tid][0]);
    }
  }
}

void Agora::CreateThreads() {
  auto& cfg = config_;
  if (cfg->BigstationMode()) {
    for (size_t i = 0; i < cfg->FftThreadNum(); i++) {
      worker_std_threads_[i] = std::thread(&Agora::WorkerFft, this, i);
    }
    for (size_t i = cfg->FftThreadNum();
         i < cfg->FftThreadNum() + cfg->ZfThreadNum(); i++) {
      worker_std_threads_[i] = std::thread(&Agora::WorkerZf, this, i);
    }
    for (size_t i = cfg->FftThreadNum() + cfg->ZfThreadNum();
         i < cfg->FftThreadNum() + cfg->ZfThreadNum() + cfg->DemulThreadNum();
         i++) {
      worker_std_threads_[i] = std::thread(&Agora::WorkerDemul, this, i);
    }
    for (size_t i =
             cfg->FftThreadNum() + cfg->ZfThreadNum() + cfg->DemulThreadNum();
         i < cfg->WorkerThreadNum(); i++) {
      worker_std_threads_[i] = std::thread(&Agora::WorkerDecode, this, i);
    }
  } else {
    for (size_t i = 0; i < cfg->WorkerThreadNum(); i++) {
      worker_std_threads_[i] = std::thread(&Agora::Worker, this, i);
    }
  }
}

void Agora::UpdateRanConfig(RanConfig rc) {
  config_->UpdateModCfgs(rc.mod_order_bits_);
}

void Agora::UpdateRxCounters(size_t frame_id, size_t symbol_id) {
  const size_t frame_slot = frame_id % kFrameWnd;
  if (config_->IsPilot(frame_id, symbol_id)) {
    rx_counters_.num_pilot_pkts_[frame_slot]++;
    if (rx_counters_.num_pilot_pkts_[frame_slot] ==
        rx_counters_.num_pilot_pkts_per_frame_) {
      rx_counters_.num_pilot_pkts_[frame_slot] = 0;
      stats_->MasterSetTsc(TsType::kPilotAllRX, frame_id);
      PrintPerFrameDone(PrintType::kPacketRXPilots, frame_id);
    }
  } else if (config_->IsCalDlPilot(frame_id, symbol_id) or
             config_->IsCalUlPilot(frame_id, symbol_id)) {
    if (++rx_counters_.num_reciprocity_pkts_[frame_slot] ==
        rx_counters_.num_reciprocity_pkts_per_frame_) {
      rx_counters_.num_reciprocity_pkts_[frame_slot] = 0;
      stats_->MasterSetTsc(TsType::kRCAllRX, frame_id);
    }
  }
  // Receive first packet in a frame
  if (rx_counters_.num_pkts_[frame_slot] == 0) {
    // schedule this frame's encoding
    for (size_t i = 0; i < config_->Frame().NumDLSyms(); i++) {
      ScheduleCodeblocks(EventType::kEncode, frame_id,
                         config_->Frame().GetDLSymbol(i));
    }
    stats_->MasterSetTsc(TsType::kPilotRX, frame_id);
    if (kDebugPrintPerFrameStart) {
      const size_t prev_frame_slot = (frame_slot + kFrameWnd - 1) % kFrameWnd;
      std::printf(
          "Main [frame %zu + %.2f ms since last frame]: Received "
          "first packet. Remaining packets in prev frame: %zu\n",
          frame_id,
          stats_->MasterGetDeltaMs(TsType::kPilotRX, frame_id, frame_id - 1),
          rx_counters_.num_pkts_[prev_frame_slot]);
    }
  }

  rx_counters_.num_pkts_[frame_slot]++;
  if (rx_counters_.num_pkts_[frame_slot] == rx_counters_.num_pkts_per_frame_) {
    stats_->MasterSetTsc(TsType::kRXDone, frame_id);
    PrintPerFrameDone(PrintType::kPacketRX, frame_id);
    rx_counters_.num_pkts_[frame_slot] = 0;
  }
}

void Agora::PrintPerFrameDone(PrintType print_type, size_t frame_id) {
  if (!kDebugPrintPerFrameDone) {
    return;
  }
  switch (print_type) {
    case (PrintType::kPacketRXPilots):
      std::printf("Main [frame %zu + %.2f ms]: Received all pilots\n", frame_id,
                  stats_->MasterGetDeltaMs(TsType::kPilotAllRX,
                                           TsType::kPilotRX, frame_id));
      break;
    case (PrintType::kPacketRX): {
      std::printf("Main [frame %zu + %.2f ms]: Received all packets\n",
                  frame_id,
                  stats_->MasterGetDeltaMs(TsType::kRXDone, TsType::kPilotRX,
                                           frame_id));
    } break;
    case (PrintType::kFFTPilots):
      std::printf("Main [frame %zu + %.2f ms]: FFT-ed all pilots\n", frame_id,
                  stats_->MasterGetDeltaMs(TsType::kFFTPilotsDone,
                                           TsType::kPilotRX, frame_id));
      break;
    case (PrintType::kFFTCal):
      std::printf(
          "Main [frame %zu + %.2f ms]: FFT-ed all calibration symbols\n",
          frame_id,
          stats_->MasterGetUsSince(TsType::kRCAllRX, frame_id) / 1000.0);
      break;
    case (PrintType::kZF):
      std::printf("Main [frame %zu + %.2f ms]: Completed zero-forcing\n",
                  frame_id,
                  stats_->MasterGetDeltaMs(TsType::kZFDone, TsType::kPilotRX,
                                           frame_id));
      break;
    case (PrintType::kDemul):
      std::printf("Main [frame %zu + %.2f ms]: Completed demodulation\n",
                  frame_id,
                  stats_->MasterGetDeltaMs(TsType::kDemulDone, TsType::kPilotRX,
                                           frame_id));
      break;
    case (PrintType::kDecode):
      std::printf(
          "Main [frame %zu + %.2f ms]: Completed LDPC decoding (%zu UL "
          "symbols)\n",
          frame_id,
          stats_->MasterGetDeltaMs(TsType::kDecodeDone, TsType::kPilotRX,
                                   frame_id),
          config_->Frame().NumULSyms());
      break;
    case (PrintType::kEncode):
      std::printf("Main [frame %zu + %.2f ms]: Completed LDPC encoding\n",
                  frame_id,
                  stats_->MasterGetDeltaMs(TsType::kEncodeDone,
                                           TsType::kPilotRX, frame_id));
      break;
    case (PrintType::kPrecode):
      std::printf("Main [frame %zu + %.2f ms]: Completed precoding\n", frame_id,
                  stats_->MasterGetDeltaMs(TsType::kPrecodeDone,
                                           TsType::kPilotRX, frame_id));
      break;
    case (PrintType::kIFFT):
      std::printf("Main [frame %zu + %.2f ms]: Completed IFFT\n", frame_id,
                  stats_->MasterGetDeltaMs(TsType::kIFFTDone, TsType::kPilotRX,
                                           frame_id));
      break;
    case (PrintType::kPacketTXFirst):
      std::printf("Main [frame %zu + %.2f ms]: Completed TX of first symbol\n",
                  frame_id,
                  stats_->MasterGetDeltaMs(TsType::kTXProcessedFirst,
                                           TsType::kPilotRX, frame_id));
      break;
    case (PrintType::kPacketTX):
      std::printf(
          "Main [frame %zu + %.2f ms]: Completed TX (%zu DL symbols)\n",
          frame_id,
          stats_->MasterGetDeltaMs(TsType::kTXDone, TsType::kPilotRX, frame_id),
          config_->Frame().NumDLSyms());
      break;
    case (PrintType::kPacketToMac):
      std::printf("Main [frame %zu + %.2f ms]: Completed MAC TX \n", frame_id,
                  stats_->MasterGetMsSince(TsType::kPilotRX, frame_id));
      break;
    default:
      std::printf("Wrong task type in frame done print!");
  }
}

void Agora::PrintPerSymbolDone(PrintType print_type, size_t frame_id,
                               size_t symbol_id) {
  if (!kDebugPrintPerSymbolDone) {
    return;
  }
  switch (print_type) {
    case (PrintType::kFFTPilots):
      std::printf(
          "Main [frame %zu symbol %zu + %.3f ms]: FFT-ed pilot symbol, "
          "%zu symbols done\n",
          frame_id, symbol_id,
          stats_->MasterGetMsSince(TsType::kPilotRX, frame_id),
          fft_counters_.GetSymbolCount(frame_id) + 1);
      break;
    case (PrintType::kFFTData):
      std::printf(
          "Main [frame %zu symbol %zu + %.3f ms]: FFT-ed data symbol, "
          "precoder status: %d\n",
          frame_id, symbol_id,
          stats_->MasterGetMsSince(TsType::kPilotRX, frame_id),
          static_cast<int>(zf_last_frame_ == frame_id));
      break;
    case (PrintType::kDemul):
      std::printf(
          "Main [frame %zu symbol %zu + %.3f ms]: Completed demodulation, "
          "%zu symbols done\n",
          frame_id, symbol_id,
          stats_->MasterGetMsSince(TsType::kPilotRX, frame_id),
          demul_counters_.GetSymbolCount(frame_id) + 1);
      break;
    case (PrintType::kDecode):
      std::printf(
          "Main [frame %zu symbol %zu + %.3f ms]: Completed decoding, "
          "%zu symbols done\n",
          frame_id, symbol_id,
          stats_->MasterGetMsSince(TsType::kPilotRX, frame_id),
          decode_counters_.GetSymbolCount(frame_id) + 1);
      break;
    case (PrintType::kEncode):
      std::printf(
          "Main [frame %zu symbol %zu + %.3f ms]: Completed encoding, "
          "%zu symbols done\n",
          frame_id, symbol_id,
          stats_->MasterGetMsSince(TsType::kPilotRX, frame_id),
          encode_counters_.GetSymbolCount(frame_id) + 1);
      break;
    case (PrintType::kPrecode):
      std::printf(
          "Main [frame %zu symbol %zu + %.3f ms]: Completed precoding, "
          "%zu symbols done\n",
          frame_id, symbol_id,
          stats_->MasterGetMsSince(TsType::kPilotRX, frame_id),
          precode_counters_.GetSymbolCount(frame_id) + 1);
      break;
    case (PrintType::kIFFT):
      std::printf(
          "Main [frame %zu symbol %zu + %.3f ms]: Completed IFFT, "
          "%zu symbols done\n",
          frame_id, symbol_id,
          stats_->MasterGetMsSince(TsType::kPilotRX, frame_id),
          ifft_counters_.GetSymbolCount(frame_id) + 1);
      break;
    case (PrintType::kPacketTX):
      std::printf(
          "Main [frame %zu symbol %zu + %.3f ms]: Completed TX, "
          "%zu symbols done\n",
          frame_id, symbol_id,
          stats_->MasterGetMsSince(TsType::kPilotRX, frame_id),
          tx_counters_.GetSymbolCount(frame_id) + 1);
      break;
    case (PrintType::kPacketToMac):
      std::printf(
          "Main [frame %zu symbol %zu + %.3f ms]: Completed MAC TX, "
          "%zu symbols done\n",
          frame_id, symbol_id,
          stats_->MasterGetMsSince(TsType::kPilotRX, frame_id),
          tomac_counters_.GetSymbolCount(frame_id) + 1);
      break;
    default:
      std::printf("Wrong task type in symbol done print!");
  }
}

void Agora::PrintPerTaskDone(PrintType print_type, size_t frame_id,
                             size_t symbol_id, size_t ant_or_sc_id) {
  if (!kDebugPrintPerTaskDone) {
    return;
  }
  switch (print_type) {
    case (PrintType::kZF):
      std::printf("Main thread: ZF done frame: %zu, subcarrier %zu\n", frame_id,
                  ant_or_sc_id);
      break;
    case (PrintType::kRC):
      std::printf("Main thread: RC done frame: %zu, subcarrier %zu\n", frame_id,
                  ant_or_sc_id);
      break;
    case (PrintType::kDemul):
      std::printf(
          "Main thread: Demodulation done frame: %zu, symbol: %zu, sc: "
          "%zu, num blocks done: %zu\n",
          frame_id, symbol_id, ant_or_sc_id,
          demul_counters_.GetTaskCount(frame_id, symbol_id));
      break;
    case (PrintType::kDecode):
      std::printf(
          "Main thread: Decoding done frame: %zu, symbol: %zu, sc: %zu, "
          "num blocks done: %zu\n",
          frame_id, symbol_id, ant_or_sc_id,
          decode_counters_.GetTaskCount(frame_id, symbol_id));
      break;
    case (PrintType::kPrecode):
      std::printf(
          "Main thread: Precoding done frame: %zu, symbol: %zu, "
          "subcarrier: %zu, total SCs: %zu\n",
          frame_id, symbol_id, ant_or_sc_id,
          precode_counters_.GetTaskCount(frame_id, symbol_id));
      break;
    case (PrintType::kIFFT):
      std::printf(
          "Main thread: IFFT done frame: %zu, symbol: %zu, antenna: %zu, "
          "total ants: %zu\n",
          frame_id, symbol_id, ant_or_sc_id,
          ifft_counters_.GetTaskCount(frame_id, symbol_id));
      break;
    case (PrintType::kPacketTX):
      std::printf(
          "Main thread: TX done frame: %zu, symbol: %zu, antenna: %zu, "
          "total packets: %zu\n",
          frame_id, symbol_id, ant_or_sc_id,
          tx_counters_.GetTaskCount(frame_id, symbol_id));
      break;
    default:
      std::printf("Wrong task type in task done print!");
  }
}

void Agora::InitializeQueues() {
  using mt_queue_t = moodycamel::ConcurrentQueue<EventData>;

  int data_symbol_num_perframe = config_->Frame().NumDataSyms();
  message_queue_ = mt_queue_t(512 * data_symbol_num_perframe);
  for (auto& c : complete_task_queue_) {
    c = mt_queue_t(256 * data_symbol_num_perframe);
  }

  // Create concurrent queues for each Doer
  for (auto& vec : sched_info_arr_) {
    for (auto& s : vec) {
      s.concurrent_q_ = mt_queue_t(256 * data_symbol_num_perframe);
      s.ptok_ = new moodycamel::ProducerToken(s.concurrent_q_);
    }
  }

  for (size_t i = 0; i < config_->SocketThreadNum(); i++) {
    rx_ptoks_ptr_[i] = new moodycamel::ProducerToken(message_queue_);
    tx_ptoks_ptr_[i] =
        new moodycamel::ProducerToken(*GetConq(EventType::kPacketTX, 0));
  }

  for (size_t i = 0; i < config_->WorkerThreadNum(); i++) {
    for (size_t j = 0; j < 2; j++) {
      worker_ptoks_ptr_[i][j] =
          new moodycamel::ProducerToken(complete_task_queue_[j]);
    }
  }
}

void Agora::InitializeUplinkBuffers() {
  auto& cfg = config_;
  const size_t task_buffer_symbol_num_ul = cfg->Frame().NumULSyms() * kFrameWnd;

  AllocBuffer1d(&task_threads_, cfg->WorkerThreadNum(),
                Agora_memory::Alignment_t::kAlign64, 0);

  socket_buffer_status_size_ =
      cfg->BsAntNum() * kFrameWnd * cfg->Frame().NumTotalSyms();
  socket_buffer_size_ = cfg->PacketLength() * socket_buffer_status_size_;

  socket_buffer_.Malloc(cfg->SocketThreadNum() /* RX */, socket_buffer_size_,
                        Agora_memory::Alignment_t::kAlign64);
  socket_buffer_status_.Calloc(cfg->SocketThreadNum() /* RX */,
                               socket_buffer_status_size_,
                               Agora_memory::Alignment_t::kAlign64);

  data_buffer_.Malloc(task_buffer_symbol_num_ul,
                      cfg->OfdmDataNum() * cfg->BsAntNum(),
                      Agora_memory::Alignment_t::kAlign64);

  equal_buffer_.Malloc(task_buffer_symbol_num_ul,
                       cfg->OfdmDataNum() * cfg->UeNum(),
                       Agora_memory::Alignment_t::kAlign64);
  ue_spec_pilot_buffer_.Calloc(
      kFrameWnd, cfg->Frame().ClientUlPilotSymbols() * cfg->UeNum(),
      Agora_memory::Alignment_t::kAlign64);

  rx_counters_.num_pkts_per_frame_ =
      cfg->BsAntNum() *
      (cfg->Frame().NumPilotSyms() + cfg->Frame().NumULSyms() +
       static_cast<size_t>(cfg->Frame().IsRecCalEnabled()));
  rx_counters_.num_pilot_pkts_per_frame_ =
      cfg->BsAntNum() * cfg->Frame().NumPilotSyms();
  rx_counters_.num_reciprocity_pkts_per_frame_ = cfg->BsAntNum();

  fft_created_count_ = 0;
  fft_counters_.Init(cfg->Frame().NumPilotSyms(), cfg->BsAntNum());
  fft_cur_frame_for_symbol_ =
      std::vector<size_t>(cfg->Frame().NumULSyms(), SIZE_MAX);

  rc_counters_.Init(cfg->BsAntNum());

  zf_counters_.Init(config_->ZfEventsPerSymbol());

  demul_counters_.Init(cfg->Frame().NumULSyms(),
                       config_->DemulEventsPerSymbol());

  decode_counters_.Init(
      cfg->Frame().NumULSyms(),
      config_->LdpcConfig().NumBlocksInSymbol() * cfg->UeNum());

  tomac_counters_.Init(cfg->Frame().NumULSyms(), cfg->UeNum());
}

void Agora::InitializeDownlinkBuffers() {
  auto& cfg = config_;
  const size_t task_buffer_symbol_num = cfg->Frame().NumDLSyms() * kFrameWnd;

  size_t dl_socket_buffer_status_size =
      cfg->BsAntNum() * kFrameWnd * cfg->Frame().NumDLSyms();
  size_t dl_socket_buffer_size =
      cfg->DlPacketLength() * dl_socket_buffer_status_size;
  AllocBuffer1d(&dl_socket_buffer_, dl_socket_buffer_size,
                Agora_memory::Alignment_t::kAlign64, 0);
  AllocBuffer1d(&dl_socket_buffer_status_, dl_socket_buffer_status_size,
                Agora_memory::Alignment_t::kAlign64, 1);

  dl_bits_buffer_.Calloc(task_buffer_symbol_num,
                         cfg->OfdmDataNum() * cfg->UeNum(),
                         Agora_memory::Alignment_t::kAlign64);
  size_t dl_bits_buffer_status_size =
      task_buffer_symbol_num * cfg->LdpcConfig().NumBlocksInSymbol();
  dl_bits_buffer_status_.Calloc(cfg->UeNum(), dl_bits_buffer_status_size,
                                Agora_memory::Alignment_t::kAlign64);

  dl_ifft_buffer_.Calloc(cfg->BsAntNum() * task_buffer_symbol_num,
                         cfg->OfdmCaNum(), Agora_memory::Alignment_t::kAlign64);
  calib_dl_buffer_.Calloc(kFrameWnd, cfg->BfAntNum() * cfg->OfdmDataNum(),
                          Agora_memory::Alignment_t::kAlign64);
  calib_ul_buffer_.Calloc(kFrameWnd, cfg->BfAntNum() * cfg->OfdmDataNum(),
                          Agora_memory::Alignment_t::kAlign64);
  // initialize the content of the last window to 1
  for (size_t i = 0; i < cfg->OfdmDataNum() * cfg->BfAntNum(); i++) {
    calib_dl_buffer_[kFrameWnd - 1][i] = {1, 0};
    calib_ul_buffer_[kFrameWnd - 1][i] = {1, 0};
  }
  dl_encoded_buffer_.Calloc(task_buffer_symbol_num,
                            Roundup<64>(cfg->OfdmDataNum()) * cfg->UeNum(),
                            Agora_memory::Alignment_t::kAlign64);

  frommac_counters_.Init(cfg->Frame().NumDLSyms(), config_->UeNum());
  encode_counters_.Init(
      cfg->Frame().NumDLSyms(),
      config_->LdpcConfig().NumBlocksInSymbol() * cfg->UeNum());
  encode_cur_frame_for_symbol_ =
      std::vector<size_t>(cfg->Frame().NumDLSyms(), SIZE_MAX);
  precode_counters_.Init(cfg->Frame().NumDLSyms(),
                         config_->DemulEventsPerSymbol());
  ifft_counters_.Init(cfg->Frame().NumDLSyms(), cfg->BsAntNum());
  tx_counters_.Init(cfg->Frame().NumDLSyms(), cfg->BsAntNum());
}

void Agora::FreeUplinkBuffers() {
  socket_buffer_.Free();
  socket_buffer_status_.Free();
  data_buffer_.Free();
  equal_buffer_.Free();
}

void Agora::FreeDownlinkBuffers() {
  FreeBuffer1d(&dl_socket_buffer_);
  FreeBuffer1d(&dl_socket_buffer_status_);

  dl_ifft_buffer_.Free();
  calib_dl_buffer_.Free();
  calib_ul_buffer_.Free();
  dl_encoded_buffer_.Free();
}

void Agora::SaveDecodeDataToFile(int frame_id) {
  auto& cfg = config_;
  const size_t num_decoded_bytes =
      cfg->NumBytesPerCb() * cfg->LdpcConfig().NumBlocksInSymbol();

  std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
  std::string filename = cur_directory + "/data/decode_data.bin";
  std::printf("Saving decode data to %s\n", filename.c_str());
  FILE* fp = fopen(filename.c_str(), "wb");

  for (size_t i = 0; i < cfg->Frame().NumULSyms(); i++) {
    for (size_t j = 0; j < cfg->UeNum(); j++) {
      uint8_t* ptr = decoded_buffer_[frame_id % kFrameWnd][i][j];
      fwrite(ptr, num_decoded_bytes, sizeof(uint8_t), fp);
    }
  }
  fclose(fp);
}

void Agora::SaveTxDataToFile(UNUSED int frame_id) {
  auto& cfg = config_;

  std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
  std::string filename = cur_directory + "/data/tx_data.bin";
  std::printf("Saving TX data to %s\n", filename.c_str());
  FILE* fp = fopen(filename.c_str(), "wb");

  for (size_t i = 0; i < cfg->Frame().NumDLSyms(); i++) {
    size_t total_data_symbol_id = cfg->GetTotalDataSymbolIdxDl(frame_id, i);

    for (size_t ant_id = 0; ant_id < cfg->BsAntNum(); ant_id++) {
      size_t offset = total_data_symbol_id * cfg->BsAntNum() + ant_id;
      struct Packet* pkt =
          (struct Packet*)(&dl_socket_buffer_[offset * cfg->DlPacketLength()]);
      short* socket_ptr = pkt->data_;
      fwrite(socket_ptr, cfg->SampsPerSymbol() * 2, sizeof(short), fp);
    }
  }
  fclose(fp);
}

void Agora::GetEqualData(float** ptr, int* size) {
  auto& cfg = config_;
  auto offset = cfg->GetTotalDataSymbolIdxUl(
      max_equaled_frame_, cfg->Frame().ClientUlPilotSymbols());
  *ptr = (float*)&equal_buffer_[offset][0];
  *size = cfg->UeNum() * cfg->OfdmDataNum() * 2;
}

extern "C" {
EXPORT Agora* AgoraNew(Config* cfg) {
  // std::printf("Size of Agora: %d\n",sizeof(Agora *));
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
