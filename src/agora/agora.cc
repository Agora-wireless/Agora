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
#include "packet_txrx_radio.h"
#include "packet_txrx_sim.h"

static const bool kDebugPrintPacketsFromMac = false;
static const bool kDebugDeferral = true;

Agora::Agora(Config* const cfg)
    : base_worker_core_offset_(cfg->CoreOffset() + 1 + cfg->SocketThreadNum()),
      config_(cfg),
      stats_(std::make_unique<Stats>(cfg)),
      phy_stats_(std::make_unique<PhyStats>(cfg, Direction::kUplink)),
      csi_buffers_(kFrameWnd, cfg->UeAntNum(),
                   cfg->BsAntNum() * cfg->OfdmDataNum()),
      ul_zf_matrices_(kFrameWnd, cfg->OfdmDataNum(),
                      cfg->BsAntNum() * cfg->UeAntNum()),
      demod_buffers_(kFrameWnd, cfg->Frame().NumULSyms(), cfg->UeAntNum(),
                     kMaxModType * cfg->OfdmDataNum()),
      decoded_buffer_(kFrameWnd, cfg->Frame().NumULSyms(), cfg->UeAntNum(),
                      cfg->LdpcConfig(Direction::kUplink).NumBlocksInSymbol() *
                          Roundup<64>(cfg->NumBytesPerCb(Direction::kUplink))),
      dl_zf_matrices_(kFrameWnd, cfg->OfdmDataNum(),
                      cfg->UeAntNum() * cfg->BsAntNum()) {
  std::string directory = TOSTRING(PROJECT_DIRECTORY);
  AGORA_LOG_INFO("Agora: project directory [%s], RDTSC frequency = %.2f GHz\n",
                 directory.c_str(), cfg->FreqGhz());

  PinToCoreWithOffset(ThreadType::kMaster, cfg->CoreOffset(), 0,
                      kEnableCoreReuse, false /* quiet */);
  CheckIncrementScheduleFrame(0, ScheduleProcessingFlags::kProcessingComplete);
  // Important to set cur_sche_frame_id_ after the call to
  // CheckIncrementScheduleFrame because it will be incremented however,
  // CheckIncrementScheduleFrame will initialize the schedule tracking variable
  // correctly.
  cur_sche_frame_id_ = 0;
  cur_proc_frame_id_ = 0;

  InitializeQueues();
  InitializeUplinkBuffers();
  InitializeDownlinkBuffers();
  InitializeThreads();
}

Agora::~Agora() {
  if (kEnableMac == true) {
    mac_std_thread_.join();
  }

  FreeUplinkBuffers();
  FreeDownlinkBuffers();
  stats_.reset();
  phy_stats_.reset();
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
    if (zf_last_frame_ == frame_id) {
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
    TryEnqueueFallback(GetConq(event_type, qid), GetPtok(event_type, qid),
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

      TryEnqueueBulkFallback(GetConq(EventType::kPacketTX, 0),
                             tx_ptoks_ptr_[enqueue_worker_id], worker.data(),
                             worker.size());
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
    case EventType::kZF:
      num_events = config_->ZfEventsPerSymbol();
      block_size = config_->ZfBlockSize();
      break;
    default:
      assert(false);
  }

  size_t qid = (frame_id & 0x1);
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
    TryEnqueueFallback(GetConq(event_type, qid), GetPtok(event_type, qid),
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
        complete_task_queue_[(cur_proc_frame_id_ & 0x1)].try_dequeue_bulk(
            events_list + num_events, max_events_needed);
  }
  return num_events;
}

void Agora::Start() {
  const auto& cfg = this->config_;

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
      std::max(kDequeueBulkSizeTXRX * (cfg->SocketThreadNum() + 1 /* MAC */),
               kDequeueBulkSizeWorker * cfg->WorkerThreadNum());
  EventData events_list[max_events_needed];

  while ((config_->Running() == true) &&
         (SignalHandler::GotExitSignal() == false)) {
    // Get a batch of events
    size_t num_events = FetchEvent(events_list, is_turn_to_dequeue_from_io);
    is_turn_to_dequeue_from_io = !is_turn_to_dequeue_from_io;

    // Handle each event
    for (size_t ev_i = 0; ev_i < num_events; ev_i++) {
      EventData& event = events_list[ev_i];
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
  for (auto& c : complete_task_queue_) {
    c = mt_queue_t(kDefaultWorkerQueueSize * data_symbol_num_perframe);
  }
  // Create concurrent queues for each Doer
  for (auto& vec : sched_info_arr_) {
    for (auto& s : vec) {
      s.concurrent_q_ =
          mt_queue_t(kDefaultWorkerQueueSize * data_symbol_num_perframe);
      s.ptok_ = new moodycamel::ProducerToken(s.concurrent_q_);
    }
  }

  for (size_t i = 0; i < config_->SocketThreadNum(); i++) {
    rx_ptoks_ptr_[i] = new moodycamel::ProducerToken(message_queue_);
    tx_ptoks_ptr_[i] =
        new moodycamel::ProducerToken(*GetConq(EventType::kPacketTX, 0));
  }

  for (size_t i = 0; i < config_->WorkerThreadNum(); i++) {
    for (size_t j = 0; j < kScheduleQueues; j++) {
      worker_ptoks_ptr_[i][j] =
          new moodycamel::ProducerToken(complete_task_queue_[j]);
    }
  }
}

void Agora::FreeQueues() {
  // remove tokens for each doer
  for (auto& vec : sched_info_arr_) {
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
      delete worker_ptoks_ptr_[i][j];
    }
  }
}

void Agora::InitializeUplinkBuffers() {
  const auto& cfg = config_;
  const size_t task_buffer_symbol_num_ul = cfg->Frame().NumULSyms() * kFrameWnd;

  socket_buffer_size_ = cfg->PacketLength() * cfg->BsAntNum() * kFrameWnd *
                        cfg->Frame().NumTotalSyms();

  socket_buffer_.Malloc(cfg->SocketThreadNum() /* RX */, socket_buffer_size_,
                        Agora_memory::Alignment_t::kAlign64);

  data_buffer_.Malloc(task_buffer_symbol_num_ul,
                      cfg->OfdmDataNum() * cfg->BsAntNum(),
                      Agora_memory::Alignment_t::kAlign64);

  equal_buffer_.Malloc(task_buffer_symbol_num_ul,
                       cfg->OfdmDataNum() * cfg->UeAntNum(),
                       Agora_memory::Alignment_t::kAlign64);
  ue_spec_pilot_buffer_.Calloc(
      kFrameWnd, cfg->Frame().ClientUlPilotSymbols() * cfg->UeAntNum(),
      Agora_memory::Alignment_t::kAlign64);

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

  zf_counters_.Init(cfg->ZfEventsPerSymbol());

  demul_counters_.Init(cfg->Frame().NumULSyms(), cfg->DemulEventsPerSymbol());

  decode_counters_.Init(
      cfg->Frame().NumULSyms(),
      cfg->LdpcConfig(Direction::kUplink).NumBlocksInSymbol() *
          cfg->UeAntNum());

  tomac_counters_.Init(cfg->Frame().NumULSyms(), cfg->UeAntNum());
}

void Agora::InitializeDownlinkBuffers() {
  if (config_->Frame().NumDLSyms() > 0) {
    AGORA_LOG_TRACE("Agora: Initializing downlink buffers\n");

    const size_t task_buffer_symbol_num =
        config_->Frame().NumDLSyms() * kFrameWnd;

    size_t dl_socket_buffer_status_size =
        config_->BsAntNum() * task_buffer_symbol_num;
    size_t dl_socket_buffer_size =
        config_->DlPacketLength() * dl_socket_buffer_status_size;
    AllocBuffer1d(&dl_socket_buffer_, dl_socket_buffer_size,
                  Agora_memory::Alignment_t::kAlign64, 0);

    size_t dl_bits_buffer_size =
        kFrameWnd * config_->MacBytesNumPerframe(Direction::kDownlink);
    this->dl_bits_buffer_.Calloc(config_->UeAntNum(), dl_bits_buffer_size,
                                 Agora_memory::Alignment_t::kAlign64);
    this->dl_bits_buffer_status_.Calloc(config_->UeAntNum(), kFrameWnd,
                                        Agora_memory::Alignment_t::kAlign64);

    dl_ifft_buffer_.Calloc(config_->BsAntNum() * task_buffer_symbol_num,
                           config_->OfdmCaNum(),
                           Agora_memory::Alignment_t::kAlign64);
    calib_dl_buffer_.Malloc(kFrameWnd,
                            config_->BfAntNum() * config_->OfdmDataNum(),
                            Agora_memory::Alignment_t::kAlign64);
    calib_ul_buffer_.Malloc(kFrameWnd,
                            config_->BfAntNum() * config_->OfdmDataNum(),
                            Agora_memory::Alignment_t::kAlign64);
    calib_dl_msum_buffer_.Malloc(kFrameWnd,
                                 config_->BfAntNum() * config_->OfdmDataNum(),
                                 Agora_memory::Alignment_t::kAlign64);
    calib_ul_msum_buffer_.Malloc(kFrameWnd,
                                 config_->BfAntNum() * config_->OfdmDataNum(),
                                 Agora_memory::Alignment_t::kAlign64);
    //initialize the calib buffers
    const complex_float complex_init = {0.0f, 0.0f};
    //const complex_float complex_init = {1.0f, 0.0f};
    for (size_t frame = 0u; frame < kFrameWnd; frame++) {
      for (size_t i = 0; i < (config_->OfdmDataNum() * config_->BfAntNum());
           i++) {
        calib_dl_buffer_[frame][i] = complex_init;
        calib_ul_buffer_[frame][i] = complex_init;
        calib_dl_msum_buffer_[frame][i] = complex_init;
        calib_ul_msum_buffer_[frame][i] = complex_init;
      }
    }
    dl_mod_bits_buffer_.Calloc(
        task_buffer_symbol_num,
        Roundup<64>(config_->GetOFDMDataNum()) * config_->UeAntNum(),
        Agora_memory::Alignment_t::kAlign64);

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
  /* Initialize TXRX threads */
  if (kUseArgos || kUseUHD) {
    packet_tx_rx_ = std::make_unique<PacketTxRxRadio>(
        cfg, cfg->CoreOffset() + 1, &message_queue_,
        GetConq(EventType::kPacketTX, 0), rx_ptoks_ptr_, tx_ptoks_ptr_,
        socket_buffer_, socket_buffer_size_ / cfg->PacketLength(),
        this->stats_->FrameStart(), dl_socket_buffer_);
#if defined(USE_DPDK)
  } else if (kUseDPDK) {
    packet_tx_rx_ = std::make_unique<PacketTxRxDpdk>(
        cfg, cfg->CoreOffset() + 1, &message_queue_,
        GetConq(EventType::kPacketTX, 0), rx_ptoks_ptr_, tx_ptoks_ptr_,
        socket_buffer_, socket_buffer_size_ / cfg->PacketLength(),
        this->stats_->FrameStart(), dl_socket_buffer_);
#endif
  } else {
    /* Default to the simulator */
    packet_tx_rx_ = std::make_unique<PacketTxRxSim>(
        cfg, cfg->CoreOffset() + 1, &message_queue_,
        GetConq(EventType::kPacketTX, 0), rx_ptoks_ptr_, tx_ptoks_ptr_,
        socket_buffer_, socket_buffer_size_ / cfg->PacketLength(),
        this->stats_->FrameStart(), dl_socket_buffer_);
  }

  if (kEnableMac == true) {
    const size_t mac_cpu_core =
        cfg->CoreOffset() + cfg->SocketThreadNum() + cfg->WorkerThreadNum() + 1;
    mac_thread_ = std::make_unique<MacThreadBaseStation>(
        cfg, mac_cpu_core, decoded_buffer_, &dl_bits_buffer_,
        &dl_bits_buffer_status_, &mac_request_queue_, &mac_response_queue_);

    mac_std_thread_ =
        std::thread(&MacThreadBaseStation::RunEventLoop, mac_thread_.get());
  }

  // Create workers
  worker_ = std::make_unique<Worker>();

  AGORA_LOG_INFO(
      "Master thread core %zu, TX/RX thread cores %zu--%zu, worker thread "
      "cores %zu--%zu\n",
      cfg->CoreOffset(), cfg->CoreOffset() + 1,
      cfg->CoreOffset() + 1 + cfg->SocketThreadNum() - 1,
      base_worker_core_offset_,
      base_worker_core_offset_ + cfg->WorkerThreadNum() - 1);
}

void Agora::FreeUplinkBuffers() {
  socket_buffer_.Free();
  data_buffer_.Free();
  equal_buffer_.Free();
  ue_spec_pilot_buffer_.Free();
}

void Agora::FreeDownlinkBuffers() {
  if (config_->Frame().NumDLSyms() > 0) {
    FreeBuffer1d(&dl_socket_buffer_);

    dl_ifft_buffer_.Free();
    calib_dl_buffer_.Free();
    calib_ul_buffer_.Free();
    calib_dl_msum_buffer_.Free();
    calib_ul_msum_buffer_.Free();
    dl_mod_bits_buffer_.Free();
    dl_bits_buffer_.Free();
    dl_bits_buffer_status_.Free();
  }
}

void Agora::SaveDecodeDataToFile(int frame_id) {
  const auto& cfg = config_;
  const size_t num_decoded_bytes =
      cfg->NumBytesPerCb(Direction::kUplink) *
      cfg->LdpcConfig(Direction::kUplink).NumBlocksInSymbol();

  std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
  std::string filename = cur_directory + "/data/decode_data.bin";
  AGORA_LOG_INFO("Saving decode data to %s\n", filename.c_str());
  FILE* fp = std::fopen(filename.c_str(), "wb");

  for (size_t i = 0; i < cfg->Frame().NumULSyms(); i++) {
    for (size_t j = 0; j < cfg->UeAntNum(); j++) {
      int8_t* ptr = decoded_buffer_[(frame_id % kFrameWnd)][i][j];
      std::fwrite(ptr, num_decoded_bytes, sizeof(uint8_t), fp);
    }
  }
  std::fclose(fp);
}

void Agora::SaveTxDataToFile(UNUSED int frame_id) {
  const auto& cfg = config_;

  std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
  std::string filename = cur_directory + "/data/tx_data.bin";
  AGORA_LOG_INFO("Saving Frame %d TX data to %s\n", frame_id, filename.c_str());
  FILE* fp = std::fopen(filename.c_str(), "wb");

  for (size_t i = 0; i < cfg->Frame().NumDLSyms(); i++) {
    size_t total_data_symbol_id = cfg->GetTotalDataSymbolIdxDl(frame_id, i);

    for (size_t ant_id = 0; ant_id < cfg->BsAntNum(); ant_id++) {
      size_t offset = total_data_symbol_id * cfg->BsAntNum() + ant_id;
      auto* pkt = reinterpret_cast<Packet*>(
          &dl_socket_buffer_[offset * cfg->DlPacketLength()]);
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
  *ptr = (float*)&equal_buffer_[offset][0];
  *size = cfg->UeAntNum() * cfg->OfdmDataNum() * 2;
}
void Agora::CheckIncrementScheduleFrame(size_t frame_id,
                                        ScheduleProcessingFlags completed) {
  this->schedule_process_flags_ += completed;
  assert(this->cur_sche_frame_id_ == frame_id);
  unused(frame_id);

  if (this->schedule_process_flags_ ==
      static_cast<uint8_t>(ScheduleProcessingFlags::kProcessingComplete)) {
    this->cur_sche_frame_id_++;
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
    assert(frame_id == this->cur_proc_frame_id_);
    if (true == kUplinkHardDemod) {
      this->demul_counters_.Reset(frame_id);
    }
    this->decode_counters_.Reset(frame_id);
    this->tomac_counters_.Reset(frame_id);
    this->ifft_counters_.Reset(frame_id);
    this->tx_counters_.Reset(frame_id);
    if (config_->Frame().NumDLSyms() > 0) {
      for (size_t ue_id = 0; ue_id < config_->UeAntNum(); ue_id++) {
        this->dl_bits_buffer_status_[ue_id][frame_id % kFrameWnd] = 0;
      }
    }
    this->cur_proc_frame_id_++;

    if (frame_id == (this->config_->FramesToTest() - 1)) {
      finished = true;
    } else {
      // Only schedule up to kScheduleQueues so we don't flood the queues
      // Cannot access the front() element if the queue is empty
      for (size_t encode = 0;
           (encode < kScheduleQueues) && (!encode_deferral_.empty());
           encode++) {
        const size_t deferred_frame = this->encode_deferral_.front();
        if (deferred_frame < (this->cur_proc_frame_id_ + kScheduleQueues)) {
          if (kDebugDeferral) {
            AGORA_LOG_INFO("   +++ Scheduling deferred frame %zu : %zu \n",
                           deferred_frame, cur_proc_frame_id_);
          }
          RtAssert(deferred_frame >= this->cur_proc_frame_id_,
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