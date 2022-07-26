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

/*
 * Start Agora process
 */

Agora::Agora(Config* const cfg)
    : config_(cfg),
      base_worker_core_offset_(cfg->CoreOffset() + 1 + cfg->SocketThreadNum()),
      stats_(std::make_unique<Stats>(cfg)),
      phy_stats_(std::make_unique<PhyStats>(cfg, Direction::kUplink)) {
  std::string directory = TOSTRING(PROJECT_DIRECTORY);
  AGORA_LOG_INFO("Agora: project directory [%s], RDTSC frequency = %.2f GHz\n",
                 directory.c_str(), config_->FreqGhz());

  // Initialization
  InitializeQueues();
  InitializeUplinkBuffers();
  InitializeDownlinkBuffers();
  InitializeThreads();

  // Manager class
  std::unique_ptr<Manager> manager_cli = std::make_unique<Manager>(
      config_, stats_.get(), phy_stats_.get(), queues_, buffers_, frame_info_);
  frame_info_->cur_sche_frame_id = 0;
  frame_info_->cur_proc_frame_id = 0;

  // // Worker class
  worker_cli = std::make_unique<Worker>(config_, stats_.get(), phy_stats_.get(),
                                        queues_, buffers_, frame_info_);

  // Start scheduling
  // manager_cli->Start();
}

Agora::~Agora() {
  if (kEnableMac) {
    threads_->mac_std_thread.join();
  }

  FreeBuffers();
  FreeQueues();
  stats_.reset();
  phy_stats_.reset();
}

void Agora::Stop() {
  AGORA_LOG_INFO("Agora: terminating\n");
  config_->Running(false);
  usleep(1000);
  packet_tx_rx_.reset();
}

void Agora::Start() {}

void Agora::FreeBuffers() {
  buffers_->socket_buffer.Free();
  buffers_->data_buffer.Free();
  buffers_->equal_buffer.Free();
  buffers_->ue_spec_pilot_buffer.Free();
  if (config_->Frame().NumDLSyms() > 0) {
    FreeBuffer1d(&buffers_->dl_socket_buffer);
    buffers_->dl_ifft_buffer.Free();
    buffers_->calib_dl_buffer.Free();
    buffers_->calib_ul_buffer.Free();
    buffers_->calib_dl_msum_buffer.Free();
    buffers_->calib_ul_msum_buffer.Free();
    buffers_->dl_mod_bits_buffer.Free();
    buffers_->dl_bits_buffer.Free();
    buffers_->dl_bits_buffer_status.Free();
  }
}

void Agora::FreeQueues() {
  // remove tokens for each doer
  for (auto& vec : sched_info_arr_) {
    for (auto& s : vec) {
      delete s.ptok;
    }
  }

  for (size_t i = 0; i < config_->SocketThreadNum(); i++) {
    delete queues_->rx_ptoks_ptr[i];
    delete queues_->tx_ptoks_ptr[i];
  }

  for (size_t i = 0; i < config_->WorkerThreadNum(); i++) {
    for (size_t j = 0; j < kScheduleQueues; j++) {
      delete queues_->worker_ptoks_ptr[i][j];
    }
  }
}

void Agora::InitializeQueues() {
  using mt_queue_t = moodycamel::ConcurrentQueue<EventData>;

  int data_symbol_num_perframe = config_->Frame().NumDataSyms();
  queues_->message_queue =
      mt_queue_t(kDefaultMessageQueueSize * data_symbol_num_perframe);
  for (auto& c : queues_->complete_task_queue) {
    c = mt_queue_t(kDefaultWorkerQueueSize * data_symbol_num_perframe);
  }
  // Create concurrent queues for each Doer
  for (auto& vec : sched_info_arr_) {
    for (auto& s : vec) {
      s.concurrent_q =
          mt_queue_t(kDefaultWorkerQueueSize * data_symbol_num_perframe);
      s.ptok = new moodycamel::ProducerToken(s.concurrent_q);
    }
  }

  for (size_t i = 0; i < config_->SocketThreadNum(); i++) {
    queues_->rx_ptoks_ptr[i] =
        new moodycamel::ProducerToken(queues_->message_queue);
    queues_->tx_ptoks_ptr[i] = new moodycamel::ProducerToken(
        *GetConq(sched_info_arr_, EventType::kPacketTX, 0));
  }

  for (size_t i = 0; i < config_->WorkerThreadNum(); i++) {
    for (size_t j = 0; j < kScheduleQueues; j++) {
      queues_->worker_ptoks_ptr[i][j] =
          new moodycamel::ProducerToken(queues_->complete_task_queue[j]);
    }
  }
}

void Agora::InitializeUplinkBuffers() {
  const size_t task_buffer_symbol_num_ul =
      config_->Frame().NumULSyms() * kFrameWnd;
  buffers_->socket_buffer_size = config_->PacketLength() * config_->BsAntNum() *
                                 kFrameWnd * config_->Frame().NumTotalSyms();

  buffers_->socket_buffer.Malloc(config_->SocketThreadNum() /* RX */,
                                 buffers_->socket_buffer_size,
                                 Agora_memory::Alignment_t::kAlign64);

  buffers_->data_buffer.Malloc(task_buffer_symbol_num_ul,
                               config_->OfdmDataNum() * config_->BsAntNum(),
                               Agora_memory::Alignment_t::kAlign64);

  buffers_->equal_buffer.Malloc(task_buffer_symbol_num_ul,
                                config_->OfdmDataNum() * config_->UeAntNum(),
                                Agora_memory::Alignment_t::kAlign64);
  buffers_->ue_spec_pilot_buffer.Calloc(
      kFrameWnd, config_->Frame().ClientUlPilotSymbols() * config_->UeAntNum(),
      Agora_memory::Alignment_t::kAlign64);

  counters_->rx_counters.num_pilot_pkts_per_frame_ =
      config_->BsAntNum() * config_->Frame().NumPilotSyms();
  // BfAntNum() for each 'L' symbol (no ref node)
  // RefRadio * NumChannels() for each 'C'.
  //rx_counters_.num_reciprocity_pkts_per_frame_ = config_->BsAntNum();
  const size_t num_rx_ul_cal_antennas = config_->BfAntNum();
  // Same as the number of rx reference antennas (ref ant + other channels)
  const size_t num_rx_dl_cal_antennas =
      config_->BsAntNum() - config_->BfAntNum();

  counters_->rx_counters.num_reciprocity_pkts_per_frame_ =
      (config_->Frame().NumULCalSyms() * num_rx_ul_cal_antennas) +
      (config_->Frame().NumDLCalSyms() * num_rx_dl_cal_antennas);

  AGORA_LOG_INFO("Agora: Total recip cal receive symbols per frame: %zu\n",
                 counters_->rx_counters.num_reciprocity_pkts_per_frame_);

  counters_->rx_counters.num_rx_pkts_per_frame_ =
      counters_->rx_counters.num_pilot_pkts_per_frame_ +
      counters_->rx_counters.num_reciprocity_pkts_per_frame_ +
      (config_->BsAntNum() * config_->Frame().NumULSyms());

  frame_info_->fft_created_count = 0;
  counters_->pilot_fft_counters.Init(config_->Frame().NumPilotSyms(),
                                     config_->BsAntNum());
  counters_->uplink_fft_counters.Init(config_->Frame().NumULSyms(),
                                      config_->BsAntNum());
  frame_info_->fft_cur_frame_for_symbol =
      std::vector<size_t>(config_->Frame().NumULSyms(), SIZE_MAX);

  counters_->rc_counters.Init(config_->BsAntNum());

  counters_->zf_counters.Init(config_->ZfEventsPerSymbol());

  counters_->demul_counters.Init(config_->Frame().NumULSyms(),
                                 config_->DemulEventsPerSymbol());

  counters_->decode_counters.Init(
      config_->Frame().NumULSyms(),
      config_->LdpcConfig(Direction::kUplink).NumBlocksInSymbol() *
          config_->UeAntNum());

  counters_->tomac_counters.Init(config_->Frame().NumULSyms(),
                                 config_->UeAntNum());
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
    AllocBuffer1d(&buffers_->dl_socket_buffer, dl_socket_buffer_size,
                  Agora_memory::Alignment_t::kAlign64, 0);

    size_t dl_bits_buffer_size =
        kFrameWnd * config_->MacBytesNumPerframe(Direction::kDownlink);
    buffers_->dl_bits_buffer.Calloc(config_->UeAntNum(), dl_bits_buffer_size,
                                    Agora_memory::Alignment_t::kAlign64);
    buffers_->dl_bits_buffer_status.Calloc(config_->UeAntNum(), kFrameWnd,
                                           Agora_memory::Alignment_t::kAlign64);

    buffers_->dl_ifft_buffer.Calloc(
        config_->BsAntNum() * task_buffer_symbol_num, config_->OfdmCaNum(),
        Agora_memory::Alignment_t::kAlign64);
    buffers_->calib_dl_buffer.Malloc(
        kFrameWnd, config_->BfAntNum() * config_->OfdmDataNum(),
        Agora_memory::Alignment_t::kAlign64);
    buffers_->calib_ul_buffer.Malloc(
        kFrameWnd, config_->BfAntNum() * config_->OfdmDataNum(),
        Agora_memory::Alignment_t::kAlign64);
    buffers_->calib_dl_msum_buffer.Malloc(
        kFrameWnd, config_->BfAntNum() * config_->OfdmDataNum(),
        Agora_memory::Alignment_t::kAlign64);
    buffers_->calib_ul_msum_buffer.Malloc(
        kFrameWnd, config_->BfAntNum() * config_->OfdmDataNum(),
        Agora_memory::Alignment_t::kAlign64);
    //initialize the calib buffers
    const complex_float complex_init = {0.0f, 0.0f};
    //const complex_float complex_init = {1.0f, 0.0f};
    for (size_t frame = 0u; frame < kFrameWnd; frame++) {
      for (size_t i = 0; i < (config_->OfdmDataNum() * config_->BfAntNum());
           i++) {
        buffers_->calib_dl_buffer[frame][i] = complex_init;
        buffers_->calib_ul_buffer[frame][i] = complex_init;
        buffers_->calib_dl_msum_buffer[frame][i] = complex_init;
        buffers_->calib_ul_msum_buffer[frame][i] = complex_init;
      }

      buffers_->dl_mod_bits_buffer.Calloc(
          task_buffer_symbol_num,
          Roundup<64>(config_->GetOFDMDataNum()) * config_->UeAntNum(),
          Agora_memory::Alignment_t::kAlign64);

      counters_->encode_counters.Init(
          config_->Frame().NumDlDataSyms(),
          config_->LdpcConfig(Direction::kDownlink).NumBlocksInSymbol() *
              config_->UeAntNum());
      frame_info_->encode_cur_frame_for_symbol =
          std::vector<size_t>(config_->Frame().NumDLSyms(), SIZE_MAX);
      frame_info_->ifft_cur_frame_for_symbol =
          std::vector<size_t>(config_->Frame().NumDLSyms(), SIZE_MAX);
      counters_->precode_counters.Init(config_->Frame().NumDLSyms(),
                                       config_->DemulEventsPerSymbol());
      // precode_cur_frame_for_symbol_ =
      //    std::vector<size_t>(config_->Frame().NumDLSyms(), SIZE_MAX);
      counters_->ifft_counters.Init(config_->Frame().NumDLSyms(),
                                    config_->BsAntNum());
      counters_->tx_counters.Init(config_->Frame().NumDLSyms(),
                                  config_->BsAntNum());
      // mac data is sent per frame, so we set max symbol to 1
      counters_->mac_to_phy_counters.Init(1, config_->UeAntNum());
    }
  }
}

void Agora::InitializeThreads() {
  /* Initialize TXRX threads */
  if (kUseArgos || kUseUHD) {
    packet_tx_rx_ = std::make_unique<PacketTxRxRadio>(
        config_, config_->CoreOffset() + 1, &queues_->message_queue,
        GetConq(sched_info_arr_, EventType::kPacketTX, 0),
        queues_->rx_ptoks_ptr, queues_->tx_ptoks_ptr, buffers_->socket_buffer,
        buffers_->socket_buffer_size / config_->PacketLength(),
        stats_->FrameStart(), buffers_->dl_socket_buffer);
#if defined(USE_DPDK)
  } else if (kUseDPDK) {
    packet_tx_rx_ = std::make_unique<PacketTxRxDpdk>(
        config_, config_->CoreOffset() + 1, &queues_->message_queue,
        GetConq(sched_info_arr_, EventType::kPacketTX, 0),
        queues_->rx_ptoks_ptr, queues_->tx_ptoks_ptr, buffers_->socket_buffer,
        buffers_->socket_buffer_size / config_->PacketLength(),
        stats_->FrameStart(), buffers_->dl_socket_buffer);
#endif
  } else {
    /* Default to the simulator */
    packet_tx_rx_ = std::make_unique<PacketTxRxSim>(
        config_, config_->CoreOffset() + 1, &queues_->message_queue,
        GetConq(sched_info_arr_, EventType::kPacketTX, 0),
        queues_->rx_ptoks_ptr, queues_->tx_ptoks_ptr, buffers_->socket_buffer,
        buffers_->socket_buffer_size / config_->PacketLength(),
        stats_->FrameStart(), buffers_->dl_socket_buffer);
  }

  if (kEnableMac == true) {
    const size_t mac_cpu_core = config_->CoreOffset() +
                                config_->SocketThreadNum() +
                                config_->WorkerThreadNum() + 1;
    threads_->mac_thread = std::make_unique<MacThreadBaseStation>(
        config_, mac_cpu_core, buffers_->decoded_buffer,
        &buffers_->dl_bits_buffer, &buffers_->dl_bits_buffer_status,
        &queues_->mac_request_queue, &queues_->mac_response_queue);

    threads_->mac_std_thread = std::thread(&MacThreadBaseStation::RunEventLoop,
                                           threads_->mac_thread.get());
  }

  // // Create worker threads
  // CreateThreads();

  AGORA_LOG_INFO(
      "Master thread core %zu, TX/RX thread cores %zu--%zu, worker thread "
      "cores %zu--%zu\n",
      config_->CoreOffset(), config_->CoreOffset() + 1,
      config_->CoreOffset() + 1 + config_->SocketThreadNum() - 1,
      base_worker_core_offset_,
      base_worker_core_offset_ + config_->WorkerThreadNum() - 1);
}

void Agora::GetEqualData(float** ptr, int* size) {
  const auto& cfg = config_;
  auto offset = cfg->GetTotalDataSymbolIdxUl(
      max_equaled_frame_, cfg->Frame().ClientUlPilotSymbols());
  *ptr = (float*)&equal_buffer_[offset][0];
  *size = cfg->UeAntNum() * cfg->OfdmDataNum() * 2;
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