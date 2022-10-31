/**
 * @file phy-ue.cc
 * @brief Implementation file for the phy ue class
 */
#include "phy-ue.h"

#include <memory>
#include <vector>

#include "logger.h"
#include "message.h"
#include "packet_txrx_client_radio.h"
#include "packet_txrx_client_sim.h"
#include "phy_ldpc_decoder_5gnr.h"
#include "phy_stats.h"
#include "recorder_thread.h"
#include "scrambler.h"
#include "signal_handler.h"
#include "utils_ldpc.h"

/* Print debug work */
static constexpr bool kDebugPrintPacketsFromMac = false;
static constexpr bool kDebugPrintPacketsToMac = false;
static constexpr size_t kDefaultQueueSize = 36;

//set the number of subcarriers to record DL CSI
static constexpr size_t kNumRecSc = 4;

//Recording parameters
static constexpr size_t kRecordFrameInterval = 1;
#if defined(ENABLE_HDF5)
static constexpr bool kRecordDownlinkFrame = true;

//set the recording types, can add multiple
static const std::vector<Agora_recorder::RecorderWorker::RecorderWorkerTypes>
    kRecorderTypes{Agora_recorder::RecorderWorker::RecorderWorkerTypes::
                       kRecorderWorkerHdf5};
#else
static constexpr bool kRecordDownlinkFrame = false;

//set the recording types, can add multiple
static const std::vector<Agora_recorder::RecorderWorker::RecorderWorkerTypes>
    kRecorderTypes{Agora_recorder::RecorderWorker::RecorderWorkerTypes::
                       kRecorderWorkerMultiFile};
#endif

PhyUe::PhyUe(Config* config)
    : stats_(std::make_unique<Stats>(config)),
      phy_stats_(std::make_unique<PhyStats>(config, Direction::kDownlink)),
      demod_buffer_(kFrameWnd, config->Frame().NumDLSyms(), config->UeAntNum(),
                    kMaxModType * Roundup<64>(config->GetOFDMDataNum())),
      decoded_buffer_(
          kFrameWnd, config->Frame().NumDLSyms(), config->UeAntNum(),
          config->LdpcConfig(Direction::kDownlink).NumBlocksInSymbol() *
              Roundup<64>(config->NumBytesPerCb(Direction::kDownlink))) {
  srand(time(nullptr));
  // TODO take into account the UeAntOffset to allow for multiple PhyUe
  // instances
  this->config_ = config;
  InitializeVarsFromCfg();

  for (size_t i = config_->OfdmDataStart();
       i < config_->OfdmDataStart() + config_->OfdmDataNum(); i++) {
    non_null_sc_ind_.push_back(i);
  }

  ue_pilot_vec_.resize(config_->UeAntNum());
  for (size_t i = 0; i < config_->UeAntNum(); i++) {
    const size_t pilot_len_samples =
        config_->SampsPerSymbol() -
        (config->OfdmTxZeroPostfix() + config->OfdmTxZeroPrefix());
    auto& ue_pilot_f = ue_pilot_vec_.at(i);
    ue_pilot_f.resize(pilot_len_samples);
    ConvertShortToFloat(
        reinterpret_cast<const short*>(
            &config_->UeSpecificPilotT()[i][config->OfdmTxZeroPrefix()]),
        reinterpret_cast<float*>(ue_pilot_f.data()), pilot_len_samples * 2);
  }

  complete_queue_ = moodycamel::ConcurrentQueue<EventData>(
      kFrameWnd * config_->Frame().NumTotalSyms() * config_->UeAntNum() *
      kDefaultQueueSize);
  work_queue_ = moodycamel::ConcurrentQueue<EventData>(
      kFrameWnd * config_->Frame().NumTotalSyms() * config_->UeAntNum() *
      kDefaultQueueSize);
  tx_queue_ = moodycamel::ConcurrentQueue<EventData>(
      kFrameWnd * config_->UeAntNum() * kDefaultQueueSize);
  to_mac_queue_ = moodycamel::ConcurrentQueue<EventData>(
      kFrameWnd * config_->UeAntNum() * kDefaultQueueSize);

  for (size_t i = 0; i < rx_thread_num_; i++) {
    rx_ptoks_ptr_[i] = new moodycamel::ProducerToken(complete_queue_);
    tx_ptoks_ptr_[i] = new moodycamel::ProducerToken(tx_queue_);
    mac_rx_ptoks_ptr_[i] = new moodycamel::ProducerToken(complete_queue_);
    mac_tx_ptoks_ptr_[i] = new moodycamel::ProducerToken(to_mac_queue_);
  }

  work_producer_token_ =
      std::make_unique<moodycamel::ProducerToken>(work_queue_);

  // uplink buffers init (tx)
  InitializeUplinkBuffers();
  // downlink buffers init (rx)
  InitializeDownlinkBuffers();

  if (kUseArgos) {
    ru_ = std::make_unique<PacketTxRxClientRadio>(
        config_, config_->UeCoreOffset() + 1, &complete_queue_, &tx_queue_,
        rx_ptoks_ptr_, tx_ptoks_ptr_, rx_buffer_,
        rx_buffer_size_ / config->PacketLength(), stats_->FrameStart(),
        tx_buffer_);
    //} else if (kUseUHD) {
  } else {
    ru_ = std::make_unique<PacketTxRxClientSim>(
        config_, config_->UeCoreOffset() + 1, &complete_queue_, &tx_queue_,
        rx_ptoks_ptr_, tx_ptoks_ptr_, rx_buffer_,
        rx_buffer_size_ / config->PacketLength(), stats_->FrameStart(),
        tx_buffer_);
  }

  size_t core_offset_worker = config_->UeCoreOffset() + 1 + rx_thread_num_;
  if (kEnableMac == true) {
    mac_thread_ = std::make_unique<MacThreadClient>(
        config_, core_offset_worker, decoded_buffer_, &ul_bits_buffer_,
        &ul_bits_buffer_status_, &to_mac_queue_, &complete_queue_);

    core_offset_worker++;
    mac_std_thread_ =
        std::thread(&MacThreadClient::RunEventLoop, mac_thread_.get());
  }

  for (size_t i = 0; i < config_->UeWorkerThreadNum(); i++) {
    auto new_worker = std::make_unique<UeWorker>(
        i, *config_, *stats_, *phy_stats_, complete_queue_, work_queue_,
        *work_producer_token_.get(), ul_bits_buffer_, ul_syms_buffer_,
        modul_buffer_, ifft_buffer_, tx_buffer_, rx_buffer_, csi_buffer_,
        equal_buffer_, non_null_sc_ind_, fft_buffer_, demod_buffer_,
        decoded_buffer_, ue_pilot_vec_);

    new_worker->Start(core_offset_worker);
    workers_.push_back(std::move(new_worker));
  }

  if (kRecordDownlinkFrame && config_->Frame().NumDLSyms() > 0) {
    auto& new_recorder = recorders_.emplace_back(
        std::make_unique<Agora_recorder::RecorderThread>(
            config_, 0, core_offset_worker + config_->UeWorkerThreadNum(),
            kFrameWnd * config_->Frame().NumTotalSyms() * config_->UeAntNum() *
                kDefaultQueueSize,
            0, config_->UeAntNum(), kRecordFrameInterval, Direction::kDownlink,
            kRecorderTypes, true));
    new_recorder->Start();
  }

  // initilize all kinds of checkers
  // Init the frame work tracking structure
  for (size_t frame = 0; frame < this->frame_tasks_.size(); frame++) {
    FrameInit(frame);
  }
  decode_counters_.Init(dl_data_symbol_perframe_, config_->UeAntNum());
  demul_counters_.Init(dl_data_symbol_perframe_, config_->UeAntNum());
  fft_dlpilot_counters_.Init(config->Frame().ClientDlPilotSymbols(),
                             config_->UeAntNum());
  fft_dldata_counters_.Init(dl_data_symbol_perframe_, config_->UeAntNum());

  /* Each UE / Radio will send a TxComplete */
  tx_counters_.Init(config_->UeAntNum());
  encode_counter_.Init(ul_data_symbol_perframe_, config_->UeAntNum());
  modulation_counters_.Init(ul_data_symbol_perframe_, config_->UeAntNum());

  const size_t num_ue = config_->UeNum();
  ue_tracker_.reserve(num_ue);
  ue_tracker_.resize(num_ue);
  for (auto& ue : ue_tracker_) {
    ue.ifft_counters_.Init(ul_symbol_perframe_, config_->NumUeChannels());
    ue.tx_pending_frame_ = 0;
    ue.tx_ready_frames_.clear();
  }

  // This usage doesn't effect the user num_reciprocity_pkts_per_frame_;
  rx_counters_.num_rx_pkts_per_frame_ =
      config_->UeAntNum() *
      (config_->Frame().NumDLSyms() + config_->Frame().NumBeaconSyms());
  rx_counters_.num_pilot_pkts_per_frame_ =
      config_->UeAntNum() * config_->Frame().ClientDlPilotSymbols();

  rx_downlink_deferral_.resize(kFrameWnd);

  // Mac counters for downlink data
  tomac_counters_.Init(config_->Frame().NumDlDataSyms(), config_->UeAntNum());
}

PhyUe::~PhyUe() {
  for (size_t i = 0; i < config_->UeWorkerThreadNum(); i++) {
    AGORA_LOG_INFO("Joining Phy worker: %zu : %zu\n", i,
                   config_->UeWorkerThreadNum());
    workers_.at(i)->Stop();
  }
  workers_.clear();

  for (size_t i = 0; i < recorders_.size(); i++) {
    AGORA_LOG_INFO("Waiting for Recording to complete %zu\n", i);
    recorders_.at(i)->Stop();
  }
  recorders_.clear();

  if (kEnableMac == true) {
    mac_std_thread_.join();
  }

  for (size_t i = 0; i < rx_thread_num_; i++) {
    delete rx_ptoks_ptr_[i];
    delete tx_ptoks_ptr_[i];
    delete mac_rx_ptoks_ptr_[i];
    delete mac_tx_ptoks_ptr_[i];
  }

  FreeUplinkBuffers();
  FreeDownlinkBuffers();
}

void PhyUe::ScheduleTask(EventData do_task,
                         moodycamel::ConcurrentQueue<EventData>* in_queue,
                         moodycamel::ProducerToken const& ptok) {
  if (in_queue->try_enqueue(ptok, do_task) == false) {
    AGORA_LOG_INFO("PhyUe: Cannot enqueue task, need more memory");
    if (in_queue->enqueue(ptok, do_task) == false) {
      AGORA_LOG_INFO("PhyUe: task enqueue failed\n");
      throw std::runtime_error("PhyUe: task enqueue failed");
    }
  }
}

void PhyUe::ScheduleWork(EventData do_task) {
  if (work_queue_.try_enqueue(*(work_producer_token_.get()), do_task) ==
      false) {
    AGORA_LOG_INFO("PhyUe: Cannot enqueue work task, need more memory");
    if (work_queue_.enqueue(*(work_producer_token_.get()), do_task) == false) {
      AGORA_LOG_INFO("PhyUe: work task enqueue failed\n");
      throw std::runtime_error("PhyUe: work task enqueue failed");
    }
  }
}

void PhyUe::ReceiveDownlinkSymbol(Packet* rx_packet, size_t tag) {
  const size_t frame_slot = rx_packet->frame_id_ % kFrameWnd;
  const size_t dl_symbol_idx =
      config_->Frame().GetDLSymbolIdx(rx_packet->symbol_id_);

  // if symbol is a pilot or we are finished with all pilot ffts for the given
  // frame
  if (dl_symbol_idx < config_->Frame().ClientDlPilotSymbols()) {
    ScheduleWork(EventData(EventType::kFFTPilot, tag));
  } else if (fft_dlpilot_counters_.IsLastSymbol(rx_packet->frame_id_)) {
    ScheduleWork(EventData(EventType::kFFT, tag));
  } else {
    std::queue<EventData>* defferal_queue =
        &rx_downlink_deferral_.at(frame_slot);

    defferal_queue->push(EventData(EventType::kFFT, tag));
  }
}

void PhyUe::ScheduleDefferedDownlinkSymbols(size_t frame_id) {
  const size_t frame_slot = frame_id % kFrameWnd;
  // Complete the csi offset
  const size_t csi_offset_base = frame_slot * config_->UeAntNum();

  for (size_t user = 0; user < config_->UeAntNum(); user++) {
    const size_t csi_offset = csi_offset_base + user;

    for (size_t ofdm_data = 0; ofdm_data < config_->OfdmDataNum();
         ofdm_data++) {
      auto* csi_buffer_ptr =
          reinterpret_cast<arma::cx_float*>(csi_buffer_[csi_offset]);

      csi_buffer_ptr[ofdm_data] /= dl_pilot_symbol_perframe_;
    }
  }
  std::queue<EventData>* defferal_queue = &rx_downlink_deferral_.at(frame_slot);

  while (!defferal_queue->empty()) {
    ScheduleWork(defferal_queue->front());
    defferal_queue->pop();
  }
}

void PhyUe::ClearCsi(size_t frame_id) {
  const size_t frame_slot = frame_id % kFrameWnd;

  if (config_->Frame().ClientDlPilotSymbols() > 0) {
    const size_t csi_offset_base = frame_slot * config_->UeAntNum();
    for (size_t user = 0; user < config_->UeAntNum(); user++) {
      const size_t csi_offset = csi_offset_base + user;
      for (size_t ofdm_data = 0u; ofdm_data < config_->OfdmDataNum();
           ofdm_data++) {
        auto* csi_buffer_ptr =
            reinterpret_cast<arma::cx_float*>(csi_buffer_[csi_offset]);

        csi_buffer_ptr[ofdm_data] = 0;
      }
    }
    fft_dlpilot_counters_.Reset(frame_id);
  }  // Only do work if there are DL pilot symbols
  assert(rx_downlink_deferral_.at(frame_slot).empty() == true);
}

void PhyUe::Stop() {
  AGORA_LOG_INFO("PhyUe: Stopping threads\n");
  config_->Running(false);
  usleep(1000);
  ru_.reset();
}

void PhyUe::Start() {
  PinToCoreWithOffset(ThreadType::kMaster, config_->UeCoreOffset(), 0);
  Table<complex_float> calib_buffer;
  calib_buffer.Malloc(kFrameWnd, config_->UeAntNum() * config_->OfdmDataNum(),
                      Agora_memory::Alignment_t::kAlign64);

  const bool start_status = ru_->StartTxRx(calib_buffer, calib_buffer);
  calib_buffer.Free();
  if (start_status == false) {
    this->Stop();
    return;
  }

  // for task_queue, main thread is producer, it is single-producer &
  // multiple consumer for task queue uplink
  moodycamel::ProducerToken ptok_mac(to_mac_queue_);

  // for message_queue, main thread is a consumer, it is multiple
  // producers & single consumer for message_queue
  moodycamel::ConsumerToken ctok(complete_queue_);

  // counter for print log
  size_t miss_count = 0;
  size_t total_count = 0;

  std::array<EventData, kDequeueBulkSizeTXRX> events_list;
  size_t ret = 0;
  max_equaled_frame_ = 0;
  size_t cur_frame_id = 0;

  while ((config_->Running() == true) &&
         (SignalHandler::GotExitSignal() == false)) {
    // get a bulk of events
    ret = complete_queue_.try_dequeue_bulk(ctok, events_list.data(),
                                           kDequeueBulkSizeTXRX);
    total_count++;
    if (total_count == 1e7) {
      total_count = 0;
      miss_count = 0;
    }
    if (ret == 0) {
      miss_count++;
      continue;
    }
    // handle each event
    for (size_t bulk_count = 0; bulk_count < ret; bulk_count++) {
      EventData& event = events_list.at(bulk_count);

      switch (event.event_type_) {
        case EventType::kPacketRX: {
          RxPacket* rx = rx_tag_t(event.tags_[0u]).rx_packet_;
          Packet* pkt = rx->RawPacket();

          if (recorders_.size() == 1) {
            rx->Use();
            recorders_.at(0)->DispatchWork(event);
          }

          const size_t frame_id = pkt->frame_id_;
          const size_t symbol_id = pkt->symbol_id_;
          const size_t ant_id = pkt->ant_id_;
          const size_t frame_slot = frame_id % kFrameWnd;
          RtAssert(pkt->frame_id_ < (cur_frame_id + kFrameWnd),
                   "Error: Received packet for future frame beyond frame "
                   "window. This can happen if PHY is running "
                   "slowly, e.g., in debug mode");

          PrintPerTaskDone(PrintType::kPacketRX, frame_id, symbol_id, ant_id);

          if (rx_counters_.num_pkts_.at(frame_slot) == 0) {
            this->stats_->MasterSetTsc(TsType::kFirstSymbolRX, frame_id);
            if (kDebugPrintPerFrameStart) {
              const size_t prev_frame_slot =
                  (frame_slot + kFrameWnd - 1) % kFrameWnd;
              AGORA_LOG_INFO(
                  "PhyUe [frame %zu + %.2f ms since last frame]: Received "
                  "first packet. Remaining packets in prev frame: %zu\n",
                  frame_id,
                  this->stats_->MasterGetDeltaMs(TsType::kFirstSymbolRX,
                                                 frame_id, frame_id - 1),
                  rx_counters_.num_pkts_.at(prev_frame_slot));
            }
          }

          if (config_->IsDlPilot(frame_id, symbol_id)) {
            rx_counters_.num_pilot_pkts_.at(frame_slot)++;
            if (rx_counters_.num_pilot_pkts_.at(frame_slot) ==
                rx_counters_.num_pilot_pkts_per_frame_) {
              rx_counters_.num_pilot_pkts_.at(frame_slot) = 0;
              stats_->MasterSetTsc(TsType::kPilotAllRX, frame_id);
              PrintPerFrameDone(PrintType::kPacketRXPilots, frame_id);
            }
          }
          rx_counters_.num_pkts_.at(frame_slot)++;
          if (rx_counters_.num_pkts_.at(frame_slot) ==
              rx_counters_.num_rx_pkts_per_frame_) {
            stats_->MasterSetTsc(TsType::kRXDone, frame_id);
            PrintPerFrameDone(PrintType::kPacketRX, frame_id);
            rx_counters_.num_pkts_.at(frame_slot) = 0;
          }

          // Schedule uplink pilots transmission and uplink processing
          if (symbol_id == config_->Frame().GetBeaconSymbolLast()) {
            // Schedule Pilot after receiving last beacon
            // (Only when in Downlink Only mode, otherwise the pilots
            // will be transmitted with the uplink data)
            if (ul_symbol_perframe_ == 0) {
              EventData do_tx_pilot_task(
                  EventType::kPacketPilotTX,
                  gen_tag_t::FrmSymUe(
                      frame_id, config_->Frame().GetPilotSymbol(ant_id), ant_id)
                      .tag_);
              ScheduleTask(do_tx_pilot_task, &tx_queue_,
                           *tx_ptoks_ptr_[ru_->AntNumToWorkerId(ant_id)]);
            } else {
              // Schedule the Uplink tasks
              for (size_t symbol_idx = 0;
                   symbol_idx < config_->Frame().NumULSyms(); symbol_idx++) {
                if (symbol_idx < config_->Frame().ClientUlPilotSymbols()) {
                  EventData do_ifft_task(
                      EventType::kIFFT,
                      gen_tag_t::FrmSymUe(
                          frame_id, config_->Frame().GetULSymbol(symbol_idx),
                          ant_id)
                          .tag_);
                  ScheduleWork(do_ifft_task);
                } else {
                  EventData do_encode_task(
                      EventType::kEncode,
                      gen_tag_t::FrmSymUe(
                          frame_id, config_->Frame().GetULSymbol(symbol_idx),
                          ant_id)
                          .tag_);
                  ScheduleWork(do_encode_task);
                }
              }  // For all UL Symbols
            }
          }

          SymbolType symbol_type = config_->GetSymbolType(symbol_id);
          if (symbol_type == SymbolType::kDL) {
            // Defer downlink processing (all pilot symbols must be fft'd
            // first)
            ReceiveDownlinkSymbol(pkt, event.tags_[0]);
          } else {
            rx->Free();
          }
        } break;

        case EventType::kFFTPilot: {
          const size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          const size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;
          const size_t ant_id = gen_tag_t(event.tags_[0]).ant_id_;

          PrintPerTaskDone(PrintType::kFFTPilots, frame_id, symbol_id, ant_id);
          const bool tasks_complete =
              fft_dlpilot_counters_.CompleteTask(frame_id, symbol_id);
          if (tasks_complete) {
            PrintPerSymbolDone(PrintType::kFFTPilots, frame_id, symbol_id);
            const bool pilot_fft_complete =
                fft_dlpilot_counters_.CompleteSymbol(frame_id);
            if (pilot_fft_complete) {
              if (kPrintPhyStats) {
                this->phy_stats_->PrintDlSnrStats(frame_id);
              }
              this->phy_stats_->RecordDlCsi(frame_id, kNumRecSc, csi_buffer_);
              this->phy_stats_->RecordDlPilotSnr(frame_id);
              this->stats_->MasterSetTsc(TsType::kFFTPilotsDone, frame_id);
              PrintPerFrameDone(PrintType::kFFTPilots, frame_id);
              ScheduleDefferedDownlinkSymbols(frame_id);
            }
          }
        } break;

        case EventType::kFFT: {
          const size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          const size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;
          const size_t ant_id = gen_tag_t(event.tags_[0]).ant_id_;

          // Schedule the Demul
          EventData do_demul_task(EventType::kDemul, event.tags_[0]);
          ScheduleWork(do_demul_task);

          PrintPerTaskDone(PrintType::kFFTData, frame_id, symbol_id, ant_id);
          const bool tasks_complete =
              fft_dldata_counters_.CompleteTask(frame_id, symbol_id);
          if (tasks_complete == true) {
            PrintPerSymbolDone(PrintType::kFFTData, frame_id, symbol_id);
            bool fft_complete = fft_dldata_counters_.CompleteSymbol(frame_id);
            if (fft_complete == true) {
              this->stats_->MasterSetTsc(TsType::kFFTDone, frame_id);
              PrintPerFrameDone(PrintType::kFFTData, frame_id);
              fft_dldata_counters_.Reset(frame_id);
              // Clear the csi buffer for the next use
              ClearCsi(frame_id);
            }
          }
        } break;

        case EventType::kDemul: {
          const size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          const size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;
          const size_t ant_id = gen_tag_t(event.tags_[0]).ant_id_;

          if (kDownlinkHardDemod == false) {
            EventData do_decode_task(EventType::kDecode, event.tags_[0]);
            ScheduleWork(do_decode_task);
          }

          PrintPerTaskDone(PrintType::kDemul, frame_id, symbol_id, ant_id);
          const bool symbol_complete =
              demul_counters_.CompleteTask(frame_id, symbol_id);
          if (symbol_complete == true) {
            PrintPerSymbolDone(PrintType::kDemul, frame_id, symbol_id);
            max_equaled_frame_ = frame_id;
            bool demul_complete = demul_counters_.CompleteSymbol(frame_id);
            if (demul_complete == true) {
              this->stats_->MasterSetTsc(TsType::kDemulDone, frame_id);
              PrintPerFrameDone(PrintType::kDemul, frame_id);
              demul_counters_.Reset(frame_id);

              this->phy_stats_->RecordEvm(frame_id);
              this->phy_stats_->RecordEvmSnr(frame_id);
              if (kDownlinkHardDemod) {
                this->phy_stats_->RecordBer(frame_id);
                this->phy_stats_->RecordSer(frame_id);
              }
              this->phy_stats_->ClearEvmBuffer(frame_id);

              if (kDownlinkHardDemod == true) {
                bool finished =
                    FrameComplete(frame_id, FrameTasksFlags::kDownlinkComplete);
                if (finished == true) {
                  if ((cur_frame_id + 1) >= config_->FramesToTest()) {
                    config_->Running(false);
                  } else {
                    FrameInit(frame_id);
                    cur_frame_id = frame_id + 1;
                  }
                }
              }
            }
          }
        } break;

        case EventType::kDecode: {
          const size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          const size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;
          const size_t ant_id = gen_tag_t(event.tags_[0]).ant_id_;

          PrintPerTaskDone(PrintType::kDecode, frame_id, symbol_id, ant_id);

          const bool symbol_complete =
              decode_counters_.CompleteTask(frame_id, symbol_id);
          if (symbol_complete == true) {
            if (kEnableMac) {
              auto base_tag = gen_tag_t::FrmSymUe(frame_id, symbol_id, 0);

              for (size_t i = 0; i < config_->UeAntNum(); i++) {
                ScheduleTask(EventData(EventType::kPacketToMac, base_tag.tag_),
                             &to_mac_queue_, ptok_mac);

                base_tag.ue_id_++;
              }
            }
            PrintPerSymbolDone(PrintType::kDecode, frame_id, symbol_id);

            bool decode_complete = decode_counters_.CompleteSymbol(frame_id);
            if (decode_complete == true) {
              this->stats_->MasterSetTsc(TsType::kDecodeDone, frame_id);
              PrintPerFrameDone(PrintType::kDecode, frame_id);
              decode_counters_.Reset(frame_id);
              this->phy_stats_->RecordBer(frame_id);
              this->phy_stats_->RecordSer(frame_id);
              bool finished =
                  FrameComplete(frame_id, FrameTasksFlags::kDownlinkComplete);
              if (finished == true) {
                if ((cur_frame_id + 1) >= config_->FramesToTest()) {
                  config_->Running(false);
                } else {
                  FrameInit(frame_id);
                  cur_frame_id = frame_id + 1;
                }
              }
            }
          }
        } break;

        case EventType::kPacketToMac: {
          const size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          const size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;
          const size_t dl_symbol_idx =
              config_->Frame().GetDLSymbolIdx(symbol_id);

          if (kDebugPrintPacketsToMac) {
            AGORA_LOG_INFO(
                "PhyUe: sent decoded packet for (frame %zu, symbol %zu:%zu) to "
                "MAC\n",
                frame_id, symbol_id, dl_symbol_idx);
          }
          const bool last_tomac_task =
              tomac_counters_.CompleteTask(frame_id, dl_symbol_idx);

          if (last_tomac_task == true) {
            PrintPerSymbolDone(PrintType::kPacketToMac, frame_id, symbol_id);

            const bool last_tomac_symbol =
                tomac_counters_.CompleteSymbol(frame_id);

            if (last_tomac_symbol == true) {
              PrintPerFrameDone(PrintType::kPacketToMac, frame_id);
              tomac_counters_.Reset(frame_id);

              const bool finished =
                  FrameComplete(frame_id, FrameTasksFlags::kMacTxComplete);
              if (finished == true) {
                if ((cur_frame_id + 1) >= config_->FramesToTest()) {
                  config_->Running(false);
                } else {
                  FrameInit(frame_id);
                  cur_frame_id = frame_id + 1;
                }
              }
            }
          }
        } break;

        case EventType::kPacketFromMac: {
          // This is an entire frame (multiple mac packets)
          const size_t ue_id = rx_mac_tag_t(event.tags_[0]).tid_;
          const size_t radio_buf_id = rx_mac_tag_t(event.tags_[0]).offset_;
          RtAssert(radio_buf_id == (expected_frame_id_from_mac_ % kFrameWnd),
                   "Radio buffer id does not match expected");

          const auto* pkt = reinterpret_cast<const MacPacketPacked*>(
              &ul_bits_buffer_[ue_id]
                              [radio_buf_id * config_->MacBytesNumPerframe(
                                                  Direction::kUplink)]);

          AGORA_LOG_TRACE(
              "PhyUe: frame %d symbol %d user %d @ offset %zu %zu @ location "
              "%zu\n",
              pkt->Frame(), pkt->Symbol(), pkt->Ue(), ue_id, radio_buf_id,
              (size_t)pkt);
          RtAssert(pkt->Frame() ==
                       static_cast<uint16_t>(expected_frame_id_from_mac_),
                   "PhyUe: Incorrect frame ID from MAC");
          current_frame_user_num_ =
              (current_frame_user_num_ + 1) % config_->UeAntNum();
          if (current_frame_user_num_ == 0) {
            expected_frame_id_from_mac_++;
          }
#if ENABLE_RB_IND
          config_->UpdateModCfgs(pkt->rb_indicator_.mod_order_bits_);
#endif
          if (kDebugPrintPacketsFromMac) {
#if ENABLE_RB_IND
            AGORA_LOG_INFO(
                "PhyUe: received packet for frame %u with modulation %zu\n",
                pkt->frame_id_, pkt->rb_indicator_.mod_order_bits_);
#endif
            std::stringstream ss;

            for (size_t ul_data_symbol = 0;
                 ul_data_symbol < config_->Frame().NumUlDataSyms();
                 ul_data_symbol++) {
              ss << "PhyUe: kPacketFromMac, frame " << pkt->Frame()
                 << ", symbol " << std::to_string(pkt->Symbol()) << " crc "
                 << std::to_string(pkt->Crc()) << " bytes: ";
              for (size_t i = 0; i < pkt->PayloadLength(); i++) {
                ss << std::to_string((pkt->Data()[i])) << ", ";
              }
              ss << std::endl;
              pkt = reinterpret_cast<const MacPacketPacked*>(
                  reinterpret_cast<const uint8_t*>(pkt) +
                  config_->MacPacketLength(Direction::kUplink));
            }
            AGORA_LOG_INFO("%s\n", ss.str().c_str());
          }
        } break;

        case EventType::kEncode: {
          const size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          const size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;
          const size_t ue_ant = gen_tag_t(event.tags_[0]).ue_id_;

          PrintPerTaskDone(PrintType::kEncode, frame_id, symbol_id, ue_ant);

          // Schedule the modul
          EventData do_modul_task(EventType::kModul, event.tags_[0]);
          ScheduleWork(do_modul_task);

          const bool symbol_complete =
              encode_counter_.CompleteTask(frame_id, symbol_id);
          if (symbol_complete == true) {
            PrintPerSymbolDone(PrintType::kEncode, frame_id, symbol_id);

            const bool encode_complete =
                encode_counter_.CompleteSymbol(frame_id);
            if (encode_complete == true) {
              stats_->MasterSetTsc(TsType::kEncodeDone, frame_id);
              PrintPerFrameDone(PrintType::kEncode, frame_id);
              encode_counter_.Reset(frame_id);
            }
          }
        } break;

        case EventType::kModul: {
          const size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          const size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;
          const size_t ue_ant = gen_tag_t(event.tags_[0]).ue_id_;

          PrintPerTaskDone(PrintType::kModul, frame_id, symbol_id, ue_ant);

          EventData do_ifft_task(
              EventType::kIFFT,
              gen_tag_t::FrmSymUe(frame_id, symbol_id, ue_ant).tag_);
          ScheduleWork(do_ifft_task);

          const bool symbol_complete =
              modulation_counters_.CompleteTask(frame_id, symbol_id);
          if (symbol_complete) {
            PrintPerSymbolDone(PrintType::kModul, frame_id, symbol_id);

            const bool mod_complete =
                modulation_counters_.CompleteSymbol(frame_id);
            if (mod_complete == true) {
              stats_->MasterSetTsc(TsType::kModulDone, frame_id);
              PrintPerFrameDone(PrintType::kModul, frame_id);
              modulation_counters_.Reset(frame_id);
            }
          }
        } break;

        case EventType::kIFFT: {
          const size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          const size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;
          const size_t ue_ant = gen_tag_t(event.tags_[0]).ue_id_;
          const size_t ue_interface = ue_ant / config_->NumUeChannels();

          PrintPerTaskDone(PrintType::kIFFT, frame_id, symbol_id, ue_ant);
          UeTxVars& ue = ue_tracker_.at(ue_interface);

          const bool symbol_complete =
              ue.ifft_counters_.CompleteTask(frame_id, symbol_id);
          if (symbol_complete) {
            PrintPerSymbolDone(PrintType::kIFFT, frame_id, symbol_id);

            const bool ifft_complete =
                ue.ifft_counters_.CompleteSymbol(frame_id);
            if (ifft_complete) {
              stats_->MasterSetTsc(TsType::kIFFTDone, frame_id);
              PrintPerFrameDone(PrintType::kIFFT, frame_id);
              ue.ifft_counters_.Reset(frame_id);

              // If the completed frame is the next in line, schedule the
              // transmission
              if (ue.tx_pending_frame_ == frame_id) {
                size_t current_frame = frame_id;

                while (ue.tx_pending_frame_ == current_frame) {
                  //Schedule Tx for all channels on the UE
                  for (size_t ch = 0; ch < config_->NumUeChannels(); ch++) {
                    const size_t ue_tx_antenna =
                        (ue_interface * config_->NumUeChannels()) + ch;
                    EventData do_tx_task(
                        EventType::kPacketTX,
                        gen_tag_t::FrmSymUe(ue.tx_pending_frame_, 0,
                                            ue_tx_antenna)
                            .tag_);
                    ScheduleTask(
                        do_tx_task, &tx_queue_,
                        *tx_ptoks_ptr_[ru_->AntNumToWorkerId(ue_tx_antenna)]);
                  }

                  const size_t next_frame = current_frame + 1;
                  ue.tx_pending_frame_ = next_frame;

                  const auto tx_next =
                      std::find(ue.tx_ready_frames_.begin(),
                                ue.tx_ready_frames_.end(), next_frame);
                  if (tx_next != ue.tx_ready_frames_.end()) {
                    // With c++20 we could check the return value of remove
                    ue.tx_ready_frames_.erase(tx_next);
                    current_frame = next_frame;
                  }
                }
              } else {
                // Otherwise defer the tx (could make this sorted insert in
                // future)
                ue.tx_ready_frames_.push_front(frame_id);
              }
            }
          }
        } break;

        // Currently this only happens when there are no UL symbols
        // (pilots or otherwise)
        case EventType::kPacketPilotTX: {
          size_t frame_id = gen_tag_t(event.tags_[0u]).frame_id_;
          size_t symbol_id = gen_tag_t(event.tags_[0u]).symbol_id_;
          size_t ant_id = gen_tag_t(event.tags_[0u]).ue_id_;

          PrintPerTaskDone(PrintType::kPacketTX, frame_id, symbol_id, ant_id);

          bool last_tx_task = tx_counters_.CompleteTask(frame_id);
          if (last_tx_task) {
            stats_->MasterSetTsc(TsType::kTXDone, frame_id);
            PrintPerFrameDone(PrintType::kPacketTX, frame_id);
            tx_counters_.Reset(frame_id);

            const bool finished =
                FrameComplete(frame_id, FrameTasksFlags::kUplinkTxComplete);
            if (finished == true) {
              if ((cur_frame_id + 1) >= config_->FramesToTest()) {
                config_->Running(false);
              } else {
                FrameInit(frame_id);
                cur_frame_id = frame_id + 1;
              }
            }
          }
        } break;

        case EventType::kPacketTX: {
          const size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          const size_t ue_ant = gen_tag_t(event.tags_[0]).ue_id_;
          RtAssert(frame_id == next_frame_processed_[ue_ant],
                   "PhyUe: Unexpected frame was transmitted!");

          ul_bits_buffer_status_[ue_ant]
                                [next_frame_processed_[ue_ant] % kFrameWnd] = 0;
          next_frame_processed_[ue_ant]++;

          PrintPerTaskDone(PrintType::kPacketTX, frame_id, 0, ue_ant);
          const bool last_tx_task = tx_counters_.CompleteTask(frame_id);
          if (last_tx_task) {
            stats_->MasterSetTsc(TsType::kTXDone, frame_id);
            PrintPerFrameDone(PrintType::kPacketTX, frame_id);
            tx_counters_.Reset(frame_id);

            const bool finished =
                FrameComplete(frame_id, FrameTasksFlags::kUplinkTxComplete);
            if (finished == true) {
              if ((cur_frame_id + 1) >= config_->FramesToTest()) {
                config_->Running(false);
              } else {
                FrameInit(frame_id);
                cur_frame_id = frame_id + 1;
              }
            }
          }
        } break;

        default:
          AGORA_LOG_INFO("Invalid Event Type!\n");
          throw std::runtime_error("PhyUe: Invalid Event Type");
      }
    }
  }
  if (kPrintPhyStats) {
    phy_stats_->PrintPhyStats();
  }
  this->Stop();
}

void PhyUe::InitializeVarsFromCfg() {
  dl_pilot_symbol_perframe_ = config_->Frame().ClientDlPilotSymbols();
  size_t ul_pilot_symbol_perframe = config_->Frame().ClientUlPilotSymbols();
  ul_symbol_perframe_ = config_->Frame().NumULSyms();
  dl_symbol_perframe_ = config_->Frame().NumDLSyms();
  dl_data_symbol_perframe_ = dl_symbol_perframe_ - dl_pilot_symbol_perframe_;
  ul_data_symbol_perframe_ = ul_symbol_perframe_ - ul_pilot_symbol_perframe;

  assert(dl_pilot_symbol_perframe_ <= dl_symbol_perframe_);
  assert(ul_pilot_symbol_perframe <= ul_symbol_perframe_);
  rx_thread_num_ =
      ((kUseArgos == true) && (config_->UeHwFramer() == false))
          ? config_->UeNum()
          : std::min(config_->UeNum(), config_->UeSocketThreadNum());

  tx_buffer_size_ = config_->PacketLength() *
                    (ul_symbol_perframe_ * config_->UeAntNum() * kFrameWnd);

  rx_buffer_size_ = config_->DlPacketLength() *
                    (dl_symbol_perframe_ + config_->Frame().NumBeaconSyms()) *
                    config_->UeAntNum() * kFrameWnd;
}

void PhyUe::InitializeUplinkBuffers() {
  // initialize ul data buffer
  ul_bits_buffer_size_ =
      kFrameWnd * config_->MacBytesNumPerframe(Direction::kUplink);
  ul_bits_buffer_.Malloc(config_->UeAntNum(), ul_bits_buffer_size_,
                         Agora_memory::Alignment_t::kAlign64);
  ul_bits_buffer_status_.Calloc(config_->UeAntNum(), kFrameWnd,
                                Agora_memory::Alignment_t::kAlign64);

  // Temp -- Using more memory than necessary to comply with the DoEncode
  // function which uses the total number of ul symbols offset (instead of
  // just the data specific ones) ul_syms_buffer_size_ =
  //    kFrameWnd * ul_symbol_perframe_ * config_->OfdmDataNum();
  // ul_syms_buffer_.Calloc(config_->UeAntNum(), ul_syms_buffer_size_,
  //                       Agora_memory::Alignment_t::kAlign64);
  const size_t ul_syms_buffer_dim1 = ul_symbol_perframe_ * kFrameWnd;
  const size_t ul_syms_buffer_dim2 =
      Roundup<64>(config_->OfdmDataNum()) * config_->UeAntNum();

  ul_syms_buffer_.Calloc(ul_syms_buffer_dim1, ul_syms_buffer_dim2,
                         Agora_memory::Alignment_t::kAlign64);

  // initialize modulation buffer
  modul_buffer_.Calloc(ul_syms_buffer_dim1,
                       config_->OfdmDataNum() * config_->UeAntNum(),
                       Agora_memory::Alignment_t::kAlign64);

  // initialize IFFT buffer
  const size_t ifft_buffer_block_num =
      config_->UeAntNum() * ul_symbol_perframe_ * kFrameWnd;
  ifft_buffer_.Calloc(ifft_buffer_block_num, config_->OfdmCaNum(),
                      Agora_memory::Alignment_t::kAlign64);

  AllocBuffer1d(&tx_buffer_, tx_buffer_size_,
                Agora_memory::Alignment_t::kAlign64, 0);
}

void PhyUe::FreeUplinkBuffers() {
  ul_bits_buffer_.Free();
  ul_bits_buffer_status_.Free();
  ul_syms_buffer_.Free();
  modul_buffer_.Free();
  ifft_buffer_.Free();

  FreeBuffer1d(&tx_buffer_);
}

void PhyUe::InitializeDownlinkBuffers() {
  // initialize rx buffer
  rx_buffer_.Malloc(rx_thread_num_, rx_buffer_size_,
                    Agora_memory::Alignment_t::kAlign64);

  // initialize FFT buffer
  size_t fft_buffer_block_num =
      config_->UeAntNum() * dl_symbol_perframe_ * kFrameWnd;
  fft_buffer_.Calloc(fft_buffer_block_num, config_->OfdmCaNum(),
                     Agora_memory::Alignment_t::kAlign64);

  // initialize CSI buffer
  csi_buffer_.Calloc(config_->UeAntNum() * kFrameWnd, config_->OfdmDataNum(),
                     Agora_memory::Alignment_t::kAlign64);
  assert(reinterpret_cast<size_t>(csi_buffer_[0]) % 64 == 0);
  if (config_->Frame().ClientDlPilotSymbols() == 0) {
    for (size_t i = 0; i < csi_buffer_.Dim1(); i++) {
      complex_float* csi_data_sc = csi_buffer_[i];
      for (size_t j = 0; j < csi_buffer_.Dim2(); j++) {
        csi_data_sc[j].re = 1.0f;
        csi_data_sc[j].im = 0.0f;
      }
    }
  }

  if (dl_data_symbol_perframe_ > 0) {
    // initialize equalized data buffer
    const size_t task_buffer_symbol_num_dl =
        dl_data_symbol_perframe_ * kFrameWnd;
    size_t buffer_size = config_->UeAntNum() * task_buffer_symbol_num_dl;
    equal_buffer_.resize(buffer_size);
    for (auto& i : equal_buffer_) {
      i.resize(config_->GetOFDMDataNum());
    }
  }
}

void PhyUe::FreeDownlinkBuffers() {
  rx_buffer_.Free();
  fft_buffer_.Free();
  csi_buffer_.Free();
}

void PhyUe::PrintPerTaskDone(PrintType print_type, size_t frame_id,
                             size_t symbol_id, size_t ant) {
  if (kDebugPrintPerTaskDone == true) {
    // if (true) {
    switch (print_type) {
      case (PrintType::kPacketRX):
        AGORA_LOG_INFO("PhyUe [frame %zu symbol %zu ant %zu]: Rx packet\n",
                       frame_id, symbol_id, ant);
        break;

      case (PrintType::kPacketTX):
        AGORA_LOG_INFO(
            "PhyUe [frame %zu symbol %zu ant %zu]: %zu User Tx "
            "finished\n",
            frame_id, symbol_id, ant, tx_counters_.GetTaskCount(frame_id) + 1);
        break;

      case (PrintType::kFFTPilots):
        AGORA_LOG_INFO(
            "PhyUe [frame %zu symbol %zu ant %zu]: Pilot FFT Equalization "
            "done\n",
            frame_id, symbol_id, ant);
        break;

      case (PrintType::kFFTData):
        AGORA_LOG_INFO(
            "PhyUe [frame %zu symbol %zu ant %zu]: Data FFT Equalization "
            "done\n",
            frame_id, symbol_id, ant);
        break;

      case (PrintType::kDemul):
        AGORA_LOG_INFO("PhyUe [frame %zu symbol %zu ant %zu]: Demul done\n",
                       frame_id, symbol_id, ant);
        break;

      case (PrintType::kDecode):
        AGORA_LOG_INFO("PhyUe [frame %zu symbol %zu ant %zu]: Decoding done\n",
                       frame_id, symbol_id, ant);
        break;

      case (PrintType::kEncode):
        AGORA_LOG_INFO("PhyUe [frame %zu symbol %zu ant %zu]: Encoding done\n",
                       frame_id, symbol_id, ant);
        break;

      case (PrintType::kModul):
        AGORA_LOG_INFO(
            "PhyUe [frame %zu symbol %zu ant %zu]: Modulation done\n", frame_id,
            symbol_id, ant);
        break;

      case (PrintType::kIFFT):
        AGORA_LOG_INFO("PhyUe [frame %zu symbol %zu ant %zu]: iFFT done\n",
                       frame_id, symbol_id, ant);
        break;

      default:
        AGORA_LOG_INFO("Wrong task type in task done print!");
    }
  }
}

void PhyUe::PrintPerSymbolDone(PrintType print_type, size_t frame_id,
                               size_t symbol_id) {
  if (kDebugPrintPerSymbolDone == true) {
    // if (true) {
    switch (print_type) {
      case (PrintType::kFFTPilots):
        AGORA_LOG_INFO(
            "PhyUe [frame %zu symbol %zu + %.3f ms]: Pilot FFT complete for "
            "%zu antennas\n",
            frame_id, symbol_id,
            this->stats_->MasterGetMsSince(TsType::kFirstSymbolRX, frame_id),
            fft_dlpilot_counters_.GetTaskCount(frame_id, symbol_id));
        break;

      case (PrintType::kFFTData):
        AGORA_LOG_INFO(
            "PhyUe [frame %zu symbol %zu + %.3f ms]: Data FFT complete for "
            "%zu antennas\n",
            frame_id, symbol_id,
            this->stats_->MasterGetMsSince(TsType::kFirstSymbolRX, frame_id),
            fft_dldata_counters_.GetTaskCount(frame_id, symbol_id));
        break;

      case (PrintType::kDemul):
        AGORA_LOG_INFO(
            "PhyUe [frame %zu symbol %zu + %.3f ms]: Demul completed for "
            "%zu antennas\n",
            frame_id, symbol_id,
            this->stats_->MasterGetMsSince(TsType::kFirstSymbolRX, frame_id),
            demul_counters_.GetTaskCount(frame_id, symbol_id));
        break;

      case (PrintType::kDecode):
        AGORA_LOG_INFO(
            "PhyUe [frame %zu symbol %zu + %.3f ms]: Decoding completed "
            "for %zu antennas\n",
            frame_id, symbol_id,
            this->stats_->MasterGetMsSince(TsType::kFirstSymbolRX, frame_id),
            decode_counters_.GetTaskCount(frame_id, symbol_id));
        break;
      case (PrintType::kEncode):
        AGORA_LOG_INFO(
            "PhyUe [frame %zu symbol %zu + %.3f ms]: Data Encode complete "
            "for %zu antennas\n",
            frame_id, symbol_id,
            this->stats_->MasterGetMsSince(TsType::kFirstSymbolRX, frame_id),
            encode_counter_.GetTaskCount(frame_id, symbol_id));
        break;

      case (PrintType::kModul):
        AGORA_LOG_INFO(
            "PhyUe [frame %zu symbol %zu + %.3f ms]: Modul completed for "
            "symbol %zu antennas\n",
            frame_id, symbol_id,
            this->stats_->MasterGetMsSince(TsType::kFirstSymbolRX, frame_id),
            modulation_counters_.GetTaskCount(frame_id, symbol_id));
        break;

      case (PrintType::kIFFT):
        AGORA_LOG_INFO(
            "PhyUe [frame %zu symbol %zu + %.3f ms]: iFFT completed for "
            "symbol\n",
            frame_id, symbol_id,
            this->stats_->MasterGetMsSince(TsType::kFirstSymbolRX, frame_id));
        break;

      case (PrintType::kPacketToMac):
        AGORA_LOG_INFO(
            "Main [frame %zu symbol %zu + %.3f ms]: Completed MAC TX, "
            "%zu symbols done\n",
            frame_id, symbol_id,
            this->stats_->MasterGetMsSince(TsType::kFirstSymbolRX, frame_id),
            tomac_counters_.GetSymbolCount(frame_id) + 1);
        break;

      default:
        AGORA_LOG_INFO("Wrong task type in symbol done print!");
    }
  }
}

void PhyUe::PrintPerFrameDone(PrintType print_type, size_t frame_id) {
  if (kDebugPrintPerFrameDone == true) {
    // if (true) {
    switch (print_type) {
      case (PrintType::kPacketRX):
        AGORA_LOG_INFO("PhyUe [frame %zu + %.2f ms]: Received all packets\n",
                       frame_id,
                       this->stats_->MasterGetDeltaMs(
                           TsType::kRXDone, TsType::kFirstSymbolRX, frame_id));
        break;

      case (PrintType::kPacketRXPilots):
        AGORA_LOG_INFO(
            "PhyUe [frame %zu + %.2f ms]: Received all pilots\n", frame_id,
            this->stats_->MasterGetDeltaMs(TsType::kPilotAllRX,
                                           TsType::kFirstSymbolRX, frame_id));
        break;

      case (PrintType::kPacketTX):
        AGORA_LOG_INFO("PhyUe [frame %zu + %.2f ms]: Completed TX\n", frame_id,
                       this->stats_->MasterGetDeltaMs(
                           TsType::kTXDone, TsType::kFirstSymbolRX, frame_id));
        break;

      case (PrintType::kFFTPilots):
        AGORA_LOG_INFO(
            "PhyUe [frame %zu + %.2f ms]: Pilot FFT finished\n", frame_id,
            this->stats_->MasterGetDeltaMs(TsType::kFFTPilotsDone,
                                           TsType::kFirstSymbolRX, frame_id));
        break;

      case (PrintType::kFFTData):
        AGORA_LOG_INFO("PhyUe [frame %zu + %.2f ms]: Data FFT finished\n",
                       frame_id,
                       this->stats_->MasterGetDeltaMs(
                           TsType::kFFTDone, TsType::kFirstSymbolRX, frame_id));
        break;

      case (PrintType::kDemul):
        AGORA_LOG_INFO(
            "PhyUe [frame %zu + %.2f ms]: Completed demodulation\n", frame_id,
            this->stats_->MasterGetDeltaMs(TsType::kDemulDone,
                                           TsType::kFirstSymbolRX, frame_id));
        break;

      case (PrintType::kDecode):
        AGORA_LOG_INFO(
            "PhyUe [frame %zu + %.2f ms]: Completed decoding\n", frame_id,
            this->stats_->MasterGetDeltaMs(TsType::kDecodeDone,
                                           TsType::kFirstSymbolRX, frame_id));
        break;

      case (PrintType::kEncode):
        AGORA_LOG_INFO(
            "PhyUe [frame %zu + %.2f ms]: Completed encoding\n", frame_id,
            this->stats_->MasterGetDeltaMs(TsType::kEncodeDone,
                                           TsType::kFirstSymbolRX, frame_id));
        break;

      case (PrintType::kModul):
        AGORA_LOG_INFO(
            "PhyUe [frame %zu + %.2f ms]: Completed modulation\n", frame_id,
            this->stats_->MasterGetDeltaMs(TsType::kModulDone,
                                           TsType::kFirstSymbolRX, frame_id));
        break;

      case (PrintType::kIFFT):
        AGORA_LOG_INFO(
            "PhyUe [frame %zu + %.2f ms]: Completed iFFT\n", frame_id,
            this->stats_->MasterGetDeltaMs(TsType::kIFFTDone,
                                           TsType::kFirstSymbolRX, frame_id));
        break;

      case (PrintType::kPacketToMac):
        AGORA_LOG_INFO(
            "PhyUe [frame %zu + %.2f ms]: Completed MAC TX \n", frame_id,
            this->stats_->MasterGetMsSince(TsType::kFirstSymbolRX, frame_id));
        break;

      /*
      case (PrintType::kPacketTXFirst): AGORA_LOG_INFO(
            "PhyUe [frame %zu + %.2f ms]: Completed TX of first symbol\n",
            frame_id,
            this->stats_->MasterGetDeltaMs(TsType::kTXProcessedFirst,
                                           TsType::kFirstSymbolRX,
      frame_id)); break;
      */
      default:
        AGORA_LOG_INFO("PhyUe: Wrong task type in frame done print!\n");
    }
  }
}

void PhyUe::GetDemulData(long long** ptr, int* size) {
  *ptr = (long long*)&equal_buffer_[max_equaled_frame_ *
                                    dl_data_symbol_perframe_][0];
  *size = config_->UeAntNum() * config_->GetOFDMDataNum();
}

void PhyUe::GetEqualData(float** ptr, int* size, int ue_id) {
  *ptr = (float*)&equal_buffer_[max_equaled_frame_ * dl_data_symbol_perframe_ *
                                    config_->UeAntNum() +
                                ue_id][0];
  *size = config_->UeAntNum() * config_->GetOFDMDataNum() * 2;
}

void PhyUe::FrameInit(size_t frame) {
  std::uint8_t initial =
      static_cast<std::uint8_t>(FrameTasksFlags::kNoWorkComplete);

  if (config_->Frame().NumDLSyms() == 0) {
    initial |= static_cast<std::uint8_t>(FrameTasksFlags::kDownlinkComplete);
  }

  if ((kEnableMac == false) || (config_->Frame().NumDLSyms() == 0)) {
    initial |= static_cast<std::uint8_t>(FrameTasksFlags::kMacTxComplete);
  }
  frame_tasks_.at(frame % kFrameWnd) = initial;
}

bool PhyUe::FrameComplete(size_t frame, FrameTasksFlags complete) {
  frame_tasks_.at(frame % kFrameWnd) |= static_cast<std::uint8_t>(complete);
  bool is_complete =
      (frame_tasks_.at(frame % kFrameWnd) ==
       static_cast<std::uint8_t>(FrameTasksFlags::kFrameComplete));
  return is_complete;
}

extern "C" {
EXPORT PhyUe* PhyUeNew(Config* cfg) {
  auto* usr = new PhyUe(cfg);
  return usr;
}
EXPORT void PhyUeStart(PhyUe* usr) { usr->Start(); }
EXPORT void PhyUeStop(/*Phy_UE *usr*/) {
  SignalHandler::SetExitSignal(true); /*usr->stop();*/
}
EXPORT void PhyUeDestroy(PhyUe* usr) { delete usr; }
EXPORT void PhyUeGetEqualData(PhyUe* usr, float** ptr, int* size, int ue) {
  return usr->GetEqualData(ptr, size, ue);
}
EXPORT void PhyUeGetDemulData(PhyUe* usr, long long** ptr, int* size) {
  return usr->GetDemulData(ptr, size);
}
}
