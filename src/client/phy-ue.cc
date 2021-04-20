/**
 * @file phy-ue.cc
 * @brief Implementation file for the phy ue class
 */
#include "phy-ue.h"

#include <memory>

#include "phy_ldpc_decoder_5gnr.h"
#include "scrambler.h"
#include "signal_handler.h"
#include "utils_ldpc.h"

/* Print debug work */
static constexpr bool kDebugPrintFft = false;
static constexpr bool kDebugPrintDemul = false;
static constexpr bool kDebugPrintDecode = false;
static constexpr bool kDebugPrintEncode = false;
static constexpr bool kDebugPrintModul = false;
static constexpr bool kDebugPrintIFFT = false;

static constexpr bool kDebugPrintPacketsFromMac = false;
static constexpr bool kDebugPrintPacketsToMac = false;
static constexpr bool kPrintLLRData = false;
static constexpr bool kPrintDecodedData = false;
static constexpr bool kPrintDownlinkPilotStats = false;
static constexpr bool kPrintEqualizedSymbols = false;
static constexpr size_t kRecordFrameIndex = 1000;
static const size_t kDefaultQueueSize = 36;

PhyUe::PhyUe(Config* config)
    : stats_(std::make_unique<Stats>(config)),
      scrambler_(std::make_unique<AgoraScrambler::Scrambler>()) {
  srand(time(nullptr));

  this->config_ = config;
  InitializeVarsFromCfg();

  std::vector<size_t> data_sc_ind;
  for (size_t i = config_->OfdmDataStart();
       i < config_->OfdmDataStart() + config_->OfdmDataNum(); i++) {
    data_sc_ind.push_back(i);
  }

  non_null_sc_ind_.insert(non_null_sc_ind_.end(), data_sc_ind.begin(),
                          data_sc_ind.end());
  std::sort(non_null_sc_ind_.begin(), non_null_sc_ind_.end());

  ue_pilot_vec_.resize(config_->UeAntNum());
  for (size_t i = 0; i < config_->UeAntNum(); i++) {
    for (size_t j = config->OfdmTxZeroPrefix();
         j < config_->SampsPerSymbol() - config->OfdmTxZeroPostfix(); j++) {
      ue_pilot_vec_[i].push_back(std::complex<float>(
          config_->UeSpecificPilotT()[i][j].real() / 32768.0f,
          config_->UeSpecificPilotT()[i][j].imag() / 32768.0f));
    }
  }

  complete_queue_ = moodycamel::ConcurrentQueue<EventData>(
      kFrameWnd * config_->Frame().NumTotalSyms() * config_->UeAntNum() *
      kDefaultQueueSize);
  work_queue_ = moodycamel::ConcurrentQueue<EventData>(
      kFrameWnd * config_->Frame().NumTotalSyms() * config_->UeAntNum() *
      kDefaultQueueSize);
  tx_queue_ = moodycamel::ConcurrentQueue<EventData>(
      kFrameWnd * config_->UeNum() * kDefaultQueueSize);
  to_mac_queue_ = moodycamel::ConcurrentQueue<EventData>(
      kFrameWnd * config_->UeNum() * kDefaultQueueSize);

  for (size_t i = 0; i < rx_thread_num_; i++) {
    rx_ptoks_ptr_[i] = new moodycamel::ProducerToken(complete_queue_);
    tx_ptoks_ptr_[i] = new moodycamel::ProducerToken(tx_queue_);
    mac_rx_ptoks_ptr_[i] = new moodycamel::ProducerToken(complete_queue_);
    mac_tx_ptoks_ptr_[i] = new moodycamel::ProducerToken(to_mac_queue_);
  }

  for (size_t i = 0; i < config_->WorkerThreadNum(); i++) {
    task_ptok_[i] = new moodycamel::ProducerToken(complete_queue_);
  }
  work_producer_token_ =
      std::make_unique<moodycamel::ProducerToken>(work_queue_);

  ru_ = std::make_unique<RadioTxRx>(config_, rx_thread_num_,
                                    config_->CoreOffset() + 1, &complete_queue_,
                                    &tx_queue_, rx_ptoks_ptr_, tx_ptoks_ptr_);

  // uplink buffers init (tx)
  InitializeUplinkBuffers();
  // downlink buffers init (rx)
  InitializeDownlinkBuffers();

  if (kEnableMac == true) {
    // TODO [ankalia]: dummy_decoded_buffer is used at the base station
    // server only, but MacThread for now requires it for the UE client too
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, uint8_t> dummy_decoded_buffer;

    const size_t mac_cpu_core = config_->CoreOffset() + 1 + rx_thread_num_;
    mac_thread_ = std::make_unique<MacThread>(
        MacThread::Mode::kClient, config_, mac_cpu_core, dummy_decoded_buffer,
        &ul_bits_buffer_, &ul_bits_buffer_status_, nullptr, /* dl bits buffer */
        nullptr /* dl bits buffer status */, &to_mac_queue_, &complete_queue_);

    mac_std_thread_ = std::thread(&MacThread::RunEventLoop, mac_thread_.get());
  }

  (void)DftiCreateDescriptor(&mkl_handle_, DFTI_SINGLE, DFTI_COMPLEX, 1,
                             config_->OfdmCaNum());
  (void)DftiCommitDescriptor(mkl_handle_);

  // initilize all kinds of checkers
  for (size_t i = 0; i < config_->WorkerThreadNum(); i++) {
    task_threads_.at(i) = std::thread(&PhyUe::TaskThread, this, i);
  }

  // Init the frame work tracking structure
  for (size_t frame = 0; frame < this->frame_tasks_.size(); frame++) {
    FrameInit(frame);
  }
  tx_counters_.Init(config_->UeAntNum());
  decode_counters_.Init(dl_data_symbol_perframe_, config_->UeAntNum());
  demul_counters_.Init(dl_data_symbol_perframe_, config_->UeAntNum());
  fft_dlpilot_counters_.Init(config->Frame().ClientDlPilotSymbols(),
                             config_->UeAntNum());
  fft_dldata_counters_.Init(dl_data_symbol_perframe_, config_->UeAntNum());

  encode_counter_.Init(config_->UeAntNum());
  modulation_counters_.Init(config_->UeAntNum());
  ifft_counters_.Init(config_->UeAntNum());

  rx_counters_.num_pkts_per_frame_ =
      config_->UeAntNum() *
      (config_->Frame().NumDLSyms() + config_->Frame().NumBeaconSyms());
  rx_counters_.num_pilot_pkts_per_frame_ =
      config_->UeAntNum() * config_->Frame().ClientDlPilotSymbols();
  // This field doesn't effect the user num_reciprocity_pkts_per_frame_;

  rx_downlink_deferral_.resize(kFrameWnd);
}

PhyUe::~PhyUe() {
  for (size_t i = 0; i < config_->WorkerThreadNum(); i++) {
    std::printf("Joining Phy worker: %zu : %zu\n", i,
                config_->WorkerThreadNum());
    task_threads_.at(i).join();
    delete task_ptok_[i];
  }

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

  DftiFreeDescriptor(&mkl_handle_);
}

void PhyUe::ScheduleTask(EventData do_task,
                         moodycamel::ConcurrentQueue<EventData>* in_queue,
                         moodycamel::ProducerToken const& ptok) {
  if (in_queue->try_enqueue(ptok, do_task) == false) {
    std::printf("PhyUe: Cannot enqueue task, need more memory");
    if (in_queue->enqueue(ptok, do_task) == false) {
      std::printf("PhyUe: task enqueue failed\n");
      throw std::runtime_error("PhyUe: task enqueue failed");
    }
  }
}

void PhyUe::ScheduleWork(EventData do_task) {
  if (work_queue_.try_enqueue(*(work_producer_token_.get()), do_task) ==
      false) {
    std::printf("PhyUe: Cannot enqueue work task, need more memory");
    if (work_queue_.enqueue(*(work_producer_token_.get()), do_task) == false) {
      std::printf("PhyUe: work task enqueue failed\n");
      throw std::runtime_error("PhyUe: work task enqueue failed");
    }
  }
}

void PhyUe::ReceiveDownlinkSymbol(struct Packet* rx_packet, size_t tag) {
  size_t frame_slot = rx_packet->frame_id_ % kFrameWnd;
  size_t dl_symbol_idx = config_->Frame().GetDLSymbolIdx(rx_packet->symbol_id_);

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
  size_t frame_slot = frame_id % kFrameWnd;
  // Complete the csi offset
  size_t csi_offset = frame_slot * config_->UeAntNum();

  for (size_t user = 0; user < config_->UeAntNum(); user++) {
    csi_offset = csi_offset + user;
    for (size_t ofdm_data = 0; ofdm_data < config_->OfdmDataNum();
         ofdm_data++) {
      auto* csi_buffer_ptr =
          reinterpret_cast<arma::cx_float*>(csi_buffer_.at(csi_offset).data());

      csi_buffer_ptr[ofdm_data] /= dl_pilot_symbol_perframe_;
    }
  }
  std::queue<EventData>* defferal_queue = &rx_downlink_deferral_.at(frame_slot);

  while (defferal_queue->empty() == false) {
    ScheduleWork(defferal_queue->front());
    defferal_queue->pop();
  }
}

void PhyUe::ClearCsi(size_t frame_id) {
  size_t frame_slot = frame_id % kFrameWnd;

  if (config_->Frame().ClientDlPilotSymbols() > 0) {
    size_t csi_offset = frame_slot * config_->UeAntNum();
    for (size_t user = 0; user < config_->UeAntNum(); user++) {
      csi_offset = csi_offset + user;
      for (size_t ofdm_data = 0; ofdm_data < config_->OfdmDataNum();
           ofdm_data++) {
        auto* csi_buffer_ptr = reinterpret_cast<arma::cx_float*>(
            csi_buffer_.at(csi_offset).data());

        csi_buffer_ptr[ofdm_data] = 0;
      }
    }
    fft_dlpilot_counters_.Reset(frame_id);
  }  // Only do work if there are DL pilot symbols
  assert(rx_downlink_deferral_.at(frame_slot).empty() == true);
}

void PhyUe::Stop() {
  std::cout << "PhyUe: Stopping threads " << std::endl;
  config_->Running(false);
  usleep(1000);
  ru_.reset();
}

void PhyUe::Start() {
  PinToCoreWithOffset(ThreadType::kMaster, config_->CoreOffset(), 0);

  if (ru_->StartTxRx(rx_buffer_, rx_buffer_status_, rx_buffer_status_size_,
                     rx_buffer_size_, tx_buffer_, tx_buffer_status_,
                     tx_buffer_status_size_, tx_buffer_size_) == false) {
    this->Stop();
    return;
  }

  // for task_queue, main thread is producer, it is single-producer &
  // multiple consumer for task queue uplink

  // TODO: make the producertokens global and try
  // "try_dequeue_from_producer(token,item)"
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
  std::vector<size_t> ifft_next_frame(config_->UeNum());
  std::vector<size_t> ifft_frame_status(config_->UeNum() * kFrameWnd, SIZE_MAX);
  while ((config_->Running() == true) &&
         (SignalHandler::GotExitSignal() == false)) {
    // get a bulk of events
    ret = complete_queue_.try_dequeue_bulk(ctok, events_list.data(),
                                           kDequeueBulkSizeTXRX);
    total_count++;
    if (total_count == 1e7) {
      // print the complete_queue_ miss rate is needed
      // std::printf("message dequeue miss rate %f\n", (float)miss_count /
      // total_count);
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
          size_t rx_thread_id = rx_tag_t(event.tags_[0]).tid_;
          size_t offset_in_current_buffer = rx_tag_t(event.tags_[0]).offset_;

          auto* pkt = reinterpret_cast<struct Packet*>(
              rx_buffer_[rx_thread_id] +
              offset_in_current_buffer * config_->PacketLength());
          size_t frame_id = pkt->frame_id_;
          size_t symbol_id = pkt->symbol_id_;
          size_t ant_id = pkt->ant_id_;
          size_t ue_id = ant_id / config_->NumChannels();
          size_t frame_slot = frame_id % kFrameWnd;
          RtAssert(pkt->frame_id_ < (cur_frame_id + kFrameWnd),
                   "Error: Received packet for future frame beyond frame "
                   "window. This can happen if PHY is running "
                   "slowly, e.g., in debug mode");

          PrintPerTaskDone(PrintType::kPacketRX, frame_id, symbol_id, ant_id);
          // TODO: Defer the scheduling
          // Receive first packet in a frame

          if (rx_counters_.num_pkts_.at(frame_slot) == 0) {
            this->stats_->MasterSetTsc(TsType::kFirstSymbolRX, frame_id);
            if (kDebugPrintPerFrameStart) {
              const size_t prev_frame_slot =
                  (frame_slot + kFrameWnd - 1) % kFrameWnd;
              std::printf(
                  "PhyUe [frame %zu + %.2f ms since last frame]: "
                  "Received "
                  "first packet. Remaining packets in prev frame: %zu\n",
                  frame_id,
                  this->stats_->MasterGetDeltaMs(TsType::kFirstSymbolRX,
                                                 frame_id, frame_id - 1),
                  rx_counters_.num_pkts_.at(prev_frame_slot));
            }
          }

          if (config_->IsPilot(frame_id, symbol_id)) {
            rx_counters_.num_pilot_pkts_.at(frame_slot)++;
            if (rx_counters_.num_pilot_pkts_.at(frame_slot) ==
                rx_counters_.num_pilot_pkts_per_frame_) {
              rx_counters_.num_pilot_pkts_.at(frame_slot) = 0;
              this->stats_->MasterSetTsc(TsType::kPilotAllRX, frame_id);
              PrintPerFrameDone(PrintType::kPacketRXPilots, frame_id);
            }
          }
          rx_counters_.num_pkts_.at(frame_slot)++;
          if (rx_counters_.num_pkts_.at(frame_slot) ==
              rx_counters_.num_pkts_per_frame_) {
            this->stats_->MasterSetTsc(TsType::kRXDone, frame_id);
            PrintPerFrameDone(PrintType::kPacketRX, frame_id);
            rx_counters_.num_pkts_.at(frame_slot) = 0;
          }

          // Schedule uplink pilots transmission and uplink processing
          if (symbol_id == config_->Frame().GetBeaconSymbolLast()) {
            if (ul_data_symbol_perframe_ == 0) {
              // Schedule Pilot after receiving last beacon
              // (Only when in Downlink Only mode, otherwise the pilots
              // will be transmitted with the uplink data)
              if (ant_id % config_->NumChannels() == 0) {
                EventData do_tx_pilot_task(
                    EventType::kPacketPilotTX,
                    gen_tag_t::FrmSymUe(
                        frame_id, config_->Frame().GetPilotSymbol(ue_id), ue_id)
                        .tag_);
                ScheduleTask(do_tx_pilot_task, &tx_queue_,
                             *tx_ptoks_ptr_[ant_id % rx_thread_num_]);
              }
            } else {
              if (ant_id % config_->NumChannels() == 0) {
                EventData do_encode_task(
                    EventType::kEncode,
                    gen_tag_t::FrmSymUe(frame_id, symbol_id, ue_id).tag_);
                ScheduleWork(do_encode_task);
              }
            }
          }

          // ** BUG with symbol_id =
          SymbolType symbol_type = config_->GetSymbolType(symbol_id);
          if (symbol_type == SymbolType::kDL) {
            // Defer downlink processing (all pilot symbols must be fft'd
            // first)
            ReceiveDownlinkSymbol(pkt, event.tags_[0]);
          } else {
            rx_buffer_status_[rx_thread_id][offset_in_current_buffer] = 0;
          }
        } break;

        case EventType::kFFTPilot: {
          size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;
          size_t ant_id = gen_tag_t(event.tags_[0]).ant_id_;

          PrintPerTaskDone(PrintType::kFFTPilots, frame_id, symbol_id, ant_id);
          bool tasks_complete =
              fft_dlpilot_counters_.CompleteTask(frame_id, symbol_id);
          if (tasks_complete == true) {
            PrintPerSymbolDone(PrintType::kFFTPilots, frame_id, symbol_id);
            bool pilot_fft_complete =
                fft_dlpilot_counters_.CompleteSymbol(frame_id);
            if (pilot_fft_complete == true) {
              this->stats_->MasterSetTsc(TsType::kFFTPilotsDone, frame_id);
              PrintPerFrameDone(PrintType::kFFTPilots, frame_id);
              ScheduleDefferedDownlinkSymbols(frame_id);
            }
          }
        } break;

        case EventType::kFFT: {
          size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;
          size_t ant_id = gen_tag_t(event.tags_[0]).ant_id_;

          // Schedule the Demul
          EventData do_demul_task(EventType::kDemul, event.tags_[0]);
          ScheduleWork(do_demul_task);

          PrintPerTaskDone(PrintType::kFFTData, frame_id, symbol_id, ant_id);
          bool tasks_complete =
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
          size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;
          size_t ant_id = gen_tag_t(event.tags_[0]).ant_id_;

          EventData do_decode_task(EventType::kDecode, event.tags_[0]);
          ScheduleWork(do_decode_task);

          PrintPerTaskDone(PrintType::kDemul, frame_id, symbol_id, ant_id);
          bool symbol_complete =
              demul_counters_.CompleteTask(frame_id, symbol_id);
          if (symbol_complete == true) {
            PrintPerSymbolDone(PrintType::kDemul, frame_id, symbol_id);
            max_equaled_frame_ = frame_id;
            bool demul_complete = demul_counters_.CompleteSymbol(frame_id);
            if (demul_complete == true) {
              this->stats_->MasterSetTsc(TsType::kDemulDone, frame_id);
              PrintPerFrameDone(PrintType::kDemul, frame_id);
              demul_counters_.Reset(frame_id);
            }
          }
        } break;

        case EventType::kDecode: {
          size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          size_t ant_id = gen_tag_t(event.tags_[0]).ant_id_;
          size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;

          if (kEnableMac) {
            ScheduleTask(EventData(EventType::kPacketToMac, event.tags_[0]),
                         &to_mac_queue_, ptok_mac);
          }
          PrintPerTaskDone(PrintType::kDecode, frame_id, symbol_id, ant_id);

          bool symbol_complete =
              decode_counters_.CompleteTask(frame_id, symbol_id);
          if (symbol_complete == true) {
            PrintPerSymbolDone(PrintType::kDecode, frame_id, symbol_id);

            bool decode_complete = decode_counters_.CompleteSymbol(frame_id);
            if (decode_complete == true) {
              this->stats_->MasterSetTsc(TsType::kDecodeDone, frame_id);
              PrintPerFrameDone(PrintType::kDecode, frame_id);
              decode_counters_.Reset(frame_id);

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
          size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;

          if (kDebugPrintPacketsToMac) {
            std::printf(
                "PhyUE: sent decoded packet for (frame %zu, symbol %zu) "
                "to "
                "MAC\n",
                frame_id, symbol_id);
          }

          bool finished =
              FrameComplete(frame_id, FrameTasksFlags::kMacTxComplete);
          if (finished == true) {
            if ((cur_frame_id + 1) >= config_->FramesToTest()) {
              config_->Running(false);
            } else {
              FrameInit(frame_id);
              cur_frame_id = frame_id + 1;
            }
          }
        } break;

        case EventType::kPacketFromMac: {
          size_t ue_id = rx_tag_t(event.tags_[0]).tid_;
          size_t radio_buf_id = rx_tag_t(event.tags_[0]).offset_;
          RtAssert(radio_buf_id == expected_frame_id_from_mac_ % kFrameWnd);

          auto* pkt = reinterpret_cast<MacPacket*>(
              &ul_bits_buffer_[ue_id]
                              [radio_buf_id * config_->MacBytesNumPerframe()]);
          RtAssert(pkt->frame_id_ == expected_frame_id_from_mac_,
                   "PhyUE: Incorrect frame ID from MAC");
          current_frame_user_num_ =
              (current_frame_user_num_ + 1) % config_->UeAntNum();
          if (current_frame_user_num_ == 0) {
            expected_frame_id_from_mac_++;
          }
          config_->UpdateModCfgs(pkt->rb_indicator_.mod_order_bits_);

          if (kDebugPrintPacketsFromMac) {
            std::printf(
                "PhyUE: received packet for frame %u with modulation "
                "%zu\n",
                pkt->frame_id_, pkt->rb_indicator_.mod_order_bits_);
            std::stringstream ss;
            ss << "PhyUE: kPacketFromMac, frame ID " << pkt->frame_id_
               << ", bytes: ";
            for (size_t i = 0; i < 4; i++) {
              ss << std::to_string((reinterpret_cast<uint8_t*>(pkt->data_)[i]))
                 << ", ";
            }
            std::printf("%s\n", ss.str().c_str());
          }

        } break;

        case EventType::kEncode: {
          /* Currently the user encodes all symbols in one completion
           * event */
          size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;
          size_t ant_id = gen_tag_t(event.tags_[0]).ant_id_;

          PrintPerTaskDone(PrintType::kEncode, frame_id, symbol_id, ant_id);

          // Schedule the modul
          EventData do_modul_task(EventType::kModul, event.tags_[0]);
          ScheduleWork(do_modul_task);

          bool last_encode = encode_counter_.CompleteTask(frame_id);
          if (last_encode == true) {
            this->stats_->MasterSetTsc(TsType::kEncodeDone, frame_id);
            PrintPerFrameDone(PrintType::kEncode, frame_id);
            encode_counter_.Reset(frame_id);
          }
        } break;

        case EventType::kModul: {
          /* Currently the user modulates all symbols in one completion
           * event */
          size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;
          size_t ue_id = gen_tag_t(event.tags_[0]).ue_id_;

          PrintPerTaskDone(PrintType::kModul, frame_id, symbol_id, ue_id);

          EventData do_ifft_task(
              EventType::kIFFT,
              gen_tag_t::FrmSymUe(frame_id, symbol_id, ue_id).tag_);
          ScheduleWork(do_ifft_task);

          bool last_encode = modulation_counters_.CompleteTask(frame_id);
          if (last_encode == true) {
            this->stats_->MasterSetTsc(TsType::kModulDone, frame_id);
            PrintPerFrameDone(PrintType::kModul, frame_id);
            modulation_counters_.Reset(frame_id);
          }
        } break;

        case EventType::kIFFT: {
          size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;
          size_t ue_id = gen_tag_t(event.tags_[0]).ue_id_;

          PrintPerTaskDone(PrintType::kIFFT, frame_id, symbol_id, ue_id);
          bool ifft_complete = ifft_counters_.CompleteTask(frame_id);

          if (ifft_complete == true) {
            this->stats_->MasterSetTsc(TsType::kIFFTDone, frame_id);
            PrintPerFrameDone(PrintType::kIFFT, frame_id);
            ifft_counters_.Reset(frame_id);
          }

          // Schedule Transmit in frame order
          ifft_frame_status.at((ue_id * kFrameWnd) + (frame_id % kFrameWnd)) =
              frame_id;
          for (size_t i = 0; (i < kFrameWnd); i++) {
            size_t ifft_stat_id =
                (ue_id * kFrameWnd) + ((i + frame_id) % kFrameWnd);
            if (ifft_frame_status.at(ifft_stat_id) ==
                ifft_next_frame.at(ue_id)) {
              EventData do_tx_task(
                  EventType::kPacketTX,
                  gen_tag_t::FrmSymUe(ifft_next_frame.at(ue_id), 0, ue_id)
                      .tag_);
              ScheduleTask(do_tx_task, &tx_queue_,
                           *tx_ptoks_ptr_[ue_id % rx_thread_num_]);
              ifft_next_frame.at(ue_id)++;
            } else { /* Done */
              break;
            }
          }
        } break;

        // Currently this only happens when there are no UL symbols
        // (pilots or otherwise)
        case EventType::kPacketPilotTX: {
          size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;
          size_t ue_id = gen_tag_t(event.tags_[0]).ue_id_;

          PrintPerTaskDone(PrintType::kPacketTX, frame_id, symbol_id, ue_id);

          bool last_tx_task = this->tx_counters_.CompleteTask(frame_id);
          if (last_tx_task) {
            this->stats_->MasterSetTsc(TsType::kTXDone, frame_id);
            PrintPerFrameDone(PrintType::kPacketTX, frame_id);
            this->tx_counters_.Reset(frame_id);

            bool finished =
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
          size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          size_t ue_id = gen_tag_t(event.tags_[0]).ue_id_;
          RtAssert(frame_id == next_frame_processed_[ue_id],
                   "PhyUE: Unexpected frame was transmitted!");

          ul_bits_buffer_status_[ue_id]
                                [next_frame_processed_[ue_id] % kFrameWnd] = 0;
          next_frame_processed_[ue_id]++;

          PrintPerTaskDone(PrintType::kPacketTX, frame_id, 0, ue_id);
          bool last_tx_task = this->tx_counters_.CompleteTask(frame_id);
          if (last_tx_task) {
            this->stats_->MasterSetTsc(TsType::kTXDone, frame_id);
            PrintPerFrameDone(PrintType::kPacketTX, frame_id);
            this->tx_counters_.Reset(frame_id);

            bool finished =
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
          std::cout << "Invalid Event Type!" << std::endl;
          throw std::runtime_error("PhyUe: Invalid Event Type");
      }
    }
  }
  if (kPrintPhyStats) {
    const size_t task_buffer_symbol_num_dl =
        dl_data_symbol_perframe_ * kFrameWnd;
    for (size_t ue_id = 0; ue_id < config_->UeAntNum(); ue_id++) {
      size_t total_decoded_bits(0);
      size_t total_bit_errors(0);
      size_t total_decoded_blocks(0);
      size_t total_block_errors(0);
      for (size_t i = 0; i < task_buffer_symbol_num_dl; i++) {
        total_decoded_bits += decoded_bits_count_[ue_id][i];
        total_bit_errors += bit_error_count_[ue_id][i];
        total_decoded_blocks += decoded_blocks_count_[ue_id][i];
        total_block_errors += block_error_count_[ue_id][i];
      }
      std::cout << "UE " << ue_id << ": bit errors (BER) " << total_bit_errors
                << "/" << total_decoded_bits << "("
                << 1.0 * total_bit_errors / total_decoded_bits
                << "), block errors (BLER) " << total_block_errors << "/"
                << total_decoded_blocks << " ("
                << 1.0 * total_block_errors / total_decoded_blocks
                << "), symbol errors " << symbol_error_count_.at(ue_id) << "/"
                << decoded_symbol_count_.at(ue_id) << " ("
                << 1.0 * symbol_error_count_.at(ue_id) /
                       decoded_symbol_count_.at(ue_id)
                << ")" << std::endl;
    }
  }
  this->Stop();
}

void PhyUe::TaskThread(int tid) {
  std::printf("User Task[%d]: started\n", tid);
  PinToCoreWithOffset(ThreadType::kWorker,
                      config_->CoreOffset() + rx_thread_num_ + 1 +
                          (kEnableMac ? rx_thread_num_ : 0),
                      tid);

  EventData event;
  while (config_->Running() == true) {
    if (work_queue_.try_dequeue_from_producer(*work_producer_token_.get(),
                                              event) == true) {
      switch (event.event_type_) {
        case EventType::kDecode: {
          DoDecode(tid, event.tags_[0]);
        } break;
        case EventType::kDemul: {
          DoDemul(tid, event.tags_[0]);
        } break;
        case EventType::kIFFT: {
          DoIfft(tid, event.tags_[0]);
        } break;
        case EventType::kModul: {
          DoModul(tid, event.tags_[0]);
        } break;
        case EventType::kEncode: {
          DoEncode(tid, event.tags_[0]);
        } break;
        case EventType::kFFTPilot: {
          DoFftPilot(tid, event.tags_[0]);
        } break;
        case EventType::kFFT: {
          DoFftData(tid, event.tags_[0]);
        } break;
        default: {
          std::printf("***** Invalid Event Type [%d] in Work Queue\n",
                      static_cast<int>(event.event_type_));
        }
      }
    }  // end dequeue
  }
}

//////////////////////////////////////////////////////////
//                   DOWNLINK Operations                //
//////////////////////////////////////////////////////////
void PhyUe::DoFftData(int tid, size_t tag) {
  size_t rx_thread_id = fft_req_tag_t(tag).tid_;
  size_t offset_in_current_buffer = fft_req_tag_t(tag).offset_;
  size_t start_tsc = GetTime::Rdtsc();

  // read info of one frame
  auto* pkt = reinterpret_cast<struct Packet*>(rx_buffer_[rx_thread_id] +
                                               offset_in_current_buffer *
                                                   config_->PacketLength());
  size_t frame_id = pkt->frame_id_;
  size_t symbol_id = pkt->symbol_id_;
  size_t ant_id = pkt->ant_id_;
  size_t frame_slot = frame_id % kFrameWnd;

  if (kDebugPrintInTask || kDebugPrintFft) {
    std::printf("User Task[%d]: Fft Data(frame %zu, symbol %zu, ant %zu)\n",
                tid, frame_id, symbol_id, ant_id);
  }

  size_t sig_offset = config_->OfdmRxZeroPrefixClient();
  if (kPrintDownlinkPilotStats) {
    if (frame_id == kRecordFrameIndex) {
      std::string fname = "rxdata" + std::to_string(symbol_id) + "_" +
                          std::to_string(ant_id) + ".bin";
      FILE* f = std::fopen(fname.c_str(), "wb");
      std::fwrite(pkt->data_, 2 * sizeof(int16_t), config_->SampsPerSymbol(),
                  f);
      std::fclose(f);
    }
  }

  // remove CP, do FFT
  size_t dl_symbol_id = config_->Frame().GetDLSymbolIdx(symbol_id);
  size_t total_dl_symbol_id = (frame_slot * dl_symbol_perframe_) + dl_symbol_id;
  size_t fft_buffer_target_id =
      (total_dl_symbol_id * config_->UeAntNum()) + ant_id;

  // transfer ushort to float
  size_t delay_offset = (sig_offset + config_->CpLen()) * 2;
  auto* fft_buff = reinterpret_cast<float*>(fft_buffer_[fft_buffer_target_id]);

  SimdConvertShortToFloat(&pkt->data_[delay_offset], fft_buff,
                          config_->OfdmCaNum() * 2);

  // perform fft
  DftiComputeForward(mkl_handle_, fft_buffer_[fft_buffer_target_id]);

  size_t csi_offset = frame_slot * config_->UeAntNum() + ant_id;
  auto* csi_buffer_ptr =
      reinterpret_cast<arma::cx_float*>(csi_buffer_.at(csi_offset).data());
  auto* fft_buffer_ptr =
      reinterpret_cast<arma::cx_float*>(fft_buffer_[fft_buffer_target_id]);

  size_t total_dl_data_symbol_id = (frame_slot * dl_data_symbol_perframe_) +
                                   (dl_symbol_id - dl_pilot_symbol_perframe_);
  size_t eq_buffer_offset =
      total_dl_data_symbol_id * config_->UeAntNum() + ant_id;

  auto* equ_buffer_ptr = reinterpret_cast<arma::cx_float*>(
      equal_buffer_.at(eq_buffer_offset).data());

  // use pilot subcarriers for phase tracking and correction
  float theta = 0;
  for (size_t j = 0; j < config_->OfdmDataNum(); j++) {
    if (j % config_->OfdmPilotSpacing() == 0) {
      equ_buffer_ptr[j] = 0;
      size_t sc_id = non_null_sc_ind_[j];
      arma::cx_float y = fft_buffer_ptr[sc_id];
      auto pilot_eq = y / csi_buffer_ptr[j];
      // FIXME: cfg->ue_specific_pilot[user_id] index creates errors
      // in the downlink receiver
      auto p = config_->UeSpecificPilot()[0][j];
      theta += arg(pilot_eq * arma::cx_float(p.re, -p.im));
    }
  }
  if (config_->GetOFDMPilotNum() > 0) {
    theta /= config_->GetOFDMPilotNum();
  }
  auto phc = exp(arma::cx_float(0, -theta));
  float evm = 0;
  for (size_t j = 0; j < config_->OfdmDataNum(); j++) {
    if (j % config_->OfdmPilotSpacing() != 0) {
      // divide fft output by pilot data to get CSI estimation
      size_t sc_id = non_null_sc_ind_[j];
      arma::cx_float y = fft_buffer_ptr[sc_id];
      equ_buffer_ptr[j] = (y / csi_buffer_ptr[j]) * phc;
      complex_float tx =
          config_->DlIqF()[dl_symbol_id][ant_id * config_->OfdmCaNum() +
                                         config_->OfdmDataStart() + j];
      evm += std::norm(equ_buffer_ptr[j] - arma::cx_float(tx.re, tx.im));
    }
  }

  if (kPrintEqualizedSymbols) {
    complex_float* tx =
        &config_->DlIqF()[dl_symbol_id][ant_id * config_->OfdmCaNum() +
                                        config_->OfdmDataStart()];
    arma::cx_fvec x_vec(reinterpret_cast<arma::cx_float*>(tx),
                        config_->OfdmDataNum(), false);
    Utils::PrintVec(x_vec, std::string("x") +
                               std::to_string(total_dl_symbol_id) +
                               std::string("_") + std::to_string(ant_id));
    arma::cx_fvec equal_vec(equ_buffer_ptr, config_->OfdmDataNum(), false);
    Utils::PrintVec(equal_vec, std::string("equ") +
                                   std::to_string(total_dl_symbol_id) +
                                   std::string("_") + std::to_string(ant_id));
  }
  evm = std::sqrt(evm) / (config_->OfdmDataNum() - config_->GetOFDMPilotNum());
  if (kPrintPhyStats) {
    std::stringstream ss;
    ss << "Frame: " << frame_id << ", Symbol: " << symbol_id
       << ", User: " << ant_id << ", EVM: " << 100 * evm
       << "%, SNR: " << -10 * std::log10(evm) << std::endl;
    std::cout << ss.str();
  }

  if (kDebugPrintPerTaskDone || kDebugPrintFft) {
    size_t fft_duration_stat = GetTime::Rdtsc() - start_tsc;
    std::printf(
        "User Task[%d]: Fft Data(frame %zu, symbol %zu, ant %zu) Duration "
        "%2.4f "
        "ms\n",
        tid, frame_id, symbol_id, ant_id,
        GetTime::CyclesToMs(fft_duration_stat, GetTime::MeasureRdtscFreq()));
  }

  // Free the rx buffer
  rx_buffer_status_[rx_thread_id][offset_in_current_buffer] = 0;
  EventData fft_finish_event = EventData(
      EventType::kFFT, gen_tag_t::FrmSymAnt(frame_id, symbol_id, ant_id).tag_);
  RtAssert(complete_queue_.enqueue(*task_ptok_[tid], fft_finish_event),
           "User Task: FFT message enqueue failed");
}

void PhyUe::DoFftPilot(int tid, size_t tag) {
  size_t rx_thread_id = fft_req_tag_t(tag).tid_;
  size_t offset_in_current_buffer = fft_req_tag_t(tag).offset_;
  size_t start_tsc = GetTime::Rdtsc();

  // read info of one frame
  auto* pkt = reinterpret_cast<struct Packet*>(
      rx_buffer_[rx_thread_id] +
      (offset_in_current_buffer * config_->PacketLength()));
  size_t frame_id = pkt->frame_id_;
  size_t symbol_id = pkt->symbol_id_;
  size_t ant_id = pkt->ant_id_;
  size_t frame_slot = frame_id % kFrameWnd;

  if (kDebugPrintInTask || kDebugPrintFft) {
    std::printf("User Task[%d]: Fft Pilot(frame %zu, symbol %zu, ant %zu)\n",
                tid, frame_id, symbol_id, ant_id);
  }

  size_t sig_offset = config_->OfdmRxZeroPrefixClient();
  if (kPrintDownlinkPilotStats) {
    SimdConvertShortToFloat(pkt->data_, reinterpret_cast<float*>(rx_samps_tmp_),
                            2 * config_->SampsPerSymbol());
    std::vector<std::complex<float>> samples_vec(
        rx_samps_tmp_, rx_samps_tmp_ + config_->SampsPerSymbol());
    size_t seq_len = ue_pilot_vec_[ant_id].size();
    std::vector<std::complex<float>> pilot_corr =
        CommsLib::CorrelateAvx(samples_vec, ue_pilot_vec_[ant_id]);
    std::vector<float> pilot_corr_abs = CommsLib::Abs2Avx(pilot_corr);
    size_t peak_offset =
        std::max_element(pilot_corr_abs.begin(), pilot_corr_abs.end()) -
        pilot_corr_abs.begin();
    size_t pilot_offset = peak_offset < seq_len ? 0 : peak_offset - seq_len;
    float noise_power = 0;
    for (size_t i = 0; i < pilot_offset; i++) {
      noise_power += std::pow(std::abs(samples_vec[i]), 2);
    }
    float signal_power = 0;
    for (size_t i = pilot_offset; i < 2 * pilot_offset; i++) {
      signal_power += std::pow(std::abs(samples_vec[i]), 2);
    }
    float snr = 10 * std::log10(signal_power / noise_power);
    std::printf(
        "User Task: Fft Pilot(frame %zu symbol %zu ant %zu) sig offset "
        "%zu, SNR %2.1f \n",
        frame_id, symbol_id, ant_id, pilot_offset, snr);
    if (frame_id == kRecordFrameIndex) {
      std::string fname = "rxpilot" + std::to_string(symbol_id) + "_" +
                          std::to_string(ant_id) + ".bin";
      FILE* f = std::fopen(fname.c_str(), "wb");
      std::fwrite(pkt->data_, 2 * sizeof(int16_t), config_->SampsPerSymbol(),
                  f);
      std::fclose(f);
    }
  }

  // remove CP, do FFT
  size_t dl_symbol_id = config_->Frame().GetDLSymbolIdx(symbol_id);
  size_t total_dl_symbol_id = (frame_slot * dl_symbol_perframe_) + dl_symbol_id;
  size_t fft_buffer_target_id =
      (total_dl_symbol_id * config_->UeAntNum()) + ant_id;

  // transfer ushort to float
  size_t delay_offset = (sig_offset + config_->CpLen()) * 2;
  auto* fft_buff = reinterpret_cast<float*>(fft_buffer_[fft_buffer_target_id]);

  SimdConvertShortToFloat(&pkt->data_[delay_offset], fft_buff,
                          config_->OfdmCaNum() * 2);

  // perform fft
  DftiComputeForward(mkl_handle_, fft_buffer_[fft_buffer_target_id]);

  size_t csi_offset = frame_slot * config_->UeAntNum() + ant_id;
  auto* csi_buffer_ptr =
      reinterpret_cast<arma::cx_float*>(csi_buffer_.at(csi_offset).data());
  auto* fft_buffer_ptr =
      reinterpret_cast<arma::cx_float*>(fft_buffer_[fft_buffer_target_id]);

  // In TDD massive MIMO, a pilot symbol needs to be sent
  // in the downlink for the user to estimate the channel
  // due to relative reciprocity calibration,
  // see Argos paper (Mobicom'12)
  if (dl_symbol_id < dl_pilot_symbol_perframe_) {
    for (size_t j = 0; j < config_->OfdmDataNum(); j++) {
      // FIXME: cfg->ue_specific_pilot[user_id] index creates errors
      // in the downlink receiver
      complex_float p = config_->UeSpecificPilot()[0][j];
      size_t sc_id = non_null_sc_ind_[j];
      csi_buffer_ptr[j] += (fft_buffer_ptr[sc_id] / arma::cx_float(p.re, p.im));
    }
  }

  if (kDebugPrintPerTaskDone || kDebugPrintFft) {
    size_t fft_duration_stat = GetTime::Rdtsc() - start_tsc;
    std::printf(
        "User Task[%d]: Fft Pilot(frame %zu, symbol %zu, ant %zu) Duration "
        "%2.4f ms\n",
        tid, frame_id, symbol_id, ant_id,
        GetTime::CyclesToMs(fft_duration_stat, GetTime::MeasureRdtscFreq()));
  }

  // Free the rx buffer
  rx_buffer_status_[rx_thread_id][offset_in_current_buffer] = 0;
  EventData fft_finish_event =
      EventData(EventType::kFFTPilot,
                gen_tag_t::FrmSymAnt(frame_id, symbol_id, ant_id).tag_);
  RtAssert(complete_queue_.enqueue(*task_ptok_[tid], fft_finish_event),
           "User Task: FFT Pilot message enqueue failed");
}

void PhyUe::DoDemul(int tid, size_t tag) {
  const size_t frame_id = gen_tag_t(tag).frame_id_;
  const size_t symbol_id = gen_tag_t(tag).symbol_id_;
  const size_t ant_id = gen_tag_t(tag).ant_id_;

  if (kDebugPrintInTask || kDebugPrintDemul) {
    std::printf("User Task[%d]: Demul  (frame %zu, symbol %zu, ant %zu)\n", tid,
                frame_id, symbol_id, ant_id);
  }
  size_t start_tsc = GetTime::Rdtsc();

  const size_t frame_slot = frame_id % kFrameWnd;
  size_t dl_symbol_id = config_->Frame().GetDLSymbolIdx(symbol_id);
  size_t total_dl_symbol_id = frame_slot * dl_data_symbol_perframe_ +
                              dl_symbol_id - dl_pilot_symbol_perframe_;
  size_t offset = total_dl_symbol_id * config_->UeAntNum() + ant_id;
  auto* equal_ptr = reinterpret_cast<float*>(&equal_buffer_[offset][0]);
  auto* demul_ptr = dl_demod_buffer_[offset];

  switch (config_->ModOrderBits()) {
    case (CommsLib::kQpsk):
      DemodQpskSoftSse(equal_ptr, demul_ptr, config_->OfdmDataNum());
      break;
    case (CommsLib::kQaM16):
      Demod16qamSoftAvx2(equal_ptr, demul_ptr, config_->OfdmDataNum());
      break;
    case (CommsLib::kQaM64):
      Demod64qamSoftAvx2(equal_ptr, demul_ptr, config_->OfdmDataNum());
      break;
    default:
      std::printf("User Task[%d]: Demul - modulation type %s not supported!\n",
                  tid, config_->Modulation().c_str());
  }

  if ((kDebugPrintPerTaskDone == true) || (kDebugPrintDemul == true)) {
    size_t dem_duration_stat = GetTime::Rdtsc() - start_tsc;
    std::printf(
        "User Task[%d]: Demul  (frame %zu, symbol %zu, ant %zu) Duration "
        "%2.4f ms\n",
        tid, frame_id, symbol_id, ant_id,
        GetTime::CyclesToMs(dem_duration_stat, GetTime::MeasureRdtscFreq()));
  }
  if (kPrintLLRData) {
    std::printf("LLR data, symbol_offset: %zu\n", offset);
    for (size_t i = 0; i < config_->OfdmDataNum(); i++) {
      std::printf("%x ", (uint8_t) * (demul_ptr + i));
    }
    std::printf("\n");
  }

  RtAssert(complete_queue_.enqueue(*task_ptok_[tid],
                                   EventData(EventType::kDemul, tag)),
           "Demodulation message enqueue failed");
}

void PhyUe::DoDecode(int tid, size_t tag) {
  const LDPCconfig& ldpc_config = config_->LdpcConfig();
  size_t frame_id = gen_tag_t(tag).frame_id_;
  size_t symbol_id = gen_tag_t(tag).symbol_id_;
  size_t ant_id = gen_tag_t(tag).ant_id_;
  if (kDebugPrintInTask || kDebugPrintDecode) {
    std::printf("User Task[%d]: Decode (frame %zu, symbol %zu, ant %zu)\n", tid,
                frame_id, symbol_id, ant_id);
  }
  size_t start_tsc = GetTime::Rdtsc();

  const size_t frame_slot = frame_id % kFrameWnd;
  size_t dl_symbol_id = config_->Frame().GetDLSymbolIdx(symbol_id);
  size_t total_dl_symbol_id = frame_slot * dl_data_symbol_perframe_ +
                              dl_symbol_id - dl_pilot_symbol_perframe_;
  size_t symbol_ant_offset = total_dl_symbol_id * config_->UeAntNum() + ant_id;

  struct bblib_ldpc_decoder_5gnr_request ldpc_decoder_5gnr_request {};
  struct bblib_ldpc_decoder_5gnr_response ldpc_decoder_5gnr_response {};

  // Decoder setup
  int16_t num_filler_bits = 0;
  int16_t num_channel_llrs = ldpc_config.NumCbCodewLen();

  ldpc_decoder_5gnr_request.numChannelLlrs = num_channel_llrs;
  ldpc_decoder_5gnr_request.numFillerBits = num_filler_bits;
  ldpc_decoder_5gnr_request.maxIterations = ldpc_config.MaxDecoderIter();
  ldpc_decoder_5gnr_request.enableEarlyTermination =
      ldpc_config.EarlyTermination();
  ldpc_decoder_5gnr_request.Zc = ldpc_config.ExpansionFactor();
  ldpc_decoder_5gnr_request.baseGraph = ldpc_config.BaseGraph();
  ldpc_decoder_5gnr_request.nRows = ldpc_config.NumRows();

  int num_msg_bits = ldpc_config.NumCbLen() - num_filler_bits;
  ldpc_decoder_5gnr_response.numMsgBits = num_msg_bits;
  ldpc_decoder_5gnr_response.varNodes = resp_var_nodes_;

  size_t block_error(0);
  for (size_t cb_id = 0; cb_id < config_->LdpcConfig().NumBlocksInSymbol();
       cb_id++) {
    size_t demod_buffer_offset =
        cb_id * ldpc_config.NumCbCodewLen() * config_->ModOrderBits();
    size_t decode_buffer_offset = cb_id * Roundup<64>(config_->NumBytesPerCb());
    auto* llr_buffer_ptr =
        &dl_demod_buffer_[symbol_ant_offset][demod_buffer_offset];
    auto* decoded_buffer_ptr =
        &dl_decode_buffer_[symbol_ant_offset][decode_buffer_offset];
    ldpc_decoder_5gnr_request.varNodes = llr_buffer_ptr;
    ldpc_decoder_5gnr_response.compactedMessageBytes = decoded_buffer_ptr;
    bblib_ldpc_decoder_5gnr(&ldpc_decoder_5gnr_request,
                            &ldpc_decoder_5gnr_response);

    if (config_->ScrambleEnabled()) {
      scrambler_->Descramble(decoded_buffer_ptr, config_->NumBytesPerCb());
    }

    if (kCollectPhyStats) {
      decoded_bits_count_[ant_id][total_dl_symbol_id] +=
          8 * config_->NumBytesPerCb();
      decoded_blocks_count_[ant_id][total_dl_symbol_id]++;
      size_t byte_error(0);
      for (size_t i = 0; i < config_->NumBytesPerCb(); i++) {
        uint8_t rx_byte = decoded_buffer_ptr[i];
        auto tx_byte = static_cast<uint8_t>(config_->GetInfoBits(
            config_->DlBits(), dl_symbol_id, ant_id, cb_id)[i]);
        uint8_t xor_byte(tx_byte ^ rx_byte);
        size_t bit_errors = 0;
        for (size_t j = 0; j < 8; j++) {
          bit_errors += xor_byte & 1;
          xor_byte >>= 1;
        }
        if (rx_byte != tx_byte) {
          byte_error++;
        }

        bit_error_count_[ant_id][total_dl_symbol_id] += bit_errors;
      }
      block_error_count_[ant_id][total_dl_symbol_id] +=
          static_cast<unsigned long>(byte_error > 0);
      block_error += static_cast<unsigned long>(byte_error > 0);
    }

    if (kPrintDecodedData) {
      std::stringstream ss;
      ss << "Decoded data (original byte) in frame " << frame_id << " symbol "
         << symbol_id << " ant " << ant_id << ":\n"
         << std::hex << std::setfill('0');
      for (size_t i = 0; i < config_->NumBytesPerCb(); i++) {
        uint8_t rx_byte = decoded_buffer_ptr[i];
        auto tx_byte = static_cast<uint8_t>(config_->GetInfoBits(
            config_->DlBits(), dl_symbol_id, ant_id, cb_id)[i]);
        ss << std::hex << std::setw(2) << static_cast<int>(rx_byte) << "("
           << static_cast<int>(tx_byte) << ") ";
      }
      ss << std::dec << std::endl;
      std::cout << ss.str();
    }
  }
  if (kCollectPhyStats) {
    decoded_symbol_count_[ant_id]++;
    symbol_error_count_[ant_id] += static_cast<unsigned long>(block_error > 0);
  }

  if ((kDebugPrintPerTaskDone == true) || (kDebugPrintDecode == true)) {
    size_t dec_duration_stat = GetTime::Rdtsc() - start_tsc;
    std::printf(
        "User Task[%d]: Decode (frame %zu, symbol %zu, ant %zu) Duration "
        "%2.4f "
        "ms\n",
        tid, frame_id, symbol_id, ant_id,
        GetTime::CyclesToMs(dec_duration_stat, GetTime::MeasureRdtscFreq()));
  }

  RtAssert(complete_queue_.enqueue(*task_ptok_[tid],
                                   EventData(EventType::kDecode, tag)),
           "Decoding message enqueue failed");
}

//////////////////////////////////////////////////////////
//                   UPLINK Operations                //
//////////////////////////////////////////////////////////

/* Encodes the entire frame for a given user
 * TODO Remove memory allocations
 */
void PhyUe::DoEncode(int tid, size_t tag) {
  const LDPCconfig& ldpc_config = config_->LdpcConfig();
  const size_t frame_id = gen_tag_t(tag).frame_id_;
  const size_t ue_id = gen_tag_t(tag).ue_id_;
  size_t frame_slot = frame_id % kFrameWnd;
  auto& cfg = config_;

  if (kDebugPrintInTask || kDebugPrintEncode) {
    std::printf("User Task[%d]: Encode (frame %zu,       , user %zu)\n", tid,
                frame_id, ue_id);
  }
  size_t start_tsc = GetTime::Rdtsc();

  auto* encoded_buffer_temp =
      static_cast<int8_t*>(Agora_memory::PaddedAlignedAlloc(
          Agora_memory::Alignment_t::kAlign64,
          LdpcEncodingEncodedBufSize(cfg->LdpcConfig().BaseGraph(),
                                     cfg->LdpcConfig().ExpansionFactor())));
  auto* parity_buffer = static_cast<int8_t*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64,
      LdpcEncodingParityBufSize(cfg->LdpcConfig().BaseGraph(),
                                cfg->LdpcConfig().ExpansionFactor())));

  size_t bytes_per_block =
      kEnableMac ? ((ldpc_config.NumCbLen()) >> 3)
                 : Roundup<64>(BitsToBytes(ldpc_config.NumCbLen()));
  size_t encoded_bytes_per_block = (ldpc_config.NumCbCodewLen() + 7) >> 3;
  auto* input_ptr = new int8_t[bytes_per_block +
                               kLdpcHelperFunctionInputBufferSizePaddingBytes];

  // Encode each UL Symbol (Data only)
  for (size_t ul_symbol_id = 0; ul_symbol_id < ul_data_symbol_perframe_;
       ul_symbol_id++) {
    size_t total_ul_symbol_id =
        (frame_slot * ul_data_symbol_perframe_) + ul_symbol_id;
    // Handle each block
    for (size_t cb_id = 0; cb_id < config_->LdpcConfig().NumBlocksInSymbol();
         cb_id++) {
      if (kEnableMac) {
        // All cb's per symbol are included in 1 mac packet
        uint8_t* ul_bits_frame = ul_bits_buffer_[ue_id] +
                                 frame_slot * config_->MacBytesNumPerframe();

        size_t mac_packet_offset = config_->MacPacketLength() * ul_symbol_id;
        size_t cb_offset = cb_id * bytes_per_block;

        std::memcpy(input_ptr,
                    reinterpret_cast<int8_t*>(
                        &ul_bits_frame[mac_packet_offset + cb_offset]),
                    bytes_per_block);
      } else {
        size_t cb_offset =
            (ue_id * cfg->LdpcConfig().NumBlocksInSymbol() + cb_id) *
            bytes_per_block;
        std::memcpy(
            input_ptr,
            &cfg->UlBits()[ul_symbol_id +
                           config_->Frame().ClientUlPilotSymbols()][cb_offset],
            bytes_per_block);
      }

      if (config_->ScrambleEnabled()) {
        scrambler_->Scramble(input_ptr, bytes_per_block);
      }

      LdpcEncodeHelper(ldpc_config.BaseGraph(), ldpc_config.ExpansionFactor(),
                       ldpc_config.NumRows(), encoded_buffer_temp,
                       parity_buffer, input_ptr);

      size_t cb_coded_bytes = ldpc_config.NumCbCodewLen() / cfg->ModOrderBits();
      size_t output_offset = (total_ul_symbol_id * config_->OfdmDataNum()) +
                             (cb_coded_bytes * cb_id);

      AdaptBitsForMod(reinterpret_cast<uint8_t*>(encoded_buffer_temp),
                      &ul_syms_buffer_[ue_id][output_offset],
                      encoded_bytes_per_block, cfg->ModOrderBits());
    }
  }

  std::free(encoded_buffer_temp);
  std::free(parity_buffer);
  delete[] input_ptr;

  if ((kDebugPrintPerTaskDone == true) || (kDebugPrintEncode == true)) {
    size_t enc_duration_stat = GetTime::Rdtsc() - start_tsc;
    std::printf(
        "User Task[%d]: Encode (frame %zu,       , user %zu) Duration "
        "%2.4f "
        "ms\n",
        tid, frame_id, ue_id,
        GetTime::CyclesToMs(enc_duration_stat, GetTime::MeasureRdtscFreq()));
  }

  RtAssert(complete_queue_.enqueue(*task_ptok_[tid],
                                   EventData(EventType::kEncode, tag)),
           "Encoding message enqueue failed");
}

void PhyUe::DoModul(int tid, size_t tag) {
  const size_t frame_id = gen_tag_t(tag).frame_id_;
  const size_t ue_id = gen_tag_t(tag).ue_id_;
  const size_t frame_slot = frame_id % kFrameWnd;

  if (kDebugPrintInTask || kDebugPrintModul) {
    std::printf("User Task[%d]: Modul  (frame %zu,       , user %zu)\n", tid,
                frame_id, ue_id);
  }
  size_t start_tsc = GetTime::Rdtsc();

  for (size_t ch = 0; ch < config_->NumChannels(); ch++) {
    size_t ant_id = ue_id * config_->NumChannels() + ch;
    for (size_t ul_symbol_id = 0; ul_symbol_id < ul_data_symbol_perframe_;
         ul_symbol_id++) {
      size_t total_ul_symbol_id =
          frame_slot * ul_data_symbol_perframe_ + ul_symbol_id;
      complex_float* modul_buf =
          &modul_buffer_[total_ul_symbol_id][ant_id * config_->OfdmDataNum()];
      auto* ul_bits = reinterpret_cast<int8_t*>(
          &ul_syms_buffer_[ant_id]
                          [total_ul_symbol_id * config_->OfdmDataNum()]);
      for (size_t sc = 0; sc < config_->OfdmDataNum(); sc++) {
        modul_buf[sc] =
            ModSingleUint8((uint8_t)ul_bits[sc], config_->ModTable());
      }
    }
  }

  if ((kDebugPrintPerTaskDone == true) || (kDebugPrintModul == true)) {
    size_t mod_duration_stat = GetTime::Rdtsc() - start_tsc;
    std::printf(
        "User Task[%d]: Modul  (frame %zu,       , user %zu) Duration "
        "%2.4f "
        "ms\n",
        tid, frame_id, ue_id,
        GetTime::CyclesToMs(mod_duration_stat, GetTime::MeasureRdtscFreq()));
  }

  RtAssert(complete_queue_.enqueue(*task_ptok_[tid],
                                   EventData(EventType::kModul, tag)),
           "Modulation complete message enqueue failed");
}

void PhyUe::DoIfft(int tid, size_t tag) {
  const size_t frame_id = gen_tag_t(tag).frame_id_;
  const size_t frame_slot = frame_id % kFrameWnd;
  const size_t ue_id = gen_tag_t(tag).ue_id_;

  if (kDebugPrintInTask || kDebugPrintIFFT) {
    std::printf("User Task[%d]: iFFT   (frame %zu,       , user %zu)\n", tid,
                frame_id, ue_id);
  }
  size_t start_tsc = GetTime::Rdtsc();

  for (size_t ch = 0; ch < config_->NumChannels(); ch++) {
    size_t ant_id = ue_id * config_->NumChannels() + ch;
    for (size_t ul_symbol_id = 0; ul_symbol_id < ul_symbol_perframe_;
         ul_symbol_id++) {
      size_t total_ul_symbol_id =
          frame_slot * ul_symbol_perframe_ + ul_symbol_id;
      size_t buff_offset = total_ul_symbol_id * config_->UeAntNum() + ant_id;
      complex_float* ifft_buff = ifft_buffer_[buff_offset];

      std::memset(ifft_buff, 0,
                  sizeof(complex_float) * config_->OfdmDataStart());
      if (ul_symbol_id < config_->Frame().ClientUlPilotSymbols()) {
        std::memcpy(ifft_buff + config_->OfdmDataStart(),
                    config_->UeSpecificPilot()[ant_id],
                    config_->OfdmDataNum() * sizeof(complex_float));
      } else {
        size_t total_ul_data_symbol_id =
            frame_slot * ul_data_symbol_perframe_ + ul_symbol_id -
            config_->Frame().ClientUlPilotSymbols();
        complex_float* modul_buff =
            &modul_buffer_[total_ul_data_symbol_id]
                          [ant_id * config_->OfdmDataNum()];
        std::memcpy(ifft_buff + config_->OfdmDataStart(), modul_buff,
                    config_->OfdmDataNum() * sizeof(complex_float));
      }
      std::memset(ifft_buff + config_->OfdmDataStop(), 0,
                  sizeof(complex_float) * config_->OfdmDataStart());

      CommsLib::IFFT(ifft_buff, config_->OfdmCaNum(), false);

      size_t tx_offset = buff_offset * config_->PacketLength();
      char* cur_tx_buffer = &tx_buffer_[tx_offset];
      auto* pkt = reinterpret_cast<struct Packet*>(cur_tx_buffer);
      auto* tx_data_ptr = reinterpret_cast<std::complex<short>*>(pkt->data_);
      CommsLib::Ifft2tx(ifft_buff, tx_data_ptr, config_->OfdmCaNum(),
                        config_->OfdmTxZeroPrefix(), config_->CpLen(),
                        config_->Scale());
    }
  }

  if ((kDebugPrintPerTaskDone == true) || (kDebugPrintIFFT == true)) {
    size_t ifft_duration_stat = GetTime::Rdtsc() - start_tsc;
    std::printf(
        "User Task[%d]: iFFT   (frame %zu,       , user %zu) Duration "
        "%2.4f "
        "ms\n",
        tid, frame_id, ue_id,
        GetTime::CyclesToMs(ifft_duration_stat, GetTime::MeasureRdtscFreq()));
  }

  RtAssert(complete_queue_.enqueue(*task_ptok_[tid],
                                   EventData(EventType::kIFFT, tag)),
           "Muliplexing message enqueue failed");
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
  rx_thread_num_ = ((kUseArgos == true) && (config_->HwFramer() == false))
                       ? config_->UeNum()
                       : std::min(config_->UeNum(), config_->SocketThreadNum());

  tx_buffer_status_size_ =
      (ul_symbol_perframe_ * config_->UeAntNum() * kFrameWnd);
  tx_buffer_size_ = config_->PacketLength() * tx_buffer_status_size_;
  rx_buffer_status_size_ =
      (dl_symbol_perframe_ + config_->Frame().NumBeaconSyms()) *
      config_->UeAntNum() * kFrameWnd;
  rx_buffer_size_ = config_->PacketLength() * rx_buffer_status_size_;
}

void PhyUe::InitializeUplinkBuffers() {
  // initialize ul data buffer
  ul_bits_buffer_size_ = kFrameWnd * config_->MacBytesNumPerframe();
  ul_bits_buffer_.Malloc(config_->UeAntNum(), ul_bits_buffer_size_,
                         Agora_memory::Alignment_t::kAlign64);
  ul_bits_buffer_status_.Calloc(config_->UeAntNum(), kFrameWnd,
                                Agora_memory::Alignment_t::kAlign64);
  ul_syms_buffer_size_ =
      kFrameWnd * ul_data_symbol_perframe_ * config_->OfdmDataNum();
  ul_syms_buffer_.Calloc(config_->UeAntNum(), ul_syms_buffer_size_,
                         Agora_memory::Alignment_t::kAlign64);

  // initialize modulation buffer
  modul_buffer_.Calloc(ul_data_symbol_perframe_ * kFrameWnd,
                       config_->OfdmDataNum() * config_->UeAntNum(),
                       Agora_memory::Alignment_t::kAlign64);

  // initialize IFFT buffer
  size_t ifft_buffer_block_num =
      config_->UeAntNum() * ul_symbol_perframe_ * kFrameWnd;
  ifft_buffer_.Calloc(ifft_buffer_block_num, config_->OfdmCaNum(),
                      Agora_memory::Alignment_t::kAlign64);

  AllocBuffer1d(&tx_buffer_, tx_buffer_size_,
                Agora_memory::Alignment_t::kAlign64, 0);
  AllocBuffer1d(&tx_buffer_status_, tx_buffer_status_size_,
                Agora_memory::Alignment_t::kAlign64, 1);
}

void PhyUe::FreeUplinkBuffers() {
  ul_bits_buffer_.Free();
  ul_bits_buffer_status_.Free();
  ul_syms_buffer_.Free();
  modul_buffer_.Free();
  ifft_buffer_.Free();

  FreeBuffer1d(&tx_buffer_);
  FreeBuffer1d(&tx_buffer_status_);
}

void PhyUe::InitializeDownlinkBuffers() {
  // initialize rx buffer
  rx_buffer_.Malloc(rx_thread_num_, rx_buffer_size_,
                    Agora_memory::Alignment_t::kAlign64);
  rx_buffer_status_.Calloc(rx_thread_num_, rx_buffer_status_size_,
                           Agora_memory::Alignment_t::kAlign64);
  AllocBuffer1d(&rx_samps_tmp_, config_->SampsPerSymbol(),
                Agora_memory::Alignment_t::kAlign64, 1);

  // initialize FFT buffer
  size_t fft_buffer_block_num =
      config_->UeAntNum() * dl_symbol_perframe_ * kFrameWnd;
  fft_buffer_.Calloc(fft_buffer_block_num, config_->OfdmCaNum(),
                     Agora_memory::Alignment_t::kAlign64);

  // initialize CSI buffer
  csi_buffer_.resize(config_->UeAntNum() * kFrameWnd);
  for (auto& i : csi_buffer_) {
    i.resize(config_->OfdmDataNum());

    for (auto& csi_value : i) {
      if (config_->Frame().ClientDlPilotSymbols() == 0) {
        csi_value.re = 1;
        csi_value.im = 0;
      } else {
        csi_value.re = 0;
        csi_value.im = 0;
      }
    }
  }

  decoded_symbol_count_ = std::vector<size_t>(config_->UeAntNum());
  symbol_error_count_ = std::vector<size_t>(config_->UeAntNum());

  if (dl_data_symbol_perframe_ > 0) {
    // initialize equalized data buffer
    const size_t task_buffer_symbol_num_dl =
        dl_data_symbol_perframe_ * kFrameWnd;
    size_t buffer_size = config_->UeAntNum() * task_buffer_symbol_num_dl;
    equal_buffer_.resize(buffer_size);
    for (auto& i : equal_buffer_) {
      i.resize(config_->OfdmDataNum());
    }

    // initialize demod buffer
    dl_demod_buffer_.Calloc(buffer_size, config_->OfdmDataNum() * kMaxModType,
                            Agora_memory::Alignment_t::kAlign64);

    // initialize decode buffer
    dl_decode_buffer_.resize(buffer_size);
    for (auto& i : dl_decode_buffer_) {
      i.resize(Roundup<64>(config_->NumBytesPerCb()) *
               config_->LdpcConfig().NumBlocksInSymbol());
    }
    resp_var_nodes_ = static_cast<int16_t*>(Agora_memory::PaddedAlignedAlloc(
        Agora_memory::Alignment_t::kAlign64, 1024 * 1024 * sizeof(int16_t)));

    decoded_bits_count_.Calloc(config_->UeAntNum(), task_buffer_symbol_num_dl,
                               Agora_memory::Alignment_t::kAlign64);
    bit_error_count_.Calloc(config_->UeAntNum(), task_buffer_symbol_num_dl,
                            Agora_memory::Alignment_t::kAlign64);

    decoded_blocks_count_.Calloc(config_->UeAntNum(), task_buffer_symbol_num_dl,
                                 Agora_memory::Alignment_t::kAlign64);
    block_error_count_.Calloc(config_->UeAntNum(), task_buffer_symbol_num_dl,
                              Agora_memory::Alignment_t::kAlign64);
  }
}

void PhyUe::FreeDownlinkBuffers() {
  rx_buffer_.Free();
  rx_buffer_status_.Free();
  FreeBuffer1d(&rx_samps_tmp_);
  fft_buffer_.Free();

  if (dl_data_symbol_perframe_ > 0) {
    dl_demod_buffer_.Free();
    std::free(resp_var_nodes_);

    decoded_bits_count_.Free();
    bit_error_count_.Free();

    decoded_blocks_count_.Free();
    block_error_count_.Free();
  }
}

void PhyUe::PrintPerTaskDone(PrintType print_type, size_t frame_id,
                             size_t symbol_id, size_t ant) {
  // if (kDebugPrintPerTaskDone == true) {
  if (true) {
    switch (print_type) {
      case (PrintType::kPacketRX):
        std::printf("PhyUE [frame %zu symbol %zu ant %zu]: Rx packet\n",
                    frame_id, symbol_id, ant);
        break;

      case (PrintType::kPacketTX):
        std::printf(
            "PhyUE [frame %zu symbol %zu ant %zu]: %zu User Tx "
            "finished\n",
            frame_id, symbol_id, ant, tx_counters_.GetTaskCount(frame_id) + 1);
        break;

      case (PrintType::kFFTPilots):
        std::printf(
            "PhyUE [frame %zu symbol %zu ant %zu]: Pilot FFT Equalization "
            "done\n",
            frame_id, symbol_id, ant);
        break;

      case (PrintType::kFFTData):
        std::printf(
            "PhyUE [frame %zu symbol %zu ant %zu]: Data FFT Equalization "
            "done\n",
            frame_id, symbol_id, ant);
        break;

      case (PrintType::kDemul):
        std::printf("PhyUE [frame %zu symbol %zu ant %zu]: Demul done\n",
                    frame_id, symbol_id, ant);
        break;

      case (PrintType::kDecode):
        std::printf("PhyUE [frame %zu symbol %zu ant %zu]: Decoding done\n",
                    frame_id, symbol_id, ant);
        break;

      case (PrintType::kEncode):
        std::printf("PhyUE [frame %zu symbol %zu ant %zu]: Encoding done\n",
                    frame_id, symbol_id, ant);
        break;

      case (PrintType::kModul):
        std::printf("PhyUE [frame %zu symbol %zu ant %zu]: Modulation done\n",
                    frame_id, symbol_id, ant);
        break;

      case (PrintType::kIFFT):
        std::printf("PhyUE [frame %zu symbol %zu ant %zu]: iFFT done\n",
                    frame_id, symbol_id, ant);
        break;

      default:
        std::printf("Wrong task type in task done print!");
    }
  }
}

void PhyUe::PrintPerSymbolDone(PrintType print_type, size_t frame_id,
                               size_t symbol_id) {
  // if (kDebugPrintPerSymbolDone == true) {
  if (true) {
    switch (print_type) {
      case (PrintType::kFFTPilots):
        std::printf(
            "PhyUe [frame %zu symbol %zu + %.3f ms]: Pilot FFT complete for "
            "%zu antennas\n",
            frame_id, symbol_id,
            this->stats_->MasterGetMsSince(TsType::kFirstSymbolRX, frame_id),
            fft_dlpilot_counters_.GetTaskCount(frame_id, symbol_id));
        break;

      case (PrintType::kFFTData):
        std::printf(
            "PhyUe [frame %zu symbol %zu + %.3f ms]: Data FFT complete for "
            "%zu antennas\n",
            frame_id, symbol_id,
            this->stats_->MasterGetMsSince(TsType::kFirstSymbolRX, frame_id),
            fft_dldata_counters_.GetTaskCount(frame_id, symbol_id));
        break;

      case (PrintType::kDemul):
        std::printf(
            "PhyUe [frame %zu symbol %zu + %.3f ms]: Demul completed for "
            "%zu "
            "antennas\n",
            frame_id, symbol_id,
            this->stats_->MasterGetMsSince(TsType::kFirstSymbolRX, frame_id),
            demul_counters_.GetTaskCount(frame_id, symbol_id));
        break;

      case (PrintType::kDecode):
        std::printf(
            "PhyUe [frame %zu symbol %zu + %.3f ms]: Decoding completed "
            "for "
            "%zu antennas\n",
            frame_id, symbol_id,
            this->stats_->MasterGetMsSince(TsType::kFirstSymbolRX, frame_id),
            decode_counters_.GetTaskCount(frame_id, symbol_id));
        break; /*
      case (PrintType::kPacketToMac):
        std::printf(
            "Main [frame %zu symbol %zu + %.3f ms]: Completed MAC TX, "
            "%zu symbols done\n",
            frame_id, symbol_id,
            this->stats_->MasterGetMsSince(TsType::kFirstSymbolRX,
      frame_id), tomac_counters_.GetSymbolCount(frame_id) + 1); break;
        */
      default:
        std::printf("Wrong task type in symbol done print!");
    }
  }
}

void PhyUe::PrintPerFrameDone(PrintType print_type, size_t frame_id) {
  if (kDebugPrintPerFrameDone == true) {
    // if (true) {
    switch (print_type) {
      case (PrintType::kPacketRX):
        std::printf("PhyUE [frame %zu + %.2f ms]: Received all packets\n",
                    frame_id,
                    this->stats_->MasterGetDeltaMs(
                        TsType::kRXDone, TsType::kFirstSymbolRX, frame_id));
        break;

      case (PrintType::kPacketRXPilots):
        std::printf("PhyUE [frame %zu + %.2f ms]: Received all pilots\n",
                    frame_id,
                    this->stats_->MasterGetDeltaMs(
                        TsType::kPilotAllRX, TsType::kFirstSymbolRX, frame_id));
        break;

      case (PrintType::kPacketTX):
        std::printf("PhyUE [frame %zu + %.2f ms]: Completed TX\n", frame_id,
                    this->stats_->MasterGetDeltaMs(
                        TsType::kTXDone, TsType::kFirstSymbolRX, frame_id));
        break;

      case (PrintType::kFFTPilots):
        std::printf(
            "PhyUE [frame %zu + %.2f ms]: Pilot FFT finished\n", frame_id,
            this->stats_->MasterGetDeltaMs(TsType::kFFTPilotsDone,
                                           TsType::kFirstSymbolRX, frame_id));
        break;

      case (PrintType::kFFTData):
        std::printf("PhyUE [frame %zu + %.2f ms]: Data FFT finished\n",
                    frame_id,
                    this->stats_->MasterGetDeltaMs(
                        TsType::kFFTDone, TsType::kFirstSymbolRX, frame_id));
        break;

      case (PrintType::kDemul):
        std::printf("PhyUE [frame %zu + %.2f ms]: Completed demodulation\n",
                    frame_id,
                    this->stats_->MasterGetDeltaMs(
                        TsType::kDemulDone, TsType::kFirstSymbolRX, frame_id));
        break;

      case (PrintType::kDecode):
        std::printf("PhyUE [frame %zu + %.2f ms]: Completed decoding\n",
                    frame_id,
                    this->stats_->MasterGetDeltaMs(
                        TsType::kDecodeDone, TsType::kFirstSymbolRX, frame_id));
        break;

      case (PrintType::kEncode):
        std::printf("PhyUE [frame %zu + %.2f ms]: Completed encoding\n",
                    frame_id,
                    this->stats_->MasterGetDeltaMs(
                        TsType::kEncodeDone, TsType::kFirstSymbolRX, frame_id));
        break;

      case (PrintType::kModul):
        std::printf("PhyUE [frame %zu + %.2f ms]: Completed modulation\n",
                    frame_id,
                    this->stats_->MasterGetDeltaMs(
                        TsType::kModulDone, TsType::kFirstSymbolRX, frame_id));
        break;

      case (PrintType::kIFFT):
        std::printf("PhyUE [frame %zu + %.2f ms]: Completed iFFT\n", frame_id,
                    this->stats_->MasterGetDeltaMs(
                        TsType::kIFFTDone, TsType::kFirstSymbolRX, frame_id));
        break;

      /*
      case (PrintType::kPacketTXFirst): std::printf(
            "PhyUE [frame %zu + %.2f ms]: Completed TX of first symbol\n",
            frame_id,
            this->stats_->MasterGetDeltaMs(TsType::kTXProcessedFirst,
                                           TsType::kFirstSymbolRX,
      frame_id)); break;

      case (PrintType::kPacketToMac): std::printf(
            "PhyUE [frame %zu + %.2f ms]: Completed MAC TX \n", frame_id,
            this->stats_->MasterGetMsSince(TsType::kFirstSymbolRX,
      frame_id)); break; */
      default:
        std::printf("PhyUE: Wrong task type in frame done print!\n");
    }
  }
}

void PhyUe::GetDemulData(long long** ptr, int* size) {
  *ptr = (long long*)&equal_buffer_[max_equaled_frame_ *
                                    dl_data_symbol_perframe_][0];
  *size = config_->UeAntNum() * config_->OfdmCaNum();
}

void PhyUe::GetEqualData(float** ptr, int* size, int ue_id) {
  *ptr = (float*)&equal_buffer_[max_equaled_frame_ * dl_data_symbol_perframe_ *
                                    config_->UeAntNum() +
                                ue_id][0];
  *size = config_->UeAntNum() * config_->OfdmDataNum() * 2;
}

void PhyUe::FrameInit(size_t frame) {
  std::uint8_t initial =
      static_cast<std::uint8_t>(FrameTasksFlags::kNoWorkComplete);

  if (config_->Frame().NumDLSyms() == 0) {
    initial |= static_cast<std::uint8_t>(FrameTasksFlags::kDownlinkComplete);
  }

  if (kEnableMac == false) {
    initial |= static_cast<std::uint8_t>(FrameTasksFlags::kMacTxComplete);
  }
  frame_tasks_.at(frame % kFrameWnd) = initial;
}

bool PhyUe::FrameComplete(size_t frame, FrameTasksFlags complete) {
  frame_tasks_.at(frame % kFrameWnd) |= static_cast<std::uint8_t>(complete);
  bool is_complete =
      (frame_tasks_.at(frame % kFrameWnd) ==
       static_cast<std::uint8_t>(FrameTasksFlags::kFrameComplete));

  // if (is_complete == true) {
  //  std::printf("**** Frame %zu complete with task %d\n", frame,
  //              static_cast<std::uint8_t>(complete));
  //}
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
