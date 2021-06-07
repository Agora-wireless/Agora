/**
 * @file phy-ue.cc
 * @brief Implementation file for the phy ue class
 */
#include "phy-ue.h"

#include <memory>

#include "phy_ldpc_decoder_5gnr.h"
#include "phy_stats.h"
#include "scrambler.h"
#include "signal_handler.h"
#include "utils_ldpc.h"

/* Print debug work */
static constexpr bool kDebugPrintPacketsFromMac = false;
static constexpr bool kDebugPrintPacketsToMac = false;

static const size_t kDefaultQueueSize = 36;

PhyUe::PhyUe(Config* config)
    : stats_(std::make_unique<Stats>(config)),
      phy_stats_(std::make_unique<PhyStats>(config)),
      demod_buffer_(kFrameWnd, config->Frame().NumDLSyms(), config->UeAntNum(),
                    kMaxModType * config->OfdmDataNum()),
      decoded_buffer_(kFrameWnd, config->Frame().NumDLSyms(),
                      config->UeAntNum(),
                      config->LdpcConfig().NumBlocksInSymbol() *
                          Roundup<64>(config->NumBytesPerCb())) {
  srand(time(nullptr));

  // TODO take into account the UeAntOffset to allow for multiple PhyUe
  // instances
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

  work_producer_token_ =
      std::make_unique<moodycamel::ProducerToken>(work_queue_);

  ru_ = std::make_unique<RadioTxRx>(config_, rx_thread_num_,
                                    config_->CoreOffset() + 1, &complete_queue_,
                                    &tx_queue_, rx_ptoks_ptr_, tx_ptoks_ptr_);

  // uplink buffers init (tx)
  InitializeUplinkBuffers();
  // downlink buffers init (rx)
  InitializeDownlinkBuffers();

  size_t core_offset_worker = config_->CoreOffset() + 1 + rx_thread_num_;
  if (kEnableMac == true) {
    mac_thread_ = std::make_unique<MacThreadClient>(
        config_, core_offset_worker, decoded_buffer_, &ul_bits_buffer_,
        &ul_bits_buffer_status_, &to_mac_queue_, &complete_queue_);

    core_offset_worker++;
    mac_std_thread_ =
        std::thread(&MacThreadClient::RunEventLoop, mac_thread_.get());
  }

  for (size_t i = 0; i < config_->WorkerThreadNum(); i++) {
    auto new_worker = std::make_unique<UeWorker>(
        i, *config_, *stats_, *phy_stats_, complete_queue_, work_queue_,
        *work_producer_token_.get(), ul_bits_buffer_, ul_syms_buffer_,
        modul_buffer_, ifft_buffer_, tx_buffer_, rx_buffer_, rx_buffer_status_,
        csi_buffer_, equal_buffer_, non_null_sc_ind_, fft_buffer_,
        demod_buffer_, decoded_buffer_, ue_pilot_vec_);

    new_worker->Start(core_offset_worker);
    workers_.push_back(std::move(new_worker));
  }

  // initilize all kinds of checkers
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

  encode_counter_.Init(ul_data_symbol_perframe_, config_->UeAntNum());
  modulation_counters_.Init(ul_data_symbol_perframe_, config_->UeAntNum());
  ifft_counters_.Init(ul_symbol_perframe_, config_->UeAntNum());

  // This usage doesn't effect the user num_reciprocity_pkts_per_frame_;
  rx_counters_.num_pkts_per_frame_ =
      config_->UeAntNum() *
      (config_->Frame().NumDLSyms() + config_->Frame().NumBeaconSyms());
  rx_counters_.num_pilot_pkts_per_frame_ =
      config_->UeAntNum() * config_->Frame().ClientDlPilotSymbols();

  rx_downlink_deferral_.resize(kFrameWnd);
}

PhyUe::~PhyUe() {
  for (size_t i = 0; i < config_->WorkerThreadNum(); i++) {
    std::printf("Joining Phy worker: %zu : %zu\n", i,
                config_->WorkerThreadNum());
    workers_.at(i)->Stop();
  }
  workers_.clear();

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
  const size_t frame_slot = frame_id % kFrameWnd;

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
  size_t ifft_next_frame = 0;
  std::vector<size_t> ifft_frame_status(kFrameWnd, SIZE_MAX);
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

          if (rx_counters_.num_pkts_.at(frame_slot) == 0) {
            this->stats_->MasterSetTsc(TsType::kFirstSymbolRX, frame_id);
            if (kDebugPrintPerFrameStart) {
              const size_t prev_frame_slot =
                  (frame_slot + kFrameWnd - 1) % kFrameWnd;
              std::printf(
                  "PhyUe [frame %zu + %.2f ms since last frame]: Received "
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
                // Schedule the Uplink tasks
                for (size_t symbol_idx = 0;
                     symbol_idx < config_->Frame().NumULSyms(); symbol_idx++) {
                  if (symbol_idx < config_->Frame().ClientUlPilotSymbols()) {
                    EventData do_ifft_task(
                        EventType::kIFFT,
                        gen_tag_t::FrmSymUe(
                            frame_id, config_->Frame().GetULSymbol(symbol_idx),
                            ue_id)
                            .tag_);
                    ScheduleWork(do_ifft_task);
                  } else {
                    EventData do_encode_task(
                        EventType::kEncode,
                        gen_tag_t::FrmSymUe(
                            frame_id, config_->Frame().GetULSymbol(symbol_idx),
                            ue_id)
                            .tag_);
                    ScheduleWork(do_encode_task);
                  }
                }
              }
            }
          }

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
          const size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          const size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;

          if (kDebugPrintPacketsToMac) {
            std::printf(
                "PhyUe: sent decoded packet for (frame %zu, symbol %zu) "
                "to MAC\n",
                frame_id, symbol_id);
          }

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
        } break;

        case EventType::kPacketFromMac: {
          // This is an entrie frame (multiple mac packets)
          size_t ue_id = rx_tag_t(event.tags_[0]).tid_;
          size_t radio_buf_id = rx_tag_t(event.tags_[0]).offset_;
          RtAssert(radio_buf_id == expected_frame_id_from_mac_ % kFrameWnd);

          auto* pkt = reinterpret_cast<MacPacket*>(
              &ul_bits_buffer_[ue_id][radio_buf_id *
                                      config_->UlMacBytesNumPerframe()]);
          RtAssert(pkt->frame_id_ == expected_frame_id_from_mac_,
                   "PhyUe: Incorrect frame ID from MAC");
          current_frame_user_num_ =
              (current_frame_user_num_ + 1) % config_->UeAntNum();
          if (current_frame_user_num_ == 0) {
            expected_frame_id_from_mac_++;
          }
          config_->UpdateModCfgs(pkt->rb_indicator_.mod_order_bits_);

          if (kDebugPrintPacketsFromMac) {
            std::printf(
                "PhyUe: received packet for frame %u with modulation %zu\n",
                pkt->frame_id_, pkt->rb_indicator_.mod_order_bits_);
            std::stringstream ss;

            for (size_t ul_data_symbol = 0;
                 ul_data_symbol < config_->Frame().NumUlDataSyms();
                 ul_data_symbol++) {
              ss << "PhyUe: kPacketFromMac, frame " << pkt->frame_id_
                 << ", symbol " << std::to_string(pkt->symbol_id_) << " crc "
                 << std::to_string(pkt->crc_) << " bytes: ";
              for (size_t i = 0; i < pkt->datalen_; i++) {
                ss << std::to_string(
                          (reinterpret_cast<uint8_t*>(pkt->data_)[i]))
                   << ", ";
              }
              ss << std::endl;
              pkt = reinterpret_cast<MacPacket*>(
                  reinterpret_cast<uint8_t*>(pkt) + config_->MacPacketLength());
            }
            std::printf("%s\n", ss.str().c_str());
          }

        } break;

        case EventType::kEncode: {
          size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;
          size_t ue_id = gen_tag_t(event.tags_[0]).ue_id_;

          PrintPerTaskDone(PrintType::kEncode, frame_id, symbol_id, ue_id);

          // Schedule the modul
          EventData do_modul_task(EventType::kModul, event.tags_[0]);
          ScheduleWork(do_modul_task);

          bool symbol_complete =
              encode_counter_.CompleteTask(frame_id, symbol_id);
          if (symbol_complete == true) {
            PrintPerSymbolDone(PrintType::kEncode, frame_id, symbol_id);

            bool encode_complete = encode_counter_.CompleteSymbol(frame_id);
            if (encode_complete == true) {
              this->stats_->MasterSetTsc(TsType::kEncodeDone, frame_id);
              PrintPerFrameDone(PrintType::kEncode, frame_id);
              encode_counter_.Reset(frame_id);
            }
          }
        } break;

        case EventType::kModul: {
          size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;
          size_t ue_id = gen_tag_t(event.tags_[0]).ue_id_;

          PrintPerTaskDone(PrintType::kModul, frame_id, symbol_id, ue_id);

          EventData do_ifft_task(
              EventType::kIFFT,
              gen_tag_t::FrmSymUe(frame_id, symbol_id, ue_id).tag_);
          ScheduleWork(do_ifft_task);

          bool symbol_complete =
              modulation_counters_.CompleteTask(frame_id, symbol_id);
          if (symbol_complete == true) {
            PrintPerSymbolDone(PrintType::kModul, frame_id, symbol_id);

            bool mod_complete = modulation_counters_.CompleteSymbol(frame_id);
            if (mod_complete == true) {
              this->stats_->MasterSetTsc(TsType::kModulDone, frame_id);
              PrintPerFrameDone(PrintType::kModul, frame_id);
              modulation_counters_.Reset(frame_id);
            }
          }
        } break;

        case EventType::kIFFT: {
          size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;
          size_t ue_id = gen_tag_t(event.tags_[0]).ue_id_;

          PrintPerTaskDone(PrintType::kIFFT, frame_id, symbol_id, ue_id);

          bool symbol_complete =
              ifft_counters_.CompleteTask(frame_id, symbol_id);
          if (symbol_complete == true) {
            PrintPerSymbolDone(PrintType::kIFFT, frame_id, symbol_id);

            bool ifft_complete = ifft_counters_.CompleteSymbol(frame_id);
            if (ifft_complete == true) {
              this->stats_->MasterSetTsc(TsType::kIFFTDone, frame_id);
              PrintPerFrameDone(PrintType::kIFFT, frame_id);
              ifft_counters_.Reset(frame_id);

              // Schedule Transmit in frame order
              ifft_frame_status.at(frame_id % kFrameWnd) = frame_id;
              for (size_t i = 0u; (i < kFrameWnd); i++) {
                size_t ifft_stat_id = (ifft_next_frame % kFrameWnd);
                if (ifft_frame_status.at(ifft_stat_id) == ifft_next_frame) {
                  // Schedule TX for all users (by packet)
                  for (size_t user = 0u; user < config_->UeAntNum(); user++) {
                    EventData do_tx_task(
                        EventType::kPacketTX,
                        gen_tag_t::FrmSymUe(ifft_next_frame, 0, user).tag_);
                    ScheduleTask(do_tx_task, &tx_queue_,
                                 *tx_ptoks_ptr_[ue_id % rx_thread_num_]);
                  }
                  ifft_next_frame++;
                } else { /* Done */
                  break;
                }
              }  // End for next transmit frame
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
                   "PhyUe: Unexpected frame was transmitted!");

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
  ul_bits_buffer_size_ = kFrameWnd * config_->UlMacBytesNumPerframe();
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

  if (dl_data_symbol_perframe_ > 0) {
    // initialize equalized data buffer
    const size_t task_buffer_symbol_num_dl =
        dl_data_symbol_perframe_ * kFrameWnd;
    size_t buffer_size = config_->UeAntNum() * task_buffer_symbol_num_dl;
    equal_buffer_.resize(buffer_size);
    for (auto& i : equal_buffer_) {
      i.resize(config_->OfdmDataNum());
    }
  }
}

void PhyUe::FreeDownlinkBuffers() {
  rx_buffer_.Free();
  rx_buffer_status_.Free();
  fft_buffer_.Free();
}

void PhyUe::PrintPerTaskDone(PrintType print_type, size_t frame_id,
                             size_t symbol_id, size_t ant) {
  if (kDebugPrintPerTaskDone == true) {
    // if (true) {
    switch (print_type) {
      case (PrintType::kPacketRX):
        std::printf("PhyUe [frame %zu symbol %zu ant %zu]: Rx packet\n",
                    frame_id, symbol_id, ant);
        break;

      case (PrintType::kPacketTX):
        std::printf(
            "PhyUe [frame %zu symbol %zu ant %zu]: %zu User Tx "
            "finished\n",
            frame_id, symbol_id, ant, tx_counters_.GetTaskCount(frame_id) + 1);
        break;

      case (PrintType::kFFTPilots):
        std::printf(
            "PhyUe [frame %zu symbol %zu ant %zu]: Pilot FFT Equalization "
            "done\n",
            frame_id, symbol_id, ant);
        break;

      case (PrintType::kFFTData):
        std::printf(
            "PhyUe [frame %zu symbol %zu ant %zu]: Data FFT Equalization "
            "done\n",
            frame_id, symbol_id, ant);
        break;

      case (PrintType::kDemul):
        std::printf("PhyUe [frame %zu symbol %zu ant %zu]: Demul done\n",
                    frame_id, symbol_id, ant);
        break;

      case (PrintType::kDecode):
        std::printf("PhyUe [frame %zu symbol %zu ant %zu]: Decoding done\n",
                    frame_id, symbol_id, ant);
        break;

      case (PrintType::kEncode):
        std::printf("PhyUe [frame %zu symbol %zu ant %zu]: Encoding done\n",
                    frame_id, symbol_id, ant);
        break;

      case (PrintType::kModul):
        std::printf("PhyUe [frame %zu symbol %zu ant %zu]: Modulation done\n",
                    frame_id, symbol_id, ant);
        break;

      case (PrintType::kIFFT):
        std::printf("PhyUe [frame %zu symbol %zu ant %zu]: iFFT done\n",
                    frame_id, symbol_id, ant);
        break;

      default:
        std::printf("Wrong task type in task done print!");
    }
  }
}

void PhyUe::PrintPerSymbolDone(PrintType print_type, size_t frame_id,
                               size_t symbol_id) {
  if (kDebugPrintPerSymbolDone == true) {
    // if (true) {
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
            "%zu antennas\n",
            frame_id, symbol_id,
            this->stats_->MasterGetMsSince(TsType::kFirstSymbolRX, frame_id),
            demul_counters_.GetTaskCount(frame_id, symbol_id));
        break;

      case (PrintType::kDecode):
        std::printf(
            "PhyUe [frame %zu symbol %zu + %.3f ms]: Decoding completed "
            "for %zu antennas\n",
            frame_id, symbol_id,
            this->stats_->MasterGetMsSince(TsType::kFirstSymbolRX, frame_id),
            decode_counters_.GetTaskCount(frame_id, symbol_id));
        break;
      case (PrintType::kEncode):
        std::printf(
            "PhyUe [frame %zu symbol %zu + %.3f ms]: Data Encode complete "
            "for "
            "%zu antennas\n",
            frame_id, symbol_id,
            this->stats_->MasterGetMsSince(TsType::kFirstSymbolRX, frame_id),
            encode_counter_.GetTaskCount(frame_id, symbol_id));
        break;

      case (PrintType::kModul):
        std::printf(
            "PhyUe [frame %zu symbol %zu + %.3f ms]: Modul completed for "
            "symbol %zu antennas\n",
            frame_id, symbol_id,
            this->stats_->MasterGetMsSince(TsType::kFirstSymbolRX, frame_id),
            modulation_counters_.GetTaskCount(frame_id, symbol_id));
        break;

      case (PrintType::kIFFT):
        std::printf(
            "PhyUe [frame %zu symbol %zu + %.3f ms]: iFFT completed for "
            "symbol %zu antennas\n",
            frame_id, symbol_id,
            this->stats_->MasterGetMsSince(TsType::kFirstSymbolRX, frame_id),
            ifft_counters_.GetTaskCount(frame_id, symbol_id));
        break;

        /*
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
        std::printf("PhyUe [frame %zu + %.2f ms]: Received all packets\n",
                    frame_id,
                    this->stats_->MasterGetDeltaMs(
                        TsType::kRXDone, TsType::kFirstSymbolRX, frame_id));
        break;

      case (PrintType::kPacketRXPilots):
        std::printf("PhyUe [frame %zu + %.2f ms]: Received all pilots\n",
                    frame_id,
                    this->stats_->MasterGetDeltaMs(
                        TsType::kPilotAllRX, TsType::kFirstSymbolRX, frame_id));
        break;

      case (PrintType::kPacketTX):
        std::printf("PhyUe [frame %zu + %.2f ms]: Completed TX\n", frame_id,
                    this->stats_->MasterGetDeltaMs(
                        TsType::kTXDone, TsType::kFirstSymbolRX, frame_id));
        break;

      case (PrintType::kFFTPilots):
        std::printf(
            "PhyUe [frame %zu + %.2f ms]: Pilot FFT finished\n", frame_id,
            this->stats_->MasterGetDeltaMs(TsType::kFFTPilotsDone,
                                           TsType::kFirstSymbolRX, frame_id));
        break;

      case (PrintType::kFFTData):
        std::printf("PhyUe [frame %zu + %.2f ms]: Data FFT finished\n",
                    frame_id,
                    this->stats_->MasterGetDeltaMs(
                        TsType::kFFTDone, TsType::kFirstSymbolRX, frame_id));
        break;

      case (PrintType::kDemul):
        std::printf("PhyUe [frame %zu + %.2f ms]: Completed demodulation\n",
                    frame_id,
                    this->stats_->MasterGetDeltaMs(
                        TsType::kDemulDone, TsType::kFirstSymbolRX, frame_id));
        break;

      case (PrintType::kDecode):
        std::printf("PhyUe [frame %zu + %.2f ms]: Completed decoding\n",
                    frame_id,
                    this->stats_->MasterGetDeltaMs(
                        TsType::kDecodeDone, TsType::kFirstSymbolRX, frame_id));
        break;

      case (PrintType::kEncode):
        std::printf("PhyUe [frame %zu + %.2f ms]: Completed encoding\n",
                    frame_id,
                    this->stats_->MasterGetDeltaMs(
                        TsType::kEncodeDone, TsType::kFirstSymbolRX, frame_id));
        break;

      case (PrintType::kModul):
        std::printf("PhyUe [frame %zu + %.2f ms]: Completed modulation\n",
                    frame_id,
                    this->stats_->MasterGetDeltaMs(
                        TsType::kModulDone, TsType::kFirstSymbolRX, frame_id));
        break;

      case (PrintType::kIFFT):
        std::printf("PhyUe [frame %zu + %.2f ms]: Completed iFFT\n", frame_id,
                    this->stats_->MasterGetDeltaMs(
                        TsType::kIFFTDone, TsType::kFirstSymbolRX, frame_id));
        break;

      /*
      case (PrintType::kPacketTXFirst): std::printf(
            "PhyUe [frame %zu + %.2f ms]: Completed TX of first symbol\n",
            frame_id,
            this->stats_->MasterGetDeltaMs(TsType::kTXProcessedFirst,
                                           TsType::kFirstSymbolRX,
      frame_id)); break;

      case (PrintType::kPacketToMac): std::printf(
            "PhyUe [frame %zu + %.2f ms]: Completed MAC TX \n", frame_id,
            this->stats_->MasterGetMsSince(TsType::kFirstSymbolRX,
      frame_id)); break; */
      default:
        std::printf("PhyUe: Wrong task type in frame done print!\n");
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
