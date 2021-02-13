/**
 * @file phy-ue.cc
 * @brief Implementation file for the phy ue class
 */
#include "phy-ue.h"

#include <memory>

#include "phy_ldpc_decoder_5gnr.h"
#include "scrambler.h"
#include "utils_ldpc.h"

using namespace arma;

static constexpr bool kDebugPrintPacketsFromMac = false;
static constexpr bool kDebugPrintPacketsToMac = false;
static constexpr bool kPrintLLRData = false;
static constexpr bool kPrintDecodedData = false;
static constexpr bool kPrintDownlinkPilotStats = false;
static constexpr bool kPrintEqualizedSymbols = false;
static constexpr size_t kRecordFrameIndex = 1000;
static const size_t kDefaultQueueSize = 36;

PhyUe::PhyUe(Config* config) {
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

  fft_queue_ = moodycamel::ConcurrentQueue<EventData>(
      kFrameWnd * dl_symbol_perframe_ * config_->UeAntNum() *
      kDefaultQueueSize);
  demul_queue_ = moodycamel::ConcurrentQueue<EventData>(
      kFrameWnd * dl_data_symbol_perframe_ * config_->UeAntNum() *
      kDefaultQueueSize);
  decode_queue_ = moodycamel::ConcurrentQueue<EventData>(
      kFrameWnd * dl_data_symbol_perframe_ * config_->UeAntNum() *
      kDefaultQueueSize);
  message_queue_ = moodycamel::ConcurrentQueue<EventData>(
      kFrameWnd * config_->Frame().NumTotalSyms() * config_->UeAntNum() *
      kDefaultQueueSize);
  encode_queue_ = moodycamel::ConcurrentQueue<EventData>(
      kFrameWnd * config_->UeNum() * kDefaultQueueSize);
  modul_queue_ = moodycamel::ConcurrentQueue<EventData>(
      kFrameWnd * config_->UeNum() * kDefaultQueueSize);
  ifft_queue_ = moodycamel::ConcurrentQueue<EventData>(
      kFrameWnd * config_->UeNum() * kDefaultQueueSize);
  tx_queue_ = moodycamel::ConcurrentQueue<EventData>(
      kFrameWnd * config_->UeNum() * kDefaultQueueSize);
  to_mac_queue_ = moodycamel::ConcurrentQueue<EventData>(
      kFrameWnd * config_->UeNum() * kDefaultQueueSize);

  for (size_t i = 0; i < rx_thread_num_; i++) {
    rx_ptoks_ptr_[i] = new moodycamel::ProducerToken(message_queue_);
    tx_ptoks_ptr_[i] = new moodycamel::ProducerToken(tx_queue_);
    mac_rx_ptoks_ptr_[i] = new moodycamel::ProducerToken(message_queue_);
    mac_tx_ptoks_ptr_[i] = new moodycamel::ProducerToken(to_mac_queue_);
  }

  for (size_t i = 0; i < config_->WorkerThreadNum(); i++) {
    task_ptok_[i] = new moodycamel::ProducerToken(message_queue_);
  }

  ru_ = std::make_unique<RadioTxRx>(config_, rx_thread_num_,
                                    config_->CoreOffset() + 1, &message_queue_,
                                    &tx_queue_, rx_ptoks_ptr_, tx_ptoks_ptr_);

  if (kEnableMac) {
    // TODO [ankalia]: dummy_decoded_buffer is used at the base station
    // server only, but MacThread for now requires it for the UE client too
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, uint8_t> dummy_decoded_buffer;

    const size_t mac_cpu_core = config_->CoreOffset() + 1 + rx_thread_num_;
    mac_thread_ = std::make_unique<MacThread>(
        MacThread::Mode::kClient, config_, mac_cpu_core, dummy_decoded_buffer,
        &ul_bits_buffer_, &ul_bits_buffer_status_, nullptr, /* dl bits buffer */
        nullptr /* dl bits buffer status */, &to_mac_queue_, &message_queue_);

    mac_std_thread_ = std::thread(&MacThread::RunEventLoop, mac_thread_.get());
  }

  // uplink buffers init (tx)
  InitializeUplinkBuffers();
  // downlink buffers init (rx)
  InitializeDownlinkBuffers();

  (void)DftiCreateDescriptor(&mkl_handle_, DFTI_SINGLE, DFTI_COMPLEX, 1,
                             config_->OfdmCaNum());
  (void)DftiCommitDescriptor(mkl_handle_);

  // initilize all kinds of checkers
  fft_status_.fill(0);
  for (auto& i : fft_checker_) {
    // 0 init
    i = std::vector<size_t>(config_->UeAntNum());
  }

  demul_status_.fill(0);
  decode_status_.fill(0);
  frame_dl_process_time_.fill(0);

  if (dl_data_symbol_perframe_ > 0) {
    for (auto& i : demul_checker_) {
      i = std::vector<size_t>(config_->UeAntNum());
    }

    for (auto& i : decode_checker_) {
      i = std::vector<size_t>(config_->UeAntNum());
    }
  }

  for (size_t i = 0; i < config_->WorkerThreadNum(); i++) {
    task_threads_.at(i) = std::thread(&PhyUe::TaskThread, this, i);
  }
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
  if (!in_queue->try_enqueue(ptok, do_task)) {
    std::printf("need more memory\n");
    if (!in_queue->enqueue(ptok, do_task)) {
      std::printf("task enqueue failed\n");
      throw std::runtime_error("PhyUe: task enqueue failed");
    }
  }
}

//////////////////////////////////////////////////////////
//                   UPLINK Operations                  //
//////////////////////////////////////////////////////////
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

  // for task_queue, main thread is producer, it is single-procuder & multiple
  // consumer for task queue uplink

  // TODO: make the producertokens global and try
  // "try_dequeue_from_producer(token,item)"
  //       combine the task queues into one queue
  moodycamel::ProducerToken ptok_fft(fft_queue_);
  moodycamel::ProducerToken ptok_modul(modul_queue_);
  moodycamel::ProducerToken ptok_demul(demul_queue_);
  moodycamel::ProducerToken ptok_decode(decode_queue_);
  moodycamel::ProducerToken ptok_mac(to_mac_queue_);
  moodycamel::ProducerToken ptok_ifft(ifft_queue_);
  moodycamel::ProducerToken ptok_encode(encode_queue_);

  // for message_queue, main thread is a consumer, it is multiple producers
  // & single consumer for message_queue
  moodycamel::ConsumerToken ctok(message_queue_);

  // counter for print log
  int miss_count = 0;
  int total_count = 0;

  EventData events_list[kDequeueBulkSizeTXRX];
  int ret = 0;
  max_equaled_frame_ = 0;
  size_t cur_frame_id = 0;
  while ((config_->Running() == true) &&
         (SignalHandler::GotExitSignal() == false)) {
    // get a bulk of events
    ret = message_queue_.try_dequeue_bulk(ctok, events_list,
                                          kDequeueBulkSizeTXRX);
    total_count++;
    if (total_count == 1e7) {
      // print the message_queue_ miss rate is needed
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
    for (int bulk_count = 0; bulk_count < ret; bulk_count++) {
      EventData& event = events_list[bulk_count];

      switch (event.event_type_) {
        case EventType::kPacketRX: {
          // int offset = event.tags[0];
          size_t rx_thread_id = rx_tag_t(event.tags_[0]).tid_;
          size_t offset_in_current_buffer = rx_tag_t(event.tags_[0]).offset_;

          auto* pkt = reinterpret_cast<struct Packet*>(
              rx_buffer_[rx_thread_id] +
              offset_in_current_buffer * config_->PacketLength());
          size_t frame_id = pkt->frame_id_;
          size_t symbol_id = pkt->symbol_id_;
          size_t ant_id = pkt->ant_id_;
          size_t ue_id = ant_id / config_->NumChannels();
          RtAssert(pkt->frame_id_ < cur_frame_id + kFrameWnd,
                   "Error: Received packet for future frame beyond frame "
                   "window. This can happen if PHY is running "
                   "slowly, e.g., in debug mode");

          size_t dl_symbol_id = 0;
          if (config_->Frame().NumDLSyms() > 0) {
            dl_symbol_id = config_->Frame().GetDLSymbol(0);
          }

          // \todo convert to this->config_->Frame().GetBeaconSymbolLast()
          if ((symbol_id == 0)  // Beacon in Sim mode!
              ||
              ((config_->HwFramer() == false) &&
               (ul_data_symbol_perframe_ == 0) && (symbol_id == dl_symbol_id) &&
               (ant_id % config_->NumChannels() ==
                0))  // first DL symbols in downlink-only mode
          ) {        // Send uplink pilots
            EventData do_tx_pilot_task(
                EventType::kPacketPilotTX,
                gen_tag_t::FrmSymUe(
                    frame_id, config_->Frame().GetPilotSymbol(ue_id), ue_id)
                    .tag_);
            ScheduleTask(do_tx_pilot_task, &tx_queue_,
                         *tx_ptoks_ptr_[ant_id % rx_thread_num_]);
          }

          if ((ul_data_symbol_perframe_ > 0) &&
              (symbol_id == 0 || symbol_id == dl_symbol_id) &&
              (ant_id % config_->NumChannels() == 0)) {
            EventData do_encode_task(
                EventType::kEncode,
                gen_tag_t::FrmSymUe(frame_id, symbol_id, ue_id).tag_);
            ScheduleTask(do_encode_task, &encode_queue_, ptok_encode);
          }

          if ((dl_data_symbol_perframe_ > 0) &&
              (config_->IsPilot(frame_id, symbol_id) ||
               config_->IsDownlink(frame_id, symbol_id))) {
            if (dl_symbol_id == config_->Frame().GetDLSymbol(0)) {
              frame_dl_process_time_[(frame_id % kFrameWnd) * kMaxUEs +
                                     ant_id] = GetTime::GetTimeUs();
            }
            EventData do_fft_task(EventType::kFFT, event.tags_[0]);
            ScheduleTask(do_fft_task, &fft_queue_, ptok_fft);
          } else {  // if we are not entering doFFT, reset buffer here
            rx_buffer_status_[rx_thread_id][offset_in_current_buffer] =
                0;  // now empty
          }
        } break;

        case EventType::kFFT: {
          size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;
          size_t ant_id = gen_tag_t(event.tags_[0]).ant_id_;
          size_t frame_slot = frame_id % kFrameWnd;
          size_t dl_symbol_idx = config_->Frame().GetDLSymbolIdx(symbol_id);
          if (dl_symbol_idx >= dl_pilot_symbol_perframe_) {
            EventData do_demul_task(EventType::kDemul, event.tags_[0]);
            ScheduleTask(do_demul_task, &demul_queue_, ptok_demul);
          }
          fft_checker_[frame_slot][ant_id]++;
          if (fft_checker_[frame_slot][ant_id] == dl_symbol_perframe_) {
            if (kDebugPrintPerTaskDone) {
              std::printf(
                  "Main thread: Equalization done frame: %zu, "
                  "ant_id %zu\n",
                  frame_id, ant_id);
            }
            fft_checker_[frame_slot][ant_id] = 0;
            fft_status_[frame_slot]++;
            if (fft_status_[frame_slot] == config_->UeAntNum()) {
              if (kDebugPrintPerFrameDone) {
                std::printf(
                    "Main thread: Equalization done on all "
                    "antennas at frame: %zu\n",
                    frame_id);
              }
              fft_status_[frame_slot] = 0;
            }
          }

        } break;

        case EventType::kDemul: {
          size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          size_t ant_id = gen_tag_t(event.tags_[0]).ant_id_;
          size_t frame_slot = frame_id % kFrameWnd;

          EventData do_decode_task(EventType::kDecode, event.tags_[0]);
          ScheduleTask(do_decode_task, &decode_queue_, ptok_decode);
          demul_checker_[frame_slot][ant_id]++;
          if (demul_checker_[frame_slot][ant_id] == dl_data_symbol_perframe_) {
            if (kDebugPrintPerTaskDone) {
              std::printf(
                  "Main thread: Demodulation done frame: %zu, "
                  "ant %zu\n",
                  frame_id, ant_id);
            }
            max_equaled_frame_ = frame_id;
            demul_checker_[frame_slot][ant_id] = 0;
            demul_status_[frame_slot]++;
            if (demul_status_[frame_slot] == config_->UeAntNum()) {
              if (kDebugPrintPerFrameDone) {
                std::printf(
                    "Main thread: Demodulation done on all "
                    "antennas at frame: %zu \n",
                    frame_id);
              }
              demul_status_[frame_slot] = 0;
            }
          }
        } break;

        case EventType::kDecode: {
          size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          size_t ant_id = gen_tag_t(event.tags_[0]).ant_id_;
          size_t frame_slot = frame_id % kFrameWnd;
          if (kEnableMac) {
            ScheduleTask(EventData(EventType::kPacketToMac, event.tags_[0]),
                         &to_mac_queue_, ptok_mac);
          }
          decode_checker_[frame_slot][ant_id]++;
          if (decode_checker_[frame_slot][ant_id] == dl_data_symbol_perframe_) {
            if (kDebugPrintPerTaskDone) {
              std::printf(
                  "Main thread: Decoding done frame: %zu, "
                  "ant %zu\n",
                  frame_id, ant_id);
            }
            decode_checker_[frame_slot][ant_id] = 0;
            decode_status_[frame_slot]++;
            frame_dl_process_time_[frame_slot * kMaxUEs + ant_id] =
                GetTime::GetTimeUs() -
                frame_dl_process_time_[frame_slot * kMaxUEs + ant_id];
            if (decode_status_[frame_slot] == config_->UeAntNum()) {
              double frame_time_total = 0;
              for (size_t i = 0; i < config_->UeAntNum(); i++) {
                frame_time_total +=
                    frame_dl_process_time_[frame_slot * kMaxUEs + i];
              }
              if (kDebugPrintPerFrameDone) {
                std::printf(
                    "Main thread: Decode done on all antennas "
                    "at frame %zu"
                    " in %.2f us\n",
                    frame_id, frame_time_total);
              }
              decode_status_[frame_slot] = 0;
              if (kEnableMac == false) {
                cur_frame_id = frame_id;
              }
            }
          }
        } break;

        case EventType::kPacketToMac: {
          size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;
          cur_frame_id = frame_id;

          if (kDebugPrintPacketsToMac) {
            std::printf(
                "Main thread: sent decoded packet for frame %zu"
                ", symbol %zu to MAC\n",
                frame_id, symbol_id);
          }

        } break;

        case EventType::kPacketFromMac: {
          // size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
          size_t ue_id = rx_tag_t(event.tags_[0]).tid_;
          size_t radio_buf_id = rx_tag_t(event.tags_[0]).offset_;
          RtAssert(radio_buf_id == expected_frame_id_from_mac_ % kFrameWnd);

          auto* pkt = reinterpret_cast<MacPacket*>(
              &ul_bits_buffer_[ue_id]
                              [radio_buf_id * config_->MacBytesNumPerframe()]);
          RtAssert(pkt->frame_id_ == expected_frame_id_from_mac_,
                   "Incorrect frame ID from MAC");
          current_frame_user_num_ =
              (current_frame_user_num_ + 1) % config_->UeAntNum();
          if (current_frame_user_num_ == 0) {
            expected_frame_id_from_mac_++;
          }

          if (kDebugPrintPacketsFromMac) {
            std::printf(
                "Main thread: received packet for frame %u with "
                "modulation %zu\n",
                pkt->frame_id_, pkt->rb_indicator_.mod_order_bits_);
            std::stringstream ss;
            ss << "PhyUE kPacketFromMac, frame ID " << pkt->frame_id_
               << ", bytes: ";
            for (size_t i = 0; i < 4; i++) {
              ss << std::to_string((reinterpret_cast<uint8_t*>(pkt->data_)[i]))
                 << ", ";
            }
            std::printf("%s\n", ss.str().c_str());
          }

        } break;

        case EventType::kEncode: {
          // size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
          // size_t data_symbol_idx = gen_tag_t(event.tags[0]).symbol_id;
          // size_t ue_id = gen_tag_t(event.tags[0]).ant_id;
          EventData do_modul_task(EventType::kModul, event.tags_[0]);
          ScheduleTask(do_modul_task, &modul_queue_, ptok_modul);

        } break;

        case EventType::kModul: {
          size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;
          size_t ue_id = gen_tag_t(event.tags_[0]).ue_id_;

          EventData do_ifft_task(
              EventType::kIFFT,
              gen_tag_t::FrmSymUe(frame_id, symbol_id, ue_id).tag_);
          ScheduleTask(do_ifft_task, &ifft_queue_, ptok_ifft);
          if (kDebugPrintPerTaskDone) {
            std::printf(
                "Main thread: frame: %zu, symbol: %zu, finished "
                "modulating "
                "uplink data for user %zu\n",
                frame_id, symbol_id, ue_id);
          }
        } break;

        case EventType::kIFFT: {
          size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          size_t ue_id = gen_tag_t(event.tags_[0]).ue_id_;
          EventData do_tx_task(EventType::kPacketTX, event.tags_[0]);
          ScheduleTask(do_tx_task, &tx_queue_,
                       *tx_ptoks_ptr_[ue_id % rx_thread_num_]);
          if (kDebugPrintPerTaskDone) {
            std::printf(
                "Main thread: frame: %zu, finished IFFT of "
                "uplink data for user %zu\n",
                frame_id, ue_id);
          }
        } break;

        case EventType::kPacketPilotTX: {
          size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;
          size_t ue_id = gen_tag_t(event.tags_[0]).ue_id_;
          if (ul_data_symbol_perframe_ == 0) {
            cur_frame_id++;
          }
          if (kDebugPrintPerSymbolDone) {
            std::printf(
                "Main thread: finished Pilot TX for user %zu"
                " in frame %zu, symbol %zu\n",
                ue_id, frame_id, symbol_id);
          }
        } break;

        case EventType::kPacketTX: {
          size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
          size_t ue_id = gen_tag_t(event.tags_[0]).ue_id_;
          cur_frame_id = frame_id;
          RtAssert(frame_id == next_frame_processed_[ue_id],
                   "Unexpected frame_id was transmitted!");

          // std::printf("PhyUE kPacketTX: Freeing buffer %zu for UE %zu\n",
          //    num_frames_consumed_[ue_id] % kFrameWnd, ue_id);
          ul_bits_buffer_status_[ue_id]
                                [next_frame_processed_[ue_id] % kFrameWnd] = 0;
          next_frame_processed_[ue_id]++;

          if (kDebugPrintPerFrameDone) {
            std::printf(
                "Main thread: finished TX for frame %zu, "
                "user %zu\n",
                frame_id, ue_id);
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
  std::printf("PhyUE: task thread %d starts\n", tid);
  PinToCoreWithOffset(ThreadType::kWorker,
                      config_->CoreOffset() + rx_thread_num_ + 1 +
                          (kEnableMac ? rx_thread_num_ : 0),
                      tid);

  // task_ptok[tid].reset(new moodycamel::ProducerToken(message_queue_));

  EventData event;
  while (config_->Running() == true) {
    if (decode_queue_.try_dequeue(event)) {
      DoDecode(tid, event.tags_[0]);
    } else if (demul_queue_.try_dequeue(event)) {
      DoDemul(tid, event.tags_[0]);
    } else if (ifft_queue_.try_dequeue(event)) {
      DoIfft(tid, event.tags_[0]);
    } else if (modul_queue_.try_dequeue(event)) {
      DoModul(tid, event.tags_[0]);
    } else if (encode_queue_.try_dequeue(event)) {
      DoEncode(tid, event.tags_[0]);
    } else if (fft_queue_.try_dequeue(event)) {
      DoFft(tid, event.tags_[0]);
    }
  }
}

//////////////////////////////////////////////////////////
//                   DOWNLINK Operations                  //
//////////////////////////////////////////////////////////

void PhyUe::DoFft(int tid, size_t tag) {
  size_t rx_thread_id = fft_req_tag_t(tag).tid_;
  size_t offset_in_current_buffer = fft_req_tag_t(tag).offset_;
  size_t start_tsc = GetTime::Rdtsc();

  // read info of one frame
  auto* pkt = reinterpret_cast<struct Packet*>(rx_buffer_[rx_thread_id] +
                                               offset_in_current_buffer *
                                                   config_->PacketLength());
  size_t frame_id = pkt->frame_id_;
  size_t symbol_id = pkt->symbol_id_;
  // int cell_id = pkt->cell_id;
  size_t ant_id = pkt->ant_id_;
  size_t frame_slot = frame_id % kFrameWnd;

  if (!config_->IsPilot(frame_id, symbol_id) &&
      !(config_->IsDownlink(frame_id, symbol_id))) {
    return;
  }

  if (kDebugPrintInTask) {
    std::printf("In doFFT TID %d: frame %zu, symbol %zu, ant_id %zu\n", tid,
                frame_id, symbol_id, ant_id);
  }

  size_t sig_offset = config_->OfdmRxZeroPrefixClient();
  if (kPrintDownlinkPilotStats) {
    if (config_->IsPilot(frame_id, symbol_id)) {
      SimdConvertShortToFloat(pkt->data_,
                              reinterpret_cast<float*>(rx_samps_tmp_),
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
      std::printf("frame %zu symbol %zu ant %zu: sig offset %zu, SNR %2.1f \n",
                  frame_id, symbol_id, ant_id, pilot_offset, snr);
      if (frame_id == kRecordFrameIndex) {
        std::string fname = "rxpilot" + std::to_string(symbol_id) + "_" +
                            std::to_string(ant_id) + ".bin";
        FILE* f = std::fopen(fname.c_str(), "wb");
        std::fwrite(pkt->data_, 2 * sizeof(int16_t), config_->SampsPerSymbol(),
                    f);
        std::fclose(f);
      }

    } else {
      if (frame_id == kRecordFrameIndex) {
        std::string fname = "rxdata" + std::to_string(symbol_id) + "_" +
                            std::to_string(ant_id) + ".bin";
        FILE* f = std::fopen(fname.c_str(), "wb");
        std::fwrite(pkt->data_, 2 * sizeof(int16_t), config_->SampsPerSymbol(),
                    f);
        std::fclose(f);
      }
    }
  }

  // remove CP, do FFT
  size_t dl_symbol_id = config_->Frame().GetDLSymbolIdx(symbol_id);
  size_t total_dl_symbol_id = frame_slot * dl_symbol_perframe_ + dl_symbol_id;
  size_t fft_buffer_target_id =
      total_dl_symbol_id * config_->UeAntNum() + ant_id;

  // transfer ushort to float
  size_t delay_offset = (sig_offset + config_->CpLen()) * 2;
  auto* fft_buff = reinterpret_cast<float*>(fft_buffer_[fft_buffer_target_id]);

  SimdConvertShortToFloat(&pkt->data_[delay_offset], fft_buff,
                          config_->OfdmCaNum() * 2);

  // perform fft
  DftiComputeForward(mkl_handle_, fft_buffer_[fft_buffer_target_id]);

  size_t csi_offset = frame_slot * config_->UeAntNum() + ant_id;
  auto* csi_buffer_ptr =
      reinterpret_cast<cx_float*>(csi_buffer_[csi_offset].data());
  auto* fft_buffer_ptr =
      reinterpret_cast<cx_float*>(fft_buffer_[fft_buffer_target_id]);

  EventData fft_finish_event;

  // In TDD massive MIMO, a pilot symbol needs to be sent
  // in the downlink for the user to estimate the channel
  // due to relative reciprocity calibration,
  // see Argos paper (Mobicom'12)
  if (dl_symbol_id < dl_pilot_symbol_perframe_) {
    for (size_t j = 0; j < config_->OfdmDataNum(); j++) {
      // divide fft output by pilot data to get CSI estimation
      if (dl_symbol_id == 0) {
        csi_buffer_ptr[j] = 0;
      }
      // FIXME: cfg->ue_specific_pilot[user_id] index creates errors
      // in the downlink receiver
      complex_float p = config_->UeSpecificPilot()[0][j];
      size_t sc_id = non_null_sc_ind_[j];
      csi_buffer_ptr[j] += (fft_buffer_ptr[sc_id] / cx_float(p.re, p.im));
      if (dl_symbol_id == dl_pilot_symbol_perframe_ - 1) {
        csi_buffer_ptr[j] /= dl_pilot_symbol_perframe_;
      }
    }
  } else {
    size_t total_dl_data_symbol_id = frame_slot * dl_data_symbol_perframe_ +
                                     dl_symbol_id - dl_pilot_symbol_perframe_;
    size_t eq_buffer_offset =
        total_dl_data_symbol_id * config_->UeAntNum() + ant_id;

    auto* equ_buffer_ptr =
        reinterpret_cast<cx_float*>(equal_buffer_[eq_buffer_offset].data());

    // use pilot subcarriers for phase tracking and correction
    float theta = 0;
    cx_float csi(1, 0);
    for (size_t j = 0; j < config_->OfdmDataNum(); j++) {
      if (j % config_->OfdmPilotSpacing() == 0) {
        equ_buffer_ptr[j] = 0;
        if (dl_pilot_symbol_perframe_ > 0) {
          csi = csi_buffer_ptr[j];
        }
        size_t sc_id = non_null_sc_ind_[j];
        cx_float y = fft_buffer_ptr[sc_id];
        auto pilot_eq = y / csi;
        // FIXME: cfg->ue_specific_pilot[user_id] index creates errors
        // in the downlink receiver
        auto p = config_->UeSpecificPilot()[0][j];
        theta += arg(pilot_eq * cx_float(p.re, -p.im));
      }
    }
    if (config_->GetOFDMPilotNum() > 0) {
      theta /= config_->GetOFDMPilotNum();
    }
    auto phc = exp(cx_float(0, -theta));
    float evm = 0;
    for (size_t j = 0; j < config_->OfdmDataNum(); j++) {
      if (j % config_->OfdmPilotSpacing() != 0) {
        // divide fft output by pilot data to get CSI estimation
        size_t sc_id = non_null_sc_ind_[j];
        if (dl_pilot_symbol_perframe_ > 0) {
          csi = csi_buffer_ptr[j];
        }
        cx_float y = fft_buffer_ptr[sc_id];
        equ_buffer_ptr[j] = (y / csi) * phc;
        complex_float tx =
            config_->DlIqF()[dl_symbol_id][ant_id * config_->OfdmCaNum() +
                                           config_->OfdmDataStart() + j];
        evm += std::norm(equ_buffer_ptr[j] - cx_float(tx.re, tx.im));
      }
    }
    if (kPrintEqualizedSymbols) {
      complex_float* tx =
          &config_->DlIqF()[dl_symbol_id][ant_id * config_->OfdmCaNum() +
                                          config_->OfdmDataStart()];
      arma::cx_fvec x_vec(reinterpret_cast<cx_float*>(tx),
                          config_->OfdmDataNum(), false);
      Utils::PrintVec(x_vec, std::string("x") +
                                 std::to_string(total_dl_symbol_id) +
                                 std::string("_") + std::to_string(ant_id));
      arma::cx_fvec equal_vec(equ_buffer_ptr, config_->OfdmDataNum(), false);
      Utils::PrintVec(equal_vec, std::string("equ") +
                                     std::to_string(total_dl_symbol_id) +
                                     std::string("_") + std::to_string(ant_id));
    }
    evm =
        std::sqrt(evm) / (config_->OfdmDataNum() - config_->GetOFDMPilotNum());
    if (kPrintPhyStats) {
      std::stringstream ss;
      ss << "Frame: " << frame_id << ", Symbol: " << symbol_id
         << ", User: " << ant_id << ", EVM: " << 100 * evm
         << "%, SNR: " << -10 * std::log10(evm) << std::endl;
      std::cout << ss.str();
    }
  }

  size_t fft_duration_stat = GetTime::Rdtsc() - start_tsc;
  if (kDebugPrintPerTaskDone) {
    std::printf(
        "FFT Duration (%zu, %zu, %zu): %2.4f us\n", frame_id, symbol_id, ant_id,
        GetTime::CyclesToUs(fft_duration_stat, GetTime::MeasureRdtscFreq()));
  }

  rx_buffer_status_[rx_thread_id][offset_in_current_buffer] = 0;  // now empty
  fft_finish_event = EventData(
      EventType::kFFT, gen_tag_t::FrmSymAnt(frame_id, symbol_id, ant_id).tag_);
  RtAssert(message_queue_.enqueue(*task_ptok_[tid], fft_finish_event),
           "FFT message enqueue failed");
}

void PhyUe::DoDemul(int tid, size_t tag) {
  const size_t frame_id = gen_tag_t(tag).frame_id_;
  const size_t symbol_id = gen_tag_t(tag).symbol_id_;
  const size_t ant_id = gen_tag_t(tag).ant_id_;
  if (kDebugPrintInTask) {
    std::printf("In doDemul TID %d: frame %zu, symbol %zu, ant_id %zu\n", tid,
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

  // demod_16qam_hard_loop(
  //    equal_ptr, (uint8_t*)demul_ptr, config_->UeAntNum());

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
      std::printf("Demodulation: modulation type %s not supported!\n",
                  config_->Modulation().c_str());
  }

  size_t dem_duration_stat = GetTime::Rdtsc() - start_tsc;
  if (kDebugPrintPerTaskDone) {
    std::printf(
        "Demodul Duration (%zu, %zu, %zu): %2.4f us\n", frame_id, symbol_id,
        ant_id,
        GetTime::CyclesToUs(dem_duration_stat, GetTime::MeasureRdtscFreq()));
  }

  if (kPrintLLRData) {
    std::printf("LLR data, symbol_offset: %zu\n", offset);
    for (size_t i = 0; i < config_->OfdmDataNum(); i++) {
      std::printf("%x ", (uint8_t) * (demul_ptr + i));
    }
    std::printf("\n");
  }

  RtAssert(message_queue_.enqueue(*task_ptok_[tid],
                                  EventData(EventType::kDemul, tag)),
           "Demodulation message enqueue failed");
}

void PhyUe::DoDecode(int tid, size_t tag) {
  const LDPCconfig& ldpc_config = config_->LdpcConfig();
  size_t frame_id = gen_tag_t(tag).frame_id_;
  size_t symbol_id = gen_tag_t(tag).symbol_id_;
  size_t ant_id = gen_tag_t(tag).ant_id_;
  if (kDebugPrintInTask) {
    std::printf("In doDecode TID %d: frame %zu, symbol %zu, ant_id %zu\n", tid,
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
      Scrambler::WlanDescramble(decoded_buffer_ptr, config_->NumBytesPerCb());
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

  size_t dec_duration_stat = GetTime::Rdtsc() - start_tsc;
  if (kDebugPrintPerTaskDone) {
    std::printf(
        "Decode Duration (%zu, %zu, %zu): %2.4f us\n", frame_id, symbol_id,
        ant_id,
        GetTime::CyclesToUs(dec_duration_stat, GetTime::MeasureRdtscFreq()));
  }

  RtAssert(message_queue_.enqueue(*task_ptok_[tid],
                                  EventData(EventType::kDecode, tag)),
           "Decoding message enqueue failed");
}

//////////////////////////////////////////////////////////
//                   UPLINK Operations                //
//////////////////////////////////////////////////////////

void PhyUe::DoEncode(int tid, size_t tag) {
  const LDPCconfig& ldpc_config = config_->LdpcConfig();
  // size_t ue_id = rx_tag_t(tag).tid;
  // size_t offset = rx_tag_t(tag).offset;
  const size_t frame_id = gen_tag_t(tag).frame_id_;
  const size_t ue_id = gen_tag_t(tag).ue_id_;
  size_t frame_slot = frame_id % kFrameWnd;
  auto& cfg = config_;
  // size_t start_tsc = worker_rdtsc();

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
      kEnableMac ? (ldpc_config.NumCbLen()) >> 3
                 : Roundup<64>(BitsToBytes(ldpc_config.NumCbLen()));
  size_t encoded_bytes_per_block = (ldpc_config.NumCbCodewLen() + 7) >> 3;
  auto* input_ptr = new int8_t[bytes_per_block +
                               kLdpcHelperFunctionInputBufferSizePaddingBytes];

  for (size_t ul_symbol_id = 0; ul_symbol_id < ul_data_symbol_perframe_;
       ul_symbol_id++) {
    size_t total_ul_symbol_id =
        frame_slot * ul_data_symbol_perframe_ + ul_symbol_id;
    for (size_t cb_id = 0; cb_id < config_->LdpcConfig().NumBlocksInSymbol();
         cb_id++) {
      if (kEnableMac) {
        uint8_t* ul_bits = ul_bits_buffer_[ue_id] +
                           frame_slot * config_->MacBytesNumPerframe();

        int input_offset = bytes_per_block *
                               cfg->LdpcConfig().NumBlocksInSymbol() *
                               ul_symbol_id +
                           bytes_per_block * cb_id;
        std::memcpy(input_ptr,
                    reinterpret_cast<int8_t*>(ul_bits + input_offset),
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
        Scrambler::WlanScramble(input_ptr, bytes_per_block);
      }

      LdpcEncodeHelper(ldpc_config.BaseGraph(), ldpc_config.ExpansionFactor(),
                       ldpc_config.NumRows(), encoded_buffer_temp,
                       parity_buffer, input_ptr);

      int cb_coded_bytes = ldpc_config.NumCbCodewLen() / cfg->ModOrderBits();
      int output_offset =
          total_ul_symbol_id * config_->OfdmDataNum() + cb_coded_bytes * cb_id;

      AdaptBitsForMod(reinterpret_cast<uint8_t*>(encoded_buffer_temp),
                      &ul_syms_buffer_[ue_id][output_offset],
                      encoded_bytes_per_block, cfg->ModOrderBits());
    }
  }
  // double duration = worker_rdtsc() - start_tsc;
  // if (cycles_to_us(duration, freq_ghz) > 500) {
  //    std::printf("Thread %d Encode takes %.2f\n", tid,
  //        cycles_to_us(duration, freq_ghz));
  //}

  std::free(encoded_buffer_temp);
  std::free(parity_buffer);
  delete[] input_ptr;

  RtAssert(message_queue_.enqueue(*task_ptok_[tid],
                                  EventData(EventType::kEncode, tag)),
           "Encoding message enqueue failed");
}

void PhyUe::DoModul(int tid, size_t tag) {
  const size_t frame_id = gen_tag_t(tag).frame_id_;
  const size_t ue_id = gen_tag_t(tag).ue_id_;
  const size_t frame_slot = frame_id % kFrameWnd;
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
  RtAssert(message_queue_.enqueue(*task_ptok_[tid],
                                  EventData(EventType::kModul, tag)),
           "Muliplexing message enqueue failed");
}

void PhyUe::DoIfft(int tid, size_t tag) {
  const size_t frame_id = gen_tag_t(tag).frame_id_;
  const size_t frame_slot = frame_id % kFrameWnd;
  const size_t ue_id = gen_tag_t(tag).ue_id_;
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
                    config_->UeSpecificPilot()[0],
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

  RtAssert(message_queue_.enqueue(*task_ptok_[tid],
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
  num_cp_us_ = std::thread::hardware_concurrency();
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
