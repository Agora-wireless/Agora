/**
 * @file agora_utils.h
 * @brief Temporary utils file for Agora class during refactoring
 */

#include <unistd.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <queue>
#include <system_error>
#include <vector>

#include "agora_helper.h"
#include "buffer.h"
#include "concurrent_queue_wrapper.h"
#include "concurrentqueue.h"
#include "config.h"
#include "dodecode.h"
#include "dodemul.h"
#include "doencode.h"
#include "dofft.h"
#include "doifft.h"
#include "doprecode.h"
#include "dozf.h"
#include "mac_thread_basestation.h"
#include "mat_logger.h"
#include "memory_manage.h"
#include "packet_txrx.h"
#include "phy_stats.h"
#include "signal_handler.h"
#include "stats.h"
#include "utils.h"

// TODO: global variables will hold variables while we disintegrate Agora class
//       once we modularize Agora, we will move these to each class

// file specific debugger variables
static const bool kDebugPrintPacketsFromMac = false;
static const bool kDebugDeferral = true;

SchedInfoT sched_info_arr[kScheduleQueues][kNumEventTypes];

/* Configuration */
Config* config;
size_t cur_proc_frame_id;
size_t base_worker_core_offset_;

/* Queue */
// Master thread's message queue for receiving packets
moodycamel::ConcurrentQueue<EventData> message_queue;
// Master-to-worker queue for MAC
moodycamel::ConcurrentQueue<EventData> mac_request_queue;
// Worker-to-master queue for MAC
moodycamel::ConcurrentQueue<EventData> mac_response_queue;
// Master thread's message queue for event completion from Doers
moodycamel::ConcurrentQueue<EventData> complete_task_queue[kScheduleQueues];
moodycamel::ProducerToken* worker_ptoks_ptr[kMaxThreads][kScheduleQueues];
// Master thread's message queue for network communication
moodycamel::ProducerToken* rx_ptoks_ptr[kMaxThreads];
moodycamel::ProducerToken* tx_ptoks_ptr[kMaxThreads];

size_t fft_created_count_;
size_t max_equaled_frame_ = SIZE_MAX;
std::unique_ptr<PacketTxRx> packet_tx_rx_;

// The thread running MAC layer functions
std::unique_ptr<MacThreadBaseStation> mac_thread_;
// Handle for the MAC thread
std::thread mac_std_thread_;

std::unique_ptr<Stats> stats_;
std::unique_ptr<PhyStats> phy_stats_;

/*****************************************************
   * Buffers
   *****************************************************/

/* Uplink */
// RX buffer size per socket RX thread
size_t socket_buffer_size_;

// Received data buffers
// 1st dimension: number of socket RX threads
// 2nd dimension: socket buffer size
Table<char> socket_buffer_;

// Preliminary CSI buffers. Each buffer has [number of antennas] rows and
// [number of OFDM data subcarriers] columns.
PtrGrid<kFrameWnd, kMaxUEs, complex_float> csi_buffers_;

// Data symbols after FFT
// 1st dimension: kFrameWnd * uplink data symbols per frame
// 2nd dimension: number of antennas * number of OFDM data subcarriers
//
// 2nd dimension data order: 32 blocks each with 32 subcarriers each:
// subcarrier 1 -- 32 of antennas, subcarrier 33 -- 64 of antennas, ...,
// subcarrier 993 -- 1024 of antennas.
Table<complex_float> data_buffer_;

// Calculated uplink zeroforcing detection matrices. Each matrix has
// [number of antennas] rows and [number of UEs] columns.
PtrGrid<kFrameWnd, kMaxDataSCs, complex_float> ul_zf_matrices_;

// Data after equalization
// 1st dimension: kFrameWnd * uplink data symbols per frame
// 2nd dimension: number of OFDM data subcarriers * number of UEs
Table<complex_float> equal_buffer_;

// Data after demodulation. Each buffer has kMaxModType * number of OFDM
// data subcarriers
PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t> demod_buffers_;

// Data after LDPC decoding. Each buffer [decoded bytes per UE] bytes.
PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t> decoded_buffer_;

Table<complex_float> ue_spec_pilot_buffer_;

// Counters related to various modules
FrameCounters pilot_fft_counters_;
FrameCounters uplink_fft_counters_;
FrameCounters zf_counters_;
FrameCounters demul_counters_;
FrameCounters decode_counters_;
FrameCounters encode_counters_;
FrameCounters precode_counters_;
FrameCounters ifft_counters_;
FrameCounters tx_counters_;
FrameCounters tomac_counters_;
FrameCounters mac_to_phy_counters_;
FrameCounters rc_counters_;
RxCounters rx_counters_;
size_t zf_last_frame_ = SIZE_MAX;
size_t rc_last_frame_ = SIZE_MAX;
size_t ifft_next_symbol_ = 0;

// Agora schedules and processes a frame in FIFO order
// cur_proc_frame_id is the frame that is currently being processed.
// cur_sche_frame_id is the frame that is currently being scheduled.
// A frame's schduling finishes before processing ends, so the two
// variables are possible to have different values.
size_t cur_sche_frame_id_ = 0;

// The frame index for a symbol whose FFT is done
std::vector<size_t> fft_cur_frame_for_symbol_;
// The frame index for a symbol whose encode is done
std::vector<size_t> encode_cur_frame_for_symbol_;
// The frame index for a symbol whose IFFT is done
std::vector<size_t> ifft_cur_frame_for_symbol_;

// The frame index for a symbol whose precode is done
std::vector<size_t> precode_cur_frame_for_symbol_;

// Per-frame queues of delayed FFT tasks. The queue contains offsets into
// TX/RX buffers.
std::array<std::queue<fft_req_tag_t>, kFrameWnd> fft_queue_arr_;

// Data for IFFT
// 1st dimension: kFrameWnd * number of antennas * number of
// data symbols per frame
// 2nd dimension: number of OFDM carriers (including non-data carriers)
Table<complex_float> dl_ifft_buffer_;

// Calculated uplink zeroforcing detection matrices. Each matrix has
// [number of UEs] rows and [number of antennas] columns.
PtrGrid<kFrameWnd, kMaxDataSCs, complex_float> dl_zf_matrices_;

// 1st dimension: kFrameWnd
// 2nd dimension: number of OFDM data subcarriers * number of antennas
Table<complex_float> calib_ul_buffer_;
Table<complex_float> calib_dl_buffer_;
Table<complex_float> calib_ul_msum_buffer_;
Table<complex_float> calib_dl_msum_buffer_;

// 1st dimension: kFrameWnd * number of data symbols per frame
// 2nd dimension: number of OFDM data subcarriers * number of UEs
Table<int8_t> dl_mod_bits_buffer_;

// 1st dimension: kFrameWnd * number of DL data symbols per frame
// 2nd dimension: number of OFDM data subcarriers * number of UEs
Table<int8_t> dl_bits_buffer_;

// 1st dimension: number of UEs
// 2nd dimension: number of OFDM data subcarriers * kFrameWnd
//                * number of DL data symbols per frame
// Use different dimensions from dl_bits_buffer_ to avoid cache false sharing
Table<int8_t> dl_bits_buffer_status_;

/**
   * Data for transmission
   *
   * Number of downlink socket buffers and status entries:
   * kFrameWnd * symbol_num_perframe * BS_ANT_NUM
   *
   * Size of each downlink socket buffer entry: packet_length bytes
   * Size of each downlink socket buffer status entry: one integer
   */
char* dl_socket_buffer_;

uint8_t schedule_process_flags_;

std::queue<size_t> encode_deferral_;
std::array<std::shared_ptr<CsvLog::MatLogger>, CsvLog::kMatLogs> mat_loggers_;

/* Core Agora functions */
// Event handler
size_t fetch_event(EventData events_list[], bool is_turn_to_dequeue_from_io) {
  const size_t max_events_needed =
      std::max(kDequeueBulkSizeTXRX * (config->SocketThreadNum() + 1 /* MAC */),
               kDequeueBulkSizeWorker * config->WorkerThreadNum());
  size_t num_events = 0;
  if (is_turn_to_dequeue_from_io) {
    for (size_t i = 0; i < config->SocketThreadNum(); i++) {
      num_events += message_queue.try_dequeue_bulk_from_producer(
          *(rx_ptoks_ptr[i]), events_list + num_events, kDequeueBulkSizeTXRX);
    }

    if (kEnableMac == true) {
      num_events += mac_response_queue.try_dequeue_bulk(
          events_list + num_events, kDequeueBulkSizeTXRX);
    }
  } else {
    num_events +=
        complete_task_queue[(cur_proc_frame_id & 0x1)].try_dequeue_bulk(
            events_list + num_events, max_events_needed);
  }
  return num_events;
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

// Worker
void HandleEventFft(size_t tag) {
  const size_t frame_id = gen_tag_t(tag).frame_id_;
  const size_t symbol_id = gen_tag_t(tag).symbol_id_;
  const SymbolType sym_type = config->GetSymbolType(symbol_id);

  if (sym_type == SymbolType::kPilot) {
    const bool last_fft_task =
        pilot_fft_counters_.CompleteTask(frame_id, symbol_id);
    if (last_fft_task == true) {
      stats_->PrintPerSymbolDone(PrintType::kFFTPilots, frame_id, symbol_id,
                                 pilot_fft_counters_);

      if ((config->Frame().IsRecCalEnabled() == false) ||
          ((config->Frame().IsRecCalEnabled() == true) &&
           (rc_last_frame_ == frame_id))) {
        // If CSI of all UEs is ready, schedule ZF/prediction
        const bool last_pilot_fft =
            pilot_fft_counters_.CompleteSymbol(frame_id);
        if (last_pilot_fft == true) {
          stats_->MasterSetTsc(TsType::kFFTPilotsDone, frame_id);
          stats_->PrintPerFrameDone(PrintType::kFFTPilots, frame_id);
          pilot_fft_counters_.Reset(frame_id);
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

// Initialization & Freeing
void InitializeQueues() {
  using mt_queue_t = moodycamel::ConcurrentQueue<EventData>;

  int data_symbol_num_perframe = config->Frame().NumDataSyms();
  message_queue =
      mt_queue_t(kDefaultMessageQueueSize * data_symbol_num_perframe);
  for (auto& c : complete_task_queue) {
    c = mt_queue_t(kDefaultWorkerQueueSize * data_symbol_num_perframe);
  }
  // Create concurrent queues for each Doer
  for (auto& vec : sched_info_arr) {
    for (auto& s : vec) {
      s.concurrent_q =
          mt_queue_t(kDefaultWorkerQueueSize * data_symbol_num_perframe);
      s.ptok = new moodycamel::ProducerToken(s.concurrent_q);
    }
  }

  for (size_t i = 0; i < config->SocketThreadNum(); i++) {
    rx_ptoks_ptr[i] = new moodycamel::ProducerToken(message_queue);
    tx_ptoks_ptr[i] = new moodycamel::ProducerToken(
        *GetConq(sched_info_arr, EventType::kPacketTX, 0));
  }

  for (size_t i = 0; i < config->WorkerThreadNum(); i++) {
    for (size_t j = 0; j < kScheduleQueues; j++) {
      worker_ptoks_ptr[i][j] =
          new moodycamel::ProducerToken(complete_task_queue[j]);
    }
  }
}

void FreeQueues() {
  // remove tokens for each doer
  for (auto& vec : sched_info_arr) {
    for (auto& s : vec) {
      delete s.ptok;
    }
  }

  for (size_t i = 0; i < config->SocketThreadNum(); i++) {
    delete rx_ptoks_ptr[i];
    delete tx_ptoks_ptr[i];
  }

  for (size_t i = 0; i < config->WorkerThreadNum(); i++) {
    for (size_t j = 0; j < kScheduleQueues; j++) {
      delete worker_ptoks_ptr[i][j];
    }
  }
}

void InitializeUplinkBuffers() {
  const size_t task_buffer_symbol_num_ul =
      config->Frame().NumULSyms() * kFrameWnd;

  socket_buffer_size_ = config->PacketLength() * config->BsAntNum() *
                        kFrameWnd * config->Frame().NumTotalSyms();

  socket_buffer_.Malloc(config->SocketThreadNum() /* RX */, socket_buffer_size_,
                        Agora_memory::Alignment_t::kAlign64);

  data_buffer_.Malloc(task_buffer_symbol_num_ul,
                      config->OfdmDataNum() * config->BsAntNum(),
                      Agora_memory::Alignment_t::kAlign64);

  equal_buffer_.Malloc(task_buffer_symbol_num_ul,
                       config->OfdmDataNum() * config->UeAntNum(),
                       Agora_memory::Alignment_t::kAlign64);
  ue_spec_pilot_buffer_.Calloc(
      kFrameWnd, config->Frame().ClientUlPilotSymbols() * config->UeAntNum(),
      Agora_memory::Alignment_t::kAlign64);

  rx_counters_.num_pilot_pkts_per_frame_ =
      config->BsAntNum() * config->Frame().NumPilotSyms();
  // BfAntNum() for each 'L' symbol (no ref node)
  // RefRadio * NumChannels() for each 'C'.
  //rx_counters_.num_reciprocity_pkts_per_frame_ = config->BsAntNum();
  const size_t num_rx_ul_cal_antennas = config->BfAntNum();
  // Same as the number of rx reference antennas (ref ant + other channels)
  const size_t num_rx_dl_cal_antennas = config->BsAntNum() - config->BfAntNum();

  rx_counters_.num_reciprocity_pkts_per_frame_ =
      (config->Frame().NumULCalSyms() * num_rx_ul_cal_antennas) +
      (config->Frame().NumDLCalSyms() * num_rx_dl_cal_antennas);

  AGORA_LOG_INFO("Agora: Total recip cal receive symbols per frame: %zu\n",
                 rx_counters_.num_reciprocity_pkts_per_frame_);

  rx_counters_.num_rx_pkts_per_frame_ =
      rx_counters_.num_pilot_pkts_per_frame_ +
      rx_counters_.num_reciprocity_pkts_per_frame_ +
      (config->BsAntNum() * config->Frame().NumULSyms());

  fft_created_count_ = 0;
  pilot_fft_counters_.Init(config->Frame().NumPilotSyms(), config->BsAntNum());
  uplink_fft_counters_.Init(config->Frame().NumULSyms(), config->BsAntNum());
  fft_cur_frame_for_symbol_ =
      std::vector<size_t>(config->Frame().NumULSyms(), SIZE_MAX);

  rc_counters_.Init(config->BsAntNum());

  zf_counters_.Init(config->ZfEventsPerSymbol());

  demul_counters_.Init(config->Frame().NumULSyms(),
                       config->DemulEventsPerSymbol());

  decode_counters_.Init(
      config->Frame().NumULSyms(),
      config->LdpcConfig(Direction::kUplink).NumBlocksInSymbol() *
          config->UeAntNum());

  tomac_counters_.Init(config->Frame().NumULSyms(), config->UeAntNum());
}

void InitializeDownlinkBuffers() {
  if (config->Frame().NumDLSyms() > 0) {
    AGORA_LOG_TRACE("Agora: Initializing downlink buffers\n");

    const size_t task_buffer_symbol_num =
        config->Frame().NumDLSyms() * kFrameWnd;

    size_t dl_socket_buffer_status_size =
        config->BsAntNum() * task_buffer_symbol_num;
    size_t dl_socket_buffer_size =
        config->DlPacketLength() * dl_socket_buffer_status_size;
    AllocBuffer1d(&dl_socket_buffer_, dl_socket_buffer_size,
                  Agora_memory::Alignment_t::kAlign64, 0);

    size_t dl_bits_buffer_size =
        kFrameWnd * config->MacBytesNumPerframe(Direction::kDownlink);
    dl_bits_buffer_.Calloc(config->UeAntNum(), dl_bits_buffer_size,
                           Agora_memory::Alignment_t::kAlign64);
    dl_bits_buffer_status_.Calloc(config->UeAntNum(), kFrameWnd,
                                  Agora_memory::Alignment_t::kAlign64);

    dl_ifft_buffer_.Calloc(config->BsAntNum() * task_buffer_symbol_num,
                           config->OfdmCaNum(),
                           Agora_memory::Alignment_t::kAlign64);
    calib_dl_buffer_.Malloc(kFrameWnd,
                            config->BfAntNum() * config->OfdmDataNum(),
                            Agora_memory::Alignment_t::kAlign64);
    calib_ul_buffer_.Malloc(kFrameWnd,
                            config->BfAntNum() * config->OfdmDataNum(),
                            Agora_memory::Alignment_t::kAlign64);
    calib_dl_msum_buffer_.Malloc(kFrameWnd,
                                 config->BfAntNum() * config->OfdmDataNum(),
                                 Agora_memory::Alignment_t::kAlign64);
    calib_ul_msum_buffer_.Malloc(kFrameWnd,
                                 config->BfAntNum() * config->OfdmDataNum(),
                                 Agora_memory::Alignment_t::kAlign64);
    //initialize the calib buffers
    const complex_float complex_init = {0.0f, 0.0f};
    //const complex_float complex_init = {1.0f, 0.0f};
    for (size_t frame = 0u; frame < kFrameWnd; frame++) {
      for (size_t i = 0; i < (config->OfdmDataNum() * config->BfAntNum());
           i++) {
        calib_dl_buffer_[frame][i] = complex_init;
        calib_ul_buffer_[frame][i] = complex_init;
        calib_dl_msum_buffer_[frame][i] = complex_init;
        calib_ul_msum_buffer_[frame][i] = complex_init;
      }
    }
    dl_mod_bits_buffer_.Calloc(
        task_buffer_symbol_num,
        Roundup<64>(config->GetOFDMDataNum()) * config->UeAntNum(),
        Agora_memory::Alignment_t::kAlign64);

    encode_counters_.Init(
        config->Frame().NumDlDataSyms(),
        config->LdpcConfig(Direction::kDownlink).NumBlocksInSymbol() *
            config->UeAntNum());
    encode_cur_frame_for_symbol_ =
        std::vector<size_t>(config->Frame().NumDLSyms(), SIZE_MAX);
    ifft_cur_frame_for_symbol_ =
        std::vector<size_t>(config->Frame().NumDLSyms(), SIZE_MAX);
    precode_counters_.Init(config->Frame().NumDLSyms(),
                           config->DemulEventsPerSymbol());
    // precode_cur_frame_for_symbol_ =
    //    std::vector<size_t>(config->Frame().NumDLSyms(), SIZE_MAX);
    ifft_counters_.Init(config->Frame().NumDLSyms(), config->BsAntNum());
    tx_counters_.Init(config->Frame().NumDLSyms(), config->BsAntNum());
    // mac data is sent per frame, so we set max symbol to 1
    mac_to_phy_counters_.Init(1, config->UeAntNum());
  }
}

void FreeUplinkBuffers() {
  socket_buffer_.Free();
  data_buffer_.Free();
  equal_buffer_.Free();
  ue_spec_pilot_buffer_.Free();
}

void FreeDownlinkBuffers() {
  if (config->Frame().NumDLSyms() > 0) {
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
