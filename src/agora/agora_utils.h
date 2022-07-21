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

/* Parameters */
// TODO: some should be moved to symbols.h
static const bool kDebugPrintPacketsFromMac = false;
static const bool kDebugDeferral = true;
static const size_t kDefaultMessageQueueSize = 512;
static const size_t kDefaultWorkerQueueSize = 256;
// Max number of worker threads allowed
static const size_t kMaxWorkerNum = 50;
static const size_t kScheduleQueues = 2;
// Dequeue batch size, used to reduce the overhead of dequeue in main thread
static const size_t kDequeueBulkSizeTXRX = 8;
static const size_t kDequeueBulkSizeWorker = 4;

enum ScheduleProcessingFlags : uint8_t {
  kNone = 0,
  kUplinkComplete = 0x1,
  kDownlinkComplete = 0x2,
  kProcessingComplete = (kUplinkComplete + kDownlinkComplete)
};

struct SchedInfoT {
  moodycamel::ConcurrentQueue<EventData> concurrent_q;
  moodycamel::ProducerToken* ptok;
};
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
std::vector<std::thread> workers_;

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

/* Util functions */
// Fetch the concurrent queue for this event type
moodycamel::ConcurrentQueue<EventData>* GetConq(EventType event_type,
                                                size_t qid) {
  return &sched_info_arr[qid][static_cast<size_t>(event_type)].concurrent_q;
}

// Fetch the producer token for this event type
moodycamel::ProducerToken* GetPtok(EventType event_type, size_t qid) {
  return sched_info_arr[qid][static_cast<size_t>(event_type)].ptok;
}

// Return a string containing the sizes of the FFT queues
std::string GetFftQueueSizesString() {
  std::ostringstream ret;
  ret << "[";
  for (size_t i = 0; i < kFrameWnd; i++) {
    ret << std::to_string(fft_queue_arr_[i].size()) << " ";
  }
  ret << "]";
  return ret.str();
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