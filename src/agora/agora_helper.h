/**
 * @file worker.h
 * @brief Declaration file for the main worker class
 */

#ifndef HELPER_H_
#define HELPER_H_

#include <unistd.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <queue>
#include <system_error>

#include "buffer.h"
#include "concurrent_queue_wrapper.h"
#include "concurrentqueue.h"
#include "config.h"
#include "mac_thread_basestation.h"
#include "mat_logger.h"
#include "memory_manage.h"
#include "packet_txrx.h"
#include "stats.h"
#include "symbols.h"

struct SchedInfo {
  moodycamel::ConcurrentQueue<EventData> concurrent_q;
  moodycamel::ProducerToken* ptok;
};

struct MessageInfo {
  moodycamel::ConcurrentQueue<EventData> complete_task_queue_[kScheduleQueues];
  moodycamel::ProducerToken* worker_ptoks_ptr_[kMaxThreads][kScheduleQueues];
  SchedInfo sched_info_arr_[kScheduleQueues][kNumEventTypes];
};

struct Buffer {
  // Table<char> socket_buffer;
  // size_t socket_buffer_size;
  PtrGrid<kFrameWnd, kMaxUEs, complex_float> csi_buffer_;
  PtrGrid<kFrameWnd, kMaxDataSCs, complex_float> ul_zf_matrix_;
  PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t> demod_buffer_;
  PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t> decoded_buffer_;
  PtrGrid<kFrameWnd, kMaxDataSCs, complex_float> dl_zf_matrix_;
  Table<complex_float> data_buffer_;
  Table<complex_float> equal_buffer_;
  Table<complex_float> ue_spec_pilot_buffer_;
  Table<complex_float> dl_ifft_buffer_;
  Table<complex_float> calib_ul_buffer_;
  Table<complex_float> calib_dl_buffer_;
  Table<complex_float> calib_ul_msum_buffer_;
  Table<complex_float> calib_dl_msum_buffer_;
  Table<int8_t> dl_mod_bits_buffer_;
  Table<int8_t> dl_bits_buffer_;
  char* dl_socket_buffer_;
  // Table<int8_t> dl_bits_buffer_status;
};

struct Counter {
  FrameCounters pilot_fft_counters;
  FrameCounters uplink_fft_counters;
  FrameCounters zf_counters;
  FrameCounters demul_counters;
  FrameCounters decode_counters;
  FrameCounters encode_counters;
  FrameCounters precode_counters;
  FrameCounters ifft_counters;
  FrameCounters tx_counters;
  FrameCounters tomac_counters;
  FrameCounters mac_to_phy_counters;
  FrameCounters rc_counters;
  RxCounters rx_counters;
};

struct FrameInfo {
  size_t* cur_sche_frame_id;
  size_t* cur_proc_frame_id;
  // size_t* fft_created_count;
  // std::vector<size_t>* fft_cur_frame_for_symbol;
  // std::vector<size_t>* encode_cur_frame_for_symbol;
  // std::vector<size_t>* ifft_cur_frame_for_symbol;
};

struct Thread {
  std::unique_ptr<PacketTxRx> packet_tx_rx;
  std::unique_ptr<MacThreadBaseStation> mac_thread;
  std::thread mac_std_thread;
};

// Fetch the concurrent queue for this event type
inline moodycamel::ConcurrentQueue<EventData>* GetConq(
    SchedInfo sched_info_arr[kScheduleQueues][kNumEventTypes],
    EventType event_type, size_t qid) {
  return &sched_info_arr[qid][static_cast<size_t>(event_type)].concurrent_q;
}

// Fetch the producer token for this event type
inline moodycamel::ProducerToken* GetPtok(
    SchedInfo sched_info_arr[kScheduleQueues][kNumEventTypes],
    EventType event_type, size_t qid) {
  return sched_info_arr[qid][static_cast<size_t>(event_type)].ptok;
}

#endif  // HELPER_H_