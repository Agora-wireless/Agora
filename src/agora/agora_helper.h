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
#include <vector>

#include "buffer.h"
#include "concurrentqueue.h"
#include "config.h"
#include "memory_manage.h"
#include "packet_txrx.h"
#include "stats.h"
#include "symbols.h"

struct SchedInfo {
  moodycamel::ConcurrentQueue<EventData> concurrent_q_;
  moodycamel::ProducerToken* ptok_;
};

// Used to communicate between the manager and the worker class
struct MessageInfo {
  moodycamel::ConcurrentQueue<EventData> complete_task_queue_[kScheduleQueues];
  moodycamel::ProducerToken* worker_ptoks_ptr_[kMaxThreads][kScheduleQueues];
  SchedInfo sched_info_arr_[kScheduleQueues][kNumEventTypes];
};

// Initialized in manager class and used in worker class
struct Buffer {
  PtrGrid<kFrameWnd, kMaxUEs, complex_float> csi_buffer_;
  PtrGrid<kFrameWnd, kMaxDataSCs, complex_float> ul_beam_matrix_;
  PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t> demod_buffer_;
  PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t> decoded_buffer_;
  PtrGrid<kFrameWnd, kMaxDataSCs, complex_float> dl_beam_matrix_;
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
};

// struct Counter {
//   FrameCounters pilot_fft_counters;
//   FrameCounters uplink_fft_counters;
//   FrameCounters zf_counters;
//   FrameCounters demul_counters;
//   FrameCounters decode_counters;
//   FrameCounters encode_counters;
//   FrameCounters precode_counters;
//   FrameCounters ifft_counters;
//   FrameCounters tx_counters;
//   FrameCounters tomac_counters;
//   FrameCounters mac_to_phy_counters;
//   FrameCounters rc_counters;
//   RxCounters rx_counters;
// };

struct FrameInfo {
  size_t cur_sche_frame_id_;
  size_t cur_proc_frame_id_;
};

#endif  // HELPER_H_