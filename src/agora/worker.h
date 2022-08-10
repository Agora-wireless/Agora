/**
 * @file worker.h
 * @brief Declaration file for the main worker class
 */

#ifndef WORKER_H_
#define WORKER_H_

#include <vector>

#include "agora_helper.h"
#include "buffer.h"
#include "concurrent_queue_wrapper.h"
#include "concurrentqueue.h"
#include "config.h"
#include "csv_logger.h"
#include "dodecode.h"
#include "dodemul.h"
#include "doencode.h"
#include "dofft.h"
#include "doifft.h"
#include "doprecode.h"
#include "dozf.h"
#include "mat_logger.h"
#include "phy_stats.h"
#include "stats.h"
#include "symbols.h"

class Worker {
 public:
  explicit Worker(Config*, Stats*, PhyStats*, MessageInfo*, Buffer*);
  ~Worker();

 private:
  //   void WorkerFft(int tid);
  //   void WorkerZf(int tid);
  //   void WorkerDemul(int tid);
  //   void WorkerDecode(int tid);
  void WorkerThread(int tid);
  void CreateThreads();

  const size_t base_worker_core_offset_;

  Config* const config_;
  std::vector<std::thread> workers_;

  // MessageInfo* queues_;
//   SchedInfo sched_info_arr_[kScheduleQueues][kNumEventTypes];
  MessageInfo* message_;
  Buffer* buffer_;
  // FrameInfo* frame_info_;
  Stats* stats_;
  PhyStats* phy_stats_;
  //   moodycamel::ConcurrentQueue<EventData> complete_task_queue[kScheduleQueues];
  //   moodycamel::ProducerToken* worker_ptok[kMaxThreads][kScheduleQueues];

  size_t cur_sche_frame_id;
  size_t cur_proc_frame_id;

  //   Table<complex_float>& data_buffer;
  //   Table<complex_float>& equal_buffer;
  //   Table<complex_float>& ue_spec_pilot_buffer;
  //   Table<complex_float>& calib_ul_buffer;
  //   Table<complex_float>& calib_dl_buffer;
  //   Table<complex_float>& calib_ul_msum_buffer;
  //   Table<complex_float>& calib_dl_msum_buffer;
  //   Table<complex_float>& dl_ifft_buffer;
  //   Table<int8_t>& dl_mod_bits_buffer;
  //   Table<int8_t>& dl_bits_buffer;
  //   PtrGrid<kFrameWnd, kMaxUEs, complex_float>& csi_buffer;
  //   PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& ul_zf_matrix;
  //   PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& dl_zf_matrix;
  //   PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& demod_buffer;
  //   PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& decoded_buffer;
  //   char* dl_socket_buffer;

  std::array<std::shared_ptr<CsvLog::MatLogger>, CsvLog::kMatLogs> mat_loggers_;
};

#endif  // WORKER_H_