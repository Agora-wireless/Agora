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
  explicit Worker(Config*, Stats*, PhyStats*, MessageInfo*, Buffer*,
                  FrameInfo*);
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

  Stats* stats_;
  PhyStats* phy_stats_;
  MessageInfo* message_;
  Buffer* buffer_;
  FrameInfo* frame_;

  size_t cur_sche_frame_id;
  size_t cur_proc_frame_id;

  std::array<std::shared_ptr<CsvLog::MatLogger>, CsvLog::kMatLogs> mat_loggers_;
};

#endif  // WORKER_H_