/**
 * @file agora_worker.h
 * @brief Declaration file for the main Agora worker class
 */

#ifndef AGORA_WORKER_H_
#define AGORA_WORKER_H_

#include "buffer.h"
#include "concurrent_queue_wrapper.h"
#include "csv_logger.h"
#include "dobeamweights.h"
#include "dodecode.h"
#include "dodemul.h"
#include "doencode.h"
#include "dofft.h"
#include "doifft.h"
#include "doprecode.h"
#include "mat_logger.h"
#include "phy_stats.h"

class AgoraWorker {
 public:
  explicit AgoraWorker(Config* /*cfg*/, Stats* /*stats*/,
                       PhyStats* /*phy_stats*/, MessageInfo* /*message*/,
                       AgoraBuffer* /*agora_buffer*/, FrameInfo* /*frame*/);
  ~AgoraWorker();

 private:
  void WorkerThread(int tid);
  void CreateThreads();

  const size_t base_worker_core_offset_;

  Config* const config_;
  std::vector<std::thread> workers_;

  Stats* stats_;
  PhyStats* phy_stats_;
  MessageInfo* message_;
  AgoraBuffer* buffer_;
  FrameInfo* frame_;

  size_t cur_sche_frame_id_;
  size_t cur_proc_frame_id_;

  std::array<std::shared_ptr<CsvLog::MatLogger>, CsvLog::kMatLogs> mat_loggers_;
};

#endif  // AGORA_WORKER_H_