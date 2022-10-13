/**
 * @file agora_worker.h
 * @brief Declaration file for the main Agora worker class
 */

#ifndef AGORA_WORKER_H_
#define AGORA_WORKER_H_

#include <memory>
#include <thread>
#include <vector>

#include "agora_buffer.h"
#include "config.h"
#include "csv_logger.h"
#include "mat_logger.h"
#include "phy_stats.h"
#include "stats.h"

class AgoraWorker {
 public:
  explicit AgoraWorker(Config* cfg, Stats* stats, PhyStats* phy_stats,
                       MessageInfo* message, AgoraBuffer* buffer,
                       FrameInfo* frame);
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
};

#endif  // AGORA_WORKER_H_