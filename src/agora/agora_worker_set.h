/**
 * @file agora_worker_set.h
 * @brief Declaration file for the main Agora worker set class
 */

#ifndef AGORA_WORKER_SET_H_
#define AGORA_WORKER_SET_H_

#include <memory>
#include <thread>
#include <vector>

#include "agora_buffer.h"
#include "config.h"
#include "csv_logger.h"
#include "mac_scheduler.h"
#include "mat_logger.h"
#include "phy_stats.h"
#include "rp_config.h"
#include "stats.h"

class AgoraWorkerSet {
 public:
  explicit AgoraWorkerSet(Config* cfg, MacScheduler* mac_sched, Stats* stats,
                          PhyStats* phy_stats, MessageInfo* message,
                          AgoraBuffer* buffer, FrameInfo* frame);
  ~AgoraWorkerSet();
  void UpdateCores(RPControlMsg rcm);
  size_t GetCoresInfo();

 private:
  void WorkerThread(int tid);
  void CreateThreads();

  const size_t base_worker_core_offset_;

  Config* const config_;

  std::vector<std::thread> workers_;
  std::vector<bool> active_core_;

  MacScheduler* mac_sched_;
  Stats* stats_;
  PhyStats* phy_stats_;
  MessageInfo* message_;
  AgoraBuffer* buffer_;
  FrameInfo* frame_;
};

#endif  // AGORA_WORKER_SET_H_
