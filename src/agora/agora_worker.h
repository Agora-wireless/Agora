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
#include "mac_scheduler.h"
#include "mat_logger.h"
#include "phy_stats.h"
#include "rp_config.h"
#include "stats.h"

class AgoraWorker {
 public:
  explicit AgoraWorker(Config* cfg, MacScheduler* mac_sched, Stats* stats,
                       PhyStats* phy_stats, MessageInfo* message,
                       AgoraBuffer* buffer, FrameInfo* frame, size_t worker_id,
                       size_t core_id);
  ~AgoraWorker();

  inline void Disable() { enabled_.store(false); }

 private:
  void WorkLoop();

  Config* const config_;
  std::atomic<bool> enabled_;

  std::thread thread_;
  const size_t worker_id_;
  const size_t core_id_;

  MacScheduler* mac_sched_;
  Stats* stats_;
  PhyStats* phy_stats_;
  MessageInfo* message_;
  AgoraBuffer* buffer_;
  FrameInfo* frame_;
};

#endif  // AGORA_WORKER_H_
