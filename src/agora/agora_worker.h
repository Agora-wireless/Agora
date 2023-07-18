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
#include "stats.h"
#include "doer.h"

class AgoraWorker {
 public:
  explicit AgoraWorker(Config* cfg, MacScheduler* mac_sched, Stats* stats,
                       PhyStats* phy_stats, MessageInfo* message,
                       AgoraBuffer* buffer, FrameInfo* frame);
  ~AgoraWorker();

  void RunWorker();

 private:
  void InitializeWorker();

  const size_t base_worker_core_offset_;

  Config* const config_;
  std::vector<std::thread> workers_;

  MacScheduler* mac_sched_;
  Stats* stats_;
  PhyStats* phy_stats_;
  MessageInfo* message_;
  AgoraBuffer* buffer_;
  FrameInfo* frame_;

  std::vector<std::shared_ptr<Doer> > computers_vec;
  std::vector<EventType> events_vec;
  int tid; // TODO: remove thread id for single-core
  size_t cur_qid;
  size_t empty_queue_itrs;
  bool empty_queue;
};

#endif  // AGORA_WORKER_H_