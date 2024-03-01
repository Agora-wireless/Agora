/**
 * @file agora_worker_set.cc
 * @brief Implementation file for the main Agora worker set (collection) class
 */

#include "agora_worker_set.h"

#include "concurrent_queue_wrapper.h"
#include "csv_logger.h"
#include "dobeamweights.h"
#include "dobroadcast.h"
#include "dodecode.h"
#include "dodemul.h"
#include "doencode.h"
#include "dofft.h"
#include "doifft.h"
#include "doprecode.h"
#include "logger.h"

AgoraWorkerSet::AgoraWorkerSet(Config* cfg, MacScheduler* mac_sched,
                               Stats* stats, PhyStats* phy_stats,
                               MessageInfo* message, AgoraBuffer* buffer,
                               FrameInfo* frame)
    : base_worker_core_offset_(cfg->CoreOffset() + 1 + cfg->SocketThreadNum() +
                               (cfg->DynamicCoreAlloc() ? 1 : 0)),
      config_(cfg),
      mac_sched_(mac_sched),
      stats_(stats),
      phy_stats_(phy_stats),
      message_(message),
      buffer_(buffer),
      frame_(frame) {
  CreateWorkers();
}

AgoraWorkerSet::~AgoraWorkerSet() {
  // Destruct all the workers
  for (auto& worker : workers_) {
    worker->Disable();
    worker.reset();
  }
  workers_.clear();
}

void AgoraWorkerSet::CreateWorkers() {
  AGORA_LOG_SYMBOL("Worker: Creating %zu workers\n",
                   config_->WorkerThreadNum());
  const auto system_codes = sysconf(_SC_NPROCESSORS_ONLN);
  active_core_.resize(system_codes);
  for (size_t i = 0; i < config_->WorkerThreadNum(); i++) {
    active_core_.at(i) = true;
    // workers_.emplace_back(&AgoraWorkerSet::WorkerThread, this, i);
    workers_.push_back(std::make_unique<AgoraWorker>(
        config_, mac_sched_, stats_, phy_stats_, message_, buffer_, frame_, i,
        i + base_worker_core_offset_));
  }
  for (long i = config_->WorkerThreadNum(); i < system_codes; i++) {
    active_core_.at(i) = false;
  }
}

void AgoraWorkerSet::UpdateCores(RPControlMsg rcm) {
  AGORA_LOG_INFO("=================================\n");
  AGORA_LOG_INFO(
      "Agora: Updating compute resources for workers thread - currently used: "
      "%ld, total: %ld\n",
      workers_.size(),
      sysconf(_SC_NPROCESSORS_ONLN) - base_worker_core_offset_);

  // Target core numbers
  const size_t current_core_num = workers_.size();
  size_t updated_core_num = workers_.size() + rcm.msg_arg_1_ - rcm.msg_arg_2_;
  // TODO: (size_t)sysconf(_SC_NPROCESSORS_ONLN) gives all available core # in the machine
  const size_t max_core_num =
      sysconf(_SC_NPROCESSORS_ONLN) - base_worker_core_offset_;

  AGORA_LOG_INFO(
      "[ALERTTTTTT]: CPU Layout Update!!! current_core_num: %zu, "
      "updated_core_num: %zu, base_worker_core_offset: %zu, max_core_num: "
      "%zu\n",
      current_core_num, updated_core_num, base_worker_core_offset_, max_core_num);

  // Update workers
  if (workers_.size() < updated_core_num) {
    // Add workers
    updated_core_num = std::min(updated_core_num, max_core_num);

    for (size_t core_i = current_core_num; core_i < updated_core_num; core_i++) {
      // Update info
      if (active_core_.at(core_i) == true) {
        std::runtime_error(
            "Attempted to add a core that is already active!!!!");
      }
      active_core_.at(core_i) = true;
      // workers_.emplace_back(&AgoraWorkerSet::WorkerThread, this, core_i);
      workers_.push_back(std::make_unique<AgoraWorker>(
          config_, mac_sched_, stats_, phy_stats_, message_, buffer_, frame_,
          core_i, core_i + base_worker_core_offset_));
      AGORA_LOG_INFO("Agora: added worker %zu at core #%zu\n", core_i,
                     core_i + base_worker_core_offset_);
    }
  } else {
    // Remove workers
    // minimum core number?
    updated_core_num = std::max(updated_core_num, kMinWorkers);
    // Remove from back to front....
    for (size_t core_i = current_core_num; core_i > updated_core_num; core_i--) {
      // \todo update the core numbers so we don't have to do this.
      const size_t core_index = core_i - 1;
      // Update info
      AGORA_LOG_INFO("Agora: removing worker #%zu\n", core_index);
      if (active_core_.at(core_index) == false) {
        std::runtime_error(
            "Attempted to remove a core that is already inactive!!!!");
      }
      active_core_.at(core_index) = false;
      // Makes sense to move this to the worker class... but won't for now.
      // RemoveCoreFromList(core_index, base_worker_core_offset_);
      // workers_.at(core_index).join();
      auto& del_worker = workers_.back();
      del_worker->Disable();
      del_worker.reset();
      workers_.pop_back();
    }
    // Might be able to add this back in and remove the pop_back();
    // workers_.resize(updated_core_num);
  }

  AGORA_LOG_INFO(
      "Agora: Compute resource update is complete - currently used: %ld, "
      "total: %ld\n",
      workers_.size(),
      sysconf(_SC_NPROCESSORS_ONLN) - base_worker_core_offset_);
  AGORA_LOG_INFO("=================================\n");
}

size_t AgoraWorkerSet::GetCoresInfo() { return workers_.size(); }
