/**
 * @file agora_worker.cc
 * @brief Implementation file for the main Agora worker class
 */

#include "agora_worker.h"

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

AgoraWorker::AgoraWorker(Config* cfg, MacScheduler* mac_sched, Stats* stats,
                         PhyStats* phy_stats, MessageInfo* message,
                         AgoraBuffer* buffer, FrameInfo* frame)
    : base_worker_core_offset_(cfg->CoreOffset() + 1 + cfg->SocketThreadNum() +
                               (cfg->DynamicCoreAlloc() ? 1 : 0)),
      config_(cfg),
      mac_sched_(mac_sched),
      stats_(stats),
      phy_stats_(phy_stats),
      message_(message),
      buffer_(buffer),
      frame_(frame) {
  CreateThreads();
}

AgoraWorker::~AgoraWorker() {
  for (auto& worker_thread : workers_) {
    AGORA_LOG_SYMBOL("Agora: Joining worker thread\n");
    if (worker_thread.joinable()) {
      worker_thread.join();
    }
  }
}

void AgoraWorker::CreateThreads() {
  AGORA_LOG_SYMBOL("Worker: Creating %zu workers\n",
                   config_->WorkerThreadNum());
  active_core_.resize(sysconf(_SC_NPROCESSORS_ONLN));
  for (size_t i = 0; i < config_->WorkerThreadNum(); i++) {
    active_core_[i] = true;
    workers_.emplace_back(&AgoraWorker::WorkerThread, this, i);
  }
  for (size_t i = config_->WorkerThreadNum();
       i < (size_t)sysconf(_SC_NPROCESSORS_ONLN); i++) {
    active_core_[i] = false;
  }
}

void AgoraWorker::UpdateCores(RPControlMsg rcm) {
  AGORA_LOG_INFO("=================================\n");
  AGORA_LOG_INFO(
      "Agora: Updating compute resources for workers thread - currently used: "
      "%ld, total: %ld\n",
      workers_.size(),
      sysconf(_SC_NPROCESSORS_ONLN) - base_worker_core_offset_);

  // Target core numbers
  const size_t start_core_id = workers_.size();
  size_t updated_core_num = workers_.size() + rcm.msg_arg_1_ - rcm.msg_arg_2_;
  // TODO: (size_t)sysconf(_SC_NPROCESSORS_ONLN) gives all available core # in the machine
  const size_t max_core_num =
      sysconf(_SC_NPROCESSORS_ONLN) - base_worker_core_offset_;

  AGORA_LOG_INFO(
      "[ALERTTTTTT]: CPU Layout Update!!! start_core_id: %zu, updated_core_num: "
      "%zu, base_worker_core_offset: %zu, max_core_num: %zu\n",
      start_core_id, updated_core_num, base_worker_core_offset_, max_core_num);

  // Update workers
  if (workers_.size() < updated_core_num) {
    // Add workers
    updated_core_num = std::min(updated_core_num, max_core_num);

    for (size_t core_i = start_core_id; core_i < updated_core_num; core_i++) {
      // Update info
      active_core_[core_i] = true;
      workers_.emplace_back(&AgoraWorker::WorkerThread, this, core_i);
      AGORA_LOG_INFO("Agora: added core # %ld\n", core_i);
    }
  } else {
    // Remove workers
    // minimum core number?
    updated_core_num = std::max(updated_core_num, (size_t) kMinWorkers);
    for (size_t core_i = start_core_id; core_i > updated_core_num; core_i--) {
      // Update info
      active_core_[core_i - 1] = false;
      RemoveCoreFromList((core_i - 1), base_worker_core_offset_);
      workers_.at(core_i - 1).join();
      AGORA_LOG_INFO("Agora: removed core # %ld\n", (core_i - 1));
    }
    workers_.resize(updated_core_num);
  }

  AGORA_LOG_INFO(
      "Agora: Compute resource update is complete - currently used: %ld, "
      "total: %ld\n",
      workers_.size(),
      sysconf(_SC_NPROCESSORS_ONLN) - base_worker_core_offset_);
  AGORA_LOG_INFO("=================================\n");
}

size_t AgoraWorker::GetCoresInfo() { return workers_.size(); }

void AgoraWorker::WorkerThread(int tid) {
  PinToCoreWithOffset(ThreadType::kWorker, base_worker_core_offset_, tid);

  /* Initialize operators */
  auto compute_beam = std::make_unique<DoBeamWeights>(
      config_, tid, buffer_->GetCsi(), buffer_->GetCalibDl(),
      buffer_->GetCalibUl(), buffer_->GetCalibDlMsum(),
      buffer_->GetCalibUlMsum(), buffer_->GetCalib(),
      buffer_->GetUlBeamMatrix(), buffer_->GetDlBeamMatrix(), mac_sched_,
      phy_stats_, stats_);

  auto compute_fft = std::make_unique<DoFFT>(
      config_, tid, buffer_->GetFft(), buffer_->GetCsi(), buffer_->GetCalibDl(),
      buffer_->GetCalibUl(), phy_stats_, stats_);

  // Downlink workers
  auto compute_ifft = std::make_unique<DoIFFT>(config_, tid, buffer_->GetIfft(),
                                               buffer_->GetDlSocket(), stats_);

  auto compute_precode = std::make_unique<DoPrecode>(
      config_, tid, buffer_->GetDlBeamMatrix(), buffer_->GetIfft(),
      buffer_->GetDlModBits(), mac_sched_, stats_);

  auto compute_encoding = std::make_unique<DoEncode>(
      config_, tid, Direction::kDownlink,
      (kEnableMac == true) ? buffer_->GetDlBits() : config_->DlBits(),
      (kEnableMac == true) ? kFrameWnd : 1, buffer_->GetDlModBits(), mac_sched_,
      stats_);

  // Uplink workers
  auto compute_decoding = std::make_unique<DoDecode>(
      config_, tid, buffer_->GetDemod(), buffer_->GetDecod(), mac_sched_,
      phy_stats_, stats_);

  auto compute_demul = std::make_unique<DoDemul>(
      config_, tid, buffer_->GetFft(), buffer_->GetUlBeamMatrix(),
      buffer_->GetUeSpecPilot(), buffer_->GetEqual(), buffer_->GetDemod(),
      mac_sched_, phy_stats_, stats_);

  auto compute_bcast = std::make_unique<DoBroadcast>(
      config_, tid, buffer_->GetDlSocket(), stats_);

  std::vector<Doer*> computers_vec;
  std::vector<EventType> events_vec;
  ///*************************
  computers_vec.push_back(compute_beam.get());
  computers_vec.push_back(compute_fft.get());
  events_vec.push_back(EventType::kBeam);
  events_vec.push_back(EventType::kFFT);

  if (config_->Frame().NumULSyms() > 0) {
    computers_vec.push_back(compute_decoding.get());
    computers_vec.push_back(compute_demul.get());
    events_vec.push_back(EventType::kDecode);
    events_vec.push_back(EventType::kDemul);
  }

  if (config_->Frame().NumDlControlSyms() > 0) {
    computers_vec.push_back(compute_bcast.get());
    events_vec.push_back(EventType::kBroadcast);
  }

  if (config_->Frame().NumDLSyms() > 0) {
    computers_vec.push_back(compute_ifft.get());
    computers_vec.push_back(compute_precode.get());
    computers_vec.push_back(compute_encoding.get());
    events_vec.push_back(EventType::kIFFT);
    events_vec.push_back(EventType::kPrecode);
    events_vec.push_back(EventType::kEncode);
  }

  size_t cur_qid = 0;
  size_t empty_queue_itrs = 0;
  bool empty_queue = true;
  while (config_->Running() == true && active_core_.at(tid) == true) {
    for (size_t i = 0; i < computers_vec.size(); i++) {
      if (computers_vec.at(i)->TryLaunch(
            *message_->GetConq(events_vec.at(i), cur_qid),
            message_->GetCompQueue(cur_qid),
            message_->GetWorkerPtok(cur_qid, tid))) {
        empty_queue = false;

        if (((computers_vec.at(i)->enq_deq_tsc_worker_.frame_id_ ==
              config_->FrameToProfile()) and
             (cur_qid ==
              (computers_vec.at(i)->enq_deq_tsc_worker_.frame_id_ & 0x1)))) {
          size_t symbol_id =
              computers_vec.at(i)->enq_deq_tsc_worker_.symbol_id_;
          if (symbol_id > config_->Frame().NumTotalSyms()) {
            symbol_id = 0;  // kBeam event does not have a valid symbol_id
          }

          stats_->LogDequeueStatsWorker(
              tid, computers_vec.at(i)->enq_deq_tsc_worker_.frame_id_,
              symbol_id,
              computers_vec.at(i)->enq_deq_tsc_worker_.dequeue_start_tsc_,
              computers_vec.at(i)->enq_deq_tsc_worker_.dequeue_end_tsc_,
              computers_vec.at(i)->enq_deq_tsc_worker_.dequeue_diff_tsc_,
              computers_vec.at(i)->enq_deq_tsc_worker_.valid_dequeue_diff_tsc_,
              events_vec.at(i));

          stats_->LogEnqueueStatsWorker(
              tid, computers_vec.at(i)->enq_deq_tsc_worker_.frame_id_,
              symbol_id,
              computers_vec.at(i)->enq_deq_tsc_worker_.enqueue_start_tsc_,
              computers_vec.at(i)->enq_deq_tsc_worker_.enqueue_end_tsc_,
              computers_vec.at(i)->enq_deq_tsc_worker_.enqueue_diff_tsc_,
              events_vec.at(i));
        }
        break;
      }
    }
    // If all queues in this set are empty for 5 iterations,
    // check the other set of queues
    if (empty_queue == true) {
      empty_queue_itrs++;
      if (empty_queue_itrs == 5) {
        if (frame_->cur_sche_frame_id_ != frame_->cur_proc_frame_id_) {
          cur_qid ^= 0x1;
        } else {
          cur_qid = (frame_->cur_sche_frame_id_ & 0x1);
        }
        empty_queue_itrs = 0;
      }
    } else {
      empty_queue = true;
    }
  }
  AGORA_LOG_SYMBOL("Agora worker %d exit\n", tid);
}
