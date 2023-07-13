/**
 * @file agora_worker.cc
 * @brief Implementation file for the main Agora worker class
 */

// debug
#include <typeinfo>

#include "agora_worker.h"

#include "concurrent_queue_wrapper.h"
#include "csv_logger.h"
#include "dobeamweights.h"
#include "dodecode.h"
#include "dodemul.h"
#include "doencode.h"
#include "dofft.h"
#include "doifft.h"
#include "doprecode.h"
#include "logger.h"

AgoraWorker::AgoraWorker(Config* cfg, Stats* stats, PhyStats* phy_stats,
                         MessageInfo* message, AgoraBuffer* buffer,
                         FrameInfo* frame)
    : base_worker_core_offset_(cfg->CoreOffset() + 1 + cfg->SocketThreadNum()),
      config_(cfg),
      stats_(stats),
      phy_stats_(phy_stats),
      message_(message),
      buffer_(buffer),
      frame_(frame) {
  CreateThreads();
}

AgoraWorker::~AgoraWorker() {
  // debug: no worker threads anymore
  // for (auto& worker_thread : workers_) {
  //   AGORA_LOG_SYMBOL("Agora: Joining worker thread\n");
  //   if (worker_thread.joinable()) {
  //     worker_thread.join();
  //   }
  // }
}

void AgoraWorker::CreateThreads() {
  /* Limit the number of thread to be 1 */
  if (config_->WorkerThreadNum() != 1) {
    AGORA_LOG_ERROR("Worker: Single-core mode allows only 1 thread!\n");
    exit(EXIT_FAILURE);
  }
  AGORA_LOG_INFO("Worker: working with %zu worker threads\n",
                 config_->WorkerThreadNum());
  
  // Directly call the worker in the same thread
  // AgoraWorker::WorkerThread(0);

  // AGORA_LOG_SYMBOL("Worker: creating %zu workers\n",
  //                  config_->WorkerThreadNum());
  // for (size_t i = 0; i < config_->WorkerThreadNum(); i++) {
  //   workers_.emplace_back(&AgoraWorker::WorkerThread, this, i);
  // }
  InitializeWorker(0);
}

void AgoraWorker::InitializeWorker(int tid) {

  AGORA_LOG_INFO("Worker: Initialize worker (function)\n");

  // PinToCoreWithOffset(ThreadType::kWorker, base_worker_core_offset_, tid);
  // AGORA_LOG_INFO("Worker: Core binding succeeded\n");

  /* Initialize operators */
  // debug: use make_shared intead of make_unique to escape the scope
  auto compute_beam = std::make_shared<DoBeamWeights>(
      config_, tid, buffer_->GetCsi(), buffer_->GetCalibDl(),
      buffer_->GetCalibUl(), buffer_->GetCalibDlMsum(),
      buffer_->GetCalibUlMsum(), buffer_->GetCalib(),
      buffer_->GetUlBeamMatrix(), buffer_->GetDlBeamMatrix(), phy_stats_,
      stats_);

  auto compute_fft = std::make_shared<DoFFT>(
      config_, tid, buffer_->GetFft(), buffer_->GetCsi(), buffer_->GetCalibDl(),
      buffer_->GetCalibUl(), phy_stats_, stats_);

  // Downlink workers
  auto compute_ifft = std::make_shared<DoIFFT>(config_, tid, buffer_->GetIfft(),
                                               buffer_->GetDlSocket(), stats_);

  auto compute_precode = std::make_shared<DoPrecode>(
      config_, tid, buffer_->GetDlBeamMatrix(), buffer_->GetIfft(),
      buffer_->GetDlModBits(), stats_);

  auto compute_encoding = std::make_shared<DoEncode>(
      config_, tid, Direction::kDownlink,
      (kEnableMac == true) ? buffer_->GetDlBits() : config_->DlBits(),
      (kEnableMac == true) ? kFrameWnd : 1, buffer_->GetDlModBits(), stats_);

  // Uplink workers
  auto compute_decoding =
      std::make_shared<DoDecode>(config_, tid, buffer_->GetDemod(),
                                 buffer_->GetDecod(), phy_stats_, stats_);

  auto compute_demul = std::make_shared<DoDemul>(
      config_, tid, buffer_->GetFft(), buffer_->GetUlBeamMatrix(),
      buffer_->GetUeSpecPilot(), buffer_->GetEqual(), buffer_->GetDemod(),
      phy_stats_, stats_);

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

  if (config_->Frame().NumDLSyms() > 0) {
    computers_vec.push_back(compute_ifft.get());
    computers_vec.push_back(compute_precode.get());
    computers_vec.push_back(compute_encoding.get());
    events_vec.push_back(EventType::kIFFT);
    events_vec.push_back(EventType::kPrecode);
    events_vec.push_back(EventType::kEncode);
  }

  AGORA_LOG_INFO("Worker: Initialization finished\n");

  cur_qid = 0;
  empty_queue_itrs = 0;
  empty_queue = true;
}

void AgoraWorker::RunWorker(int tid) {
  if (config_->Running() == true) {
    // debug
    AGORA_LOG_INFO("Worker: Enters the main loop\n");
    printf("[debug] Worker: Enters the main loop\n");
    for (size_t i = 0; i < computers_vec.size(); i++) {
      // AGORA_LOG_INFO("Worker: Get the first working pointer\n");
      // // The worker cannot find anything in the queue
      // AGORA_LOG_INFO("Worker: Computer Vector Length = %d\n",
      //                computers_vec.size());
      // AGORA_LOG_INFO("Worker: Task queue length = %d\n", message_)
      // exit(EXIT_FAILURE);
      // printf("[debug] Worker: computers_vec length = %ld\n", computers_vec.size());
      // printf("[debug] Worker: message_ pointer = %p\n", message_);
      // printf("[debug] Worker: cur_qid = %ld\n", cur_qid);
      // printf("[debug] Worker: tid = %d\n", tid);
      // printf("[debug] Worker: events_vec length = %ld\n", events_vec.size());
      // printf("[debug] Worker: *message_->GetConq(events_vec.at(i), cur_qid) = %p\n", &(*message_->GetConq(events_vec.at(i), cur_qid)));
      // printf("[debug] Worker: message_->GetCompQueue(cur_qid) = %p\n", &(message_->GetCompQueue(cur_qid)));
      // printf("[debug] Worker: message_->GetWorkerPtok(cur_qid, tid) = %p\n", message_->GetWorkerPtok(cur_qid, tid));

      // use `c++filt -t` to "demangle" (interpret) the output type from log
      // printf("[debug] Worker: computers_vec.at(%ld) type = %s\n", i, typeid(computers_vec.at(i)).name());
      printf("[debug] Worker: *computers_vec.at(%ld) type = %s\n", i, typeid(*computers_vec.at(i)).name());
      if (computers_vec.at(i)->TryLaunch(
              *message_->GetConq(events_vec.at(i), cur_qid),
              message_->GetCompQueue(cur_qid),
              message_->GetWorkerPtok(cur_qid, tid))) {
        empty_queue = false;

        AGORA_LOG_INFO("Worker: Finished one task from the concurrent queue\n");
        printf("[debug] Worker: finished one task from the concurrent queue\n");
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
  // AGORA_LOG_SYMBOL("Agora worker %d exit\n", tid);
}
