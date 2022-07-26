/**
 * @file worker.cc
 * @brief Implementation file for the main worker class
 */

#include "worker.h"

Worker::Worker(Config* cfg, Stats* stats,
               PhyStats* phy_stats, MessageInfo* queues,
               Buffer* buffers, FrameInfo* frame_info)
    : base_worker_core_offset_(cfg->CoreOffset() + 1 + cfg->SocketThreadNum()),
      config_(cfg),
      stats_(stats),
      phy_stats_(phy_stats),
      queues_(queues),
      buffers_(buffers),
      frame_info_(frame_info) {
  // create threads
  CreateThreads();
}

Worker::~Worker() {
  for (auto& worker_thread : workers_) {
    AGORA_LOG_SYMBOL("Agora: Joining worker thread\n");
    worker_thread.join();
  }
}

void Worker::CreateThreads() {
  //   if (config_->BigstationMode() == true) {
  //     for (size_t i = 0; i < config_->FftThreadNum(); i++) {
  //       workers_.emplace_back(&Worker::WorkerFft, this, i);
  //     }
  //     for (size_t i = config_->FftThreadNum();
  //          i < config_->FftThreadNum() + config_->ZfThreadNum(); i++) {
  //       workers_.emplace_back(&Worker::WorkerZf, this, i);
  //     }
  //     for (size_t i = config_->FftThreadNum() + config_->ZfThreadNum();
  //          i < config_->FftThreadNum() + config_->ZfThreadNum() +
  //                  config_->DemulThreadNum();
  //          i++) {
  //       workers_.emplace_back(&Worker::WorkerDemul, this, i);
  //     }
  //     for (size_t i = config_->FftThreadNum() + config_->ZfThreadNum() +
  //                     config_->DemulThreadNum();
  //          i < config_->WorkerThreadNum(); i++) {
  //       workers_.emplace_back(&Worker::WorkerDecode, this, i);
  //     }
  //   } else {
  AGORA_LOG_SYMBOL("Worker: creating %zu workers\n",
                   config_->WorkerThreadNum());
  for (size_t i = 0; i < config_->WorkerThreadNum(); i++) {
    workers_.emplace_back(&Worker::WorkerThread, this, i);
  }
  //   }
  if (kEnableMatLog) {
    for (size_t i = 0; i < mat_loggers_.size(); i++) {
      if ((i != CsvLog::kMatDLZF) || (config->Frame().NumDLSyms() > 0)) {
        mat_loggers_.at(i) =
            std::make_shared<CsvLog::MatLogger>(config->RadioId().at(0), i);
      }
    }
  }
}

void Worker::WorkerThread(int tid) {
  PinToCoreWithOffset(ThreadType::kWorker, base_worker_core_offset_, tid);

  /* Initialize operators */
  auto compute_zf = std::make_unique<DoZF>(
      config_, tid, buffers_->csi_buffer, buffers_->calib_dl_buffer,
      buffers_->calib_ul_buffer, buffers_->calib_dl_msum_buffer,
      buffers_->calib_ul_msum_buffer, buffers_->ul_zf_matrices,
      buffers_->dl_zf_matrices, phy_stats_.get(), stats_.get(),
      mat_loggers_.at(CsvLog::kMatCSI), mat_loggers_.at(CsvLog::kMatDLZF));

  auto compute_fft = std::make_unique<DoFFT>(
      config_, tid, buffers_->data_buffer, buffers_->csi_buffers,
      buffers_->calib_dl_buffer, buffers_->calib_ul_buffer, phy_stats_.get(),
      stats_.get());

  // Downlink workers
  auto compute_ifft =
      std::make_unique<DoIFFT>(config_, tid, buffers_->dl_ifft_buffer,
                               buffers_->dl_socket_buffer, stats_.get());

  auto compute_precode = std::make_unique<DoPrecode>(
      config_, tid, buffers_->dl_zf_matrices, buffers_->dl_ifft_buffer,
      buffers_->dl_mod_bits_buffer, stats_.get());

  auto compute_encoding = std::make_unique<DoEncode>(
      config_, tid, Direction::kDownlink,
      (kEnableMac == true) ? buffers_->dl_bits_buffer : config_->DlBits(),
      (kEnableMac == true) ? kFrameWnd : 1, buffers_->dl_mod_bits_buffer,
      stats_.get());

  // Uplink workers
  auto compute_decoding = std::make_unique<DoDecode>(
      config_, tid, buffers_->demod_buffers, buffers_->decoded_buffer,
      phy_stats_.get(), stats_.get());

  auto compute_demul = std::make_unique<DoDemul>(
      config_, tid, buffers_->data_buffer, buffers_->ul_zf_matrices,
      buffers_->ue_spec_pilot_buffer, buffers_->equal_buffer,
      buffers_->demod_buffers, phy_stats_.get(), stats_.get());

  std::vector<Doer*> computers_vec;
  std::vector<EventType> events_vec;
  ///*************************
  computers_vec.push_back(compute_zf.get());
  computers_vec.push_back(compute_fft.get());
  events_vec.push_back(EventType::kZF);
  events_vec.push_back(EventType::kFFT);

  if (config->Frame().NumULSyms() > 0) {
    computers_vec.push_back(compute_decoding.get());
    computers_vec.push_back(compute_demul.get());
    events_vec.push_back(EventType::kDecode);
    events_vec.push_back(EventType::kDemul);
  }

  if (config->Frame().NumDLSyms() > 0) {
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
  while (config->Running() == true) {
    for (size_t i = 0; i < computers_vec.size(); i++) {
      if (computers_vec.at(i)->TryLaunch(
              *GetConq(sched_info_arr_, events_vec.at(i), cur_qid),
              queues_->complete_task_queue[cur_qid],
              queues_->worker_ptoks_ptr[tid][cur_qid])) {
        empty_queue = false;
        break;
      }
    }
    // If all queues in this set are empty for 5 iterations,
    // check the other set of queues
    if (empty_queue == true) {
      empty_queue_itrs++;
      if (empty_queue_itrs == 5) {
        if (frame_info_->cur_sche_frame_id != frame_info_->cur_proc_frame_id) {
          cur_qid ^= 0x1;
        } else {
          cur_qid = (frame_info_->cur_sche_frame_id & 0x1);
        }
        empty_queue_itrs = 0;
      }
    } else {
      empty_queue = true;
    }
  }
  AGORA_LOG_SYMBOL("Agora worker %d exit\n", tid);
}