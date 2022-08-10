/**
 * @file worker.cc
 * @brief Implementation file for the main worker class
 */

#include "worker.h"

Worker::Worker(Config* cfg, Stats* stats, PhyStats* phy_stats,
               //  MessageInfo* queues, FrameInfo* frame_info,
               MessageInfo* message, Buffer* buffer)
    : base_worker_core_offset_(cfg->CoreOffset() + 1 + cfg->SocketThreadNum()),
      config_(cfg),
      stats_(stats),
      phy_stats_(phy_stats),
      // queues_(queues),
      // frame_info_(frame_info),
      message_(message),
      buffer_(buffer) {
  std::printf("Testing if all variables are sent correctly\n");
  std::printf("size of buffer: %d, %zu, %zu\n",
              buffer_->data_buffer_.IsAllocated(), buffer_->data_buffer_.Dim1(),
              buffer_->data_buffer_.Dim2());
  std::printf("size of queue %zu\n",
              message_->sched_info_arr_[0][0].concurrent_q.size_approx());
  std::printf("Create thread call\n");
  CreateThreads();
  std::printf("Done calling CreateThread\n");
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
    std::printf("Worker: %zu\n", i);
    workers_.emplace_back(&Worker::WorkerThread, this, i);
  }
  //   }

  if (kEnableMatLog) {
    for (size_t i = 0; i < mat_loggers_.size(); i++) {
      mat_loggers_.at(i) = std::make_shared<CsvLog::MatLogger>(i, "BS");
    }
  }
}

void Worker::WorkerThread(int tid) {
  PinToCoreWithOffset(ThreadType::kWorker, base_worker_core_offset_, tid);

  /* Initialize operators */
  auto compute_zf = std::make_unique<DoZF>(
      this->config_, tid, buffer_->csi_buffer_, buffer_->calib_dl_buffer_,
      buffer_->calib_ul_buffer_, buffer_->calib_dl_msum_buffer_,
      buffer_->calib_ul_msum_buffer_, buffer_->ul_zf_matrix_,
      buffer_->dl_zf_matrix_, phy_stats_, stats_,
      mat_loggers_.at(CsvLog::kDLCSI), mat_loggers_.at(CsvLog::kDLZF));

  auto compute_fft = std::make_unique<DoFFT>(
      config_, tid, buffer_->data_buffer_, buffer_->csi_buffer_,
      buffer_->calib_dl_buffer_, buffer_->calib_ul_buffer_, phy_stats_, stats_);

  // // Downlink workers
  auto compute_ifft =
      std::make_unique<DoIFFT>(config_, tid, buffer_->dl_ifft_buffer_,
                               buffer_->dl_socket_buffer_, stats_);

  auto compute_precode = std::make_unique<DoPrecode>(
      config_, tid, buffer_->dl_zf_matrix_, buffer_->dl_ifft_buffer_,
      buffer_->dl_mod_bits_buffer_, stats_);

  auto compute_encoding = std::make_unique<DoEncode>(
      config_, tid, Direction::kDownlink,
      (kEnableMac == true) ? buffer_->dl_bits_buffer_ : config_->DlBits(),
      (kEnableMac == true) ? kFrameWnd : 1, buffer_->dl_mod_bits_buffer_,
      stats_);

  // Uplink workers
  auto compute_decoding =
      std::make_unique<DoDecode>(config_, tid, buffer_->demod_buffer_,
                                 buffer_->decoded_buffer_, phy_stats_, stats_);

  auto compute_demul = std::make_unique<DoDemul>(
      config_, tid, buffer_->data_buffer_, buffer_->ul_zf_matrix_,
      buffer_->ue_spec_pilot_buffer_, buffer_->equal_buffer_,
      buffer_->demod_buffer_, phy_stats_, stats_);

  std::vector<Doer*> computers_vec;
  std::vector<EventType> events_vec;
  ///*************************
  computers_vec.push_back(compute_zf.get());
  computers_vec.push_back(compute_fft.get());
  events_vec.push_back(EventType::kZF);
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

  size_t cur_qid = 0;
  size_t empty_queue_itrs = 0;
  bool empty_queue = true;
  while (config_->Running() == true) {
    for (size_t i = 0; i < computers_vec.size(); i++) {
      if (computers_vec.at(i)->TryLaunch(
              *GetConq(message_->sched_info_arr_, events_vec.at(i), cur_qid),
              message_->complete_task_queue_[cur_qid],
              message_->worker_ptoks_ptr_[tid][cur_qid])) {
        empty_queue = false;
        break;
      }
    }
    // If all queues in this set are empty for 5 iterations,
    // check the other set of queues
    if (empty_queue == true) {
      empty_queue_itrs++;
      if (empty_queue_itrs == 5) {
        if (cur_sche_frame_id != cur_proc_frame_id) {
          cur_qid ^= 0x1;
        } else {
          cur_qid = (cur_sche_frame_id & 0x1);
        }
        empty_queue_itrs = 0;
      }
    } else {
      empty_queue = true;
    }
  }
  AGORA_LOG_SYMBOL("Agora worker %d exit\n", tid);
}