/*
 Copyright (c) 2018-2020, Rice University
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

---------------------------------------------------------------------
 Event based message queue thread class for the recorder worker
---------------------------------------------------------------------
*/

#include "recorder_thread.h"
#include "logger.h"
#include "utils.h"
#include "H5Cpp.h"

namespace Recorder {
RecorderThread::RecorderThread(Config* in_cfg,
                                std::vector<RecorderWorkerFactory *> &factories,
                                H5::H5File *h5_file,
                                size_t thread_id, int core,
                                size_t queue_size, size_t antenna_offset,
                                size_t num_antennas, bool wait_signal)
    : event_queue_(queue_size),
      producer_token_(event_queue_),
      thread_(),
      id_(thread_id),
      core_alloc_(core),
      wait_signal_(wait_signal),
      antenna_offset_(antenna_offset),
      num_antennas_(num_antennas) {
  packet_length_ = in_cfg->PacketLength();

  for(auto &rec_fact: factories) {
    RecorderWorker *worker = rec_fact->GenWorker(in_cfg, h5_file);
    worker_mapping_.insert(std::pair<EventType, RecorderWorker *>
                          (worker->GetEventType(), worker));
  }

  running_.store(true);
  thread_ = std::thread(&RecorderThread::DoRecording, this);
}

RecorderThread::~RecorderThread() {
  // Wait for thread to cleanly finish the messages in the queue
  if (thread_.joinable() == true) {
    EventData event;
    /* Empty event will fail key lookup in worker_mapping_ */
    this->DispatchWork(event);
    this->thread_.join();
  }

  // Free all workers
  for(auto itr: worker_mapping_) {
    delete itr.second;
  }
}

/* TODO:  handle producer token better */
// Returns true for success, false otherwise
bool RecorderThread::DispatchWork(const EventData& event) {
  // MLPD_TRACE("Dispatching work\n");
  bool ret = true;
  if (event_queue_.try_enqueue(producer_token_, event) == 0) {
    MLPD_WARN("Queue limit has reached! try to increase queue size.\n");
    if (event_queue_.enqueue(producer_token_, event) == 0) {
      MLPD_ERROR("Record task enqueue failed\n");
      // throw std::runtime_error("Record task enqueue failed");
      ret = false;
    }
  }

  if(this->wait_signal_ == true) {
    std::lock_guard guard(sync_);
    this->condition_.notify_all();
  }
  return ret;
}

void RecorderThread::DoRecording() {
  if (core_alloc_ >= 0) {
    MLPD_INFO("Pinning recording thread %zu to core %d\n", id_, core_alloc_);
    PinToCoreWithOffset(ThreadType::kMaster, core_alloc_, id_, true);
  }

  moodycamel::ConsumerToken ctok(event_queue_);

  EventData event;
  bool ret = false;
  while (running_.load() == true) {
    ret = event_queue_.try_dequeue(ctok, event);

    if (ret == false) /* Queue empty */ {
      if(wait_signal_ == true) {
        std::unique_lock<std::mutex> thread_wait(sync_);
        // Wait until a new message exists.
        // TODO: should eliminate the CPU polling
        condition_.wait(thread_wait, [this, &ctok, &event] {
          return this->event_queue_.try_dequeue(ctok, event);
        });
        /* return from here with a valid event to process */
        ret = true;
      }
    }

    if (ret == true) {
      HandleEvent(event);
    }
  }
}

void RecorderThread::HandleEvent(const EventData& event) {
  auto itr = worker_mapping_.find(event.event_type_);
  if(itr == worker_mapping_.end()) {
    running_.store(false);
  } else {
    itr->second->Record(
        static_cast<void *>(rx_tag_t(event.tags_[0]).rx_packet_->RawPacket()));

    rx_tag_t(event.tags_[0]).rx_packet_->Free();
  }
}
};  // End namespace Recorder