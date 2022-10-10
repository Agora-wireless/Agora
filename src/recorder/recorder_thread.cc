/*
 Copyright (c) 2018-2020, Rice University
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

---------------------------------------------------------------------
 Event based message queue thread class for the recorder worker
---------------------------------------------------------------------
*/

#include "recorder_thread.h"

#include "logger.h"
#include "message.h"
#include "utils.h"

namespace Agora_recorder {
RecorderThread::RecorderThread(
    const Config* in_cfg, size_t thread_id, int core, size_t queue_size,
    size_t antenna_offset, size_t num_antennas, size_t interval,
    Direction rx_direction,
    const std::vector<RecorderWorker::RecorderWorkerTypes>& types,
    bool wait_signal)
    : event_queue_(queue_size),
      producer_token_(event_queue_),
      id_(thread_id),
      core_alloc_(core),
      wait_signal_(wait_signal) {
  /// Create Workers
  for (const auto& worker_type : types) {
    workers_.emplace_back(RecorderWorker::Create(worker_type, in_cfg,
                                                 antenna_offset, num_antennas,
                                                 interval, rx_direction));
  }

  for (auto& worker : workers_) {
    worker->Init();
  }
  running_ = false;
}

RecorderThread::~RecorderThread() { Finalize(); }

// Launching thread in seperate function to guarantee that the object is fully
// constructed before calling member function
void RecorderThread::Start() {
  AGORA_LOG_INFO("Launching recorder task thread with id: %zu and core %d\n",
                 id_, core_alloc_);
  {
    std::lock_guard<std::mutex> thread_lock(sync_);
    thread_ = std::thread(&RecorderThread::DoRecording, this);
    running_ = true;
  }
  condition_.notify_all();
}

/* Cleanly allows the thread to exit */
void RecorderThread::Stop() {
  EventData event;
  event.event_type_ = EventType::kThreadTermination;
  DispatchWork(event);
}

void RecorderThread::Finalize() {
  // Wait for thread to cleanly finish the messages in the queue
  if (thread_.joinable() == true) {
    AGORA_LOG_INFO("Joining Recorder Thread on CPU %d \n", sched_getcpu());
    Stop();
    thread_.join();
  }
}

// Returns true for success, false otherwise
bool RecorderThread::DispatchWork(const EventData& event) {
  AGORA_LOG_TRACE("RecorderThread::Dispatching work\n");
  bool ret = true;
  if (event_queue_.try_enqueue(producer_token_, event) == false) {
    AGORA_LOG_WARN("Queue limit has reached! try to increase queue size.\n");
    if (event_queue_.enqueue(producer_token_, event) == false) {
      AGORA_LOG_ERROR("Record task enqueue failed\n");
      throw std::runtime_error("Record task enqueue failed");
      ret = false;
    }
  }

  if (wait_signal_ == true) {
    if (ret == true) {
      std::lock_guard<std::mutex> thread_lock(sync_);
    }
    condition_.notify_all();
  }
  return ret;
}

void RecorderThread::DoRecording() {
  // Sync the start
  {
    std::unique_lock<std::mutex> thread_wait(sync_);
    condition_.wait(thread_wait, [this] { return running_; });
  }

  if (core_alloc_ >= 0) {
    AGORA_LOG_INFO("Pinning recording thread %zu to core %d\n", id_,
                   core_alloc_);
    PinToCoreWithOffset(ThreadType::kRecorderWorker, core_alloc_, id_, true);
  }

  moodycamel::ConsumerToken ctok(event_queue_);
  AGORA_LOG_INFO(
      "Recording thread[%zu], writter count %zu, has %zu antennas starting at "
      "%zu\n",
      id_, workers_.size(), workers_.at(0)->NumAntennas(),
      workers_.at(0)->AntennaOffset());

  EventData event;
  while (running_) {
    auto ret = event_queue_.try_dequeue(ctok, event);

    if (ret == false) /* Queue empty */
    {
      if (wait_signal_ == true) {
        std::unique_lock<std::mutex> thread_wait(sync_);
        /* Wait until a new message exists, should eliminate the CPU polling */
        condition_.wait(thread_wait, [this, &ctok, &event] {
          return event_queue_.try_dequeue(ctok, event);
        });
        /* return from here with a valid event to process */
        ret = true;
      }
    }

    if (ret == true) {
      HandleEvent(event);
    }
  }

  for (auto& worker : workers_) {
    worker->Finalize();
  }
}

void RecorderThread::HandleEvent(const EventData& event) {
  if (event.event_type_ == EventType::kThreadTermination) {
    running_ = false;
  } else {
    auto* rx_packet = rx_tag_t(event.tags_[0u]).rx_packet_;
    if (event.event_type_ == EventType::kPacketRX) {
      for (auto& worker : workers_) {
        worker->Record(rx_packet->RawPacket());
      }
    }
    rx_packet->Free();
  }
}
};  // End namespace Agora_recorder