/**
 * @file message_box_thread.cc
 * @brief Defination file for MessageBoxThread class
 */
#include "message_box_thread.h"

MessageBoxThread::MessageBoxThread(int core_id, size_t thread_id,
                                   size_t queue_size)
    : event_queue_(queue_size),
      producer_token_(event_queue_),
      id_(thread_id),
      core_alloc_(core_id) {}

MessageBoxThread::~MessageBoxThread() { Finalize(); }

// Launching thread in seperate function to guarantee that the object is fully
// constructed before calling member function
void MessageBoxThread::Start() {
  AGORA_LOG_INFO("Launching thread with id: %zu and core %d\n", id_,
                 core_alloc_);
  thread_ = std::thread(&MessageBoxThread::EventLoop, this);
}

/* Cleanly allows the thread to exit */
void MessageBoxThread::Stop() {
  EventData event;
  event.event_type_ = EventType::kThreadTermination;
  DispatchWork(event);
}

void MessageBoxThread::Finalize() {
  // Wait for thread to cleanly finish the messages in the queue
  if (thread_.joinable() == true) {
    AGORA_LOG_INFO("Joining thread with id: %zu and core %d\n", id_,
                   core_alloc_);
    Stop();
    thread_.join();
  }
}

// Returns true for success, false otherwise
bool MessageBoxThread::DispatchWork(const EventData& event) {
  AGORA_LOG_TRACE("MessageBoxThread::Dispatching work\n");
  bool ret = true;
  if (event_queue_.try_enqueue(producer_token_, event) == false) {
    AGORA_LOG_WARN(
        "MessageBoxThread[%zu]: Queue limit has reached! try to increase queue "
        "size.\n",
        tid_);
    if (event_queue_.enqueue(producer_token_, event) == false) {
      AGORA_LOG_ERROR("MessageBoxThread[%zu]: task enqueue failed\n", tid_);
      throw std::runtime_error("MessageBoxThread task enqueue failed");
      ret = false;
    }
  }
  return ret;
}

void MessageBoxThread::EventLoop() {
  if (core_alloc_ >= 0) {
    AGORA_LOG_INFO("MessageBoxThread[%zu]: assigning thread %zu to core %d\n",
                   id_, core_alloc_);
    PinToCoreWithOffset(ThreadType::kWorker, core_alloc_, id_, true);
  }
  running_ = true;
  moodycamel::ConsumerToken ctok(event_queue_);

  EventData event;
  while (running_) {
    /// \todo Modify to bulk dequeue when time allows
    auto ret = event_queue_.try_dequeue(ctok, event);

    if (ret == false) /* Queue empty */
    {
      //if (wait_signal_ == true) {
      //  std::unique_lock<std::mutex> thread_wait(sync_);
      //  /* Wait until a new message exists, should eliminate the CPU polling */
      //  condition_.wait(thread_wait, [this, &ctok, &event] {
      //    return event_queue_.try_dequeue(ctok, event);
      //  });
      //  ret = true;
    } else {
      HandleEvent(event);
    }
  }

  for (auto& worker : workers_) {
    worker->Finalize();
  }
  running_ = false;
}