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

namespace Agora_recorder {
RecorderThread::RecorderThread(Config* in_cfg, H5::H5File *h5_file,
                              size_t thread_id, int core,
                              size_t queue_size, size_t antenna_offset,
                              size_t num_antennas, bool wait_signal)
    : event_queue_(queue_size),
      producer_token_(event_queue_),
      thread_(),
      rx_record_(in_cfg, h5_file),
      id_(thread_id),
      core_alloc_(core),
      wait_signal_(wait_signal),
      antenna_offset_(antenna_offset),
      num_antennas_(num_antennas) {
  packet_length_ = in_cfg->PacketLength();
  running_ = false;
}

RecorderThread::~RecorderThread() { Finalize(); }

// Launching thread in separate function to guarantee that the object is fully
// constructed before calling member function
void RecorderThread::Start() {
  MLPD_INFO("Launching recorder task thread with id: %zu and core %d\n",
            this->id_, this->core_alloc_);
  {
    std::lock_guard<std::mutex> thread_lock(this->sync_);
    this->thread_ = std::thread(&RecorderThread::DoRecording, this);
    this->running_ = true;
  }
  this->condition_.notify_all();
}

/* Cleanly allows the thread to exit */
void RecorderThread::Stop() {
  EventData event;
  /* Empty event will fail key lookup in worker_mapping_ */
  this->DispatchWork(event);
}

void RecorderThread::Finalize() {
  // Wait for thread to cleanly finish the messages in the queue
  if (this->thread_.joinable() == true) {
    MLPD_TRACE("Joining Recorder Thread on CPU %d \n", sched_getcpu());
    this->Stop();
    this->thread_.join();
  }
}

/* TODO:  handle producer token better */
// Returns true for success, false otherwise
bool RecorderThread::DispatchWork(const EventData& event) {
  // MLPD_TRACE("Dispatching work\n");
  bool ret = true;
  if (this->event_queue_.try_enqueue(this->producer_token_, event) == 0) {
    MLPD_WARN("Queue limit has reached! try to increase queue size.\n");
    if (this->event_queue_.enqueue(this->producer_token_, event) == 0) {
      MLPD_ERROR("Record task enqueue failed\n");
      throw std::runtime_error("Record task enqueue failed");
      ret = false;
    }
  }

  if (this->wait_signal_ == true) {
    if (ret == true) {
      std::lock_guard<std::mutex> thread_lock(this->sync_);
    }
    this->condition_.notify_all();
  }
  return ret;
}

void RecorderThread::DoRecording() {
  // Sync the start
  {
    std::unique_lock<std::mutex> thread_wait(this->sync_);
    this->condition_.wait(thread_wait, [this] { return this->running_; });
  }

  if (this->core_alloc_ >= 0) {
    MLPD_INFO("Pinning recording thread %zu to core %d\n", this->id_,
              this->core_alloc_);
    PinToCoreWithOffset(ThreadType::kMaster, this->core_alloc_, this->id_,
                        true);
  }

  moodycamel::ConsumerToken ctok(this->event_queue_);
  MLPD_INFO("Recording thread %zu has %zu antennas starting at %zu\n",
            this->id_, this->num_antennas_,
            this->antenna_offset_);

  EventData event;
  bool ret = false;
  while (this->running_ == true) {
    ret = this->event_queue_.try_dequeue(ctok, event);

    if (ret == false) /* Queue empty */
    {
      if (this->wait_signal_ == true) {
        std::unique_lock<std::mutex> thread_wait(this->sync_);
        // Wait until a new message exists.
        // TODO: should eliminate the CPU polling
        this->condition_.wait(thread_wait, [this, &ctok, &event] {
          return this->event_queue_.try_dequeue(ctok, event);
        });
        /* return from here with a valid event to process */
        ret = true;
      }
    }

    if (ret == true) {
      this->HandleEvent(event);
    }
  }
}

void RecorderThread::HandleEvent(const RecordEventData& event) {
  if (event.event_type_ == kThreadTermination) {
    this->running_ = false;
  } else {
    if (event.event_type_ == kTaskRecordRx) {
      this->rx_record_.Record(
          this->id_,
          rx_tag_t(event.record_event_.tags_[0]).rx_packet_->RawPacket());
    }

    /* Free up the buffer memory */
    rx_tag_t(event.record_event_.tags_[0]).rx_packet_->Free();
  }
}
};  // End namespace Agora_recorder