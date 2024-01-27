/**
 * @file doer.h
 * @brief Declaration file for the Doer class.  The is the base class for all
 * agora doers
 */
#ifndef DOER_H_
#define DOER_H_

#include <cstddef>

#include "concurrent_queue_wrapper.h"
#include "concurrentqueue.h"
#include "gettime.h"
#include "config.h"
#include "gettime.h"
#include "message.h"
#include "utils.h"

class Doer {
 public:
  virtual bool TryLaunch(
      moodycamel::ConcurrentQueue<EventData>& task_queue,
      moodycamel::ConcurrentQueue<EventData>& complete_task_queue,
      moodycamel::ProducerToken* worker_ptok) {
    EventData req_event;

    enq_deq_tsc_worker_.dequeue_start_tsc_ = GetTime::WorkerRdtsc();
    auto ret = task_queue.try_dequeue(req_event);
    enq_deq_tsc_worker_.dequeue_end_tsc_ = GetTime::WorkerRdtsc();
    enq_deq_tsc_worker_.dequeue_diff_tsc_ =
        enq_deq_tsc_worker_.dequeue_end_tsc_ -
        enq_deq_tsc_worker_.dequeue_start_tsc_;
    /// Each event is handled by 1 Doer(Thread) and each tag is processed sequentually
    if (ret) {
      // We will enqueue one response event containing results for all
      // request tags in the request event
      EventData resp_event;
      resp_event.num_tags_ = req_event.num_tags_;
      resp_event.event_type_ = req_event.event_type_;

      for (size_t i = 0; i < req_event.num_tags_; i++) {
        if (req_event.event_type_ == EventType::kFFT) {
          Packet* pkt =
              fft_req_tag_t(req_event.tags_.at(i)).rx_packet_->RawPacket();
          enq_deq_tsc_worker_.frame_id_ = pkt->frame_id_;
          enq_deq_tsc_worker_.symbol_id_ = pkt->symbol_id_;
        } else {
          // CAUTION: Proper value in symbol_id_ below is subject to the type of req_event
          enq_deq_tsc_worker_.frame_id_ =
              gen_tag_t(req_event.tags_.at(i)).frame_id_;
          enq_deq_tsc_worker_.symbol_id_ =
              gen_tag_t(req_event.tags_.at(i)).symbol_id_;
        }
        EventData doer_comp = Launch(req_event.tags_.at(i));
        RtAssert(doer_comp.num_tags_ == 1, "Invalid num_tags in resp");
        resp_event.tags_.at(i) = doer_comp.tags_.at(0);
        RtAssert(resp_event.event_type_ == doer_comp.event_type_,
                 "Invalid event type in resp");
      }
      enq_deq_tsc_worker_.enqueue_start_tsc_ = GetTime::WorkerRdtsc();
      TryEnqueueFallback(&complete_task_queue, worker_ptok, resp_event);
      enq_deq_tsc_worker_.enqueue_end_tsc_ = GetTime::WorkerRdtsc();
      enq_deq_tsc_worker_.enqueue_diff_tsc_ =
          enq_deq_tsc_worker_.enqueue_end_tsc_ -
          enq_deq_tsc_worker_.enqueue_start_tsc_;
      enq_deq_tsc_worker_.valid_dequeue_diff_tsc_ =
          enq_deq_tsc_worker_.dequeue_diff_tsc_;
      return true;
    }
    return false;
  }

  /// The main event handling function that performs Doer-specific work.
  /// Doers that handle only one event type use this signature.
  virtual EventData Launch(size_t tag) {
    unused(tag);
    RtAssert(false, "Doer: Launch(tag) not implemented");
    return {};
  }

  /// The main event handling function that performs Doer-specific work.
  /// Doers that handle multiple event types use this signature.
  virtual EventData Launch(size_t tag, EventType event_type) {
    unused(tag);
    unused(event_type);
    RtAssert(false, "Doer: Launch(tag, event_type) not implemented");
    return {};
  }

  struct {
    size_t frame_id_ = 0;
    size_t symbol_id_ = 0;
    size_t dequeue_start_tsc_ = 0;
    size_t dequeue_end_tsc_ = 0;
    size_t dequeue_diff_tsc_ = 0;
    size_t valid_dequeue_diff_tsc_ = 0;
    size_t enqueue_start_tsc_ = 0;
    size_t enqueue_end_tsc_ = 0;
    size_t enqueue_diff_tsc_ = 0;
  } enq_deq_tsc_worker_;

 protected:
  Doer(Config* in_config, int in_tid) : cfg_(in_config), tid_(in_tid) {}
  virtual ~Doer() = default;

  Config* cfg_;
  int tid_;  // Thread ID of this Doer
};
#endif  // DOER_H_
