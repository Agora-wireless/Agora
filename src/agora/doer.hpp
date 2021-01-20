#ifndef DOER
#define DOER

class Config;
#include "buffer.hpp"
#include "concurrent_queue_wrapper.hpp"
#include "concurrentqueue.h"
#include "logger.h"
#include "stats.hpp"

class Doer {
 public:
  virtual bool try_launch(
      moodycamel::ConcurrentQueue<EventData>& task_queue,
      moodycamel::ConcurrentQueue<EventData>& complete_task_queue,
      moodycamel::ProducerToken* worker_ptok) {
    EventData req_event;
    if (task_queue.try_dequeue(req_event)) {
      // We will enqueue one response event containing results for all
      // request tags in the request event
      EventData resp_event;
      resp_event.num_tags_ = req_event.num_tags_;

      for (size_t i = 0; i < req_event.num_tags_; i++) {
        EventData resp_i = launch(req_event.tags_[i]);
        rt_assert(resp_i.num_tags_ == 1, "Invalid num_tags in resp");
        resp_event.tags_[i] = resp_i.tags_[0];
        resp_event.event_type_ = resp_i.event_type_;
      }

      try_enqueue_fallback(&complete_task_queue, worker_ptok, resp_event);
      return true;
    }
    return false;
  }

  /// The main event handling function that performs Doer-specific work.
  /// Doers that handle only one event type use this signature.
  virtual EventData launch(size_t tag) {
    _unused(tag);
    rt_assert(false, "Doer: launch(tag) not implemented");
    return EventData();
  }

  /// The main event handling function that performs Doer-specific work.
  /// Doers that handle multiple event types use this signature.
  virtual EventData launch(size_t tag, EventType event_type) {
    _unused(tag);
    _unused(event_type);
    rt_assert(false, "Doer: launch(tag, event_type) not implemented");
    return EventData();
  }

 protected:
  Doer(Config* in_config, int in_tid) : cfg_(in_config), tid_(in_tid) {}

  virtual ~Doer() = default;

  Config* cfg_;
  int tid_;  // Thread ID of this Doer
};
#endif /* DOER */
