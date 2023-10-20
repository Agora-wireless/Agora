/**
 * @file doer.h
 * @brief Declaration file for the Doer class.  The is the base class for all
 * agora doers
 */
#ifndef DOER_H_
#define DOER_H_

#include <cstddef>
#include <queue>

#include "concurrent_queue_wrapper.h"
#include "concurrentqueue.h"
#include "config.h"
#include "message.h"
#include "utils.h"

class Doer {
 public:
  virtual bool TryLaunch(
      std::queue<EventData>& task_queue,
      std::queue<EventData>& complete_task_queue) {
    EventData req_event;

    ///Each event is handled by 1 Doer(Thread) and each tag is processed sequentually
    if (!task_queue.empty()) { // TODO: remove try launch but directly launch
      req_event = task_queue.front();
      task_queue.pop();
      // We will enqueue one response event containing results for all
      // request tags in the request event
      EventData resp_event;
      resp_event.num_tags_ = req_event.num_tags_;
      resp_event.event_type_ = req_event.event_type_;

      for (size_t i = 0; i < req_event.num_tags_; i++) {
        EventData doer_comp = Launch(req_event.tags_.at(i));
        RtAssert(doer_comp.num_tags_ == 1, "Invalid num_tags in resp");
        resp_event.tags_.at(i) = doer_comp.tags_.at(0);
        RtAssert(resp_event.event_type_ == doer_comp.event_type_,
                 "Invalid event type in resp");
      }

      complete_task_queue.push(resp_event);
      return true;
    }
    return false;
  }

  /// The main event handling function that performs Doer-specific work.
  /// Doers that handle only one event type use this signature.
  virtual EventData Launch(size_t tag) {
    unused(tag);
    RtAssert(false, "Doer: Launch(tag) not implemented");
    return EventData();
  }

  /// The main event handling function that performs Doer-specific work.
  /// Doers that handle multiple event types use this signature.
  virtual EventData Launch(size_t tag, EventType event_type) {
    unused(tag);
    unused(event_type);
    RtAssert(false, "Doer: Launch(tag, event_type) not implemented");
    return EventData();
  }

 protected:
  Doer(Config* in_config, int in_tid) : cfg_(in_config), tid_(in_tid) {}
  virtual ~Doer() = default;

  Config* cfg_;
  int tid_;  // Thread ID of this Doer
};
#endif  // DOER_H_
