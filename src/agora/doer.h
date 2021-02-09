#ifndef DOER
#define DOER

class Config;
#include "buffer.inc"
#include "concurrent_queue_wrapper.inc"
#include "concurrentqueue.h"
#include "logger.inc"
#include "stats.h"

class Doer {
public:
    virtual bool try_launch(moodycamel::ConcurrentQueue<Event_data>& task_queue,
        moodycamel::ConcurrentQueue<Event_data>& complete_task_queue,
        moodycamel::ProducerToken* worker_ptok)
    {
        Event_data req_event;
        if (task_queue.try_dequeue(req_event)) {
            // We will enqueue one response event containing results for all
            // request tags in the request event
            Event_data resp_event;
            resp_event.num_tags = req_event.num_tags;

            for (size_t i = 0; i < req_event.num_tags; i++) {
                Event_data resp_i = launch(req_event.tags[i]);
                rt_assert(resp_i.num_tags == 1, "Invalid num_tags in resp");
                resp_event.tags[i] = resp_i.tags[0];
                resp_event.event_type = resp_i.event_type;
            }

            try_enqueue_fallback(&complete_task_queue, worker_ptok, resp_event);
            return true;
        }
        return false;
    }

    /// The main event handling function that performs Doer-specific work.
    /// Doers that handle only one event type use this signature.
    virtual Event_data launch(size_t tag)
    {
        _unused(tag);
        rt_assert(false, "Doer: launch(tag) not implemented");
        return Event_data();
    }

    /// The main event handling function that performs Doer-specific work.
    /// Doers that handle multiple event types use this signature.
    virtual Event_data launch(size_t tag, EventType event_type)
    {
        _unused(tag);
        _unused(event_type);
        rt_assert(false, "Doer: launch(tag, event_type) not implemented");
        return Event_data();
    }

protected:
    Doer(Config* in_config, int in_tid)
        : cfg(in_config)
        , tid(in_tid)
    {
    }

    virtual ~Doer() = default;

    Config* cfg;
    int tid; // Thread ID of this Doer
};
#endif /* DOER */
