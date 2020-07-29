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
    virtual bool try_launch(void)
    {
        Event_data req_event;
        if (task_queue_.try_dequeue(req_event)) {
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

            try_enqueue_fallback(
                &complete_task_queue, worker_producer_token, resp_event);
            return true;
        }
        return false;
    }
    virtual Event_data launch(size_t tag) = 0;

protected:
    Doer(Config* in_config, int in_tid, double freq_ghz,
        moodycamel::ConcurrentQueue<Event_data>& in_task_queue,
        moodycamel::ConcurrentQueue<Event_data>& complete_task_queue,
        moodycamel::ProducerToken* worker_producer_token)
        : cfg(in_config)
        , tid(in_tid)
        , freq_ghz(freq_ghz)
        , task_queue_(in_task_queue)
        , complete_task_queue(complete_task_queue)
        , worker_producer_token(worker_producer_token)
    {
    }

    Config* cfg;
    int tid; // Thread ID of this Doer
    double freq_ghz; // RDTSC frequency in GHz
    moodycamel::ConcurrentQueue<Event_data>& task_queue_;
    moodycamel::ConcurrentQueue<Event_data>& complete_task_queue;
    moodycamel::ProducerToken* worker_producer_token;
};
#endif /* DOER */
