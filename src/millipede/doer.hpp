#ifndef DOER
#define DOER

class Config;
#include "buffer.hpp"
#include "concurrent_queue_wrapper.hpp"
#include "concurrentqueue.h"
#include "stats.hpp"

class Doer {
public:
    virtual bool try_launch(void)
    {
        Event_data event;
        if (task_queue_.try_dequeue(event)) {
            for (size_t i = 0; i < event.num_tags; i++) {
                Event_data finish_event = launch(event.tags[i]);
                try_enqueue_fallback(
                    complete_task_queue, *worker_producer_token, finish_event);
            }
            return true;
        }
        return false;
    }

protected:
    virtual Event_data launch(int tag) = 0;
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
