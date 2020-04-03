#ifndef DOER
#define DOER

class Config;
#include "buffer.hpp"
#include "concurrent_queue_wrapper.hpp"
#include "concurrentqueue.h"

class Doer {
public:
    virtual bool try_launch(void)
    {
        Event_data event;
        if (task_queue_.try_dequeue(event)) {
            if (event.num_offsets == 0) {
                Event_data finish_event = launch(event.data);
                try_enqueue_fallback(
                    complete_task_queue, *worker_producer_token, finish_event);
            } else {
                Event_data finish_event;
                Event_data temp_event;
                finish_event.num_offsets = event.num_offsets;
                for (int i = 0; i < event.num_offsets; i++) {
                    temp_event = launch(event.offsets[i]);
                    finish_event.offsets[i] = temp_event.data;
                }
                finish_event.event_type = temp_event.event_type;
                try_enqueue_fallback(
                    complete_task_queue, *worker_producer_token, finish_event);
            }
            return true;
        }
        return false;
    }

protected:
    virtual Event_data launch(int offset) = 0;
    Doer(Config* in_config, int in_tid,
        moodycamel::ConcurrentQueue<Event_data>& in_task_queue,
        moodycamel::ConcurrentQueue<Event_data>& complete_task_queue,
        moodycamel::ProducerToken* worker_producer_token)
        : cfg(in_config)
        , tid(in_tid)
        , task_queue_(in_task_queue)
        , complete_task_queue(complete_task_queue)
        , worker_producer_token(worker_producer_token)
    {
    }

    Config* cfg;
    int tid;
    moodycamel::ConcurrentQueue<Event_data>& task_queue_;
    moodycamel::ConcurrentQueue<Event_data>& complete_task_queue;
    moodycamel::ProducerToken* worker_producer_token;
};
#endif /* DOER */
