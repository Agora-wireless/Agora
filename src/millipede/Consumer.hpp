#ifndef CONSUMER
#define CONSUMER

#include "buffer.hpp"
#include "concurrentqueue.h"

class Consumer {
public:
    Consumer(moodycamel::ConcurrentQueue<event_data_t>& out_queue,
        moodycamel::ProducerToken& out_token, int task_count = 0,
        EventType task_type = EventType::kInvalid);
    void handle(const event_data_t& event) const;
    void try_handle(const event_data_t& event) const;
    void schedule_task_set(int task_setid) const;

private:
    moodycamel::ConcurrentQueue<event_data_t>& out_queue_;
    moodycamel::ProducerToken& out_token_;
    int task_count;
    EventType task_type;
};

inline Consumer::Consumer(moodycamel::ConcurrentQueue<event_data_t>& out_queue,
    moodycamel::ProducerToken& out_token, int task_count, EventType task_type)
    : out_queue_(out_queue)
    , out_token_(out_token)
    , task_count(task_count)
    , task_type(task_type)
{
}

inline void Consumer::handle(const event_data_t& event) const
{
    if (!out_queue_.enqueue(out_token_, event)) {
        printf("message enqueue failed\n");
        exit(0);
    }
}

inline void Consumer::try_handle(const event_data_t& event) const
{
    if (!out_queue_.try_enqueue(out_token_, event)) {
        printf("need more memory\n");
        handle(event);
    }
}

inline void Consumer::schedule_task_set(int task_setid) const
{
    event_data_t do_task(task_type, task_setid * task_count);
    for (int i = 0; i < task_count; i++) {
        try_handle(do_task);
        do_task.data++;
    }
}

#endif /* CONSUMER */
