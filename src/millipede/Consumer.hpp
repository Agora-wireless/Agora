#ifndef CONSUMER
#define CONSUMER

#include "concurrentqueue.h"

class Consumer {
public:
    Consumer(moodycamel::ConcurrentQueue<Event_data>& out_queue,
        moodycamel::ProducerToken& out_token, int task_count = 0,
        int task_tag = 0);
    void handle(const Event_data& event) const;
    void try_handle(const Event_data& event) const;
    void schedule_task_set(int task_setid) const;

private:
    moodycamel::ConcurrentQueue<Event_data>& out_queue_;
    moodycamel::ProducerToken& out_token_;
    int task_count_;
    int task_tag_;
};

inline Consumer::Consumer(moodycamel::ConcurrentQueue<Event_data>& out_queue,
    moodycamel::ProducerToken& out_token, int task_count, int task_tag)
    : out_queue_(out_queue)
    , out_token_(out_token)
    , task_count_(task_count)
    , task_tag_(task_tag)
{
}

inline void Consumer::handle(const Event_data& event) const
{
    if (!out_queue_.enqueue(out_token_, event)) {
        printf("message enqueue failed\n");
        exit(0);
    }
}

inline void Consumer::try_handle(const Event_data& event) const
{
    if (!out_queue_.try_enqueue(out_token_, event)) {
        printf("need more memory\n");
        handle(event);
    }
}

inline void Consumer::schedule_task_set(int task_setid) const
{
    Event_data do_task;
    do_task.event_type = task_tag_;
    do_task.data = task_setid * task_count_;
    for (int i = 0; i < task_count_; i++) {
        try_handle(do_task);
        do_task.data++;
    }
}

#endif /* CONSUMER */
