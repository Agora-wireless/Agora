#ifndef CONCURRENT_QUEUE_WRAPPER
#define CONCURRENT_QUEUE_WRAPPER

#include "buffer.hpp"
#include "concurrentqueue.h"
#include "utils.h"

class ConcurrentQueueWrapper {
public:
    ConcurrentQueueWrapper(moodycamel::ConcurrentQueue<Event_data>& out_queue,
        moodycamel::ProducerToken& out_token, int task_count = 0,
        EventType task_type = EventType::kInvalid);
    void try_enqueue(const Event_data& event) const;
    void schedule_task_set(int task_setid) const;

private:
    moodycamel::ConcurrentQueue<Event_data>& out_queue_;
    moodycamel::ProducerToken& out_token_;
    int task_count;
    EventType task_type;
};

inline ConcurrentQueueWrapper::ConcurrentQueueWrapper(
    moodycamel::ConcurrentQueue<Event_data>& out_queue,
    moodycamel::ProducerToken& out_token, int task_count, EventType task_type)
    : out_queue_(out_queue)
    , out_token_(out_token)
    , task_count(task_count)
    , task_type(task_type)
{
}

inline void ConcurrentQueueWrapper::try_enqueue(const Event_data& event) const
{
    if (!out_queue_.try_enqueue(out_token_, event)) {
        printf("Need more memory\n");
        rt_assert(
            out_queue_.enqueue(out_token_, event), "Message enqueue failed");
    }
}

inline void ConcurrentQueueWrapper::schedule_task_set(int task_setid) const
{
    Event_data event_list[task_count];
    for (int i = 0; i < task_count; i++) {
        event_list[i].event_type = task_type;
        event_list[i].data = task_setid * task_count + i;
    }

    if (!out_queue_.try_enqueue_bulk(out_token_, event_list, task_count)) {
        printf("Need more memory\n");
        rt_assert(out_queue_.enqueue_bulk(out_token_, event_list, task_count),
            "Message bulk enqueue failed\n");
    }
}

#endif /* CONCURRENT_QUEUE_WRAPPER */
