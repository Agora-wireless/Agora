#ifndef CONSUMER
#define CONSUMER

#include "concurrentqueue.h"

class Consumer {
public:
    Consumer(moodycamel::ConcurrentQueue<Event_data>& out_queue,
        moodycamel::ProducerToken& out_token);
    void handle(const Event_data& event) const;
    void try_handle(const Event_data& event) const;

private:
    moodycamel::ConcurrentQueue<Event_data>& out_queue_;
    moodycamel::ProducerToken& out_token_;
};

inline Consumer::Consumer(moodycamel::ConcurrentQueue<Event_data>& out_queue,
    moodycamel::ProducerToken& out_token)
    : out_queue_(out_queue)
    , out_token_(out_token)
{
}

inline void
Consumer::handle(const Event_data& event) const
{
    if (!out_queue_.enqueue(out_token_, event)) {
        printf("message enqueue failed\n");
        exit(0);
    }
}

inline void
Consumer::try_handle(const Event_data& event) const
{
    if (!out_queue_.try_enqueue(out_token_, event)) {
        printf("need more memory\n");
        handle(event);
    }
}

#endif /* CONSUMER */
