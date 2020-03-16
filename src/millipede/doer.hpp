#ifndef DOER
#define DOER

class Config;
class Consumer;
#include "concurrentqueue.h"

class Doer {
public:
    virtual bool try_launch(void)
    {
        Event_data event;
        if (task_queue_.try_dequeue(event)) {
            launch(event.data);
            return true;
        }
        return false;
    }

protected:
    virtual void launch(int offset) = 0;
    Doer(Config* in_config, int in_tid,
        moodycamel::ConcurrentQueue<Event_data>& in_task_queue,
        Consumer& in_consumer)
        : config_(in_config)
        , tid(in_tid)
        , task_queue_(in_task_queue)
        , consumer_(in_consumer)
    {
    }

    Config* config_;
    int tid;
    moodycamel::ConcurrentQueue<Event_data>& task_queue_;
    Consumer& consumer_;
};
#endif /* DOER */
