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
    /// The main event handling function that performs Doer-specific work.
    /// Doers that handle only one event type use this signature.
    virtual EventData Launch(size_t tag)
    {
        _unused(tag);
        rt_assert(false, "Doer: launch(tag) not implemented");
        return EventData();
    }

    /// The main event handling function that performs Doer-specific work.
    /// Doers that handle multiple event types use this signature.
    virtual EventData Launch(size_t tag, EventType event_type)
    {
        _unused(tag);
        _unused(event_type);
        rt_assert(false, "Doer: launch(tag, event_type) not implemented");
        return EventData();
    }

    void RegisterQueues(moodycamel::ConcurrentQueue<EventData>* task_queue,
        moodycamel::ConcurrentQueue<EventData>* complete_queue,
        moodycamel::ProducerToken* complete_queue_token)
    {
        task_queue_ = task_queue;
        complete_queue_ = complete_queue;
        complete_queue_token_ = complete_queue_token;
    }

protected:
    Doer(Config* in_config, int in_tid, double freq_ghz)
        : cfg_(in_config)
        , tid_(in_tid)
        , freq_ghz_(freq_ghz)
    {
    }

    virtual ~Doer() = default;

    Config* cfg_;
    int tid_; // Thread ID of this Doer
    double freq_ghz_; // RDTSC frequency in GHz

    // Centralized scheduler mode
    moodycamel::ConcurrentQueue<EventData>* task_queue_;
    moodycamel::ConcurrentQueue<EventData>* complete_queue_;
    moodycamel::ProducerToken* complete_queue_token_;
};
#endif /* DOER */
