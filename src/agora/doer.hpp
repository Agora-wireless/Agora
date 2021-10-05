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
    Doer(Config* in_config, int in_tid, double freq_ghz)
        : cfg(in_config)
        , tid(in_tid)
        , freq_ghz(freq_ghz)
    {
    }

    virtual ~Doer() = default;

    Config* cfg;
    int tid; // Thread ID of this Doer
    double freq_ghz; // RDTSC frequency in GHz
};
#endif /* DOER */
