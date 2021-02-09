#ifndef CONCURRENT_QUEUE_WRAPPER
#define CONCURRENT_QUEUE_WRAPPER

#include "buffer.inc"
#include "concurrentqueue.h"
#include "utils.h"

/// Enqueue one event to a concurrent queue and print a warning message
/// if we're short on queue space
static inline void try_enqueue_fallback(
    moodycamel::ConcurrentQueue<Event_data>* mc_queue,
    moodycamel::ProducerToken* producer_token, const Event_data& event)
{
    if (!mc_queue->try_enqueue(*producer_token, event)) {
        std::printf("Need more memory\n");
        rt_assert(mc_queue->enqueue(*producer_token, event),
            "Message enqueue failed");
    }
}

/// Enqueue one event to a concurrent queue and print a warning message
/// if we're short on queue space
static inline void try_enqueue_fallback(
    moodycamel::ConcurrentQueue<Event_data>* mc_queue, const Event_data& event)
{
    if (!mc_queue->try_enqueue(event)) {
        std::printf("Need more memory\n");
        rt_assert(mc_queue->enqueue(event), "Message enqueue failed");
    }
}

/// Enqueue a batch of events to a concurrent queue and print a warning message
/// if we're short on queue space
static inline void try_enqueue_bulk_fallback(
    moodycamel::ConcurrentQueue<Event_data>& mc_queue,
    moodycamel::ProducerToken& producer_token, const Event_data* event_list,
    size_t num_events)
{

    if (!mc_queue.try_enqueue_bulk(producer_token, event_list, num_events)) {
        std::printf("Need more memory\n");
        rt_assert(mc_queue.enqueue_bulk(producer_token, event_list, num_events),
            "Message bulk enqueue failed\n");
    }
}

#endif /* CONCURRENT_QUEUE_WRAPPER */
