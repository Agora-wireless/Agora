#ifndef CONCURRENT_QUEUE_WRAPPER
#define CONCURRENT_QUEUE_WRAPPER

#include "buffer.h"
#include "concurrentqueue.h"
#include "utils.h"

/// Enqueue one event to a concurrent queue and print a warning message
/// if we're short on queue space
static inline void TryEnqueueFallback(
    moodycamel::ConcurrentQueue<EventData>* mc_queue,
    moodycamel::ProducerToken* producer_token, const EventData& event) {
  if (!mc_queue->try_enqueue(*producer_token, event)) {
    std::printf("Need more memory\n");
    RtAssert(mc_queue->enqueue(*producer_token, event),
             "Message enqueue failed");
  }
}

/// Enqueue one event to a concurrent queue and print a warning message
/// if we're short on queue space
static inline void TryEnqueueFallback(
    moodycamel::ConcurrentQueue<EventData>* mc_queue, const EventData& event) {
  if (!mc_queue->try_enqueue(event)) {
    std::printf("Need more memory\n");
    RtAssert(mc_queue->enqueue(event), "Message enqueue failed");
  }
}

/// Enqueue a batch of events to a concurrent queue and print a warning message
/// if we're short on queue space
static inline void TryEnqueueBulkFallback(
    moodycamel::ConcurrentQueue<EventData>* mc_queue,
    moodycamel::ProducerToken* producer_token, const EventData* event_list,
    size_t num_events) {
  if (!mc_queue->try_enqueue_bulk(*producer_token, event_list, num_events)) {
    std::printf("Need more memory\n");
    RtAssert(mc_queue->enqueue_bulk(*producer_token, event_list, num_events),
             "Message bulk enqueue failed\n");
  }
}

#endif /* CONCURRENT_QUEUE_WRAPPER */
