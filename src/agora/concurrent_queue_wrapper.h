/**
 * @file concurrent_queue_wrapper.inc
 * @brief Wrapper class for the moodycamel concurrent queue
 */

#ifndef CONCURRENT_QUEUE_WRAPPER_H_
#define CONCURRENT_QUEUE_WRAPPER_H_

#include <cstddef>

#include "buffer.h"
#include "concurrentqueue.h"
#include "message.h"
#include "symbols.h"

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

// Fetch the concurrent queue for this event type
inline moodycamel::ConcurrentQueue<EventData>* GetConq(
    SchedInfo sched_info_arr[kScheduleQueues][kNumEventTypes],
    EventType event_type, size_t qid) {
  return &sched_info_arr[qid][static_cast<size_t>(event_type)].concurrent_q_;
}

// Fetch the producer token for this event type
inline moodycamel::ProducerToken* GetPtok(
    SchedInfo sched_info_arr[kScheduleQueues][kNumEventTypes],
    EventType event_type, size_t qid) {
  return sched_info_arr[qid][static_cast<size_t>(event_type)].ptok_;
}

#endif  // CONCURRENT_QUEUE_WRAPPER_H_
