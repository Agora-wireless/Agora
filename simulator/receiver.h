/**
 * @file receiver.h
 * @brief Declaration file for the receiver class
 */
#ifndef RECEIVER_H_
#define RECEIVER_H_

#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <ctime>
#include <iostream>
#include <mutex>
#include <numeric>
#include <vector>

#include "concurrentqueue.h"
#include "config.h"
#include "gettime.h"
#include "message.h"
#include "symbols.h"

using ushort = unsigned short;
class Receiver {
 public:
  explicit Receiver(Config* cfg, size_t rx_thread_num = 1,
                    size_t core_offset = 1);

  /**
   * rx_thread_num: RX thread number
   * in_queue: message queue to communicate with main thread
   */
  Receiver(Config* cfg, size_t rx_thread_num, size_t core_offset,
           moodycamel::ConcurrentQueue<EventData>* in_queue_message,
           moodycamel::ProducerToken** in_rx_ptoks);
  ~Receiver() = default;

  /**
   * Called in main threads to start the socket threads
   * in_buffer: ring buffer to save packets
   * in_buffer_status: record the status of each memory block (0: empty, 1:
   * full) in_buffer_frame_num: number of packets the ring buffer could hold
   * in_buffer_length: size of ring buffer
   * in_core_id: attach socket threads to {in_core_id, ..., in_core_id +
   * RX_THREAD_NUM - 1}
   */
  std::vector<std::thread> StartRecv(Table<char>& in_buffer,
                                     size_t in_buffer_length,
                                     Table<double>& in_frame_start);

  /**
   * receive thread
   * context: ReceiverContext type
   */
  void* LoopRecv(size_t tid);

 private:
  // Dimension 1: socket_thread
  // Dimension 2: rx_packet
  std::vector<std::vector<RxPacket>> rx_packets_;
  size_t buffers_per_thread_;

  size_t rx_thread_num_;

  Table<double>* frame_start_;
  moodycamel::ConcurrentQueue<EventData>* message_queue_;
  moodycamel::ProducerToken** rx_ptoks_;
  size_t core_id_;
  Config* cfg_;
};

#endif  // RECEIVER_H_
