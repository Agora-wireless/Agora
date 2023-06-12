/**
 * @file simulator.h
 * @brief Declaration file for the simulator class
 */
#ifndef SIMULATOR_H_
#define SIMULATOR_H_

#include <emmintrin.h>
#include <immintrin.h>
#include <unistd.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <queue>
#include <system_error>
#include <tuple>
#include <vector>

#include "concurrentqueue.h"
#include "config.h"
#include "gettime.h"
#include "memory_manage.h"
#include "message.h"
#include "receiver.h"
#include "sender.h"
#include "signal_handler.h"

class Simulator {
 public:
  /* optimization parameters for block transpose (see the slides for more
   * details) */
  static constexpr size_t kTransposeBlockSize = 8;
  static constexpr size_t kTransposeBlockNum = 256;
  /* dequeue bulk size, used to reduce the overhead of dequeue in main thread
   */
  static constexpr size_t kDequeueBulkSize = 32;
  static constexpr size_t kDequeueBulkSizeSingle = 8;

  Simulator(Config* cfg, size_t task_thread_num, size_t core_offset,
            size_t sender_delay);

  ~Simulator();

  void Start();
  void Stop();

  inline void UpdateFrameCount(int* frame_count);
  void UpdateRxCounters(size_t frame_id, size_t frame_id_in_buffer,
                        size_t symbol_id, size_t ant_id);
  void PrintPerFrameDone(PrintType print_type, size_t frame_id);

 private:
  size_t task_thread_num_;
  size_t socket_rx_thread_num_;
  size_t socket_tx_thread_num_;
  size_t core_offset_;

  /* lookup table for 16 QAM, real and imag */

  Config* config_;
  std::unique_ptr<Receiver> receiver_;
  std::unique_ptr<Sender> sender_;

  /**
   * Received data
   *
   * First dimension: SOCKET_THREAD_NUM
   *
   * Second dimension of socket_buffer: kFrameWnd * BS_ANT_NUM *
   * symbol_num_perframe * packet_length
   *
   * Second dimension of buffer status: kFrameWnd * BS_ANT_NUM *
   * symbol_num_perframe
   */
  Table<char> socket_buffer_;
  size_t socket_buffer_size_;

  /* status checkers used by master thread */
  /* used to check if RX for all antennas and all symbols in a frame is done
   * (max: BS_ANT_NUM * dl symbols) */
  size_t* rx_counter_packets_;

  /*****************************************************
   * Concurrent queues
   *****************************************************/
  /* main thread message queue for data receiving */
  moodycamel::ConcurrentQueue<EventData> message_queue_;
  /* main thread message queue for task completion*/
  moodycamel::ConcurrentQueue<EventData> complete_task_queue_;

  /* Tokens */
  moodycamel::ProducerToken** rx_ptoks_ptr_;

  /*****************************************************
   * Timestamps and counters used in worker threads
   *****************************************************/
  Table<double> frame_start_;
  double* frame_start_receive_;
  double* frame_end_receive_;
  double* frame_start_tx_;
  double* frame_end_tx_;

  void InitializeQueues();
  void InitializeBuffers();
  void FreeBuffers();
  void FreeQueues();
};

#endif  // SIMULATOR_H_
