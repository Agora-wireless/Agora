/**
 * @file simulator.h
 * @brief Declaration file for the simulator class
 */
#ifndef SIMULATOR_H_
#define SIMULATOR_H_

#include <emmintrin.h>
#include <fcntl.h>
#include <immintrin.h>
#include <pthread.h>
#include <unistd.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <queue>
#include <system_error>
#include <tuple>
#include <vector>

#include "buffer.h"
#include "concurrentqueue.h"
#include "config.h"
#include "gettime.h"
#include "memory_manage.h"
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
  // while loop of task thread
  static void* TaskThread(void* context);

  struct EventHandlerContext {
    Simulator* obj_ptr_;
    size_t id_;
  };

  inline void UpdateFrameCount(int* frame_count);

  void UpdateRxCounters(size_t frame_id, size_t frame_id_in_buffer,
                        size_t symbol_id, size_t ant_id);
  void PrintPerFrameDone(PrintType print_type, size_t frame_id);

  void InitializeVarsFromCfg(Config* cfg);
  void InitializeQueues();
  void InitializeUplinkBuffers();
  void FreeUplinkBuffers();

 private:
  size_t bs_ant_num_;
  size_t ue_num_;
  size_t ofdm_ca_num_;
  size_t ofdm_data_num_;
  size_t symbol_num_perframe_, data_symbol_num_perframe_;
  size_t ul_data_symbol_num_perframe_, dl_data_symbol_num_perframe_;
  size_t dl_data_symbol_start_, dl_data_symbol_end_;
  size_t packet_length_;

  size_t task_thread_num_, socket_rx_thread_num_, socket_tx_thread_num_;
  size_t core_offset_;
  size_t demul_block_size_, demul_block_num_;

  /* lookup table for 16 QAM, real and imag */
  Table<float> qam16_table_;
  // float *pilots_;
  Config* config_;
  size_t max_equaled_frame_ = 0;
  float csi_format_offset_;
  size_t buffer_frame_num_;
  size_t max_packet_num_per_frame_;
  std::unique_ptr<Receiver> receiver_;
  std::unique_ptr<Sender> sender_;
  pthread_t* task_threads_;
  EventHandlerContext* context_;

  // Uplink buffers

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
  Table<int> socket_buffer_status_;

  size_t socket_buffer_size_;
  size_t socket_buffer_status_size_;

  /* Uplink status checkers used by master thread */
  /* used to check if RX for all antennas and all symbols in a frame is done
   * (max: BS_ANT_NUM * symbol_num_perframe) */
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
  moodycamel::ProducerToken** tx_ptoks_ptr_;
  moodycamel::ProducerToken** task_ptoks_ptr_;

  /*****************************************************
   * Timestamps and counters used in worker threads
   *****************************************************/
  Table<double> frame_start_;
  double* frame_start_receive_;
  double* frame_end_receive_;
  double* frame_start_tx_;
  double* frame_end_tx_;
};

#endif  // SIMULATOR_H_
