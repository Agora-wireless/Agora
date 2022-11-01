/**
 * @file mac_sender.h
 * @brief Declaration file for the simple mac sender class
 */
#ifndef MAC_SENDER_H_
#define MAC_SENDER_H_

#include <cstdint>
#include <functional>
#include <thread>
#include <vector>

#include "concurrentqueue.h"
#include "config.h"
#include "mac_data_receiver.h"
#include "memory_manage.h"
#include "message.h"

class MacSender {
 public:
  static constexpr size_t kDequeueBulkSize = 4;
  static constexpr size_t kMessageQueueSize = 1024;

  /**
   * @brief Create and optionally start a Sender Client that sends data
   * packets to a agora mac interface
   *
   * @param config The Agora config
   *
   * @param data_filename  Name of the file where the data bytes exist
   *
   * @param socket_thread_num Number of worker threads sending packets
   *
   * @param mac_packets_per_frame Number of mac packets in frame
   *
   * @param core_offset The master thread runs on core [core_offset]. Worker
   * thread #i runs on core [core_offset + i]
   *
   * @param frame_duration_us The TTI slot duration in us
   *
   * @param inter_frame_delay Delay between frames
   *
   * @param enable_slow_start If 1, the sender initially sends frames in a
   * duration larger than the TTI
   */
  MacSender(Config* cfg, std::string& data_filename, size_t mac_packet_length,
            size_t mac_payload_max_length, size_t packets_per_frame,
            std::string server_address, size_t server_rx_port,
            std::function<size_t(size_t)> get_data_symbol_id,
            size_t core_offset = 30, size_t worker_thread_num = 1,
            size_t update_thread_num = 1, size_t frame_duration_us = 0,
            size_t inter_frame_delay = 0, size_t enable_slow_start = 1,
            bool create_thread_for_master = false);
  ~MacSender();

  void StartTx();

  // in_frame_start and in_frame_end must have space for at least
  // kNumStatsFrames entries
  void StartTxfromMain(double* in_frame_start, double* in_frame_end);

 private:
  void* MasterThread(size_t tid);
  void* WorkerThread(size_t tid);
  void* DataUpdateThread(size_t tid, size_t num_data_sources);

  // Get number of CPU ticks for a symbol given a frame index
  uint64_t GetTicksForFrame(size_t frame_id) const;
  size_t GetMaxSymbolId() const;

  // Launch threads to run worker with thread IDs from tid_start to tid_end
  void CreateWorkerThreads(size_t num_workers);

  void UpdateTxBuffer(MacDataReceiver* data_source, gen_tag_t tag);
  void WriteStatsToFile(size_t tx_frame_count) const;

  void ScheduleFrame(size_t frame);
  void LoadFrame(size_t frame);
  size_t TagToTxBuffersIndex(gen_tag_t tag) const;

  Config* cfg_;
  const double freq_ghz_;           // RDTSC frequency in GHz
  const double ticks_per_usec_;     // RDTSC frequency in GHz
  const size_t worker_thread_num_;  // Number of worker threads sending pkts
  const size_t update_thread_num_;  // Number of Tx buffer update threads
  const size_t enable_slow_start_;  // If 1, send frames slowly at first

  // The master thread runs on core core_offset. Worker threads use cores
  // {core_offset + 1, ..., core_offset + thread_num - 1}
  const size_t core_offset_;
  size_t frame_duration_us_;
  const size_t inter_frame_delay_;

  // RDTSC clock ticks between the start of transmission of two symbols in
  // the steady state
  uint64_t ticks_all_;

  // ticks_wnd_1 and ticks_wnd_2 are the RDTSC clock ticks between the start
  // of transmission of two symbols for the first several frames
  uint64_t ticks_wnd1_;
  uint64_t ticks_wnd2_;

  // RDTSC clock ticks between the end of a frame and the start of the next
  // frame
  const uint64_t ticks_inter_frame_;

  moodycamel::ConcurrentQueue<size_t> send_queue_ =
      moodycamel::ConcurrentQueue<size_t>(kMessageQueueSize);
  moodycamel::ConcurrentQueue<size_t> completion_queue_ =
      moodycamel::ConcurrentQueue<size_t>(kMessageQueueSize);
  moodycamel::ProducerToken** task_ptok_;

  std::vector<moodycamel::ConcurrentQueue<size_t>> data_update_queue_;

  double* frame_start_;
  double* frame_end_;

  std::vector<std::thread> threads_;

  Table<uint8_t> tx_buffers_;
  size_t tx_buffer_pkt_offset_;
  std::string data_filename_;

  size_t mac_packet_length_;
  size_t mac_payload_max_length_;
  size_t packets_per_frame_;
  const std::string server_address_;
  const size_t server_rx_port_;
  std::function<size_t(size_t)> get_data_symbol_id_;
  const bool has_master_thread_;
};

#endif  // MAC_SENDER_H_
