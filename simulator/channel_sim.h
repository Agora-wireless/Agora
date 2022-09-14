/**
 * @file channel_sim.h
 * @brief Declaration file for the channel simulator class
 */
#ifndef CHANNEL_SIM_H_
#define CHANNEL_SIM_H_

#include <array>
#include <cstddef>
#include <memory>
#include <thread>
#include <vector>

#include "channel.h"
#include "chsim_worker_storage.h"
#include "concurrentqueue.h"
#include "config.h"
#include "message.h"
#include "time_frame_counters.h"
#include "udp_comm.h"

/**
 * @brief Simualtor for many-antenna MU-MIMO channel to work with
 * Agora BS and UE applications. It generates channel matrice(s)
 * and applies it to incoming baseband samples from BS and sends them
 * to the UE application. Similarly, applies the same channel (TDD) to
 * uplink baseband samples from UE and sends them to BS.
 */
class ChannelSim {
 public:
  ChannelSim(const Config* const config, size_t bs_thread_num,
             size_t user_thread_num, size_t worker_thread_num,
             size_t in_core_offset = 30,
             std::string in_chan_type = std::string("RAYLEIGH"),
             double in_chan_snr = 20);
  ~ChannelSim();

  void Run();

  static void* RxLoop(ChSimRxStorage* rx_storage);
  // Loop thread receiving symbols from client antennas
  void* UeRxLoop(size_t tid);

  // Loop thread receiving symbols from BS antennas
  void* BsRxLoop(size_t tid);

  void ScheduleTask(EventData do_task,
                    moodycamel::ConcurrentQueue<EventData>* in_queue,
                    moodycamel::ProducerToken const& ptok);

  // Calls DoTxBs / DoTxUser
  void* TaskThread(size_t tid);
  // Transmits symbol to BS antennas after applying channel
  void DoTxBs(ChSimWorkerStorage* local, size_t tag);

  // Transmit symbols to client antennas after applying channel
  void DoTxUser(ChSimWorkerStorage* local, size_t tag);

 private:
  void DoTx(size_t frame_id, size_t symbol_id, size_t max_ant,
            size_t ant_per_socket, const arma::cx_float* source_data,
            SimdAlignByteVector* udp_pkt_buf,
            std::vector<std::unique_ptr<UDPComm>>& udp_senders);

  std::vector<std::pair<std::thread, std::unique_ptr<ChSimRxStorage>>>
  CreateRxThreads();
  size_t AddRxThreads(
      size_t desired_threads, size_t total_interfaces,
      std::vector<std::unique_ptr<UDPComm>>& comm, ChSimRxBuffer* rx_buffer,
      std::vector<std::pair<std::thread, std::unique_ptr<ChSimRxStorage>>>&
          rx_threads_out);

  // BS-facing sockets
  std::vector<std::unique_ptr<UDPComm>> bs_comm_;
  // UE-facing sockets
  std::vector<std::unique_ptr<UDPComm>> ue_comm_;

  const Config* const cfg_;
  std::unique_ptr<Channel> channel_;

  // Data buffer for received symbols from BS antennas (downlink)
  std::unique_ptr<ChSimRxBuffer> rx_buffer_bs_;

  // Data buffer for received symbols from client antennas (uplink)
  std::unique_ptr<ChSimRxBuffer> rx_buffer_ue_;

  // Task Queue for tasks related to incoming BS packets
  moodycamel::ConcurrentQueue<EventData> task_queue_bs_;

  // Task Queue for tasks related to incoming Users' packets
  moodycamel::ConcurrentQueue<EventData> task_queue_user_;

  // Master thread's message queue for event completions;
  moodycamel::ConcurrentQueue<EventData> message_queue_;
  std::array<std::unique_ptr<moodycamel::ProducerToken>, kMaxThreads>
      task_ptok_;

  std::vector<std::thread> task_threads_;

  size_t ul_data_plus_pilot_symbols_;
  size_t dl_data_plus_beacon_symbols_;
  size_t payload_length_;

  size_t bs_thread_num_;
  size_t user_thread_num_;
  size_t bs_socket_num_;
  size_t user_socket_num_;
  size_t worker_thread_num_;
  size_t core_offset_;

  std::string channel_type_;
  double channel_snr_;

  TimeFrameCounters ue_rx_;
  TimeFrameCounters ue_tx_;
  TimeFrameCounters bs_rx_;
  TimeFrameCounters bs_tx_;
};

#endif  // CHANNEL_SIM_H_
