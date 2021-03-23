/**
 * @file channel_sim.h
 * @brief Declaration file for the channel simulator class
 */
#ifndef CHANNEL_SIM_H_
#define CHANNEL_SIM_H_

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <algorithm>
#include <armadillo>
#include <ctime>
#include <iomanip>
#include <numeric>

#include "buffer.h"
#include "channel.h"
#include "concurrent_queue_wrapper.h"
#include "config.h"
#include "gettime.h"
#include "memory_manage.h"
#include "net.h"
#include "signal_handler.h"
#include "symbols.h"
#include "udp_client.h"
#include "udp_server.h"

/**
 * @brief Simualtor for many-antenna MU-MIMO channel to work with
 * Agora BS and UE applications. It generates channel matrice(s)
 * and applies it to incoming baseband samples from BS and sends them
 * to the UE application. Similarly, applies the same channel (TDD) to
 * uplink baseband samples from UE and sends them to BS.
 */
class ChannelSim {
 public:
  ChannelSim(const Config* const config_bs, const Config* const config_ue,
             size_t bs_thread_num, size_t user_thread_num,
             size_t worker_thread_num, size_t in_core_offset = 30,
             std::string in_chan_type = std::string("RAYLEIGH"),
             double in_chan_snr = 20);
  ~ChannelSim();

  void Start();

  // Loop thread receiving symbols from client antennas
  void* UeRxLoop(int tid);

  // Loop thread receiving symbols from BS antennas
  void* BsRxLoop(int tid);

  // Transmits symbol to BS antennas after applying channel
  void DoTxBs(int tid, size_t tag);

  // Transmit symbols to client antennas after applying channel
  void DoTxUser(int tid, size_t tag);

  void ScheduleTask(EventData do_task,
                    moodycamel::ConcurrentQueue<EventData>* in_queue,
                    moodycamel::ProducerToken const& ptok);
  void* TaskThread(int tid);

 private:
  void DoTx(size_t frame_id, size_t symbol_id, size_t max_ant,
            std::vector<char>& tx_buffer, size_t buffer_offset,
            std::vector<int>& send_socket,
            std::vector<struct sockaddr_in>& servaddr,
            arma::cx_fmat& format_dest);

  std::vector<struct sockaddr_in> servaddr_bs_;  // BS-facing server
                                                 // addresses
  std::vector<int> socket_bs_;                   // BS-facing sockets
  std::vector<struct sockaddr_in> servaddr_ue_;  // UE-facing server addresses
  std::vector<int> socket_ue_;                   // UE-facing sockets

  const Config* const bscfg_;
  const Config* const uecfg_;
  std::unique_ptr<Channel> channel_;

  // Data buffer for symbols to be transmitted to BS antennas (uplink)
  std::vector<char> tx_buffer_bs_;

  // Data buffer for symbols to be transmitted to client antennas (downlink)
  std::vector<char> tx_buffer_ue_;

  // Data buffer for received symbols from BS antennas (downlink)
  std::vector<char> rx_buffer_bs_;

  // Data buffer for received symbols from client antennas (uplink)
  std::vector<char> rx_buffer_ue_;

  // Task Queue for tasks related to incoming BS packets
  moodycamel::ConcurrentQueue<EventData> task_queue_bs_;

  // Task Queue for tasks related to incoming Users' packets
  moodycamel::ConcurrentQueue<EventData> task_queue_user_;

  // Master thread's message queue for event completions;
  moodycamel::ConcurrentQueue<EventData> message_queue_;
  moodycamel::ProducerToken* task_ptok_[kMaxThreads];

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

  size_t* bs_rx_counter_;
  size_t* user_rx_counter_;
  std::array<size_t, kFrameWnd> bs_tx_counter_;
  std::array<size_t, kFrameWnd> user_tx_counter_;

  inline size_t GetDlSymbolIdx(size_t /*frame_id*/, size_t symbol_id) const {
    if (symbol_id == 0) {
      return 0;
    } else {
      return bscfg_->Frame().GetDLSymbolIdx(symbol_id) + 1;
    }
  }
};

#endif  // CHANNEL_SIM_H_
