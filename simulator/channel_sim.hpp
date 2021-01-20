#ifndef SIMULATOR_HEADER
#define SIMULATOR_HEADER

#include <arpa/inet.h>
#include <netinet/in.h>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <algorithm>
#include <armadillo>
#include <ctime>
#include <iomanip>
#include <numeric>

#include "Symbols.hpp"
#include "buffer.hpp"
#include "channel.hpp"
#include "concurrent_queue_wrapper.hpp"
#include "config.hpp"
#include "gettime.h"
#include "memory_manage.h"
#include "net.hpp"
#include "signalHandler.hpp"
#include "udp_client.h"
#include "udp_server.h"

using namespace arma;

/**
 * @brief Simualtor for many-antenna MU-MIMO channel to work with
 * Agora BS and UE applications. It generates channel matrice(s)
 * and applies it to incoming baseband samples from BS and sends them
 * to the UE application. Similarly, applies the same channel (TDD) to
 * uplink baseband samples from UE and sends them to BS.
 */
class ChannelSim {
 public:
  ChannelSim(Config* bscfg, Config* uecfg, size_t bs_thread_num,
             size_t user_thread_num, size_t worker_thread_num,
             size_t in_core_offset = 30, std::string in_chan_type = "RAYLEIGH",
             double in_chan_snr = 20);
  ~ChannelSim();

  void start();

  // Loop thread receiving symbols from client antennas
  void* ue_rx_loop(int tid);

  // Loop thread receiving symbols from BS antennas
  void* bs_rx_loop(int tid);

  // Transmits symbol to BS antennas after applying channel
  void do_tx_bs(int tid, size_t tag);

  // Transmit symbols to client antennas after applying channel
  void do_tx_user(int tid, size_t tag);

  void schedule_task(Event_data do_task,
                     moodycamel::ConcurrentQueue<Event_data>* in_queue,
                     moodycamel::ProducerToken const& ptok);
  void* taskThread(int tid);

 private:
  std::vector<struct sockaddr_in> servaddr_bs_;  // BS-facing server addresses
  std::vector<int> socket_bs_;                   // BS-facing sockets
  std::vector<struct sockaddr_in> servaddr_ue_;  // UE-facing server addresses
  std::vector<int> socket_ue_;                   // UE-facing sockets

  Config* bscfg;
  Config* uecfg;
  Channel* channel;

  // Data buffer for symbols to be transmitted to BS antennas (uplink)
  std::vector<char> tx_buffer_bs;

  // Data buffer for symbols to be transmitted to client antennas (downlink)
  std::vector<char> tx_buffer_ue;

  // Data buffer for received symbols from BS antennas (downlink)
  std::vector<char> rx_buffer_bs;

  // Data buffer for received symbols from client antennas (uplink)
  std::vector<char> rx_buffer_ue;

  // Task Queue for tasks related to incoming BS packets
  moodycamel::ConcurrentQueue<Event_data> task_queue_bs;

  // Task Queue for tasks related to incoming Users' packets
  moodycamel::ConcurrentQueue<Event_data> task_queue_user;

  // Master thread's message queue for event completions;
  moodycamel::ConcurrentQueue<Event_data> message_queue_;
  moodycamel::ProducerToken* task_ptok[kMaxThreads];

  pthread_t* task_threads;

  size_t ul_data_plus_pilot_symbols;
  size_t dl_data_plus_beacon_symbols;
  size_t payload_length;

  size_t bs_thread_num;
  size_t user_thread_num;
  size_t bs_socket_num;
  size_t user_socket_num;
  size_t worker_thread_num;
  size_t core_offset;

  std::string channel_type;
  double channel_snr;

  size_t* bs_rx_counter_;
  size_t* user_rx_counter_;
  size_t bs_tx_counter_[kFrameWnd];
  size_t user_tx_counter_[kFrameWnd];

  inline size_t get_dl_symbol_idx(size_t frame_id, size_t symbol_id) const {
    if (symbol_id == 0) {
      return 0;
    } else {
      return bscfg->GetDLSymbolIdx(frame_id, symbol_id) + 1;
    }
  }
};

#endif
