/**
 * @file txrx.h
 * @brief Common definations for PacketTXRX. Including datapath
 * functions for communicating with simulators.
 */

#ifndef PACKETTXRX_H_
#define PACKETTXRX_H_

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <ctime>
#include <iostream>
#include <mutex>
#include <numeric>
#include <vector>

#include "buffer.h"
#include "concurrentqueue.h"
#include "config.h"
#include "gettime.h"
#include "net.h"
#include "radio_lib.h"
#include "symbols.h"

#ifdef USE_DPDK
#include "dpdk_transport.h"
#endif

/**
 * @brief Implementations of this class provide packet I/O for Agora.
 *
 * In the vanilla mode, this class provides socket or DPDK-based packet I/O to
 * Agora (running on the base station server or client) for communicating
 * with simulated peers.
 *
 * In the "Argos" mode, this class provides SoapySDR-based communication for
 * Agora (running on the base station server or client) for communicating
 * with real wireless hardware peers (antenna hubs for the server, UE devices
 * for the client).
 */
class PacketTXRX {
 public:
  static const int kMaxSocketNum = 10;  // Max number of socket threads allowed

  explicit PacketTXRX(Config* cfg, size_t in_core_offset = 1);

  PacketTXRX(Config* cfg, size_t core_offset,
             moodycamel::ConcurrentQueue<EventData>* queue_message,
             moodycamel::ConcurrentQueue<EventData>* queue_task,
             moodycamel::ProducerToken** rx_ptoks,
             moodycamel::ProducerToken** tx_ptoks);
  ~PacketTXRX();

#ifdef USE_DPDK
  // At thread [tid], receive packets from the NIC and enqueue them to the
  // master thread
  uint16_t DpdkRecv(int tid, uint16_t port_id, uint16_t queue_id,
                    size_t& prev_frame_id, size_t& rx_offset);
#endif

  /**
   * @brief Start the network I/O threads
   *
   * @param buffer Ring buffer to save packets
   * @param buffer_status Status of each packet buffer (0: empty, 1: full)
   * @packet_num_in_buffer Total number of buffers in an RX ring
   *
   * @return True on successfully starting the network I/O threads, false
   * otherwise
   */
  bool StartTxRx(Table<char>& buffer, Table<int>& buffer_status,
                 size_t packet_num_in_buffer, Table<size_t>& frame_start,
                 char* tx_buffer, Table<complex_float>& calib_dl_buffer_,
                 Table<complex_float>& calib_ul_buffer_);

  void SendBeacon(int tid, size_t frame_id);

 private:
  void LoopTxRx(int tid);  // The thread function for thread [tid]
  int DequeueSend(int tid);
  struct Packet* RecvEnqueue(int tid, int radio_id, int rx_offset);

  void LoopTxRxArgos(int tid);
  int DequeueSendArgos(int tid);
  std::vector<struct Packet*> RecvEnqueueArgos(int tid, int radio_id,
                                               int rx_offset);

  long long rx_time_bs_;
  long long tx_time_bs_;
  void LoopTxRxUsrp(int tid);
  int DequeueSendUsrp(int tid);
  int DequeueSendUsrp(int tid, int frame_id, int symbol_id);
  struct Packet* RecvEnqueueUsrp(int tid, int radio_id, int rx_offset,
                                 int frame_id, int symbol_id);

  Config* cfg_;

  // The network I/O threads run on cores
  // {core_offset, ..., core_offset + socket_thread_num - 1}
  const size_t core_offset_;
  const size_t ant_per_cell_;
  const size_t socket_thread_num_;

  // Handle for socket threads
  std::array<std::thread, kMaxSocketNum> socket_std_threads_;
  Table<char>* buffer_;
  Table<int>* buffer_status_;
  size_t packet_num_in_buffer_;
  char* tx_buffer_;
  Table<size_t>* frame_start_;
  moodycamel::ConcurrentQueue<EventData>* message_queue_;
  moodycamel::ConcurrentQueue<EventData>* task_queue_;
  moodycamel::ProducerToken** rx_ptoks_;
  moodycamel::ProducerToken** tx_ptoks_;

  std::vector<struct sockaddr_in> bs_rru_sockaddr_;
  std::vector<int> socket_;

#ifdef USE_DPDK
  uint32_t bs_rru_addr;     // IPv4 address of the simulator sender
  uint32_t bs_server_addr;  // IPv4 address of the Agora server
  struct rte_mempool* mbuf_pool;
#endif

  RadioConfig* radioconfig_;  // Used only in Argos mode
};

#endif  // PACKETTXRX_H_
