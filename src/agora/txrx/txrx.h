/**
 * @file txrx.h
 * @brief Common definations for PacketTXRX. Including datapath
 * functions for communicating with simulators.
 */

#ifndef PACKETTXRX_H_
#define PACKETTXRX_H_

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
#include "radio_lib.h"
#include "symbols.h"
#include "udp_client.h"
#include "udp_server.h"

#if defined(USE_DPDK)
#include "dpdk_transport.h"

// Removed support for copy free dpdk memory due to slowdown issue.
//#define USE_DPDK_MEMORY

#if defined(USE_DPDK_MEMORY)
class DPDKRxPacket : public RxPacket {
 public:
  DPDKRxPacket() : RxPacket() { mem_ = nullptr; }
  explicit DPDKRxPacket(const DPDKRxPacket& copy) : RxPacket(copy) {
    mem_ = copy.mem_;
  }
  ~DPDKRxPacket() = default;
  inline bool Set(rte_mbuf* mem, Packet* in_pkt) {
    mem_ = mem;
    return RxPacket::Set(in_pkt);
  }

 private:
  rte_mbuf* mem_;
  inline void GcPacket() override {
    // std::printf("Garbage collecting the memory for DPDKRxPacket\n");
    rte_pktmbuf_free(mem_);
    this->Set(nullptr, nullptr);
  }
};
#endif  // defined(USE_DPDK_MEMORY)
#endif  //  defined(USE_DPDK)

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
  explicit PacketTXRX(Config* cfg, size_t in_core_offset = 1);

  PacketTXRX(Config* cfg, size_t core_offset,
             moodycamel::ConcurrentQueue<EventData>* queue_message,
             moodycamel::ConcurrentQueue<EventData>* queue_task,
             moodycamel::ProducerToken** rx_ptoks,
             moodycamel::ProducerToken** tx_ptoks);
  ~PacketTXRX();

#if defined(USE_DPDK)
  // At thread [tid], receive packets from the NIC and enqueue them to the
  // master thread
  uint16_t DpdkRecv(int tid, uint16_t port_id, uint16_t queue_id,
                    size_t& prev_frame_id, size_t& rx_slot);
#endif

  /**
   * @brief Start the network I/O threads
   *
   * @param buffer Ring buffer to save packets
   * @param packet_num_in_buffer Total number of buffers in an RX ring
   *
   * @return True on successfully starting the network I/O threads, false
   * otherwise
   */
  bool StartTxRx(Table<char>& buffer, size_t packet_num_in_buffer,
                 Table<size_t>& frame_start, char* tx_buffer,
                 Table<complex_float>& calib_dl_buffer_,
                 Table<complex_float>& calib_ul_buffer_);

  void SendBeacon(int tid, size_t frame_id);

 private:
  void LoopTxRx(size_t tid);  // The thread function for thread [tid]
  size_t DequeueSend(int tid);
  Packet* RecvEnqueue(size_t tid, size_t radio_id, size_t rx_offset);

  void LoopTxRxArgos(size_t tid);
  size_t DequeueSendArgos(int tid);
  std::vector<Packet*> RecvEnqueueArgos(size_t tid, size_t radio_id,
                                        size_t rx_slot);

  long long rx_time_bs_;
  long long tx_time_bs_;
  void LoopTxRxUsrp(size_t tid);
  int DequeueSendUsrp(int tid);
  int DequeueSendUsrp(int tid, int frame_id, int symbol_id);
  Packet* RecvEnqueueUsrp(size_t tid, size_t radio_id, size_t rx_slot,
                          size_t frame_id, size_t symbol_id);

  Config* cfg_;

  // The network I/O threads run on cores
  // {core_offset, ..., core_offset + socket_thread_num - 1}
  const size_t core_offset_;
  const size_t ant_per_cell_;
  const size_t socket_thread_num_;

  // Handle for socket threads
  std::vector<std::thread> socket_std_threads_;
  size_t buffers_per_socket_;

  char* tx_buffer_;
  Table<size_t>* frame_start_;
  moodycamel::ConcurrentQueue<EventData>* message_queue_;
  moodycamel::ConcurrentQueue<EventData>* task_queue_;
  moodycamel::ProducerToken** rx_ptoks_;
  moodycamel::ProducerToken** tx_ptoks_;

  std::vector<std::unique_ptr<UDPServer>> udp_servers_;
  std::vector<std::unique_ptr<UDPClient>> udp_clients_;

  std::atomic<size_t> threads_started_;

#if defined(USE_DPDK)
  std::vector<uint16_t> port_ids_;
  uint32_t bs_rru_addr_;     // IPv4 address of the simulator sender
  uint32_t bs_server_addr_;  // IPv4 address of the Agora server
  struct rte_mempool* mbuf_pool_;

  // Dimension 1: socket_thread
  // Dimension 2: rx_packet
#if defined(USE_DPDK_MEMORY)
  std::vector<std::vector<DPDKRxPacket>> rx_packets_;
#else
  std::vector<std::vector<RxPacket>> rx_packets_;
#endif  // defined(USE_DPDK_MEMORY)
#else
  // Dimension 1: socket_thread
  // Dimension 2: rx_packet
  std::vector<std::vector<RxPacket>> rx_packets_;
#endif  // defined(USE_DPDK)

  std::unique_ptr<RadioConfig> radioconfig_;  // Used only in Argos mode
};

#endif  // PACKETTXRX_H_
