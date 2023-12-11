/**
 * @file packet_txrx.h
 * @brief Common definations for PacketTxRx. Including datapath
 * functions for communicating with simulators.
 */

#ifndef PACKETTXRX_H_
#define PACKETTXRX_H_

#include <vector>

#include "common_typedef_sdk.h"
#include "concurrentqueue.h"
#include "config.h"
#include "message.h"
#include "txrx_worker.h"

namespace AgoraTxRx {
enum TxRxTypes { kBaseStation, kUserEquiptment };
}

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
class PacketTxRx {
 public:
  PacketTxRx(AgoraTxRx::TxRxTypes type, Config* const cfg, size_t core_offset,
             moodycamel::ConcurrentQueue<EventData>* event_notify_q,
             moodycamel::ConcurrentQueue<EventData>* tx_pending_q,
             moodycamel::ProducerToken** notify_producer_tokens,
             moodycamel::ProducerToken** tx_producer_tokens,
             Table<char>& rx_buffer, size_t packet_num_in_buffer,
             Table<size_t>& frame_start, char* tx_buffer);
  virtual ~PacketTxRx();

  /**
   * @brief Start the network I/O threads
   *
   * @return True on successfully starting the network I/O threads, false
   * otherwise
   */
  virtual bool StartTxRx(Table<complex_float>& calib_dl_buffer,
                         Table<complex_float>& calib_ul_buffer);

  /**
   * @brief Convert the antenna id to txrx worker id
   *
   * @param ant_num antenna number
   *
   * @return The worker id
   */
  size_t AntNumToWorkerId(size_t ant_num) const;

 protected:
  bool StopTxRx();
  //Align all worker threads to common start event (this call)
  void NotifyWorkers();
  inline size_t NumberTotalWorkers() const { return worker_thread_count_; }
  inline size_t NumberTotalInterfaces() const {
    return interface_to_worker_.size();
  }
  inline const size_t& InterfaceToWorker(size_t interface) const {
    return interface_to_worker_.at(interface);
  }
  inline size_t NumChannels() const { return num_channels_; }

  std::vector<std::unique_ptr<TxRxWorker>> worker_threads_;
  Config* const cfg_;

  const size_t core_offset_;
  moodycamel::ConcurrentQueue<EventData>* event_notify_q_;
  moodycamel::ConcurrentQueue<EventData>* tx_pending_q_;

  //Producer tokens for posting messages to event_notify_q_
  moodycamel::ProducerToken** notify_producer_tokens_;
  //Producers assigned to tx_pending_q (used for retrieving messages)
  moodycamel::ProducerToken** tx_producer_tokens_;

  ///Owned by this class and shared with the workers
  std::mutex mutex_;
  std::condition_variable cond_;
  std::atomic<bool> proceed_;

 private:
  virtual bool CreateWorker(size_t tid, size_t interface_count,
                            size_t interface_offset, size_t* rx_frame_start,
                            std::vector<RxPacket>& rx_memory,
                            std::byte* const tx_memory) = 0;

  // Dimension 1: socket_thread
  // Dimension 2: rx_packet
  std::vector<std::vector<RxPacket>> rx_packets_;
  std::byte* const tx_memory_;
  Table<size_t>& frame_start_;

  size_t worker_thread_count_;
  std::vector<size_t> interface_to_worker_;
  const AgoraTxRx::TxRxTypes type_;
  size_t num_channels_;
};

#endif  // PACKETTXRX_H_
