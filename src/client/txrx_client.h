/**
 * @file txrx_client.h
 * @brief Declaration file for the radio txrx class
 */
#ifndef RADIOTXRX_H_
#define RADIOTXRX_H_

#include <sys/types.h>

#include <complex>
#include <fstream>
#include <thread>
#include <vector>

#include "client_radio.h"
#include "concurrentqueue.h"
#include "udp_client.h"
#include "udp_server.h"
#include "utils.h"

/**
 * @brief Implementations of this class provide packet I/O for Agora clients
 * (UEs).
 *
 * In the vanilla mode, this class provides socket packet I/O for communicating
 * with channel simulator.
 *
 * In the "Argos" mode, this class uses SoapySDR-based functions (from
 * client_radio.cpp) to communicate with Agora client wireless hardware. Since
 * clients require over-the-air time synchronization with Agora BS, this class
 * provides two types of functions for this purpose: 1) TX/RX with
 * hardware-offloaded synchronization (loop_tx_rx_argos) where sync happens in
 * the wireless hardware, where only downlink symbols' samples are received
 * (guard symbols are discarded in hardware) and uplink symbols are sent to the
 * wireless hardware to be sent at the correct time w.r.t BS. 2) software-based
 * syncronization (loop_tx_rx_argos_sync) where all of sychronization and
 * time-keeping w.r.t. BS happens in this function as well as downlink and
 * uplink receive and transmit functions.
 */
class RadioTxRx {
 public:
  RadioTxRx(Config* const cfg, int n_threads, int in_core_id);
  /**
   * N_THREAD: socket thread number
   * mode: tx=1 or rx=0 operation
   * in_queue: message queue to communicate with main thread
   */
  RadioTxRx(Config* const config, int n_threads, int in_core_id,
            moodycamel::ConcurrentQueue<EventData>* in_message_queue,
            moodycamel::ConcurrentQueue<EventData>* in_task_queue,
            moodycamel::ProducerToken** in_rx_ptoks,
            moodycamel::ProducerToken** in_tx_ptoks);
  ~RadioTxRx();

  /**
   * @brief called in main threads to start the socket threads
   *
   * @param in_buffer: rx ring buffer to save received packets
   *
   * @param in_buffer_status: record the status of each memory block (0: empty,
   * 1: full)
   *
   * @param in_buffer_frame_num: number of sample packets (one symbol per
   * packet) the ring buffer could hold
   *
   * @param in_buffer_length: size of ring buffer in bytes
   *
   * @param in_tx_buffer: tx ring buffer of samples processed by PHY for
   * transmission
   *
   * @param in_buffer_frame_num: number of samples packets (one symbol
   * per packet) the tx ring buffer could hold
   *
   * @param in_buffer_length: size of ring buffer in bytes
   */
  bool StartTxRx(Table<char>& in_buffer, size_t in_buffer_length,
                 char* in_tx_buffer, int* in_tx_buffer_status,
                 int in_tx_buffer_frame_num, int in_tx_buffer_length);

  /**
   * @brief receives a packet from channel simulator through socket index
   * (radio_id) and writes to an offset (rx_offset) in the receive buffer
   * (buffer_)
   */
  struct Packet* RecvEnqueue(size_t tid, size_t ant_id, size_t rx_slot);

  /**
   * @brief transmits a tx packet that is ready from PHY through socket
   * to channel simualtor
   */
  int DequeueSend(int tid);

  /**
   * @brief receives a packet from hardware through radio index (radio_id)
   * and writes to an offset (rx_offset) in the receive buffer (buffer_)
   */
  struct Packet* RecvEnqueueArgos(size_t tid, size_t radio_id, size_t& frame_id,
                                  size_t& symbol_id, size_t rx_slot,
                                  bool dummy_enqueue);

  /**
   * @brief transmits a tx samples packet that is ready from PHY through client
   * wireless hardware
   */
  int DequeueSendArgos(int tid, long long time0);

  /**
   * @brief loop thread function that performs sample Packet I/O in simulation
   * mode.
   */
  void* LoopTxRx(size_t tid);

  /**
   * @brief loop thread function that performs sample TX/RX to/from client
   * wireless hardware where the time synchornization with BS occurs in the
   * hardware. The fucntion reads timestamps from received symbols and uses the
   * timestamps to schedules transmit of processed uplink symbols with some
   * offset in the future from client wireless hardware.
   */
  void* LoopTxRxArgos(size_t tid);

  /**
   * @brief loop thread function that performs sample TX/RX to/from client
   * wireless hardware The function performs correlation on the received samples
   * first to find a beacon transmitted by the BS continuously. Once found locks
   * to that beacon as the time reference for the start of BS frames. It tries
   * to re-lock to the beacon every 1000 frames in case of time drifts. Based on
   * the beacon time reference, it writes downlink symbols to rx buffers and
   * schedules uplink symbols for transmission from the hardware.
   */
  void* LoopTxRxArgosSync(size_t tid);
  void* LoopTxRxUsrpSync(size_t tid);

 private:
  std::mutex mutex_;
  std::condition_variable cond_;
  std::atomic<bool> thread_sync_;

  std::vector<std::thread> txrx_threads_;

  Config* const config_;

  // Used only in Argos mode
  std::unique_ptr<ClientRadioConfig> radioconfig_;

  std::vector<std::unique_ptr<UDPClient>> udp_clients_;
  std::vector<std::unique_ptr<UDPServer>> udp_servers_;

  // Dimension 1: socket_thread
  // Dimension 2: rx_packet
  std::vector<std::vector<RxPacket>> rx_packets_;
  size_t buffers_per_thread_;

  char* tx_buffer_;
  int* tx_buffer_status_;
  int tx_buffer_frame_num_;
  int tx_buffer_length_;

  size_t thread_num_;
  int tx_thread_num_;
  // pointer of message_queue_
  moodycamel::ConcurrentQueue<EventData>* message_queue_;
  moodycamel::ConcurrentQueue<EventData>* task_queue_;
  moodycamel::ProducerToken** rx_ptoks_;
  moodycamel::ProducerToken** tx_ptoks_;
  int core_id_;
  int tx_core_id_;

  // helper buffers
  std::vector<void*> pilot_buff0_;
  std::vector<void*> pilot_buff1_;
};
#endif  // RADIOTXRX_H_
