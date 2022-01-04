/**
 * @file txrx_worker.h
 * @brief txrx worker thread definition.  This is the parent / interface
 */
#ifndef TXRX_WORKER_H_
#define TXRX_WORKER_H_

#include <thread>
#include <vector>

#include "concurrentqueue.h"
#include "config.h"

class TxRxWorker {
 public:
  TxRxWorker(size_t core_offset, size_t tid, size_t interface_count,
             size_t interface_offset, Config* const config,
             size_t* rx_frame_start,
             moodycamel::ConcurrentQueue<EventData>* event_notify_q,
             moodycamel::ConcurrentQueue<EventData>* tx_pending_q,
             moodycamel::ProducerToken& tx_producer,
             moodycamel::ProducerToken& notify_producer,
             std::vector<RxPacket>& rx_memory, std::byte* const tx_memory);

  virtual ~TxRxWorker();

  virtual void Start();
  virtual void Stop();
  virtual void DoTxRx() = 0;

  inline size_t Id() const { return tid_; }
  inline bool Started() const { return started_; }
  inline bool Running() const { return running_; }

 protected:
  inline Config* Configuration() { return cfg_; }

 private:
  TxRxWorker() = delete;
  Config* const cfg_;

 protected:
  const size_t tid_;
  const size_t core_offset_;
  const size_t num_interfaces_;
  const size_t interface_offset_;
  const size_t channels_per_interface_;
  const size_t ant_per_cell_;

  size_t* const rx_frame_start_;

  std::vector<RxPacket>& rx_memory_;
  std::byte* const tx_memory_;

  moodycamel::ConcurrentQueue<EventData>* event_notify_q_;
  moodycamel::ConcurrentQueue<EventData>* tx_pending_q_;
  //foreign producer of tx messages (used for TX only)
  moodycamel::ProducerToken& tx_producer_token_;
  //local producer of notification messages (used for TX and RX)
  moodycamel::ProducerToken& notify_producer_token_;

  std::thread thread_;
  bool running_;
  bool started_;
};
#endif  // TXRX_WORKER_H_
