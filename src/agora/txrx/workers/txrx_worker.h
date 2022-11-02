/**
 * @file txrx_worker.h
 * @brief txrx worker thread definition.  This is the parent / interface
 */
#ifndef TXRX_WORKER_H_
#define TXRX_WORKER_H_

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

#include "concurrentqueue.h"
#include "config.h"
#include "message.h"

class TxRxWorker {
 public:
  static constexpr bool kDebugTxMemory = false;

  TxRxWorker(size_t core_offset, size_t tid, size_t interface_count,
             size_t interface_offset, size_t channels_per_interface,
             Config* const config, size_t* rx_frame_start,
             moodycamel::ConcurrentQueue<EventData>* event_notify_q,
             moodycamel::ConcurrentQueue<EventData>* tx_pending_q,
             moodycamel::ProducerToken& tx_producer,
             moodycamel::ProducerToken& notify_producer,
             std::vector<RxPacket>& rx_memory, std::byte* const tx_memory,
             std::mutex& sync_mutex, std::condition_variable& sync_cond,
             std::atomic<bool>& can_proceed);
  TxRxWorker() = delete;

  virtual ~TxRxWorker();

  virtual void Start();
  virtual void Stop();
  virtual void DoTxRx() = 0;

  inline size_t Id() const { return tid_; }
  inline bool Started() const { return started_; }
  inline bool Running() const { return running_; }

 protected:
  void WaitSync();
  inline Config* Configuration() { return cfg_; }
  bool NotifyComplete(const EventData& complete_event);
  std::vector<EventData> GetPendingTxEvents(size_t max_events = 0);
  RxPacket& GetRxPacket();
  void ReturnRxPacket(RxPacket& unused_packet);
  Packet* GetTxPacket(size_t frame, size_t symbol, size_t ant);
  Packet* GetUlTxPacket(size_t frame, size_t symbol, size_t ant);

  const size_t tid_;
  const size_t core_offset_;
  const size_t num_interfaces_;
  const size_t interface_offset_;
  const size_t channels_per_interface_;
  size_t* const rx_frame_start_;
  bool running_;

  ///Owned by the parent TxRx object for sync
  std::mutex& mutex_;
  std::condition_variable& cond_;
  std::atomic<bool>& can_proceed_;

 private:
  Config* const cfg_;
  std::thread thread_;

  size_t rx_memory_idx_;
  std::vector<RxPacket>& rx_memory_;
  std::byte* const tx_memory_;

  moodycamel::ConcurrentQueue<EventData>* event_notify_q_;
  moodycamel::ConcurrentQueue<EventData>* tx_pending_q_;
  //foreign producer of tx messages (used for TX only)
  moodycamel::ProducerToken& tx_producer_token_;
  //local producer of notification messages (used for TX and RX)
  moodycamel::ProducerToken& notify_producer_token_;

  bool started_;
};
#endif  // TXRX_WORKER_H_
