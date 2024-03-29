/**
 * @file txrx_worker_usrp.h
 * @brief Implementation of PacketTxRx datapath functions for communicating
 * with usrp hardware
 */

#ifndef TXRX_WORKER_USRP_H_
#define TXRX_WORKER_USRP_H_

#include <memory>
#include <vector>

#include "message.h"
#include "radio_set.h"
#include "txrx_worker.h"

class TxRxWorkerUsrp : public TxRxWorker {
 public:
  TxRxWorkerUsrp(size_t core_offset, size_t tid, size_t radio_hi,
                 size_t radio_lo, Config* const config, size_t* rx_frame_start,
                 moodycamel::ConcurrentQueue<EventData>* event_notify_q,
                 moodycamel::ConcurrentQueue<EventData>* tx_pending_q,
                 moodycamel::ProducerToken& tx_producer,
                 moodycamel::ProducerToken& notify_producer,
                 std::vector<RxPacket>& rx_memory, std::byte* const tx_memory,
                 std::mutex& sync_mutex, std::condition_variable& sync_cond,
                 std::atomic<bool>& can_proceed, RadioSet& radio_config);
  TxRxWorkerUsrp() = delete;
  ~TxRxWorkerUsrp() final;

  void DoTxRx() final;

 private:
  int DequeueSend();
  int DequeueSend(int frame_id, int symbol_id);
  std::vector<Packet*> RecvEnqueue(size_t radio_id, size_t frame_id,
                                   size_t symbol_id,
                                   const std::vector<void*>& discard_locs);

  long long GetRxTime(size_t radio_id, std::vector<void*>& rx_locs);
  long long DiscardRxFrames(size_t radio_id, size_t frames_to_ignore,
                            std::vector<void*>& rx_locs);
  void TxBeacon(size_t radio_id, size_t tx_frame_number,
                const std::vector<void*>& tx_locs, long long time0);

  long long rx_time_bs_;
  long long tx_time_bs_;
  // This object is created / owned by the parent process
  RadioSet& radio_config_;
};
#endif  // TXRX_WORKER_USRP_H_
