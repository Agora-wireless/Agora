/**
 * @file txrx_worker_usrp.h
 * @brief Implementation of PacketTxRx datapath functions for communicating
 * with real hardware
 */

#ifndef TXRX_WORKER_USRP_H_
#define TXRX_WORKER_USRP_H_

#include <memory>
#include <vector>

#include "buffer.h"
#include "radio_lib.h"
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
                 RadioConfig* const radio_config);

  ~TxRxWorkerUsrp() final override;

  void DoTxRx() final override;

 private:
  TxRxWorkerUsrp() = delete;
  int DequeueSend();
  int DequeueSend(int frame_id, int symbol_id);
  Packet* RecvEnqueue(size_t radio_id, size_t rx_slot, size_t frame_id,
                      size_t symbol_id);

  long long rx_time_bs_;
  long long tx_time_bs_;
  // This object is created / owned by the parent process
  RadioConfig* const radio_config_;
};
#endif  // TXRX_WORKER_USRP_H_
