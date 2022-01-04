/**
 * @file txrx_worker_argos.h
 * @brief Implementation of PacketTxRx datapath functions for communicating
 * with real Argos hardware
 */

#ifndef TXRX_WORKER_ARGOS_H_
#define TXRX_WORKER_ARGOS_H_

#include <memory>
#include <vector>

#include "buffer.h"
#include "radio_lib.h"
#include "txrx_worker.h"

class TxRxWorkerArgos : public TxRxWorker {
 public:
  TxRxWorkerArgos(size_t core_offset, size_t tid, size_t interface_count,
                  size_t interface_offset, Config* const config,
                  size_t* rx_frame_start,
                  moodycamel::ConcurrentQueue<EventData>* event_notify_q,
                  moodycamel::ConcurrentQueue<EventData>* tx_pending_q,
                  moodycamel::ProducerToken& tx_producer,
                  moodycamel::ProducerToken& notify_producer,
                  std::vector<RxPacket>& rx_memory, std::byte* const tx_memory,
                  RadioConfig* const radio_config);

  ~TxRxWorkerArgos() final override;
  void DoTxRx() final override;

 private:
  TxRxWorkerArgos() = delete;
  size_t DequeueSend();
  std::vector<Packet*> RecvEnqueue(size_t interface_id, size_t rx_slot);

  // This object is created / owned by the parent process
  RadioConfig* const radio_config_;
};
#endif  // TXRX_WORKER_SIM_H_
