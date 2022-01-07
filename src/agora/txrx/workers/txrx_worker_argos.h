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
  size_t DequeueSend(long long time0);
  std::vector<Packet*> RecvEnqueue(size_t interface_id, size_t global_frame_id,
                                   size_t global_symbol_id);

  void TxReciprocityCalibPilots(size_t frame_id, size_t radio_id,
                                long long time0);

  void TxBeaconHw(size_t frame_id, size_t interface_id, long long time0);
  bool IsTxSymbolNext(size_t radio_id, size_t current_symbol);
  int GetTxFlags(size_t radio_id, size_t tx_symbol_id);

  bool IsRxSymbol(size_t interface, size_t symbol_id);
  size_t UpdateRxInterface(size_t last_interface, size_t last_rx_symbol);
  // This object is created / owned by the parent process
  RadioConfig* const radio_config_;
};
#endif  // TXRX_WORKER_SIM_H_
