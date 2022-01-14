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

namespace TxRxWorkerRx {
struct RxParameters {
  size_t symbol_ = 0;
  size_t interface_ = 0;
};
}  // namespace TxRxWorkerRx

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
                  std::mutex& sync_mutex, std::condition_variable& sync_cond,
                  std::atomic<bool>& can_proceed, RadioConfig& radio_config);

  ~TxRxWorkerArgos() final;
  void DoTxRx() final;

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

  TxRxWorkerRx::RxParameters UpdateRxInterface(
      const TxRxWorkerRx::RxParameters& last_rx);
  // This object is created / owned by the parent process
  RadioConfig& radio_config_;
};
#endif  // TXRX_WORKER_SIM_H_
