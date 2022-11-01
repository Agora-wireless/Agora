/**
 * @file txrx_worker_hw.h
 * @brief Implementation of PacketTxRx datapath functions for communicating
 * with real iris / faros hardware
 */

#ifndef TXRX_WORKER_HW_H_
#define TXRX_WORKER_HW_H_

#include <memory>
#include <vector>

#include "message.h"
#include "radio_lib.h"
#include "rx_status_tracker.h"
#include "txrx_worker.h"

namespace TxRxWorkerRx {
struct RxParameters {
  size_t symbol_ = 0;
  size_t interface_ = 0;
};

struct RxTimeTracker {
  size_t start_ticks_;
  size_t end_ticks_;
};
}  // namespace TxRxWorkerRx

class TxRxWorkerHw : public TxRxWorker {
 public:
  TxRxWorkerHw(size_t core_offset, size_t tid, size_t interface_count,
               size_t interface_offset, Config* const config,
               size_t* rx_frame_start,
               moodycamel::ConcurrentQueue<EventData>* event_notify_q,
               moodycamel::ConcurrentQueue<EventData>* tx_pending_q,
               moodycamel::ProducerToken& tx_producer,
               moodycamel::ProducerToken& notify_producer,
               std::vector<RxPacket>& rx_memory, std::byte* const tx_memory,
               std::mutex& sync_mutex, std::condition_variable& sync_cond,
               std::atomic<bool>& can_proceed, RadioConfig& radio_config);
  TxRxWorkerHw() = delete;
  ~TxRxWorkerHw() final;
  void DoTxRx() final;

 private:
  size_t DoTx(long long time0);
  std::vector<RxPacket*> DoRx(size_t interface_id, size_t& global_frame_id,
                              size_t& global_symbol_id);

  void ScheduleTxInit(size_t frames_to_schedule, long long time0);
  void TxDownlinkZeros(size_t frame_id, size_t radio_id, long long time0);

  void TxReciprocityCalibPilots(size_t frame_id, size_t radio_id,
                                long long time0);

  void TxBeaconHw(size_t frame_id, size_t interface_id, long long time0);
  bool IsTxSymbolNext(size_t radio_id, size_t current_symbol);
  Radio::TxFlags GetTxFlags(size_t radio_id, size_t tx_symbol_id);
  long long int GetHwTime();

  bool IsRxSymbol(size_t interface, size_t symbol_id);

  //DoRx helper routines
  void InitRxStatus();
  void ResetRxStatus(size_t interface, bool reuse_memory);

  TxRxWorkerRx::RxParameters UpdateRxInterface(
      const TxRxWorkerRx::RxParameters& last_rx);

  void PrintRxSymbolTiming(std::vector<TxRxWorkerRx::RxTimeTracker>& rx_times,
                           size_t current_frame, size_t current_symbol,
                           size_t next_symbol);

  // This object is created / owned by the parent process
  RadioConfig& radio_config_;
  size_t program_start_ticks_;
  const double freq_ghz_;

  std::vector<std::complex<int16_t>> zeros_;

  //For each interface.
  std::vector<TxRxWorkerRx::RxStatusTracker> rx_status_;
  std::vector<bool> first_symbol_;
};
#endif  // TXRX_WORKER_SIM_H_
