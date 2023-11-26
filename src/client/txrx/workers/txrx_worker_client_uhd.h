/**
 * @file txrx_worker_client_uhd.h
 * @brief Implementation of PacketTxRxRadio datapath functions for communicating
 * with real usrp hardware
 */

#ifndef TXRX_WORKER_CLIENT_UHD_H_
#define TXRX_WORKER_CLIENT_UHD_H_

#include <atomic>
#include <memory>
#include <vector>

#include "message.h"
#include "radio_set.h"
#include "rx_status_tracker.h"
#include "txrx_worker.h"

class TxRxWorkerClientUhd : public TxRxWorker {
 public:
  TxRxWorkerClientUhd(size_t core_offset, size_t tid, size_t interface_count,
                      size_t interface_offset, Config* const config,
                      size_t* rx_frame_start,
                      moodycamel::ConcurrentQueue<EventData>* event_notify_q,
                      moodycamel::ConcurrentQueue<EventData>* tx_pending_q,
                      moodycamel::ConcurrentQueue<EventData>* wired_ctrl_q,
                      moodycamel::ProducerToken& tx_producer,
                      moodycamel::ProducerToken& notify_producer,
                      std::vector<RxPacket>& rx_memory,
                      std::byte* const tx_memory, std::mutex& sync_mutex,
                      std::condition_variable& sync_cond,
                      std::atomic<bool>& can_proceed, RadioSet& radio_config);
  TxRxWorkerClientUhd() = delete;
  ~TxRxWorkerClientUhd() final;
  void DoTxRx() final;

 private:
  size_t DoTxThread(const long long time0);
  size_t DoTx(const long long time0);
  std::vector<Packet*> DoRx(size_t interface_id, size_t frame_id,
                            size_t symbol_id, long long& receive_time);

  ssize_t SyncBeacon(size_t local_interface, size_t sample_window);
  ssize_t FindSyncBeacon(const std::complex<int16_t>* check_data,
                         size_t sample_window, float corr_scale = 1.f);
  void AdjustRx(size_t local_interface, size_t discard_samples);
  bool IsRxSymbol(size_t symbol_id);
  void TxUplinkSymbols(size_t radio_id, size_t frame_id, long long time0);
  void TxPilot(size_t pilot_ant, size_t frame_id, long long time0);

  //DoRx helper routines
  void InitRxStatus();
  void ResetRxStatus(size_t interface, bool reuse_memory);

  // This object is created / owned by the parent process
  RadioSet& radio_;
  size_t program_start_ticks_;
  std::atomic<long long> rx_time_ue_;

  std::vector<std::vector<std::complex<int16_t>>> frame_zeros_;
  std::vector<std::vector<std::complex<int16_t>>> frame_storage_;
  std::vector<RxPacket> rx_frame_pkts_;
  std::vector<RxPacket*> rx_pkts_ptrs_;

  //For each interface.
  std::vector<TxRxWorkerRx::RxStatusTracker> rx_status_;
};
#endif  // TXRX_WORKER_CLIENT_UHD_H_
