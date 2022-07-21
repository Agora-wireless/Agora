/**
 * @file agora.h
 * @brief Declaration file for the main agora class
 */

#ifndef AGORA_H_
#define AGORA_H_

#include <unistd.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <queue>
#include <system_error>
#include <vector>

#include "buffer.h"
#include "concurrent_queue_wrapper.h"
#include "concurrentqueue.h"
#include "config.h"
#include "dodecode.h"
#include "dodemul.h"
#include "doencode.h"
#include "dofft.h"
#include "doifft.h"
#include "doprecode.h"
#include "dozf.h"
#include "mac_thread_basestation.h"
#include "mat_logger.h"
#include "memory_manage.h"
#include "packet_txrx.h"
#include "phy_stats.h"
#include "signal_handler.h"
#include "stats.h"
#include "utils.h"

class Agora {
 public:
  explicit Agora(
      Config* /*cfg*/);  /// Create an Agora object and start the worker threads
  ~Agora();

  void Start();  /// The main Agora event loop
  void Stop();
  void GetEqualData(float** ptr, int* size);

  // Flags that allow developer control over Agora internals
  struct {
    //     void getEqualData(float** ptr, int* size);Before exiting, save
    //     LDPC-decoded or demodulated data to a file
    bool enable_save_decode_data_to_file_ = false;

    // Before exiting, save data sent on downlink to a file
    bool enable_save_tx_data_to_file_ = false;
  } flags_;

 private:
  /// Determines if all the work has been completed on the frame_id
  /// Completion is determined based on the ifft, tx, decode, and tomac
  /// counters. If frame processing is complete.  All of the work counters are
  /// reset and the cur_proc_frame_id_ is incremented.  Returns true if all
  /// processing is complete AND the frame_id is the last frame to test. False
  /// otherwise.
  bool CheckFrameComplete(size_t frame_id);

  void WorkerFft(int tid);
  void WorkerZf(int tid);
  void WorkerDemul(int tid);
  void WorkerDecode(int tid);
  void Worker(int tid);

  void CreateThreads();  /// Launch worker threads

  void InitializeQueues();
  void InitializeUplinkBuffers();
  void InitializeDownlinkBuffers();
  void FreeQueues();
  void FreeUplinkBuffers();
  void FreeDownlinkBuffers();

  void SaveDecodeDataToFile(int frame_id);
  void SaveTxDataToFile(int frame_id);

  void HandleEventFft(size_t tag);
  void UpdateRxCounters(size_t frame_id, size_t symbol_id);
  void PrintPerFrameDone(PrintType print_type, size_t frame_id);
  void PrintPerSymbolDone(PrintType print_type, size_t frame_id,
                          size_t symbol_id);
  void PrintPerTaskDone(PrintType print_type, size_t frame_id, size_t symbol_id,
                        size_t ant_or_sc_id);

  /// Update Agora's RAN config parameters
  void UpdateRanConfig(RanConfig rc);

  void ScheduleSubcarriers(EventType event_type, size_t frame_id,
                           size_t symbol_id);
  void ScheduleAntennas(EventType event_type, size_t frame_id,
                        size_t symbol_id);
  void ScheduleAntennasTX(size_t frame_id, size_t symbol_id);
  void ScheduleDownlinkProcessing(size_t frame_id);

  /**
   * @brief Schedule LDPC decoding or encoding over code blocks
   * @param task_type Either LDPC decoding or LDPC encoding
   * @param frame_id The monotonically increasing frame ID
   * @param symbol_idx The index of the symbol among uplink symbols for LDPC
   * decoding, and among downlink symbols for LDPC encoding
   */
  void ScheduleCodeblocks(EventType event_type, Direction dir, size_t frame_id,
                          size_t symbol_idx);

  void ScheduleUsers(EventType event_type, size_t frame_id, size_t symbol_id);

  // Send current frame's SNR measurements from PHY to MAC
  void SendSnrReport(EventType event_type, size_t frame_id, size_t symbol_id);
};

#endif  // AGORA_H_
