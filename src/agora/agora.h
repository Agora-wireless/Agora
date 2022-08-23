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
#include "dobeamweights.h"
#include "dodecode.h"
#include "dodemul.h"
#include "doencode.h"
#include "dofft.h"
#include "doifft.h"
#include "doprecode.h"
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
  enum ScheduleProcessingFlags : uint8_t {
    kNone = 0,
    kUplinkComplete = 0x1,
    kDownlinkComplete = 0x2,
    kProcessingComplete = (kUplinkComplete + kDownlinkComplete)
  };

  /// Determines if all the work has been completed on the frame_id
  /// Completion is determined based on the ifft, tx, decode, and tomac
  /// counters. If frame processing is complete.  All of the work counters are
  /// reset and the cur_proc_frame_id_ is incremented.  Returns true if all
  /// processing is complete AND the frame_id is the last frame to test. False
  /// otherwise.
  bool CheckFrameComplete(size_t frame_id);

  /// Increments the cur_sche_frame_id when all ScheduleProcessingFlags have
  /// been acheived.
  void CheckIncrementScheduleFrame(size_t frame_id,
                                   ScheduleProcessingFlags completed);

  void WorkerFft(int tid);
  void WorkerBeam(int tid);
  void WorkerDemul(int tid);
  void WorkerDecode(int tid);
  void Worker(int tid);

  void CreateThreads();  /// Launch worker threads

  void InitializeQueues();
  void InitializeCounters();
  void InitializeThreads();
  void FreeQueues();

  void SaveDecodeDataToFile(int frame_id);
  void SaveTxDataToFile(int frame_id);

  void HandleEventFft(size_t tag);
  void UpdateRxCounters(size_t frame_id, size_t symbol_id);

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

  /// Fetch the concurrent queue for this event type
  moodycamel::ConcurrentQueue<EventData>* GetConq(EventType event_type,
                                                  size_t qid) {
    return &sched_info_arr_[qid][static_cast<size_t>(event_type)].concurrent_q_;
  }

  /// Fetch the producer token for this event type
  moodycamel::ProducerToken* GetPtok(EventType event_type, size_t qid) const {
    return sched_info_arr_[qid][static_cast<size_t>(event_type)].ptok_;
  }

  /// Return a string containing the sizes of the FFT queues
  std::string GetFftQueueSizesString() const {
    std::ostringstream ret;
    ret << "[";
    for (size_t i = 0; i < kFrameWnd; i++) {
      ret << std::to_string(fft_queue_arr_[i].size()) << " ";
    }
    ret << "]";
    return ret.str();
  }

  // Worker thread i runs on core base_worker_core_offset + i
  const size_t base_worker_core_offset_;

  Config* const config_;
  size_t fft_created_count_;
  size_t max_equaled_frame_ = SIZE_MAX;
  std::unique_ptr<PacketTxRx> packet_tx_rx_;

  // The thread running MAC layer functions
  std::unique_ptr<MacThreadBaseStation> mac_thread_;
  // Handle for the MAC thread
  std::thread mac_std_thread_;
  std::vector<std::thread> workers_;

  std::unique_ptr<Stats> stats_;
  std::unique_ptr<PhyStats> phy_stats_;

  //Agora Buffer containment
  std::unique_ptr<AgoraBuffer> agora_memory_;

  // Counters related to various modules
  FrameCounters pilot_fft_counters_;
  FrameCounters uplink_fft_counters_;
  FrameCounters beam_counters_;
  FrameCounters demul_counters_;
  FrameCounters decode_counters_;
  FrameCounters encode_counters_;
  FrameCounters precode_counters_;
  FrameCounters ifft_counters_;
  FrameCounters tx_counters_;
  FrameCounters tomac_counters_;
  FrameCounters mac_to_phy_counters_;
  FrameCounters rc_counters_;
  RxCounters rx_counters_;
  size_t beam_last_frame_ = SIZE_MAX;
  size_t rc_last_frame_ = SIZE_MAX;
  size_t ifft_next_symbol_ = 0;

  // Agora schedules and processes a frame in FIFO order
  // cur_proc_frame_id is the frame that is currently being processed.
  // cur_sche_frame_id is the frame that is currently being scheduled.
  // A frame's schduling finishes before processing ends, so the two
  // variables are possible to have different values.
  size_t cur_proc_frame_id_ = 0;
  size_t cur_sche_frame_id_ = 0;

  // The frame index for a symbol whose FFT is done
  std::vector<size_t> fft_cur_frame_for_symbol_;
  // The frame index for a symbol whose encode is done
  std::vector<size_t> encode_cur_frame_for_symbol_;
  // The frame index for a symbol whose IFFT is done
  std::vector<size_t> ifft_cur_frame_for_symbol_;

  // The frame index for a symbol whose precode is done
  std::vector<size_t> precode_cur_frame_for_symbol_;

  // Per-frame queues of delayed FFT tasks. The queue contains offsets into
  // TX/RX buffers.
  std::array<std::queue<fft_req_tag_t>, kFrameWnd> fft_queue_arr_;

  struct SchedInfoT {
    moodycamel::ConcurrentQueue<EventData> concurrent_q_;
    moodycamel::ProducerToken* ptok_;
  };
  SchedInfoT sched_info_arr_[kScheduleQueues][kNumEventTypes];

  // Master thread's message queue for receiving packets
  moodycamel::ConcurrentQueue<EventData> message_queue_;

  // Master-to-worker queue for MAC
  moodycamel::ConcurrentQueue<EventData> mac_request_queue_;

  // Worker-to-master queue for MAC
  moodycamel::ConcurrentQueue<EventData> mac_response_queue_;

  // Master thread's message queue for event completion from Doers;
  moodycamel::ConcurrentQueue<EventData> complete_task_queue_[kScheduleQueues];
  moodycamel::ProducerToken* worker_ptoks_ptr_[kMaxThreads][kScheduleQueues];

  moodycamel::ProducerToken* rx_ptoks_ptr_[kMaxThreads];
  moodycamel::ProducerToken* tx_ptoks_ptr_[kMaxThreads];

  uint8_t schedule_process_flags_;

  std::queue<size_t> encode_deferral_;
  std::array<std::shared_ptr<CsvLog::MatLogger>, CsvLog::kMatLogs> mat_loggers_;
};

#endif  // AGORA_H_
