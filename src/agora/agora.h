#ifndef AGORA_HEAD
#define AGORA_HEAD

#include <pthread.h>
#include <unistd.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <queue>
#include <system_error>
#include <vector>

#include "buffer.inc"
#include "concurrent_queue_wrapper.inc"
#include "concurrentqueue.h"
#include "config.h"
#include "docoding.h"
#include "dodemul.h"
#include "dofft.h"
#include "doprecode.h"
#include "dozf.h"
#include "mac_thread.h"
#include "memory_manage.h"
#include "phy_stats.h"
#include "signal_handler.h"
#include "stats.h"
#include "txrx.h"
#include "utils.h"

class Agora {
 public:
  // Dequeue batch size, used to reduce the overhead of dequeue in main thread
  static const int kDequeueBulkSizeTXRX = 8;
  static const int kDequeueBulkSizeWorker = 4;

  static const int kMaxWorkerNum = 50;  // Max number of worker threads allowed

  Agora(
      Config* /*cfg*/);  /// Create an Agora object and start the worker threads
  ~Agora();

  void Start();  /// The main Agora event loop
  void Stop();

  void WorkerFft(int tid);
  void WorkerZf(int tid);
  void WorkerDemul(int tid);
  void WorkerDecode(int tid);
  void Worker(int tid);

  void CreateThreads();  /// Launch worker threads

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

  /**
   * @brief Schedule LDPC decoding or encoding over code blocks
   * @param task_type Either LDPC decoding or LDPC encoding
   * @param frame_id The monotonically increasing frame ID
   * @param symbol_idx The index of the symbol among uplink symbols for LDPC
   * decoding, and among downlink symbols for LDPC encoding
   */
  void ScheduleCodeblocks(EventType event_type, size_t frame_id,
                          size_t symbol_idx);

  void ScheduleUsers(EventType event_type, size_t frame_id, size_t symbol_id);

  // Send current frame's SNR measurements from PHY to MAC
  void SendSnrReport(EventType event_type, size_t frame_id, size_t symbol_id);

  void InitializeQueues();
  void InitializeUplinkBuffers();
  void InitializeDownlinkBuffers();
  void FreeUplinkBuffers();
  void FreeDownlinkBuffers();

  void SaveDecodeDataToFile(int frame_id);
  void SaveTxDataToFile(int frame_id);
  void GetEqualData(float** ptr, int* size);

  // Flags that allow developer control over Agora internals
  struct {
    // Before exiting, save LDPC-decoded or demodulated data to a file
    bool enable_save_decode_data_to_file_ = false;

    // Before exiting, save data sent on downlink to a file
    bool enable_save_tx_data_to_file_ = false;
  } flags_;

 private:
  /// Fetch the concurrent queue for this event type
  moodycamel::ConcurrentQueue<EventData>* GetConq(EventType event_type,
                                                  size_t qid) {
    return &sched_info_arr_[qid][static_cast<size_t>(event_type)].concurrent_q_;
  }

  /// Fetch the producer token for this event type
  moodycamel::ProducerToken* GetPtok(EventType event_type, size_t qid) {
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

  Config* config_;
  size_t fft_created_count_;
  size_t max_equaled_frame_ = SIZE_MAX;
  std::unique_ptr<PacketTXRX> packet_tx_rx_;

  MacThread* mac_thread_;       // The thread running MAC layer functions
  std::thread mac_std_thread_;  // Handle for the MAC thread
  std::thread worker_std_threads_[kMaxWorkerNum];  // Handle for worker threads

  Stats* stats_;
  PhyStats* phy_stats_;
  pthread_t* task_threads_;

  /*****************************************************
   * Buffers
   *****************************************************/

  /* Uplink */
  size_t socket_buffer_size_;  // RX buffer size per socket RX thread

  // Max number of packets that can be buffered in one RX thread
  size_t socket_buffer_status_size_;

  // Received data buffers
  // 1st dimension: number of socket RX threads
  // 2nd dimension: socket buffer size
  Table<char> socket_buffer_;

  // Status of received data buffers
  // 1st dimension: number of socket RX threads
  // 2nd dimension: socket buffer status size
  Table<int> socket_buffer_status_;

  // Preliminary CSI buffers. Each buffer has [number of antennas] rows and
  // [number of OFDM data subcarriers] columns.
  PtrGrid<kFrameWnd, kMaxUEs, complex_float> csi_buffers_;

  // Data symbols after FFT
  // 1st dimension: kFrameWnd * uplink data symbols per frame
  // 2nd dimension: number of antennas * number of OFDM data subcarriers
  //
  // 2nd dimension data order: 32 blocks each with 32 subcarriers each:
  // subcarrier 1 -- 32 of antennas, subcarrier 33 -- 64 of antennas, ...,
  // subcarrier 993 -- 1024 of antennas.
  Table<complex_float> data_buffer_;

  // Calculated uplink zeroforcing detection matrices. Each matrix has
  // [number of antennas] rows and [number of UEs] columns.
  PtrGrid<kFrameWnd, kMaxDataSCs, complex_float> ul_zf_matrices_;

  // Data after equalization
  // 1st dimension: kFrameWnd * uplink data symbols per frame
  // 2nd dimension: number of OFDM data subcarriers * number of UEs
  Table<complex_float> equal_buffer_;

  // Data after demodulation. Each buffer has kMaxModType * number of OFDM
  // data subcarriers
  PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t> demod_buffers_;

  // Data after LDPC decoding. Each buffer [decoded bytes per UE] bytes.
  PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, uint8_t> decoded_buffer_;

  Table<complex_float> ue_spec_pilot_buffer_;

  // Counters related to various modules
  FrameCounters fft_counters_;
  FrameCounters zf_counters_;
  FrameCounters demul_counters_;
  FrameCounters decode_counters_;
  FrameCounters encode_counters_;
  FrameCounters precode_counters_;
  FrameCounters ifft_counters_;
  FrameCounters tx_counters_;
  FrameCounters tomac_counters_;
  FrameCounters frommac_counters_;
  FrameCounters rc_counters_;
  RxCounters rx_counters_;
  size_t zf_last_frame_ = SIZE_MAX;
  size_t rc_last_frame_ = SIZE_MAX;

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

  // Per-frame queues of delayed FFT tasks. The queue contains offsets into
  // TX/RX buffers.
  std::array<std::queue<fft_req_tag_t>, kFrameWnd> fft_queue_arr_;

  // Data for IFFT
  // 1st dimension: kFrameWnd * number of antennas * number of
  // data symbols per frame
  // 2nd dimension: number of OFDM carriers (including non-data carriers)
  Table<complex_float> dl_ifft_buffer_;

  // Calculated uplink zeroforcing detection matrices. Each matrix has
  // [number of UEs] rows and [number of antennas] columns.
  PtrGrid<kFrameWnd, kMaxDataSCs, complex_float> dl_zf_matrices_;

  // 1st dimension: kFrameWnd
  // 2nd dimension: number of OFDM data subcarriers * number of antennas
  Table<complex_float> calib_ul_buffer_;
  Table<complex_float> calib_dl_buffer_;

  // 1st dimension: kFrameWnd * number of data symbols per frame
  // 2nd dimension: number of OFDM data subcarriers * number of UEs
  Table<int8_t> dl_encoded_buffer_;

  // 1st dimension: kFrameWnd * number of DL data symbols per frame
  // 2nd dimension: number of OFDM data subcarriers * number of UEs
  Table<uint8_t> dl_bits_buffer_;

  // 1st dimension: number of UEs
  // 2nd dimension: number of OFDM data subcarriers * kFrameWnd
  //                * number of DL data symbols per frame
  // Use different dimensions from dl_bits_buffer_ to avoid cache false sharing
  Table<uint8_t> dl_bits_buffer_status_;

  /**
   * Data for transmission
   *
   * Number of downlink socket buffers and status entries:
   * kFrameWnd * symbol_num_perframe * BS_ANT_NUM
   *
   * Size of each downlink socket buffer entry: packet_length bytes
   * Size of each downlink socket buffer status entry: one integer
   */
  char* dl_socket_buffer_;
  int* dl_socket_buffer_status_;

  struct SchedInfoT {
    moodycamel::ConcurrentQueue<EventData> concurrent_q_;
    moodycamel::ProducerToken* ptok_;
  };
  SchedInfoT sched_info_arr_[2][kNumEventTypes];

  // Master thread's message queue for receiving packets
  moodycamel::ConcurrentQueue<EventData> message_queue_;

  // Master-to-worker queue for MAC
  moodycamel::ConcurrentQueue<EventData> mac_request_queue_;

  // Worker-to-master queue for MAC
  moodycamel::ConcurrentQueue<EventData> mac_response_queue_;

  // Master thread's message queue for event completion from Doers;
  moodycamel::ConcurrentQueue<EventData> complete_task_queue_[2];
  moodycamel::ProducerToken* worker_ptoks_ptr_[kMaxThreads][2];

  moodycamel::ProducerToken* rx_ptoks_ptr_[kMaxThreads];
  moodycamel::ProducerToken* tx_ptoks_ptr_[kMaxThreads];
};

#endif
