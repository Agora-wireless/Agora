/**
 * @file phy-ue.h
 * @brief Declaration file for the phy ue class
 */

#ifndef PHY_UE_H_
#define PHY_UE_H_

#include <array>
#include <list>
#include <queue>
#include <thread>
#include <vector>

#include "common_typedef_sdk.h"
#include "comms-lib.h"
#include "concurrent_queue_wrapper.h"
#include "concurrentqueue.h"
#include "config.h"
#include "datatype_conversion.h"
#include "mac_thread_client.h"
#include "message.h"
#include "modulation.h"
#include "packet_txrx.h"
#include "phy_stats.h"
#include "recorder_thread.h"
#include "simd_types.h"
#include "stats.h"
#include "ue_worker.h"

class PhyUe {
 public:
  enum class FrameTasksFlags : std::uint8_t {
    kNoWorkComplete = 0x00,
    kDownlinkComplete = 0x01,
    kUplinkTxComplete = 0x02,
    kMacTxComplete = 0x04,
    kFrameComplete = (kDownlinkComplete | kMacTxComplete | kUplinkTxComplete)
  };

  explicit PhyUe(Config* config);
  ~PhyUe();

  void Start();
  void Stop();

  void GetEqualData(float** ptr, int* size, int /*ue_id*/);
  void GetDemulData(long long** ptr, int* size);

 private:
  void PrintPerTaskDone(PrintType print_type, size_t frame_id, size_t symbol_id,
                        size_t ant);
  void PrintPerSymbolDone(PrintType print_type, size_t frame_id,
                          size_t symbol_id);
  void PrintPerFrameDone(PrintType print_type, size_t frame_id);

  void ReceiveDownlinkSymbol(Packet* rx_packet, size_t tag);
  void ScheduleDefferedDownlinkSymbols(size_t frame_id);
  void ClearCsi(size_t frame_id);

  std::vector<std::queue<EventData>> rx_downlink_deferral_;
  std::unique_ptr<Stats> stats_;
  std::unique_ptr<PhyStats> phy_stats_;
  RxCounters rx_counters_;

  /*****************************************************
   * Downlink
   *****************************************************/
  void InitializeDownlinkBuffers();

  /*****************************************************
   * Uplink
   *****************************************************/
  void InitializeUplinkBuffers();

  /* Add tasks into task queue based on event type */
  void ScheduleTask(EventData do_task,
                    moodycamel::ConcurrentQueue<EventData>* in_queue,
                    moodycamel::ProducerToken const& ptok);
  void ScheduleWork(EventData do_task);

  std::unique_ptr<moodycamel::ProducerToken> work_producer_token_;

  void InitializeVarsFromCfg();

  void FrameInit(size_t frame);
  // Tracks the tasks completed for the frame
  bool FrameComplete(size_t frame, FrameTasksFlags complete);

  void FreeUplinkBuffers();
  void FreeDownlinkBuffers();

  Config* config_;
  size_t dl_pilot_symbol_perframe_;
  size_t ul_data_symbol_perframe_;
  size_t dl_data_symbol_perframe_;
  size_t ul_symbol_perframe_;
  size_t dl_symbol_perframe_;
  size_t rx_thread_num_;

  std::array<std::uint8_t, kFrameWnd> frame_tasks_;

  // The thread running MAC layer functions
  std::unique_ptr<MacThreadClient> mac_thread_;
  // Handle for the MAC thread
  std::thread mac_std_thread_;

  // The frame ID of the next MAC packet we expect to receive from the MAC
  // thread
  size_t expected_frame_id_from_mac_ = 0;
  size_t current_frame_user_num_ = 0;

  // next_processed_frame_[i] is the next frame index on the uplink
  // to be processed and transmitted by the PHY for UE #i
  std::array<size_t, kMaxUEs> next_frame_processed_ = {};

  /*****************************************************
   * Uplink
   *****************************************************/
  /**
   * Transmit data
   *
   * Number of transmit buffers (size = packet_length) and buffer status
   * entries: TX_THREAD_NUM * TX_BUFFER_FRAME_NUM * UE_NUM * DL_SYM_PER_FRAME
   */
  char* tx_buffer_;
  size_t tx_buffer_size_;

  /**
   * Data for IFFT, (prefix added)
   * First dimension: IFFT_buffer_block_num = BS_ANT_NUM *
   *   dl_data_symbol_perframe * kFrameWnd
   * Second dimension: OFDM_CA_NUM
   */
  Table<complex_float> ifft_buffer_;

  /**
   * Data before modulation
   * First dimension: data_symbol_num_perframe * kFrameWnd
   * Second dimension: OFDM_CA_NUM * UE_NUM
   */
  Table<int8_t> ul_bits_buffer_;
  Table<int8_t> ul_bits_buffer_status_;
  size_t ul_bits_buffer_size_;

  Table<int8_t> ul_syms_buffer_;
  size_t ul_syms_buffer_size_;
  /**
   * Data after modulation
   * First dimension: data_symbol_num_perframe * kFrameWnd
   * Second dimension: OFDM_CA_NUM * UE_NUM
   */
  Table<complex_float> modul_buffer_;

  // Remote unit
  std::unique_ptr<PacketTxRx> ru_;

  /**
   * Received data
   *
   * Number of RX buffers (size = packet_length) and buffer status
   * entries: RX_THREAD_NUM * RX_BUFFER_FRAME_NUM * BS_ANT_NUM *
   * symbol_num_perframe
   */
  Table<char> rx_buffer_;
  size_t rx_buffer_size_;

  /**
   * Data for FFT, after time sync (prefix removed)
   * First dimension: FFT_buffer_block_num = BS_ANT_NUM *
   * symbol_num_perframe * kFrameWnd Second dimension:
   * OFDM_CA_NUM
   */
  Table<complex_float> fft_buffer_;

  /**
   * Estimated CSI data
   * First dimension: OFDM_CA_NUM * kFrameWnd
   * Second dimension: BS_ANT_NUM * UE_NUM
   */
  Table<complex_float> csi_buffer_;

  /**
   * Data after equalization
   * First dimension: data_symbol_num_perframe * kFrameWnd
   * Second dimension: OFDM_CA_NUM * UE_NUM
   */
  std::vector<SimdAlignCxFltVector> equal_buffer_;

  // Data after demodulation. Each buffer has kMaxModType * number of OFDM
  // data subcarriers
  PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t> demod_buffer_;

  // Data after LDPC decoding. Each buffer [decoded bytes per UE] bytes.
  PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t> decoded_buffer_;

  std::vector<size_t> non_null_sc_ind_;
  std::vector<std::vector<std::complex<float>>> ue_pilot_vec_;

  // Communication queues
  moodycamel::ConcurrentQueue<EventData> complete_queue_;
  moodycamel::ConcurrentQueue<EventData> work_queue_;

  moodycamel::ConcurrentQueue<EventData> tx_queue_;
  moodycamel::ConcurrentQueue<EventData> to_mac_queue_;

  // std::vector<std::thread> worker_threads_;
  std::vector<std::unique_ptr<UeWorker>> workers_;

  moodycamel::ProducerToken* rx_ptoks_ptr_[kMaxThreads];
  moodycamel::ProducerToken* tx_ptoks_ptr_[kMaxThreads];
  moodycamel::ProducerToken* mac_rx_ptoks_ptr_[kMaxThreads];
  moodycamel::ProducerToken* mac_tx_ptoks_ptr_[kMaxThreads];

  // all checkers
  FrameCounters tx_counters_;
  // Downlink (Rx)
  FrameCounters decode_counters_;
  FrameCounters demul_counters_;
  FrameCounters fft_dldata_counters_;
  FrameCounters fft_dlpilot_counters_;
  // Uplink (Tx)
  FrameCounters encode_counter_;
  FrameCounters modulation_counters_;

  struct UeTxVars {
    FrameCounters ifft_counters_;
    std::list<size_t> tx_ready_frames_;
    size_t tx_pending_frame_;
  };
  std::vector<UeTxVars> ue_tracker_;

  FrameCounters tomac_counters_;

  size_t max_equaled_frame_ = 0;
  std::vector<std::unique_ptr<Agora_recorder::RecorderThread>> recorders_;
};
#endif  // PHY_UE_H_
