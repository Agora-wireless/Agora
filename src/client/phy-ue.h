/**
 * @file phy-ue.h
 * @brief Declaration file for the phy ue class
 */

#ifndef PHY_UE_H_
#define PHY_UE_H_

#include <array>
#include <queue>
#include <thread>
#include <vector>

#include "buffer.h"
#include "comms-lib.h"
#include "concurrent_queue_wrapper.h"
#include "concurrentqueue.h"
#include "config.h"
#include "datatype_conversion.h"
#include "doencode.h"
#include "mac_thread.h"
#include "mkl_dfti.h"
#include "modulation.h"
#include "stats.h"
#include "txrx_client.h"

static const size_t kVectorAlignment = 64;

using myVec = std::vector<complex_float, boost::alignment::aligned_allocator<
                                             complex_float, kVectorAlignment>>;

class PhyUe {
 public:
  enum class FrameTasksFlags : std::uint8_t {
    kNoWorkComplete = 0x00,
    kDownlinkComplete = 0x01,
    // kUplinkComplete = 0x02,
    kUplinkTxComplete = 0x02,
    kMacTxComplete = 0x04,
    kFrameComplete = (kDownlinkComplete | kMacTxComplete | kUplinkTxComplete)
  };

  // dequeue bulk size, used to reduce the overhead of dequeue in main
  // thread
  static const int kDequeueBulkSizeTXRX = 8;

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

  void ReceiveDownlinkSymbol(struct Packet* rx_packet, size_t tag);
  void ScheduleDefferedDownlinkSymbols(size_t frame_id);
  void ClearCsi(size_t frame_id);
  std::vector<std::queue<EventData>> rx_downlink_deferral_;
  std::unique_ptr<Stats> stats_;
  RxCounters rx_counters_;

  /*****************************************************
   * Downlink
   *****************************************************/
  void InitializeDownlinkBuffers();

  /**
   * modulate data from nUEs and does spatial multiplexing by applying
   * beamweights
   */
  void DoEncodeUe(DoEncode* encoder, moodycamel::ProducerToken* ptok,
                  size_t tag);
  void DoModul(int /*tid*/, size_t /*tag*/);
  void DoIfft(int /*tid*/, size_t /*tag*/);

  /*****************************************************
   * Uplink
   *****************************************************/
  void InitializeUplinkBuffers();

  /**
   * Do FFT task for one OFDM symbol
   * @param tid: task thread index, used for selecting muplans and task ptok
   * @param offset: offset of the OFDM symbol in rx_buffer_
   * Buffers: rx_buffer_, fft_buffer_, csi_buffer_, ul_data_buffer_
   *     Input buffer: rx_buffer_
   *     Output buffer: csi_buffer_ if symbol is pilot
   *                    ul_data_buffer_ if symbol is data
   *     Intermediate buffer: fft_buffer_ (FFT_inputs, FFT_outputs)
   * Offsets:
   *     rx_buffer_:
   *         dim1: socket thread index: (offset / # of OFDM symbols per
   * thread) dim2: OFDM symbol index in this socket thread (offset - # of
   * symbols in previous threads) FFT_inputs, FFT_outputs: dim1: frame index
   * * # of OFDM symbols per frame + symbol index * # of atennas + antenna
   * index dim2: subcarrier index csi_buffer_: dim1: frame index * FFT size +
   * subcarrier index in the current frame dim2: user index * # of antennas +
   * antenna index ul_data_buffer_: dim1: frame index * # of data symbols
   * per frame + data symbol index dim2: transpose block index * block size
   * * # of antennas + antenna index * block size Event offset: frame index *
   * # of symbol per frame + symbol index Description:
   *     1. copy received data (one OFDM symbol) from rx_buffer to
   * fft_buffer_.FFT_inputs (remove CP)
   *     2. perform FFT on fft_buffer_.FFT_inputs and store results in
   * fft_buffer_.FFT_outputs
   *     3. if symbol is pilot, do channel estimation from
   * fft_buffer_.FFT_outputs to csi_buffer_ if symbol is data, copy data
   * from fft_buffer_.FFT_outputs to ul_data_buffer_ and do block transpose
   *     4. add an event to the message queue to infrom main thread the
   * completion of this task
   */
  void DoFftPilot(int /*tid*/, size_t /*tag*/);
  void DoFftData(int /*tid*/, size_t /*tag*/);

  /**
   * Do demodulation task for a block of subcarriers (demul_block_size)
   * @param tid: task thread index, used for selecting spm_buffer and task
   * ptok
   * @param offset: offset of the first subcarrier in the block in
   * ul_data_buffer_ Buffers: ul_data_buffer_, spm_buffer_, precoder_buffer_,
   * equal_buffer_, demul_buffer_ Input buffer: ul_data_buffer_,
   * precoder_buffer_ Output buffer: demul_buffer_ Intermediate buffer:
   * spm_buffer, equal_buffer_ Offsets: ul_data_buffer_: dim1: frame index * #
   * of data symbols per frame + data symbol index dim2: transpose block
   * index * block size * # of antennas + antenna index * block size
   *     spm_buffer:
   *         dim1: task thread index
   *         dim2: antenna index
   *     precoder_buffer_:
   *         dim1: frame index * FFT size + subcarrier index in the current
   * frame equal_buffer_, demul_buffer: dim1: frame index * # of data
   * symbols per frame + data symbol index dim2: subcarrier index * # of
   * users Event offset: offset Description:
   *     1. for each subcarrier in the block, block-wisely copy data from
   * ul_data_buffer_ to spm_buffer_
   *     2. perform equalization with data and percoder matrixes
   *     3. perform demodulation on equalized data matrix
   *     4. add an event to the message queue to infrom main thread the
   * completion of this task
   */
  void DoDemul(int /*tid*/, size_t /*tag*/);
  void DoDecode(int /*tid*/, size_t /*tag*/);

  void TaskThread(int tid);

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
  std::unique_ptr<MacThread> mac_thread_;
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
  int* tx_buffer_status_;

  size_t tx_buffer_size_;
  int tx_buffer_status_size_;

  /**
   * Data for IFFT, (prefix added)
   * First dimension: IFFT_buffer_block_num = BS_ANT_NUM *
   *   dl_data_symbol_perframe * kFrameWnd
   * Second dimension: OFDM_CA_NUM
   */
  Table<complex_float> ifft_buffer_;
  DFTI_DESCRIPTOR_HANDLE mkl_handle_;

  /**
   * Data before modulation
   * First dimension: data_symbol_num_perframe * kFrameWnd
   * Second dimension: OFDM_CA_NUM * UE_NUM
   */
  Table<uint8_t> ul_bits_buffer_;
  Table<uint8_t> ul_bits_buffer_status_;
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
  std::unique_ptr<RadioTxRx> ru_;

  /**
   * Received data
   *
   * Number of RX buffers (size = packet_length) and buffer status
   * entries: RX_THREAD_NUM * RX_BUFFER_FRAME_NUM * BS_ANT_NUM *
   * symbol_num_perframe
   */
  Table<char> rx_buffer_;
  Table<int> rx_buffer_status_;

  size_t rx_buffer_size_;
  int rx_buffer_status_size_;

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
  std::vector<myVec> csi_buffer_;

  /**
   * Data after equalization
   * First dimension: data_symbol_num_perframe * kFrameWnd
   * Second dimension: OFDM_CA_NUM * UE_NUM
   */
  std::vector<myVec> equal_buffer_;

  /**
   * Data symbols after IFFT
   * First dimension: total symbol number in the buffer:
   * data_symbol_num_perframe * kFrameWnd second dimension:
   * BS_ANT_NUM * OFDM_CA_NUM second dimension data order: SC1-32 of ants,
   * SC33-64 of ants, ..., SC993-1024 of ants (32 blocks each with 32
   * subcarriers)
   */
  Table<int8_t> dl_demod_buffer_;

  std::vector<std::vector<uint8_t>> dl_decode_buffer_;
  std::complex<float>* rx_samps_tmp_;  // Temp buffer for received samples

  int16_t* resp_var_nodes_;
  std::vector<std::complex<float>> pilot_sc_val_;
  std::vector<size_t> non_null_sc_ind_;
  std::vector<std::vector<std::complex<float>>> ue_pilot_vec_;
  Table<size_t> decoded_bits_count_;
  Table<size_t> bit_error_count_;
  Table<size_t> decoded_blocks_count_;
  Table<size_t> block_error_count_;
  std::vector<size_t> decoded_symbol_count_;
  std::vector<size_t> symbol_error_count_;

  // Communication queues
  moodycamel::ConcurrentQueue<EventData> complete_queue_;
  moodycamel::ConcurrentQueue<EventData> work_queue_;

  moodycamel::ConcurrentQueue<EventData> tx_queue_;
  moodycamel::ConcurrentQueue<EventData> to_mac_queue_;

  std::array<std::thread, kMaxThreads> task_threads_;

  moodycamel::ProducerToken* rx_ptoks_ptr_[kMaxThreads];
  moodycamel::ProducerToken* tx_ptoks_ptr_[kMaxThreads];
  moodycamel::ProducerToken* mac_rx_ptoks_ptr_[kMaxThreads];
  moodycamel::ProducerToken* mac_tx_ptoks_ptr_[kMaxThreads];
  moodycamel::ProducerToken* task_ptok_[kMaxThreads];

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
  FrameCounters ifft_counters_;

  size_t max_equaled_frame_ = 0;
};
#endif  // PHY_UE_H_
