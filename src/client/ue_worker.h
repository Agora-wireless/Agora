/**
 * @file ue_worker.h
 * @brief Declaration file for the phy ue class
 */

#ifndef UE_WORKER_H_
#define UE_WORKER_H_

#include <complex>
#include <thread>
#include <vector>

#include "concurrentqueue.h"
#include "config.h"
#include "csv_logger.h"
#include "dodecode_client.h"
#include "doencode.h"
#include "doifft_client.h"
#include "message.h"
#include "mkl_dfti.h"
#include "simd_types.h"
#include "stats.h"

class UeWorker {
 public:
  explicit UeWorker(
      size_t tid, Config& config, Stats& shared_stats,
      PhyStats& shared_phy_stats,
      moodycamel::ConcurrentQueue<EventData>& notify_queue,
      moodycamel::ConcurrentQueue<EventData>& work_queue,
      moodycamel::ProducerToken& work_producer, Table<int8_t>& ul_bits_buffer,
      Table<int8_t>& encoded_buffer, Table<complex_float>& modul_buffer,
      Table<complex_float>& ifft_buffer, char* const tx_buffer,
      Table<char>& rx_buffer, Table<complex_float>& csi_buffer,
      std::vector<SimdAlignCxFltVector>& equal_buffer,
      std::vector<size_t>& non_null_sc_ind, Table<complex_float>& fft_buffer,
      PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& demod_buffer,
      PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& decoded_buffer,
      std::vector<std::vector<std::complex<float>>>& ue_pilot_vec);
  ~UeWorker();

  void Start(size_t core_offset);
  void Stop();

 private:
  void TaskThread(size_t core_offset);

  /**
   * modulate data from nUEs and does spatial multiplexing by applying
   * beamweights
   */
  void DoEncodeUe(DoEncode* encoder, size_t tag);
  void DoModul(size_t tag);
  void DoIfftUe(DoIFFTClient* iffter, size_t tag);
  void DoIfft(size_t tag);

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
   * index dim2: subcarrier index csi_buffer_: dim1: frame index * FFT size
   * + subcarrier index in the current frame dim2: user index * # of
   * antennas + antenna index ul_data_buffer_: dim1: frame index * # of data
   * symbols per frame + data symbol index dim2: transpose block index *
   * block size
   * * # of antennas + antenna index * block size Event offset: frame index
   * * # of symbol per frame + symbol index Description:
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
  void DoFftPilot(size_t tag);
  void DoFftData(size_t tag);

  /**
   * Do demodulation task for a block of subcarriers (demul_block_size)
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
  void DoDemul(size_t tag);
  void DoDecodeUe(DoDecodeClient* decoder, size_t tag);

  size_t tid_;

  DFTI_DESCRIPTOR_HANDLE mkl_handle_;
  std::unique_ptr<moodycamel::ProducerToken> ptok_;
  std::thread thread_;
  std::complex<float>* rx_samps_tmp_;  // Temp buffer for received samples

  // Shared Queues
  moodycamel::ConcurrentQueue<EventData>& notify_queue_;
  moodycamel::ConcurrentQueue<EventData>& work_queue_;
  moodycamel::ProducerToken& work_producer_token_;

  // Shared Objects
  Config& config_;
  Stats& stats_;
  PhyStats& phy_stats_;

  // Shared Buffers
  // Uplink
  Table<int8_t>& ul_bits_buffer_;
  Table<int8_t>& encoded_buffer_;
  Table<complex_float>& modul_buffer_;
  Table<complex_float>& ifft_buffer_;
  char* const tx_buffer_;

  // Downlink
  Table<char>& rx_buffer_;
  Table<complex_float>& csi_buffer_;
  std::vector<SimdAlignCxFltVector>& equal_buffer_;
  std::vector<size_t>& non_null_sc_ind_;
  Table<complex_float>& fft_buffer_;
  PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& demod_buffer_;
  PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& decoded_buffer_;

  std::vector<std::vector<std::complex<float>>>& ue_pilot_vec_;
};
#endif  // UE_WORKER_H_
