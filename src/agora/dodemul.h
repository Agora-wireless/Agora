/**
 * @file dodemul.h
 * @brief Declaration file for the DoDemul class.
 */
#ifndef DODEMUL_H_
#define DODEMUL_H_

#include "armadillo"
#include "common_typedef_sdk.h"
#include "concurrentqueue.h"
#include "config.h"
#include "doer.h"
#include "memory_manage.h"
#include "mkl_dfti.h"
#include "phy_stats.h"
#include "stats.h"
#include "symbols.h"

class DoDemul : public Doer {
 public:
  DoDemul(Config* config, int tid, Table<complex_float>& data_buffer,
          PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& ul_beam_matrices,
          Table<complex_float>& ue_spec_pilot_buffer,
          Table<complex_float>& equal_buffer,
          PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& demod_buffers_,
          PhyStats* in_phy_stats, Stats* in_stats_manager);
  ~DoDemul() override;

  /**
   * Do demodulation task for a block of subcarriers (demul_block_size)
   * @param tid: task thread index, used for selecting data_gather_buffer
   * and task ptok
   * @param offset: offset of the first subcarrier in the block in
   * data_buffer_ Buffers: data_buffer_, data_gather_buffer_, precoder_buffer_,
   * equal_buffer_, demod_hard_buffer_ Input buffer: data_buffer_,
   * precoder_buffer_ Output buffer: demod_hard_buffer_ Intermediate buffer:
   * data_gather_buffer, equal_buffer_ Offsets: data_buffer_: dim1: frame index
   * * # of data symbols per frame + data symbol index dim2: transpose block
   * index * block size * # of antennas + antenna index * block size
   *     data_gather_buffer:
   *         dim1: task thread index
   *         dim2: antenna index
   *     precoder_buffer_:
   *         dim1: frame index * FFT size + subcarrier index in the current
   * frame equal_buffer_, demul_buffer: dim1: frame index * # of data
   * symbols per frame + data symbol index dim2: subcarrier index * # of
   * users Event offset: offset Description:
   *     1. for each subcarrier in the block, block-wisely copy data from
   * data_buffer_ to data_gather_buffer_
   *     2. perform equalization with data and percoder matrixes
   *     3. perform demodulation on equalized data matrix
   *     4. add an event to the message queue to infrom main thread the
   * completion of this task
   */
  EventData Launch(size_t tag) override;

 private:
  Table<complex_float>& data_buffer_;
  PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& ul_beam_matrices_;
  Table<complex_float>& ue_spec_pilot_buffer_;
  Table<complex_float>& equal_buffer_;
  PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& demod_buffers_;
  DurationStat* duration_stat_;
  PhyStats* phy_stats_;

  /// Intermediate buffer to gather raw data. Size = subcarriers per cacheline
  /// times number of antennas
  complex_float* data_gather_buffer_;

  // Intermediate buffers for equalized data
  complex_float* equaled_buffer_temp_;
  complex_float* equaled_buffer_temp_transposed_;
  arma::cx_fmat ue_pilot_data_;
  int ue_num_simd256_;

#if defined(USE_MKL_JIT)
  void* jitter_;
  cgemm_jit_kernel_t mkl_jit_cgemm_;
#endif
};

#endif  // DODEMUL_H_
