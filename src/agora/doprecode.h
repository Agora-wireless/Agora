/**
 * @file doprecode.h
 * @brief Declaration file for the DoPrecode class.
 */
#ifndef DOPRECODE_H_
#define DOPRECODE_H_

#include <cstdint>
#include <iostream>
#include <vector>

#include "common_typedef_sdk.h"
#include "config.h"
#include "doer.h"
#include "memory_manage.h"
#include "message.h"
#include "mkl_dfti.h"
#include "stats.h"
#include "symbols.h"

class DoPrecode : public Doer {
 public:
  DoPrecode(Config* in_config, int in_tid,
            PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& dl_beam_matrices_,
            Table<complex_float>& in_dl_ifft_buffer,
            Table<int8_t>& dl_encoded_or_raw_data, Stats* in_stats_manager);
  ~DoPrecode() override;

  /**
   * Do demodulation task for a block of subcarriers (demul_block_size)
   * @param tid: task thread index, used for selecting spm_buffer and task
   * ptok
   * @param offset: offset of the first subcarrier in the block in
   * data_buffer_ Buffers: data_buffer_, spm_buffer_, precoder_buffer_,
   * equal_buffer_, demul_hard_buffer_ Input buffer: data_buffer_,
   * precoder_buffer_ Output buffer: demul_hard_buffer_ Intermediate buffer:
   * spm_buffer, equal_buffer_ Offsets: data_buffer_: dim1: frame index * # of
   * data symbols per frame + data symbol index dim2: transpose block
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
   * data_buffer_ to spm_buffer_
   *     2. perform equalization with data and percoder matrixes
   *     3. perform demodulation on equalized data matrix
   *     4. add an event to the message queue to infrom main thread the
   * completion of this task
   */
  EventData Launch(size_t tag) override;

  // Load input data for a single UE and a single subcarrier
  void LoadInputData(size_t symbol_idx_dl, size_t total_data_symbol_idx,
                     size_t user_id, size_t sc_id, size_t sc_id_in_block);
  void PrecodingPerSc(size_t frame_slot, size_t sc_id, size_t sc_id_in_block);

 private:
  PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& dl_beam_matrices_;
  Table<complex_float>& dl_ifft_buffer_;
  Table<int8_t>& dl_raw_data_;
  Table<float> qam_table_;
  DurationStat* duration_stat_;
  complex_float* modulated_buffer_temp_;
  complex_float* precoded_buffer_temp_;
#if defined(USE_MKL_JIT)
  void* jitter_;
  cgemm_jit_kernel_t my_cgemm_;
#endif
};

#endif  // DOPRECODE_H_
