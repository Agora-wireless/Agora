#ifndef DYDEMUL
#define DYDEMUL

#include "Symbols.hpp"
#include "buffer.hpp"
#include "concurrentqueue.h"
#include "config.hpp"
#include "control.hpp"
#include "doer.hpp"
#include "gettime.h"
#include "modulation.hpp"
#include <armadillo>
#include <iostream>
#include <mkl.h>
#include <stdio.h>
#include <string.h>
#include <vector>

// Just-in-time optimization for MKL cgemm is available only after MKL 2019
// update 3. Disable this on systems with an older MKL version.
#if __INTEL_MKL__ >= 2020 || (__INTEL_MKL__ == 2019 && __INTEL_MKL_UPDATE__ > 3)
#define USE_MKL_JIT 1
#else
#define USE_MKL_JIT 0
#endif

using namespace arma;
class DyDemul : public Doer {
public:
    DyDemul(Config* config, int tid, double freq_ghz,
        Table<char>& freq_domain_iq_buffer,
        PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& ul_zf_matrices,
        Table<complex_float>& equal_buffer,
        PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& demod_buffer_to_send,
        std::vector<std::vector<ControlInfo>>& control_info_table,
        std::vector<size_t>& control_idx_list);
    ~DyDemul();

    /**
     * Do demodulation task for a block of subcarriers (demul_block_size)
     * @param tid: task thread index, used for selecting data_gather_buffer 
     * and task ptok
     * @param offset: offset of the first subcarrier in the block in
     * data_buffer_ Buffers: data_buffer_, data_gather_buffer_, precoder_buffer_,
     * equal_buffer_, demod_hard_buffer_ Input buffer: data_buffer_,
     * precoder_buffer_ Output buffer: demod_hard_buffer_ Intermediate buffer:
     * data_gather_buffer, equal_buffer_ Offsets: data_buffer_: dim1: frame index * # of
     * data symbols per frame + data symbol index dim2: transpose block
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

    void Launch(
        size_t frame_id, size_t symbol_idx_ul, size_t base_sc_id, size_t sc_block_size);

    void PrintOverhead() {
        printf("DoDemul thread %u overhead: total time: %.2lfms, "
            "preprocess: %.2lfms (%.2lf%%), equal: %.2lfms (%zu, %.2lf%%), "
            "demod: %.2lfms (%zu, %.2lf%%)\n", tid_, cycles_to_ms(total_cycles_, freq_ghz_),
            cycles_to_ms(preprocess_cycles_, freq_ghz_), preprocess_cycles_ * 100.0f / total_cycles_,
            cycles_to_ms(equal_cycles_, freq_ghz_), equal_count_, equal_cycles_ * 100.0f / total_cycles_,
            cycles_to_ms(demod_cycles_, freq_ghz_), demod_count_, demod_cycles_ * 100.0f / total_cycles_);
    }

private:
    PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& ul_zf_matrices_;
    Table<complex_float>& equal_buffer_; // Totally unused for now because of we always disable kExportConstellation
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& demod_buffer_to_send_;
    Table<char>& freq_domain_iq_buffer_;

    /// Intermediate buffer to gather raw data. Size = subcarriers per cacheline
    /// times number of antennas
    complex_float* data_gather_buffer_;

    // Intermediate buffers for equalized data
    complex_float* equaled_buffer_temp_;
    complex_float* equaled_buffer_temp_transposed_;

#if USE_MKL_JIT
    void* jitter[kMaxUEs];
    cgemm_jit_kernel_t mkl_jit_cgemm[kMaxUEs];
#endif

    size_t total_cycles_ = 0;
    size_t preprocess_cycles_ = 0;
    size_t equal_cycles_ = 0;
    size_t demod_cycles_ = 0;
    size_t equal_count_ = 0;
    size_t demod_count_ = 0;

    // Control info
    std::vector<std::vector<ControlInfo>>& control_info_table_;
    std::vector<size_t>& control_idx_list_;
};

#endif
