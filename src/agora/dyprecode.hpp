#ifndef DYPRECODE
#define DYPRECODE

#include "Symbols.hpp"
#include "buffer.hpp"
#include "concurrentqueue.h"
#include "config.hpp"
#include "control.hpp"
#include "doer.hpp"
#include "gettime.h"
#include "memory_manage.h"
#include "modulation.hpp"
#include <armadillo>
#include <iostream>
#include <mkl.h>
#include <stdio.h>
#include <string.h>
#include <vector>
// #include "mkl_dfti.h"

class DyPrecode : public Doer {
public:
    DyPrecode(Config* in_config, int in_tid, double freq_ghz,
        Table<int8_t>& encoded_buffer_to_precode,
        PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& dl_zf_matrices,
        Table<complex_float>& dl_ifft_buffer,
        std::vector<std::vector<ControlInfo>>& control_info_table,
        std::vector<size_t>& control_idx_list);
    ~DyPrecode();

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
    void Launch(size_t frame_id, size_t symbol_idx_dl, size_t base_sc_id, size_t sc_block_size);

    size_t task_count_ = 0;

private:
    PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& dl_zf_matrices_;
    Table<complex_float>& dl_ifft_buffer_;
    Table<int8_t>& encoded_buffer_to_precode_;
    Table<float> qam_table_;
    complex_float* modulated_buffer_temp_;
    complex_float* precoded_buffer_temp_;

    void* jitter[kMaxUEs];
    cgemm_jit_kernel_t mkl_jit_cgemm[kMaxUEs];

    // Control info
    std::vector<std::vector<ControlInfo>>& control_info_table_;
    std::vector<size_t>& control_idx_list_;

};

#endif
