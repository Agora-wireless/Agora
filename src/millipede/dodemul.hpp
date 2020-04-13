/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#ifndef DODEMUL
#define DODEMUL

#include "Symbols.hpp"
#include "buffer.hpp"
#include "concurrentqueue.h"
#include "config.hpp"
#include "doer.hpp"
#include "gettime.h"
#include "modulation.hpp"
#include "stats.hpp"
#include <armadillo>
#include <iostream>
#include <stdio.h> /* for fprintf */
#include <string.h> /* for memcpy */
#include <vector>
// #include "mkl_dfti.h"

class DoDemul : public Doer {
public:
    DoDemul(Config* in_config, int in_tid, double freq_ghz,
        moodycamel::ConcurrentQueue<Event_data>& in_task_queue,
        moodycamel::ConcurrentQueue<Event_data>& complete_task_queue,
        moodycamel::ProducerToken* worker_producer_token,
        Table<complex_float>& in_data_buffer,
        Table<complex_float>& in_precoder_buffer,
        Table<complex_float>& in_equal_buffer,
        Table<uint8_t>& in_demul_hard_buffer,
        Table<int8_t>& in_demod_soft_buffer, Stats* in_stats_manager);
    ~DoDemul();

    /**
     * Do demodulation task for a block of subcarriers (demul_block_size)
     * @param tid: task thread index, used for selecting spm_buffer and task
     * ptok
     * @param offset: offset of the first subcarrier in the block in
     * data_buffer_ Buffers: data_buffer_, spm_buffer_, precoder_buffer_,
     * equal_buffer_, demod_hard_buffer_ Input buffer: data_buffer_,
     * precoder_buffer_ Output buffer: demod_hard_buffer_ Intermediate buffer:
     * spm_buffer, equal_buffer_ Offsets: data_buffer_: dim1: frame index * # of
     * data subframes per frame + data subframe index dim2: transpose block
     * index * block size * # of antennas + antenna index * block size
     *     spm_buffer:
     *         dim1: task thread index
     *         dim2: antenna index
     *     precoder_buffer_:
     *         dim1: frame index * FFT size + subcarrier index in the current
     * frame equal_buffer_, demul_buffer: dim1: frame index * # of data
     * subframes per frame + data subframe index dim2: subcarrier index * # of
     * users Event offset: offset Description:
     *     1. for each subcarrier in the block, block-wisely copy data from
     * data_buffer_ to spm_buffer_
     *     2. perform equalization with data and percoder matrixes
     *     3. perform demodulation on equalized data matrix
     *     4. add an event to the message queue to infrom main thread the
     * completion of this task
     */
    Event_data launch(int offset);

    Event_data DemulSingleSC(int offset);

private:
    Table<complex_float>& data_buffer_;
    Table<complex_float>& precoder_buffer_;
    Table<complex_float>& equal_buffer_;
    Table<uint8_t>& demod_hard_buffer_;
    Table<int8_t>& demod_soft_buffer_;
    DurationStat* duration_stat;

    /**
     * Intermediate buffer to gather raw data
     * First dimension: TASK_THREAD_NUM
     * Second dimension: BS_ANT_NUM */
    complex_float* spm_buffer;

    /**
     * Intermediate buffers for equalized data
     * dimension: UE_NUM * demul_block_size */
    complex_float* equaled_buffer_temp;
    complex_float* equaled_buffer_temp_transposed;

    int ue_num_simd256;
};

#endif
