/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#ifndef DOZF
#define DOZF

#include "Symbols.hpp"
#include "buffer.hpp"
#include "concurrentqueue.h"
#include "config.hpp"
#include "doer.hpp"
#include "gettime.h"
#include "offset.h"
#include "stats.hpp"
#include <armadillo>
#include <iostream>
#include <stdio.h> /* for fprintf */
#include <string.h> /* for memcpy */
#include <vector>
// #include "mkl_dfti.h"

class DoZF : public Doer {
public:
    DoZF(Config* in_config, int in_tid,
        moodycamel::ConcurrentQueue<event_data_t>& in_task_queue,
        Consumer& in_consumer, Table<complex_float>& in_csi_buffer,
        Table<complex_float>& in_recip_buffer,
        Table<complex_float>& in_ul_precoder_buffer,
        Table<complex_float>& in_dl_precoder_buffer, Stats* in_stats_manager);
    ~DoZF();

    /**
     * Do ZF task for one subcarrier with all pilots in a frame
     * @param tid: task thread index, used for selecting task ptok
     * @param offset: offset of the subcarrier in csi_buffer_
     * Buffers: csi_buffer_, precoder_buffer_
     *     Input buffer: csi_buffer_
     *     Output buffer: precoder_buffer_
     * Offsets:
     *     csi_buffer_, precoder_buffer_:
     *         dim1: frame index * FFT size + subcarrier index in the current
     * frame Event offset: offset Description:
     *     1. perform pseudo-inverse (pinv) on csi_buffer_ and store results in
     * precoder_buffer_
     *     2. add an event to the message queue to infrom main thread the
     * completion of this task
     */
    void launch(int offset);

private:
    void finish(int offset);
    void ZF_time_orthogonal(int offset);
    void precoder(void* mat_input, int frame_id, int sc_id, int offset,
        bool downlink_mode);

    void ZF_freq_orthogonal(int offset);

    /**
     * Do prediction task for one subcarrier
     * @param tid: task thread index, used for selecting task ptok
     * @param offset: offset of the subcarrier in csi_buffer_
     * Buffers: csi_buffer_, pred_csi_buffer_, precoder_buffer_
     *     Input buffer: csi_buffer_
     *     Output buffer: precoder_buffer_
     *     Intermediate buffer: pred_csi_buffer_
     * Offsets:
     *     csi_buffer_:
     *         dim1: frame index * FFT size + subcarrier index in the current
     * frame pred_csi_buffer: dim1: subcarrier index in the current frame
     *     precoder_buffer_:
     *         dim1: (frame index + 1) * FFT size + subcarrier index in the
     * current frame Event offset: offset Description:
     *     1. predict CSI (copy CSI from the current frame if prediction is
     * based on stale CSI)
     *     2. perform pseudo-inverse (pinv) on pred_csi_buffer_ and store
     * results in precoder_buffer_
     *     3. add an event to the message queue to infrom main thread the
     * completion of this task
     */
    void Predict(int offset);

    Table<complex_float> csi_buffer_;
    complex_float* pred_csi_buffer;
    Table<complex_float> recip_buffer_;
    Table<complex_float> ul_precoder_buffer_;
    Table<complex_float> dl_precoder_buffer_;

    Table<double>* ZF_task_duration;
    int* ZF_task_count;

    /**
     * Intermediate buffer to gather CSI
     * First dimension: TASK_THREAD_NUM
     * Second dimension: BS_ANT_NUM * UE_NUM */
    complex_float* csi_gather_buffer;
};

#endif
