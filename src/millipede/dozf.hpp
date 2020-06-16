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
#include "stats.hpp"
#include <armadillo>
#include <iostream>
#include <stdio.h>
#include <string.h>

class DoZF : public Doer {
public:
    DoZF(Config* in_config, int tid, double freq_ghz,
        moodycamel::ConcurrentQueue<Event_data>& task_queue,
        moodycamel::ConcurrentQueue<Event_data>& complete_task_queue,
        moodycamel::ProducerToken* worker_producer_token,
        Table<complex_float>& csi_buffer, Table<complex_float>& recip_buffer,
        Table<complex_float>& equalizer_buffer,
        Table<complex_float>& precoder_buffer, Stats* stats_manager);
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
    Event_data launch(size_t tag);

private:
    void ZF_time_orthogonal(size_t tag);

    /// Compute equalization matrix and/or precoder using this CSI matrix and
    /// calibration buffer as input
    void compute_precoder(const arma::cx_fmat& mat_csi,
        const complex_float* recip_buf, complex_float* mat_equalizer,
        complex_float* mat_precoder);

    void ZF_freq_orthogonal(size_t tag);

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
    void Predict(size_t offset);

    Table<complex_float> csi_buffer_;
    complex_float* pred_csi_buffer;
    Table<complex_float> recip_buffer_;
    Table<complex_float> equalizer_buffer_;
    Table<complex_float> precoder_buffer_;
    DurationStat* duration_stat;

    complex_float* csi_gather_buffer; // Intermediate buffer to gather CSI
};

#endif
