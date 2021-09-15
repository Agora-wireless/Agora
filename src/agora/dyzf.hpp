#ifndef DYZF
#define DYZF

#include "Symbols.hpp"
#include "buffer.hpp"
#include "concurrentqueue.h"
#include "config.hpp"
#include "control.hpp"
#include "doer.hpp"
#include "gettime.h"
#include "stats.hpp"
#include "utils.h"
#include <armadillo>
#include <iostream>
#include <stdio.h>
#include <string.h>

class DyZF : public Doer {
public:
    DyZF(Config* in_config, int tid, double freq_ghz,
        moodycamel::ConcurrentQueue<Event_data>& task_queue,
        moodycamel::ConcurrentQueue<Event_data>& complete_task_queue,
        moodycamel::ProducerToken* worker_producer_token,
        PtrGrid<kFrameWnd, kMaxUEs, complex_float>& csi_buffers,
        Table<complex_float>& calib_buffer,
        PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& ul_zf_matrices_,
        PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& dl_zf_matrices_,
        std::vector<std::vector<ControlInfo>>& control_info_table_,
        std::vector<size_t>& control_idx_list_,
        Stats* stats_manager);
    ~DyZF();

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

    double get_zf_tsc_per_task() {return cycles_to_us(zf_tsc_ / zf_count_, freq_ghz);}

private:
    void ZF_time_orthogonal(size_t tag);

    /// Compute the uplink zeroforcing detector matrix and/or the downlink
    /// zeroforcing precoder using this CSI matrix and calibration buffer
    void compute_precoder(const arma::cx_fmat& mat_csi,
        complex_float* calib_buf, complex_float* mat_ul_zf,
        complex_float* mat_dl_zf, size_t ue_num = 0);

    void ZF_freq_orthogonal(size_t tag);
    void ZF_freq_orthogonal_dynamic(size_t tag);

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

    PtrGrid<kFrameWnd, kMaxUEs, complex_float>& csi_buffers_;
    complex_float* pred_csi_buffer;
    Table<complex_float> calib_buffer_;
    PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& ul_zf_matrices_;
    PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& dl_zf_matrices_;
    DurationStat* duration_stat;

    complex_float* csi_gather_buffer; // Intermediate buffer to gather CSI
    // Intermediate buffer to gather reciprical calibration data vector
    complex_float* calib_gather_buffer;

    // Control info
    std::vector<std::vector<ControlInfo>>& control_info_table_;
    std::vector<size_t>& control_idx_list_;

    size_t zf_tsc_ = 0;
    size_t zf_count_ = 0;
};

#endif
