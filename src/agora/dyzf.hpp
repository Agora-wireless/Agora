#ifndef DYZF
#define DYZF

#include "Symbols.hpp"
#include "buffer.hpp"
#include "concurrentqueue.h"
#include "config.hpp"
#include "control.hpp"
#include "doer.hpp"
#include "gettime.h"
#include "utils.h"
#include <armadillo>
#include <iostream>
#include <stdio.h>
#include <string.h>

class DyZF : public Doer {
public:
    DyZF(Config* in_config, int tid, double freq_ghz,
        PtrGrid<kFrameWnd, kMaxUEs, complex_float>& csi_buffers,
        Table<complex_float>& calib_buffer,
        PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& ul_zf_matrices_,
        PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& dl_zf_matrices_,
        std::vector<std::vector<ControlInfo>>& control_info_table_,
        std::vector<size_t>& control_idx_list_);
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
    EventData Launch(size_t tag);

    void ZFFreqOrthogonalStatic(size_t tag);

    double GetZfTscPerTask() {return zf_count_ == 0 ? 0 : cycles_to_us(zf_tsc_ / zf_count_, freq_ghz_);}

private:
    void ZFTimeOrthogonal(size_t tag);
    void ZFFreqOrthogonal(size_t tag);

    /// Compute the uplink zeroforcing detector matrix and/or the downlink
    /// zeroforcing precoder using this CSI matrix and calibration buffer
    void computePrecoder(const arma::cx_fmat& mat_csi,
        complex_float* calib_buf, complex_float* mat_ul_zf,
        complex_float* mat_dl_zf, size_t ue_num = 0);
    void computeULPrecoder(const arma::cx_fmat& mat_csi,
        complex_float* calib_buf, complex_float* mat_ul_zf);
    void computeDLPrecoder(const arma::cx_fmat& mat_csi,
        complex_float* calib_buf, complex_float* mat_dl_zf);

    PtrGrid<kFrameWnd, kMaxUEs, complex_float>& csi_buffers_;
    Table<complex_float> calib_buffer_;
    PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& ul_zf_matrices_;
    PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& dl_zf_matrices_;

    complex_float* csi_gather_buffer_; // Intermediate buffer to gather CSI
    // Intermediate buffer to gather reciprical calibration data vector
    complex_float* calib_gather_buffer_;

    // Control info
    std::vector<std::vector<ControlInfo>>& control_info_table_;
    std::vector<size_t>& control_idx_list_;

    size_t zf_tsc_ = 0;
    size_t zf_count_ = 0;
};

#endif
