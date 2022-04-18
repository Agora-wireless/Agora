/**
 * @file dozf.h
 * @brief Declaration file for the DoZf class.  Zero forcing for one subcarrier.
 */
#ifndef DOZF_H_
#define DOZF_H_

#include <armadillo>
#include <iostream>

#include "buffer.h"
#include "concurrentqueue.h"
#include "config.h"
#include "doer.h"
#include "gettime.h"
#include "mat_logger.h"
#include "phy_stats.h"
#include "stats.h"
#include "symbols.h"
#include "utils.h"

class DoZF : public Doer {
 public:
  DoZF(Config* in_config, int tid,
       PtrGrid<kFrameWnd, kMaxUEs, complex_float>& csi_buffers,
       Table<complex_float>& calib_dl_buffer,
       Table<complex_float>& calib_ul_buffer,
       Table<complex_float>& calib_dl_msum_buffer,
       Table<complex_float>& calib_ul_msum_buffer,
       PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& ul_zf_matrices_,
       PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& dl_zf_matrices_,
       PhyStats* in_phy_stats, Stats* stats_manager,
       std::shared_ptr<CsvLog::MatLogger> csi_logger = {},
       std::shared_ptr<CsvLog::MatLogger> dlzf_logger = {});
  ~DoZF() override;

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
  EventData Launch(size_t tag) override;

 private:
  void ZfTimeOrthogonal(size_t tag);

  /// Compute the uplink zeroforcing detector matrix and/or the downlink
  /// zeroforcing precoder using this CSI matrix and calibration buffer
  float ComputePrecoder(const arma::cx_fmat& mat_csi,
                        const arma::cx_fvec& calib_sc_vec,
                        complex_float* ul_zf_mem, complex_float* dl_zf_mem);
  void ComputeCalib(size_t frame_id, size_t sc_id, arma::cx_fvec& calib_sc_vec);
  void ZfFreqOrthogonal(size_t tag);

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
  complex_float* pred_csi_buffer_;

  //Should be read only (Set by FFT and read by Zf)
  Table<complex_float>& calib_dl_buffer_;
  Table<complex_float>& calib_ul_buffer_;

  //Shared by all doZf objects
  Table<complex_float>& calib_dl_msum_buffer_;
  Table<complex_float>& calib_ul_msum_buffer_;
  PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& ul_zf_matrices_;
  PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& dl_zf_matrices_;
  DurationStat* duration_stat_;

  complex_float* csi_gather_buffer_;  // Intermediate buffer to gather CSI
  // Intermediate buffer to gather reciprical calibration data vector
  complex_float* calib_gather_buffer_;
  std::unique_ptr<arma::cx_fvec> calib_sc_vec_ptr_;

  PhyStats* phy_stats_;
  arma::uvec ext_ref_id_;
  size_t num_ext_ref_;

  std::shared_ptr<CsvLog::MatLogger> csi_logger_;
  std::shared_ptr<CsvLog::MatLogger> dlzf_logger_;
};

#endif  // DOZF_H_
