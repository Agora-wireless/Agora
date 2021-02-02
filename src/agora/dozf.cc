/**
 * @file dozf.cc
 * @brief Implementation file for the DoZf class.  Zero forcing for one
 * subcarrier.
 */
#include "dozf.h"

#include "concurrent_queue_wrapper.inc"
#include "doer.h"

static constexpr bool kUseSIMDGather = true;
// Calculate the zeroforcing receiver using the formula W_zf = inv(H' * H) * H'.
// This is faster but less accurate than using an SVD-based pseudoinverse.
static constexpr size_t kUseInverseForZF = 1u;

DoZF::DoZF(Config* config, int tid,
           PtrGrid<kFrameWnd, kMaxUEs, complex_float>& csi_buffers,
           Table<complex_float>& calib_dl_buffer,
           Table<complex_float>& calib_ul_buffer,
           PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& ul_zf_matrices,
           PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& dl_zf_matrices,
           Stats* stats_manager)
    : Doer(config, tid),
      csi_buffers_(csi_buffers),
      calib_dl_buffer_(calib_dl_buffer),
      calib_ul_buffer_(calib_ul_buffer),
      ul_zf_matrices_(ul_zf_matrices),
      dl_zf_matrices_(dl_zf_matrices) {
  duration_stat_ = stats_manager->GetDurationStat(DoerType::kZF, tid);
  pred_csi_buffer_ =
      static_cast<complex_float*>(Agora_memory::PaddedAlignedAlloc(
          Agora_memory::Alignment_t::kAlign64,
          kMaxAntennas * kMaxUEs * sizeof(complex_float)));
  csi_gather_buffer_ =
      static_cast<complex_float*>(Agora_memory::PaddedAlignedAlloc(
          Agora_memory::Alignment_t::kAlign64,
          kMaxAntennas * kMaxUEs * sizeof(complex_float)));
  calib_gather_buffer_ = static_cast<complex_float*>(
      Agora_memory::PaddedAlignedAlloc(Agora_memory::Alignment_t::kAlign64,
                                       kMaxAntennas * sizeof(complex_float)));
}

DoZF::~DoZF() {
  std::free(pred_csi_buffer_);
  std::free(csi_gather_buffer_);
  std::free(calib_gather_buffer_);
}

EventData DoZF::Launch(size_t tag) {
  if (cfg_->FreqOrthogonalPilot()) {
    ZfFreqOrthogonal(tag);
  } else {
    ZfTimeOrthogonal(tag);
  }

  return EventData(EventType::kZF, tag);
}

void DoZF::ComputePrecoder(const arma::cx_fmat& mat_csi,
                           complex_float* calib_ptr, complex_float* _mat_ul_zf,
                           complex_float* _mat_dl_zf) {
  arma::cx_fmat mat_ul_zf(reinterpret_cast<arma::cx_float*>(_mat_ul_zf),
                          cfg_->UeNum(), cfg_->BsAntNum(), false);
  arma::cx_fmat mat_ul_zf_tmp;
  if (kUseInverseForZF != 0u) {
    try {
      mat_ul_zf_tmp = arma::inv_sympd(mat_csi.t() * mat_csi) * mat_csi.t();
    } catch (std::runtime_error&) {
      MLPD_WARN("Failed to invert channel matrix, falling back to pinv()\n");
      arma::pinv(mat_ul_zf_tmp, mat_csi, 1e-2, "dc");
    }
  } else {
    arma::pinv(mat_ul_zf_tmp, mat_csi, 1e-2, "dc");
  }

  if (cfg_->Frame().NumDLSyms() > 0) {
    arma::cx_fvec vec_calib(reinterpret_cast<arma::cx_float*>(calib_ptr),
                            cfg_->BfAntNum(), false);

    // with orthonormal calib matrix:
    // pinv(calib * csi) = pinv(csi)*inv(calib)
    arma::cx_fmat calib_mat = arma::diagmat(arma::sign(vec_calib));
    arma::cx_fmat mat_dl_zf_tmp = mat_ul_zf_tmp * inv(calib_mat);

    // We should be scaling the beamforming matrix, so the IFFT
    // output can be scaled with ofdm_ca_num() across all antennas.
    // See Argos paper (Mobicom 2012) Sec. 3.4 for details.
    float scale = 1 / (abs(mat_dl_zf_tmp).max());
    mat_dl_zf_tmp *= scale;

    if (cfg_->ExternalRefNode()) {
      mat_dl_zf_tmp.insert_cols(
          cfg_->RefAnt(),
          arma::cx_fmat(cfg_->UeNum(), cfg_->NumChannels(), arma::fill::zeros));
    }
    arma::cx_fmat mat_dl_zf(reinterpret_cast<arma::cx_float*>(_mat_dl_zf),
                            cfg_->BsAntNum(), cfg_->UeNum(), false);
    mat_dl_zf = mat_dl_zf_tmp.st();
  }
  if (cfg_->ExternalRefNode() == true) {
    mat_ul_zf_tmp.insert_cols(
        cfg_->RefAnt(),
        arma::cx_fmat(cfg_->UeNum(), cfg_->NumChannels(), arma::fill::zeros));
  }
  mat_ul_zf = mat_ul_zf_tmp;
}

// Gather data of one symbol from partially-transposed buffer
// produced by dofft
static inline void PartialTransposeGather(size_t cur_sc_id, float* src,
                                          float*& dst, size_t bs_ant_num) {
  // The SIMD and non-SIMD methods are equivalent.
  size_t ant_start = 0;
  if (kUseSIMDGather and bs_ant_num >= 4) {
    __m256i index = _mm256_setr_epi32(
        0, 1, kTransposeBlockSize * 2, kTransposeBlockSize * 2 + 1,
        kTransposeBlockSize * 4, kTransposeBlockSize * 4 + 1,
        kTransposeBlockSize * 6, kTransposeBlockSize * 6 + 1);

    const size_t transpose_block_id = cur_sc_id / kTransposeBlockSize;
    const size_t sc_inblock_idx = cur_sc_id % kTransposeBlockSize;
    const size_t offset_in_src_buffer =
        transpose_block_id * bs_ant_num * kTransposeBlockSize + sc_inblock_idx;

    src = src + offset_in_src_buffer * 2;
    for (size_t ant_idx = 0; ant_idx < bs_ant_num; ant_idx += 4) {
      // fetch 4 complex floats for 4 ants
      auto t = _mm256_i32gather_ps(src, index, 4);
      _mm256_storeu_ps(dst, t);
      src += 8 * kTransposeBlockSize;
      dst += 8;
    }
    // Set the of the remaining antennas to use non-SIMD gather
    ant_start = bs_ant_num / 4 * 4;
  }
  if (ant_start < bs_ant_num) {
    const size_t pt_base_offset =
        (cur_sc_id / kTransposeBlockSize) * (kTransposeBlockSize * bs_ant_num);
    auto* cx_src = reinterpret_cast<complex_float*>(src);
    complex_float* cx_dst = (complex_float*)dst + ant_start;
    for (size_t ant_i = ant_start; ant_i < bs_ant_num; ant_i++) {
      *cx_dst = cx_src[pt_base_offset + (ant_i * kTransposeBlockSize) +
                       (cur_sc_id % kTransposeBlockSize)];
      cx_dst++;
    }
  }
}

// Gather data of one symbol from partially-transposed buffer
// produced by dofft
static inline void TransposeGather(size_t cur_sc_id, float* src, float*& dst,
                                   size_t bs_ant_num, size_t ofdm_data_num) {
  auto* cx_src = reinterpret_cast<complex_float*>(src);
  auto* cx_dst = reinterpret_cast<complex_float*>(dst);
  for (size_t ant_i = 0; ant_i < bs_ant_num; ant_i++) {
    *cx_dst = cx_src[ant_i * ofdm_data_num + cur_sc_id];
    cx_dst++;
  }
}

void DoZF::ZfTimeOrthogonal(size_t tag) {
  const size_t frame_id = gen_tag_t(tag).frame_id_;
  const size_t base_sc_id = gen_tag_t(tag).sc_id_;
  const size_t frame_slot = frame_id % kFrameWnd;
  if (kDebugPrintInTask) {
    std::printf("In doZF thread %d: frame: %zu, base subcarrier: %zu\n", tid_,
                frame_id, base_sc_id);
  }
  size_t num_subcarriers =
      std::min(cfg_->ZfBlockSize(), cfg_->OfdmDataNum() - base_sc_id);

  // Handle each subcarrier one by one
  for (size_t i = 0; i < num_subcarriers; i++) {
    size_t start_tsc1 = WorkerRdtsc();
    const size_t cur_sc_id = base_sc_id + i;

    // Gather CSI matrices of each pilot from partially-transposed CSIs.
    for (size_t ue_idx = 0; ue_idx < cfg_->UeNum(); ue_idx++) {
      auto* dst_csi_ptr = reinterpret_cast<float*>(csi_gather_buffer_ +
                                                   cfg_->BsAntNum() * ue_idx);
      if (kUsePartialTrans) {
        PartialTransposeGather(cur_sc_id,
                               (float*)csi_buffers_[frame_slot][ue_idx],
                               dst_csi_ptr, cfg_->BsAntNum());
      } else {
        TransposeGather(cur_sc_id, (float*)csi_buffers_[frame_slot][ue_idx],
                        dst_csi_ptr, cfg_->BsAntNum(), cfg_->OfdmDataNum());
      }
    }

    duration_stat_->task_duration_[1] += WorkerRdtsc() - start_tsc1;
    arma::cx_fmat mat_csi((arma::cx_float*)csi_gather_buffer_, cfg_->BsAntNum(),
                          cfg_->UeNum(), false);

    if (cfg_->Frame().NumDLSyms() > 0) {
      arma::cx_fvec calib_vec(
          reinterpret_cast<arma::cx_float*>(calib_gather_buffer_),
          cfg_->BfAntNum(), false);
      size_t frame_cal_slot = kFrameWnd - 1;
      size_t frame_cal_slot_prev = kFrameWnd - 1;
      if (cfg_->Frame().IsRecCalEnabled() && frame_id >= TX_FRAME_DELTA) {
        size_t frame_grp_id = (frame_id - TX_FRAME_DELTA) / cfg_->AntGroupNum();

        // use the previous window which has a full set of calibration results
        frame_cal_slot = (frame_grp_id + kFrameWnd - 1) % kFrameWnd;
        if (frame_id >= TX_FRAME_DELTA + cfg_->AntGroupNum()) {
          frame_cal_slot_prev = (frame_grp_id + kFrameWnd - 2) % kFrameWnd;
        }
      }
      arma::cx_fmat calib_dl_mat(
          reinterpret_cast<arma::cx_float*>(calib_dl_buffer_[frame_cal_slot]),
          cfg_->OfdmDataNum(), cfg_->BfAntNum(), false);
      arma::cx_fmat calib_ul_mat(
          reinterpret_cast<arma::cx_float*>(calib_ul_buffer_[frame_cal_slot]),
          cfg_->OfdmDataNum(), cfg_->BfAntNum(), false);
      arma::cx_fmat calib_dl_mat_prev(
          reinterpret_cast<arma::cx_float*>(
              calib_dl_buffer_[frame_cal_slot_prev]),
          cfg_->OfdmDataNum(), cfg_->BfAntNum(), false);
      arma::cx_fmat calib_ul_mat_prev(
          reinterpret_cast<arma::cx_float*>(
              calib_ul_buffer_[frame_cal_slot_prev]),
          cfg_->OfdmDataNum(), cfg_->BfAntNum(), false);
      arma::cx_fmat calib_dl_mat_mean = (calib_dl_mat + calib_dl_mat_prev) / 2;
      arma::cx_fmat calib_ul_mat_mean = (calib_ul_mat + calib_ul_mat_prev) / 2;
      arma::cx_fvec calib_dl_vec = calib_dl_mat_mean.row(cur_sc_id).st();
      arma::cx_fvec calib_ul_vec = calib_ul_mat_mean.row(cur_sc_id).st();
      calib_vec = calib_dl_vec / calib_ul_vec;

      if (cfg_->ExternalRefNode()) {
        mat_csi.shed_rows(cfg_->RefAnt(),
                          cfg_->RefAnt() + cfg_->NumChannels() - 1);
      }
    }

    double start_tsc2 = WorkerRdtsc();
    duration_stat_->task_duration_[2] += start_tsc2 - start_tsc1;
    ComputePrecoder(mat_csi, calib_gather_buffer_,
                    ul_zf_matrices_[frame_slot][cur_sc_id],
                    dl_zf_matrices_[frame_slot][cur_sc_id]);

    // cout<<"Precoder:" <<mat_output<<endl;
    double duration3 = WorkerRdtsc() - start_tsc2;
    duration_stat_->task_duration_[3] += duration3;
    duration_stat_->task_count_++;
    duration_stat_->task_duration_[0] += WorkerRdtsc() - start_tsc1;
    // if (duration > 500) {
    //     std::printf("Thread %d ZF takes %.2f\n", tid, duration);
    // }
  }
}

void DoZF::ZfFreqOrthogonal(size_t tag) {
  const size_t frame_id = gen_tag_t(tag).frame_id_;
  const size_t base_sc_id = gen_tag_t(tag).sc_id_;
  const size_t frame_slot = frame_id % kFrameWnd;
  if (kDebugPrintInTask) {
    std::printf(
        "In doZF thread %d: frame: %zu, subcarrier: %zu, block: %zu, "
        "bs_ant_num(): %zu\n",
        tid_, frame_id, base_sc_id, base_sc_id / cfg_->UeNum(),
        cfg_->BsAntNum());
  }

  double start_tsc1 = WorkerRdtsc();

  // Gather CSIs from partially-transposed CSIs
  for (size_t i = 0; i < cfg_->UeNum(); i++) {
    const size_t cur_sc_id = base_sc_id + i;
    auto* dst_csi_ptr =
        reinterpret_cast<float*>(csi_gather_buffer_ + cfg_->BsAntNum() * i);
    PartialTransposeGather(cur_sc_id, (float*)csi_buffers_[frame_slot][0],
                           dst_csi_ptr, cfg_->BsAntNum());
  }
  if (cfg_->Frame().NumDLSyms() > 0) {
    arma::cx_fvec calib_vec(
        reinterpret_cast<arma::cx_float*>(calib_gather_buffer_),
        cfg_->BfAntNum(), false);
    size_t frame_cal_slot = kFrameWnd - 1;
    size_t frame_cal_slot_prev = kFrameWnd - 1;
    if (cfg_->Frame().IsRecCalEnabled() && (frame_id >= TX_FRAME_DELTA)) {
      size_t frame_grp_id = (frame_id - TX_FRAME_DELTA) / cfg_->AntGroupNum();

      // use the previous window which has a full set of calibration results
      frame_cal_slot = (frame_grp_id + kFrameWnd - 1) % kFrameWnd;
      if (frame_id >= TX_FRAME_DELTA + cfg_->AntGroupNum())
        frame_cal_slot_prev = (frame_grp_id + kFrameWnd - 2) % kFrameWnd;
    }
    arma::cx_fmat calib_dl_mat(
        reinterpret_cast<arma::cx_float*>(calib_dl_buffer_[frame_cal_slot]),
        cfg_->OfdmDataNum(), cfg_->BfAntNum(), false);
    arma::cx_fvec calib_ul_mat(
        reinterpret_cast<arma::cx_float*>(calib_ul_buffer_[frame_cal_slot]),
        cfg_->OfdmDataNum(), cfg_->BfAntNum() != 0u, false);

    arma::cx_fmat calib_dl_mat_prev(reinterpret_cast<arma::cx_float*>(
                                        calib_dl_buffer_[frame_cal_slot_prev]),
                                    cfg_->OfdmDataNum(), cfg_->BfAntNum(),
                                    false);
    arma::cx_fmat calib_ul_mat_prev(reinterpret_cast<arma::cx_float*>(
                                        calib_ul_buffer_[frame_cal_slot_prev]),
                                    cfg_->OfdmDataNum(), cfg_->BfAntNum(),
                                    false);
    arma::cx_fmat calib_dl_mat_mean = (calib_dl_mat + calib_dl_mat_prev) / 2;
    arma::cx_fmat calib_ul_mat_mean = (calib_ul_mat + calib_ul_mat_prev) / 2;
    arma::cx_fvec calib_dl_vec = calib_dl_mat_mean.row(base_sc_id).st();
    arma::cx_fvec calib_ul_vec = calib_ul_mat_mean.row(base_sc_id).st();
    calib_vec = calib_dl_vec / calib_ul_vec;
  }

  duration_stat_->task_duration_[1] += WorkerRdtsc() - start_tsc1;
  arma::cx_fmat mat_csi(reinterpret_cast<arma::cx_float*>(csi_gather_buffer_),
                        cfg_->BsAntNum(), cfg_->UeNum(), false);

  ComputePrecoder(mat_csi, calib_gather_buffer_,
                  ul_zf_matrices_[frame_slot][cfg_->GetZfScId(base_sc_id)],
                  dl_zf_matrices_[frame_slot][cfg_->GetZfScId(base_sc_id)]);

  double start_tsc2 = WorkerRdtsc();
  duration_stat_->task_duration_[2] += start_tsc2 - start_tsc1;

  // cout<<"Precoder:" <<mat_output<<endl;
  duration_stat_->task_duration_[3] += WorkerRdtsc() - start_tsc2;
  duration_stat_->task_count_++;
  duration_stat_->task_duration_[0] += WorkerRdtsc() - start_tsc1;

  // if (duration > 500) {
  //     std::printf("Thread %d ZF takes %.2f\n", tid, duration);
  // }
}

// Currently unused
/*
void DoZF::Predict(size_t tag)
{
    size_t frame_id = gen_tag_t(tag).frame_id;
    size_t base_sc_id = gen_tag_t(tag).sc_id;

    // Use stale CSI as predicted CSI
    // TODO: add prediction algorithm
    const size_t offset_in_buffer
        = ((frame_id % kFrameWnd) * cfg_->ofdm_data_num())
        + base_sc_id;
    auto* ptr_in = (arma::cx_float*)pred_csi_buffer;
    std::memcpy(ptr_in, (arma::cx_float*)csi_buffer_[offset_in_buffer],
        sizeof(arma::cx_float) * cfg_->bs_ant_num() * cfg_->ue_num());
    arma::cx_fmat mat_input(ptr_in, cfg_->bs_ant_num(), cfg_->ue_num(), false);

    // Input matrix and calibration are for current frame, output precoders are
    // for the next frame
    compute_precoder(mat_input,
        cfg_->GetCalibBuffer(calib_buffer_, frame_id, base_sc_id),
        cfg_->get_ul_zf_mat(ul_zf_buffer_, frame_id + 1, base_sc_id),
        cfg_->get_dl_zf_mat(dl_zf_buffer_, frame_id + 1, base_sc_id));
}
*/
