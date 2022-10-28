/**
 * @file dobeamweights.cc
 * @brief Implementation file for the DoBeamWeights class.  Calculates Precoder/Detector  for one
 * subcarrier.
 */
#include "dobeamweights.h"

#include "comms-lib.h"
#include "concurrent_queue_wrapper.h"
#include "doer.h"
#include "logger.h"

static constexpr bool kUseSIMDGather = true;
// Calculate the zeroforcing receiver using the formula W_zf = inv(H' * H) * H'.
// This is faster but less accurate than using an SVD-based pseudoinverse.
static constexpr bool kUseInverseForZF = true;
static constexpr bool kUseUlZfForDownlink = true;

DoBeamWeights::DoBeamWeights(
    Config* config, int tid,
    PtrGrid<kFrameWnd, kMaxUEs, complex_float>& csi_buffers,
    Table<complex_float>& calib_dl_buffer,
    Table<complex_float>& calib_ul_buffer,
    Table<complex_float>& calib_dl_msum_buffer,
    Table<complex_float>& calib_ul_msum_buffer,
    PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& ul_beam_matrices,
    PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& dl_beam_matrices,
    PhyStats* in_phy_stats, Stats* stats_manager)
    : Doer(config, tid),
      csi_buffers_(csi_buffers),
      calib_dl_buffer_(calib_dl_buffer),
      calib_ul_buffer_(calib_ul_buffer),
      calib_dl_msum_buffer_(calib_dl_msum_buffer),
      calib_ul_msum_buffer_(calib_ul_msum_buffer),
      ul_beam_matrices_(ul_beam_matrices),
      dl_beam_matrices_(dl_beam_matrices),
      phy_stats_(in_phy_stats) {
  duration_stat_ = stats_manager->GetDurationStat(DoerType::kBeam, tid);
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

  calib_sc_vec_ptr_ = std::make_unique<arma::cx_fvec>(
      reinterpret_cast<arma::cx_float*>(calib_gather_buffer_), cfg_->BfAntNum(),
      false);

  //Init to identity
  calib_sc_vec_ptr_->fill(arma::cx_float(1.0f, 0.0f));

  num_ext_ref_ = 0;
  for (size_t i = 0; i < cfg_->NumCells(); i++) {
    if (cfg_->ExternalRefNode(i)) {
      num_ext_ref_++;
    }
  }
  if (num_ext_ref_ > 0) {
    ext_ref_id_.zeros(num_ext_ref_ * cfg_->NumChannels());
    size_t ext_id = 0;
    for (size_t i = 0; i < cfg_->NumCells(); i++) {
      if (cfg_->ExternalRefNode(i)) {
        for (size_t j = 0; j < cfg_->NumChannels(); j++) {
          ext_ref_id_.at(ext_id * cfg_->NumChannels() + j) =
              (cfg_->RefRadio(i) * cfg_->NumChannels()) + j;
        }
        ext_id++;
      }
    }
  }
}

DoBeamWeights::~DoBeamWeights() {
  std::free(pred_csi_buffer_);
  std::free(csi_gather_buffer_);
  calib_sc_vec_ptr_.reset();
  std::free(calib_gather_buffer_);
}

EventData DoBeamWeights::Launch(size_t tag) {
  if (cfg_->FreqOrthogonalPilot()) {
    ComputePartialCsiBeams(tag);
  } else {
    ComputeFullCsiBeams(tag);
  }

  return EventData(EventType::kBeam, tag);
}

float DoBeamWeights::ComputePrecoder(size_t frame_id, size_t cur_sc_id,
                                     const arma::cx_fmat& mat_csi,
                                     const arma::cx_fvec& calib_sc_vec,
                                     const float noise,
                                     complex_float* ul_beam_mem,
                                     complex_float* dl_beam_mem) {
  if (kEnableMatLog) {
    phy_stats_->UpdateUlCsi(frame_id, cur_sc_id, mat_csi);
  }
  arma::cx_fmat mat_ul_beam(reinterpret_cast<arma::cx_float*>(ul_beam_mem),
                            cfg_->UeAntNum(), cfg_->BsAntNum(), false);
  arma::cx_fmat mat_ul_beam_tmp;
  switch (cfg_->BeamformingAlgo()) {
    case CommsLib::BeamformingAlgorithm::kZF:
      if (kUseInverseForZF) {
        try {
          mat_ul_beam_tmp =
              arma::inv_sympd(mat_csi.t() * mat_csi) * mat_csi.t();
        } catch (std::runtime_error&) {
          AGORA_LOG_WARN(
              "Failed to invert channel matrix, falling back to pinv()\n");
          arma::pinv(mat_ul_beam_tmp, mat_csi, 1e-2, "dc");
        }
      } else {
        arma::pinv(mat_ul_beam_tmp, mat_csi, 1e-2, "dc");
      }
      break;
    case CommsLib::BeamformingAlgorithm::kMMSE:
      mat_ul_beam_tmp =
          arma::inv_sympd(mat_csi.t() * mat_csi +
                          noise * arma::eye<arma::cx_fmat>(cfg_->UeAntNum(),
                                                           cfg_->UeAntNum())) *
          mat_csi.t();
      break;
    case CommsLib::BeamformingAlgorithm::kMRC:
      mat_ul_beam_tmp = mat_csi.t();
      break;
    default:
      AGORA_LOG_ERROR("Beamforming algorithm is not implemented!");
  }

  if (cfg_->Frame().NumDLSyms() > 0) {
    arma::cx_fmat mat_dl_beam_tmp;
    if (kUseUlZfForDownlink == true) {
      // With orthonormal calib matrix:
      // pinv(calib * csi) = pinv(csi)*inv(calib)
      // This probably causes a performance hit since we are throwing
      // magnitude info away by taking the sign of the calibration matrix
      // Inv is already acheived by UL over DL division outside this function
      arma::cx_fmat inv_calib_mat = arma::diagmat(arma::sign(calib_sc_vec));
      mat_dl_beam_tmp = mat_ul_beam_tmp * inv_calib_mat;
    } else {
      arma::cx_fmat mat_dl_csi = inv(arma::diagmat(calib_sc_vec)) * mat_csi;
      if (kEnableMatLog) {
        phy_stats_->UpdateDlCsi(frame_id, cur_sc_id, mat_dl_csi);
      }
      switch (cfg_->BeamformingAlgo()) {
        case CommsLib::BeamformingAlgorithm::kZF:
          if (kUseInverseForZF) {
            try {
              mat_dl_beam_tmp =
                  arma::inv_sympd(mat_dl_csi.t() * mat_dl_csi) * mat_dl_csi.t();
            } catch (std::runtime_error&) {
              AGORA_LOG_WARN(
                  "Failed to invert channel matrix, falling back to pinv()\n");
              arma::pinv(mat_dl_beam_tmp, mat_csi, 1e-2, "dc");
            }
          } else {
            arma::pinv(mat_dl_beam_tmp, mat_csi, 1e-2, "dc");
          }
          break;
        case CommsLib::BeamformingAlgorithm::kMMSE:
          mat_dl_beam_tmp =
              arma::inv_sympd(mat_dl_csi.t() * mat_dl_csi +
                              noise * arma::eye<arma::cx_fmat>(
                                          cfg_->UeAntNum(), cfg_->UeAntNum())) *
              mat_dl_csi.t();
          break;
        case CommsLib::BeamformingAlgorithm::kMRC:
          mat_dl_beam_tmp = mat_dl_csi.t();
          break;
        default:
          AGORA_LOG_ERROR("Beamforming algorithm is not implemented!");
      }
    }
    // We should be scaling the beamforming matrix, so the IFFT
    // output can be scaled with OfdmCaNum() across all antennas.
    // See Argos paper (Mobicom 2012) Sec. 3.4 for details.
    const float scale = 1 / (abs(mat_dl_beam_tmp).max());
    mat_dl_beam_tmp = mat_dl_beam_tmp * scale;

    for (size_t i = 0; i < cfg_->NumCells(); i++) {
      if (cfg_->ExternalRefNode(i)) {
        // Zero out all antennas on the reference radio
        mat_dl_beam_tmp.insert_cols(
            (cfg_->RefRadio(i) * cfg_->NumChannels()),
            arma::cx_fmat(cfg_->UeAntNum(), cfg_->NumChannels(),
                          arma::fill::zeros));
      }
    }
    arma::cx_fmat mat_dl_beam(reinterpret_cast<arma::cx_float*>(dl_beam_mem),
                              cfg_->BsAntNum(), cfg_->UeAntNum(), false);
    mat_dl_beam = mat_dl_beam_tmp.st();
    if (kEnableMatLog) {
      phy_stats_->UpdateDlBeam(frame_id, cur_sc_id, mat_dl_beam);
    }
  }
  for (int i = (int)cfg_->NumCells() - 1; i >= 0; i--) {
    if (cfg_->ExternalRefNode(i) == true) {
      mat_ul_beam_tmp.insert_cols(
          (cfg_->RefRadio(i) * cfg_->NumChannels()),
          arma::cx_fmat(cfg_->UeAntNum(), cfg_->NumChannels(),
                        arma::fill::zeros));
    }
  }
  mat_ul_beam = mat_ul_beam_tmp;
  float rcond = -1;
  if (kPrintBeamStats) {
    rcond = arma::rcond(mat_csi.t() * mat_csi);
  }
  return rcond;
}

// Called for each frame_id / sc_id
// Updates calib_sc_vec
void DoBeamWeights::ComputeCalib(size_t frame_id, size_t sc_id,
                                 arma::cx_fvec& calib_sc_vec) {
  const size_t frames_to_complete = cfg_->RecipCalFrameCnt();
  if (cfg_->Frame().IsRecCalEnabled() && (frame_id >= frames_to_complete)) {
    const size_t cal_slot_current = cfg_->RecipCalIndex(frame_id);
    const bool frame_update = ((frame_id % frames_to_complete) == 0);

    // Use the previous window which has a full set of calibration results
    const size_t cal_slot_complete =
        cfg_->ModifyRecCalIndex(cal_slot_current, -1);

    // update moving sum
    arma::cx_fmat cur_calib_dl_msum_mat(
        reinterpret_cast<arma::cx_float*>(
            calib_dl_msum_buffer_[cal_slot_complete]),
        cfg_->OfdmDataNum(), cfg_->BfAntNum(), false);
    arma::cx_fmat cur_calib_ul_msum_mat(
        reinterpret_cast<arma::cx_float*>(
            calib_ul_msum_buffer_[cal_slot_complete]),
        cfg_->OfdmDataNum(), cfg_->BfAntNum(), false);

    // Update the moving sum
    if (frame_update) {
      // Add the most recently completed value
      const arma::cx_fmat cur_calib_dl_mat(
          reinterpret_cast<arma::cx_float*>(
              calib_dl_buffer_[cal_slot_complete]),
          cfg_->OfdmDataNum(), cfg_->BfAntNum(), false);
      const arma::cx_fmat cur_calib_ul_mat(
          reinterpret_cast<arma::cx_float*>(
              calib_ul_buffer_[cal_slot_complete]),
          cfg_->OfdmDataNum(), cfg_->BfAntNum(), false);

      // oldest frame data in buffer but could be partially written with newest values
      // using the second oldest....
      const size_t cal_slot_old = cfg_->ModifyRecCalIndex(cal_slot_current, +1);

      const arma::cx_fmat old_calib_dl_mat(
          reinterpret_cast<arma::cx_float*>(calib_dl_buffer_[cal_slot_old]),
          cfg_->OfdmDataNum(), cfg_->BfAntNum(), false);
      const arma::cx_fmat old_calib_ul_mat(
          reinterpret_cast<arma::cx_float*>(calib_ul_buffer_[cal_slot_old]),
          cfg_->OfdmDataNum(), cfg_->BfAntNum(), false);

      const size_t cal_slot_prev =
          cfg_->ModifyRecCalIndex(cal_slot_complete, -1);
      const arma::cx_fmat prev_calib_dl_msum_mat(
          reinterpret_cast<arma::cx_float*>(
              calib_dl_msum_buffer_[cal_slot_prev]),
          cfg_->OfdmDataNum(), cfg_->BfAntNum(), false);
      const arma::cx_fmat prev_calib_ul_msum_mat(
          reinterpret_cast<arma::cx_float*>(
              calib_ul_msum_buffer_[cal_slot_prev]),
          cfg_->OfdmDataNum(), cfg_->BfAntNum(), false);

      if (sc_id == 0) {
        AGORA_LOG_TRACE(
            "DoBeamWeights[%d]: (Frame %zu, sc_id %zu), ComputeCalib updating "
            "calib at slot %zu : prev %zu, old %zu\n",
            tid_, frame_id, sc_id, cal_slot_complete, cal_slot_prev,
            cal_slot_old);
      }

      // Add new value to old rolling sum.  Then subtract out the oldest.
      cur_calib_dl_msum_mat.row(sc_id) =
          (cur_calib_dl_mat.row(sc_id) + prev_calib_dl_msum_mat.row(sc_id)) -
          old_calib_dl_mat.row(sc_id);
      cur_calib_ul_msum_mat.row(sc_id) =
          (cur_calib_ul_mat.row(sc_id) + prev_calib_ul_msum_mat.row(sc_id)) -
          old_calib_ul_mat.row(sc_id);
    }

    calib_sc_vec =
        (cur_calib_ul_msum_mat.row(sc_id) / cur_calib_dl_msum_mat.row(sc_id))
            .st();

    if (frame_update) {
      phy_stats_->RecordCalibMat(frame_id, sc_id, calib_sc_vec);
    }
  }
  // Otherwise calib_sc_vec = identity from init
}

// Gather data of one symbol from partially-transposed buffer
// produced by dofft
static inline void PartialTransposeGather(size_t cur_sc_id, float* src,
                                          float*& dst, size_t bs_ant_num) {
  // The SIMD and non-SIMD methods are equivalent.

#ifdef __AVX512F__
  static constexpr size_t kAntNumPerSimd = 8;
#else
  static constexpr size_t kAntNumPerSimd = 4;
#endif

  size_t ant_start = 0;
  if (kUseSIMDGather && (bs_ant_num >= kAntNumPerSimd)) {
    const size_t transpose_block_id = cur_sc_id / kTransposeBlockSize;
    const size_t sc_inblock_idx = cur_sc_id % kTransposeBlockSize;
    const size_t offset_in_src_buffer =
        transpose_block_id * bs_ant_num * kTransposeBlockSize + sc_inblock_idx;

    src = src + offset_in_src_buffer * 2;
#ifdef __AVX512F__
    __m512i index = _mm512_setr_epi32(
        0, 1, kTransposeBlockSize * 2, kTransposeBlockSize * 2 + 1,
        kTransposeBlockSize * 4, kTransposeBlockSize * 4 + 1,
        kTransposeBlockSize * 6, kTransposeBlockSize * 6 + 1,
        kTransposeBlockSize * 8, kTransposeBlockSize * 8 + 1,
        kTransposeBlockSize * 10, kTransposeBlockSize * 10 + 1,
        kTransposeBlockSize * 12, kTransposeBlockSize * 12 + 1,
        kTransposeBlockSize * 14, kTransposeBlockSize * 14 + 1);
    for (size_t ant_idx = 0; ant_idx < bs_ant_num; ant_idx += kAntNumPerSimd) {
      // fetch 4 complex floats for 4 ants
      __m512 t = (kTransposeBlockSize == 1)
                     ? _mm512_load_ps(src)
                     : _mm512_i32gather_ps(index, src, 4);
      _mm512_storeu_ps(dst, t);
      src += kAntNumPerSimd * kTransposeBlockSize * 2;
      dst += kAntNumPerSimd * 2;
    }
#else
    __m256i index = _mm256_setr_epi32(
        0, 1, kTransposeBlockSize * 2, kTransposeBlockSize * 2 + 1,
        kTransposeBlockSize * 4, kTransposeBlockSize * 4 + 1,
        kTransposeBlockSize * 6, kTransposeBlockSize * 6 + 1);
    for (size_t ant_idx = 0; ant_idx < bs_ant_num; ant_idx += kAntNumPerSimd) {
      // fetch 4 complex floats for 4 ants
      __m256 t = _mm256_i32gather_ps(src, index, 4);
      _mm256_storeu_ps(dst, t);
      src += kAntNumPerSimd * kTransposeBlockSize * 2;
      dst += kAntNumPerSimd * 2;
    }
#endif
    // Set the of the remaining antennas to use non-SIMD gather
    ant_start = bs_ant_num - (bs_ant_num % kAntNumPerSimd);
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

void DoBeamWeights::ComputeFullCsiBeams(size_t tag) {
  const size_t frame_id = gen_tag_t(tag).frame_id_;
  const size_t base_sc_id = gen_tag_t(tag).sc_id_;
  const size_t frame_slot = frame_id % kFrameWnd;
  if (kDebugPrintInTask) {
    std::printf("In doZF thread %d: frame: %zu, base subcarrier: %zu\n", tid_,
                frame_id, base_sc_id);
  }
  size_t num_subcarriers =
      std::min(cfg_->BeamBlockSize(), cfg_->OfdmDataNum() - base_sc_id);

  // Handle each subcarrier one by one
  for (size_t i = 0; i < num_subcarriers; i++) {
    arma::cx_fvec& cal_sc_vec = *calib_sc_vec_ptr_;
    const size_t start_tsc1 = GetTime::WorkerRdtsc();
    const size_t cur_sc_id = base_sc_id + i;

    // Gather CSI matrices of each pilot from partially-transposed CSIs.
    for (size_t ue_idx = 0; ue_idx < cfg_->UeAntNum(); ue_idx++) {
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

    size_t start_tsc2 = GetTime::WorkerRdtsc();
    duration_stat_->task_duration_[1] += start_tsc2 - start_tsc1;

    arma::cx_fmat mat_csi((arma::cx_float*)csi_gather_buffer_, cfg_->BsAntNum(),
                          cfg_->UeAntNum(), false);

    if (cfg_->Frame().NumDLSyms() > 0) {
      ComputeCalib(frame_id, cur_sc_id, cal_sc_vec);
    }
    if (num_ext_ref_ > 0) {
      mat_csi.shed_rows(ext_ref_id_);
    }

    double start_tsc3 = GetTime::WorkerRdtsc();
    duration_stat_->task_duration_[2] += start_tsc3 - start_tsc2;

    float noise = 0;
    if (cfg_->BeamformingAlgo() == CommsLib::BeamformingAlgorithm::kMMSE) {
      noise = phy_stats_->GetNoise(frame_id);
    }
    auto rcond =
        ComputePrecoder(frame_id, cur_sc_id, mat_csi, cal_sc_vec, noise,
                        ul_beam_matrices_[frame_slot][cur_sc_id],
                        dl_beam_matrices_[frame_slot][cur_sc_id]);
    if (kPrintBeamStats) {
      phy_stats_->UpdateCsiCond(frame_id, cur_sc_id, rcond);
    }

    duration_stat_->task_duration_[3] += GetTime::WorkerRdtsc() - start_tsc3;
    duration_stat_->task_count_++;
    duration_stat_->task_duration_[0] += GetTime::WorkerRdtsc() - start_tsc1;
    // if (duration > 500) {
    //     std::printf("Thread %d ZF takes %.2f\n", tid, duration);
    // }
  }
}

void DoBeamWeights::ComputePartialCsiBeams(size_t tag) {
  const size_t frame_id = gen_tag_t(tag).frame_id_;
  const size_t base_sc_id = gen_tag_t(tag).sc_id_;
  const size_t frame_slot = frame_id % kFrameWnd;
  arma::cx_fvec& cal_sc_vec = *calib_sc_vec_ptr_;
  if (kDebugPrintInTask) {
    std::printf(
        "In doZF thread %d: frame: %zu, subcarrier: %zu, block: %zu, "
        "Basestation ant number: %zu\n",
        tid_, frame_id, base_sc_id, base_sc_id / cfg_->UeAntNum(),
        cfg_->BsAntNum());
  }

  double start_tsc1 = GetTime::WorkerRdtsc();

  // Gather CSIs from partially-transposed CSIs
  for (size_t i = 0; i < cfg_->UeAntNum(); i++) {
    const size_t cur_sc_id = base_sc_id + i;
    auto* dst_csi_ptr =
        reinterpret_cast<float*>(csi_gather_buffer_ + cfg_->BsAntNum() * i);
    PartialTransposeGather(cur_sc_id, (float*)csi_buffers_[frame_slot][0],
                           dst_csi_ptr, cfg_->BsAntNum());
  }

  size_t start_tsc2 = GetTime::WorkerRdtsc();
  duration_stat_->task_duration_[1] += start_tsc2 - start_tsc1;

  if (cfg_->Frame().NumDLSyms() > 0) {
    size_t cal_slot_current;
    if (cfg_->Frame().IsRecCalEnabled()) {
      cal_slot_current = cfg_->RecipCalIndex(frame_id);
    } else {
      cal_slot_current = frame_id;
    }

    // use the previous window which has a full set of calibration results
    const size_t cal_slot_complete =
        cfg_->ModifyRecCalIndex(cal_slot_current, -1);
    const size_t cal_slot_prev = cfg_->ModifyRecCalIndex(cal_slot_current, -2);

    const arma::cx_fmat calib_dl_mat(
        reinterpret_cast<arma::cx_float*>(calib_dl_buffer_[cal_slot_complete]),
        cfg_->OfdmDataNum(), cfg_->BfAntNum(), false);
    const arma::cx_fmat calib_ul_mat(
        reinterpret_cast<arma::cx_float*>(calib_ul_buffer_[cal_slot_complete]),
        cfg_->OfdmDataNum(), cfg_->BfAntNum(), false);
    const arma::cx_fmat calib_dl_mat_prev(
        reinterpret_cast<arma::cx_float*>(calib_dl_buffer_[cal_slot_prev]),
        cfg_->OfdmDataNum(), cfg_->BfAntNum(), false);
    const arma::cx_fmat calib_ul_mat_prev(
        reinterpret_cast<arma::cx_float*>(calib_ul_buffer_[cal_slot_prev]),
        cfg_->OfdmDataNum(), cfg_->BfAntNum(), false);
    arma::cx_fvec calib_dl_vec =
        (calib_dl_mat.row(base_sc_id) + calib_dl_mat_prev.row(base_sc_id)).st();
    arma::cx_fvec calib_ul_vec =
        (calib_ul_mat.row(base_sc_id) + calib_ul_mat_prev.row(base_sc_id)).st();
    cal_sc_vec = calib_dl_vec / calib_ul_vec;
  }

  double start_tsc3 = GetTime::WorkerRdtsc();
  duration_stat_->task_duration_[2] += start_tsc3 - start_tsc2;

  arma::cx_fmat mat_csi(reinterpret_cast<arma::cx_float*>(csi_gather_buffer_),
                        cfg_->BsAntNum(), cfg_->UeAntNum(), false);

  float noise = 0;
  if (cfg_->BeamformingAlgo() == CommsLib::BeamformingAlgorithm::kMMSE) {
    noise = phy_stats_->GetNoise(frame_id);
  }
  ComputePrecoder(frame_id, base_sc_id, mat_csi, cal_sc_vec, noise,
                  ul_beam_matrices_[frame_slot][cfg_->GetBeamScId(base_sc_id)],
                  dl_beam_matrices_[frame_slot][cfg_->GetBeamScId(base_sc_id)]);

  duration_stat_->task_duration_[3] += GetTime::WorkerRdtsc() - start_tsc3;
  duration_stat_->task_count_++;
  duration_stat_->task_duration_[0] += GetTime::WorkerRdtsc() - start_tsc1;

  // if (duration > 500) {
  //     std::printf("Thread %d ZF takes %.2f\n", tid, duration);
  // }
}

// Currently unused
/*
void DoBeamWeights::Predict(size_t tag)
{
    size_t frame_id = gen_tag_t(tag).frame_id;
    size_t base_sc_id = gen_tag_t(tag).sc_id;

    // Use stale CSI as predicted CSI
    // TODO: add prediction algorithm
    const size_t offset_in_buffer
        = ((frame_id % kFrameWnd) * cfg_->OfdmDataNum())
        + base_sc_id;
    auto* ptr_in = (arma::cx_float*)pred_csi_buffer;
    std::memcpy(ptr_in, (arma::cx_float*)csi_buffer_[offset_in_buffer],
        sizeof(arma::cx_float) * cfg->BsAntNum() * cfg->UE_NUM);
    arma::cx_fmat mat_input(ptr_in, cfg->BsAntNum(), cfg->UE_NUM, false);

    // Input matrix and calibration are for current frame, output precoders are
    // for the next frame
    compute_precoder(mat_input,
        cfg_->GetCalibBuffer(calib_buffer_, frame_id, base_sc_id),
        cfg_->get_ul_beam_mat(ul_beam_buffer_, frame_id + 1, base_sc_id),
        cfg_->get_dl_beam_mat(dl_beam_buffer_, frame_id + 1, base_sc_id));
}
*/
