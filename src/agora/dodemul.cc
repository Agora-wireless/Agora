/**
 * @file dodemul.cc
 * @brief Implmentation file for the DoDemul class.
 */
#include "dodemul.h"

#include "comms-lib.h"
#include "concurrent_queue_wrapper.h"
#include "modulation.h"

static constexpr bool kUseSIMDGather = true;

DoDemul::DoDemul(
    Config* config, int tid, Table<complex_float>& data_buffer,
    PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& ul_beam_matrices,
    Table<complex_float>& ue_spec_pilot_buffer,
    Table<complex_float>& equal_buffer,
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& demod_buffers,
    MacScheduler* mac_sched, PhyStats* in_phy_stats, Stats* stats_manager)
    : Doer(config, tid),
      data_buffer_(data_buffer),
      ul_beam_matrices_(ul_beam_matrices),
      ue_spec_pilot_buffer_(ue_spec_pilot_buffer),
      equal_buffer_(equal_buffer),
      demod_buffers_(demod_buffers),
      mac_sched_(mac_sched),
      phy_stats_(in_phy_stats) {
  duration_stat_equal_ = stats_manager->GetDurationStat(DoerType::kEqual, tid);
  duration_stat_demul_ = stats_manager->GetDurationStat(DoerType::kDemul, tid);

  data_gather_buffer_ =
      static_cast<complex_float*>(Agora_memory::PaddedAlignedAlloc(
          Agora_memory::Alignment_t::kAlign64,
          kSCsPerCacheline * kMaxAntennas * sizeof(complex_float)));
  equaled_buffer_temp_ =
      static_cast<complex_float*>(Agora_memory::PaddedAlignedAlloc(
          Agora_memory::Alignment_t::kAlign64,
          cfg_->DemulBlockSize() * kMaxUEs * sizeof(complex_float)));
  equaled_buffer_temp_transposed_ =
      static_cast<complex_float*>(Agora_memory::PaddedAlignedAlloc(
          Agora_memory::Alignment_t::kAlign64,
          cfg_->DemulBlockSize() * kMaxUEs * sizeof(complex_float)));

  // phase offset calibration data
  arma::cx_float* ue_pilot_ptr =
      reinterpret_cast<arma::cx_float*>(cfg_->UeSpecificPilot()[0]);
  arma::cx_fmat mat_pilot_data(ue_pilot_ptr, cfg_->OfdmDataNum(),
                               cfg_->UeAntNum(), false);
  ue_pilot_data_ = mat_pilot_data.st();
  arma::cx_fvec vec_pilot_data_(ue_pilot_ptr, cfg_->OfdmDataNum(), false);
  vec_pilot_data = vec_pilot_data_;

#if defined(USE_MKL_JIT)
  MKL_Complex8 alpha = {1, 0};
  MKL_Complex8 beta = {0, 0};

  mkl_jit_status_t status =
      mkl_jit_create_cgemm(&jitter_, MKL_COL_MAJOR, MKL_NOTRANS, MKL_NOTRANS,
                           cfg_->SpatialStreamsNum(), 1, cfg_->BsAntNum(),
                           &alpha, cfg_->SpatialStreamsNum(), cfg_->BsAntNum(),
                           &beta, cfg_->SpatialStreamsNum());
  if (MKL_JIT_ERROR == status) {
    std::fprintf(
        stderr,
        "Error: insufficient memory to JIT and store the DGEMM kernel\n");
    throw std::runtime_error(
        "DoDemul: insufficient memory to JIT and store the DGEMM kernel");
  }
  mkl_jit_cgemm_ = mkl_jit_get_cgemm_ptr(jitter_);
#endif

  // // Init LUT for sin/cos
  // sin_lut.set_size(lut_size);
  // cos_lut.set_size(lut_size);
  // for (int i = 0; i < lut_size; ++i) {
  //   double angle = i * lut_step;
  //   sin_lut(i) = std::sin(angle);
  //   cos_lut(i) = std::cos(angle);
  // }
}

DoDemul::~DoDemul() {
  std::free(data_gather_buffer_);
  std::free(equaled_buffer_temp_);
  std::free(equaled_buffer_temp_transposed_);

#if defined(USE_MKL_JIT)
  mkl_jit_status_t status = mkl_jit_destroy(jitter_);
  if (MKL_JIT_ERROR == status) {
    std::fprintf(stderr, "!!!!Error: Error while destorying MKL JIT\n");
  }
#endif
}

EventData DoDemul::Launch(size_t tag) {
  const size_t frame_id = gen_tag_t(tag).frame_id_;
  const size_t symbol_id = gen_tag_t(tag).symbol_id_;
  const size_t base_sc_id = gen_tag_t(tag).sc_id_;

  const size_t symbol_idx_ul = this->cfg_->Frame().GetULSymbolIdx(symbol_id);
  const size_t total_data_symbol_idx_ul =
      cfg_->GetTotalDataSymbolIdxUl(frame_id, symbol_idx_ul);
  const complex_float* data_buf = data_buffer_[total_data_symbol_idx_ul];

  const size_t frame_slot = frame_id % kFrameWnd;
  size_t start_equal_tsc = GetTime::WorkerRdtsc();

  if (kDebugPrintInTask == true) {
    std::printf(
        "In doDemul tid %d: frame: %zu, symbol idx: %zu, symbol idx ul: %zu, "
        "subcarrier: %zu, databuffer idx %zu \n",
        tid_, frame_id, symbol_id, symbol_idx_ul, base_sc_id,
        total_data_symbol_idx_ul);
  }

  size_t max_sc_ite =
      std::min(cfg_->DemulBlockSize(), cfg_->OfdmDataNum() - base_sc_id);
  assert(max_sc_ite % kSCsPerCacheline == 0);

  // Accelerate (vectorized computation) 1x1 antenna config
  if (cfg_->UeAntNum() == 1 && cfg_->BsAntNum() == 1) {

    // Step 1: Equalization
    arma::cx_float* equal_ptr = nullptr;
    if (kExportConstellation) {
      equal_ptr = (arma::cx_float*)(&equal_buffer_[total_data_symbol_idx_ul]
                                                  [base_sc_id]);
    } else {
      equal_ptr = (arma::cx_float*)(&equaled_buffer_temp_[0]);
    }
    arma::cx_fvec vec_equaled(equal_ptr, max_sc_ite, false);

    arma::cx_float* data_ptr = (arma::cx_float*)(&data_buf[base_sc_id]);
    // not consider multi-antenna case (antena offset is omitted)
    arma::cx_float* ul_beam_ptr = reinterpret_cast<arma::cx_float*>(
        ul_beam_matrices_[frame_slot][0]); // pick the first element

    // assuming cfg->BsAntNum() == 1, reducing a dimension
    arma::cx_fvec vec_data(data_ptr, max_sc_ite, false);
    arma::cx_fvec vec_ul_beam(max_sc_ite); // init empty vec
    for (size_t i = 0; i < max_sc_ite; ++i) {
      vec_ul_beam(i) = ul_beam_ptr[cfg_->GetBeamScId(base_sc_id + i)];
    }
    vec_equaled = vec_ul_beam % vec_data;

    // Step 2: Phase shift calibration

    // Enable phase shift calibration
    if (cfg_->Frame().ClientUlPilotSymbols() > 0) {

      // Reset previous frame
      if (symbol_idx_ul == 0 && base_sc_id == 0) {
        arma::cx_float* phase_shift_ptr = reinterpret_cast<arma::cx_float*>(
            ue_spec_pilot_buffer_[(frame_id - 1) % kFrameWnd]);
        arma::cx_fmat mat_phase_shift(phase_shift_ptr, cfg_->UeAntNum(),
                                      cfg_->Frame().ClientUlPilotSymbols(),
                                      false);
        mat_phase_shift.fill(0);
      }

      // Calc new phase shift
      if (symbol_idx_ul < cfg_->Frame().ClientUlPilotSymbols()) {
        arma::cx_float* phase_shift_ptr = reinterpret_cast<arma::cx_float*>(
          &ue_spec_pilot_buffer_[frame_id % kFrameWnd]
                                [symbol_idx_ul * cfg_->UeAntNum()]);
        arma::cx_fmat mat_phase_shift(phase_shift_ptr, cfg_->UeAntNum(), 1,
                                  false);
        arma::cx_fvec vec_ue_pilot_data_ = vec_pilot_data.subvec(base_sc_id, base_sc_id+max_sc_ite-1);

        mat_phase_shift += sum(vec_equaled % conj(vec_ue_pilot_data_));
        // mat_phase_shift += sum(sign(vec_equaled % conj(vec_ue_pilot_data_)));
        // sign should be able to optimize out but the result will be different
      }

      // Calculate the unit phase shift based on the first subcarrier
      // Check the special case condition to avoid reading wrong memory location
      RtAssert(cfg_->UeAntNum() == 1 && cfg_->Frame().ClientUlPilotSymbols() == 2);
      if (symbol_idx_ul == cfg_->Frame().ClientUlPilotSymbols() && base_sc_id == 0) { 
        arma::cx_float* pilot_corr_ptr = reinterpret_cast<arma::cx_float*>(
            ue_spec_pilot_buffer_[frame_id % kFrameWnd]);
        arma::cx_fvec pilot_corr_vec(pilot_corr_ptr,
                                    cfg_->Frame().ClientUlPilotSymbols(), false);
        theta_vec = arg(pilot_corr_vec);
        theta_inc_f = theta_vec(cfg_->Frame().ClientUlPilotSymbols()-1) - theta_vec(0);
        // theta_inc /= (float)std::max(
        //     1, static_cast<int>(cfg_->Frame().ClientUlPilotSymbols() - 1));
      }

      // Apply previously calc'ed phase shift to data
      if (symbol_idx_ul >= cfg_->Frame().ClientUlPilotSymbols()) {
        float cur_theta_f = theta_vec(0) + (symbol_idx_ul * theta_inc_f);
        vec_equaled *= arma::cx_float(cos(-cur_theta_f), sin(-cur_theta_f));
      }

      // Not update EVM for the special, time-exclusive case

      duration_stat_equal_->task_count_++;
    }
  } else if (cfg_->UeAntNum() == 2 && cfg_->BsAntNum() == 2) {
    // Accelerate (vectorized computation) 2x2 antenna config
    // Step 0: Re-arrange data
    complex_float* dst = data_gather_buffer_;
    for (size_t i = 0; i < max_sc_ite; i++) {
      const size_t partial_transpose_block_base =
          ((base_sc_id + i) / kTransposeBlockSize) *
          (kTransposeBlockSize * cfg_->BsAntNum());

      // Populate data_gather_buffer as a row-major matrix with max_sc_ite rows
      // and BsAntNum() columns

      for (size_t ant_i = 0; ant_i < cfg_->BsAntNum(); ant_i++) {
        *dst++ = kUsePartialTrans
                    ? data_buf[partial_transpose_block_base +
                              (ant_i * kTransposeBlockSize) +
                              ((base_sc_id + i) % kTransposeBlockSize)]
                    : data_buf[ant_i * cfg_->OfdmDataNum() + base_sc_id + i];
      }
    }
    arma::cx_float* data_ptr =
      (arma::cx_float*)(&data_gather_buffer_[base_sc_id]);
    arma::cx_fcube cub_data(data_ptr, cfg_->BsAntNum(), 1, max_sc_ite, false);
    // cub_data.print("cub_data");

    // arma::cx_fcube cub_ul_beam(cfg_->UeAntNum(), cfg_->BsAntNum(), max_sc_ite);
    // for (size_t i = 0; i < max_sc_ite; ++i) {
    //   arma::cx_float* ul_beam_ptr = reinterpret_cast<arma::cx_float*>(
    //     ul_beam_matrices_[frame_slot][cfg_->GetBeamScId(base_sc_id + i)]);
    //   arma::cx_fmat mat_ul_beam(ul_beam_ptr,
    //                             cfg_->UeAntNum(), cfg_->BsAntNum(), false);
    //   cub_ul_beam.slice(i) = mat_ul_beam;
    // }
    arma::cx_float* ul_beam_ptr = reinterpret_cast<arma::cx_float*>(
      ul_beam_matrices_[frame_slot][cfg_->GetBeamScId(base_sc_id)]);
    arma::cx_fcube cub_ul_beam(ul_beam_ptr, cfg_->UeAntNum(),
                               cfg_->BsAntNum(), max_sc_ite, false);

    // Step 1: Equalization
    arma::cx_float* equal_ptr = nullptr;
    if (kExportConstellation) {
      equal_ptr = (arma::cx_float*)(&equal_buffer_[total_data_symbol_idx_ul]
                                                  [base_sc_id]);
    } else {
      equal_ptr = (arma::cx_float*)(&equaled_buffer_temp_[0]);
    }
    arma::cx_fcube cub_equaled(equal_ptr, cfg_->BsAntNum(), 1, max_sc_ite, false);
    // cub_equaled.print("cub_equaled");

    // for (size_t i = 0; i < max_sc_ite; ++i) {
    //   cub_equaled.slice(i) = cub_ul_beam.slice(i) * cub_data.slice(i);
    // }
    cub_equaled.tube(0, 0) =
      cub_ul_beam.tube(0, 0) % cub_data.tube(0, 0) +
      cub_ul_beam.tube(0, 1) % cub_data.tube(1, 0);
    cub_equaled.tube(1, 0) =
      cub_ul_beam.tube(1, 0) % cub_data.tube(0, 0) +
      cub_ul_beam.tube(1, 1) % cub_data.tube(1, 0);

    // Step 2: Phase shift calibration

    // Enable phase shift calibration
    if (cfg_->Frame().ClientUlPilotSymbols() > 0) {

      if (symbol_idx_ul == 0 && base_sc_id == 0) {
        // Reset previous frame
        arma::cx_float* phase_shift_ptr = reinterpret_cast<arma::cx_float*>(
            ue_spec_pilot_buffer_[(frame_id - 1) % kFrameWnd]);
        arma::cx_fmat mat_phase_shift(phase_shift_ptr, cfg_->UeAntNum(),
                                      cfg_->Frame().ClientUlPilotSymbols(),
                                      false);
        mat_phase_shift.fill(0);
      }

      // Calc new phase shift
      if (symbol_idx_ul < cfg_->Frame().ClientUlPilotSymbols()) {
        arma::cx_float* phase_shift_ptr = reinterpret_cast<arma::cx_float*>(
          &ue_spec_pilot_buffer_[frame_id % kFrameWnd]
                                [symbol_idx_ul * cfg_->UeAntNum()]);
        arma::cx_fmat mat_phase_shift(phase_shift_ptr, cfg_->UeAntNum(), 1,
                                  false);
        // printf("base_sc_id = %ld, end = %ld\n", base_sc_id, base_sc_id+max_sc_ite-1);
        // arma::cx_fcube cub_ue_pilot_data_ =
        //   cub_pilot_data.slices(base_sc_id, base_sc_id+max_sc_ite-1);
        // TODO: data ordering issue to solve

        arma::cx_fmat mat_ue_pilot_data_ =
          ue_pilot_data_.cols(base_sc_id, base_sc_id+max_sc_ite-1);
        // if use fvec or fcolvec, then transpose mat_ue_pilot_data_ by
        // mat_ue_pilot_data_.row(0).st()
        arma::cx_frowvec vec_tube_equal_0 =
          cub_equaled(arma::span(0), arma::span(0), arma::span::all);
        arma::cx_frowvec vec_tube_equal_1 =
          cub_equaled(arma::span(1), arma::span(0), arma::span::all);

        mat_phase_shift.col(0).row(0) += arma::sum(
          vec_tube_equal_0 % arma::conj(mat_ue_pilot_data_.row(0))
        );
        mat_phase_shift.col(0).row(1) += arma::sum(
          vec_tube_equal_1 % arma::conj(mat_ue_pilot_data_.row(1))
        );

        // for (size_t i = 0; i < max_sc_ite; ++i) {
        //   arma::cx_fmat shift_sc =
        //     // cub_equaled.slice(i) % arma::conj(cub_ue_pilot_data_.slice(i));
        //     cub_equaled.slice(i) % arma::conj(ue_pilot_data_.col(i));
        //     // cub_equaled.slice(i) % sign(conj(mat_ue_pilot_data_.slice(i)));
        //   mat_phase_shift += shift_sc;
        // }
        // mat_phase_shift += sum(vec_equaled % conj(vec_ue_pilot_data_));
        // mat_phase_shift += sum(sign(vec_equaled % conj(vec_ue_pilot_data_)));
        // sign should be able to optimize out but the result will be different
      }

      // Calculate the unit phase shift based on the first subcarrier
      // Check the special case condition to avoid reading wrong memory location
      RtAssert(cfg_->UeAntNum() == 2 &&
              cfg_->Frame().ClientUlPilotSymbols() == 2);
      if (symbol_idx_ul == cfg_->Frame().ClientUlPilotSymbols() &&
          base_sc_id == 0) { 
        arma::cx_float* pilot_corr_ptr = reinterpret_cast<arma::cx_float*>(
            ue_spec_pilot_buffer_[frame_id % kFrameWnd]);
        arma::cx_fmat pilot_corr_mat(pilot_corr_ptr, cfg_->UeAntNum(),
                                        cfg_->Frame().ClientUlPilotSymbols(),
                                        false);
        theta_mat = arg(pilot_corr_mat);
        theta_inc =
            theta_mat.col(cfg_->Frame().ClientUlPilotSymbols()-1) -
            theta_mat.col(0);
        // theta_inc /= (float)std::max(
        //     1, static_cast<int>(cfg_->Frame().ClientUlPilotSymbols() - 1));
      }

      // Apply previously calc'ed phase shift to data
      if (symbol_idx_ul >= cfg_->Frame().ClientUlPilotSymbols()) {
        arma::fmat cur_theta = theta_mat.col(0) + (symbol_idx_ul * theta_inc);
        arma::cx_fmat mat_phase_correct =
            arma::cx_fmat(cos(-cur_theta), sin(-cur_theta));
        cub_equaled.each_slice() %= mat_phase_correct;
        // mat_equaled %= mat_phase_correct;
        // vec_equaled *= arma::cx_float(cos(-cur_theta_f), sin(-cur_theta_f));
      }

      duration_stat_equal_->task_count_++;
    }
  } else {

    // Iterate through cache lines
    for (size_t i = 0; i < max_sc_ite; i += kSCsPerCacheline) {
      // size_t start_equal_tsc0 = GetTime::WorkerRdtsc();

      // Step 1: Populate data_gather_buffer as a row-major matrix with
      // kSCsPerCacheline rows and BsAntNum() columns

      // Since kSCsPerCacheline divides demul_block_size and
      // kTransposeBlockSize, all subcarriers (base_sc_id + i) lie in the
      // same partial transpose block.
      const size_t partial_transpose_block_base =
          ((base_sc_id + i) / kTransposeBlockSize) *
          (kTransposeBlockSize * cfg_->BsAntNum());

#ifdef __AVX512F__
      static constexpr size_t kAntNumPerSimd = 8;
#else
      static constexpr size_t kAntNumPerSimd = 4;
#endif

      size_t ant_start = 0;
      if (kUseSIMDGather && kUsePartialTrans &&
          (cfg_->BsAntNum() % kAntNumPerSimd) == 0) {
        // Gather data for all antennas and 8 subcarriers in the same cache
        // line, 1 subcarrier and 4 (AVX2) or 8 (AVX512) ants per iteration
        size_t cur_sc_offset =
            partial_transpose_block_base + (base_sc_id + i) % kTransposeBlockSize;
        const float* src =
            reinterpret_cast<const float*>(&data_buf[cur_sc_offset]);
        float* dst = reinterpret_cast<float*>(data_gather_buffer_);
#ifdef __AVX512F__
        __m512i index = _mm512_setr_epi32(
            0, 1, kTransposeBlockSize * 2, kTransposeBlockSize * 2 + 1,
            kTransposeBlockSize * 4, kTransposeBlockSize * 4 + 1,
            kTransposeBlockSize * 6, kTransposeBlockSize * 6 + 1,
            kTransposeBlockSize * 8, kTransposeBlockSize * 8 + 1,
            kTransposeBlockSize * 10, kTransposeBlockSize * 10 + 1,
            kTransposeBlockSize * 12, kTransposeBlockSize * 12 + 1,
            kTransposeBlockSize * 14, kTransposeBlockSize * 14 + 1);
        for (size_t ant_i = 0; ant_i < cfg_->BsAntNum();
            ant_i += kAntNumPerSimd) {
          for (size_t j = 0; j < kSCsPerCacheline; j++) {
            __m512 data_rx = kTransposeBlockSize == 1
                                ? _mm512_load_ps(&src[j * cfg_->BsAntNum() * 2])
                                : _mm512_i32gather_ps(index, &src[j * 2], 4);

            assert((reinterpret_cast<intptr_t>(&dst[j * cfg_->BsAntNum() * 2]) %
                    (kAntNumPerSimd * sizeof(float) * 2)) == 0);
            assert((reinterpret_cast<intptr_t>(&src[j * cfg_->BsAntNum() * 2]) %
                    (kAntNumPerSimd * sizeof(float) * 2)) == 0);
            _mm512_store_ps(&dst[j * cfg_->BsAntNum() * 2], data_rx);
          }
          src += kAntNumPerSimd * kTransposeBlockSize * 2;
          dst += kAntNumPerSimd * 2;
        }
#else
        __m256i index = _mm256_setr_epi32(
            0, 1, kTransposeBlockSize * 2, kTransposeBlockSize * 2 + 1,
            kTransposeBlockSize * 4, kTransposeBlockSize * 4 + 1,
            kTransposeBlockSize * 6, kTransposeBlockSize * 6 + 1);
        for (size_t ant_i = 0; ant_i < cfg_->BsAntNum();
            ant_i += kAntNumPerSimd) {
          for (size_t j = 0; j < kSCsPerCacheline; j++) {
            assert((reinterpret_cast<intptr_t>(&dst[j * cfg_->BsAntNum() * 2]) %
                    (kAntNumPerSimd * sizeof(float) * 2)) == 0);
            __m256 data_rx = _mm256_i32gather_ps(&src[j * 2], index, 4);
            _mm256_store_ps(&dst[j * cfg_->BsAntNum() * 2], data_rx);
          }
          src += kAntNumPerSimd * kTransposeBlockSize * 2;
          dst += kAntNumPerSimd * 2;
        }
#endif
        // Set the remaining number of antennas for non-SIMD gather
        ant_start = cfg_->BsAntNum() - (cfg_->BsAntNum() % kAntNumPerSimd);
      }
      if (ant_start < cfg_->BsAntNum()) {
        complex_float* dst = data_gather_buffer_ + ant_start;
        for (size_t j = 0; j < kSCsPerCacheline; j++) {
          for (size_t ant_i = ant_start; ant_i < cfg_->BsAntNum(); ant_i++) {
            *dst++ =
                kUsePartialTrans
                    ? data_buf[partial_transpose_block_base +
                              (ant_i * kTransposeBlockSize) +
                              ((base_sc_id + i + j) % kTransposeBlockSize)]
                    : data_buf[ant_i * cfg_->OfdmDataNum() + base_sc_id + i + j];
          }
        }
      }

      // Step 2: For each subcarrier, perform equalization by multiplying the
      // subcarrier's data from each antenna with the subcarrier's precoder
      for (size_t j = 0; j < kSCsPerCacheline; j++) {
        const size_t cur_sc_id = base_sc_id + i + j;

        arma::cx_float* equal_ptr = nullptr;
        if (kExportConstellation) {
          equal_ptr =
              (arma::cx_float*)(&equal_buffer_[total_data_symbol_idx_ul]
                                              [cur_sc_id *
                                              cfg_->SpatialStreamsNum()]);
        } else {
          equal_ptr =
              (arma::cx_float*)(&equaled_buffer_temp_[(cur_sc_id - base_sc_id) *
                                                      cfg_->SpatialStreamsNum()]);
        }
        arma::cx_fmat mat_equaled(equal_ptr, cfg_->SpatialStreamsNum(), 1, false);

        arma::cx_float* data_ptr = reinterpret_cast<arma::cx_float*>(
            &data_gather_buffer_[j * cfg_->BsAntNum()]);
        arma::cx_float* ul_beam_ptr = reinterpret_cast<arma::cx_float*>(
            ul_beam_matrices_[frame_slot][cfg_->GetBeamScId(cur_sc_id)]);

#if defined(USE_MKL_JIT)
        mkl_jit_cgemm_(jitter_, (MKL_Complex8*)ul_beam_ptr,
                      (MKL_Complex8*)data_ptr, (MKL_Complex8*)equal_ptr);
#else
        arma::cx_fmat mat_data(data_ptr, cfg_->BsAntNum(), 1, false);

        arma::cx_fmat mat_ul_beam(ul_beam_ptr, cfg_->SpatialStreamsNum(),
                                  cfg_->BsAntNum(), false);
        mat_equaled = mat_ul_beam * mat_data;
#endif

        // Enable phase shift calibration
        if (cfg_->Frame().ClientUlPilotSymbols() > 0) {
          // Calc new phase shift
          if (symbol_idx_ul < cfg_->Frame().ClientUlPilotSymbols()) {  
            if (symbol_idx_ul == 0 && cur_sc_id == 0) {
              // Reset previous frame
              arma::cx_float* phase_shift_ptr = reinterpret_cast<arma::cx_float*>(
                  ue_spec_pilot_buffer_[(frame_id - 1) % kFrameWnd]);
              arma::cx_fmat mat_phase_shift(phase_shift_ptr, cfg_->UeAntNum(),
                                            cfg_->Frame().ClientUlPilotSymbols(),
                                            false);
              mat_phase_shift.fill(0);
            }
            arma::cx_float* phase_shift_ptr = reinterpret_cast<arma::cx_float*>(
                &ue_spec_pilot_buffer_[frame_id % kFrameWnd]
                                      [symbol_idx_ul * cfg_->UeAntNum()]);
            arma::cx_fmat mat_phase_shift(phase_shift_ptr, cfg_->UeAntNum(), 1,
                                          false);
            arma::cx_fmat shift_sc =
                sign(mat_equaled % conj(ue_pilot_data_.col(cur_sc_id)));
            mat_phase_shift += shift_sc;
          }
          if (symbol_idx_ul == cfg_->Frame().ClientUlPilotSymbols() && cur_sc_id == 0) { 
            arma::cx_float* pilot_corr_ptr = reinterpret_cast<arma::cx_float*>(
                ue_spec_pilot_buffer_[frame_id % kFrameWnd]);
            arma::cx_fmat pilot_corr_mat(pilot_corr_ptr, cfg_->UeAntNum(),
                                        cfg_->Frame().ClientUlPilotSymbols(),
                                        false);
            theta_mat = arg(pilot_corr_mat);
            theta_inc = theta_mat.col(cfg_->Frame().ClientUlPilotSymbols()-1) - theta_mat.col(0);
            theta_inc /= (float)std::max(
                1, static_cast<int>(cfg_->Frame().ClientUlPilotSymbols() - 1));
          }

          // apply previously calc'ed phase shift to data
          if (symbol_idx_ul >= cfg_->Frame().ClientUlPilotSymbols()) {
            arma::fmat cur_theta = theta_mat.col(0) + (symbol_idx_ul * theta_inc);
            arma::cx_fmat mat_phase_correct = arma::cx_fmat(cos(-cur_theta), sin(-cur_theta));
            // arma::cx_fmat mat_phase_correct = arma::cx_fmat(cos_lut_func(-cur_theta), sin_lut_func(-cur_theta));
            mat_equaled %= mat_phase_correct;

#if !defined(TIME_EXCLUSIVE)
            auto ue_list = mac_sched_->ScheduledUeList(frame_id, cur_sc_id);
            const size_t data_symbol_idx_ul =
                symbol_idx_ul - this->cfg_->Frame().ClientUlPilotSymbols();
            // Measure EVM from ground truth
            phy_stats_->UpdateEvm(frame_id, data_symbol_idx_ul, cur_sc_id,
                                  mat_equaled.col(0), ue_list);
#endif
          }
        }
        duration_stat_equal_->task_count_++;
      }
    }
  }

  duration_stat_equal_->task_duration_[0] += GetTime::WorkerRdtsc() - start_equal_tsc;
  size_t start_demul_tsc = GetTime::WorkerRdtsc();

  __m256i index2 = _mm256_setr_epi32(
      0, 1, cfg_->SpatialStreamsNum() * 2, cfg_->SpatialStreamsNum() * 2 + 1,
      cfg_->SpatialStreamsNum() * 4, cfg_->SpatialStreamsNum() * 4 + 1,
      cfg_->SpatialStreamsNum() * 6, cfg_->SpatialStreamsNum() * 6 + 1);
  auto* equal_t_ptr = reinterpret_cast<float*>(equaled_buffer_temp_transposed_);
  for (size_t ss_id = 0; ss_id < cfg_->SpatialStreamsNum(); ss_id++) {
    float* equal_ptr = nullptr;
    if (kExportConstellation) {
      equal_ptr = reinterpret_cast<float*>(
          &equal_buffer_[total_data_symbol_idx_ul]
                        [base_sc_id * cfg_->SpatialStreamsNum() + ss_id]);
    } else {
      equal_ptr = reinterpret_cast<float*>(equaled_buffer_temp_ + ss_id);
    }
    size_t k_num_double_in_sim_d256 = sizeof(__m256) / sizeof(double);  // == 4
    for (size_t j = 0; j < max_sc_ite / k_num_double_in_sim_d256; j++) {
      __m256 equal_t_temp = _mm256_i32gather_ps(equal_ptr, index2, 4);
      _mm256_store_ps(equal_t_ptr, equal_t_temp);
      equal_t_ptr += 8;
      equal_ptr += cfg_->SpatialStreamsNum() * k_num_double_in_sim_d256 * 2;
    }
    equal_t_ptr = (float*)(equaled_buffer_temp_transposed_);
    int8_t* demod_ptr = demod_buffers_[frame_slot][symbol_idx_ul][ss_id] +
                        (cfg_->ModOrderBits(Direction::kUplink) * base_sc_id);
    size_t start_demul_tsc0 = GetTime::WorkerRdtsc();
    Demodulate(equal_t_ptr, demod_ptr, max_sc_ite,
               cfg_->ModOrderBits(Direction::kUplink), kUplinkHardDemod);
    duration_stat_demul_->task_duration_[1] = GetTime::WorkerRdtsc() - start_demul_tsc0;
    duration_stat_demul_->task_count_++;
    // if hard demod is enabled calculate BER with modulated bits
    if (((kPrintPhyStats || kEnableCsvLog) && kUplinkHardDemod) &&
        (symbol_idx_ul >= cfg_->Frame().ClientUlPilotSymbols())) {
      size_t ue_id = mac_sched_->ScheduledUeIndex(frame_id, base_sc_id, ss_id);
      phy_stats_->UpdateDecodedBits(
          ue_id, total_data_symbol_idx_ul, frame_slot,
          max_sc_ite * cfg_->ModOrderBits(Direction::kUplink));
      // Each block here is max_sc_ite
      phy_stats_->IncrementDecodedBlocks(ue_id, total_data_symbol_idx_ul,
                                         frame_slot);
      size_t block_error(0);
      int8_t* tx_bytes =
          cfg_->GetModBitsBuf(cfg_->UlModBits(), Direction::kUplink, 0,
                              symbol_idx_ul, ue_id, base_sc_id);
      for (size_t i = 0; i < max_sc_ite; i++) {
        uint8_t rx_byte = static_cast<uint8_t>(demod_ptr[i]);
        uint8_t tx_byte = static_cast<uint8_t>(tx_bytes[i]);
        phy_stats_->UpdateBitErrors(ue_id, total_data_symbol_idx_ul, frame_slot,
                                    tx_byte, rx_byte);
        if (rx_byte != tx_byte) {
          block_error++;
        }
      }
      phy_stats_->UpdateBlockErrors(ue_id, total_data_symbol_idx_ul, frame_slot,
                                    block_error);
    }

    // std::printf("In doDemul thread %d: frame: %d, symbol: %d, sc_id: %d \n",
    //     tid, frame_id, symbol_idx_ul, base_sc_id);
    // cout << "Demuled data : \n ";
    // cout << " UE " << ue_id << ": ";
    // for (int k = 0; k < max_sc_ite * cfg->ModOrderBits(Direction::kUplink); k++)
    //   std::printf("%i ", demul_ptr[k]);
    // cout << endl;
  }
  duration_stat_demul_->task_duration_[0] += GetTime::WorkerRdtsc() - start_demul_tsc;
  return EventData(EventType::kDemul, tag);
}

// // Look-Up Table-based sine function
// inline arma::fmat DoDemul::sin_lut_func(const arma::fmat& angles) {
//   arma::fmat result(angles.n_rows, angles.n_cols);
//   for (int i = 0; i < angles.n_elem; ++i) {
//     int index = static_cast<int>((angles(i) / lut_max_angle) * lut_size) % lut_size;
//     result(i) = sin_lut(index);
//   }
//   return result;
// }

// // Look-Up Table-based cosine function
// inline arma::fmat DoDemul::cos_lut_func(const arma::fmat& angles) {
//   arma::fmat result(angles.n_rows, angles.n_cols);
//   for (int i = 0; i < angles.n_elem; ++i) {
//     int index = static_cast<int>((angles(i) / lut_max_angle) * lut_size) % lut_size;
//     result(i) = cos_lut(index);
//   }
//   return result;
// }
