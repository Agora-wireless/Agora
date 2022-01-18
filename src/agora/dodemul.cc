/**
 * @file dodemul.cc
 * @brief Implmentation file for the DoDemul class.
 */
#include "dodemul.h"

#include "concurrent_queue_wrapper.h"

static constexpr bool kUseSIMDGather = true;

DoDemul::DoDemul(Config* config, int tid, Table<complex_float>& data_buffer,
                 PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& ul_zf_matrices,
                 Table<complex_float>& ue_spec_pilot_buffer,
                 Table<complex_float>& equal_buffer, Stats* stats_manager)
    : Doer(config, tid),
      data_buffer_(data_buffer),
      ul_zf_matrices_(ul_zf_matrices),
      ue_spec_pilot_buffer_(ue_spec_pilot_buffer),
      equal_buffer_(equal_buffer) {
  duration_stat_ = stats_manager->GetDurationStat(DoerType::kDemul, tid);

  data_gather_buffer_ =
      static_cast<complex_float*>(Agora_memory::PaddedAlignedAlloc(
          Agora_memory::Alignment_t::kAlign64,
          kSCsPerCacheline * kMaxAntennas * sizeof(complex_float)));

  // phase offset calibration data
  auto* ue_pilot_ptr =
      reinterpret_cast<arma::cx_float*>(cfg_->UeSpecificPilot()[0]);
  arma::cx_fmat mat_pilot_data(ue_pilot_ptr, cfg_->OfdmDataNum(),
                               cfg_->UeAntNum(), false);
  ue_pilot_data_ = mat_pilot_data.st();

  equal_buffer_tmp_ =
      static_cast<arma::cx_float*>(Agora_memory::PaddedAlignedAlloc(
          Agora_memory::Alignment_t::kAlign64,
          cfg_->UeAntNum() * sizeof(arma::cx_float)));
#if USE_MKL_JIT
  MKL_Complex8 alpha = {1, 0};
  MKL_Complex8 beta = {0, 0};

  mkl_jit_status_t status = mkl_jit_create_cgemm(
      &jitter_, MKL_COL_MAJOR, MKL_NOTRANS, MKL_NOTRANS, cfg_->UeAntNum(), 1,
      cfg_->BsAntNum(), &alpha, cfg_->UeAntNum(), cfg_->BsAntNum(), &beta,
      cfg_->UeAntNum());
  if (MKL_JIT_ERROR == status) {
    std::fprintf(
        stderr,
        "Error: insufficient memory to JIT and store the DGEMM kernel\n");
    throw std::runtime_error(
        "DoDemul: insufficient memory to JIT and store the DGEMM kernel");
  }
  mkl_jit_cgemm_ = mkl_jit_get_cgemm_ptr(jitter_);
#endif
}

DoDemul::~DoDemul() {
  std::free(data_gather_buffer_);

#if USE_MKL_JIT
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
  size_t start_tsc = GetTime::WorkerRdtsc();

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
  // Iterate through cache lines
  for (size_t i = 0; i < max_sc_ite; i += kSCsPerCacheline) {
    size_t start_tsc0 = GetTime::WorkerRdtsc();

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

          assert((reinterpret_cast<size_t>(&dst[j * cfg_->BsAntNum() * 2]) %
                  (kAntNumPerSimd * sizeof(float) * 2)) == 0);
          assert((reinterpret_cast<size_t>(&src[j * cfg_->BsAntNum() * 2]) %
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
          assert((reinterpret_cast<size_t>(&src[j * 2]) %
                  (kAntNumPerSimd * sizeof(float) * 2)) == 0);
          assert((reinterpret_cast<size_t>(&dst[j * cfg_->BsAntNum() * 2]) %
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
    duration_stat_->task_duration_[1] += GetTime::WorkerRdtsc() - start_tsc0;

    // Step 2: For each subcarrier, perform equalization by multiplying the
    // subcarrier's data from each antenna with the subcarrier's precoder
    for (size_t j = 0; j < kSCsPerCacheline; j++) {
      const size_t cur_sc_id = base_sc_id + i + j;

      // Reset previous frame's phase correction buffer
      if (symbol_idx_ul == 0 && cur_sc_id == 0) {
        auto* phase_shift_ptr = reinterpret_cast<arma::cx_float*>(
            ue_spec_pilot_buffer_[(frame_id - 1) % kFrameWnd]);
        arma::cx_fmat mat_phase_shift(phase_shift_ptr, cfg_->UeAntNum(),
                                      cfg_->Frame().NumULSyms(), false);
        mat_phase_shift.fill(0);
      }
      arma::cx_float* equal_ptr = nullptr;
      if (cfg_->IsDataSubcarrier(cur_sc_id) == true) {
        size_t cur_data_sc_id = cfg_->GetOFDMDataIndex(cur_sc_id);
        equal_ptr = (arma::cx_float*)(&equal_buffer_[total_data_symbol_idx_ul]
                                                    [cur_data_sc_id *
                                                     cfg_->UeAntNum()]);
      } else {
        equal_ptr = equal_buffer_tmp_;
      }
      arma::cx_fmat mat_equaled(equal_ptr, cfg_->UeAntNum(), 1, false);

      auto* data_ptr = reinterpret_cast<arma::cx_float*>(
          &data_gather_buffer_[j * cfg_->BsAntNum()]);
      // size_t start_tsc2 = worker_rdtsc();
      auto* ul_zf_ptr = reinterpret_cast<arma::cx_float*>(
          ul_zf_matrices_[frame_slot][cfg_->GetZfScId(cur_sc_id)]);

      size_t start_tsc2 = GetTime::WorkerRdtsc();
#if USE_MKL_JIT
      mkl_jit_cgemm_(jitter_, (MKL_Complex8*)ul_zf_ptr, (MKL_Complex8*)data_ptr,
                     (MKL_Complex8*)equal_ptr);
#else
      arma::cx_fmat mat_data(data_ptr, cfg_->BsAntNum(), 1, false);

      arma::cx_fmat mat_ul_zf(ul_zf_ptr, cfg_->UeAntNum(), cfg_->BsAntNum(),
                              false);
      mat_equaled = mat_ul_zf * mat_data;
#endif

      if (cfg_->IsDataSubcarrier(cur_sc_id) == false) {
        auto* phase_shift_ptr = reinterpret_cast<arma::cx_float*>(
            &ue_spec_pilot_buffer_[frame_id % kFrameWnd]
                                  [symbol_idx_ul * cfg_->UeAntNum()]);
        arma::cx_fmat mat_phase_shift(phase_shift_ptr, cfg_->UeAntNum(), 1,
                                      false);
        arma::cx_fmat shift_sc =
            sign(mat_equaled % conj(ue_pilot_data_.col(cur_sc_id)));
        mat_phase_shift += shift_sc;
      }
      size_t start_tsc3 = GetTime::WorkerRdtsc();
      duration_stat_->task_duration_[2] += start_tsc3 - start_tsc2;
      duration_stat_->task_count_++;
    }
  }

  duration_stat_->task_duration_[0] += GetTime::WorkerRdtsc() - start_tsc;
  return EventData(EventType::kDemul, tag);
}
