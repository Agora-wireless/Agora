/**
 * @file dodemod.cc
 * @brief Implmentation file for the DoDemod class.
 */
#include "dodemod.h"

#include "concurrent_queue_wrapper.h"

static constexpr bool kUseSIMDGather = true;

DoDemod::DoDemod(
    Config* config, int tid, Table<complex_float>& ue_spec_pilot_buffer,
    Table<complex_float>& equal_buffer,
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& demod_buffers,
    PhyStats* in_phy_stats, Stats* stats_manager)
    : Doer(config, tid),
      ue_spec_pilot_buffer_(ue_spec_pilot_buffer),
      equal_buffer_(equal_buffer),
      demod_buffers_(demod_buffers),
      phy_stats_(in_phy_stats) {
  duration_stat_ = stats_manager->GetDurationStat(DoerType::kDemul, tid);

  equaled_buffer_temp_ =
      static_cast<complex_float*>(Agora_memory::PaddedAlignedAlloc(
          Agora_memory::Alignment_t::kAlign64,
          cfg_->DemulBlockSize() * kMaxUEs * sizeof(complex_float)));
  equaled_buffer_temp_transposed_ =
      static_cast<complex_float*>(Agora_memory::PaddedAlignedAlloc(
          Agora_memory::Alignment_t::kAlign64,
          cfg_->DemulBlockSize() * kMaxUEs * sizeof(complex_float)));

  // phase offset calibration data
  auto* ue_pilot_ptr =
      reinterpret_cast<arma::cx_float*>(cfg_->UeSpecificPilot()[0]);
  arma::cx_fmat mat_pilot_data(ue_pilot_ptr, cfg_->OfdmDataNum(),
                               cfg_->UeAntNum(), false);
  ue_pilot_data_ = mat_pilot_data.st();
}

DoDemod::~DoDemod() {
  std::free(equaled_buffer_temp_);
  std::free(equaled_buffer_temp_transposed_);
}

EventData DoDemod::Launch(size_t tag) {
  const size_t frame_id = gen_tag_t(tag).frame_id_;
  const size_t symbol_id = gen_tag_t(tag).symbol_id_;
  const size_t base_sc_id = gen_tag_t(tag).sc_id_;

  const size_t symbol_idx_ul = this->cfg_->Frame().GetULSymbolIdx(symbol_id);
  const size_t total_data_symbol_idx_ul =
      cfg_->GetTotalDataSymbolIdxUl(frame_id, symbol_idx_ul);
  const complex_float* data_buf = equal_buffer_[total_data_symbol_idx_ul];

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
    // Step 2: For each subcarrier, perform equalization by multiplying the
    // subcarrier's data from each antenna with the subcarrier's precoder
    for (size_t j = 0; j < kSCsPerCacheline; j++) {
      const size_t cur_sc_id = base_sc_id + i + j;

      arma::cx_float* equal_ptr = nullptr;
      if (kExportConstellation) {
        equal_ptr =
            (arma::cx_float*)(&equal_buffer_[total_data_symbol_idx_ul]
                                            [cur_sc_id * cfg_->UeNum()]);
      } else {
        equal_ptr =
            (arma::cx_float*)(&equaled_buffer_temp_[(cur_sc_id - base_sc_id) *
                                                    cfg_->UeNum()]);
      }
      arma::cx_fmat mat_equaled(equal_ptr, cfg_->UeNum(), 1, false);

      size_t start_tsc0 = GetTime::WorkerRdtsc();
      // apply previously calc'ed phase shift to data
      auto* pilot_corr_ptr = reinterpret_cast<arma::cx_float*>(
          ue_spec_pilot_buffer_[frame_id % kFrameWnd]);
      arma::cx_fmat pilot_corr_mat(pilot_corr_ptr, cfg_->UeNum(),
                                   cfg_->Frame().NumULSyms(), false);
      arma::fmat theta_mat = arg(pilot_corr_mat.col(symbol_idx_ul));
      arma::fmat cur_theta =
          theta_mat / (cfg_->OfdmDataNum() / cfg_->OfdmPilotSpacing());
      arma::cx_fmat mat_phase_correct =
          arma::zeros<arma::cx_fmat>(size(cur_theta));
      mat_phase_correct.set_real(cos(-cur_theta));
      mat_phase_correct.set_imag(sin(-cur_theta));
      mat_equaled %= mat_phase_correct;

      // Measure EVM from ground truth
      if (symbol_idx_ul == cfg_->Frame().NumULSyms() - 1) {
        phy_stats_->UpdateEvmStats(frame_id, cur_sc_id, mat_equaled);
        if (kPrintPhyStats && cur_sc_id == 0) {
          phy_stats_->PrintEvmStats(frame_id - 1);
        }
      }
      size_t start_tsc2 = GetTime::WorkerRdtsc();
      duration_stat_->task_duration_[1] += start_tsc2 - start_tsc0;
      duration_stat_->task_count_++;
    }
  }

  size_t start_tsc3 = GetTime::WorkerRdtsc();
  __m256i index2 = _mm256_setr_epi32(
      0, 1, cfg_->UeNum() * 2, cfg_->UeNum() * 2 + 1, cfg_->UeNum() * 4,
      cfg_->UeNum() * 4 + 1, cfg_->UeNum() * 6, cfg_->UeNum() * 6 + 1);
  auto* equal_t_ptr = reinterpret_cast<float*>(equaled_buffer_temp_transposed_);
  for (size_t i = 0; i < cfg_->UeNum(); i++) {
    float* equal_ptr = nullptr;
    if (kExportConstellation) {
      equal_ptr = reinterpret_cast<float*>(
          &equal_buffer_[total_data_symbol_idx_ul]
                        [base_sc_id * cfg_->UeNum() + i]);
    } else {
      equal_ptr = reinterpret_cast<float*>(equaled_buffer_temp_ + i);
    }
    size_t k_num_double_in_sim_d256 = sizeof(__m256) / sizeof(double);  // == 4
    for (size_t j = 0; j < max_sc_ite / k_num_double_in_sim_d256; j++) {
      __m256 equal_t_temp = _mm256_i32gather_ps(equal_ptr, index2, 4);
      _mm256_store_ps(equal_t_ptr, equal_t_temp);
      equal_t_ptr += 8;
      equal_ptr += cfg_->UeNum() * k_num_double_in_sim_d256 * 2;
    }
    equal_t_ptr = (float*)(equaled_buffer_temp_transposed_);
    int8_t* demod_ptr = demod_buffers_[frame_slot][symbol_idx_ul][i] +
                        (cfg_->ModOrderBits() * base_sc_id);

    switch (cfg_->ModOrderBits()) {
      case (CommsLib::kQpsk):
        DemodQpskSoftSse(equal_t_ptr, demod_ptr, max_sc_ite);
        break;
      case (CommsLib::kQaM16):
        Demod16qamSoftAvx2(equal_t_ptr, demod_ptr, max_sc_ite);
        break;
      case (CommsLib::kQaM64):
        Demod64qamSoftAvx2(equal_t_ptr, demod_ptr, max_sc_ite);
        break;
      default:
        std::printf("Demodulation: modulation type %s not supported!\n",
                    cfg_->Modulation().c_str());
    }
    // std::printf("In doDemul thread %d: frame: %d, symbol: %d, sc_id: %d \n",
    //     tid, frame_id, symbol_idx_ul, base_sc_id);
    // cout << "Demuled data : \n ";
    // cout << " UE " << i << ": ";
    // for (int k = 0; k < max_sc_ite * cfg->ModOrderBits(); k++)
    //     std::printf("%i ", demul_ptr[k]);
    // cout << endl;
  }

  duration_stat_->task_duration_[2] += GetTime::WorkerRdtsc() - start_tsc3;
  duration_stat_->task_duration_[0] += GetTime::WorkerRdtsc() - start_tsc;
  return EventData(EventType::kDemod, tag);
}
