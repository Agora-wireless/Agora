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
  duration_stat_ = stats_manager->GetDurationStat(DoerType::kDemod, tid);

  equaled_buffer_temp_ =
      static_cast<complex_float*>(Agora_memory::PaddedAlignedAlloc(
          Agora_memory::Alignment_t::kAlign64,
          cfg_->DemodBlockSize() * kMaxUEs * sizeof(complex_float)));
  equaled_buffer_temp_transposed_ =
      static_cast<complex_float*>(Agora_memory::PaddedAlignedAlloc(
          Agora_memory::Alignment_t::kAlign64,
          cfg_->DemodBlockSize() * kMaxUEs * sizeof(complex_float)));

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

  const size_t frame_slot = frame_id % kFrameWnd;
  size_t start_tsc = GetTime::WorkerRdtsc();

  if (kDebugPrintInTask == true) {
    std::printf(
        "In doDemod tid %d: frame: %zu, symbol idx: %zu, symbol idx ul: %zu, "
        "subcarrier: %zu, databuffer idx %zu \n",
        tid_, frame_id, symbol_id, symbol_idx_ul, base_sc_id,
        total_data_symbol_idx_ul);
  }

  size_t base_data_sc_id = SIZE_MAX;
  if (cfg_->IsDataSubcarrier(base_sc_id) == true)
    base_data_sc_id = cfg_->GetOFDMDataIndex(base_sc_id);
  else if (base_sc_id + 1 < cfg_->OfdmDataNum())
    base_data_sc_id = cfg_->GetOFDMDataIndex(base_sc_id + 1);
  if (base_data_sc_id != SIZE_MAX) {
    // This block of code runs OfdmDataNum() / DemulBlockSize
    // but only needs to run one time for each OFDM symbol
    auto* pilot_corr_ptr = reinterpret_cast<arma::cx_float*>(
        ue_spec_pilot_buffer_[frame_id % kFrameWnd]);
    arma::cx_fmat pilot_corr_mat(pilot_corr_ptr, cfg_->UeAntNum(),
                                 cfg_->Frame().NumULSyms(), false);
    arma::fmat theta_mat = arg(pilot_corr_mat.col(symbol_idx_ul));
    arma::fmat cur_theta = theta_mat / cfg_->GetOFDMPilotNum();
    arma::cx_fmat mat_phase_correct =
        arma::zeros<arma::cx_fmat>(size(cur_theta));
    mat_phase_correct.set_real(cos(-cur_theta));
    mat_phase_correct.set_imag(sin(-cur_theta));

    size_t max_sc_ite = std::min(cfg_->DemodBlockSize(),
                                 cfg_->GetOFDMDataNum() - base_data_sc_id);
    // assert(max_sc_ite % kSCsPerCacheline == 0);
    // Iterate through cache lines
    for (size_t i = 0; i < max_sc_ite; i += kSCsPerCacheline) {
      // Step 2: For each subcarrier, perform phase correction by multiplying the
      // subcarrier's equalized data with mat_phase_correct
      for (size_t j = 0; j < kSCsPerCacheline; j++) {
        const size_t cur_data_sc_id = base_data_sc_id + i + j;
        size_t start_tsc0 = GetTime::WorkerRdtsc();
        arma::cx_float* equal_ptr = nullptr;
        if (kExportConstellation) {
          equal_ptr = (arma::cx_float*)(&equal_buffer_[total_data_symbol_idx_ul]
                                                      [cur_data_sc_id *
                                                       cfg_->UeAntNum()]);
        } else {
          equal_ptr =
              (arma::cx_float*)(&equaled_buffer_temp_[(cur_data_sc_id -
                                                       base_data_sc_id) *
                                                      cfg_->UeAntNum()]);
        }
        arma::cx_fmat mat_equaled(equal_ptr, cfg_->UeAntNum(), 1, false);

        // apply previously calc'ed phase shift to data
        mat_equaled %= mat_phase_correct;

        // Measure EVM from ground truth
        // Currently only last UL symbol is measured
        if (symbol_idx_ul == cfg_->Frame().NumULSyms() - 1) {
          phy_stats_->UpdateEvmStats(
              frame_id, cfg_->GetOFDMSymbolIndex(cur_data_sc_id), mat_equaled);
          if (kPrintPhyStats && cur_data_sc_id == 0) {
            phy_stats_->PrintEvmStats(frame_id - 1);
          }
        }
        size_t start_tsc2 = GetTime::WorkerRdtsc();
        duration_stat_->task_duration_[1] += start_tsc2 - start_tsc0;
        duration_stat_->task_count_++;
      }
    }

    size_t start_tsc3 = GetTime::WorkerRdtsc();
    __m256i index2 =
        _mm256_setr_epi32(0, 1, cfg_->UeAntNum() * 2, cfg_->UeAntNum() * 2 + 1,
                          cfg_->UeAntNum() * 4, cfg_->UeAntNum() * 4 + 1,
                          cfg_->UeAntNum() * 6, cfg_->UeAntNum() * 6 + 1);
    auto* equal_t_ptr =
        reinterpret_cast<float*>(equaled_buffer_temp_transposed_);
    for (size_t ue_id = 0; ue_id < cfg_->UeAntNum(); ue_id++) {
      float* equal_ptr = nullptr;
      if (kExportConstellation) {
        equal_ptr = reinterpret_cast<float*>(
            &equal_buffer_[total_data_symbol_idx_ul]
                          [base_data_sc_id * cfg_->UeAntNum() + ue_id]);
      } else {
        equal_ptr = reinterpret_cast<float*>(equaled_buffer_temp_ + ue_id);
      }
      size_t k_num_double_in_sim_d256 =
          sizeof(__m256) / sizeof(double);  // == 4
      for (size_t j = 0; j < max_sc_ite / k_num_double_in_sim_d256; j++) {
        __m256 equal_t_temp = _mm256_i32gather_ps(equal_ptr, index2, 4);
        _mm256_store_ps(equal_t_ptr, equal_t_temp);
        equal_t_ptr += 8;
        equal_ptr += cfg_->UeAntNum() * k_num_double_in_sim_d256 * 2;
      }
      equal_t_ptr = (float*)(equaled_buffer_temp_transposed_);
      int8_t* demod_ptr =
          demod_buffers_[frame_slot][symbol_idx_ul][ue_id] +
          (cfg_->ModOrderBits(Direction::kUplink) * base_data_sc_id);

      Demodulate(equal_t_ptr, demod_ptr, symbol_idx_ul,
                 total_data_symbol_idx_ul, ue_id, base_sc_id, max_sc_ite);
      //switch (cfg_->ModOrderBits(Direction::kUplink)) {
      //  case (CommsLib::kQpsk):
      //    kUplinkHardDemod
      //        ? DemodQpskHardLoop(equal_t_ptr,
      //                            reinterpret_cast<uint8_t*>(demod_ptr),
      //                            max_sc_ite)
      //        : DemodQpskSoftSse(equal_t_ptr, demod_ptr, max_sc_ite);
      //    break;
      //  case (CommsLib::kQaM16):
      //    kUplinkHardDemod
      //        ? Demod16qamHardAvx2(equal_t_ptr,
      //                             reinterpret_cast<uint8_t*>(demod_ptr),
      //                             max_sc_ite)
      //        : Demod16qamSoftAvx2(equal_t_ptr, demod_ptr, max_sc_ite);
      //    break;
      //  case (CommsLib::kQaM64):
      //    kUplinkHardDemod
      //        ? Demod64qamHardAvx2(equal_t_ptr,
      //                             reinterpret_cast<uint8_t*>(demod_ptr),
      //                             max_sc_ite)
      //        : Demod64qamSoftAvx2(equal_t_ptr, demod_ptr, max_sc_ite);
      //    break;
      //  default:
      //    std::printf("Demodulation: modulation type %s not supported!\n",
      //                cfg_->Modulation(Direction::kUplink).c_str());
      //}
      //if ((kUplinkHardDemod == true) && (kPrintPhyStats == true) &&
      //    (symbol_idx_ul >= cfg_->Frame().ClientUlPilotSymbols())) {
      //  phy_stats_->UpdateDecodedBits(
      //      ue_id, total_data_symbol_idx_ul,
      //      max_sc_ite * cfg_->ModOrderBits(Direction::kUplink));
      //  // Each block here is max_sc_ite
      //  phy_stats_->IncrementDecodedBlocks(ue_id, total_data_symbol_idx_ul);
      //  size_t block_error(0);
      //  int8_t* tx_bytes =
      //      cfg_->GetModBitsBuf(cfg_->UlModBits(), Direction::kUplink, 0,
      //                          symbol_idx_ul, ue_id, base_data_sc_id);
      //  for (size_t i = 0; i < max_sc_ite; i++) {
      //    uint8_t rx_byte = static_cast<uint8_t>(demod_ptr[i]);
      //    uint8_t tx_byte = static_cast<uint8_t>(tx_bytes[i]);
      //    phy_stats_->UpdateBitErrors(ue_id, total_data_symbol_idx_ul, tx_byte,
      //                                rx_byte);
      //    if (rx_byte != tx_byte) {
      //      block_error++;
      //    }
      //  }
      //  phy_stats_->UpdateBlockErrors(ue_id, total_data_symbol_idx_ul,
      //                                block_error);
      //}

      // std::printf("In doDemod thread %d: frame: %d, symbol: %d, sc_id: %d \n",
      //     tid, frame_id, symbol_idx_ul, base_sc_id);
      // cout << "Demoded data : \n ";
      // cout << " UE " << ue_id << ": ";
      // for (int k = 0; k < max_sc_ite * cfg->ModOrderBits(Direction::kUplink); k++)
      //   std::printf("%i ", demod_ptr[k]);
      // cout << endl;
    }

    duration_stat_->task_duration_[2] += GetTime::WorkerRdtsc() - start_tsc3;
    duration_stat_->task_duration_[0] += GetTime::WorkerRdtsc() - start_tsc;
  }
  return EventData(EventType::kDemod, tag);
}

void DoDemod::Demodulate(float* equal_t_ptr, int8_t* demod_ptr,
                         size_t symbol_idx_ul, size_t total_data_symbol_idx_ul,
                         size_t ue_id, size_t base_sc_id, size_t max_sc_ite) {
  size_t non_aligned_sc = base_sc_id % kSCsPerCacheline;
  size_t aligned_sc = max_sc_ite;
  if (non_aligned_sc > 0) {
    aligned_sc -= non_aligned_sc;
    switch (cfg_->ModOrderBits(Direction::kUplink)) {
      case (CommsLib::kQpsk):
        kUplinkHardDemod
            ? DemodQpskHardLoop(equal_t_ptr,
                                reinterpret_cast<uint8_t*>(demod_ptr),
                                non_aligned_sc)
            : DemodQpskSoftLoop(equal_t_ptr, demod_ptr, non_aligned_sc);
        demod_ptr += 2 * non_aligned_sc;
        break;
      case (CommsLib::kQaM16):
        kUplinkHardDemod
            ? Demod16qamHardLoop(equal_t_ptr,
                                 reinterpret_cast<uint8_t*>(demod_ptr),
                                 non_aligned_sc)
            : Demod16qamSoftLoop(equal_t_ptr, demod_ptr, non_aligned_sc);
        demod_ptr += 4 * non_aligned_sc;
        break;
      case (CommsLib::kQaM64):
        kUplinkHardDemod
            ? Demod64qamHardLoop(equal_t_ptr,
                                 reinterpret_cast<uint8_t*>(demod_ptr),
                                 non_aligned_sc)
            : Demod64qamSoftLoop(equal_t_ptr, demod_ptr, non_aligned_sc);
        demod_ptr += 6 * non_aligned_sc;
        break;
      default:
        std::printf("Demodulation: modulation type %s not supported!\n",
                    cfg_->Modulation(Direction::kUplink).c_str());
    }
    equal_t_ptr += (kSCsPerCacheline - non_aligned_sc) * 2;
  }
  switch (cfg_->ModOrderBits(Direction::kUplink)) {
    case (CommsLib::kQpsk):
      kUplinkHardDemod
          ? DemodQpskHardLoop(equal_t_ptr,
                              reinterpret_cast<uint8_t*>(demod_ptr), aligned_sc)
          : DemodQpskSoftSse(equal_t_ptr, demod_ptr, aligned_sc);
      break;
    case (CommsLib::kQaM16):
      kUplinkHardDemod
          ? Demod16qamHardAvx2(
                equal_t_ptr, reinterpret_cast<uint8_t*>(demod_ptr), aligned_sc)
          : Demod16qamSoftAvx2(equal_t_ptr, demod_ptr, aligned_sc);
      break;
    case (CommsLib::kQaM64):
      kUplinkHardDemod
          ? Demod64qamHardAvx2(
                equal_t_ptr, reinterpret_cast<uint8_t*>(demod_ptr), aligned_sc)
          : Demod64qamSoftAvx2(equal_t_ptr, demod_ptr, aligned_sc);
      break;
    default:
      std::printf("Demodulation: modulation type %s not supported!\n",
                  cfg_->Modulation(Direction::kUplink).c_str());
  }
  if ((kUplinkHardDemod == true) && (kPrintPhyStats == true) &&
      (symbol_idx_ul >= cfg_->Frame().ClientUlPilotSymbols())) {
    phy_stats_->UpdateDecodedBits(
        ue_id, total_data_symbol_idx_ul,
        max_sc_ite * cfg_->ModOrderBits(Direction::kUplink));
    // Each block here is max_sc_ite
    phy_stats_->IncrementDecodedBlocks(ue_id, total_data_symbol_idx_ul);
    size_t block_error(0);
    int8_t* tx_bytes =
        cfg_->GetModBitsBuf(cfg_->UlModBits(), Direction::kUplink, 0,
                            symbol_idx_ul, ue_id, base_sc_id);
    for (size_t i = 0; i < max_sc_ite; i++) {
      uint8_t rx_byte = static_cast<uint8_t>(demod_ptr[i]);
      uint8_t tx_byte = static_cast<uint8_t>(tx_bytes[i]);
      phy_stats_->UpdateBitErrors(ue_id, total_data_symbol_idx_ul, tx_byte,
                                  rx_byte);
      if (rx_byte != tx_byte) {
        block_error++;
      }
    }
    phy_stats_->UpdateBlockErrors(ue_id, total_data_symbol_idx_ul, block_error);
  }
}
