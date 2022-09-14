/**
 * @file doprecode.cc
 * @brief Implementation file for the DoPrecode class.
 */
#include "doprecode.h"

#include "concurrent_queue_wrapper.h"
#include "modulation.h"

static constexpr bool kUseSpatialLocality = true;

DoPrecode::DoPrecode(
    Config* in_config, int in_tid,
    PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& dl_beam_matrices,
    Table<complex_float>& in_dl_ifft_buffer,
    Table<int8_t>& dl_encoded_or_raw_data /* Encoded if LDPC is enabled */,
    Stats* in_stats_manager)
    : Doer(in_config, in_tid),
      dl_beam_matrices_(dl_beam_matrices),
      dl_ifft_buffer_(in_dl_ifft_buffer),
      dl_raw_data_(dl_encoded_or_raw_data) {
  duration_stat_ =
      in_stats_manager->GetDurationStat(DoerType::kPrecode, in_tid);

  AllocBuffer1d(&modulated_buffer_temp_, kSCsPerCacheline * cfg_->UeAntNum(),
                Agora_memory::Alignment_t::kAlign64, 0);
  AllocBuffer1d(&precoded_buffer_temp_,
                cfg_->DemulBlockSize() * cfg_->BsAntNum(),
                Agora_memory::Alignment_t::kAlign64, 0);

#if defined(USE_MKL_JIT)
  MKL_Complex8 alpha = {1, 0};
  MKL_Complex8 beta = {0, 0};
  // Input: A: BsAntNum() x UeAntNum() , B: UeAntNum() x 1
  // Output: C: BsAntNum() x 1
  // Leading dimensions: A: bs_ant_num(), B: ue_num(), C: bs_ant_num()
  mkl_jit_status_t status = mkl_jit_create_cgemm(
      &jitter_, MKL_COL_MAJOR, MKL_NOTRANS, MKL_NOTRANS, cfg_->BsAntNum(), 1,
      cfg_->UeAntNum(), &alpha, cfg_->BsAntNum(), cfg_->UeAntNum(), &beta,
      cfg_->BsAntNum());

  if (MKL_JIT_ERROR == status) {
    std::fprintf(
        stderr,
        "Error: insufficient memory to JIT and store the DGEMM kernel\n");
    throw std::runtime_error(
        "DoPrecode: insufficient memory to JIT and store the DGEMM kernel");
  }
  my_cgemm_ = mkl_jit_get_cgemm_ptr(jitter_);
#endif
}

DoPrecode::~DoPrecode() {
  FreeBuffer1d(&modulated_buffer_temp_);
  FreeBuffer1d(&precoded_buffer_temp_);

#if defined(USE_MKL_JIT)
  mkl_jit_status_t status = mkl_jit_destroy(jitter_);
  if (MKL_JIT_ERROR == status) {
    std::fprintf(stderr, "!!!!Error: Error while destorying MKL JIT\n");
  }
#endif
}

EventData DoPrecode::Launch(size_t tag) {
  size_t start_tsc = GetTime::WorkerRdtsc();
  const size_t frame_id = gen_tag_t(tag).frame_id_;
  const size_t base_sc_id = gen_tag_t(tag).sc_id_;
  const size_t symbol_id = gen_tag_t(tag).symbol_id_;
  const size_t symbol_idx_dl = cfg_->Frame().GetDLSymbolIdx(symbol_id);
  const size_t total_data_symbol_idx =
      cfg_->GetTotalDataSymbolIdxDl(frame_id, symbol_idx_dl);
  const size_t frame_slot = frame_id % kFrameWnd;

  // Mark pilot subcarriers in this block
  // In downlink pilot symbols, all subcarriers are used as pilots
  // In downlink data symbols, pilot subcarriers are every
  // OfdmPilotSpacing() subcarriers
  // if (symbol_idx_dl < cfg->Frame().ClientDlPilotSymbols()) {
  //     std::memset(pilot_sc_flags, 1, cfg->DemulBlockSize() *
  //     sizeof(size_t));
  // } else {
  //     // Find subcarriers used as pilot in this block
  //     std::memset(pilot_sc_flags, 0, cfg->DemulBlockSize() *
  //     sizeof(size_t)); size_t remainder = base_sc_id %
  //     cfg->OfdmPilotSpacing(); size_t first_pilot_sc
  //         = remainder > 0 ? (cfg->OfdmPilotSpacing() - remainder) : 0;
  //     for (size_t i = first_pilot_sc; i < cfg->DemulBlockSize();
  //          i += cfg->OfdmPilotSpacing())
  //         pilot_sc_flags[i] = 1;
  // }

  if (kDebugPrintInTask) {
    std::printf(
        "In doPrecode thread %d: frame %zu, symbol %zu, subcarrier %zu\n", tid_,
        frame_id, symbol_id, base_sc_id);
  }

  size_t max_sc_ite =
      std::min(cfg_->DemulBlockSize(), cfg_->OfdmDataNum() - base_sc_id);

  if (kUseSpatialLocality) {
    for (size_t i = 0; i < max_sc_ite; i = i + kSCsPerCacheline) {
      size_t start_tsc1 = GetTime::WorkerRdtsc();
      for (size_t user_id = 0; user_id < cfg_->UeAntNum(); user_id++) {
        for (size_t j = 0; j < kSCsPerCacheline; j++) {
          LoadInputData(symbol_idx_dl, total_data_symbol_idx, user_id,
                        base_sc_id + i + j, j);
        }
      }

      size_t start_tsc2 = GetTime::WorkerRdtsc();
      duration_stat_->task_duration_[1] += start_tsc2 - start_tsc1;
      for (size_t j = 0; j < kSCsPerCacheline; j++) {
        PrecodingPerSc(frame_slot, base_sc_id + i + j, i + j);
      }
      duration_stat_->task_count_ =
          duration_stat_->task_count_ + kSCsPerCacheline;
      duration_stat_->task_duration_[2] += GetTime::WorkerRdtsc() - start_tsc2;
    }
  } else {
    for (size_t i = 0; i < max_sc_ite; i++) {
      size_t start_tsc1 = GetTime::WorkerRdtsc();
      int cur_sc_id = base_sc_id + i;
      for (size_t user_id = 0; user_id < cfg_->UeAntNum(); user_id++) {
        LoadInputData(symbol_idx_dl, total_data_symbol_idx, user_id, cur_sc_id,
                      0);
      }
      size_t start_tsc2 = GetTime::WorkerRdtsc();
      duration_stat_->task_duration_[1] += start_tsc2 - start_tsc1;

      PrecodingPerSc(frame_slot, cur_sc_id, i);
      duration_stat_->task_count_++;
      duration_stat_->task_duration_[2] += GetTime::WorkerRdtsc() - start_tsc2;
    }
  }

  size_t start_tsc3 = GetTime::WorkerRdtsc();

  __m256i index = _mm256_setr_epi64x(0, cfg_->BsAntNum(), cfg_->BsAntNum() * 2,
                                     cfg_->BsAntNum() * 3);
  auto* precoded_ptr = reinterpret_cast<float*>(precoded_buffer_temp_);
  for (size_t ant_id = 0; ant_id < cfg_->BsAntNum(); ant_id++) {
    int ifft_buffer_offset = ant_id + cfg_->BsAntNum() * total_data_symbol_idx;
    auto* ifft_ptr = reinterpret_cast<float*>(
        &dl_ifft_buffer_[ifft_buffer_offset]
                        [base_sc_id + cfg_->OfdmDataStart()]);
    for (size_t i = 0; i < cfg_->DemulBlockSize() / 4; i++) {
      float* input_shifted_ptr =
          precoded_ptr + 4 * i * 2 * cfg_->BsAntNum() + ant_id * 2;
      __m256d t_data = _mm256_i64gather_pd(
          reinterpret_cast<double*>(input_shifted_ptr), index, 8);
      _mm256_stream_pd(reinterpret_cast<double*>(ifft_ptr + i * 8), t_data);
    }
  }
  duration_stat_->task_duration_[3] += GetTime::WorkerRdtsc() - start_tsc3;
  duration_stat_->task_duration_[0] += GetTime::WorkerRdtsc() - start_tsc;
  if (kDebugPrintInTask) {
    std::printf(
        "In doPrecode thread %d: finished frame: %zu, symbol: %zu, "
        "subcarrier: %zu\n",
        tid_, frame_id, symbol_id, base_sc_id);
  }
  return EventData(EventType::kPrecode, tag);
}

void DoPrecode::LoadInputData(size_t symbol_idx_dl,
                              size_t total_data_symbol_idx, size_t user_id,
                              size_t sc_id, size_t sc_id_in_block) {
  complex_float* data_ptr =
      modulated_buffer_temp_ + sc_id_in_block * cfg_->UeAntNum();
  if ((symbol_idx_dl < cfg_->Frame().ClientDlPilotSymbols()) ||
      (cfg_->IsDataSubcarrier(sc_id) == false)) {
    data_ptr[user_id] = cfg_->UeSpecificPilot()[user_id][sc_id];
  } else {
    int8_t* raw_data_ptr =
        &dl_raw_data_[total_data_symbol_idx]
                     [cfg_->GetOFDMDataIndex(sc_id) +
                      Roundup<64>(cfg_->GetOFDMDataNum()) * user_id];
    data_ptr[user_id] = ModSingleUint8((uint8_t)(*raw_data_ptr),
                                       cfg_->ModTable(Direction::kDownlink));
  }
}

void DoPrecode::PrecodingPerSc(size_t frame_slot, size_t sc_id,
                               size_t sc_id_in_block) {
  arma::cx_float* precoder_ptr = reinterpret_cast<arma::cx_float*>(
      dl_beam_matrices_[frame_slot][cfg_->GetBeamScId(sc_id)]);
  arma::cx_float* data_ptr = reinterpret_cast<arma::cx_float*>(
      modulated_buffer_temp_ +
      (kUseSpatialLocality
           ? (sc_id_in_block % kSCsPerCacheline * cfg_->UeAntNum())
           : 0));
  arma::cx_float* precoded_ptr = reinterpret_cast<arma::cx_float*>(
      precoded_buffer_temp_ + sc_id_in_block * cfg_->BsAntNum());
#if defined(USE_MKL_JIT)
  my_cgemm_(jitter_, (MKL_Complex8*)precoder_ptr, (MKL_Complex8*)data_ptr,
            (MKL_Complex8*)precoded_ptr);
#else
  arma::cx_fmat mat_precoder(precoder_ptr, cfg_->BsAntNum(), cfg_->UeAntNum(),
                             false);
  arma::cx_fmat mat_data(data_ptr, cfg_->UeAntNum(), 1, false);
  arma::cx_fmat mat_precoded(precoded_ptr, cfg_->BsAntNum(), 1, false);
  mat_precoded = mat_precoder * mat_data;
  // cout << "Precoder: \n" << mat_precoder << endl;
  // cout << "Data: \n" << mat_data << endl;
  // cout << "Precoded data: \n" << mat_precoded << endl;
#endif
}
