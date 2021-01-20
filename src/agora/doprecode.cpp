#include "doprecode.hpp"

#include "concurrent_queue_wrapper.hpp"

using namespace arma;
static constexpr bool kUseSpatialLocality = true;

DoPrecode::DoPrecode(
    Config* in_config, int in_tid,
    PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& dl_zf_matrices,
    Table<complex_float>& in_dl_ifft_buffer,
    Table<int8_t>& dl_encoded_or_raw_data /* Encoded if LDPC is enabled */,
    Stats* in_stats_manager)
    : Doer(in_config, in_tid),
      dl_zf_matrices_(dl_zf_matrices),
      dl_ifft_buffer_(in_dl_ifft_buffer),
      dl_raw_data_(dl_encoded_or_raw_data) {
  duration_stat_ = in_stats_manager->GetDurationStat(DoerType::kPrecode, in_tid);

  alloc_buffer_1d(&modulated_buffer_temp_, kSCsPerCacheline * cfg_->ue_num(),
                  Agora_memory::Alignment_t::k64Align, 0);
  alloc_buffer_1d(&precoded_buffer_temp_,
                  cfg_->demul_block_size() * cfg_->bs_ant_num(),
                  Agora_memory::Alignment_t::k64Align, 0);

#if USE_MKL_JIT
  MKL_Complex8 alpha = {1, 0};
  MKL_Complex8 beta = {0, 0};
  // Input: A: bs_ant_num() x ue_num() , B: ue_num() x 1
  // Output: C: bs_ant_num() x 1
  // Leading dimensions: A: bs_ant_num(), B: ue_num(), C: bs_ant_num()
  mkl_jit_status_t status = mkl_jit_create_cgemm(
      &jitter_, MKL_COL_MAJOR, MKL_NOTRANS, MKL_NOTRANS, cfg_->bs_ant_num(), 1,
      cfg_->ue_num(), &alpha, cfg_->bs_ant_num(), cfg_->ue_num(), &beta,
      cfg_->bs_ant_num());

  if (MKL_JIT_ERROR == status) {
    std::fprintf(
        stderr,
        "Error: insufficient memory to JIT and store the DGEMM kernel\n");
    std::exit(1);
  }
  my_cgemm_ = mkl_jit_get_cgemm_ptr(jitter_);
#endif
}

DoPrecode::~DoPrecode() {
  free_buffer_1d(&modulated_buffer_temp_);
  free_buffer_1d(&precoded_buffer_temp_);

#if USE_MKL_JIT
  mkl_jit_status_t status = mkl_jit_destroy(jitter_);
  if (MKL_JIT_ERROR == status) {
    std::fprintf(stderr, "!!!!Error: Error while destorying MKL JIT\n");
  }
#endif
}

EventData DoPrecode::launch(size_t tag) {
  size_t start_tsc = worker_rdtsc();
  const size_t frame_id = gen_tag_t(tag).frame_id_;
  const size_t base_sc_id = gen_tag_t(tag).sc_id_;
  const size_t symbol_id = gen_tag_t(tag).symbol_id_;
  const size_t symbol_idx_dl = cfg_->GetDLSymbolIdx(frame_id, symbol_id);
  const size_t total_data_symbol_idx =
      cfg_->GetTotalDataSymbolIdxDl(frame_id, symbol_idx_dl);
  const size_t frame_slot = frame_id % kFrameWnd;

  // Mark pilot subcarriers in this block
  // In downlink pilot symbols, all subcarriers are used as pilots
  // In downlink data symbols, pilot subcarriers are every
  // ofdm_pilot_spacing() subcarriers
  // if (symbol_idx_dl < cfg->frame().client_dl_pilot_symbols()) {
  //     std::memset(pilot_sc_flags, 1, cfg->demul_block_size() *
  //     sizeof(size_t));
  // } else {
  //     // Find subcarriers used as pilot in this block
  //     std::memset(pilot_sc_flags, 0, cfg->demul_block_size() *
  //     sizeof(size_t)); size_t remainder = base_sc_id %
  //     cfg->ofdm_pilot_spacing(); size_t first_pilot_sc
  //         = remainder > 0 ? (cfg->ofdm_pilot_spacing() - remainder) : 0;
  //     for (size_t i = first_pilot_sc; i < cfg->demul_block_size();
  //          i += cfg->ofdm_pilot_spacing())
  //         pilot_sc_flags[i] = 1;
  // }

  if (kDebugPrintInTask) {
    std::printf(
        "In doPrecode thread %d: frame %zu, symbol %zu, subcarrier %zu\n", tid_,
        frame_id, symbol_id, base_sc_id);
  }

  size_t max_sc_ite =
      std::min(cfg_->demul_block_size(), cfg_->ofdm_data_num() - base_sc_id);

  if (kUseSpatialLocality) {
    for (size_t i = 0; i < max_sc_ite; i = i + kSCsPerCacheline) {
      size_t start_tsc1 = worker_rdtsc();
      for (size_t user_id = 0; user_id < cfg_->ue_num(); user_id++)
        for (size_t j = 0; j < kSCsPerCacheline; j++)
          load_input_data(symbol_idx_dl, total_data_symbol_idx, user_id,
                          base_sc_id + i + j, j);

      size_t start_tsc2 = worker_rdtsc();
      duration_stat_->task_duration_[1] += start_tsc2 - start_tsc1;
      for (size_t j = 0; j < kSCsPerCacheline; j++)
        precoding_per_sc(frame_slot, base_sc_id + i + j, i + j);
      duration_stat_->task_count_ = duration_stat_->task_count_ + kSCsPerCacheline;
      duration_stat_->task_duration_[2] += worker_rdtsc() - start_tsc2;
    }
  } else {
    for (size_t i = 0; i < max_sc_ite; i++) {
      size_t start_tsc1 = worker_rdtsc();
      int cur_sc_id = base_sc_id + i;
      for (size_t user_id = 0; user_id < cfg_->ue_num(); user_id++)
        load_input_data(symbol_idx_dl, total_data_symbol_idx, user_id,
                        cur_sc_id, 0);
      size_t start_tsc2 = worker_rdtsc();
      duration_stat_->task_duration_[1] += start_tsc2 - start_tsc1;

      precoding_per_sc(frame_slot, cur_sc_id, i);
      duration_stat_->task_count_++;
      duration_stat_->task_duration_[2] += worker_rdtsc() - start_tsc2;
    }
  }

  size_t start_tsc3 = worker_rdtsc();

  __m256i index = _mm256_setr_epi64x(
      0, cfg_->bs_ant_num(), cfg_->bs_ant_num() * 2, cfg_->bs_ant_num() * 3);
  float* precoded_ptr = (float*)precoded_buffer_temp_;
  for (size_t ant_id = 0; ant_id < cfg_->bs_ant_num(); ant_id++) {
    int ifft_buffer_offset = ant_id + cfg_->bs_ant_num() * total_data_symbol_idx;
    float* ifft_ptr =
        (float*)&dl_ifft_buffer_[ifft_buffer_offset]
                                [base_sc_id + cfg_->ofdm_data_start()];
    for (size_t i = 0; i < cfg_->demul_block_size() / 4; i++) {
      float* input_shifted_ptr =
          precoded_ptr + 4 * i * 2 * cfg_->bs_ant_num() + ant_id * 2;
      __m256d t_data =
          _mm256_i64gather_pd((double*)input_shifted_ptr, index, 8);
      _mm256_stream_pd((double*)(ifft_ptr + i * 8), t_data);
    }
  }
  duration_stat_->task_duration_[3] += worker_rdtsc() - start_tsc3;
  duration_stat_->task_duration_[0] += worker_rdtsc() - start_tsc;
  if (kDebugPrintInTask) {
    std::printf(
        "In doPrecode thread %d: finished frame: %zu, symbol: %zu, "
        "subcarrier: %zu\n",
        tid_, frame_id, symbol_id, base_sc_id);
  }
  return EventData(EventType::kPrecode, tag);
}

void DoPrecode::load_input_data(size_t symbol_idx_dl,
                                size_t total_data_symbol_idx, size_t user_id,
                                size_t sc_id, size_t sc_id_in_block) {
  complex_float* data_ptr =
      modulated_buffer_temp_ + sc_id_in_block * cfg_->ue_num();
  if ((symbol_idx_dl < cfg_->frame().client_dl_pilot_symbols()) ||
      ((sc_id % cfg_->ofdm_pilot_spacing()) == 0)) {
    data_ptr[user_id] = cfg_->ue_specific_pilot()[user_id][sc_id];
  } else {
    int8_t* raw_data_ptr =
        &dl_raw_data_[total_data_symbol_idx]
                    [sc_id + roundup<64>(cfg_->ofdm_data_num()) * user_id];
    data_ptr[user_id] =
        mod_single_uint8((uint8_t)(*raw_data_ptr), cfg_->mod_table());
  }
}

void DoPrecode::precoding_per_sc(size_t frame_slot, size_t sc_id,
                                 size_t sc_id_in_block) {
  auto* precoder_ptr = reinterpret_cast<cx_float*>(
      dl_zf_matrices_[frame_slot][cfg_->GetZfScId(sc_id)]);
  auto* data_ptr = reinterpret_cast<cx_float*>(
      modulated_buffer_temp_ +
      (kUseSpatialLocality ? (sc_id_in_block % kSCsPerCacheline * cfg_->ue_num())
                           : 0));
  auto* precoded_ptr = reinterpret_cast<cx_float*>(
      precoded_buffer_temp_ + sc_id_in_block * cfg_->bs_ant_num());
#if USE_MKL_JIT
  my_cgemm_(jitter_, (MKL_Complex8*)precoder_ptr, (MKL_Complex8*)data_ptr,
           (MKL_Complex8*)precoded_ptr);
#else
  cx_fmat mat_precoder(precoder_ptr, cfg->bs_ant_num(), cfg->ue_num(), false);
  cx_fmat mat_data(data_ptr, cfg->ue_num(), 1, false);
  cx_fmat mat_precoded(precoded_ptr, cfg->bs_ant_num(), 1, false);
  mat_precoded = mat_precoder * mat_data;
  // cout << "Precoder: \n" << mat_precoder << endl;
  // cout << "Data: \n" << mat_data << endl;
  // cout << "Precoded data: \n" << mat_precoded << endl;
#endif
}
