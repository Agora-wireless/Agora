/**
 * @file doprecode.cc
 * @brief Implementation file for the DoPrecode class.
 */
#include "doprecode.h"

#include "concurrent_queue_wrapper.h"

static constexpr bool kUseSpatialLocality = true;

static size_t countfunc = 0;

DoPrecode::DoPrecode(
    Config* in_config, int in_tid,
    PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& dl_zf_matrices,
    Table<complex_float>& in_dl_ifft_buffer,
    Table<int8_t>& dl_encoded_or_raw_data /* Encoded if LDPC is enabled */,
    Stats* in_stats_manager,
    std::shared_ptr<CsvLog::MatLogger> dlzf_logger)
    : Doer(in_config, in_tid),
      dl_zf_matrices_(dl_zf_matrices),
      dl_ifft_buffer_(in_dl_ifft_buffer),
      dl_raw_data_(dl_encoded_or_raw_data),
      dlzf_logger_(std::move(dlzf_logger)) {
  duration_stat_ =
      in_stats_manager->GetDurationStat(DoerType::kPrecode, in_tid);

  AllocBuffer1d(&modulated_buffer_temp_, kSCsPerCacheline * cfg_->UeAntNum(),
                Agora_memory::Alignment_t::kAlign64, 0);
  AllocBuffer1d(&precoded_buffer_temp_,
                cfg_->DemulBlockSize() * cfg_->BsAntNum(),
                Agora_memory::Alignment_t::kAlign64, 0);

#if USE_MKL_JIT
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

#if USE_MKL_JIT
  mkl_jit_status_t status = mkl_jit_destroy(jitter_);
  if (MKL_JIT_ERROR == status) {
    std::fprintf(stderr, "!!!!Error: Error while destorying MKL JIT\n");
  }
#endif
  std::cout << "PrecodingPerSc runs " << countfunc << " times per frame\n"
            << "USE_MKL_JIT = " << USE_MKL_JIT << std::endl;
}

EventData DoPrecode::Launch(size_t tag) {
  size_t start_tsc = GetTime::WorkerRdtsc();
  const size_t frame_id = gen_tag_t(tag).frame_id_;
  const size_t base_sc_id = gen_tag_t(tag).sc_id_;
  const size_t symbol_id = gen_tag_t(tag).symbol_id_;
  const size_t symbol_idx_dl = cfg_->Frame().GetDLSymbolIdx(symbol_id);
  const size_t total_data_symbol_idx =
      cfg_->GetTotalDataSymbolIdxDl(frame_id, symbol_idx_dl);
  //const size_t frame_slot = frame_id % kFrameWnd;

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
        if (frame_id == 0) {
          countfunc++;
        }
        PrecodingPerSc(frame_id, symbol_idx_dl, base_sc_id + i + j, i + j);
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

      PrecodingPerSc(frame_id, symbol_idx_dl, cur_sc_id, i);
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

void DoPrecode::PrecodingPerSc(size_t frame_id, size_t symbol_idx_dl,
                               size_t sc_id, size_t sc_id_in_block) {
  const size_t frame_slot = frame_id % kFrameWnd;
  auto* precoder_ptr = reinterpret_cast<arma::cx_float*>(
      dl_zf_matrices_[frame_slot][cfg_->GetZfScId(sc_id)]);
  auto* data_ptr = reinterpret_cast<arma::cx_float*>(
      modulated_buffer_temp_ +
      (kUseSpatialLocality
           ? (sc_id_in_block % kSCsPerCacheline * cfg_->UeAntNum())
           : 0));
  auto* precoded_ptr = reinterpret_cast<arma::cx_float*>(
      precoded_buffer_temp_ + sc_id_in_block * cfg_->BsAntNum());

  arma::cx_fcube cube_precoder(precoder_ptr, cfg_->BsAntNum(), cfg_->UeAntNum(),
                               cfg_->Frame().NumDLSyms(), false);
  arma::cx_fmat mat_data(data_ptr, cfg_->UeAntNum(), 1, false);
  arma::cx_fmat mat_precoded(precoded_ptr, cfg_->BsAntNum(), 1, false);

    /*************************************************************************
    *      Baseline#4: experiment with ASM random bmfm (per-frame-level)
    **************************************************************************/
   // Assuming SU-DM, the mat_dl_zf should be of Nt-by-1; however, Agora also takes REF
    // into account, resulting in an (Nt+1)-by-1 mat_dl_zf;
    // E.g., using one chain at BS, mat_dl_zf is of 9-by-1.

  //arma::fmat thinvec = arma::ones<arma::fmat>(mat_dl_zf.n_rows-1, 1);
  
  
  //const size_t bsradio = cfg_->BsAntNum() - 1;
  //const size_t OFF = 3; // num of OFF antennas in one-chain BS

  //arma::vec indexx = arma::randperm<arma::vec>(bsradio, OFF);
  //arma::vec indexx1 = arma::randperm<arma::vec>(bsradio, OFF); //NEW (for thinvec1 generation)
  //arma::vec indexx2 = arma:randperm<arma::vec>(bsradio, OFF);  //NEW (for thinvec2 generation)

  //arma::fmat thinvec = arma::ones<arma::fmat>(cfg_->BsAntNum(), 1);
  //arma::fmat thinvec1 = arma::ones<arma::fmat>(cfg_->BsAntNum(), 1); //NEW (initialize thinvec as all-one)
  //arma::fmat thinvec2 = arma::ones<arma::fmat>(cfg_->BsAntNum(), 1);   //NEW (initialize thinvec as all-one)
  // PERHAPS NEED to ensure we didn't mess up random seed here, i.e., thinvec, thinvec1, and thinvec2 are all random


  //for (size_t i = 0; i < indexx.n_rows; i++) {
  //  thinvec(indexx(i)) = 0.0f;
    //thinvec1(indexx1(i)) = 0.0f;   //NEW (done generating the 2nd thinvec)
    //thinvec2(indexx2(i)) = 0.0f;   //NEW (done genreating the last thinvec)
  //}

  //==================OLD STUFF TO REMOVE===================================================
  //arma::fmat mat_singleton;   // there might be more efficient way
  //mat_singleton.zeros(1,1);
  //thinvec = arma::join_vert(thinvec, mat_singleton); //append an 0 to thinvec, to match size of mat_dl_zf

  // thinvec.ones(size(thinvec));    //CHECK: ALL-ONE THINVEC

  // if (symbol_idx_dl==0) {
  //   thinvec.ones(size(thinvec));  // check, all-one thinvec
  // }
  //=====================================================================================

  //arma::cx_fmat mat_precoder_thin = mat_precoder % thinvec;  // mat_precoder has been glb normed by dozf already!
  //arma::cx_fmat mat_precoder_thin1 = mat_precoder % thinvec1;   //NEW:mat_precoder_thin1 = "w2" in arma_test
  //arma::cx_fmat mat_precoder_thin2 = mat_precoder % thinvec2;   //NEW:mat_precoder_thin2 =  "w3" in arma_test

  /* THE FOLLOWING EFFECTIVE CHANNEL GAIN CALCULATION STILL REMAIN AS PSEUDO-CODE, BECAUSE
   we need to figure out:
    a). how to access the mat_dl_csi; (maybe make it to be global variable within dozf.cc??)
    b). given mat_dl_csi format, how to correctly extract DL CSI for each SC (recall there are many zeros)
    HENCE, BELOW WE USE h to REPRESENT EACH SC's CHANNEL VECTOR, momentarily.
  */
  //=======================BEGIN PSEUDO-CODE===================================
  // arma::mat heff_1; arma::mat heff_2; arma::mat heff_3;  //NEW:initialize effective channel gains
  // heff_1 = arma::abs(h.t() * mat_precoder_thin);   //NEW: effective channel 1, mismatch remains
  // heff_2 = arma::abs(h.t() * mat_precoder_thin1);  //NEW: effective channel 2, mismatch remains
  // heff_3 = arma::abs(h.t() * mat_precoder_thin2);  //NEW: effective channel 3, mismatch remains
  // MAY need to change "h.t()" to simply h, once size mismatch error returned; actually I tend to believe it should be h...
  //=======================END OF PSUDO-CODE, and continue===================================

  // arma::mat heff_vec; heff_vec.set_size(3, 1); //NEW:hardcoded size of 3, which is number of D slots in total
  // heff_vec(0,0) = arma::conv_to<float>::from(heff_1); //NEW: group above channel gains into a column vector
  // heff_vec(1,0) = arma::conv_to<float>::from(heff_2); //NEW: group into a column vector
  // heff_vec(2,0) = arma::conv_to<float>::from(heff_3); //NEW: group into a column vector
  // mat_precoder_thin *= arma::min(heff_vec)/arma::conv_to<float>::from(heff_1); //NEW: second scaling
  // mat_precoder_thin1 *= arma::min(heff_vec)/arma::conv_to<float>::from(heff_2); //NEW: second scaling
  // mat_precoder_thin2 *= arma::min(heff_vec)/arma::conv_to<float>::from(heff_3); //NEW: second scaling

  const arma::cx_fmat& mat_precoder = cube_precoder.slice(symbol_idx_dl);  // this is where we specify the "page num", the output is still called matprecoder
  if (dlzf_logger_) {
    dlzf_logger_->UpdateMatBuf(frame_id, symbol_idx_dl, sc_id, mat_precoder);
  }

#if USE_MKL_JIT
  my_cgemm_(jitter_, (MKL_Complex8*)mat_precoder.memptr(),
            (MKL_Complex8*)data_ptr, (MKL_Complex8*)precoded_ptr);
            // functions the same as below
#else
  mat_precoded = mat_precoder * mat_data; // SUBF or OFDM-SSC
  // mat_precoded1 = mat_precoder_thin1 * mat_data; // NEW: not quite if we want to do this...
  // mat_precoded2 = mat_precoder_thin2 * mat_data; // NEW: not quite...

  // ANI: the precoded vector is the sum of info-bearing signal (which is mat_precoded), and an artificial
  // noise signal; 
  // psuedo-code:
  // mat_ANI = arma::null(mat) * arma::randn(size of mat_dl_zf)
  // mat_precoded  = mat_precoded +  mat_ANI
#endif
  // std::cout << "Precoder: \n" << mat_precoder << std::endl;
  // std::cout << "Data: \n" << mat_data << std::endl;
  // std::cout << "Precoded data: \n" << mat_precoded << std::endl;
  
  // std::cout << "thinning vector: \n" << thinvec << std::endl;
  // std::cout << "cfg_->BsAntNum():" << cfg_->BsAntNum() << std::endl;
}
