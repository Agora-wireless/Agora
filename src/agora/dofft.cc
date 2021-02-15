#include "dofft.h"

#include "concurrent_queue_wrapper.h"
#include "datatype_conversion.h"

using namespace arma;

static constexpr bool kPrintFFTInput = false;
static constexpr bool kPrintIFFTOutput = false;
static constexpr bool kPrintSocketOutput = false;
static constexpr bool kUseOutOfPlaceIFFT = false;
static constexpr bool kMemcpyBeforeIFFT = true;
static constexpr bool kPrintPilotCorrStats = false;

DoFFT::DoFFT(Config* config, int tid, Table<char>& socket_buffer,
             Table<int>& socket_buffer_status,
             Table<complex_float>& data_buffer,
             PtrGrid<kFrameWnd, kMaxUEs, complex_float>& csi_buffers,
             Table<complex_float>& calib_dl_buffer,
             Table<complex_float>& calib_ul_buffer, PhyStats* in_phy_stats,
             Stats* stats_manager)
    : Doer(config, tid),
      socket_buffer_(socket_buffer),
      socket_buffer_status_(socket_buffer_status),
      data_buffer_(data_buffer),
      csi_buffers_(csi_buffers),
      calib_dl_buffer_(calib_dl_buffer),
      calib_ul_buffer_(calib_ul_buffer),
      phy_stats_(in_phy_stats) {
  duration_stat_fft_ = stats_manager->GetDurationStat(DoerType::kFFT, tid);
  duration_stat_csi_ = stats_manager->GetDurationStat(DoerType::kCSI, tid);
  DftiCreateDescriptor(&mkl_handle_, DFTI_SINGLE, DFTI_COMPLEX, 1,
                       cfg_->ofdm_ca_num_);
  DftiCommitDescriptor(mkl_handle_);

  // Aligned for SIMD
  fft_inout_ = static_cast<complex_float*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kK64Align,
      cfg_->ofdm_ca_num_ * sizeof(complex_float)));
  temp_16bits_iq_ = static_cast<uint16_t*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kK64Align, 32 * sizeof(uint16_t)));
  rx_samps_tmp_ =
      static_cast<std::complex<float>*>(Agora_memory::PaddedAlignedAlloc(
          Agora_memory::Alignment_t::kK64Align,
          cfg_->samps_per_symbol_ * sizeof(std::complex<float>)));
}

DoFFT::~DoFFT() {
  DftiFreeDescriptor(&mkl_handle_);
  std::free(fft_inout_);
  std::free(rx_samps_tmp_);
  calib_ul_buffer_.Free();
  calib_dl_buffer_.Free();
}

// @brief
// @in_vec: input complex data to estimate regression params
// @x0: value of the first x data (assumed consecutive integers)
// @out_vec: output complex data with linearly regressed phase
static inline void CalibRegressionEstimate(arma::cx_fvec in_vec,
                                           arma::cx_fvec& out_vec, size_t x0) {
  size_t in_len = in_vec.size();
  size_t out_len = out_vec.size();
  std::vector<float> scs(out_len, 0);
  for (size_t i = 0; i < out_len; i++) {
    scs[i] = i;
  }
  arma::fvec x_vec((float*)scs.data() + x0, in_len, false);
  arma::fvec in_phase = arg(in_vec);
  arma::fvec in_mag = abs(in_vec);

  // finding regression parameters from the input vector
  // https://www.cse.wustl.edu/~jain/iucee/ftp/k_14slr.pdf
  arma::fvec xy = in_phase % x_vec;
  float xbar = arma::mean(x_vec);
  float ybar = arma::mean(in_phase);
  float coeff = (arma::sum(xy) - in_len * xbar * ybar) /
                (arma::sum(arma::square(x_vec)) - in_len * xbar * xbar);
  float intercept = ybar - coeff * xbar;

  // extrapolating phase for the target subcarrier using regression
  arma::fvec x_vec_all((float*)scs.data(), out_len, false);
  arma::fvec tar_angle = coeff * x_vec_all + intercept;
  out_vec.set_real(arma::cos(tar_angle));
  out_vec.set_imag(arma::sin(tar_angle));
  out_vec *= arma::mean(in_mag);
}

EventData DoFFT::Launch(size_t tag) {
  size_t socket_thread_id = fft_req_tag_t(tag).tid_;
  size_t buf_offset = fft_req_tag_t(tag).offset_;
  size_t start_tsc = WorkerRdtsc();
  auto* pkt = (Packet*)(socket_buffer_[socket_thread_id] +
                        buf_offset * cfg_->packet_length_);
  size_t frame_id = pkt->frame_id_;
  size_t frame_slot = frame_id % kFrameWnd;
  size_t symbol_id = pkt->symbol_id_;
  size_t ant_id = pkt->ant_id_;
  SymbolType sym_type = cfg_->GetSymbolType(frame_id, symbol_id);

  if (cfg_->fft_in_rru_) {
    SimdConvertFloat16ToFloat32(
        reinterpret_cast<float*>(fft_inout_),
        reinterpret_cast<float*>(
            &pkt->data_[2 * cfg_->ofdm_rx_zero_prefix_bs_]),
        cfg_->ofdm_ca_num_ * 2);
  } else {
    if (kUse12BitIQ) {
      SimdConvert12bitIqToFloat(
          (uint8_t*)pkt->data_ + 3 * cfg_->ofdm_rx_zero_prefix_bs_,
          reinterpret_cast<float*>(fft_inout_), temp_16bits_iq_,
          cfg_->ofdm_ca_num_ * 3);
    } else {
      size_t sample_offset = cfg_->ofdm_rx_zero_prefix_bs_;
      if (sym_type == SymbolType::kCalDL) {
        sample_offset = cfg_->ofdm_rx_zero_prefix_cal_dl_;
      } else if (sym_type == SymbolType::kCalUL) {
        sample_offset = cfg_->ofdm_rx_zero_prefix_cal_ul_;
      }
      SimdConvertShortToFloat(&pkt->data_[2 * sample_offset],
                              reinterpret_cast<float*>(fft_inout_),
                              cfg_->ofdm_ca_num_ * 2);
    }
    if (kDebugPrintInTask) {
      std::printf("In doFFT thread %d: frame: %zu, symbol: %zu, ant: %zu\n",
                  tid_, frame_id, symbol_id, ant_id);
    }
    if (kPrintPilotCorrStats && sym_type == SymbolType::kPilot) {
      SimdConvertShortToFloat(pkt->data_,
                              reinterpret_cast<float*>(rx_samps_tmp_),
                              2 * cfg_->samps_per_symbol_);
      std::vector<std::complex<float>> samples_vec(
          rx_samps_tmp_, rx_samps_tmp_ + cfg_->samps_per_symbol_);
      std::vector<std::complex<float>> pilot_corr =
          CommsLib::CorrelateAvx(samples_vec, cfg_->pilot_cf32_);
      std::vector<float> pilot_corr_abs = CommsLib::Abs2Avx(pilot_corr);
      size_t peak_offset =
          std::max_element(pilot_corr_abs.begin(), pilot_corr_abs.end()) -
          pilot_corr_abs.begin();
      float peak = pilot_corr_abs[peak_offset];
      size_t seq_len = cfg_->pilot_cf32_.size();
      size_t sig_offset = peak_offset < seq_len ? 0 : peak_offset - seq_len;
      printf(
          "In doFFT thread %d: frame: %zu, symbol: %zu, ant: %zu, "
          "sig_offset %zu, peak %2.4f\n",
          tid_, frame_id, symbol_id, ant_id, sig_offset, peak);
    }
    if (kPrintFFTInput) {
      std::stringstream ss;
      ss << "FFT_input" << ant_id << "=[";
      for (size_t i = 0; i < cfg_->ofdm_ca_num_; i++) {
        ss << std::fixed << std::setw(5) << std::setprecision(3)
           << fft_inout_[i].re << "+1j*" << fft_inout_[i].im << " ";
      }
      ss << "];" << std::endl;
      std::cout << ss.str();
    }
  }

  DurationStat dummy_duration_stat;  // TODO: timing for calibration symbols
  DurationStat* duration_stat = nullptr;
  if (sym_type == SymbolType::kUL) {
    duration_stat = duration_stat_fft_;
  } else if (sym_type == SymbolType::kPilot) {
    duration_stat = duration_stat_csi_;
  } else {
    duration_stat = &dummy_duration_stat;  // For calibration symbols
  }

  size_t start_tsc1 = WorkerRdtsc();
  duration_stat->task_duration_[1] += start_tsc1 - start_tsc;

  if (!cfg_->fft_in_rru_) {
    DftiComputeForward(
        mkl_handle_,
        reinterpret_cast<float*>(fft_inout_));  // Compute FFT in-place
  }

  size_t start_tsc2 = WorkerRdtsc();
  duration_stat->task_duration_[2] += start_tsc2 - start_tsc1;

  if (sym_type == SymbolType::kPilot) {
    if (kCollectPhyStats) {
      phy_stats_->UpdatePilotSnr(
          frame_id, cfg_->GetPilotSymbolIdx(frame_id, symbol_id), fft_inout_);
    }
    const size_t ue_id = cfg_->GetPilotSymbolIdx(frame_id, symbol_id);
    PartialTranspose(csi_buffers_[frame_slot][ue_id], ant_id,
                     SymbolType::kPilot);
  } else if (sym_type == SymbolType::kUL) {
    PartialTranspose(cfg_->GetDataBuf(data_buffer_, frame_id, symbol_id),
                     ant_id, SymbolType::kUL);
  } else if (sym_type == SymbolType::kCalUL and ant_id != cfg_->ref_ant_) {
    // Only process uplink for antennas that also do downlink in this frame
    // for consistency with calib downlink processing.
    if (frame_id >= TX_FRAME_DELTA &&
        ant_id / cfg_->ant_per_group_ ==
            (frame_id - TX_FRAME_DELTA) % cfg_->ant_group_num_) {
      size_t frame_grp_id = (frame_id - TX_FRAME_DELTA) / cfg_->ant_group_num_;
      size_t frame_grp_slot = frame_grp_id % kFrameWnd;
      PartialTranspose(
          &calib_ul_buffer_[frame_grp_slot][ant_id * cfg_->ofdm_data_num_],
          ant_id, sym_type);
    }
  } else if (sym_type == SymbolType::kCalDL and ant_id == cfg_->ref_ant_) {
    if (frame_id >= TX_FRAME_DELTA) {
      size_t frame_grp_id = (frame_id - TX_FRAME_DELTA) / cfg_->ant_group_num_;
      size_t frame_grp_slot = frame_grp_id % kFrameWnd;
      size_t cur_ant = frame_id - (frame_grp_id * cfg_->ant_group_num_);
      complex_float* calib_dl_ptr =
          &calib_dl_buffer_[frame_grp_slot][cur_ant * cfg_->ofdm_data_num_];
      PartialTranspose(calib_dl_ptr, ant_id, sym_type);
    }
  } else {
    RtAssert(false, "Unknown or unsupported symbol type");
  }

  duration_stat->task_duration_[3] += WorkerRdtsc() - start_tsc2;
  socket_buffer_status_[socket_thread_id][buf_offset] = 0;  // Reset sock buf
  duration_stat->task_count_++;
  duration_stat->task_duration_[0] += WorkerRdtsc() - start_tsc;
  return EventData(EventType::kFFT,
                   gen_tag_t::FrmSym(pkt->frame_id_, pkt->symbol_id_).tag_);
}

void DoFFT::PartialTranspose(complex_float* out_buf, size_t ant_id,
                             SymbolType symbol_type) const {
  // We have OFDM_DATA_NUM % kTransposeBlockSize == 0
  const size_t num_blocks = cfg_->ofdm_data_num_ / kTransposeBlockSize;

  for (size_t block_idx = 0; block_idx < num_blocks; block_idx++) {
    const size_t block_base_offset =
        block_idx * (kTransposeBlockSize * cfg_->bs_ant_num_);
    // We have kTransposeBlockSize % kSCsPerCacheline == 0
    for (size_t sc_j = 0; sc_j < kTransposeBlockSize;
         sc_j += kSCsPerCacheline) {
      const size_t sc_idx = (block_idx * kTransposeBlockSize) + sc_j;
      const complex_float* src = &fft_inout_[sc_idx + cfg_->ofdm_data_start_];

      complex_float* dst = nullptr;
      if (symbol_type == SymbolType::kCalDL ||
          symbol_type == SymbolType::kCalUL) {
        dst = &out_buf[sc_idx];
      } else {
        dst = kUsePartialTrans
                  ? &out_buf[block_base_offset +
                             (ant_id * kTransposeBlockSize) + sc_j]
                  : &out_buf[(cfg_->ofdm_data_num_ * ant_id) + sc_j +
                             block_idx * kTransposeBlockSize];
      }

      // With either of AVX-512 or AVX2, load one cacheline =
      // 16 float values = 8 subcarriers = kSCsPerCacheline

#ifdef __AVX512F__
            // AVX-512.
            __m512 fft_result
                = _mm512_load_ps(reinterpret_cast<const float*>(src));
            if (symbol_type == SymbolType::kPilot) {
                __m512 pilot_tx = _mm512_set_ps(cfg_->pilots_sgn_[sc_idx + 7].im,
                    cfg_->pilots_sgn_[sc_idx + 7].re,
                    cfg_->pilots_sgn_[sc_idx + 6].im,
                    cfg_->pilots_sgn_[sc_idx + 6].re,
                    cfg_->pilots_sgn_[sc_idx + 5].im,
                    cfg_->pilots_sgn_[sc_idx + 5].re,
                    cfg_->pilots_sgn_[sc_idx + 4].im,
                    cfg_->pilots_sgn_[sc_idx + 4].re,
                    cfg_->pilots_sgn_[sc_idx + 3].im,
                    cfg_->pilots_sgn_[sc_idx + 3].re,
                    cfg_->pilots_sgn_[sc_idx + 2].im,
                    cfg_->pilots_sgn_[sc_idx + 2].re,
                    cfg_->pilots_sgn_[sc_idx + 1].im,
                    cfg_->pilots_sgn_[sc_idx + 1].re,
                    cfg_->pilots_sgn_[sc_idx].im, cfg_->pilots_sgn_[sc_idx].re);
                fft_result = CommsLib::M512ComplexCf32Mult(
                    fft_result, pilot_tx, true);
            }
            _mm512_stream_ps(reinterpret_cast<float*>(dst), fft_result);
#else
      __m256 fft_result0 = _mm256_load_ps(reinterpret_cast<const float*>(src));
      __m256 fft_result1 =
          _mm256_load_ps(reinterpret_cast<const float*>(src + 4));
      if (symbol_type == SymbolType::kPilot) {
        __m256 pilot_tx0 = _mm256_set_ps(
            cfg_->pilots_sgn_[sc_idx + 3].im, cfg_->pilots_sgn_[sc_idx + 3].re,
            cfg_->pilots_sgn_[sc_idx + 2].im, cfg_->pilots_sgn_[sc_idx + 2].re,
            cfg_->pilots_sgn_[sc_idx + 1].im, cfg_->pilots_sgn_[sc_idx + 1].re,
            cfg_->pilots_sgn_[sc_idx].im, cfg_->pilots_sgn_[sc_idx].re);
        fft_result0 =
            CommsLib::M256ComplexCf32Mult(fft_result0, pilot_tx0, true);

        __m256 pilot_tx1 = _mm256_set_ps(
            cfg_->pilots_sgn_[sc_idx + 7].im, cfg_->pilots_sgn_[sc_idx + 7].re,
            cfg_->pilots_sgn_[sc_idx + 6].im, cfg_->pilots_sgn_[sc_idx + 6].re,
            cfg_->pilots_sgn_[sc_idx + 5].im, cfg_->pilots_sgn_[sc_idx + 5].re,
            cfg_->pilots_sgn_[sc_idx + 4].im, cfg_->pilots_sgn_[sc_idx + 4].re);
        fft_result1 =
            CommsLib::M256ComplexCf32Mult(fft_result1, pilot_tx1, true);
      }
      _mm256_stream_ps(reinterpret_cast<float*>(dst), fft_result0);
      _mm256_stream_ps(reinterpret_cast<float*>(dst + 4), fft_result1);
#endif
    }
  }
}

DoIFFT::DoIFFT(Config* in_config, int in_tid,
               Table<complex_float>& in_dl_ifft_buffer,
               char* in_dl_socket_buffer, Stats* in_stats_manager)
    : Doer(in_config, in_tid),
      dl_ifft_buffer_(in_dl_ifft_buffer),
      dl_socket_buffer_(in_dl_socket_buffer) {
  duration_stat_ = in_stats_manager->GetDurationStat(DoerType::kIFFT, in_tid);
  DftiCreateDescriptor(&mkl_handle_, DFTI_SINGLE, DFTI_COMPLEX, 1,
                       cfg_->ofdm_ca_num_);
  if (kUseOutOfPlaceIFFT) {
    DftiSetValue(mkl_handle_, DFTI_PLACEMENT, DFTI_NOT_INPLACE);
  }
  DftiCommitDescriptor(mkl_handle_);

  // Aligned for SIMD
  ifft_out_ = static_cast<float*>(
      Agora_memory::PaddedAlignedAlloc(Agora_memory::Alignment_t::kK64Align,
                                       2 * cfg_->ofdm_ca_num_ * sizeof(float)));
  ifft_scale_factor_ = cfg_->ofdm_ca_num_ / std::sqrt(cfg_->bf_ant_num_ * 1.f);
}

DoIFFT::~DoIFFT() { DftiFreeDescriptor(&mkl_handle_); }

EventData DoIFFT::Launch(size_t tag) {
  size_t start_tsc = WorkerRdtsc();
  size_t ant_id = gen_tag_t(tag).ant_id_;
  size_t frame_id = gen_tag_t(tag).frame_id_;
  size_t symbol_id = gen_tag_t(tag).symbol_id_;
  size_t symbol_idx_dl = cfg_->GetDlSymbolIdx(frame_id, symbol_id);

  if (kDebugPrintInTask) {
    std::printf("In doIFFT thread %d: frame: %zu, symbol: %zu, antenna: %zu\n",
                tid_, frame_id, symbol_id, ant_id);
  }

  size_t offset = (cfg_->GetTotalDataSymbolIdxDl(frame_id, symbol_idx_dl) *
                   cfg_->bs_ant_num_) +
                  ant_id;

  size_t start_tsc1 = WorkerRdtsc();
  duration_stat_->task_duration_[1] += start_tsc1 - start_tsc;

  auto* ifft_in_ptr = reinterpret_cast<float*>(dl_ifft_buffer_[offset]);
  auto* ifft_out_ptr =
      (kUseOutOfPlaceIFFT || kMemcpyBeforeIFFT) ? ifft_out_ : ifft_in_ptr;

  if (kMemcpyBeforeIFFT) {
    std::memset(ifft_out_ptr, 0, sizeof(float) * cfg_->ofdm_data_start_ * 2);
    std::memset(ifft_out_ptr + (cfg_->ofdm_data_stop_) * 2, 0,
                sizeof(float) * cfg_->ofdm_data_start_ * 2);
    std::memcpy(ifft_out_ptr + (cfg_->ofdm_data_start_) * 2,
                ifft_in_ptr + (cfg_->ofdm_data_start_) * 2,
                sizeof(float) * cfg_->ofdm_data_num_ * 2);
    DftiComputeBackward(mkl_handle_, ifft_out_ptr);
  } else {
    if (kUseOutOfPlaceIFFT) {
      // Use out-of-place IFFT here is faster than in place IFFT
      // There is no need to reset non-data subcarriers in ifft input
      // to 0 since their values are not changed after IFFT
      DftiComputeBackward(mkl_handle_, ifft_in_ptr, ifft_out_ptr);
    } else {
      std::memset(ifft_in_ptr, 0, sizeof(float) * cfg_->ofdm_data_start_ * 2);
      std::memset(ifft_in_ptr + (cfg_->ofdm_data_stop_) * 2, 0,
                  sizeof(float) * cfg_->ofdm_data_start_ * 2);
      DftiComputeBackward(mkl_handle_, ifft_in_ptr);
    }
  }

  if (kPrintIFFTOutput) {
    std::stringstream ss;
    ss << "IFFT_output" << ant_id << "=[";
    for (size_t i = 0; i < cfg_->ofdm_ca_num_; i++) {
      ss << std::fixed << std::setw(5) << std::setprecision(3)
         << dl_ifft_buffer_[offset][i].re << "+1j*"
         << dl_ifft_buffer_[offset][i].im << " ";
    }
    ss << "];" << std::endl;
    std::cout << ss.str();
  }

  size_t start_tsc2 = WorkerRdtsc();
  duration_stat_->task_duration_[2] += start_tsc2 - start_tsc1;

  struct Packet* pkt =
      (struct Packet*)&dl_socket_buffer_[offset * cfg_->dl_packet_length_];
  short* socket_ptr = &pkt->data_[2 * cfg_->ofdm_tx_zero_prefix_];

  // IFFT scaled results by OFDM_CA_NUM, we scale down IFFT results
  // during data type coversion
  SimdConvertFloatToShort(ifft_out_ptr, socket_ptr, cfg_->ofdm_ca_num_,
                          cfg_->cp_len_, ifft_scale_factor_);

  duration_stat_->task_duration_[3] += WorkerRdtsc() - start_tsc2;

  if (kPrintSocketOutput) {
    std::stringstream ss;
    ss << "socket_tx_data" << ant_id << "=[";
    for (size_t i = 0; i < cfg_->samps_per_symbol_; i++) {
      ss << socket_ptr[i * 2] << "+1j*" << socket_ptr[i * 2 + 1] << " ";
    }
    ss << "];" << std::endl;
    std::cout << ss.str();
  }

  duration_stat_->task_count_++;
  duration_stat_->task_duration_[0] += WorkerRdtsc() - start_tsc;
  return EventData(EventType::kIFFT, tag);
}
