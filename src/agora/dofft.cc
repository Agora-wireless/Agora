/**
 * @file dofft.cc
 * @brief Implementation file for the DoFFT class.
 */
#include "dofft.h"

#include "comms-lib.h"
#include "concurrent_queue_wrapper.h"
#include "datatype_conversion.h"
#include "logger.h"

static constexpr bool kPrintFFTInput = false;
static constexpr bool kPrintInputPilot = false;
static constexpr bool kPrintPilotCorrStats = false;

DoFFT::DoFFT(Config* config, size_t tid, Table<complex_float>& data_buffer,
             PtrGrid<kFrameWnd, kMaxUEs, complex_float>& csi_buffers,
             Table<complex_float>& calib_dl_buffer,
             Table<complex_float>& calib_ul_buffer, PhyStats* in_phy_stats,
             Stats* stats_manager)
    : Doer(config, tid),
      data_buffer_(data_buffer),
      csi_buffers_(csi_buffers),
      calib_dl_buffer_(calib_dl_buffer),
      calib_ul_buffer_(calib_ul_buffer),
      phy_stats_(in_phy_stats) {
  duration_stat_fft_ = stats_manager->GetDurationStat(DoerType::kFFT, tid);
  duration_stat_csi_ = stats_manager->GetDurationStat(DoerType::kCSI, tid);
  DftiCreateDescriptor(&mkl_handle_, DFTI_SINGLE, DFTI_COMPLEX, 1,
                       cfg_->OfdmCaNum());
  DftiCommitDescriptor(mkl_handle_);

  // Aligned for SIMD
  fft_inout_ = static_cast<complex_float*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64,
      cfg_->OfdmCaNum() * sizeof(complex_float)));
  fft_shift_tmp_ = static_cast<complex_float*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64,
      cfg_->OfdmCaNum() * sizeof(complex_float)));
  temp_16bits_iq_ = static_cast<uint16_t*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, 32 * sizeof(uint16_t)));
  rx_samps_tmp_ =
      static_cast<std::complex<float>*>(Agora_memory::PaddedAlignedAlloc(
          Agora_memory::Alignment_t::kAlign64,
          cfg_->SampsPerSymbol() * sizeof(std::complex<float>)));
}

DoFFT::~DoFFT() {
  DftiFreeDescriptor(&mkl_handle_);
  std::free(fft_inout_);
  std::free(fft_shift_tmp_);
  std::free(rx_samps_tmp_);
  std::free(temp_16bits_iq_);
}

// @brief
// @in_vec: input complex data to estimate regression params
// @x0: value of the first x data (assumed consecutive integers)
// @out_vec: output complex data with linearly regressed phase
static inline void CalibRegressionEstimate(const arma::cx_fvec& in_vec,
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
  const size_t start_tsc = GetTime::WorkerRdtsc();
  Packet* pkt = fft_req_tag_t(tag).rx_packet_->RawPacket();
  const size_t frame_id = pkt->frame_id_;
  const size_t frame_slot = frame_id % kFrameWnd;
  const size_t symbol_id = pkt->symbol_id_;
  const size_t ant_id = pkt->ant_id_;
  const size_t radio_id = ant_id / cfg_->NumChannels();
  const size_t cell_id = pkt->cell_id_;
  const SymbolType sym_type = cfg_->GetSymbolType(symbol_id);

  if (cfg_->FftInRru() == true) {
    SimdConvertFloat16ToFloat32(
        reinterpret_cast<float*>(fft_inout_),
        reinterpret_cast<float*>(&pkt->data_[2 * cfg_->OfdmRxZeroPrefixBs()]),
        cfg_->OfdmCaNum() * 2);
  } else {
    if (kUse12BitIQ) {
      SimdConvert12bitIqToFloat(
          (uint8_t*)pkt->data_ + 3 * cfg_->OfdmRxZeroPrefixBs(),
          reinterpret_cast<float*>(fft_inout_), temp_16bits_iq_,
          cfg_->OfdmCaNum() * 3);
    } else {
      size_t sample_offset = cfg_->OfdmRxZeroPrefixBs();
      if (sym_type == SymbolType::kCalDL) {
        sample_offset = cfg_->OfdmRxZeroPrefixCalDl();
      } else if (sym_type == SymbolType::kCalUL) {
        sample_offset = cfg_->OfdmRxZeroPrefixCalUl();
      }
      SimdConvertShortToFloat(&pkt->data_[2 * sample_offset],
                              reinterpret_cast<float*>(fft_inout_),
                              cfg_->OfdmCaNum() * 2);
    }
    if (kDebugPrintInTask) {
      std::printf("In doFFT thread %d: frame: %zu, symbol: %zu, ant: %zu\n",
                  tid_, frame_id, symbol_id, ant_id);
    }

    if ((kPrintPilotCorrStats == true) &&
        ((sym_type == SymbolType::kPilot) || (sym_type == SymbolType::kCalUL) ||
         ((sym_type == SymbolType::kCalDL) &&
          (ant_id == cfg_->RefAnt(cell_id))))) {
      SimdConvertShortToFloat(pkt->data_,
                              reinterpret_cast<float*>(rx_samps_tmp_),
                              2 * cfg_->SampsPerSymbol());
      std::vector<std::complex<float>> samples_vec(
          rx_samps_tmp_, rx_samps_tmp_ + cfg_->SampsPerSymbol());
      std::vector<std::complex<float>> pilot_corr =
          CommsLib::CorrelateAvx(samples_vec, cfg_->PilotCf32());
      std::vector<float> pilot_corr_abs = CommsLib::Abs2Avx(pilot_corr);
      size_t peak_offset =
          std::max_element(pilot_corr_abs.begin(), pilot_corr_abs.end()) -
          pilot_corr_abs.begin();
      float peak = pilot_corr_abs[peak_offset];
      size_t seq_len = cfg_->PilotCf32().size();
      size_t sig_offset = peak_offset < seq_len ? 0 : peak_offset - seq_len;
      printf(
          "In doFFT thread %d: frame: %zu, symbol: %zu, ant: %zu, "
          "sig_offset %zu, peak %2.4f\n",
          tid_, frame_id, symbol_id, ant_id, sig_offset, peak);
    }
    if (kPrintInputPilot) {
      std::stringstream ss;
      ss << "FFT_input_" << symbol_id << "_" << ant_id << "=[";
      for (size_t i = 0; i < cfg_->SampsPerSymbol(); i++) {
        ss << pkt->data_[2 * i] << "+1j*" << pkt->data_[2 * i + 1] << " ";
      }
      ss << "];" << std::endl;
      std::cout << ss.str();
    }
    if (kPrintFFTInput) {
      std::stringstream ss;
      ss << "FFT_input_" << symbol_id << "_" << ant_id << "=[";
      for (size_t i = 0; i < cfg_->OfdmCaNum(); i++) {
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

  size_t start_tsc1 = GetTime::WorkerRdtsc();
  duration_stat->task_duration_.at(1) += start_tsc1 - start_tsc;

  if (!cfg_->FftInRru() == true) {
    DftiComputeForward(
        mkl_handle_,
        reinterpret_cast<float*>(fft_inout_));  // Compute FFT in-place
  }

  //// FFT shift the buffer
  std::memcpy(fft_shift_tmp_, fft_inout_, sizeof(float) * cfg_->OfdmCaNum());
  std::memcpy(fft_inout_, fft_inout_ + cfg_->OfdmCaNum() / 2,
              sizeof(float) * cfg_->OfdmCaNum());
  std::memcpy(fft_inout_ + cfg_->OfdmCaNum() / 2, fft_shift_tmp_,
              sizeof(float) * cfg_->OfdmCaNum());

  size_t start_tsc2 = GetTime::WorkerRdtsc();
  duration_stat->task_duration_.at(2) += start_tsc2 - start_tsc1;

  if (sym_type == SymbolType::kPilot) {
    size_t pilot_symbol_id = cfg_->Frame().GetPilotSymbolIdx(symbol_id);
    if (kCollectPhyStats) {
      phy_stats_->UpdateUlPilotSnr(frame_id, pilot_symbol_id, ant_id,
                                   fft_inout_);
    }
    const size_t ue_id = pilot_symbol_id;
    PartialTranspose(csi_buffers_[frame_slot][ue_id], ant_id,
                     SymbolType::kPilot);
  } else if (sym_type == SymbolType::kUL) {
    PartialTranspose(cfg_->GetDataBuf(data_buffer_, frame_id, symbol_id),
                     ant_id, SymbolType::kUL);
  } else if (sym_type == SymbolType::kCalUL) {
    // Only process uplink for antennas that also do downlink in this frame
    // for consistency with calib downlink processing.
    const size_t cal_index = cfg_->RecipCalUlRxIndex(frame_id, ant_id);
    if (cal_index != SIZE_MAX) {
      AGORA_LOG_TRACE(
          "DoFFT[%d]: (Frame %zu, Symbol %zu, Ant %zu) - Received a CalUl "
          "symbol for current cal index %zu\n",
          tid_, frame_id, symbol_id, ant_id, cal_index);

      complex_float* calib_ul_ptr =
          &calib_ul_buffer_[cal_index][ant_id * cfg_->OfdmDataNum()];

      PartialTranspose(calib_ul_ptr, ant_id, sym_type);
      phy_stats_->UpdateCalibPilotSnr(cal_index, 1, ant_id, fft_inout_);
    }
    RtAssert(radio_id != cfg_->RefRadio(cell_id),
             "Received a Cal Ul symbol for an antenna on the reference radio");
  } else if (sym_type == SymbolType::kCalDL) {
    if (ant_id == cfg_->RefAnt(cell_id)) {
      //Find out what antenna transmitted a pilot on this symbol 'C'
      const size_t pilot_tx_ant = cfg_->RecipCalDlAnt(frame_id, symbol_id);
      const size_t cal_index = cfg_->RecipCalUlRxIndex(frame_id, pilot_tx_ant);

      AGORA_LOG_TRACE(
          "DoFFT[%d]: (Frame %zu, Symbol %zu, Ant %zu) - Received a CalDL "
          "symbol for current cal index %zu\n",
          tid_, frame_id, symbol_id, pilot_tx_ant, cal_index);
      RtAssert(cal_index != SIZE_MAX, "Out of bounds index");

      complex_float* calib_dl_ptr =
          &calib_dl_buffer_[cal_index][pilot_tx_ant * cfg_->OfdmDataNum()];
      PartialTranspose(calib_dl_ptr, pilot_tx_ant, sym_type);
      phy_stats_->UpdateCalibPilotSnr(cal_index, 0, pilot_tx_ant, fft_inout_);
    }
    RtAssert(
        radio_id == cfg_->RefRadio(cell_id),
        "Received a Cal Dl symbol for an antenna not on the reference radio");
  } else {
    std::string error_message =
        "Unknown or unsupported symbol type " +
        std::to_string(static_cast<int>(sym_type)) + std::string(" at frame ") +
        std::to_string(frame_id) + std::string(" symbol ") +
        std::to_string(symbol_id) + std::string(" antenna ") +
        std::to_string(ant_id) + "\n";
    RtAssert(false, error_message);
  }

  duration_stat->task_duration_[3] += GetTime::WorkerRdtsc() - start_tsc2;

  fft_req_tag_t(tag).rx_packet_->Free();
  duration_stat->task_count_++;
  duration_stat->task_duration_[0] += GetTime::WorkerRdtsc() - start_tsc;
  return EventData(EventType::kFFT,
                   gen_tag_t::FrmSym(pkt->frame_id_, pkt->symbol_id_).tag_);
}

void DoFFT::PartialTranspose(complex_float* out_buf, size_t ant_id,
                             SymbolType symbol_type) const {
  // We have OfdmDataNum() % kTransposeBlockSize == 0
  const size_t num_blocks = cfg_->OfdmDataNum() / kTransposeBlockSize;

  for (size_t block_idx = 0; block_idx < num_blocks; block_idx++) {
    const size_t block_base_offset =
        block_idx * (kTransposeBlockSize * cfg_->BsAntNum());
    // We have kTransposeBlockSize % kSCsPerCacheline == 0
    for (size_t sc_j = 0; sc_j < kTransposeBlockSize;
         sc_j += kSCsPerCacheline) {
      const size_t sc_idx = (block_idx * kTransposeBlockSize) + sc_j;
      const complex_float* src = &fft_inout_[sc_idx + cfg_->OfdmDataStart()];

      complex_float* dst = nullptr;
      if ((symbol_type == SymbolType::kCalDL) ||
          (symbol_type == SymbolType::kCalUL)) {
        dst = &out_buf[sc_idx];
      } else {
        dst = kUsePartialTrans
                  ? &out_buf[block_base_offset +
                             (ant_id * kTransposeBlockSize) + sc_j]
                  : &out_buf[(cfg_->OfdmDataNum() * ant_id) + sc_j +
                             block_idx * kTransposeBlockSize];
      }

      // With either of AVX-512 or AVX2, load one cacheline =
      // 16 float values = 8 subcarriers = kSCsPerCacheline

#ifdef __AVX512F__
      // AVX-512.
      __m512 fft_result = _mm512_load_ps(reinterpret_cast<const float*>(src));
      if (symbol_type == SymbolType::kPilot) {
        __m512 pilot_tx = _mm512_set_ps(
            cfg_->PilotsSgn()[sc_idx + 7].im, cfg_->PilotsSgn()[sc_idx + 7].re,
            cfg_->PilotsSgn()[sc_idx + 6].im, cfg_->PilotsSgn()[sc_idx + 6].re,
            cfg_->PilotsSgn()[sc_idx + 5].im, cfg_->PilotsSgn()[sc_idx + 5].re,
            cfg_->PilotsSgn()[sc_idx + 4].im, cfg_->PilotsSgn()[sc_idx + 4].re,
            cfg_->PilotsSgn()[sc_idx + 3].im, cfg_->PilotsSgn()[sc_idx + 3].re,
            cfg_->PilotsSgn()[sc_idx + 2].im, cfg_->PilotsSgn()[sc_idx + 2].re,
            cfg_->PilotsSgn()[sc_idx + 1].im, cfg_->PilotsSgn()[sc_idx + 1].re,
            cfg_->PilotsSgn()[sc_idx].im, cfg_->PilotsSgn()[sc_idx].re);
        fft_result = CommsLib::M512ComplexCf32Mult(fft_result, pilot_tx, true);
      }
      _mm512_stream_ps(reinterpret_cast<float*>(dst), fft_result);
#else
      __m256 fft_result0 = _mm256_load_ps(reinterpret_cast<const float*>(src));
      __m256 fft_result1 =
          _mm256_load_ps(reinterpret_cast<const float*>(src + 4));
      if (symbol_type == SymbolType::kPilot) {
        __m256 pilot_tx0 = _mm256_set_ps(
            cfg_->PilotsSgn()[sc_idx + 3].im, cfg_->PilotsSgn()[sc_idx + 3].re,
            cfg_->PilotsSgn()[sc_idx + 2].im, cfg_->PilotsSgn()[sc_idx + 2].re,
            cfg_->PilotsSgn()[sc_idx + 1].im, cfg_->PilotsSgn()[sc_idx + 1].re,
            cfg_->PilotsSgn()[sc_idx].im, cfg_->PilotsSgn()[sc_idx].re);
        fft_result0 =
            CommsLib::M256ComplexCf32Mult(fft_result0, pilot_tx0, true);

        __m256 pilot_tx1 = _mm256_set_ps(
            cfg_->PilotsSgn()[sc_idx + 7].im, cfg_->PilotsSgn()[sc_idx + 7].re,
            cfg_->PilotsSgn()[sc_idx + 6].im, cfg_->PilotsSgn()[sc_idx + 6].re,
            cfg_->PilotsSgn()[sc_idx + 5].im, cfg_->PilotsSgn()[sc_idx + 5].re,
            cfg_->PilotsSgn()[sc_idx + 4].im, cfg_->PilotsSgn()[sc_idx + 4].re);
        fft_result1 =
            CommsLib::M256ComplexCf32Mult(fft_result1, pilot_tx1, true);
      }
      _mm256_stream_ps(reinterpret_cast<float*>(dst), fft_result0);
      _mm256_stream_ps(reinterpret_cast<float*>(dst + 4), fft_result1);
#endif
    }
  }
}
