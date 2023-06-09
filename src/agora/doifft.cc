/**
 * @file doifft.cc
 * @brief Implementation file for the DoIFFT class.
 */
#include "doifft.h"

#include "comms-lib.h"
#include "concurrent_queue_wrapper.h"
#include "datatype_conversion.h"
#include "logger.h"

static constexpr bool kPrintIFFTOutput = false;
static constexpr bool kPrintSocketOutput = false;
static constexpr bool kUseOutOfPlaceIFFT = false;
static constexpr bool kMemcpyBeforeIFFT = true;
static constexpr bool kPrintIfftStats = false;

DoIFFT::DoIFFT(Config* in_config, int in_tid,
               Table<complex_float>& in_dl_ifft_buffer,
               char* in_dl_socket_buffer, Stats* in_stats_manager)
    : Doer(in_config, in_tid),
      dl_ifft_buffer_(in_dl_ifft_buffer),
      dl_socket_buffer_(in_dl_socket_buffer) {
  duration_stat_ = in_stats_manager->GetDurationStat(DoerType::kIFFT, in_tid);
  DftiCreateDescriptor(&mkl_handle_, DFTI_SINGLE, DFTI_COMPLEX, 1,
                       cfg_->OfdmCaNum());
  if (kUseOutOfPlaceIFFT) {
    DftiSetValue(mkl_handle_, DFTI_PLACEMENT, DFTI_NOT_INPLACE);
  }
  DftiCommitDescriptor(mkl_handle_);

  // Aligned for SIMD
  ifft_out_ = static_cast<float*>(
      Agora_memory::PaddedAlignedAlloc(Agora_memory::Alignment_t::kAlign64,
                                       2 * cfg_->OfdmCaNum() * sizeof(float)));
  ifft_shift_tmp_ = static_cast<complex_float*>(
      Agora_memory::PaddedAlignedAlloc(Agora_memory::Alignment_t::kAlign64,
                                       2 * cfg_->OfdmCaNum() * sizeof(float)));
  ifft_scale_factor_ = cfg_->OfdmCaNum();
}

DoIFFT::~DoIFFT() {
  DftiFreeDescriptor(&mkl_handle_);
  std::free(ifft_out_);
  std::free(ifft_shift_tmp_);
}

EventData DoIFFT::Launch(size_t tag) {
  size_t start_tsc = GetTime::WorkerRdtsc();

  const size_t frame_id = gen_tag_t(tag).frame_id_;
  const size_t symbol_id = gen_tag_t(tag).symbol_id_;
  const size_t ant_id = gen_tag_t(tag).ant_id_;

  const size_t symbol_idx_dl = cfg_->Frame().GetDLSymbolIdx(symbol_id);

  if (kDebugPrintInTask) {
    std::printf("In doIFFT thread %d: frame: %zu, symbol: %zu, antenna: %zu\n",
                tid_, frame_id, symbol_id, ant_id);
  }

  const size_t offset =
      (cfg_->GetTotalDataSymbolIdxDl(frame_id, symbol_idx_dl) *
       cfg_->BsAntNum()) +
      ant_id;

  const size_t start_tsc1 = GetTime::WorkerRdtsc();
  duration_stat_->task_duration_[1u] += start_tsc1 - start_tsc;

  auto* ifft_in_ptr = reinterpret_cast<float*>(dl_ifft_buffer_[offset]);
  auto* ifft_out_ptr =
      (kUseOutOfPlaceIFFT || kMemcpyBeforeIFFT) ? ifft_out_ : ifft_in_ptr;

  std::memset(ifft_in_ptr, 0, sizeof(float) * cfg_->OfdmDataStart() * 2);
  std::memset(ifft_in_ptr + (cfg_->OfdmDataStop()) * 2, 0,
              sizeof(float) * cfg_->OfdmDataStart() * 2);
  CommsLib::FFTShift(reinterpret_cast<complex_float*>(ifft_in_ptr),
                     ifft_shift_tmp_, cfg_->OfdmCaNum());
  if (kMemcpyBeforeIFFT) {
    std::memcpy(ifft_out_ptr, ifft_in_ptr,
                sizeof(float) * cfg_->OfdmCaNum() * 2);
    DftiComputeBackward(mkl_handle_, ifft_out_ptr);
  } else {
    if (kUseOutOfPlaceIFFT) {
      // Use out-of-place IFFT here is faster than in place IFFT
      // There is no need to reset non-data subcarriers in ifft input
      // to 0 since their values are not changed after IFFT
      DftiComputeBackward(mkl_handle_, ifft_in_ptr, ifft_out_ptr);
    } else {
      DftiComputeBackward(mkl_handle_, ifft_in_ptr);
    }
  }

  bool clipping = false;
  float max_abs = 0;
  for (size_t i = 0; i < 2 * cfg_->OfdmCaNum(); i++) {
    float sample_val = ifft_out_ptr[i] / ifft_scale_factor_;
    if (sample_val >= 1) {
      clipping = true;
      break;
    }
    if (std::abs(sample_val) > max_abs) {
      max_abs = std::abs(sample_val);
    }
  }
  if (clipping) {
    AGORA_LOG_WARN("Clipping occured in Frame %zu, Symbol %zu, Antenna %zu\n",
                   frame_id, symbol_id, ant_id);
  }
  if (ant_id < cfg_->BfAntNum() && max_abs < 1e-4) {
    AGORA_LOG_WARN("Possibly bad antenna %zu with max sample value %2.2f\n",
                   ant_id, max_abs);
  }
  if (kPrintIfftStats) {
    std::printf("%2.3f\n", max_abs);
  }

  if (kPrintIFFTOutput) {
    std::stringstream ss;
    ss << "IFFT_output" << ant_id << "=[";
    for (size_t i = 0; i < cfg_->OfdmCaNum(); i++) {
      ss << std::fixed << std::setw(5) << std::setprecision(3)
         << dl_ifft_buffer_[offset][i].re << "+1j*"
         << dl_ifft_buffer_[offset][i].im << " ";
    }
    ss << "];" << std::endl;
    std::cout << ss.str();
  }

  const size_t start_tsc2 = GetTime::WorkerRdtsc();
  duration_stat_->task_duration_[2u] += start_tsc2 - start_tsc1;

  auto* pkt = reinterpret_cast<Packet*>(
      &dl_socket_buffer_[offset * cfg_->DlPacketLength()]);
  short* socket_ptr = &pkt->data_[2u * cfg_->OfdmTxZeroPrefix()];

  // IFFT scaled results by OfdmCaNum(), we scale down IFFT results
  // during data type coversion.  * 2 complex float -> float
  SimdConvertFloatToShort(ifft_out_ptr, socket_ptr, cfg_->OfdmCaNum() * 2,
                          cfg_->CpLen() * 2, ifft_scale_factor_);

  duration_stat_->task_duration_[3u] += GetTime::WorkerRdtsc() - start_tsc2;

  if (kPrintSocketOutput) {
    std::stringstream ss;
    ss << "socket_tx_data" << ant_id << "_" << symbol_idx_dl << "=[";
    for (size_t i = 0; i < cfg_->SampsPerSymbol(); i++) {
      ss << socket_ptr[i * 2] << "+1j*" << socket_ptr[i * 2 + 1] << " ";
    }
    ss << "];" << std::endl;
    std::cout << ss.str();
  }

  duration_stat_->task_count_++;
  duration_stat_->task_duration_[0u] += GetTime::WorkerRdtsc() - start_tsc;
  return EventData(EventType::kIFFT, tag);
}
