// Copyright (c) 2018-2020, Rice University
// RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

/**
 * @file comms-lib.cpp
 * @brief Generate training sequence for pilots and preambles.
 *
 * Supports:
 *    STS - 802.11 Short training sequence. Generates one symbol, 16 complex
 *    I/Q samples.
 *    LTS - 802.11 Long training sequence. Generates 2.5 symbols, cp length
 *    of 32 samples, for a total of 160 complex I/Q samples.
 *    LTE Zadoff Chu - Generates the 25th root length-63 Zadoff-Chu sequence.
 *    Total of 63-long complex IQ samples.
 *    Gold IFFT - Total of 128-long complex IQ samples including a 32-sample
 *    cyclic prefix
 *    Hadamard - Real valued sequence. Possible lenghts:
 *    {2, 4, 8, 16, 32, 64}
 * @author Oscar Bejarano: obejarano@rice.edu
 *         Rahman Doost-Mohamamdy: doost@rice.edu
 */

#include "comms-lib.h"

#include <utility>

#include "comms-constants.inc"
#include "datatype_conversion.h"
#include "logger.h"
#include "utils.h"

size_t CommsLib::FindPilotSeq(const std::vector<std::complex<float>>& iq,
                              const std::vector<std::complex<float>>& pilot,
                              size_t seq_len) {
  // Re-arrange into complex vector, flip, and compute conjugate
  std::vector<std::complex<float>> pilot_conj;
  for (size_t i = 0; i < seq_len; i++) {
    // conjugate
    pilot_conj.push_back(std::conj(pilot[seq_len - i - 1]));
  }

  // Equivalent to numpy's sign function
  auto iq_sign = CommsLib::Csign(iq);

  // Convolution
  auto pilot_corr = CommsLib::Convolve(iq_sign, pilot_conj);

  // Find all peaks
  auto best_peak = std::max_element(pilot_corr.begin(), pilot_corr.end()) -
                   pilot_corr.begin();
  return best_peak;
}

int CommsLib::FindLts(const std::vector<std::complex<double>>& iq,
                      int seq_len) {
  /*
   * Find 802.11-based LTS (Long Training Sequence)
   * Input:
   *     iq        - IQ complex samples (vector)
   *     seq_len    - Length of sequence
   * Output:
   *     best_peak - LTS peak index (correlation peak)
   */

  float lts_thresh = 0.8;
  int best_peak;

  // Original LTS sequence
  auto lts_seq = CommsLib::GetSequence(seq_len, kLtsSeq);

  // Re-arrange into complex vector, flip, and compute conjugate
  std::vector<std::complex<double>> lts_sym;
  std::vector<std::complex<double>> lts_sym_conj;
  for (int i = 0; i < 64; i++) {
    // lts_seq is a 2x160 matrix (real/imag by seq_len=160 elements)
    // grab one symbol and flip around
    lts_sym.emplace_back(lts_seq.at(0).at(seq_len - 1 - i),
                         lts_seq.at(1).at(seq_len - 1 - i));
    // conjugate
    lts_sym_conj.push_back(std::conj(lts_sym.at(i)));
  }

  // Equivalent to numpy's sign function
  auto iq_sign = CommsLib::Csign(iq);

  // Convolution
  auto lts_corr = CommsLib::Convolve(iq_sign, lts_sym_conj);
  auto lts_peak = *std::max_element(lts_corr.begin(), lts_corr.end());

  // Find all peaks
  std::vector<int> peaks;
  for (size_t i = 0; i < lts_corr.size(); i++) {
    if (std::abs(lts_corr[i]) > (lts_thresh * lts_peak)) {
      // Index of valid peaks
      peaks.push_back(i);
    }
  }

  std::vector<std::vector<int>> x_vec(peaks.size());
  std::vector<std::vector<int>> y_vec(peaks.size());
  CommsLib::Meshgrid(peaks, peaks, x_vec, y_vec);

  // Find peaks that are 64 samples apart
  std::vector<int> valid_peaks;
  for (size_t i = 0; i < x_vec.size(); i++) {
    for (size_t j = 0; j < x_vec[0].size(); j++) {
      int idx_diff = y_vec[i][j] - x_vec[i][j];
      if (idx_diff == static_cast<int>(lts_sym.size())) {
        valid_peaks.push_back(peaks[i]);
      }
    }
  }
  // Use first LTS found
  if (valid_peaks.empty()) {
    best_peak = -1;
  } else {
    best_peak = valid_peaks[0];
  }

  return best_peak;
}

float CommsLib::FindMaxAbs(const Table<complex_float>& in, size_t dim1,
                           size_t dim2) {
  float max_val = 0;
  for (size_t i = 0; i < dim1; i++) {
    const float cur_max_val = CommsLib::FindMaxAbs(in.At(i), dim2);
    if (cur_max_val > max_val) {
      max_val = cur_max_val;
    }
  }
  return max_val;
}

float CommsLib::FindMaxAbs(const complex_float* in, size_t len) {
  float max_val = 0;
  for (size_t j = 0; j < len; j++) {
    const auto cur_val = CommsLib::AbsCf(in[j]);
    if (cur_val > max_val) {
      max_val = cur_val;
    }
  }
  return max_val;
}

float CommsLib::FindMeanAbs(const Table<complex_float>& in, size_t dim1,
                            size_t dim2) {
  float mean_val = 0;
  for (size_t i = 0; i < dim1; i++) {
    mean_val += CommsLib::FindMaxAbs(in.At(i), dim2);
  }
  return mean_val / (dim1 * dim2);
}

float CommsLib::FindMeanAbs(const complex_float* in, size_t len) {
  float mean_val = 0;
  for (size_t j = 0; j < len; j++) {
    mean_val += CommsLib::AbsCf(in[j]);
  }
  return mean_val / len;
}

void CommsLib::Meshgrid(const std::vector<int>& x_in,
                        const std::vector<int>& y_in,
                        std::vector<std::vector<int>>& x,
                        std::vector<std::vector<int>>& y) {
  /*
   * Simplified version of numpy's meshgrid function. Input vectors must be of
   * same length. Returns coordinate matrices from coordinate vectors.
   */
  const int nx = x_in.size();
  const int ny = y_in.size();

  if (nx != ny) {
    throw std::invalid_argument(
        " Input vectors to meshgrid function must have same length. ");
  }
  for (int i = 0; i < nx; i++) {
    for (int j = 0; j < ny; j++) {
      x[i].push_back(x_in[j]);
      y[i].push_back(y_in[i]);
      // std::cout << "XXXX x[" << i << "][" << j << "]: " << x[i][j] <<
      // std::endl; std::cout << "YYYY y[" << i << "][" << j << "]: " <<
      // y[i][j] << std::endl;
    }
  }
}

template <typename T>
std::vector<std::complex<T>> CommsLib::Csign(std::vector<std::complex<T>> iq) {
  /*
   * Return element-wise indication of the sign of a number (for complex
   * vector).
   *
   * For complex-valued inputs:
   *     sign(x.real) + 0j if x.real != 0 else sign(x.imag) + 0j
   *
   * where sign(x) is given by
   *     -1 if x < 0, 0 if x==0, 1 if x > 0
   */
  std::vector<std::complex<T>> iq_sign;
  for (int i = 0; i < static_cast<int>(iq.size()); i++) {
    // sign(x.real) + 0j if x.real != 0 else sign(x.imag) + 0j
    std::complex<T> x = iq[i];
    if (x.real() != 0) {
      iq_sign.push_back((x.real() > 0) ? 1 : (x.real() < 0) ? -1 : 0);
    } else {
      iq_sign.push_back((x.imag() > 0) ? 1 : (x.imag() < 0) ? -1 : 0);
    }
  }
  return iq_sign;
}

template <typename T>
std::vector<T> CommsLib::Convolve(std::vector<std::complex<T>> const& f,
                                  std::vector<std::complex<T>> const& g) {
  /* Convolution of two vectors
   * Source:
   * https://stackoverflow.com/questions/24518989/how-to-perform-1-dimensional-valid-convolution
   */
  int const nf = f.size();
  int const ng = g.size();
  int const n = nf + ng - 1;
  std::vector<T> out(n, 0);
  std::vector<std::complex<T>> outc(n, 0);
  for (auto i(0); i < n; ++i) {
    int const jmn = (i >= ng - 1) ? i - (ng - 1) : 0;
    int const jmx = (i < nf - 1) ? i : nf - 1;
    for (auto j(jmn); j <= jmx; ++j) {
      outc[i] += f[j] * g[i - j];
    }
    out[i] += abs(outc[i]);
  }
  return out;
}

std::vector<float> CommsLib::MagnitudeFft(
    std::vector<std::complex<float>> const& samps,
    std::vector<float> const& win, size_t fft_size) {
  std::vector<std::complex<float>> pre_fft(samps.size());

  for (size_t n = 0; n < fft_size; n++) {
    pre_fft[n] = samps[n] * win[n];
  }

  CommsLib::FFT(pre_fft, fft_size);
  //pre_fft has now been modified, giving it another name for code clarity
  std::vector<std::complex<float>>& fft_samps = pre_fft;

  // compute magnitudes
  std::vector<float> fft_mag;
  fft_mag.reserve(fft_size);
  for (size_t n = (fft_size / 2); n < fft_size; n++) {
    fft_mag.push_back(std::norm(fft_samps[n]));
  }
  for (size_t n = 0; n < fft_size / 2; n++) {
    fft_mag.push_back(std::norm(fft_samps[n]));
  }
  // not sure why we need reverse here, but
  // this seems to give the right spectrum
  std::reverse(fft_mag.begin(), fft_mag.end());
  return fft_mag;
}

// Take ffsSize samples of (1 - cos(x)) / 2 from 0 up to 2pi
std::vector<float> CommsLib::HannWindowFunction(size_t fft_size) {
  std::vector<float> win_fcn(1, 0);
  double step = 2 * M_PI / fft_size;

  // Compute the samples for the first half.
  for (size_t n = 1; n < fft_size / 2; n++) {
    win_fcn.push_back((1 - std::cos(step * n)) / 2);
  }
  // If a sample lies at the center, just use (1-cos(pi))/2 == 1.
  if (fft_size % 2 == 0) {
    win_fcn.push_back(1);
  }
  // The second half is a mirror image of the first, so just copy.
  for (size_t n = fft_size / 2 + 1; n < fft_size; n++) {
    win_fcn.push_back(win_fcn[fft_size - n]);
  }
  return win_fcn;
}

double CommsLib::WindowFunctionPower(std::vector<float> const& win) {
  double window_power = (0);
  size_t window_size = win.size();
  for (float i : win) {
    window_power += std::norm(i);
  }
  window_power = std::sqrt(window_power / window_size);
  return 20 * std::log10(window_size * window_power);
}

float CommsLib::FindTone(std::vector<float> const& magnitude, double win_gain,
                         double fft_bin, size_t fft_size, const size_t delta) {
  /*
   * Find the tone level at a specific interval in the input Power Spectrum
   * fft_bins assumed interval is [-0.5, 0.5] which is coverted to [0,
   * fft_size-1]
   */
  // make sure we don't exceed array bounds
  const size_t first =
      std::max<size_t>(0, std::lround((fft_bin + 0.5) * fft_size) - delta);
  const size_t last = std::min<size_t>(
      fft_size - 1, std::lround((fft_bin + 0.5) * fft_size) + delta);
  float ref_level = magnitude[last];
  for (size_t n = first; n < last; n++) {
    if (magnitude[n] > ref_level) {
      ref_level = magnitude[n];
    }
  }
  return 10 * std::max(std::log10(ref_level), (float)(-20.0f)) -
         (float)win_gain;
}

float CommsLib::MeasureTone(std::vector<std::complex<float>> const& samps,
                            std::vector<float> const& win, double win_gain,
                            double fft_bin, size_t fft_size,
                            const size_t delta) {
  return FindTone(MagnitudeFft(samps, win, fft_size), win_gain, fft_bin,
                  fft_size, delta);
}

std::vector<size_t> CommsLib::GetDataSc(size_t fft_size, size_t data_sc_num,
                                        size_t pilot_sc_offset,
                                        size_t pilot_sc_spacing) {
  std::vector<size_t> data_sc;
  if (fft_size == kFftSize_80211) {
    // We follow 802.11 PHY format here
    const size_t sc_ind[48u] = {1,  2,  3,  4,  5,  6,  8,  9,  10, 11, 12, 13,
                                14, 15, 16, 17, 18, 19, 20, 22, 23, 24, 25, 26,
                                38, 39, 40, 41, 42, 44, 45, 46, 47, 48, 49, 50,
                                51, 52, 53, 54, 55, 56, 58, 59, 60, 61, 62, 63};
    data_sc.assign(sc_ind, sc_ind + 48u);
  } else {
    // Allocate the center subcarriers as data
    const size_t start_sc = (fft_size - data_sc_num) / 2;
    const size_t stop_sc = start_sc + data_sc_num;
    for (size_t i = start_sc; i < stop_sc; i++) {
      const size_t sc_cnt = i - start_sc;
      if ((sc_cnt % pilot_sc_spacing) != pilot_sc_offset) {
        data_sc.push_back(i);
      }
    }
  }
  return data_sc;
}

std::vector<size_t> CommsLib::GetNullSc(size_t fft_size, size_t data_sc_num) {
  std::vector<size_t> null_sc;
  if (fft_size == kFftSize_80211) {
    // We follow 802.11 PHY format here
    size_t null[12u] = {0, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37};
    null_sc.assign(null, null + 12u);
  } else {  // Allocate the boundary subcarriers as null
    const size_t start_sc = (fft_size - data_sc_num) / 2;
    const size_t stop_sc = start_sc + data_sc_num;
    for (size_t i = 0; i < start_sc; i++) {
      null_sc.push_back(i);
    }
    for (size_t i = stop_sc; i < fft_size; i++) {
      null_sc.push_back(i);
    }
  }
  return null_sc;
}

std::vector<std::complex<float>> CommsLib::GetPilotScValue(
    size_t fft_size, size_t data_sc_num, size_t pilot_sc_offset,
    size_t pilot_sc_spacing) {
  std::vector<std::complex<float>> pilot_sc;
  if (fft_size == kFftSize_80211) {
    // We follow 802.11 PHY format here
    std::complex<float> sc_val[4] = {
        std::complex<float>(1.0f, 0.0f), std::complex<float>(1.0f, 0.0f),
        std::complex<float>(-1.0f, 0.0f), std::complex<float>(1.0f, 0.0f)};
    pilot_sc.assign(sc_val, sc_val + 4u);
  } else {
    const auto zc_seq_double =
        CommsLib::GetSequence(data_sc_num, CommsLib::kLteZadoffChu);
    const auto zc_seq = Utils::DoubleToCfloat(zc_seq_double);
    for (size_t i = pilot_sc_offset; i < zc_seq.size(); i += pilot_sc_spacing) {
      pilot_sc.push_back(zc_seq.at(i));
    }
  }
  return pilot_sc;
}

std::vector<size_t> CommsLib::GetPilotScIdx(size_t fft_size, size_t data_sc_num,
                                            size_t pilot_sc_offset,
                                            size_t pilot_sc_spacing) {
  std::vector<size_t> pilot_sc;
  if (fft_size == kFftSize_80211) {
    // We follow 802.11 standard here
    const size_t sc_ind[4u] = {7, 21, 43, 57};
    pilot_sc.assign(sc_ind, sc_ind + 4);
  } else {  // consider center subcarriers
    const size_t start_sc = (fft_size - data_sc_num) / 2;
    const size_t stop_sc = start_sc + data_sc_num;
    // pilot at the center of each RB
    for (size_t i = start_sc + pilot_sc_offset; i < stop_sc;
         i += pilot_sc_spacing) {
      pilot_sc.push_back(i);
    }
  }
  return pilot_sc;
}

MKL_LONG CommsLib::IFFT(std::vector<std::complex<float>>& in_out, int fft_size,
                        bool normalize) {
  DFTI_DESCRIPTOR_HANDLE mkl_handle;
  MKL_LONG status;
  status =
      DftiCreateDescriptor(&mkl_handle, DFTI_SINGLE, DFTI_COMPLEX, 1, fft_size);
  if (status != DFTI_NO_ERROR) {
    AGORA_LOG_ERROR("Error creating descriptor in CommsLib::IFFT%s\n",
                    DftiErrorMessage(status));
  } else {
    status = DftiCommitDescriptor(mkl_handle);
  }
  if (status != DFTI_NO_ERROR) {
    AGORA_LOG_ERROR("Error committing descriptor in CommsLib::IFFT%s\n",
                    DftiErrorMessage(status));
  } else {
    status = DftiComputeBackward(mkl_handle, in_out.data());
  }
  if (status != DFTI_NO_ERROR) {
    AGORA_LOG_ERROR("Error compute backward in CommsLib::IFFT%s\n",
                    DftiErrorMessage(status));
  } else {
    status = DftiFreeDescriptor(&mkl_handle);
  }
  if (status != DFTI_NO_ERROR) {
    AGORA_LOG_ERROR("Error free descriptor in CommsLib::IFFT%s\n",
                    DftiErrorMessage(status));
  } else {
    if (normalize) {
      float max_val = 0;
      const float scale = 0.5;
      for (int i = 0; i < fft_size; i++) {
        if (std::abs(in_out[i]) > max_val) {
          max_val = std::abs(in_out[i]);
        }
      }
      // std::cout << "IFFT output is normalized with "
      //         << std::to_string(max_val) << std::endl;
      for (int i = 0; i < fft_size; i++) {
        in_out[i] /= (max_val / scale);
      }
    } else {
      for (int i = 0; i < fft_size; i++) {
        in_out[i] /= fft_size;
      }
    }
  }
  return status;
}

MKL_LONG CommsLib::FFT(std::vector<std::complex<float>>& in_out, int fft_size) {
  DFTI_DESCRIPTOR_HANDLE mkl_handle;
  MKL_LONG status =
      DftiCreateDescriptor(&mkl_handle, DFTI_SINGLE, DFTI_COMPLEX, 1, fft_size);

  if (status != DFTI_NO_ERROR) {
    AGORA_LOG_ERROR("Error creating descriptor in CommsLib::FFT%s\n",
                    DftiErrorMessage(status));
  } else {
    status = DftiCommitDescriptor(mkl_handle);
  }

  if (status != DFTI_NO_ERROR) {
    AGORA_LOG_ERROR("Error committing descriptor in CommsLib::FFT%s\n",
                    DftiErrorMessage(status));
  } else {
    /* compute FFT */
    status = DftiComputeForward(mkl_handle, in_out.data());
  }
  if (status != DFTI_NO_ERROR) {
    AGORA_LOG_ERROR("Error compute forward in CommsLib::FFT%s\n",
                    DftiErrorMessage(status));
  } else {
    status = DftiFreeDescriptor(&mkl_handle);
  }
  if (status != DFTI_NO_ERROR) {
    AGORA_LOG_ERROR("Error freeing descriptor in CommsLib::FFT%s\n",
                    DftiErrorMessage(status));
  }
  return status;
}

MKL_LONG CommsLib::IFFT(complex_float* in_out, int fft_size, bool normalize) {
  DFTI_DESCRIPTOR_HANDLE mkl_handle;

  MKL_LONG status =
      DftiCreateDescriptor(&mkl_handle, DFTI_SINGLE, DFTI_COMPLEX, 1, fft_size);
  if (status != DFTI_NO_ERROR) {
    AGORA_LOG_ERROR("Error creating descriptor in CommsLib::IFFT%s\n",
                    DftiErrorMessage(status));
  } else {
    status = DftiCommitDescriptor(mkl_handle);
  }
  if (status != DFTI_NO_ERROR) {
    AGORA_LOG_ERROR("Error committing descriptor in CommsLib::IFFT%s\n",
                    DftiErrorMessage(status));
  } else {
    status = DftiComputeBackward(mkl_handle, in_out);
  }
  if (status != DFTI_NO_ERROR) {
    AGORA_LOG_ERROR("Error compute backward in CommsLib::IFFT%s\n",
                    DftiErrorMessage(status));
  } else {
    status = DftiFreeDescriptor(&mkl_handle);
  }
  if (status != DFTI_NO_ERROR) {
    AGORA_LOG_ERROR("Error free descriptor in CommsLib::IFFT%s\n",
                    DftiErrorMessage(status));
  } else {
    if (normalize == true) {
      float max_val = 0;
      const float scale = 0.5;
      for (int i = 0; i < fft_size; i++) {
        const float sc_abs =
            std::abs(std::complex<float>(in_out[i].re, in_out[i].im));
        if (sc_abs > max_val) {
          max_val = sc_abs;
        }
      }
      // std::cout << "IFFT output is normalized with "
      //         << std::to_string(max_val) << std::endl;
      for (int i = 0; i < fft_size; i++) {
        in_out[i] = {in_out[i].re / (max_val / scale),
                     in_out[i].im / (max_val / scale)};
      }
    } else {
      for (int i = 0; i < fft_size; i++) {
        in_out[i].re /= fft_size;
        in_out[i].im /= fft_size;
      }
    }
  }
  return status;
}

MKL_LONG CommsLib::FFT(complex_float* in_out, int fft_size) {
  DFTI_DESCRIPTOR_HANDLE mkl_handle;
  MKL_LONG status =
      DftiCreateDescriptor(&mkl_handle, DFTI_SINGLE, DFTI_COMPLEX, 1, fft_size);
  if (status != DFTI_NO_ERROR) {
    AGORA_LOG_ERROR("Error creating descriptor in CommsLib::IFFT%s\n",
                    DftiErrorMessage(status));
  } else {
    status = DftiCommitDescriptor(mkl_handle);
  }
  if (status != DFTI_NO_ERROR) {
    AGORA_LOG_ERROR("Error committing descriptor in CommsLib::IFFT%s\n",
                    DftiErrorMessage(status));
  } else {
    /* compute FFT */
    status = DftiComputeForward(mkl_handle, in_out);
  }
  if (status != DFTI_NO_ERROR) {
    AGORA_LOG_ERROR("Error computing forward in CommsLib::IFFT%s\n",
                    DftiErrorMessage(status));
  } else {
    status = DftiFreeDescriptor(&mkl_handle);
  }
  if (status != DFTI_NO_ERROR) {
    AGORA_LOG_ERROR("Error freeing descriptor in CommsLib::IFFT%s\n",
                    DftiErrorMessage(status));
  }
  return status;
}

void CommsLib::FFTShift(complex_float* in, complex_float* tmp, int fft_size) {
  std::memcpy(tmp, in + fft_size / 2, sizeof(float) * fft_size);
  std::memcpy(in + fft_size / 2, in, sizeof(float) * fft_size);
  std::memcpy(in, tmp, sizeof(float) * fft_size);
}

std::vector<std::complex<float>> CommsLib::FFTShift(
    const std::vector<std::complex<float>>& in) {
  const size_t fft_size = in.size();
  std::vector<std::complex<float>> out(fft_size);
  std::vector<std::complex<float>> in_freq_shifted;
  in_freq_shifted.insert(in_freq_shifted.end(), in.begin() + fft_size / 2,
                         in.end());
  in_freq_shifted.insert(in_freq_shifted.end(), in.begin(),
                         in.begin() + fft_size / 2);
  std::memcpy(out.data(), in_freq_shifted.data(),
              fft_size * sizeof(std::complex<float>));
  return out;
}

std::vector<complex_float> CommsLib::FFTShift(
    const std::vector<complex_float>& in) {
  const size_t fft_size = in.size();
  std::vector<complex_float> out(fft_size);
  std::vector<complex_float> in_freq_shifted;
  in_freq_shifted.insert(in_freq_shifted.end(), in.begin() + fft_size / 2,
                         in.end());
  in_freq_shifted.insert(in_freq_shifted.end(), in.begin(),
                         in.begin() + fft_size / 2);
  std::memcpy(out.data(), in_freq_shifted.data(), fft_size * sizeof(float) * 2);
  return out;
}

float CommsLib::ComputeOfdmSnr(const std::vector<std::complex<float>>& data_t,
                               size_t data_start_index,
                               size_t data_stop_index) {
  RtAssert(
      (data_t.size() > data_stop_index) && (data_stop_index > data_start_index),
      "Invalid ComputeOfdmSnr Inputs!");

  //Copy the const input to an output vector and do an inplace transform
  auto fft_data(data_t);
  CommsLib::FFT(fft_data, fft_data.size());
  auto fft_data_shift = CommsLib::FFTShift(fft_data);
  const auto fft_mag = CommsLib::Abs2Avx(fft_data_shift);
  float rssi = 0;
  float noise = 0;
  for (size_t i = 0; i < fft_mag.size(); i++) {
    rssi += fft_mag.at(i);
    if (i < data_start_index || i >= data_stop_index) {
      noise += fft_mag.at(i);
    }
  }
  const size_t noise_sc_size =
      fft_data_shift.size() - (data_stop_index - data_start_index);
  noise *= (fft_mag.size() / noise_sc_size);
  return (10.0f * std::log10((rssi - noise) / noise));
}

std::vector<std::complex<float>> CommsLib::ComposePartialPilotSym(
    const std::vector<std::complex<float>>& pilot, size_t offset,
    size_t pilot_sc_num, size_t fft_size, size_t data_size, size_t data_start,
    size_t cp_len, bool interleaved_pilot, bool time_domain) {
  std::vector<std::complex<float>> result(fft_size, 0);
  const size_t period = data_size / pilot_sc_num;
  for (size_t i = 0; i < pilot_sc_num; i++) {
    const size_t index =
        interleaved_pilot ? (i * period + offset) : (i + offset);
    result[index + data_start] = pilot[index];
  }
  if (time_domain) {
    CommsLib::IFFT(result, fft_size);
    for (auto& i : result) {
      i /= std::sqrt(period);
    }
    result.insert(result.begin(), result.end() - cp_len,
                  result.end());  // add CP
  }
  return result;
}

void CommsLib::Ifft2tx(const complex_float* in, std::complex<short>* out,
                       size_t N, size_t prefix, size_t cp, float scale) {
  ConvertFloatToShort(reinterpret_cast<const float*>(in),
                      reinterpret_cast<short*>(&out[prefix]), N * 2, cp * 2,
                      scale);
}

std::vector<std::complex<float>> CommsLib::Modulate(
    const std::vector<int8_t>& in, int type) {
  std::vector<std::complex<float>> out(in.size());
  if (type == kQpsk) {
    float qpsk_table[2][4];  // = init_qpsk();
    float scale = 1 / std::sqrt(2);
    float mod_qpsk[2] = {-scale, scale};
    for (int i = 0; i < 4; i++) {
      qpsk_table[0][i] = mod_qpsk[i / 2];
      qpsk_table[1][i] = mod_qpsk[i % 2];
    }
    for (size_t i = 0; i < in.size(); i++) {
      if (in[i] >= 0 && in[i] < 4) {
        out[i] =
            std::complex<float>(qpsk_table[0][in[i]], qpsk_table[1][in[i]]);
      } else {
        std::cout << "Error: No compatible input vector!" << std::endl;
        break;
      }
    }
  } else if (type == kQaM16) {
    float qam16_table[2][16];  //= init_qam16();
    const float scale = 1 / sqrt(10);
    const float mod_16qam[4] = {-3 * scale, -1 * scale, 3 * scale, scale};
    for (int i = 0; i < 16; i++) {
      qam16_table[0][i] = mod_16qam[i / 4];
      qam16_table[1][i] = mod_16qam[i % 4];
    }
    for (size_t i = 0; i < in.size(); i++) {
      if (in[i] >= 0 && in[i] < 16) {
        out[i] =
            std::complex<float>(qam16_table[0][in[i]], qam16_table[1][in[i]]);
      } else {
        std::cout << "Error: No compatible input vector!" << std::endl;
        break;
      }
    }
  } else if (type == kQaM64) {
    float qam64_table[2][64];  // = init_qam64();
    const float scale = 1 / sqrt(42);
    const float mod_64qam[8] = {-7 * scale, -5 * scale, -3 * scale, -1 * scale,
                                scale,      3 * scale,  5 * scale,  7 * scale};
    for (int i = 0; i < 64; i++) {
      qam64_table[0][i] = mod_64qam[i / 8];
      qam64_table[1][i] = mod_64qam[i % 8];
    }
    for (size_t i = 0; i < in.size(); i++) {
      if (in[i] >= 0 && in[i] < 64) {
        out[i] =
            std::complex<float>(qam64_table[0][in[i]], qam64_table[1][in[i]]);
      } else {
        std::cout << "Error: No compatible input vector!" << std::endl;
        break;
      }
    }
  } else {
    // Not Supported
    std::cout << "Modulation Type " << type << " not supported!" << std::endl;
  }
  return out;
}

std::vector<std::complex<float>> CommsLib::SeqCyclicShift(
    const std::vector<std::complex<float>>& in, float alpha) {
  std::vector<std::complex<float>> out(in.size(), 0);
  for (size_t i = 0; i < in.size(); i++) {
    out[i] = in[i] * std::exp(std::complex<float>(0, i * alpha));
  }
  return out;
}

std::vector<std::vector<double>> CommsLib::GetSequence(size_t seq_len,
                                                       int type) {
  std::vector<std::vector<double>> matrix;

  if (type == kStsSeq) {
    // STS - 802.11 Short training sequence (one symbol)
    matrix.resize(2);
    const size_t sts_seq_len = 16;

    std::vector<std::complex<float>> sts_freq(kStsSeqArray,
                                              kStsSeqArray + kFftSize_80211);
    // Perform ifft with ifft-shift on sts_freq
    auto sts_iq = CommsLib::FFTShift(sts_freq);
    CommsLib::IFFT(sts_iq, kFftSize_80211, false);

    size_t out_seq_len = sts_seq_len;
    size_t frac_seq_len = out_seq_len % sts_seq_len;
    matrix[0].resize(out_seq_len);
    matrix[1].resize(out_seq_len);
    for (size_t i = 0; i < out_seq_len; i++) {
      matrix[0][i] = sts_iq[(i + frac_seq_len) % sts_seq_len].real();
      matrix[1][i] = sts_iq[(i + frac_seq_len) % sts_seq_len].imag();
    }
  } else if (type == kLtsFSeq || type == kLtsSeq) {
    matrix.resize(2);
    const size_t lts_seq_len = kFftSize_80211;

    std::vector<std::complex<float>> lts_freq(kLtsSeqArray,
                                              kLtsSeqArray + lts_seq_len);
    if (type == kLtsFSeq) {
      matrix[0].resize(lts_seq_len);
      matrix[1].resize(lts_seq_len);
      for (size_t i = 0; i < lts_seq_len; i++) {
        matrix[0][i] = lts_freq[i].real();
        matrix[1][i] = lts_freq[i].imag();
      }
    } else {
      // Perform ifft with ifft-shift on lts_freq
      auto lts_iq = CommsLib::FFTShift(lts_freq);
      CommsLib::IFFT(lts_iq, lts_seq_len, false);

      size_t out_seq_len = lts_seq_len;  // TODO: use 160
      size_t frac_seq_len = out_seq_len % lts_seq_len;
      matrix[0].resize(out_seq_len);
      matrix[1].resize(out_seq_len);
      for (size_t i = 0; i < out_seq_len; i++) {
        matrix[0][i] = lts_iq[(i + frac_seq_len) % lts_seq_len].real();
        matrix[1][i] = lts_iq[(i + frac_seq_len) % lts_seq_len].imag();
      }
    }
  } else if (type == kLteZadoffChu) {
    // https://www.etsi.org/deliver/etsi_ts/136200_136299/136211/10.01.00_60/ts_136211v100100p.pdf
    // ETSI TS 136 211 V10.1.0 (sec. 5.5)
    matrix.resize(2);
    double u = 1;  // Cell ID 1
    double v = 0;
    int m = kPrimeArray[308];
    for (int j = 0; j < 308; j++) {
      if (kPrimeArray[j] < seq_len && kPrimeArray[j + 1] > seq_len) {
        m = kPrimeArray[j];
        break;
      }
    }
    double qh = m * (u + 1) / 31;
    double q = std::floor(qh + 0.5) + v * std::pow(-1, std::floor(2 * qh));
    std::vector<double> a;
    for (size_t i = 0; i < seq_len; i++) {
      int m_loop = i % m;
      double a_re = std::cos(-M_PI * q * m_loop * (m_loop + 1) / m);
      double a_im = std::sin(-M_PI * q * m_loop * (m_loop + 1) / m);
      matrix[0].push_back(a_re);
      matrix[1].push_back(a_im);
    }
  } else if (type == kGoldIfft) {
    // Gold IFFT Sequence - seq_length=128, cp=32, upsample=1
    matrix.resize(2);
    const size_t gold_seq_len = 128;

    // Use row 52 in gold-127
    std::vector<int> gold_freq_real(kGoldSeqArray, kGoldSeqArray + 127);

    // Insert 0 at center freq, construct inter-leaved quad code
    gold_freq_real.insert(gold_freq_real.begin() + 63, 0);

    std::vector<std::complex<float>> gold_freq(2 * gold_seq_len);
    for (size_t i = 0; i < gold_seq_len; i++) {
      gold_freq[2 * i] =
          std::complex<float>(gold_freq_real[i], gold_freq_real[i]);
    }

    // Perform ifft with ifft-shift on gold_freq
    auto gold_ifft_iq = CommsLib::FFTShift(gold_freq);
    CommsLib::IFFT(gold_ifft_iq, 2 * gold_seq_len, true);

    matrix[0].resize(gold_seq_len);
    matrix[1].resize(gold_seq_len);
    for (size_t i = 0; i < gold_seq_len; i++) {
      matrix[0][i] = gold_ifft_iq[i].real();
      matrix[1][i] = gold_ifft_iq[i].imag();
    }

  } else if (type == kHadamard) {
    // Hadamard - using Sylvester's construction for powers of 2.
    matrix.resize(seq_len);
    if ((seq_len & (seq_len - 1)) == 0) {
      for (size_t i = 0; i < seq_len; i++) {
        matrix[i].resize(seq_len);
        for (size_t j = 0; j < seq_len; j++) {
          matrix[i][j] = Hadamard2(i, j);
        }
      }
    }
  }
#if DEBUG_PRINT
  std::cout << "Num elements in first vector: \t " << matrix[0].size()
            << "   Number of rows: " << matrix.size() << std::endl;
  for (int i = 0; i < matrix.size(); i++) {
    for (int j = 0; j < matrix[i].size(); j++) {
      std::cout << "Values[" << i << "][" << j << "]: \t " << matrix[i][j]
                << std::endl;
    }
  }
#endif
  return matrix;
}

/*
int main(int argc, char *argv[])
{
    std::vector<std::vector<double> > sequence;
    int type = atoi(argv[1]);
    int N = atoi(argv[2]); 	// If Hadamard, possible N: {2, 4, 8, 16, 32,
64} sequence = SequenceGen::getSequence(N, type); return 0;
}
*/
