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

int CommsLib::FindLts(const std::vector<std::complex<double>>& iq, int seqLen) {
  /*
   * Find 802.11-based LTS (Long Training Sequence)
   * Input:
   *     iq        - IQ complex samples (vector)
   *     seqLen    - Length of sequence
   * Output:
   *     best_peak - LTS peak index (correlation peak)
   */

  float lts_thresh = 0.8;
  std::vector<std::vector<double>> lts_seq;
  int best_peak;

  // Original LTS sequence
  lts_seq = CommsLib::GetSequence(seqLen, kLtsSeq);

  // Re-arrange into complex vector, flip, and compute conjugate
  std::vector<std::complex<double>> lts_sym;
  std::vector<std::complex<double>> lts_sym_conj;
  for (int i = 0; i < 64; i++) {
    // lts_seq is a 2x160 matrix (real/imag by seqLen=160 elements)
    // grab one symbol and flip around
    lts_sym.emplace_back(lts_seq[0][seqLen - 1 - i],
                         lts_seq[1][seqLen - 1 - i]);
    // conjugate
    lts_sym_conj.push_back(std::conj(lts_sym[i]));
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
    std::vector<float> const& win, size_t fftSize) {
  std::vector<std::complex<float>> pre_fft(samps.size());

  for (size_t n = 0; n < fftSize; n++) {
    pre_fft[n] = samps[n] * win[n];
  }

  CommsLib::FFT(pre_fft, fftSize);
  //pre_fft has now been modified, giving it another name for code clarity
  std::vector<std::complex<float>>& fft_samps = pre_fft;

  // compute magnitudes
  std::vector<float> fft_mag;
  fft_mag.reserve(fftSize);
  for (size_t n = (fftSize / 2); n < fftSize; n++) {
    fft_mag.push_back(std::norm(fft_samps[n]));
  }
  for (size_t n = 0; n < fftSize / 2; n++) {
    fft_mag.push_back(std::norm(fft_samps[n]));
  }
  // not sure why we need reverse here, but
  // this seems to give the right spectrum
  std::reverse(fft_mag.begin(), fft_mag.end());
  return fft_mag;
}

// Take ffsSize samples of (1 - cos(x)) / 2 from 0 up to 2pi
std::vector<float> CommsLib::HannWindowFunction(size_t fftSize) {
  std::vector<float> win_fcn(1, 0);
  double step = 2 * M_PI / fftSize;

  // Compute the samples for the first half.
  for (size_t n = 1; n < fftSize / 2; n++) {
    win_fcn.push_back((1 - std::cos(step * n)) / 2);
  }
  // If a sample lies at the center, just use (1-cos(pi))/2 == 1.
  if (fftSize % 2 == 0) {
    win_fcn.push_back(1);
  }
  // The second half is a mirror image of the first, so just copy.
  for (size_t n = fftSize / 2 + 1; n < fftSize; n++) {
    win_fcn.push_back(win_fcn[fftSize - n]);
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

float CommsLib::FindTone(std::vector<float> const& magnitude, double winGain,
                         double fftBin, size_t fftSize, const size_t delta) {
  /*
   * Find the tone level at a specific interval in the input Power Spectrum
   * fftBins assumed interval is [-0.5, 0.5] which is coverted to [0,
   * fftSize-1]
   */
  // make sure we don't exceed array bounds
  size_t first =
      std::max<size_t>(0, std::lround((fftBin + 0.5) * fftSize) - delta);
  size_t last = std::min<size_t>(fftSize - 1,
                                 std::lround((fftBin + 0.5) * fftSize) + delta);
  float ref_level = magnitude[last];
  for (size_t n = first; n < last; n++) {
    if (magnitude[n] > ref_level) {
      ref_level = magnitude[n];
    }
  }
  return 10 * std::max(std::log10(ref_level), (float)(-20.0)) - (float)winGain;
}

float CommsLib::MeasureTone(std::vector<std::complex<float>> const& samps,
                            std::vector<float> const& win, double winGain,
                            double fftBin, size_t fftSize, const size_t delta) {
  return FindTone(MagnitudeFft(samps, win, fftSize), winGain, fftBin, fftSize,
                  delta);
}

std::vector<int> CommsLib::GetDataSc(int fftSize) {
  std::vector<int> data_sc;
  if (fftSize == 64) {
    int sc_ind[48] = {1,  2,  3,  4,  5,  6,  8,  9,  10, 11, 12, 13,
                      14, 15, 16, 17, 18, 19, 20, 22, 23, 24, 25, 26,
                      38, 39, 40, 41, 42, 44, 45, 46, 47, 48, 49, 50,
                      51, 52, 53, 54, 55, 56, 58, 59, 60, 61, 62, 63};
    data_sc.assign(sc_ind, sc_ind + 48);
  } else {
    for (int i = 0; i < fftSize; i++) {
      data_sc.push_back(i);
    }
  }
  return data_sc;
}

std::vector<int> CommsLib::GetNullSc(int fftSize) {
  std::vector<int> null_sc;
  if (fftSize == 64) {
    int null[12] = {0, 1, 2, 3, 4, 5, 32, 59, 60, 61, 62, 63};
    null_sc.assign(null, null + 12);
  }
  return null_sc;
}

std::vector<int> CommsLib::GetPilotScInd(int fftSize) {
  std::vector<int> pilot_sc;
  if (fftSize == 64) {
    int sc_ind[4] = {7, 21, 43, 57};
    pilot_sc.assign(sc_ind, sc_ind + 4);
  }
  return pilot_sc;
}

std::vector<std::complex<float>> CommsLib::GetPilotSc(int fftSize) {
  std::vector<std::complex<float>> pilot_sc;
  if (fftSize == 64) {
    std::complex<float> sc_val[4] = {1.0, 1.0, -1.0, 1.0};
    pilot_sc.assign(sc_val, sc_val + 4);
  }
  return pilot_sc;
}

MKL_LONG CommsLib::IFFT(std::vector<std::complex<float>>& in_out, int fftsize,
                        bool normalize) {
  DFTI_DESCRIPTOR_HANDLE mkl_handle;
  MKL_LONG status;
  status =
      DftiCreateDescriptor(&mkl_handle, DFTI_SINGLE, DFTI_COMPLEX, 1, fftsize);
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
      for (int i = 0; i < fftsize; i++) {
        if (std::abs(in_out[i]) > max_val) {
          max_val = std::abs(in_out[i]);
        }
      }
      // std::cout << "IFFT output is normalized with "
      //         << std::to_string(max_val) << std::endl;
      for (int i = 0; i < fftsize; i++) {
        in_out[i] /= (max_val / scale);
      }
    } else {
      for (int i = 0; i < fftsize; i++) {
        in_out[i] /= fftsize;
      }
    }
  }
  return status;
}

MKL_LONG CommsLib::FFT(std::vector<std::complex<float>>& in_out, int fftsize) {
  DFTI_DESCRIPTOR_HANDLE mkl_handle;
  MKL_LONG status =
      DftiCreateDescriptor(&mkl_handle, DFTI_SINGLE, DFTI_COMPLEX, 1, fftsize);

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

MKL_LONG CommsLib::IFFT(complex_float* in_out, int fftsize, bool normalize) {
  DFTI_DESCRIPTOR_HANDLE mkl_handle;

  MKL_LONG status =
      DftiCreateDescriptor(&mkl_handle, DFTI_SINGLE, DFTI_COMPLEX, 1, fftsize);
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
      for (int i = 0; i < fftsize; i++) {
        const float sc_abs =
            std::abs(std::complex<float>(in_out[i].re, in_out[i].im));
        if (sc_abs > max_val) {
          max_val = sc_abs;
        }
      }
      // std::cout << "IFFT output is normalized with "
      //         << std::to_string(max_val) << std::endl;
      for (int i = 0; i < fftsize; i++) {
        in_out[i] = {in_out[i].re / (max_val / scale),
                     in_out[i].im / (max_val / scale)};
      }
    } else {
      for (int i = 0; i < fftsize; i++) {
        in_out[i].re /= fftsize;
        in_out[i].im /= fftsize;
      }
    }
  }
  return status;
}

MKL_LONG CommsLib::FFT(complex_float* in_out, int fftsize) {
  DFTI_DESCRIPTOR_HANDLE mkl_handle;
  MKL_LONG status =
      DftiCreateDescriptor(&mkl_handle, DFTI_SINGLE, DFTI_COMPLEX, 1, fftsize);
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

void CommsLib::FFTShift(complex_float* in, complex_float* tmp, int fftsize) {
  std::memcpy(tmp, in + fftsize / 2, sizeof(float) * fftsize);
  std::memcpy(in + fftsize / 2, in, sizeof(float) * fftsize);
  std::memcpy(in, tmp, sizeof(float) * fftsize);
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
  const auto fft_mag = CommsLib::Abs2Avx(fft_data);
  float rssi = 0;
  float noise = 0;
  for (size_t i = 0; i < fft_mag.size(); i++) {
    rssi += fft_mag.at(i);
    if (i < data_start_index || i >= data_stop_index) {
      noise += fft_mag.at(i);
    }
  }
  const size_t noise_sc_size =
      fft_data.size() - (data_stop_index - data_start_index);
  noise *= (fft_mag.size() / noise_sc_size);
  return (10.0f * std::log10((rssi - noise) / noise));
}

std::vector<std::complex<float>> CommsLib::ComposePartialPilotSym(
    const std::vector<std::complex<float>>& pilot, size_t offset,
    size_t pilot_sc_num, size_t fftSize, size_t dataSize, size_t dataStart,
    size_t CP_LEN, bool interleaved_pilot, bool timeDomain) {
  std::vector<std::complex<float>> result(fftSize, 0);
  size_t period = dataSize / pilot_sc_num;
  for (size_t i = 0; i < pilot_sc_num; i++) {
    const size_t index = interleaved_pilot ? i * period + offset : i + offset;
    result[index + dataStart] = pilot[index];
  }
  if (timeDomain) {
    CommsLib::IFFT(result, fftSize);
    for (auto& i : result) {
      i /= std::sqrt(period);
    }
    result.insert(result.begin(), result.end() - CP_LEN,
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

std::vector<std::vector<double>> CommsLib::GetSequence(int N, int type) {
  std::vector<std::vector<double>> matrix;

  if (type == kStsSeq) {
    // STS - 802.11 Short training sequence (one symbol)
    matrix.resize(2);

    double sts_re[16] = {0.04599876, -0.13244371, -0.01347272, 0.1427553,
                         0.09199751, 0.1427553,   -0.01347272, -0.13244371,
                         0.04599876, 0.00233959,  -0.07852478, -0.01265117,
                         0,          -0.01265117, -0.07852478, 0.00233959};

    double sts_im[16] = {0.04599876, 0.00233959,  -0.07852478, -0.01265117,
                         0.0,        -0.01265117, -0.07852478, 0.00233959,
                         0.04599876, -0.13244371, -0.01347272, 0.1427553,
                         0.09199751, 0.1427553,   -0.01347272, -0.13244371};

    for (int j = 0; j < 2; j++) {
      std::vector<double> a;
      for (int i = 0; i < 16; i++) {
        if (j == 0) {
          a.push_back(sts_re[i]);
        } else {
          a.push_back(sts_im[i]);
        }
      }
      matrix[j] = a;
    }
  } else if (type == kLtsFSeq) {
    matrix.resize(1);
    std::vector<double> lts_f = {
        0, 1,  -1, -1, 1,  1,  -1, 1, -1, 1,  -1, -1, -1, -1, -1, 1,
        1, -1, -1, 1,  -1, 1,  -1, 1, 1,  1,  1,  0,  0,  0,  0,  0,
        0, 0,  0,  0,  0,  0,  1,  1, -1, -1, 1,  1,  -1, 1,  -1, 1,
        1, 1,  1,  1,  1,  -1, -1, 1, 1,  -1, 1,  -1, 1,  1,  1,  1};
    matrix[0] = (lts_f);
  } else if (type == kLtsSeq) {
    // LTS - 802.11 Long training sequence (2.5 symbols, cp length of 32
    // samples)
    matrix.resize(2);

    double lts_re[160] = {-0.15625,
                          0.012284590458567165,
                          0.09171654912240956,
                          -0.09188755526278,
                          -0.002805944173488664,
                          0.07507369706822604,
                          -0.12732435990770957,
                          -0.12188700906074086,
                          -0.03504126073623884,
                          -0.056455128448539,
                          -0.060310100316213804,
                          0.06955684740689412,
                          0.08221832230305733,
                          -0.1312626089753594,
                          -0.05720634587149917,
                          0.03691794200106715,
                          0.0625,
                          0.1192390885103326,
                          -0.022483206307774027,
                          0.05866876712873733,
                          0.0244758515211019,
                          -0.13680487681585982,
                          0.0009889797089880949,
                          0.05333773437415131,
                          0.09754126073623881,
                          -0.03831596747441851,
                          -0.11513121478170157,
                          0.05982384485901423,
                          0.021111770349329442,
                          0.09683188459112747,
                          0.0397496983535005,
                          -0.005121250360419827,
                          0.15625,
                          -0.005121250360419823,
                          0.0397496983535005,
                          0.09683188459112749,
                          0.02111177034932945,
                          0.05982384485901426,
                          -0.11513121478170157,
                          -0.0383159674744185,
                          0.09754126073623884,
                          0.05333773437415131,
                          0.0009889797089880983,
                          -0.1368048768158598,
                          0.024475851521101908,
                          0.05866876712873735,
                          -0.02248320630777403,
                          0.1192390885103326,
                          0.0625,
                          0.03691794200106713,
                          -0.05720634587149916,
                          -0.1312626089753594,
                          0.08221832230305731,
                          0.06955684740689413,
                          -0.0603101003162138,
                          -0.05645512844853901,
                          -0.03504126073623881,
                          -0.12188700906074088,
                          -0.12732435990770957,
                          0.07507369706822604,
                          -0.002805944173488671,
                          -0.09188755526278002,
                          0.09171654912240956,
                          0.01228459045856714,
                          -0.15625,
                          0.012284590458567165,
                          0.09171654912240956,
                          -0.09188755526278,
                          -0.002805944173488664,
                          0.07507369706822604,
                          -0.12732435990770957,
                          -0.12188700906074086,
                          -0.03504126073623884,
                          -0.056455128448539,
                          -0.060310100316213804,
                          0.06955684740689412,
                          0.08221832230305733,
                          -0.1312626089753594,
                          -0.05720634587149917,
                          0.03691794200106715,
                          0.0625,
                          0.1192390885103326,
                          -0.022483206307774027,
                          0.05866876712873733,
                          0.0244758515211019,
                          -0.13680487681585982,
                          0.0009889797089880949,
                          0.05333773437415131,
                          0.09754126073623881,
                          -0.03831596747441851,
                          -0.11513121478170157,
                          0.05982384485901423,
                          0.021111770349329442,
                          0.09683188459112747,
                          0.0397496983535005,
                          -0.005121250360419827,
                          0.15625,
                          -0.005121250360419823,
                          0.0397496983535005,
                          0.09683188459112749,
                          0.02111177034932945,
                          0.05982384485901426,
                          -0.11513121478170157,
                          -0.0383159674744185,
                          0.09754126073623884,
                          0.05333773437415131,
                          0.0009889797089880983,
                          -0.1368048768158598,
                          0.024475851521101908,
                          0.05866876712873735,
                          -0.02248320630777403,
                          0.1192390885103326,
                          0.0625,
                          0.03691794200106713,
                          -0.05720634587149916,
                          -0.1312626089753594,
                          0.08221832230305731,
                          0.06955684740689413,
                          -0.0603101003162138,
                          -0.05645512844853901,
                          -0.03504126073623881,
                          -0.12188700906074088,
                          -0.12732435990770957,
                          0.07507369706822604,
                          -0.002805944173488671,
                          -0.09188755526278002,
                          0.09171654912240956,
                          0.01228459045856714,
                          -0.15625,
                          0.012284590458567165,
                          0.09171654912240956,
                          -0.09188755526278,
                          -0.002805944173488664,
                          0.07507369706822604,
                          -0.12732435990770957,
                          -0.12188700906074086,
                          -0.03504126073623884,
                          -0.056455128448539,
                          -0.060310100316213804,
                          0.06955684740689412,
                          0.08221832230305733,
                          -0.1312626089753594,
                          -0.05720634587149917,
                          0.03691794200106715,
                          0.0625,
                          0.1192390885103326,
                          -0.022483206307774027,
                          0.05866876712873733,
                          0.0244758515211019,
                          -0.13680487681585982,
                          0.0009889797089880949,
                          0.05333773437415131,
                          0.09754126073623881,
                          -0.03831596747441851,
                          -0.11513121478170157,
                          0.05982384485901423,
                          0.021111770349329442,
                          0.09683188459112747,
                          0.0397496983535005,
                          -0.005121250360419827};

    double lts_im[160] = {0.0,
                          -0.09759955359207202,
                          -0.10587165981863113,
                          -0.11512870891096853,
                          -0.053774266476545984,
                          0.07404041892509948,
                          0.020501379986300285,
                          0.01656621813913718,
                          0.15088834764831843,
                          0.021803920607437133,
                          -0.08128612411572139,
                          -0.014121958590578302,
                          -0.09235655195372787,
                          -0.06522722901814465,
                          -0.039298588174111096,
                          -0.0983441502870872,
                          0.0625,
                          0.004095594414801514,
                          -0.1606573329526341,
                          0.01493899945069943,
                          0.05853179569459056,
                          0.04737981136568012,
                          0.11500464362403023,
                          -0.0040763264805083466,
                          0.025888347648318433,
                          0.10617091261510256,
                          0.05518049537437035,
                          0.08770675983572167,
                          -0.027885918828227545,
                          -0.08279790948776067,
                          0.11115794305116433,
                          0.12032513267372755,
                          0.0,
                          -0.1203251326737275,
                          -0.11115794305116432,
                          0.08279790948776065,
                          0.027885918828227538,
                          -0.0877067598357217,
                          -0.05518049537437036,
                          -0.10617091261510254,
                          -0.025888347648318433,
                          0.00407632648050834,
                          -0.11500464362403023,
                          -0.04737981136568013,
                          -0.05853179569459056,
                          -0.014938999450699438,
                          0.16065733295263412,
                          -0.0040955944148015275,
                          -0.0625,
                          0.09834415028708718,
                          0.0392985881741111,
                          0.06522722901814465,
                          0.09235655195372787,
                          0.014121958590578316,
                          0.08128612411572139,
                          -0.021803920607437126,
                          -0.15088834764831843,
                          -0.01656621813913719,
                          -0.02050137998630029,
                          -0.07404041892509945,
                          0.05377426647654598,
                          0.11512870891096855,
                          0.10587165981863114,
                          0.09759955359207204,
                          0.0,
                          -0.09759955359207202,
                          -0.10587165981863113,
                          -0.11512870891096853,
                          -0.053774266476545984,
                          0.07404041892509948,
                          0.020501379986300285,
                          0.01656621813913718,
                          0.15088834764831843,
                          0.021803920607437133,
                          -0.08128612411572139,
                          -0.014121958590578302,
                          -0.09235655195372787,
                          -0.06522722901814465,
                          -0.039298588174111096,
                          -0.0983441502870872,
                          0.0625,
                          0.004095594414801514,
                          -0.1606573329526341,
                          0.01493899945069943,
                          0.05853179569459056,
                          0.04737981136568012,
                          0.11500464362403023,
                          -0.0040763264805083466,
                          0.025888347648318433,
                          0.10617091261510256,
                          0.05518049537437035,
                          0.08770675983572167,
                          -0.027885918828227545,
                          -0.08279790948776067,
                          0.11115794305116433,
                          0.12032513267372755,
                          0.0,
                          -0.1203251326737275,
                          -0.11115794305116432,
                          0.08279790948776065,
                          0.027885918828227538,
                          -0.0877067598357217,
                          -0.05518049537437036,
                          -0.10617091261510254,
                          -0.025888347648318433,
                          0.00407632648050834,
                          -0.11500464362403023,
                          -0.04737981136568013,
                          -0.05853179569459056,
                          -0.014938999450699438,
                          0.16065733295263412,
                          -0.0040955944148015275,
                          -0.0625,
                          0.09834415028708718,
                          0.0392985881741111,
                          0.06522722901814465,
                          0.09235655195372787,
                          0.014121958590578316,
                          0.08128612411572139,
                          -0.021803920607437126,
                          -0.15088834764831843,
                          -0.01656621813913719,
                          -0.02050137998630029,
                          -0.07404041892509945,
                          0.05377426647654598,
                          0.11512870891096855,
                          0.10587165981863114,
                          0.09759955359207204,
                          0.0,
                          -0.09759955359207202,
                          -0.10587165981863113,
                          -0.11512870891096853,
                          -0.053774266476545984,
                          0.07404041892509948,
                          0.020501379986300285,
                          0.01656621813913718,
                          0.15088834764831843,
                          0.021803920607437133,
                          -0.08128612411572139,
                          -0.014121958590578302,
                          -0.09235655195372787,
                          -0.06522722901814465,
                          -0.039298588174111096,
                          -0.0983441502870872,
                          0.0625,
                          0.004095594414801514,
                          -0.1606573329526341,
                          0.01493899945069943,
                          0.05853179569459056,
                          0.04737981136568012,
                          0.11500464362403023,
                          -0.0040763264805083466,
                          0.025888347648318433,
                          0.10617091261510256,
                          0.05518049537437035,
                          0.08770675983572167,
                          -0.027885918828227545,
                          -0.08279790948776067,
                          0.11115794305116433,
                          0.12032513267372755};

    // Grab the last N samples (sequence length specified, provide more
    // flexibility)
    int start_idx = 160 - N;
    for (int j = 0; j < 2; j++) {
      std::vector<double> a;
      for (int i = start_idx; i < 160; i++) {
        if (j == 0) {
          a.push_back(lts_re[i]);
        } else {
          a.push_back(lts_im[i]);
        }
      }
      matrix[j] = a;
    }
  } else if (type == kLteZadoffChu) {
    // https://www.etsi.org/deliver/etsi_ts/136200_136299/136211/10.01.00_60/ts_136211v100100p.pdf
    // ETSI TS 136 211 V10.1.0 (sec. 5.5)
    matrix.resize(2);
    double u = 1;  // Cell ID 1
    double v = 0;
    int prime[309] = {
        2,    3,    5,    7,    11,   13,   17,   19,   23,   29,   31,   37,
        41,   43,   47,   53,   59,   61,   67,   71,   73,   79,   83,   89,
        97,   101,  103,  107,  109,  113,  127,  131,  137,  139,  149,  151,
        157,  163,  167,  173,  179,  181,  191,  193,  197,  199,  211,  223,
        227,  229,  233,  239,  241,  251,  257,  263,  269,  271,  277,  281,
        283,  293,  307,  311,  313,  317,  331,  337,  347,  349,  353,  359,
        367,  373,  379,  383,  389,  397,  401,  409,  419,  421,  431,  433,
        439,  443,  449,  457,  461,  463,  467,  479,  487,  491,  499,  503,
        509,  521,  523,  541,  547,  557,  563,  569,  571,  577,  587,  593,
        599,  601,  607,  613,  617,  619,  631,  641,  643,  647,  653,  659,
        661,  673,  677,  683,  691,  701,  709,  719,  727,  733,  739,  743,
        751,  757,  761,  769,  773,  787,  797,  809,  811,  821,  823,  827,
        829,  839,  853,  857,  859,  863,  877,  881,  883,  887,  907,  911,
        919,  929,  937,  941,  947,  953,  967,  971,  977,  983,  991,  997,
        1009, 1013, 1019, 1021, 1031, 1033, 1039, 1049, 1051, 1061, 1063, 1069,
        1087, 1091, 1093, 1097, 1103, 1109, 1117, 1123, 1129, 1151, 1153, 1163,
        1171, 1181, 1187, 1193, 1201, 1213, 1217, 1223, 1229, 1231, 1237, 1249,
        1259, 1277, 1279, 1283, 1289, 1291, 1297, 1301, 1303, 1307, 1319, 1321,
        1327, 1361, 1367, 1373, 1381, 1399, 1409, 1423, 1427, 1429, 1433, 1439,
        1447, 1451, 1453, 1459, 1471, 1481, 1483, 1487, 1489, 1493, 1499, 1511,
        1523, 1531, 1543, 1549, 1553, 1559, 1567, 1571, 1579, 1583, 1597, 1601,
        1607, 1609, 1613, 1619, 1621, 1627, 1637, 1657, 1663, 1667, 1669, 1693,
        1697, 1699, 1709, 1721, 1723, 1733, 1741, 1747, 1753, 1759, 1777, 1783,
        1787, 1789, 1801, 1811, 1823, 1831, 1847, 1861, 1867, 1871, 1873, 1877,
        1879, 1889, 1901, 1907, 1913, 1931, 1933, 1949, 1951, 1973, 1979, 1987,
        1993, 1997, 1999, 2003, 2011, 2017, 2027, 2029, 2039};
    int m = prime[308];
    for (int j = 0; j < 308; j++) {
      if (prime[j] < N && prime[j + 1] > N) {
        m = prime[j];
        break;
      }
    }
    double qh = m * (u + 1) / 31;
    double q = std::floor(qh + 0.5) + v * std::pow(-1, std::floor(2 * qh));
    std::vector<double> a;
    for (int i = 0; i < N; i++) {
      int m_loop = i % m;
      double a_re = std::cos(-M_PI * q * m_loop * (m_loop + 1) / m);
      double a_im = std::sin(-M_PI * q * m_loop * (m_loop + 1) / m);
      matrix[0].push_back(a_re);
      matrix[1].push_back(a_im);
    }
  } else if (type == kGoldIfft) {
    // Gold IFFT Sequence - seq_length=128, cp=32, upsample=1
    matrix.resize(2);

    double lts_re[128] = {
        -0.5646359,   0.4669951,    0.8769358,    0.5407985,    -0.48144832,
        -0.88476783,  0.33639774,   -0.43609348,  -0.26278743,  0.6910331,
        -0.25535262,  0.11774132,   0.46892625,   0.77644444,   -0.14834122,
        -0.13464923,  -0.26617187,  0.1341292,    0.133574,     0.15594807,
        -0.057847068, 0.3967621,    0.047606125,  0.01414329,   0.41560003,
        0.12632199,   -0.33603117,  -0.5669182,   -0.2004348,   0.55602646,
        0.24340886,   -0.16611233,  0.7904902,    -0.42025912,  -0.38651145,
        -0.14808364,  -0.27662534,  -0.74715126,  0.5908927,    -0.75451213,
        -0.33933204,  0.36646086,   -0.57852495,  0.10015667,   -0.34719938,
        0.35134,      0.7383081,    -0.3743101,   -0.53234375,  -0.33714586,
        0.012157675,  -0.399321,    -0.3871609,   0.27705255,   0.4469853,
        -0.16857521,  0.60894567,   -0.04652265,  0.21421923,   0.014229958,
        0.87569416,   -0.28046992,  0.64841086,   0.06317055,   -0.037642393,
        -0.7303067,   0.6826409,    -0.091142215, -0.080362685, 0.1991867,
        0.3268059,    0.6429179,    0.26278743,   -0.088880904, 0.25250778,
        0.2633651,    -0.7295981,   -0.15740044,  -0.44250035,  -0.0022179564,
        0.26617187,   -0.33556038,  -0.38437498,  -0.8211783,   0.641319,
        0.3527957,    -0.062620886, 0.4227164,    -0.23919682,  0.18401834,
        -0.14366682,  0.016121548,  -0.25830117,  0.82918876,   0.92221844,
        0.31633607,   -0.18821196,  -0.9082796,   0.11038142,   0.008659021,
        -0.18971694,  -0.40438867,  -0.12019706,  -0.6811534,   0.33933204,
        -0.40837204,  0.22615194,   0.38991654,   0.18199626,   -0.1321399,
        0.19951832,   0.7384663,    0.53234375,   0.030798966,  0.40922493,
        0.4283689,    -0.37271422,  0.22344504,   0.24096492,   0.1736422,
        0.4192076,    -0.42793053,  0.37122476,   -0.008662291, 0.008916863,
        0.34757638,   -0.35418823,  0.3462311};

    double lts_im[128] = {
        -0.5646359,   0.3462311,    -0.35418823,  0.34757638,   0.008916863,
        -0.008662291, 0.37122476,   -0.42793053,  0.4192076,    0.1736422,
        0.24096492,   0.22344504,   -0.37271422,  0.4283689,    0.40922493,
        0.030798966,  0.53234375,   0.7384663,    0.19951832,   -0.1321399,
        0.18199626,   0.38991654,   0.22615194,   -0.40837204,  0.33933204,
        -0.6811534,   -0.12019706,  -0.40438867,  -0.18971694,  0.008659021,
        0.11038142,   -0.9082796,   -0.18821196,  0.31633607,   0.92221844,
        0.82918876,   -0.25830117,  0.016121548,  -0.14366682,  0.18401834,
        -0.23919682,  0.4227164,    -0.062620886, 0.3527957,    0.641319,
        -0.8211783,   -0.38437498,  -0.33556038,  0.26617187,   -0.0022179564,
        -0.44250035,  -0.15740044,  -0.7295981,   0.2633651,    0.25250778,
        -0.088880904, 0.26278743,   0.6429179,    0.3268059,    0.1991867,
        -0.080362685, -0.091142215, 0.6826409,    -0.7303067,   -0.037642393,
        0.06317055,   0.64841086,   -0.28046992,  0.87569416,   0.014229958,
        0.21421923,   -0.04652265,  0.60894567,   -0.16857521,  0.4469853,
        0.27705255,   -0.3871609,   -0.399321,    0.012157675,  -0.33714586,
        -0.53234375,  -0.3743101,   0.7383081,    0.35134,      -0.34719938,
        0.10015667,   -0.57852495,  0.36646086,   -0.33933204,  -0.75451213,
        0.5908927,    -0.74715126,  -0.27662534,  -0.14808364,  -0.38651145,
        -0.42025912,  0.7904902,    -0.16611233,  0.24340886,   0.55602646,
        -0.2004348,   -0.5669182,   -0.33603117,  0.12632199,   0.41560003,
        0.01414329,   0.047606125,  0.3967621,    -0.057847068, 0.15594807,
        0.133574,     0.1341292,    -0.26617187,  -0.13464923,  -0.14834122,
        0.77644444,   0.46892625,   0.11774132,   -0.25535262,  0.6910331,
        -0.26278743,  -0.43609348,  0.33639774,   -0.88476783,  -0.48144832,
        0.5407985,    0.8769358,    0.4669951};

    for (int j = 0; j < 2; j++) {
      std::vector<double> a;
      for (int i = 0; i < 128; i++) {
        if (j == 0) {
          a.push_back(lts_re[i]);
        } else {
          a.push_back(lts_im[i]);
        }
      }
      matrix[j] = a;
    }
  } else if (type == kHadamard) {
    // Hadamard - using Sylvester's construction for powers of 2.
    matrix.resize(N);
    if ((N & (N - 1)) == 0) {
      for (int i = 0; i < N; i++) {
        matrix[i].resize(N);
        for (int j = 0; j < N; j++) {
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
