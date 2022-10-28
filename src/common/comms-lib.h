// Copyright (c) 2018-2020, Rice University
// RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

/**
 * @file comms-lib.h
 * @brief Communications Library:
 *    a) Generate pilot/preamble sequences
 *    b) OFDM modulation
 * @author Rahman Doost-Mohamamdy: doost@rice.edu
 *         Oscar Bejarano: obejarano@rice.edu
 */
#ifndef COMMSLIB_H_
#define COMMSLIB_H_

#include <cmath>
#include <complex>
#include <map>
#include <string>
#include <vector>

#include "common_typedef_sdk.h"
#include "immintrin.h"
#include "memory_manage.h"
#include "mkl_dfti.h"

static const std::map<std::string, size_t> kBeamformingStr{
    {"ZF", 0}, {"MMSE", 1}, {"MRC", 2}};

class CommsLib {
 public:
  enum SequenceType {
    kStsSeq,
    kLtsSeq,
    kLtsFSeq,
    kLteZadoffChu,
    kGoldIfft,
    kHadamard
  };

  enum ModulationOrder {
    kBpsk = 1,
    kQpsk = 2,
    kQaM16 = 4,
    kQaM64 = 6,
    kQaM256 = 8
  };

  enum BeamformingAlgorithm { kZF = 0, kMMSE = 1, kMRC = 2 };

  explicit CommsLib(std::string);
  ~CommsLib();

  static std::vector<std::vector<double>> GetSequence(size_t seq_len, int type);
  static std::vector<std::complex<float>> Modulate(
      const std::vector<int8_t>& in, int type);

  static std::vector<size_t> GetDataSc(size_t fft_size, size_t data_sc_num,
                                       size_t pilot_sc_offset,
                                       size_t pilot_sc_spacing);
  static std::vector<size_t> GetNullSc(size_t fft_size, size_t data_sc_num);
  static std::vector<std::complex<float>> GetPilotScValue(
      size_t fft_size, size_t data_sc_num, size_t pilot_sc_offset,
      size_t pilot_sc_spacing);
  static std::vector<size_t> GetPilotScIdx(size_t fft_size, size_t data_sc_num,
                                           size_t pilot_sc_offset,
                                           size_t pilot_sc_spacing);

  static MKL_LONG FFT(std::vector<std::complex<float>>& in_out, int fft_size);
  static MKL_LONG IFFT(std::vector<std::complex<float>>& in_out, int fft_size,
                       bool normalize = true);
  static MKL_LONG FFT(complex_float* in_out, int fft_size);
  static MKL_LONG IFFT(complex_float* in_out, int fft_size,
                       bool normalize = true);
  static std::vector<std::complex<float>> FFTShift(
      const std::vector<std::complex<float>>& in);
  static std::vector<complex_float> FFTShift(
      const std::vector<complex_float>& in);
  static void FFTShift(complex_float* in, complex_float* tmp, int fft_size);

  static float ComputeOfdmSnr(const std::vector<std::complex<float>>& data_t,
                              size_t data_start_index, size_t data_stop_index);
  static size_t FindPilotSeq(const std::vector<std::complex<float>>& iq,
                             const std::vector<std::complex<float>>& pilot,
                             size_t seq_len);
  static int FindLts(const std::vector<std::complex<double>>& iq, int seq_len);
  template <typename T>
  static std::vector<T> Convolve(std::vector<std::complex<T>> const& f,
                                 std::vector<std::complex<T>> const& g);
  template <typename T>
  static std::vector<std::complex<T>> Csign(std::vector<std::complex<T>> iq);
  static void Meshgrid(const std::vector<int>& x_in,
                       const std::vector<int>& y_in,
                       std::vector<std::vector<int>>& x,
                       std::vector<std::vector<int>>& y);
  static inline int Hadamard2(int i, int j) {
    return (__builtin_parity(i & j) != 0 ? -1 : 1);
  }
  static std::vector<float> MagnitudeFft(
      std::vector<std::complex<float>> const& samps,
      std::vector<float> const& win, size_t fft_size);
  static std::vector<float> HannWindowFunction(size_t fft_size);
  static double WindowFunctionPower(std::vector<float> const& win);
  static float FindTone(std::vector<float> const& magnitude, double win_gain,
                        double fft_bin, size_t fft_size,
                        const size_t delta = 10);
  static float MeasureTone(std::vector<std::complex<float>> const& samps,
                           std::vector<float> const& win, double win_gain,
                           double fft_bin, size_t fft_size,
                           const size_t delta = 10);
  static std::vector<std::complex<float>> ComposePartialPilotSym(
      const std::vector<std::complex<float>>& pilot, size_t offset,
      size_t pilot_sc_num, size_t fft_size, size_t data_size, size_t data_start,
      size_t cp_len, bool interleaved_pilot, bool time_domain = true);
  static std::vector<std::complex<float>> SeqCyclicShift(
      const std::vector<std::complex<float>>& in, float alpha);
  static float FindMaxAbs(const complex_float* in, size_t len);
  static float FindMaxAbs(const Table<complex_float>& in, size_t dim1,
                          size_t dim2);
  static float FindMeanAbs(const complex_float* in, size_t len);
  static float FindMeanAbs(const Table<complex_float>& in, size_t dim1,
                           size_t dim2);
  static void Ifft2tx(const complex_float* in, std::complex<short>* out,
                      size_t N, size_t prefix, size_t cp, float scale);
  static float AbsCf(complex_float d) {
    return std::abs(std::complex<float>(d.re, d.im));
  }
  static int FindBeaconAvx(const std::vector<std::complex<float>>& iq,
                           const std::vector<std::complex<float>>& seq,
                           float corr_scale = 1.f);

  ///Find Beacon with raw samples from the radio
  static ssize_t FindBeaconAvx(const std::complex<int16_t>* iq,
                               const std::vector<std::complex<float>>& seq,
                               size_t sample_window, float corr_scale = 1.f);

  static std::vector<float> CorrelateAvxS(std::vector<float> const& f,
                                          std::vector<float> const& g);
  static std::vector<float> Abs2Avx(std::vector<std::complex<float>> const& f);
  static std::vector<int32_t> Abs2Avx(
      std::vector<std::complex<int16_t>> const& f);
  static std::vector<std::complex<float>> AutoCorrMultAvx(
      std::vector<std::complex<float>> const& f, const int dly,
      const bool conj = true);
  static std::vector<std::complex<int16_t>> AutoCorrMultAvx(
      std::vector<std::complex<int16_t>> const& f, const int dly,
      const bool conj = true);
  static std::vector<std::complex<float>> CorrelateAvx(
      std::vector<std::complex<float>> const& f,
      std::vector<std::complex<float>> const& g);
  static std::vector<std::complex<float>> ComplexMultAvx(
      std::vector<std::complex<float>> const& f,
      std::vector<std::complex<float>> const& g, const bool conj);
  static std::vector<std::complex<int16_t>> ComplexMultAvx(
      std::vector<std::complex<int16_t>> const& f,
      std::vector<std::complex<int16_t>> const& g, const bool conj);
  static std::vector<std::complex<int16_t>> CorrelateAvx(
      std::vector<std::complex<int16_t>> const& f,
      std::vector<std::complex<int16_t>> const& g);

  static __m256 M256ComplexCf32Mult(__m256 data1, __m256 data2, bool conj);
#ifdef __AVX512F__
  static __m512 M512ComplexCf32Mult(__m512 data1, __m512 data2, bool conj);
#endif
};

#endif  // COMMSLIB_H_
