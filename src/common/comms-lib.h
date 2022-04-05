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

#include <immintrin.h>
#include <unistd.h>

#include <algorithm>
#include <complex>
#include <fstream>
#include <iostream>
#include <vector>

#include "buffer.h"
#include "memory_manage.h"
#include "mkl_dfti.h"

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

  explicit CommsLib(std::string);
  ~CommsLib();

  static std::vector<std::vector<double>> GetSequence(int N, int type);
  static std::vector<std::complex<float>> Modulate(
      const std::vector<int8_t>& in, int type);
  static std::vector<int> GetDataSc(int fftSize);
  static std::vector<int> GetNullSc(int fftSize);
  static std::vector<int> GetPilotScInd(int fftSize);
  static std::vector<std::complex<float>> GetPilotSc(int fftSize);
  static MKL_LONG FFT(std::vector<std::complex<float>>& in_out, int fftsize);
  static MKL_LONG IFFT(std::vector<std::complex<float>>& in_out, int fftsize,
                       bool normalize = true);
  static MKL_LONG FFT(complex_float* in_out, int fftsize);
  static MKL_LONG IFFT(complex_float* in_out, int fftsize,
                       bool normalize = true);
  static float ComputeOfdmSnr(const std::vector<std::complex<float>>& in,
                              size_t data_start_index, size_t data_stop_index);
  static size_t FindPilotSeq(const std::vector<std::complex<float>>& iq,
                             const std::vector<std::complex<float>>& pilot,
                             size_t seq_len);
  static int FindLts(const std::vector<std::complex<double>>& iq, int seqLen);
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
      std::vector<float> const& win, size_t fftSize);
  static std::vector<float> HannWindowFunction(size_t fftSize);
  static double WindowFunctionPower(std::vector<float> const& win);
  // template <typename T>
  // static T findTone(std::vector<T> const&, double, double, size_t, const
  // size_t delta = 10);
  static float FindTone(std::vector<float> const& magnitude, double winGain,
                        double fftBin, size_t fftSize, const size_t delta = 10);
  static float MeasureTone(std::vector<std::complex<float>> const& samps,
                           std::vector<float> const& win, double winGain,
                           double fftBin, size_t fftSize,
                           const size_t delta = 10);
  static std::vector<std::complex<float>> ComposePartialPilotSym(
      const std::vector<std::complex<float>>& pilot, size_t offset,
      size_t pilot_sc_num, size_t fftSize, size_t dataSize, size_t dataStart,
      size_t CP_LEN, bool interleaved_pilot, bool timeDomain = true);
  static std::vector<std::complex<float>> SeqCyclicShift(
      const std::vector<std::complex<float>>& in, float alpha);
  static float FindMaxAbs(const complex_float* in, size_t len);
  static float FindMaxAbs(const Table<complex_float>& in, size_t dim1,
                          size_t dim2);
  static void Ifft2tx(const complex_float* in, std::complex<short>* out,
                      size_t N, size_t prefix, size_t cp, float scale);
  static float AbsCf(complex_float d) {
    return std::abs(std::complex<float>(d.re, d.im));
  }
  static int FindBeaconAvx(const std::vector<std::complex<float>>& iq,
                           const std::vector<std::complex<float>>& seq);
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
