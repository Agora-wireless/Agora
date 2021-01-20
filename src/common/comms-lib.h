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

#ifndef COMMSLIB_HEADER
#define COMMSLIB_HEADER

#include <immintrin.h>
#include <unistd.h>

#include <algorithm>
#include <complex>
#include <fstream>
#include <iostream>
#include <vector>

#include "buffer.hpp"
#include "memory_manage.h"
#include "mkl_dfti.h"

class CommsLib {
 public:
  enum SequenceType {
    STS_SEQ,
    LTS_SEQ,
    LTS_F_SEQ,
    LTE_ZADOFF_CHU,
    GOLD_IFFT,
    HADAMARD
  };

  enum ModulationOrder { QPSK = 2, QAM16 = 4, QAM64 = 6 };

  CommsLib(std::string);
  ~CommsLib();

  static std::vector<std::vector<double>> GetSequence(int N, int type);
  static std::vector<std::complex<float>> Modulate(std::vector<int8_t>, int);
  static std::vector<int> GetDataSc(int fftSize);
  static std::vector<int> GetNullSc(int fftSize);
  static std::vector<int> GetPilotScInd(int fftSize);
  static std::vector<std::complex<float>> GetPilotSc(int fftSize);
  static std::vector<std::complex<float>> FFT(std::vector<std::complex<float>>,
                                              int);
  static std::vector<std::complex<float>> IFFT(std::vector<std::complex<float>>,
                                               int, bool scale = true);
  static void FFT(complex_float*, int);
  static void IFFT(complex_float*, int, bool normalize = true);
  static size_t FindPilotSeq(std::vector<std::complex<float>> iq,
                               std::vector<std::complex<float>> pilot,
                               size_t seqLen);
  static int FindLts(std::vector<std::complex<double>> iq, int seqLen);
  template <typename T>
  static std::vector<T> Convolve(std::vector<std::complex<T>> const& f,
                                 std::vector<std::complex<T>> const& g);
  template <typename T>
  static std::vector<std::complex<T>> Csign(std::vector<std::complex<T>> iq);
  static void Meshgrid(std::vector<int> x_in, std::vector<int> y_in,
                       std::vector<std::vector<int>>& x,
                       std::vector<std::vector<int>>& y);
  static inline int Hadamard2(int i, int j) {
    return (__builtin_parity(i & j) != 0 ? -1 : 1);
  }
  static std::vector<float> MagnitudeFft(
      std::vector<std::complex<float>> const&, std::vector<float> const&,
      size_t);
  static std::vector<float> HannWindowFunction(size_t);
  static double WindowFunctionPower(std::vector<float> const&);
  // template <typename T>
  // static T findTone(std::vector<T> const&, double, double, size_t, const
  // size_t delta = 10);
  static float FindTone(std::vector<float> const&, double, double, size_t,
                        const size_t delta = 10);
  static float MeasureTone(std::vector<std::complex<float>> const&,
                           std::vector<float> const&, double, double, size_t,
                           const size_t delta = 10);
  static std::vector<std::complex<float>> ComposePartialPilotSym(
      std::vector<std::complex<float>>, size_t, size_t, size_t, size_t, size_t,
      size_t, bool, bool timeDomain = true);
  static std::vector<std::complex<float>> SeqCyclicShift(
      std::vector<std::complex<float>>, float);
  static float FindMaxAbs(complex_float*, size_t);
  static float FindMaxAbs(Table<complex_float>, size_t, size_t);
  static void Ifft2tx(complex_float*, std::complex<short>*, size_t, size_t,
                      size_t, float);
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
};

#endif
