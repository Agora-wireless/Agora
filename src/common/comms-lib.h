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

#include "buffer.hpp"
#include "memory_manage.h"
#include "mkl_dfti.h"
#include <algorithm>
#include <complex>
#include <fstream>
#include <immintrin.h>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <vector>

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

    static std::vector<std::vector<double>> getSequence(int N, int type);
    static std::vector<std::complex<float>> modulate(std::vector<int8_t>, int);
    static std::vector<int> getDataSc(int fftSize);
    static std::vector<int> getNullSc(int fftSize);
    static std::vector<int> getPilotScInd(int fftSize);
    static std::vector<std::complex<float>> getPilotSc(int fftSize);
    static std::vector<std::complex<float>> FFT(
        std::vector<std::complex<float>>, int);
    static std::vector<std::complex<float>> IFFT(
        std::vector<std::complex<float>>, int, bool scale = true);
    static void FFT(complex_float*, int);
    static void IFFT(complex_float*, int, bool normalize = true);
    static size_t find_pilot_seq(std::vector<std::complex<float>> iq,
        std::vector<std::complex<float>> pilot, size_t seqLen);
    static int findLTS(std::vector<std::complex<double>> iq, int seqLen);
    template <typename T>
    static std::vector<T> convolve(std::vector<std::complex<T>> const& f,
        std::vector<std::complex<T>> const& g);
    template <typename T>
    static std::vector<std::complex<T>> csign(std::vector<std::complex<T>> iq);
    static void meshgrid(std::vector<int> x_in, std::vector<int> y_in,
        std::vector<std::vector<int>>& x, std::vector<std::vector<int>>& y);
    static inline int hadamard2(int i, int j)
    {
        return (__builtin_parity(i & j) != 0 ? -1 : 1);
    }
    static std::vector<float> magnitudeFFT(
        std::vector<std::complex<float>> const&, std::vector<float> const&,
        size_t);
    static std::vector<float> hannWindowFunction(size_t);
    static double windowFunctionPower(std::vector<float> const&);
    // template <typename T>
    // static T findTone(std::vector<T> const&, double, double, size_t, const
    // size_t delta = 10);
    static float findTone(std::vector<float> const&, double, double, size_t,
        const size_t delta = 10);
    static float measureTone(std::vector<std::complex<float>> const&,
        std::vector<float> const&, double, double, size_t,
        const size_t delta = 10);
    static std::vector<std::complex<float>> composeRefSymbol(
        std::vector<std::complex<float>>, size_t, size_t, size_t, size_t,
        size_t, size_t, bool timeDomain = true);
    static std::vector<std::complex<float>> seqCyclicShift(
        std::vector<std::complex<float>>, float);
    static float find_max_abs(complex_float*, size_t);
    static float find_max_abs(Table<complex_float>, size_t, size_t);
    static void ifft2tx(
        complex_float*, std::complex<short>*, size_t, size_t, size_t, float);
    static float abs_cf(complex_float d)
    {
        return std::abs(std::complex<float>(d.re, d.im));
    }
    static int find_beacon_avx(const std::vector<std::complex<float>>& iq,
        const std::vector<std::complex<float>>& seq);
    static std::vector<float> correlate_avx_s(
        std::vector<float> const& f, std::vector<float> const& g);
    static std::vector<float> abs2_avx(
        std::vector<std::complex<float>> const& f);
    static std::vector<int32_t> abs2_avx(
        std::vector<std::complex<int16_t>> const& f);
    static std::vector<std::complex<float>> auto_corr_mult_avx(
        std::vector<std::complex<float>> const& f, const int dly,
        const bool conj = true);
    static std::vector<std::complex<int16_t>> auto_corr_mult_avx(
        std::vector<std::complex<int16_t>> const& f, const int dly,
        const bool conj = true);
    static std::vector<std::complex<float>> correlate_avx(
        std::vector<std::complex<float>> const& f,
        std::vector<std::complex<float>> const& g);
    static std::vector<std::complex<float>> complex_mult_avx(
        std::vector<std::complex<float>> const& f,
        std::vector<std::complex<float>> const& g, const bool conj);
    static std::vector<std::complex<int16_t>> complex_mult_avx(
        std::vector<std::complex<int16_t>> const& f,
        std::vector<std::complex<int16_t>> const& g, const bool conj);
    static std::vector<std::complex<int16_t>> correlate_avx(
        std::vector<std::complex<int16_t>> const& f,
        std::vector<std::complex<int16_t>> const& g);

    static __m256 __m256_complex_cf32_mult(
        __m256 data1, __m256 data2, bool conj);
};

#endif
