#ifndef MODULATION_H_
#define MODULATION_H_

#include <emmintrin.h>
#include <immintrin.h>

#include <cmath>
#include <iostream>

#include "common_typedef_sdk.h"
#include "gettime.h"
#include "memory_manage.h"
#include "message.h"
#include "symbols.h"

#define BPSK_LEVEL M_SQRT1_2
#define QPSK_LEVEL M_SQRT1_2

#define SCALE_BYTE_CONV_QPSK 20
#define SCALE_BYTE_CONV_QAM16 100
#define SCALE_BYTE_CONV_QAM64 100
#define SCALE_BYTE_CONV_QAM256 100
#define QAM16_THRESHOLD (2 / sqrt(10))
#define QAM64_THRESHOLD_1 (2 / sqrt(42))
#define QAM64_THRESHOLD_2 (4 / sqrt(42))
#define QAM64_THRESHOLD_3 (6 / sqrt(42))
#define QAM256_THRESHOLD_1 (2 / sqrt(170))
#define QAM256_THRESHOLD_2 (4 / sqrt(170))
#define QAM256_THRESHOLD_3 (6 / sqrt(170))
#define QAM256_THRESHOLD_4 (8 / sqrt(170))
#define QAM256_THRESHOLD_5 (10 / sqrt(170))
#define QAM256_THRESHOLD_6 (12 / sqrt(170))
#define QAM256_THRESHOLD_7 (14 / sqrt(170))

static const std::map<std::string, size_t> kModulStringMap{
    {"BPSK", 1},  {"QPSK", 2},   {"16QAM", 4},
    {"64QAM", 6}, {"256QAM", 8}, {"1024QAM", 10}};

static inline std::string MapModToStr(size_t mod_order) {
  switch (mod_order) {
    case 1:
      return std::string("BPSK");
    case 2:
      return std::string("QPSK");
    case 4:
      return std::string("16QAM");
    case 6:
      return std::string("64QAM");
    case 8:
      return std::string("256QAM");
    case 10:
      return std::string("1024QAM");
    default:
      return std::string("UNKNOWN!");
  }
}

void InitModulationTable(Table<complex_float>& table, size_t mod_order);
void InitQpskTable(Table<complex_float>& table);
void InitQam16Table(Table<complex_float>& table);
void InitQam64Table(Table<complex_float>& table);
void InitQam256Table(Table<complex_float>& table);

complex_float ModSingle(int x, Table<complex_float>& mod_table);
complex_float ModSingleUint8(uint8_t x, Table<complex_float>& mod_table);
void ModSimd(uint8_t* in, complex_float*& out, size_t len,
             Table<complex_float>& mod_table);

void DemodQpskHardLoop(const float* vec_in, uint8_t* vec_out, int num);
void DemodQpskSoftSse(float* x, int8_t* z, int len);

void Demod16qamHardLoop(const float* vec_in, uint8_t* vec_out, int num);
void Demod16qamHardSse(float* vec_in, uint8_t* vec_out, int num);
void Demod16qamHardAvx2(float* vec_in, uint8_t* vec_out, int num);

void Demod16qamSoftLoop(const float* vec_in, int8_t* llr, int num);
void Demod16qamSoftSse(float* vec_in, int8_t* llr, int num);
void Demod16qamSoftAvx2(float* vec_in, int8_t* llr, int num);

void Demod64qamHardLoop(const float* vec_in, uint8_t* vec_out, int num);
void Demod64qamHardSse(float* vec_in, uint8_t* vec_out, int num);
void Demod64qamHardAvx2(float* vec_in, uint8_t* vec_out, int num);

void Demod64qamSoftLoop(const float* vec_in, int8_t* llr, int num);
void Demod64qamSoftSse(float* vec_in, int8_t* llr, int num);
void Demod64qamSoftAvx2(float* vec_in, int8_t* llr, int num);

void Demod256qamHardLoop(const float* vec_in, uint8_t* vec_out, int num);
void Demod256qamHardSse(float* vec_in, uint8_t* vec_out, int num);
void Demod256qamHardAvx2(float* vec_in, uint8_t* vec_out, int num);
#ifdef __AVX512F__
void Demod256qamHardAvx512(float* vec_in, uint8_t* vec_out, int num);
#endif
void Demod256qamSoftLoop(const float* vec_in, int8_t* llr, int num);
void Demod256qamSoftSse(const float* vec_in, int8_t* llr, int num);
void Demod256qamSoftAvx2(const float* vec_in, int8_t* llr, int num);

#ifdef __AVX512F__
void Demod256qamSoftAvx512(const float* vec_in, int8_t* llr, int num);
#endif
void Print256Epi8(__m256i var);

#endif  // MODULATION_H_
