#ifndef MOD_COMMON
#define MOD_COMMON

#include "symbols.h"
#include "buffer.inc"
#include "gettime.h"
#include "memory_manage.h"
#include <cmath>
#include <emmintrin.h>
#include <immintrin.h>
#include <iostream>

#define BPSK_LEVEL M_SQRT1_2
#define QPSK_LEVEL M_SQRT1_2

#define SCALE_BYTE_CONV_QPSK 20
#define SCALE_BYTE_CONV_QAM16 100
#define SCALE_BYTE_CONV_QAM64 100
#define QAM16_THRESHOLD 2 / sqrt(10)
#define QAM64_THRESHOLD_1 2 / sqrt(42)
#define QAM64_THRESHOLD_2 4 / sqrt(42)
#define QAM64_THRESHOLD_3 6 / sqrt(42)

void init_modulation_table(Table<complex_float>& table, size_t mod_order);
void init_qpsk_table(Table<complex_float>& table);
void init_qam16_table(Table<complex_float>& table);
void init_qam64_table(Table<complex_float>& table);

complex_float mod_single(int x, Table<complex_float>& mod_table);
complex_float mod_single_uint8(uint8_t x, Table<complex_float>& mod_table);
void mod_simd(uint8_t* in, complex_float*& out, size_t len,
    Table<complex_float>& mod_table);

void demod_qpsk_soft_sse(float* vec_in, int8_t* llr, int num);

void demod_16qam_hard_loop(float* vec_in, uint8_t* vec_out, int num);
void demod_16qam_hard_sse(float* vec_in, uint8_t* vec_out, int num);
void demod_16qam_hard_avx2(float* vec_in, uint8_t* vec_out, int num);

void demod_16qam_soft_loop(float* vec_in, int8_t* llr, int num);
void demod_16qam_soft_sse(float* vec_in, int8_t* llr, int num);
void demod_16qam_soft_avx2(float* vec_in, int8_t* llr, int num);

void demod_64qam_hard_loop(float* vec_in, uint8_t* vec_out, int num);
void demod_64qam_hard_sse(float* vec_in, uint8_t* vec_out, int num);
void demod_64qam_hard_avx2(float* vec_in, uint8_t* vec_out, int num);

void demod_64qam_soft_loop(float* vec_in, int8_t* llr, int num);
void demod_64qam_soft_sse(float* vec_in, int8_t* llr, int num);
void demod_64qam_soft_avx2(float* vec_in, int8_t* llr, int num);

void print256_epi8(__m256i var);

#endif
