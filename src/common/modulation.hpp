/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#ifndef MOD_COMMON
#define MOD_COMMON

#include "Symbols.hpp"
#include "buffer.hpp"
#include "gettime.h"
#include "memory_manage.h"
#include <cmath>
#include <emmintrin.h>
#include <immintrin.h>
#include <iostream>
#include <stdio.h>

#define SCALE_BYTE_CONV_QAM16 100
#define SCALE_BYTE_CONV_QAM64 100
#define QAM16_THRESHOLD 2 / sqrt(10)
#define QAM64_THRESHOLD_1 2 / sqrt(42)
#define QAM64_THRESHOLD_2 4 / sqrt(42)
#define QAM64_THRESHOLD_3 6 / sqrt(42)

void init_modulation_table(Table<float>& table, size_t mod_order);
void init_qpsk_table(Table<float>& table);
void init_qam16_table(Table<float>& table);
void init_qam64_table(Table<float>& table);

complex_float mod_single(int x, Table<float>& mod_table);
complex_float mod_single_uint8(uint8_t x, Table<float>& mod_table);

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
