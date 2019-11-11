/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 * 
 */
#ifndef COMPUTE_COMMON
#define COMPUTE_COMMON

#include <iostream> 
#include <stdio.h>
#include <immintrin.h>
#include <emmintrin.h>
// #include <armadillo>
#include "buffer.hpp"
#include <cmath>

#include "gettime.h"
#include "memory_manage.h"

#define SCALE_BYTE_CONV_QAM16 	100
#define SCALE_BYTE_CONV_QAM64 	100
#define QAM16_THRESHOLD 		2/sqrt(10)
#define QAM64_THRESHOLD_1       2/sqrt(42)
#define QAM64_THRESHOLD_2       4/sqrt(42)
#define QAM64_THRESHOLD_3       6/sqrt(42)

// using namespace arma;
// using namespace std;
float **init_modulation_table(size_t mod_order);
float **init_qpsk_table();
// void init_qam16_table(float **qam16_table);
float **init_qam16_table();
float **init_qam64_table();


complex_float mod_single(int x, float **mod_table);
complex_float mod_single_uint8(uint8_t x, float **mod_table);

void demod_16qam_hard_loop(float *vec_in, uint8_t *vec_out, int num);
void demod_16qam_hard_sse(float *vec_in, uint8_t *vec_out, int num);
void demod_16qam_hard_avx2(float *vec_in, uint8_t *vec_out, int num);

void demod_16qam_soft_loop(float *vec_in, int8_t *llr, int num);
void demod_16qam_soft_sse(float *vec_in, int8_t *llr, int num);
void demod_16qam_soft_avx2(float *vec_in, int8_t *llr, int num);

void demod_64qam_hard_loop(float *vec_in, uint8_t *vec_out, int num);
void demod_64qam_hard_sse(float *vec_in, uint8_t *vec_out, int num);
void demod_64qam_hard_avx2(float *vec_in, uint8_t *vec_out, int num);

void demod_64qam_soft_loop(float *vec_in, int8_t *llr, int num);
void demod_64qam_soft_sse(float *vec_in, int8_t *llr, int num);
void demod_64qam_soft_avx2(float *vec_in, int8_t *llr, int num);

void print256_epi8(__m256i var);

#endif
