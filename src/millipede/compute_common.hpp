/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 * 
 */
#ifndef COMPUTE_COMMON
#define COMPUTE_COMMON

#include <iostream> 
#include <armadillo>
#include <stdio.h>
#include <immintrin.h>
#include <emmintrin.h>
#include "buffer.hpp"
#include "gettime.h"

using namespace arma;
void init_qpsk_table(float **qam16_table);
void init_qam16_table(float **qam16_table);
void init_qam64_table(float **qam16_table);
complex_float divide(complex_float e1, complex_float e2);
imat demod_16qam(cx_fmat x);
void demod_16qam_loop(float *vec_in, uint8_t *vec_out, int ue_num);
void demod_16qam_loop_simd(float *vec_in, uint8_t *vec_out, int ue_num, int num_simd256);
// inline arma::cx_fmat mod_16qam(arma::imat x);
complex_float mod_16qam_single(int x, float **qam16_table);



#endif
