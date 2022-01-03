#ifndef _LDPC_ENCODER_CYCLESHIFT_H
#define _LDPC_ENCODER_CYCLESHIFT_H

/*******************************************************************************
 * Include public/global header files
 ******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include "xmmintrin.h" // SSE
#include "emmintrin.h" // SSE 2
#include "pmmintrin.h" // SSE 3
#include "tmmintrin.h" // SSSE 3
#include "smmintrin.h" // SSE 4 for media
#include <immintrin.h> // AVX

inline __m512i cycle_bit_left_shift_from288to384(__m512i data, int16_t cycLeftShift, int16_t zcSize);//1ways
inline __m512i cycle_bit_left_shift_from144to256(__m512i data, int16_t cycLeftShift, int16_t zcSize);//2ways
inline __m512i cycle_bit_left_shift_from72to128(__m512i data, int16_t cycLeftShift, int16_t zcSize);//4ways
inline __m512i cycle_bit_left_shift_less_than_64(__m512i data, int16_t cycLeftShift, int16_t zcSize);//8ways
inline __m512i cycle_bit_left_shift_special(__m512i data, int16_t cycLeftShift, int16_t zcSize);

typedef __m512i (* CYCLE_BIT_LEFT_SHIFT)(__m512i, int16_t, int16_t);
CYCLE_BIT_LEFT_SHIFT ldpc_select_left_shift_func(int16_t zcSize);
#endif
