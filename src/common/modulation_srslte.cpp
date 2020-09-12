/**
 * Copyright 2013-2020 Software Radio Systems Limited
 *
 * This file is part of srsLTE.
 *
 * srsLTE is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsLTE is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

#include "modulation.hpp"

void demod_16qam_soft_loop(float* vec_in, int8_t* llr, int num)
{
    for (int i = 0; i < num; i++) {
        int8_t yre = (int8_t)(SCALE_BYTE_CONV_QAM16 * (vec_in[2 * i]));
        int8_t yim = (int8_t)(SCALE_BYTE_CONV_QAM16 * (vec_in[2 * i + 1]));

        llr[4 * i + 0] = yre;
        llr[4 * i + 1] = yim;
        llr[4 * i + 2] = 2 * SCALE_BYTE_CONV_QAM16 / sqrt(10) - abs(yre);
        llr[4 * i + 3] = 2 * SCALE_BYTE_CONV_QAM16 / sqrt(10) - abs(yim);
    }
}

void demod_16qam_soft_sse(float* vec_in, int8_t* llr, int num)
{
    float* symbolsPtr = vec_in;
    __m128i* resultPtr = (__m128i*)llr;
    __m128 symbol1, symbol2, symbol3, symbol4;
    __m128i symbol_i1, symbol_i2, symbol_i3, symbol_i4, symbol_i, symbol_abs,
        symbol_12, symbol_34;
    __m128i offset = _mm_set1_epi8(2 * SCALE_BYTE_CONV_QAM16 / sqrt(10));
    __m128i result1n, result1a, result2n, result2a;
    __m128 scale_v = _mm_set1_ps(SCALE_BYTE_CONV_QAM16);

    __m128i shuffle_negated_1 = _mm_set_epi8(
        0xff, 0xff, 7, 6, 0xff, 0xff, 5, 4, 0xff, 0xff, 3, 2, 0xff, 0xff, 1, 0);
    __m128i shuffle_abs_1 = _mm_set_epi8(
        7, 6, 0xff, 0xff, 5, 4, 0xff, 0xff, 3, 2, 0xff, 0xff, 1, 0, 0xff, 0xff);

    __m128i shuffle_negated_2 = _mm_set_epi8(0xff, 0xff, 15, 14, 0xff, 0xff, 13,
        12, 0xff, 0xff, 11, 10, 0xff, 0xff, 9, 8);
    __m128i shuffle_abs_2 = _mm_set_epi8(15, 14, 0xff, 0xff, 13, 12, 0xff, 0xff,
        11, 10, 0xff, 0xff, 9, 8, 0xff, 0xff);

    for (int i = 0; i < num / 8; i++) {
        symbol1 = _mm_load_ps(symbolsPtr);
        symbolsPtr += 4;
        symbol2 = _mm_load_ps(symbolsPtr);
        symbolsPtr += 4;
        symbol3 = _mm_load_ps(symbolsPtr);
        symbolsPtr += 4;
        symbol4 = _mm_load_ps(symbolsPtr);
        symbolsPtr += 4;
        symbol_i1 = _mm_cvtps_epi32(_mm_mul_ps(symbol1, scale_v));
        symbol_i2 = _mm_cvtps_epi32(_mm_mul_ps(symbol2, scale_v));
        symbol_i3 = _mm_cvtps_epi32(_mm_mul_ps(symbol3, scale_v));
        symbol_i4 = _mm_cvtps_epi32(_mm_mul_ps(symbol4, scale_v));
        symbol_12 = _mm_packs_epi32(symbol_i1, symbol_i2);
        symbol_34 = _mm_packs_epi32(symbol_i3, symbol_i4);
        symbol_i = _mm_packs_epi16(symbol_12, symbol_34);

        symbol_abs = _mm_abs_epi8(symbol_i);
        symbol_abs = _mm_sub_epi8(offset, symbol_abs);

        result1n = _mm_shuffle_epi8(symbol_i, shuffle_negated_1);
        result1a = _mm_shuffle_epi8(symbol_abs, shuffle_abs_1);

        result2n = _mm_shuffle_epi8(symbol_i, shuffle_negated_2);
        result2a = _mm_shuffle_epi8(symbol_abs, shuffle_abs_2);

        _mm_store_si128(resultPtr, _mm_or_si128(result1n, result1a));
        resultPtr++;
        _mm_store_si128(resultPtr, _mm_or_si128(result2n, result2a));
        resultPtr++;
    }
    // Demodulate last symbols
    for (int i = 8 * (num / 8); i < num; i++) {
        int8_t yre = (int8_t)(SCALE_BYTE_CONV_QAM16 * (vec_in[2 * i]));
        int8_t yim = (int8_t)(SCALE_BYTE_CONV_QAM16 * (vec_in[2 * i + 1]));

        llr[4 * i + 0] = yre;
        llr[4 * i + 1] = yim;
        llr[4 * i + 2] = 2 * SCALE_BYTE_CONV_QAM16 / sqrt(10) - abs(yre);
        llr[4 * i + 3] = 2 * SCALE_BYTE_CONV_QAM16 / sqrt(10) - abs(yim);
    }

    // for (int i = 0; i < ue_num; i++) {
    //     printf("sse: in: %.2f, %.2f, out: %i %i %i %i\n", vec_in[2*i],
    //     vec_in[2*i+1], llr[4*i+0], llr[4*i+1], llr[4*i+2], llr[4*i+3]);
    // }
}

void demod_64qam_soft_loop(float* vec_in, int8_t* llr, int num)
{
    for (int i = 0; i < num; i++) {
        float yre = (int8_t)(SCALE_BYTE_CONV_QAM64 * (vec_in[2 * i]));
        float yim = (int8_t)(SCALE_BYTE_CONV_QAM64 * (vec_in[2 * i + 1]));

        llr[6 * i + 0] = yre;
        llr[6 * i + 1] = yim;
        llr[6 * i + 2] = 4 * SCALE_BYTE_CONV_QAM64 / sqrt(42) - abs(yre);
        llr[6 * i + 3] = 4 * SCALE_BYTE_CONV_QAM64 / sqrt(42) - abs(yim);
        llr[6 * i + 4]
            = 2 * SCALE_BYTE_CONV_QAM64 / sqrt(42) - abs(llr[6 * i + 2]);
        llr[6 * i + 5]
            = 2 * SCALE_BYTE_CONV_QAM64 / sqrt(42) - abs(llr[6 * i + 3]);
    }
}

void demod_64qam_soft_sse(float* vec_in, int8_t* llr, int num)
{
    float* symbolsPtr = (float*)vec_in;
    __m128i* resultPtr = (__m128i*)llr;
    __m128 symbol1, symbol2, symbol3, symbol4;
    __m128i symbol_i1, symbol_i2, symbol_i3, symbol_i4, symbol_i, symbol_abs,
        symbol_abs2, symbol_12, symbol_34;
    __m128i offset1 = _mm_set1_epi8(4 * SCALE_BYTE_CONV_QAM64 / sqrt(42));
    __m128i offset2 = _mm_set1_epi8(2 * SCALE_BYTE_CONV_QAM64 / sqrt(42));
    __m128 scale_v = _mm_set1_ps(SCALE_BYTE_CONV_QAM64);
    __m128i result11, result12, result13, result22, result21, result23,
        result31, result32, result33;

    __m128i shuffle_negated_1 = _mm_set_epi8(0xff, 0xff, 5, 4, 0xff, 0xff, 0xff,
        0xff, 3, 2, 0xff, 0xff, 0xff, 0xff, 1, 0);
    __m128i shuffle_negated_2 = _mm_set_epi8(11, 10, 0xff, 0xff, 0xff, 0xff, 9,
        8, 0xff, 0xff, 0xff, 0xff, 7, 6, 0xff, 0xff);
    __m128i shuffle_negated_3 = _mm_set_epi8(0xff, 0xff, 0xff, 0xff, 15, 14,
        0xff, 0xff, 0xff, 0xff, 13, 12, 0xff, 0xff, 0xff, 0xff);

    __m128i shuffle_abs_1 = _mm_set_epi8(5, 4, 0xff, 0xff, 0xff, 0xff, 3, 2,
        0xff, 0xff, 0xff, 0xff, 1, 0, 0xff, 0xff);
    __m128i shuffle_abs_2 = _mm_set_epi8(0xff, 0xff, 0xff, 0xff, 9, 8, 0xff,
        0xff, 0xff, 0xff, 7, 6, 0xff, 0xff, 0xff, 0xff);
    __m128i shuffle_abs_3 = _mm_set_epi8(0xff, 0xff, 15, 14, 0xff, 0xff, 0xff,
        0xff, 13, 12, 0xff, 0xff, 0xff, 0xff, 11, 10);

    __m128i shuffle_abs2_1 = _mm_set_epi8(0xff, 0xff, 0xff, 0xff, 3, 2, 0xff,
        0xff, 0xff, 0xff, 1, 0, 0xff, 0xff, 0xff, 0xff);
    __m128i shuffle_abs2_2 = _mm_set_epi8(0xff, 0xff, 9, 8, 0xff, 0xff, 0xff,
        0xff, 7, 6, 0xff, 0xff, 0xff, 0xff, 5, 4);
    __m128i shuffle_abs2_3 = _mm_set_epi8(15, 14, 0xff, 0xff, 0xff, 0xff, 13,
        12, 0xff, 0xff, 0xff, 0xff, 11, 10, 0xff, 0xff);

    for (int i = 0; i < num / 8; i++) {
        symbol1 = _mm_load_ps(symbolsPtr);
        symbolsPtr += 4;
        symbol2 = _mm_load_ps(symbolsPtr);
        symbolsPtr += 4;
        symbol3 = _mm_load_ps(symbolsPtr);
        symbolsPtr += 4;
        symbol4 = _mm_load_ps(symbolsPtr);
        symbolsPtr += 4;
        symbol_i1 = _mm_cvtps_epi32(_mm_mul_ps(symbol1, scale_v));
        symbol_i2 = _mm_cvtps_epi32(_mm_mul_ps(symbol2, scale_v));
        symbol_i3 = _mm_cvtps_epi32(_mm_mul_ps(symbol3, scale_v));
        symbol_i4 = _mm_cvtps_epi32(_mm_mul_ps(symbol4, scale_v));
        symbol_12 = _mm_packs_epi32(symbol_i1, symbol_i2);
        symbol_34 = _mm_packs_epi32(symbol_i3, symbol_i4);
        symbol_i = _mm_packs_epi16(symbol_12, symbol_34);

        symbol_abs = _mm_abs_epi8(symbol_i);
        symbol_abs = _mm_sub_epi8(offset1, symbol_abs);
        symbol_abs2 = _mm_sub_epi8(offset2, _mm_abs_epi8(symbol_abs));

        result11 = _mm_shuffle_epi8(symbol_i, shuffle_negated_1);
        result12 = _mm_shuffle_epi8(symbol_abs, shuffle_abs_1);
        result13 = _mm_shuffle_epi8(symbol_abs2, shuffle_abs2_1);

        result21 = _mm_shuffle_epi8(symbol_i, shuffle_negated_2);
        result22 = _mm_shuffle_epi8(symbol_abs, shuffle_abs_2);
        result23 = _mm_shuffle_epi8(symbol_abs2, shuffle_abs2_2);

        result31 = _mm_shuffle_epi8(symbol_i, shuffle_negated_3);
        result32 = _mm_shuffle_epi8(symbol_abs, shuffle_abs_3);
        result33 = _mm_shuffle_epi8(symbol_abs2, shuffle_abs2_3);

        _mm_store_si128(resultPtr,
            _mm_or_si128(_mm_or_si128(result11, result12), result13));
        resultPtr++;
        _mm_store_si128(resultPtr,
            _mm_or_si128(_mm_or_si128(result21, result22), result23));
        resultPtr++;
        _mm_store_si128(resultPtr,
            _mm_or_si128(_mm_or_si128(result31, result32), result33));
        resultPtr++;
    }
    for (int i = 8 * (num / 8); i < num; i++) {
        float yre = (int8_t)(SCALE_BYTE_CONV_QAM64 * (vec_in[2 * i]));
        float yim = (int8_t)(SCALE_BYTE_CONV_QAM64 * (vec_in[2 * i + 1]));

        llr[6 * i + 0] = yre;
        llr[6 * i + 1] = yim;
        llr[6 * i + 2] = 4 * SCALE_BYTE_CONV_QAM64 / sqrt(42) - abs(yre);
        llr[6 * i + 3] = 4 * SCALE_BYTE_CONV_QAM64 / sqrt(42) - abs(yim);
        llr[6 * i + 4]
            = 2 * SCALE_BYTE_CONV_QAM64 / sqrt(42) - abs(llr[6 * i + 2]);
        llr[6 * i + 5]
            = 2 * SCALE_BYTE_CONV_QAM64 / sqrt(42) - abs(llr[6 * i + 3]);
    }
}
