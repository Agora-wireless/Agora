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

#include "modulation.h"

void Demod16qamSoftLoop(const float* vec_in, int8_t* llr, int num) {
  for (int i = 0; i < num; i++) {
    auto yre = static_cast<int8_t>(SCALE_BYTE_CONV_QAM16 * (vec_in[2 * i]));
    auto yim = static_cast<int8_t>(SCALE_BYTE_CONV_QAM16 * (vec_in[2 * i + 1]));

    llr[4 * i + 0] = yre;
    llr[4 * i + 1] = yim;
    llr[4 * i + 2] = 2 * SCALE_BYTE_CONV_QAM16 / sqrt(10) - abs(yre);
    llr[4 * i + 3] = 2 * SCALE_BYTE_CONV_QAM16 / sqrt(10) - abs(yim);
  }
}

void Demod16qamSoftSse(float* vec_in, int8_t* llr, int num) {
  float* symbols_ptr = vec_in;
  auto* result_ptr = reinterpret_cast<__m128i*>(llr);
  __m128 symbol1;
  __m128 symbol2;
  __m128 symbol3;
  __m128 symbol4;
  __m128i symbol_i1;
  __m128i symbol_i2;
  __m128i symbol_i3;
  __m128i symbol_i4;
  __m128i symbol_i;
  __m128i symbol_abs;
  __m128i symbol_12;
  __m128i symbol_34;
  __m128i offset = _mm_set1_epi8(2 * SCALE_BYTE_CONV_QAM16 / sqrt(10));
  __m128i result1n;
  __m128i result1a;
  __m128i result2n;
  __m128i result2a;
  __m128 scale_v = _mm_set1_ps(SCALE_BYTE_CONV_QAM16);

  __m128i shuffle_negated_1 = _mm_set_epi8(0xff, 0xff, 7, 6, 0xff, 0xff, 5, 4,
                                           0xff, 0xff, 3, 2, 0xff, 0xff, 1, 0);
  __m128i shuffle_abs_1 = _mm_set_epi8(7, 6, 0xff, 0xff, 5, 4, 0xff, 0xff, 3, 2,
                                       0xff, 0xff, 1, 0, 0xff, 0xff);

  __m128i shuffle_negated_2 =
      _mm_set_epi8(0xff, 0xff, 15, 14, 0xff, 0xff, 13, 12, 0xff, 0xff, 11, 10,
                   0xff, 0xff, 9, 8);
  __m128i shuffle_abs_2 = _mm_set_epi8(15, 14, 0xff, 0xff, 13, 12, 0xff, 0xff,
                                       11, 10, 0xff, 0xff, 9, 8, 0xff, 0xff);

  for (int i = 0; i < num / 8; i++) {
    symbol1 = _mm_load_ps(symbols_ptr);
    symbols_ptr += 4;
    symbol2 = _mm_load_ps(symbols_ptr);
    symbols_ptr += 4;
    symbol3 = _mm_load_ps(symbols_ptr);
    symbols_ptr += 4;
    symbol4 = _mm_load_ps(symbols_ptr);
    symbols_ptr += 4;
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

    _mm_store_si128(result_ptr, _mm_or_si128(result1n, result1a));
    result_ptr++;
    _mm_store_si128(result_ptr, _mm_or_si128(result2n, result2a));
    result_ptr++;
  }
  // Demodulate last symbols
  for (int i = 8 * (num / 8); i < num; i++) {
    auto yre = static_cast<int8_t>(SCALE_BYTE_CONV_QAM16 * (vec_in[2 * i]));
    auto yim = static_cast<int8_t>(SCALE_BYTE_CONV_QAM16 * (vec_in[2 * i + 1]));

    llr[4 * i + 0] = yre;
    llr[4 * i + 1] = yim;
    llr[4 * i + 2] = 2 * SCALE_BYTE_CONV_QAM16 / sqrt(10) - abs(yre);
    llr[4 * i + 3] = 2 * SCALE_BYTE_CONV_QAM16 / sqrt(10) - abs(yim);
  }

  // for (int i = 0; i < ue_num; i++) {
  //     std::printf("sse: in: %.2f, %.2f, out: %i %i %i %i\n", vec_in[2*i],
  //     vec_in[2*i+1], llr[4*i+0], llr[4*i+1], llr[4*i+2], llr[4*i+3]);
  // }
}

void Demod64qamSoftLoop(const float* vec_in, int8_t* llr, int num) {
  for (int i = 0; i < num; i++) {
    float yre = (int8_t)(SCALE_BYTE_CONV_QAM64 * (vec_in[2 * i]));
    float yim = (int8_t)(SCALE_BYTE_CONV_QAM64 * (vec_in[2 * i + 1]));

    llr[6 * i + 0] = yre;
    llr[6 * i + 1] = yim;
    llr[6 * i + 2] = 4 * SCALE_BYTE_CONV_QAM64 / sqrt(42) - abs(yre);
    llr[6 * i + 3] = 4 * SCALE_BYTE_CONV_QAM64 / sqrt(42) - abs(yim);
    llr[6 * i + 4] = 2 * SCALE_BYTE_CONV_QAM64 / sqrt(42) - abs(llr[6 * i + 2]);
    llr[6 * i + 5] = 2 * SCALE_BYTE_CONV_QAM64 / sqrt(42) - abs(llr[6 * i + 3]);
  }
}

void Demod64qamSoftSse(float* vec_in, int8_t* llr, int num) {
  auto* symbols_ptr = static_cast<float*>(vec_in);
  auto* result_ptr = reinterpret_cast<__m128i*>(llr);
  __m128 symbol1;
  __m128 symbol2;
  __m128 symbol3;
  __m128 symbol4;
  __m128i symbol_i1;
  __m128i symbol_i2;
  __m128i symbol_i3;
  __m128i symbol_i4;
  __m128i symbol_i;
  __m128i symbol_abs;
  __m128i symbol_abs2;
  __m128i symbol_12;
  __m128i symbol_34;
  __m128i offset1 = _mm_set1_epi8(4 * SCALE_BYTE_CONV_QAM64 / sqrt(42));
  __m128i offset2 = _mm_set1_epi8(2 * SCALE_BYTE_CONV_QAM64 / sqrt(42));
  __m128 scale_v = _mm_set1_ps(SCALE_BYTE_CONV_QAM64);
  __m128i result11;
  __m128i result12;
  __m128i result13;
  __m128i result22;
  __m128i result21;
  __m128i result23;
  __m128i result31;
  __m128i result32;
  __m128i result33;

  __m128i shuffle_negated_1 =
      _mm_set_epi8(0xff, 0xff, 5, 4, 0xff, 0xff, 0xff, 0xff, 3, 2, 0xff, 0xff,
                   0xff, 0xff, 1, 0);
  __m128i shuffle_negated_2 =
      _mm_set_epi8(11, 10, 0xff, 0xff, 0xff, 0xff, 9, 8, 0xff, 0xff, 0xff, 0xff,
                   7, 6, 0xff, 0xff);
  __m128i shuffle_negated_3 =
      _mm_set_epi8(0xff, 0xff, 0xff, 0xff, 15, 14, 0xff, 0xff, 0xff, 0xff, 13,
                   12, 0xff, 0xff, 0xff, 0xff);

  __m128i shuffle_abs_1 = _mm_set_epi8(5, 4, 0xff, 0xff, 0xff, 0xff, 3, 2, 0xff,
                                       0xff, 0xff, 0xff, 1, 0, 0xff, 0xff);
  __m128i shuffle_abs_2 =
      _mm_set_epi8(0xff, 0xff, 0xff, 0xff, 9, 8, 0xff, 0xff, 0xff, 0xff, 7, 6,
                   0xff, 0xff, 0xff, 0xff);
  __m128i shuffle_abs_3 =
      _mm_set_epi8(0xff, 0xff, 15, 14, 0xff, 0xff, 0xff, 0xff, 13, 12, 0xff,
                   0xff, 0xff, 0xff, 11, 10);

  __m128i shuffle_abs2_1 =
      _mm_set_epi8(0xff, 0xff, 0xff, 0xff, 3, 2, 0xff, 0xff, 0xff, 0xff, 1, 0,
                   0xff, 0xff, 0xff, 0xff);
  __m128i shuffle_abs2_2 =
      _mm_set_epi8(0xff, 0xff, 9, 8, 0xff, 0xff, 0xff, 0xff, 7, 6, 0xff, 0xff,
                   0xff, 0xff, 5, 4);
  __m128i shuffle_abs2_3 =
      _mm_set_epi8(15, 14, 0xff, 0xff, 0xff, 0xff, 13, 12, 0xff, 0xff, 0xff,
                   0xff, 11, 10, 0xff, 0xff);

  for (int i = 0; i < num / 8; i++) {
    symbol1 = _mm_load_ps(symbols_ptr);
    symbols_ptr += 4;
    symbol2 = _mm_load_ps(symbols_ptr);
    symbols_ptr += 4;
    symbol3 = _mm_load_ps(symbols_ptr);
    symbols_ptr += 4;
    symbol4 = _mm_load_ps(symbols_ptr);
    symbols_ptr += 4;
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

    _mm_store_si128(result_ptr,
                    _mm_or_si128(_mm_or_si128(result11, result12), result13));
    result_ptr++;
    _mm_store_si128(result_ptr,
                    _mm_or_si128(_mm_or_si128(result21, result22), result23));
    result_ptr++;
    _mm_store_si128(result_ptr,
                    _mm_or_si128(_mm_or_si128(result31, result32), result33));
    result_ptr++;
  }
  for (int i = 8 * (num / 8); i < num; i++) {
    float yre = (int8_t)(SCALE_BYTE_CONV_QAM64 * (vec_in[2 * i]));
    float yim = (int8_t)(SCALE_BYTE_CONV_QAM64 * (vec_in[2 * i + 1]));

    llr[6 * i + 0] = yre;
    llr[6 * i + 1] = yim;
    llr[6 * i + 2] = 4 * SCALE_BYTE_CONV_QAM64 / sqrt(42) - abs(yre);
    llr[6 * i + 3] = 4 * SCALE_BYTE_CONV_QAM64 / sqrt(42) - abs(yim);
    llr[6 * i + 4] = 2 * SCALE_BYTE_CONV_QAM64 / sqrt(42) - abs(llr[6 * i + 2]);
    llr[6 * i + 5] = 2 * SCALE_BYTE_CONV_QAM64 / sqrt(42) - abs(llr[6 * i + 3]);
  }
}

void DemodQpskSoftSse(float* x, int8_t* z, int len) {
  int i = 0;

  // Force the use of SSE here instead of AVX since the implementations requires
  // too many permutes across 128-bit boundaries

  __m128 s = _mm_set1_ps(-SCALE_BYTE_CONV_QPSK * M_SQRT2);
  if (((size_t)(x)&0x0F) == 0 && ((size_t)(z)&0x0F) == 0) {
    for (; i < len - 16 + 1; i += 16) {
      __m128 a = _mm_load_ps(&x[i]);
      __m128 b = _mm_load_ps(&x[i + 1 * 4]);
      __m128 c = _mm_load_ps(&x[i + 2 * 4]);
      __m128 d = _mm_load_ps(&x[i + 3 * 4]);

      __m128 sa = _mm_mul_ps(a, s);
      __m128 sb = _mm_mul_ps(b, s);
      __m128 sc = _mm_mul_ps(c, s);
      __m128 sd = _mm_mul_ps(d, s);

      __m128i ai = _mm_cvttps_epi32(sa);
      __m128i bi = _mm_cvttps_epi32(sb);
      __m128i ci = _mm_cvttps_epi32(sc);
      __m128i di = _mm_cvttps_epi32(sd);
      __m128i ab = _mm_packs_epi32(ai, bi);
      __m128i cd = _mm_packs_epi32(ci, di);

      __m128i i8 = _mm_packs_epi16(ab, cd);

      _mm_store_si128((__m128i*)&z[i], i8);
    }
  } else {
    for (; i < len - 16 + 1; i += 16) {
      __m128 a = _mm_load_ps(&x[i]);
      __m128 b = _mm_load_ps(&x[i + 1 * 4]);
      __m128 c = _mm_load_ps(&x[i + 2 * 4]);
      __m128 d = _mm_load_ps(&x[i + 3 * 4]);

      __m128 sa = _mm_mul_ps(a, s);
      __m128 sb = _mm_mul_ps(b, s);
      __m128 sc = _mm_mul_ps(c, s);
      __m128 sd = _mm_mul_ps(d, s);

      __m128i ai = _mm_cvttps_epi32(sa);
      __m128i bi = _mm_cvttps_epi32(sb);
      __m128i ci = _mm_cvttps_epi32(sc);
      __m128i di = _mm_cvttps_epi32(sd);
      __m128i ab = _mm_packs_epi32(ai, bi);
      __m128i cd = _mm_packs_epi32(ci, di);

      __m128i i8 = _mm_packs_epi16(ab, cd);

      _mm_storeu_si128((__m128i*)&z[i], i8);
    }
  }

  for (; i < len; i++) {
    z[i] = (int8_t)(x[i] * -SCALE_BYTE_CONV_QPSK * M_SQRT2);
  }
}
