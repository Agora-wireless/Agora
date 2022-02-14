/**
 * @file datatype-conversion.inc
 * @brief SIMD functions for converting between data types
 */

#ifndef DATATYPE_CONVERSION_INC_
#define DATATYPE_CONVERSION_INC_

#include <emmintrin.h>
#include <immintrin.h>

#include <bitset>

#include "utils.h"

//#define DATATYPE_MEMORY_CHECK

// Convert a short array [in_buf] to a float array [out_buf]. Each array must
// have [n_elems] elements.
// in_buf and out_buf must be 64-byte aligned
// n_elems must be a multiple of 16
// reference:
// https://stackoverflow.com/questions/50597764/convert-signed-short-to-float-in-c-simd
static inline void SimdConvertShortToFloat(const short* in_buf, float* out_buf,
                                           size_t n_elems) {
#if defined(DATATYPE_MEMORY_CHECK)
  RtAssert(((n_elems % 16) == 0) &&
               ((reinterpret_cast<size_t>(in_buf) % 64) == 0) &&
               ((reinterpret_cast<size_t>(out_buf) % 64) == 0),
           "Data Alignment not correct before calling into AVX optimizations");
#endif

#if defined(__AVX512F__)
  const __m512 magic = _mm512_set1_ps(float((1 << 23) + (1 << 15)) / 32768.f);
  const __m512i magic_i = _mm512_castps_si512(magic);
  for (size_t i = 0; i < n_elems; i += 16) {
    /* get input */
    __m256i val = _mm256_load_si256((__m256i*)(in_buf + i));  // port 2,3
    /* interleave with 0x0000 */
    __m512i val_unpacked = _mm512_cvtepu16_epi32(val);  // port 5
    /* convert by xor-ing and subtracting magic value:
     * VPXOR avoids port5 bottlenecks on Intel CPUs before SKL */
    __m512i val_f_int = _mm512_xor_si512(val_unpacked, magic_i);  // port 0,1,5
    __m512 val_f = _mm512_castsi512_ps(val_f_int);   // no instruction
    __m512 converted = _mm512_sub_ps(val_f, magic);  // port 1,5 ?
    _mm512_store_ps(out_buf + i, converted);         // port 2,3,4,7
  }
#else
  const __m256 magic = _mm256_set1_ps(float((1 << 23) + (1 << 15)) / 32768.f);
  const __m256i magic_i = _mm256_castps_si256(magic);
  for (size_t i = 0; i < n_elems; i += 16) {
    /* get input */
    __m128i val = _mm_load_si128((__m128i*)(in_buf + i));  // port 2,3

    __m128i val1 = _mm_load_si128((__m128i*)(in_buf + i + 8));
    /* interleave with 0x0000 */
    __m256i val_unpacked = _mm256_cvtepu16_epi32(val);  // port 5
    /* convert by xor-ing and subtracting magic value:
     * VPXOR avoids port5 bottlenecks on Intel CPUs before SKL */
    __m256i val_f_int = _mm256_xor_si256(val_unpacked, magic_i);  // port 0,1,5
    __m256 val_f = _mm256_castsi256_ps(val_f_int);   // no instruction
    __m256 converted = _mm256_sub_ps(val_f, magic);  // port 1,5 ?
    _mm256_store_ps(out_buf + i, converted);         // port 2,3,4,7

    __m256i val_unpacked1 = _mm256_cvtepu16_epi32(val1);  // port 5
    __m256i val_f_int1 =
        _mm256_xor_si256(val_unpacked1, magic_i);      // port 0,1,5
    __m256 val_f1 = _mm256_castsi256_ps(val_f_int1);   // no instruction
    __m256 converted1 = _mm256_sub_ps(val_f1, magic);  // port 1,5 ?
    _mm256_store_ps(out_buf + i + 8, converted1);      // port 2,3,4,7
  }
#endif
}

// Convert a float array [in_buf] to a short array [out_buf]. Input array must
// have [n_elems] elements. Output array must have [n_elems + cp_len] elements.
// in_buf and out_buf must be 64-byte aligned
// n_elems must be a multiple of 8 for AVX2 and 16 for AVX512
// scale_down_factor is used for scaling down values in the input array
static inline void SimdConvertFloatToShort(const float* in_buf, short* out_buf,
                                           size_t n_elems, size_t cp_len,
                                           size_t scale_down_factor) {
#if defined(DATATYPE_MEMORY_CHECK)
  RtAssert(((n_elems % 16) == 0) &&
               ((reinterpret_cast<size_t>(in_buf) % 64) == 0) &&
               ((reinterpret_cast<size_t>(out_buf) % 64) == 0),
           "Data Alignment not correct before calling into AVX optimizations");
#endif

  const float scale_factor_float = 32768.0 / scale_down_factor;

#ifdef __AVX512F__
  const __m512 scale_factor = _mm512_set1_ps(scale_factor_float);
  const __m512i permute_index = _mm512_setr_epi64(0, 2, 4, 6, 1, 3, 5, 7);
  for (size_t i = 0; i < n_elems; i += 16) {
    __m512 in1 = _mm512_load_ps(in_buf + 2 * i);
    __m512 in2 = _mm512_load_ps(in_buf + 2 * i + 16);
    __m512 scaled_in1 = _mm512_mul_ps(in1, scale_factor);
    __m512 scaled_in2 = _mm512_mul_ps(in2, scale_factor);
    __m512i integer1 = _mm512_cvtps_epi32(scaled_in1);
    __m512i integer2 = _mm512_cvtps_epi32(scaled_in2);
    integer1 = _mm512_packs_epi32(integer1, integer2);
    integer1 = _mm512_permutexvar_epi64(permute_index, integer1);
    _mm512_stream_si512((__m512i*)&out_buf[2 * (i + cp_len)], integer1);
    // Set cyclic prefix
    if (i >= n_elems - cp_len) {
      _mm512_stream_si512((__m512i*)&out_buf[2 * (i + cp_len - n_elems)],
                          integer1);
    }
  }
#else
  const __m256 scale_factor = _mm256_set1_ps(scale_factor_float);
  for (size_t i = 0; i < n_elems; i += 8) {
    __m256 in1 = _mm256_load_ps(in_buf + 2 * i);
    __m256 in2 = _mm256_load_ps(in_buf + 2 * i + 8);
    __m256 scaled_in1 = _mm256_mul_ps(in1, scale_factor);
    __m256 scaled_in2 = _mm256_mul_ps(in2, scale_factor);
    __m256i integer1 = _mm256_cvtps_epi32(scaled_in1);
    __m256i integer2 = _mm256_cvtps_epi32(scaled_in2);
    integer1 = _mm256_packs_epi32(integer1, integer2);
    integer1 = _mm256_permute4x64_epi64(integer1, 0xD8);
    _mm256_stream_si256((__m256i*)&out_buf[2 * (i + cp_len)], integer1);
    // Set cyclic prefix
    if (i >= n_elems - cp_len) {
      _mm256_stream_si256((__m256i*)&out_buf[2 * (i + cp_len - n_elems)],
                          integer1);
    }
  }
#endif
}

// Convert a float IQ array [in_buf] to an uint8_t IQ array [out_buf].
// Each float is converted to 12-bit data (2 floats corresponds to 3 uint8_t).
// Input array must have [n_elems] elements.
// Output array must have [n_elems / 2 * 3] elements.
// n_elems must be multiples of 2
static inline void ConvertFloatTo12bitIq(const float* in_buf, uint8_t* out_buf,
                                         size_t n_elems) {
#if defined(DATATYPE_MEMORY_CHECK)
  RtAssert(((n_elems % 2) == 0) &&
           "ConvertFloatTo12bitIq n_elems not multiple of 2");
#endif
  size_t index_short = 0;
  for (size_t i = 0; i < n_elems; i = i + 2) {
    auto temp_i = static_cast<unsigned short>(in_buf[i] * 32768 * 4);
    auto temp_q = static_cast<unsigned short>(in_buf[i + 1] * 32768 * 4);
    // Take the higher 12 bits and ignore the lower 4 bits
    out_buf[index_short] = (uint8_t)(temp_i >> 4);
    out_buf[index_short + 1] =
        ((uint8_t)(temp_i >> 12)) | ((uint8_t)(temp_q & 0xf0));
    out_buf[index_short + 2] = (uint8_t)(temp_q >> 8);
    if (kDebug12BitIQ) {
      std::cout << "i: " << i << " " << std::bitset<16>(temp_i) << " "
                << std::bitset<16>(temp_q) << " => "
                << std::bitset<8>(out_buf[index_short]) << " "
                << std::bitset<8>(out_buf[index_short + 1]) << " "
                << std::bitset<8>(out_buf[index_short + 2]) << std::endl;
      std::printf("Original: %.4f+%.4fi \n", in_buf[i], in_buf[i + 1]);
    }
    index_short += 3;
  }
}

#ifdef __AVX512F__
static inline void SimdConvert16bitIqToFloat(__m256i val, float* out_buf,
                                             __m512 magic, __m512i magic_i) {
  /* interleave with 0x0000 */
  __m512i val_unpacked = _mm512_cvtepu16_epi32(val);  // port 5
  /* convert by xor-ing and subtracting magic value:
   * VPXOR avoids port5 bottlenecks on Intel CPUs before SKL */
  __m512i val_f_int = _mm512_xor_si512(val_unpacked, magic_i);  // port 0,1,5
  __m512 val_f = _mm512_castsi512_ps(val_f_int);   // no instruction
  __m512 converted = _mm512_sub_ps(val_f, magic);  // port 1,5 ?
  _mm512_store_ps(out_buf, converted);             // port 2,3,4,7
}
#endif

static inline void Convert12bitIqTo16bitIq(uint8_t* in_buf, uint16_t* out_buf,
                                           size_t n_elems) {
#if defined(DATATYPE_MEMORY_CHECK)
  RtAssert(((n_elems % 16) == 0) &&
               ((reinterpret_cast<size_t>(in_buf) % 32) == 0) &&
               ((reinterpret_cast<size_t>(out_buf) % 32) == 0),
           "Convert12bitIqTo16bitIq: Data Alignment not correct before calling "
           "into AVX optimizations");
#endif
  for (size_t i = 0; i < n_elems; i += 16) {
    _mm256_loadu_si256((__m256i const*)in_buf);
    _mm256_loadu_si256((__m256i const*)(in_buf + 16));
    __m256i temp_i =
        _mm256_setr_epi16(*(uint16_t*)in_buf, *(uint16_t*)(in_buf + 3),
                          *(uint16_t*)(in_buf + 6), *(uint16_t*)(in_buf + 9),
                          *(uint16_t*)(in_buf + 12), *(uint16_t*)(in_buf + 15),
                          *(uint16_t*)(in_buf + 18), *(uint16_t*)(in_buf + 21),
                          *(uint16_t*)(in_buf + 24), *(uint16_t*)(in_buf + 27),
                          *(uint16_t*)(in_buf + 30), *(uint16_t*)(in_buf + 33),
                          *(uint16_t*)(in_buf + 36), *(uint16_t*)(in_buf + 39),
                          *(uint16_t*)(in_buf + 42), *(uint16_t*)(in_buf + 45));

    __m256i mask_q = _mm256_set1_epi16(0xfff0);
    __m256i temp_q =
        _mm256_setr_epi16(*(uint16_t*)(in_buf + 1), *(uint16_t*)(in_buf + 4),
                          *(uint16_t*)(in_buf + 7), *(uint16_t*)(in_buf + 10),
                          *(uint16_t*)(in_buf + 13), *(uint16_t*)(in_buf + 16),
                          *(uint16_t*)(in_buf + 19), *(uint16_t*)(in_buf + 22),
                          *(uint16_t*)(in_buf + 25), *(uint16_t*)(in_buf + 28),
                          *(uint16_t*)(in_buf + 31), *(uint16_t*)(in_buf + 34),
                          *(uint16_t*)(in_buf + 37), *(uint16_t*)(in_buf + 40),
                          *(uint16_t*)(in_buf + 43), *(uint16_t*)(in_buf + 46));

    temp_q = _mm256_and_si256(temp_q, mask_q);  // Set lower 4 bits to 0
    temp_i = _mm256_slli_epi16(temp_i, 4);      // Shift left by 4 bits

    __m256i iq_0 = _mm256_unpacklo_epi16(temp_i, temp_q);
    __m256i iq_1 = _mm256_unpackhi_epi16(temp_i, temp_q);
    __m256i output_0 = _mm256_permute2f128_si256(iq_0, iq_1, 0x20);
    __m256i output_1 = _mm256_permute2f128_si256(iq_0, iq_1, 0x31);
    _mm256_store_si256((__m256i*)(out_buf + i * 2), output_0);
    _mm256_store_si256((__m256i*)(out_buf + i * 2 + 16), output_1);
  }

  // for (size_t i = 0; i < n_elems; i++) {
  //     // out_buf[i * 2]
  //     //     = (((uint16_t)in_buf[i * 3]) << 8) | (in_buf[i * 3 + 1] & 0xf0);
  //     // out_buf[i * 2 + 1] = (uint16_t)in_buf[i * 3 + 2] << 4
  //     //     | ((uint16_t)(in_buf[i * 3 + 1] & 0xf) << 12);
  //     // if (kDebug12BitIQ) {
  //     std::cout << "i: " << i << " " << std::bitset<8>(in_buf[i * 3]) << " "
  //               << std::bitset<8>(in_buf[i * 3 + 1]) << " "
  //               << std::bitset<8>(in_buf[i * 3 + 2]) << "=>"
  //               << std::bitset<16>(out_buf[i * 2]) << " "
  //               << std::bitset<16>(out_buf[i * 2 + 1]) << std::endl;
  //     // }
  // }
}

// Convert an uint8_t IQ array [in_buf] to a float IQ array [out_buf].
// Each 12-bit I/Q is converted to a float (3 uint8_t corresponds to 2 floats).
// Input array must have [n_elems] elements.
// Output array must have [n_elems / 3 * 2] elements.
// n_elems must be multiples of 3
static inline void SimdConvert12bitIqToFloat(const uint8_t* in_buf,
                                             float* out_buf,
                                             const uint16_t* in_16bits_buf,
                                             size_t n_elems) {
  unused(in_16bits_buf);
#ifdef __AVX512F__
  const __m512 magic = _mm512_set1_ps(float((1 << 23) + (1 << 15)) / 131072.f);
  const __m512i magic_i = _mm512_castps_si512(magic);
#else
  const __m256 magic = _mm256_set1_ps(float((1 << 23) + (1 << 15)) / 131072.f);
  const __m256i magic_i = _mm256_castps_si256(magic);
#endif
#ifdef __AVX512F__
  for (size_t i = 0; i < n_elems / 3; i += 32) {
    __m512i temp_i =
        _mm512_set_epi16(*(uint16_t*)(in_buf + 93), *(uint16_t*)(in_buf + 90),
                         *(uint16_t*)(in_buf + 87), *(uint16_t*)(in_buf + 84),
                         *(uint16_t*)(in_buf + 81), *(uint16_t*)(in_buf + 78),
                         *(uint16_t*)(in_buf + 75), *(uint16_t*)(in_buf + 72),
                         *(uint16_t*)(in_buf + 69), *(uint16_t*)(in_buf + 66),
                         *(uint16_t*)(in_buf + 63), *(uint16_t*)(in_buf + 60),
                         *(uint16_t*)(in_buf + 57), *(uint16_t*)(in_buf + 54),
                         *(uint16_t*)(in_buf + 51), *(uint16_t*)(in_buf + 48),
                         *(uint16_t*)(in_buf + 45), *(uint16_t*)(in_buf + 42),
                         *(uint16_t*)(in_buf + 39), *(uint16_t*)(in_buf + 36),
                         *(uint16_t*)(in_buf + 33), *(uint16_t*)(in_buf + 30),
                         *(uint16_t*)(in_buf + 27), *(uint16_t*)(in_buf + 24),
                         *(uint16_t*)(in_buf + 21), *(uint16_t*)(in_buf + 18),
                         *(uint16_t*)(in_buf + 15), *(uint16_t*)(in_buf + 12),
                         *(uint16_t*)(in_buf + 9), *(uint16_t*)(in_buf + 6),
                         *(uint16_t*)(in_buf + 3), *(uint16_t*)(in_buf + 0));

    __m512i mask_q = _mm512_set1_epi16(0xfff0);
    __m512i temp_q =
        _mm512_set_epi16(*(uint16_t*)(in_buf + 94), *(uint16_t*)(in_buf + 91),
                         *(uint16_t*)(in_buf + 88), *(uint16_t*)(in_buf + 85),
                         *(uint16_t*)(in_buf + 82), *(uint16_t*)(in_buf + 79),
                         *(uint16_t*)(in_buf + 76), *(uint16_t*)(in_buf + 73),
                         *(uint16_t*)(in_buf + 70), *(uint16_t*)(in_buf + 67),
                         *(uint16_t*)(in_buf + 64), *(uint16_t*)(in_buf + 61),
                         *(uint16_t*)(in_buf + 58), *(uint16_t*)(in_buf + 55),
                         *(uint16_t*)(in_buf + 52), *(uint16_t*)(in_buf + 49),
                         *(uint16_t*)(in_buf + 46), *(uint16_t*)(in_buf + 43),
                         *(uint16_t*)(in_buf + 40), *(uint16_t*)(in_buf + 37),
                         *(uint16_t*)(in_buf + 34), *(uint16_t*)(in_buf + 31),
                         *(uint16_t*)(in_buf + 28), *(uint16_t*)(in_buf + 25),
                         *(uint16_t*)(in_buf + 22), *(uint16_t*)(in_buf + 19),
                         *(uint16_t*)(in_buf + 16), *(uint16_t*)(in_buf + 13),
                         *(uint16_t*)(in_buf + 10), *(uint16_t*)(in_buf + 7),
                         *(uint16_t*)(in_buf + 4), *(uint16_t*)(in_buf + 1));

    temp_q = _mm512_and_si512(temp_q, mask_q);  // Set lower 4 bits to 0
    temp_i = _mm512_slli_epi16(temp_i, 4);      // Shift left by 4 bits

    __m512i iq_0 = _mm512_unpacklo_epi16(temp_i, temp_q);
    __m512i iq_1 = _mm512_unpackhi_epi16(temp_i, temp_q);
    __m512i output_0 = _mm512_permutex2var_epi64(
        iq_0, _mm512_set_epi64(11, 10, 3, 2, 9, 8, 1, 0), iq_1);
    __m512i output_1 = _mm512_permutex2var_epi64(
        iq_0, _mm512_set_epi64(15, 14, 7, 6, 13, 12, 5, 4), iq_1);

    SimdConvert16bitIqToFloat(_mm512_extracti64x4_epi64(output_0, 0),
                              out_buf + i * 2, magic, magic_i);
    SimdConvert16bitIqToFloat(_mm512_extracti64x4_epi64(output_0, 1),
                              out_buf + i * 2 + 16, magic, magic_i);
    SimdConvert16bitIqToFloat(_mm512_extracti64x4_epi64(output_1, 0),
                              out_buf + i * 2 + 32, magic, magic_i);
    SimdConvert16bitIqToFloat(_mm512_extracti64x4_epi64(output_1, 1),
                              out_buf + i * 2 + 48, magic, magic_i);
    in_buf += 96;
  }

#else
  for (size_t i = 0; i < n_elems / 3; i += 16) {
    // Convert 16 IQ smaples from 48 uint8_t to 32 shorts
    // convert_12bit_iq_to_16bit_iq(in_buf + i * 3, in_16bits_buf, 16);
    __m256i temp_i =
        _mm256_setr_epi16(*(uint16_t*)in_buf, *(uint16_t*)(in_buf + 3),
                          *(uint16_t*)(in_buf + 6), *(uint16_t*)(in_buf + 9),
                          *(uint16_t*)(in_buf + 12), *(uint16_t*)(in_buf + 15),
                          *(uint16_t*)(in_buf + 18), *(uint16_t*)(in_buf + 21),
                          *(uint16_t*)(in_buf + 24), *(uint16_t*)(in_buf + 27),
                          *(uint16_t*)(in_buf + 30), *(uint16_t*)(in_buf + 33),
                          *(uint16_t*)(in_buf + 36), *(uint16_t*)(in_buf + 39),
                          *(uint16_t*)(in_buf + 42), *(uint16_t*)(in_buf + 45));

    __m256i mask_q = _mm256_set1_epi16(0xfff0);
    __m256i temp_q =
        _mm256_setr_epi16(*(uint16_t*)(in_buf + 1), *(uint16_t*)(in_buf + 4),
                          *(uint16_t*)(in_buf + 7), *(uint16_t*)(in_buf + 10),
                          *(uint16_t*)(in_buf + 13), *(uint16_t*)(in_buf + 16),
                          *(uint16_t*)(in_buf + 19), *(uint16_t*)(in_buf + 22),
                          *(uint16_t*)(in_buf + 25), *(uint16_t*)(in_buf + 28),
                          *(uint16_t*)(in_buf + 31), *(uint16_t*)(in_buf + 34),
                          *(uint16_t*)(in_buf + 37), *(uint16_t*)(in_buf + 40),
                          *(uint16_t*)(in_buf + 43), *(uint16_t*)(in_buf + 46));

    temp_q = _mm256_and_si256(temp_q, mask_q);  // Set lower 4 bits to 0
    temp_i = _mm256_slli_epi16(temp_i, 4);      // Shift left by 4 bits

    __m256i iq_0 = _mm256_unpacklo_epi16(temp_i, temp_q);
    __m256i iq_1 = _mm256_unpackhi_epi16(temp_i, temp_q);
    __m256i output_0 = _mm256_permute2f128_si256(iq_0, iq_1, 0x20);
    __m256i output_1 = _mm256_permute2f128_si256(iq_0, iq_1, 0x31);

    _mm256_store_si256((__m256i*)(in_16bits_buf), output_0);
    _mm256_store_si256((__m256i*)(in_16bits_buf + 16), output_1);

    // Conver short to float
    for (size_t j = 0; j < 2; j++) {
      /* get input */
      __m128i val =
          _mm_load_si128((__m128i*)(in_16bits_buf + j * 16));  // port 2,3

      __m128i val1 = _mm_load_si128((__m128i*)(in_16bits_buf + j * 16 + 8));
      /* interleave with 0x0000 */
      __m256i val_unpacked = _mm256_cvtepu16_epi32(val);  // port 5
      /* convert by xor-ing and subtracting magic value:
       * VPXOR avoids port5 bottlenecks on Intel CPUs before SKL */
      __m256i val_f_int =
          _mm256_xor_si256(val_unpacked, magic_i);           // port 0,1,5
      __m256 val_f = _mm256_castsi256_ps(val_f_int);         // no instruction
      __m256 converted = _mm256_sub_ps(val_f, magic);        // port 1,5 ?
      _mm256_store_ps(out_buf + i * 2 + j * 16, converted);  // port 2,3,4,7

      __m256i val_unpacked1 = _mm256_cvtepu16_epi32(val1);  // port 5
      __m256i val_f_int1 =
          _mm256_xor_si256(val_unpacked1, magic_i);      // port 0,1,5
      __m256 val_f1 = _mm256_castsi256_ps(val_f_int1);   // no instruction
      __m256 converted1 = _mm256_sub_ps(val_f1, magic);  // port 1,5 ?
      _mm256_store_ps(out_buf + i * 2 + j * 16 + 8,
                      converted1);  // port 2,3,4,7
    }
    in_buf += 48;
  }
#endif
}

// Convert a float16 array [in_buf] to a float32 array [out_buf]. Each array
// must have [n_elems] elements
// in_buf and out_buf must be 64-byte aligned
// n_elems must be a multiple of 16
static inline void SimdConvertFloat16ToFloat32(float* out_buf,
                                               const float* in_buf,
                                               size_t n_elems) {
#if defined(DATATYPE_MEMORY_CHECK)
  RtAssert(
      ((n_elems % 16) == 0) && ((reinterpret_cast<size_t>(in_buf) % 32) == 0) &&
          ((reinterpret_cast<size_t>(out_buf) % 32) == 0),
      "SimdConvertFloat16ToFloat32: Data Alignment not correct before calling "
      "into AVX optimizations");
#endif
#ifdef __AVX512F__
  for (size_t i = 0; i < n_elems; i += 16) {
    __m256i val_a = _mm256_load_si256((__m256i*)(in_buf + i / 2));
    __m512 val = _mm512_cvtph_ps(val_a);
    _mm512_store_ps(out_buf + i, val);
  }
#else
  for (size_t i = 0; i < n_elems; i += 8) {
    __m128i val_a = _mm_load_si128((__m128i*)(in_buf + i / 2));
    __m256 val = _mm256_cvtph_ps(val_a);
    _mm256_store_ps(out_buf + i, val);
  }
#endif
}

// Convert a float32 array [in_buf] to a float16 array [out_buf]. Each array
// must have [n_elems] elements
// in_buf and out_buf must be 64-byte aligned
// n_elems must be a multiple of 16
static inline void SimdConvertFloat32ToFloat16(float* out_buf,
                                               const float* in_buf,
                                               size_t n_elems) {
#if defined(DATATYPE_MEMORY_CHECK)
  RtAssert(
      ((n_elems % 16) == 0) && ((reinterpret_cast<size_t>(in_buf) % 32) == 0) &&
          ((reinterpret_cast<size_t>(out_buf) % 32) == 0),
      "SimdConvertFloat32ToFloat16: Data Alignment not correct before calling "
      "into AVX optimizations");
#endif

#ifdef __AVX512F__
  for (size_t i = 0; i < n_elems; i += 16) {
    __m512 val_a = _mm512_load_ps(in_buf + i);
    __m256i val = _mm512_cvtps_ph(val_a, _MM_FROUND_NO_EXC);
    _mm256_store_si256(reinterpret_cast<__m256i*>(out_buf + i / 2), val);
  }
#else
  for (size_t i = 0; i < n_elems; i += 8) {
    __m256 val_a = _mm256_load_ps(in_buf + i);
    __m128i val = _mm256_cvtps_ph(val_a, _MM_FROUND_NO_EXC);
    _mm_store_si128(reinterpret_cast<__m128i*>(out_buf + i / 2), val);
  }
#endif
}

#endif  // DATATYPE_CONVERSION_INC_