/**
 * @file datatype-conversion.h
 * @brief SIMD functions for converting between data types
 */

#ifndef DATATYPE_CONVERSION
#define DATATYPE_CONVERSION

#include <emmintrin.h>
#include <immintrin.h>

#include <bitset>

#include "utils.h"

// Convert a short array [in_buf] to a float array [out_buf]. Each array must
// have [n_elems] elements.
// in_buf and out_buf must be 64-byte aligned
// n_elems must be a multiple of 16
// reference:
// https://stackoverflow.com/questions/50597764/convert-signed-short-to-float-in-c-simd
static inline void simd_convert_short_to_float(const short* in_buf,
                                               float* out_buf, size_t n_elems) {
#ifdef __AVX512F__
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
static inline void simd_convert_float_to_short(const float* in_buf,
                                               short* out_buf, size_t n_elems,
                                               size_t cp_len,
                                               size_t scale_down_factor) {
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
    if (i >= n_elems - cp_len)
      _mm512_stream_si512((__m512i*)&out_buf[2 * (i + cp_len - n_elems)],
                          integer1);
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
    if (i >= n_elems - cp_len)
      _mm256_stream_si256((__m256i*)&out_buf[2 * (i + cp_len - n_elems)],
                          integer1);
  }
#endif
}

// Convert a float IQ array [in_buf] to an uint8_t IQ array [out_buf].
// Each float is converted to 12-bit data (2 floats corresponds to 3 uint8_t).
// Input array must have [n_elems] elements.
// Output array must have [n_elems / 2 * 3] elements.
// n_elems must be multiples of 2
static inline void convert_float_to_12bit_iq(const float* in_buf,
                                             uint8_t* out_buf, size_t n_elems) {
  size_t index_short = 0;
  for (size_t i = 0; i < n_elems; i = i + 2) {
    ushort temp_I = (unsigned short)(in_buf[i] * 32768 * 4);
    ushort temp_Q = (unsigned short)(in_buf[i + 1] * 32768 * 4);
    // Take the higher 12 bits and ignore the lower 4 bits
    out_buf[index_short] = (uint8_t)(temp_I >> 4);
    out_buf[index_short + 1] =
        ((uint8_t)(temp_I >> 12)) | ((uint8_t)(temp_Q & 0xf0));
    out_buf[index_short + 2] = (uint8_t)(temp_Q >> 8);
    if (kDebug12BitIQ) {
      std::cout << "i: " << i << " " << std::bitset<16>(temp_I) << " "
                << std::bitset<16>(temp_Q) << " => "
                << std::bitset<8>(out_buf[index_short]) << " "
                << std::bitset<8>(out_buf[index_short + 1]) << " "
                << std::bitset<8>(out_buf[index_short + 2]) << std::endl;
      std::printf("Original: %.4f+%.4fi \n", in_buf[i], in_buf[i + 1]);
    }
    index_short += 3;
  }
}

#ifdef __AVX512F__
static inline void simd_convert_16bit_iq_to_float(__m256i val, float* out_buf,
                                                  __m512 magic,
                                                  __m512i magic_i) {
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

static inline void convert_12bit_iq_to_16bit_iq(uint8_t* in_buf,
                                                uint16_t* out_buf,
                                                size_t n_elems) {
  for (size_t i = 0; i < n_elems; i += 16) {
    _mm256_loadu_si256((__m256i const*)in_buf);
    _mm256_loadu_si256((__m256i const*)(in_buf + 16));
    __m256i temp_I =
        _mm256_setr_epi16(*(uint16_t*)in_buf, *(uint16_t*)(in_buf + 3),
                          *(uint16_t*)(in_buf + 6), *(uint16_t*)(in_buf + 9),
                          *(uint16_t*)(in_buf + 12), *(uint16_t*)(in_buf + 15),
                          *(uint16_t*)(in_buf + 18), *(uint16_t*)(in_buf + 21),
                          *(uint16_t*)(in_buf + 24), *(uint16_t*)(in_buf + 27),
                          *(uint16_t*)(in_buf + 30), *(uint16_t*)(in_buf + 33),
                          *(uint16_t*)(in_buf + 36), *(uint16_t*)(in_buf + 39),
                          *(uint16_t*)(in_buf + 42), *(uint16_t*)(in_buf + 45));

    __m256i mask_Q = _mm256_set1_epi16(0xfff0);
    __m256i temp_Q =
        _mm256_setr_epi16(*(uint16_t*)(in_buf + 1), *(uint16_t*)(in_buf + 4),
                          *(uint16_t*)(in_buf + 7), *(uint16_t*)(in_buf + 10),
                          *(uint16_t*)(in_buf + 13), *(uint16_t*)(in_buf + 16),
                          *(uint16_t*)(in_buf + 19), *(uint16_t*)(in_buf + 22),
                          *(uint16_t*)(in_buf + 25), *(uint16_t*)(in_buf + 28),
                          *(uint16_t*)(in_buf + 31), *(uint16_t*)(in_buf + 34),
                          *(uint16_t*)(in_buf + 37), *(uint16_t*)(in_buf + 40),
                          *(uint16_t*)(in_buf + 43), *(uint16_t*)(in_buf + 46));

    temp_Q = _mm256_and_si256(temp_Q, mask_Q);  // Set lower 4 bits to 0
    temp_I = _mm256_slli_epi16(temp_I, 4);      // Shift left by 4 bits

    __m256i iq_0 = _mm256_unpacklo_epi16(temp_I, temp_Q);
    __m256i iq_1 = _mm256_unpackhi_epi16(temp_I, temp_Q);
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
static inline void simd_convert_12bit_iq_to_float(uint8_t* in_buf,
                                                  float* out_buf,
                                                  uint16_t* in_16bits_buf,
                                                  size_t n_elems) {
  _unused(in_16bits_buf);
#ifdef __AVX512F__
  const __m512 magic = _mm512_set1_ps(float((1 << 23) + (1 << 15)) / 131072.f);
  const __m512i magic_i = _mm512_castps_si512(magic);
#else
  const __m256 magic = _mm256_set1_ps(float((1 << 23) + (1 << 15)) / 131072.f);
  const __m256i magic_i = _mm256_castps_si256(magic);
#endif
  for (size_t i = 0; i < n_elems / 3; i += 16) {
    // Convert 16 IQ smaples from 48 uint8_t to 32 shorts
    // convert_12bit_iq_to_16bit_iq(in_buf + i * 3, in_16bits_buf, 16);
    __m256i temp_I =
        _mm256_setr_epi16(*(uint16_t*)in_buf, *(uint16_t*)(in_buf + 3),
                          *(uint16_t*)(in_buf + 6), *(uint16_t*)(in_buf + 9),
                          *(uint16_t*)(in_buf + 12), *(uint16_t*)(in_buf + 15),
                          *(uint16_t*)(in_buf + 18), *(uint16_t*)(in_buf + 21),
                          *(uint16_t*)(in_buf + 24), *(uint16_t*)(in_buf + 27),
                          *(uint16_t*)(in_buf + 30), *(uint16_t*)(in_buf + 33),
                          *(uint16_t*)(in_buf + 36), *(uint16_t*)(in_buf + 39),
                          *(uint16_t*)(in_buf + 42), *(uint16_t*)(in_buf + 45));

    __m256i mask_Q = _mm256_set1_epi16(0xfff0);
    __m256i temp_Q =
        _mm256_setr_epi16(*(uint16_t*)(in_buf + 1), *(uint16_t*)(in_buf + 4),
                          *(uint16_t*)(in_buf + 7), *(uint16_t*)(in_buf + 10),
                          *(uint16_t*)(in_buf + 13), *(uint16_t*)(in_buf + 16),
                          *(uint16_t*)(in_buf + 19), *(uint16_t*)(in_buf + 22),
                          *(uint16_t*)(in_buf + 25), *(uint16_t*)(in_buf + 28),
                          *(uint16_t*)(in_buf + 31), *(uint16_t*)(in_buf + 34),
                          *(uint16_t*)(in_buf + 37), *(uint16_t*)(in_buf + 40),
                          *(uint16_t*)(in_buf + 43), *(uint16_t*)(in_buf + 46));

    temp_Q = _mm256_and_si256(temp_Q, mask_Q);  // Set lower 4 bits to 0
    temp_I = _mm256_slli_epi16(temp_I, 4);      // Shift left by 4 bits

    __m256i iq_0 = _mm256_unpacklo_epi16(temp_I, temp_Q);
    __m256i iq_1 = _mm256_unpackhi_epi16(temp_I, temp_Q);
    __m256i output_0 = _mm256_permute2f128_si256(iq_0, iq_1, 0x20);
    __m256i output_1 = _mm256_permute2f128_si256(iq_0, iq_1, 0x31);
#ifdef __AVX512F__
    simd_convert_16bit_iq_to_float(output_0, out_buf + i * 2, magic, magic_i);
    simd_convert_16bit_iq_to_float(output_1, out_buf + i * 2 + 16, magic,
                                   magic_i);
#else
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
#endif
    in_buf += 48;
  }
}

// Convert a float16 array [in_buf] to a float32 array [out_buf]. Each array
// must have [n_elems] elements
// in_buf and out_buf must be 64-byte aligned
// n_elems must be a multiple of 16
static inline void simd_convert_float16_to_float32(float* out_buf,
                                                   const float* in_buf,
                                                   size_t n_elems) {
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
static inline void simd_convert_float32_to_float16(float* out_buf,
                                                   const float* in_buf,
                                                   size_t n_elems) {
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

#endif