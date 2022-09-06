/**
 * @file datatype-conversion.h
 * @brief SIMD functions for converting between data types
 */

#ifndef DATATYPE_CONVERSION_H_
#define DATATYPE_CONVERSION_H_

#include <emmintrin.h>
#include <immintrin.h>

#include <bitset>

#include "utils.h"

//#define DATATYPE_MEMORY_CHECK
//Needs to be a factor of 2?
static constexpr float kShrtFltConvFactor = 32768.0f;

#if defined(__AVX512F__)
constexpr size_t kAvx512Bits = 512;
constexpr size_t kAvx512Bytes = kAvx512Bits / 8;
constexpr size_t kAvx512FloatsPerInstr = kAvx512Bytes / sizeof(float);
//2 because of the 32->16 ints
constexpr size_t kAvx512FloatsPerLoop = kAvx512FloatsPerInstr * 2;
constexpr size_t kAvx512ShortsPerInstr = kAvx512Bytes / sizeof(short);
//Half because read->expand->use 512 instr
constexpr size_t kAvx512ShortsPerLoop = kAvx512ShortsPerInstr / 2;
#endif

constexpr size_t kAvx2Bits = 256;
constexpr size_t kAvx2Bytes = kAvx2Bits / 8;
constexpr size_t kAvx2FloatsPerInstr = kAvx2Bytes / sizeof(float);
//2 because of the 32->16 ints
constexpr size_t kAvx2FloatsPerLoop = kAvx2FloatsPerInstr * 2;
constexpr size_t kAvx2ShortsPerInstr = kAvx2Bytes / sizeof(short);
//Half because read->expand->use 256 instr
constexpr size_t kAvx2ShortsPerLoop = kAvx2ShortsPerInstr / 2;

///Produces outputs -1->+0.999
static inline void ConvertShortToFloat(const short* in_buf, float* out_buf,
                                       size_t n_elems) {
  for (size_t i = 0; i < n_elems; i++) {
    out_buf[i] = static_cast<float>(in_buf[i]) / kShrtFltConvFactor;
  }
}

static inline void SimdConvertShortToFloatAVX512(const short* in_buf,
                                                 float* out_buf,
                                                 size_t n_elems) {
#if defined(__AVX512F__)
#if defined(DATATYPE_MEMORY_CHECK)
  RtAssert(((n_elems % kAvx512ShortsPerLoop) == 0) &&
               ((reinterpret_cast<intptr_t>(in_buf) % kAvx512Bytes) == 0) &&
               ((reinterpret_cast<intptr_t>(out_buf) % kAvx512Bytes) == 0),
           "Data Alignment not correct before calling into AVX optimizations");
#endif

  const bool unaligned =
      ((reinterpret_cast<intptr_t>(in_buf) % kAvx512Bytes) > 0);
  const __m512 magic =
      _mm512_set1_ps(float((1 << 23) + (1 << 15)) / kShrtFltConvFactor);
  const __m512i magic_i = _mm512_castps_si512(magic);
  for (size_t i = 0; i < n_elems; i += kAvx512ShortsPerLoop) {
    // Load shorts with 1/2 instr so we have room for expansion
    // port 2,3
    const __m256i val =
        unaligned
            ? _mm256_loadu_si256(reinterpret_cast<const __m256i*>(in_buf + i))
            : _mm256_load_si256(reinterpret_cast<const __m256i*>(in_buf + i));
    // Expand and  interleave with 0x0000
    const __m512i val_unpacked = _mm512_cvtepu16_epi32(val);  // port 5
    /* convert by xor-ing and subtracting magic value:
     * VPXOR avoids port5 bottlenecks on Intel CPUs before SKL */
    const __m512i val_f_int =
        _mm512_xor_si512(val_unpacked, magic_i);           // port 0,1,5
    const __m512 val_f = _mm512_castsi512_ps(val_f_int);   // no instruction
    const __m512 converted = _mm512_sub_ps(val_f, magic);  // port 1,5 ?
    _mm512_store_ps(out_buf + i, converted);               // port 2,3,4,7
  }
#else
  unused(in_buf);
  unused(out_buf);
  unused(n_elems);
  throw std::runtime_error("AVX512 is not supported");
#endif
}

static inline void SimdConvertShortToFloatAVX2(const short* in_buf,
                                               float* out_buf, size_t n_elems) {
#if defined(DATATYPE_MEMORY_CHECK)
  RtAssert(((n_elems % kAvx2ShortsPerLoop) == 0) &&
               ((reinterpret_cast<intptr_t>(in_buf) % kAvx2Bytes) == 0) &&
               ((reinterpret_cast<intptr_t>(out_buf) % kAvx2Bytes) == 0),
           "Data Alignment not correct before calling into AVX optimizations");
#endif
  const bool unaligned =
      ((reinterpret_cast<intptr_t>(in_buf) % kAvx2Bytes) > 0);
  //Divisior must be power of 2?
  const __m256 magic =
      _mm256_set1_ps(float((1 << 23) + (1 << 15)) / kShrtFltConvFactor);
  const __m256i magic_i = _mm256_castps_si256(magic);
  for (size_t i = 0; i < n_elems; i += kAvx2ShortsPerLoop) {
    // port 2,3
    const __m128i val =
        unaligned
            ? _mm_loadu_si128(reinterpret_cast<const __m128i*>(in_buf + i))
            : _mm_load_si128(reinterpret_cast<const __m128i*>(in_buf + i));

    // expand to 32bits and interleave with 0x0000
    const __m256i val_unpacked = _mm256_cvtepu16_epi32(val);  // port 5
    /* convert by xor-ing and subtracting magic value:
     * VPXOR avoids port5 bottlenecks on Intel CPUs before SKL */
    const __m256i val_f_int =
        _mm256_xor_si256(val_unpacked, magic_i);           // port 0,1,5
    const __m256 val_f = _mm256_castsi256_ps(val_f_int);   // no instruction
    const __m256 converted = _mm256_sub_ps(val_f, magic);  // port 1,5 ?
    _mm256_store_ps(out_buf + i, converted);               // port 2,3,4,7
  }
}

// Convert a short array [in_buf] to a float array [out_buf]. Each array must
// have [n_elems] elements.
// in_buf and out_buf must be 64-byte aligned
// n_elems must be a multiple of 16
static inline void SimdConvertShortToFloat(const short* in_buf, float* out_buf,
                                           size_t n_elems) {
#if defined(__AVX512F__)
  return SimdConvertShortToFloatAVX512(in_buf, out_buf, n_elems);
#else
  return SimdConvertShortToFloatAVX2(in_buf, out_buf, n_elems);
#endif
}

// Convert a float array [in_buf] to a short array [out_buf]. Input array must
// have [n_elems] elements. Output array must have [n_elems + n_prefix] elements.
// in_buf and out_buf must be 64-byte aligned
// n_elems must be a multiple of 16 for AVX512
// scale_down_factor is used for scaling down values in the input array
static inline void SimdConvertFloatToShortAVX512(const float* in_buf,
                                                 short* out_buf, size_t n_elems,
                                                 size_t n_prefix,
                                                 float scale_down_factor) {
#if defined(__AVX512F__)
#if defined(DATATYPE_MEMORY_CHECK)
  constexpr size_t kAvx512ShortPerInstr = kAvx512Bytes / sizeof(short);
  RtAssert(((n_elems % kAvx512FloatsPerInstr) == 0) &&
               ((n_prefix % kAvx512ShortPerInstr) == 0) &&
               ((reinterpret_cast<intptr_t>(in_buf) % kAvx512Bytes) == 0) &&
               ((reinterpret_cast<intptr_t>(out_buf) % kAvx512Bytes) == 0),
           "Data Alignment not correct before calling into AVX optimizations");
#endif
  const float scale_factor_float = kShrtFltConvFactor / scale_down_factor;
  const __m512 scale_factor = _mm512_set1_ps(scale_factor_float);
  const __m512i permute_index = _mm512_setr_epi64(0, 2, 4, 6, 1, 3, 5, 7);
  for (size_t i = 0; i < n_elems; i += kAvx512FloatsPerLoop) {
    const __m512 in1 = _mm512_load_ps(&in_buf[i]);
    const __m512 in2 = _mm512_load_ps(&in_buf[i + kAvx512FloatsPerInstr]);
    const __m512 scaled_in1 = _mm512_mul_ps(in1, scale_factor);
    const __m512 scaled_in2 = _mm512_mul_ps(in2, scale_factor);
    const __m512i int32_1 = _mm512_cvtps_epi32(scaled_in1);
    const __m512i int32_2 = _mm512_cvtps_epi32(scaled_in2);
    const __m512i short_int16 = _mm512_packs_epi32(int32_1, int32_2);
    const __m512i shuffled =
        _mm512_permutexvar_epi64(permute_index, short_int16);
    _mm512_stream_si512(reinterpret_cast<__m512i*>(&out_buf[i + n_prefix]),
                        shuffled);
    // Prepend / Set cyclic prefix
    const size_t repeat_idx = n_elems - n_prefix;
    if (i >= repeat_idx) {
      _mm512_stream_si512(reinterpret_cast<__m512i*>(&out_buf[i - repeat_idx]),
                          shuffled);
    }
  }
#else
  unused(in_buf);
  unused(out_buf);
  unused(n_elems);
  unused(n_prefix);
  unused(scale_down_factor);
  throw std::runtime_error("AVX512 is not supported");
#endif
}

// Convert a float array [in_buf] to a short array [out_buf]. Input array must
// have [n_elems] elements. Output array must have [n_elems + n_prefix] elements.
// in_buf and out_buf must be 64-byte aligned
// n_elems must be a multiple of 8 for AVX2
// scale_down_factor is used for scaling down values in the input array
static inline void SimdConvertFloatToShortAVX2(const float* in_buf,
                                               short* out_buf, size_t n_elems,
                                               size_t n_prefix,
                                               float scale_down_factor) {
#if defined(DATATYPE_MEMORY_CHECK)
  constexpr size_t kAvx2ShortPerInstr = kAvx2Bytes / sizeof(short);
  RtAssert(((n_elems % kAvx2FloatsPerLoop) == 0) &&
               ((n_prefix % kAvx2ShortPerInstr) == 0) &&
               ((reinterpret_cast<intptr_t>(in_buf) % kAvx2Bytes) == 0) &&
               ((reinterpret_cast<intptr_t>(out_buf) % kAvx2Bytes) == 0),
           "Data Alignment not correct before calling into AVX optimizations");
#endif

  const float scale_factor_float = kShrtFltConvFactor / scale_down_factor;

  const __m256 scale_factor = _mm256_set1_ps(scale_factor_float);
  //Operates on 2 elements at a time
  for (size_t i = 0; i < n_elems; i += kAvx2FloatsPerLoop) {
    const __m256 in1 = _mm256_load_ps(&in_buf[i]);
    //Grab the next value, and interate over 2 values
    const __m256 in2 = _mm256_load_ps(&in_buf[i + kAvx2FloatsPerInstr]);
    const __m256 scaled_in1 = _mm256_mul_ps(in1, scale_factor);
    const __m256 scaled_in2 = _mm256_mul_ps(in2, scale_factor);
    //Packed float to 32bit ints (_mm256_cvttps_epi32 vs _mm256_cvtps_epi32)
    const __m256i integer1 = _mm256_cvtps_epi32(scaled_in1);
    const __m256i integer2 = _mm256_cvtps_epi32(scaled_in2);
    //Convert dword to word and saturate
    const __m256i short_ints = _mm256_packs_epi32(integer1, integer2);
    //packing shuffles groups of 4 floats
    const __m256i slice = _mm256_permute4x64_epi64(short_ints, 0xD8);
    // _mm256_store_si256 or _mm256_stream_si256 (cache vs non-temperal) offset by n_prefix
    _mm256_stream_si256(reinterpret_cast<__m256i*>(&out_buf[i + n_prefix]),
                        slice);
    // Prepend / Set cyclic prefix
    const size_t repeat_idx = n_elems - n_prefix;
    if (i >= repeat_idx) {
      _mm256_stream_si256(reinterpret_cast<__m256i*>(&out_buf[i - repeat_idx]),
                          slice);
    }
  }
}

// Convert a float array [in_buf] to a short array [out_buf]. Input array must
// have [n_elems] elements. Output array must have [n_elems + n_prefix] elements.
// in_buf and out_buf must be 64-byte aligned
// n_prefix prepends the output with the last n_prefix values
// scale_down_factor is used for scaling down values in the input array
static inline void ConvertFloatToShort(const float* in_buf, short* out_buf,
                                       size_t n_elems, size_t n_prefix = 0,
                                       float scale_down_factor = 1.0f) {
  for (size_t i = 0; i < n_elems; i++) {
    short converted_value;
    const float scaled_value =
        in_buf[i] * (kShrtFltConvFactor / scale_down_factor);

    //Saturate the output
    if (scaled_value >= SHRT_MAX) {
      converted_value = SHRT_MAX;
    } else if (scaled_value <= SHRT_MIN) {
      converted_value = SHRT_MIN;
    } else {
      converted_value = static_cast<short>(scaled_value);
    }
    out_buf[i + n_prefix] = converted_value;
  }
  //Prepend with last cp len
  for (size_t i = 0; i < n_prefix; i++) {
    out_buf[i] = out_buf[i + n_elems];
  }
}

// Convert a float array [in_buf] to a short array [out_buf]. Input array must
// have [n_elems] elements. Output array must have [n_elems + n_prefix] elements.
// in_buf and out_buf must be 64-byte aligned
// n_elems must be a multiple of 8 for AVX2 and 16 for AVX512
// scale_down_factor is used for scaling down values in the input array
static inline void SimdConvertFloatToShort(const float* in_buf, short* out_buf,
                                           size_t n_elems, size_t n_prefix = 0,
                                           float scale_down_factor = 1.0f) {
#if defined(__AVX512F__)
  SimdConvertFloatToShortAVX512(in_buf, out_buf, n_elems, n_prefix,
                                scale_down_factor);
#else
  SimdConvertFloatToShortAVX2(in_buf, out_buf, n_elems, n_prefix,
                              scale_down_factor);
#endif
}

//Assumes complex float == float float
static inline void SimdConvertCxFloatToCxShort(
    const std::complex<float>* in_buf, std::complex<short>* out_buf,
    size_t n_elems, size_t n_prefix, float scale_down_factor) {
  const auto* in = reinterpret_cast<const float*>(in_buf);
  auto* out = reinterpret_cast<short*>(out_buf);
#if defined(__AVX512F__)
  SimdConvertFloatToShortAVX512(in, out, n_elems * 2, n_prefix * 2,
                                scale_down_factor);
#else
  SimdConvertFloatToShortAVX2(in, out, n_elems * 2, n_prefix * 2,
                              scale_down_factor);
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
  RtAssert((n_elems % 2) == 0,
           "ConvertFloatTo12bitIq n_elems not multiple of 2");
#endif
  size_t index_short = 0;
  for (size_t i = 0; i < n_elems; i = i + 2) {
    const auto temp_i =
        static_cast<unsigned short>(in_buf[i] * kShrtFltConvFactor * 4);
    const auto temp_q =
        static_cast<unsigned short>(in_buf[i + 1] * kShrtFltConvFactor * 4);
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
               ((reinterpret_cast<intptr_t>(in_buf) % kAvx2Bytes) == 0) &&
               ((reinterpret_cast<intptr_t>(out_buf) % kAvx2Bytes) == 0),
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
  RtAssert(((n_elems % 16) == 0) &&
               ((reinterpret_cast<intptr_t>(in_buf) % 64) == 0) &&
               ((reinterpret_cast<intptr_t>(out_buf) % 64) == 0),
           "SimdConvertFloat16ToFloat32: Data Alignment not correct before "
           "calling into AVX optimizations");
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
  RtAssert(((n_elems % 16) == 0) &&
               ((reinterpret_cast<intptr_t>(in_buf) % 64) == 0) &&
               ((reinterpret_cast<intptr_t>(out_buf) % 64) == 0),
           "SimdConvertFloat32ToFloat16: Data Alignment not correct before "
           "calling into AVX optimizations");
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
#endif  // DATATYPE_CONVERSION_H_