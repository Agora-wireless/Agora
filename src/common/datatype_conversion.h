/**
 * @file datatype-conversion.h
 * @brief SIMD functions for converting between data types
 */

#ifndef DATATYPE_CONVERSION
#define DATATYPE_CONVERSION

#include <emmintrin.h>
#include <immintrin.h>

// Convert a short array [in_buf] to a float array [out_buf]. Each array must
// have [n_elems] elements.
// in_buf and out_buf must be 64-byte aligned
// n_elems must be a multiple of 16
// reference:
// https://stackoverflow.com/questions/50597764/convert-signed-short-to-float-in-c-simd
static inline void simd_convert_short_to_float(
    const short* in_buf, float* out_buf, size_t n_elems)
{
#ifdef __AVX512F__
    const __m512 magic = _mm512_set1_ps(float((1 << 23) + (1 << 15)) / 32768.f);
    const __m512i magic_i = _mm512_castps_si512(magic);
    for (size_t i = 0; i < n_elems; i += 16) {
        /* get input */
        __m256i val = _mm256_load_si256((__m256i*)(in_buf + i)); // port 2,3
        /* interleave with 0x0000 */
        __m512i val_unpacked = _mm512_cvtepu16_epi32(val); // port 5
        /* convert by xor-ing and subtracting magic value:
         * VPXOR avoids port5 bottlenecks on Intel CPUs before SKL */
        __m512i val_f_int
            = _mm512_xor_si512(val_unpacked, magic_i); // port 0,1,5
        __m512 val_f = _mm512_castsi512_ps(val_f_int); // no instruction
        __m512 converted = _mm512_sub_ps(val_f, magic); // port 1,5 ?
        _mm512_store_ps(out_buf + i, converted); // port 2,3,4,7
    }
#else
    const __m256 magic = _mm256_set1_ps(float((1 << 23) + (1 << 15)) / 32768.f);
    const __m256i magic_i = _mm256_castps_si256(magic);
    for (size_t i = 0; i < n_elems; i += 16) {
        /* get input */
        __m128i val = _mm_load_si128((__m128i*)(in_buf + i)); // port 2,3

        __m128i val1 = _mm_load_si128((__m128i*)(in_buf + i + 8));
        /* interleave with 0x0000 */
        __m256i val_unpacked = _mm256_cvtepu16_epi32(val); // port 5
        /* convert by xor-ing and subtracting magic value:
         * VPXOR avoids port5 bottlenecks on Intel CPUs before SKL */
        __m256i val_f_int
            = _mm256_xor_si256(val_unpacked, magic_i); // port 0,1,5
        __m256 val_f = _mm256_castsi256_ps(val_f_int); // no instruction
        __m256 converted = _mm256_sub_ps(val_f, magic); // port 1,5 ?
        _mm256_store_ps(out_buf + i, converted); // port 2,3,4,7

        __m256i val_unpacked1 = _mm256_cvtepu16_epi32(val1); // port 5
        __m256i val_f_int1
            = _mm256_xor_si256(val_unpacked1, magic_i); // port 0,1,5
        __m256 val_f1 = _mm256_castsi256_ps(val_f_int1); // no instruction
        __m256 converted1 = _mm256_sub_ps(val_f1, magic); // port 1,5 ?
        _mm256_store_ps(out_buf + i + 8, converted1); // port 2,3,4,7
    }
#endif
}

// Convert a float array [in_buf] to a short array [out_buf]. Input array must
// have [n_elems] elements. Output array must have [n_elems + cp_len] elements.
// in_buf and out_buf must be 64-byte aligned
// n_elems must be a multiple of 8 for AVX2 and 16 for AVX512
// scale_down_factor is used for scaling down values in the input array
static inline void simd_convert_float_to_short(const float* in_buf,
    short* out_buf, size_t n_elems, size_t cp_len, size_t scale_down_factor)
{

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
            _mm512_stream_si512(
                (__m512i*)&out_buf[2 * (i + cp_len - n_elems)], integer1);
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
            _mm256_stream_si256(
                (__m256i*)&out_buf[2 * (i + cp_len - n_elems)], integer1);
    }
#endif
}

// Convert a float16 array [in_buf] to a float32 array [out_buf]. Each array
// must have [n_elems] elements
// in_buf and out_buf must be 64-byte aligned
// n_elems must be a multiple of 16
static inline void simd_convert_float16_to_float32(
    float* out_buf, const float* in_buf, size_t n_elems)
{
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
static inline void simd_convert_float32_to_float16(
    float* out_buf, const float* in_buf, size_t n_elems)
{
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