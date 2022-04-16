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

static inline void simd_convert_float_to_short(
    const float* in_buf, short* out_buf, size_t n_elems)
{
    for (size_t i = 0; i < n_elems; i += 16) {
        /* ifft scaled results by OFDM_CA_NUM */
        __m256 scale_factor = _mm256_set1_ps(32768.0);
        __m256 ifft1 = _mm256_load_ps(in_buf + i);
        __m256 ifft2 = _mm256_load_ps(in_buf + i + 8);
        __m256 scaled_ifft1 = _mm256_mul_ps(ifft1, scale_factor);
        __m256 scaled_ifft2 = _mm256_mul_ps(ifft2, scale_factor);
        __m256i integer1 = _mm256_cvtps_epi32(scaled_ifft1);
        __m256i integer2 = _mm256_cvtps_epi32(scaled_ifft2);
        integer1 = _mm256_packs_epi32(integer1, integer2);
        integer1 = _mm256_permute4x64_epi64(integer1, 0xD8);
        //_mm256_stream_si256((__m256i*)&socket_ptr[2 * sc_id], integer1);
        _mm256_stream_si256(
            (__m256i*)&out_buf[i], integer1);
        if (i >= n_elems) // add CP
            _mm256_stream_si256((__m256i*)&out_buf[i - n_elems],
                integer1);
    }
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

static inline void single_convert_float32_to_float16(
    unsigned short* out_buf, unsigned int* in_buf)
{
    unsigned int fltInt32 = *(in_buf);
    unsigned short fltInt16;

    fltInt16 = (fltInt32 >> 31) << 5;
    unsigned short tmp = (fltInt32 >> 23) & 0xff;
    tmp = (tmp - 0x70) & ((unsigned int)((int)(0x70 - tmp) >> 4) >> 27);
    fltInt16 = (fltInt16 | tmp) << 10;
    fltInt16 |= (fltInt32 >> 13) & 0x3ff;
    *out_buf = fltInt16;
}

#endif