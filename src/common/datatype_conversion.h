#ifndef DATATYPE_CONVERSION
#define DATATYPE_CONVERSION

#include <emmintrin.h>
#include <immintrin.h>

void simd_convert_short_to_float(short* in_buf, float* out_buf, size_t length)
{
#ifdef __AVX512F__
    const __m512 magic = _mm512_set1_ps(float((1 << 23) + (1 << 15)) / 32768.f);
    const __m512i magic_i = _mm512_castps_si512(magic);
    for (size_t i = 0; i < length; i += 16) {
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
    for (size_t i = 0; i < length; i += 16) {
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

void simd_convert_float16_to_float32(
    float* in_buf, float* out_buf, size_t length)
{
#ifdef __AVX512F__
    for (size_t i = 0; i < length; i += 16) {
        __m256i val_a
            = _mm256_load_si256(reinterpret_cast<__m256i*>(in_buf + i / 2));
        __m512 val = _mm512_cvtph_ps(val_a);
        _mm512_store_ps(out_buf + i, val);
    }
#else
    for (size_t i = 0; i < length; i += 8) {
        __m128i val_a
            = _mm_load_si128(reinterpret_cast<__m128i*>(in_buf + i / 2));
        __m256 val = _mm256_cvtph_ps(val_a);
        _mm256_store_ps(out_buf + i, val);
    }
#endif
}

void simd_convert_float32_to_float16(
    float* in_buf, float* out_buf, size_t length)
{
#ifdef __AVX512F__
    for (size_t i = 0; i < length; i += 16) {
        __m512 val_a = _mm512_load_ps(in_buf + i);
        __m256i val = _mm512_cvtps_ph(val_a, _MM_FROUND_NO_EXC);
        _mm256_store_si256(reinterpret_cast<__m256i*>(out_buf + i / 2), val);
    }
#else
    for (size_t i = 0; i < length; i += 8) {
        __m256 val_a = _mm256_load_ps(in_buf + i);
        __m128i val = _mm256_cvtps_ph(val_a, _MM_FROUND_NO_EXC);
        _mm_store_si128(reinterpret_cast<__m128i*>(out_buf + i / 2), val);
    }
#endif
}

#endif