#ifndef UTILS_SIMD
#define UTILS_SIMD

#include <stdlib.h>

void simd_convert_short_to_float(short* in_buf, float* out_buf, size_t length);

// The memory should be 64-byte aligned
void simd_convert_float16_to_float32(
    float* in_buf, float* out_buf, size_t length);

// The memory should be 64-byte aligned
void simd_convert_float32_to_float16(
    float* in_buf, float* out_buf, size_t length);

#endif