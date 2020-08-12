#include <gtest/gtest.h>
// For some reason, gtest include order matters
#include "datatype_conversion.h"
#include <malloc.h>

static constexpr size_t kSIMDTestNum = 1024;

TEST(SIMD, float_32_to_16)
{
    constexpr float allowed_error = 1e-3;
    float* in_buf
        = reinterpret_cast<float*>(memalign(64, kSIMDTestNum * sizeof(float)));
    for (size_t i = 0; i < kSIMDTestNum; i++) {
        in_buf[i] = static_cast<float>(rand()) / RAND_MAX;
    }

    float* medium = reinterpret_cast<float*>(
        memalign(64, kSIMDTestNum / 2 * sizeof(float)));
    simd_convert_float32_to_float16(medium, in_buf, kSIMDTestNum);

    float* out_buf
        = reinterpret_cast<float*>(memalign(64, kSIMDTestNum * sizeof(float)));
    simd_convert_float16_to_float32(medium, out_buf, kSIMDTestNum);

    for (size_t i = 0; i < kSIMDTestNum; i++) {
        ASSERT_LE(abs(in_buf[i] - out_buf[i]), allowed_error);
    }

    free(in_buf);
    free(medium);
    free(out_buf);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}