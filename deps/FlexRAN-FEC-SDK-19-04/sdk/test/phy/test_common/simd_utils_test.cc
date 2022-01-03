/**********************************************************************
*
* INTEL CONFIDENTIAL
* Copyright 2009-2019 Intel Corporation All Rights Reserved.
* 
* The source code contained or described herein and all documents related to the
* source code ("Material") are owned by Intel Corporation or its suppliers or
* licensors. Title to the Material remains with Intel Corporation or its
* suppliers and licensors. The Material may contain trade secrets and proprietary
* and confidential information of Intel Corporation and its suppliers and
* licensors, and is protected by worldwide copyright and trade secret laws and
* treaty provisions. No part of the Material may be used, copied, reproduced,
* modified, published, uploaded, posted, transmitted, distributed, or disclosed
* in any way without Intel's prior express written permission.
* 
* No license under any patent, copyright, trade secret or other intellectual
* property right is granted to or conferred upon you by disclosure or delivery
* of the Materials, either expressly, by implication, inducement, estoppel or
* otherwise. Any license under such intellectual property rights must be
* express and approved by Intel in writing.
* 
* Unless otherwise agreed by Intel in writing, you may not remove or alter this
* notice or any other notice embedded in Materials by Intel or Intel's suppliers
* or licensors in any way.
* 
*  version: SDK-jenkins-FlexRAN-SDK-REL-448-g3be238
*
**********************************************************************/

#include "simd_utils.hpp"

#include "common.hpp"

bool Equals(__m256 lhs, __m256 rhs) {
    const auto isEq = _mm256_cmp_ps(lhs, rhs, _CMP_EQ_OQ);
    return _mm256_movemask_ps(isEq) == 0xFF;
}

bool Equals(__m512 lhs, __m512 rhs) {
    const auto isEq = _mm512_cmpeq_ps_mask (lhs, rhs);
    return isEq == 0xFFFF;
}

bool Equals_epi16(__m256i lhs, __m256i rhs) {
    const auto isEq = _mm256_cmpeq_epi16(lhs, rhs);
    return _mm256_movemask_epi8(isEq) == 0xFFFFFFFFFFFFFFFF;
}

bool Equals_epi16(__m512i lhs, __m512i rhs) {
    const auto isEq = _mm512_cmpeq_epi16_mask(lhs, rhs);
    return isEq == 0xFFFFFFFF;
}

bool Equals_epi32(__m256i lhs, __m256i rhs) {
    const auto isEq = _mm256_cmpeq_epi32(lhs, rhs);
    return _mm256_movemask_epi8(isEq) == 0xFFFFFFFFFFFFFFFF;
}

bool Equals_epi32(__m512i lhs, __m512i rhs) {
    const auto isEq = _mm512_cmpeq_epi32_mask(lhs, rhs);
    return isEq == 0xFFFF;
}

#ifdef _BBLIB_AVX2_
TEST(SimdUtilsCheck, CopyInvertedSignAvx2)
{
    F32vec8 from = _mm256_set_ps(1.0, 2.0, 3.0, 4.0, -5.0, -6.0, -7.0, -8.0);
    F32vec8 to   = _mm256_set_ps(2.0, -1.0, 3.0, -4.0, 0.2, -0.1, 0.3, -0.4);

    auto result = _mm256_set_ps(-2.0, -1.0, -3.0, -4.0, 0.2, 0.1, 0.3, 0.4);

    ASSERT_TRUE(Equals(CopyInvertedSign(from, to), result));
}

TEST(SimdUtilsCheck, NegativeAbsAvx2)
{
    F32vec8 from = _mm256_set_ps(1.0, 2.0, 3.0, 4.0, -5.0, -6.0, -7.0, -8.0);

    F32vec8 result = _mm256_set_ps(-1.0, -2.0, -3.0, -4.0, -5.0, -6.0, -7.0, -8.0);

    ASSERT_TRUE(Equals(NegativeAbs(from), result));
}

TEST(SimdUtilsCheck, Pack8Avx2)
{
    float input[64] = {1.0f, 2.0f, 3.0f, 4.0f, -5.0f, -6.0f, -7.0f, -8.0f,
                       -1.0f, -2.0f, -3.0f, -4.0f, -5.0f, -6.0f, -7.0f, -8.0f,
                       1.0f, 2.0f, 3.0f, 4.0f, -5.0f, -6.0f, -7.0f, -8.0f,
                       -1.0f, -2.0f, -3.0f, -4.0f, -5.0f, -6.0f, -7.0f, -8.0f};

    float output[64];

    ASSERT_THROW(PackSimd8(input, output, 12), std::runtime_error);

    unsigned sizes[] = {8, 16, 24, 32, 64};

    for(auto size : sizes)
    {
        PackSimd8(input, output, size);

        for (size_t i = 0; i < size / 8; ++i) {
            for (size_t s = 0; s < 8; ++s) {
                ASSERT_EQ(input[s * (size / 8) + i], output[i * 8 + s]);
            }
        }
    }
}

TEST(SimdUtilsCheck, Unpack8Avx2)
{
    float input[64] = {1.0f, 2.0f, 3.0f, 4.0f, -5.0f, -6.0f, -7.0f, -8.0f,
                       -1.0f, -2.0f, -3.0f, -4.0f, -5.0f, -6.0f, -7.0f, -8.0f,
                       1.0f, 2.0f, 3.0f, 4.0f, -5.0f, -6.0f, -7.0f, -8.0f,
                       -1.0f, -2.0f, -3.0f, -4.0f, -5.0f, -6.0f, -7.0f, -8.0f};

    float packed[64];
    float output[64];

    ASSERT_THROW(UnpackSimd8(packed, output, 12), std::runtime_error);

    unsigned sizes[] = {8, 16, 24, 32, 64};

    for(auto size : sizes)
    {
        for (size_t i = 0; i < (size / 8); ++i) {
            for (size_t s = 0; s < 8; ++s) {
                packed[i * 8 + s] = input[s * (size / 8) + i];
            }
        }

        UnpackSimd8(packed, output, size);

        for (unsigned index = 0; index < size; index++)
            ASSERT_EQ(input[index], output[index]);
    }
}

TEST(SimdUtilsCheck, StoreMaskFxpAvx2)
{
    constexpr int values_per_vec = sizeof(I16vec16) / sizeof(int16_t);

    auto input = _mm256_setr_epi16(0, 1, 2, 3, 4, 5, 6, 7,
                                   8, 9, 10, 11, 12, 13, 14, 15);

    int16_t output[values_per_vec] {0};

    // mask 1111 1111 1111 1111
    __mmask32 mask = 0xFFFF;
    int16_t reference[values_per_vec] {0, 1, 2, 3, 4, 5, 6, 7,
                           8, 9, 10, 11, 12, 13, 14, 15};
    store_mask((I16vec16*)output, mask, input);
    ASSERT_ARRAY_EQ(reference, output, values_per_vec);

    // mask 0101 0101 0101 0101
    mask = 0x5555;
    int16_t reference2[values_per_vec] {0, 0, 2, 0, 4, 0, 6, 0,
                            8, 0, 10, 0, 12, 0, 14, 0};
    std::memset(output, 0, sizeof(output));
    store_mask((I16vec16*)output, mask, input);
    ASSERT_ARRAY_EQ(reference2, output, values_per_vec);

    // mask 0000 0000 0000 0010
    mask = 0x2;
    int16_t reference3[values_per_vec] {0, 1};
    std::memset(output, 0, sizeof(output));
    store_mask((I16vec16*)output, mask, input);
    ASSERT_ARRAY_EQ(reference3, output, values_per_vec);
}

TEST(SimdUtilsCheck, StoreMaskFlpAvx2)
{
    constexpr int values_per_vec = sizeof(F32vec8) / sizeof(float);
    constexpr double precision = 10e-5;

    auto input = _mm256_setr_ps(0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f);
    float output[values_per_vec] {0};

    // mask 1111 1111
    __mmask8 mask = 0xFF;
    float reference[values_per_vec] {0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f};
    store_mask((F32vec8*)output, mask, input);
    ASSERT_ARRAY_NEAR(reference, output, values_per_vec, precision);

    // mask 0101 0101
    mask = 0x55;
    float reference2[values_per_vec] {0.0f, 0.0f, 2.0f, 0.0f, 4.0f, 0.0f, 6.0f, 0.0f};
    std::memset(output, 0, sizeof(output));
    store_mask((F32vec8*)output, mask, input);
    ASSERT_ARRAY_NEAR(reference2, output, values_per_vec, precision);

    // mask 0000 0010
    mask = 0x2;
    float reference3[values_per_vec] {0.0f, 1.0f};
    std::memset(output, 0, sizeof(output));
    store_mask((F32vec8*)output, mask, input);
    ASSERT_ARRAY_NEAR(reference3, output, values_per_vec, precision);
}

TEST(SimdUtilsCheck, SumComplexVecElem16bAccAvx2)
{
    auto vector = _mm256_set_epi16(7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1, 0, 0);

    auto result = _mm256_set1_epi16(28);

    ASSERT_TRUE(Equals_epi16(sum_complex_vec_elem_16b_acc(vector), result));
}

TEST(SimdUtilsCheck, SumComplexVecElem32bAccAvx2)
{
    auto vector = _mm256_set_epi16(4103, 4103, 4102, 4102, 4101, 4101, 4100, 4100, 4099, 4099, 4098, 4098, 4097, 4097, 4096, 4096);

    auto result = _mm256_set1_epi32(32796);

    ASSERT_TRUE(Equals_epi32(sum_complex_vec_elem_32b_acc(vector), result));
}

TEST(SimdUtilsCheck, SumComplexVecElemAvx2)
{
    constexpr int num_of_val = 8;
    constexpr double tolerance = 1e-5;

    float *temp = generate_random_real_numbers<float>(num_of_val, 64, 0.0f, 1.0f);
    auto vector = _mm256_set_ps(temp[7], temp[6], temp[5], temp[4], temp[3], temp[2], temp[1], temp[0]);

    auto sum = sum_complex_vec_elem(vector);
    float *addr = aligned_malloc<float>(num_of_val, 64);

    _mm256_store_ps((float*)addr, sum);

    auto ref_sum_real = temp[6] + temp[4] + temp[2] + temp[0];
    auto ref_sum_imag = temp[7] + temp[5] + temp[3] + temp[1];

    ASSERT_NEAR(ref_sum_real, addr[0], tolerance);
    ASSERT_NEAR(ref_sum_imag, addr[1], tolerance);
}
#endif

#ifdef _BBLIB_AVX512_
TEST(SimdUtilsCheck, CopyInvertedSignAvx512)
{
    F32vec16 from = _mm512_set_ps(1.0, 2.0, 3.0, 4.0, -5.0, -6.0, -7.0, -8.0, 1.0, 2.0, 3.0, 4.0, -5.0, -6.0, -7.0, -8.0);
    F32vec16 to   = _mm512_set_ps(2.0, -1.0, 3.0, -4.0, 0.2, -0.1, 0.3, -0.4, 2.0, -1.0, 3.0, -4.0, 0.2, -0.1, 0.3, -0.4);

    auto result = _mm512_set_ps(-2.0, -1.0, -3.0, -4.0, 0.2, 0.1, 0.3, 0.4, -2.0, -1.0, -3.0, -4.0, 0.2, 0.1, 0.3, 0.4);

    ASSERT_TRUE(Equals(CopyInvertedSign(from, to), result));
}

TEST(SimdUtilsCheck, NegativeAbsAvx512)
{
    F32vec16 from = _mm512_set_ps(1.0, 2.0, 3.0, 4.0, -5.0, -6.0, -7.0, -8.0, 1.0, 2.0, 3.0, 4.0, -5.0, -6.0, -7.0, -8.0);

    F32vec16 result = _mm512_set_ps(-1.0, -2.0, -3.0, -4.0, -5.0, -6.0, -7.0, -8.0, -1.0, -2.0, -3.0, -4.0, -5.0, -6.0, -7.0, -8.0);

    ASSERT_TRUE(Equals(NegativeAbs(from), result));
}

TEST(SimdUtilsCheck, Pack16Avx512)
{
    float input[256] = {1.0f, 2.0f, 3.0f, 4.0f, -5.0f, -6.0f, -7.0f, -8.0f,
                       -1.0f, -2.0f, -3.0f, -4.0f, -5.0f, -6.0f, -7.0f, -8.0f,
                       1.0f, 2.0f, 3.0f, 4.0f, -5.0f, -6.0f, -7.0f, -8.0f,
                       -1.0f, -2.0f, -3.0f, -4.0f, -5.0f, -6.0f, -7.0f, -8.0f,
                        1.0f, 2.0f, 3.0f, 4.0f, -5.0f, -6.0f, -7.0f, -8.0f,
                        -1.0f, -2.0f, -3.0f, -4.0f, -5.0f, -6.0f, -7.0f, -8.0f,
                        1.0f, 2.0f, 3.0f, 4.0f, -5.0f, -6.0f, -7.0f, -8.0f,
                        -1.0f, -2.0f, -3.0f, -4.0f, -5.0f, -6.0f, -7.0f, -8.0f,
                        1.0f, 2.0f, 3.0f, 4.0f, -5.0f, -6.0f, -7.0f, -8.0f,
                        -1.0f, -2.0f, -3.0f, -4.0f, -5.0f, -6.0f, -7.0f, -8.0f,
                        1.0f, 2.0f, 3.0f, 4.0f, -5.0f, -6.0f, -7.0f, -8.0f,
                        -1.0f, -2.0f, -3.0f, -4.0f, -5.0f, -6.0f, -7.0f, -8.0f,
                        1.0f, 2.0f, 3.0f, 4.0f, -5.0f, -6.0f, -7.0f, -8.0f,
                        -1.0f, -2.0f, -3.0f, -4.0f, -5.0f, -6.0f, -7.0f, -8.0f,
                        1.0f, 2.0f, 3.0f, 4.0f, -5.0f, -6.0f, -7.0f, -8.0f,
                        -1.0f, -2.0f, -3.0f, -4.0f, -5.0f, -6.0f, -7.0f, -8.0f};

    float output[256];

    ASSERT_THROW(PackSimd16(input, output, 12), std::runtime_error);

    unsigned sizes[] = {16, 32, 64, 96, 128, 192, 256};

    for(auto size : sizes)
    {
        PackSimd16(input, output, size);

        for (size_t i = 0; i < size / 16; ++i) {
            for (size_t s = 0; s < 16; ++s) {
                ASSERT_EQ(input[s * (size / 16) + i], output[i * 16 + s]);
            }
        }
    }
}

TEST(SimdUtilsCheck, Unpack16Avx512)
{
    float input[256] = {1.0f, 2.0f, 3.0f, 4.0f, -5.0f, -6.0f, -7.0f, -8.0f,
                        -1.0f, -2.0f, -3.0f, -4.0f, -5.0f, -6.0f, -7.0f, -8.0f,
                        1.0f, 2.0f, 3.0f, 4.0f, -5.0f, -6.0f, -7.0f, -8.0f,
                        -1.0f, -2.0f, -3.0f, -4.0f, -5.0f, -6.0f, -7.0f, -8.0f,
                        1.0f, 2.0f, 3.0f, 4.0f, -5.0f, -6.0f, -7.0f, -8.0f,
                        -1.0f, -2.0f, -3.0f, -4.0f, -5.0f, -6.0f, -7.0f, -8.0f,
                        1.0f, 2.0f, 3.0f, 4.0f, -5.0f, -6.0f, -7.0f, -8.0f,
                        -1.0f, -2.0f, -3.0f, -4.0f, -5.0f, -6.0f, -7.0f, -8.0f,
                        1.0f, 2.0f, 3.0f, 4.0f, -5.0f, -6.0f, -7.0f, -8.0f,
                        -1.0f, -2.0f, -3.0f, -4.0f, -5.0f, -6.0f, -7.0f, -8.0f,
                        1.0f, 2.0f, 3.0f, 4.0f, -5.0f, -6.0f, -7.0f, -8.0f,
                        -1.0f, -2.0f, -3.0f, -4.0f, -5.0f, -6.0f, -7.0f, -8.0f,
                        1.0f, 2.0f, 3.0f, 4.0f, -5.0f, -6.0f, -7.0f, -8.0f,
                        -1.0f, -2.0f, -3.0f, -4.0f, -5.0f, -6.0f, -7.0f, -8.0f,
                        1.0f, 2.0f, 3.0f, 4.0f, -5.0f, -6.0f, -7.0f, -8.0f,
                        -1.0f, -2.0f, -3.0f, -4.0f, -5.0f, -6.0f, -7.0f, -8.0f};

    float packed[256];
    float output[256];

    ASSERT_THROW(UnpackSimd16(packed, output, 12), std::runtime_error);

    unsigned sizes[] = {16, 32, 64, 96, 128, 192, 256};

    for(auto size : sizes)
    {
        for (size_t i = 0; i < (size / 16); ++i) {
            for (size_t s = 0; s < 16; ++s) {
                packed[i * 16 + s] = input[s * (size / 16) + i];
            }
        }

        UnpackSimd16(packed, output, size);

        for (unsigned index = 0; index < size; index++)
            ASSERT_EQ(input[index], output[index]);
    }
}

TEST(SimdUtilsCheck, StoreMaskFxpAvx512)
{
    constexpr int values_per_vec = sizeof(I16vec32) / sizeof(int16_t);

    auto input = _mm512_set_epi16(31, 30, 29, 28, 27, 26, 25, 24,
                                  23, 22, 21, 20, 19, 18, 17, 16,
                                  15, 14, 13, 12, 11, 10, 9, 8,
                                  7, 6, 5, 4, 3, 2, 1, 0);

    int16_t output[values_per_vec] {0};

    // mask 1111 1111 1111 1111 1111 1111 1111 1111
    __mmask32 mask = 0xFFFFFFFF;
    int16_t reference[values_per_vec] {0, 1, 2, 3, 4, 5, 6, 7,
                                       8, 9, 10, 11, 12, 13, 14, 15,
                                       16, 17, 18, 19, 20, 21, 22, 23,
                                       24, 25, 26, 27, 28, 29, 30, 31};
    store_mask((I16vec32*)output, mask, input);
    ASSERT_ARRAY_EQ(reference, output, 32);

    // mask 0101 0101 0101 0101 0101 0101 0101 0101
    mask = 0x55555555;
    int16_t reference2[values_per_vec] {0, 0, 2, 0, 4, 0, 6, 0,
                                        8, 0, 10, 0, 12, 0, 14, 0,
                                        16, 0, 18, 0, 20, 0, 22, 0,
                                        24, 0, 26, 0, 28, 0, 30, 0};
    std::memset(output, 0, sizeof(output));
    store_mask((I16vec32*)output, mask, input);
    ASSERT_ARRAY_EQ(reference2, output, values_per_vec);

    // mask 0000 0000 0000 0000 0000 0000 0000 0010
    mask = 0x2;
    int16_t reference3[values_per_vec] {0, 1};
    std::memset(output, 0, sizeof(output));
    store_mask((I16vec32*)output, mask, input);
    ASSERT_ARRAY_EQ(reference3, output, values_per_vec);
}

TEST(SimdUtilsCheck, StoreMaskFlpAvx512)
{
    constexpr int values_per_vec = sizeof(F32vec16) / sizeof(float);
    constexpr double precision = 10e-5;

    auto input = _mm512_setr_ps(0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f,
                                8.0f, 9.0f, 10.0f, 11.0f, 12.0f, 13.0f, 14.0f, 15.0f);
    float output[values_per_vec] {0};

    // mask 1111 1111 1111 1111
    __mmask16 mask = 0xFFFF;
    float reference[values_per_vec] {0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f,
                                     8.0f, 9.0f, 10.0f, 11.0f, 12.0f, 13.0f, 14.0f, 15.0f};
    store_mask((F32vec16*)output, mask, input);
    ASSERT_ARRAY_NEAR(reference, output, values_per_vec, precision);

    // mask 0101 0101 0101 0101
    mask = 0x5555;
    float reference2[values_per_vec] {0.0f, 0.0f, 2.0f, 0.0f, 4.0f, 0.0f, 6.0f, 0.0f,
                                      8.0f, 0.0f, 10.0f, 0.0f, 12.0f, 0.0f, 14.0f, 0.0f};
    std::memset(output, 0, sizeof(output));
    store_mask((F32vec16*)output, mask, input);
    ASSERT_ARRAY_NEAR(reference2, output, values_per_vec, precision);

    // mask 0000 0000 0000 0010
    mask = 0x2;
    float reference3[values_per_vec] {0.0f, 1.0f};
    std::memset(output, 0, sizeof(output));
    store_mask((F32vec16*)output, mask, input);
    ASSERT_ARRAY_NEAR(reference3, output, values_per_vec, precision);
}

TEST(SimdUtilsCheck, SumComplexVecElem16bAccAvx512)
{
    auto vector = _mm512_set_epi16(15, 15, 14, 14, 13, 13, 12, 12, 11, 11, 10, 10, 9, 9, 8, 8,
                                   7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1, 0, 0);

    auto result = _mm512_set1_epi16(120);
for (int i=0; i < 1000; i++)
{
    ASSERT_TRUE(Equals_epi16(sum_complex_vec_elem_16b_acc(vector), result));
}
}

TEST(SimdUtilsCheck, SumComplexVecElem32bAccAvx512)
{
    auto vector = _mm512_set_epi16(4111, 4111, 4110, 4110, 4109, 4109, 4108, 4108, 4107, 4107, 4106, 4106, 4105, 4105, 4104, 4104,
                                   4103, 4103, 4102, 4102, 4101, 4101, 4100, 4100, 4099, 4099, 4098, 4098, 4097, 4097, 4096, 4096);

    auto result = _mm512_set1_epi32(65656);

    ASSERT_TRUE(Equals_epi32(sum_complex_vec_elem_32b_acc(vector), result));
}

#endif

