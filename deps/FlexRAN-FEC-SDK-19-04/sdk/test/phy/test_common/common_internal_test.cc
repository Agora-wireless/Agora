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

#include "common_typedef_simd.hpp"
#include "sdk_version.h"

#include <gtest/gtest.h>

bool Equals(__m256i lhs, __m256i rhs) {
    const auto isEq = _mm256_cmpeq_epi32(lhs, rhs);
    return _mm256_movemask_ps(_mm256_castsi256_ps(isEq)) == 0xFF;
}

TEST(SDKVersionCheck, PopulateString)
{
    char* output = new char[BBLIB_SDK_VERSION_STRING_MAX_LEN];

    const char* input = "FlexRAN SDK bblib_X version sdk-19.04-ea1-1-g3be2380";

    bblib_sdk_version(&output, &input, BBLIB_SDK_VERSION_STRING_MAX_LEN);

    ASSERT_EQ(strcmp(input, output), 0);

    delete[] output;
}

TEST(SDKVersionCheck, BufferSizeLessThanOne)
{
    int ret = bblib_sdk_version(nullptr, nullptr, 0);

    ASSERT_EQ(ret, -1);
}

TEST(SDKVersionCheck, BufferTooSmall)
{
    char* output = new char[1];

    const char* input = "FlexRAN SDK bblib_X version sdk-19.04-ea1-1-g3be2380";

    int ret = bblib_sdk_version(&output, &input, 1);

    ASSERT_EQ(ret, -1);

    delete[] output;
}

#ifdef _BBLIB_AVX2_
TEST(I16vec16Check, TestConstructors)
{
    auto a = _mm256_set_epi32(0x7, 0x6, 0x5, 0x4, 0x3, 0x2, 0x1, 0x0);

    I16vec16 b(a);

    __m256i c = b;

    ASSERT_TRUE(Equals(a, c));
}

TEST(I32vec8Check, TestConstructors)
{
    auto a = _mm256_set_epi32(0x7, 0x6, 0x5, 0x4, 0x3, 0x2, 0x1, 0x0);

    I32vec8 b(a);

    __m256i c = b;

    ASSERT_TRUE(Equals(a, c));
}

TEST(I32vec8Check, TestBroadcast)
{
    auto a = _mm256_set1_epi32(0x7);

    I32vec8 b(0x7);

    ASSERT_TRUE(Equals(a, b));
}

TEST(I32vec8Check, TestAddition)
{
    auto a = _mm256_set_epi32(0x7, 0x6, 0x5, 0x4, 0x3, 0x2, 0x1, 0x0);
    auto b = _mm256_set_epi32(0xe, 0xd, 0xc, 0xb, 0xa, 0x9, 0x8, 0x7);

    I32vec8 c(0x7);

    c += a;

    ASSERT_TRUE(Equals(c, b));
}

TEST(I8vec32Check, TestConstructors)
{
    auto a = _mm256_set1_epi32(0x7);

    I8vec32 b(a);

    __m256i c = b;

    ASSERT_TRUE(Equals(a, c));
}

TEST(I8vec32Check, TestSet)
{
    auto a = _mm256_setr_epi8(0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                              0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
                              0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
                              0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f);

    I8vec32 b(0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
              0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
              0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
              0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f);

    ASSERT_TRUE(Equals(a, b));
}
#endif
