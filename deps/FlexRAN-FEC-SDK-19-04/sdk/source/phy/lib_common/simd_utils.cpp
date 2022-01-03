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

#include <immintrin.h>
#include <dvec.h>

#include <complex>
#include <iostream>
#include <stdexcept>

#include "simd_utils.hpp"

#include "common_typedef_sdk.h"

inline static int NumRowStore(const int numRows, const int currentLoop) {
    if ((numRows - 8 * currentLoop) > 8) {
        return 8;
    } else {
        return numRows - 8 * currentLoop;
    }
}

static void StoreLo(F32vec8 *&outV, const int numStore, const int currentLoop,
                    const F32vec8 p_0, const F32vec8 p_1, const F32vec8 p_2, const F32vec8 p_3) {
    const auto outOffset = 8 * currentLoop;

    if (numStore > 0)
        outV[0 + outOffset] = p_0;

    if (numStore > 1)
        outV[1 + outOffset] = p_1;

    if (numStore > 2)
        outV[2 + outOffset] = p_2;

    if (numStore > 3)
        outV[3 + outOffset] = p_3;
}

static void StoreHi(F32vec8 *&outV, const int numStore, const int currentLoop,
                    const F32vec8 p_4, const F32vec8 p_5, const F32vec8 p_6, const F32vec8 p_7) {
    const auto outOffset = 8 * currentLoop;

    if (numStore > 4)
        outV[4 + outOffset] = p_4;

    if (numStore > 5)
        outV[5 + outOffset] = p_5;

    if (numStore > 6)
        outV[6 + outOffset] = p_6;

    if (numStore > 7)
        outV[7 + outOffset] = p_7;
}

static __m256i CreateMask(const size_t numRows, const int currentLoop) {
    int numStore = NumRowStore(numRows, currentLoop);

    __align(64) constexpr int32_t k_maskLut[72] = {
            0, 0, 0, 0, 0, 0, 0, 0,
            -1, 0, 0, 0, 0, 0, 0, 0,
            -1, -1, 0, 0, 0, 0, 0, 0,
            -1, -1, -1, 0, 0, 0, 0, 0,
            -1, -1, -1, -1, 0, 0, 0, 0,
            -1, -1, -1, -1, -1, 0, 0, 0,
            -1, -1, -1, -1, -1, -1, 0, 0,
            -1, -1, -1, -1, -1, -1, -1, 0,
            -1, -1, -1, -1, -1, -1, -1, -1
    };

    return _mm256_load_si256((const __m256i *) (k_maskLut + numStore * 8));
}

template<typename T, unsigned k_simdSize>
static void GenericPack(const T *input, T *output, size_t numElements) {
    size_t elementsPerBlock = numElements / k_simdSize;

    // Slow but sure way.
    for (size_t i = 0; i < elementsPerBlock; ++i) {
        for (size_t s = 0; s < k_simdSize; ++s) {
            output[i * k_simdSize + s] = input[s * elementsPerBlock + i];
        }
    }
}

template<typename T, unsigned k_simdSize>
static void GenericUnpack(const T *input, T *output, size_t numElements) {
    size_t elementsPerBlock = numElements / k_simdSize;

    // Slow but sure way.
    for (size_t i = 0; i < elementsPerBlock; ++i) {
        for (size_t s = 0; s < k_simdSize; ++s) {
            output[s * elementsPerBlock + i] = input[i * k_simdSize + s];
        }
    }
}

static void Pack16x4(const float *input, float *output, size_t numElements) {
    F32vec16 *inputV = (F32vec16 *) input;

    const auto lo_abcd = _mm512_unpacklo_ps(inputV[0], inputV[1]);
    const auto hi_abcd = _mm512_unpackhi_ps(inputV[0], inputV[1]);

    const auto lo_efgh = _mm512_unpacklo_ps(inputV[2], inputV[3]);
    const auto hi_efgh = _mm512_unpackhi_ps(inputV[2], inputV[3]);

    const auto k_idxLo = _mm512_setr_epi32(0, 4, 8, 12, 1, 5, 9, 13, 16, 20, 24, 28, 17, 21, 25,
                                           29);
    const auto k_idxHi = _mm512_setr_epi32(2, 6, 10, 14, 3, 7, 11, 15, 18, 22, 26, 30, 19, 23, 27,
                                           31);

    F32vec16 *outputV = (F32vec16 *) output;
    outputV[0] = _mm512_permutex2var_ps(lo_abcd, k_idxLo, lo_efgh);
    outputV[1] = _mm512_permutex2var_ps(lo_abcd, k_idxHi, lo_efgh);
    outputV[2] = _mm512_permutex2var_ps(hi_abcd, k_idxLo, hi_efgh);
    outputV[3] = _mm512_permutex2var_ps(hi_abcd, k_idxHi, hi_efgh);
}

static void Pack16x8(const float *input, float *output, size_t numElements) {
    F32vec16 *inputV = (F32vec16 *) input;
    F32vec16 *outputV = (F32vec16 *) output;

    const auto upLo0 = _mm512_unpacklo_ps(inputV[0], inputV[1]);
    const auto upLo1 = _mm512_unpacklo_ps(inputV[2], inputV[3]);
    const auto upLo2 = _mm512_unpacklo_ps(inputV[4], inputV[5]);
    const auto upLo3 = _mm512_unpacklo_ps(inputV[6], inputV[7]);

    const auto half04_0 = _mm512_unpacklo_ps(upLo0, upLo1);
    const auto half04_1 = _mm512_unpacklo_ps(upLo2, upLo3);

    const auto half15_0 = _mm512_unpackhi_ps(upLo0, upLo1);
    const auto half15_1 = _mm512_unpackhi_ps(upLo2, upLo3);

    const auto k_idx_even = _mm512_setr_epi32(0, 8, 2, 10, 1, 9, 3, 11, 16, 24, 18, 26, 17, 25, 19,
                                              27);
    const auto k_idx_old = _mm512_setr_epi32(4, 12, 6, 14, 5, 13, 7, 15, 20, 28, 22, 30, 21, 29, 23,
                                             31);

    outputV[0] = _mm512_permutex2var_ps(half04_0, k_idx_even, half04_1);
    outputV[4] = _mm512_permutex2var_ps(half04_0, k_idx_old, half04_1);

    outputV[1] = _mm512_permutex2var_ps(half15_0, k_idx_even, half15_1);
    outputV[5] = _mm512_permutex2var_ps(half15_0, k_idx_old, half15_1);

    const auto upHi0 = _mm512_unpackhi_ps(inputV[0], inputV[1]);
    const auto upHi1 = _mm512_unpackhi_ps(inputV[2], inputV[3]);
    const auto upHi2 = _mm512_unpackhi_ps(inputV[4], inputV[5]);
    const auto upHi3 = _mm512_unpackhi_ps(inputV[6], inputV[7]);

    const auto half26_0 = _mm512_unpacklo_ps(upHi0, upHi1);
    const auto half26_1 = _mm512_unpacklo_ps(upHi2, upHi3);

    const auto half37_0 = _mm512_unpackhi_ps(upHi0, upHi1);
    const auto half37_1 = _mm512_unpackhi_ps(upHi2, upHi3);

    outputV[2] = _mm512_permutex2var_ps(half26_0, k_idx_even, half26_1);
    outputV[6] = _mm512_permutex2var_ps(half26_0, k_idx_old, half26_1);

    outputV[3] = _mm512_permutex2var_ps(half37_0, k_idx_even, half37_1);
    outputV[7] = _mm512_permutex2var_ps(half37_0, k_idx_old, half37_1);
}

static void Transpose16x16(const float *input, float *output, size_t numElements) {
    F32vec16 *inputV = (F32vec16 *) input;
    F32vec16 *outputV = (F32vec16 *) output;

    auto CreateRows = [&](F32vec16 src0, F32vec16 src1, F32vec16 src2, F32vec16 src3,
                          F32vec16 &dst0, F32vec16 &dst1, F32vec16 &dst2, F32vec16 &dst3) {
        constexpr __mmask16 maskMid = 0b0000111111110000;
        constexpr __mmask16 maskFront = 0b1111000011110000;
        constexpr __mmask16 maskBack = 0b0000111100001111;

        const auto temp0 = _mm512_mask_shuffle_f32x4(src0, maskMid, src1, src2,
                                                     _MM_SHUFFLE(0, 0, 0, 0));
        const auto temp1 = _mm512_mask_shuffle_f32x4(src1, maskBack, src0, src2,
                                                     _MM_SHUFFLE(1, 1, 1, 1));
        const auto temp2 = _mm512_mask_shuffle_f32x4(src2, maskFront, src1, src0,
                                                     _MM_SHUFFLE(2, 2, 2, 2));
        const auto temp3 = _mm512_mask_shuffle_f32x4(src0, maskMid, src1, src2,
                                                     _MM_SHUFFLE(3, 3, 3, 3));

        const auto k_idx0 = _mm512_setr_epi32(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 16, 17, 18, 19);
        const auto k_idx1 = _mm512_setr_epi32(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 20, 21, 22, 23);
        const auto k_idx2 = _mm512_setr_epi32(12, 13, 14, 15, 4, 5, 6, 7, 8, 9, 10, 11, 24, 25, 26,
                                              27);
        const auto k_idx3 = _mm512_setr_epi32(12, 13, 14, 15, 4, 5, 6, 7, 8, 9, 10, 11, 28, 29, 30,
                                              31);

        dst0 = _mm512_permutex2var_ps(temp0, k_idx0, src3);
        dst1 = _mm512_permutex2var_ps(temp1, k_idx1, src3);
        dst2 = _mm512_permutex2var_ps(temp2, k_idx2, src3);
        dst3 = _mm512_permutex2var_ps(temp3, k_idx3, src3);
    };

    auto Do2UnpackLo = [&](int x, F32vec16 &result0, F32vec16 &result1) {
        result0 = _mm512_unpacklo_ps(inputV[x], inputV[x + 2]);
        result1 = _mm512_unpacklo_ps(inputV[x + 1], inputV[x + 3]);
    };

    auto Do2UnpackHi = [&](int x, F32vec16 &result0, F32vec16 &result1) {
        result0 = _mm512_unpackhi_ps(inputV[x], inputV[x + 2]);
        result1 = _mm512_unpackhi_ps(inputV[x + 1], inputV[x + 3]);
    };

    F32vec16 up_L0_1, up_L0_2;
    Do2UnpackLo(0, up_L0_1, up_L0_2);

    F32vec16 up_L0_3, up_L0_4;
    Do2UnpackLo(4, up_L0_3, up_L0_4);

    F32vec16 up_L0_5, up_L0_6;
    Do2UnpackLo(8, up_L0_5, up_L0_6);

    F32vec16 up_L0_7, up_L0_8;
    Do2UnpackLo(12, up_L0_7, up_L0_8);

    const F32vec16 R0 = _mm512_unpacklo_ps(up_L0_1, up_L0_2);
    const F32vec16 R1 = _mm512_unpacklo_ps(up_L0_3, up_L0_4);
    const F32vec16 R2 = _mm512_unpacklo_ps(up_L0_5, up_L0_6);
    const F32vec16 R3 = _mm512_unpacklo_ps(up_L0_7, up_L0_8);

    CreateRows(R0, R1, R2, R3, outputV[0], outputV[4], outputV[8], outputV[12]);

    const F32vec16 R4 = _mm512_unpackhi_ps(up_L0_1, up_L0_2);
    const F32vec16 R5 = _mm512_unpackhi_ps(up_L0_3, up_L0_4);
    const F32vec16 R6 = _mm512_unpackhi_ps(up_L0_5, up_L0_6);
    const F32vec16 R7 = _mm512_unpackhi_ps(up_L0_7, up_L0_8);

    CreateRows(R4, R5, R6, R7, outputV[1], outputV[5], outputV[9], outputV[13]);


    F32vec16 up_H0_1, up_H0_2;
    Do2UnpackHi(0, up_H0_1, up_H0_2);

    F32vec16 up_H0_3, up_H0_4;
    Do2UnpackHi(4, up_H0_3, up_H0_4);

    F32vec16 up_H0_5, up_H0_6;
    Do2UnpackHi(8, up_H0_5, up_H0_6);

    F32vec16 up_H0_7, up_H0_8;
    Do2UnpackHi(12, up_H0_7, up_H0_8);

    const F32vec16 R8 = _mm512_unpacklo_ps(up_H0_1, up_H0_2);
    const F32vec16 R9 = _mm512_unpacklo_ps(up_H0_3, up_H0_4);
    const F32vec16 R10 = _mm512_unpacklo_ps(up_H0_5, up_H0_6);
    const F32vec16 R11 = _mm512_unpacklo_ps(up_H0_7, up_H0_8);

    CreateRows(R8, R9, R10, R11, outputV[2], outputV[6], outputV[10], outputV[14]);

    const F32vec16 R12 = _mm512_unpackhi_ps(up_H0_1, up_H0_2);
    const F32vec16 R13 = _mm512_unpackhi_ps(up_H0_3, up_H0_4);
    const F32vec16 R14 = _mm512_unpackhi_ps(up_H0_5, up_H0_6);
    const F32vec16 R15 = _mm512_unpackhi_ps(up_H0_7, up_H0_8);

    CreateRows(R12, R13, R14, R15, outputV[3], outputV[7], outputV[11], outputV[15]);
}

static void Unpack16x4(const float *input, float *output, size_t numElements) {
    F32vec16 *inputV = (F32vec16 *) input;

    const auto k_idxLo = _mm512_setr_epi32(0, 16, 4, 20, 1, 17, 5, 21, 2, 18, 6, 22, 3, 19, 7, 23);
    const auto k_idxHi = _mm512_setr_epi32(8, 24, 12, 28, 9, 25, 13, 29, 10, 26, 14, 30, 11, 27, 15,
                                           31);

    const auto temp0 = _mm512_permutex2var_ps(inputV[0], k_idxLo, inputV[2]);
    const auto temp1 = _mm512_permutex2var_ps(inputV[1], k_idxLo, inputV[3]);
    const auto temp2 = _mm512_permutex2var_ps(inputV[0], k_idxHi, inputV[2]);
    const auto temp3 = _mm512_permutex2var_ps(inputV[1], k_idxHi, inputV[3]);

    F32vec16 *outputV = (F32vec16 *) output;
    outputV[0] = _mm512_unpacklo_ps(temp0, temp1);
    outputV[1] = _mm512_unpackhi_ps(temp0, temp1);
    outputV[2] = _mm512_unpacklo_ps(temp2, temp3);
    outputV[3] = _mm512_unpackhi_ps(temp2, temp3);
}

static void Unpack16x8(const float *input, float *output, size_t numElements) {
    F32vec16 *inputV = (F32vec16 *) input;
    F32vec16 *outputV = (F32vec16 *) output;

    auto CreateRows = [&](F32vec16 src0, F32vec16 src1, F32vec16 src2, F32vec16 src3,
                          F32vec16 &dst0, F32vec16 &dst1, F32vec16 &dst2, F32vec16 &dst3) {
        constexpr __mmask16 maskMid = 0b0000111111110000;
        constexpr __mmask16 maskFront = 0b1111000011110000;
        constexpr __mmask16 maskBack = 0b0000111100001111;

        const auto temp0 = _mm512_mask_shuffle_f32x4(src0, maskMid, src1, src2,
                                                     _MM_SHUFFLE(0, 0, 0, 0));
        const auto temp1 = _mm512_mask_shuffle_f32x4(src1, maskBack, src0, src2,
                                                     _MM_SHUFFLE(1, 1, 1, 1));
        const auto temp2 = _mm512_mask_shuffle_f32x4(src2, maskFront, src1, src0,
                                                     _MM_SHUFFLE(2, 2, 2, 2));
        const auto temp3 = _mm512_mask_shuffle_f32x4(src0, maskMid, src1, src2,
                                                     _MM_SHUFFLE(3, 3, 3, 3));

        const auto k_idx0 = _mm512_setr_epi32(0, 1, 4, 5, 8, 9, 16, 17, 2, 3, 6, 7, 10, 11, 18, 19);
        const auto k_idx1 = _mm512_setr_epi32(0, 1, 4, 5, 8, 9, 20, 21, 2, 3, 6, 7, 10, 11, 22, 23);
        const auto k_idx2 = _mm512_setr_epi32(12, 13, 4, 5, 8, 9, 24, 25, 14, 15, 6, 7, 10, 11, 26,
                                              27);
        const auto k_idx3 = _mm512_setr_epi32(12, 13, 4, 5, 8, 9, 28, 29, 14, 15, 6, 7, 10, 11, 30,
                                              31);

        dst0 = _mm512_permutex2var_ps(temp0, k_idx0, src3);
        dst1 = _mm512_permutex2var_ps(temp1, k_idx1, src3);
        dst2 = _mm512_permutex2var_ps(temp2, k_idx2, src3);
        dst3 = _mm512_permutex2var_ps(temp3, k_idx3, src3);
    };

    const auto r0 = _mm512_unpacklo_ps(inputV[0], inputV[1]);
    const auto r1 = _mm512_unpacklo_ps(inputV[2], inputV[3]);
    const auto r2 = _mm512_unpacklo_ps(inputV[4], inputV[5]);
    const auto r3 = _mm512_unpacklo_ps(inputV[6], inputV[7]);
    CreateRows(r0, r1, r2, r3, outputV[0], outputV[2], outputV[4], outputV[6]);

    const auto r4 = _mm512_unpackhi_ps(inputV[0], inputV[1]);
    const auto r5 = _mm512_unpackhi_ps(inputV[2], inputV[3]);
    const auto r6 = _mm512_unpackhi_ps(inputV[4], inputV[5]);
    const auto r7 = _mm512_unpackhi_ps(inputV[6], inputV[7]);
    CreateRows(r4, r5, r6, r7, outputV[1], outputV[3], outputV[5], outputV[7]);
}

static void Pack8xN(const float *input, float *output, size_t numElements) {
    const size_t numRows = numElements / 8;
    F32vec8 *outV = (F32vec8 *) output;
    const int numLoops = (numRows + 7) / 8;

    for (int currentLoop = 0; currentLoop < numLoops; currentLoop++) {
        auto Src = [&](int index) {
            const int offset = 8 * currentLoop + numRows * index;
            return _mm256_load_ps(input + offset);
        };

        constexpr int k_laneSwap_2_1 = 0b00100000;
        constexpr int k_laneSwap_1_2 = 0b00110001;

        const auto t_0 = _mm256_unpacklo_ps(Src(0), Src(1));
        const auto t_1 = _mm256_unpackhi_ps(Src(0), Src(1));
        const auto t_2 = _mm256_unpacklo_ps(Src(2), Src(3));
        const auto t_3 = _mm256_unpackhi_ps(Src(2), Src(3));
        const auto t_4 = _mm256_unpacklo_ps(Src(4), Src(5));
        const auto t_5 = _mm256_unpackhi_ps(Src(4), Src(5));
        const auto t_6 = _mm256_unpacklo_ps(Src(6), Src(7));
        const auto t_7 = _mm256_unpackhi_ps(Src(6), Src(7));

        const auto s_0 = _mm256_shuffle_ps(t_0, t_2, _MM_SHUFFLE(1, 0, 1, 0));
        const auto s_1 = _mm256_shuffle_ps(t_0, t_2, _MM_SHUFFLE(3, 2, 3, 2));
        const auto s_2 = _mm256_shuffle_ps(t_1, t_3, _MM_SHUFFLE(1, 0, 1, 0));
        const auto s_3 = _mm256_shuffle_ps(t_1, t_3, _MM_SHUFFLE(3, 2, 3, 2));
        const auto s_4 = _mm256_shuffle_ps(t_4, t_6, _MM_SHUFFLE(1, 0, 1, 0));
        const auto s_5 = _mm256_shuffle_ps(t_4, t_6, _MM_SHUFFLE(3, 2, 3, 2));
        const auto s_6 = _mm256_shuffle_ps(t_5, t_7, _MM_SHUFFLE(1, 0, 1, 0));
        const auto s_7 = _mm256_shuffle_ps(t_5, t_7, _MM_SHUFFLE(3, 2, 3, 2));

        const auto p_0 = _mm256_permute2f128_ps(s_0, s_4, k_laneSwap_2_1);
        const auto p_1 = _mm256_permute2f128_ps(s_1, s_5, k_laneSwap_2_1);
        const auto p_2 = _mm256_permute2f128_ps(s_2, s_6, k_laneSwap_2_1);
        const auto p_3 = _mm256_permute2f128_ps(s_3, s_7, k_laneSwap_2_1);
        const auto p_4 = _mm256_permute2f128_ps(s_0, s_4, k_laneSwap_1_2);
        const auto p_5 = _mm256_permute2f128_ps(s_1, s_5, k_laneSwap_1_2);
        const auto p_6 = _mm256_permute2f128_ps(s_2, s_6, k_laneSwap_1_2);
        const auto p_7 = _mm256_permute2f128_ps(s_3, s_7, k_laneSwap_1_2);

        const int numStore = NumRowStore(numRows, currentLoop);

        StoreLo(outV, numStore, currentLoop, p_0, p_1, p_2, p_3);
        StoreHi(outV, numStore, currentLoop, p_4, p_5, p_6, p_7);
    }
}


static void Pack64xN(const float *input, float *output, size_t numElements) {
    F32vec8 *outV = (F32vec8 *) output;
    F32vec8 *inV = (F32vec8 *) input;
    const int numLoops = numElements / 64;

    for (int currentLoop = 0; currentLoop < numLoops; currentLoop++) {
        auto Src = [&](int x) {
            return inV[x * numLoops + currentLoop];
        };

        const auto t_0 = _mm256_unpacklo_ps(Src(0), Src(1));
        const auto t_1 = _mm256_unpackhi_ps(Src(0), Src(1));
        const auto t_2 = _mm256_unpacklo_ps(Src(2), Src(3));
        const auto t_3 = _mm256_unpackhi_ps(Src(2), Src(3));
        const auto t_4 = _mm256_unpacklo_ps(Src(4), Src(5));
        const auto t_5 = _mm256_unpackhi_ps(Src(4), Src(5));
        const auto t_6 = _mm256_unpacklo_ps(Src(6), Src(7));
        const auto t_7 = _mm256_unpackhi_ps(Src(6), Src(7));

        const auto s_0 = _mm256_shuffle_ps(t_0, t_2, _MM_SHUFFLE(1, 0, 1, 0));
        const auto s_1 = _mm256_shuffle_ps(t_0, t_2, _MM_SHUFFLE(3, 2, 3, 2));
        const auto s_2 = _mm256_shuffle_ps(t_1, t_3, _MM_SHUFFLE(1, 0, 1, 0));
        const auto s_3 = _mm256_shuffle_ps(t_1, t_3, _MM_SHUFFLE(3, 2, 3, 2));
        const auto s_4 = _mm256_shuffle_ps(t_4, t_6, _MM_SHUFFLE(1, 0, 1, 0));
        const auto s_5 = _mm256_shuffle_ps(t_4, t_6, _MM_SHUFFLE(3, 2, 3, 2));
        const auto s_6 = _mm256_shuffle_ps(t_5, t_7, _MM_SHUFFLE(1, 0, 1, 0));
        const auto s_7 = _mm256_shuffle_ps(t_5, t_7, _MM_SHUFFLE(3, 2, 3, 2));

        constexpr int k_laneSwap_2_1 = 0b00100000;
        constexpr int k_laneSwap_1_2 = 0b00110001;

        auto Dst = [&](int x) -> F32vec8 & {
            return outV[x + 8 * currentLoop];
        };

        Dst(0) = _mm256_permute2f128_ps(s_0, s_4, k_laneSwap_2_1);
        Dst(1) = _mm256_permute2f128_ps(s_1, s_5, k_laneSwap_2_1);
        Dst(2) = _mm256_permute2f128_ps(s_2, s_6, k_laneSwap_2_1);
        Dst(3) = _mm256_permute2f128_ps(s_3, s_7, k_laneSwap_2_1);
        Dst(4) = _mm256_permute2f128_ps(s_0, s_4, k_laneSwap_1_2);
        Dst(5) = _mm256_permute2f128_ps(s_1, s_5, k_laneSwap_1_2);
        Dst(6) = _mm256_permute2f128_ps(s_2, s_6, k_laneSwap_1_2);
        Dst(7) = _mm256_permute2f128_ps(s_3, s_7, k_laneSwap_1_2);
    }
}

static void Unpack8xN(const float *input, float *output, size_t numElements) {
    const size_t numRows = numElements / 8;
    const int numLoops = (numRows + 7) / 8;

    for (int currentLoop = 0; currentLoop < numLoops; currentLoop++) {
        auto Src = [&](int index) {
            const auto offset = (index * 8 + (64 * currentLoop));
            return _mm256_load_ps(input + offset);
        };

        const auto t_0 = _mm256_unpacklo_ps(Src(0), Src(1));
        const auto t_1 = _mm256_unpackhi_ps(Src(0), Src(1));
        const auto t_2 = _mm256_unpacklo_ps(Src(2), Src(3));
        const auto t_3 = _mm256_unpackhi_ps(Src(2), Src(3));
        const auto t_4 = _mm256_unpacklo_ps(Src(4), Src(5));
        const auto t_5 = _mm256_unpackhi_ps(Src(4), Src(5));
        const auto t_6 = _mm256_unpacklo_ps(Src(6), Src(7));
        const auto t_7 = _mm256_unpackhi_ps(Src(6), Src(7));

        const auto s_0 = _mm256_shuffle_ps(t_0, t_2, _MM_SHUFFLE(1, 0, 1, 0));
        const auto s_1 = _mm256_shuffle_ps(t_0, t_2, _MM_SHUFFLE(3, 2, 3, 2));
        const auto s_2 = _mm256_shuffle_ps(t_1, t_3, _MM_SHUFFLE(1, 0, 1, 0));
        const auto s_3 = _mm256_shuffle_ps(t_1, t_3, _MM_SHUFFLE(3, 2, 3, 2));
        const auto s_4 = _mm256_shuffle_ps(t_4, t_6, _MM_SHUFFLE(1, 0, 1, 0));
        const auto s_5 = _mm256_shuffle_ps(t_4, t_6, _MM_SHUFFLE(3, 2, 3, 2));
        const auto s_6 = _mm256_shuffle_ps(t_5, t_7, _MM_SHUFFLE(1, 0, 1, 0));
        const auto s_7 = _mm256_shuffle_ps(t_5, t_7, _MM_SHUFFLE(3, 2, 3, 2));

        constexpr int k_laneSwap_2_1 = 0b00100000;
        constexpr int k_laneSwap_1_2 = 0b00110001;

        const auto p_0 = _mm256_permute2f128_ps(s_0, s_4, k_laneSwap_2_1);
        const auto p_1 = _mm256_permute2f128_ps(s_1, s_5, k_laneSwap_2_1);
        const auto p_2 = _mm256_permute2f128_ps(s_2, s_6, k_laneSwap_2_1);
        const auto p_3 = _mm256_permute2f128_ps(s_3, s_7, k_laneSwap_2_1);
        const auto p_4 = _mm256_permute2f128_ps(s_0, s_4, k_laneSwap_1_2);
        const auto p_5 = _mm256_permute2f128_ps(s_1, s_5, k_laneSwap_1_2);
        const auto p_6 = _mm256_permute2f128_ps(s_2, s_6, k_laneSwap_1_2);
        const auto p_7 = _mm256_permute2f128_ps(s_3, s_7, k_laneSwap_1_2);

        auto mask = CreateMask(numRows, currentLoop);

        auto MaskStore = [&](F32vec8 vector, int index) {
            const auto outOffset = (8 * currentLoop + numRows * index);
            _mm256_maskstore_ps(output + outOffset, mask, vector);
        };

        MaskStore(p_0, 0);
        MaskStore(p_1, 1);
        MaskStore(p_2, 2);
        MaskStore(p_3, 3);
        MaskStore(p_4, 4);
        MaskStore(p_5, 5);
        MaskStore(p_6, 6);
        MaskStore(p_7, 7);
    }
}

static void Unpack64xN(const float *input, float *output, size_t numElements) {
    F32vec8 *outV = (F32vec8 *) output;
    F32vec8 *inV = (F32vec8 *) input;
    const int numLoops = numElements / 64;

    for (int currentLoop = 0; currentLoop < numLoops; currentLoop++) {
        auto Src = [&](int x) {
            return inV[x + 8 * currentLoop];
        };

        const auto t_0 = _mm256_unpacklo_ps(Src(0), Src(1));
        const auto t_1 = _mm256_unpackhi_ps(Src(0), Src(1));
        const auto t_2 = _mm256_unpacklo_ps(Src(2), Src(3));
        const auto t_3 = _mm256_unpackhi_ps(Src(2), Src(3));
        const auto t_4 = _mm256_unpacklo_ps(Src(4), Src(5));
        const auto t_5 = _mm256_unpackhi_ps(Src(4), Src(5));
        const auto t_6 = _mm256_unpacklo_ps(Src(6), Src(7));
        const auto t_7 = _mm256_unpackhi_ps(Src(6), Src(7));

        const auto s_0 = _mm256_shuffle_ps(t_0, t_2, _MM_SHUFFLE(1, 0, 1, 0));
        const auto s_1 = _mm256_shuffle_ps(t_0, t_2, _MM_SHUFFLE(3, 2, 3, 2));
        const auto s_2 = _mm256_shuffle_ps(t_1, t_3, _MM_SHUFFLE(1, 0, 1, 0));
        const auto s_3 = _mm256_shuffle_ps(t_1, t_3, _MM_SHUFFLE(3, 2, 3, 2));
        const auto s_4 = _mm256_shuffle_ps(t_4, t_6, _MM_SHUFFLE(1, 0, 1, 0));
        const auto s_5 = _mm256_shuffle_ps(t_4, t_6, _MM_SHUFFLE(3, 2, 3, 2));
        const auto s_6 = _mm256_shuffle_ps(t_5, t_7, _MM_SHUFFLE(1, 0, 1, 0));
        const auto s_7 = _mm256_shuffle_ps(t_5, t_7, _MM_SHUFFLE(3, 2, 3, 2));

        auto Dst = [&](int x) -> F32vec8 & {
            return outV[x * numLoops + currentLoop];
        };

        constexpr int k_laneSwap_2_1 = 0b00100000;
        constexpr int k_laneSwap_1_2 = 0b00110001;

        Dst(0) = _mm256_permute2f128_ps(s_0, s_4, k_laneSwap_2_1);
        Dst(1) = _mm256_permute2f128_ps(s_1, s_5, k_laneSwap_2_1);
        Dst(2) = _mm256_permute2f128_ps(s_2, s_6, k_laneSwap_2_1);
        Dst(3) = _mm256_permute2f128_ps(s_3, s_7, k_laneSwap_2_1);
        Dst(4) = _mm256_permute2f128_ps(s_0, s_4, k_laneSwap_1_2);
        Dst(5) = _mm256_permute2f128_ps(s_1, s_5, k_laneSwap_1_2);
        Dst(6) = _mm256_permute2f128_ps(s_2, s_6, k_laneSwap_1_2);
        Dst(7) = _mm256_permute2f128_ps(s_3, s_7, k_laneSwap_1_2);
    }
}

template<typename T>
void PackSimd8(const T *input, T *output, size_t numElements) {
    if (numElements % 8 != 0)
        throw std::runtime_error("Expected input to be divisible by 8");

    if (numElements % 64 == 0) {
        Pack64xN(input, output, numElements);
    } else {
        Pack8xN(input, output, numElements);
    }
}

template<typename T>
void UnpackSimd8(const T *input, T *output, size_t numElements) {
    if (numElements % 8 != 0)
        throw std::runtime_error("Expected input to be divisible by 8");

    if (numElements % 64 == 0) {
        Unpack64xN(input, output, numElements);
    } else {
        Unpack8xN(input, output, numElements);
    }
}

template<typename T>
void PackSimd16(const T *input, T *output, size_t numElements) {
    constexpr unsigned k_simdSize = 16;
    if (numElements % k_simdSize != 0) {
        throw std::runtime_error("Expected input to be divisible by 16");
    } else if (numElements == 16 * 4) {
        Pack16x4(input, output, numElements);
    } else if (numElements == 16 * 8) {
        Pack16x8(input, output, numElements);
    } else if (numElements == 16 * 16) {
        Transpose16x16(input, output, numElements);
    } else {
        GenericPack<T, 16>(input, output, numElements);
    }
}

template<typename T>
void UnpackSimd16(const T *input, T *output, size_t numElements) {
    const unsigned k_simdSize = 16;

    if (numElements % k_simdSize != 0) {
        throw std::runtime_error("Expected input to be divisible by 16");
    } else if (numElements == 16 * 4) {
        Unpack16x4(input, output, numElements);
    } else if (numElements == 16 * 8) {
        Unpack16x8(input, output, numElements);
    } else if (numElements == 16 * 16) {
        Transpose16x16(input, output, numElements);
    } else {
        GenericUnpack<T, 16>(input, output, numElements);
    }
}

template
void PackSimd8(const float *input, float *output, size_t numElements);

template
void UnpackSimd8(const float *input, float *output, size_t numElements);

template
void PackSimd16(const float *input, float *output, size_t numElements);

template
void UnpackSimd16(const float *input, float *output, size_t numElements);
