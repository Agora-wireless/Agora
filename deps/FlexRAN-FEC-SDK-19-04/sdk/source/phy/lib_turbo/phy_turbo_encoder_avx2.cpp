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

/*
* @file   phy_turbo_encoder_avx2.cpp
* @brief  turbo encoder
*/

#include <cstdint>

#include <immintrin.h>

#include "common_typedef_sdk.h"

#include "phy_turbo.h"
#include "phy_turbo_internal.h"
#if defined (_BBLIB_AVX2_) || defined (_BBLIB_AVX512_)
extern __align(64) uint16_t g_OutputWinTable8_sdk[256][8];
extern __align(64) uint8_t g_TailWinTable_sdk[8];

/* const for 128-bit processing */
extern __m128i qp128;

__m256i shuffleMask256 = _mm256_setr_epi8(15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0);
extern __m128i shuffleMask;

extern __m128i Mask64;

extern __m128i g18;

/*!
    \brief turbo encoder internal function for 128 bit wide date process kernel
 */
inline __m128i a_fun(__m128i cw)
{
    __m128i at1, at2, at3, at4, a;
    at1 = _mm_clmulepi64_si128(cw, qp128, 0x11);
    at2 = _mm_clmulepi64_si128(cw, qp128, 0x01);
    at3 = _mm_clmulepi64_si128(cw, qp128, 0x10);
    at4 = _mm_xor_si128(at2, at3);
    at4 = _mm_srli_si128(at4, 8);
    a = _mm_xor_si128(at1, at4);
    a = _mm_xor_si128(a, cw);
    return a;
}

#define unlikely_local(x)     __builtin_expect(!!(x), 0)

/*!
    \brief Init turbo encoder
 */
struct init_turbo_encoder_avx2
{
    init_turbo_encoder_avx2()
    {

    bblib_print_turbo_version();

    }
};

init_turbo_encoder_avx2 do_constructor_turbo_encoder_avx2;

/**
 ******************************************************************************
 * @fn int32_t bblib_lte_turbo_encoder_avx2(const struct bblib_turbo_encoder_request *request,
    struct bblib_turbo_encoder_response *response)
 * @brief turbo encoder for LTE
 *******************************************************************************/
int32_t
bblib_lte_turbo_encoder_avx2(const struct bblib_turbo_encoder_request *request,
    struct bblib_turbo_encoder_response *response)
{
    __align(64) uint8_t input_win_2[MAX_DATA_LEN_INTERLEAVE*4];

    bblib_lte_turbo_interleaver_8windows_sse(request->case_id, request->input_win,
                                             input_win_2);

    __m256i cw0, cw1, b0, b1, x0, x1, y0, y1, yt0, yt1, yr0, yr1;
    __m128i cw0_tail, cw1_tail, b0_tail, b1_tail, a0_tail, a1_tail, x0_tail, x1_tail;
    __m128i yt00, yt01, yt10, yt11, yr00, yr01, yr10, yr11, a00, a01, a10, a11, y00, y01, y10, y11;
    __m128i cw00, cw01, cw10, cw11, x00, x01, x10, x11, b00, b10;
    __m256i a0 = { 0 };
    __m256i a1 = { 0 };
    a01 = _mm_setzero_si128();
    a11 = _mm_setzero_si128();
    int32_t len256, lens, idx;
    int8_t bt0 = 0, bt1 = 0, bt2 = 0;
    int8_t b80, b81, yr80, yr81;

    uint8_t State0, State1, Tail0, Tail1, Tmp1, Tmp2;
    uint16_t TmpShort1, TmpShort2;

    len256 = request->length & 0xFFFFFFE0;
    if (unlikely_local(len256 > 0))
    {
        b0 = _mm256_setzero_si256();
        yr0 = _mm256_setzero_si256();
        b1 = _mm256_setr_epi8(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x20);
        yr1 = _mm256_setzero_si256();
        b00 = _mm_setzero_si128();
        b10 = _mm_setzero_si128();
        for (idx = 0; idx < len256; idx += 32)
        {
            cw0 = _mm256_loadu_si256((__m256i *)(request->input_win + idx));
            cw1 = _mm256_loadu_si256((__m256i *)(input_win_2 + idx));
            _mm256_storeu_si256((__m256i *)(response->output_win_0 + idx), cw0);
            cw0 = _mm256_shuffle_epi8(cw0, shuffleMask256);
            cw1 = _mm256_shuffle_epi8(cw1, shuffleMask256);

            cw00 = _mm256_extracti128_si256(cw0, 0);
            cw10 = _mm256_extracti128_si256(cw1, 0);
            cw00 = _mm_xor_si128(cw00, b00);
            cw10 = _mm_xor_si128(cw10, b10);
            a00 = a_fun(cw00);
            a10 = a_fun(cw10);
            x00 = _mm_slli_si128(a00, 15);
            x10 = _mm_slli_si128(a10, 15);
            b00 = _mm_slli_epi64(x00, 1);
            b10 = _mm_slli_epi64(x10, 1);
            b00 = _mm_xor_si128(b00, x00);
            b10 = _mm_xor_si128(b10, x10);
            b00 = _mm_slli_epi64(b00, 5);
            b10 = _mm_slli_epi64(b10, 5);

            cw01 = _mm256_extracti128_si256(cw0, 1);
            cw11 = _mm256_extracti128_si256(cw1, 1);
            cw01 = _mm_xor_si128(cw01, b00);
            cw11 = _mm_xor_si128(cw11, b10);
            a01 = a_fun(cw01);
            a11 = a_fun(cw11);
            x01 = _mm_slli_si128(a01, 15);
            x11 = _mm_slli_si128(a11, 15);
            b00 = _mm_slli_epi64(x01, 1);
            b10 = _mm_slli_epi64(x11, 1);
            b00 = _mm_xor_si128(b00, x01);
            b10 = _mm_xor_si128(b10, x11);
            b00 = _mm_slli_epi64(b00, 5);
            b10 = _mm_slli_epi64(b10, 5);

            b0 = _mm256_set_m128i(b00, b00);
            b1 = _mm256_set_m128i(b10, b10);

            yt00 = _mm_clmulepi64_si128(a00, g18, 0x11);
            yt10 = _mm_clmulepi64_si128(a10, g18, 0x11);
            yt00 = _mm_xor_si128(yt00, a00);
            yt10 = _mm_xor_si128(yt10, a10);
            y00 = _mm_clmulepi64_si128(a00, g18, 0x10);
            y10 = _mm_clmulepi64_si128(a10, g18, 0x10);

            yt01 = _mm_clmulepi64_si128(a01, g18, 0x11);
            yt11 = _mm_clmulepi64_si128(a11, g18, 0x11);
            yt01 = _mm_xor_si128(yt01, a01);
            yt11 = _mm_xor_si128(yt11, a11);
            y01 = _mm_clmulepi64_si128(a01, g18, 0x10);
            y11 = _mm_clmulepi64_si128(a11, g18, 0x10);

            x0 = _mm256_set_m128i(x01, x00);
            x1 = _mm256_set_m128i(x11, x10);

            y0 = _mm256_set_m128i(y01, y00);
            y1 = _mm256_set_m128i(y11, y10);

            yt0 = _mm256_set_m128i(yt01, yt00);
            yt1 = _mm256_set_m128i(yt11, yt10);

            y0 = _mm256_srli_si256(y0, 8);
            y1 = _mm256_srli_si256(y1, 8);
            yt0 = _mm256_xor_si256(yt0, y0);
            yt1 = _mm256_xor_si256(yt1, y1);

            yr00 = _mm256_extracti128_si256(yr0, 1);
            yr10 = _mm256_extracti128_si256(yr1, 1);

            yr01 = _mm_slli_epi64(x00, 2);
            yr11 = _mm_slli_epi64(x10, 2);
            yr01 = _mm_xor_si128(x00, yr01);
            yr11 = _mm_xor_si128(x10, yr11);
            yr01 = _mm_slli_epi64(yr01, 5);
            yr11 = _mm_slli_epi64(yr11, 5);
            yr0 = _mm256_set_m128i(yr01, yr00);
            yr1 = _mm256_set_m128i(yr11, yr10);

            yt0 = _mm256_xor_si256(yt0, yr0);
            yt1 = _mm256_xor_si256(yt1, yr1);
            yt0 = _mm256_shuffle_epi8(yt0, shuffleMask256);
            yt1 = _mm256_shuffle_epi8(yt1, shuffleMask256);

            _mm256_storeu_si256((__m256i *)(response->output_win_1 + idx), yt0);
            _mm256_storeu_si256((__m256i *)(response->output_win_2 + idx), yt1);

            yr0 = _mm256_slli_epi64(x0, 2);
            yr1 = _mm256_slli_epi64(x1, 2);
            yr0 = _mm256_xor_si256(x0, yr0);
            yr1 = _mm256_xor_si256(x1, yr1);
            yr0 = _mm256_slli_epi64(yr0, 5);
            yr1 = _mm256_slli_epi64(yr1, 5);
        }
    }

    lens = request->length - len256;

    if (unlikely_local(lens == 0))
    {
        /* tail processing */
        b80 = _mm256_extract_epi8(b0, 31);
        yr80 = _mm256_extract_epi8(yr0, 31);
        b81 = _mm256_extract_epi8(b1, 31);
        yr81 = _mm256_extract_epi8(yr1, 31);

        bt0 = bt1 = bt2 = 0;

        bt0 = bt0 | (b80 & 0x80);
        bt1 = bt1 | (yr80 & 0x80);
        bt2 = bt2 | ((b80 & 0x40) << 1);

        bt0 = bt0 | (yr80 & 0x40);
        bt1 = bt1 | ((b80 & 0x20) << 1);
        bt2 = bt2 | ((yr80 & 0x20) << 1);

        bt0 = bt0 | ((b81 & 0x80) >> 2);
        bt1 = bt1 | ((yr81 & 0x80) >> 2);
        bt2 = bt2 | ((b81 & 0x40) >> 1);

        bt0 = bt0 | ((yr81 & 0x40) >> 2);
        bt1 = bt1 | ((b81 & 0x20) >> 1);
        bt2 = bt2 | ((yr81 & 0x20) >> 1);

        *(response->output_win_0 + request->length) = bt0;
        *(response->output_win_1 + request->length) = bt1;
        *(response->output_win_2 + request->length) = bt2;
        return 0;
    }
    else
    {
        while (lens >= 64)
        {
            cw0_tail = _mm_loadu_si64(request->input_win + len256);
            cw1_tail = _mm_loadu_si64(input_win_2 + len256);
            _mm_storeu_si64(response->output_win_0 + len256, cw0_tail);
            cw0_tail = _mm_slli_si128(cw0_tail, 8);
            cw1_tail = _mm_slli_si128(cw1_tail, 8);
            cw0_tail = _mm_shuffle_epi8(cw0_tail, shuffleMask);
            cw1_tail = _mm_shuffle_epi8(cw1_tail, shuffleMask);

            b0_tail = _mm256_extracti128_si256(b0, 1);
            b1_tail = _mm256_extracti128_si256(b1, 1);
            a0_tail = _mm256_extracti128_si256(a0, 1);
            a1_tail = _mm256_extracti128_si256(a1, 1);

            cw0_tail = _mm_xor_si128(cw0_tail, b0_tail);
            cw1_tail = _mm_xor_si128(cw1_tail, b1_tail);

            x0_tail = _mm_clmulepi64_si128(a0_tail, qp128, 0x11);
            a0_tail = _mm_xor_si128(a0_tail, x0_tail);
            a0_tail = _mm_and_si128(a0_tail, Mask64);

            x1_tail = _mm_clmulepi64_si128(a1_tail, qp128, 0x11);
            a1_tail = _mm_xor_si128(a1_tail, x1_tail);
            a1_tail = _mm_and_si128(a1_tail, Mask64);

            yt01 = _mm_clmulepi64_si128(a0_tail, g18, 0x11);
            yt11 = _mm_clmulepi64_si128(a1_tail, g18, 0x11);
            yt01 = _mm_xor_si128(yt01, a0_tail);
            yt11 = _mm_xor_si128(yt11, a1_tail);

            yt01 = _mm_xor_si128(yt01, yr01);
            yt11 = _mm_xor_si128(yt11, yr11);
            yt01 = _mm_shuffle_epi8(yt01, shuffleMask);
            yt11 = _mm_shuffle_epi8(yt11, shuffleMask);

            _mm_storeu_si64(response->output_win_1 + len256, yt01);
            _mm_storeu_si64(response->output_win_2 + len256, yt11);

            State0 = _mm_extract_epi8(a0_tail, 8) & 0x7;
            State1 = _mm_extract_epi8(a1_tail, 8) & 0x7;

            len256 += 64;
            lens = request->length - len256;

            if (lens == 0)
            {
                /* tail processing */
                b80 = _mm_extract_epi8(b0_tail, 15);
                yr80 = _mm_extract_epi8(yr01, 15);
                b81 = _mm_extract_epi8(b1_tail, 15);
                yr81 = _mm_extract_epi8(yr11, 15);

                bt0 = bt1 = bt2 = 0;

                bt0 = bt0 | (b80 & 0x80);
                bt1 = bt1 | (yr80 & 0x80);
                bt2 = bt2 | ((b80 & 0x40) << 1);

                bt0 = bt0 | (yr80 & 0x40);
                bt1 = bt1 | ((b80 & 0x20) << 1);
                bt2 = bt2 | ((yr80 & 0x20) << 1);

                bt0 = bt0 | ((b81 & 0x80) >> 2);
                bt1 = bt1 | ((yr81 & 0x80) >> 2);
                bt2 = bt2 | ((b81 & 0x40) >> 1);

                bt0 = bt0 | ((yr81 & 0x40) >> 2);
                bt1 = bt1 | ((b81 & 0x20) >> 1);
                bt2 = bt2 | ((yr81 & 0x20) >> 1);

                *(response->output_win_0 + request->length) = bt0;
                *(response->output_win_1 + request->length) = bt1;
                *(response->output_win_2 + request->length) = bt2;

                return 0;
            }
            else
            {
                State0 = _mm_extract_epi8(a0_tail, 8) & 0x7;
                State1 = _mm_extract_epi8(a1_tail, 8) & 0x7;
            }
        }
        State0 = _mm_extract_epi8(a01, 0) & 0x7;
        State1 = _mm_extract_epi8(a11, 0) & 0x7;

        for (idx = len256; idx < request->length; idx++)
        {
            Tmp1 = *(request->input_win + idx);
            Tmp2 = *(input_win_2 + idx);

            TmpShort1 = g_OutputWinTable8_sdk[Tmp1][State0];
            TmpShort2 = g_OutputWinTable8_sdk[Tmp2][State1];

            *(response->output_win_0 + idx) = Tmp1;
            *(response->output_win_1 + idx) = (uint8_t)(TmpShort1 >> 8);
            *(response->output_win_2 + idx) = (uint8_t)(TmpShort2 >> 8);

            State0 = (uint8_t)(TmpShort1 & 0x07);
            State1 = (uint8_t)(TmpShort2 & 0x07);
        }

        Tail0 = g_TailWinTable_sdk[State0];
        Tail1 = g_TailWinTable_sdk[State1];
        Tail1 = Tail1 >> 2;

        *(response->output_win_0 + request->length) = (Tail0 & (128 + 64)) + (Tail1 & (32 + 16));
        Tail0 = Tail0 << 2;
        Tail1 = Tail1 << 2;
        *(response->output_win_1 + request->length) = (Tail0 & (128 + 64)) + (Tail1 & (32 + 16));
        Tail0 = Tail0 << 2;
        Tail1 = Tail1 << 2;
        *(response->output_win_2 + request->length) = (Tail0 & (128 + 64)) + (Tail1 & (32 + 16));

    }

    return 0;
}
#else
int32_t
bblib_lte_turbo_encoder_avx2(const struct bblib_turbo_encoder_request *request,
    struct bblib_turbo_encoder_response *response)
{
    printf("bblib_turbo requires AVX2 ISA support to run\n");
    return(-1);
}
#endif