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
 * @file   phy_turbo_encoder_avx512.cpp
 * @brief  turbo encoder 
*/

#include <cstdint>

#include <immintrin.h>

#include "common_typedef_sdk.h"

#include "phy_turbo.h"
#include "phy_turbo_internal.h"
#if defined (_BBLIB_AVX512_)
extern __align(64) uint16_t g_OutputWinTable8_sdk[256][8];
extern __align(64) uint8_t g_TailWinTable_sdk[8];

/* const for 512-bit processing */
extern __m128i qp128;
extern __m128i g18;

__m512i shuffleMask512 = _mm512_set_epi32( 0x00010203, 0x04050607, 0x08090a0b, 0x0c0d0e0f, 
0x00010203, 0x04050607, 0x08090a0b, 0x0c0d0e0f, 
0x00010203, 0x04050607, 0x08090a0b, 0x0c0d0e0f, 
0x00010203, 0x04050607, 0x08090a0b, 0x0c0d0e0f);

extern __m128i shuffleMask;

extern __m128i Mask64;

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

struct init_turbo_encoder_avx512
{
    init_turbo_encoder_avx512()
    {
            
    bblib_print_turbo_version();

    }
};

init_turbo_encoder_avx512 do_constructor_turbo_encoder_avx512;

int32_t
bblib_lte_turbo_encoder_avx512(const struct bblib_turbo_encoder_request *request,
    struct bblib_turbo_encoder_response *response)
{
    __align(64) uint8_t input_win_2[MAX_DATA_LEN_INTERLEAVE*4];

    bblib_lte_turbo_interleaver_8windows_sse(request->case_id, request->input_win,
                                            input_win_2);
    
    __m512i cw0, cw1;
    __m128i *ptr0, *ptr1;
    __m128i tmp_0;
    __m128i tmp_1;
    __m128i b0, b1, yt0, yt1, yr0, yr1, x0, x1, y0, y1;
    __m128i a0 = {0};
    __m128i a1 = {0};
    int32_t len512, lens, idx;
    int8_t bt0 = 0, bt1 = 0, bt2 = 0;
    int8_t b80, b81, yr80, yr81;

    uint8_t State0, State1, Tail0, Tail1, Tmp1, Tmp2;
    uint16_t TmpShort1, TmpShort2;

    len512 = request->length & 0xFFFFFFC0;
    if (unlikely_local(len512 > 0)) 
    {
        b0 = _mm_setzero_si128();
        yr0 = _mm_setzero_si128();
        b1 = _mm_setzero_si128();
        yr1 = _mm_setzero_si128();
        for (idx = 0; idx < len512; idx+= 64) 
        {
            cw0 = _mm512_loadu_si512((__m512i *)(request->input_win + idx));
            cw1 = _mm512_loadu_si512((__m512i *)(input_win_2 + idx));
            _mm512_storeu_si512((__m512i *)(response->output_win_0 + idx), cw0);
            cw0 = _mm512_shuffle_epi8 (cw0, shuffleMask512);
            cw1 = _mm512_shuffle_epi8 (cw1, shuffleMask512);
            
            ptr0 = (__m128i *)(&cw0);
            ptr1 = (__m128i *)(&cw1);
            
            ptr0[0] = _mm_xor_si128(ptr0[0], b0);
            ptr1[0] = _mm_xor_si128(ptr1[0], b1);

            a0 = a_fun(ptr0[0]);
            a1 = a_fun(ptr1[0]);

            x0 = _mm_slli_si128(a0, 15);
            x1 = _mm_slli_si128(a1, 15);
            b0 = _mm_slli_epi64(x0, 1);
            b1 = _mm_slli_epi64(x1, 1);
            b0 = _mm_xor_si128(b0, x0);
            b1 = _mm_xor_si128(b1, x1);
            b0 = _mm_slli_epi64(b0, 5);
            b1 = _mm_slli_epi64(b1, 5);

            yt0 = _mm_clmulepi64_si128(a0, g18, 0x11);
            yt1 = _mm_clmulepi64_si128(a1, g18, 0x11);
            yt0 = _mm_xor_si128(yt0, a0);
            yt1 = _mm_xor_si128(yt1, a1);
            y0 = _mm_clmulepi64_si128(a0, g18, 0x10);
            y1 = _mm_clmulepi64_si128(a1, g18, 0x10);
            y0 = _mm_srli_si128(y0, 8);
            y1 = _mm_srli_si128(y1, 8);
            yt0 = _mm_xor_si128(yt0, y0);
            yt1 = _mm_xor_si128(yt1, y1);

            yt0 = _mm_xor_si128(yt0, yr0);
            yt1 = _mm_xor_si128(yt1, yr1);
            ptr0[0] = _mm_shuffle_epi8(yt0, shuffleMask);
            ptr1[0] = _mm_shuffle_epi8(yt1, shuffleMask);

            yr0 = _mm_slli_epi64(x0, 2);
            yr1 = _mm_slli_epi64(x1, 2);
            yr0 = _mm_xor_si128(x0, yr0);
            yr1 = _mm_xor_si128(x1, yr1);
            yr0 = _mm_slli_epi64(yr0, 5);
            yr1 = _mm_slli_epi64(yr1, 5);

            ptr0[1] = _mm_xor_si128(ptr0[1], b0);
            ptr1[1] = _mm_xor_si128(ptr1[1], b1);

            a0 = a_fun(ptr0[1]);
            a1 = a_fun(ptr1[1]);

            x0 = _mm_slli_si128(a0, 15);
            x1 = _mm_slli_si128(a1, 15);
            b0 = _mm_slli_epi64(x0, 1);
            b1 = _mm_slli_epi64(x1, 1);
            b0 = _mm_xor_si128(b0, x0);
            b1 = _mm_xor_si128(b1, x1);
            b0 = _mm_slli_epi64(b0, 5);
            b1 = _mm_slli_epi64(b1, 5);

            yt0 = _mm_clmulepi64_si128(a0, g18, 0x11);
            yt1 = _mm_clmulepi64_si128(a1, g18, 0x11);
            yt0 = _mm_xor_si128(yt0, a0);
            yt1 = _mm_xor_si128(yt1, a1);
            y0 = _mm_clmulepi64_si128(a0, g18, 0x10);
            y1 = _mm_clmulepi64_si128(a1, g18, 0x10);
            y0 = _mm_srli_si128(y0, 8);
            y1 = _mm_srli_si128(y1, 8);
            yt0 = _mm_xor_si128(yt0, y0);
            yt1 = _mm_xor_si128(yt1, y1);

            yt0 = _mm_xor_si128(yt0, yr0);
            yt1 = _mm_xor_si128(yt1, yr1);
            ptr0[1] = _mm_shuffle_epi8(yt0, shuffleMask);
            ptr1[1] = _mm_shuffle_epi8(yt1, shuffleMask);

            yr0 = _mm_slli_epi64(x0, 2);
            yr1 = _mm_slli_epi64(x1, 2);
            yr0 = _mm_xor_si128(x0, yr0);
            yr1 = _mm_xor_si128(x1, yr1);
            yr0 = _mm_slli_epi64(yr0, 5);
            yr1 = _mm_slli_epi64(yr1, 5);

            ptr0[2] = _mm_xor_si128(ptr0[2], b0);
            ptr1[2] = _mm_xor_si128(ptr1[2], b1);

            a0 = a_fun(ptr0[2]);
            a1 = a_fun(ptr1[2]);

            x0 = _mm_slli_si128(a0, 15);
            x1 = _mm_slli_si128(a1, 15);
            b0 = _mm_slli_epi64(x0, 1);
            b1 = _mm_slli_epi64(x1, 1);
            b0 = _mm_xor_si128(b0, x0);
            b1 = _mm_xor_si128(b1, x1);
            b0 = _mm_slli_epi64(b0, 5);
            b1 = _mm_slli_epi64(b1, 5);

            yt0 = _mm_clmulepi64_si128(a0, g18, 0x11);
            yt1 = _mm_clmulepi64_si128(a1, g18, 0x11);
            yt0 = _mm_xor_si128(yt0, a0);
            yt1 = _mm_xor_si128(yt1, a1);
            y0 = _mm_clmulepi64_si128(a0, g18, 0x10);
            y1 = _mm_clmulepi64_si128(a1, g18, 0x10);
            y0 = _mm_srli_si128(y0, 8);
            y1 = _mm_srli_si128(y1, 8);
            yt0 = _mm_xor_si128(yt0, y0);
            yt1 = _mm_xor_si128(yt1, y1);

            yt0 = _mm_xor_si128(yt0, yr0);
            yt1 = _mm_xor_si128(yt1, yr1);
            ptr0[2] = _mm_shuffle_epi8(yt0, shuffleMask);
            ptr1[2] = _mm_shuffle_epi8(yt1, shuffleMask);

            yr0 = _mm_slli_epi64(x0, 2);
            yr1 = _mm_slli_epi64(x1, 2);
            yr0 = _mm_xor_si128(x0, yr0);
            yr1 = _mm_xor_si128(x1, yr1);
            yr0 = _mm_slli_epi64(yr0, 5);
            yr1 = _mm_slli_epi64(yr1, 5);
            
            ptr0[3] = _mm_xor_si128(ptr0[3], b0);
            ptr1[3] = _mm_xor_si128(ptr1[3], b1);

            a0 = a_fun(ptr0[3]);
            a1 = a_fun(ptr1[3]);

            x0 = _mm_slli_si128(a0, 15);
            x1 = _mm_slli_si128(a1, 15);
            b0 = _mm_slli_epi64(x0, 1);
            b1 = _mm_slli_epi64(x1, 1);
            b0 = _mm_xor_si128(b0, x0);
            b1 = _mm_xor_si128(b1, x1);
            b0 = _mm_slli_epi64(b0, 5);
            b1 = _mm_slli_epi64(b1, 5);

            yt0 = _mm_clmulepi64_si128(a0, g18, 0x11);
            yt1 = _mm_clmulepi64_si128(a1, g18, 0x11);
            yt0 = _mm_xor_si128(yt0, a0);
            yt1 = _mm_xor_si128(yt1, a1);
            y0 = _mm_clmulepi64_si128(a0, g18, 0x10);
            y1 = _mm_clmulepi64_si128(a1, g18, 0x10);
            y0 = _mm_srli_si128(y0, 8);
            y1 = _mm_srli_si128(y1, 8);
            yt0 = _mm_xor_si128(yt0, y0);
            yt1 = _mm_xor_si128(yt1, y1);

            yt0 = _mm_xor_si128(yt0, yr0);
            yt1 = _mm_xor_si128(yt1, yr1);
            ptr0[3] = _mm_shuffle_epi8(yt0, shuffleMask);
            ptr1[3] = _mm_shuffle_epi8(yt1, shuffleMask);

            _mm512_storeu_si512((__m512i *)(response->output_win_1 + idx), cw0);
            _mm512_storeu_si512((__m512i *)(response->output_win_2 + idx), cw1);

            yr0 = _mm_slli_epi64(x0, 2);
            yr1 = _mm_slli_epi64(x1, 2);
            yr0 = _mm_xor_si128(x0, yr0);
            yr1 = _mm_xor_si128(x1, yr1);
            yr0 = _mm_slli_epi64(yr0, 5);
            yr1 = _mm_slli_epi64(yr1, 5);
        }
    }
    lens = request->length - len512;
    if (unlikely_local(lens == 0)) 
    {
        /* tail processing */
        b80 = _mm_extract_epi8(b0, 15);
        yr80 = _mm_extract_epi8(yr0, 15);
        b81 = _mm_extract_epi8(b1, 15);
        yr81 = _mm_extract_epi8(yr1, 15);

        bt0 = bt1 = bt2 = 0;

        bt0 = bt0 | (b80 & 0x80);
        bt1 = bt1 | (yr80 & 0x80);
        bt2 = bt2 | ((b80 & 0x40) <<1);

        bt0 = bt0 | (yr80 & 0x40);
        bt1 = bt1 | ((b80 & 0x20) <<1);
        bt2 = bt2 | ((yr80 & 0x20) <<1);

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
        while (lens >= 16) 
        {
            tmp_0 = _mm_loadu_si128((__m128i *)(request->input_win + len512));
            tmp_1 = _mm_loadu_si128((__m128i *)(input_win_2 + len512));
            _mm_storeu_si128((__m128i *)(response->output_win_0 + len512), tmp_0);
            tmp_0 = _mm_shuffle_epi8 (tmp_0, shuffleMask);
            tmp_1 = _mm_shuffle_epi8 (tmp_1, shuffleMask);

            tmp_0 = _mm_xor_si128(tmp_0, b0);
            tmp_1 = _mm_xor_si128(tmp_1, b1);

            a0 = a_fun(tmp_0);
            a1 = a_fun(tmp_1);

            x0 = _mm_slli_si128(a0, 15);
            x1 = _mm_slli_si128(a1, 15);
            b0 = _mm_slli_epi64(x0, 1);
            b1 = _mm_slli_epi64(x1, 1);
            b0 = _mm_xor_si128(b0, x0);
            b1 = _mm_xor_si128(b1, x1);
            b0 = _mm_slli_epi64(b0, 5);
            b1 = _mm_slli_epi64(b1, 5);

            yt0 = _mm_clmulepi64_si128(a0, g18, 0x11);
            yt1 = _mm_clmulepi64_si128(a1, g18, 0x11);
            yt0 = _mm_xor_si128(yt0, a0);
            yt1 = _mm_xor_si128(yt1, a1);
            y0 = _mm_clmulepi64_si128(a0, g18, 0x10);
            y1 = _mm_clmulepi64_si128(a1, g18, 0x10);
            y0 = _mm_srli_si128(y0, 8);
            y1 = _mm_srli_si128(y1, 8);
            yt0 = _mm_xor_si128(yt0, y0);
            yt1 = _mm_xor_si128(yt1, y1);

            yt0 = _mm_xor_si128(yt0, yr0);
            yt1 = _mm_xor_si128(yt1, yr1);
            yt0 = _mm_shuffle_epi8(yt0, shuffleMask);
            yt1 = _mm_shuffle_epi8(yt1, shuffleMask);

            _mm_storeu_si128((__m128i *)(response->output_win_1 + len512), yt0);
            _mm_storeu_si128((__m128i *)(response->output_win_2 + len512), yt1);

            yr0 = _mm_slli_epi64(x0, 2);
            yr1 = _mm_slli_epi64(x1, 2);
            yr0 = _mm_xor_si128(x0, yr0);
            yr1 = _mm_xor_si128(x1, yr1);
            yr0 = _mm_slli_epi64(yr0, 5);
            yr1 = _mm_slli_epi64(yr1, 5);

            len512 += 16;
            lens = request->length - len512;

            if (lens == 0) 
            {
                /* tail processing */
                b80 = _mm_extract_epi8(b0, 15);
                yr80 = _mm_extract_epi8(yr0, 15);
                b81 = _mm_extract_epi8(b1, 15);
                yr81 = _mm_extract_epi8(yr1, 15);

                bt0 = bt1 = bt2 = 0;

                bt0 = bt0 | (b80 & 0x80);
                bt1 = bt1 | (yr80 & 0x80);
                bt2 = bt2 | ((b80 & 0x40) <<1);

                bt0 = bt0 | (yr80 & 0x40);
                bt1 = bt1 | ((b80 & 0x20) <<1);
                bt2 = bt2 | ((yr80 & 0x20) <<1);

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
        }
        State0 = _mm_extract_epi8(a0, 0) & 0x7;
        State1 = _mm_extract_epi8(a1, 0) & 0x7;

        for(idx = len512; idx < request->length; idx++)
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
        Tail1 = Tail1>>2;

        *(response->output_win_0 + request->length) = (Tail0 & (128+64)) + (Tail1 & (32+16));
        Tail0 = Tail0 << 2;
        Tail1 = Tail1 << 2;
        *(response->output_win_1 + request->length) = (Tail0 & (128+64)) + (Tail1 & (32+16));
        Tail0 = Tail0 << 2;
        Tail1 = Tail1 << 2;
        *(response->output_win_2 + request->length) = (Tail0 & (128+64)) + (Tail1 & (32+16));

    }

    return 0;
}
#else
int32_t
bblib_lte_turbo_encoder_avx512(const struct bblib_turbo_encoder_request *request,
    struct bblib_turbo_encoder_response *response)
{
    printf("bblib_turbo requires AVX512 ISA support to run\n");
    return(-1);
}
#endif