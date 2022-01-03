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
 * @file   rate_matching_lte_K6144_sse.cpp
 * @brief  Implementation LTE rate matching, Rate matching with large code block length, when CaseIndex >92 in TS 136.212 table 5.1.3-3, with SSE instructions
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "smmintrin.h" /* SSE 4 for media */

#include "common_typedef_sdk.h"
#include "divide.h"
#include "phy_rate_match_internal.h"
#if defined(_BBLIB_SSE4_2_) || defined(_BBLIB_AVX2_) || defined(_BBLIB_AVX512_)


void static
subblock_interleaver_d01(int32_t nLen, uint8_t * pInput, uint8_t * pOutput )
{
    __m128i vByte15_00, vByte31_16, vByte47_32, vByte63_48;
    __m128i * pIn;
    pIn = (__m128i *) pInput;
    int16_t * pOut;
    pOut = (int16_t *) pOutput;

    const static __m128i vShuf = _mm_set_epi8 ( 15,11,7,3 ,14,10,6,2 ,13,9,5,1, 12,8,4,0 );
    __m128i vtmp0, vtmp1, vtmp2, vtmp3, vtmp5, vtmp6;
    __m128i vReady;

    int32_t trans_bits;

    int32_t j;
    for (j=0; j<13; j++)
    {
        vByte15_00 = _mm_lddqu_si128 (pIn++);
        vByte31_16 = _mm_lddqu_si128 (pIn++);
        vByte47_32 = _mm_lddqu_si128 (pIn++);
        vByte63_48 = _mm_lddqu_si128 (pIn++);

        vtmp0 = _mm_shuffle_epi8 (vByte15_00, vShuf);
        vtmp1 = _mm_shuffle_epi8 (vByte31_16, vShuf);
        vtmp2 = _mm_shuffle_epi8 (vByte47_32, vShuf);
        vtmp3 = _mm_shuffle_epi8 (vByte63_48, vShuf);

        vtmp5 = _mm_unpacklo_epi32 (vtmp0, vtmp1);
        vtmp6 = _mm_unpacklo_epi32 (vtmp2, vtmp3);

        vReady = _mm_unpacklo_epi64 (vtmp5, vtmp6); /* 0, 4, 8, ... */

        trans_bits = _mm_movemask_epi8 (vReady); /* b7, ... to 24 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 7 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b6, ... to 8 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 23 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b5, ... to 16 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 15 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b4, ... to 0 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 31 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b3, ... to 31 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 0 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b2, ... to 15 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 16 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b1, ... to 23 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 8 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b0, ... to 7 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 24 * 13 + j) = (int16_t) trans_bits;

        vReady = _mm_unpackhi_epi64 (vtmp5, vtmp6); /* 1, 5, 9, ... */

        trans_bits = _mm_movemask_epi8 (vReady); /* b15, ... to 26 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 4 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b14, ... to 10 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 20 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b13, ... to 18 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 12 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b12, ... to 2 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 28 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b11, ... to 28 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 2 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b10, ... to 12 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 18 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b9, ... to 20 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 10 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b8, ... to 4 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 26 * 13 + j) = (int16_t) trans_bits;

        vtmp5 = _mm_unpackhi_epi32 (vtmp0, vtmp1);
        vtmp6 = _mm_unpackhi_epi32 (vtmp2, vtmp3);

        vReady = _mm_unpacklo_epi64 (vtmp5, vtmp6); /*2, 6, 10, ... */

        trans_bits = _mm_movemask_epi8 (vReady); /* b23, ... to 25 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 6 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b22, ... to 9 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 22 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b21, ... to 17 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 14 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b20, ... to 1 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 30 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b19, ... to 30 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 1 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b18, ... to 14 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 17 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b17, ... to 22 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 9 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b16, ... to 6 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 25 * 13 + j) = (int16_t) trans_bits;

        vReady = _mm_unpackhi_epi64 (vtmp5, vtmp6); /* 3, 7, 11, ... */

        trans_bits = _mm_movemask_epi8 (vReady); /* b31, ... to 27 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 5 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b30, ... to 11 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 21 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b29, ... to 19 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 13 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b28, ... to 3 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 29 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b27, ... to 29 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 3 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b26, ... to 13 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 19 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b25, ... to 21 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 11 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b24, ... to 5 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 27 * 13 + j) = (int16_t) trans_bits;
    }

    return;
}

void static
subblock_interleaver_d02(int32_t nLen, uint8_t * pInput, uint8_t * pOutput)
{
    __m128i vByte15_00, vByte31_16, vByte47_32, vByte63_48;
    __m128i * pIn;
    pIn = (__m128i *) pInput;
    int16_t * pOut;
    pOut = (int16_t *) pOutput;

    const static __m128i vShuf = _mm_set_epi8 ( 15,11,7,3 ,14,10,6,2 ,13,9,5,1, 12,8,4,0 );
    __m128i vtmp0, vtmp1, vtmp2, vtmp3, vtmp5, vtmp6;
    __m128i vReady;

    int32_t trans_bits;

    int32_t j;
    for (j=0; j<13; j++)
    {
        vByte15_00 = _mm_lddqu_si128 (pIn++);
        vByte31_16 = _mm_lddqu_si128 (pIn++);
        vByte47_32 = _mm_lddqu_si128 (pIn++);
        vByte63_48 = _mm_lddqu_si128 (pIn++);

        vtmp0 = _mm_shuffle_epi8 (vByte15_00, vShuf);
        vtmp1 = _mm_shuffle_epi8 (vByte31_16, vShuf);
        vtmp2 = _mm_shuffle_epi8 (vByte47_32, vShuf);
        vtmp3 = _mm_shuffle_epi8 (vByte63_48, vShuf);

        vtmp5 = _mm_unpacklo_epi32 (vtmp0, vtmp1);
        vtmp6 = _mm_unpacklo_epi32 (vtmp2, vtmp3);

        vReady = _mm_unpacklo_epi64 (vtmp5, vtmp6); /* 0, 4, 8, ... */

        trans_bits = _mm_movemask_epi8 (vReady); /* b7, ... to 8 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 27 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b6, ... to 16 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 7 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b5, ... to 0 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 23 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b4, ... to 31 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 15 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b3, ... to 15 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 31 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b2, ... to 23 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 0 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b1, ... to 7 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 16 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b0, ... to 27 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 8 * 13 + j) = (int16_t) trans_bits;

        vReady = _mm_unpackhi_epi64 (vtmp5, vtmp6); /* 1, 5, 9, ... */

        trans_bits = _mm_movemask_epi8 (vReady); /* b15, ... to 10 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 24 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b14, ... to 18 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 4 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b13, ... to 2 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 20 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b12, ... to 28 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 12 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b11, ... to 12 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 28 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b10, ... to 20 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 2 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b9, ... to 4 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 18 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b8, ... to 24 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 10 * 13 + j) = (int16_t) trans_bits;

        vtmp5 = _mm_unpackhi_epi32 (vtmp0, vtmp1);
        vtmp6 = _mm_unpackhi_epi32 (vtmp2, vtmp3);

        vReady = _mm_unpacklo_epi64 (vtmp5, vtmp6); /* 2, 6, 10, ... */

        trans_bits = _mm_movemask_epi8 (vReady); /* b23, ... to 9 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 26 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b22, ... to 17 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 6 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b21, ... to 1 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 22 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b20, ... to 30 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 14 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b19, ... to 14 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 30 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b18, ... to 22 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 1 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b17, ... to 6 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 17 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b16, ... to 26 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 9 * 13 + j) = (int16_t) trans_bits;

        vReady = _mm_unpackhi_epi64 (vtmp5, vtmp6); /*3, 7, 11, ... */

        trans_bits = _mm_movemask_epi8 (vReady); /* b31, ... to 11 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 25 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b30, ... to 19 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 5 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b29, ... to 3 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 21 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b28, ... to 29 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 13 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b27, ... to 13 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 29 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b26, ... to 21 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 3 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b25, ... to 5 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 19 * 13 + j) = (int16_t) trans_bits;

        trans_bits = _mm_movemask_epi8 (vReady); /* b24, ... to 25 */
        vReady = _mm_slli_epi16 (vReady, 1);
        * (pOut + 11 * 13 + j) = (int16_t) trans_bits;
    }

    return;
}


int32_t
bit_selection_complete(int32_t E, int32_t k0, int32_t Ncb,int32_t Kidx, int32_t nLen, uint8_t *pout, uint8_t *psout )
{
    __m128i v1, v12_mix, vtmp,v12_perm, vtmp1;
    __m128i vconst0, vconst1, vconst2,vconstF, vbitmask;
    vconst0 = _mm_setzero_si128 ();
    vconst1 = _mm_set1_epi8 (1);
    vconst2 = _mm_set1_epi8 (2);
    vconstF = _mm_set1_epi8 (0xFF);
    vbitmask = _mm_set_epi8 (0x1, 0x2, 0x4, 0x8, 0x10,  0x20, 0x40, 0x80, 0x1, 0x2, 0x4, 0x8, 0x10,  0x20, 0x40, 0x80);
    __m128i vshuf0, vshuf1, vshuf2, vshuf3, vshuf4, vshuf5, vshuf6, vshuf7;
    vshuf0 = _mm_set_epi8 (1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0);
    vshuf1 = _mm_add_epi8 (vshuf0, vconst2);
    vshuf2 = _mm_add_epi8 (vshuf1, vconst2);
    vshuf3 = _mm_add_epi8 (vshuf2, vconst2);
    vshuf4 = _mm_add_epi8 (vshuf3, vconst2);
    vshuf5 = _mm_add_epi8 (vshuf4, vconst2);
    vshuf6 = _mm_add_epi8 (vshuf5, vconst2);
    vshuf7 = _mm_add_epi8 (vshuf6, vconst2);
    __m128i vconstE = _mm_set_epi8 ( 0x0,0x0,0x0,0x0, 0x0,0x0,0x0,0x0, 0x0,0x0,0x0,0x0, 0x0,0x0,0x0,0xFF);
    __m128i vconstY = _mm_set_epi8 ( 0x0,0x0,0x0,0x0, 0x0,0x0,0x0,0x0,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF);
    int32_t i, tmp, j;
    /* Kidx is the type of codeblock length in 188 selection */
    int32_t k0_m;
    int32_t Num_NULL = g_nNum_NULL[Kidx];
    int32_t ni, nj;
    k0_m = k0 % Ncb;
    for (ni = 0; ni < Num_NULL; ni++)
    {
        if (g_nIndex_NULL[Kidx][ni] > k0_m)
            break;
    }
    int32_t k0_n = k0_m - ni;

    /* ni stands for the NULL number between 1 ~ k0  */
    for (nj = 0; nj< Num_NULL; nj ++)
    {
        if (g_nIndex_NULL[Kidx][nj] > Ncb)
            break;
    }
    int32_t Ncb_n = Ncb - nj;
    /* nj stands for the NULL number between 1 ~ Ncb  */

    /* t_beg means the primary position for the first bit; */
    int32_t t_beg = k0_n;

    /* ========== bit selection ============ */
    int32_t nBit= 0;

     int32_t nByte1 = 0;
    int32_t nBit1 = 0;
    nByte1 = t_beg / 8;
    /* nBits means: there are nBit1 bits needs to be thrown in the first byte */
    nBit1 = t_beg % 8;

    int32_t nByte2 = 0;
    int32_t nBit2 = 0;
    nByte2 = Ncb_n / 8;

    /* nBit2 is for the 2st concatenation */
    /* nBit2 means the number of bits needs to be kept for the last byte */
    nBit2 = Ncb_n % 64;

    int32_t cnt = 0, cmt = 0;
    int32_t tLen = nByte2 - nByte1;

    /* nBit3 is for the first concatenation */
    /* nBit3 means: in the last SSE register, there are nBit3 bits needs to be kept */
    int32_t nBit3 = 0;
    nBit3 = Ncb_n % 8 + (tLen % 8) * 8;

    __m128i vout, vin, vin1, vout1, vtmps;

    /* The nBit points to the t_beg position */
    nBit = nBit1;
    uint8_t *s_out = pout + nByte1;
    i = 0;
    vtmp1 = _mm_setzero_si128 ();

    /* processing the first byte */
    vtmps = _mm_loadu_si128((__m128i*)(s_out + i));
    /* == leaves nBit empty bits in the high half, throw the low nBit data== */

    vout = _mm_srli_epi64(vtmps, nBit);

    for(i = 8; i <= tLen; i += 8)
    {
        vtmps = _mm_loadu_si128((__m128i*)(s_out + i));
        /* ==== prepare the low nBit in the current 128 bit, put it at high position === */
        vout1 = _mm_slli_epi64(vtmps, 64 - nBit);
        vin1 = _mm_or_si128(vout1, vout);

        /* =======  SSE register's low 64 bit are reversed internal byte ==== */
        vtmp = vin1;
        v1 = _mm_shuffle_epi8 (vtmp, vshuf0);
        v12_perm = _mm_and_si128 (v1,vbitmask);
        v12_mix = _mm_cmpeq_epi8 (v12_perm, vconst0);
        v12_mix  = _mm_xor_si128 (v12_mix, vconstF);
        tmp = _mm_movemask_epi8 (v12_mix);
        vtmp1 = _mm_insert_epi16(vtmp1, tmp, 0);

        v1 = _mm_shuffle_epi8 (vtmp, vshuf1);
        v12_perm = _mm_and_si128 (v1,vbitmask);
        v12_mix = _mm_cmpeq_epi8 (v12_perm, vconst0);
        v12_mix  = _mm_xor_si128 (v12_mix, vconstF);
        tmp = _mm_movemask_epi8 (v12_mix);
        vtmp1 = _mm_insert_epi16(vtmp1, tmp, 1);

        v1 = _mm_shuffle_epi8 (vtmp, vshuf2);
        v12_perm = _mm_and_si128 (v1,vbitmask);
        v12_mix = _mm_cmpeq_epi8 (v12_perm, vconst0);
        v12_mix  = _mm_xor_si128 (v12_mix, vconstF);
        tmp = _mm_movemask_epi8 (v12_mix);
        vtmp1 = _mm_insert_epi16(vtmp1, tmp, 2);

        v1 = _mm_shuffle_epi8 (vtmp, vshuf3);
        v12_perm = _mm_and_si128 (v1,vbitmask);
        v12_mix = _mm_cmpeq_epi8 (v12_perm, vconst0);
        v12_mix  = _mm_xor_si128 (v12_mix, vconstF);
        tmp = _mm_movemask_epi8 (v12_mix);
        vtmp1 = _mm_insert_epi16(vtmp1, tmp, 3);

        vin = vtmp1;

        /* ====== store ======== */
        _mm_storel_epi64((__m128i *)(psout + (cnt >> 3)), vin);
        cnt += 64;
        if(cnt >= E)
            break;
        /* ======= leaves nBit empty position at high position ==== */
        vout = _mm_srli_epi64(vtmps, nBit);
    }

    if(cnt == E)
        return 0;

    /* The 1st concatenation */
    if((cnt < E) && (i >= tLen))
    {
        /* vout and nBit needs to be adjusted */
        if(nBit3 > nBit1)
        {
            nBit = 64 - (nBit3 - nBit);
            vout1 = _mm_srli_epi64(vconstY, nBit);
            vout = _mm_and_si128(vout, vout1);
        }
        else if(nBit3 < nBit1)
        {
            nBit = nBit - nBit3;
            vout1 = _mm_srli_epi64(vconstY, nBit);
            vout = _mm_and_si128(vin1, vout1);
            cnt -= 64;
            /* Notice that data is capured from vin */
            /* since vin contains Null bits, cnt is reduced from 64 */
        }
        else  /* nBit - nBit3 ==0 */
        {
            nBit = 64;
            vout = _mm_setzero_si128();
        }
    }

    /* Bellow is the process of situation aht E reads to the end of Ncb; */
    /* and stars from 0 position again; */
    /* we use while to resolve the read/concatenation times and times again; */
    /* we use cnt to control the times of reading; */
    s_out = pout;
    cmt = 0;
    while((cnt < E))
    {
        cmt++;
        for(j = 0; j < nByte2; j += 8)
        {
            vtmps = _mm_loadu_si128((__m128i*)(s_out + j));

            /* Prepare the low nBit in the current 128bit, put them at high position */
            vout1 = _mm_slli_epi64(vtmps, 64 - nBit);
            vin1 = _mm_or_si128(vout1, vout);

            /* =======  SSE register's low 64 bit are reversed internal byte ==== */
            vtmp = vin1;
            v1 = _mm_shuffle_epi8 (vtmp, vshuf0);
            v12_perm = _mm_and_si128 (v1,vbitmask);
            v12_mix = _mm_cmpeq_epi8 (v12_perm, vconst0);
            v12_mix  = _mm_xor_si128 (v12_mix, vconstF);
            tmp = _mm_movemask_epi8 (v12_mix);
            vtmp1 = _mm_insert_epi16(vtmp1, tmp, 0);

            v1 = _mm_shuffle_epi8 (vtmp, vshuf1);
            v12_perm = _mm_and_si128 (v1,vbitmask);
            v12_mix = _mm_cmpeq_epi8 (v12_perm, vconst0);
            v12_mix  = _mm_xor_si128 (v12_mix, vconstF);
            tmp = _mm_movemask_epi8 (v12_mix);
            vtmp1 = _mm_insert_epi16(vtmp1, tmp, 1);

            v1 = _mm_shuffle_epi8 (vtmp, vshuf2);
            v12_perm = _mm_and_si128 (v1,vbitmask);
            v12_mix = _mm_cmpeq_epi8 (v12_perm, vconst0);
            v12_mix  = _mm_xor_si128 (v12_mix, vconstF);
            tmp = _mm_movemask_epi8 (v12_mix);
            vtmp1 = _mm_insert_epi16(vtmp1, tmp, 2);

            v1 = _mm_shuffle_epi8 (vtmp, vshuf3);
            v12_perm = _mm_and_si128 (v1,vbitmask);
            v12_mix = _mm_cmpeq_epi8 (v12_perm, vconst0);
            v12_mix  = _mm_xor_si128 (v12_mix, vconstF);
            tmp = _mm_movemask_epi8 (v12_mix);
            vtmp1 = _mm_insert_epi16(vtmp1, tmp, 3);

            vin = vtmp1;

            /* =========== store======== */
            _mm_storel_epi64((__m128i*)(psout + (cnt >> 3)), vin);
            cnt += 64;
            if(cnt >= E)
                break;
            /* ======= leaves nBit empty position at high positions ==== */
            vout = _mm_srli_epi64(vtmps, nBit);
        }

        /* Notice that when enough bits is collected, we need to jump out of 2 iteration */
        if(cnt >= E)
            break;

        /* The 2nd concatenation  */
        /* after each iteration, nBit and vout needs to be adjusted!! */
        if(nBit2 > nBit)
        {
            nBit = 64 - (nBit2 - nBit);
            vout1 = _mm_srli_epi64(vconstY, nBit);
            vout = _mm_and_si128(vout, vout1);
        }
        else
        {
            nBit = nBit - nBit2;
            vout1 = _mm_srli_epi64(vconstY, nBit);
            vout = _mm_and_si128(vin1, vout1);
            cnt -= 64;
            /* Notice that we capture data from vin */
            /* There are NULL bits in vin, hence cnt needs to be reduced 64 */
        }
    }
    if(cnt == E)
        return 0;

    /* === to adjust the last byte according to the length of output=== */
    int32_t ntmp = E / 8;
    int32_t ntmpb = E % 8;
    psout[ntmp] = (psout[ntmp]) & (255 << (8-ntmpb));

    /* Bellow we remove the un-useful bits, and set them to be 0 */
    if(cnt > E)
    {
        vtmps = _mm_loadu_si128((__m128i*)(&psout[ntmp]));
        vin = _mm_and_si128(vtmps, vconstE);
        _mm_storeu_si128((__m128i*)(&psout[ntmp]), vin);
    }

    return 0;

}

void static bit_collection(int32_t Kidx, int32_t nLen, int32_t sLen, uint8_t * pv0, uint8_t * pv1, uint8_t * pv2, uint8_t * pOutput)
{
    int32_t i,cnt, tmp;

    /* v1 and v2 perm bit by bit */
    uint16_t v12[832+20];

    __m128i v1_16, v2_16, v12_mix, v12_perm, vtmp;
    __m128i vconst0, vconst1, vbitmask,vconstF;
    vconst0 = _mm_setzero_si128 ();
    vconst1 = _mm_set1_epi8 (1);
    vconstF = _mm_set_epi8 ( 0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0, 0xFF, 0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF);
    vbitmask = _mm_set_epi8 (0x80, 0x80, 0x40, 0x40, 0x20, 0x20, 0x10, 0x10, 0x8, 0x8, 0x4, 0x4, 0x2, 0x2, 0x1, 0x1);

    __m128i vshuf0, vshuf1, vshuf2, vshuf3, vshuf4, vshuf5, vshuf6, vshuf7;
    vshuf0 = _mm_set_epi8 (8, 0, 8, 0, 8, 0, 8, 0, 8, 0, 8, 0, 8, 0, 8, 0);
    vshuf1 = _mm_add_epi8 (vshuf0, vconst1);
    vshuf2 = _mm_add_epi8 (vshuf1, vconst1);
    vshuf3 = _mm_add_epi8 (vshuf2, vconst1);
    vshuf4 = _mm_add_epi8 (vshuf3, vconst1);
    vshuf5 = _mm_add_epi8 (vshuf4, vconst1);
    vshuf6 = _mm_add_epi8 (vshuf5, vconst1);
    vshuf7 = _mm_add_epi8 (vshuf6, vconst1);

    __m128i * p1;
    __m128i * p2;
    __m128i * p12;
    cnt = 0;
    vconst1 = _mm_set1_epi8 (0xff);
    for (i=0; i<27; i++)
    {
        p1 = (__m128i *) (pv1 + i*26);
        p2 = (__m128i *) (pv2 + i*26);
        p12 = (__m128i *) (v12 + i*26);

        v1_16 = _mm_lddqu_si128 (p1++);
        v2_16 = _mm_lddqu_si128 (p2++);

        v12_mix = _mm_unpacklo_epi64 (v1_16, v2_16); vtmp = _mm_setzero_si128 ();

        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf0); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 0);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf1); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 1);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf2); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 2);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf3); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 3);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf4); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 4);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf5); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 5);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf6); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 6);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf7); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 7);
        vtmp = _mm_xor_si128 (vtmp, vconst1);
        _mm_storeu_si128 (p12++, vtmp);


        /* -------------------------------------------------------- */
        v12_mix = _mm_unpackhi_epi64 (v1_16, v2_16); vtmp = _mm_setzero_si128 ();

        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf0); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 0);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf1); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 1);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf2); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 2);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf3); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 3);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf4); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 4);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf5); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 5);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf6); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 6);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf7); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 7);
        vtmp = _mm_xor_si128 (vtmp, vconst1);
        _mm_storeu_si128 (p12++, vtmp);

        /* -------------------------------------------------------- */
        v1_16 = _mm_lddqu_si128 (p1++);
        v2_16 = _mm_lddqu_si128 (p2++);

        v12_mix = _mm_unpacklo_epi64 (v1_16, v2_16); vtmp = _mm_setzero_si128 ();

        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf0); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 0);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf1); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 1);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf2); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 2);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf3); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 3);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf4); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 4);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf5); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 5);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf6); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 6);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf7); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 7);
        vtmp = _mm_xor_si128 (vtmp, vconst1);
        _mm_storeu_si128 (p12++, vtmp);

        /* -------------------------------------------------------- */
        v12_mix = _mm_unpackhi_epi64 (v1_16, v2_16); vtmp = _mm_setzero_si128 ();
        cnt += 24;

        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf0); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); v12[cnt++] = (uint16_t) (tmp ^ 0xffff);

        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf1); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); v12[cnt++] = (uint16_t) (tmp ^ 0xffff);
    }

    for (i=27; i<28; i++)
    {
        p1 = (__m128i *) (pv1 + i*26);
        p2 = (__m128i *) (pv2 + i*26);
        p12 = (__m128i *) (v12 + i*26);

        v1_16 = _mm_lddqu_si128 (p1++);
        v2_16 = _mm_lddqu_si128 (p2++);

        v12_mix = _mm_unpacklo_epi64 (v2_16, v1_16); vtmp = _mm_setzero_si128 ();

        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf0); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 0);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf1); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 1);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf2); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 2);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf3); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 3);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf4); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 4);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf5); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 5);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf6); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 6);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf7); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 7);
        vtmp = _mm_xor_si128 (vtmp, vconst1);
        _mm_storeu_si128 (p12++, vtmp);

        /* -------------------------------------------------------- */
        v12_mix = _mm_unpackhi_epi64 (v2_16, v1_16); vtmp = _mm_setzero_si128 ();

        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf0); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 0);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf1); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 1);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf2); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 2);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf3); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 3);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf4); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 4);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf5); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 5);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf6); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 6);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf7); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 7);
        vtmp = _mm_xor_si128 (vtmp, vconst1);
        _mm_storeu_si128 (p12++, vtmp);

        /* -------------------------------------------------------- */
        v1_16 = _mm_lddqu_si128 (p1++);
        v2_16 = _mm_lddqu_si128 (p2++);

        v12_mix = _mm_unpacklo_epi64 (v2_16, v1_16); vtmp = _mm_setzero_si128 ();

        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf0); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 0);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf1); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 1);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf2); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 2);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf3); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 3);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf4); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 4);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf5); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 5);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf6); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 6);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf7); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 7);
        vtmp = _mm_xor_si128 (vtmp, vconst1);
        _mm_storeu_si128 (p12++, vtmp);

        /* -------------------------------------------------------- */
        v12_mix = _mm_unpackhi_epi64 (v2_16, v1_16); vtmp = _mm_setzero_si128 ();
        cnt += 24;

        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf0); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); v12[cnt++] = (uint16_t) (tmp ^ 0xffff);

        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf1); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); v12[cnt++] = (uint16_t) (tmp ^ 0xffff);
    }

    for (i=28; i<32; i++)
    {
        p1 = (__m128i *) (pv1 + i*26);
        p2 = (__m128i *) (pv2 + i*26);
        p12 = (__m128i *) (v12 + i*26);

        v1_16 = _mm_lddqu_si128 (p1++);
        v2_16 = _mm_lddqu_si128 (p2++);

        v12_mix = _mm_unpacklo_epi64 (v1_16, v2_16); vtmp = _mm_setzero_si128 ();

        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf0); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 0);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf1); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 1);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf2); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 2);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf3); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 3);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf4); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 4);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf5); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 5);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf6); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 6);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf7); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 7);
        vtmp = _mm_xor_si128 (vtmp, vconst1);
        _mm_storeu_si128 (p12++, vtmp);

        /* -------------------------------------------------------- */
        v12_mix = _mm_unpackhi_epi64 (v1_16, v2_16); vtmp = _mm_setzero_si128 ();

        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf0); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 0);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf1); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 1);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf2); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 2);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf3); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 3);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf4); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 4);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf5); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 5);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf6); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 6);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf7); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 7);
        vtmp = _mm_xor_si128 (vtmp, vconst1);
        _mm_storeu_si128 (p12++, vtmp);

        /* -------------------------------------------------------- */
        v1_16 = _mm_lddqu_si128 (p1++);
        v2_16 = _mm_lddqu_si128 (p2++);

        v12_mix = _mm_unpacklo_epi64 (v1_16, v2_16); vtmp = _mm_setzero_si128 ();

        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf0); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 0);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf1); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 1);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf2); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 2);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf3); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 3);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf4); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 4);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf5); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 5);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf6); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 6);
        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf7); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); vtmp = _mm_insert_epi16(vtmp, tmp, 7);
        vtmp = _mm_xor_si128 (vtmp, vconst1);
        _mm_storeu_si128 (p12++, vtmp);

        /* -------------------------------------------------------- */
        v12_mix = _mm_unpackhi_epi64 (v1_16, v2_16); vtmp = _mm_setzero_si128 ();
        cnt += 24;

        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf0); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); v12[cnt++] = (uint16_t) (tmp ^ 0xffff);

        v12_perm = _mm_shuffle_epi8 (v12_mix, vshuf1); v12_perm = _mm_and_si128 (v12_perm, vbitmask); v12_perm = _mm_cmpeq_epi8 (v12_perm, vconst0);
        tmp = _mm_movemask_epi8 (v12_perm); v12[cnt++] = (uint16_t) (tmp ^ 0xffff);
    }

    /* bit concat and output */
    /* ================= bit concat and output for system bits =========== */
    __m128i vtmps, vout1, vout2,vout3, vout4, vout5, vout6, vout;

    int32_t nBits0, nBits1, nBits2, nBits3,nBits4, nBitst;
    int32_t nSSE = 0;
    int32_t nLine = 0;

    int32_t nBitVec0[32] = {0};
    int32_t nBitVec1[32] = {0};

    nLine = nLen / 32;


    /* nBits0 is the number of bit in each line, how many bits are left out of SSE64 bit register */
    nBits0 = (nLen / 32) % 64;
    nBits1 = nBits0 + 1;
    nBits2 = nBits0 * 2;
    nBits3 = nBits0 * 2 + 1;
    nBits4 = (nBits0 + 1) * 2;

    nSSE = nLine / 64;

    if( ((nLine % 64) > 0) || (nLen % 32) > 0)
        nSSE ++;

    for(int32_t iTmp = 0; iTmp < 32; iTmp++)
    {
        if((iTmp == 7) || (iTmp == 15) || (iTmp == 23) || (iTmp == 31))
            nBitVec0[iTmp] = nBits1;
        else
            nBitVec0[iTmp] = nBits0;
    }

    for(int32_t iTmp = 0; iTmp < 32; iTmp++)
    {
        if((iTmp == 7) || (iTmp == 15) || (iTmp == 23) )
            nBitVec1[iTmp] = nBits4;
        else if((iTmp == 27) || (iTmp == 31))
            nBitVec1[iTmp] = nBits3;
        else
            nBitVec1[iTmp] = nBits2;
    }

    switch (nSSE)
    {
        case 1:
            {
            /* The concatenation for information bits */
                nBitst = 0;
                cnt = 0;
                vout = vconst0;
                for(int32_t iTmp = 0; iTmp < 32; iTmp++)
                {
                    vtmps = _mm_loadu_si128((__m128i*)(pv0 + iTmp * 26 + 0));
                    vout1 = _mm_slli_epi64(vtmps, nBitst);
                    vout2 = _mm_or_si128(vout1, vout);

                    /* Notice that here in vtmps, the usefule bits numner < 64 */
                    vout3 = _mm_srli_epi64(vtmps, 64 - nBitst);
                    nBitst += nBitVec0[iTmp];

                    if(nBitst < 64)
                    {
                        /* remove the un-useful bits to be zero */
                        vout1 = _mm_srli_epi64(vconstF, 64 - nBitst);
                        vout = _mm_and_si128(vout1, vout2);
                    }
                    else
                    {
                        /* Here we collect more than 1 64 bit register, and adjust nBits */
                        _mm_storel_epi64((__m128i *)(pOutput+cnt), vout2);
                        cnt += 8;
                        nBitst -= 64;

                        vout1 = _mm_srli_epi64(vconstF, 64 - nBitst);
                        vout = _mm_and_si128(vout1, vout3);
                    }
                }

                /* concatenation for parity bits */
                uint16_t * ps12 = v12;
                for(int32_t iTmp = 0; iTmp < 32; iTmp++)
                {
                    vtmps = _mm_loadu_si128((__m128i*)(ps12 + iTmp * 26 + 0));
                    vout1 = _mm_slli_epi64(vtmps, nBitst);
                    vout2 = _mm_or_si128(vout1, vout);

                    /* Notice that the useful bits number maybe n>=64 or n<64 */
                    vout3 = _mm_srli_epi64(vtmps, 64 - nBitst);
                    vout4 = _mm_shuffle_epi32(vtmps, 78);
                    vout4 = _mm_and_si128(vout4, vconstF);
                    vout5 = _mm_slli_epi64(vout4, nBitst);
                    vout3 = _mm_or_si128(vout3, vout5);

                    nBitst += nBitVec1[iTmp];

                    if(nBitst < 64)
                    {
                        /* remove un-useful bit, set to 0 */
                        vout1 = _mm_srli_epi64(vconstF, 64 - nBitst);
                        vout = _mm_and_si128(vout1, vout2);
                    }
                    else if( (nBitst >= 64) && (nBitst < 128))
                    {
                        /* collect a full 64 bits, store, and adjust nBits */
                        _mm_storel_epi64((__m128i *)(pOutput+cnt), vout2);
                        cnt += 8;
                        nBitst -= 64;

                        /* remove un-useful bit, set to 0 */
                        vout1 = _mm_srli_epi64(vconstF, 64 - nBitst);
                        vout = _mm_and_si128(vout1, vout3);
                    }
                    else
                    {
                        /* The useful bit must be N<64*3 */
                        /* Here we collect full 2 64 bits, store them, and then adjust nBits */
                        _mm_storel_epi64((__m128i *)(pOutput+cnt), vout2);
                        cnt += 8;
                        nBitst -= 64;

                        vout = vout3;
                        _mm_storel_epi64((__m128i *)(pOutput+cnt), vout);
                        cnt += 8;
                        nBitst -= 64;

                        /* Here in vout3, the low nBitst is the output */
                        /* nBist < 64 */
                        vout4 = _mm_shuffle_epi32(vout, 78);
                        /* remove the un-useful bits */
                        vout1 = _mm_srli_epi64(vconstF, 64 - nBitst);
                        vout = _mm_and_si128(vout1, vout4);
                    }

                    if( (iTmp == 31)) /* Process the tail bits, store them */
                        _mm_storel_epi64((__m128i *)(pOutput+cnt), vout);

                }

                break;
            }
        case 2:
            {
                /* for information bits concatenation */
                nBitst = 0;
                cnt = 0;
                vout = vconst0;
                for(int32_t iTmp = 0; iTmp < 32; iTmp++)
                {
                    vtmps = _mm_loadu_si128((__m128i*)(pv0 + iTmp * 26 + 0));
                    vout1 = _mm_slli_epi64(vtmps, nBitst);
                    vout2 = _mm_or_si128(vout1, vout);   /* the 1st 64bit data */
                    _mm_storel_epi64((__m128i *)(pOutput+cnt), vout2);
                    cnt += 8;
                    vout = _mm_srli_epi64(vtmps, 64-nBitst);

                    vtmps = _mm_loadu_si128((__m128i*)(pv0 + iTmp * 26 + 8)); /* the 3rd 64bit, includes nBitVec0[iTmp] useful bits */
                    vout1 = _mm_slli_epi64(vtmps, nBitst);
                    vout2 = _mm_or_si128(vout1, vout);
                    nBitst += nBitVec0[iTmp];

                    if(nBitst < 64)
                    {
                        vout1 = _mm_srli_epi64(vconstF, 64 - nBitst);
                        vout = _mm_and_si128(vout1, vout2);
                    }
                    else
                    {
                        _mm_storel_epi64((__m128i *)(pOutput+cnt), vout2);
                        cnt += 8;
                        vout = _mm_shuffle_epi32(vout2, 78);
                        nBitst -= 64;

                        vout1 = _mm_srli_epi64(vconstF, 64 - nBitst);
                        vout = _mm_and_si128(vout1, vout);
                    }
                }

                /* for parity bits concatenation */
                uint16_t * ps12 = v12;
                for(int32_t iTmp = 0; iTmp < 32; iTmp++)
                {
                    vtmps = _mm_loadu_si128((__m128i*)(ps12 + iTmp * 26 + 0));
                    vout1 = _mm_slli_epi64(vtmps, nBitst);
                    vout2 = _mm_or_si128(vout1, vout);   /* the 1st 64 bit */
                    _mm_storel_epi64((__m128i *)(pOutput+cnt), vout2);
                    cnt += 8;
                    vout = _mm_srli_epi64(vtmps, 64-nBitst);

                    vtmps = _mm_loadu_si128((__m128i*)(ps12 + iTmp * 26 + 4));
                    vout1 = _mm_slli_epi64(vtmps, nBitst);
                    vout2 = _mm_or_si128(vout1, vout);   /* the 2nd 64 bit */
                    _mm_storel_epi64((__m128i *)(pOutput+cnt), vout2);
                    cnt += 8;
                    vout = _mm_srli_epi64(vtmps, 64-nBitst);

                    vtmps = _mm_loadu_si128((__m128i*)(ps12 + iTmp * 26 + 8));
                    vout1 = _mm_slli_epi64(vtmps, nBitst);
                    vout2 = _mm_or_si128(vout1, vout);
                    nBitst += nBitVec1[iTmp];

                    if(nBitst < 64)
                    {
                        vout1 = _mm_srli_epi64(vconstF, 64 - nBitst);
                        vout = _mm_and_si128(vout1, vout2);
                    }
                    else if( (nBitst >= 64) && (nBitst < 128))
                    {
                        _mm_storel_epi64((__m128i *)(pOutput+cnt), vout2);  /* collect 1 whole 64 bit, adjust nBits */
                        cnt += 8;
                        nBitst -= 64;
                        /*  vout = _mm_srli_si128(vout2, 64); */
                        vout = _mm_shuffle_epi32(vout2, 78);
                        vout1 = _mm_srli_epi64(vconstF, 64 - nBitst);
                        vout = _mm_and_si128(vout1, vout);
                    }
                    else   /* N < 64 * 3 */
                    {
                        _mm_storel_epi64((__m128i *)(pOutput+cnt), vout2);  /* collect 2 whole 64 bit, adjust nBits */
                        cnt += 8;
                        nBitst -= 64;

                        vout = _mm_shuffle_epi32(vout2, 78);
                        _mm_storel_epi64((__m128i *)(pOutput+cnt), vout);
                        cnt += 8;
                        nBitst -= 64;

                        /* shift left nBist00 bits */
                        int32_t nBit000 = 64 * 2 - nBitVec1[iTmp];
                        vout3 = _mm_slli_epi64(vtmps, nBit000);
                        vout4 = _mm_shuffle_epi32(vtmps, 78);
                        vout5 = _mm_srli_epi64(vout4, 64 - nBit000);
                        vout6 = _mm_or_si128(vout3, vout5);

                        /* the high nBitst bits of vout3 is the output we want */
                        /* nBist < 64 */
                        vout3 = _mm_shuffle_epi32(vout6, 78);
                        vout2 = _mm_srli_epi64(vout3, 64 - nBitst);
                        vout1 = _mm_srli_epi64(vconstF, 64 - nBitst);
                        vout = _mm_and_si128(vout1, vout2);
                    }

                    if((iTmp == 31))
                        _mm_storel_epi64((__m128i *)(pOutput+cnt), vout);

                }

                break;
            }
        case 3:
            {
                /* for information bits */
                nBitst = 0;
                cnt = 0;
                vout = vconst0;
                for(int32_t iTmp = 0; iTmp < 32; iTmp++)
                {
                    vtmps = _mm_loadu_si128((__m128i*)(pv0 + iTmp * 26 + 0));
                    vout1 = _mm_slli_epi64(vtmps, nBitst);
                    vout2 = _mm_or_si128(vout1, vout);   /* 1st 64 bit */
                    _mm_storel_epi64((__m128i *)(pOutput+cnt), vout2);
                    cnt += 8;
                    vout = _mm_srli_epi64(vtmps, 64-nBitst);

                    vtmps = _mm_loadu_si128((__m128i*)(pv0 + iTmp * 26 + 8));
                    vout1 = _mm_slli_epi64(vtmps, nBitst);
                    vout2 = _mm_or_si128(vout1, vout);   /* 2nd 64 bit */
                    _mm_storel_epi64((__m128i *)(pOutput+cnt), vout2);
                    cnt += 8;
                    vout = _mm_srli_epi64(vtmps, 64-nBitst);

                    vtmps = _mm_loadu_si128((__m128i*)(pv0 + iTmp * 26 + 16)); /* 3rd 64 bit */
                    vout1 = _mm_slli_epi64(vtmps, nBitst);
                    vout2 = _mm_or_si128(vout1, vout);
                    nBitst += nBitVec0[iTmp];

                    if(nBitst < 64)
                    {
                        vout1 = _mm_srli_epi64(vconstF, 64 - nBitst);
                        vout = _mm_and_si128(vout1, vout2);
                    }
                    else
                    {
                        _mm_storel_epi64((__m128i *)(pOutput+cnt), vout2);
                        cnt += 8;
                        vout = _mm_shuffle_epi32(vout2, 78);
                        nBitst -= 64;

                        vout1 = _mm_srli_epi64(vconstF, 64 - nBitst);
                        vout = _mm_and_si128(vout1, vout);
                    }
                }

                /* bit connect */
                uint16_t * ps12 = v12;
                for(int32_t iTmp = 0; iTmp < 32; iTmp++)
                {
                     vtmps = _mm_loadu_si128((__m128i*)(ps12 + iTmp * 26 + 0));
                     vout1 = _mm_slli_epi64(vtmps, nBitst);
                     vout2 = _mm_or_si128(vout1, vout);   /* 1st 64 bit */
                     _mm_storel_epi64((__m128i *)(pOutput+cnt), vout2);
                     cnt += 8;
                     vout = _mm_srli_epi64(vtmps, 64-nBitst);

                    vtmps = _mm_loadu_si128((__m128i*)(ps12 + iTmp * 26 + 4));
                    vout1 = _mm_slli_epi64(vtmps, nBitst);
                    vout2 = _mm_or_si128(vout1, vout);   /* 2nd 64 bit */
                    _mm_storel_epi64((__m128i *)(pOutput+cnt), vout2);
                    cnt += 8;
                    vout = _mm_srli_epi64(vtmps, 64-nBitst);

                    vtmps = _mm_loadu_si128((__m128i*)(ps12 + iTmp * 26 + 8));
                    vout1 = _mm_slli_epi64(vtmps, nBitst);
                    vout2 = _mm_or_si128(vout1, vout);   /* 3rd 64 bit */
                    _mm_storel_epi64((__m128i *)(pOutput+cnt), vout2);
                    cnt += 8;
                    vout = _mm_srli_epi64(vtmps, 64-nBitst);

                    vtmps = _mm_loadu_si128((__m128i*)(ps12 + iTmp * 26 + 12));
                    vout1 = _mm_slli_epi64(vtmps, nBitst);
                    vout2 = _mm_or_si128(vout1, vout);   /* 4th 64 bit */
                    _mm_storel_epi64((__m128i *)(pOutput+cnt), vout2);
                    cnt += 8;
                    vout = _mm_srli_epi64(vtmps, 64-nBitst);

                    vtmps = _mm_loadu_si128((__m128i*)(ps12 + iTmp * 26 + 16));
                    vout1 = _mm_slli_epi64(vtmps, nBitst);
                    vout2 = _mm_or_si128(vout1, vout);
                    nBitst += nBitVec1[iTmp];

                    if(nBitst < 64)
                    {
                        vout1 = _mm_srli_epi64(vconstF, 64 - nBitst);
                        vout = _mm_and_si128(vout1, vout2);
                    }
                    else if( (nBitst >= 64) && (nBitst < 128))
                    {
                        _mm_storel_epi64((__m128i *)(pOutput+cnt), vout2);
                        cnt += 8;
                        nBitst -= 64;
                        /*  vout = _mm_srli_si128(vout2, 64); */
                        vout = _mm_shuffle_epi32(vout2, 78);
                        vout1 = _mm_srli_epi64(vconstF, 64 - nBitst);
                        vout = _mm_and_si128(vout1, vout);
                    }
                    else   /* N < 64 * 3 */
                    {
                        _mm_storel_epi64((__m128i *)(pOutput+cnt), vout2);
                        cnt += 8;
                        nBitst -= 64;

                        vout = _mm_shuffle_epi32(vout2, 78);
                        _mm_storel_epi64((__m128i *)(pOutput+cnt), vout);
                        cnt += 8;
                        nBitst -= 64;

                        /* shift left nBist00 */
                        int32_t nBit000 = 64 * 2 - nBitVec1[iTmp];
                        vout3 = _mm_slli_epi64(vtmps, nBit000);
                        vout4 = _mm_shuffle_epi32(vtmps, 78);
                        vout5 = _mm_srli_epi64(vout4, 64 - nBit000);
                        vout6 = _mm_or_si128(vout3, vout5);

                        /* the high nBitst of vout3 is output */
                        /* nBist < 64 */
                        vout3 = _mm_shuffle_epi32(vout6, 78);
                        vout2 = _mm_srli_epi64(vout3, 64 - nBitst);
                        vout1 = _mm_srli_epi64(vconstF, 64 - nBitst);
                        vout = _mm_and_si128(vout1, vout2);
                    }

                    if( (iTmp == 31))
                        _mm_storel_epi64((__m128i *)(pOutput+cnt), vout);

                }

                break;
            }
        case 4:
            {
                /* for information bits */
                nBitst = 0;
                cnt = 0;
                vout = vconst0;
                for(int32_t iTmp = 0; iTmp < 32; iTmp++)
                {
                    vtmps = _mm_loadu_si128((__m128i*)(pv0 + iTmp * 26 + 0));
                    vout1 = _mm_slli_epi64(vtmps, nBitst);
                    vout2 = _mm_or_si128(vout1, vout);   /* 1st  64 bit */
                    _mm_storel_epi64((__m128i *)(pOutput+cnt), vout2);
                    cnt += 8;
                    vout = _mm_srli_epi64(vtmps, 64-nBitst);

                    vtmps = _mm_loadu_si128((__m128i*)(pv0 + iTmp * 26 + 8));
                    vout1 = _mm_slli_epi64(vtmps, nBitst);
                    vout2 = _mm_or_si128(vout1, vout);   /* 2nd 64 bit */
                    _mm_storel_epi64((__m128i *)(pOutput+cnt), vout2);
                    cnt += 8;
                    vout = _mm_srli_epi64(vtmps, 64-nBitst);

                    vtmps = _mm_loadu_si128((__m128i*)(pv0 + iTmp * 26 + 16));
                    vout1 = _mm_slli_epi64(vtmps, nBitst);
                    vout2 = _mm_or_si128(vout1, vout);   /* 3rd 64 bit */
                    _mm_storel_epi64((__m128i *)(pOutput+cnt), vout2);
                    cnt += 8;
                    vout = _mm_srli_epi64(vtmps, 64-nBitst);

                    vtmps = _mm_loadu_si128((__m128i*)(pv0 + iTmp * 26 + 24)); /* 4th 64 bit contains nBitVec0[iTmp] useful bits */
                    vout1 = _mm_slli_epi64(vtmps, nBitst);
                    vout2 = _mm_or_si128(vout1, vout);   /* update nBitst */
                    nBitst += nBitVec0[iTmp];

                    if(nBitst < 64)
                    {
                        vout1 = _mm_srli_epi64(vconstF, 64 - nBitst);
                        vout = _mm_and_si128(vout1, vout2);
                    }
                    else
                    {
                        _mm_storel_epi64((__m128i *)(pOutput+cnt), vout2);
                        cnt += 8;
                        vout = _mm_shuffle_epi32(vout2, 78);
                        nBitst -= 64;

                        vout1 = _mm_srli_epi64(vconstF, 64 - nBitst);
                        vout = _mm_and_si128(vout1, vout);
                    }
                }

                /* for parity bits */
                uint16_t * ps12 = v12;
                for(int32_t iTmp = 0; iTmp < 32; iTmp++)
                {
                    vtmps = _mm_loadu_si128((__m128i*)(ps12 + iTmp * 26 + 0));
                    vout1 = _mm_slli_epi64(vtmps, nBitst);
                    vout2 = _mm_or_si128(vout1, vout);   /* 1st 64 bit */
                    _mm_storel_epi64((__m128i *)(pOutput+cnt), vout2);
                    cnt += 8;
                    vout = _mm_srli_epi64(vtmps, 64-nBitst);

                    vtmps = _mm_loadu_si128((__m128i*)(ps12 + iTmp * 26 + 4));
                    vout1 = _mm_slli_epi64(vtmps, nBitst);
                    vout2 = _mm_or_si128(vout1, vout);   /* 2nd 64 bit */
                    _mm_storel_epi64((__m128i *)(pOutput+cnt), vout2);
                    cnt += 8;
                    vout = _mm_srli_epi64(vtmps, 64-nBitst);

                    vtmps = _mm_loadu_si128((__m128i*)(ps12 + iTmp * 26 + 8));
                    vout1 = _mm_slli_epi64(vtmps, nBitst);
                    vout2 = _mm_or_si128(vout1, vout);   /* 3rd 64 bit */
                    _mm_storel_epi64((__m128i *)(pOutput+cnt), vout2);
                    cnt += 8;
                    vout = _mm_srli_epi64(vtmps, 64-nBitst);

                    vtmps = _mm_loadu_si128((__m128i*)(ps12 + iTmp * 26 + 12));
                    vout1 = _mm_slli_epi64(vtmps, nBitst);
                    vout2 = _mm_or_si128(vout1, vout);   /* 4th 64 bit */
                    _mm_storel_epi64((__m128i *)(pOutput+cnt), vout2);
                    cnt += 8;
                    vout = _mm_srli_epi64(vtmps, 64-nBitst);

                    vtmps = _mm_loadu_si128((__m128i*)(ps12 + iTmp * 26 + 16));
                    vout1 = _mm_slli_epi64(vtmps, nBitst);
                    vout2 = _mm_or_si128(vout1, vout);   /* 5th 64 bit */
                    _mm_storel_epi64((__m128i *)(pOutput+cnt), vout2);
                    cnt += 8;
                    vout = _mm_srli_epi64(vtmps, 64-nBitst);

                    vtmps = _mm_loadu_si128((__m128i*)(ps12 + iTmp * 26 + 20));
                    vout1 = _mm_slli_epi64(vtmps, nBitst);
                    vout2 = _mm_or_si128(vout1, vout);   /* 6th 64 bit */
                    _mm_storel_epi64((__m128i *)(pOutput+cnt), vout2);
                    cnt += 8;
                    vout = _mm_srli_epi64(vtmps, 64-nBitst);

                    vtmps = _mm_loadu_si128((__m128i*)(ps12 + iTmp * 26 + 24));
                    vout1 = _mm_slli_epi64(vtmps, nBitst);
                    vout2 = _mm_or_si128(vout1, vout);   /* update nBitst */
                    nBitst += nBitVec1[iTmp];

                    if(nBitst < 64)
                    {
                        vout1 = _mm_srli_epi64(vconstF, 64 - nBitst);
                        vout = _mm_and_si128(vout1, vout2);
                    }
                    else if( (nBitst >= 64) && (nBitst < 128))
                    {
                        _mm_storel_epi64((__m128i *)(pOutput+cnt), vout2);
                        cnt += 8;
                        nBitst -= 64;
                        vout = _mm_shuffle_epi32(vout2, 78);
                        vout1 = _mm_srli_epi64(vconstF, 64 - nBitst);
                        vout = _mm_and_si128(vout1, vout);
                    }
                    else   /* N <  64 * 3 */
                    {
                        _mm_storel_epi64((__m128i *)(pOutput+cnt), vout2);
                        cnt += 8;
                        nBitst -= 64;

                        vout = _mm_shuffle_epi32(vout2, 78);
                        _mm_storel_epi64((__m128i *)(pOutput+cnt), vout);
                        cnt += 8;
                        nBitst -= 64;

                        /* shift left nBist00 bits */
                        int32_t nBit000 = 64 * 2 - nBitVec1[iTmp];
                        vout3 = _mm_slli_epi64(vtmps, nBit000);
                        vout4 = _mm_shuffle_epi32(vtmps, 78);
                        vout5 = _mm_srli_epi64(vout4, 64 - nBit000);
                        vout6 = _mm_or_si128(vout3, vout5);

                        /* the high nBitst for vout3 is the output */
                        /* nBist < 64 */
                        vout3 = _mm_shuffle_epi32(vout6, 78);
                        vout2 = _mm_srli_epi64(vout3, 64 - nBitst);
                        vout1 = _mm_srli_epi64(vconstF, 64 - nBitst);
                        vout = _mm_and_si128(vout1, vout2);
                    }

                    if( (iTmp == 31))
                        _mm_storel_epi64((__m128i *)(pOutput+cnt), vout);

                }

                 break;
            }
        default :
            printf("nSSE error!\n");
    }

    return;
}


void bit_reverse1(uint8_t *pin, uint8_t *pout ,int32_t len)
{
    int32_t i;
    __m128i vin, vout, vtmp;

    __m128i vmask01 = _mm_set1_epi8 (0x01);
    __m128i vmask02 = _mm_set1_epi8 (0x02);
    __m128i vmask04 = _mm_set1_epi8 (0x04);
    __m128i vmask08 = _mm_set1_epi8 (0x08);
    __m128i vmask10 = _mm_set1_epi8 (0x10);
    __m128i vmask20 = _mm_set1_epi8 (0x20);
    __m128i vmask40 = _mm_set1_epi8 (0x40);
    __m128i vmask80 = _mm_set1_epi8 (0x80);


    int32_t time=len>>4;;
    int32_t tlen=0;
    for (i=0; i<time; i++)
    {

        vin = _mm_lddqu_si128 ((__m128i const*) (pin+tlen));
        vout = _mm_setzero_si128 ();

        vtmp = _mm_srli_epi16 (vin, 7);
        vtmp = _mm_and_si128 (vtmp, vmask01);
        vout = _mm_or_si128 (vout, vtmp);

        vtmp = _mm_srli_epi16 (vin, 5);
        vtmp = _mm_and_si128 (vtmp, vmask02);
        vout = _mm_or_si128 (vout, vtmp);

        vtmp = _mm_srli_epi16 (vin, 3);
        vtmp = _mm_and_si128 (vtmp, vmask04);
        vout = _mm_or_si128 (vout, vtmp);

        vtmp = _mm_srli_epi16 (vin, 1);
        vtmp = _mm_and_si128 (vtmp, vmask08);
        vout = _mm_or_si128 (vout, vtmp);

        vtmp = _mm_slli_epi16 (vin, 1);
        vtmp = _mm_and_si128 (vtmp, vmask10);
        vout = _mm_or_si128 (vout, vtmp);

        vtmp = _mm_slli_epi16 (vin, 3);
        vtmp = _mm_and_si128 (vtmp, vmask20);
        vout = _mm_or_si128 (vout, vtmp);

        vtmp = _mm_slli_epi16 (vin, 5);
        vtmp = _mm_and_si128 (vtmp, vmask40);
        vout = _mm_or_si128 (vout, vtmp);

        vtmp = _mm_slli_epi16 (vin, 7);
        vtmp = _mm_and_si128 (vtmp, vmask80);
        vout = _mm_or_si128 (vout, vtmp);

        _mm_storeu_si128 ((__m128i *) (pout+tlen), vout);
        tlen = tlen + 16;
    }

    for(i=(time<<4);i < len; i++)
    {
        pout[i] = ((pin[i] << 7) & 0x80) | ((pin[i] << 5) & 0x40) | ((pin[i] << 3) & 0x20) | ((pin[i] << 1) & 0x10)
                | ((pin[i] >> 1) & 0x08) | ((pin[i] >> 3) & 0x04) | ((pin[i] >> 5) & 0x02) | ((pin[i] >> 7) & 0x01);
    }
}

/**
 * @brief Rate matching with large code block length, when CaseIndex >92 in TS 136.212 table 5.1.3-3, with SSE instructions
 * @param[in] r index of current code block in all code blocks
 * @param[in] C Total number of code blocks
 * @param[in] direction flag of DL or UL, 1 for DL and 0 for UL
 * @param[in] Nsoft Total number of soft bits according to UE categories
 * @param[in] KMIMO 2, which is related to MIMO type
 * @param[in] MDL_HARQ Maximum number of DL HARQ
 * @param[in] G length of bits before modulation for 1 UE in 1 subframe
 * @param[in] NL Number of layer
 * @param[in] Qm Modulation type, which can be 2/4/6, for QPSK/QAM16/QAM64
 * @param[in] rvidx Redundancy version, which can be 0/1/2/3
 * @param[in] bypass_rvidx If set ignore rvidx and set k0 to 0
 * @param[in] Kidx Postion in turbo code internal interleave table, TS 136.212 table 5.1.3-3
 * @param[in] nLen Length of input data in bits, but the function reads 832 bytes for all the length
 * @param[in] tin0 pointer to input stream 0 from turbo encoder
 * @param[in] tin1 pointer to input stream 1 from turbo encoder
 * @param[in] tin2 pointer to input stream 2 from turbo encoder
 * @param[out] output buffer for data stream after rate matching
 * @param[in] OutputLen Accumulated output length in bytes of rate matching before this code block
 * @param[in] pTable address for bit to byte table
 * @note tin0, tin1, tin2, and output need to be aligned with 128bits
 * @return success: return 0, else: return -1
 */
int32_t lte_rate_matching_lte_k6144_sse(int32_t r, int32_t C, int8_t direction, int32_t Nsoft, int32_t KMIMO,
    int32_t MDL_HARQ, int32_t G,int32_t NL, int32_t Qm, int32_t rvidx, int8_t bypass_rvidx,  int32_t Kidx, int32_t nLen,
    uint8_t * pd0, uint8_t * pd1, uint8_t * pd2,
    uint8_t * pOutput, int32_t OutputLen, int32_t* plen_table )
{

    int32_t k0;
    int32_t nCol = 32;
    int32_t nRow = nLen / 32 + 1;
    int32_t Kw = nCol * nRow * 3;
    int32_t NIR = Nsoft / (KMIMO * (MIN(MDL_HARQ, MLIMIT)));
    int32_t Ncb = Kw;        /*  1, DL;  0, UL */
    if (direction == 1)
          Ncb = MIN((NIR / C), Kw);
    else if(direction == 0)
          Ncb = Kw;
        else
          printf("direction error!\n");

    float G1 = (float) G / (NL * Qm);
    float G1_f = floor(G1);
    float G1_fe = G1 -  G1_f;

    float gamma = (int32_t)G1_f % C + G1_fe;
    int32_t E = NL * Qm;

    if (r <= (C - gamma - 1))
        E *= floori(G, (NL * Qm) * C);
    else
        E *= ceili(G, (NL * Qm) * C);


    /* calculate k0, and k1 without NULL */

    int32_t temp3 = Ncb / (8 * nRow);
    if(Ncb % (8 * nRow) > 0)
        temp3++;

    if (bypass_rvidx == 1)
        k0 = 0;
    else
        k0 =  nRow * (2 * temp3 * rvidx + 2);
/* ================================================================= */
    uint8_t v0[832], v1[832], v2[832];
    uint8_t pOut[20000] = {0};

    subblock_interleaver_d01( nLen, pd0, v0 );
    subblock_interleaver_d01( nLen, pd1, v1 );
    subblock_interleaver_d02( nLen, pd2, v2 );

    int32_t sLen = Ncb / 8 + 2;
        bit_collection(Kidx, nLen, sLen, v0, v1, v2, pOut);

    int32_t tLen = nLen * 3 / 8 + 1;
    bit_selection_complete( E, k0, Ncb, Kidx, tLen, pOut, pOutput );

    OutputLen = OutputLen + E / 8 + 1;

    *plen_table = E;

    return 0;
}
#else
int32_t lte_rate_matching_lte_k6144_sse(int32_t r, int32_t C, int8_t direction, int32_t Nsoft, int32_t KMIMO,
    int32_t MDL_HARQ, int32_t G,int32_t NL, int32_t Qm, int32_t rvidx, int32_t Kidx, int32_t nLen,
    uint8_t * pd0, uint8_t * pd1, uint8_t * pd2,
    uint8_t * pOutput, int32_t OutputLen, int32_t* plen_table)
{
    printf("bblib_rate_matching requires at least SSE4.2 ISA support to run\n");
    return(-1);
}
#endif
