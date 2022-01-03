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
 * @file   rate_matching_lte_short_sse.cpp
 * @brief  Implementation LTE rate matching, Rate matching with short code block length, when CaseIndex<92 in TS 136.212 table 5.1.3-3, with SSE instructions
*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <smmintrin.h> /* SSE 4 for media */
#include <string.h>

#include "common_typedef_sdk.h"
#include "divide.h"
#include "phy_rate_match_internal.h"
#if defined(_BBLIB_SSE4_2_) || defined(_BBLIB_AVX2_) || defined(_BBLIB_AVX512_)
int32_t unpack_byte_to_bit(uint8_t* pInput, uint8_t* pOutput, int32_t Len, int8_t direction)
{
    int32_t ByteIdx, LenR, LenM, Merge;
    __m128i seq_byte, shift;
    seq_byte = _mm_setzero_si128();
    if (direction == 1) // DL
    {
        shift = _mm_set_epi8(0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15);
    }
    else // UL
    {
        shift = _mm_set_epi8(7,6,5,4,3,2,1,0,15,14,13,12,11,10,9,8);
    }
    LenR = Len & 0x0000000F;
    LenM = Len & 0xFFFFFFF0;
    for(ByteIdx = 0; ByteIdx < LenM; ByteIdx += 16)
    {
        seq_byte = _mm_load_si128((__m128i *)pInput);
        pInput += 16;
        seq_byte = _mm_shuffle_epi8(seq_byte,shift);
        Merge = _mm_movemask_epi8(seq_byte);
        *pOutput =(uint8_t) (Merge >> 8 & 0x000000FF);
        pOutput ++;
        *pOutput = (uint8_t) (Merge & 0x000000FF);
        pOutput ++;
    }
    if(LenR == 0)
    {
        return 0;
    }
    else if (LenR > 0)
    {
        uint8_t merge_char = 0;
        if (direction == 1) //DL
        {
            for (ByteIdx = 0; ByteIdx < LenR; ByteIdx++)
            {
                merge_char = merge_char + ((*pInput) >> ByteIdx);
                *pInput++;
            }
        }
        else    //else UL
        {
            uint8_t temp8;
            for (ByteIdx = 0; ByteIdx < LenR; ByteIdx++)
            {
                temp8 = -*pInput;
                merge_char = merge_char + ((temp8) << ByteIdx);
                *pInput++;
            }
        }
        *pOutput = merge_char;
        return 0;
    }
    else
    {
        printf("Please check LenR:%d Len:%d\n", LenR, Len);
        return 1;
    }
}
/**
 * @brief Rate matching with large code block length, when CaseIndex<92 in TS 136.212 table 5.1.3-3, with SSE instructions
 * @param[in] r index of current code block in all code blocks
 * @param[in] C Total number of code blocks
 * @param[in] direction flag of DL or UL, 1 for DL and 0 for UL
 * @param[in] Nsoft Total number of soft bits according to UE categories
 * @param[in] KMIMO 2, which is related to MIMO type
 * @param[in] MDL_HARQ Maximum number of DL HARQ
 * @param[in] G length of bits before modulation for 1 UE in 1 subframe
 * @param[in] NL Number of layer
 * @param[in] Qm Modulation type, which can be 2/4/6
 * @param[in] rvidx Redundancy version, which can be 0/1/2/3, for QPSK/QAM16/QAM64
 * @param[in] bypass_rvidx If set ignore rvidx and set k0 to 0
 * @param[in] Kidx Postion in turbo code internal interleave table, TS 136.212 table 5.1.3-3
 * @param[in] nLen Length of input data in bits
 * @param[in] tin0 pointer to input stream 0 from turbo encoder
 * @param[in] tin1 pointer to input stream 1 from turbo encoder
 * @param[in] tin2 pointer to input stream 2 from turbo encoder
 * @param[out] output buffer for data stream after rate matching
 * @param[in] OutputLen Accumulated output length in bytes of rate matching before this code block
 * @param[in] pTable address for bit to byte table
 * @note tin0, tin1, tin2, and output need to be aligned with 128bits
 * @return success: return 0, else: return -1
 */
int32_t rate_matching_turbo_lte_short_sse(int32_t r, int32_t C, int8_t direction, int32_t Nsoft, int32_t KMIMO,
                    int32_t MDL_HARQ, int32_t G,int32_t NL, int32_t Qm, int32_t rvidx, int8_t bypass_rvidx, int32_t Kidx,
                    int32_t nLen, uint8_t * tin0,uint8_t * tin1,
                    uint8_t * tin2, uint8_t * output,
                    uint32_t OutputLen,uint8_t *pTable)
{
    int32_t k0,i;
    int32_t nCol = 32;
    int32_t nRow = nLen / 32 + 1;
    uint8_t *v0, *tmp1;
    uint8_t dd[18444];
    uint8_t *d0 = dd;
    uint8_t *d1 = d0 + nLen;
    uint8_t *d2 = d0 + (nLen << 1);
    /* here, d0, d1, d2 should be in the continuous address. */
    uint8_t tbuffer[18444];
	__align(16) uint8_t tmpOut[18444];
    uint8_t *s0 = tmpOut;

    /* Part 0: bit to byte transfer */
    int32_t LenR = ((nLen - 4) >> 3) & 0x00000001;
    int32_t LenM = ((nLen - 4) >> 3) & 0xFFFFFFFE;

    __m128i InByte0, InByte1, InByte2, InByte00, InByte11, InByte22;
    InByte0 = _mm_setzero_si128();
    InByte00 = _mm_setzero_si128();
    InByte1 = _mm_setzero_si128();
    InByte11 = _mm_setzero_si128();
    InByte2 = _mm_setzero_si128();
    InByte22 = _mm_setzero_si128();

    for(int32_t idx = 0; idx < LenM; idx += 2)
    {
        InByte0 = _mm_load_si128((__m128i *)(pTable + ((*tin0) << 4)));
        tin0++;
        InByte00 = _mm_load_si128((__m128i *)(pTable + ((*tin0) << 4)));
        tin0++;
        InByte0 = _mm_blend_epi16(InByte0, InByte00, 0xF0);
        _mm_storeu_si128((__m128i *)d0, InByte0);
        d0 += 16;

        InByte1 = _mm_load_si128((__m128i *)(pTable + ((*tin1) << 4)));
        tin1++;
        InByte11 = _mm_load_si128((__m128i *)(pTable + ((*tin1) << 4)));
        tin1++;
        InByte1 = _mm_blend_epi16(InByte1, InByte11, 0xF0);
        _mm_storeu_si128((__m128i *)d1, InByte1);
        d1 += 16;

        InByte2 = _mm_load_si128((__m128i *)(pTable + ((*tin2) << 4)));
        tin2++;
        InByte22 = _mm_load_si128((__m128i *)(pTable + ((*tin2) << 4)));
        tin2++;
        InByte2 = _mm_blend_epi16(InByte2, InByte22, 0xF0);
        _mm_storeu_si128((__m128i *)d2, InByte2);
        d2 += 16;
    }

    if(LenR == 1)
    {
        d0[7] = ((*tin0) << 7) & 0x80;
        d0[6] = ((*tin0) << 6) & 0x80;
        d0[5] = ((*tin0) << 5) & 0x80;
        d0[4] = ((*tin0) << 4) & 0x80;
        d0[3] = ((*tin0) << 3) & 0x80;
        d0[2] = ((*tin0) << 2) & 0x80;
        d0[1] = ((*tin0) << 1) & 0x80;
        d0[0] = (*tin0) & 0x80;
        d0 += 8;
        tin0++;

        d1[7] = ((*tin1) << 7) & 0x80;
        d1[6] = ((*tin1) << 6) & 0x80;
        d1[5] = ((*tin1) << 5) & 0x80;
        d1[4] = ((*tin1) << 4) & 0x80;
        d1[3] = ((*tin1) << 3) & 0x80;
        d1[2] = ((*tin1) << 2) & 0x80;
        d1[1] = ((*tin1) << 1) & 0x80;
        d1[0] = (*tin1) & 0x80;
        d1 += 8;
        tin1++;

        d2[7] = ((*tin2) << 7) & 0x80;
        d2[6] = ((*tin2) << 6) & 0x80;
        d2[5] = ((*tin2) << 5) & 0x80;
        d2[4] = ((*tin2) << 4) & 0x80;
        d2[3] = ((*tin2) << 3) & 0x80;
        d2[2] = ((*tin2) << 2) & 0x80;
        d2[1] = ((*tin2) << 1) & 0x80;
        d2[0] = (*tin2) & 0x80;
        d2 += 8;
        tin2++;
    }

    d0[3] = ((*tin0) << 3) & 0x80;
    d0[2] = ((*tin0) << 2) & 0x80;
    d0[1] = ((*tin0) << 1) & 0x80;
    d0[0] = (*tin0) & 0x80 ;

    d1[3] = ((*tin1) << 3) & 0x80;
    d1[2] = ((*tin1) << 2) & 0x80;
    d1[1] = ((*tin1) << 1) & 0x80;
    d1[0] = (*tin1) & 0x80;

    d2[3] = ((*tin2) << 3) & 0x80;
    d2[2] = ((*tin2) << 2) & 0x80;
    d2[1] = ((*tin2) << 1) & 0x80;
    d2[0] = (*tin2) & 0x80;

    d0 = &dd[0];

    /* ======== Part1: calculation of number E ======== */

    int32_t Kw = nCol * nRow * 3;
	int32_t NIR = Nsoft / (KMIMO * (MIN(MDL_HARQ, MLIMIT)));
    int32_t Ncb = Kw;    /*  1, DL;  0, UL */
    if (direction == 1)  Ncb = MIN(floori(NIR, C), Kw);
    else if(direction == 0)  Ncb = Kw;

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

    /* ============================================================== */
    /* Kidx is the type of codeblock length in 188 selection */
    int32_t Num_NULL = g_nNum_NULL[Kidx];
    int32_t ni, nj;
    for(ni = 0; ni < Num_NULL; ni++)
    {
        if(g_nIndex_NULL[Kidx][ni] > k0)
            break;
    }
    int32_t k0_n = k0 - ni;
    /* ni stands for the NULL number between 1 ~ k0  */

    for(nj = 0; nj < Num_NULL; nj++)
    {
        if(g_nIndex_NULL[Kidx][nj] > Ncb)
            break;
    }
    int32_t Ncb_n = Ncb - nj;
    /* nj stands for the NULL number between 1 ~ Ncb  */

    /* ====== Part 2, sub-block interleave using look up table;====== */
    /* here optimize to only calculate those bits that are used*/

    v0 = tbuffer;
    int32_t t1n, t2, t2n, t3;
    int32_t t1 = k0_n % Ncb;     /* reading starting position*/

    int32_t t_beg = 0;    /* the starting position of table look up*/
    int32_t t_end = Ncb_n;    /* the ending position of table look up*/

    if(t1 + E <= Ncb_n)
    {
        t_beg = t1;
        t_end = t1 + E;
    }
    else
    {
        t_beg = 0;
        t_end = Ncb_n;
    }

    for(i = t_beg; i < t_end; i++)
    {
        tmp1 = d0 + g_ratetable[Kidx][i];
        v0[i] = *tmp1;
    }

    /* ==========   Part 3: bit collection  ========= */
    /* copy the bits from 0~ Ncb_n, starting from k0_n;*/
    if(t1 + E <= Ncb_n)
    {
        memcpy(s0, &tbuffer[t1], E);
    }
    else
    {
        t1n = (Ncb_n - t1);
        t2 = (E - t1n) / Ncb_n; /* times of full copy from 0~Ncb_n*/

        memcpy(s0, &tbuffer[t1], t1n);

        if(t2 == 0)
            memcpy(&s0[t1n], &tbuffer[0], E - t1n);
        else
        {
            t2n = t1n + t2 * Ncb_n;
            t3 = E - t2n;    /* number of last copy */
            for(int32_t icp = 0; icp < t2; icp++)
                memcpy(&s0[t1n + Ncb_n * icp], &tbuffer[0], Ncb_n);
            memcpy(&s0[t2n], &tbuffer[0], t3);
        }
    }

    unpack_byte_to_bit(s0, output, E, direction);
    OutputLen = OutputLen + E/8;

    return 0;

}
#else
int32_t rate_matching_turbo_lte_short_sse(int32_t r, int32_t C, int8_t direction, int32_t Nsoft, int32_t KMIMO, 
                    int32_t MDL_HARQ, int32_t G,int32_t NL, int32_t Qm, int32_t rvidx,int32_t Kidx,
                    int32_t nLen, uint8_t * tin0,uint8_t * tin1,
                    uint8_t * tin2, uint8_t * output,
                    uint32_t OutputLen,uint8_t *pTable)
{
    printf("bblib_rate_matching requires at least SSE4.2 ISA support to run\n");
    exit(-1);
}
#endif