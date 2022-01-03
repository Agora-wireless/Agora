/*******************************************************************************
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
*******************************************************************************/

/*
 * @file   phy_turbo_decoder_8windows_see.h
 * @brief  this file performs the turbo Decoder when CW size is multiple of 8
 */

#include <cstdint>
#include <cstdio>

#include "immintrin.h"

#include "phy_turbo_internal.h"
#include "phy_crc.h"
#include "phy_turbo.h"
#if defined(_BBLIB_SSE4_2_) || defined(_BBLIB_AVX2_) || defined(_BBLIB_AVX512_)
#define INF 32768

static const int32_t table_interleaver[188][3] = {
    40,3,10, 48,7,12, 56,19,42, 64,7,16, 72,7,18, 80,11,20, 88,5,22, 96,11,24, 104,7,26, 112,41,84,
    120,103,90, 128,15,32, 136,9,34, 144,17,108, 152,9,38, 160,21,120, 168,101,84, 176,21,44, 184,57,46, 192,23,48,
    200,13,50, 208,27,52, 216,11,36, 224,27,56, 232,85,58, 240,29,60, 248,33,62, 256,15,32, 264,17,198, 272,33,68,
    280,103,210, 288,19,36, 296,19,74, 304,37,76, 312,19,78, 320,21,120, 328,21,82, 336,115,84, 344,193,86, 352,21,44,
    360,133,90, 368,81,46, 376,45,94, 384,23,48, 392,243,98, 400,151,40, 408,155,102, 416,25,52, 424,51,106, 432,47,72,
    440,91,110, 448,29,168, 456,29,114, 464,247,58, 472,29,118, 480,89,180, 488,91,122, 496,157,62, 504,55,84, 512,31,64,
    528,17,66, 544,35,68, 560,227,420, 576,65,96, 592,19,74, 608,37,76, 624,41,234, 640,39,80, 656,185,82, 672,43,252,
    688,21,86, 704,155,44, 720,79,120, 736,139,92, 752,23,94, 768,217,48, 784,25,98, 800,17,80, 816,127,102, 832,25,52,
    848,239,106, 864,17,48, 880,137,110, 896,215,112, 912,29,114, 928,15,58, 944,147,118, 960,29,60, 976,59,122, 992,65,124,
    1008,55,84, 1024,31,64, 1056,17,66, 1088,171,204, 1120,67,140, 1152,35,72, 1184,19,74, 1216,39,76, 1248,19,78, 1280,199,240,
    1312,21,82, 1344,211,252, 1376,21,86, 1408,43,88, 1440,149,60, 1472,45,92, 1504,49,846, 1536,71,48, 1568,13,28, 1600,17,80,
    1632,25,102, 1664,183,104, 1696,55,954, 1728,127,96, 1760,27,110, 1792,29,112, 1824,29,114, 1856,57,116, 1888,45,354, 1920,31,120,
    1952,59,610, 1984,185,124, 2016,113,420, 2048,31,64, 2112,17,66, 2176,171,136, 2240,209,420, 2304,253,216, 2368,367,444, 2432,265,456,
    2496,181,468, 2560,39,80, 2624,27,164, 2688,127,504, 2752,143,172, 2816,43,88, 2880,29,300, 2944,45,92, 3008,157,188, 3072,47,96,
    3136,13,28, 3200,111,240, 3264,443,204, 3328,51,104, 3392,51,212, 3456,451,192, 3520,257,220, 3584,57,336, 3648,313,228, 3712,271,232,
    3776,179,236, 3840,331,120, 3904,363,244, 3968,375,248,  4032,127,168, 4096,31,64, 4160,33,130, 4224,43,264, 4288,33,134, 4352,477,408,
    4416,35,138, 4480,233,280, 4544,357,142, 4608,337,480, 4672,37,146, 4736,71,444, 4800,71,120, 4864,37,152, 4928,39,462, 4992,127,234,
    5056,39,158, 5120,39,80, 5184,31,96, 5248,113,902, 5312,41,166, 5376,251,336, 5440,43,170, 5504,21,86, 5568,43,174, 5632,45,176,
    5696,45,178, 5760,161,120, 5824,89,182, 5888,323,184, 5952,47,186, 6016,23,94, 6080,47,190, 6144,263,480,
};

static const __m128i Zero = _mm_setzero_si128();
static const __m128i Neg = _mm_set1_epi16(-1);
static const __m128i NextBit0 = _mm_setr_epi16(-1,-1,1,1,1,1,-1,-1);
static const __m128i NextBit1 = _mm_setr_epi16(1,1,-1,-1,-1,-1,1,1);
static const __m128i Shuffle = _mm_setr_epi8(0,2,4,6,8,10,12,14, 16,16,16,16,16,16,16,16);
static const __m128i MoveShuffle = _mm_setr_epi8(7,6,5,4,3,2,1,0,15,14,13,12,11,10,9,8);
static const __m128i NextState0 = _mm_setr_epi8(0,1, 8,9, 10,11, 2,3, 4,5, 12,13, 14,15, 6,7);
static const __m128i NextState1 = _mm_setr_epi8(8,9, 0,1, 2,3, 10,11, 12,13, 4,5, 6,7, 14,15);
static const __m128i LastState0 = _mm_setr_epi8(0,1, 6,7, 8,9, 14,15, 2,3, 4,5, 10,11, 12,13);
static const __m128i LastState1 = _mm_setr_epi8(2,3, 4,5, 10,11, 12,13, 0,1, 6,7, 8,9, 14,15);

#define Calculate_Gamma(sys, par, ext, out0, out1) \
{ \
    par0 = _mm_sign_epi16(par, NextBit0); \
    par1 = _mm_sign_epi16(par, NextBit1); \
    out0 = _mm_subs_epi16(par0, sys); \
    out0 = _mm_subs_epi16(out0, ext); \
    out0 = _mm_srai_epi16(out0, 1); \
    out1 = _mm_adds_epi16(par1, sys); \
    out1 = _mm_adds_epi16(out1, ext); \
    out1 = _mm_srai_epi16(out1, 1); \
}

#define Calculate_LLR(ag_addr, betaIn, betaOut, llrOut0, llrOut1) \
{ \
    gamma1 = _mm_load_si128((__m128i *)ag_addr); ag_addr--; \
    gamma0 = _mm_load_si128((__m128i *)ag_addr); ag_addr--; \
    alpha = _mm_load_si128((__m128i *)ag_addr); ag_addr--; \
    beta0 = _mm_shuffle_epi8(betaIn, NextState0); \
    beta1 = _mm_shuffle_epi8(betaIn, NextState1); \
    llrOut0 = _mm_adds_epi16(alpha, gamma0); \
    llrOut0 = _mm_adds_epi16(llrOut0, beta0); \
    llrOut1 = _mm_adds_epi16(alpha, gamma1); \
    llrOut1 = _mm_adds_epi16(llrOut1, beta1); \
     \
    beta0 = _mm_adds_epi16(beta0, gamma0); \
    beta1 = _mm_adds_epi16(beta1, gamma1); \
    betaOut = _mm_max_epi16(beta0, beta1); \
}

#define _MM_TRANSEPOSE8_EPI16(llr0, llr1, llr2, llr3, llr4, llr5, llr6, llr7) \
{ \
    __m128i tmp0, tmp1, tmp2, tmp3, tmp4, tmp5, tmp6, tmp7, tmp; \
    tmp0 = _mm_unpacklo_epi16(llr0, llr1); \
    tmp1 = _mm_unpackhi_epi16(llr0, llr1); \
    tmp2 = _mm_unpacklo_epi16(llr2, llr3); \
    tmp3 = _mm_unpackhi_epi16(llr2, llr3); \
    tmp4 = _mm_unpacklo_epi16(llr4, llr5); \
    tmp5 = _mm_unpackhi_epi16(llr4, llr5); \
    tmp6 = _mm_unpacklo_epi16(llr6, llr7); \
    tmp7 = _mm_unpackhi_epi16(llr6, llr7); \
     \
    tmp = _mm_unpacklo_epi32(tmp0, tmp2); \
    tmp0 = _mm_unpackhi_epi32(tmp0, tmp2); \
    tmp2 = _mm_unpacklo_epi32(tmp1, tmp3); \
    tmp1 = _mm_unpackhi_epi32(tmp1, tmp3); \
    tmp3 = _mm_unpacklo_epi32(tmp4, tmp6); \
    tmp4 = _mm_unpackhi_epi32(tmp4, tmp6); \
    tmp6 = _mm_unpacklo_epi32(tmp5, tmp7); \
    tmp5 = _mm_unpackhi_epi32(tmp5, tmp7); \
     \
    llr0 = _mm_unpacklo_epi64(tmp, tmp3); \
    llr1 = _mm_unpackhi_epi64(tmp, tmp3); \
    llr2 = _mm_unpacklo_epi64(tmp0, tmp4); \
    llr3 = _mm_unpackhi_epi64(tmp0, tmp4); \
    llr4 = _mm_unpacklo_epi64(tmp2, tmp6); \
    llr5 = _mm_unpackhi_epi64(tmp2, tmp6); \
    llr6 = _mm_unpacklo_epi64(tmp1, tmp5); \
    llr7 = _mm_unpackhi_epi64(tmp1, tmp5); \
}

#define _MM_MAX8_EPI16(llr0, llr1, llr2, llr3, llr4, llr5, llr6, llr7, maxOut) \
{ \
    __m128i tmp0, tmp1; \
    tmp0 = _mm_max_epi16(llr0, llr1); \
    tmp1 = _mm_max_epi16(tmp0, llr2); \
    tmp0 = _mm_max_epi16(tmp1, llr3); \
    tmp1 = _mm_max_epi16(tmp0, llr4); \
    tmp0 = _mm_max_epi16(tmp1, llr5); \
    tmp1 = _mm_max_epi16(tmp0, llr6); \
    maxOut = _mm_max_epi16(tmp1, llr7); \
}

void create_interleaver(int32_t id, uint16_t *interleaverTable)
{
    int32_t i;
    uint16_t pos;
    int32_t index = (id - 40) / 8;
    int32_t k = table_interleaver[index][0];
    int32_t f1 = table_interleaver[index][1];
    int32_t f2 = table_interleaver[index][2];
    int32_t delta = f1 + f2;
    int32_t m = 2*f2;

    pos = 0;
    for (i=0; i<id; ++i)
    {
        //pos = ((f1 + f2 * i) * i) % k;
        interleaverTable[i] = pos;
        pos = (pos + delta) % k;
        delta = (delta + m) % k;
    }
}

void siso1_decoder(int32_t K, int8_t *pSys, int16_t *pLLRin, int16_t *pLLRout,
                           int16_t *pAG, int8_t *tailBits, int16_t *tailLLRin, int16_t *tailLLRout,
                           int16_t *pTailAG, uint8_t *out_bit)
{
    int32_t kIdx, i;
    uint16_t bitTemp;

    __m128i par0, par1;
    __m128i gamma0, gamma1, alpha, alpha0, alpha1, beta0, beta1, system, parity, extrinsic, maxOut0, maxOut1;
    __m128i llr0, llr1, llr2, llr3, llr4, llr5, llr6, llr7, llr8, llr9, llr10, llr11, llr12, llr13, llr14, llr15;
    __m128i temp0, temp1, llrtemp, llrtemp0, llrtemp1;
    __m128i zerotemp0 = _mm_setzero_si128();
    __m128i zerotemp1 = _mm_setzero_si128();

    int32_t Lw = K / 8;
    int32_t TAIL_BITS = 3;
    int8_t *pSysTmp = pSys;
    int16_t *pLLRinTmp = (int16_t *)pLLRin;
    int8_t *tSysTmp = tailBits;
    int16_t *tinTmp = (int16_t *)tailLLRin;
    __m128i *pAGTmp = (__m128i *)pAG;
    __m128i *pTailTmp = (__m128i *)pTailAG;

    //Calculate Alpha
    temp0 = _mm_setr_epi16(0, -INF, -INF, -INF, -INF, -INF, -INF, -INF);
    for (kIdx=0; kIdx<K; ++kIdx)
    {
        _mm_store_si128((__m128i *)pAGTmp, temp0); pAGTmp++;
        system = _mm_set1_epi16((int16_t)(*pSysTmp)); pSysTmp++;
        parity = _mm_set1_epi16((int16_t)(*pSysTmp)); pSysTmp++;
        extrinsic = _mm_set1_epi16((int16_t)(*pLLRinTmp)); pLLRinTmp++;
        Calculate_Gamma(system, parity, extrinsic, gamma0, gamma1);
        _mm_store_si128((__m128i *)pAGTmp, gamma0); pAGTmp++;
        _mm_store_si128((__m128i *)pAGTmp, gamma1); pAGTmp++;
        gamma0 = _mm_shuffle_epi8(gamma0, LastState0);
        gamma1 = _mm_shuffle_epi8(gamma1, LastState1);
        alpha0 = _mm_shuffle_epi8(temp0, LastState0);
        alpha1 = _mm_shuffle_epi8(temp0, LastState1);
        alpha0 = _mm_adds_epi16(alpha0, gamma0);
        alpha1 = _mm_adds_epi16(alpha1, gamma1);
        temp0 = _mm_max_epi16(alpha0, alpha1);
    }
    for (kIdx=0; kIdx<TAIL_BITS-1; ++kIdx)
    {
        _mm_store_si128((__m128i *)pTailTmp, temp0); pTailTmp++;
        system = _mm_set1_epi16((int16_t)(*tSysTmp)); tSysTmp++;
        parity = _mm_set1_epi16((int16_t)(*tSysTmp)); tSysTmp++;
        extrinsic = _mm_set1_epi16((int16_t)(*tinTmp)); tinTmp++;
        Calculate_Gamma(system, parity, extrinsic, gamma0, gamma1);
        _mm_store_si128((__m128i *)pTailTmp, gamma0); pTailTmp++;
        _mm_store_si128((__m128i *)pTailTmp, gamma1); pTailTmp++;
        gamma0 = _mm_shuffle_epi8(gamma0, LastState0);
        gamma1 = _mm_shuffle_epi8(gamma1, LastState1);
        alpha0 = _mm_shuffle_epi8(temp0, LastState0);
        alpha1 = _mm_shuffle_epi8(temp0, LastState1);
        alpha0 = _mm_adds_epi16(alpha0, gamma0);
        alpha1 = _mm_adds_epi16(alpha1, gamma1);
        temp0 = _mm_max_epi16(alpha0, alpha1);
    }
    _mm_store_si128((__m128i *)pTailTmp, temp0); pTailTmp++;
    system = _mm_set1_epi16((int16_t)(*tSysTmp)); tSysTmp++;
    parity = _mm_set1_epi16((int16_t)(*tSysTmp));
    extrinsic = _mm_set1_epi16((int16_t)(*tinTmp));
    Calculate_Gamma(system, parity, extrinsic, gamma0, gamma1);
    _mm_store_si128((__m128i *)pTailTmp, gamma0); pTailTmp++;
    _mm_store_si128((__m128i *)pTailTmp, gamma1);

    //Calculate Beta and LLR
    kIdx = TAIL_BITS-1;
    pTailTmp = (__m128i *)pTailAG + 3*TAIL_BITS - 1;
    temp0 = _mm_setr_epi16(0, -INF, -INF, -INF, -INF, -INF, -INF, -INF);
    Calculate_LLR((__m128i *)pTailTmp, temp0, temp1, llr0, llr8);
    Calculate_LLR((__m128i *)pTailTmp, temp1, temp0, llr1, llr9);
    Calculate_LLR((__m128i *)pTailTmp, temp0, temp1, llr2, llr10);
    _MM_TRANSEPOSE8_EPI16(llr10, llr2, llr9, llr1, llr8, llr0, zerotemp0, zerotemp1);
    _MM_MAX8_EPI16(llr0, llr8, llr1, llr9, llr2, llr10, zerotemp0, zerotemp1, maxOut0);
    llrtemp0 = _mm_hsubs_epi16(maxOut0, Zero);
    //llr - system - extrinsic(next)
    tailLLRout[kIdx-2] = (_mm_extract_epi16(llrtemp0, 0) - tailBits[2*(kIdx - 2)] - tailLLRin[kIdx-2]) * 0.75;
    tailLLRout[kIdx-1] = (_mm_extract_epi16(llrtemp0, 1) - tailBits[2*(kIdx - 1)] - tailLLRin[kIdx-1]) * 0.75;
    tailLLRout[kIdx] = (_mm_extract_epi16(llrtemp0, 2) - tailBits[2*kIdx] - tailLLRin[kIdx]) * 0.75;

    kIdx = K-1;
    temp0 = temp1;
    pSysTmp = pSys + Lw*16 - 16;
    pLLRinTmp = pLLRin + Lw*8 - 8;
    int16_t *pLLRoutTmp = pLLRout + Lw*8 - 8;
    pAGTmp = (__m128i *)pAG + 3*K - 1;
    Calculate_LLR((__m128i *)pAGTmp, temp0, temp1, llr0, llr8);
    Calculate_LLR((__m128i *)pAGTmp, temp1, temp0, llr1, llr9);
    Calculate_LLR((__m128i *)pAGTmp, temp0, temp1, llr2, llr10);
    Calculate_LLR((__m128i *)pAGTmp, temp1, temp0, llr3, llr11);
    Calculate_LLR((__m128i *)pAGTmp, temp0, temp1, llr4, llr12);
    Calculate_LLR((__m128i *)pAGTmp, temp1, temp0, llr5, llr13);
    Calculate_LLR((__m128i *)pAGTmp, temp0, temp1, llr6, llr14);
    Calculate_LLR((__m128i *)pAGTmp, temp1, temp0, llr7, llr15);
    _MM_TRANSEPOSE8_EPI16(llr7, llr6, llr5, llr4, llr3, llr2, llr1, llr0);
    _MM_MAX8_EPI16(llr0, llr1, llr2, llr3, llr4, llr5, llr6, llr7, maxOut0);
    _MM_TRANSEPOSE8_EPI16(llr15, llr14, llr13, llr12, llr11, llr10, llr9, llr8);
    _MM_MAX8_EPI16(llr8, llr9, llr10, llr11, llr12, llr13, llr14, llr15, maxOut1);
    llrtemp1 = _mm_subs_epi16(maxOut1, maxOut0);
    //llr - system - extrinsic(next)
    system = _mm_load_si128((__m128i *)pSysTmp); pSysTmp -= 16;
    system = _mm_cvtepi8_epi16(_mm_shuffle_epi8(system, Shuffle));
    extrinsic = _mm_load_si128((__m128i *)pLLRinTmp); pLLRinTmp -= 8;
    llrtemp = _mm_subs_epi16(_mm_subs_epi16(llrtemp1, system), extrinsic);
    llrtemp = _mm_subs_epi16(llrtemp, _mm_srai_epi16(llrtemp, 2));   //llrE * 0.75
    _mm_store_si128((__m128i *)pLLRoutTmp, llrtemp); pLLRoutTmp -= 8;

    i = (K>>3) - 1;
    llrtemp1 = _mm_sign_epi16(llrtemp1, Neg);
    llrtemp0 = _mm_sign_epi16(llrtemp0, Neg);
    llrtemp = _mm_packs_epi16(llrtemp1, llrtemp0);
    llrtemp = _mm_shuffle_epi8(llrtemp, MoveShuffle);
    bitTemp = (uint16_t)_mm_movemask_epi8(llrtemp);
    out_bit[i] = (uint8_t)bitTemp;

    for (kIdx=K-9, i=i-2; kIdx>0; kIdx-=16, i-=2)
    {
        Calculate_LLR((__m128i *)pAGTmp, temp0, temp1, llr0, llr8);
        Calculate_LLR((__m128i *)pAGTmp, temp1, temp0, llr1, llr9);
        Calculate_LLR((__m128i *)pAGTmp, temp0, temp1, llr2, llr10);
        Calculate_LLR((__m128i *)pAGTmp, temp1, temp0, llr3, llr11);
        Calculate_LLR((__m128i *)pAGTmp, temp0, temp1, llr4, llr12);
        Calculate_LLR((__m128i *)pAGTmp, temp1, temp0, llr5, llr13);
        Calculate_LLR((__m128i *)pAGTmp, temp0, temp1, llr6, llr14);
        Calculate_LLR((__m128i *)pAGTmp, temp1, temp0, llr7, llr15);
        _MM_TRANSEPOSE8_EPI16(llr7, llr6, llr5, llr4, llr3, llr2, llr1, llr0);
        _MM_MAX8_EPI16(llr0, llr1, llr2, llr3, llr4, llr5, llr6, llr7, maxOut0);
        _MM_TRANSEPOSE8_EPI16(llr15, llr14, llr13, llr12, llr11, llr10, llr9, llr8);
        _MM_MAX8_EPI16(llr8, llr9, llr10, llr11, llr12, llr13, llr14, llr15, maxOut1);
        llrtemp0 = _mm_subs_epi16(maxOut1, maxOut0);
        system = _mm_load_si128((__m128i *)pSysTmp); pSysTmp -= 16;
        system = _mm_cvtepi8_epi16(_mm_shuffle_epi8(system, Shuffle));
        extrinsic = _mm_load_si128((__m128i *)pLLRinTmp); pLLRinTmp -= 8;
        llrtemp = _mm_subs_epi16(_mm_subs_epi16(llrtemp0, system), extrinsic);
        llrtemp = _mm_subs_epi16(llrtemp, _mm_srai_epi16(llrtemp, 2));
        _mm_store_si128((__m128i *)pLLRoutTmp, llrtemp); pLLRoutTmp -= 8;

        Calculate_LLR((__m128i *)pAGTmp, temp0, temp1, llr0, llr8);
        Calculate_LLR((__m128i *)pAGTmp, temp1, temp0, llr1, llr9);
        Calculate_LLR((__m128i *)pAGTmp, temp0, temp1, llr2, llr10);
        Calculate_LLR((__m128i *)pAGTmp, temp1, temp0, llr3, llr11);
        Calculate_LLR((__m128i *)pAGTmp, temp0, temp1, llr4, llr12);
        Calculate_LLR((__m128i *)pAGTmp, temp1, temp0, llr5, llr13);
        Calculate_LLR((__m128i *)pAGTmp, temp0, temp1, llr6, llr14);
        Calculate_LLR((__m128i *)pAGTmp, temp1, temp0, llr7, llr15);
        _MM_TRANSEPOSE8_EPI16(llr7, llr6, llr5, llr4, llr3, llr2, llr1, llr0);
        _MM_MAX8_EPI16(llr0, llr1, llr2, llr3, llr4, llr5, llr6, llr7, maxOut0);
        _MM_TRANSEPOSE8_EPI16(llr15, llr14, llr13, llr12, llr11, llr10, llr9, llr8);
        _MM_MAX8_EPI16(llr8, llr9, llr10, llr11, llr12, llr13, llr14, llr15, maxOut1);
        llrtemp1 = _mm_subs_epi16(maxOut1, maxOut0);
        system = _mm_load_si128((__m128i *)pSysTmp); pSysTmp -= 16;
        system = _mm_cvtepi8_epi16(_mm_shuffle_epi8(system, Shuffle));
        extrinsic = _mm_load_si128((__m128i *)pLLRinTmp); pLLRinTmp -= 8;
        llrtemp = _mm_subs_epi16(_mm_subs_epi16(llrtemp1, system), extrinsic);
        llrtemp = _mm_subs_epi16(llrtemp, _mm_srai_epi16(llrtemp, 2));
        _mm_store_si128((__m128i *)pLLRoutTmp, llrtemp); pLLRoutTmp -= 8;

        llrtemp1 = _mm_sign_epi16(llrtemp1, Neg);
        llrtemp0 = _mm_sign_epi16(llrtemp0, Neg);
        llrtemp = _mm_packs_epi16(llrtemp1, llrtemp0);
        llrtemp = _mm_shuffle_epi8(llrtemp, MoveShuffle);
        bitTemp = _mm_movemask_epi8(llrtemp);
        *(uint16_t *)(out_bit + i) = bitTemp;
    }
}

void siso2_decoder(int32_t K, int8_t *pSys, int16_t *pLLRin, int16_t *pLLRout,
                           int16_t *pAG, int8_t *tailBits, int16_t *tailLLRin,
                           int16_t *tailLLRout, int16_t *pTailAG)
{
    int32_t kIdx;

    __m128i par0, par1;
    __m128i gamma0, gamma1, alpha, alpha0, alpha1, beta0, beta1, system, parity, extrinsic, maxOut0, maxOut1;
    __m128i llr0, llr1, llr2, llr3, llr4, llr5, llr6, llr7, llr8, llr9, llr10, llr11, llr12, llr13, llr14, llr15;
    __m128i temp0, temp1, llrtemp, llrtemp0;
    __m128i zerotemp0 = _mm_setzero_si128();
    __m128i zerotemp1 = _mm_setzero_si128();

    int32_t Lw = K / 8;
    int32_t TAIL_BITS = 3;
    int8_t *pSysTmp = pSys;
    int16_t *pLLRinTmp = (int16_t *)pLLRin;
    int8_t *tSysTmp = tailBits;
    int16_t *tinTmp = (int16_t *)tailLLRin;
    __m128i *pAGTmp = (__m128i *)pAG;
    __m128i *pTailTmp = (__m128i *)pTailAG;

    //Calculate Alpha
    temp0 = _mm_setr_epi16(0, -INF, -INF, -INF, -INF, -INF, -INF, -INF);
    for (kIdx=0; kIdx<K; ++kIdx)
    {
        _mm_store_si128((__m128i *)pAGTmp, temp0); pAGTmp++;
        system = _mm_set1_epi16((int16_t)(*pSysTmp)); pSysTmp++;
        parity = _mm_set1_epi16((int16_t)(*pSysTmp)); pSysTmp++;
        extrinsic = _mm_set1_epi16((int16_t)(*pLLRinTmp)); pLLRinTmp++;
        Calculate_Gamma(system, parity, extrinsic, gamma0, gamma1);
        _mm_store_si128((__m128i *)pAGTmp, gamma0); pAGTmp++;
        _mm_store_si128((__m128i *)pAGTmp, gamma1); pAGTmp++;
        gamma0 = _mm_shuffle_epi8(gamma0, LastState0);
        gamma1 = _mm_shuffle_epi8(gamma1, LastState1);
        alpha0 = _mm_shuffle_epi8(temp0, LastState0);
        alpha1 = _mm_shuffle_epi8(temp0, LastState1);
        alpha0 = _mm_adds_epi16(alpha0, gamma0);
        alpha1 = _mm_adds_epi16(alpha1, gamma1);
        temp0 = _mm_max_epi16(alpha0, alpha1);
    }
    for (kIdx=0; kIdx<TAIL_BITS-1; ++kIdx)
    {
        _mm_store_si128((__m128i *)pTailTmp, temp0); pTailTmp++;
        system = _mm_set1_epi16((int16_t)(*tSysTmp)); tSysTmp++;
        parity = _mm_set1_epi16((int16_t)(*tSysTmp)); tSysTmp++;
        extrinsic = _mm_set1_epi16((int16_t)(*tinTmp)); tinTmp++;
        Calculate_Gamma(system, parity, extrinsic, gamma0, gamma1);
        _mm_store_si128((__m128i *)pTailTmp, gamma0); pTailTmp++;
        _mm_store_si128((__m128i *)pTailTmp, gamma1); pTailTmp++;
        gamma0 = _mm_shuffle_epi8(gamma0, LastState0);
        gamma1 = _mm_shuffle_epi8(gamma1, LastState1);
        alpha0 = _mm_shuffle_epi8(temp0, LastState0);
        alpha1 = _mm_shuffle_epi8(temp0, LastState1);
        alpha0 = _mm_adds_epi16(alpha0, gamma0);
        alpha1 = _mm_adds_epi16(alpha1, gamma1);
        temp0 = _mm_max_epi16(alpha0, alpha1);
    }
    _mm_store_si128((__m128i *)pTailTmp, temp0); pTailTmp++;
    system = _mm_set1_epi16((int16_t)(*tSysTmp)); tSysTmp++;
    parity = _mm_set1_epi16((int16_t)(*tSysTmp));
    extrinsic = _mm_set1_epi16((int16_t)(*tinTmp));
    Calculate_Gamma(system, parity, extrinsic, gamma0, gamma1);
    _mm_store_si128((__m128i *)pTailTmp, gamma0); pTailTmp++;
    _mm_store_si128((__m128i *)pTailTmp, gamma1);

    //Calculate Beta and LLR
    kIdx = TAIL_BITS-1;
    pTailTmp = (__m128i *)pTailAG + 3*TAIL_BITS - 1;
    temp0 = _mm_setr_epi16(0, -INF, -INF, -INF, -INF, -INF, -INF, -INF);
    Calculate_LLR((__m128i *)pTailTmp, temp0, temp1, llr0, llr8);
    Calculate_LLR((__m128i *)pTailTmp, temp1, temp0, llr1, llr9);
    Calculate_LLR((__m128i *)pTailTmp, temp0, temp1, llr2, llr10);
    _MM_TRANSEPOSE8_EPI16(llr10, llr2, llr9, llr1, llr8, llr0, zerotemp0, zerotemp1);
    _MM_MAX8_EPI16(llr0, llr8, llr1, llr9, llr2, llr10, zerotemp0, zerotemp1, maxOut0);
    llrtemp0 = _mm_hsubs_epi16(maxOut0, Zero);
    //llr - system - extrinsic(next)
    tailLLRout[kIdx-2] = (_mm_extract_epi16(llrtemp0, 0) - tailBits[2*(kIdx - 2)] - tailLLRin[kIdx-2]) * 0.75;
    tailLLRout[kIdx-1] = (_mm_extract_epi16(llrtemp0, 1) - tailBits[2*(kIdx - 1)] - tailLLRin[kIdx-1]) * 0.75;
    tailLLRout[kIdx] = (_mm_extract_epi16(llrtemp0, 2) - tailBits[2*kIdx] - tailLLRin[kIdx]) * 0.75;

    //kIdx = K-1;
    temp0 = temp1;
    pSysTmp = pSys + Lw*16 - 16;
    pLLRinTmp = pLLRin + Lw*8 - 8;
    int16_t *pLLRoutTmp = pLLRout + Lw*8 - 8;
    pAGTmp = (__m128i *)pAG + 3*K - 1;

    for (kIdx=K-1; kIdx>0; kIdx-=8)
    {
        Calculate_LLR((__m128i *)pAGTmp, temp0, temp1, llr0, llr8);
        Calculate_LLR((__m128i *)pAGTmp, temp1, temp0, llr1, llr9);
        Calculate_LLR((__m128i *)pAGTmp, temp0, temp1, llr2, llr10);
        Calculate_LLR((__m128i *)pAGTmp, temp1, temp0, llr3, llr11);
        Calculate_LLR((__m128i *)pAGTmp, temp0, temp1, llr4, llr12);
        Calculate_LLR((__m128i *)pAGTmp, temp1, temp0, llr5, llr13);
        Calculate_LLR((__m128i *)pAGTmp, temp0, temp1, llr6, llr14);
        Calculate_LLR((__m128i *)pAGTmp, temp1, temp0, llr7, llr15);
        _MM_TRANSEPOSE8_EPI16(llr7, llr6, llr5, llr4, llr3, llr2, llr1, llr0);
        _MM_MAX8_EPI16(llr0, llr1, llr2, llr3, llr4, llr5, llr6, llr7, maxOut0);
        _MM_TRANSEPOSE8_EPI16(llr15, llr14, llr13, llr12, llr11, llr10, llr9, llr8);
        _MM_MAX8_EPI16(llr8, llr9, llr10, llr11, llr12, llr13, llr14, llr15, maxOut1);
        llrtemp0 = _mm_subs_epi16(maxOut1, maxOut0);
        system = _mm_load_si128((__m128i *)pSysTmp); pSysTmp -= 16;
        system = _mm_cvtepi8_epi16(_mm_shuffle_epi8(system, Shuffle));
        extrinsic = _mm_load_si128((__m128i *)pLLRinTmp); pLLRinTmp -= 8;
        llrtemp = _mm_subs_epi16(_mm_subs_epi16(llrtemp0, system), extrinsic);
        llrtemp = _mm_subs_epi16(llrtemp, _mm_srai_epi16(llrtemp, 2));
        _mm_store_si128((__m128i *)pLLRoutTmp, llrtemp); pLLRoutTmp -= 8;
    }
}

struct init_turbo_decoder_8windows_sse
{
    init_turbo_decoder_8windows_sse()
    {

        bblib_print_turbo_version();

    }
};

init_turbo_decoder_8windows_sse do_constructor_turbo_decode_8_sse;

int32_t
bblib_lte_turbo_decoder_8windows_sse(const struct bblib_turbo_decoder_request *request,
    struct bblib_turbo_decoder_response *response)
{
    int32_t retVal = -1;
    int32_t NumIter = 0;
    int32_t C = request->c;
    int32_t K = request->k;
    if ((K&0xF)==0)
    {
        printf("turbo_decoder_8windows_sse: Not 8 windows Turbo Code.\n");
        return -1;
    }
    int32_t Lw = K >> 3;
    int32_t numMaxIter = request->max_iter_num;
    int8_t *pLLRTail = request->input;
    int8_t *pSys = request->input + 48;
    int8_t *pSysInter = request->input + Lw*16*3 + 48;
    int16_t *pLLRin = (int16_t *)(request->input + Lw*16 + 48);
    int16_t *pLLRout = (int16_t *)(request->input + Lw*16*2 + 48);
    int16_t *tailLLRin = (int16_t *)(request->input + 16);
    int16_t *tailLLRout = (int16_t *)(request->input + 32);
    int16_t *pAG = (int16_t *)response->ag_buf;
    uint8_t *out_bit = response->output;

    int32_t i, j, kIdx, pos;
    __align(64) int16_t TailAG[72];
    uint16_t interleaverTable[504];
    int16_t *pTailAG = (int16_t *)TailAG;
    int8_t tailBits[12];

    tailBits[0] = *(pLLRTail + 0); tailBits[1] = *(pLLRTail + 4);
    tailBits[2] = *(pLLRTail + 8); tailBits[3] = *(pLLRTail + 1);
    tailBits[4] = *(pLLRTail + 5); tailBits[5] = *(pLLRTail + 9);
    tailBits[6] = *(pLLRTail + 2); tailBits[7] = *(pLLRTail + 6);
    tailBits[8] = *(pLLRTail + 10); tailBits[9] = *(pLLRTail + 3);
    tailBits[10] = *(pLLRTail + 7); tailBits[11] = *(pLLRTail + 11);
    int8_t *tailBits0 = tailBits + 0;
    int8_t *tailBits1 = tailBits + 6;
    /* Zeroing first extrinsic information*/
    _mm_store_si128((__m128i *)tailLLRin, Zero);
    for (i=0; i<Lw; ++i)
    {
        _mm_store_si128((__m128i *)pLLRin, Zero);
        pLLRin += 8;
    }
    pLLRin = (int16_t *)(request->input + Lw*16 + 48);

    /* SISO1*/
    siso1_decoder(K, pSys, pLLRin, pLLRout, pAG, tailBits0, tailLLRin, tailLLRout, pTailAG, (uint8_t *)out_bit);

    struct bblib_crc_request crc_request;
    crc_request.data = out_bit;
    crc_request.len = ((K >> 3) - 3)*8;
    
    struct bblib_crc_response crc_response;
    crc_response.data = crc_request.data;
    

    NumIter++;
    if (C>1)
    {
        bblib_lte_crc24b_check_sse(&crc_request, &crc_response); 
    }
    else
    {
        bblib_lte_crc24a_check_sse(&crc_request, &crc_response);
    }
    if (crc_response.check_passed)
    {
        if (request->early_term_disable)
            retVal = NumIter;
        else
            return NumIter;
    }

    /*Interleaver y */
    create_interleaver(K, (uint16_t *)interleaverTable);
    for (kIdx=0; kIdx<K; ++kIdx)
    {
        pos = interleaverTable[kIdx];
        *(pSysInter + 2*kIdx) = *(pSys + 2*pos);
    }
    for (j=0; j<numMaxIter; ++j)
    {
        /* Interleaver */
        for (kIdx=0; kIdx<K; ++kIdx)
        {
            pos = interleaverTable[kIdx];
            *(pLLRin + kIdx) = *(pLLRout + pos);
        }

        /* SISO2 */
        siso2_decoder(K, pSysInter, pLLRin, pLLRout, pAG, tailBits1, tailLLRin, tailLLRout, pTailAG);
        NumIter++;
        /* De-Interleaver */
        for (kIdx=0; kIdx<K; ++kIdx)
        {
            pos = interleaverTable[kIdx];
            *(pLLRin + pos) = *(pLLRout + kIdx);
        }

        /* SISO1 */
        siso1_decoder(K, pSys, pLLRin, pLLRout, pAG, tailBits0, tailLLRin, tailLLRout, pTailAG, (uint8_t *)out_bit);

        struct bblib_crc_request crc_request;
        crc_request.data = out_bit;
        crc_request.len = ((K >> 3) - 3)*8;
        
        struct bblib_crc_response crc_response;
        crc_response.data = crc_request.data;
        

        NumIter++;
        if (C>1)
        {
            bblib_lte_crc24b_check_sse(&crc_request, &crc_response);
        }
        else
        {
            bblib_lte_crc24a_check_sse(&crc_request, &crc_response);
        }
        if (crc_response.check_passed)
        {
            if (request->early_term_disable)
                retVal = NumIter;
            else
                return NumIter;
        }
    }

    return retVal;
}
#else
int32_t
bblib_lte_turbo_decoder_8windows_sse(const struct bblib_turbo_decoder_request *request,
    struct bblib_turbo_decoder_response *response)
{
    printf("bblib_turbo requires at least SSE4.2 ISA support to run\n");
    exit(-1);
}
#endif
