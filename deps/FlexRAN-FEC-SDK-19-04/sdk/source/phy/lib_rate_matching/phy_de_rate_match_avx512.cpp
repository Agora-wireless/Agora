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
 * @file   phy_rate_dematching_lte_avx512.cpp
 * @brief  Implementation of Derate matching wint AVX512 instrinsics, including HARQ combine,
 * sub-block deinterleaver, and adapter to Turbo decoder.
*/

/*******************************************************************************
 * Include public/global header files
 *******************************************************************************/

#include <cstdlib>
#include <cmath>
#include "immintrin.h"

#include "phy_rate_match.h"
#include "phy_rate_match_internal.h"
#if defined (_BBLIB_AVX512_)

/* With AVX512 there are better performance results when the fillNull arrays are
   defined outside the function as static const variables. */
static const int8_t fillNullInf[4][32] = {
                        {-1,0,0,0,0,0,0,0,-1,0,0,0,0,0,0,0,-1,0,0,0,0,0,0,0,-1,0,0,0,0,0,0,0},
                        {-1,0,-1,0,-1,0,0,0,-1,0,-1,0,-1,0,0,0,-1,0,-1,0,-1,0,0,0,-1,0,-1,0,-1,0,0,0,},
                        {-1,-1,-1,0,-1,0,-1,0,-1,-1,-1,0,-1,0,-1,0,-1,-1,-1,0,-1,0,-1,0,-1,-1,-1,0,-1,0,-1,0},
                        {-1,-1,-1,-1,-1,-1,-1,0,-1,-1,-1,-1,-1,-1,-1,0,-1,-1,-1,-1,-1,-1,-1,0,-1,-1,-1,-1,-1,-1,-1,0}};
static const int8_t fillNullPar1[4][32] = {
                            {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0}};
static const int32_t fillNullNum[4] = {4,12,20,28};

__m512i idx01= _mm512_set_epi64(0xD, 0xC, 0x5, 0x4, 0x9, 0x8, 0x1, 0x0);
__m512i idx23= _mm512_set_epi64(0xF, 0xE, 0x7, 0x6, 0xB, 0xA, 0x3, 0x2);

__m512i idx0= _mm512_set_epi64(0xB, 0x3, 0xA, 0x2, 0x9, 0x1, 0x8, 0x0);
__m512i idx1= _mm512_set_epi64(0xF, 0x7, 0xE, 0x6, 0xD, 0x5, 0xC, 0x4);

__m512i idx2ForFlag0 =_mm512_set_epi64(0xB, 0xA, 0x3, 0x2, 0x9, 0x8, 0x1, 0x0);
__m512i idx3ForFlag0 =_mm512_set_epi64(0xF, 0xE, 0x7, 0x6, 0xD, 0xC, 0x5, 0x4);
__m512i idx2ForFlag1 =_mm512_set_epi64(0xD, 0xC, 0x5, 0x4, 0x9, 0x8, 0x1, 0x0);
__m512i idx3ForFlag1 =_mm512_set_epi64(0xF, 0xE, 0x7, 0x6, 0xB, 0xA, 0x3, 0x2);

__m512i VMAX_512 = _mm512_set1_epi8(16);
__m512i VMIN_512 = _mm512_set1_epi8(-16);

__m128i VMAX_128 = _mm_set1_epi8(16);
__m128i VMIN_128 = _mm_set1_epi8(-16);

#define UNPACKEPI8_AVX512(in0,in1,out0,out1)\
{\
    out0 = _mm512_unpacklo_epi8 (in0, in1);\
    out1 = _mm512_unpackhi_epi8 (in0, in1);\
}
#define UNPACKEPI16_AVX512(in0,in1,in2,in3,out0,out1,out2,out3)\
{\
    out0 = _mm512_unpacklo_epi16 (in0, in1);\
    out1 = _mm512_unpackhi_epi16 (in0, in1);\
    out2 = _mm512_unpacklo_epi16 (in2, in3);\
    out3 = _mm512_unpackhi_epi16 (in2, in3);\
}
#define UNPACKEPI32_AVX512(in0,in1,in2,in3,out0,out1,out2,out3)\
{\
    out0 = _mm512_unpacklo_epi32 (in0, in1);\
    out1 = _mm512_unpackhi_epi32 (in0, in1);\
    out2 = _mm512_unpacklo_epi32 (in2, in3);\
    out3 = _mm512_unpackhi_epi32 (in2, in3);\
}
#define UNPACKEPI64_AVX512(in0,in1,in2,in3,out0,out1,out2,out3)\
{\
    out0 = _mm512_unpacklo_epi64 (in0, in1);\
    out1 = _mm512_unpackhi_epi64 (in0, in1);\
    out2 = _mm512_unpacklo_epi64 (in2, in3);\
    out3 = _mm512_unpackhi_epi64 (in2, in3);\
}

struct bblib_rate_match_init_avx512
{
    bblib_rate_match_init_avx512()
    {

#if !defined(_BBLIB_AVX2_) && !defined(_BBLIB_AVX512_)
        printf("__func__ rate_match cannot run with this CPU type, needs AVX2 or greater.\n");
        exit(-1);
#endif
        bblib_print_rate_match_version();

    }
};

bblib_rate_match_init_avx512 do_constructor_de_rate_matching_avx512;

static void addByteWithNum_avx512(uint8_t *pIn, uint8_t *pOut, int32_t Len)
{
    int8_t value = 0;
    for(int32_t i=0; i<Len; i++)
    {
        value = *pOut + *pIn;
        if(value>MAX_VALUE)
            value=MAX_VALUE;
        if(value<MIN_VALUE)
            value=MIN_VALUE;

        *pOut = value;
        pIn++;
        pOut++;
    }
}


void wrapAdd2HarqBuffer_avx512(uint8_t *pIn, uint8_t *pOut, int32_t len)
{
    int32_t i;
    int32_t byteStep_64 = len&0xffffffc0;
    int32_t temp = len&0x3f;
    int32_t byteStep_16 = temp &0x30;
    int32_t byteStep_1 = temp &0xf;

    __m128i ymm128_0, ymm128_1;
    __m512i ymm512_0, ymm512_1;

    for(i=0; i<byteStep_64; i+=64)
    {
        ymm512_0 = _mm512_loadu_si512 ((__m256i const *) pIn);
        ymm512_1 = _mm512_loadu_si512 ((__m256i const *) pOut);
        ymm512_0 = _mm512_adds_epi8 (ymm512_0, ymm512_1);
        ymm512_1 = _mm512_max_epi8(VMIN_512, ymm512_0);
        ymm512_0 = _mm512_min_epi8(VMAX_512, ymm512_1);
        pIn = pIn + 64;
        _mm512_storeu_si512 ((__m512i *) pOut, ymm512_0);
        pOut = pOut + 64;
    }

    for(i=0; i<byteStep_16; i+=16)
    {
        ymm128_0 = _mm_loadu_si128 ((__m128i const *) pIn);
        ymm128_1 = _mm_loadu_si128 ((__m128i const *) pOut);
        ymm128_0 = _mm_adds_epi8 (ymm128_0, ymm128_1);
        ymm128_1 = _mm_max_epi8(VMIN_128, ymm128_0);
        ymm128_0 = _mm_min_epi8(VMAX_128, ymm128_1);
        pIn = pIn + 16;
        _mm_storeu_si128 ((__m128i *) pOut, ymm128_0);
        pOut = pOut + 16;
    }

    addByteWithNum_avx512(pIn, pOut, byteStep_1);
}


template <circular_buffer_format CIRCULAR_BUFFER_FORMAT>
void setColumnInf(uint8_t *pHarq, uint8_t* (&pColInf)[32], int32_t Rsubblock, int32_t nNull, int32_t flag);

template<>
inline void setColumnInf<BBLIB_CIRCULAR_BUFFER_WITHOUT_PADDING>(uint8_t *pHarq, uint8_t* (&pColInf)[32],
                                                                int32_t Rsubblock, int32_t nNull, int32_t flag)
{
    int32_t fillNullInfType = 0;

    uint8_t *pHarqTmp = pHarq;

    /* find the NULL fill type */
#pragma unroll (4)
    for(int32_t i = 0; i < 4; i++)
    {
        if(nNull == fillNullNum[i])
        {
            fillNullInfType = i;
            break;
        }
    }

#pragma unroll (32)
    for(int32_t i = 0; i < 32; i++)
    {
        pHarqTmp = pHarqTmp + fillNullInf[fillNullInfType][i] +
                              fillNullInf[fillNullInfType][i] * flag +
                              fillNullPar1[fillNullInfType][i] * flag;
        pColInf[i] = pHarqTmp;
        pHarqTmp = pHarqTmp + Rsubblock;
    }
}

template<>
inline void setColumnInf<BBLIB_FULL_CIRCULAR_BUFFER>(uint8_t *pHarq, uint8_t* (&pColInf)[32],
                                                     int32_t Rsubblock, int32_t nNull, int32_t flag)
{
    uint8_t *pHarqTmp = pHarq;

#pragma unroll (32)
    for(int32_t i = 0; i < 32; i++)
    {
        pColInf[i] = pHarqTmp;
        pHarqTmp = pHarqTmp + Rsubblock;
    }
}

template <circular_buffer_format CIRCULAR_BUFFER_FORMAT>
inline void harqDeInteleaveBlock_avx512(uint8_t *pHarq, uint8_t *pDeInteleave0,
    uint8_t *pDeInteleave1, const int32_t Rsubblock, const int32_t nNull,
    const int32_t flag)
{
    int32_t i, j, k = 0;
    uint8_t *pColInf[32] {};
    uint8_t *pDeInteleaveTmp0,*pDeInteleaveTmp1;

    __m512i ymm[32];
    __m512i ymmTemp[32];
    __m512i ymm0,ymm1,ymm2,ymm3;

    pDeInteleaveTmp0 = pDeInteleave0;
    pDeInteleaveTmp1 = pDeInteleave1;

    /* if first block, nSubBlock equals Rsubblock, otherwise nSubBlock equals 2*Rsubblock */
    int32_t nRow = Rsubblock + Rsubblock * flag;

    /* get the info bit addr for each col */
    setColumnInf<CIRCULAR_BUFFER_FORMAT>(pHarq, pColInf, nRow, nNull, flag);

    for(i = 0; i < nRow; i += 64)
    {
#pragma unroll(32)
        for(j = 0; j < 32; j++)
        {
            ymm[j] = _mm512_loadu_si512 ((__m256i const *)pColInf[j]);
            pColInf[j] = pColInf[j] + 64;
        }

        /* ymm0 - 0:1 s0-s7 s16-s23 s32-s39 s48-s55 */
        /* ymm1 - 0:1 s8-s15 s24-s31 s40-s47 s56-s63 */
        UNPACKEPI8_AVX512(ymm[0], ymm[16],ymm0,ymm1);
        /* 2:3 s0-s7 s16-s23 s32-s39 s48-s55; s8-s15 s24-s31 s40-s47 s56-s63*/
        UNPACKEPI8_AVX512(ymm[8], ymm[24],ymm[0],ymm[16]);
        /* 4:5 s0-s7 s16-s23 s32-s39 s48-s55; s8-s15 s24-s31 s40-s47 s56-s63*/
        UNPACKEPI8_AVX512(ymm[4], ymm[20],ymm[8],ymm[24]);
        /* 6:7 s0-s7 s16-s23 s32-s39 s48-s55; s8-s15 s24-s31 s40-s47 s56-s63*/
        UNPACKEPI8_AVX512(ymm[12], ymm[28],ymm[4],ymm[20]);
        /* 8:9 s0-s7 s16-s23 s32-s39 s48-s55; s8-s15 s24-s31 s40-s47 s56-s63*/
        UNPACKEPI8_AVX512(ymm[2], ymm[18],ymm[12],ymm[28]);
        /* 10:11 s0-s7 s16-s23 s32-s39 s48-s55; s8-s15 s24-s31 s40-s47 s56-s63*/
        UNPACKEPI8_AVX512(ymm[10], ymm[26],ymm[2],ymm[18]);
        /* 12:13 s0-s7 s16-s23 s32-s39 s48-s55; s8-s15 s24-s31 s40-s47 s56-s63*/
        UNPACKEPI8_AVX512(ymm[6], ymm[22],ymm[10],ymm[26]);
        /* 14:15 s0-s7 s16-s23 s32-s39 s48-s55; s8-s15 s24-s31 s40-s47 s56-s63*/
        UNPACKEPI8_AVX512(ymm[14], ymm[30],ymm[6],ymm[22]);
        /* 16:17 s0-s7 s16-s23 s32-s39 s48-s55; s8-s15 s24-s31 s40-s47 s56-s63*/
        UNPACKEPI8_AVX512(ymm[1], ymm[17],ymm[14],ymm[30]);
        /* 18:19 s0-s7 s16-s23 s32-s39 s48-s55; s8-s15 s24-s31 s40-s47 s56-s63*/
        UNPACKEPI8_AVX512(ymm[9], ymm[25],ymm[1],ymm[17]);
        /* 20:21 s0-s7 s16-s23 s32-s39 s48-s55; s8-s15 s24-s31 s40-s47 s56-s63*/
        UNPACKEPI8_AVX512(ymm[5], ymm[21],ymm[9],ymm[25]);
        /* 22:23 s0-s7 s16-s23 s32-s39 s48-s55; s8-s15 s24-s31 s40-s47 s56-s63*/
        UNPACKEPI8_AVX512(ymm[13], ymm[29],ymm[5],ymm[21]);
        /* 24:25 s0-s7 s16-s23 s32-s39 s48-s55; s8-s15 s24-s31 s40-s47 s56-s63*/
        UNPACKEPI8_AVX512(ymm[3], ymm[19],ymm[13],ymm[29]);
        /* 26:27 s0-s7 s16-s23 s32-s39 s48-s55; s8-s15 s24-s31 s40-s47 s56-s63*/
        UNPACKEPI8_AVX512(ymm[11], ymm[27],ymm[3],ymm[19]);
        /* 28:29 s0-s7 s16-s23 s32-s39 s48-s55; s8-s15 s24-s31 s40-s47 s56-s63*/
        UNPACKEPI8_AVX512(ymm[7], ymm[23],ymm[11],ymm[27]);
        /* 30:31 s0-s7 s16-s23 s32-s39 s48-s55; s8-s15 s24-s31 s40-s47 s56-s63*/
        UNPACKEPI8_AVX512(ymm[15], ymm[31],ymm[7],ymm[23]);


        /* ymm[15] - 0:3 s0-s3 s16-s19 s32-s35 s48-s51 */
        /* ymm[31] - 0:3 s4-s7 s20-s23 s36-s39 s52-s55 */
        /* ymm2 -     0:3 s8-s11 s24-s27 s40-s43 s56-s59 */
        /* ymm3 -     0:3 s12-s15 s28-s31 s44-s47 s60-s63 */
        UNPACKEPI16_AVX512(ymm0,    ymm[0],ymm1,  ymm[16], ymm[15],ymm[31],ymm2,   ymm3);

        /* ymm0 -     4:7 s0-s3 s16-s19 s32-s35 s48-s51 */
        /* ymm[0] -   4:7 s4-s7 s20-s23 s36-s39 s52-s55 */
        /* ymm1 -     4:7 s8-s11 s24-s27 s40-s43 s56-s59 */
        /* ymm[16] - 4:7 s12-s15 s28-s31 s44-s47 s60-s63 */
        UNPACKEPI16_AVX512(ymm[8],  ymm[4],ymm[24],ymm[20],ymm0,   ymm[0], ymm1,   ymm[16]);
        /* 8:11  s0-s3s16-s19 s4-s7s20-s23 s8-s11s24-s27 s12-s15s28-s31 ... */
        UNPACKEPI16_AVX512(ymm[12], ymm[2],ymm[28],ymm[18],ymm[8], ymm[4], ymm[24],ymm[20]);
        /* 12:15 s0-s3s16-s19 s4-s7s20-s23 s8-s11s24-s27 s12-s15s28-s31 ... */
        UNPACKEPI16_AVX512(ymm[10], ymm[6],ymm[26],ymm[22],ymm[12], ymm[2],ymm[28],ymm[18]);
        /* 16:19 s0-s3s16-s19 s4-s7s20-s23 s8-s11s24-s27 s12-s15s28-s31 ... */
        UNPACKEPI16_AVX512(ymm[14], ymm[1],ymm[30],ymm[17],ymm[10], ymm[6],ymm[26],ymm[22]);
        /* 20:23 s0-s3s16-s19 s4-s7s20-s23 s8-s11s24-s27 s12-s15s28-s31 ... */
        UNPACKEPI16_AVX512(ymm[9],  ymm[5],ymm[25],ymm[21],ymm[14], ymm[1],ymm[30],ymm[17]);
        /* 24:27 s0-s3s16-s19 s4-s7s20-s23 s8-s11s24-s27 s12-s15s28-s31 ... */
        UNPACKEPI16_AVX512(ymm[13], ymm[3],ymm[29],ymm[19],ymm[9],  ymm[5],ymm[25],ymm[21]);
        /* 28-31 s0-s3s16-s19 s4-s7s20-s23 s8-s11s24-s27 s12-s15s28-s31 ... */
        UNPACKEPI16_AVX512(ymm[11], ymm[7],ymm[27],ymm[23],ymm[13], ymm[3],ymm[29],ymm[19]);

        /* ymm[11] - 0:7 s0s1 s16s17 s32s33 s48s49 */
        /* ymm[7] -   0:7 s2s3 s18s19 s34s35 s50s51 */
        /* ymm[27] - 0:7 s4s5 s20s21 s36s37 s52s53 */
        /* ymm[23] - 0:7 s6s7 s22s23 s38s39 s54s55 */
        UNPACKEPI32_AVX512(ymm[15], ymm0,    ymm[31], ymm[0],  ymm[11], ymm[7],ymm[27], ymm[23]);

        /* ymm[15] - 0:7 s8s9 s24s25 s40s41 s56s57 */
        /* ymm0 -   0:7 s10s11 s26s27 s42s43 s58s59 */
        /* ymm[31] - 0:7 s12s13 s28s29 s44s45 s60s61 */
        /* ymm[0] -   0:7 s14s15 s30s31 s46s47 s62s63 */
        UNPACKEPI32_AVX512(ymm2,    ymm1,    ymm3,    ymm[16], ymm[15], ymm0,  ymm[31], ymm[0]);

        /* ymm2 -   8:15 s0s1 s16s17 s32s33 s48s49 */
        /* ymm1 -   8:15 s2s3 s18s19 s34s35 s50s51 */
        /* ymm3 -   8:15 s4s5 s20s21 s36s37 s52s53 */
        /* ymm[16] - 8:15 s6s7 s22s23 s38s39 s54s55 */
        UNPACKEPI32_AVX512(ymm[8],  ymm[12], ymm[4], ymm[2],   ymm2,    ymm1,  ymm3,    ymm[16]);
        /* ymm[8] - 8:15 s8s9 s24s25 s40s41 s56s57 */
        /* ymm[12] -8:15 s10s11 s26s27 s42s43 s58s59 */
        /* ymm[4] - 8:15 s12s13 s28s29 s44s45 s60s61 */
        /* ymm[2] - 8:15 s14s15 s30s31 s46s47 s62s63 */
        UNPACKEPI32_AVX512(ymm[24], ymm[28], ymm[20], ymm[18], ymm[8],  ymm[12], ymm[4], ymm[2]);

        /* 16:23   s0s1s16s17 s2s3s18s19 s4s5s20s21 s6s7s22s23 */
        UNPACKEPI32_AVX512(ymm[10], ymm[14], ymm[6], ymm[1],   ymm[24], ymm[28], ymm[20], ymm[18]);
        /* s8s9s24s25 s10s11s26s27 s12s13s28s29 s14s15s30s31 */
        UNPACKEPI32_AVX512(ymm[26], ymm[30], ymm[22], ymm[17], ymm[10], ymm[14], ymm[6], ymm[1]);
        /* 24:31   s0s1s16s17 s2s3s18s19 s4s5s20s21 s6s7s22s23 */
        UNPACKEPI32_AVX512(ymm[9], ymm[13], ymm[5], ymm[3],    ymm[26], ymm[30], ymm[22], ymm[17]);
        /* s8s9s24s25 s10s11s26s27 s12s13s28s29 s14s15s30s31 */
        UNPACKEPI32_AVX512(ymm[25], ymm[29], ymm[21], ymm[19], ymm[9], ymm[13], ymm[5], ymm[3]);

/*
        __m512i _mm512_maskz_permutex2var_epi64 (__mmask8 k, __m512i a, __m512i idx, __m512i b)
#include "immintrin.h"
        Instruction: vpermi2q zmm {k}, zmm, zmm
                     vpermt2q zmm {k}, zmm, zmm
        CPUID Flags: AVX512F
        Description
        Shuffle 64-bit integers in a and b across lanes using the corresponding selector and index in idx, and store the results in dst using zeromask k (elements are zeroed out when the corresponding mask bit is not set).
        Operation
        FOR j := 0 to 7
            i := j*64
            off := idx[i+2:i]*64
            IF k[j]
                dst[i+63:i] := (idx[i+3]) ? b[off+63:off] : a[off+63:off]
            ELSE
                dst[i+63:i] := 0
            FI
        ENDFOR
        dst[MAX:512] := 0


__m512i _mm512_permutex2var_epi64 (__m512i a, __m512i idx, __m512i b)
#include "immintrin.h"
Instruction: vpermi2q zmm {k}, zmm, zmm
             vpermt2q zmm {k}, zmm, zmm
CPUID Flags: AVX512F
Description
Shuffle 64-bit integers in a and b across lanes using the corresponding selector and index in idx, and store the results in dst.
Operation
FOR j := 0 to 7
	i := j*64
	off := idx[i+2:i]*64
	dst[i+63:i] := idx[i+3] ? b[off+63:off] : a[off+63:off]
ENDFOR
dst[MAX:512] := 0



*/

        //ymmTemp[0] - 0:7 s0s1 s2s3 s32s33 s34s35
        ymmTemp[0] = _mm512_permutex2var_epi64(ymm[11], idx01, ymm[7]);
        //ymmTemp[4] - 0:7 s16s17 s18s19 s48s49 s50s51
        ymmTemp[4] = _mm512_permutex2var_epi64(ymm[11], idx23, ymm[7]);
        //ymmTemp[1] - 0:7 s4s5 s6s7 s36s37 s38s39
        ymmTemp[1] = _mm512_permutex2var_epi64(ymm[27], idx01, ymm[23]);
        //ymmTemp[5] - 0:7 s20s21 s22s23 s52s53 s54s55
        ymmTemp[5] = _mm512_permutex2var_epi64(ymm[27], idx23, ymm[23]);
        //ymmTemp[2] - 0:7 s8s9 s10s11 s40s41 s42s43
        ymmTemp[2] = _mm512_permutex2var_epi64(ymm[15], idx01, ymm0);
        //ymmTemp[6] - 0:7 s24s25 s26s27 s56s57 s58s59
        ymmTemp[6] = _mm512_permutex2var_epi64(ymm[15], idx23, ymm0);
        //ymmTemp[3] - 0:7 s12s13 s14s15  s44s45 s46s47
        ymmTemp[3] = _mm512_permutex2var_epi64(ymm[31], idx01, ymm[0]);
        //ymmTemp[7] - 0:7 s28s29 s30s31 s60s61 s62s63
        ymmTemp[7] = _mm512_permutex2var_epi64(ymm[31], idx23, ymm[0]);

        //ymmTemp[8] -  8:15 s0s1 s2s3 s32s33 s34s35
        ymmTemp[8] = _mm512_permutex2var_epi64(ymm2, idx01, ymm1);
        //ymmTemp[12] - 8:15 s16s17 s18s19 s48s49 s50s51
        ymmTemp[12] =_mm512_permutex2var_epi64( ymm2, idx23, ymm1);
        //ymmTemp[9] -  8:15 s4s5 s6s7  s36s37 s38s39
        ymmTemp[9] = _mm512_permutex2var_epi64( ymm3, idx01, ymm[16]);
        //ymmTemp[13] - 8:15 s20s21 s22s23 s52s53 s54s55
        ymmTemp[13] =_mm512_permutex2var_epi64( ymm3, idx23, ymm[16]);
        //ymmTemp[10] - 8:15 s8s9 s10s11 s40s41 s42s43
        ymmTemp[10] = _mm512_permutex2var_epi64( ymm[8], idx01, ymm[12]);
        //ymmTemp[14] - 8:15 s24s25 s26s27 s56s57 s58s59
        ymmTemp[14] = _mm512_permutex2var_epi64( ymm[8], idx23, ymm[12]);
        //ymmTemp[11] - 8:15 s12s13 s14s15  s44s45 s46s47
        ymmTemp[11] = _mm512_permutex2var_epi64( ymm[4], idx01, ymm[2]);
        //ymmTemp[15] - 8:15 s28s29 s30s31 s60s61 s62s63
        ymmTemp[15] = _mm512_permutex2var_epi64( ymm[4], idx23, ymm[2]);

        //ymmTemp[16] - 16:23 s0s1 s2s3 s32s33 s34s35
        ymmTemp[16] = _mm512_permutex2var_epi64( ymm[24], idx01, ymm[28]);
        //ymmTemp[20] - 16:23 s16s17 s18s19 s48s49 s50s51
        ymmTemp[20] = _mm512_permutex2var_epi64( ymm[24], idx23, ymm[28]);
        //ymmTemp[17] - 16:23 s4s5 s6s7  s36s37 s38s39
        ymmTemp[17] = _mm512_permutex2var_epi64( ymm[20], idx01, ymm[18]);
        //ymmTemp[21] - 16:23 s20s21 s22s23 s52s53 s54s55
        ymmTemp[21] = _mm512_permutex2var_epi64( ymm[20], idx23, ymm[18]);
        //ymmTemp[18] - 16:23 s8s9 s10s11 s40s41 s42s43
        ymmTemp[18] = _mm512_permutex2var_epi64( ymm[10], idx01, ymm[14]);
        //ymmTemp[22] - 16:23 s24s25 s26s27 s56s57 s58s59
        ymmTemp[22] = _mm512_permutex2var_epi64( ymm[10], idx23, ymm[14]);
        //ymmTemp[19] - 16:23 s12s13 s14s15  s44s45 s46s47
        ymmTemp[19] = _mm512_permutex2var_epi64( ymm[6], idx01, ymm[1]);
        //ymmTemp[23] - 16:23 s28s29 s30s31 s60s61 s62s63
        ymmTemp[23] = _mm512_permutex2var_epi64( ymm[6], idx23, ymm[1]);

        //ymmTemp[24] - 24:31 s0s1 s2s3 s32s33 s34s35
        ymmTemp[24] = _mm512_permutex2var_epi64( ymm[26], idx01, ymm[30]);
        //ymmTemp[28] - 24:31 s16s17 s18s19 s48s49 s50s51
        ymmTemp[28] = _mm512_permutex2var_epi64( ymm[26], idx23, ymm[30]);
        //ymmTemp[25] - 24:31 s4s5 s6s7  s36s37 s38s39
        ymmTemp[25] = _mm512_permutex2var_epi64( ymm[22], idx01, ymm[17]);
        //ymmTemp[29] - 24:31 s20s21 s22s23 s52s53 s54s55
        ymmTemp[29] = _mm512_permutex2var_epi64( ymm[22], idx23, ymm[17]);
        //ymmTemp[26] - 24:31 s8s9 s10s11 s40s41 s42s43
        ymmTemp[26] = _mm512_permutex2var_epi64( ymm[9], idx01, ymm[13]);
        //ymmTemp[30] - 24:31 s24s25 s26s27 s56s57 s58s59
        ymmTemp[30] = _mm512_permutex2var_epi64( ymm[9], idx23, ymm[13]);
        //ymmTemp[27] - 24:31 s12s13 s14s15  s44s45 s46s47
        ymmTemp[27] = _mm512_permutex2var_epi64( ymm[5], idx01, ymm[3]);
        //ymmTemp[31] - 24:31 s28s29 s30s31 s60s61 s62s63
        ymmTemp[31] = _mm512_permutex2var_epi64( ymm[5], idx23, ymm[3]);

        //ymmTemp[0] -  0:7 s0s1 s2s3 s32s33 s34s35
        //ymmTemp[8] -  8:15 s0s1 s2s3 s32s33 s34s35
        ymm[0] =_mm512_permutex2var_epi64( ymmTemp[0], idx0, ymmTemp[8]);   // 0:15 s0 s1 s2 s3
        ymm[2] =_mm512_permutex2var_epi64( ymmTemp[0], idx1, ymmTemp[8]);   // 0:15 s32 s33 s34 s35
        //ymmTemp[16] - 16:23 s0s1 s2s3 s32s33 s34s35
        //ymmTemp[24] - 24:31 s0s1 s2s3 s32s33 s34s35
        ymm[1] =_mm512_permutex2var_epi64( ymmTemp[16], idx0, ymmTemp[24]); // 16:31 s0 s1 s2 s3
        ymm[3] =_mm512_permutex2var_epi64( ymmTemp[16], idx1, ymmTemp[24]); // 16:31 s32 s33 s34 s35

        //ymmTemp[1] - 0:7  s4s5 s6s7 s36s37 s38s39
        //ymmTemp[9] - 8:15 s4s5 s6s7  s36s37 s38s39
        ymm[4] =_mm512_permutex2var_epi64( ymmTemp[1], idx0, ymmTemp[9]);   // 0:15 s4 s5 s6 s7
        ymm[6] =_mm512_permutex2var_epi64( ymmTemp[1], idx1, ymmTemp[9]);   // 0:15 s36 s37 s38 s39
        //ymmTemp[17] - 16:23 s4s5 s6s7  s36s37 s38s39
        //ymmTemp[25] - 24:31 s4s5 s6s7  s36s37 s38s39
        ymm[5] =_mm512_permutex2var_epi64( ymmTemp[17], idx0, ymmTemp[25]); // 16:31 s4 s5 s6 s7
        ymm[7] =_mm512_permutex2var_epi64( ymmTemp[17], idx1, ymmTemp[25]); // 16:31 s36 s37 s38 s39

        //ymmTemp[2] - 0:7 s8s9 s10s11 s40s41 s42s43
        //ymmTemp[10] - 8:15 s8s9 s10s11 s40s41 s42s43
        ymm[8] =_mm512_permutex2var_epi64( ymmTemp[2], idx0, ymmTemp[10]);   // 0:15 s8 s9 s10 s11
        ymm[10] =_mm512_permutex2var_epi64( ymmTemp[2], idx1, ymmTemp[10]);   // 0:15 s40 s41 s42 s43
        //ymmTemp[18] - 16:23 s8s9 s10s11 s40s41 s42s43
        //ymmTemp[26] - 24:31 s8s9 s10s11 s40s41 s42s43
        ymm[9] =_mm512_permutex2var_epi64( ymmTemp[18], idx0, ymmTemp[26]); // 16:31 s8 s9 s10 s11
        ymm[11] =_mm512_permutex2var_epi64( ymmTemp[18], idx1, ymmTemp[26]); // 16:31 s40 s41 s42 s43

        //ymmTemp[3] -  0:7 s12s13 s14s15  s44s45 s46s47
        //ymmTemp[11] - 8:15 s12s13 s14s15  s44s45 s46s47
        ymm[12] =_mm512_permutex2var_epi64( ymmTemp[3], idx0, ymmTemp[11]);   // 0:15 s12 s13 s14 s15
        ymm[14] =_mm512_permutex2var_epi64( ymmTemp[3], idx1, ymmTemp[11]);   // 0:15 s44 s45 s46 s47
        //ymmTemp[19] - 16:23 s12s13 s14s15  s44s45 s46s47
        //ymmTemp[27] - 24:31 s12s13 s14s15  s44s45 s46s47
        ymm[13] =_mm512_permutex2var_epi64( ymmTemp[19], idx0, ymmTemp[27]); // 16:31 s12 s13 s14 s15
        ymm[15] =_mm512_permutex2var_epi64( ymmTemp[19], idx1, ymmTemp[27]); // 16:31 s44 s45 s46 s47

        //ymmTemp[4] - 0:7 s16s17 s18s19 s48s49 s50s51
        //ymmTemp[12] - 8:15 s16s17 s18s19 s48s49 s50s51
        ymm[16] =_mm512_permutex2var_epi64( ymmTemp[4], idx0, ymmTemp[12]);   // 0:15 s16 s17 s18 s19
        ymm[18] =_mm512_permutex2var_epi64( ymmTemp[4], idx1, ymmTemp[12]);   // 0:15 s48 s49 s50 s51
        //ymmTemp[20] - 16:23 s16s17 s18s19 s48s49 s50s51
        //ymmTemp[28] - 24:31 s16s17 s18s19 s48s49 s50s51
        ymm[17] =_mm512_permutex2var_epi64( ymmTemp[20], idx0, ymmTemp[28]); // 16:31 s16 s17 s18 s19
        ymm[19] =_mm512_permutex2var_epi64( ymmTemp[20], idx1, ymmTemp[28]); // 16:31 s48 s49 s50 s51

        //ymmTemp[5] - 0:7 s20s21 s22s23 s52s53 s54s55
        //ymmTemp[13] - 8:15 s20s21 s22s23 s52s53 s54s55
        ymm[20] =_mm512_permutex2var_epi64( ymmTemp[5], idx0, ymmTemp[13]);   // 0:15 s20 s21 s22 s23
        ymm[22] =_mm512_permutex2var_epi64( ymmTemp[5], idx1, ymmTemp[13]);   // 0:15 s52 s53 s54 s55
        //ymmTemp[21] - 16:23 s20s21 s22s23 s52s53 s54s55
        //ymmTemp[29] - 24:31 s20s21 s22s23 s52s53 s54s55
        ymm[21] =_mm512_permutex2var_epi64( ymmTemp[21], idx0, ymmTemp[29]); // 16:31 s20 s21 s22 s23
        ymm[23] =_mm512_permutex2var_epi64( ymmTemp[21], idx1, ymmTemp[29]); // 16:31 s52 s53 s54 s55

        //ymmTemp[6] - 0:7 s24s25 s26s27 s56s57 s58s59
        //ymmTemp[14] - 8:15 s24s25 s26s27 s56s57 s58s59
        ymm[24] =_mm512_permutex2var_epi64( ymmTemp[6], idx0, ymmTemp[14]);   // 0:15 s24 s25 s26 s27
        ymm[26] =_mm512_permutex2var_epi64( ymmTemp[6], idx1, ymmTemp[14]);   // 0:15 s56 s57 s58 s59
        //ymmTemp[22] - 16:23 s24s25 s26s27 s56s57 s58s59
        //ymmTemp[30] - 24:31 s24s25 s26s27 s56s57 s58s59
        ymm[25] =_mm512_permutex2var_epi64( ymmTemp[22], idx0, ymmTemp[30]); // 16:31 s24 s25 s26 s27
        ymm[27] =_mm512_permutex2var_epi64( ymmTemp[22], idx1, ymmTemp[30]); // 16:31 s56 s57 s58 s59

        //ymmTemp[7] - 0:7 s28s29 s30s31 s60s61 s62s63
        //ymmTemp[15] - 8:15 s28s29 s30s31 s60s61 s62s63
        ymm[28] =_mm512_permutex2var_epi64( ymmTemp[7], idx0, ymmTemp[15]);   // 0:15 s28 s29 s30 s31
        ymm[30] =_mm512_permutex2var_epi64( ymmTemp[7], idx1, ymmTemp[15]);   // 0:15 s60 s61 s62 s63
        //ymmTemp[23] - 16:23 s28s29 s30s31 s60s61 s62s63
        //ymmTemp[31] - 24:31 s28s29 s30s31 s60s61 s62s63
        ymm[29] =_mm512_permutex2var_epi64( ymmTemp[23], idx0, ymmTemp[31]); // 16:31 s28 s29 s30 s31
        ymm[31] =_mm512_permutex2var_epi64( ymmTemp[23], idx1, ymmTemp[31]); // 16:31 s60 s61 s62 s63


        if(flag == 0)
        {
            //for fetching 0:31 s0 s1 s2 s3 and s32 s33 s34 s35
            for(int32_t loop1 = 0; loop1 < 32 && k < nRow; loop1 += 4,k += 4)
            {
                _mm512_storeu_si512(pDeInteleaveTmp0, _mm512_permutex2var_epi64( ymm[loop1], idx2ForFlag0, ymm[loop1+1]));   // 0:31 s0 s1
                pDeInteleaveTmp0 += 64;

                _mm512_storeu_si512(pDeInteleaveTmp0, _mm512_permutex2var_epi64( ymm[loop1], idx3ForFlag0, ymm[loop1+1]));   // 0:31 s2 s3
                pDeInteleaveTmp0 += 64;

            }

            for(int32_t loop1 = 0; loop1 < 32 && k < nRow; loop1 += 4,k += 4)
            {
                _mm512_storeu_si512(pDeInteleaveTmp0, _mm512_permutex2var_epi64( ymm[loop1+2], idx2ForFlag0, ymm[loop1+3]));   // 0:31 s32 s33
                pDeInteleaveTmp0 += 64;

                _mm512_storeu_si512(pDeInteleaveTmp0, _mm512_permutex2var_epi64( ymm[loop1+2], idx3ForFlag0, ymm[loop1+3]));   // 0:31 s34 s35
                pDeInteleaveTmp0 += 64;
            }
        }
        else
        {
            //for fetching 0:31 s0 s2 s1 s3 and s32 s34 s33 s35
            for(int32_t loop1 = 0; loop1 < 32 && k < nRow; loop1 += 4,k += 4)
            {
                _mm512_storeu_si512(pDeInteleaveTmp0, _mm512_permutex2var_epi64( ymm[loop1], idx2ForFlag1, ymm[loop1+1]));   // 0:31 s0 s2
                pDeInteleaveTmp0 += 64;

                _mm512_storeu_si512(pDeInteleaveTmp1, _mm512_permutex2var_epi64( ymm[loop1], idx3ForFlag1, ymm[loop1+1]));   // 0:31 s1 s3
                pDeInteleaveTmp1 += 64;
            }

            for(int32_t loop1 = 0; loop1 < 32 && k < nRow; loop1 += 4,k += 4)
            {
                _mm512_storeu_si512(pDeInteleaveTmp0, _mm512_permutex2var_epi64( ymm[loop1+2], idx2ForFlag1, ymm[loop1+3]));   // 0:31 s32 s34
                pDeInteleaveTmp0 += 64;

                _mm512_storeu_si512(pDeInteleaveTmp1, _mm512_permutex2var_epi64( ymm[loop1+2], idx3ForFlag1, ymm[loop1+3]));   // 0:31 s33 s35
                pDeInteleaveTmp1 += 64;
            }
        }
    }
}

int32_t bblib_deinterleave_ul_avx512(const struct bblib_deinterleave_ul_request *request,
    struct bblib_deinterleave_ul_response *response)
{
    uint8_t *pHarq = request->pharqbuffer;
    uint8_t *pDeInteleave = response->pinteleavebuffer;
    int32_t ncb = request->ncb;

    int32_t oneThirdNcb = ncb/3;
    int32_t Rsubblock = ceil((float)oneThirdNcb/(float)32);
    int32_t subBlockWithNull = (Rsubblock <<5);
    int32_t nNull = 0;

    switch (request->circ_buffer)
    {
        case BBLIB_CIRCULAR_BUFFER_WITHOUT_PADDING:
            nNull = subBlockWithNull - oneThirdNcb;
            /* deIntealve for block0 */
            harqDeInteleaveBlock_avx512<BBLIB_CIRCULAR_BUFFER_WITHOUT_PADDING>(pHarq,
                pDeInteleave, 0, Rsubblock, nNull, 0);
            /* deIndtealve for block1 block2 */
            harqDeInteleaveBlock_avx512<BBLIB_CIRCULAR_BUFFER_WITHOUT_PADDING>(pHarq+oneThirdNcb,
                pDeInteleave+subBlockWithNull+64, pDeInteleave+(subBlockWithNull+64)*2, Rsubblock, nNull, 1);
            break;
        case BBLIB_FULL_CIRCULAR_BUFFER:
            /* ncb includes dummy bits in the full circular buffer case
               and nNull is not used, no need to calculate it. */
            /* deIntealve for block0 */
            harqDeInteleaveBlock_avx512<BBLIB_FULL_CIRCULAR_BUFFER>(pHarq,
                pDeInteleave, 0, Rsubblock, nNull, 0);
            /* deIndtealve for block1 block2 */
            harqDeInteleaveBlock_avx512<BBLIB_FULL_CIRCULAR_BUFFER>(pHarq+oneThirdNcb,
                pDeInteleave+subBlockWithNull+64, pDeInteleave+(subBlockWithNull+64)*2, Rsubblock, nNull, 1);
    }

    return 0;
}

void turboAdapterK16_avx512(uint8_t *pDeInteleave, uint8_t *pTurbo, const int32_t oneThirdNcb, const int32_t nNull, const int32_t KBlock, int32_t isinverted)
{
    int32_t i, j, k, index=0;
    int32_t KBlockDiv16 = (KBlock>>4);
    int32_t cntOf64Byte = 0;
    int32_t cntOf16Byte = 0;

    uint8_t *paD0[16],*paD1[16],*paD2[16];
    uint8_t *pD0,*pD1,*pD2;
    uint8_t *pOut=pTurbo;

    __m512i yamm0[18],yamm1[18];
    __m512i ymmTemp0[16],ymmTemp1[16];
    __m512i ymm0,ymm1,ymm2,ymm3;
    __m512i yZero512 = _mm512_set1_epi8 (0);
    __m256i yZero256 = _mm256_set1_epi8 (0);
    __m128i yZero128 = _mm_set1_epi8 (0);
    __m128i *pTmp;

    __m512i idx[4][3];
    __mmask8 kFlag[3];

    kFlag[0] = 0x3C;
    kFlag[1] = 0xCF;
    kFlag[2] = 0xF3;

    idx[0][0] = _mm512_set_epi64(0xFF, 0xFF, 0x9, 0x8, 0x1, 0x0, 0xFF, 0xFF);   //xx d0s0 d1s0 xx
    idx[0][1] = _mm512_set_epi64(0x5, 0x4, 0xFF, 0xFF, 0x9, 0x8, 0x1, 0x0);     //d0s1 d1s1 xx d0s2
    idx[0][2] = _mm512_set_epi64(0xD, 0xC, 0x1, 0x0, 0xFF, 0xFF, 0x9, 0x8);     //d1s2 xx d0s3 d1s3

    idx[1][0] = _mm512_set_epi64(0xFF, 0xFF, 0xB, 0xA, 0x3, 0x2, 0xFF, 0xFF); //xx d0s16 d1s16 xx
    idx[1][1] = _mm512_set_epi64(0x7, 0x6, 0xFF, 0xFF, 0xB, 0xA, 0x3, 0x2);     //d0s17 d1s17 xx d0s18
    idx[1][2] = _mm512_set_epi64(0xF, 0xE, 0x3, 0x2, 0xFF, 0xFF, 0xB, 0xA);     //d1s18 xx d0s19 d1s19

    idx[2][0] = _mm512_set_epi64(0xFF, 0xFF, 0xD, 0xC, 0x5, 0x4, 0xFF, 0xFF);   //xx d0s32 d1s32 xx
    idx[2][1] = _mm512_set_epi64(0x5, 0x4, 0xFF, 0xFF, 0xD, 0xC, 0x1, 0x0);      //d0s33 d1s33 xx d0s34
    idx[2][2] = _mm512_set_epi64(0xD, 0xC, 0x5, 0x4, 0xFF, 0xFF, 0x9, 0x8);     //d1s34 xx d0s35 d1s35

    idx[3][0] = _mm512_set_epi64(0xFF, 0xFF, 0xF, 0xE, 0x7, 0x6, 0xFF, 0xFF); //xx d0s48 d1s48 xx
    idx[3][1] = _mm512_set_epi64(0x7, 0x6, 0xFF, 0xFF, 0xF, 0xE, 0x3, 0x2);     //d0s49 d1s49 xx d0s50
    idx[3][2] = _mm512_set_epi64(0xF, 0xE, 0x7, 0x6, 0xFF, 0xFF, 0xB, 0xA);     //d1s50 xx d0s51 d1s51


    /* inital address for data */
    pD0 = pDeInteleave + nNull;
    pD1 = pD0 + oneThirdNcb + nNull + 64;
    pD2 = pD1 + oneThirdNcb + nNull + 64 - 1;

    paD0[0] = pD0;
    paD1[0] = pD1;
    paD2[0] = pD2;
    for(i=1; i<16; i++)
    {
        paD0[i] = paD0[i-1] + KBlockDiv16;
        paD1[i] = paD1[i-1] + KBlockDiv16;
        paD2[i] = paD2[i-1] + KBlockDiv16;
    }

    /* process for first 12 byte */
    ymm0 = _mm512_loadu_si512 ((__m512i const *)(pD0+KBlock));
    ymm1 = _mm512_loadu_si512 ((__m512i const *)(pD1+KBlock));
    ymm2 = _mm512_loadu_si512 ((__m512i const *)(pD2+KBlock));
    ymm3 = _mm512_unpacklo_epi32 (ymm0, ymm1);
    ymm0 = _mm512_unpacklo_epi64 (ymm3, ymm2);
    if (!isinverted)
    {
        ymm0 = _mm512_subs_epi8(yZero512, ymm0);
    }
    _mm512_storeu_si512 ((__m512i *)pOut, ymm0);
    pOut = pOut + 12;

    _mm512_storeu_si512 ((__m512i *)pOut, yZero512);
    pOut = pOut + 36;

    /*D0,D1*/
    for(k=0; k<KBlockDiv16; k+=64)
    {
        for(j=0; j<16; j++)
        {
            yamm0[j] = _mm512_loadu_si512 ((__m512i const *)paD0[j]);
            paD0[j] = paD0[j] + 64;
            yamm1[j] = _mm512_loadu_si512 ((__m512i const *)paD1[j]);
            paD1[j] = paD1[j] + 64;
        }
        if (!isinverted)
        {
            for(j=0; j<16; j++)
            {
                yamm0[j] = _mm512_subs_epi8(yZero512, yamm0[j]);
                yamm1[j] = _mm512_subs_epi8(yZero512, yamm1[j]);
            }
        }
        /*D0*/
        /* 0:1 s0-s7s16-s23s32-s39s48-s55
            0:1 s8-s15s24-s31s40-s47s56-s63 */
        UNPACKEPI8_AVX512(yamm0[0], yamm0[1], ymm0, ymm1);
        /* 2:3 s0-s7s16-s23s32-s39s48-s55
            2:3 s8-s15s24-s31s40-s47s56-s63 */
        UNPACKEPI8_AVX512(yamm0[2], yamm0[3], yamm0[0], yamm0[1]);
        /* 4:5 s0-s7s16-s23s32-s39s48-s55  s8-s15s24-s31s40-s47s56-s63 */
        UNPACKEPI8_AVX512(yamm0[4], yamm0[5], yamm0[2], yamm0[3]);
        /* 6:7 s0-s7s16-s23s32-s39s48-s55  s8-s15s24-s31s40-s47s56-s63 */
        UNPACKEPI8_AVX512(yamm0[6], yamm0[7], yamm0[4], yamm0[5]);
        /* 8:9 s0-s7s16-s23s32-s39s48-s55  s8-s15s24-s31s40-s47s56-s63 */
        UNPACKEPI8_AVX512(yamm0[8], yamm0[9], yamm0[6], yamm0[7]);
        /* 10:11 s0-s7s16-s23s32-s39s48-s55  s8-s15s24-s31s40-s47s56-s63 */
        UNPACKEPI8_AVX512(yamm0[10], yamm0[11], yamm0[8], yamm0[9]);
        /* 12:13 s0-s7s16-s23s32-s39s48-s55  s8-s15s24-s31s40-s47s56-s63 */
        UNPACKEPI8_AVX512(yamm0[12], yamm0[13], yamm0[10], yamm0[11]);
        /* 14:15 s0-s7s16-s23s32-s39s48-s55  s8-s15s24-s31s40-s47s56-s63 */
        UNPACKEPI8_AVX512(yamm0[14], yamm0[15], yamm0[12], yamm0[13]);

        /*D1*/
        /* 0:1 s0-s7s16-s23s32-s39s48-s55
            0:1 s8-s15s24-s31s40-s47s56-s63 */
        UNPACKEPI8_AVX512(yamm1[0], yamm1[1], ymm2, ymm3);
        /* 2:3 s0-s7s16-s23s32-s37s48-s55  s8-s15s24-s31s40-s47s56-s63 */
        UNPACKEPI8_AVX512(yamm1[2], yamm1[3], yamm1[0], yamm1[1]);
        /* 4:5 s0-s7s16-s23s32-s37s48-s55  s8-s15s24-s31s40-s47s56-s63 */
        UNPACKEPI8_AVX512(yamm1[4], yamm1[5], yamm1[2], yamm1[3]);
        /* 6:7 s0-s7s16-s23s32-s37s48-s55  s8-s15s24-s31s40-s47s56-s63 */
        UNPACKEPI8_AVX512(yamm1[6], yamm1[7], yamm1[4], yamm1[5]);
        /* 8:9 s0-s7s16-s23s32-s37s48-s55  s8-s15s24-s31s40-s47s56-s63 */
        UNPACKEPI8_AVX512(yamm1[8], yamm1[9],yamm1[6], yamm1[7]);
        /* 10:11 s0-s7s16-s23s32-s37s48-s55  s8-s15s24-s31s40-s47s56-s63 */
        UNPACKEPI8_AVX512(yamm1[10], yamm1[11], yamm1[8], yamm1[9]);
        /* 12:13 s0-s7s16-s23s32-s37s48-s55  s8-s15s24-s31s40-s47s56-s63 */
        UNPACKEPI8_AVX512(yamm1[12], yamm1[13], yamm1[10], yamm1[11]);
        /* 14:15 s0-s7s16-s23s32-s37s48-s55  s8-s15s24-s31s40-s47s56-s63 */
        UNPACKEPI8_AVX512(yamm1[14], yamm1[15], yamm1[12], yamm1[13]);

        /*D0*/
        /* 0:3 s0-s3 s16-s19 s32-s35 s48-s51
            0:3 s4-s7 s20-s23 s36-s39 s52-s55
            0:3 s8-s11 s24-s27 s40-s43 s56-s59
            0:3 s12-s15 s28-s31 s44-s47 s60-s63 */
        UNPACKEPI16_AVX512(ymm0, yamm0[0],ymm1, yamm0[1], yamm0[14], yamm0[15], yamm0[16], yamm0[17]);
        /* 4:7 s0-s3 s16-s19 s32-s35 s48-s51
            4:7 s4-s7 s20-s23 s36-s39 s52-s55
            4:7 s8-s11 s24-s27 s40-s43 s56-s59
            4:7 s12-s15 s28-s31 s44-s47 s60-s63 */
        UNPACKEPI16_AVX512(yamm0[2], yamm0[4], yamm0[3], yamm0[5], ymm0, ymm1, yamm0[0], yamm0[1]);
        /* 8:11 s0-s3 s16-s19 s32-s35 s48-s51
            8:11 s4-s7 s20-s23 s36-s39 s52-s55
            8:11 s8-s11 s24-s27 s40-s43 s56-s59
            8:11 s12-s15 s28-s31 s44-s47 s60-s63 */
        UNPACKEPI16_AVX512(yamm0[6], yamm0[8], yamm0[7], yamm0[9], yamm0[2], yamm0[3], yamm0[4], yamm0[5]);
        /* 12:15 s0-s3 s16-s19 s32-s35 s48-s51
            12:15 s4-s7 s20-s23 s36-s39 s52-s55
            12:15 s8-s11 s24-s27 s40-s43 s56-s59
            12:15 s12-s15 s28-s31 s44-s47 s60-s63 */
        UNPACKEPI16_AVX512(yamm0[10], yamm0[12], yamm0[11], yamm0[13], yamm0[6], yamm0[7], yamm0[8], yamm0[9]);

        /*D1*/
        /* 0:3 s0-s3 s16-s19 s32-s35 s48-s51
            0:3 s4-s7 s20-s23 s36-s39 s52-s55
            0:3 s8-s11 s24-s27 s40-s43 s56-s59
            0:3 s12-s15 s28-s31 s44-s47 s60-s63 */
        UNPACKEPI16_AVX512(ymm2, yamm1[0], ymm3, yamm1[1], yamm1[14], yamm1[15], yamm1[16], yamm1[17]);
        /* 4:7 s0-s3 s16-s19 s32-s35 s48-s51
            4:7 s4-s7 s20-s23 s36-s39 s52-s55
            4:7 s8-s11 s24-s27 s40-s43 s56-s59
            4:7 s12-s15 s28-s31 s44-s47 s60-s63 */
        UNPACKEPI16_AVX512(yamm1[2], yamm1[4], yamm1[3], yamm1[5], ymm2, ymm3, yamm1[0], yamm1[1]);
        /* 8:11 s0-s3 s16-s19 s32-s35 s48-s51
            8:11 s4-s7 s20-s23 s36-s39 s52-s55
            8:11 s8-s11 s24-s27 s40-s43 s56-s59
            8:11 s12-s15 s28-s31 s44-s47 s60-s63 */
        UNPACKEPI16_AVX512(yamm1[6], yamm1[8], yamm1[7], yamm1[9], yamm1[2], yamm1[3], yamm1[4], yamm1[5]);
        /* 12:15 s0-s3 s16-s19 s32-s35 s48-s51
            12:15 s4-s7 s20-s23 s36-s39 s52-s55
            12:15 s8-s11 s24-s27 s40-s43 s56-s59
            12:15 s12-s15 s28-s31 s44-s47 s60-s63 */
        UNPACKEPI16_AVX512(yamm1[10], yamm1[12], yamm1[11], yamm1[13], yamm1[6], yamm1[7], yamm1[8], yamm1[9]);


        /*D0*/
        /* 0:7 s0s1 s16s17 s32s33 s48s49 - yamm0[10]
            0:7 s2s3 s18s19 s34s35 s50s51 -yamm0[11]
            0:7 s4s5 s20s21 s36s37 s52s53 -yamm0[12]
            0:7 s6s7 s22s23 s38s39 s54s55 - yamm0[13]) */
        UNPACKEPI32_AVX512(yamm0[14], ymm0, yamm0[15], ymm1, yamm0[10], yamm0[11], yamm0[12], yamm0[13]);

        /* 0:7 s8s9   s24s25 s40s41 s56s57 - ymm0
            0:7 s10s11 s26s27 s42s43 s58s59 - ymm1
            0:7 s12s13 s28s29 s44s45 s60s61 - yamm0[14]
            0:7 s14s15 s30s31 s46s47 s62s63 - yamm0[15]) */
        UNPACKEPI32_AVX512(yamm0[16], yamm0[0], yamm0[17], yamm0[1], ymm0, ymm1, yamm0[14], yamm0[15]);

        /* 8:15 s0s1 s16s17 s32s33 s48s49 - yamm0[0]
            8:15 s2s3 s18s19 s34s35 s50s51 - yamm0[1]
            8:15 s4s5 s20s21 s36s37 s52s53 - yamm0[16]
            8:15 s6s7 s22s23 s38s39 s54s55 - yamm0[17] */
        UNPACKEPI32_AVX512(yamm0[2], yamm0[6], yamm0[3], yamm0[7], yamm0[0], yamm0[1], yamm0[16], yamm0[17]);

        /* 8:15 s8s9   s24s25 s40s41 s56s57 - yamm0[2]
            8:15 s10s11 s26s27 s42s43 s58s59 - yamm0[3]
            8:15 s12s13 s28s29 s44s45 s60s61 - yamm0[6]
            8:15 s14s15 s30s31 s46s47 s62s63 - yamm0[7] */
        UNPACKEPI32_AVX512(yamm0[4], yamm0[8], yamm0[5], yamm0[9], yamm0[2], yamm0[3], yamm0[6], yamm0[7]);

        /*D1*/
        /* 0:7 s0s1 s16s17 s32s33 s48s49 - yamm1[10]
            0:7 s2s3 s18s19 s34s35 s50s51 -yamm1[11]
            0:7 s4s5 s20s21 s36s37 s52s53 -yamm1[12]
            0:7 s6s7 s22s23 s38s39 s54s55 - yamm1[13]) */
        UNPACKEPI32_AVX512(yamm1[14], ymm2, yamm1[15], ymm3, yamm1[10], yamm1[11], yamm1[12], yamm1[13]);
        /* 0:7 s8s9   s24s25 s40s41 s56s57 - ymm2
            0:7 s10s11 s26s27 s42s43 s58s59 - ymm3
            0:7 s12s13 s28s29 s44s45 s60s61 - yamm1[14]
            0:7 s14s15 s30s31 s46s47 s62s63 - yamm1[15]) */
        UNPACKEPI32_AVX512(yamm1[16], yamm1[0], yamm1[17], yamm1[1], ymm2, ymm3, yamm1[14], yamm1[15]);
        /* 8:15 s0s1 s16s17 s32s33 s48s49 - yamm1[0]
            8:15 s2s3 s18s19 s34s35 s50s51 - yamm1[1]
            8:15 s4s5 s20s21 s36s37 s52s53 - yamm1[16]
            8:15 s6s7 s22s23 s38s39 s54s55 - yamm1[17] */
        UNPACKEPI32_AVX512(yamm1[2], yamm1[6], yamm1[3], yamm1[7], yamm1[0], yamm1[1], yamm1[16], yamm1[17]);
        /* 8:15 s8s9   s24s25 s40s41 s56s57 - yamm1[2]
            8:15 s10s11 s26s27 s42s43 s58s59 - yamm1[3]
            8:15 s12s13 s28s29 s44s45 s60s61 - yamm1[6]
            8:15 s14s15 s30s31 s46s47 s62s63 - yamm1[7] */
        UNPACKEPI32_AVX512(yamm1[4], yamm1[8], yamm1[5], yamm1[9], yamm1[2],  yamm1[3], yamm1[6], yamm1[7]);


        /*D0*/
        /* 0:15
        ymmTemp0[0] - s0 s16 s32 s48
        ymmTemp0[1] - s1 s17 s33 s49
        ymmTemp0[2] - s2 s18 s34 s50
        ymmTemp0[3] - s3 s19 s35 s51*/
        UNPACKEPI64_AVX512( yamm0[10], yamm0[0], yamm0[11], yamm0[1], ymmTemp0[0], ymmTemp0[1], ymmTemp0[2], ymmTemp0[3]);
        /* 0:15
        ymmTemp0[4] - s4 s20 s36 s52
        ymmTemp0[5] - s5 s21 s37 s53
        ymmTemp0[6] - s6 s22 s38 s54
        ymmTemp0[7] - s7 s23 s39 s55*/
        UNPACKEPI64_AVX512( yamm0[12], yamm0[16], yamm0[13],yamm0[17], ymmTemp0[4], ymmTemp0[5], ymmTemp0[6], ymmTemp0[7]);
        /* 0:15
        ymmTemp0[8] -   s8 s24 s40 s56
        ymmTemp0[9] -   s9 s25 s41 s57
        ymmTemp0[10] - s10 s26 s42 s58
        ymmTemp0[11] - s11 s27 s43 s59 */
        UNPACKEPI64_AVX512( ymm0, yamm0[2], ymm1, yamm0[3], ymmTemp0[8], ymmTemp0[9], ymmTemp0[10], ymmTemp0[11]);
        /* 0:15
        ymmTemp0[12] - s12 s28 s44 s60
        ymmTemp0[13] - s13 s29 s45 s61
        ymmTemp0[14] - s14 s30 s46 s62
        ymmTemp0[15] - s15 s31 s47 s63 */
        UNPACKEPI64_AVX512( yamm0[14], yamm0[6], yamm0[15], yamm0[7], ymmTemp0[12], ymmTemp0[13], ymmTemp0[14], ymmTemp0[15]);

        /*D1*/
        /* 0:15   s0s16s32s48 s1s17s33s49 s2s18s34s50 s3s19s35s51 */
        UNPACKEPI64_AVX512( yamm1[10], yamm1[0], yamm1[11], yamm1[1], ymmTemp1[0], ymmTemp1[1], ymmTemp1[2], ymmTemp1[3]);
        /* 0:15   s4s20s36s52 s5s21s37s53 s6s22s38s54 s7s23s39s55 */
        UNPACKEPI64_AVX512( yamm1[12], yamm1[16], yamm1[13], yamm1[17], ymmTemp1[4], ymmTemp1[5], ymmTemp1[6], ymmTemp1[7]);
        /* 0:15   s8s24s40s56 s9s25s41s57 s10s26s42s58 s11s27s43s59 */
        UNPACKEPI64_AVX512( ymm2, yamm1[2], ymm3, yamm1[3], ymmTemp1[8], ymmTemp1[9], ymmTemp1[10], ymmTemp1[11]);
        /* 0:15   s12s28s44s60 s13s29s45s61 s14s30s46s62 s15s31s47s63 */
        UNPACKEPI64_AVX512( yamm1[14], yamm1[6], yamm1[15], yamm1[7], ymmTemp1[12], ymmTemp1[13], ymmTemp1[14], ymmTemp1[15]);

        /* for packing 512bits with zero, d0 and d1, need to re-pack following data */
        yamm0[0] = _mm512_shuffle_i64x2(ymmTemp0[1], ymmTemp0[2], 0x44);     //s1 s17 s2 s18
        yamm0[8] = _mm512_shuffle_i64x2(ymmTemp0[1], ymmTemp0[2], 0xEE);     //s33 s49 s34 s50
        yamm0[1] = _mm512_shuffle_i64x2(ymmTemp0[5], ymmTemp0[6], 0x44);     //s5 s21 s6 s22
        yamm0[9] = _mm512_shuffle_i64x2(ymmTemp0[5], ymmTemp0[6], 0xEE);     //s37 s53 s38 s54
        yamm0[2] = _mm512_shuffle_i64x2(ymmTemp0[9], ymmTemp0[10], 0x44);     //s9 s25 s10 s26
        yamm0[10] = _mm512_shuffle_i64x2(ymmTemp0[9], ymmTemp0[10], 0xEE);     //s41 s57 s42 s58
        yamm0[3] = _mm512_shuffle_i64x2(ymmTemp0[13], ymmTemp0[14], 0x44);     //s13 s29 s14 s30
        yamm0[11] = _mm512_shuffle_i64x2(ymmTemp0[13], ymmTemp0[14], 0xEE);     //s45 s61 s46 s62

        yamm1[0] = _mm512_shuffle_i64x2(ymmTemp1[2], ymmTemp1[3], 0x44);     //s2 s18 s3 s19
        yamm1[8] = _mm512_shuffle_i64x2(ymmTemp1[2], ymmTemp1[3], 0xEE);     //s34 s50 s35 s51
        yamm1[1] = _mm512_shuffle_i64x2(ymmTemp1[6], ymmTemp1[7], 0x44);     //s6 s22 s7 s23
        yamm1[9] = _mm512_shuffle_i64x2(ymmTemp1[6], ymmTemp1[7], 0xEE);     //s38 s54 s39 s55
        yamm1[2] = _mm512_shuffle_i64x2(ymmTemp1[10], ymmTemp1[11], 0x44);     //s10 s26 s11 s27
        yamm1[10] = _mm512_shuffle_i64x2(ymmTemp1[10], ymmTemp1[11], 0xEE);     //s42 s58 s43 s59
        yamm1[3] = _mm512_shuffle_i64x2(ymmTemp1[14], ymmTemp1[15], 0x44);     //s14 s30 s15 s31
        yamm1[11] = _mm512_shuffle_i64x2(ymmTemp1[14], ymmTemp1[15], 0xEE);     //s46 s62 s47 s63


        if((k+64)<=KBlockDiv16)
        {
            for(i=0, j=0; i<16; i+=4, j++)
            {
                _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[0], ymmTemp0[i], idx[0][0], ymmTemp1[i]));
                pOut = pOut + 64;
                _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[1], yamm0[j], idx[0][1], ymmTemp1[i+1]));
                pOut = pOut + 64;
                _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[2], ymmTemp0[i+3], idx[0][2], yamm1[j]));
                pOut = pOut + 64;
            }

            for(i=0, j=0; i<16; i+=4, j++)
            {
                _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[0], ymmTemp0[i], idx[1][0], ymmTemp1[i]));
                pOut = pOut + 64;
                _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[1], yamm0[j], idx[1][1], ymmTemp1[i+1]));
                pOut = pOut + 64;
                _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[2], ymmTemp0[i+3], idx[1][2], yamm1[j]));
                pOut = pOut + 64;
            }

            for(i=0, j=8; i<16; i+=4, j++)
            {
                _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[0], ymmTemp0[i], idx[2][0], ymmTemp1[i]));
                pOut = pOut + 64;
                _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[1], yamm0[j], idx[2][1], ymmTemp1[i+1]));
                pOut = pOut + 64;
                _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[2], ymmTemp0[i+3], idx[2][2], yamm1[j]));
                pOut = pOut + 64;
            }

            for(i=0, j=8; i<16; i+=4, j++)
            {
                _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[0], ymmTemp0[i], idx[3][0], ymmTemp1[i]));
                pOut = pOut + 64;
                _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[1], yamm0[j], idx[3][1], ymmTemp1[i+1]));
                pOut = pOut + 64;
                _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[2], ymmTemp0[i+3], idx[3][2], yamm1[j]));
                pOut = pOut + 64;
            }
        }
        else
        {
            if((k+48)<=KBlockDiv16)
            {
                for(i=0, j=0; i<16; i+=4, j++)
                {
                    _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[0], ymmTemp0[i], idx[0][0], ymmTemp1[i]));
                    pOut = pOut + 64;
                    _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[1], yamm0[j], idx[0][1], ymmTemp1[i+1]));
                    pOut = pOut + 64;
                    _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[2], ymmTemp0[i+3], idx[0][2], yamm1[j]));
                    pOut = pOut + 64;
                }

                for(i=0, j=0; i<16; i+=4, j++)
                {
                    _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[0], ymmTemp0[i], idx[1][0], ymmTemp1[i]));
                    pOut = pOut + 64;
                    _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[1], yamm0[j], idx[1][1], ymmTemp1[i+1]));
                    pOut = pOut + 64;
                    _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[2], ymmTemp0[i+3], idx[1][2], yamm1[j]));
                    pOut = pOut + 64;
                }

                for(i=0, j=8; i<16; i+=4, j++)
                {
                    _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[0], ymmTemp0[i], idx[2][0], ymmTemp1[i]));
                    pOut = pOut + 64;
                    _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[1], yamm0[j], idx[2][1], ymmTemp1[i+1]));
                    pOut = pOut + 64;
                    _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[2], ymmTemp0[i+3], idx[2][2], yamm1[j]));
                    pOut = pOut + 64;
                }

                cntOf64Byte = (KBlockDiv16 - k - 48)>>2;
                cntOf16Byte = KBlockDiv16 - k - 48 - (cntOf64Byte<<2);

                j = 8;
                index = 3;

            }
            else if((k+32)<=KBlockDiv16)
            {
                for(i=0, j=0; i<16; i+=4, j++)
                {
                    _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[0], ymmTemp0[i], idx[0][0], ymmTemp1[i]));
                    pOut = pOut + 64;
                    _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[1], yamm0[j], idx[0][1], ymmTemp1[i+1]));
                    pOut = pOut + 64;
                    _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[2], ymmTemp0[i+3], idx[0][2], yamm1[j]));
                    pOut = pOut + 64;
                }

                for(i=0, j=0; i<16; i+=4, j++)
                {
                    _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[0], ymmTemp0[i], idx[1][0], ymmTemp1[i]));
                    pOut = pOut + 64;
                    _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[1], yamm0[j], idx[1][1], ymmTemp1[i+1]));
                    pOut = pOut + 64;
                    _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[2], ymmTemp0[i+3], idx[1][2], yamm1[j]));
                    pOut = pOut + 64;
                }

                cntOf64Byte = (KBlockDiv16 - k - 32)>>2;
                cntOf16Byte = KBlockDiv16 - k - 32 - (cntOf64Byte<<2);

                j = 8;
                index = 2;

            }
            else if((k+16)<=KBlockDiv16)
            {
                for(i=0, j=0; i<16; i+=4, j++)
                {
                    _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[0], ymmTemp0[i], idx[0][0], ymmTemp1[i]));
                    pOut = pOut + 64;
                    _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[1], yamm0[j], idx[0][1], ymmTemp1[i+1]));
                    pOut = pOut + 64;
                    _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[2], ymmTemp0[i+3], idx[0][2], yamm1[j]));
                    pOut = pOut + 64;
                }

                cntOf64Byte = (KBlockDiv16 - k - 16)>>2;
                cntOf16Byte = KBlockDiv16 - k - 16 - (cntOf64Byte<<2);

                j = 0;
                index = 1;

            }
            else
            {
                cntOf64Byte = (KBlockDiv16 - k)>>2;
                cntOf16Byte = KBlockDiv16 - k - (cntOf64Byte<<2);

                j = 0;
                index = 0;

            }

            switch(cntOf64Byte)
            {
                case 3:
                {
                    for(i=0; i<12; i+=4, j++)
                    {
                        _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[0], ymmTemp0[i], idx[index][0], ymmTemp1[i]));
                        pOut = pOut + 64;
                        _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[1], yamm0[j], idx[index][1], ymmTemp1[i+1]));
                        pOut = pOut + 64;
                        _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[2], ymmTemp0[i+3], idx[index][2], yamm1[j]));
                        pOut = pOut + 64;
                    }

                    break;
                }
                case 2:
                {
                    for(i=0; i<8; i+=4, j++)
                    {
                        _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[0], ymmTemp0[i], idx[index][0], ymmTemp1[i]));
                        pOut = pOut + 64;
                        _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[1], yamm0[j], idx[index][1], ymmTemp1[i+1]));
                        pOut = pOut + 64;
                        _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[2], ymmTemp0[i+3], idx[index][2], yamm1[j]));
                        pOut = pOut + 64;
                    }

                    break;
                }
                case 1:
                {
                    for(i=0; i<4; i+=4, j++)
                    {
                        _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[0], ymmTemp0[i], idx[index][0], ymmTemp1[i]));
                        pOut = pOut + 64;
                        _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[1], yamm0[j], idx[index][1], ymmTemp1[i+1]));
                        pOut = pOut + 64;
                        _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[2], ymmTemp0[i+3], idx[index][2], yamm1[j]));
                        pOut = pOut + 64;
                    }

                    break;
                }
                case 0:
                {
                    i = 0;

                    break;
                }

            }

            if(cntOf16Byte == 1)
            {
                _mm_storeu_si128 ((__m128i *)pOut, yZero128);
                pOut = pOut + 16;

                pTmp = (__m128i *)&ymmTemp0[i];
                _mm_storeu_si128 ((__m128i *)pOut, *(pTmp+index));
                pOut = pOut + 16;

                pTmp = (__m128i *)&ymmTemp1[i];
                _mm_storeu_si128 ((__m128i *)pOut, *(pTmp+index));
                pOut = pOut + 16;
            }
            else if(cntOf16Byte == 2)
            {
                _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[0], ymmTemp0[i], idx[index][0], ymmTemp1[i]));
                pOut = pOut + 64;
                //pOut = pOut + 64;
                pTmp = (__m128i *)&ymmTemp0[i+1];
                _mm_storeu_si128 ((__m128i *)pOut, *(pTmp+index));
                pOut = pOut + 16;

                pTmp = (__m128i *)&ymmTemp1[i+1];
                _mm_storeu_si128 ((__m128i *)pOut, *(pTmp+index));
                pOut = pOut + 16;
            }
            else if(cntOf16Byte == 3)
            {
                _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[0], ymmTemp0[i], idx[index][0], ymmTemp1[i]));
                pOut = pOut + 64;

                _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[1], yamm0[j], idx[index][1], ymmTemp1[i+1]));
                pOut = pOut + 64;

                pTmp = (__m128i *)&ymmTemp1[i+2];
                _mm_storeu_si128 ((__m128i *)pOut, *(pTmp+index));
                pOut = pOut + 16;
            }

        }

    }

    /*D2*/
    for(k=0; k<KBlockDiv16; k+=64)
    {
        for(j=0; j<16; j++)
        {
            yamm0[j] = _mm512_loadu_si512 ((__m256i const *)paD2[j]);
        	paD2[j] = paD2[j] + 64;
        }
        if (!isinverted)
        {
            for(j=0; j<16; j++)
            {
                yamm0[j] = _mm512_subs_epi8(yZero512, yamm0[j]);
            }
        }
        /* 0:1 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8_AVX512(yamm0[0], yamm0[1],ymm0,ymm1);
        /* 2:3 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8_AVX512(yamm0[2], yamm0[3],yamm0[0], yamm0[1]);
        /* 4:5 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8_AVX512(yamm0[4], yamm0[5],yamm0[2], yamm0[3]);
        /* 6:7 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8_AVX512(yamm0[6], yamm0[7],yamm0[4], yamm0[5]);
        /* 8:9 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8_AVX512(yamm0[8], yamm0[9],yamm0[6], yamm0[7]);
        /* 10:11 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8_AVX512(yamm0[10], yamm0[11],yamm0[8], yamm0[9]);
        /* 12:13 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8_AVX512(yamm0[12], yamm0[13],yamm0[10], yamm0[11]);
        /* 14:15 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8_AVX512(yamm0[14], yamm0[15],yamm0[12], yamm0[13]);

        /* 0:3   s0-s3s16-s19 s4-s7s20-s23 s8-s11s24-s27 s12-s15s28-s31 */
        UNPACKEPI16_AVX512(ymm0,    yamm0[0],ymm1,  yamm0[1], yamm0[14],yamm0[15],yamm0[16],yamm0[17]);
        /* 4:7   s0-s3s16-s19 s4-s7s20-s23 s8-s11s24-s27 s12-s15s28-s31 */
        UNPACKEPI16_AVX512(yamm0[2],  yamm0[4],yamm0[3],yamm0[5],ymm0,   ymm1, yamm0[0],   yamm0[1]);
        /* 8:11  s0-s3s16-s19 s4-s7s20-s23 s8-s11s24-s27 s12-s15s28-s31 */
        UNPACKEPI16_AVX512(yamm0[6],  yamm0[8],yamm0[7],yamm0[9], yamm0[2],  yamm0[3],yamm0[4],yamm0[5]);
        /* 12:15 s0-s3s16-s19 s4-s7s20-s23 s8-s11s24-s27 s12-s15s28-s31 */
        UNPACKEPI16_AVX512(yamm0[10],  yamm0[12],yamm0[11],yamm0[13],yamm0[6],  yamm0[7],yamm0[8],yamm0[9]);

        /* 0:7   s0s1s16s17 s2s3s18s19 s4s5s20s21 s6s7s22s23 */
        UNPACKEPI32_AVX512(yamm0[14], ymm0,    yamm0[15], ymm1,  yamm0[10],  yamm0[11],yamm0[12],yamm0[13]);
        /* s8s9s24s25 s10s11s26s27 s12s13s28s29 s14s15s30s31 */
        UNPACKEPI32_AVX512(yamm0[16],    yamm0[0],    yamm0[17],    yamm0[1], ymm0, ymm1,  yamm0[14],  yamm0[15]);
        /* 8:15   s0s1s16s17 s2s3s18s19 s4s5s20s21 s6s7s22s23 */
        UNPACKEPI32_AVX512(yamm0[2],  yamm0[6], yamm0[3], yamm0[7],   yamm0[0],    yamm0[1],  yamm0[16],    yamm0[17]);
        /* s8s9s24s25 s10s11s26s27 s12s13s28s29 s14s15s30s31 */
        UNPACKEPI32_AVX512(yamm0[4], yamm0[8], yamm0[5], yamm0[9], yamm0[2],  yamm0[3], yamm0[6], yamm0[7]);

        /* 0:15   s0s16 s1s17 s2s18 s3s19 */
        UNPACKEPI64_AVX512( yamm0[10], yamm0[0], yamm0[11], yamm0[1],        ymmTemp1[0], ymmTemp1[1], ymmTemp1[2], ymmTemp1[3]);
        /* 0:15   s4s20 s5s21 s6s22 s7s23 */
        UNPACKEPI64_AVX512( yamm0[12], yamm0[16], yamm0[13],yamm0[17],    ymmTemp1[4], ymmTemp1[5], ymmTemp1[6], ymmTemp1[7]);
        /* 0:15   s8s24 s9s25 s10s26 s11s27 */
        UNPACKEPI64_AVX512( ymm0, yamm0[2], ymm1, yamm0[3],    ymmTemp1[8], ymmTemp1[9], ymmTemp1[10], ymmTemp1[11]);
        /* 0:15   s12s28 s13s29 s14s30 s15s31 */
        UNPACKEPI64_AVX512( yamm0[14], yamm0[6], yamm0[15], yamm0[7],    ymmTemp1[12], ymmTemp1[13], ymmTemp1[14], ymmTemp1[15]);


        yamm1[0] = _mm512_shuffle_i64x2(ymmTemp1[2], ymmTemp1[3], 0x44);     //s2 s18 s3 s19
        yamm1[8] = _mm512_shuffle_i64x2(ymmTemp1[2], ymmTemp1[3], 0xEE);     //s34 s50 s35 s51
        yamm1[1] = _mm512_shuffle_i64x2(ymmTemp1[6], ymmTemp1[7], 0x44);     //s6 s22 s7 s23
        yamm1[9] = _mm512_shuffle_i64x2(ymmTemp1[6], ymmTemp1[7], 0xEE);     //s38 s54 s39 s55
        yamm1[2] = _mm512_shuffle_i64x2(ymmTemp1[10], ymmTemp1[11], 0x44);     //s10 s26 s11 s27
        yamm1[10] = _mm512_shuffle_i64x2(ymmTemp1[10], ymmTemp1[11], 0xEE);     //s42 s58 s43 s59
        yamm1[3] = _mm512_shuffle_i64x2(ymmTemp1[14], ymmTemp1[15], 0x44);     //s14 s30 s15 s31
        yamm1[11] = _mm512_shuffle_i64x2(ymmTemp1[14], ymmTemp1[15], 0xEE);     //s46 s62 s47 s63

        if((k+64)<=KBlockDiv16)
        {
            for(i=0, j=0; i<16; i+=4, j++)
            {
                _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[0], yZero512, idx[0][0], ymmTemp1[i]));
                pOut = pOut + 64;
                _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[1], yZero512, idx[0][1], ymmTemp1[i+1]));
                pOut = pOut + 64;
                _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[2], yZero512, idx[0][2], yamm1[j]));
                pOut = pOut + 64;
            }

            for(i=0, j=0; i<16; i+=4, j++)
            {
                _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[0], yZero512, idx[1][0], ymmTemp1[i]));
                pOut = pOut + 64;
                _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[1], yZero512, idx[1][1], ymmTemp1[i+1]));
                pOut = pOut + 64;
                _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[2], yZero512, idx[1][2], yamm1[j]));
                pOut = pOut + 64;
            }

            for(i=0, j=8; i<16; i+=4, j++)
            {
                _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[0], yZero512, idx[2][0], ymmTemp1[i]));
                pOut = pOut + 64;
                _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[1], yZero512, idx[2][1], ymmTemp1[i+1]));
                pOut = pOut + 64;
                _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[2], yZero512, idx[2][2], yamm1[j]));
                pOut = pOut + 64;
            }

            for(i=0, j=8; i<16; i+=4, j++)
            {
                _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[0], yZero512, idx[3][0], ymmTemp1[i]));
                pOut = pOut + 64;
                _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[1], yZero512, idx[3][1], ymmTemp1[i+1]));
                pOut = pOut + 64;
                _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[2], yZero512, idx[3][2], yamm1[j]));
                pOut = pOut + 64;
            }
        }
        else
        {
            if((k+48)<=KBlockDiv16)
            {
                for(i=0, j=0; i<16; i+=4, j++)
                {
                    _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[0], yZero512, idx[0][0], ymmTemp1[i]));
                    pOut = pOut + 64;
                    _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[1], yZero512, idx[0][1], ymmTemp1[i+1]));
                    pOut = pOut + 64;
                    _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[2], yZero512, idx[0][2], yamm1[j]));
                    pOut = pOut + 64;
                }

                for(i=0, j=0; i<16; i+=4, j++)
                {
                    _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[0], yZero512, idx[1][0], ymmTemp1[i]));
                    pOut = pOut + 64;
                    _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[1], yZero512, idx[1][1], ymmTemp1[i+1]));
                    pOut = pOut + 64;
                    _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[2], yZero512, idx[1][2], yamm1[j]));
                    pOut = pOut + 64;
                }

                for(i=0, j=8; i<16; i+=4, j++)
                {
                    _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[0], yZero512, idx[2][0], ymmTemp1[i]));
                    pOut = pOut + 64;
                    _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[1], yZero512, idx[2][1], ymmTemp1[i+1]));
                    pOut = pOut + 64;
                    _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[2], yZero512, idx[2][2], yamm1[j]));
                    pOut = pOut + 64;
                }

                cntOf64Byte = (KBlockDiv16 - k - 48)>>2;
                cntOf16Byte = KBlockDiv16 - k - 48 - (cntOf64Byte<<2);

                j = 8;
                index = 3;

            }
            else if((k+32)<=KBlockDiv16)
            {
                for(i=0, j=0; i<16; i+=4, j++)
                {
                    _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[0], yZero512, idx[0][0], ymmTemp1[i]));
                    pOut = pOut + 64;
                    _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[1], yZero512, idx[0][1], ymmTemp1[i+1]));
                    pOut = pOut + 64;
                    _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[2], yZero512, idx[0][2], yamm1[j]));
                    pOut = pOut + 64;
                }

                for(i=0, j=0; i<16; i+=4, j++)
                {
                    _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[0], yZero512, idx[1][0], ymmTemp1[i]));
                    pOut = pOut + 64;
                    _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[1], yZero512, idx[1][1], ymmTemp1[i+1]));
                    pOut = pOut + 64;
                    _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[2], yZero512, idx[1][2], yamm1[j]));
                    pOut = pOut + 64;
                }

                cntOf64Byte = (KBlockDiv16 - k - 32)>>2;
                cntOf16Byte = KBlockDiv16 - k - 32 - (cntOf64Byte<<2);

                j = 8;
                index = 2;
            }
            else if((k+16)<=KBlockDiv16)
            {
                for(i=0, j=0; i<16; i+=4, j++)
                {
                    _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[0], yZero512, idx[0][0], ymmTemp1[i]));
                    pOut = pOut + 64;
                    _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[1], yZero512, idx[0][1], ymmTemp1[i+1]));
                    pOut = pOut + 64;
                    _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[2], yZero512, idx[0][2], yamm1[j]));
                    pOut = pOut + 64;
                }

                cntOf64Byte = (KBlockDiv16 - k - 16)>>2;
                cntOf16Byte = KBlockDiv16 - k - 16 - (cntOf64Byte<<2);

                j = 0;
                index = 1;

            }
            else
            {
                cntOf64Byte = (KBlockDiv16 - k)>>2;
                cntOf16Byte = KBlockDiv16 - k - (cntOf64Byte<<2);

                j = 0;
                index = 0;

            }

            switch(cntOf64Byte)
            {
                case 3:
                {
                    for(i=0; i<12; i+=4, j++)
                    {
                        _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[0], yZero512, idx[index][0], ymmTemp1[i]));
                        pOut = pOut + 64;
                        _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[1], yZero512, idx[index][1], ymmTemp1[i+1]));
                        pOut = pOut + 64;
                        _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[2], yZero512, idx[index][2], yamm1[j]));
                        pOut = pOut + 64;
                    }

                    break;
                }
                case 2:
                {
                    for(i=0; i<8; i+=4, j++)
                    {
                        _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[0], yZero512, idx[index][0], ymmTemp1[i]));
                        pOut = pOut + 64;
                        _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[1], yZero512, idx[index][1], ymmTemp1[i+1]));
                        pOut = pOut + 64;
                        _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[2], yZero512, idx[index][2], yamm1[j]));
                        pOut = pOut + 64;
                    }

                    break;
                }
                case 1:
                {
                    for(i=0; i<4; i+=4, j++)
                    {
                        _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[0], yZero512, idx[index][0], ymmTemp1[i]));
                        pOut = pOut + 64;
                        _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[1], yZero512, idx[index][1], ymmTemp1[i+1]));
                        pOut = pOut + 64;
                        _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[2], yZero512, idx[index][2], yamm1[j]));
                        pOut = pOut + 64;
                    }

                    break;
                }
                case 0:
                {
                    i = 0;

                    break;
                }

            }

            if(cntOf16Byte == 1)
            {
                _mm256_storeu_si256 ((__m256i *)pOut, yZero256);
                pOut = pOut + 32;

                pTmp = (__m128i *)&ymmTemp1[i];
                _mm_storeu_si128 ((__m128i *)pOut, *(pTmp+index));
                pOut = pOut + 16;
            }
            else if(cntOf16Byte == 2)
            {
                _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[0], yZero512, idx[index][0], ymmTemp1[i]));
                pOut = pOut + 64;

                //pTmp = (__m128i *)&ymmTemp1[i+1];
                _mm_storeu_si128 ((__m128i *)pOut, yZero128);
                pOut = pOut + 16;

                pTmp = (__m128i *)&ymmTemp1[i+1];
                _mm_storeu_si128 ((__m128i *)pOut, *(pTmp+index));
                pOut = pOut + 16;
            }
            else if(cntOf16Byte == 3)
            {
                _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[0], yZero512, idx[index][0], ymmTemp1[i]));
                pOut = pOut + 64;

                _mm512_storeu_si512 ((__m512i *)pOut, _mm512_maskz_permutex2var_epi64(kFlag[1], yZero512, idx[index][1], ymmTemp1[i+1]));
                pOut = pOut + 64;

                pTmp = (__m128i *)&ymmTemp1[i+2];
                _mm_storeu_si128 ((__m128i *)pOut, *(pTmp+index));
                pOut = pOut + 16;
            }

        }

    }

}

void turboAdapterK8_avx512(uint8_t *pDeInteleave, uint8_t *pTurbo, const int32_t oneThirdNcb, const int32_t nNull, const int32_t KBlock, int32_t isinverted)
{
    int32_t KBlockDiv8 = 0;
    uint8_t *pD0,*pD1,*pD2, *pOut = pTurbo;;

    __m512i ymm0,ymm1,ymm2,ymm3;
    __m512i yZero = _mm512_set1_epi8 (0);
    __m512i idx0 = _mm512_set_epi64(0xB, 0xA, 0x3, 0x2, 0x9, 0x8, 0x1, 0x0);   //xx d0s0 d1s0 xx
    __m512i idx1 = _mm512_set_epi64(0xF, 0xE, 0x7, 0x6, 0xD, 0xC, 0x5, 0x4);   //xx d0s0 d1s0 xx

    /* calculate parameter */
    KBlockDiv8 = (KBlock>>3);
    /* inital address for data */
    pD0 = pDeInteleave + nNull;
    pD1 = pD0 + oneThirdNcb + nNull + 64;
    pD2 = pD1 + oneThirdNcb + nNull + 64 - 1;

    /* process for first 12 byte */
    ymm0 = _mm512_loadu_si512 ((__m512i const *)(pD0+KBlock));
    ymm1 = _mm512_loadu_si512 ((__m512i const *)(pD1+KBlock));
    ymm2 = _mm512_loadu_si512 ((__m512i const *)(pD2+KBlock));
    ymm3 = _mm512_unpacklo_epi32 (ymm0, ymm1);
    ymm0 = _mm512_unpacklo_epi64 (ymm3, ymm2);

    if (!isinverted)
    {
        ymm0 = _mm512_subs_epi8(yZero, ymm0);
    }
    _mm512_storeu_si512 ((__m512i *)pOut, ymm0);
    pOut = pOut + 12;
    _mm512_storeu_si512 ((__m512i *)pOut, yZero);
    pOut = pOut + 36;

    /*D0,D1*/
    for(int32_t i=0; i<KBlock; i+=64)
    {
        ymm0 = _mm512_loadu_si512 ((__m512i const *)pD0);
        pD0 = pD0 + 64;
        ymm1 = _mm512_loadu_si512 ((__m512i const *)pD1);
        pD1 = pD1 + 64;

        /* 0:1 s0-s7s16-s23s32-s39s48-s55 -- ymm2
            0:1 s8-s15s24-s31s40-s47s56-s63  -- ymm3 */
        UNPACKEPI8_AVX512(ymm0,ymm1,ymm2,ymm3);
        ymm0 = _mm512_permutex2var_epi64(ymm2, idx0, ymm3);//_mm256_permute2x128_si256 (ymm2,ymm3, 0x20);
        ymm1 = _mm512_permutex2var_epi64(ymm2, idx1, ymm3);//_mm256_permute2x128_si256 (ymm2, ymm3,0x31);

        if (!isinverted)
        {
            ymm0 = _mm512_subs_epi8(yZero, ymm0);
            ymm1 = _mm512_subs_epi8(yZero, ymm1);
        }
        _mm512_storeu_si512 ((__m512i *)pOut, ymm0);
        pOut = pOut + 64;
        _mm512_storeu_si512 ((__m512i *)pOut, ymm1);
        pOut = pOut + 64;
    }

    pOut = pTurbo + 48 + KBlock*6;

    /*D2*/
    for(int32_t i=0; i<KBlock; i+=64)
    {
        ymm0 = _mm512_loadu_si512 ((__m512i const *)pD2);
        pD2 = pD2 + 64;

        /* 0:1 s0-s7s16-s23s32-s39s48-s55 -- ymm2
            0:1 s8-s15s24-s31s40-s47s56-s63  -- ymm3 */
        UNPACKEPI8_AVX512(yZero,ymm0,ymm2,ymm3);
        ymm0 = _mm512_permutex2var_epi64(ymm2, idx0, ymm3);//_mm256_permute2x128_si256 (ymm2,ymm3, 0x20);
        ymm1 = _mm512_permutex2var_epi64(ymm2, idx1, ymm3);//_mm256_permute2x128_si256 (ymm2, ymm3,0x31);

        if (!isinverted)
        {
            ymm0 = _mm512_subs_epi8(yZero, ymm0);
            ymm1 = _mm512_subs_epi8(yZero, ymm1);
        }
        _mm512_storeu_si512 ((__m512i *)pOut, ymm0);
        pOut = pOut + 64;
        _mm512_storeu_si512 ((__m512i *)pOut, ymm1);
        pOut = pOut + 64;
    }
}

int32_t bblib_turbo_adapter_ul_avx512(const struct bblib_turbo_adapter_ul_request *request,
    struct bblib_turbo_adapter_ul_response *response)
{
    uint8_t *pDeInteleave = request->pinteleavebuffer;
    uint8_t *pTurbo = response->pharqout;
    int32_t ncb = request->ncb;
    int32_t isinverted = request->isinverted;

    int32_t oneThirdNcb = ncb/3;
    int32_t Rsubblock = ceil((float)oneThirdNcb/(float)32);
    int32_t subBlockWithNull = (Rsubblock <<5);
    int32_t nNull = subBlockWithNull - oneThirdNcb;

    int32_t KBlock = oneThirdNcb - 4;

    if((KBlock%16) == 0)
    {
        turboAdapterK16_avx512(pDeInteleave, pTurbo, oneThirdNcb, nNull, KBlock, isinverted);
    }
    else
    {
        turboAdapterK8_avx512(pDeInteleave, pTurbo, oneThirdNcb, nNull, KBlock, isinverted);
    }

    return 0;
}

int32_t bblib_harq_combine_ul_avx512(const struct bblib_harq_combine_ul_request *request,
    struct bblib_harq_combine_ul_response *response)
{
    uint8_t *pIn = request->pdmout;
    uint8_t *pHarq = response->pharqbuffer;
    int32_t k0withoutnull = request->k0withoutnull;
    int32_t ncb = request->ncb;
    int32_t EWithoutNull = request->e;
    int32_t isretx = request->isretx;

    int32_t i;
    int32_t NcbSubK0Start = 0;
    int32_t ESubK0Start = 0;
    int32_t wrapLen = 0;
    int32_t ncbStep_64 = 0, ncbStep_16=0, ncbStep_1=0, ncbTmp=0;
    __m512i ymm0_avx512;
    __m128i ymm0_sse128;
    uint8_t *pInTmp, *pHarqTmp;

    ESubK0Start = EWithoutNull;
    NcbSubK0Start = ncb - k0withoutnull;
    /* first transmition should fill zero to harq buffer */
    if(isretx == 0)
    {
        /* fill zero into harq buffer */
        ncbStep_64 = ncb&0xffffffc0;
        ncbTmp = ncb&0x3f;
        ncbStep_16 = ncbTmp &0x30;
        ncbStep_1 = ncbTmp &0xf;

        pHarqTmp = pHarq;
        ymm0_avx512 = _mm512_set1_epi8 (0);
        ymm0_sse128 = _mm_set1_epi8 (0);

        for(i=0; i<ncbStep_64; i+=64)
        {
            _mm512_storeu_si512 ((__m512i *) pHarqTmp, ymm0_avx512);
            pHarqTmp = pHarqTmp + 64;
        }

        for(i=0; i<ncbStep_16; i+=16)
        {
            _mm_storeu_si128 ((__m128i *) pHarqTmp, ymm0_sse128);
            pHarqTmp = pHarqTmp + 16;
        }

        for(i=0; i<ncbStep_1; i++)
        {
            *pHarqTmp++ = 0;
        }
    }


    if(NcbSubK0Start >= ESubK0Start)//e-K0<ncb-K0
    {
        /* copy e-K0 size data into harq buffer */
        pInTmp = pIn;
        pHarqTmp = pHarq+k0withoutnull;
        wrapAdd2HarqBuffer_avx512(pInTmp, pHarqTmp,EWithoutNull);
    }
    else/* e-K0>ncb-K0 */
    {
        /* copy ncb-K0 size data into harq buffer */
        pInTmp = pIn;
        pHarqTmp = pHarq+k0withoutnull;
        wrapAdd2HarqBuffer_avx512(pInTmp, pHarqTmp,NcbSubK0Start);
        /* copy ncb-K0 size data into harq buffer */
        wrapLen = EWithoutNull - NcbSubK0Start;
        pInTmp = pIn + NcbSubK0Start;
        pHarqTmp = pHarq;//move harq buffer addr to the start place
        wrapAdd2HarqBuffer_avx512(pInTmp, pHarqTmp, wrapLen);
    }

    return 0;
}

int32_t bblib_rate_match_ul_avx512(const struct bblib_rate_match_ul_request *request,
    struct bblib_rate_match_ul_response *response)
{
    int32_t ret;
    struct bblib_harq_combine_ul_request harq_request;
    struct bblib_harq_combine_ul_response harq_response;
    struct bblib_deinterleave_ul_request deint_request;
    struct bblib_deinterleave_ul_response deint_response;
    struct bblib_turbo_adapter_ul_request adapter_request;
    struct bblib_turbo_adapter_ul_response adapter_response;

    harq_request.e = request->e;
    harq_request.k0withoutnull = request->k0withoutnull;
    harq_request.isretx = request->isretx;
    harq_request.ncb = request->ncb;
    harq_request.pdmout = request->pdmout;
    harq_response.pharqbuffer = response->pharqbuffer;

    /* ncb is length without padding, circular buffer does not contain padding
       dummy bits. */
    deint_request.ncb = request->ncb;
    deint_request.circ_buffer = BBLIB_CIRCULAR_BUFFER_WITHOUT_PADDING;
    deint_request.pharqbuffer = response->pharqbuffer;
    deint_response.pinteleavebuffer = response->pinteleavebuffer;

    adapter_request.ncb = request->ncb;
    adapter_request.isinverted = request->isinverted;
    adapter_request.pinteleavebuffer = response->pinteleavebuffer;
    adapter_response.pharqout = response->pharqout;

    ret = bblib_harq_combine_ul_avx512(&harq_request, &harq_response);
    if (ret != 0)
        return ret;
    ret = bblib_deinterleave_ul_avx512(&deint_request, &deint_response);
    if (ret != 0)
        return ret;
    ret = bblib_turbo_adapter_ul_avx512(&adapter_request, &adapter_response);
    if (ret != 0)
        return ret;

    return(0);
}
#else
int32_t bblib_rate_match_ul_avx512(const struct bblib_rate_match_ul_request *request,
    struct bblib_rate_match_ul_response *response)
{
    printf("bblib_rate_matching requires AVX512 ISA support to run\n");
    return(-1);
}
int32_t bblib_deinterleave_ul_avx512(const struct bblib_deinterleave_ul_request *request,
        struct bblib_deinterleave_ul_response *response)
{
    printf("bblib_rate_matching requires AVX512 ISA support to run\n");
    return(-1);
}
int32_t bblib_harq_combine_ul_avx512(const struct bblib_harq_combine_ul_request *request,
        struct bblib_harq_combine_ul_response *response)
{
    printf("bblib_rate_matching requires AVX512 ISA support to run\n");
    return(-1);
}
int32_t bblib_turbo_adapter_ul_avx512(const struct bblib_turbo_adapter_ul_request *request,
        struct bblib_turbo_adapter_ul_response *response)
{
    printf("bblib_rate_matching requires AVX512 ISA support to run\n");
    return(-1);
}
#endif
