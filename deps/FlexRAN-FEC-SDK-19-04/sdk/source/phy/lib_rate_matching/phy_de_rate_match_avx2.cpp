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
 * @file
 * @brief  Implementation of Derate matching, including HARQ combine, sub-block deinterleaver, and adapter to Turbo decoder.
*/

/*******************************************************************************
 * Include public/global header files
 *******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <immintrin.h> /* AVX */

#include "phy_rate_match.h"
#include "phy_rate_match_internal.h"


#if defined (_BBLIB_AVX2_) || defined (_BBLIB_AVX512_)
struct bblib_de_rate_match_init_avx2
{
    bblib_de_rate_match_init_avx2()
    {

#if !defined(_BBLIB_AVX2_) && !defined(_BBLIB_AVX512_)
    printf("__func__ rate_match cannot run with this CPU type, needs AVX2 or greater.\n");
    exit(-1);
#endif
    bblib_print_rate_match_version();

    }
};

bblib_de_rate_match_init_avx2 do_constructor_de_rate_matching_avx2;

void addByteWithNum(uint8_t **pIn, uint8_t *pOut, int32_t Len)
{
    int8_t value = 0;
    for(int32_t i=0; i<Len; i++)
    {
        value = *pOut + **pIn;
        if(value>MAX_VALUE)value=MAX_VALUE;
        if(value<MIN_VALUE)value=MIN_VALUE;
        *pOut = value;
        *pIn = *pIn + 1;
        pOut++;
    }
}
void wrapAdd2HarqBuffer(uint8_t **pIn, uint8_t *pOut, int32_t len)
{
    int32_t len0,len1;
    __m256i ymm0,ymm1;
    len0 = len&0xffffffe0;
    len1 = len&0x1f;
    __m256i VMAX = _mm256_set1_epi8(MAX_VALUE);
        __m256i VMIN = _mm256_set1_epi8(MIN_VALUE);
    for(int32_t i=0; i<len0; i=i+32)
    {
        ymm0 = _mm256_loadu_si256 ((__m256i const *) *pIn);
        ymm1 = _mm256_loadu_si256 ((__m256i const *) pOut);
        ymm0 = _mm256_adds_epi8 (ymm0, ymm1);
        ymm1 = _mm256_max_epi8(VMIN, ymm0);
        ymm0 = _mm256_min_epi8(VMAX, ymm1);
        *pIn = *pIn + 32;
        _mm256_storeu_si256 ((__m256i *) pOut, ymm0);
        pOut = pOut + 32;
    }
    addByteWithNum(pIn, pOut, len1);
}

/**
 * @brief This function implements HARQ combine
 * @param[in] pIn demodulation output, and input of HARQ
 * @param[out] pHarq output of HARQ combine, and input of sub-block deinterleaver
 * @param[in] k0withoutnull K0 without NULL based on RV, position of this input HARQ sequence in ring buffer
 * @param[in] ncb length of cyclic buffer
 * @param[in] EWithoutNull rate dematching input length in bytes
 * @param[in] isretx flag of retransmission, 0: no retransmission, 1: retransmission
 * @note pIn and pHarq need to be aligned with 256 bits
 * @return success: return 0, else: return -1
 */
int32_t bblib_harq_combine_ul_avx2(const struct bblib_harq_combine_ul_request *request,
        struct bblib_harq_combine_ul_response *response)
{
    uint8_t *pIn = request->pdmout;
    uint8_t *pHarq = response->pharqbuffer;
    int32_t k0withoutnull = request->k0withoutnull;
    int32_t ncb = request->ncb;
    int32_t EWithoutNull = request->e;
    int32_t isretx = request->isretx;
    int32_t NcbSubK0Start = 0;
    int32_t ESubK0Start = 0;
    int32_t wrapLen = 0;
    int32_t Ncb0 = 0, Ncb1=0;
    __m256i ymm0;
    uint8_t *pInTmp, *pHarqTmp;

    ESubK0Start = EWithoutNull;
    NcbSubK0Start = ncb - k0withoutnull;
    /* first transmition should fill zero to harq buffer */
    if(isretx == 0)
    {
        /* fill zero into harq buffer */
        Ncb0 = ncb&0xffffffe0;
        Ncb1 = ncb&0x1f;
        pHarqTmp = pHarq;
        ymm0 = _mm256_set1_epi8 (0);
        for(int32_t i=0; i<Ncb0; i=i+32)
        {
            _mm256_storeu_si256 ((__m256i *) pHarqTmp, ymm0);
            pHarqTmp = pHarqTmp + 32;
        }
        for(int32_t i=0; i<Ncb1; i=i+1)
        {
            *pHarqTmp++ = 0;
        }
    }

    if(NcbSubK0Start >= ESubK0Start)//e-K0<ncb-K0
    {
        /* copy e-K0 size data into harq buffer */
        pInTmp = pIn;
        pHarqTmp = pHarq+k0withoutnull;
        wrapAdd2HarqBuffer(&pInTmp, pHarqTmp,EWithoutNull);
    }
    else/* e-K0>ncb-K0 */
    {
        /* copy ncb-K0 size data into harq buffer */
        pInTmp = pIn;
        pHarqTmp = pHarq+k0withoutnull;
        wrapAdd2HarqBuffer(&pInTmp, pHarqTmp,NcbSubK0Start);
        /* copy ncb-K0 size data into harq buffer */
        wrapLen = EWithoutNull - NcbSubK0Start;
        pHarqTmp = pHarq;//move harq buffer addr to the start place
        wrapAdd2HarqBuffer(&pInTmp, pHarqTmp, wrapLen);
    }

    return 0;
}
#define UNPACKEPI8(in0,in1,out0,out1)\
{\
    out0 = _mm256_unpacklo_epi8 (in0, in1);\
    out1 = _mm256_unpackhi_epi8 (in0, in1);\
}
#define UNPACKEPI16(in0,in1,in2,in3,out0,out1,out2,out3)\
{\
    out0 = _mm256_unpacklo_epi16 (in0, in1);\
    out1 = _mm256_unpackhi_epi16 (in0, in1);\
    out2 = _mm256_unpacklo_epi16 (in2, in3);\
    out3 = _mm256_unpackhi_epi16 (in2, in3);\
}
#define UNPACKEPI32(in0,in1,in2,in3,out0,out1,out2,out3)\
{\
    out0 = _mm256_unpacklo_epi32 (in0, in1);\
    out1 = _mm256_unpackhi_epi32 (in0, in1);\
    out2 = _mm256_unpacklo_epi32 (in2, in3);\
    out3 = _mm256_unpackhi_epi32 (in2, in3);\
}
#define UNPACKEPI64(in0,in1,in2,in3,out0,out1,out2,out3)\
{\
    out0 = _mm256_unpacklo_epi64 (in0, in1);\
    out1 = _mm256_unpackhi_epi64 (in0, in1);\
    out2 = _mm256_unpacklo_epi64 (in2, in3);\
    out3 = _mm256_unpackhi_epi64 (in2, in3);\
}
#define PERMUTEANDSTORELOW128(in0,in1,pOut)\
{\
    _mm256_storeu_si256 ((__m256i *)pOut, _mm256_permute2x128_si256 (in0, in1, 0x20));\
    pOut = pOut + 32;\
}
#define PERMUTEANDSTOREHIGH128(in0,in1,pOut)\
{\
    _mm256_storeu_si256 ((__m256i *)pOut, _mm256_permute2x128_si256 (in0, in1, 0x31));\
    pOut = pOut + 32;\
}

template <circular_buffer_format CIRCULAR_BUFFER_FORMAT>
void setColumnInf(uint8_t *pHarq, uint8_t* (&pColInf)[32], int32_t Rsubblock, int32_t Nd, int32_t flag);

template<>
inline void setColumnInf<BBLIB_CIRCULAR_BUFFER_WITHOUT_PADDING>(uint8_t *pHarq, uint8_t* (&pColInf)[32],
                                                                int32_t Rsubblock, int32_t Nd, int32_t flag)
{
    constexpr int8_t fillNullInf[4][32] = {{-1,0,0,0,0,0,0,0,-1,0,0,0,0,0,0,0,-1,0,0,0,0,0,0,0,-1,0,0,0,0,0,0,0},
                            {-1,0,-1,0,-1,0,0,0,-1,0,-1,0,-1,0,0,0,-1,0,-1,0,-1,0,0,0,-1,0,-1,0,-1,0,0,0,},
                            {-1,-1,-1,0,-1,0,-1,0,-1,-1,-1,0,-1,0,-1,0,-1,-1,-1,0,-1,0,-1,0,-1,-1,-1,0,-1,0,-1,0},
                            {-1,-1,-1,-1,-1,-1,-1,0,-1,-1,-1,-1,-1,-1,-1,0,-1,-1,-1,-1,-1,-1,-1,0,-1,-1,-1,-1,-1,-1,-1,0}};
    constexpr int8_t fillNullPar1[4][32] = {{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0}};
    constexpr int32_t fillNullNum[4] = {4,12,20,28};

    int32_t fillNullInfType = 0;

    uint8_t *pHarqTmp = pHarq;

    /* find the NULL fill type */
#pragma unroll (4)
    for(int32_t i = 0; i < 4; i++)
    {
        if(Nd==fillNullNum[i])
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
                                                     int32_t Rsubblock, int32_t Nd, int32_t flag)
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
inline void harqDeInteleaveBlock(uint8_t *pHarq, uint8_t *pDeInteleave0, uint8_t *pDeInteleave1,
                                 int32_t Rsubblock, int32_t Nd, int32_t flag)
{
    int32_t k = 0;
    uint8_t *pColInf[32] {};
    uint8_t *pDeInteleaveTmp0, *pDeInteleaveTmp1;

    __m256i ymm[32];
    __m256i ymmTemp[32];
    __m256i ymm0,ymm1,ymm2,ymm3;

    pDeInteleaveTmp0 = pDeInteleave0;
    pDeInteleaveTmp1 = pDeInteleave1;

    /* if fist block Rsubblock don't change, if not first block Rsubblock will be 2*Rsubblock */
    Rsubblock = Rsubblock + Rsubblock * flag;

    /* get the info bit addr for each col */
    setColumnInf<CIRCULAR_BUFFER_FORMAT>(pHarq, pColInf, Rsubblock, Nd, flag);

    for(int32_t i = 0; i < Rsubblock; i = i + 32)
    {
#pragma unroll (32)
        for(int32_t j = 0; j < 32; j++)
        {
            ymm[j] = _mm256_loadu_si256 ((__m256i const *)pColInf[j]);
            pColInf[j] = pColInf[j] + 32;
        }

        /* 0:1 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(ymm[0], ymm[16],ymm0,ymm1);
        /* 2:3 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(ymm[8], ymm[24],ymm[0],ymm[16]);
        /* 4:5 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(ymm[4], ymm[20],ymm[8],ymm[24]);
        /* 6:7 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(ymm[12], ymm[28],ymm[4],ymm[20]);
        /* 8:9 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(ymm[2], ymm[18],ymm[12],ymm[28]);
        /* 10:11 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(ymm[10], ymm[26],ymm[2],ymm[18]);
        /* 12:13 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(ymm[6], ymm[22],ymm[10],ymm[26]);
        /* 14:15 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(ymm[14], ymm[30],ymm[6],ymm[22]);
        /* 16:17 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(ymm[1], ymm[17],ymm[14],ymm[30]);
        /* 18:19 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(ymm[9], ymm[25],ymm[1],ymm[17]);
        /* 20:21 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(ymm[5], ymm[21],ymm[9],ymm[25]);
        /* 22:23 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(ymm[13], ymm[29],ymm[5],ymm[21]);
        /* 24:25 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(ymm[3], ymm[19],ymm[13],ymm[29]);
        /* 26:27 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(ymm[11], ymm[27],ymm[3],ymm[19]);
        /* 28:29 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(ymm[7], ymm[23],ymm[11],ymm[27]);
        /* 30:31 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(ymm[15], ymm[31],ymm[7],ymm[23]);

        /* 0:3   s0-s3s16-s19 s4-s7s20-s23 s8-s11s24-s27 s12-s15s28-s31 */
        UNPACKEPI16(ymm0,    ymm[0],ymm1,  ymm[16], ymm[15],ymm[31],ymm2,   ymm3);
        /* 4:7   s0-s3s16-s19 s4-s7s20-s23 s8-s11s24-s27 s12-s15s28-s31 */
        UNPACKEPI16(ymm[8],  ymm[4],ymm[24],ymm[20],ymm0,   ymm[0], ymm1,   ymm[16]);
        /* 8:11  s0-s3s16-s19 s4-s7s20-s23 s8-s11s24-s27 s12-s15s28-s31 */
        UNPACKEPI16(ymm[12], ymm[2],ymm[28],ymm[18],ymm[8], ymm[4], ymm[24],ymm[20]);
        /* 12:15 s0-s3s16-s19 s4-s7s20-s23 s8-s11s24-s27 s12-s15s28-s31 */
        UNPACKEPI16(ymm[10], ymm[6],ymm[26],ymm[22],ymm[12], ymm[2],ymm[28],ymm[18]);
        /* 16:19 s0-s3s16-s19 s4-s7s20-s23 s8-s11s24-s27 s12-s15s28-s31 */
        UNPACKEPI16(ymm[14], ymm[1],ymm[30],ymm[17],ymm[10], ymm[6],ymm[26],ymm[22]);
        /* 20:23 s0-s3s16-s19 s4-s7s20-s23 s8-s11s24-s27 s12-s15s28-s31 */
        UNPACKEPI16(ymm[9],  ymm[5],ymm[25],ymm[21],ymm[14], ymm[1],ymm[30],ymm[17]);
        /* 24:27 s0-s3s16-s19 s4-s7s20-s23 s8-s11s24-s27 s12-s15s28-s31 */
        UNPACKEPI16(ymm[13], ymm[3],ymm[29],ymm[19],ymm[9],  ymm[5],ymm[25],ymm[21]);
        /* 28-31 s0-s3s16-s19 s4-s7s20-s23 s8-s11s24-s27 s12-s15s28-s31 */
        UNPACKEPI16(ymm[11], ymm[7],ymm[27],ymm[23],ymm[13], ymm[3],ymm[29],ymm[19]);

        /* 0:7   s0s1s16s17 s2s3s18s19 s4s5s20s21 s6s7s22s23 */
        UNPACKEPI32(ymm[15], ymm0,    ymm[31], ymm[0],  ymm[11], ymm[7],ymm[27], ymm[23]);
        /* s8s9s24s25 s10s11s26s27 s12s13s28s29 s14s15s30s31 */
        UNPACKEPI32(ymm2,    ymm1,    ymm3,    ymm[16], ymm[15], ymm0,  ymm[31], ymm[0]);
        /* 8:15   s0s1s16s17 s2s3s18s19 s4s5s20s21 s6s7s22s23 */
        UNPACKEPI32(ymm[8],  ymm[12], ymm[4], ymm[2],   ymm2,    ymm1,  ymm3,    ymm[16]);
        /* s8s9s24s25 s10s11s26s27 s12s13s28s29 s14s15s30s31 */
        UNPACKEPI32(ymm[24], ymm[28], ymm[20], ymm[18], ymm[8],  ymm[12], ymm[4], ymm[2]);
        /* 16:23   s0s1s16s17 s2s3s18s19 s4s5s20s21 s6s7s22s23 */
        UNPACKEPI32(ymm[10], ymm[14], ymm[6], ymm[1],   ymm[24], ymm[28], ymm[20], ymm[18]);
        /* s8s9s24s25 s10s11s26s27 s12s13s28s29 s14s15s30s31 */
        UNPACKEPI32(ymm[26], ymm[30], ymm[22], ymm[17], ymm[10], ymm[14], ymm[6], ymm[1]);
        /* 24:31   s0s1s16s17 s2s3s18s19 s4s5s20s21 s6s7s22s23 */
        UNPACKEPI32(ymm[9], ymm[13], ymm[5], ymm[3],    ymm[26], ymm[30], ymm[22], ymm[17]);
        /* s8s9s24s25 s10s11s26s27 s12s13s28s29 s14s15s30s31 */
        UNPACKEPI32(ymm[25], ymm[29], ymm[21], ymm[19], ymm[9], ymm[13], ymm[5], ymm[3]);

        /* 0:15   s0s16 s1s17 s2s18 s3s19 */
        UNPACKEPI64( ymm[11], ymm2, ymm[7], ymm1,        ymmTemp[0], ymmTemp[2], ymmTemp[4], ymmTemp[6]);
        /* 0:15   s4s20 s5s21 s6s22 s7s23 */
        UNPACKEPI64( ymm[27], ymm3, ymm[23], ymm[16],    ymmTemp[8], ymmTemp[10], ymmTemp[12], ymmTemp[14]);
        /* 0:15   s8s24 s9s25 s10s26 s11s27 */
        UNPACKEPI64( ymm[15], ymm[8], ymm0, ymm[12],    ymmTemp[16], ymmTemp[18], ymmTemp[20], ymmTemp[22]);
        /* 0:15   s12s28 s13s29 s14s30 s15s31 */
        UNPACKEPI64( ymm[31], ymm[4], ymm[0], ymm[2],    ymmTemp[24], ymmTemp[26], ymmTemp[28], ymmTemp[30]);
        /* 16:31   s0s16 s1s17 s2s18 s3s19 */
        UNPACKEPI64( ymm[24], ymm[26], ymm[28], ymm[30],  ymmTemp[1], ymmTemp[3], ymmTemp[5], ymmTemp[7]);
        /* 16:31   s4s20 s5s21 s6s22 s7s23 */
        UNPACKEPI64( ymm[20], ymm[22], ymm[18], ymm[17],     ymmTemp[9], ymmTemp[11], ymmTemp[13], ymmTemp[15]);
        /* 16:31   s8s24 s9s25 s10s26 s11s27 */
        UNPACKEPI64( ymm[10], ymm[9], ymm[14], ymm[13], ymmTemp[17], ymmTemp[19], ymmTemp[21], ymmTemp[23]);
        /* 16:31   s12s28 s13s29 s14s30 s15s31 */
        UNPACKEPI64( ymm[6], ymm[5], ymm[1], ymm[3], ymmTemp[25], ymmTemp[27], ymmTemp[29], ymmTemp[31]);

        for(int32_t loop1=0; loop1 < 32 && k < Rsubblock; loop1 += 4, k += 2)
        {
            /* flag 0:store block0. flag 1:store block1. */
            PERMUTEANDSTORELOW128(ymmTemp[loop1], ymmTemp[loop1+1],pDeInteleaveTmp0);/* sx */
            /* flag 0:store block0. flag 1:store block2. */
            if(flag==0){PERMUTEANDSTORELOW128(ymmTemp[loop1+2], ymmTemp[loop1+3],pDeInteleaveTmp0);}/* sx */
            else{PERMUTEANDSTORELOW128(ymmTemp[loop1+2], ymmTemp[loop1+3],pDeInteleaveTmp1);}/* sx */
        }
        for(int32_t loop1 = 0; loop1 < 32 && k < Rsubblock; loop1 += 4,k += 2)
        {
            PERMUTEANDSTOREHIGH128(ymmTemp[loop1], ymmTemp[loop1+1],pDeInteleaveTmp0);/* sx */

            if(flag==0){PERMUTEANDSTOREHIGH128(ymmTemp[loop1+2], ymmTemp[loop1+3],pDeInteleaveTmp0);}/* sx */
            else{PERMUTEANDSTOREHIGH128(ymmTemp[loop1+2], ymmTemp[loop1+3],pDeInteleaveTmp1);}/* sx */
        }
    }
}

/**
 * @brief This function implements deinterlaver of 3 sub-blocks
 * @param[in] pHarq output of HARQ combine, and input of sub-block deinterleaver
 * @param[out] pDeInteleave output of deinterleaver, and input of turbo decode adapter
 * @param[in] ncb length of cyclic buffer
 * @note pHarq and pDeInteleave need to be aligned with 256 bits
 * @return success: return 0, else: return -1
 */
int32_t bblib_deinterleave_ul_avx2(const struct bblib_deinterleave_ul_request *request,
        struct bblib_deinterleave_ul_response *response)
{
    int32_t subBlockWithNull = 0, Nd = 0; /* Nd - numer of dummy */
    uint8_t *pHarq = request->pharqbuffer;
    uint8_t *pDeInteleave = response->pinteleavebuffer;
    int32_t ncb = request->ncb;

    int32_t D = ncb / 3;
    int32_t Rsubblock = ceil((float)D / (float)32);

    switch (request->circ_buffer)
    {
        case BBLIB_CIRCULAR_BUFFER_WITHOUT_PADDING:

            subBlockWithNull = Rsubblock * 32;
            Nd = subBlockWithNull - D;
            /* deIntealve for block0 */
            harqDeInteleaveBlock<BBLIB_CIRCULAR_BUFFER_WITHOUT_PADDING>(pHarq, pDeInteleave, 0, Rsubblock, Nd, 0);
            /* deIndtealve for block1 block2 */
            harqDeInteleaveBlock<BBLIB_CIRCULAR_BUFFER_WITHOUT_PADDING>(pHarq+D, pDeInteleave+subBlockWithNull+64, pDeInteleave+(subBlockWithNull+64)*2, Rsubblock, Nd, 1);
            break;
        case BBLIB_FULL_CIRCULAR_BUFFER:
            /* ncb includes dummy bits in the full circular buffer case
               and nNull is not used, no need to calculate it. */
            subBlockWithNull = D;
            /* deIntealve for block0 */
            harqDeInteleaveBlock<BBLIB_FULL_CIRCULAR_BUFFER>(pHarq, pDeInteleave, 0, Rsubblock, 0, 0);
            /* deIndtealve for block1 block2 */
            harqDeInteleaveBlock<BBLIB_FULL_CIRCULAR_BUFFER>(pHarq+D, pDeInteleave+subBlockWithNull+64, pDeInteleave+(subBlockWithNull+64)*2, Rsubblock, 0, 1);
    }

    return 0;
}

void turboAdapterK16(uint8_t *pDeInteleave, uint8_t *pTurbo, int32_t ncb, int32_t isinverted)
{
    int32_t D = 0, KBlock = 0, Rsubblock = 0, Nd = 0, KBlockDiv16 = 0;
    int32_t k = 0;
    uint8_t *paD0[16],*paD1[16],*paD2[16];
    uint8_t *pD0,*pD1,*pD2, *pOut;

    __m256i yamm0[18],yamm1[18];
    __m256i ymmTemp0[16],ymmTemp1[16];
    __m256i ymm0,ymm1,ymm2,ymm3;
    const __m256i yZero256 = _mm256_setzero_si256();
    const __m128i yZero128 = _mm_setzero_si128();
    /* calculate parameter */
    D = ncb/3;
    KBlock = D - 4;
    KBlockDiv16 = KBlock/16;
    Rsubblock = ceil((float)D/(float)32);
    Nd = Rsubblock*32-D;
    /* inital address for data */
    pD0 = pDeInteleave + Nd;
    pD1 = pD0 + D + Nd + 64;
    pD2 = pD1 + D + Nd + 64 - 1;
    paD0[0] = pD0;
    paD1[0] = pD1;
    paD2[0] = pD2;
    for(int32_t i=1; i<16; i++)
    {
        paD0[i] = paD0[i-1] + KBlockDiv16;
        paD1[i] = paD1[i-1] + KBlockDiv16;
        paD2[i] = paD2[i-1] + KBlockDiv16;
    }

    /* process for first 12 byte */
    ymm0 = _mm256_loadu_si256 ((__m256i const *)(pD0+KBlock));
    ymm1 = _mm256_loadu_si256 ((__m256i const *)(pD1+KBlock));
    ymm2 = _mm256_loadu_si256 ((__m256i const *)(pD2+KBlock));
    ymm3 = _mm256_unpacklo_epi32 (ymm0, ymm1);
    ymm0 = _mm256_unpacklo_epi64 (ymm3, ymm2);
    if (!isinverted)
    {
        /* Turbo decoder expects positive soft decisions for '1' bit. */
        ymm0 = _mm256_subs_epi8(yZero256, ymm0);
    }
    pOut = pTurbo;
    _mm256_storeu_si256 ((__m256i *)pOut, ymm0);
    pOut = pOut + 12;
    _mm256_storeu_si256 ((__m256i *)pOut, yZero256);
    pOut = pOut + 32;
    _mm256_storeu_si256 ((__m256i *)pOut, yZero256);
    pOut = pOut + 4;
    /*D0,D1*/
    for(int32_t i=0; i<KBlockDiv16; i=i+32)
    {
        for(int32_t j=0; j<16; j++)
        {
            yamm0[j] = _mm256_loadu_si256 ((__m256i const *)paD0[j]);
            paD0[j] = paD0[j] + 32;
            yamm1[j] = _mm256_loadu_si256 ((__m256i const *)paD1[j]);
            paD1[j] = paD1[j] + 32;
        }
        /*D0*/
        /* 0:1 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(yamm0[0], yamm0[1],ymm0,ymm1);
        /* 2:3 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(yamm0[2], yamm0[3],yamm0[0], yamm0[1]);
        /* 4:5 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(yamm0[4], yamm0[5],yamm0[2], yamm0[3]);
        /* 6:7 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(yamm0[6], yamm0[7],yamm0[4], yamm0[5]);
        /* 8:9 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(yamm0[8], yamm0[9],yamm0[6], yamm0[7]);
        /* 10:11 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(yamm0[10], yamm0[11],yamm0[8], yamm0[9]);
        /* 12:13 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(yamm0[12], yamm0[13],yamm0[10], yamm0[11]);
        /* 14:15 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(yamm0[14], yamm0[15],yamm0[12], yamm0[13]);
        /*D1*/
        /* 0:1 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(yamm1[0], yamm1[1],ymm2,ymm3);
        /* 2:3 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(yamm1[2], yamm1[3],yamm1[0], yamm1[1]);
        /* 4:5 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(yamm1[4], yamm1[5],yamm1[2], yamm1[3]);
        /* 6:7 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(yamm1[6], yamm1[7],yamm1[4], yamm1[5]);
        /* 8:9 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(yamm1[8], yamm1[9],yamm1[6], yamm1[7]);
        /* 10:11 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(yamm1[10], yamm1[11],yamm1[8], yamm1[9]);
        /* 12:13 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(yamm1[12], yamm1[13],yamm1[10], yamm1[11]);
        /* 14:15 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(yamm1[14], yamm1[15],yamm1[12], yamm1[13]);
        /*D0*/
        /* 0:3   s0-s3s16-s19 s4-s7s20-s23 s8-s11s24-s27 s12-s15s28-s31 */
        UNPACKEPI16(ymm0,    yamm0[0],ymm1,  yamm0[1], yamm0[14],yamm0[15],yamm0[16],yamm0[17]);
        /* 4:7   s0-s3s16-s19 s4-s7s20-s23 s8-s11s24-s27 s12-s15s28-s31 */
        UNPACKEPI16(yamm0[2],  yamm0[4],yamm0[3],yamm0[5],ymm0,   ymm1, yamm0[0],   yamm0[1]);
        /* 8:11  s0-s3s16-s19 s4-s7s20-s23 s8-s11s24-s27 s12-s15s28-s31 */
        UNPACKEPI16(yamm0[6],  yamm0[8],yamm0[7],yamm0[9], yamm0[2],  yamm0[3],yamm0[4],yamm0[5]);
        /* 12:15 s0-s3s16-s19 s4-s7s20-s23 s8-s11s24-s27 s12-s15s28-s31 */
        UNPACKEPI16(yamm0[10],  yamm0[12],yamm0[11],yamm0[13],yamm0[6],  yamm0[7],yamm0[8],yamm0[9]);
        /*D2*/
        /* 0:3   s0-s3s16-s19 s4-s7s20-s23 s8-s11s24-s27 s12-s15s28-s31 */
        UNPACKEPI16(ymm2,    yamm1[0],ymm3,  yamm1[1], yamm1[14],yamm1[15],yamm1[16],yamm1[17]);
        /* 4:7   s0-s3s16-s19 s4-s7s20-s23 s8-s11s24-s27 s12-s15s28-s31 */
        UNPACKEPI16(yamm1[2],  yamm1[4],yamm1[3],yamm1[5],ymm2,   ymm3, yamm1[0],   yamm1[1]);
        /* 8:11  s0-s3s16-s19 s4-s7s20-s23 s8-s11s24-s27 s12-s15s28-s31 */
        UNPACKEPI16(yamm1[6],  yamm1[8],yamm1[7],yamm1[9], yamm1[2],  yamm1[3],yamm1[4],yamm1[5]);
        /* 12:15 s0-s3s16-s19 s4-s7s20-s23 s8-s11s24-s27 s12-s15s28-s31 */
        UNPACKEPI16(yamm1[10],  yamm1[12],yamm1[11],yamm1[13],yamm1[6],  yamm1[7],yamm1[8],yamm1[9]);
        /*D0*/
        /* 0:7   s0s1s16s17 s2s3s18s19 s4s5s20s21 s6s7s22s23 */
        UNPACKEPI32(yamm0[14], ymm0,    yamm0[15], ymm1,  yamm0[10],  yamm0[11],yamm0[12],yamm0[13]);
        /* s8s9s24s25 s10s11s26s27 s12s13s28s29 s14s15s30s31 */
        UNPACKEPI32(yamm0[16],    yamm0[0],    yamm0[17],    yamm0[1], ymm0, ymm1,  yamm0[14],  yamm0[15]);
        /* 8:15   s0s1s16s17 s2s3s18s19 s4s5s20s21 s6s7s22s23 */
        UNPACKEPI32(yamm0[2],  yamm0[6], yamm0[3], yamm0[7],   yamm0[0],    yamm0[1],  yamm0[16],    yamm0[17]);
        /* s8s9s24s25 s10s11s26s27 s12s13s28s29 s14s15s30s31 */
        UNPACKEPI32(yamm0[4], yamm0[8], yamm0[5], yamm0[9], yamm0[2],  yamm0[3], yamm0[6], yamm0[7]);
        /*D1*/
        /* 0:7   s0s1s16s17 s2s3s18s19 s4s5s20s21 s6s7s22s23 */
        UNPACKEPI32(yamm1[14], ymm2,    yamm1[15], ymm3,  yamm1[10],  yamm1[11],yamm1[12],yamm1[13]);
        /* s8s9s24s25 s10s11s26s27 s12s13s28s29 s14s15s30s31 */
        UNPACKEPI32(yamm1[16],    yamm1[0],    yamm1[17],    yamm1[1], ymm2, ymm3,  yamm1[14],  yamm1[15]);
        /* 8:15   s0s1s16s17 s2s3s18s19 s4s5s20s21 s6s7s22s23 */
        UNPACKEPI32(yamm1[2],  yamm1[6], yamm1[3], yamm1[7],   yamm1[0],    yamm1[1],  yamm1[16],    yamm1[17]);
        /* s8s9s24s25 s10s11s26s27 s12s13s28s29 s14s15s30s31 */
        UNPACKEPI32(yamm1[4], yamm1[8], yamm1[5], yamm1[9], yamm1[2],  yamm1[3], yamm1[6], yamm1[7]);
        /*D0*/
        /* 0:15   s0s16 s1s17 s2s18 s3s19 */
        UNPACKEPI64( yamm0[10], yamm0[0], yamm0[11], yamm0[1],        ymmTemp0[0], ymmTemp0[1], ymmTemp0[2], ymmTemp0[3]);
        /* 0:15   s4s20 s5s21 s6s22 s7s23 */
        UNPACKEPI64( yamm0[12], yamm0[16], yamm0[13],yamm0[17],    ymmTemp0[4], ymmTemp0[5], ymmTemp0[6], ymmTemp0[7]);
        /* 0:15   s8s24 s9s25 s10s26 s11s27 */
        UNPACKEPI64( ymm0, yamm0[2], ymm1, yamm0[3],    ymmTemp0[8], ymmTemp0[9], ymmTemp0[10], ymmTemp0[11]);
        /* 0:15   s12s28 s13s29 s14s30 s15s31 */
        UNPACKEPI64( yamm0[14], yamm0[6], yamm0[15], yamm0[7],    ymmTemp0[12], ymmTemp0[13], ymmTemp0[14], ymmTemp0[15]);
        /*D1*/
        /* 0:15   s0s16 s1s17 s2s18 s3s19 */
        UNPACKEPI64( yamm1[10], yamm1[0], yamm1[11], yamm1[1],        ymmTemp1[0], ymmTemp1[1], ymmTemp1[2], ymmTemp1[3]);
        /* 0:15   s4s20 s5s21 s6s22 s7s23 */
        UNPACKEPI64( yamm1[12], yamm1[16], yamm1[13],yamm1[17],    ymmTemp1[4], ymmTemp1[5], ymmTemp1[6], ymmTemp1[7]);
        /* 0:15   s8s24 s9s25 s10s26 s11s27 */
        UNPACKEPI64( ymm2, yamm1[2], ymm3, yamm1[3],    ymmTemp1[8], ymmTemp1[9], ymmTemp1[10], ymmTemp1[11]);
        /* 0:15   s12s28 s13s29 s14s30 s15s31 */
        UNPACKEPI64( yamm1[14], yamm1[6], yamm1[15], yamm1[7],    ymmTemp1[12], ymmTemp1[13], ymmTemp1[14], ymmTemp1[15]);

        if (!isinverted)
        {
            int32_t k_temp = k;
            for(int32_t loop1=0;loop1<16&&k_temp<KBlockDiv16;loop1+=1,k_temp+=1)
            {
                ymmTemp0[loop1] = _mm256_subs_epi8(yZero256, ymmTemp0[loop1]);
                ymmTemp1[loop1] = _mm256_subs_epi8(yZero256, ymmTemp1[loop1]);
            }
        }
        for(int32_t loop1=0;loop1<16&&k<KBlockDiv16;loop1+=1,k+=1)
        {
            _mm_storeu_si128 ((__m128i *)pOut, yZero128);
            pOut = pOut + 16;
            PERMUTEANDSTORELOW128(ymmTemp0[loop1], ymmTemp1[loop1],pOut);//sx
        }
        for(int32_t loop1=0;loop1<16&&k<KBlockDiv16;loop1+=1,k+=1)
        {
            _mm_storeu_si128 ((__m128i *)pOut, yZero128);
            pOut = pOut + 16;
            PERMUTEANDSTOREHIGH128(ymmTemp0[loop1], ymmTemp1[loop1],pOut);//sx
        }
    }

    /*D2*/
    k = 0;
    for(int32_t i=0; i<KBlockDiv16; i=i+32)
    {
        for(int32_t j=0; j<16; j++)
        {
            yamm0[j] = _mm256_loadu_si256 ((__m256i const *)paD2[j]);
            paD2[j] = paD2[j] + 32;
        }
        /*D0*/
        /* 0:1 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(yamm0[0], yamm0[1],ymm0,ymm1);
        /* 2:3 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(yamm0[2], yamm0[3],yamm0[0], yamm0[1]);
        /* 4:5 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(yamm0[4], yamm0[5],yamm0[2], yamm0[3]);
        /* 6:7 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(yamm0[6], yamm0[7],yamm0[4], yamm0[5]);
        /* 8:9 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(yamm0[8], yamm0[9],yamm0[6], yamm0[7]);
        /* 10:11 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(yamm0[10], yamm0[11],yamm0[8], yamm0[9]);
        /* 12:13 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(yamm0[12], yamm0[13],yamm0[10], yamm0[11]);
        /* 14:15 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(yamm0[14], yamm0[15],yamm0[12], yamm0[13]);
        /*D0*/
        /* 0:3   s0-s3s16-s19 s4-s7s20-s23 s8-s11s24-s27 s12-s15s28-s31 */
        UNPACKEPI16(ymm0,    yamm0[0],ymm1,  yamm0[1], yamm0[14],yamm0[15],yamm0[16],yamm0[17]);
        /* 4:7   s0-s3s16-s19 s4-s7s20-s23 s8-s11s24-s27 s12-s15s28-s31 */
        UNPACKEPI16(yamm0[2],  yamm0[4],yamm0[3],yamm0[5],ymm0,   ymm1, yamm0[0],   yamm0[1]);
        /* 8:11  s0-s3s16-s19 s4-s7s20-s23 s8-s11s24-s27 s12-s15s28-s31 */
        UNPACKEPI16(yamm0[6],  yamm0[8],yamm0[7],yamm0[9], yamm0[2],  yamm0[3],yamm0[4],yamm0[5]);
        /* 12:15 s0-s3s16-s19 s4-s7s20-s23 s8-s11s24-s27 s12-s15s28-s31 */
        UNPACKEPI16(yamm0[10],  yamm0[12],yamm0[11],yamm0[13],yamm0[6],  yamm0[7],yamm0[8],yamm0[9]);
        /*D0*/
        /* 0:7   s0s1s16s17 s2s3s18s19 s4s5s20s21 s6s7s22s23 */
        UNPACKEPI32(yamm0[14], ymm0,    yamm0[15], ymm1,  yamm0[10],  yamm0[11],yamm0[12],yamm0[13]);
        /* s8s9s24s25 s10s11s26s27 s12s13s28s29 s14s15s30s31 */
        UNPACKEPI32(yamm0[16],    yamm0[0],    yamm0[17],    yamm0[1], ymm0, ymm1,  yamm0[14],  yamm0[15]);
        /* 8:15   s0s1s16s17 s2s3s18s19 s4s5s20s21 s6s7s22s23 */
        UNPACKEPI32(yamm0[2],  yamm0[6], yamm0[3], yamm0[7],   yamm0[0],    yamm0[1],  yamm0[16],    yamm0[17]);
        /* s8s9s24s25 s10s11s26s27 s12s13s28s29 s14s15s30s31 */
        UNPACKEPI32(yamm0[4], yamm0[8], yamm0[5], yamm0[9], yamm0[2],  yamm0[3], yamm0[6], yamm0[7]);
        /*D0*/
        /* 0:15   s0s16 s1s17 s2s18 s3s19 */
        UNPACKEPI64( yamm0[10], yamm0[0], yamm0[11], yamm0[1],        ymmTemp0[0], ymmTemp0[1], ymmTemp0[2], ymmTemp0[3]);
        /* 0:15   s4s20 s5s21 s6s22 s7s23 */
        UNPACKEPI64( yamm0[12], yamm0[16], yamm0[13],yamm0[17],    ymmTemp0[4], ymmTemp0[5], ymmTemp0[6], ymmTemp0[7]);
        /* 0:15   s8s24 s9s25 s10s26 s11s27 */
        UNPACKEPI64( ymm0, yamm0[2], ymm1, yamm0[3],    ymmTemp0[8], ymmTemp0[9], ymmTemp0[10], ymmTemp0[11]);
        /* 0:15   s12s28 s13s29 s14s30 s15s31 */
        UNPACKEPI64( yamm0[14], yamm0[6], yamm0[15], yamm0[7],    ymmTemp0[12], ymmTemp0[13], ymmTemp0[14], ymmTemp0[15]);

        if (!isinverted)
        {
            int32_t k_temp = k;
            for(int32_t loop1=0;loop1<16&&k_temp<KBlockDiv16;loop1+=1,k_temp+=1)
            {
                ymmTemp0[loop1] = _mm256_subs_epi8(yZero256, ymmTemp0[loop1]);
            }
        }
        for(int32_t loop1=0;loop1<16&&k<KBlockDiv16;loop1+=1,k+=1)
        {
            _mm256_storeu_si256 ((__m256i *)pOut, yZero256);
            pOut = pOut + 32;
            _mm_storeu_si128 ((__m128i*) pOut, _mm256_extracti128_si256 (ymmTemp0[loop1], 0));
            pOut = pOut + 16;
        }
        for(int32_t loop1=0;loop1<16&&k<KBlockDiv16;loop1+=1,k+=1)
        {
            _mm256_storeu_si256 ((__m256i *)pOut, yZero256);
            pOut = pOut + 32;
            _mm_storeu_si128 ((__m128i*) pOut, _mm256_extracti128_si256 (ymmTemp0[loop1], 1));
            pOut = pOut + 16;
        }
    }

}

void turboAdapterK8(uint8_t *pDeInteleave, uint8_t *pTurbo, int32_t ncb, int32_t isinverted)
{
    int32_t D = 0, KBlock = 0, Rsubblock = 0, Nd = 0, KBlockDiv8 = 0;
    uint8_t *pD0,*pD1,*pD2, *pOut;

    __m256i ymm0,ymm1,ymm2,ymm3;
    const __m256i yZero = _mm256_setzero_si256();
    /* calculate parameter */
    D = ncb/3;
    KBlock = D - 4;
    KBlockDiv8 = KBlock/8;
    Rsubblock = ceil((float)D/(float)32);
    Nd = Rsubblock*32-D;
    /* inital address for data */
    pD0 = pDeInteleave + Nd;
    pD1 = pD0 + D + Nd + 64;
    pD2 = pD1 + D + Nd + 64 - 1;

    /* process for first 12 byte */
    ymm0 = _mm256_loadu_si256 ((__m256i const *)(pD0+KBlock));
    ymm1 = _mm256_loadu_si256 ((__m256i const *)(pD1+KBlock));
    ymm2 = _mm256_loadu_si256 ((__m256i const *)(pD2+KBlock));
    ymm3 = _mm256_unpacklo_epi32 (ymm0, ymm1);
    ymm0 = _mm256_unpacklo_epi64 (ymm3, ymm2);
    if (!isinverted)
    {
        /* Turbo decoder expects positive soft decisions for '1' bit. */
        ymm0 = _mm256_subs_epi8(yZero, ymm0);
    }
    pOut = pTurbo;
    _mm256_storeu_si256 ((__m256i *)pOut, ymm0);
    pOut = pOut + 12;
    _mm256_storeu_si256 ((__m256i *)pOut, yZero);
    pOut = pOut + 32;
    _mm256_storeu_si256 ((__m256i *)pOut, yZero);
    pOut = pOut + 4;
    /*D0,D1*/
    for(int32_t i=0; i<KBlock; i=i+32)
    {
        ymm0 = _mm256_loadu_si256 ((__m256i const *)pD0);
        pD0 = pD0 + 32;
        ymm1 = _mm256_loadu_si256 ((__m256i const *)pD1);
        pD1 = pD1 + 32;
        /* 0:1 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(ymm0,ymm1,ymm2,ymm3);
        ymm0 = _mm256_permute2x128_si256 (ymm2,ymm3, 0x20);
        ymm1 = _mm256_permute2x128_si256 (ymm2, ymm3,0x31);
        if (!isinverted)
        {
            ymm0 = _mm256_subs_epi8(yZero, ymm0);
            ymm1 = _mm256_subs_epi8(yZero, ymm1);
        }
        _mm256_storeu_si256 ((__m256i *)pOut, ymm0);
        pOut = pOut + 32;
        _mm256_storeu_si256 ((__m256i *)pOut, ymm1);
        pOut = pOut + 32;
    }
    pOut = pTurbo + 48 + KBlock*6;
    /*D2*/
    for(int32_t i=0; i<KBlock; i=i+32)
    {
        ymm0 = _mm256_loadu_si256 ((__m256i const *)pD2);
        pD2 = pD2 + 32;
        /* 0:1 s0-s7s16-s23 s8-s15s24-s31 */
        UNPACKEPI8(yZero,ymm0,ymm2,ymm3);
        ymm0 = _mm256_permute2x128_si256 (ymm2,ymm3, 0x20);
        ymm1 = _mm256_permute2x128_si256 (ymm2, ymm3,0x31);
        if (!isinverted)
        {
            ymm0 = _mm256_subs_epi8(yZero, ymm0);
            ymm1 = _mm256_subs_epi8(yZero, ymm1);
        }
        _mm256_storeu_si256 ((__m256i *)pOut, ymm0);
        pOut = pOut + 32;
        _mm256_storeu_si256 ((__m256i *)pOut, ymm1);
        pOut = pOut + 32;
    }
}

/**
 * @brief This function implements adapter for turbo decoder
 * @param[in] pDeInteleave output of deinterleaver, and input of turbo decode adapter
 * @param[out] pTurbo output of turbo adapter
 * @param[in] ncb length of cyclic buffer
 * @param[in] K rate matching transmission length
 * @note pDeInteleave and pTurbo need to be aligned with 256 bits
 * @return success: return 0, else: return -1
 */
int32_t bblib_turbo_adapter_ul_avx2(const struct bblib_turbo_adapter_ul_request *request,
        struct bblib_turbo_adapter_ul_response *response)
{
    uint8_t *pDeInteleave = request->pinteleavebuffer;
    uint8_t *pTurbo = response->pharqout;
    int32_t ncb = request->ncb;
    int32_t D = 0, KBlock = 0, Rsubblock  = 0, subBlockWithNull = 0;

    D = ncb/3;
    KBlock = D - 4;
    Rsubblock = ceil((float)D/(float)32);
    subBlockWithNull = Rsubblock * 32;
    if((KBlock%16) == 0)
    {
        turboAdapterK16(pDeInteleave, pTurbo, ncb, request->isinverted);
    }
    else
    {
        turboAdapterK8(pDeInteleave, pTurbo, ncb, request->isinverted);
    }

    return 0;
}

/** @brief Uplink rate matching for LTE
 * Includes HARQ combining, subblock deinterleaving and data formatting for turbo decoding
 * @param[in] request structure containing configuration information and input data
 * @param[out] response structure containing kernel outputs
 * @return success: return 0, else: return -1
 */
int32_t bblib_rate_match_ul_avx2(const struct bblib_rate_match_ul_request *request,
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

    deint_request.ncb = request->ncb;
    deint_request.circ_buffer = BBLIB_CIRCULAR_BUFFER_WITHOUT_PADDING;
    deint_request.pharqbuffer = response->pharqbuffer;
    deint_response.pinteleavebuffer = response->pinteleavebuffer;

    adapter_request.ncb = request->ncb;
    adapter_request.isinverted = request->isinverted;
    adapter_request.pinteleavebuffer = response->pinteleavebuffer;
    adapter_response.pharqout = response->pharqout;

    ret = bblib_harq_combine_ul_avx2(&harq_request, &harq_response);
    if (ret != 0)
        return ret;
    ret = bblib_deinterleave_ul_avx2(&deint_request, &deint_response);
    if (ret != 0)
        return ret;
    ret = bblib_turbo_adapter_ul_avx2(&adapter_request, &adapter_response);
    if (ret != 0)
        return ret;

    return(0);
}
#else
int32_t bblib_rate_match_ul_avx2(const struct bblib_rate_match_ul_request *request,
        struct bblib_rate_match_ul_response *response)
{
    printf("bblib_rate_matching requires AVX2 ISA support to run\n");
    return(-1);
}
int32_t bblib_deinterleave_ul_avx2(const struct bblib_deinterleave_ul_request *request,
        struct bblib_deinterleave_ul_response *response)
{
    printf("bblib_rate_matching requires AVX2 ISA support to run\n");
    return(-1);
}
int32_t bblib_harq_combine_ul_avx2(const struct bblib_harq_combine_ul_request *request,
        struct bblib_harq_combine_ul_response *response)
{
    printf("bblib_rate_matching requires AVX2 ISA support to run\n");
    return(-1);
}
int32_t bblib_turbo_adapter_ul_avx2(const struct bblib_turbo_adapter_ul_request *request,
        struct bblib_turbo_adapter_ul_response *response)
{
    printf("bblib_rate_matching requires AVX2 ISA support to run\n");
    return(-1);
}
#endif
