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

/**********************************************************************
*  @file
*  @brief  Implementation of fast LTE rate matching based on AVX2 instruction set, aimed to
*            use in LTE system. According to 3GPP TS 36.212 Rel 8, this file implementates from
*            bit sub-block interleaver, bit collection and bit selection.
*
*
*  @author Ruqiu Cao (ruqiu.cao@intel.com)
*
**********************************************************************/

/**********************************************************************
* Include public/global header files
**********************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <malloc.h>
#include <ipps.h>
#include <immintrin.h>  /* AVX */

#include "phy_rate_match.h"
#include "phy_rate_match_internal.h"
#if defined (_BBLIB_AVX2_) || defined (_BBLIB_AVX512_)

/*******************************************************************************
 * global macro and paramter declaration
 ******************************************************************************/

#define LEN (896)


/*******************************************************************************
 * Function declaration
 ******************************************************************************/
struct bblib_rate_match_init_avx2
{
    bblib_rate_match_init_avx2()
    {

#if !defined(_BBLIB_AVX2_) && !defined(_BBLIB_AVX512_)
        printf("__func__ rate_match cannot run with this CPU type, needs AVX2 or greater.\n");
        exit(-1);
#endif
        bblib_print_rate_match_version();

    }
};

bblib_rate_match_init_avx2 do_constructor_rate_matching_avx2;

int32_t bblib_rate_match_dl_avx2(const struct bblib_rate_match_dl_request *request,
        struct bblib_rate_match_dl_response *response)
{
    return rate_matching_turbo_lte_avx2(
            request->r,
            request->C,
            request->direction,
            request->Nsoft,
            request->KMIMO,
            request->MDL_HARQ,
            request->G,
            request->NL,
            request->Qm,
            request->rvidx,
            request->bypass_rvidx,
            request->Kidx,
            request->nLen,
            request->tin0,
            request->tin1,
            request->tin2,
            response->output,
            response->OutputLen);
}


//-------------------------------------------------------------------------------------------
/** @ingroup group_lte_source_phy_fec_enc
 *
 *  @param[in]   len - length of input buffer in bytes
 *  @param[in]   pIn - pointer to input buffer (systematic or partiy1 bit stream)
 *  @param[out]  pOut - pointer to output buffer
 *
 *  @return  no return value
 *
 *  @description
 *  bit reverse in internal byte, which is bit7 swap to bit0, bit6 swap to bit1,etc.
 *  3GPP TS 36.212 Rel 8 Sec.5.1.4.1.1, which code block size is 6144
 *
**/
//-------------------------------------------------------------------------------------------
static void bit_reverse(uint8_t *pIn, uint8_t *pOut ,int32_t len)
{
    int32_t i;
    __m256i vin, vout, vtmp;

    __m256i vmask01 = _mm256_set1_epi8 (0x01);
    __m256i vmask02 = _mm256_set1_epi8 (0x02);
    __m256i vmask04 = _mm256_set1_epi8 (0x04);
    __m256i vmask08 = _mm256_set1_epi8 (0x08);
    __m256i vmask10 = _mm256_set1_epi8 (0x10);
    __m256i vmask20 = _mm256_set1_epi8 (0x20);
    __m256i vmask40 = _mm256_set1_epi8 (0x40);
    __m256i vmask80 = _mm256_set1_epi8 (0x80);

    __m128i vin_128, vout_128, vtmp_128;
    __m128i vmask128_01 = _mm_set1_epi8 (0x01);
    __m128i vmask128_02 = _mm_set1_epi8 (0x02);
    __m128i vmask128_04 = _mm_set1_epi8 (0x04);
    __m128i vmask128_08 = _mm_set1_epi8 (0x08);
    __m128i vmask128_10 = _mm_set1_epi8 (0x10);
    __m128i vmask128_20 = _mm_set1_epi8 (0x20);
    __m128i vmask128_40 = _mm_set1_epi8 (0x40);
    __m128i vmask128_80 = _mm_set1_epi8 (0x80);


    int32_t times = len>>5;
    int32_t tlen = 0;

    for (i=0; i<times; i++)
    {
        vin = _mm256_lddqu_si256 ((__m256i const*) (pIn+tlen));
        vout = _mm256_setzero_si256 ();

        vtmp = _mm256_srli_epi64 (vin, 7);
        vtmp = _mm256_and_si256 (vtmp, vmask01);
        vout = _mm256_or_si256 (vout, vtmp);

        vtmp = _mm256_srli_epi64 (vin, 5);
        vtmp = _mm256_and_si256 (vtmp, vmask02);
        vout = _mm256_or_si256 (vout, vtmp);

        vtmp = _mm256_srli_epi64 (vin, 3);
        vtmp = _mm256_and_si256 (vtmp, vmask04);
        vout = _mm256_or_si256 (vout, vtmp);

        vtmp = _mm256_srli_epi64 (vin, 1);
        vtmp = _mm256_and_si256 (vtmp, vmask08);
        vout = _mm256_or_si256 (vout, vtmp);

        vtmp = _mm256_slli_epi64 (vin, 1);
        vtmp = _mm256_and_si256 (vtmp, vmask10);
        vout = _mm256_or_si256 (vout, vtmp);

        vtmp = _mm256_slli_epi64 (vin, 3);
        vtmp = _mm256_and_si256 (vtmp, vmask20);
        vout = _mm256_or_si256 (vout, vtmp);

        vtmp = _mm256_slli_epi64 (vin, 5);
        vtmp = _mm256_and_si256 (vtmp, vmask40);
        vout = _mm256_or_si256 (vout, vtmp);

        vtmp = _mm256_slli_epi64 (vin, 7);
        vtmp = _mm256_and_si256 (vtmp, vmask80);
        vout = _mm256_or_si256 (vout, vtmp);

        _mm256_storeu_si256 ((__m256i *) (pOut+tlen), vout);
        tlen = tlen + 32;
    }


    if((len-(times<<5) )> 16)
    {
        vin_128 = _mm_lddqu_si128 ((__m128i const*) (pIn+tlen));
        vout_128 = _mm_setzero_si128 ();

        vtmp_128 = _mm_srli_epi16 (vin_128, 7);
        vtmp_128 = _mm_and_si128 (vtmp_128, vmask128_01);
        vout_128 = _mm_or_si128 (vout_128, vtmp_128);

        vtmp_128 = _mm_srli_epi16 (vin_128, 5);
        vtmp_128 = _mm_and_si128 (vtmp_128, vmask128_02);
        vout_128 = _mm_or_si128 (vout_128, vtmp_128);

        vtmp_128 = _mm_srli_epi16 (vin_128, 3);
        vtmp_128 = _mm_and_si128 (vtmp_128, vmask128_04);
        vout_128 = _mm_or_si128 (vout_128, vtmp_128);

        vtmp_128 = _mm_srli_epi16 (vin_128, 1);
        vtmp_128 = _mm_and_si128 (vtmp_128, vmask128_08);
        vout_128 = _mm_or_si128 (vout_128, vtmp_128);

        vtmp_128 = _mm_slli_epi16 (vin_128, 1);
        vtmp_128 = _mm_and_si128 (vtmp_128, vmask128_10);
        vout_128 = _mm_or_si128 (vout_128, vtmp_128);

        vtmp_128 = _mm_slli_epi16 (vin_128, 3);
        vtmp_128 = _mm_and_si128 (vtmp_128, vmask128_20);
        vout_128 = _mm_or_si128 (vout_128, vtmp_128);

        vtmp_128 = _mm_slli_epi16 (vin_128, 5);
        vtmp_128 = _mm_and_si128 (vtmp_128, vmask128_40);
        vout_128 = _mm_or_si128 (vout_128, vtmp_128);

        vtmp_128 = _mm_slli_epi16 (vin_128, 7);
        vtmp_128 = _mm_and_si128 (vtmp_128, vmask128_80);
        vout_128 = _mm_or_si128 (vout_128, vtmp_128);

        _mm_storeu_si128 ((__m128i *) (pOut+tlen), vout_128);
        tlen = tlen + 16;

        for(i=((times<<5) + 16); i<len; i++)
        {
            pOut[i] = ((pIn[i] << 7) & 0x80) | ((pIn[i] << 5) & 0x40) | ((pIn[i] << 3) & 0x20) | ((pIn[i] << 1) & 0x10)
                    | ((pIn[i] >> 1) & 0x08) | ((pIn[i] >> 3) & 0x04) | ((pIn[i] >> 5) & 0x02) | ((pIn[i] >> 7) & 0x01);
        }
    }
    else
    {
        for(i=(times<<5); i<len; i++)
        {
            pOut[i] = ((pIn[i] << 7) & 0x80) | ((pIn[i] << 5) & 0x40) | ((pIn[i] << 3) & 0x20) | ((pIn[i] << 1) & 0x10)
                    | ((pIn[i] >> 1) & 0x08) | ((pIn[i] >> 3) & 0x04) | ((pIn[i] >> 5) & 0x02) | ((pIn[i] >> 7) & 0x01);
        }
    }

}



//-------------------------------------------------------------------------------------------
/** @ingroup group_lte_source_phy_fec_enc
 *
 *  @param[in]   nLen - length of input buffer
 *  @param[in]   nRound - number of round operation depend on code block size
 *  @param[in]   numOfNull - number of null bits for one code block
 *  @param[in]   pInput - pointer to input buffer (systematic or partiy1 bit stream)
 *  @param[out]  pOutput - pointer to output buffer
 *
 *  @return  no return value
 *
 *  @description
 *  Sub block interleaver for systematic and parity bit stream (d0d1) as described in
 *  3GPP TS 36.211 Rel 8 Sec.5.1.4.1.1
 *
**/
//-------------------------------------------------------------------------------------------
inline static void subblock_interleaver_d0d1_avx2(const int32_t nLen, const uint32_t nRound, const uint32_t numOfNull, uint8_t * pInput, uint8_t * pOutput)
{
    __m256i vByte31_00, vByte63_32, vByte95_64, vByte127_96, vByte32;
    __m256i * pIn256 = (__m256i *) pInput;
    __m256i vShuf256;
    __m256i vtmp00, vtmp11, vtmp22, vtmp33, vtmp44, vtmp55, vtmp66, vtmp77, vtmp88, vtmp99;
    __m256i vReady256;

    int32_t *pOut32 = (int32_t *) pOutput;
    int32_t trans_bits;
    int32_t j, step = 7;

    vShuf256 = _mm256_set_epi8(15, 11, 7, 3, 14, 10, 6, 2, 13, 9, 5, 1, 12, 8, 4, 0, 15, 11, 7, 3, 14, 10, 6, 2, 13, 9, 5, 1, 12, 8, 4, 0);
    vByte32 = _mm256_set_epi32(0x7, 0x3, 0x5, 0x1, 0x6, 0x2, 0x4, 0x0);

    switch(numOfNull)
    {
        case 4:
        {
#pragma ivdep
#pragma vector aligned
#pragma loop_count min=1, max=6
            for (j=0; j<nRound; j++) /* 6*128*8bits = 6144bits */
            {
                vByte31_00 = _mm256_lddqu_si256 (pIn256++);
                vByte63_32 = _mm256_lddqu_si256 (pIn256++);
                vByte95_64 = _mm256_lddqu_si256 (pIn256++);
                vByte127_96 = _mm256_lddqu_si256 (pIn256++);

                vtmp00 = _mm256_shuffle_epi8 (vByte31_00, vShuf256); // 31, 27, 23, 19, 30, 26, 22, 18, 29, 25, 21, 17, 28, 24, 20, 16     15, 11, 7, 3, 14, 10, 6, 2, 13, 9, 5, 1, 12, 8, 4, 0

                vtmp55 = _mm256_permutevar8x32_epi32(vtmp00, vByte32);  //31, 27, 23, 19, 15, 11, 7, 3    29, 25, 21, 17,  13, 9, 5, 1,    30, 26, 22,  18, 14, 10, 6, 2    28, 24, 20, 16,  12, 8, 4, 0

                vtmp11 = _mm256_shuffle_epi8 (vByte63_32, vShuf256); // 31+32, 27+32, 23+32, 19+32, 30+32, 26+32, 22+32, 18+32, 29+32, 25+32, 21+32, 17+32, 60, 56, 52, 48    15+32, 11+32, 7+32, 3+32, 14+32, 10+32, 6+32, 2+32, 13+32, 9+32, 5+32, 1+32, 44, 40, 36, 32
                vtmp66 = _mm256_permutevar8x32_epi32(vtmp11, vByte32); // 31+32, 27+32, 23+32, 19+32, 15+32, 11+32, 7+32, 3+32     30+32, 26+32, 22+32, 18+32, 14+32, 10+32, 6+32, 2+32     29+32, 25+32, 21+32, 17+32, 13+32, 9+32, 5+32, 1+32,     60, 56, 52, 48, 44, 40, 36, 32

                vtmp22 = _mm256_shuffle_epi8 (vByte95_64, vShuf256);
                vtmp77= _mm256_permutevar8x32_epi32 (vtmp22, vByte32);

                vtmp33 = _mm256_shuffle_epi8 (vByte127_96, vShuf256);
                vtmp88 = _mm256_permutevar8x32_epi32 (vtmp33, vByte32);

                vtmp44 = _mm256_unpacklo_epi64 (vtmp55, vtmp66); //61, 57, 53, 49, 45, 41, 37, 33, 29, 25, 21, 17, 13, 9, 5, 1    60, 56, 52, 48, 44, 40, 36, 32 28, 24, 20, 16, 12, 8, 4, 0

                vtmp99 = _mm256_unpacklo_epi64 (vtmp77, vtmp88);// 61+64, 57, 53, 49, 45, 41, 37, 33, 29, 25, 21, 17, 13, 9, 5, 1+64    60+64 56, 52, 48, 44, 40, 36, 32 28, 24, 20, 16, 12, 8, 4, 0+64

                vReady256 = _mm256_permute2x128_si256(vtmp44, vtmp99, 0x20);  // 124..., 64, 60, 56, 52, 48, 44, 40, 36, 32 28, 24, 20, 16, 12, 8, 4, 0

                trans_bits = _mm256_movemask_epi8 (vReady256); // bit7
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 4 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b6
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 20 * step + j) =  trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b5
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 12* step + j) =  trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b4
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 28 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b3
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 2 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b2
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 18 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b1
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 10 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b0
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 26 * step + j) = trans_bits;

                vReady256 = _mm256_permute2x128_si256(vtmp44, vtmp99, 0x31);  // 125..., 65, 61, 57, 53, 49, 45, 41, 37, 33, 29, 25, 21, 17, 13, 9, 5, 1

                trans_bits = _mm256_movemask_epi8 (vReady256); // b15,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 6 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b14,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 22 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b13,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 14 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b12,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 30 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b11,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 1 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b10,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 17 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b9,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 9 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b8,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 25 * step + j) = trans_bits;


                vtmp44 = _mm256_unpackhi_epi64 (vtmp55, vtmp66);   // 31+32, 27, 23, 19, 15, 11, 7, 3+32   31, 27, 23, 19, 15, 11, 7, 3   30+32, 26, 22, 18, 14, 10, 6, 2+32   30, 26, 22, 18, 14, 10, 6, 2
                vtmp99 = _mm256_unpackhi_epi64 (vtmp77, vtmp88);

                vReady256 = _mm256_permute2x128_si256(vtmp44, vtmp99, 0x20);  // 126..., 30, 26, 22, 18, 14, 10, 6, 2

                trans_bits = _mm256_movemask_epi8 (vReady256); // b23, ... to 25
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 5 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b22, ... to 9
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 21 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b21, ... to 17
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 13 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b20, ... to 1
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 29 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b19, ... to 30
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 3 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b18, ... to 14
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 19 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b17, ... to 22
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 11 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b16, ... to 6
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 27 * step + j) = trans_bits;

                vReady256 = _mm256_permute2x128_si256(vtmp44, vtmp99, 0x31); //127..., 31, 27, 23, 19, 15, 11, 7, 3

                trans_bits = _mm256_movemask_epi8 (vReady256); // b31, ... to 27
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 7 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b30, ... to 11
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 23 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b29, ... to 19
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 15 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b28, ... to 3
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 31 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b27, ... to 29
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 0 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b26, ... to 13
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 16 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b25, ... to 21
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 8 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b24, ... to 5
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 24 * step + j) = trans_bits;

            }

            break;
        }
        case 12:
        {
#pragma ivdep
#pragma vector aligned
#pragma loop_count min=1, max=6
            for (j=0; j<nRound; j++) /* 6*128*8bits = 6144bits */
            {
                vByte31_00 = _mm256_lddqu_si256 (pIn256++);
                vByte63_32 = _mm256_lddqu_si256 (pIn256++);
                vByte95_64 = _mm256_lddqu_si256 (pIn256++);
                vByte127_96 = _mm256_lddqu_si256 (pIn256++);

                vtmp00 = _mm256_shuffle_epi8 (vByte31_00, vShuf256); // 31, 27, 23, 19, 30, 26, 22, 18, 29, 25, 21, 17, 28, 24, 20, 16     15, 11, 7, 3, 14, 10, 6, 2, 13, 9, 5, 1, 12, 8, 4, 0
                vtmp55 = _mm256_permutevar8x32_epi32(vtmp00, vByte32);  //31, 27, 23, 19, 15, 11, 7, 3    29, 25, 21, 17,  13, 9, 5, 1,    30, 26, 22,  18, 14, 10, 6, 2    28, 24, 20, 16,  12, 8, 4, 0

                vtmp11 = _mm256_shuffle_epi8 (vByte63_32, vShuf256); // 31+32, 27+32, 23+32, 19+32, 30+32, 26+32, 22+32, 18+32, 29+32, 25+32, 21+32, 17+32, 60, 56, 52, 48    15+32, 11+32, 7+32, 3+32, 14+32, 10+32, 6+32, 2+32, 13+32, 9+32, 5+32, 1+32, 44, 40, 36, 32
                vtmp66 = _mm256_permutevar8x32_epi32(vtmp11, vByte32); // 31+32, 27+32, 23+32, 19+32, 15+32, 11+32, 7+32, 3+32     30+32, 26+32, 22+32, 18+32, 14+32, 10+32, 6+32, 2+32     29+32, 25+32, 21+32, 17+32, 13+32, 9+32, 5+32, 1+32,     60, 56, 52, 48, 44, 40, 36, 32


                vtmp22 = _mm256_shuffle_epi8 (vByte95_64, vShuf256);
                vtmp77= _mm256_permutevar8x32_epi32 (vtmp22, vByte32);

                vtmp33 = _mm256_shuffle_epi8 (vByte127_96, vShuf256);
                vtmp88 = _mm256_permutevar8x32_epi32 (vtmp33, vByte32);

                vtmp44 = _mm256_unpacklo_epi64 (vtmp55, vtmp66); //61, 57, 53, 49, 45, 41, 37, 33, 29, 25, 21, 17, 13, 9, 5, 1    60, 56, 52, 48, 44, 40, 36, 32 28, 24, 20, 16, 12, 8, 4, 0
                vtmp99 = _mm256_unpacklo_epi64 (vtmp77, vtmp88);// 61+64, 57, 53, 49, 45, 41, 37, 33, 29, 25, 21, 17, 13, 9, 5, 1+64    60+64 56, 52, 48, 44, 40, 36, 32 28, 24, 20, 16, 12, 8, 4, 0+64

                vReady256 = _mm256_permute2x128_si256(vtmp44, vtmp99, 0x20);  // 124..., 64, 60, 56, 52, 48, 44, 40, 36, 32 28, 24, 20, 16, 12, 8, 4, 0

                trans_bits = _mm256_movemask_epi8 (vReady256); // bit7
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 6 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b6
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 22 * step + j) =  trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b5
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 14 * step + j) =  trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b4
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 30 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b3
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 1 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b2
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 17 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b1
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 9 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b0
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 25 * step + j) = trans_bits;

                vReady256 = _mm256_permute2x128_si256(vtmp44, vtmp99, 0x31);  // 125..., 65, 61, 57, 53, 49, 45, 41, 37, 33, 29, 25, 21, 17, 13, 9, 5, 1

                trans_bits = _mm256_movemask_epi8 (vReady256); // b15,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 5 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b14,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 21 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b13,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 13 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b12,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 29 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b11,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 3 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b10,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 19 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b9,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 11 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b8,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 27 * step + j) = trans_bits;

                vtmp44 = _mm256_unpackhi_epi64 (vtmp55, vtmp66);   // 31+32, 27, 23, 19, 15, 11, 7, 3+32   31, 27, 23, 19, 15, 11, 7, 3   30+32, 26, 22, 18, 14, 10, 6, 2+32   30, 26, 22, 18, 14, 10, 6, 2
                vtmp99 = _mm256_unpackhi_epi64 (vtmp77, vtmp88);

                vReady256 = _mm256_permute2x128_si256(vtmp44, vtmp99, 0x20);  // 126..., 30, 26, 22, 18, 14, 10, 6, 2

                trans_bits = _mm256_movemask_epi8 (vReady256); // b23, ... to 25
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 7 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b22, ... to 9
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 23 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b21, ... to 17
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 15 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b20, ... to 1
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 31 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b19, ... to 30
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 0 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b18, ... to 14
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 16 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b17, ... to 22
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 8 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b16, ... to 6
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 24 * step + j) = trans_bits;

                vReady256 = _mm256_permute2x128_si256(vtmp44, vtmp99, 0x31); //127..., 31, 27, 23, 19, 15, 11, 7, 3

                trans_bits = _mm256_movemask_epi8 (vReady256); // b31, ... to 27
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 4 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b30, ... to 11
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 20 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b29, ... to 19
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 12 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b28, ... to 3
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 28 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b27, ... to 29
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 2 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b26, ... to 13
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 18 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b25, ... to 21
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 10 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b24, ... to 5
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 26 * step + j) = trans_bits;

            }

            break;
        }

        case 20:
        {
#pragma ivdep
#pragma vector aligned
#pragma loop_count min=1, max=6
            for (j=0; j<nRound; j++) /* 6*128*8bits = 6144bits */
            {
                vByte31_00 = _mm256_lddqu_si256 (pIn256++);
                vByte63_32 = _mm256_lddqu_si256 (pIn256++);
                vByte95_64 = _mm256_lddqu_si256 (pIn256++);
                vByte127_96 = _mm256_lddqu_si256 (pIn256++);

                vtmp00 = _mm256_shuffle_epi8 (vByte31_00, vShuf256); // 31, 27, 23, 19, 30, 26, 22, 18, 29, 25, 21, 17, 28, 24, 20, 16     15, 11, 7, 3, 14, 10, 6, 2, 13, 9, 5, 1, 12, 8, 4, 0
                vtmp55 = _mm256_permutevar8x32_epi32(vtmp00, vByte32);  //31, 27, 23, 19, 15, 11, 7, 3    29, 25, 21, 17,  13, 9, 5, 1,    30, 26, 22,  18, 14, 10, 6, 2    28, 24, 20, 16,  12, 8, 4, 0

                vtmp11 = _mm256_shuffle_epi8 (vByte63_32, vShuf256); // 31+32, 27+32, 23+32, 19+32, 30+32, 26+32, 22+32, 18+32, 29+32, 25+32, 21+32, 17+32, 60, 56, 52, 48    15+32, 11+32, 7+32, 3+32, 14+32, 10+32, 6+32, 2+32, 13+32, 9+32, 5+32, 1+32, 44, 40, 36, 32
                vtmp66 = _mm256_permutevar8x32_epi32(vtmp11, vByte32); // 31+32, 27+32, 23+32, 19+32, 15+32, 11+32, 7+32, 3+32     30+32, 26+32, 22+32, 18+32, 14+32, 10+32, 6+32, 2+32     29+32, 25+32, 21+32, 17+32, 13+32, 9+32, 5+32, 1+32,     60, 56, 52, 48, 44, 40, 36, 32


                vtmp22 = _mm256_shuffle_epi8 (vByte95_64, vShuf256);
                vtmp77= _mm256_permutevar8x32_epi32 (vtmp22, vByte32);

                vtmp33 = _mm256_shuffle_epi8 (vByte127_96, vShuf256);
                vtmp88 = _mm256_permutevar8x32_epi32 (vtmp33, vByte32);

                vtmp44 = _mm256_unpacklo_epi64 (vtmp55, vtmp66); //61, 57, 53, 49, 45, 41, 37, 33, 29, 25, 21, 17, 13, 9, 5, 1    60, 56, 52, 48, 44, 40, 36, 32 28, 24, 20, 16, 12, 8, 4, 0
                vtmp99 = _mm256_unpacklo_epi64 (vtmp77, vtmp88);// 61+64, 57, 53, 49, 45, 41, 37, 33, 29, 25, 21, 17, 13, 9, 5, 1+64    60+64 56, 52, 48, 44, 40, 36, 32 28, 24, 20, 16, 12, 8, 4, 0+64

                vReady256 = _mm256_permute2x128_si256(vtmp44, vtmp99, 0x20);  // 124..., 64, 60, 56, 52, 48, 44, 40, 36, 32 28, 24, 20, 16, 12, 8, 4, 0

                trans_bits = _mm256_movemask_epi8 (vReady256); // bit7
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 5 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b6
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 21 * step + j) =  trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b5
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 13 * step + j) =  trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b4
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 29 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b3
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 3 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b2
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 19 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b1
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 11 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b0
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 27 * step + j) = trans_bits;

                vReady256 = _mm256_permute2x128_si256(vtmp44, vtmp99, 0x31);  // 125..., 65, 61, 57, 53, 49, 45, 41, 37, 33, 29, 25, 21, 17, 13, 9, 5, 1

                trans_bits = _mm256_movemask_epi8 (vReady256); // b15,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 7 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b14,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 23 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b13,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 15 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b12,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 31 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b11,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 0 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b10,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 16 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b9,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 8 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b8,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 24 * step + j) = trans_bits;

                vtmp44 = _mm256_unpackhi_epi64 (vtmp55, vtmp66);   // 31+32, 27, 23, 19, 15, 11, 7, 3+32   31, 27, 23, 19, 15, 11, 7, 3   30+32, 26, 22, 18, 14, 10, 6, 2+32   30, 26, 22, 18, 14, 10, 6, 2
                vtmp99 = _mm256_unpackhi_epi64 (vtmp77, vtmp88);

                vReady256 = _mm256_permute2x128_si256(vtmp44, vtmp99, 0x20);  // 126..., 30, 26, 22, 18, 14, 10, 6, 2

                trans_bits = _mm256_movemask_epi8 (vReady256); // b23, ... to 25
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 4 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b22, ... to 9
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 20 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b21, ... to 17
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 12 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b20, ... to 1
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 28 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b19, ... to 30
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 2 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b18, ... to 14
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 18 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b17, ... to 22
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 10 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b16, ... to 6
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 26 * step + j) = trans_bits;

                vReady256 = _mm256_permute2x128_si256(vtmp44, vtmp99, 0x31); //127..., 31, 27, 23, 19, 15, 11, 7, 3

                trans_bits = _mm256_movemask_epi8 (vReady256); // b31, ... to 27
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 6 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b30, ... to 11
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 22 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b29, ... to 19
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 14 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b28, ... to 3
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 30 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b27, ... to 29
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 1 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b26, ... to 13
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 17 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b25, ... to 21
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 9 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b24, ... to 5
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 25 * step + j) = trans_bits;

            }

            break;
        }

        case 28:
        {
#pragma ivdep
#pragma vector aligned
#pragma loop_count min=1, max=6
            for (j=0; j<nRound; j++) /* 6*128*8bits = 6144bits */
            {
                vByte31_00 = _mm256_lddqu_si256 (pIn256++);
                vByte63_32 = _mm256_lddqu_si256 (pIn256++);
                vByte95_64 = _mm256_lddqu_si256 (pIn256++);
                vByte127_96 = _mm256_lddqu_si256 (pIn256++);

                vtmp00 = _mm256_shuffle_epi8 (vByte31_00, vShuf256); // 31, 27, 23, 19, 30, 26, 22, 18, 29, 25, 21, 17, 28, 24, 20, 16     15, 11, 7, 3, 14, 10, 6, 2, 13, 9, 5, 1, 12, 8, 4, 0

                vtmp55 = _mm256_permutevar8x32_epi32(vtmp00, vByte32);  //31, 27, 23, 19, 15, 11, 7, 3    29, 25, 21, 17,  13, 9, 5, 1,    30, 26, 22,  18, 14, 10, 6, 2    28, 24, 20, 16,  12, 8, 4, 0

                vtmp11 = _mm256_shuffle_epi8 (vByte63_32, vShuf256); // 31+32, 27+32, 23+32, 19+32, 30+32, 26+32, 22+32, 18+32, 29+32, 25+32, 21+32, 17+32, 60, 56, 52, 48    15+32, 11+32, 7+32, 3+32, 14+32, 10+32, 6+32, 2+32, 13+32, 9+32, 5+32, 1+32, 44, 40, 36, 32
                vtmp66 = _mm256_permutevar8x32_epi32(vtmp11, vByte32); // 31+32, 27+32, 23+32, 19+32, 15+32, 11+32, 7+32, 3+32     30+32, 26+32, 22+32, 18+32, 14+32, 10+32, 6+32, 2+32     29+32, 25+32, 21+32, 17+32, 13+32, 9+32, 5+32, 1+32,     60, 56, 52, 48, 44, 40, 36, 32

                vtmp22 = _mm256_shuffle_epi8 (vByte95_64, vShuf256);
                vtmp77= _mm256_permutevar8x32_epi32 (vtmp22, vByte32);

                vtmp33 = _mm256_shuffle_epi8 (vByte127_96, vShuf256);
                vtmp88 = _mm256_permutevar8x32_epi32 (vtmp33, vByte32);

                vtmp44 = _mm256_unpacklo_epi64 (vtmp55, vtmp66); //61, 57, 53, 49, 45, 41, 37, 33, 29, 25, 21, 17, 13, 9, 5, 1    60, 56, 52, 48, 44, 40, 36, 32 28, 24, 20, 16, 12, 8, 4, 0

                vtmp99 = _mm256_unpacklo_epi64 (vtmp77, vtmp88);// 61+64, 57, 53, 49, 45, 41, 37, 33, 29, 25, 21, 17, 13, 9, 5, 1+64    60+64 56, 52, 48, 44, 40, 36, 32 28, 24, 20, 16, 12, 8, 4, 0+64

                vReady256 = _mm256_permute2x128_si256(vtmp44, vtmp99, 0x20);  // 124..., 64, 60, 56, 52, 48, 44, 40, 36, 32 28, 24, 20, 16, 12, 8, 4, 0

                trans_bits = _mm256_movemask_epi8 (vReady256); // bit7
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 7 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b6
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 23 * step + j) = trans_bits;


                trans_bits = _mm256_movemask_epi8 (vReady256); // b5
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 15 * step + j) =  trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b4
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 31* step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b3
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 0 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b2
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 16 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b1
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 8 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b0
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 24 * step + j) = trans_bits;

                vReady256 = _mm256_permute2x128_si256(vtmp44, vtmp99, 0x31);  // 125..., 65, 61, 57, 53, 49, 45, 41, 37, 33, 29, 25, 21, 17, 13, 9, 5, 1

                trans_bits = _mm256_movemask_epi8 (vReady256); // b15,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 4 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b14,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 20 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b13,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 12 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b12,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 28 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b11,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 2 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b10,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 18 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b9,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 10 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b8,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 26 * step + j) = trans_bits;

                vtmp44 = _mm256_unpackhi_epi64 (vtmp55, vtmp66);   // 31+32, 27, 23, 19, 15, 11, 7, 3+32   31, 27, 23, 19, 15, 11, 7, 3   30+32, 26, 22, 18, 14, 10, 6, 2+32   30, 26, 22, 18, 14, 10, 6, 2
                vtmp99 = _mm256_unpackhi_epi64 (vtmp77, vtmp88);

                vReady256 = _mm256_permute2x128_si256(vtmp44, vtmp99, 0x20);  // 126..., 30, 26, 22, 18, 14, 10, 6, 2

                trans_bits = _mm256_movemask_epi8 (vReady256); // b23, ... to 25
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 6 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b22, ... to 9
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 22 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b21, ... to 17
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 14 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b20, ... to 1
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 30 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b19, ... to 30
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 1 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b18, ... to 14
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 17 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b17, ... to 22
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 9 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b16, ... to 6
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 25 * step + j) = trans_bits;

                vReady256 = _mm256_permute2x128_si256(vtmp44, vtmp99, 0x31); //127..., 31, 27, 23, 19, 15, 11, 7, 3

                trans_bits = _mm256_movemask_epi8 (vReady256); // b31, ... to 27
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 5 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b30, ... to 11
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 21 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b29, ... to 19
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 13 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b28, ... to 3
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 29 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b27, ... to 29
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 3 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b26, ... to 13
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 19* step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b25, ... to 21
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 11 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b24, ... to 5
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 27 * step + j) = trans_bits;

            }

            break;
        }

    }

    return;
}



//-------------------------------------------------------------------------------------------
/** @ingroup group_lte_source_phy_fec_enc
 *
 *  @param[in]   nLen - length of input buffer
 *  @param[in]   nRound - number of round operation depend on code block size
 *  @param[in]   numOfNull - number of null bits for one code block
 *  @param[in]   pInput - pointer to input buffer (parity2 bit stream)
 *  @param[out]  pOutput - pointer to output buffer
 *
 *  @return  no return value
 *
 *  @description
 *  Sub block interleaver for parity bit stream (d2) as described in
 *  3GPP TS 36.211 Rel 8 Sec.5.1.4.1.1
 *
**/
//-------------------------------------------------------------------------------------------
inline static void subblock_interleaver_d2_avx2(const int32_t nLen, const uint32_t nRound, const uint32_t numOfNull, uint8_t * pInput, uint8_t * pOutput)
{
    __m256i vByte31_00, vByte63_32, vByte95_64, vByte127_96, vByte32;
    __m256i * pIn256 = (__m256i *) pInput;
    __m256i vShuf256;
    __m256i vtmp00, vtmp11, vtmp22, vtmp33, vtmp44, vtmp55, vtmp66, vtmp77, vtmp88, vtmp99;
    __m256i vReady256;

    int32_t *pOut32 = (int32_t *) pOutput;
    int32_t trans_bits;
    int32_t j, step = 7;

    //vShuf = _mm_set_epi8(15, 11, 7, 3, 14, 10, 6, 2, 13, 9, 5, 1, 12, 8, 4, 0);

    vShuf256 = _mm256_set_epi8(15, 11, 7, 3, 14, 10, 6, 2, 13, 9, 5, 1, 12, 8, 4, 0, 15, 11, 7, 3, 14, 10, 6, 2, 13, 9, 5, 1, 12, 8, 4, 0);
    vByte32 = _mm256_set_epi32(0x7, 0x3, 0x5, 0x1, 0x6, 0x2, 0x4, 0x0);

    switch(numOfNull)
    {
        case 4:
        {
#pragma ivdep
#pragma vector aligned
#pragma loop_count min=1, max=6
            for (j=0; j<nRound; j++) /* 6*128*8bits = 6144bits */
            {
                vByte31_00 = _mm256_lddqu_si256 (pIn256++);
                vByte63_32 = _mm256_lddqu_si256 (pIn256++);
                vByte95_64 = _mm256_lddqu_si256 (pIn256++);
                vByte127_96 = _mm256_lddqu_si256 (pIn256++);

                vtmp00 = _mm256_shuffle_epi8 (vByte31_00, vShuf256); // 31, 27, 23, 19, 30, 26, 22, 18, 29, 25, 21, 17, 28, 24, 20, 16     15, 11, 7, 3, 14, 10, 6, 2, 13, 9, 5, 1, 12, 8, 4, 0
                vtmp55 = _mm256_permutevar8x32_epi32(vtmp00, vByte32);  //31, 27, 23, 19, 15, 11, 7, 3    29, 25, 21, 17,  13, 9, 5, 1,    30, 26, 22,  18, 14, 10, 6, 2    28, 24, 20, 16,  12, 8, 4, 0

                vtmp11 = _mm256_shuffle_epi8 (vByte63_32, vShuf256); // 31+32, 27+32, 23+32, 19+32, 30+32, 26+32, 22+32, 18+32, 29+32, 25+32, 21+32, 17+32, 60, 56, 52, 48    15+32, 11+32, 7+32, 3+32, 14+32, 10+32, 6+32, 2+32, 13+32, 9+32, 5+32, 1+32, 44, 40, 36, 32
                vtmp66 = _mm256_permutevar8x32_epi32(vtmp11, vByte32); // 31+32, 27+32, 23+32, 19+32, 15+32, 11+32, 7+32, 3+32     30+32, 26+32, 22+32, 18+32, 14+32, 10+32, 6+32, 2+32     29+32, 25+32, 21+32, 17+32, 13+32, 9+32, 5+32, 1+32,     60, 56, 52, 48, 44, 40, 36, 32


                vtmp22 = _mm256_shuffle_epi8 (vByte95_64, vShuf256);
                vtmp77= _mm256_permutevar8x32_epi32 (vtmp22, vByte32);

                vtmp33 = _mm256_shuffle_epi8 (vByte127_96, vShuf256);
                vtmp88 = _mm256_permutevar8x32_epi32 (vtmp33, vByte32);

                vtmp44 = _mm256_unpacklo_epi64 (vtmp55, vtmp66); //61, 57, 53, 49, 45, 41, 37, 33, 29, 25, 21, 17, 13, 9, 5, 1    60, 56, 52, 48, 44, 40, 36, 32 28, 24, 20, 16, 12, 8, 4, 0
                vtmp99 = _mm256_unpacklo_epi64 (vtmp77, vtmp88);// 61+64, 57, 53, 49, 45, 41, 37, 33, 29, 25, 21, 17, 13, 9, 5, 1+64    60+64 56, 52, 48, 44, 40, 36, 32 28, 24, 20, 16, 12, 8, 4, 0+64

                vReady256 = _mm256_permute2x128_si256(vtmp44, vtmp99, 0x20);  // 124..., 64, 60, 56, 52, 48, 44, 40, 36, 32 28, 24, 20, 16, 12, 8, 4, 0

                trans_bits = _mm256_movemask_epi8 (vReady256); // bit7
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 24 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b6
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 4 * step + j) =  trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b5
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 20 * step + j) =  trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b4
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 12 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b3
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 28 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b2
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 2 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b1
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 18 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b0
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 10 * step + j) = trans_bits;

                vReady256 = _mm256_permute2x128_si256(vtmp44, vtmp99, 0x31);  // 125..., 65, 61, 57, 53, 49, 45, 41, 37, 33, 29, 25, 21, 17, 13, 9, 5, 1

                trans_bits = _mm256_movemask_epi8 (vReady256); // b15,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 26 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b14,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 6 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b13,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 22 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b12,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 14 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b11,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 30 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b10,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 1 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b9,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 17 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b8,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 9 * step + j) = trans_bits;


                vtmp44 = _mm256_unpackhi_epi64 (vtmp55, vtmp66);   // 31+32, 27, 23, 19, 15, 11, 7, 3+32   31, 27, 23, 19, 15, 11, 7, 3   30+32, 26, 22, 18, 14, 10, 6, 2+32   30, 26, 22, 18, 14, 10, 6, 2
                vtmp99 = _mm256_unpackhi_epi64 (vtmp77, vtmp88);

                vReady256 = _mm256_permute2x128_si256(vtmp44, vtmp99, 0x20);  // 126..., 30, 26, 22, 18, 14, 10, 6, 2

                trans_bits = _mm256_movemask_epi8 (vReady256); // b23, ... to 25
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 25 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b22, ... to 9
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 5 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b21, ... to 17
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 21 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b20, ... to 1
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 13 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b19, ... to 30
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 29 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b18, ... to 14
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 3 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b17, ... to 22
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 19 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b16, ... to 6
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 11 * step + j) = trans_bits;

                vReady256 = _mm256_permute2x128_si256(vtmp44, vtmp99, 0x31); //127..., 31, 27, 23, 19, 15, 11, 7, 3

                trans_bits = _mm256_movemask_epi8 (vReady256); // b31, ... to 27
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 27 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b30, ... to 11
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 7 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b29, ... to 19
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 23 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b28, ... to 3
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 15 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b27, ... to 29
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 31 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b26, ... to 13
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 0 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b25, ... to 21
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 16 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b24, ... to 5
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 8 * step + j) = trans_bits;

            }

            break;
        }
        case 12:
        {
#pragma ivdep
#pragma vector aligned
#pragma loop_count min=1, max=6
            for (j=0; j<nRound; j++) /* 6*128*8bits = 6144bits */
            {
                vByte31_00 = _mm256_lddqu_si256 (pIn256++);
                vByte63_32 = _mm256_lddqu_si256 (pIn256++);
                vByte95_64 = _mm256_lddqu_si256 (pIn256++);
                vByte127_96 = _mm256_lddqu_si256 (pIn256++);

                vtmp00 = _mm256_shuffle_epi8 (vByte31_00, vShuf256); // 31, 27, 23, 19, 30, 26, 22, 18, 29, 25, 21, 17, 28, 24, 20, 16     15, 11, 7, 3, 14, 10, 6, 2, 13, 9, 5, 1, 12, 8, 4, 0
                vtmp55 = _mm256_permutevar8x32_epi32(vtmp00, vByte32);  //31, 27, 23, 19, 15, 11, 7, 3    29, 25, 21, 17,  13, 9, 5, 1,    30, 26, 22,  18, 14, 10, 6, 2    28, 24, 20, 16,  12, 8, 4, 0

                vtmp11 = _mm256_shuffle_epi8 (vByte63_32, vShuf256); // 31+32, 27+32, 23+32, 19+32, 30+32, 26+32, 22+32, 18+32, 29+32, 25+32, 21+32, 17+32, 60, 56, 52, 48    15+32, 11+32, 7+32, 3+32, 14+32, 10+32, 6+32, 2+32, 13+32, 9+32, 5+32, 1+32, 44, 40, 36, 32
                vtmp66 = _mm256_permutevar8x32_epi32(vtmp11, vByte32); // 31+32, 27+32, 23+32, 19+32, 15+32, 11+32, 7+32, 3+32     30+32, 26+32, 22+32, 18+32, 14+32, 10+32, 6+32, 2+32     29+32, 25+32, 21+32, 17+32, 13+32, 9+32, 5+32, 1+32,     60, 56, 52, 48, 44, 40, 36, 32


                vtmp22 = _mm256_shuffle_epi8 (vByte95_64, vShuf256);
                vtmp77= _mm256_permutevar8x32_epi32 (vtmp22, vByte32);

                vtmp33 = _mm256_shuffle_epi8 (vByte127_96, vShuf256);
                vtmp88 = _mm256_permutevar8x32_epi32 (vtmp33, vByte32);

                vtmp44 = _mm256_unpacklo_epi64 (vtmp55, vtmp66); //61, 57, 53, 49, 45, 41, 37, 33, 29, 25, 21, 17, 13, 9, 5, 1    60, 56, 52, 48, 44, 40, 36, 32 28, 24, 20, 16, 12, 8, 4, 0
                vtmp99 = _mm256_unpacklo_epi64 (vtmp77, vtmp88);// 61+64, 57, 53, 49, 45, 41, 37, 33, 29, 25, 21, 17, 13, 9, 5, 1+64    60+64 56, 52, 48, 44, 40, 36, 32 28, 24, 20, 16, 12, 8, 4, 0+64

                vReady256 = _mm256_permute2x128_si256(vtmp44, vtmp99, 0x20);  // 124..., 64, 60, 56, 52, 48, 44, 40, 36, 32 28, 24, 20, 16, 12, 8, 4, 0

                trans_bits = _mm256_movemask_epi8 (vReady256); // bit7
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 26 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b14,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 6 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b13,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 22 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b12,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 14 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b11,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 30 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b10,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 1 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b9,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 17 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b8,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 9 * step + j) = trans_bits;


                vReady256 = _mm256_permute2x128_si256(vtmp44, vtmp99, 0x31);  // 125..., 65, 61, 57, 53, 49, 45, 41, 37, 33, 29, 25, 21, 17, 13, 9, 5, 1

                trans_bits = _mm256_movemask_epi8 (vReady256); // b15,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 25 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b22, ... to 9
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 5 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b21, ... to 17
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 21 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b20, ... to 1
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 13 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b19, ... to 30
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 29 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b18, ... to 14
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 3 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b17, ... to 22
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 19 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b16, ... to 6
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 11 * step + j) = trans_bits;


                vtmp44 = _mm256_unpackhi_epi64 (vtmp55, vtmp66);   // 31+32, 27, 23, 19, 15, 11, 7, 3+32   31, 27, 23, 19, 15, 11, 7, 3   30+32, 26, 22, 18, 14, 10, 6, 2+32   30, 26, 22, 18, 14, 10, 6, 2
                vtmp99 = _mm256_unpackhi_epi64 (vtmp77, vtmp88);

                vReady256 = _mm256_permute2x128_si256(vtmp44, vtmp99, 0x20);  // 126..., 30, 26, 22, 18, 14, 10, 6, 2

                trans_bits = _mm256_movemask_epi8 (vReady256); // b23, ... to 25
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 27 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b30, ... to 11
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 7 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b29, ... to 19
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 23 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b28, ... to 3
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 15 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b27, ... to 29
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 31 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b26, ... to 13
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 0 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b25, ... to 21
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 16 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b24, ... to 5
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 8 * step + j) = trans_bits;


                vReady256 = _mm256_permute2x128_si256(vtmp44, vtmp99, 0x31); //127..., 31, 27, 23, 19, 15, 11, 7, 3

                trans_bits = _mm256_movemask_epi8 (vReady256); // b31, ... to 27
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 24 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b6
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 4 * step + j) =  trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b5
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 20 * step + j) =  trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b4
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 12 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b3
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 28 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b2
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 2 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b1
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 18 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b0
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 10 * step + j) = trans_bits;


            }

            break;
        }

        case 20:
        {
#pragma ivdep
#pragma vector aligned
#pragma loop_count min=1, max=6
            for (j=0; j<nRound; j++) /* 6*128*8bits = 6144bits */
            {
                vByte31_00 = _mm256_lddqu_si256 (pIn256++);
                vByte63_32 = _mm256_lddqu_si256 (pIn256++);
                vByte95_64 = _mm256_lddqu_si256 (pIn256++);
                vByte127_96 = _mm256_lddqu_si256 (pIn256++);

                vtmp00 = _mm256_shuffle_epi8 (vByte31_00, vShuf256); // 31, 27, 23, 19, 30, 26, 22, 18, 29, 25, 21, 17, 28, 24, 20, 16     15, 11, 7, 3, 14, 10, 6, 2, 13, 9, 5, 1, 12, 8, 4, 0
                vtmp55 = _mm256_permutevar8x32_epi32(vtmp00, vByte32);  //31, 27, 23, 19, 15, 11, 7, 3    29, 25, 21, 17,  13, 9, 5, 1,    30, 26, 22,  18, 14, 10, 6, 2    28, 24, 20, 16,  12, 8, 4, 0

                vtmp11 = _mm256_shuffle_epi8 (vByte63_32, vShuf256); // 31+32, 27+32, 23+32, 19+32, 30+32, 26+32, 22+32, 18+32, 29+32, 25+32, 21+32, 17+32, 60, 56, 52, 48    15+32, 11+32, 7+32, 3+32, 14+32, 10+32, 6+32, 2+32, 13+32, 9+32, 5+32, 1+32, 44, 40, 36, 32
                vtmp66 = _mm256_permutevar8x32_epi32(vtmp11, vByte32); // 31+32, 27+32, 23+32, 19+32, 15+32, 11+32, 7+32, 3+32     30+32, 26+32, 22+32, 18+32, 14+32, 10+32, 6+32, 2+32     29+32, 25+32, 21+32, 17+32, 13+32, 9+32, 5+32, 1+32,     60, 56, 52, 48, 44, 40, 36, 32


                vtmp22 = _mm256_shuffle_epi8 (vByte95_64, vShuf256);
                vtmp77= _mm256_permutevar8x32_epi32 (vtmp22, vByte32);

                vtmp33 = _mm256_shuffle_epi8 (vByte127_96, vShuf256);
                vtmp88 = _mm256_permutevar8x32_epi32 (vtmp33, vByte32);

                vtmp44 = _mm256_unpacklo_epi64 (vtmp55, vtmp66); //61, 57, 53, 49, 45, 41, 37, 33, 29, 25, 21, 17, 13, 9, 5, 1    60, 56, 52, 48, 44, 40, 36, 32 28, 24, 20, 16, 12, 8, 4, 0
                vtmp99 = _mm256_unpacklo_epi64 (vtmp77, vtmp88);// 61+64, 57, 53, 49, 45, 41, 37, 33, 29, 25, 21, 17, 13, 9, 5, 1+64    60+64 56, 52, 48, 44, 40, 36, 32 28, 24, 20, 16, 12, 8, 4, 0+64

                vReady256 = _mm256_permute2x128_si256(vtmp44, vtmp99, 0x20);  // 124..., 64, 60, 56, 52, 48, 44, 40, 36, 32 28, 24, 20, 16, 12, 8, 4, 0

                trans_bits = _mm256_movemask_epi8 (vReady256); // bit7
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 25 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b22, ... to 9
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 5 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b21, ... to 17
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 21 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b20, ... to 1
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 13 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b19, ... to 30
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 29 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b18, ... to 14
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 3 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b17, ... to 22
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 19 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b16, ... to 6
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 11 * step + j) = trans_bits;


                vReady256 = _mm256_permute2x128_si256(vtmp44, vtmp99, 0x31);  // 125..., 65, 61, 57, 53, 49, 45, 41, 37, 33, 29, 25, 21, 17, 13, 9, 5, 1

                trans_bits = _mm256_movemask_epi8 (vReady256); // b15,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 27 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b30, ... to 11
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 7 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b29, ... to 19
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 23 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b28, ... to 3
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 15 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b27, ... to 29
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 31 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b26, ... to 13
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 0 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b25, ... to 21
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 16 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b24, ... to 5
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 8 * step + j) = trans_bits;


                vtmp44 = _mm256_unpackhi_epi64 (vtmp55, vtmp66);   // 31+32, 27, 23, 19, 15, 11, 7, 3+32   31, 27, 23, 19, 15, 11, 7, 3   30+32, 26, 22, 18, 14, 10, 6, 2+32   30, 26, 22, 18, 14, 10, 6, 2
                vtmp99 = _mm256_unpackhi_epi64 (vtmp77, vtmp88);

                vReady256 = _mm256_permute2x128_si256(vtmp44, vtmp99, 0x20);  // 126..., 30, 26, 22, 18, 14, 10, 6, 2

                trans_bits = _mm256_movemask_epi8 (vReady256); // b23, ... to 25
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 24 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b6
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 4 * step + j) =  trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b5
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 20 * step + j) =  trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b4
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 12 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b3
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 28 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b2
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 2 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b1
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 18 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b0
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 10 * step + j) = trans_bits;


                vReady256 = _mm256_permute2x128_si256(vtmp44, vtmp99, 0x31); //127..., 31, 27, 23, 19, 15, 11, 7, 3

                trans_bits = _mm256_movemask_epi8 (vReady256); // b31, ... to 27
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 26 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b14,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 6 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b13,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 22 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b12,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 14 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b11,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 30 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b10,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 1 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b9,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 17 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b8,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 9 * step + j) = trans_bits;


            }

            break;
        }

        case 28:
        {
#pragma ivdep
#pragma vector aligned
#pragma loop_count min=1, max=6
            for (j=0; j<nRound; j++) /* 6*128*8bits = 6144bits */
            {
                vByte31_00 = _mm256_lddqu_si256 (pIn256++);
                vByte63_32 = _mm256_lddqu_si256 (pIn256++);
                vByte95_64 = _mm256_lddqu_si256 (pIn256++);
                vByte127_96 = _mm256_lddqu_si256 (pIn256++);

                vtmp00 = _mm256_shuffle_epi8 (vByte31_00, vShuf256); // 31, 27, 23, 19, 30, 26, 22, 18, 29, 25, 21, 17, 28, 24, 20, 16     15, 11, 7, 3, 14, 10, 6, 2, 13, 9, 5, 1, 12, 8, 4, 0
                vtmp55 = _mm256_permutevar8x32_epi32(vtmp00, vByte32);  //31, 27, 23, 19, 15, 11, 7, 3    29, 25, 21, 17,  13, 9, 5, 1,    30, 26, 22,  18, 14, 10, 6, 2    28, 24, 20, 16,  12, 8, 4, 0

                vtmp11 = _mm256_shuffle_epi8 (vByte63_32, vShuf256); // 31+32, 27+32, 23+32, 19+32, 30+32, 26+32, 22+32, 18+32, 29+32, 25+32, 21+32, 17+32, 60, 56, 52, 48    15+32, 11+32, 7+32, 3+32, 14+32, 10+32, 6+32, 2+32, 13+32, 9+32, 5+32, 1+32, 44, 40, 36, 32
                vtmp66 = _mm256_permutevar8x32_epi32(vtmp11, vByte32); // 31+32, 27+32, 23+32, 19+32, 15+32, 11+32, 7+32, 3+32     30+32, 26+32, 22+32, 18+32, 14+32, 10+32, 6+32, 2+32     29+32, 25+32, 21+32, 17+32, 13+32, 9+32, 5+32, 1+32,     60, 56, 52, 48, 44, 40, 36, 32


                vtmp22 = _mm256_shuffle_epi8 (vByte95_64, vShuf256);
                vtmp77= _mm256_permutevar8x32_epi32 (vtmp22, vByte32);

                vtmp33 = _mm256_shuffle_epi8 (vByte127_96, vShuf256);
                vtmp88 = _mm256_permutevar8x32_epi32 (vtmp33, vByte32);

                vtmp44 = _mm256_unpacklo_epi64 (vtmp55, vtmp66); //61, 57, 53, 49, 45, 41, 37, 33, 29, 25, 21, 17, 13, 9, 5, 1    60, 56, 52, 48, 44, 40, 36, 32 28, 24, 20, 16, 12, 8, 4, 0
                vtmp99 = _mm256_unpacklo_epi64 (vtmp77, vtmp88);// 61+64, 57, 53, 49, 45, 41, 37, 33, 29, 25, 21, 17, 13, 9, 5, 1+64    60+64 56, 52, 48, 44, 40, 36, 32 28, 24, 20, 16, 12, 8, 4, 0+64

                vReady256 = _mm256_permute2x128_si256(vtmp44, vtmp99, 0x20);  // 124..., 64, 60, 56, 52, 48, 44, 40, 36, 32 28, 24, 20, 16, 12, 8, 4, 0

                trans_bits = _mm256_movemask_epi8 (vReady256); // bit7
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 27 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b6
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 7 * step + j) =  trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b5
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 23 * step + j) =  trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b4
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 15 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b3
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 31 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b2
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 0 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b1
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 16 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b0
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 8 * step + j) = trans_bits;

                vReady256 = _mm256_permute2x128_si256(vtmp44, vtmp99, 0x31);  // 125..., 65, 61, 57, 53, 49, 45, 41, 37, 33, 29, 25, 21, 17, 13, 9, 5, 1

                trans_bits = _mm256_movemask_epi8 (vReady256); // b15,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 24 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b14,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 4 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b13,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 20 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b12,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 12 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b11,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 28 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b10,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 2 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b9,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 18 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b8,
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 10 * step + j) = trans_bits;

                vtmp44 = _mm256_unpackhi_epi64 (vtmp55, vtmp66);   // 31+32, 27, 23, 19, 15, 11, 7, 3+32   31, 27, 23, 19, 15, 11, 7, 3   30+32, 26, 22, 18, 14, 10, 6, 2+32   30, 26, 22, 18, 14, 10, 6, 2
                vtmp99 = _mm256_unpackhi_epi64 (vtmp77, vtmp88);

                vReady256 = _mm256_permute2x128_si256(vtmp44, vtmp99, 0x20);  // 126..., 30, 26, 22, 18, 14, 10, 6, 2

                trans_bits = _mm256_movemask_epi8 (vReady256); // b23, ... to 25
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 26 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b22, ... to 9
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 6 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b21, ... to 17
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 22 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b20, ... to 1
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 14 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b19, ... to 30
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 30 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b18, ... to 14
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 1 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b17, ... to 22
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 17 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b16, ... to 6
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 9 * step + j) = trans_bits;

                vReady256 = _mm256_permute2x128_si256(vtmp44, vtmp99, 0x31); //127..., 31, 27, 23, 19, 15, 11, 7, 3

                trans_bits = _mm256_movemask_epi8 (vReady256); // b31, ... to 27
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 25 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b30, ... to 11
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 5 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b29, ... to 19
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 21 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b28, ... to 3
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 13* step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b27, ... to 29
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 29 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b26, ... to 13
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 3 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b25, ... to 21
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 19 * step + j) = trans_bits;

                trans_bits = _mm256_movemask_epi8 (vReady256); // b24, ... to 5
                vReady256 = _mm256_slli_epi64 (vReady256, 1);
                * (pOut32 + 11 * step + j) = trans_bits;

            }

            break;
        }

    }

    return;
}






//-------------------------------------------------------------------------------------------
/** @ingroup group_lte_source_phy_fec_enc
 *
 *  @param[in]   nLen - Length of input data
 *  @param[in]   numOfNull - number of null bits for one code block
 *  @param[in]   pv0 - pointer to stream 0 from output of sub-block interleaver
 *  @param[in]   pv1 - pointer to stream 1 from output of sub-block interleaver
 *  @param[in]   pv2 - pointer to stream 2 from output of sub-block interleaverO
 *  @param[out]  pOutput - pointer to final circular buffer output
 *
 *  @return  no return value
 *
 *  @description
 *  Function implements bit collection as described in 3GPP TS 36.211 Rel 8 Sec.5.1.4.1.2
 *
**/
//-------------------------------------------------------------------------------------------
inline static void bit_collection_avx2(const int32_t nLen, const uint32_t numOfNull, uint8_t * pv0, uint8_t * pv1, uint8_t * pv2, uint8_t * pOutput)
{
    __m256i v1_32, v2_32, v12_mix, v12_perm, vtmp;
    __m256i vconst00, vconst11, vbitmask;
    __m256i vshuf0, vshuf1, vshuf2, vshuf3, vshuf4, vshuf5, vshuf6, vshuf7;

    __m256i vshuf;
    __m256i * p1;
    __m256i * p2;
    __m256i * p12;

    uint32_t specialPos = 27;
    uint32_t step = 28;//, accStep=0;//24;
    int32_t nBits0, nBits1, nBits2, nBits3, nBits4, nBits;
    int32_t *tmpPtr = (int32_t *)&vtmp;
    uint32_t dstBitOffset = 0;
    uint32_t dstByte = 0;
    int32_t nLine;
    int32_t nBitVec0[32] = {0};
    int32_t nBitVec1[32] = {0};
    int32_t i, cnt = 0;

    // v1 and v2 perm bit by bit
    uint16_t v12[896+20];

    uint8_t * pOut = pOutput;
    int32_t nBitst;
    __m128i vtmps, vout1, vout2, vout3, vout4, vout5, vout6, vout;

    __m128i vconst0, vconstF;

    vconst0 = _mm_setzero_si128();
    vconstF = _mm_set_epi8(0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF);

    vconst00 = _mm256_setzero_si256 ();
    vconst11 = _mm256_set1_epi8(1);
    vbitmask = _mm256_set_epi16 (0x8080, 0x4040, 0x2020, 0x1010, 0x0808, 0x0404, 0x0202, 0x0101, 0x8080, 0x4040, 0x2020, 0x1010, 0x0808, 0x0404, 0x0202, 0x0101);

    vshuf0 = _mm256_set_epi8 (8, 0, 8, 0, 8, 0, 8, 0, 8, 0, 8, 0, 8, 0, 8, 0, 8, 0, 8, 0, 8, 0, 8, 0, 8, 0, 8, 0, 8, 0, 8, 0);
    vshuf1 = _mm256_add_epi8 (vshuf0, vconst11);
    vshuf2 = _mm256_add_epi8 (vshuf1, vconst11);
    vshuf3 = _mm256_add_epi8 (vshuf2, vconst11);
    vshuf4 = _mm256_add_epi8 (vshuf3, vconst11);
    vshuf5 = _mm256_add_epi8 (vshuf4, vconst11);
    vshuf6 = _mm256_add_epi8 (vshuf5, vconst11);
    vshuf7 = _mm256_add_epi8 (vshuf6, vconst11);

     vshuf = _mm256_set_epi8 (15,14, 11,10, 7,6, 3,2, 13,12, 9,8, 5,4, 1,0, 15,14, 11,10, 7,6, 3,2, 13,12, 9,8, 5,4, 1,0);

    // bit concat and output
    // ================= bit concat and output for system bits ===========

    /* nBits0 is the number of bit in each line, how many bits are left out of SSE64 bit register */
    nBits0 = (nLen / 32) % 64;
    nBits1 = nBits0 + 1;
    nBits2 = nBits0 * 2;
    nBits3 = nBits0 * 2 + 1;
    nBits4 = (nBits0 + 1) * 2;

    nLine = nLen / 32;

    int32_t nSSE = nLine / 64;

    if( ((nLine % 64) > 0) || (nLen % 32) > 0)
        nSSE ++;


#if 1
        switch(numOfNull)
        {
            case 4:
            {
                specialPos = 24;

                for(int32_t iTmp = 0; iTmp < 32; iTmp++)
                {
                    if((iTmp == 0) || (iTmp == 8) || (iTmp == 16) ||(iTmp == 24))
                    {
                        nBitVec0[iTmp] = nBits0;
                    }
                    else
                    {
                        nBitVec0[iTmp] = nBits1;
                    }
                }

                for(int32_t iTmp = 0; iTmp < 32; iTmp++)
                {
                    if((iTmp == 0) || (iTmp == 8) || (iTmp == 16) )
                    {
                        nBitVec1[iTmp] = nBits2;
                    }
                    else if((iTmp == 24) ||(iTmp == 31))
                    {
                        nBitVec1[iTmp] = nBits3;
                    }
                    else
                    {
                        nBitVec1[iTmp] = nBits4;
                    }
                }

                break;
            }

            case 12:
            {
                specialPos = 26;

                for(int32_t iTmp = 0; iTmp < 32; iTmp++)
                {
                    if((iTmp == 0) || (iTmp == 2) || (iTmp == 4) || (iTmp == 8)
                       ||(iTmp == 10) || (iTmp == 12) || (iTmp == 16) || (iTmp == 18)
                       ||(iTmp == 20) || (iTmp == 24) || (iTmp == 26) || (iTmp == 28))
                    {
                        nBitVec0[iTmp] = nBits0;
                    }
                    else
                    {
                        nBitVec0[iTmp] = nBits1;
                    }
                }

                for(int32_t iTmp = 0; iTmp < 32; iTmp++)
                {
                    if((iTmp == 0) || (iTmp == 2) || (iTmp == 4) || (iTmp == 8)
                       ||(iTmp == 10) || (iTmp == 12) || (iTmp == 16) || (iTmp == 18)
                       ||(iTmp == 20) || (iTmp == 24) || (iTmp == 28))
                    {
                        nBitVec1[iTmp] = nBits2;
                    }
                    else if((iTmp == 26) ||(iTmp == 31))
                    {
                        nBitVec1[iTmp] = nBits3;
                    }
                    else
                    {
                        nBitVec1[iTmp] = nBits4;
                    }

                }
                break;

            }

            case 20:
            {
                specialPos = 25;

                for(int32_t iTmp = 0; iTmp < 32; iTmp++)
                {
                    if((iTmp == 3) || (iTmp == 5) || (iTmp == 7) || (iTmp == 11)
                       ||(iTmp == 13) || (iTmp == 15) || (iTmp == 19) || (iTmp == 21)
                       ||(iTmp == 23) || (iTmp == 27) || (iTmp == 29) || (iTmp == 31))
                    {
                        nBitVec0[iTmp] = nBits1;
                    }
                    else
                    {
                        nBitVec0[iTmp] = nBits0;
                    }
                }

                for(int32_t iTmp = 0; iTmp < 32; iTmp++)
                {
                    if((iTmp == 3) || (iTmp == 5) || (iTmp == 7) || (iTmp == 11)
                       ||(iTmp == 13) || (iTmp == 15) || (iTmp == 19) || (iTmp == 21)
                       ||(iTmp == 23) || (iTmp == 27) || (iTmp == 29))
                    {
                        nBitVec1[iTmp] = nBits4;
                    }
                    else if( (iTmp == 25) ||(iTmp == 31) )
                    {
                        nBitVec1[iTmp] = nBits3;
                    }
                    else
                    {
                        nBitVec1[iTmp] = nBits2;
                    }
                }

                break;
            }

            break;


            case 28:
            {
                specialPos = 27;

                for(int32_t iTmp = 0; iTmp < 32; iTmp++)
                {
                    if((iTmp == 7) || (iTmp == 15) || (iTmp == 23) || (iTmp == 31))
                    {
                        nBitVec0[iTmp] = nBits1;
                    }
                    else
                    {
                        nBitVec0[iTmp] = nBits0;
                    }
                }

                for(int32_t iTmp = 0; iTmp < 32; iTmp++)
                {
                    if((iTmp == 7) || (iTmp == 15) || (iTmp == 23) )
                    {
                        nBitVec1[iTmp] = nBits4;
                    }
                    else if((iTmp == 27) || (iTmp == 31))
                    {
                        nBitVec1[iTmp] = nBits3;
                    }
                    else
                    {
                        nBitVec1[iTmp] = nBits2;
                    }
                }

                break;
            }
        }
#endif

    vconst11 = _mm256_set1_epi8 (0xFF);

    for (i=0; i<specialPos; i++)
    {
        p1 = (__m256i *) (pv1 + i*step);
        p2 = (__m256i *) (pv2 + i*step);
        p12 = (__m256i *) (v12 + i*step);

        v1_32 = _mm256_lddqu_si256 (p1++); /* v1_0, v1_1, v1_2, ... v1_15, ..., v1_31 */
        v2_32 = _mm256_lddqu_si256 (p2++); /* v2_0, v2_1, v2_2, ... v2_15, ..., v2_31 */

        v12_mix = _mm256_permute2x128_si256(v1_32, v2_32, 0x20); /* v1_0, v1_1, v1_2, v1_3, ... v1_15, v2_0, v2_1, v2_2,v2_3 ... v2_15 */
        /* v1_0, v1_1, v1_2, v1_3, v1_4, v1_5, v1_6, v1_7, v2_0, v2_1, v2_2,v2_3, v2_4, v2_5, v2_6, v2_7, v1_8, ..., v2_15 */
        v12_mix = _mm256_permute4x64_epi64(v12_mix, 0xD8);

        vtmp = _mm256_setzero_si256 ();

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf0); /* get 4-8bits from every 32bits: v1_0, v2_0, v1_0, v2_0, v1_0, v2_0 ... v1_0, v2_0, v1_1, v2_1, ..., v1_1, v2_1 */
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask); /* fetch out all the 8bits(from bit0 to bit7) in the 4 bytes (v1_0, v2_0, v1_1, v2_1), and group as 32 bytes(0 or 2^n for each)*/
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00); /* each bype compared with 0 for the 31 bytes above, if equal, then set the byte as 0xff, else set as 0x0 */
        *(tmpPtr + 0) = _mm256_movemask_epi8 (v12_perm); /* save the 7th bit for each byte, and forms as 32bits word */

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf1);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
        *(tmpPtr + 1) = _mm256_movemask_epi8 (v12_perm);

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf2);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
        *(tmpPtr + 2) = _mm256_movemask_epi8 (v12_perm);

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf3);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
        *(tmpPtr + 3) = _mm256_movemask_epi8 (v12_perm);

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf4);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
        *(tmpPtr + 4) = _mm256_movemask_epi8 (v12_perm);

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf5);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
        *(tmpPtr + 5) = _mm256_movemask_epi8 (v12_perm);

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf6);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
        *(tmpPtr + 6) = _mm256_movemask_epi8 (v12_perm);

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf7);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
        *(tmpPtr + 7) = _mm256_movemask_epi8(v12_perm);


        /* v1_0, v2_0, v1_8, v2_8,    v1_1, v2_1, v1_9, v2_9,
           v1_2, v2_2, v1_10, v2_10,    v1_3, v2_3, v1_11, v2_11,
           v1_4, v2_4, v1_12, v2_12,    v1_5, v2_5, v1_13, v2_13,
           v1_6, v2_6, v1_14, v2_14,    v1_7, v2_7, v1_15, v2_15 */
        vtmp = _mm256_xor_si256 (vtmp, vconst11);

        /* v1_0, v2_0, v1_1, v2_1, v1_2, v2_2, v1_3, v2_3,
           v1_8, v2_8, v1_9, v2_9, v1_10, v2_10, v1_11, v2_11,
           v1_4, v2_4, v1_5, v2_5, v1_6, v2_6, v1_7, v2_7,
           v1_12, v2_12,    v1_13, v2_13, v1_14, v2_14, v1_15, v2_15 */
        vtmp = _mm256_shuffle_epi8(vtmp, vshuf);

        /* v1_0, v2_0, v1_1, v2_1, v1_2, v2_2, v1_3, v2_3,
           v1_4, v2_4, v1_5, v2_5, v1_6, v2_6, v1_7, v2_7,
           v1_8, v2_8, v1_9, v2_9, v1_10, v2_10,    v1_11, v2_11,
           v1_12, v2_12,    v1_13, v2_13, v1_14, v2_14, v1_15, v2_15 */
        vtmp = _mm256_permute4x64_epi64(vtmp, 0xD8);

        _mm256_storeu_si256 (p12++, vtmp);


        //--------------------------------------------------------
        v12_mix = _mm256_permute2x128_si256(v1_32, v2_32, 0x31);
        v12_mix = _mm256_permute4x64_epi64(v12_mix, 0xD8);
        vtmp = _mm256_setzero_si256 ();

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf0); /* get 4-8bits from every 32bits: v1_0, v2_0, v1_0, v2_0, v1_0, v2_0 ... v1_0, v2_0, v1_1, v2_1, ..., v1_1, v2_1 */
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask); /* fetch out all the 8bits(from bit0 to bit7) in the 4 bytes (v1_0, v2_0, v1_1, v2_1), and group as 32 bytes(0 or 2^n for each)*/
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00); /* each bype compared with 0 for the 31 bytes above, if equal, then set the byte as 0xff, else set as 0x0 */
        *(tmpPtr + 0) = _mm256_movemask_epi8 (v12_perm); /* save the 7th bit for each byte, and forms as 32bits word */

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf1);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
        *(tmpPtr + 1) = _mm256_movemask_epi8 (v12_perm);

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf2);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
        *(tmpPtr + 2) = _mm256_movemask_epi8 (v12_perm);

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf3);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
        *(tmpPtr + 3) = _mm256_movemask_epi8 (v12_perm);

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf4);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
        *(tmpPtr + 4) = _mm256_movemask_epi8 (v12_perm);

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf5);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
        *(tmpPtr + 5) = _mm256_movemask_epi8 (v12_perm);

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf6);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
        *(tmpPtr + 6) = _mm256_movemask_epi8 (v12_perm);

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf7);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
        *(tmpPtr + 7) = _mm256_movemask_epi8 (v12_perm);

        vtmp = _mm256_xor_si256 (vtmp, vconst11);

        vtmp = _mm256_shuffle_epi8(vtmp, vshuf);

        vtmp = _mm256_permute4x64_epi64(vtmp, 0xD8);
        _mm256_storeu_si256 (p12++, vtmp);

    }

    i=specialPos;
    {
        p1 = (__m256i *) (pv1 + i*step);
        p2 = (__m256i *) (pv2 + i*step);
        p12 = (__m256i *) (v12 + i * step);

        v1_32 = _mm256_lddqu_si256 (p1++); /* v1_0, v1_1, v1_2, ... v1_15, ..., v1_31 */
        v2_32 = _mm256_lddqu_si256 (p2++); /* v2_0, v2_1, v2_2, ... v2_15, ..., v2_31 */

        v12_mix = _mm256_permute2x128_si256(v2_32, v1_32, 0x20); /* v1_0, v1_1, v1_2, v1_3, ... v1_15, v2_0, v2_1, v2_2,v2_3 ... v2_15 */
        v12_mix = _mm256_permute4x64_epi64(v12_mix, 0xD8);

        vtmp = _mm256_setzero_si256 ();

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf0); /* get 4-8bits from every 32bits: v1_0, v2_0, v1_0, v2_0, v1_0, v2_0 ... v1_0, v2_0, v1_1, v2_1, ..., v1_1, v2_1 */
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask); /* fetch out all the 8bits(from bit0 to bit7) in the 4 bytes (v1_0, v2_0, v1_1, v2_1), and group as 32 bytes(0 or 2^n for each)*/
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00); /* each bype compared with 0 for the 31 bytes above, if equal, then set the byte as 0xff, else set as 0x0 */
        *(tmpPtr + 0) = _mm256_movemask_epi8 (v12_perm); /* save the 7th bit for each byte, and forms as 32bits word */

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf1);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
        *(tmpPtr + 1) = _mm256_movemask_epi8 (v12_perm);

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf2);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
        *(tmpPtr + 2) = _mm256_movemask_epi8 (v12_perm);

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf3);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
        *(tmpPtr + 3) = _mm256_movemask_epi8 (v12_perm);

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf4);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
        *(tmpPtr + 4) = _mm256_movemask_epi8 (v12_perm);

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf5);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
        *(tmpPtr + 5) = _mm256_movemask_epi8 (v12_perm);

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf6);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
        *(tmpPtr + 6) = _mm256_movemask_epi8 (v12_perm);

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf7);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
		*(tmpPtr + 7) = _mm256_movemask_epi8(v12_perm);

        vtmp = _mm256_xor_si256 (vtmp, vconst11);

        vtmp = _mm256_shuffle_epi8(vtmp, vshuf);

        vtmp = _mm256_permute4x64_epi64(vtmp, 0xD8);

        _mm256_storeu_si256 (p12++, vtmp);


        //--------------------------------------------------------
        v12_mix = _mm256_permute2x128_si256(v2_32, v1_32, 0x31);
        v12_mix = _mm256_permute4x64_epi64(v12_mix, 0xD8);
        vtmp = _mm256_setzero_si256 ();

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf0); /* get 4-8bits from every 32bits: v1_0, v2_0, v1_0, v2_0, v1_0, v2_0 ... v1_0, v2_0, v1_1, v2_1, ..., v1_1, v2_1 */
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask); /* fetch out all the 8bits(from bit0 to bit7) in the 4 bytes (v1_0, v2_0, v1_1, v2_1), and group as 32 bytes(0 or 2^n for each)*/
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00); /* each bype compared with 0 for the 31 bytes above, if equal, then set the byte as 0xff, else set as 0x0 */
        *(tmpPtr + 0) = _mm256_movemask_epi8 (v12_perm); /* save the 7th bit for each byte, and forms as 32bits word */

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf1);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
        *(tmpPtr + 1) = _mm256_movemask_epi8 (v12_perm);

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf2);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
        *(tmpPtr + 2) = _mm256_movemask_epi8 (v12_perm);

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf3);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
        *(tmpPtr + 3) = _mm256_movemask_epi8 (v12_perm);

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf4);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
        *(tmpPtr + 4) = _mm256_movemask_epi8 (v12_perm);

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf5);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
        *(tmpPtr + 5) = _mm256_movemask_epi8 (v12_perm);

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf6);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
        *(tmpPtr + 6) = _mm256_movemask_epi8 (v12_perm);

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf7);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
        *(tmpPtr + 7) = _mm256_movemask_epi8 (v12_perm);

        vtmp = _mm256_xor_si256 (vtmp, vconst11);

        vtmp = _mm256_shuffle_epi8(vtmp, vshuf);

        vtmp = _mm256_permute4x64_epi64(vtmp, 0xD8);
        _mm256_storeu_si256 (p12++, vtmp);

    }

    for (i=specialPos+1; i<32; i++)
    {
        p1 = (__m256i *) (pv1 + i*step);
        p2 = (__m256i *) (pv2 + i*step);
		p12 = (__m256i *) (v12 + i * step);

        v1_32 = _mm256_lddqu_si256 (p1++); /* v1_0, v1_1, v1_2, ... v1_15, ..., v1_31 */
        v2_32 = _mm256_lddqu_si256 (p2++); /* v2_0, v2_1, v2_2, ... v2_15, ..., v2_31 */

        v12_mix = _mm256_permute2x128_si256(v1_32, v2_32, 0x20); /* v1_0, v1_1, v1_2, v1_3, ... v1_15, v2_0, v2_1, v2_2,v2_3 ... v2_15 */
        v12_mix = _mm256_permute4x64_epi64(v12_mix, 0xD8);

        vtmp = _mm256_setzero_si256 ();

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf0); /* get 4-8bits from every 32bits: v1_0, v2_0, v1_0, v2_0, v1_0, v2_0 ... v1_0, v2_0, v1_1, v2_1, ..., v1_1, v2_1 */
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask); /* fetch out all the 8bits(from bit0 to bit7) in the 4 bytes (v1_0, v2_0, v1_1, v2_1), and group as 32 bytes(0 or 2^n for each)*/
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00); /* each bype compared with 0 for the 31 bytes above, if equal, then set the byte as 0xff, else set as 0x0 */
        *(tmpPtr + 0) = _mm256_movemask_epi8 (v12_perm); /* save the 7th bit for each byte, and forms as 32bits word */

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf1);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
        *(tmpPtr + 1) = _mm256_movemask_epi8 (v12_perm);

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf2);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
        *(tmpPtr + 2) = _mm256_movemask_epi8 (v12_perm);

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf3);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
        *(tmpPtr + 3) = _mm256_movemask_epi8 (v12_perm);

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf4);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
        *(tmpPtr + 4) = _mm256_movemask_epi8 (v12_perm);

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf5);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
        *(tmpPtr + 5) = _mm256_movemask_epi8 (v12_perm);

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf6);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
        *(tmpPtr + 6) = _mm256_movemask_epi8 (v12_perm);

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf7);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
		*(tmpPtr + 7) = _mm256_movemask_epi8(v12_perm);

       vtmp = _mm256_xor_si256 (vtmp, vconst11);

       vtmp = _mm256_shuffle_epi8(vtmp, vshuf);

       vtmp = _mm256_permute4x64_epi64(vtmp, 0xD8);
        _mm256_storeu_si256 (p12++, vtmp);


        //--------------------------------------------------------
        v12_mix = _mm256_permute2x128_si256(v1_32, v2_32, 0x31);
        v12_mix = _mm256_permute4x64_epi64(v12_mix, 0xD8);
        vtmp = _mm256_setzero_si256 ();

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf0); /* get 4-8bits from every 32bits: v1_0, v2_0, v1_0, v2_0, v1_0, v2_0 ... v1_0, v2_0, v1_1, v2_1, ..., v1_1, v2_1 */
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask); /* fetch out all the 8bits(from bit0 to bit7) in the 4 bytes (v1_0, v2_0, v1_1, v2_1), and group as 32 bytes(0 or 2^n for each)*/
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00); /* each bype compared with 0 for the 31 bytes above, if equal, then set the byte as 0xff, else set as 0x0 */
        *(tmpPtr + 0) = _mm256_movemask_epi8 (v12_perm); /* save the 7th bit for each byte, and forms as 32bits word */

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf1);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
        *(tmpPtr + 1) = _mm256_movemask_epi8 (v12_perm);

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf2);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
        *(tmpPtr + 2) = _mm256_movemask_epi8 (v12_perm);

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf3);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
        *(tmpPtr + 3) = _mm256_movemask_epi8 (v12_perm);

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf4);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
        *(tmpPtr + 4) = _mm256_movemask_epi8 (v12_perm);

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf5);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
        *(tmpPtr + 5) = _mm256_movemask_epi8 (v12_perm);

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf6);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
        *(tmpPtr + 6) = _mm256_movemask_epi8 (v12_perm);

        v12_perm = _mm256_shuffle_epi8 (v12_mix, vshuf7);
        v12_perm = _mm256_and_si256 (v12_perm, vbitmask);
        v12_perm = _mm256_cmpeq_epi8 (v12_perm, vconst00);
        *(tmpPtr + 7) = _mm256_movemask_epi8 (v12_perm);

        vtmp = _mm256_xor_si256 (vtmp, vconst11);

        vtmp = _mm256_shuffle_epi8(vtmp, vshuf);

        vtmp = _mm256_permute4x64_epi64(vtmp, 0xD8);
        _mm256_storeu_si256 (p12++, vtmp);

    }

    if(nLen>6144)
    {
        int8_t tmpV1 = (*(pv1+7*step+24)) &0x1;
        int8_t tmpV2 = (*(pv2+7*step+24)) &0x1;

        v12[7*step+24] = (tmpV2<<1) + tmpV1;

        tmpV1 = (*(pv1+15*step+24)) &0x1;
        tmpV2 = (*(pv2+15*step+24)) &0x1;
        v12[15*step+24] = (tmpV2<<1) + tmpV1;

        tmpV1 = (*(pv1+23*step+24)) &0x1;
        tmpV2 = (*(pv2+23*step+24)) &0x1;
        v12[23*step+24] = (tmpV2<<1) + tmpV1;

        tmpV2 = (*(pv2+27*step+24)) &0x1;
        v12[27*step+24] = tmpV2;

        tmpV1 = (*(pv1+31*step+24)) &0x1;
        v12[31*step+24] = tmpV1;

        /*int8_t tmpV1 = (*(pv1+0*step+24)) &0x1;
        int8_t tmpV2 = (*(pv2+0*step+24)) &0x1;

        v12[0*step+24] = (tmpV2<<1) + tmpV1;

        tmpV1 = (*(pv1+8*step+24)) &0x1;
        tmpV2 = (*(pv2+8*step+24)) &0x1;
        v12[8*step+24] = (tmpV2<<1) + tmpV1;

        tmpV1 = (*(pv1+16*step+24)) &0x1;
        tmpV2 = (*(pv2+16*step+24)) &0x1;
        v12[16*step+24] = (tmpV2<<1) + tmpV1;

        tmpV2 = (*(pv2+31*step+24)) &0x1;
        v12[31*step+24] = tmpV2;

        tmpV1 = (*(pv1+24*step+24)) &0x1;
        v12[31*step+24] = tmpV1;*/

    }


#if 1
/* Calling IPP bit copy is worst than use SSE instrinsic for bit collection */
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
                vtmps = _mm_loadu_si128((__m128i*)(pv0 + iTmp * step + 0));
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
                    _mm_storel_epi64((__m128i *)(pOut+cnt), vout2);
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
                vtmps = _mm_loadu_si128((__m128i*)(ps12 + iTmp * step + 0));
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
                    _mm_storel_epi64((__m128i *)(pOut+cnt), vout2);
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
                    _mm_storel_epi64((__m128i *)(pOut+cnt), vout2);
                    cnt += 8;
                    nBitst -= 64;

                    vout = vout3;
                    _mm_storel_epi64((__m128i *)(pOut+cnt), vout);
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
                    _mm_storel_epi64((__m128i *)(pOut+cnt), vout);

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
                vtmps = _mm_loadu_si128((__m128i*)(pv0 + iTmp * step + 0));
                vout1 = _mm_slli_epi64(vtmps, nBitst);
                vout2 = _mm_or_si128(vout1, vout);   /* the 1st 64bit data */
                _mm_storel_epi64((__m128i *)(pOut+cnt), vout2);
                cnt += 8;
                vout = _mm_srli_epi64(vtmps, 64-nBitst);

                vtmps = _mm_loadu_si128((__m128i*)(pv0 + iTmp * step + 8)); /* the 3rd 64bit, includes nBitVec0[iTmp] useful bits */
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
                    _mm_storel_epi64((__m128i *)(pOut+cnt), vout2);
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
                vtmps = _mm_loadu_si128((__m128i*)(ps12 + iTmp * step + 0));
                vout1 = _mm_slli_epi64(vtmps, nBitst);
                vout2 = _mm_or_si128(vout1, vout);   /* the 1st 64 bit */
                _mm_storel_epi64((__m128i *)(pOut+cnt), vout2);
                cnt += 8;
                vout = _mm_srli_epi64(vtmps, 64-nBitst);

                vtmps = _mm_loadu_si128((__m128i*)(ps12 + iTmp * step + 4));
                vout1 = _mm_slli_epi64(vtmps, nBitst);
                vout2 = _mm_or_si128(vout1, vout);   /* the 2nd 64 bit */
                _mm_storel_epi64((__m128i *)(pOut+cnt), vout2);
                cnt += 8;
                vout = _mm_srli_epi64(vtmps, 64-nBitst);

                vtmps = _mm_loadu_si128((__m128i*)(ps12 + iTmp * step + 8));
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
                    _mm_storel_epi64((__m128i *)(pOut+cnt), vout2);  /* collect 1 whole 64 bit, adjust nBits */
                    cnt += 8;
                    nBitst -= 64;
                    /*  vout = _mm_srli_si128(vout2, 64); */
                    vout = _mm_shuffle_epi32(vout2, 78);
                    vout1 = _mm_srli_epi64(vconstF, 64 - nBitst);
                    vout = _mm_and_si128(vout1, vout);
                }
                else   /* N < 64 * 3 */
                {
                    _mm_storel_epi64((__m128i *)(pOut+cnt), vout2);  /* collect 2 whole 64 bit, adjust nBits */
                    cnt += 8;
                    nBitst -= 64;

                    vout = _mm_shuffle_epi32(vout2, 78);
                    _mm_storel_epi64((__m128i *)(pOut+cnt), vout);
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
                    _mm_storel_epi64((__m128i *)(pOut+cnt), vout);

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
                vtmps = _mm_loadu_si128((__m128i*)(pv0 + iTmp * step + 0));
                vout1 = _mm_slli_epi64(vtmps, nBitst);
                vout2 = _mm_or_si128(vout1, vout);   /* 1st 64 bit */
                _mm_storel_epi64((__m128i *)(pOut+cnt), vout2);
                cnt += 8;
                vout = _mm_srli_epi64(vtmps, 64-nBitst);

                vtmps = _mm_loadu_si128((__m128i*)(pv0 + iTmp * step + 8));
                vout1 = _mm_slli_epi64(vtmps, nBitst);
                vout2 = _mm_or_si128(vout1, vout);   /* 2nd 64 bit */
                _mm_storel_epi64((__m128i *)(pOut+cnt), vout2);
                cnt += 8;
                vout = _mm_srli_epi64(vtmps, 64-nBitst);

                vtmps = _mm_loadu_si128((__m128i*)(pv0 + iTmp * step + 16)); /* 3rd 64 bit */
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
                    _mm_storel_epi64((__m128i *)(pOut+cnt), vout2);
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
                 vtmps = _mm_loadu_si128((__m128i*)(ps12 + iTmp * step + 0));
                 vout1 = _mm_slli_epi64(vtmps, nBitst);
                 vout2 = _mm_or_si128(vout1, vout);   /* 1st 64 bit */
                 _mm_storel_epi64((__m128i *)(pOut+cnt), vout2);
                 cnt += 8;
                 vout = _mm_srli_epi64(vtmps, 64-nBitst);

                vtmps = _mm_loadu_si128((__m128i*)(ps12 + iTmp * step + 4));
                vout1 = _mm_slli_epi64(vtmps, nBitst);
                vout2 = _mm_or_si128(vout1, vout);   /* 2nd 64 bit */
                _mm_storel_epi64((__m128i *)(pOut+cnt), vout2);
                cnt += 8;
                vout = _mm_srli_epi64(vtmps, 64-nBitst);

                vtmps = _mm_loadu_si128((__m128i*)(ps12 + iTmp * step + 8));
                vout1 = _mm_slli_epi64(vtmps, nBitst);
                vout2 = _mm_or_si128(vout1, vout);   /* 3rd 64 bit */
                _mm_storel_epi64((__m128i *)(pOut+cnt), vout2);
                cnt += 8;
                vout = _mm_srli_epi64(vtmps, 64-nBitst);

                vtmps = _mm_loadu_si128((__m128i*)(ps12 + iTmp * step + 12));
                vout1 = _mm_slli_epi64(vtmps, nBitst);
                vout2 = _mm_or_si128(vout1, vout);   /* 4th 64 bit */
                _mm_storel_epi64((__m128i *)(pOut+cnt), vout2);
                cnt += 8;
                vout = _mm_srli_epi64(vtmps, 64-nBitst);

                vtmps = _mm_loadu_si128((__m128i*)(ps12 + iTmp * step + 16));
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
                    _mm_storel_epi64((__m128i *)(pOut+cnt), vout2);
                    cnt += 8;
                    nBitst -= 64;
                    /*  vout = _mm_srli_si128(vout2, 64); */
                    vout = _mm_shuffle_epi32(vout2, 78);
                    vout1 = _mm_srli_epi64(vconstF, 64 - nBitst);
                    vout = _mm_and_si128(vout1, vout);
                }
                else   /* N < 64 * 3 */
                {
                    _mm_storel_epi64((__m128i *)(pOut+cnt), vout2);
                    cnt += 8;
                    nBitst -= 64;

                    vout = _mm_shuffle_epi32(vout2, 78);
                    _mm_storel_epi64((__m128i *)(pOut+cnt), vout);
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
                    _mm_storel_epi64((__m128i *)(pOut+cnt), vout);

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
                vtmps = _mm_loadu_si128((__m128i*)(pv0 + iTmp * step + 0));
                vout1 = _mm_slli_epi64(vtmps, nBitst);
                vout2 = _mm_or_si128(vout1, vout);   /* 1st  64 bit */
                _mm_storel_epi64((__m128i *)(pOut+cnt), vout2);
                cnt += 8;
                vout = _mm_srli_epi64(vtmps, 64-nBitst);

                vtmps = _mm_loadu_si128((__m128i*)(pv0 + iTmp * step + 8));
                vout1 = _mm_slli_epi64(vtmps, nBitst);
                vout2 = _mm_or_si128(vout1, vout);   /* 2nd 64 bit */
                _mm_storel_epi64((__m128i *)(pOut+cnt), vout2);
                cnt += 8;
                vout = _mm_srli_epi64(vtmps, 64-nBitst);

                vtmps = _mm_loadu_si128((__m128i*)(pv0 + iTmp * step + 16));
                vout1 = _mm_slli_epi64(vtmps, nBitst);
                vout2 = _mm_or_si128(vout1, vout);   /* 3rd 64 bit */
                _mm_storel_epi64((__m128i *)(pOut+cnt), vout2);
                cnt += 8;
                vout = _mm_srli_epi64(vtmps, 64-nBitst);

                vtmps = _mm_loadu_si128((__m128i*)(pv0 + iTmp * step + 24)); /* 4th 64 bit contains nBitVec0[iTmp] useful bits */
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
                    _mm_storel_epi64((__m128i *)(pOut+cnt), vout2);
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
                vtmps = _mm_loadu_si128((__m128i*)(ps12 + iTmp * step + 0));
                vout1 = _mm_slli_epi64(vtmps, nBitst);
                vout2 = _mm_or_si128(vout1, vout);   /* 1st 64 bit */
                _mm_storel_epi64((__m128i *)(pOut+cnt), vout2);
                cnt += 8;
                vout = _mm_srli_epi64(vtmps, 64-nBitst);

                vtmps = _mm_loadu_si128((__m128i*)(ps12 + iTmp * step + 4));
                vout1 = _mm_slli_epi64(vtmps, nBitst);
                vout2 = _mm_or_si128(vout1, vout);   /* 2nd 64 bit */
                _mm_storel_epi64((__m128i *)(pOut+cnt), vout2);
                cnt += 8;
                vout = _mm_srli_epi64(vtmps, 64-nBitst);

                vtmps = _mm_loadu_si128((__m128i*)(ps12 + iTmp * step + 8));
                vout1 = _mm_slli_epi64(vtmps, nBitst);
                vout2 = _mm_or_si128(vout1, vout);   /* 3rd 64 bit */
                _mm_storel_epi64((__m128i *)(pOut+cnt), vout2);
                cnt += 8;
                vout = _mm_srli_epi64(vtmps, 64-nBitst);

                vtmps = _mm_loadu_si128((__m128i*)(ps12 + iTmp * step + 12));
                vout1 = _mm_slli_epi64(vtmps, nBitst);
                vout2 = _mm_or_si128(vout1, vout);   /* 4th 64 bit */
                _mm_storel_epi64((__m128i *)(pOut+cnt), vout2);
                cnt += 8;
                vout = _mm_srli_epi64(vtmps, 64-nBitst);

                vtmps = _mm_loadu_si128((__m128i*)(ps12 + iTmp * step + 16));
                vout1 = _mm_slli_epi64(vtmps, nBitst);
                vout2 = _mm_or_si128(vout1, vout);   /* 5th 64 bit */
                _mm_storel_epi64((__m128i *)(pOut+cnt), vout2);
                cnt += 8;
                vout = _mm_srli_epi64(vtmps, 64-nBitst);

                vtmps = _mm_loadu_si128((__m128i*)(ps12 + iTmp * step + 20));
                vout1 = _mm_slli_epi64(vtmps, nBitst);
                vout2 = _mm_or_si128(vout1, vout);   /* 6th 64 bit */
                _mm_storel_epi64((__m128i *)(pOut+cnt), vout2);
                cnt += 8;
                vout = _mm_srli_epi64(vtmps, 64-nBitst);

                vtmps = _mm_loadu_si128((__m128i*)(ps12 + iTmp * step + 24));
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
                    _mm_storel_epi64((__m128i *)(pOut+cnt), vout2);
                    cnt += 8;
                    nBitst -= 64;
                    vout = _mm_shuffle_epi32(vout2, 78);
                    vout1 = _mm_srli_epi64(vconstF, 64 - nBitst);
                    vout = _mm_and_si128(vout1, vout);
                }
                else   /* N <  64 * 3 */
                {
                    _mm_storel_epi64((__m128i *)(pOut+cnt), vout2);
                    cnt += 8;
                    nBitst -= 64;

                    vout = _mm_shuffle_epi32(vout2, 78);
                    _mm_storel_epi64((__m128i *)(pOut+cnt), vout);
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
                    _mm_storel_epi64((__m128i *)(pOut+cnt), vout);

            }

             break;
        }
    default :
        printf("nSSE error!\n");
}
#endif

    return;
}


//-------------------------------------------------------------------------------------------
/** @ingroup group_lte_source_phy_fec_enc
 *
 *  @param[in]   E - number of bits to be selected
 *  @param[in]   k0 - the start bit to be selected
 *  @param[in]   Ncb - the total selection bits, including null bits
 *  @param[in]   Kidx - Postion in interleave 188 table
 *  @param[in]   nLen - Length of input data
 *  @param[in]   totalNullBits - the total null bits
 *  @param[in]   pIn - pointer to collection bits
 *  @param[out]  pOut - pointer to final circular buffer output
 *
 *  @return  no return value
 *
 *  @description
 *  Function implements bit collection as described in 3GPP TS 36.211 Rel 8 Sec.5.1.4.1.2
 *
**/
//-------------------------------------------------------------------------------------------
static int32_t
bit_selection_complete_avx2(int32_t E, int32_t k0, int32_t Ncb,int32_t Kidx, int32_t Kw, uint32_t totalNullBits, uint8_t *pIn, uint8_t *pOut )
{
    int32_t i, tmp, j;
    /* Kidx is the type of codeblock length in 188 selection */
    int32_t k0_m;
    int32_t Num_NULL = totalNullBits;//g_nNum_NULL[Kidx];
    int32_t ni, nj;
    k0_m = k0 % Ncb;
    for (ni = 0; ni < Num_NULL; ni++)
    {
        if (g_nIndex_NULL[Kidx][ni] > k0_m)
            break;
    }
    int32_t k0_n = k0_m - ni;

    if(Ncb != Kw)
    {
        /* nj stands for the NULL number between 1 ~ Ncb  */
        for (nj=Num_NULL-1; nj>0; nj--)
        {
            if (g_nIndex_NULL[Kidx][nj] < Ncb)
                break;
        }
        /* update real total number of null bits */
        totalNullBits = nj + 1;
    }

    int32_t Ncb_n = Ncb - totalNullBits;
    /* nj stands for the NULL number between 1 ~ Ncb  */

    int32_t t_beg = k0_n;

    int32_t offsetBytes = (t_beg>>3);
    int32_t srcBitOffset = t_beg - (offsetBytes<<3);
    int32_t dstBitOffset;
    int32_t totalSelcBits;

    int32_t numOfBitsLeft = Ncb_n - t_beg;//(tLen <<3);

    if(numOfBitsLeft>E)
    {
         ippsCopyBE_1u ((Ipp8u*) (pIn + offsetBytes) , srcBitOffset, (Ipp8u* )pOut, 0,  E);
    }
    else
    {
        ippsCopyBE_1u ((Ipp8u*) (pIn + offsetBytes) , srcBitOffset, (Ipp8u* )pOut, 0,  numOfBitsLeft);
        /* update destination offset */
        offsetBytes = (numOfBitsLeft>>3);
        dstBitOffset = numOfBitsLeft - (offsetBytes<<3);

        if((Ncb - totalNullBits) >= (E - numOfBitsLeft))
        {
            ippsCopyBE_1u ((Ipp8u*) (pIn) , 0, (Ipp8u* )(pOut+offsetBytes), dstBitOffset,  E - numOfBitsLeft);
        }
        else
        {
            totalSelcBits = numOfBitsLeft;
            while(totalSelcBits<E)
            {
                ippsCopyBE_1u ((Ipp8u*) (pIn) , 0, (Ipp8u* )(pOut+offsetBytes), dstBitOffset,  (Ncb - totalNullBits));
                offsetBytes += (Ncb - totalNullBits)>>3;
                dstBitOffset += Ncb - totalNullBits - (((Ncb - totalNullBits)>>3)<<3);
                if(dstBitOffset>=8)
                {
                    offsetBytes++;
                    dstBitOffset -= 8;
                }
                totalSelcBits += (Ncb - totalNullBits);
            }
        }

    }

    return 0;

}

//-------------------------------------------------------------------------------------------
/** @ingroup group_lte_source_phy_fec_enc
 *
 *  @param[in]   r - Currently value of code block
 *  @param[in]   C - Total number of code block
 *  @param[in]   direction - DL = 0 or UL = 1
 *  @param[in]   Nsoft - total bits related to client grade
 *  @param[in]   KMIMO - 1 or 2 related to MIMO type
 *  @param[in]   MDL_HARQ - Maximum number of HARQ
 *  @param[in]   G - Length of output bits
 *  @param[in]   NL - Number of layer
 *  @param[in]   Qm - Modulation type
 *  @param[in]   rvidx- redundancy version number for this transmission (0,1,2,3)
 *  @param[in]   bypass_rvidx - If set ignore rvidx and set k0 to 0
 *  @param[in]   Kidx - Postion in interleave 188 table
 *  @param[in]   nLen - Length of input data
 *  @param[in]   pd0 - pointer to buffer containing systematic bits
 *  @param[in]   pd1 - pointer to buffer containing parity1 bits
 *  @param[in]   pd2- pointer to buffer containing parity2 bits
 *  @param[out]  pOutput- pointer to rate matching output
 *  @param[in]   outputLen- Lenth after ratematching
 *  @param[out]  plen_table - store rate matching output length for this code block
 *
 *  @return  0 on success, negative on failure
 *
 *  @description
 *  Turbo Encoder Rate Matching as described in
 *  3GPP (LTE) standard TS 36.212 Section 5.1.4, which implemented with AVX2 instruction set, aimed to
 *            use in LTE system.
 *
**/
//-------------------------------------------------------------------------------------------


int32_t rate_matching_turbo_lte_avx2(int32_t r, int32_t C, int8_t direction, int32_t Nsoft, int32_t KMIMO,
    int32_t MDL_HARQ, int32_t G,int NL, int32_t Qm, int32_t rvidx, int8_t bypass_rvidx, int32_t Kidx, const int32_t nLen,
    uint8_t * pd0, uint8_t * pd1, uint8_t * pd2, uint8_t * pOutput, int32_t outputLen)
{
    uint8_t v0[LEN]={0}, v1[LEN]={0}, v2[LEN]={0};
    uint8_t outTmp0[6144] = {0};
    uint8_t outTmp1[6144] = {0};

    uint32_t selectBytes = 0;
    uint32_t nRound = (nLen>>10) + 1;
    int32_t k0;
    int32_t nCol = 32;
    int32_t nRow = nLen / 32 + 1;
    int32_t Kw = nCol * nRow * 3;
    uint32_t numOfNull = nCol*nRow - nLen;
    int32_t NIR = Nsoft / (KMIMO * (MIN(MDL_HARQ, MLIMIT)));
    int32_t Ncb = Kw;       //  1, DL  //  0, UL

    if (direction == 1)
        Ncb = MIN((NIR / C), Kw);
    else if (direction == 0)
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

    int32_t temp3 = Ncb / (8 * nRow);

    if(Ncb>(8 * nRow * temp3))
        temp3++;

    if (bypass_rvidx == 1)
        k0 = 0;
    else
        k0 =  nRow * (2 * temp3 * rvidx + 2);
//=================================================================

    subblock_interleaver_d0d1_avx2(nLen, nRound, numOfNull, pd0, v0);
    subblock_interleaver_d0d1_avx2(nLen, nRound, numOfNull, pd1, v1);
    subblock_interleaver_d2_avx2(nLen, nRound, numOfNull, pd2, v2);

    bit_collection_avx2(nLen, numOfNull, v0, v1, v2, outTmp0);

    bit_selection_complete_avx2(E, k0, Ncb, Kidx, Kw, 3 * numOfNull, outTmp0, outTmp1);
    bit_reverse(outTmp1, pOutput, E / 8 + 1);

    return 0;
}
#else
int32_t rate_matching_turbo_lte_avx2(int32_t r, int32_t C, int8_t direction, int32_t Nsoft, int32_t KMIMO,
    int32_t MDL_HARQ, int32_t G,int NL, int32_t Qm, int32_t rvidx, int32_t Kidx, const int32_t nLen,
    uint8_t * pd0, uint8_t * pd1, uint8_t * pd2, uint8_t * pOutput, int32_t outputLen)
{
    printf("bblib_rate_matching requires AVX2 ISA support to run\n");
    return(-1);
}
int32_t bblib_rate_match_dl_avx2(const struct bblib_rate_match_dl_request *request,
        struct bblib_rate_match_dl_response *response)
{
    printf("bblib_rate_matching requires AVX2 ISA support to run\n");
    return(-1);
}
#endif
