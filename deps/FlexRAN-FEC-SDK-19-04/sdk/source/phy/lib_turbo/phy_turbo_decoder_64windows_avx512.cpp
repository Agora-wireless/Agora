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

/*******************************************************************************
*  @file phy_turbo_decoder_64windows_avx512.cpp
*  @brief this file performs the turbo Decoder when CW size is multiple of 64.
*  Trellis diagram:        0-00->0
                           0-11->4
                           1-00->4
                           1-11->0
                           2-01->5
                           2-10->1
                           3-01->1
                           3-10->5
                           4-01->2
                           4-10->6
                           5-01->6
                           5-10->2
                           6-00->7
                           6-11->3
                           7-00->3
                           7-11->7

*
*******************************************************************************/

/* turbo encoder and decoder function */
#include <cstdint>
#include <cstdio>

#include "immintrin.h"

#include "phy_crc.h"
#include "phy_turbo.h"
#include "phy_turbo_internal.h"
#if defined (_BBLIB_AVX512_)
static __m512i TD_constant512 = _mm512_setzero_si512();

#define TURBO_OFFSET (-80)
static __m512i TD_Offset = _mm512_set1_epi8(TURBO_OFFSET);
static __m512i signBitMask = _mm512_set1_epi8(0x80);
static __m512i tailBitMask = _mm512_set1_epi8(0xFC);
static __m512i init_alpha = _mm512_set_epi8 (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
											 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
											 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -128);

static __m512i k_alpha = _mm512_set_epi8(15, 14, 13, 12, 11, 10,  9,  8,  7,  6,  5,  4,  3,  2,  1,  0,
                                         15, 14, 13, 12, 11, 10,  9,  8,  7,  6,  5,  4,  3,  2,  1,  0,
                                         15, 14, 13, 12, 11, 10,  9,  8,  7,  6,  5,  4,  3,  2,  1,  0,
                                         14, 13, 12, 11, 10,  9,  8,  7,  6,  5,  4,  3,  2,  1,  0, 15);
static __m512i k_beta = _mm512_set_epi8( 0, 15, 14, 13, 12, 11, 10,  9,  8,  7,  6,  5,  4,  3,  2,  1,
                                        15, 14, 13, 12, 11, 10,  9,  8,  7,  6,  5,  4,  3,  2,  1,  0,
                                        15, 14, 13, 12, 11, 10,  9,  8,  7,  6,  5,  4,  3,  2,  1,  0,
                                        15, 14, 13, 12, 11, 10,  9,  8,  7,  6,  5,  4,  3,  2,  1,  0);
static __m512i copy_beta = _mm512_setzero_si512();

static __m256i TD_constant_0_16 = _mm256_set_epi8(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
                                                  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15);
static __m256i TD_vshuf1_BitTranspose_16windows = _mm256_set_epi8(1, 3, 5, 7, 9, 11, 13, 15, 0, 2, 4, 6, 8, 10, 12, 14,
                                                                  1, 3, 5, 7, 9, 11, 13, 15, 0, 2, 4, 6, 8, 10, 12, 14);
static __m128i TD_vshuf2_BitTranspose_16windows = _mm_setr_epi8(7, 6, 5, 4, 3, 2, 1, 0, 15, 14, 13, 12, 11, 10, 9, 8);


/* update the alpha for the next iter.           *
 * input            alpha0  alpha1 alpha2 alpha3 *
 * output           alpha3 alpha0 alpha1 alpha2  */
#define SHUFFLE_ALPHA(in, out)\
{\
    out = _mm512_shuffle_i64x2 (in, in, 0x93);\
    out = _mm512_shuffle_epi8 (out, k_alpha);\
    out = _mm512_mask_blend_epi8 (0x1, out, init_alpha);\
}

/* update the beta for the next iter.           *
 * input  beta0 beta1 beta2 beta3               *
 * output beta1 beta2 beta3 beta0               */
#define SHUFFLE_BETA(value, in, out)\
{\
    copy_beta = _mm512_set1_epi8(value);\
    out = _mm512_shuffle_i64x2 (in, in, 0x39);\
    out = _mm512_shuffle_epi8 (out, k_beta);\
    out = _mm512_mask_blend_epi8 (0x8000000000000000, out, copy_beta);\
}

#define CALC_ALPHA_BETA(info0, info1, in0, in1, tmp0, tmp1, out0, out1)\
{\
    tmp0 = _mm512_adds_epi8(info0, in0);\
    tmp1 = _mm512_adds_epi8(info1, in1);\
    out1 = _mm512_max_epi8(tmp0, tmp1);\
    out1 = _mm512_subs_epi8(out1, out0);\
}

#define CALC_DELTA(alpha0, alpha1, beta0, in0, in1, tmpDelta0, tmpDelta1)\
{\
    tmpDelta0 = _mm512_adds_epi8(alpha0, in0);\
    tmpDelta0 = _mm512_adds_epi8(tmpDelta0, beta0);\
    tmpDelta1 = _mm512_adds_epi8(alpha1, in1);\
    tmpDelta1 = _mm512_adds_epi8(tmpDelta1, beta0);\
}

#define SCALE_075(a)\
{\
    __m512i signBit;\
    __m512i signBit1;\
    signBit = _mm512_and_si512(a, signBitMask);\
    signBit1 = _mm512_srli_epi16(signBit, 1);\
    signBit = _mm512_or_si512(signBit, signBit1);\
    signBit1 = _mm512_and_si512(a, tailBitMask);\
    signBit1 = _mm512_srli_epi16(signBit1, 2);\
    signBit1 = _mm512_or_si512(signBit1, signBit);\
    a = _mm512_subs_epi8(a, signBit1);\
}

int32_t SISO_64windows(int8_t *OutputAddress, int8_t *InputAddress,
                       int32_t *InterleaverInterRowAddr, int8_t *InterleaverIntraRowPattern, int32_t *InterleaverIntraRowPatSel,
                       int8_t *Tempalpha_sigma, __m512i *initalpha, __m512i* initbeta, int8_t *tailbeta, int32_t WindowSize,
                       uint16_t *pCodeBlockBits);

void BitTranspose_16windows_new(int32_t K, uint16_t * pin, uint8_t * pout);

inline void init_beta_comp(int8_t *a, int8_t *c, __m512i* initbeta, int8_t *tailbeta)
{

    int8_t beta[4];

    /* according to state transition */
    beta[1] = a[1] + c[1];
    beta[2] = a[2] + c[2];
    beta[3] = beta[2] + a[1];

    tailbeta[1] = a[0] + c[0];               /*xk[0]+zk[0]*/
    tailbeta[2] = beta[1] + a[0];            /*/xk[0]+xk[1]+zk[1]*/
    tailbeta[3] = beta[1] + c[0];            /*xk[0]+xk[1]+zk[1]*/
    tailbeta[4] = beta[3] + c[0];            /*zk[0]+xk[1]+xk[2]+zk[2]*/
    tailbeta[5] = beta[3] + a[0];            /*xk[0]+xk[1]+xk[2]+zk[2]*/
    tailbeta[7] = beta[2] + c[1];            /*zk[1]+xk[2]+zk[2]*/
    tailbeta[6] = tailbeta[7] + tailbeta[1]; /*xk[0]+zk[0]+zk[1]+xk[2]+zk[2]*/

    copy_beta = _mm512_set1_epi8(tailbeta[1]);
    initbeta[1] = _mm512_mask_blend_epi8 (0x8000000000000000, initbeta[1], copy_beta);
    copy_beta = _mm512_set1_epi8(tailbeta[2]);
    initbeta[2] = _mm512_mask_blend_epi8 (0x8000000000000000, initbeta[2], copy_beta);
    copy_beta = _mm512_set1_epi8(tailbeta[3]);
    initbeta[3] = _mm512_mask_blend_epi8 (0x8000000000000000, initbeta[3], copy_beta);
    copy_beta = _mm512_set1_epi8(tailbeta[4]);
    initbeta[4] = _mm512_mask_blend_epi8 (0x8000000000000000, initbeta[4], copy_beta);
    copy_beta = _mm512_set1_epi8(tailbeta[5]);
    initbeta[5] = _mm512_mask_blend_epi8 (0x8000000000000000, initbeta[5], copy_beta);
    copy_beta = _mm512_set1_epi8(tailbeta[6]);
    initbeta[6] = _mm512_mask_blend_epi8 (0x8000000000000000, initbeta[6], copy_beta);
    copy_beta = _mm512_set1_epi8(tailbeta[7]);
    initbeta[7] = _mm512_mask_blend_epi8 (0x8000000000000000, initbeta[7], copy_beta);
}

int32_t bblib_lte_turbo_decoder_64windows_avx512(const struct bblib_turbo_decoder_request *request,
                                                 struct bblib_turbo_decoder_response *response)
{
    int32_t retVal = -1;
    if (request == NULL || response == NULL)
    {
        printf("TurboDecoder_64windows input address invalid \n");
        return -1;
    }
    int32_t NumIter = 0;
    int32_t C = request->c;
    int32_t K = request->k;
    if ((K&0x3F)!=0)
    {
        printf("turbo_decoder_64windows_avx512: K mod 64 is NOT 0.\n");
        return -1;
    }
    int32_t numMaxIter = request->max_iter_num;
    int32_t numMaxIterUse;
    if(numMaxIter == 0)
        numMaxIterUse = 3;
    else
        //numMaxIterUse = numMaxIter;
        numMaxIterUse = numMaxIter;
    int32_t Lwin = K>>4;
    int32_t Lwin6 = K>>6;            //win size
    int32_t Kidx = request->k_idx-1; //from 36.211 Table 5.1.3
    int32_t ofst = g_TurboInterleaver.offset[Kidx];
    int8_t * pIntraRowPattern = &(g_TurboInterleaver.pattern[0][0]);
    int8_t * pLLR_tail = request->input;
    uint8_t *pout = response->output;
    int32_t i, j;

    /* adapter the interleaver Addr and DeInterleaver Addr for win64 */
    int32_t * pInterleaverInterRowAddr = &(g_TurboInterleaver.inter_row_out_addr_for_interleaver[ofst]);
    int32_t * pInterleaverIntraRowPatSel = &(g_TurboInterleaver.intra_row_perm_pattern_for_interleaver[ofst]);
    int32_t * pDeInterleaverInterRowAddr = &(g_TurboInterleaver.inter_row_out_addr_for_deinterleaver[ofst]);
    int32_t * pDeInterleaverIntraRowPatSel = &(g_TurboInterleaver.intra_row_perm_pattern_for_deinterleaver[ofst]);

    __m512i in_line_addr, out_line_addr;
    int32_t vLen = (Lwin%16==0)?(Lwin>>4):((Lwin>>4)+1);
    int32_t InterleaverIntra[vLen*16];
    int32_t InterleaverInter[vLen*16];

    for(i=0; i<vLen; i++)
    {
        in_line_addr = _mm512_load_si512((__m512i *)(pInterleaverIntraRowPatSel+i*16));
        in_line_addr = _mm512_rol_epi32 (in_line_addr, 4);
        _mm512_store_si512((__m512i *)&InterleaverIntra[i*16], in_line_addr);
        out_line_addr = _mm512_load_si512((__m512i *)(pInterleaverInterRowAddr+i*16));
        out_line_addr = _mm512_add_epi32(_mm512_rol_epi32(out_line_addr, 5),_mm512_rol_epi32(out_line_addr, 4));
        _mm512_store_si512((__m512i *)&InterleaverInter[i*16], out_line_addr);
    }

    /* init alpha */
    __m512i initalpha[8], initbeta[8], initalpha_2[8], initbeta_2[8];
    for (i = 1; i < 8; i++)
    {
        initalpha[i] = init_alpha;
        initalpha_2[i] = init_alpha;
        initbeta[i] = _mm512_setzero_si512();
        initbeta_2[i] =_mm512_setzero_si512();
    }

    /* init beta from tail_bit */
    /* x0k[0], z0k[1], x1k[0], z1k[1], z0k[0], x0k[2],z1k[0],x1k[2],x0k[1],z0k[2],x1k[1],z1k[2] */
    int8_t x0k[3], z0k[3], x1k[3], z1k[3], tailbeta[8], tailbeta_2[8];
    x0k[0] = *(pLLR_tail); x0k[1] = *(pLLR_tail + 8); x0k[2] = *(pLLR_tail + 5);
    z0k[0] = *(pLLR_tail + 4); z0k[1] = *(pLLR_tail + 1); z0k[2] = *(pLLR_tail + 9);
    init_beta_comp(x0k, z0k, initbeta, tailbeta);
    x1k[0] = *(pLLR_tail + 2); x1k[1] = *(pLLR_tail + 10); x1k[2] = *(pLLR_tail + 7);
    z1k[0] = *(pLLR_tail + 6); z1k[1] = *(pLLR_tail + 3); z1k[2] = *(pLLR_tail + 11);
    init_beta_comp(x1k, z1k, initbeta_2, tailbeta_2);

    /* Preparing for iteration */
    /* xtrinsic information, LLR for systematic bits, LLR for parity bits */
    int8_t* pLeXP1 = request->input + 48;           //48=16*3
    int8_t* pLeXP2 = request->input + 48 * (Lwin+1);//the first 48 is tail bit.

    int8_t* pAG = response->ag_buf; //pAG = &(AG[0][0]);
    uint16_t* p_winCodeBlockBits = response->cb_buf;
    int32_t min_dist;
    if (numMaxIter != 0)
    {
        min_dist = SISO_64windows(pLeXP2, pLeXP1,
                                  InterleaverInter, pIntraRowPattern, InterleaverIntra,
                                  pAG, initalpha, initbeta, tailbeta, Lwin6,
                                  p_winCodeBlockBits);
        NumIter++;
        if ((min_dist > 2) && (numMaxIter != 0))
        {
            BitTranspose_16windows_new(K, p_winCodeBlockBits, pout);

            struct bblib_crc_request crc_request;
            crc_request.data = pout;
            crc_request.len = ((K >> 3) - 3)*8;
            
            struct bblib_crc_response crc_response;
            crc_response.data = crc_request.data;
            

            if (C > 1)
            {
                bblib_lte_crc24b_check_avx512(&crc_request, &crc_response);
            }
            else
            {
                bblib_lte_crc24a_check_avx512(&crc_request, &crc_response);
            }
            if (crc_response.check_passed)
            {
                if (request->early_term_disable)
                    retVal = NumIter;
                else
                    return NumIter;
            }
            
        }
    }

    /* prepare systematic LLR for second branch */
    int8_t * pSysLLR1 = pLeXP1 + 16;
    int8_t * pSysLLR2 = pLeXP2 + 16;
    /* interleave for second branch */
    __m128i vtmp, vshuf;

    /*for (i=0; i< Lwin6; i++)
    {
        vtmp = _mm512_load_si512((__m512i const*)(pSysLLR1));
        in_line_addr0 = *(pInterleaverIntraRowPatSel + i);
        in_line_addr1 = *(pInterleaverIntraRowPatSel + i +   Lwin6);
        in_line_addr2 = *(pInterleaverIntraRowPatSel + i + 2*Lwin6);
        in_line_addr3 = *(pInterleaverIntraRowPatSel + i + 3*Lwin6);

        out_line_addr0 = *(pInterleaverInterRowAddr + i);
        out_line_addr1 = *(pInterleaverInterRowAddr + i + Lwin6);
        out_line_addr2 = *(pInterleaverInterRowAddr + i + 2*Lwin6);
        out_line_addr3 = *(pInterleaverInterRowAddr + i + 3*Lwin6);

        vshuf = _mm512_inserti32x4(vshuf, *(__m128i *)(pIntraRowPattern+in_line_addr0), 0);
        vshuf = _mm512_inserti32x4(vshuf, *(__m128i *)(pIntraRowPattern+in_line_addr1), 1);
        vshuf = _mm512_inserti32x4(vshuf, *(__m128i *)(pIntraRowPattern+in_line_addr2), 2);
        vshuf = _mm512_inserti32x4(vshuf, *(__m128i *)(pIntraRowPattern+in_line_addr3), 3);
        vtmp = _mm512_shuffle_epi8(vtmp, vshuf);

        delta_tmp0= _mm512_extracti32x8_epi32(vtmp, 0);
        delta_tmp1= _mm512_extracti32x8_epi32(vtmp, 1);
        _mm256_storeu2_m128i((__m128i *)(pSysLLR2 + out_line_addr1), (__m128i *)(pSysLLR2 + out_line_addr0), delta_tmp0);
        _mm256_storeu2_m128i((__m128i *)(pSysLLR2 + out_line_addr3), (__m128i *)(pSysLLR2 + out_line_addr2), delta_tmp1);
        pSysLLR1 += 192;
    }*/
    for (i = 0; i < Lwin; i++)
    {
        vtmp = _mm_load_si128((__m128i const*)(pSysLLR1));
        vshuf = _mm_load_si128((__m128i const*)(pIntraRowPattern+*(InterleaverIntra+i)));
        vtmp = _mm_shuffle_epi8 (vtmp, vshuf);
        _mm_store_si128((__m128i *)(pSysLLR2+*(InterleaverInter+i)), vtmp);
        pSysLLR1 += 48;
    }
    // calculate the DeInterleaver table first
    int32_t DeInterleaverIntra[vLen*16];
    int32_t DeInterleaverInter[vLen*16];
    for(i=0; i<vLen; i++)
    {
        in_line_addr = _mm512_load_si512((__m512i *)(pDeInterleaverIntraRowPatSel+i*16));
        in_line_addr = _mm512_rol_epi32 (in_line_addr, 4);                                                      //*16
        _mm512_store_si512((__m512i *)&DeInterleaverIntra[i*16], in_line_addr);
        out_line_addr = _mm512_load_si512((__m512i *)(pDeInterleaverInterRowAddr+i*16));
        out_line_addr = _mm512_add_epi32(_mm512_rol_epi32(out_line_addr, 5),_mm512_rol_epi32(out_line_addr, 4));//*48
        _mm512_store_si512((__m512i *)&DeInterleaverInter[i*16], out_line_addr);
    }

    for (j = 0; j < numMaxIterUse; j++)
    {
        SISO_64windows(pLeXP1, pLeXP2,
                       DeInterleaverInter, pIntraRowPattern, DeInterleaverIntra,
                       pAG, initalpha_2, initbeta_2, tailbeta_2, Lwin6,
                       p_winCodeBlockBits);
        NumIter++;
        min_dist = SISO_64windows(pLeXP2, pLeXP1,
                                  InterleaverInter, pIntraRowPattern, InterleaverIntra,
                                  pAG, initalpha, initbeta, tailbeta, Lwin6,
                                  p_winCodeBlockBits);
        NumIter++;

        if ((min_dist > 2) && (numMaxIter != 0))
        {
            BitTranspose_16windows_new(K, p_winCodeBlockBits, pout);
            struct bblib_crc_request crc_request;
            crc_request.data = pout;
            crc_request.len = ((K >> 3) - 3)*8;
            
            struct bblib_crc_response crc_response;
            crc_response.data = crc_request.data;
            
            
            if (C > 1)
            {
                bblib_lte_crc24b_check_avx512(&crc_request, &crc_response);
            }
            else
            {
                bblib_lte_crc24a_check_avx512(&crc_request, &crc_response);
            }
            if (crc_response.check_passed)
            {
                if (request->early_term_disable)
                    retVal = NumIter;
                else
                    return NumIter;
            }
        }
    }
    BitTranspose_16windows_new(K, p_winCodeBlockBits, pout);

    struct bblib_crc_request crc_request;
    crc_request.data = pout;
    crc_request.len = (K/8 - 3)*8;
    
    struct bblib_crc_response crc_response;
    crc_response.data = crc_request.data;
    
    if (C > 1)
    {
        bblib_lte_crc24b_check_avx512(&crc_request, &crc_response);
    }
    else
    {
        bblib_lte_crc24a_check_avx512(&crc_request, &crc_response);
    }
    if (crc_response.check_passed)
    {
        return NumIter;
    }
    return retVal;
}

/* return val is 0.75*a */


// OutputAddress                          <--pLeXP2
// InputAddress                           <--pLeXP1
// InterleaverInterRowAddr
// InterleaverIntraRowPattern
// InterleaverIntraRowPatSel
// Tempalpha_sigma                        <--pAG
// initalpha
// initbeta
// tailbeta
// WindowSize                             <--Lwin6
// pCodeBlockBits                         <--p_winCodeBlockBits

int32_t SISO_64windows(int8_t *OutputAddress, int8_t *InputAddress,
          int32_t *InterleaverInterRowAddr, int8_t *InterleaverIntraRowPattern, int32_t *InterleaverIntraRowPatSel,
          int8_t *Tempalpha_sigma, __m512i *initalpha, __m512i* initbeta, int8_t *tailbeta, int32_t WindowSize,
          uint16_t *pCodeBlockBits)
{
    __declspec (align(64)) __m512i alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6, alpha7;
    __declspec (align(64)) __m512i beta0, beta1, beta2, beta3, beta4, beta5, beta6, beta7;
    __declspec (align(64)) __m512i initalpha1,initalpha2,initalpha3,initalpha4,initalpha5,initalpha6,initalpha7;
    __declspec (align(64)) __m512i initbeta1,initbeta2,initbeta3,initbeta4,initbeta5,initbeta6,initbeta7;
    __declspec (align(64)) __m512i tmpalpha0[8], tmpalpha1[8], tmpbeta0[8], tmpbeta1[8];

    __declspec (align(64)) __m512i tmpDelta01, tmpDelta10,tmpDelta20, tmpDelta30, tmpDelta40, tmpDelta50, tmpDelta60, tmpDelta70;
    __declspec (align(64)) __m512i tmpDelta11,tmpDelta21, tmpDelta31, tmpDelta41, tmpDelta51, tmpDelta61, tmpDelta71;
    __declspec (align(64))__m512i xs, xe, xp, xa;
    __m512i delta0, delta1, delta;

    __m512i min_distance = _mm512_set1_epi8(127);
    __m512i* output_addr = (__m512i *)(Tempalpha_sigma);

    __m128i* input_addr0 = (__m128i *)(InputAddress);
    __m128i* input_addr1 = (__m128i *)(InputAddress) +   3*WindowSize;
    __m128i* input_addr2 = (__m128i *)(InputAddress) + 2*3*WindowSize;
    __m128i* input_addr3 = (__m128i *)(InputAddress) + 3*3*WindowSize;

    int32_t in_line_addr0, in_line_addr1, in_line_addr2, in_line_addr3;
    int32_t out_line_addr0, out_line_addr1, out_line_addr2, out_line_addr3;

    __m256i delta_tmp0, delta_tmp1;
    __m128i min0, min1;

    int32_t i,j;
    int32_t bitWord0, bitWord1;

     //aligned 256 bit
    initalpha1 = _mm512_load_si512(initalpha + 1);
    initalpha2 = _mm512_load_si512(initalpha + 2);
    initalpha3 = _mm512_load_si512(initalpha + 3);
    initalpha4 = _mm512_load_si512(initalpha + 4);
    initalpha5 = _mm512_load_si512(initalpha + 5);
    initalpha6 = _mm512_load_si512(initalpha + 6);
    initalpha7 = _mm512_load_si512(initalpha + 7);

    for (i = 0; i < WindowSize; i++)
    {
        //xe = _mm512_load_si512(input_addr++); /* load extrinsic information */
        xe = _mm512_inserti32x4(xe, *input_addr0++, 0);
        xe = _mm512_inserti32x4(xe, *input_addr1++, 1);
        xe = _mm512_inserti32x4(xe, *input_addr2++, 2);
        xe = _mm512_inserti32x4(xe, *input_addr3++, 3);
        SCALE_075(xe);

        //xs = _mm512_load_si512(input_addr++); /* load systematic information */
        //xp = _mm512_load_si512(input_addr++); /* load parity information */
        xs = _mm512_inserti32x4(xs, *input_addr0++, 0);
        xs = _mm512_inserti32x4(xs, *input_addr1++, 1);
        xs = _mm512_inserti32x4(xs, *input_addr2++, 2);
        xs = _mm512_inserti32x4(xs, *input_addr3++, 3);

        xp = _mm512_inserti32x4(xp, *input_addr0++, 0);
        xp = _mm512_inserti32x4(xp, *input_addr1++, 1);
        xp = _mm512_inserti32x4(xp, *input_addr2++, 2);
        xp = _mm512_inserti32x4(xp, *input_addr3++, 3);

        xs = _mm512_adds_epi8(xs, xe);        /* xa = extrinsic infor llr + systematic llr */
        xp = _mm512_adds_epi8(xp, TD_Offset); /* parity llr - 80 */
        xa = _mm512_adds_epi8(xs, xp);        /* xa = extrinsic infor llr + systematic llr + parity llr */

        /* store xs for beta  computation */
        /*[xp, xa, xs] per 256            */
        _mm512_store_si512(output_addr++, xs);
        _mm512_store_si512(output_addr++, xa);
        _mm512_store_si512(output_addr++, xp);

        xs = _mm512_adds_epi8(xs, TD_Offset);

        /* store alpha for delta computation */
        /*[initalpha7, ..., initalpha3, initalpha2, initalpha1,xp, xa, xs]*/
        _mm512_store_si512(output_addr++, initalpha1);
        _mm512_store_si512(output_addr++, initalpha2);
        _mm512_store_si512(output_addr++, initalpha3);
        _mm512_store_si512(output_addr++, initalpha4);
        _mm512_store_si512(output_addr++, initalpha5);
        _mm512_store_si512(output_addr++, initalpha6);
        _mm512_store_si512(output_addr++, initalpha7);

        /* initalpha0 = TD_Offset
           xk --1, zk --0, add xs
           xk --0, zk --1, add xp
           xk --1, zk --1, add xa
           xk --0, zk-- 0, add -80
           gamma = x*xs+x*xe+z*xp = x*(xs+xe)+z*xp */
        alpha0 = _mm512_adds_epi8(initalpha1, xa);
        alpha0 = _mm512_max_epi8(alpha0, TD_Offset);

        /* S3->S1, input 0  S2->S1, input 1 */
        //tmpalpha10 = _mm512_adds_epi8(initalpha3, xp);
        //tmpalpha11 = _mm512_adds_epi8(initalpha2, xs);
        //alpha1 = _mm512_max_epi8(tmpalpha10, tmpalpha11);
        //alpha1 = _mm512_subs_epi8(alpha1, alpha0);
        CALC_ALPHA_BETA(initalpha3, initalpha2, xp, xs, tmpalpha0[1], tmpalpha1[1], alpha0, alpha1);

        /*S4->S2, input 0  S5->S2, input 1 */
        //tmpalpha20 = _mm512_adds_epi8(initalpha4, xp);
        //tmpalpha21 = _mm512_adds_epi8(initalpha5, xs);
        //alpha2 = _mm512_max_epi8(tmpalpha20, tmpalpha21);
        //alpha2 = _mm512_subs_epi8(alpha2, alpha0);
        CALC_ALPHA_BETA(initalpha4, initalpha5, xp, xs, tmpalpha0[2], tmpalpha1[2], alpha0, alpha2);

        /*S6->S3, input 0  S7->S3, input 1 */
        //tmpalpha30 = _mm512_adds_epi8(initalpha6, xa);
        //tmpalpha31 = _mm512_adds_epi8(initalpha7, TD_Offset);
        //alpha3 = _mm512_max_epi8(tmpalpha30, tmpalpha31);
        //alpha3 = _mm512_subs_epi8(alpha3, alpha0);
        CALC_ALPHA_BETA(initalpha6, initalpha7, xa, TD_Offset, tmpalpha0[3], tmpalpha1[3], alpha0, alpha3);

        /*S1->S4, input 0  S0->S4, input 1 */
        alpha4 = _mm512_adds_epi8(initalpha1, TD_Offset);
        alpha4 = _mm512_max_epi8(alpha4, xa);
        alpha4 = _mm512_subs_epi8(alpha4, alpha0);

        /*S2->S5, input 0  S3->S5, input 1 */
        //tmpalpha50 = _mm512_adds_epi8(initalpha2, xp);
        //tmpalpha51 = _mm512_adds_epi8(initalpha3, xs);
        //alpha5 = _mm512_max_epi8(tmpalpha50, tmpalpha51);
        //alpha5 = _mm512_subs_epi8(alpha5, alpha0);
        CALC_ALPHA_BETA(initalpha2, initalpha3, xp, xs, tmpalpha0[5], tmpalpha1[5], alpha0, alpha5);

        /*S5->S6, input 0  S4->S6, input 1 */
        //tmpalpha60 = _mm512_adds_epi8(initalpha5, xp);
        //tmpalpha61 = _mm512_adds_epi8(initalpha4, xs);
        //alpha6 = _mm512_max_epi8(tmpalpha60, tmpalpha61);
        //alpha6 = _mm512_subs_epi8(alpha6, alpha0);
        CALC_ALPHA_BETA(initalpha5, initalpha4, xp, xs, tmpalpha0[6], tmpalpha1[6], alpha0, alpha6);

        /*S7->S7, input 0  S6->S7, input 1 */
        //tmpalpha70 = _mm512_adds_epi8(initalpha7, xa);
        //tmpalpha71 = _mm512_adds_epi8(initalpha6, TD_Offset);
        //alpha7 = _mm512_max_epi8(tmpalpha70, tmpalpha71);
        //alpha7 = _mm512_subs_epi8(alpha7, alpha0);
        CALC_ALPHA_BETA(initalpha7, initalpha6, xa, TD_Offset, tmpalpha0[7], tmpalpha1[7], alpha0, alpha7);

        initalpha1 = alpha1;
        initalpha2 = alpha2;
        initalpha3 = alpha3;
        initalpha4 = alpha4;
        initalpha5 = alpha5;
        initalpha6 = alpha6;
        initalpha7 = alpha7;
    }

    SHUFFLE_ALPHA (initalpha1, initalpha[1]);
    SHUFFLE_ALPHA (initalpha2, initalpha[2]);
    SHUFFLE_ALPHA (initalpha3, initalpha[3]);
    SHUFFLE_ALPHA (initalpha4, initalpha[4]);
    SHUFFLE_ALPHA (initalpha5, initalpha[5]);
    SHUFFLE_ALPHA (initalpha6, initalpha[6]);
    SHUFFLE_ALPHA (initalpha7, initalpha[7]);

    /* calculate beta */
    initbeta1 = _mm512_load_si512(initbeta + 1);
    initbeta2 = _mm512_load_si512(initbeta + 2);
    initbeta3 = _mm512_load_si512(initbeta + 3);
    initbeta4 = _mm512_load_si512(initbeta + 4);
    initbeta5 = _mm512_load_si512(initbeta + 5);
    initbeta6 = _mm512_load_si512(initbeta + 6);
    initbeta7 = _mm512_load_si512(initbeta + 7);
    output_addr --;

    for (i = WindowSize-1;i >= 0; i--)
    {
        /* load alpha, xs, xp and xa */
        alpha7 = _mm512_load_si512(output_addr--);
        alpha6 = _mm512_load_si512(output_addr--);
        alpha5 = _mm512_load_si512(output_addr--);
        alpha4 = _mm512_load_si512(output_addr--);
        alpha3 = _mm512_load_si512(output_addr--);
        alpha2 = _mm512_load_si512(output_addr--);
        alpha1 = _mm512_load_si512(output_addr--);
        xp = _mm512_load_si512(output_addr--);
        xa = _mm512_load_si512(output_addr--);
        xs = _mm512_load_si512(output_addr--);

        /* compute extrinsic informatio and LLR of each bit
           tempDelta = z*xp + alpha + beta
           zk --1, add xp
           zk-- 0, add -80
           tempDelat00, input0, S0-->S0,  alpha0+bata0
           tempDelat01, input1, S1-->S0,  xp+alpha1+bata0 */
        tmpDelta01 = _mm512_adds_epi8(alpha1, xp);

        /*tempDelat10, input0, S3-->S1,  xp+alpha3+bata1
          tempDelat11, input1, S2-->S1,  xp+alpha2+bata1 */
        //tempDelta10 = _mm512_adds_epi8(alpha3, xp);
        //tempDelta10 = _mm512_adds_epi8(tempDelta10, initbeta1);
        //tempDelta11 = _mm512_adds_epi8(alpha2, TD_Offset);
        //tempDelta11 = _mm512_adds_epi8(tempDelta11, initbeta1);
        CALC_DELTA(alpha3, alpha2, initbeta1, xp, TD_Offset, tmpDelta10, tmpDelta11);

        /*tempDelat20, input0, S4-->S2,  xp+alpha4+bata2
          tempDelat21, input1, S5-->S2,  xp+alpha5+bata2 */
        //tempDelta20 = _mm512_adds_epi8(alpha4, xp);
        //tempDelta20 = _mm512_adds_epi8(tempDelta20, initbeta2);
        //tempDelta21 = _mm512_adds_epi8(alpha5, TD_Offset);
        //tempDelta21 = _mm512_adds_epi8(tempDelta21, initbeta2);
        CALC_DELTA(alpha4, alpha5, initbeta2, xp, TD_Offset, tmpDelta20, tmpDelta21);

        /*tempDelat30, input0, S7-->S4,  xp+alpha7+bata3
          tempDelat31, input1, S6-->S4,  xp+alpha6+bata3 */
        //tempDelta30 = _mm512_adds_epi8(alpha7, TD_Offset);
        //tempDelta30 = _mm512_adds_epi8(tempDelta30, initbeta3);
        //tempDelta31 = _mm512_adds_epi8(alpha6, xp);
        //tempDelta31 = _mm512_adds_epi8(tempDelta31, initbeta3);
        CALC_DELTA(alpha7, alpha6, initbeta3, TD_Offset, xp, tmpDelta30, tmpDelta31);

        /*tempDelat40, input0, S1-->S4,  xp+alpha1+bata4
          tempDelat41, input1, S0-->S4,  xp+alpha0+bata4 */
        tmpDelta40 = _mm512_adds_epi8(alpha1, TD_Offset);
        tmpDelta40 = _mm512_adds_epi8(tmpDelta40, initbeta4);
        tmpDelta41 = _mm512_adds_epi8(xp, initbeta4);

        /*tempDelat50, input0, S2-->S5,  xp+alpha2+bata5
          tempDelat51, input1, S3-->S5,  xp+alpha3+bata5 */
        //tempDelta50 = _mm512_adds_epi8(alpha2, xp);
        //tempDelta50 = _mm512_adds_epi8(tempDelta50, initbeta5);
        //tempDelta51 = _mm512_adds_epi8(alpha3, TD_Offset);
        //tempDelta51 = _mm512_adds_epi8(tempDelta51, initbeta5);
        CALC_DELTA(alpha2, alpha3, initbeta5, xp, TD_Offset, tmpDelta50, tmpDelta51);

        /*tempDelat60, input0, S5-->S6,  xp+alpha5+bata6
          tempDelat61, input1, S4-->S6,  xp+alpha5+bata6 */
        //tempDelta60 = _mm512_adds_epi8(alpha5, xp);
        //tempDelta60 = _mm512_adds_epi8(tmpDelta60, initbeta6);
        //tempDelta61 = _mm512_adds_epi8(alpha4, TD_Offset);
        //tempDelta61 = _mm512_adds_epi8(tmpDelta61, initbeta6);
        CALC_DELTA(alpha5, alpha4, initbeta6, xp, TD_Offset, tmpDelta60, tmpDelta61);

        /*tempDelat70, input0, S6-->S7,  xp+alpha6+bata7
          tempDelat71, input1, S7-->S7,  xp+alpha7+bata7 */
        //tempDelta70 = _mm512_adds_epi8(alpha6, TD_Offset);
        //tempDelta70 = _mm512_adds_epi8(tempDelta70, initbeta7);
        //tempDelta71 = _mm512_adds_epi8(alpha7, xp);
        //tempDelta71 = _mm512_adds_epi8(tempDelta71, initbeta7);
        CALC_DELTA(alpha6, alpha7, initbeta7, TD_Offset, xp, tmpDelta70, tmpDelta71);

        tmpDelta10 = _mm512_max_epi8(TD_Offset, tmpDelta10);
        tmpDelta11 = _mm512_max_epi8(tmpDelta01, tmpDelta11);

        tmpDelta30 = _mm512_max_epi8(tmpDelta30, tmpDelta20);
        tmpDelta31 = _mm512_max_epi8(tmpDelta31, tmpDelta21);

        tmpDelta50 = _mm512_max_epi8(tmpDelta50, tmpDelta40);
        tmpDelta51 = _mm512_max_epi8(tmpDelta51, tmpDelta41);

        tmpDelta70 = _mm512_max_epi8(tmpDelta70, tmpDelta60);
        tmpDelta71 = _mm512_max_epi8(tmpDelta71, tmpDelta61);

        tmpDelta30 = _mm512_max_epi8(tmpDelta30, tmpDelta10);
        tmpDelta31 = _mm512_max_epi8(tmpDelta31, tmpDelta11);

        tmpDelta70 = _mm512_max_epi8(tmpDelta70, tmpDelta50);
        tmpDelta71 = _mm512_max_epi8(tmpDelta71, tmpDelta51);

        delta0 = _mm512_max_epi8(tmpDelta30, tmpDelta70);
        delta1 = _mm512_max_epi8(tmpDelta31, tmpDelta71);

        delta0 = _mm512_subs_epi8(delta1, delta0);// new extrinsic
        delta = _mm512_adds_epi8(delta0, xs);     // new LLR of uncoded bit, add xs+xe

        /*hard judge the LLR and store in pCodeBlockBits.*/
        /*LLR = extrinsic info +xa */
#ifdef _LOG_P1_P0
        delta = _mm512_subs_epi8(TD_constant512, delta);// 0-delta
#endif
        //delta = _mm512_shuffle_epi8(delta, TD_constant_0_63);
        delta_tmp0 = _mm512_extracti32x8_epi32(delta, 0);      //low 128
        delta_tmp1 = _mm512_extracti32x8_epi32(delta, 1);
        //delta_tmp0 = _mm256_shuffle_epi8(delta_tmp0, TD_constant_0_16);
        //delta_tmp1 = _mm256_shuffle_epi8(delta_tmp1, TD_constant_0_16);
        bitWord0 = _mm256_movemask_epi8(delta_tmp0);  //get the sign bit (7bit) for each 8 intergrate number, put to int_16 for each bit
        bitWord1 = _mm256_movemask_epi8(delta_tmp1);
        *(pCodeBlockBits + i) = bitWord0;
        *(pCodeBlockBits + WindowSize+ i) = bitWord0 >> 16;
        *(pCodeBlockBits + 2*WindowSize+ i) = bitWord1;
        *(pCodeBlockBits + 3*WindowSize+ i) = bitWord1 >> 16;

        delta = _mm512_abs_epi8(delta);
        min_distance  = _mm512_min_epi8(min_distance, delta); //store the min distance for each win

        /* save extrinsic after interleaver */
        in_line_addr0 = *(InterleaverIntraRowPatSel + i);
        in_line_addr1 = *(InterleaverIntraRowPatSel + i +   WindowSize);
        in_line_addr2 = *(InterleaverIntraRowPatSel + i + 2*WindowSize);
        in_line_addr3 = *(InterleaverIntraRowPatSel + i + 3*WindowSize);

        out_line_addr0 = *(InterleaverInterRowAddr + i);
        out_line_addr1 = *(InterleaverInterRowAddr + i + WindowSize);
        out_line_addr2 = *(InterleaverInterRowAddr + i + 2*WindowSize);
        out_line_addr3 = *(InterleaverInterRowAddr + i + 3*WindowSize);

        delta1 = _mm512_inserti32x4(delta1, *(__m128i *)(InterleaverIntraRowPattern+in_line_addr0), 0);
        delta1 = _mm512_inserti32x4(delta1, *(__m128i *)(InterleaverIntraRowPattern+in_line_addr1), 1);
        delta1 = _mm512_inserti32x4(delta1, *(__m128i *)(InterleaverIntraRowPattern+in_line_addr2), 2);
        delta1 = _mm512_inserti32x4(delta1, *(__m128i *)(InterleaverIntraRowPattern+in_line_addr3), 3);
        delta0 = _mm512_shuffle_epi8(delta0, delta1);

        delta_tmp0= _mm512_extracti32x8_epi32(delta0, 0);
        delta_tmp1= _mm512_extracti32x8_epi32(delta0, 1);
        _mm256_storeu2_m128i((__m128i *)(OutputAddress + out_line_addr1), (__m128i *)(OutputAddress + out_line_addr0), delta_tmp0);
        _mm256_storeu2_m128i((__m128i *)(OutputAddress + out_line_addr3), (__m128i *)(OutputAddress + out_line_addr2), delta_tmp1);

        xs = _mm512_adds_epi8(xs, TD_Offset);// avoid saturation
        /******** update beta ********/
        /*xk --1, zk --0, add xs
                  xk --0, zk --1, add xp
                  xk --1, zk --1, add xa
                  xk --0, zk-- 0, add -80
                  input1, S0-->S4  input0, S0-->S0            */
        beta0 = _mm512_adds_epi8(initbeta4, xa);
        beta0 = _mm512_max_epi8(TD_Offset, beta0);

        /*input1, S1-->S0  input0, S1-->S4 */
        beta1 = _mm512_adds_epi8(initbeta4, TD_Offset);
        beta1= _mm512_max_epi8(beta1, xa);
        beta1 = _mm512_subs_epi8(beta1, beta0);

        /*input1, S2-->S5  input0, S2-->S1 */
        //tempbeta20 = _mm512_adds_epi8(initbeta5, xp);
        //tempbeta21 = _mm512_adds_epi8(initbeta1, xs);
        //beta2 = _mm512_max_epi8(tempbeta20, tempbeta21);
        //beta2 = _mm512_subs_epi8(beta2, beta0);
        CALC_ALPHA_BETA(initbeta5, initbeta1, xp, xs, tmpbeta0[2], tmpbeta1[2], beta0, beta2);

        /*input1, S3-->S1  input0, S3-->S5 */
        //tempbeta30 = _mm512_adds_epi8(initbeta1, xp);
        //tempbeta31 = _mm512_adds_epi8(initbeta5, xs);
        //beta3 = _mm512_max_epi8(tempbeta30, tempbeta31);
        //beta3 = _mm512_subs_epi8(beta3, beta0);
        CALC_ALPHA_BETA(initbeta1, initbeta5, xp, xs, tmpbeta0[3], tmpbeta1[3], beta0, beta3);

        /*input1, S4-->S2  input0, S4-->S6 */
        //tempbeta40 = _mm512_adds_epi8(initbeta2, xp);
        //tempbeta41 = _mm512_adds_epi8(initbeta6, xs);
        //beta4 = _mm512_max_epi8(tempbeta40, tempbeta41);
        //beta4 = _mm512_subs_epi8(beta4, beta0);
        CALC_ALPHA_BETA(initbeta2, initbeta6, xp, xs, tmpbeta0[4], tmpbeta1[4], beta0, beta4);

        /*input1, S5-->S6  input0, S5-->S2 */
        //tempbeta50 = _mm512_adds_epi8(initbeta6, xp);
        //tempbeta51 = _mm512_adds_epi8(initbeta2, xs);
        //beta5 = _mm512_max_epi8(tempbeta50, tempbeta51);
        //beta5 = _mm512_subs_epi8(beta5, beta0);
        CALC_ALPHA_BETA(initbeta6, initbeta2, xp, xs, tmpbeta0[5], tmpbeta1[5], beta0, beta5);

        /*input1, S6-->S3  input0, S6-->S7 */
        //tempbeta60 = _mm512_adds_epi8(initbeta3, xa);
        //beta6 = _mm512_adds_epi8(initbeta7, TD_Offset);
        //beta6 = _mm512_max_epi8(beta6, tempbeta60);
        //beta6 = _mm512_subs_epi8(beta6, beta0);
        CALC_ALPHA_BETA(initbeta3, initbeta7, xa, TD_Offset, tmpbeta0[6], tmpbeta1[6], beta0, beta6);

        /*input1, S7-->S0  input0, S7-->S3 */
        //tempbeta70 = _mm512_adds_epi8(initbeta7, xa);
        //beta7 = _mm512_adds_epi8(initbeta3, TD_Offset);
        //beta7 = _mm512_max_epi8(beta7, tempbeta70);
        //beta7 = _mm512_subs_epi8(beta7, beta0);
        CALC_ALPHA_BETA(initbeta7, initbeta3, xa, TD_Offset, tmpbeta0[7], tmpbeta1[7], beta0, beta7);

        initbeta1 = beta1;
        initbeta2 = beta2;
        initbeta3 = beta3;
        initbeta4 = beta4;
        initbeta5 = beta5;
        initbeta6 = beta6;
        initbeta7 = beta7;
    }

    //shuffle the beta state and insert init value
    SHUFFLE_BETA(tailbeta[1], initbeta1, initbeta[1]);
    SHUFFLE_BETA(tailbeta[2], initbeta2, initbeta[2]);
    SHUFFLE_BETA(tailbeta[3], initbeta3, initbeta[3]);
    SHUFFLE_BETA(tailbeta[4], initbeta4, initbeta[4]);
    SHUFFLE_BETA(tailbeta[5], initbeta5, initbeta[5]);
    SHUFFLE_BETA(tailbeta[6], initbeta6, initbeta[6]);
    SHUFFLE_BETA(tailbeta[7], initbeta7, initbeta[7]);

    delta_tmp0 = _mm512_extracti32x8_epi32(min_distance, 0);
    delta_tmp1 = _mm512_extracti32x8_epi32(min_distance, 1);
    delta_tmp0 = _mm256_min_epi8(delta_tmp0, delta_tmp1);
    min0 = _mm256_extractf128_si256(delta_tmp0, 0);
    min1 = _mm256_extractf128_si256(delta_tmp0, 1);
    min0 = _mm_min_epi8(min0, min1);
    min1 = _mm_minpos_epu16(min0);//get min for each 8bit, min for 0~15 and min pos for 16~18
    min0 = _mm_slli_si128(min0,1);// left shift 8 bit, get min pos
    min0 = _mm_minpos_epu16(min0);//get min for each 8bit, min for 0~15 and min pos for 16~18
    min0 = _mm_srli_si128(min0,1);
    min1 = _mm_srli_si128(min1,1);
    min0 = _mm_min_epi8(min0, min1);
    return _mm_extract_epi8(min0,0);
}

void BitTranspose_16windows_new(int32_t K, uint16_t * pin, uint8_t * pout)
{

    int32_t i, j;

    int32_t Lwin = K >> 4;
    int32_t vLen = (Lwin%8==0)? (Lwin>>3):(Lwin>>3)+1;
    __m256i v1, v2;
    int32_t tmp[8];
    __declspec (align(64)) uint8_t mat_out[16][48];

    for (i=0; i<vLen; i=i+2)
    {
        v1 = _mm256_loadu2_m128i((__m128i *)(pin+8), (__m128i *)(pin));
        pin += 16;
        for(j=0; j<8; j++)
        {
            v2 = _mm256_shuffle_epi8(v1, TD_vshuf1_BitTranspose_16windows);
            tmp[j] = _mm256_movemask_epi8(v2);
            v1 = _mm256_slli_epi16(v1, 1);
            mat_out[7-j][i] = tmp[j];
            mat_out[15-j][i] = tmp[j]>>8;
            mat_out[7-j][i+1] = tmp[j]>>16;
            mat_out[15-j][i+1] = tmp[j]>>24;
        }
    }
    /* ------------------------------------------------------------------------------------------ */
    int32_t resBitNum = 0;
    int32_t newBitNum;
    int32_t tailBitNum = Lwin & 0x3F;
    int32_t NumBitIn = Lwin >> 6;
    __m128i vnow, vnext, vnewbits, vmask;
    uint8_t* ptmp = &(mat_out[0][0]);

    /* if there is no tail bit, read and write directely */
    if (tailBitNum == 0)
    {
        for (i=0; i<16; i++)
        {
            ptmp = &(mat_out[i][0]);
            for (j=0; j<NumBitIn; j++)
            {
                vnow = _mm_loadl_epi64((__m128i const *)ptmp); ptmp += 8;
                _mm_storel_epi64 ((__m128i *)pout, vnow); pout += 8;
            }
        }
    }
    else
    {
        vnow = _mm_setzero_si128();
        for (i=0; i<16; i++)
        {
            ptmp = &(mat_out[i][0]);                                                   /* read from a new line */
            newBitNum = 64 - resBitNum;                                                /* there're resBitNum old bits in vnow's high end */
            vmask = _mm_set1_epi32(-1);
            vmask = _mm_slli_epi64(vmask, newBitNum);                                  /* prepare to remove newBitNum null bits from vnow's low end */
            vnow = _mm_and_si128(vnow, vmask);                                         /* remove newBitNum null bits from vnow's low end */

            for (j=0; j<NumBitIn; j++)
            {
                vnext = _mm_loadl_epi64((__m128i const *)ptmp); ptmp += 8;             /* read 64 new bits in */
                vnext = _mm_shuffle_epi8(vnext, TD_vshuf2_BitTranspose_16windows);     /* little endian to big endian */
                vnewbits = _mm_srli_epi64(vnext, resBitNum);                           /* get newBitNum new bits in vnewbits's low end , big*/
                vnow = _mm_or_si128(vnow, vnewbits);                                   /* combined into vnow  big*/
                vnow = _mm_shuffle_epi8(vnow, TD_vshuf2_BitTranspose_16windows);       /* big endian to little endian */
                _mm_storel_epi64((__m128i *)pout, vnow); pout += 8;
                vnow = _mm_slli_epi64(vnext, newBitNum);                               /* get resBitNum old bits in vnow's high end big */
                vnow = _mm_and_si128(vnow, vmask);                                     /* remove newBitNum null bits from vnow's low end */
            }

            if (resBitNum+tailBitNum>=64)                                              /* with tailBitNum new bits, it is enough for 64 bits plus resBitNum old bits */
            {
                vnext = _mm_loadl_epi64((__m128i const *)ptmp);                        /* ptmp += 8;  read 64 new bits in */
                vnext = _mm_shuffle_epi8(vnext, TD_vshuf2_BitTranspose_16windows);     /* little endian to big endian */
                vnewbits = _mm_srli_epi64(vnext, resBitNum);                           /* get newBitNum new bits in vnewbits's low end */
                vnow = _mm_or_si128(vnow, vnewbits);                                   /* combined into vnow */
                vnow = _mm_shuffle_epi8(vnow, TD_vshuf2_BitTranspose_16windows);       /* big endian to little endian */
                _mm_storel_epi64((__m128i *)pout, vnow); pout += 8;
                vnow = _mm_slli_epi64(vnext, newBitNum);                               /* get resBitNum old bits in vnow's high end */
                resBitNum = resBitNum + tailBitNum - 64;                               /* now resBitNum is resBitNum+tailBitNum-64 */
            }
            else
            {
                vnext = _mm_loadl_epi64((__m128i const *)ptmp);                        /*ptmp += 8; read 64 new bits in */
                vnext = _mm_shuffle_epi8(vnext, TD_vshuf2_BitTranspose_16windows);     /* little endian to big endian */
                vnewbits = _mm_srli_epi64(vnext, resBitNum);                           /* get newBitNum new bits in vnewbits's low end */
                vnow = _mm_or_si128(vnow, vnewbits);                                   /* combined into vnow */
                resBitNum = resBitNum + tailBitNum;                                    /* now resBitNum is resBitNum+tailBitNum */
            }
        }
        if (resBitNum!=0)
        {
            vnow = _mm_shuffle_epi8(vnow, TD_vshuf2_BitTranspose_16windows);           /* big endian to little endian*/
            _mm_storel_epi64((__m128i *)pout, vnow);
        }
    }
}

#else
int32_t bblib_lte_turbo_decoder_64windows_avx512(const struct bblib_turbo_decoder_request *request,
                                                 struct bblib_turbo_decoder_response *response)
{
    printf("bblib_turbo requires AVX512 ISA support to run\n");
    return(-1);
}
#endif
