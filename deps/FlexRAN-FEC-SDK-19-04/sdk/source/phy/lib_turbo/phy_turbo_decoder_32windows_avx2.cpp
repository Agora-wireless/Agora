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
*  @file phy_turbo_decoder_32windows_avx.cpp
*  @brief this file performs the turbo Decoder when CW size is multiple of 32.
*  @author Yang, Xuebin (xuebin.yang@intel.com)
*  Trellis diagram:  0-00->0
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

#include "phy_turbo_internal.h"
#include "phy_crc.h"
#include "phy_turbo.h"
#if defined (_BBLIB_AVX2_) || defined (_BBLIB_AVX512_)
static __m256i TD_constant256 = _mm256_setr_epi8(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
static __m128i TD_constant_0_16 = _mm_setr_epi8(15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0);

#define TURBO_OFFSET (-80)
static __m256i TD_Offset = _mm256_setr_epi8(
    TURBO_OFFSET, TURBO_OFFSET, TURBO_OFFSET, TURBO_OFFSET,
    TURBO_OFFSET, TURBO_OFFSET, TURBO_OFFSET, TURBO_OFFSET,
    TURBO_OFFSET, TURBO_OFFSET, TURBO_OFFSET, TURBO_OFFSET,
    TURBO_OFFSET, TURBO_OFFSET, TURBO_OFFSET, TURBO_OFFSET,
    TURBO_OFFSET, TURBO_OFFSET, TURBO_OFFSET, TURBO_OFFSET,
    TURBO_OFFSET, TURBO_OFFSET, TURBO_OFFSET, TURBO_OFFSET,
    TURBO_OFFSET, TURBO_OFFSET, TURBO_OFFSET, TURBO_OFFSET,
    TURBO_OFFSET, TURBO_OFFSET, TURBO_OFFSET, TURBO_OFFSET);

int32_t SISO_32windows(int8_t *OutputAddress, int8_t *InputAddress,
                                int32_t *InterleaverInterRowAddr, int8_t *InterleaverIntraRowPattern, int32_t *InterleaverIntraRowPatSel,
                                int8_t *Tempalpha_sigma, __m256i *initalpha, __m256i* initbeta, int8_t *tailbeta, int32_t WindowSize,
                                uint16_t *pCodeBlockBits);

void BitTranspose_16windows(int32_t K, uint16_t * pin, uint8_t * pout);

inline void init_beta_comp(int8_t *a, int8_t *c, __m256i* initBeta, int8_t *tailBeta) {

    int8_t beta[8][4];

    beta[1][2] = a[2] + c[2];

    beta[1][1] = a[1] + c[1];
    beta[2][1] = beta[1][2] + a[1];
    tailBeta[7] = beta[1][2] + c[1];

    tailBeta[1] = a[0] + c[0];
    tailBeta[2] = beta[1][1] + a[0];
    tailBeta[3] = beta[1][1] + c[0];
    tailBeta[4] = beta[2][1] + c[0];
    tailBeta[5] = beta[2][1] + a[0];
    tailBeta[6] = tailBeta[7] + tailBeta[1];

    initBeta[1] = _mm256_insert_epi8(initBeta[1] , tailBeta[1], 31) ;
    initBeta[2] = _mm256_insert_epi8(initBeta[2] , tailBeta[2], 31) ;
    initBeta[3] = _mm256_insert_epi8(initBeta[3] , tailBeta[3], 31) ;
    initBeta[4] = _mm256_insert_epi8(initBeta[4] , tailBeta[4], 31) ;
    initBeta[5] = _mm256_insert_epi8(initBeta[5] , tailBeta[5], 31) ;
    initBeta[6] = _mm256_insert_epi8(initBeta[6] , tailBeta[6], 31) ;
    initBeta[7] = _mm256_insert_epi8(initBeta[7] , tailBeta[7], 31) ;

    return;
}

inline __m256i shuffle_alpha_tail(__m256i a)
{
     __m128i x0, x1;
     __m256i b = _mm256_setzero_si256();
     x0 = _mm256_extractf128_si256(a, 0);
     x1 = _mm256_extractf128_si256(a, 1);
     x1 = _mm_slli_si128(x1, 1);
     b = _mm256_inserti128_si256(b, x1, 0);
     b = _mm256_inserti128_si256(b, x0, 1);
     return b;
}

inline __m256i shuffle_beta_tail(__m256i a)
{
     __m128i x0, x1;
     __m256i b = _mm256_setzero_si256();
     x0 = _mm256_extractf128_si256(a, 0);
     x1 = _mm256_extractf128_si256(a, 1);
     x0 = _mm_srli_si128(x0, 1);
     b = _mm256_inserti128_si256(b, x1, 0);
     b = _mm256_inserti128_si256(b, x0, 1);
     return b;
}

struct init_turbo_decoder_32windows_avx2
{
    init_turbo_decoder_32windows_avx2()
    {

    bblib_print_turbo_version();

    }
};

init_turbo_decoder_32windows_avx2 do_constructor_turbo_decoder_32_avx2;

int32_t
bblib_lte_turbo_decoder_32windows_avx2(const struct bblib_turbo_decoder_request *request,
    struct bblib_turbo_decoder_response *response)
{
    int32_t retVal = -1;
    int32_t NumIter = 0;
    int32_t C = request->c;
    int32_t K = request->k;
    if ((K&0x1F)!=0)
    {
        printf("turbo_decoder_32windows_avx2: K mod 32 is NOT 0.\n");
        return -1;
    }
    int32_t numMaxIter = request->max_iter_num;
    int32_t numMaxIterUse;
    if(numMaxIter == 0)
        numMaxIterUse = 3;
    else
        numMaxIterUse = numMaxIter;
    int32_t Lwin = K >> 4, Lwin5 = K >> 5;
    int32_t Kidx = request->k_idx - 1;
    int32_t ofst = g_TurboInterleaver.offset[Kidx];
    int32_t * pInterleaverInterRowAddr = &(g_TurboInterleaver.inter_row_out_addr_for_interleaver[ofst]);
    int32_t * pInterleaverIntraRowPatSel = &(g_TurboInterleaver.intra_row_perm_pattern_for_interleaver[ofst]);
    int32_t * pDeInterleaverInterRowAddr = &(g_TurboInterleaver.inter_row_out_addr_for_deinterleaver[ofst]);
    int32_t * pDeInterleaverIntraRowPatSel = &(g_TurboInterleaver.intra_row_perm_pattern_for_deinterleaver[ofst]);
    int8_t * pIntraRowPattern = &(g_TurboInterleaver.pattern[0][0]);
    int8_t * pLLR_tail = request->input;

    uint8_t *pout = response->output;

    // tail bit and init alpha, beta
    int32_t i, j;
    __m256i initalpha[8], initbeta[8], initalpha_2[8], initbeta_2[8];
    for (i=1;i<8;i++)
    {
        initalpha[i] = _mm256_setzero_si256();
        initbeta[i] = _mm256_setzero_si256();
        initalpha_2[i] = _mm256_setzero_si256();
        initbeta_2[i] = _mm256_setzero_si256();
    }
    for (i = 1; i < 8; i ++) {
        initalpha[i] = _mm256_insert_epi8(initalpha[i] , -128, 0) ;
        initalpha_2[i] = _mm256_insert_epi8(initalpha[i] , -128, 0) ;
    }

    int8_t x0k[3], z0k[3], x1k[3], z1k[3], tailbeta[8], tailbeta_2[8];

    x0k[0] = *(pLLR_tail); x0k[1] = *(pLLR_tail + 8); x0k[2] = *(pLLR_tail + 5);
    z0k[0] = *(pLLR_tail + 4); z0k[1] = *(pLLR_tail + 1); z0k[2] = *(pLLR_tail + 9);
    init_beta_comp(x0k, z0k, initbeta, tailbeta);

    x1k[0] = *(pLLR_tail +2); x1k[1] = *(pLLR_tail + 10); x1k[2] = *(pLLR_tail + 7);
    z1k[0] = *(pLLR_tail + 6); z1k[1] = *(pLLR_tail + 3); z1k[2] = *(pLLR_tail + 11);
    init_beta_comp(x1k, z1k, initbeta_2, tailbeta_2);

    /* Preparing for iteration */
    int8_t* pLeXP1; // extrinsic information, LLR for systematic bits, LLR for parity bits ==> RSC1
    int8_t* pLeXP2; // extrinsic information, LLR for systematic bits, LLR for parity bits ==> RSC2
    pLeXP1 = request->input + 48;
    pLeXP2 = request->input + 48 * (Lwin+1);
    int8_t * pSysLLR1;
    int8_t * pSysLLR2;

    int8_t* pAG; // Alpha and Gamma
    pAG = response->ag_buf; //pAG = &(AG[0][0]);
    uint16_t * p_winCodeBlockBits;
    p_winCodeBlockBits = response->cb_buf;

    /* Zeroing first extrinsic information */
    __m128i vtmp;
    __m128i vshuf;
    int32_t out_line_addr;
    int32_t pattern_sel;
    vtmp = _mm_setzero_si128();
    for (i=0; i<Lwin; i++)
    {
        _mm_store_si128((__m128i *)(pLeXP1), vtmp);
        pLeXP1 += 48;
    }
    pLeXP1 = request->input + 48;

    int32_t min_dist;

    if(numMaxIter != 0) {
        min_dist = SISO_32windows(pLeXP2, pLeXP1,
            pInterleaverInterRowAddr, pIntraRowPattern, pInterleaverIntraRowPatSel,
            pAG, initalpha, initbeta, tailbeta, Lwin5,
            p_winCodeBlockBits);
        NumIter++;

        if ((min_dist>2) && (numMaxIter != 0))
        {
            BitTranspose_16windows(K, p_winCodeBlockBits, pout);

            struct bblib_crc_request crc_request;
            crc_request.data = pout;
            crc_request.len = ((K >> 3) - 3)*8;

            struct bblib_crc_response crc_response;
            crc_response.data = crc_request.data;


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
    }

    /* prepare systematic LLR for second branch */
    pSysLLR1 = pLeXP1 + 16;
    pSysLLR2 = pLeXP2 + 16;
    for (i=0; i<Lwin; i++)
    {
        vtmp = _mm_load_si128((__m128i const*)(pSysLLR1));
        out_line_addr = *(pInterleaverInterRowAddr+i);
        out_line_addr = (out_line_addr<<5) + (out_line_addr<<4);
        pattern_sel = *(pInterleaverIntraRowPatSel+i);
        vshuf = _mm_load_si128((__m128i const*)(pIntraRowPattern+(pattern_sel<<4)));
        vtmp = _mm_shuffle_epi8 (vtmp, vshuf);
        _mm_store_si128((__m128i *)(pSysLLR2+out_line_addr), vtmp);
        pSysLLR1 += 48;
    }


    for (j=0; j<numMaxIterUse; j++)
    {
        SISO_32windows(pLeXP1, pLeXP2,
            pDeInterleaverInterRowAddr, pIntraRowPattern, pDeInterleaverIntraRowPatSel,
            pAG, initalpha_2, initbeta_2, tailbeta_2, Lwin5,
            p_winCodeBlockBits);
        NumIter++;

        min_dist = SISO_32windows(pLeXP2, pLeXP1,
            pInterleaverInterRowAddr, pIntraRowPattern, pInterleaverIntraRowPatSel,
            pAG, initalpha, initbeta, tailbeta, Lwin5,
            p_winCodeBlockBits);
        NumIter++;

        if ((min_dist > 2) && (numMaxIter != 0))
        {
            BitTranspose_16windows(K, p_winCodeBlockBits, pout);

            struct bblib_crc_request crc_request;
            crc_request.data = pout;
            crc_request.len = ((K >> 3) - 3)*8;

            struct bblib_crc_response crc_response;
            crc_response.data = crc_request.data;

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
    }

    BitTranspose_16windows(K, p_winCodeBlockBits, pout);

    struct bblib_crc_request crc_request;
    crc_request.data = pout;
    crc_request.len = (K/8 - 3)*8;

    struct bblib_crc_response crc_response;
    crc_response.data = crc_request.data;

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
        return NumIter;
    }

    return retVal;
}

static __m256i signBitMask = _mm256_setr_epi8(0x80, 0x80, 0x80, 0x80,0x80, 0x80, 0x80,
                                   0x80,0x80, 0x80, 0x80, 0x80,0x80, 0x80, 0x80, 0x80,
                                   0x80, 0x80, 0x80, 0x80,0x80, 0x80, 0x80, 0x80,0x80,
                                   0x80, 0x80, 0x80,0x80, 0x80, 0x80, 0x80);
static __m256i tailBitMask = _mm256_setr_epi8(0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC,
                                              0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC,
                                              0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC,
                                              0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC);

/* return val is 0.75*a */
inline __m256i scale075(__m256i a);
__m256i scale075(__m256i a){
    __m256i signBit = _mm256_and_si256(a, signBitMask), b;
    __m256i signBit1 = _mm256_srli_epi16(signBit, 1);
    b = _mm256_and_si256(a, tailBitMask);
    b = _mm256_srli_epi16(b, 2);
    signBit = _mm256_or_si256(signBit, signBit1);
    b = _mm256_or_si256(b, signBit);
    b = _mm256_subs_epi8(a, b);
    return b;
}

int32_t SISO_32windows(int8_t *OutputAddress, int8_t *InputAddress,
          int32_t *InterleaverInterRowAddr, int8_t *InterleaverIntraRowPattern, int32_t *InterleaverIntraRowPatSel,
          int8_t *Tempalpha_sigma, __m256i *initalpha, __m256i* initbeta, int8_t *tailbeta, int32_t WindowSize,
          uint16_t *pCodeBlockBits)
{
     __m256i alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6, alpha7;
     __m256i tmpalpha10, tmpalpha11, tmpalpha20, tmpalpha21, tmpalpha30;
     __m256i tmpalpha50, tmpalpha51, tmpalpha60, tmpalpha61, tmpalpha70;

     __m256i beta0, beta1, beta2, beta3, beta4, beta5, beta6, beta7;
     __m256i tempbeta20, tempbeta21, tempbeta30, tempbeta31, tempbeta40, tempbeta50;
     __m256i tempbeta41, tempbeta51, tempbeta60, tempbeta70;

     __m256i delta0, delta1;
     __m256i tempDelta01, tempDelta10, tempDelta11, tempDelta20, tempDelta21, tempDelta30, tempDelta31, tempDelta40;
     __m256i tempDelta41,tempDelta50, tempDelta51, tempDelta60, tempDelta61, tempDelta70, tempDelta71, delta;

     __m256i xs, xe, xp, xa;
     __m256i initalpha1,initalpha2,initalpha3,initalpha4,initalpha5,initalpha6,initalpha7;
     __m256i initbeta1,initbeta2,initbeta3,initbeta4,initbeta5,initbeta6,initbeta7;

     __m128i *input_addr0, *input_addr1, min0, min1;
     __m256i *output_addr;

     int32_t i, pattern_sel0, pattern_sel1, out_line_addr0, out_line_addr1;
     __m256i min_distance = _mm256_set1_epi8(127);

     int16_t bitWord0, bitWord1;

     initalpha1 = _mm256_load_si256(initalpha + 1);
     initalpha2 = _mm256_load_si256(initalpha + 2);
     initalpha3 = _mm256_load_si256(initalpha + 3);
     initalpha4 = _mm256_load_si256(initalpha + 4);
     initalpha5 = _mm256_load_si256(initalpha + 5);
     initalpha6 = _mm256_load_si256(initalpha + 6);
     initalpha7 = _mm256_load_si256(initalpha + 7);

     // calculate alhpa
     input_addr0 = (__m128i *)(InputAddress);
     input_addr1 = (__m128i *)(InputAddress) + 3 * WindowSize;
     output_addr = (__m256i *)(Tempalpha_sigma);
     for (i=0;i<WindowSize;i = i++)
     {
         /* load extrinsic information */
         xe = _mm256_loadu2_m128i((__m128i const*)(input_addr1 ++), (__m128i const*)(input_addr0 ++));
         xe = scale075(xe);

         /* load systematic informaiton */
         xs = _mm256_loadu2_m128i((__m128i const*)(input_addr1 ++), (__m128i const*)(input_addr0 ++));

         /* load parity informaiton */
         xp = _mm256_loadu2_m128i((__m128i const*)(input_addr1 ++), (__m128i const*)(input_addr0 ++));

         xs = _mm256_adds_epi8(xs, xe); // xa = extrinsic infor llr + systematic llr
         xp = _mm256_adds_epi8(xp, TD_Offset);
         xa = _mm256_adds_epi8(xs, xp); // xa = extrinsic infor llr + systematic llr + parity llr

         /* store xs for beta  computation */
         _mm256_store_si256(output_addr++, xs);
         _mm256_store_si256(output_addr++, xa);
         _mm256_store_si256(output_addr++, xp);

         xs = _mm256_adds_epi8(xs, TD_Offset);

         /* store alpha for delta computation */
         _mm256_store_si256(output_addr++, initalpha1);
         _mm256_store_si256(output_addr++, initalpha2);
         _mm256_store_si256(output_addr++, initalpha3);
         _mm256_store_si256(output_addr++, initalpha4);
         _mm256_store_si256(output_addr++, initalpha5);
         _mm256_store_si256(output_addr++, initalpha6);
         _mm256_store_si256(output_addr++, initalpha7);

         alpha0 = _mm256_adds_epi8(initalpha1, xa);
         alpha0 = _mm256_max_epi8(alpha0, TD_Offset);

         tmpalpha10 = _mm256_adds_epi8(initalpha3, xp);
         tmpalpha11 = _mm256_adds_epi8(initalpha2, xs);
         alpha1 = _mm256_max_epi8(tmpalpha10, tmpalpha11);
         alpha1 = _mm256_subs_epi8(alpha1, alpha0);

         tmpalpha20 = _mm256_adds_epi8(initalpha4, xp);
         tmpalpha21 = _mm256_adds_epi8(initalpha5, xs);
         alpha2 = _mm256_max_epi8(tmpalpha20, tmpalpha21);
         alpha2 = _mm256_subs_epi8(alpha2, alpha0);

         tmpalpha30 = _mm256_adds_epi8(initalpha6, xa);
         alpha3 = _mm256_adds_epi8(initalpha7, TD_Offset);
         alpha3 = _mm256_max_epi8(alpha3, tmpalpha30);
         alpha3 = _mm256_subs_epi8(alpha3, alpha0);

         alpha4 = _mm256_adds_epi8(initalpha1, TD_Offset);
         alpha4 = _mm256_max_epi8(alpha4, xa);
         alpha4 = _mm256_subs_epi8(alpha4, alpha0);

         tmpalpha50 = _mm256_adds_epi8(initalpha2, xp);
         tmpalpha51 = _mm256_adds_epi8(initalpha3, xs);
         alpha5 = _mm256_max_epi8(tmpalpha50, tmpalpha51);
         alpha5 = _mm256_subs_epi8(alpha5, alpha0);

         tmpalpha60 = _mm256_adds_epi8(initalpha5, xp);
         tmpalpha61 = _mm256_adds_epi8(initalpha4, xs);
         alpha6 = _mm256_max_epi8(tmpalpha60, tmpalpha61);
         alpha6 = _mm256_subs_epi8(alpha6, alpha0);

         tmpalpha70 = _mm256_adds_epi8(initalpha7, xa);
         alpha7 = _mm256_adds_epi8(initalpha6, TD_Offset);
         alpha7 = _mm256_max_epi8(alpha7, tmpalpha70);
         alpha7 = _mm256_subs_epi8(alpha7, alpha0);

         initalpha1 = alpha1;
         initalpha2 = alpha2;
         initalpha3 = alpha3;
         initalpha4 = alpha4;
         initalpha5 = alpha5;
         initalpha6 = alpha6;
         initalpha7 = alpha7;
     }

     initalpha1 = shuffle_alpha_tail(initalpha1);
     initalpha2 = shuffle_alpha_tail(initalpha2);
     initalpha3 = shuffle_alpha_tail(initalpha3);
     initalpha4 = shuffle_alpha_tail(initalpha4);
     initalpha5 = shuffle_alpha_tail(initalpha5);
     initalpha6 = shuffle_alpha_tail(initalpha6);
     initalpha7 = shuffle_alpha_tail(initalpha7);

     initalpha[1] = _mm256_insert_epi8(initalpha1 , -128, 0) ;
     initalpha[2] = _mm256_insert_epi8(initalpha2 , -128, 0) ;
     initalpha[3] = _mm256_insert_epi8(initalpha3 , -128, 0) ;
     initalpha[4] = _mm256_insert_epi8(initalpha4 , -128, 0) ;
     initalpha[5] = _mm256_insert_epi8(initalpha5 , -128, 0) ;
     initalpha[6] = _mm256_insert_epi8(initalpha6 , -128, 0) ;
     initalpha[7] = _mm256_insert_epi8(initalpha7 , -128, 0) ;

     // calculate beta
     initbeta1 = _mm256_load_si256(initbeta + 1);
     initbeta2 = _mm256_load_si256(initbeta + 2);
     initbeta3 = _mm256_load_si256(initbeta + 3);
     initbeta4 = _mm256_load_si256(initbeta + 4);
     initbeta5 = _mm256_load_si256(initbeta + 5);
     initbeta6 = _mm256_load_si256(initbeta + 6);
     initbeta7 = _mm256_load_si256(initbeta + 7);

     output_addr --;
     for (i=WindowSize-1;i>=0; i--)
     {
         /* load alpha, xs, xp and xa */
         alpha7 = _mm256_load_si256(output_addr--);
         alpha6 = _mm256_load_si256(output_addr--);
         alpha5 = _mm256_load_si256(output_addr--);
         alpha4 = _mm256_load_si256(output_addr--);
         alpha3 = _mm256_load_si256(output_addr--);
         alpha2 = _mm256_load_si256(output_addr--);
         alpha1 = _mm256_load_si256(output_addr--);
         xp = _mm256_load_si256(output_addr--);
         xa = _mm256_load_si256(output_addr--);
         xs = _mm256_load_si256(output_addr--);

         /* compute extrinsic informatio and LLR of each bit */
         tempDelta01 = _mm256_adds_epi8(alpha1, xp);

         tempDelta10 = _mm256_adds_epi8(alpha3, xp);
         tempDelta10 = _mm256_adds_epi8(tempDelta10, initbeta1);
         tempDelta11 = _mm256_adds_epi8(alpha2, TD_Offset);
         tempDelta11 = _mm256_adds_epi8(tempDelta11, initbeta1);

         tempDelta20 = _mm256_adds_epi8(alpha4, xp);
         tempDelta20 = _mm256_adds_epi8(tempDelta20, initbeta2);
         tempDelta21 = _mm256_adds_epi8(alpha5, TD_Offset);
         tempDelta21 = _mm256_adds_epi8(tempDelta21, initbeta2);

         tempDelta30 = _mm256_adds_epi8(alpha7, TD_Offset);
         tempDelta30 = _mm256_adds_epi8(tempDelta30, initbeta3);
         tempDelta31 = _mm256_adds_epi8(alpha6, xp);
         tempDelta31 = _mm256_adds_epi8(tempDelta31, initbeta3);

         tempDelta40 = _mm256_adds_epi8(alpha1, TD_Offset);
         tempDelta40 = _mm256_adds_epi8(tempDelta40, initbeta4);
         tempDelta41 = _mm256_adds_epi8(xp, initbeta4);

         tempDelta50 = _mm256_adds_epi8(alpha2, xp);
         tempDelta50 = _mm256_adds_epi8(tempDelta50, initbeta5);
         tempDelta51 = _mm256_adds_epi8(alpha3, TD_Offset);
         tempDelta51 = _mm256_adds_epi8(tempDelta51, initbeta5);

         tempDelta60 = _mm256_adds_epi8(alpha5, xp);
         tempDelta60 = _mm256_adds_epi8(tempDelta60, initbeta6);
         tempDelta61 = _mm256_adds_epi8(alpha4, TD_Offset);
         tempDelta61 = _mm256_adds_epi8(tempDelta61, initbeta6);

         tempDelta70 = _mm256_adds_epi8(alpha6, TD_Offset);
         tempDelta70 = _mm256_adds_epi8(tempDelta70, initbeta7);
         tempDelta71 = _mm256_adds_epi8(alpha7, xp);
         tempDelta71 = _mm256_adds_epi8(tempDelta71, initbeta7);

         tempDelta10 = _mm256_max_epi8(TD_Offset, tempDelta10);
         tempDelta11 = _mm256_max_epi8(tempDelta01, tempDelta11);

         tempDelta30 = _mm256_max_epi8(tempDelta30, tempDelta20);
         tempDelta31 = _mm256_max_epi8(tempDelta31, tempDelta21);

         tempDelta50 = _mm256_max_epi8(tempDelta50, tempDelta40);
         tempDelta51 = _mm256_max_epi8(tempDelta51, tempDelta41);

         tempDelta70 = _mm256_max_epi8(tempDelta70, tempDelta60);
         tempDelta71 = _mm256_max_epi8(tempDelta71, tempDelta61);

         tempDelta30 = _mm256_max_epi8(tempDelta30, tempDelta10);
         tempDelta31 = _mm256_max_epi8(tempDelta31, tempDelta11);

         tempDelta70 = _mm256_max_epi8(tempDelta70, tempDelta50);
         tempDelta71 = _mm256_max_epi8(tempDelta71, tempDelta51);

         delta0 = _mm256_max_epi8(tempDelta30, tempDelta70);
         delta1 = _mm256_max_epi8(tempDelta31, tempDelta71);

         delta0 = _mm256_subs_epi8(delta1, delta0);
         delta = _mm256_adds_epi8(delta0, xs); // new LLR of uncoded bit

#ifdef _LOG_P1_P0
         delta = _mm256_subs_epi8(TD_constant256, delta);
#endif
         min0 = _mm256_extractf128_si256(delta, 0);
         min0 = _mm_shuffle_epi8(min0, TD_constant_0_16);
         min1 = _mm256_extractf128_si256(delta, 1);
         min1 = _mm_shuffle_epi8(min1, TD_constant_0_16);
         bitWord0 = _mm_movemask_epi8(min0);
         bitWord1 = _mm_movemask_epi8(min1);
         *(pCodeBlockBits + i) = bitWord0;
         *(pCodeBlockBits + i + WindowSize) = bitWord1;

         delta = _mm256_abs_epi8(delta);
         min_distance  = _mm256_min_epi8(min_distance, delta);

         /* save extrinsic */
         pattern_sel0 = *(InterleaverIntraRowPatSel + i);
         pattern_sel1 = *(InterleaverIntraRowPatSel + i + WindowSize);

         out_line_addr0 = *(InterleaverInterRowAddr + i);
         out_line_addr0 = (out_line_addr0<<5) + (out_line_addr0<<4); // out_line_addr = out_line_addr * 48;
         out_line_addr1 = *(InterleaverInterRowAddr + i + WindowSize);
         out_line_addr1 = (out_line_addr1<<5) + (out_line_addr1<<4); // out_line_addr = out_line_addr * 48;

         delta1 = _mm256_loadu2_m128i((__m128i const*)(InterleaverIntraRowPattern + (pattern_sel1<<4)),
                                      (__m128i const*)(InterleaverIntraRowPattern + (pattern_sel0<<4)));
         delta0 = _mm256_shuffle_epi8 (delta0, delta1);
         _mm256_storeu2_m128i((__m128i *)(OutputAddress + out_line_addr1),
                              (__m128i *)(OutputAddress + out_line_addr0), delta0);

         xs = _mm256_adds_epi8(xs, TD_Offset);

         /******** update beta ********/
         beta0 = _mm256_adds_epi8(initbeta4, xa);
         beta0 = _mm256_max_epi8(TD_Offset, beta0);

         beta1 = _mm256_adds_epi8(initbeta4, TD_Offset);
         beta1= _mm256_max_epi8(beta1, xa);
         beta1 = _mm256_subs_epi8(beta1, beta0);

         tempbeta20 = _mm256_adds_epi8(initbeta5, xp);
         tempbeta21 = _mm256_adds_epi8(initbeta1, xs);
         beta2 = _mm256_max_epi8(tempbeta20, tempbeta21);
         beta2 = _mm256_subs_epi8(beta2, beta0);

         tempbeta30 = _mm256_adds_epi8(initbeta1, xp);
         tempbeta31 = _mm256_adds_epi8(initbeta5, xs);
         beta3 = _mm256_max_epi8(tempbeta30, tempbeta31);
         beta3 = _mm256_subs_epi8(beta3, beta0);

         tempbeta40 = _mm256_adds_epi8(initbeta2, xp);
         tempbeta41 = _mm256_adds_epi8(initbeta6, xs);
         beta4 = _mm256_max_epi8(tempbeta40, tempbeta41);
         beta4 = _mm256_subs_epi8(beta4, beta0);

         tempbeta50 = _mm256_adds_epi8(initbeta6, xp);
         tempbeta51 = _mm256_adds_epi8(initbeta2, xs);
         beta5 = _mm256_max_epi8(tempbeta50, tempbeta51);
         beta5 = _mm256_subs_epi8(beta5, beta0);

         tempbeta60 = _mm256_adds_epi8(initbeta3, xa);
         beta6 = _mm256_adds_epi8(initbeta7, TD_Offset);
         beta6 = _mm256_max_epi8(beta6, tempbeta60);
         beta6 = _mm256_subs_epi8(beta6, beta0);

         tempbeta70 = _mm256_adds_epi8(initbeta7, xa);
         beta7 = _mm256_adds_epi8(initbeta3, TD_Offset);
         beta7 = _mm256_max_epi8(beta7, tempbeta70);
         beta7 = _mm256_subs_epi8(beta7, beta0);

         initbeta1 = beta1;
         initbeta2 = beta2;
         initbeta3 = beta3;
         initbeta4 = beta4;
         initbeta5 = beta5;
         initbeta6 = beta6;
         initbeta7 = beta7;
     }

     initbeta1 = shuffle_beta_tail(initbeta1);
     initbeta2 = shuffle_beta_tail(initbeta2);
     initbeta3 = shuffle_beta_tail(initbeta3);
     initbeta4 = shuffle_beta_tail(initbeta4);
     initbeta5 = shuffle_beta_tail(initbeta5);
     initbeta6 = shuffle_beta_tail(initbeta6);
     initbeta7 = shuffle_beta_tail(initbeta7);

     initbeta[1] = _mm256_insert_epi8(initbeta1 , tailbeta[1], 31) ;
     initbeta[2] = _mm256_insert_epi8(initbeta2 , tailbeta[2], 31) ;
     initbeta[3] = _mm256_insert_epi8(initbeta3 , tailbeta[3], 31) ;
     initbeta[4] = _mm256_insert_epi8(initbeta4 , tailbeta[4], 31) ;
     initbeta[5] = _mm256_insert_epi8(initbeta5 , tailbeta[5], 31) ;
     initbeta[6] = _mm256_insert_epi8(initbeta6 , tailbeta[6], 31) ;
     initbeta[7] = _mm256_insert_epi8(initbeta7 , tailbeta[7], 31) ;

     min0 = _mm256_extractf128_si256 (min_distance, 0);
     min1 = _mm256_extractf128_si256 (min_distance, 1);
     min0 = _mm_min_epi8(min0, min1);
     min1 = _mm_minpos_epu16(min0);
     min0 = _mm_slli_si128(min0,1);
     min0 = _mm_minpos_epu16(min0);
     min0 = _mm_srli_si128(min0,1);
     min1 = _mm_srli_si128(min1,1);
     min0 = _mm_min_epi8(min0, min1);

     return _mm_extract_epi8(min0,0);
}
#else
int32_t SISO_32windows(int8_t *OutputAddress, int8_t *InputAddress,
          int32_t *InterleaverInterRowAddr, int8_t *InterleaverIntraRowPattern, int32_t *InterleaverIntraRowPatSel,
          int8_t *Tempalpha_sigma, __m256i *initalpha, __m256i* initbeta, int8_t *tailbeta, int32_t WindowSize,
          uint16_t *pCodeBlockBits)
{
    printf("bblib_turbo requires AVX2 ISA support to run\n");
    return(-1);
}
int32_t
bblib_lte_turbo_decoder_32windows_avx2(const struct bblib_turbo_decoder_request *request,
    struct bblib_turbo_decoder_response *response)
{
    printf("bblib_turbo requires AVX2 ISA support to run\n");
    return(-1);
}
#endif
