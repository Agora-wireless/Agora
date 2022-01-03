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
 * @file   phy_turbo_decoder_16windows_see.h
 * @brief  this file performs the turbo Decoder when CW size is multiple of 16.
 */

#include <cstdint>
#include <cstdio>

#include "immintrin.h"

#include "phy_turbo_internal.h"
#include "phy_crc.h"
#include "phy_turbo.h"
#if defined(_BBLIB_SSE4_2_) || defined(_BBLIB_AVX2_) || defined(_BBLIB_AVX512_)
static __m128i TD_constant128 = _mm_setr_epi8(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
static __m128i TD_constantminus80 = _mm_setr_epi8(-80,-80,-80,-80,-80,-80,-80,-80,-80,-80,-80,-80,-80,-80,-80,-80);
static __m128i TD_constant0 = _mm_setr_epi8(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
static __m128i TD_constant_0_16 = _mm_setr_epi8(15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0);
static __m128i TD_vshuf1_BitTranspose_16windows = _mm_setr_epi8(14, 12, 10, 8, 6, 4, 2, 0, 15, 13, 11, 9, 7, 5, 3, 1);
static __m128i TD_vshuf2_BitTranspose_16windows = _mm_setr_epi8(7, 6, 5, 4, 3, 2, 1, 0, 8, 9, 10, 11, 12, 13, 14, 15);

int32_t SISO1_16windows(int8_t *OutputAddress, int8_t *InputAddress,
                                int32_t *InterleaverInterRowAddr, int8_t *InterleaverIntraRowPattern, int32_t *InterleaverIntraRowPatSel,
                                int8_t *Tempalpha_sigma, __m128i *initalpha, __m128i* initbeta, int8_t *tailbeta, int32_t WindowSize,
                                uint16_t *pCodeBlockBits);

int32_t SISO2_16windows(int8_t *OutputAddress, int8_t *InputAddress,
                                int32_t *DeInterleaverInterRowAddr, int8_t *DeInterleaverIntraRowPattern, int32_t *DeInterleaverIntraRowPatSel,
                                int8_t *Tempalpha_sigma, __m128i *initalpha, __m128i* initbeta, int8_t *tailbeta, int32_t WindowSize,
                                uint16_t *pCodeBlockBits);

void BitTranspose_16windows(int32_t K, uint16_t * pin, uint8_t * pout);

struct init_turbo_decoder_16windows_sse
{
    init_turbo_decoder_16windows_sse()
    {

    bblib_print_turbo_version();

    }
};

init_turbo_decoder_16windows_sse do_constructor_turbo_decoder_16_sse;

int32_t
bblib_lte_turbo_decoder_16windows_sse(const struct bblib_turbo_decoder_request *request,
    struct bblib_turbo_decoder_response *response)
{
    int32_t retVal = -1;
    int32_t NumIter = 0;
    int32_t C = request->c;
    int32_t K = request->k;
    if ((K&0xF)!=0)
    {
        printf("turbo_decoder_16windows_sse: K mod 16 is NOT 0.\n");
        return -1;
    }
    int32_t numMaxIter = request->max_iter_num;
    int32_t Lwin = K >> 4;
    int32_t Kidx = request->k_idx - 1;
    int32_t ofst = g_TurboInterleaver.offset[Kidx];
    int32_t * pInterleaverInterRowAddr = &(g_TurboInterleaver.inter_row_out_addr_for_interleaver[ofst]);
    int32_t * pInterleaverIntraRowPatSel = &(g_TurboInterleaver.intra_row_perm_pattern_for_interleaver[ofst]);
    int32_t * pDeInterleaverInterRowAddr = &(g_TurboInterleaver.inter_row_out_addr_for_deinterleaver[ofst]);
    int32_t * pDeInterleaverIntraRowPatSel = &(g_TurboInterleaver.intra_row_perm_pattern_for_deinterleaver[ofst]);
    int8_t * pIntraRowPattern = &(g_TurboInterleaver.pattern[0][0]);
    int8_t * pLLR_tail = request->input;

    uint8_t *pout = response->output;

    /*  tail bit and init alpha, beta */
    int32_t i, j;
    __m128i initalpha[8], initbeta[8], initalpha_2[8], initbeta_2[8];
    for (i=1;i<8;i++)
    {
        initalpha[i] = _mm_set1_epi8(0);
        initbeta[i] = _mm_set1_epi8(0);
        initalpha_2[i] = _mm_set1_epi8(0);
        initbeta_2[i] = _mm_set1_epi8(0);
    }
    initalpha[1] = _mm_insert_epi8(initalpha[1] , -128, 0) ;
    initalpha[2] = _mm_insert_epi8(initalpha[2] , -128, 0) ;
    initalpha[3] = _mm_insert_epi8(initalpha[3] , -128, 0) ;
    initalpha[4] = _mm_insert_epi8(initalpha[4] , -128, 0) ;
    initalpha[5] = _mm_insert_epi8(initalpha[5] , -128, 0) ;
    initalpha[6] = _mm_insert_epi8(initalpha[6] , -128, 0) ;
    initalpha[7] = _mm_insert_epi8(initalpha[7] , -128, 0) ;

    initalpha_2[1] = _mm_insert_epi8(initalpha[1] , -128, 0) ;
    initalpha_2[2] = _mm_insert_epi8(initalpha[2] , -128, 0) ;
    initalpha_2[3] = _mm_insert_epi8(initalpha[3] , -128, 0) ;
    initalpha_2[4] = _mm_insert_epi8(initalpha[4] , -128, 0) ;
    initalpha_2[5] = _mm_insert_epi8(initalpha[5] , -128, 0) ;
    initalpha_2[6] = _mm_insert_epi8(initalpha[6] , -128, 0) ;
    initalpha_2[7] = _mm_insert_epi8(initalpha[7] , -128, 0) ;

    int8_t xk0, xk1, xk2, zk0, zk1, zk2;
    int8_t xaptk0, xaptk1, xaptk2, zaptk0, zaptk1, zaptk2;
    xk0 = *(pLLR_tail+5); xk1 = *(pLLR_tail+8); xk2 = *(pLLR_tail);
    zk0 = *(pLLR_tail+9); zk1 = *(pLLR_tail+1); zk2 = *(pLLR_tail+4);
    xaptk0 = *(pLLR_tail+7); xaptk1 = *(pLLR_tail+10); xaptk2 = *(pLLR_tail+2);
    zaptk0 = *(pLLR_tail+11); zaptk1 = *(pLLR_tail+3); zaptk2 = *(pLLR_tail+6);

    int8_t tailbeta[8], tailbeta_2[8], r1, r2 ,r3;
    r1 = zk0; r2 = xk0; r3 = r1 + r2; tailbeta[1] = r3;
    r1 = zaptk0; r2 = xaptk0; r3 = r1 + r2; tailbeta_2[1] = r3;
    r1 = zk1; r2 = xk1; r3 = r1 + r2; tailbeta[2] = r2 + tailbeta[1]; tailbeta[3] = r1 + tailbeta[1]; tailbeta[1] = r3;
    r1 = zaptk1; r2 = xaptk1; r3 = r1 + r2; tailbeta_2[2] = r2 + tailbeta_2[1]; tailbeta_2[3] = r1 + tailbeta_2[1]; tailbeta_2[1] = r3;

    r1 = zk2; r2 = xk2; r3 = r1 + r2;
    tailbeta[7] = tailbeta[3];
    tailbeta[6] = r3 + tailbeta[3];
    tailbeta[5] = r2 + tailbeta[2];
    tailbeta[4] = r1 + tailbeta[2];
    tailbeta[3] = r1 + tailbeta[1];
    tailbeta[2] = r2 + tailbeta[1];
    tailbeta[1] = r3;

    r1 = zaptk2; r2 = xaptk2; r3 = r1 + r2;
    tailbeta_2[7] = tailbeta_2[3];
    tailbeta_2[6] = r3 + tailbeta_2[3];
    tailbeta_2[5] = r2 + tailbeta_2[2];
    tailbeta_2[4] = r1 + tailbeta_2[2];
    tailbeta_2[3] = r1 + tailbeta_2[1];
    tailbeta_2[2] = r2 + tailbeta_2[1];
    tailbeta_2[1] = r3;

    initbeta[1] = _mm_insert_epi8(initbeta[1] , tailbeta[1], 15) ;
    initbeta[2] = _mm_insert_epi8(initbeta[2] , tailbeta[2], 15) ;
    initbeta[3] = _mm_insert_epi8(initbeta[3] , tailbeta[3], 15) ;
    initbeta[4] = _mm_insert_epi8(initbeta[4] , tailbeta[4], 15) ;
    initbeta[5] = _mm_insert_epi8(initbeta[5] , tailbeta[5], 15) ;
    initbeta[6] = _mm_insert_epi8(initbeta[6] , tailbeta[6], 15) ;
    initbeta[7] = _mm_insert_epi8(initbeta[7] , tailbeta[7], 15) ;

    initbeta_2[1] = _mm_insert_epi8(initbeta_2[1] , tailbeta_2[1], 15) ;
    initbeta_2[2] = _mm_insert_epi8(initbeta_2[2] , tailbeta_2[2], 15) ;
    initbeta_2[3] = _mm_insert_epi8(initbeta_2[3] , tailbeta_2[3], 15) ;
    initbeta_2[4] = _mm_insert_epi8(initbeta_2[4] , tailbeta_2[4], 15) ;
    initbeta_2[5] = _mm_insert_epi8(initbeta_2[5] , tailbeta_2[5], 15) ;
    initbeta_2[6] = _mm_insert_epi8(initbeta_2[6] , tailbeta_2[6], 15) ;
    initbeta_2[7] = _mm_insert_epi8(initbeta_2[7] , tailbeta_2[7], 15) ;

    /* Preparing for iteration */
    int8_t* pLeXP1; /* extrinsic information, LLR for systematic bits, LLR for parity bits ==> RSC1 */
    int8_t* pLeXP2; /* extrinsic information, LLR for systematic bits, LLR for parity bits ==> RSC2 */
    pLeXP1 = request->input + 48;
    pLeXP2 = request->input + 48 * (Lwin+1);
    int8_t * pSysLLR1;
    int8_t * pSysLLR2;

    int8_t* pAG; /* Alpha and Gamma */
    pAG = response->ag_buf; /* pAG = &(AG[0][0]); */
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

    /* SISO */
    int32_t min_dist;

    min_dist = SISO1_16windows(pLeXP2, pLeXP1,
    pInterleaverInterRowAddr, pIntraRowPattern, pInterleaverIntraRowPatSel,
    pAG, initalpha, initbeta, tailbeta, Lwin,
    p_winCodeBlockBits);
    NumIter++;

    if (min_dist>2)
    {
        BitTranspose_16windows(K, p_winCodeBlockBits, pout);

        struct bblib_crc_request crc_request;
        crc_request.data = pout;
        crc_request.len = ((K >> 3) - 3)*8;
        
        struct bblib_crc_response crc_response;
        crc_response.data = crc_request.data;
        

        /* CRC here */
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

    /* Interleaver for systematic bits here */
    pSysLLR1 = pLeXP1 + 16;
    pSysLLR2 = pLeXP2 + 16;
    for (i=0; i<Lwin; i++)
    {
    vtmp = _mm_load_si128((__m128i const*)(pSysLLR1));
    out_line_addr = *(pInterleaverInterRowAddr+i);
    out_line_addr = (out_line_addr<<5) + (out_line_addr<<4); /* out_line_addr = out_line_addr * 48; */
    pattern_sel = *(pInterleaverIntraRowPatSel+i);
    vshuf = _mm_load_si128((__m128i const*)(pIntraRowPattern+(pattern_sel<<4))); /* pattern_sel * 16 */
    vtmp = _mm_shuffle_epi8 (vtmp, vshuf);
    _mm_store_si128((__m128i *)(pSysLLR2+out_line_addr), vtmp);
    pSysLLR1 += 48;
    }

    /* or (j=0; j<NUM_MAX_TURBO_ITER; j++) */
    for (j=0; j<numMaxIter; j++) /* change to support configurable max iteration time */
    {
        SISO2_16windows(pLeXP1, pLeXP2,
        pDeInterleaverInterRowAddr, pIntraRowPattern, pDeInterleaverIntraRowPatSel,
        pAG, initalpha_2, initbeta_2, tailbeta_2, Lwin,
        p_winCodeBlockBits);
        NumIter++;

        min_dist = SISO1_16windows(pLeXP2, pLeXP1,
        pInterleaverInterRowAddr, pIntraRowPattern, pInterleaverIntraRowPatSel,
        pAG, initalpha, initbeta, tailbeta, Lwin,
        p_winCodeBlockBits);
        NumIter++;

        if (min_dist>2)
        {
            BitTranspose_16windows(K, p_winCodeBlockBits, pout);

            struct bblib_crc_request crc_request;
            crc_request.data = pout;
            crc_request.len = ((K >> 3) - 3)*8;
            
            struct bblib_crc_response crc_response;
            crc_response.data = crc_request.data;
            

            /* CRC here */
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

int32_t
bblib_lte_turbo_decoder_16windows_3iteration_sse(const struct bblib_turbo_decoder_request *request,
    struct bblib_turbo_decoder_response *response)
{
    int32_t NumIter = 0;
    int32_t C = request->c;
    int32_t K = request->k;
    if (K%16!=0)
    {
        printf("turbo_decoder_16windows_3iteration_sse: K mod 16 is NOT 0.\n");
        return -1;
    }

    int32_t Lwin = K >> 4;
    int32_t Kidx = request->k_idx - 1;
    int32_t ofst = g_TurboInterleaver.offset[Kidx];
    int32_t * pInterleaverInterRowAddr = &(g_TurboInterleaver.inter_row_out_addr_for_interleaver[ofst]);
    int32_t * pInterleaverIntraRowPatSel = &(g_TurboInterleaver.intra_row_perm_pattern_for_interleaver[ofst]);
    int32_t * pDeInterleaverInterRowAddr = &(g_TurboInterleaver.inter_row_out_addr_for_deinterleaver[ofst]);
    int32_t * pDeInterleaverIntraRowPatSel = &(g_TurboInterleaver.intra_row_perm_pattern_for_deinterleaver[ofst]);
    int8_t * pIntraRowPattern = &(g_TurboInterleaver.pattern[0][0]);
    int8_t * pLLR_tail = request->input;

    uint8_t *pout = response->output;

    /*  tail bit and init alpha, beta */
    int32_t i, j;
    __m128i initalpha[8], initbeta[8], initalpha_2[8], initbeta_2[8];
    for (i=1;i<8;i++)
    {
        initalpha[i] = _mm_set1_epi8(0);
        initbeta[i] = _mm_set1_epi8(0);
        initalpha_2[i] = _mm_set1_epi8(0);
        initbeta_2[i] = _mm_set1_epi8(0);
    }
    initalpha[1] = _mm_insert_epi8(initalpha[1] , -128, 0) ;
    initalpha[2] = _mm_insert_epi8(initalpha[2] , -128, 0) ;
    initalpha[3] = _mm_insert_epi8(initalpha[3] , -128, 0) ;
    initalpha[4] = _mm_insert_epi8(initalpha[4] , -128, 0) ;
    initalpha[5] = _mm_insert_epi8(initalpha[5] , -128, 0) ;
    initalpha[6] = _mm_insert_epi8(initalpha[6] , -128, 0) ;
    initalpha[7] = _mm_insert_epi8(initalpha[7] , -128, 0) ;

    initalpha_2[1] = _mm_insert_epi8(initalpha[1] , -128, 0) ;
    initalpha_2[2] = _mm_insert_epi8(initalpha[2] , -128, 0) ;
    initalpha_2[3] = _mm_insert_epi8(initalpha[3] , -128, 0) ;
    initalpha_2[4] = _mm_insert_epi8(initalpha[4] , -128, 0) ;
    initalpha_2[5] = _mm_insert_epi8(initalpha[5] , -128, 0) ;
    initalpha_2[6] = _mm_insert_epi8(initalpha[6] , -128, 0) ;
    initalpha_2[7] = _mm_insert_epi8(initalpha[7] , -128, 0) ;

    int8_t xk0, xk1, xk2, zk0, zk1, zk2;
    int8_t xaptk0, xaptk1, xaptk2, zaptk0, zaptk1, zaptk2;
    xk0 = *(pLLR_tail+5); xk1 = *(pLLR_tail+8); xk2 = *(pLLR_tail);
    zk0 = *(pLLR_tail+9); zk1 = *(pLLR_tail+1); zk2 = *(pLLR_tail+4);
    xaptk0 = *(pLLR_tail+7); xaptk1 = *(pLLR_tail+10); xaptk2 = *(pLLR_tail+2);
    zaptk0 = *(pLLR_tail+11); zaptk1 = *(pLLR_tail+3); zaptk2 = *(pLLR_tail+6);

    int8_t tailbeta[8], tailbeta_2[8], r1, r2 ,r3;
    r1 = zk0; r2 = xk0; r3 = r1 + r2; tailbeta[1] = r3;
    r1 = zaptk0; r2 = xaptk0; r3 = r1 + r2; tailbeta_2[1] = r3;
    r1 = zk1; r2 = xk1; r3 = r1 + r2; tailbeta[2] = r2 + tailbeta[1]; tailbeta[3] = r1 + tailbeta[1]; tailbeta[1] = r3;
    r1 = zaptk1; r2 = xaptk1; r3 = r1 + r2; tailbeta_2[2] = r2 + tailbeta_2[1]; tailbeta_2[3] = r1 + tailbeta_2[1]; tailbeta_2[1] = r3;

    r1 = zk2; r2 = xk2; r3 = r1 + r2;
    tailbeta[7] = tailbeta[3];
    tailbeta[6] = r3 + tailbeta[3];
    tailbeta[5] = r2 + tailbeta[2];
    tailbeta[4] = r1 + tailbeta[2];
    tailbeta[3] = r1 + tailbeta[1];
    tailbeta[2] = r2 + tailbeta[1];
    tailbeta[1] = r3;

    r1 = zaptk2; r2 = xaptk2; r3 = r1 + r2;
    tailbeta_2[7] = tailbeta_2[3];
    tailbeta_2[6] = r3 + tailbeta_2[3];
    tailbeta_2[5] = r2 + tailbeta_2[2];
    tailbeta_2[4] = r1 + tailbeta_2[2];
    tailbeta_2[3] = r1 + tailbeta_2[1];
    tailbeta_2[2] = r2 + tailbeta_2[1];
    tailbeta_2[1] = r3;

    initbeta[1] = _mm_insert_epi8(initbeta[1] , tailbeta[1], 15) ;
    initbeta[2] = _mm_insert_epi8(initbeta[2] , tailbeta[2], 15) ;
    initbeta[3] = _mm_insert_epi8(initbeta[3] , tailbeta[3], 15) ;
    initbeta[4] = _mm_insert_epi8(initbeta[4] , tailbeta[4], 15) ;
    initbeta[5] = _mm_insert_epi8(initbeta[5] , tailbeta[5], 15) ;
    initbeta[6] = _mm_insert_epi8(initbeta[6] , tailbeta[6], 15) ;
    initbeta[7] = _mm_insert_epi8(initbeta[7] , tailbeta[7], 15) ;

    initbeta_2[1] = _mm_insert_epi8(initbeta_2[1] , tailbeta_2[1], 15) ;
    initbeta_2[2] = _mm_insert_epi8(initbeta_2[2] , tailbeta_2[2], 15) ;
    initbeta_2[3] = _mm_insert_epi8(initbeta_2[3] , tailbeta_2[3], 15) ;
    initbeta_2[4] = _mm_insert_epi8(initbeta_2[4] , tailbeta_2[4], 15) ;
    initbeta_2[5] = _mm_insert_epi8(initbeta_2[5] , tailbeta_2[5], 15) ;
    initbeta_2[6] = _mm_insert_epi8(initbeta_2[6] , tailbeta_2[6], 15) ;
    initbeta_2[7] = _mm_insert_epi8(initbeta_2[7] , tailbeta_2[7], 15) ;

    /* Preparing for iteration */
    int8_t* pLeXP1; /* extrinsic information, LLR for systematic bits, LLR for parity bits ==> RSC1 */
    int8_t* pLeXP2; /*  extrinsic information, LLR for systematic bits, LLR for parity bits ==> RSC2 */
    pLeXP1 = request->input + 48;
    pLeXP2 = request->input + 48 * (Lwin+1);
    int8_t * pSysLLR1;
    int8_t * pSysLLR2;

    int8_t* pAG; /* Alpha and Gamma */
    pAG = response->ag_buf; /* pAG = &(AG[0][0]); */
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

    /* SISO */
    SISO1_16windows(pLeXP2, pLeXP1,
    pInterleaverInterRowAddr, pIntraRowPattern, pInterleaverIntraRowPatSel,
    pAG, initalpha, initbeta, tailbeta, Lwin,
    p_winCodeBlockBits);
    NumIter++;

    /* Interleaver for systematic bits here */
    pSysLLR1 = pLeXP1 + 16;
    pSysLLR2 = pLeXP2 + 16;
    for (i=0; i<Lwin; i++)
    {
        vtmp = _mm_load_si128((__m128i const*)(pSysLLR1));
        out_line_addr = *(pInterleaverInterRowAddr+i);
        out_line_addr = (out_line_addr<<5) + (out_line_addr<<4); /* out_line_addr = out_line_addr * 48; */
        pattern_sel = *(pInterleaverIntraRowPatSel+i);
        vshuf = _mm_load_si128((__m128i const*)(pIntraRowPattern+(pattern_sel<<4))); /*  pattern_sel * 16 */
        vtmp = _mm_shuffle_epi8 (vtmp, vshuf);
        _mm_store_si128((__m128i *)(pSysLLR2+out_line_addr), vtmp);
        pSysLLR1 += 48;
    }

    for (j=0; j<3; j++)
    {
        SISO2_16windows(pLeXP1, pLeXP2,
        pDeInterleaverInterRowAddr, pIntraRowPattern, pDeInterleaverIntraRowPatSel,
        pAG, initalpha_2, initbeta_2, tailbeta_2, Lwin,
        p_winCodeBlockBits);
        NumIter++;


        SISO1_16windows(pLeXP2, pLeXP1,
        pInterleaverInterRowAddr, pIntraRowPattern, pInterleaverIntraRowPatSel,
        pAG, initalpha, initbeta, tailbeta, Lwin,
        p_winCodeBlockBits);
        NumIter++;

    }

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
        return NumIter;
    }

    return -1;
}



int32_t SISO1_16windows(int8_t *OutputAddress, int8_t *InputAddress,
                                 int32_t *InterleaverInterRowAddr, int8_t *InterleaverIntraRowPattern, int32_t *InterleaverIntraRowPatSel,
                                 int8_t *Tempalpha_sigma, __m128i *initalpha, __m128i* initbeta, int8_t *tailbeta, int32_t WindowSize,
                                 uint16_t *pCodeBlockBits)
{
     __m128i alpha_beta0, alpha_beta1, alpha_beta2, alpha_beta3, alpha_beta4, alpha_beta5, alpha_beta6, alpha_beta7;

     __m128i sigma1, sigma2, sigma3;
     __m128i /*initalpha0,*/initalpha1,initalpha2,initalpha3,initalpha4,initalpha5,initalpha6,initalpha7;
     __m128i /*initbeta0,*/ initbeta1,initbeta2,initbeta3,initbeta4,initbeta5,initbeta6,initbeta7;

     __m128i *input_addr, *output_addr;
     /*initalpha0 = initalpha[0];*/
     initalpha1 = initalpha[1];
     initalpha2 = initalpha[2];
     initalpha3 = initalpha[3];
     initalpha4 = initalpha[4];
     initalpha5 = initalpha[5];
     initalpha6 = initalpha[6];
     initalpha7 = initalpha[7];

     int32_t i;
     int32_t out_line_addr;
     int32_t pattern_sel;

     __m128i min_distance = _mm_set1_epi8(127);
     __m128i sigmamask_hi = _mm_set1_epi16((int16_t)0xFF00);
     __m128i sigmamask_lo = _mm_set1_epi16((int16_t)0x00FF);

    /* calculate alhpa and sigma[1:3] */
    for (i=0;i<WindowSize;i = i++)
    {
        input_addr = (__m128i *)(InputAddress)+3*i ;
        sigma1 = _mm_load_si128((__m128i const*)(input_addr));
        input_addr = input_addr + 1;

        /* E*0.75 */
        alpha_beta2 = _mm_abs_epi8(sigma1);
        alpha_beta4 = _mm_slli_si128(alpha_beta2,1); /* shift left 1 byte */
        alpha_beta4 = _mm_srai_epi16(alpha_beta4,2); /* * 1/4 */
        alpha_beta0 = _mm_srai_epi16(alpha_beta2 ,2); /* * 1/4 */
        alpha_beta0 = _mm_and_si128(alpha_beta0, sigmamask_hi); /* 1/4 A */
        alpha_beta4 = _mm_srli_si128(alpha_beta4, 1);
        alpha_beta4 = _mm_and_si128(alpha_beta4, sigmamask_lo); /* 1/4 A */
        alpha_beta0 = _mm_adds_epi8(alpha_beta0,alpha_beta4);  /* 0.75 * (A B) */

        alpha_beta2 = _mm_subs_epi8(alpha_beta2 , alpha_beta0);/* 0.75*E obtaned */
        sigma1 = _mm_sign_epi8(alpha_beta2, sigma1);

        sigma2 = _mm_load_si128((__m128i const*)(input_addr)); /* Xs */
        input_addr = input_addr + 1;
        alpha_beta0 = _mm_load_si128((__m128i const*)(input_addr)); /* Xp = r1 */

        /*sigma0 = 0 = r0 */
        sigma2 = _mm_adds_epi8( sigma2, sigma1);  /* E + Xs  = r2 */
        sigma3 = _mm_adds_epi8( alpha_beta0, sigma2); /* E+Xs+Xp  = r3 */

        /* alpha_beta1 = _mm_adds_epi8( sigma1, sigma0);  /*  E + Xs - 80  = r1 */
        /* alpha_beta2 = _mm_adds_epi8( sigma2, sigma0); /*  Xp - 80      = r2 */
        /* alpha_beta3 = _mm_adds_epi8( sigma3, sigma0); /*  E+Xs+Xp - 80      = r3 */
        output_addr = (__m128i *)(Tempalpha_sigma) + 10*(WindowSize - 1 - i);
        _mm_store_si128(output_addr, alpha_beta0); /* save r1 */
        output_addr = output_addr + 1;
        _mm_store_si128(output_addr, sigma2); /* save r2 */
        output_addr = output_addr + 1;
        _mm_store_si128(output_addr, sigma3); /* save r3 */
        output_addr = output_addr + 1;
        sigma1 = alpha_beta0;

        sigma1 = _mm_adds_epi8(sigma1, TD_constantminus80);
        sigma2 = _mm_adds_epi8(sigma2, TD_constantminus80);
        sigma3 = _mm_adds_epi8(sigma3, TD_constantminus80);

        /* Alpha(1) = maxstar(r0 + initAlpha(1), r3 + initAlpha(2)); */
        alpha_beta0 = _mm_adds_epi8(sigma3, initalpha1);
        alpha_beta0 = _mm_max_epi8(alpha_beta0, TD_constantminus80);

        _mm_store_si128((__m128i *)(output_addr), initalpha1); /* save alpha(K-1) */
        output_addr = output_addr + 1;
        _mm_store_si128((__m128i *)(output_addr), initalpha2); /* save alpha(K-1) */
        output_addr = output_addr + 1;
        _mm_store_si128((__m128i *)(output_addr), initalpha3); /* save alpha(K-1) */
        output_addr = output_addr + 1;
        _mm_store_si128((__m128i *)(output_addr), initalpha4); /* save alpha(K-1) */
        output_addr = output_addr + 1;
        _mm_store_si128((__m128i *)(output_addr), initalpha5); /* save alpha(K-1) */
        output_addr = output_addr + 1;
        _mm_store_si128((__m128i *)(output_addr), initalpha6); /* save alpha(K-1) */
        output_addr = output_addr + 1;
        _mm_store_si128((__m128i *)(output_addr), initalpha7); /* save alpha(K-1) */

        /* Alpha(2) = maxstar(r2 + initAlpha(3), r1 + initAlpha(4) ); */
        alpha_beta2 = _mm_adds_epi8(sigma1, initalpha3);
        alpha_beta3 = _mm_adds_epi8(sigma2, initalpha2); /* temp result */
        alpha_beta2 = _mm_max_epi8(alpha_beta2, alpha_beta3);
        alpha_beta2 = _mm_subs_epi8(alpha_beta2, alpha_beta0); /* alpha[1] - alpha[0]; */
        alpha_beta4 = alpha_beta2;

        /* Alpha(5) = maxstar(r3 + initAlpha(1), r0 + initAlpha(2)); */
        alpha_beta3 = _mm_max_epi8(_mm_adds_epi8(initalpha1, TD_constantminus80) , sigma3);
        alpha_beta1 = _mm_subs_epi8(alpha_beta3, alpha_beta0); /* alpha2 - alpha[0] */
        alpha_beta5 = alpha_beta1;  /* alpha_beta4 is temp buffer for initalpha4; */

        /* Alpha(6) = maxstar(r1 + initAlpha(3), r2 + initAlpha(4) ); */
        alpha_beta2 = _mm_adds_epi8(sigma2, initalpha3);
        alpha_beta3 = _mm_adds_epi8(sigma1, initalpha2);
        alpha_beta2 = _mm_max_epi8(alpha_beta2 , alpha_beta3);
        alpha_beta2 = _mm_subs_epi8(alpha_beta2, alpha_beta0); /* alpha2 - alpha[0] */
        alpha_beta6 = alpha_beta2;

        /***************************************/
        initalpha1 = alpha_beta4; /* reuse alpha_beta4; */

        /* Alpha(3) = maxstar(r1 + initAlpha(5), r2 + initAlpha(6)); */
        alpha_beta2 = _mm_adds_epi8(sigma2, initalpha5);
        alpha_beta3 = _mm_adds_epi8(sigma1, initalpha4);
        alpha_beta2 = _mm_max_epi8(alpha_beta2 , alpha_beta3);
        alpha_beta2 = _mm_subs_epi8(alpha_beta2, alpha_beta0); /* alpha2 - alpha[0] */
        initalpha2 = alpha_beta2; /* alpha_beta4 is temp buffer for initalpha2; */


        /* Alpha(4) = maxstar(r3 + initAlpha(7), r0 + initAlpha(8) ); */
        alpha_beta2 = _mm_adds_epi8(sigma3, initalpha6);
        alpha_beta2 = _mm_max_epi8(alpha_beta2 , _mm_adds_epi8(initalpha7, TD_constantminus80));
        alpha_beta2 = _mm_subs_epi8(alpha_beta2, alpha_beta0); /* alpha2 - alpha[0] */
        initalpha3 = alpha_beta2; /* shift initalhpa to next window */

        /* Alpha(7) = maxstar(r2 + initAlpha(5), r1 + initAlpha(6)); */
        alpha_beta2 = _mm_adds_epi8(sigma1, initalpha5);
        alpha_beta3 = _mm_adds_epi8(sigma2, initalpha4);
        alpha_beta2 = _mm_max_epi8(alpha_beta2 , alpha_beta3);
        alpha_beta2 = _mm_subs_epi8(alpha_beta2, alpha_beta0); /* alpha2 - alpha[0] */
        alpha_beta4 = alpha_beta2; /* shift initalhpa to next window initalhpa[6] */

        /* Alpha(8) = maxstar(r0 + initAlpha(7), r3 + initAlpha(8) ); */
        alpha_beta2 = _mm_adds_epi8(sigma3, initalpha7);
        alpha_beta2 = _mm_max_epi8(alpha_beta2 , _mm_adds_epi8(initalpha6, TD_constantminus80));
        alpha_beta2 = _mm_subs_epi8(alpha_beta2, alpha_beta0); /* alpha2 - alpha[0] */
        initalpha7 = alpha_beta2; /* shift initalhpa to next window */

        initalpha6 =  alpha_beta4;
        initalpha4 =  alpha_beta5;
        initalpha5 =  alpha_beta6;
    }

    initalpha1 =_mm_slli_si128(initalpha1, 1);
    initalpha2 =_mm_slli_si128(initalpha2, 1);
    initalpha3 =_mm_slli_si128(initalpha3, 1);
    initalpha4 =_mm_slli_si128(initalpha4, 1);
    initalpha5 =_mm_slli_si128(initalpha5, 1);
    initalpha6 =_mm_slli_si128(initalpha6, 1);
    initalpha7 =_mm_slli_si128(initalpha7, 1);

    initalpha[1] = _mm_insert_epi8(initalpha1 , -128, 0) ;
    initalpha[2] = _mm_insert_epi8(initalpha2 , -128, 0) ;
    initalpha[3] = _mm_insert_epi8(initalpha3 , -128, 0) ;
    initalpha[4] = _mm_insert_epi8(initalpha4 , -128, 0) ;
    initalpha[5] = _mm_insert_epi8(initalpha5 , -128, 0) ;
    initalpha[6] = _mm_insert_epi8(initalpha6 , -128, 0) ;
    initalpha[7] = _mm_insert_epi8(initalpha7 , -128, 0) ;

    /* end of initalpha */
    /*********** last bit is differenct to update initalhpa *************/
    /* TD_constant128 = _mm_set1_epi8(-80); */

    /*initbeta0 =  initbeta[0];*/
    initbeta1 =  initbeta[1];
    initbeta2 =  initbeta[2];
    initbeta3 =  initbeta[3];
    initbeta4 =  initbeta[4];
    initbeta5 =  initbeta[5];
    initbeta6 =  initbeta[6];
    initbeta7 =  initbeta[7];
    /* start beta calculation */
    for (i=WindowSize-1;i>=0; i--)
    {
        input_addr = (__m128i *)(Tempalpha_sigma) +10*(WindowSize - 1 - i);
        sigma1 = _mm_load_si128((__m128i const*)(input_addr)); /*  r1 = xp */
        input_addr = input_addr + 1;
        sigma2 = _mm_load_si128((__m128i const*)(input_addr)); /* r2 = xs+e */
        input_addr = input_addr + 1;
        sigma3 = _mm_load_si128((__m128i const*)(input_addr)); /*  r3 = xp+xs+e */
        input_addr = input_addr + 1;

        sigma1 = _mm_adds_epi8(sigma1, TD_constantminus80);
        sigma2 = _mm_adds_epi8(sigma2, TD_constantminus80);
        sigma3 = _mm_adds_epi8(sigma3, TD_constantminus80);
        /*alpha[0] = 0; */
        alpha_beta0 = _mm_load_si128((__m128i const*)(input_addr)); /* load alpha[1] */
        input_addr = input_addr + 1;

        /* Lamda(1,1) = alpha(1) +  gama(4) -offset+beta(5)  -offset2  ; */
        /* Lamda(1,2) = alpha(2) + gama(4) -offset+ beta(1) -offset2; */
        /* Lamda(2,1) = alpha(1) + gama(1) -offset + beta(1) -offset2; */
        /* Lamda(2,2) = alpha(2) + gama(1) -offset +  beta(5)-offset2; */

        alpha_beta2 =  _mm_adds_epi8(sigma3, initbeta4); /* Lamda(1,1) */
        /* Lamda(2,1) = sigma0; */
        /* alpha_beta2=   _mm_adds_epi8(alpha_beta1, alpha_beta0); /* Lamda(1,1); */

        alpha_beta1 =  _mm_adds_epi8(sigma3, alpha_beta0); /* Lamda(1,2) */
        alpha_beta3 =  _mm_adds_epi8(TD_constantminus80, alpha_beta0); /* Lamda(2,2) */
        alpha_beta3 =  _mm_adds_epi8( alpha_beta3, initbeta4); /* Lamda(2,2) */

        alpha_beta2 =  _mm_max_epi8(alpha_beta2 , alpha_beta1); /* max(Lamda(1,1), Lamda(1,2));*/
        alpha_beta3 =  _mm_max_epi8(alpha_beta3 , TD_constantminus80); /* max(Lamda(2,1), Lamda(2,2));*/

        /* Lamda(1,3) = alpha(3) + gama(3)-offset+ beta(2) -offset2; */
        /* Lamda(1,4) = alpha(4) + gama(3) -offset+ beta(6)- offset2;*/

        /* Lamda(2,3) = alpha(3) + gama(2) -offset+ beta(6) - offset2; */
        /* Lamda(2,4) = alpha(4) + gama(2) -offset+ beta(2) -offset2; */

        alpha_beta0 = _mm_load_si128((__m128i const*)(input_addr)); /* load alpha[2] => ALpha(3) */
        input_addr = input_addr + 1;
        alpha_beta1 = _mm_load_si128((__m128i const*)(input_addr)); /* load alpha[3] => Alpha(4) */
        input_addr = input_addr + 1;

        alpha_beta4 = _mm_adds_epi8(sigma2, alpha_beta0); /* Lamda(1,3) = alpha(3) + gama(3) */
        alpha_beta5 = _mm_adds_epi8(sigma2, alpha_beta1); /* Lamda(1,4) = alpha(4) + gama(3) */
        alpha_beta6 = _mm_adds_epi8(sigma1, alpha_beta0); /* Lamda(2,3) = alpha(3) + gama(2) */
        alpha_beta7 = _mm_adds_epi8(sigma1, alpha_beta1); /* Lamda(2,4) = alpha(4) + gama(2) */

        alpha_beta4 = _mm_adds_epi8(alpha_beta4, initbeta1); /*  Lamda(1,3) = alpha(3) + gama(3) + beta2; */
        alpha_beta6 = _mm_adds_epi8(alpha_beta6, initbeta5); /* Lamda(2,3) = alpha(3) + gama(2) + beta6 */

        alpha_beta7 = _mm_adds_epi8(alpha_beta7, initbeta1); /* Lamda(2,4) = alpha(3) + gama(2) + beta2 */
        alpha_beta3 = _mm_max_epi8(alpha_beta3 , alpha_beta6); /* max(Lamda(2,1), Lamda(2,2), Lamda(2,3)); */
        alpha_beta2 = _mm_max_epi8(alpha_beta2 , alpha_beta4); /* max(Lamda(1,1), Lamda(1,2), Lamda(1,3)); */
        alpha_beta5 = _mm_adds_epi8(alpha_beta5, initbeta5); /*  Lamda(1,4) = alpha(4) + gama(3) + beta6; */
        alpha_beta2 = _mm_max_epi8(alpha_beta2 , alpha_beta5); /*  max(Lamda(1,1), Lamda(1,2), Lamda(1,3), Lamda(1,4)); */
        alpha_beta3 = _mm_max_epi8(alpha_beta3 , alpha_beta7); /*  max(Lamda(2,1), Lamda(2,2), Lamda(2,3), Lamda(2,4)); */


        /* Lamda(1,5) = alpha(5) +  gama(3)-offset+  beta(7)-offset2; */
        /* Lamda(1,6) = alpha(6) + gama(3) -offset+ beta(3)- offset2; */
        /* Lamda(2,5) = alpha(5) + gama(2) -offset+ beta(3)- offset2; */
        /* Lamda(2,6) = alpha(6) + gama(2) -offset+ beta(7)- offset2; */
        alpha_beta0 = _mm_load_si128((__m128i const*)(input_addr)); /*  load alpha[4]*/
        input_addr = input_addr + 1;
        alpha_beta1 = _mm_load_si128((__m128i const*)(input_addr)); /*  load alpha[5] */
        input_addr = input_addr + 1;

        alpha_beta4 = _mm_adds_epi8(sigma2, alpha_beta0); /* Lamda(1,5) = alpha(5) + gama(3) */
        alpha_beta5 = _mm_adds_epi8(sigma2, alpha_beta1); /* Lamda(1,6) = alpha(6) + gama(3) */
        alpha_beta6 = _mm_adds_epi8(sigma1, alpha_beta0); /* Lamda(2,5) = alpha(5) + gama(2) */
        alpha_beta7 = _mm_adds_epi8(sigma1, alpha_beta1); /* Lamda(2,6) = alpha(6) + gama(2) */

        alpha_beta4 = _mm_adds_epi8(alpha_beta4, initbeta6); /*  Lamda(1,5) = alpha(5) + gama(3) + beta6; */
        alpha_beta6 = _mm_adds_epi8(alpha_beta6, initbeta2); /* Lamda(2,5) = alpha(5) + gama(2) + beta3 */
        alpha_beta7 = _mm_adds_epi8(alpha_beta7, initbeta6); /* Lamda(2,6) = alpha(6) + gama(2) + beta7 */
        alpha_beta5 = _mm_adds_epi8(alpha_beta5, initbeta2); /*  Lamda(1,6) = alpha(4) + gama(3) + beta3; */

        alpha_beta2 = _mm_max_epi8(alpha_beta2 , alpha_beta4); /* max(Lamda(1,1), Lamda(1,2), Lamda(1,3), Lamda(1,4) Lamda(1,5)); */
        alpha_beta3 = _mm_max_epi8(alpha_beta3 , alpha_beta6); /* max(Lamda(2,1), Lamda(2,2), Lamda(2,3), Lamda(2,4) Lamda(2,5)); */


        alpha_beta2 = _mm_max_epi8(alpha_beta2 , alpha_beta5); /*  max(Lamda(1,1), Lamda(1,2), Lamda(1,3), Lamda(1,4) Lamda(1,5) Lamda(1,6)); */
        alpha_beta3 = _mm_max_epi8(alpha_beta3 , alpha_beta7); /*  max(Lamda(2,1), Lamda(2,2), Lamda(2,3), Lamda(2,4) Lamda(2,5) Lamda(2,6) ); */


        /* Lamda(1,7) = alpha(7) + gama(4) -offset+  beta(4)-offset2; */
        /* Lamda(1,8) = alpha(8) + gama(4)-offset+ beta(8) -offset2 ; */
        /* Lamda(2,7) = alpha(7) + gama(1) -offset+ beta(8) -offset2; */
        /* Lamda(2,8) = alpha(8) + gama(1) -offset+ beta(4) -offset2; */
        alpha_beta0 = _mm_load_si128((__m128i const*)(input_addr)); /*  load alpha[6] */
        input_addr = input_addr + 1;
        alpha_beta1 = _mm_load_si128((__m128i const*)(input_addr)); /*  load alpha[7] */

        alpha_beta4 = _mm_adds_epi8(sigma3, alpha_beta0); /* Lamda(1,7) = alpha(7) + gama(4) */
        alpha_beta5 = _mm_adds_epi8(sigma3, alpha_beta1); /* Lamda(1,8) = alpha(8) + gama(4) */
        alpha_beta6 = _mm_adds_epi8(TD_constantminus80, alpha_beta0); /* Lamda(2,7) = alpha(7) + gama(1) */
        alpha_beta7 = _mm_adds_epi8(TD_constantminus80, alpha_beta1); /* Lamda(2,8) = alpha(8) + gama(1) */


        alpha_beta4 = _mm_adds_epi8(alpha_beta4, initbeta3); /*  Lamda(1,7) = alpha(7) + gama(4) +  beta(4); */
        alpha_beta6 = _mm_adds_epi8(alpha_beta6, initbeta7); /*  Lamda(2,7) = alpha(7) + gama(1) + beta(8); */
        alpha_beta7 = _mm_adds_epi8(alpha_beta7, initbeta3); /* Lamda(2,8) = alpha(8) + gama(1) +  beta4 */
        alpha_beta5 = _mm_adds_epi8(alpha_beta5, initbeta7); /* Lamda(1,8) = alpha(8) + gama(4)+ beta(8) */

        alpha_beta2 =  _mm_max_epi8(alpha_beta2 , alpha_beta4);
        alpha_beta3 =  _mm_max_epi8(alpha_beta3 , alpha_beta6);
        alpha_beta2 =  _mm_max_epi8(alpha_beta2 , alpha_beta5);
        alpha_beta3 =  _mm_max_epi8(alpha_beta3 , alpha_beta7);

        /******************************************************************/
        alpha_beta2 = _mm_subs_epi8(alpha_beta2, alpha_beta3); /*  Lamda obtained */

#ifdef _LOG_P1_P0
        *(pCodeBlockBits + i) = (uint16_t) _mm_movemask_epi8(_mm_shuffle_epi8(_mm_subs_epi8(TD_constant0, alpha_beta2), TD_constant_0_16));
#else
        *(pCodeBlockBits + i) = (uint16_t) _mm_movemask_epi8(_mm_shuffle_epi8(alpha_beta2, TD_constant_0_16));
#endif


        alpha_beta3 = _mm_abs_epi8(alpha_beta2); /*  abs(Lamda); */
        min_distance  = _mm_min_epi8(min_distance,alpha_beta3);

        /* E = 0.75*(Lamda - Xs - E;) */
        alpha_beta4 = _mm_subs_epi8(sigma2, TD_constantminus80);
        alpha_beta3 = _mm_subs_epi8(alpha_beta2,alpha_beta4);

        /*  save E, that's in alpha_beta3 */
        out_line_addr = *(InterleaverInterRowAddr+i);
        out_line_addr = (out_line_addr<<5) + (out_line_addr<<4); /*  out_line_addr = out_line_addr * 48; */
        pattern_sel = *(InterleaverIntraRowPatSel+i);
        alpha_beta1 = _mm_load_si128((__m128i const*)(InterleaverIntraRowPattern+(pattern_sel<<4))); /*  pattern_sel * 16 */
        alpha_beta3 = _mm_shuffle_epi8 (alpha_beta3, alpha_beta1);
        _mm_store_si128((__m128i *)(OutputAddress+out_line_addr), alpha_beta3);

        /*  alpha_beta3  /* E = e*0.75; */
        /******************************************************************/


        /******** update beta ********/
        /* Beta(1) = maxstar(r0 + initBeta(1), r3 + initBeta(5) ); */
        alpha_beta0 = _mm_adds_epi8(sigma3, initbeta4);
        alpha_beta0 = _mm_max_epi8(alpha_beta0,TD_constantminus80  ); /* beta(1) saved in alpha_beta0; */


        /* Beta(2) = maxstar(r3 + initBeta(1), r0 + initBeta(5) ); */
        alpha_beta1 = _mm_adds_epi8(TD_constantminus80, initbeta4);
        alpha_beta1 = _mm_max_epi8(alpha_beta1 , sigma3); /* beta(2) saved in alpha_beta1; */
        alpha_beta1 = _mm_subs_epi8(alpha_beta1,alpha_beta0);

        /* Beta(3) = maxstar(r2 + initBeta(2), r1 + initBeta(6) ); */
        alpha_beta2 = _mm_adds_epi8(sigma1, initbeta5);
        alpha_beta3 = _mm_adds_epi8(sigma2, initbeta1);
        alpha_beta2 = _mm_max_epi8(alpha_beta2, alpha_beta3);  /* beta(3) saved in alpha_beta2 */
        alpha_beta2 = _mm_subs_epi8(alpha_beta2,alpha_beta0);

        /* initbeta1 = alpha_beta1; /* release alpha_beta1; */

        /* Beta(4) = maxstar(r1 + initBeta(2), r2 + initBeta(6) ); */
        alpha_beta4 = _mm_adds_epi8(sigma2, initbeta5);
        alpha_beta3 = _mm_adds_epi8(sigma1, initbeta1);
        alpha_beta7 = _mm_max_epi8(alpha_beta4,alpha_beta3);  /* beta(4) saved in alpha_beta7 */
        alpha_beta7 = _mm_subs_epi8(alpha_beta7,alpha_beta0);


        /* Beta(5) = maxstar(r1 + initBeta(3), r2 + initBeta(7) ); */
        alpha_beta3 = _mm_adds_epi8(sigma2, initbeta6);
        alpha_beta4 = _mm_adds_epi8(sigma1, initbeta2);
        initbeta4   = _mm_max_epi8(alpha_beta4,alpha_beta3);
        initbeta4 = _mm_subs_epi8(initbeta4,alpha_beta0);

        /* Beta(6) = maxstar(r2 + initBeta(3), r1 + initBeta(7) ); */
        alpha_beta3 = _mm_adds_epi8(sigma1, initbeta6);
        alpha_beta4 = _mm_adds_epi8(sigma2, initbeta2);
        initbeta5   = _mm_max_epi8(alpha_beta3,alpha_beta4);
        initbeta5 = _mm_subs_epi8(initbeta5,alpha_beta0);

        /* Beta(7) = maxstar(r3 + initBeta(4), r0 + initBeta(8) ); */
        alpha_beta3 = _mm_adds_epi8(TD_constantminus80, initbeta7);
        alpha_beta4 = _mm_adds_epi8(sigma3, initbeta3);
        initbeta6   = _mm_max_epi8(alpha_beta3,alpha_beta4);
        initbeta6 = _mm_subs_epi8(initbeta6,alpha_beta0);
        /* Beta(8) = maxstar(r0 + initBeta(4), r3 + initBeta(8) );*/
        alpha_beta3 = _mm_adds_epi8(sigma3, initbeta7);
        alpha_beta4 = _mm_adds_epi8(TD_constantminus80, initbeta3);
        initbeta7   = _mm_max_epi8(alpha_beta3,alpha_beta4);
        initbeta7 = _mm_subs_epi8(initbeta7,alpha_beta0);


        initbeta1 = alpha_beta1;
        initbeta2 = alpha_beta2;
        initbeta3 =  alpha_beta7;

        /******** end of beta calculation *********/

    }
    initbeta1 =_mm_srli_si128(initbeta1, 1);
    initbeta2 =_mm_srli_si128(initbeta2, 1);
    initbeta3 =_mm_srli_si128(initbeta3, 1);
    initbeta4 =_mm_srli_si128(initbeta4, 1);
    initbeta5 =_mm_srli_si128(initbeta5, 1);
    initbeta6 =_mm_srli_si128(initbeta6, 1);
    initbeta7 =_mm_srli_si128(initbeta7, 1);

    initbeta[1] = _mm_insert_epi8(initbeta1 , tailbeta[1], 15) ;
    initbeta[2] = _mm_insert_epi8(initbeta2 , tailbeta[2], 15) ;
    initbeta[3] = _mm_insert_epi8(initbeta3 , tailbeta[3], 15) ;
    initbeta[4] = _mm_insert_epi8(initbeta4 , tailbeta[4], 15) ;
    initbeta[5] = _mm_insert_epi8(initbeta5 , tailbeta[5], 15) ;
    initbeta[6] = _mm_insert_epi8(initbeta6 , tailbeta[6], 15) ;
    initbeta[7] = _mm_insert_epi8(initbeta7 , tailbeta[7], 15) ;

    min_distance  = _mm_abs_epi8(min_distance);
    alpha_beta1 = _mm_minpos_epu16(min_distance);
    alpha_beta2 = _mm_slli_si128(min_distance,1);
    alpha_beta2 = _mm_minpos_epu16(alpha_beta2);
    alpha_beta1 = _mm_srli_si128(alpha_beta1,1);
    alpha_beta2 = _mm_srli_si128(alpha_beta2,1);
    alpha_beta1 = _mm_max_epi8(alpha_beta1, alpha_beta2);

    return _mm_extract_epi8(alpha_beta1,0);
}


int32_t SISO2_16windows(int8_t *OutputAddress, int8_t *InputAddress,
                                 int32_t *DeInterleaverInterRowAddr, int8_t *DeInterleaverIntraRowPattern, int32_t *DeInterleaverIntraRowPatSel,
                                 int8_t *Tempalpha_sigma, __m128i *initalpha, __m128i* initbeta, int8_t *tailbeta, int32_t WindowSize,
                                 uint16_t *pCodeBlockBits)
{
    __m128i alpha_beta0, alpha_beta1, alpha_beta2, alpha_beta3, alpha_beta4, alpha_beta5, alpha_beta6, alpha_beta7;

    __m128i sigma1, sigma2, sigma3;
    __m128i /*initalpha0,*/initalpha1,initalpha2,initalpha3,initalpha4,initalpha5,initalpha6,initalpha7;
    __m128i /*initbeta0, */initbeta1,initbeta2,initbeta3,initbeta4,initbeta5,initbeta6,initbeta7;

    __m128i *input_addr, *output_addr;
    /*initalpha0 = initalpha[0];*/
    initalpha1 = initalpha[1];
    initalpha2 = initalpha[2];
    initalpha3 = initalpha[3];
    initalpha4 = initalpha[4];
    initalpha5 = initalpha[5];
    initalpha6 = initalpha[6];
    initalpha7 = initalpha[7];

    int32_t i;
    int32_t out_line_addr;
    int32_t pattern_sel;

    __m128i min_distance = _mm_set1_epi8(127);
    __m128i sigmamask_hi = _mm_set1_epi16((int16_t)0xFF00);
    __m128i sigmamask_lo = _mm_set1_epi16((int16_t)0x00FF);

    /* calculate alhpa and sigma[1:3]*/
    for (i=0;i<WindowSize;i = i++)
    {
        input_addr = (__m128i *)(InputAddress)+3*i ;
        /*sigma1 = _mm_load_si128((__m128i const*)(InputAddress+48*i+0)); */
        sigma1 = _mm_load_si128((__m128i const*)(input_addr));
        input_addr = input_addr + 1;

        /* E*0.75*/
        alpha_beta2 = _mm_abs_epi8(sigma1);
        alpha_beta4 = _mm_slli_si128(alpha_beta2,1); /*shift left 1 byte */
        alpha_beta4 = _mm_srai_epi16(alpha_beta4,2); /* 1/4 */
        alpha_beta0 = _mm_srai_epi16(alpha_beta2 ,2); /*  1/4 */
        alpha_beta0 = _mm_and_si128(alpha_beta0, sigmamask_hi); /* 1/4 A */
        alpha_beta4 = _mm_srli_si128(alpha_beta4, 1);
        alpha_beta4 = _mm_and_si128(alpha_beta4, sigmamask_lo); /* 1/4 A */
        alpha_beta0 = _mm_adds_epi8(alpha_beta0,alpha_beta4);  /*0.75 * (A B) */

        alpha_beta2 = _mm_subs_epi8(alpha_beta2 , alpha_beta0); /* 0.75*E obtaned */
        sigma1 = _mm_sign_epi8(alpha_beta2, sigma1);

        /* sigma2 = _mm_load_si128((__m128i const*)(InputAddress+48*i+16));  Xs */
        /* alpha_beta0 = _mm_load_si128((__m128i const*)(InputAddress+48*i+32));  Xp = r1*/

        sigma2 = _mm_load_si128((__m128i const*)(input_addr)); /* Xs */
        input_addr = input_addr + 1;
        alpha_beta0 = _mm_load_si128((__m128i const*)(input_addr)); /* Xp = r1 */

        /*sigma0 = 0 = r0 */
        sigma2 = _mm_adds_epi8( sigma2, sigma1);  /* E + Xs  = r2 */
        sigma3 = _mm_adds_epi8( alpha_beta0, sigma2); /* E+Xs+Xp  = r3 */

        /* alpha_beta1 = _mm_adds_epi8( sigma1, sigma0);  // E + Xs - 80  = r1 */
        output_addr = (__m128i *)(Tempalpha_sigma) + 10*(WindowSize - 1 - i);
        _mm_store_si128(output_addr, alpha_beta0); /*save r1 */
        output_addr = output_addr + 1;
        _mm_store_si128(output_addr, sigma2); /* save r2 */
        output_addr = output_addr + 1;
        _mm_store_si128(output_addr, sigma3); /* save r3 */
        output_addr = output_addr + 1;
        sigma1 = alpha_beta0;

        /* Alpha(1) = maxstar(r0 + initAlpha(1), r3 + initAlpha(2)); */
        alpha_beta0 = _mm_adds_epi8(sigma3, initalpha1);
        alpha_beta0 = _mm_max_epi8(alpha_beta0, TD_constant128);

        _mm_store_si128((__m128i *)(output_addr), initalpha1); /*save alpha(K-1) */
        output_addr = output_addr + 1;
        _mm_store_si128((__m128i *)(output_addr), initalpha2); /* save alpha(K-1) */
        output_addr = output_addr + 1;
        _mm_store_si128((__m128i *)(output_addr), initalpha3); /* save alpha(K-1) */
        output_addr = output_addr + 1;
        _mm_store_si128((__m128i *)(output_addr), initalpha4); /* save alpha(K-1) */
        output_addr = output_addr + 1;
        _mm_store_si128((__m128i *)(output_addr), initalpha5); /* save alpha(K-1) */
        output_addr = output_addr + 1;
        _mm_store_si128((__m128i *)(output_addr), initalpha6); /* save alpha(K-1) */
        output_addr = output_addr + 1;
        _mm_store_si128((__m128i *)(output_addr), initalpha7); /* save alpha(K-1) */

        /* Alpha(2) = maxstar(r2 + initAlpha(3), r1 + initAlpha(4) ); */
        alpha_beta2 = _mm_adds_epi8(sigma1, initalpha3);
        alpha_beta3 = _mm_adds_epi8(sigma2, initalpha2); /* temp result */
        alpha_beta2 = _mm_max_epi8(alpha_beta2, alpha_beta3);
        alpha_beta2 = _mm_subs_epi8(alpha_beta2, alpha_beta0); /* alpha[1] - alpha[0]; */
        alpha_beta4 = alpha_beta2;

        /* Alpha(5) = maxstar(r3 + initAlpha(1), r0 + initAlpha(2)); */
        alpha_beta3 = _mm_max_epi8(initalpha1 , sigma3);
        alpha_beta1 = _mm_subs_epi8(alpha_beta3, alpha_beta0); /* alpha2 - alpha[0] */
        alpha_beta5 = alpha_beta1;  /* alpha_beta4 is temp buffer for initalpha4;*/

        /* Alpha(6) = maxstar(r1 + initAlpha(3), r2 + initAlpha(4) ); */
        alpha_beta2 = _mm_adds_epi8(sigma2, initalpha3);
        alpha_beta3 = _mm_adds_epi8(sigma1, initalpha2);
        alpha_beta2 = _mm_max_epi8(alpha_beta2 , alpha_beta3);
        alpha_beta2 = _mm_subs_epi8(alpha_beta2, alpha_beta0); /* alpha2 - alpha[0] */
        alpha_beta6 = alpha_beta2;

        /***************************************/
        initalpha1 = alpha_beta4; /* reuse alpha_beta4; */

        /* Alpha(3) = maxstar(r1 + initAlpha(5), r2 + initAlpha(6)); */
        alpha_beta2 = _mm_adds_epi8(sigma2, initalpha5);
        alpha_beta3 = _mm_adds_epi8(sigma1, initalpha4);
        alpha_beta2 = _mm_max_epi8(alpha_beta2 , alpha_beta3);
        alpha_beta2 = _mm_subs_epi8(alpha_beta2, alpha_beta0); /* alpha2 - alpha[0] */
        initalpha2 = alpha_beta2; /* alpha_beta4 is temp buffer for initalpha2; */

        /* Alpha(4) = maxstar(r3 + initAlpha(7), r0 + initAlpha(8) ); */
        alpha_beta2 = _mm_adds_epi8(sigma3, initalpha6);
        alpha_beta2 = _mm_max_epi8(alpha_beta2 , initalpha7);
        alpha_beta2 = _mm_subs_epi8(alpha_beta2, alpha_beta0); /*alpha2 - alpha[0] */
        initalpha3 = alpha_beta2; /* shift initalhpa to next window */

        /* Alpha(7) = maxstar(r2 + initAlpha(5), r1 + initAlpha(6)); */
        alpha_beta2 = _mm_adds_epi8(sigma1, initalpha5);
        alpha_beta3 = _mm_adds_epi8(sigma2, initalpha4);
        alpha_beta2 = _mm_max_epi8(alpha_beta2 , alpha_beta3);
        alpha_beta2 = _mm_subs_epi8(alpha_beta2, alpha_beta0); /*alpha2 - alpha[0] */
        alpha_beta4 = alpha_beta2; /* shift initalhpa to next window initalhpa[6] */

        /* Alpha(8) = maxstar(r0 + initAlpha(7), r3 + initAlpha(8) ); */
        alpha_beta2 = _mm_adds_epi8(sigma3, initalpha7);
        alpha_beta2 = _mm_max_epi8(alpha_beta2 , initalpha6);
        alpha_beta2 = _mm_subs_epi8(alpha_beta2, alpha_beta0); /*alpha2 - alpha[0] */
        initalpha7 = alpha_beta2; /* shift initalhpa to next window */

        initalpha6 =  alpha_beta4;
        initalpha4 =  alpha_beta5;
        initalpha5 =  alpha_beta6;
    }

    initalpha1 =_mm_slli_si128(initalpha1, 1);
    initalpha2 =_mm_slli_si128(initalpha2, 1);
    initalpha3 =_mm_slli_si128(initalpha3, 1);
    initalpha4 =_mm_slli_si128(initalpha4, 1);
    initalpha5 =_mm_slli_si128(initalpha5, 1);
    initalpha6 =_mm_slli_si128(initalpha6, 1);
    initalpha7 =_mm_slli_si128(initalpha7, 1);

    initalpha[1] = _mm_insert_epi8(initalpha1 , -128, 0) ;
    initalpha[2] = _mm_insert_epi8(initalpha2 , -128, 0) ;
    initalpha[3] = _mm_insert_epi8(initalpha3 , -128, 0) ;
    initalpha[4] = _mm_insert_epi8(initalpha4 , -128, 0) ;
    initalpha[5] = _mm_insert_epi8(initalpha5 , -128, 0) ;
    initalpha[6] = _mm_insert_epi8(initalpha6 , -128, 0) ;
    initalpha[7] = _mm_insert_epi8(initalpha7 , -128, 0) ;

    /* end of initalpha */
    /*********** last bit is differenct to update initalhpa *************/

    /*initbeta0 =  initbeta[0];*/
    initbeta1 =  initbeta[1];
    initbeta2 =  initbeta[2];
    initbeta3 =  initbeta[3];
    initbeta4 =  initbeta[4];
    initbeta5 =  initbeta[5];
    initbeta6 =  initbeta[6];
    initbeta7 =  initbeta[7];
    /* start beta calculation */
    for (i=WindowSize-1;i>=0; i--)
    {
        input_addr = (__m128i *)(Tempalpha_sigma) +10*(WindowSize - 1 - i);
        sigma1 = _mm_load_si128((__m128i const*)(input_addr)); /* r1 = xp*/
        input_addr = input_addr + 1;
        sigma2 = _mm_load_si128((__m128i const*)(input_addr)); /* r2 = xs+e */
        input_addr = input_addr + 1;
        sigma3 = _mm_load_si128((__m128i const*)(input_addr)); /*  r3 = xp+xs+e */
        input_addr = input_addr + 1;

        sigma1 = _mm_adds_epi8(sigma1, TD_constantminus80);
        sigma2 = _mm_adds_epi8(sigma2, TD_constantminus80);
        sigma3 = _mm_adds_epi8(sigma3, TD_constantminus80);

        alpha_beta0 = _mm_load_si128((__m128i const*)(input_addr)); /* load alpha[1] */
        input_addr = input_addr + 1;

        /*Lamda(1,1) = alpha(1) +  gama(4) -offset+beta(5)  -offset2  ;*/
        /*Lamda(1,2) = alpha(2) + gama(4) -offset+ beta(1) -offset2; */
        /*Lamda(2,1) = alpha(1) + gama(1) -offset + beta(1) -offset2; */
        /*Lamda(2,2) = alpha(2) + gama(1) -offset +  beta(5)-offset2; */

        alpha_beta2 =  _mm_adds_epi8(sigma3, initbeta4); /*Lamda(1,1) */
        /*Lamda(2,1) = sigma0;*/

        alpha_beta1 =  _mm_adds_epi8(sigma3, alpha_beta0); /* Lamda(1,2) */
        alpha_beta3 =  _mm_adds_epi8(TD_constantminus80, alpha_beta0); /* Lamda(2,2) */
        alpha_beta3 =  _mm_adds_epi8( alpha_beta3, initbeta4); /* Lamda(2,2) */

        alpha_beta2 =  _mm_max_epi8(alpha_beta2 , alpha_beta1); /* max(Lamda(1,1), Lamda(1,2)); */
        alpha_beta3 =  _mm_max_epi8(alpha_beta3 , TD_constantminus80); /* max(Lamda(2,1), Lamda(2,2)); */

        /* Lamda(1,3) = alpha(3) + gama(3)-offset+ beta(2) -offset2; */
        /* Lamda(1,4) = alpha(4) + gama(3) -offset+ beta(6)- offset2; */

        /* Lamda(2,3) = alpha(3) + gama(2) -offset+ beta(6) - offset2; */
        /* Lamda(2,4) = alpha(4) + gama(2) -offset+ beta(2) -offset2; */

        alpha_beta0 = _mm_load_si128((__m128i const*)(input_addr)); /* load alpha[2] => ALpha(3) */
        input_addr = input_addr + 1;
        alpha_beta1 = _mm_load_si128((__m128i const*)(input_addr)); /* load alpha[3] => Alpha(4) */
        input_addr = input_addr + 1;

        alpha_beta4 = _mm_adds_epi8(sigma2, alpha_beta0); /*Lamda(1,3) = alpha(3) + gama(3) */
        alpha_beta5 = _mm_adds_epi8(sigma2, alpha_beta1); /*Lamda(1,4) = alpha(4) + gama(3) */
        alpha_beta6 = _mm_adds_epi8(sigma1, alpha_beta0); /*Lamda(2,3) = alpha(3) + gama(2) */
        alpha_beta7 = _mm_adds_epi8(sigma1, alpha_beta1); /*Lamda(2,4) = alpha(4) + gama(2) */

        alpha_beta4 = _mm_adds_epi8(alpha_beta4, initbeta1); /* Lamda(1,3) = alpha(3) + gama(3) + beta2;*/
        alpha_beta6 = _mm_adds_epi8(alpha_beta6, initbeta5); /*Lamda(2,3) = alpha(3) + gama(2) + beta6 */

        alpha_beta7 = _mm_adds_epi8(alpha_beta7, initbeta1); /*Lamda(2,4) = alpha(3) + gama(2) + beta2 */
        alpha_beta3 = _mm_max_epi8(alpha_beta3 , alpha_beta6); /*max(Lamda(2,1), Lamda(2,2), Lamda(2,3)); */
        alpha_beta2 = _mm_max_epi8(alpha_beta2 , alpha_beta4); /*max(Lamda(1,1), Lamda(1,2), Lamda(1,3)); */
        alpha_beta5 = _mm_adds_epi8(alpha_beta5, initbeta5); /* Lamda(1,4) = alpha(4) + gama(3) + beta6; */
        alpha_beta2 = _mm_max_epi8(alpha_beta2 , alpha_beta5); /* max(Lamda(1,1), Lamda(1,2), Lamda(1,3), Lamda(1,4));*/
        alpha_beta3 = _mm_max_epi8(alpha_beta3 , alpha_beta7); /* max(Lamda(2,1), Lamda(2,2), Lamda(2,3), Lamda(2,4)); */


        /*Lamda(1,5) = alpha(5) +  gama(3)-offset+  beta(7)-offset2; */
        /*Lamda(1,6) = alpha(6) + gama(3) -offset+ beta(3)- offset2; */
        /*Lamda(2,5) = alpha(5) + gama(2) -offset+ beta(3)- offset2; */
        /*Lamda(2,6) = alpha(6) + gama(2) -offset+ beta(7)- offset2; */
        alpha_beta0 = _mm_load_si128((__m128i const*)(input_addr)); /* load alpha[4] */
        input_addr = input_addr + 1;
        alpha_beta1 = _mm_load_si128((__m128i const*)(input_addr)); /* load alpha[5] */
        input_addr = input_addr + 1;

        alpha_beta4 = _mm_adds_epi8(sigma2, alpha_beta0); /* Lamda(1,5) = alpha(5) + gama(3) */
        alpha_beta5 = _mm_adds_epi8(sigma2, alpha_beta1); /* Lamda(1,6) = alpha(6) + gama(3) */
        alpha_beta6 = _mm_adds_epi8(sigma1, alpha_beta0); /* Lamda(2,5) = alpha(5) + gama(2) */
        alpha_beta7 = _mm_adds_epi8(sigma1, alpha_beta1); /* Lamda(2,6) = alpha(6) + gama(2) */

        alpha_beta4 = _mm_adds_epi8(alpha_beta4, initbeta6); /* Lamda(1,5) = alpha(5) + gama(3) + beta6;*/
        alpha_beta6 = _mm_adds_epi8(alpha_beta6, initbeta2); /* Lamda(2,5) = alpha(5) + gama(2) + beta3 */
        alpha_beta7 = _mm_adds_epi8(alpha_beta7, initbeta6); /* Lamda(2,6) = alpha(6) + gama(2) + beta7 */
        alpha_beta5 = _mm_adds_epi8(alpha_beta5, initbeta2); /* Lamda(1,6) = alpha(4) + gama(3) + beta3; */

        alpha_beta2 = _mm_max_epi8(alpha_beta2 , alpha_beta4); /* max(Lamda(1,1), Lamda(1,2), Lamda(1,3), Lamda(1,4) Lamda(1,5)); */
        alpha_beta3 = _mm_max_epi8(alpha_beta3 , alpha_beta6); /* max(Lamda(2,1), Lamda(2,2), Lamda(2,3), Lamda(2,4) Lamda(2,5)); */


        alpha_beta2 = _mm_max_epi8(alpha_beta2 , alpha_beta5); /* max(Lamda(1,1), Lamda(1,2), Lamda(1,3), Lamda(1,4) Lamda(1,5) Lamda(1,6)); */
        alpha_beta3 = _mm_max_epi8(alpha_beta3 , alpha_beta7); /* max(Lamda(2,1), Lamda(2,2), Lamda(2,3), Lamda(2,4) Lamda(2,5) Lamda(2,6) ); */


        /* Lamda(1,7) = alpha(7) + gama(4) -offset+  beta(4)-offset2; */
        /* Lamda(1,8) = alpha(8) + gama(4)-offset+ beta(8) -offset2 ; */
        /* Lamda(2,7) = alpha(7) + gama(1) -offset+ beta(8) -offset2; */
        /* Lamda(2,8) = alpha(8) + gama(1) -offset+ beta(4) -offset2; */
        alpha_beta0 = _mm_load_si128((__m128i const*)(input_addr)); /* load alpha[6] */
        input_addr = input_addr + 1;
        alpha_beta1 = _mm_load_si128((__m128i const*)(input_addr)); /* load alpha[7] */

        alpha_beta4 = _mm_adds_epi8(sigma3, alpha_beta0); /* Lamda(1,7) = alpha(7) + gama(4) */
        alpha_beta5 = _mm_adds_epi8(sigma3, alpha_beta1); /* Lamda(1,8) = alpha(8) + gama(4) */
        alpha_beta6 = _mm_adds_epi8(TD_constantminus80, alpha_beta0); /* Lamda(2,7) = alpha(7) + gama(1) */
        alpha_beta7 = _mm_adds_epi8(TD_constantminus80, alpha_beta1); /* Lamda(2,8) = alpha(8) + gama(1) */

        alpha_beta4 = _mm_adds_epi8(alpha_beta4, initbeta3); /* Lamda(1,7) = alpha(7) + gama(4) +  beta(4); */
        alpha_beta6 = _mm_adds_epi8(alpha_beta6, initbeta7); /* Lamda(2,7) = alpha(7) + gama(1) + beta(8); */
        alpha_beta7 = _mm_adds_epi8(alpha_beta7, initbeta3); /* Lamda(2,8) = alpha(8) + gama(1) +  beta4 */
        alpha_beta5 = _mm_adds_epi8(alpha_beta5, initbeta7); /* Lamda(1,8) = alpha(8) + gama(4)+ beta(8) */

        alpha_beta2 =  _mm_max_epi8(alpha_beta2 , alpha_beta4);
        alpha_beta3 =  _mm_max_epi8(alpha_beta3 , alpha_beta6);
        alpha_beta2 =  _mm_max_epi8(alpha_beta2 , alpha_beta5);
        alpha_beta3 =  _mm_max_epi8(alpha_beta3 , alpha_beta7);

        /******************************************************************/
        alpha_beta2 = _mm_subs_epi8(alpha_beta2, alpha_beta3); /* Lamda obtained */

        alpha_beta3 = _mm_abs_epi8(alpha_beta2); /* abs(Lamda); */
        min_distance  = _mm_min_epi8(min_distance,alpha_beta3);


        /* E = 0.75*(Lamda - Xs - E;) */
        alpha_beta4 = _mm_subs_epi8(sigma2, TD_constantminus80);
        alpha_beta3 = _mm_subs_epi8(alpha_beta2,alpha_beta4);

        /* save E, that's in alpha_beta3 */
        out_line_addr = *(DeInterleaverInterRowAddr+i);
        out_line_addr = (out_line_addr<<5) + (out_line_addr<<4); /* out_line_addr = out_line_addr * 48; */
        pattern_sel = *(DeInterleaverIntraRowPatSel+i);
        alpha_beta1 = _mm_load_si128((__m128i const*)(DeInterleaverIntraRowPattern+(pattern_sel<<4))); /* pattern_sel * 16 */
        alpha_beta3 = _mm_shuffle_epi8 (alpha_beta3, alpha_beta1);
        _mm_store_si128((__m128i *)(OutputAddress+out_line_addr), alpha_beta3);

        /* alpha_beta3  E = e*0.75; */
        /******************************************************************/

        /******** update beta ********/
        /* Beta(1) = maxstar(r0 + initBeta(1), r3 + initBeta(5) ); */

        alpha_beta0 = _mm_adds_epi8(sigma3, initbeta4);
        alpha_beta0 = _mm_max_epi8(alpha_beta0,TD_constantminus80  ); /*beta(1) saved in alpha_beta0;*/


        /* Beta(2) = maxstar(r3 + initBeta(1), r0 + initBeta(5) );*/
        alpha_beta1 = _mm_adds_epi8(TD_constantminus80, initbeta4);
        alpha_beta1 = _mm_max_epi8(alpha_beta1 , sigma3); /* beta(2) saved in alpha_beta1; */
        alpha_beta1 = _mm_subs_epi8(alpha_beta1,alpha_beta0);

        /* Beta(3) = maxstar(r2 + initBeta(2), r1 + initBeta(6) ); */
        alpha_beta2 = _mm_adds_epi8(sigma1, initbeta5);
        alpha_beta3 = _mm_adds_epi8(sigma2, initbeta1);
        alpha_beta2 = _mm_max_epi8(alpha_beta2, alpha_beta3);  /* beta(3) saved in alpha_beta2 */
        alpha_beta2 = _mm_subs_epi8(alpha_beta2,alpha_beta0);

        /*initbeta1 = alpha_beta1; //release alpha_beta1; */

        /* Beta(4) = maxstar(r1 + initBeta(2), r2 + initBeta(6) ); */
        alpha_beta4 = _mm_adds_epi8(sigma2, initbeta5);
        alpha_beta3 = _mm_adds_epi8(sigma1, initbeta1);
        alpha_beta7 = _mm_max_epi8(alpha_beta4,alpha_beta3);  /* beta(4) saved in alpha_beta7 */
        alpha_beta7 = _mm_subs_epi8(alpha_beta7,alpha_beta0);

        /* Beta(5) = maxstar(r1 + initBeta(3), r2 + initBeta(7) ); */
        alpha_beta3 = _mm_adds_epi8(sigma2, initbeta6);
        alpha_beta4 = _mm_adds_epi8(sigma1, initbeta2);
        initbeta4   = _mm_max_epi8(alpha_beta4,alpha_beta3);
        initbeta4 = _mm_subs_epi8(initbeta4,alpha_beta0);

        /* Beta(6) = maxstar(r2 + initBeta(3), r1 + initBeta(7) ); */
        alpha_beta3 = _mm_adds_epi8(sigma1, initbeta6);
        alpha_beta4 = _mm_adds_epi8(sigma2, initbeta2);
        initbeta5   = _mm_max_epi8(alpha_beta3,alpha_beta4);
        initbeta5 = _mm_subs_epi8(initbeta5,alpha_beta0);

        /* Beta(7) = maxstar(r3 + initBeta(4), r0 + initBeta(8) ); */
        alpha_beta3 = _mm_adds_epi8(TD_constantminus80, initbeta7);
        alpha_beta4 = _mm_adds_epi8(sigma3, initbeta3);
        initbeta6   = _mm_max_epi8(alpha_beta3,alpha_beta4);
        initbeta6 = _mm_subs_epi8(initbeta6,alpha_beta0);
        /* Beta(8) = maxstar(r0 + initBeta(4), r3 + initBeta(8) ); */
        alpha_beta3 = _mm_adds_epi8(sigma3, initbeta7);
        alpha_beta4 = _mm_adds_epi8(TD_constantminus80, initbeta3);
        initbeta7   = _mm_max_epi8(alpha_beta3,alpha_beta4);
        initbeta7 = _mm_subs_epi8(initbeta7,alpha_beta0);


        initbeta1 = alpha_beta1;
        initbeta2 = alpha_beta2;
        initbeta3 =  alpha_beta7;

        /******** end of beta calculation *********/

    }
    initbeta1 =_mm_srli_si128(initbeta1, 1);
    initbeta2 =_mm_srli_si128(initbeta2, 1);
    initbeta3 =_mm_srli_si128(initbeta3, 1);
    initbeta4 =_mm_srli_si128(initbeta4, 1);
    initbeta5 =_mm_srli_si128(initbeta5, 1);
    initbeta6 =_mm_srli_si128(initbeta6, 1);
    initbeta7 =_mm_srli_si128(initbeta7, 1);

    initbeta[1] = _mm_insert_epi8(initbeta1 , tailbeta[1], 15) ;
    initbeta[2] = _mm_insert_epi8(initbeta2 , tailbeta[2], 15) ;
    initbeta[3] = _mm_insert_epi8(initbeta3 , tailbeta[3], 15) ;
    initbeta[4] = _mm_insert_epi8(initbeta4 , tailbeta[4], 15) ;
    initbeta[5] = _mm_insert_epi8(initbeta5 , tailbeta[5], 15) ;
    initbeta[6] = _mm_insert_epi8(initbeta6 , tailbeta[6], 15) ;
    initbeta[7] = _mm_insert_epi8(initbeta7 , tailbeta[7], 15) ;

    min_distance  = _mm_abs_epi8(min_distance);
    alpha_beta1 = _mm_minpos_epu16(min_distance);
    alpha_beta2 = _mm_slli_si128(min_distance,1);
    alpha_beta2 = _mm_minpos_epu16(alpha_beta2);
    alpha_beta1 = _mm_srli_si128(alpha_beta1,1);
    alpha_beta2 = _mm_srli_si128(alpha_beta2,1);
    alpha_beta1 = _mm_max_epi8(alpha_beta1, alpha_beta2);


    return _mm_extract_epi8(alpha_beta1,0);
}

void BitTranspose_16windows(int32_t K, uint16_t * pin, uint8_t * pout)
{

    int32_t i, j;

    int32_t Lwin = K >> 4;
    int32_t vLen;
    if (Lwin%8==0) vLen = Lwin >> 3;
    else vLen = (Lwin >> 3) + 1;

    __m128i v1, v2;
    int32_t tmp0, tmp1, tmp2, tmp3, tmp4, tmp5, tmp6, tmp7;
    __declspec (align(64)) uint8_t mat_out[16][48];

    for (i=0; i<vLen; i++)
    {
        v1 = _mm_load_si128((__m128i *)pin);
        pin += 8;

        v2 = _mm_shuffle_epi8(v1, TD_vshuf1_BitTranspose_16windows); tmp0 = _mm_movemask_epi8(v2); v1 = _mm_slli_epi16(v1, 1);
        v2 = _mm_shuffle_epi8(v1, TD_vshuf1_BitTranspose_16windows); tmp1 = _mm_movemask_epi8(v2); v1 = _mm_slli_epi16(v1, 1);
        v2 = _mm_shuffle_epi8(v1, TD_vshuf1_BitTranspose_16windows); tmp2 = _mm_movemask_epi8(v2); v1 = _mm_slli_epi16(v1, 1);
        v2 = _mm_shuffle_epi8(v1, TD_vshuf1_BitTranspose_16windows); tmp3 = _mm_movemask_epi8(v2); v1 = _mm_slli_epi16(v1, 1);
        v2 = _mm_shuffle_epi8(v1, TD_vshuf1_BitTranspose_16windows); tmp4 = _mm_movemask_epi8(v2); v1 = _mm_slli_epi16(v1, 1);
        v2 = _mm_shuffle_epi8(v1, TD_vshuf1_BitTranspose_16windows); tmp5 = _mm_movemask_epi8(v2); v1 = _mm_slli_epi16(v1, 1);
        v2 = _mm_shuffle_epi8(v1, TD_vshuf1_BitTranspose_16windows); tmp6 = _mm_movemask_epi8(v2); v1 = _mm_slli_epi16(v1, 1);
        v2 = _mm_shuffle_epi8(v1, TD_vshuf1_BitTranspose_16windows); tmp7 = _mm_movemask_epi8(v2); v1 = _mm_slli_epi16(v1, 1);

        mat_out[8][i] = tmp0; tmp0 = tmp0>>8; mat_out[0][i] = tmp0;
        mat_out[9][i] = tmp1; tmp1 = tmp1>>8; mat_out[1][i] = tmp1;
        mat_out[10][i] = tmp2; tmp2 = tmp2>>8; mat_out[2][i] = tmp2;
        mat_out[11][i] = tmp3; tmp3 = tmp3>>8; mat_out[3][i] = tmp3;
        mat_out[12][i] = tmp4; tmp4 = tmp4>>8; mat_out[4][i] = tmp4;
        mat_out[13][i] = tmp5; tmp5 = tmp5>>8; mat_out[5][i] = tmp5;
        mat_out[14][i] = tmp6; tmp6 = tmp6>>8; mat_out[6][i] = tmp6;
        mat_out[15][i] = tmp7; tmp7 = tmp7>>8; mat_out[7][i] = tmp7;
    }

    /* ------------------------------------------------------------------------------------------ */
    int32_t resBitNum;
    int32_t newBitNum;
    int32_t tailBitNum = Lwin & 63;
    int32_t Num64BitIn = Lwin >> 6;

    __m128i vnow, vnext, vnewbits, vmask;
    uint8_t * ptmp = &(mat_out[0][0]);

    if (tailBitNum==0)
    {
        for (i=0; i<16; i++)
        {
            ptmp = &(mat_out[i][0]); /* read from a new line*/
            for (j=0; j<Num64BitIn; j++)
            {
            vnow = _mm_loadl_epi64((__m128i const *)ptmp); ptmp += 8;
            _mm_storel_epi64((__m128i *)pout, vnow); pout += 8;
            }
        }
    }
    else
    {
        resBitNum = 0;
        vnow = _mm_setzero_si128();
        for (i=0; i<16; i++)
        {
            ptmp = &(mat_out[i][0]); /* read from a new line */

            newBitNum = 64 - resBitNum; /* there're resBitNum old bits in vnow's high end */
            vmask = _mm_set1_epi32(-1);
            vmask = _mm_slli_epi64(vmask, newBitNum); /* prepare to remove newBitNum null bits from vnow's low end */
            vnow = _mm_and_si128(vnow, vmask); /* remove newBitNum null bits from vnow's low end */

            for (j=0; j<Num64BitIn; j++)
            {
                vnext = _mm_loadl_epi64((__m128i const *)ptmp); ptmp += 8; /* read 64 new bits in */
                vnext = _mm_shuffle_epi8(vnext, TD_vshuf2_BitTranspose_16windows); /* little endian to big endian */
                vnewbits = _mm_srli_epi64(vnext, resBitNum); /* get newBitNum new bits in vnewbits's low end */
                vnow = _mm_or_si128(vnow, vnewbits); /* combined into vnow */
                vnow = _mm_shuffle_epi8(vnow, TD_vshuf2_BitTranspose_16windows); /* big endian to little endian */
                _mm_storel_epi64((__m128i *)pout, vnow); pout += 8; /* store */
                vnow = _mm_slli_epi64(vnext, newBitNum); /* get resBitNum old bits in vnow's high end */
                vnow = _mm_and_si128(vnow, vmask); /* remove newBitNum null bits from vnow's low end */
            }

            if (resBitNum+tailBitNum>=64) /* with tailBitNum new bits, it is enough for 64 bits plus resBitNum old bits */
            {
                vnext = _mm_loadl_epi64((__m128i const *)ptmp); /* ptmp += 8; // read 64 new bits in */
                vnext = _mm_shuffle_epi8(vnext, TD_vshuf2_BitTranspose_16windows); /* little endian to big endian */
                vnewbits = _mm_srli_epi64(vnext, resBitNum); /* get newBitNum new bits in vnewbits's low end */
                vnow = _mm_or_si128(vnow, vnewbits); /* combined into vnow */
                vnow = _mm_shuffle_epi8(vnow, TD_vshuf2_BitTranspose_16windows); /* big endian to little endian */
                _mm_storel_epi64((__m128i *)pout, vnow); pout += 8; /* store */
                vnow = _mm_slli_epi64(vnext, newBitNum); /* get resBitNum old bits in vnow's high end */
                resBitNum = resBitNum + tailBitNum - 64; /* now resBitNum is resBitNum+tailBitNum-64 */
            }
            else
            {
                vnext = _mm_loadl_epi64((__m128i const *)ptmp); /*ptmp += 8; // read 64 new bits in */
                vnext = _mm_shuffle_epi8(vnext, TD_vshuf2_BitTranspose_16windows); /* little endian to big endian */
                vnewbits = _mm_srli_epi64(vnext, resBitNum); /* get newBitNum new bits in vnewbits's low end */
                vnow = _mm_or_si128(vnow, vnewbits); /* combined into vnow */
                resBitNum = resBitNum + tailBitNum; /* now resBitNum is resBitNum+tailBitNum */
            }
        }
        if (resBitNum!=0)
        {
            vnow = _mm_shuffle_epi8(vnow, TD_vshuf2_BitTranspose_16windows); /* big endian to little endian*/
            _mm_storel_epi64((__m128i *)pout, vnow);
        }
    }

    return;
}
#else
void BitTranspose_16windows(int32_t K, uint16_t * pin, uint8_t * pout)
{
    printf("bblib_turbo requires at least SSE4.2 ISA support to run\n");
    exit(-1);
}
#endif
