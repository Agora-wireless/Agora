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
 * @file   phy_turbo_fastInterleave_sse.cpp
 * @brief  Implementaion of the LTE turbo fast interleave base on the SSE instruction set.
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>

#include <immintrin.h>

#include "phy_turbo_internal.h"
#if defined(_BBLIB_SSE4_2_) || defined(_BBLIB_AVX2_) || defined(_BBLIB_AVX512_)
#define MAX_PATH_LEN 1024

/**@struct _Turbo_Interleaver_Input_Assistant
 * @brief this sturcture is Turbo Interleave structure
 */
typedef struct 
{
    int32_t K;
    int32_t Lwin;
    int32_t Nseg;
    int16_t ByteInAddr[7][8];
    int8_t BitLeftShift[7][8];
    int32_t nTailByte;
    int32_t nTailBit;
} _Turbo_Interleaver_Input_Assistant;

/**@struct _Turbo_Interleaver_Input_Assistant
 * @brief this sturcture is Turbo Interleave para structure
 */
typedef struct 
{
    uint8_t * pInput;
    uint8_t * pOutput;
    __m128i * pvIntra_row_perm_shuffle_vector;
    int8_t * p_intra_row_perm_mode;
    int16_t * p_inter_row_out_addr;
    _Turbo_Interleaver_Input_Assistant * p_Input_Assistant;
} _Turbo_Interleaver_Para;

/* FastInterleave Table */
__m128i g_vIntra_row_perm_shuffle_vector[64];
int8_t g_intra_row_perm_mode_sdk[188][840];
int16_t g_inter_row_out_addr_sdk[188][840];
_Turbo_Interleaver_Input_Assistant g_Turbo_Intx_Input_Assistant_sdk[188];


#define _MM_BR_EPI128(vin_t, vout_t)\
{\
        vout_t = _mm_setzero_si128 ();\
        \
        vtmp_t = _mm_srli_epi16 (vin_t, 7);\
        vtmp_t = _mm_and_si128 (vtmp_t, vmask1);\
        vout_t = _mm_or_si128(vout_t, vtmp_t);\
\
        vtmp_t = _mm_srli_epi16 (vin_t, 5);\
        vtmp_t = _mm_and_si128 (vtmp_t, vmask2);\
        vout_t = _mm_or_si128(vout_t, vtmp_t);\
\
        vtmp_t = _mm_srli_epi16 (vin_t, 3);\
        vtmp_t = _mm_and_si128 (vtmp_t, vmask3);\
        vout_t = _mm_or_si128(vout_t, vtmp_t);\
\
        vtmp_t = _mm_srli_epi16 (vin_t, 1);\
        vtmp_t = _mm_and_si128 (vtmp_t, vmask4);\
        vout_t = _mm_or_si128(vout_t, vtmp_t);\
\
        vtmp_t = _mm_slli_epi16 (vin_t, 1);\
        vtmp_t = _mm_and_si128 (vtmp_t, vmask5);\
        vout_t = _mm_or_si128(vout_t, vtmp_t);\
\
        vtmp_t = _mm_slli_epi16 (vin_t, 3);\
        vtmp_t = _mm_and_si128 (vtmp_t, vmask6);\
        vout_t = _mm_or_si128(vout_t, vtmp_t);\
\
        vtmp_t = _mm_slli_epi16 (vin_t, 5);\
        vtmp_t = _mm_and_si128 (vtmp_t, vmask7);\
        vout_t = _mm_or_si128(vout_t, vtmp_t);\
\
        vtmp_t = _mm_slli_epi16 (vin_t, 7);\
        vtmp_t = _mm_and_si128 (vtmp_t, vmask8);\
        vout_t = _mm_or_si128(vout_t, vtmp_t);\
}


#define _MM_SLLI_EPI128(vin, vout, count) \
{ \
    vtmp0 = _mm_srli_epi64 (vin, count); \
    vtmp1 = _mm_shuffle_epi8 (vin, vshuf); \
    vtmp2 = _mm_slli_epi64 (vtmp1, 64-count); \
    vout = _mm_or_si128 (vtmp0, vtmp2); \
}

#define BIT_MATRIX_TRANSPOSE_8BYTEOUT_8Windows(vin) \
{ \
    vtmp0 = _mm_slli_epi16 (vin, 15); \
    vtmp1 = _mm_slli_epi16 (vin, 14); \
    vtmp2 = _mm_slli_epi16 (vin, 13); \
    vtmp3 = _mm_slli_epi16 (vin, 12); \
    vtmp4 = _mm_slli_epi16 (vin, 11); \
    vtmp5 = _mm_slli_epi16 (vin, 10); \
    vtmp6 = _mm_slli_epi16 (vin, 9); \
    vtmp7 = _mm_slli_epi16 (vin, 8); \
    cnt_tmp = cnt; \
    vtmp0 = _mm_shuffle_epi8 (vtmp0, *(pvIntra_row_perm_shuffle_vector+(*(p_intra_row_perm_mode+cnt++)))); \
    vtmp1 = _mm_shuffle_epi8 (vtmp1, *(pvIntra_row_perm_shuffle_vector+(*(p_intra_row_perm_mode+cnt++)))); \
    vtmp2 = _mm_shuffle_epi8 (vtmp2, *(pvIntra_row_perm_shuffle_vector+(*(p_intra_row_perm_mode+cnt++)))); \
    vtmp3 = _mm_shuffle_epi8 (vtmp3, *(pvIntra_row_perm_shuffle_vector+(*(p_intra_row_perm_mode+cnt++)))); \
    vtmp4 = _mm_shuffle_epi8 (vtmp4, *(pvIntra_row_perm_shuffle_vector+(*(p_intra_row_perm_mode+cnt++)))); \
    vtmp5 = _mm_shuffle_epi8 (vtmp5, *(pvIntra_row_perm_shuffle_vector+(*(p_intra_row_perm_mode+cnt++)))); \
    vtmp6 = _mm_shuffle_epi8 (vtmp6, *(pvIntra_row_perm_shuffle_vector+(*(p_intra_row_perm_mode+cnt++)))); \
    vtmp7 = _mm_shuffle_epi8 (vtmp7, *(pvIntra_row_perm_shuffle_vector+(*(p_intra_row_perm_mode+cnt++)))); \
    tmp_buf[*(p_inter_row_out_addr+cnt_tmp++)] = _mm_movemask_epi8 (vtmp0); \
    tmp_buf[*(p_inter_row_out_addr+cnt_tmp++)] = _mm_movemask_epi8 (vtmp1); \
    tmp_buf[*(p_inter_row_out_addr+cnt_tmp++)] = _mm_movemask_epi8 (vtmp2); \
    tmp_buf[*(p_inter_row_out_addr+cnt_tmp++)] = _mm_movemask_epi8 (vtmp3); \
    tmp_buf[*(p_inter_row_out_addr+cnt_tmp++)] = _mm_movemask_epi8 (vtmp4); \
    tmp_buf[*(p_inter_row_out_addr+cnt_tmp++)] = _mm_movemask_epi8 (vtmp5); \
    tmp_buf[*(p_inter_row_out_addr+cnt_tmp++)] = _mm_movemask_epi8 (vtmp6); \
    tmp_buf[*(p_inter_row_out_addr+cnt_tmp++)] = _mm_movemask_epi8 (vtmp7); \
}

#define BIT_MATRIX_TRANSPOSE_16BYTEOUT_8Windows(vin) \
{ \
    vtmp0 = _mm_slli_epi16 (vin, 15); \
    vtmp1 = _mm_slli_epi16 (vin, 14); \
    vtmp2 = _mm_slli_epi16 (vin, 13); \
    vtmp3 = _mm_slli_epi16 (vin, 12); \
    vtmp4 = _mm_slli_epi16 (vin, 11); \
    vtmp5 = _mm_slli_epi16 (vin, 10); \
    vtmp6 = _mm_slli_epi16 (vin, 9); \
    vtmp7 = _mm_slli_epi16 (vin, 8); \
    vtmp8 = _mm_slli_epi16 (vin, 7); \
    vtmp9 = _mm_slli_epi16 (vin, 6); \
    vtmpa = _mm_slli_epi16 (vin, 5); \
    vtmpb = _mm_slli_epi16 (vin, 4); \
    vtmpc = _mm_slli_epi16 (vin, 3); \
    vtmpd = _mm_slli_epi16 (vin, 2); \
    vtmpe = _mm_slli_epi16 (vin, 1); \
    vtmpf = _mm_slli_epi16 (vin, 0); \
    cnt_tmp = cnt; \
    vtmp0 = _mm_shuffle_epi8 (vtmp0, *(pvIntra_row_perm_shuffle_vector+(*(p_intra_row_perm_mode+cnt++)))); \
    vtmp1 = _mm_shuffle_epi8 (vtmp1, *(pvIntra_row_perm_shuffle_vector+(*(p_intra_row_perm_mode+cnt++)))); \
    vtmp2 = _mm_shuffle_epi8 (vtmp2, *(pvIntra_row_perm_shuffle_vector+(*(p_intra_row_perm_mode+cnt++)))); \
    vtmp3 = _mm_shuffle_epi8 (vtmp3, *(pvIntra_row_perm_shuffle_vector+(*(p_intra_row_perm_mode+cnt++)))); \
    vtmp4 = _mm_shuffle_epi8 (vtmp4, *(pvIntra_row_perm_shuffle_vector+(*(p_intra_row_perm_mode+cnt++)))); \
    vtmp5 = _mm_shuffle_epi8 (vtmp5, *(pvIntra_row_perm_shuffle_vector+(*(p_intra_row_perm_mode+cnt++)))); \
    vtmp6 = _mm_shuffle_epi8 (vtmp6, *(pvIntra_row_perm_shuffle_vector+(*(p_intra_row_perm_mode+cnt++)))); \
    vtmp7 = _mm_shuffle_epi8 (vtmp7, *(pvIntra_row_perm_shuffle_vector+(*(p_intra_row_perm_mode+cnt++)))); \
    vtmp8 = _mm_shuffle_epi8 (vtmp8, *(pvIntra_row_perm_shuffle_vector+(*(p_intra_row_perm_mode+cnt++)))); \
    vtmp9 = _mm_shuffle_epi8 (vtmp9, *(pvIntra_row_perm_shuffle_vector+(*(p_intra_row_perm_mode+cnt++)))); \
    vtmpa = _mm_shuffle_epi8 (vtmpa, *(pvIntra_row_perm_shuffle_vector+(*(p_intra_row_perm_mode+cnt++)))); \
    vtmpb = _mm_shuffle_epi8 (vtmpb, *(pvIntra_row_perm_shuffle_vector+(*(p_intra_row_perm_mode+cnt++)))); \
    vtmpc = _mm_shuffle_epi8 (vtmpc, *(pvIntra_row_perm_shuffle_vector+(*(p_intra_row_perm_mode+cnt++)))); \
    vtmpd = _mm_shuffle_epi8 (vtmpd, *(pvIntra_row_perm_shuffle_vector+(*(p_intra_row_perm_mode+cnt++)))); \
    vtmpe = _mm_shuffle_epi8 (vtmpe, *(pvIntra_row_perm_shuffle_vector+(*(p_intra_row_perm_mode+cnt++)))); \
    vtmpf = _mm_shuffle_epi8 (vtmpf, *(pvIntra_row_perm_shuffle_vector+(*(p_intra_row_perm_mode+cnt++)))); \
    tmp_buf[*(p_inter_row_out_addr+cnt_tmp++)] = _mm_movemask_epi8 (vtmp0); \
    tmp_buf[*(p_inter_row_out_addr+cnt_tmp++)] = _mm_movemask_epi8 (vtmp1); \
    tmp_buf[*(p_inter_row_out_addr+cnt_tmp++)] = _mm_movemask_epi8 (vtmp2); \
    tmp_buf[*(p_inter_row_out_addr+cnt_tmp++)] = _mm_movemask_epi8 (vtmp3); \
    tmp_buf[*(p_inter_row_out_addr+cnt_tmp++)] = _mm_movemask_epi8 (vtmp4); \
    tmp_buf[*(p_inter_row_out_addr+cnt_tmp++)] = _mm_movemask_epi8 (vtmp5); \
    tmp_buf[*(p_inter_row_out_addr+cnt_tmp++)] = _mm_movemask_epi8 (vtmp6); \
    tmp_buf[*(p_inter_row_out_addr+cnt_tmp++)] = _mm_movemask_epi8 (vtmp7); \
    tmp_buf[*(p_inter_row_out_addr+cnt_tmp++)] = _mm_movemask_epi8 (vtmp8); \
    tmp_buf[*(p_inter_row_out_addr+cnt_tmp++)] = _mm_movemask_epi8 (vtmp9); \
    tmp_buf[*(p_inter_row_out_addr+cnt_tmp++)] = _mm_movemask_epi8 (vtmpa); \
    tmp_buf[*(p_inter_row_out_addr+cnt_tmp++)] = _mm_movemask_epi8 (vtmpb); \
    tmp_buf[*(p_inter_row_out_addr+cnt_tmp++)] = _mm_movemask_epi8 (vtmpc); \
    tmp_buf[*(p_inter_row_out_addr+cnt_tmp++)] = _mm_movemask_epi8 (vtmpd); \
    tmp_buf[*(p_inter_row_out_addr+cnt_tmp++)] = _mm_movemask_epi8 (vtmpe); \
    tmp_buf[*(p_inter_row_out_addr+cnt_tmp++)] = _mm_movemask_epi8 (vtmpf); \
}

/** @fn lte_turbo_interleaver_8windows_sse
 *  @brief  Interleaver function
 *  @param [in out] p_para
 *  @return void
 */
void turbo_interleaver_8windows_sse(_Turbo_Interleaver_Para * p_para)
{
    /* internal temp buffer */
    uint8_t tmp_buf[769];
    uint16_t tmp_buf_2[384];

    __m128i vtmp_t;

    __m128i vmask1 = _mm_set1_epi8(0x01);
    __m128i vmask2 = _mm_set1_epi8(0x02);
    __m128i vmask3 = _mm_set1_epi8(0x04);
    __m128i vmask4 = _mm_set1_epi8(0x08);
    __m128i vmask5 = _mm_set1_epi8(0x10);
    __m128i vmask6 = _mm_set1_epi8(0x20);
    __m128i vmask7 = _mm_set1_epi8(0x40);
    __m128i vmask8 = _mm_set1_epi8(0x80);
    
    uint8_t * pInput = p_para->pInput;
    uint8_t * pOutput = p_para->pOutput;
    __m128i * pvIntra_row_perm_shuffle_vector = p_para->pvIntra_row_perm_shuffle_vector;
    int8_t * p_intra_row_perm_mode = p_para->p_intra_row_perm_mode;
    int16_t * p_inter_row_out_addr = p_para->p_inter_row_out_addr;
    _Turbo_Interleaver_Input_Assistant * p_Input_Assistant = p_para->p_Input_Assistant;

    int32_t K = p_Input_Assistant->K;
    int32_t Nseg = p_Input_Assistant->Nseg;

    __m128i * ptmp;
    uint16_t * ptmp2;

    int32_t i, j, cnt, cnt_tmp;
    __m128i vA70, vB70, vC70, vD70, vE70, vF70, vG70, vH70;
    __m128i vA701, vB701, vC701, vD701, vE701, vF701, vG701, vH701;
    __m128i vBA30, vBA74, vDC30, vDC74, vFE30, vFE74, vHG30, vHG74;
    __m128i vDCBA10, vDCBA32, vDCBA54, vDCBA76, vHGFE10, vHGFE32, vHGFE54, vHGFE76;
    __m128i vHGFEBCDA0, vHGFEBCDA1, vHGFEBCDA2, vHGFEBCDA3, vHGFEBCDA4, vHGFEBCDA5, vHGFEBCDA6, vHGFEBCDA7;
    __m128i vtmp0, vtmp1, vtmp2, vtmp3, vtmp4, vtmp5, vtmp6, vtmp7;
    __m128i vtmp8, vtmp9, vtmpa, vtmpb, vtmpc, vtmpd, vtmpe, vtmpf;
    __m128i vshuf = _mm_set_epi8 (7, 6, 5, 4, 3, 2, 1, 0, 15, 14, 13, 12, 11, 10, 9, 8);
    __m128i vAllOne = _mm_set1_epi8 (0xff);
    __m128i vout, vout1;

    /* matrix transpose 1, intra-row shuffle and inter-row shuffle */
    cnt = 0;
    ptmp = (__m128i *) tmp_buf;
    for (i=0; i<Nseg; i++)
    {
        vA701 = _mm_lddqu_si128 ((__m128i const*) (pInput+p_Input_Assistant->ByteInAddr[i][0]));
        _MM_BR_EPI128(vA701, vA70);        _MM_SLLI_EPI128(vA70, vA70, p_Input_Assistant->BitLeftShift[i][0]);
        vB701 = _mm_lddqu_si128 ((__m128i const*) (pInput+p_Input_Assistant->ByteInAddr[i][1]));
        _MM_BR_EPI128(vB701, vB70);        _MM_SLLI_EPI128(vB70, vB70, p_Input_Assistant->BitLeftShift[i][1]);
        vC701 = _mm_lddqu_si128 ((__m128i const*) (pInput+p_Input_Assistant->ByteInAddr[i][2]));
        _MM_BR_EPI128(vC701, vC70);        _MM_SLLI_EPI128(vC70, vC70, p_Input_Assistant->BitLeftShift[i][2]);
        vD701 = _mm_lddqu_si128 ((__m128i const*) (pInput+p_Input_Assistant->ByteInAddr[i][3]));
        _MM_BR_EPI128(vD701, vD70);        _MM_SLLI_EPI128(vD70, vD70, p_Input_Assistant->BitLeftShift[i][3]);
        vE701 = _mm_lddqu_si128 ((__m128i const*) (pInput+p_Input_Assistant->ByteInAddr[i][4]));
        _MM_BR_EPI128(vE701, vE70);        _MM_SLLI_EPI128(vE70, vE70, p_Input_Assistant->BitLeftShift[i][4]);
        vF701 = _mm_lddqu_si128 ((__m128i const*) (pInput+p_Input_Assistant->ByteInAddr[i][5]));
        _MM_BR_EPI128(vF701, vF70);        _MM_SLLI_EPI128(vF70, vF70, p_Input_Assistant->BitLeftShift[i][5]);
        vG701 = _mm_lddqu_si128 ((__m128i const*) (pInput+p_Input_Assistant->ByteInAddr[i][6]));
        _MM_BR_EPI128(vG701, vG70);        _MM_SLLI_EPI128(vG70, vG70, p_Input_Assistant->BitLeftShift[i][6]);
        vH701 = _mm_lddqu_si128 ((__m128i const*) (pInput+p_Input_Assistant->ByteInAddr[i][7]));
        _MM_BR_EPI128(vH701, vH70);        _MM_SLLI_EPI128(vH70, vH70, p_Input_Assistant->BitLeftShift[i][7]);

        vBA30 = _mm_unpacklo_epi16 (vA70, vB70); 
        vBA74 = _mm_unpackhi_epi16 (vA70, vB70);
        vDC30 = _mm_unpacklo_epi16 (vC70, vD70);
        vDC74 = _mm_unpackhi_epi16 (vC70, vD70);
        vFE30 = _mm_unpacklo_epi16 (vE70, vF70);
        vFE74 = _mm_unpackhi_epi16 (vE70, vF70);
        vHG30 = _mm_unpacklo_epi16 (vG70, vH70);
        vHG74 = _mm_unpackhi_epi16 (vG70, vH70);
        vDCBA10 = _mm_unpacklo_epi32 (vBA30, vDC30); 
        vDCBA32 = _mm_unpackhi_epi32 (vBA30, vDC30);
        vDCBA54 = _mm_unpacklo_epi32 (vBA74, vDC74);
        vDCBA76 = _mm_unpackhi_epi32 (vBA74, vDC74);
        vHGFE10 = _mm_unpacklo_epi32 (vFE30, vHG30); 
        vHGFE32 = _mm_unpackhi_epi32 (vFE30, vHG30);
        vHGFE54 = _mm_unpacklo_epi32 (vFE74, vHG74);
        vHGFE76 = _mm_unpackhi_epi32 (vFE74, vHG74);
        vHGFEBCDA0 = _mm_unpacklo_epi64 (vDCBA10, vHGFE10);
        vHGFEBCDA1 = _mm_unpackhi_epi64 (vDCBA10, vHGFE10);
        vHGFEBCDA2 = _mm_unpacklo_epi64 (vDCBA32, vHGFE32);
        vHGFEBCDA3 = _mm_unpackhi_epi64 (vDCBA32, vHGFE32);
        vHGFEBCDA4 = _mm_unpacklo_epi64 (vDCBA54, vHGFE54);
        vHGFEBCDA5 = _mm_unpackhi_epi64 (vDCBA54, vHGFE54);
        vHGFEBCDA6 = _mm_unpacklo_epi64 (vDCBA76, vHGFE76);
        vHGFEBCDA7 = _mm_unpackhi_epi64 (vDCBA76, vHGFE76);
        
        BIT_MATRIX_TRANSPOSE_16BYTEOUT_8Windows(vHGFEBCDA0);
        BIT_MATRIX_TRANSPOSE_16BYTEOUT_8Windows(vHGFEBCDA1);
        BIT_MATRIX_TRANSPOSE_16BYTEOUT_8Windows(vHGFEBCDA2);
        BIT_MATRIX_TRANSPOSE_16BYTEOUT_8Windows(vHGFEBCDA3);
        BIT_MATRIX_TRANSPOSE_16BYTEOUT_8Windows(vHGFEBCDA4);
        BIT_MATRIX_TRANSPOSE_16BYTEOUT_8Windows(vHGFEBCDA5);
        BIT_MATRIX_TRANSPOSE_16BYTEOUT_8Windows(vHGFEBCDA6);
        BIT_MATRIX_TRANSPOSE_8BYTEOUT_8Windows(vHGFEBCDA7);
    }

    /* bit transpose back */
    int32_t Nseg2 = (K%128!=0) ? (K/128 + 1) : (K/128);
    for (i=0; i<Nseg2; i++)
    {
        vtmp0 = _mm_lddqu_si128(ptmp++);
        vtmp1 = _mm_slli_epi16 (vtmp0, 7);
        vtmp2 = _mm_slli_epi16 (vtmp0, 6);
        vtmp3 = _mm_slli_epi16 (vtmp0, 5);
        vtmp4 = _mm_slli_epi16 (vtmp0, 4);
        vtmp5 = _mm_slli_epi16 (vtmp0, 3);
        vtmp6 = _mm_slli_epi16 (vtmp0, 2);
        vtmp7 = _mm_slli_epi16 (vtmp0, 1);
        vtmp8 = _mm_slli_epi16 (vtmp0, 0);
        tmp_buf_2[0*48+i] = _mm_movemask_epi8(vtmp1);
        tmp_buf_2[1*48+i] = _mm_movemask_epi8(vtmp2);
        tmp_buf_2[2*48+i] = _mm_movemask_epi8(vtmp3);
        tmp_buf_2[3*48+i] = _mm_movemask_epi8(vtmp4);
        tmp_buf_2[4*48+i] = _mm_movemask_epi8(vtmp5);
        tmp_buf_2[5*48+i] = _mm_movemask_epi8(vtmp6);
        tmp_buf_2[6*48+i] = _mm_movemask_epi8(vtmp7);
        tmp_buf_2[7*48+i] = _mm_movemask_epi8(vtmp8);
    }

    /* concatenation */
    int32_t Lwin = K/8;
    int32_t Nseg3 = Lwin/64;
    int32_t nRestBit = (Lwin%64!=0) ? (Lwin%64) : 0;
    int32_t running_rest_bit = 0;
    vout = _mm_setzero_si128 ();
    for (i=0; i<8; i++)
    {
        ptmp2 = &(tmp_buf_2[i*48]);
        for (j=0; j<Nseg3; j++)
        {
            vtmp0 = _mm_loadl_epi64 ((__m128i const *)ptmp2); 
            ptmp2+=4;
            vtmp1 = _mm_slli_epi64 (vtmp0, running_rest_bit);
            vtmp2 = _mm_srli_epi64 (vtmp0, 64-running_rest_bit);
            vout = _mm_xor_si128 (vtmp1, vout);
            _MM_BR_EPI128(vout, vout1);
            _mm_storel_epi64 ((__m128i *)pOutput, vout1);
            pOutput+=8;
            vout = vtmp2;
        }
        if (nRestBit!=0)
        {
            int32_t N_rest_tmp = nRestBit + running_rest_bit;
            if (N_rest_tmp>=64)
            {
                vtmp0 = _mm_loadl_epi64 ((__m128i const *)ptmp2); 
                vtmp2 = _mm_srli_epi64 (vAllOne, 64-nRestBit);
                vtmp0 = _mm_and_si128 (vtmp0, vtmp2);
                vtmp1 = _mm_slli_epi64 (vtmp0, running_rest_bit);
                vtmp2 = _mm_srli_epi64 (vtmp0, 64-running_rest_bit);
                vout = _mm_xor_si128 (vtmp1, vout);
                _MM_BR_EPI128(vout, vout1);
                _mm_storel_epi64 ((__m128i *)pOutput, vout1);
                pOutput+=8;
                vout = vtmp2;
                running_rest_bit = N_rest_tmp - 64;
            }
            else
            {
                vtmp0 = _mm_loadl_epi64 ((__m128i const *)ptmp2); 
                vtmp2 = _mm_srli_epi64 (vAllOne, 64-nRestBit);
                vtmp0 = _mm_and_si128 (vtmp0, vtmp2);
                vtmp1 = _mm_slli_epi64 (vtmp0, running_rest_bit);
                vout = _mm_xor_si128 (vtmp1, vout);
                running_rest_bit = N_rest_tmp;
            }
        }
    }
    _MM_BR_EPI128(vout, vout1);
    _mm_storel_epi64 ((__m128i *)pOutput, vout1);

    return;
}

/* used to generate interleave table */
static void initInterleaveTable(char* pTabelPath, void* p1, int8_t (*p2)[840], int16_t (*p3)[840], void* p4)
{
    FILE * fp;
    int32_t count;
    char filename[MAX_PATH_LEN];
    char interleavetable[] = "/source/phy/lib_turbo/Fast_LteTurboInterleaver.bin";

    if ((strlen(interleavetable) + strlen(pTabelPath)) >= MAX_PATH_LEN)
    {
        printf("config invalid: tablePath = %s\n", pTabelPath);
        return ;
    }
    strncpy(filename, pTabelPath, strlen(pTabelPath) + 1);
    strncat(filename, interleavetable, strlen(interleavetable) + 1);
    fp = fopen(filename, "rb");
    if(fp != NULL)
    {
        count = fread(p1, sizeof(int8_t), 1024, fp);
        for(int l = 0; l < 188; l++)
             count = fread(p2[l], sizeof(int8_t), 840, fp);
        for(int l = 0; l < 188; l++)
             count = fread(p3[l], sizeof(int16_t), 840, fp);
          
        count = fread(p4, sizeof(_Turbo_Interleaver_Input_Assistant), 188, fp);
        fclose(fp);
    }
    else
    {
      printf("Cannot open fast interleaver table: %s\n", filename);
      return ;
    }
}

void
bblib_lte_turbo_interleaver_initTable(char* pTabelPath)
{
    /* FastInterleave Table */
    void *p1, *p4;
    p1 = (void*)&g_vIntra_row_perm_shuffle_vector[0];
    p4 = (void*)&g_Turbo_Intx_Input_Assistant_sdk[0];

    initInterleaveTable(pTabelPath, p1, g_intra_row_perm_mode_sdk,g_inter_row_out_addr_sdk,p4);
}

int32_t
bblib_lte_turbo_interleaver_8windows_sse(uint8_t caseId, uint8_t *pInData, uint8_t* pOutData)
{
    _Turbo_Interleaver_Para Turbo_Interleaver_Para;

    Turbo_Interleaver_Para.pInput = pInData;
    Turbo_Interleaver_Para.pOutput = pOutData;

    Turbo_Interleaver_Para.pvIntra_row_perm_shuffle_vector = &g_vIntra_row_perm_shuffle_vector[0];
    if(caseId > 0)
    {
        Turbo_Interleaver_Para.p_intra_row_perm_mode = &g_intra_row_perm_mode_sdk[caseId-1][0];
        Turbo_Interleaver_Para.p_inter_row_out_addr = &g_inter_row_out_addr_sdk[caseId-1][0];
        Turbo_Interleaver_Para.p_Input_Assistant = &g_Turbo_Intx_Input_Assistant_sdk[caseId-1];

        turbo_interleaver_8windows_sse( &Turbo_Interleaver_Para );
        return 1;
    }
    else
    {
        return 0; /* case id is invalid.*/
    }
    
}
#else
int32_t
bblib_lte_turbo_interleaver_8windows_sse(uint8_t caseId, uint8_t *pInData, uint8_t* pOutData)
{
    printf("bblib_turbo requires at least SSE4.2 ISA support to run\n");
    return(-1);
}
#endif