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
 * @file phy_rate_dematching_5gnr_avx512.cpp
 * @brief  Implementation for rate dematching functions
 */

#include "phy_rate_dematching_5gnr_internal.h"

#include <string.h>
#include <ipp.h>
#include <ipps.h>
#include <immintrin.h> // AVX

__m512i LLR_VMAX_512 = _mm512_set1_epi8(LLR_VAL);
__m512i LLR_VMIN_512 = _mm512_set1_epi8(-LLR_VAL);

void wrapAdd2HarqBuffer_avx512(int8_t *pIn, int8_t *pOut, int32_t len)
{
    int32_t i;
    int32_t byteStep_64 = len&0xffffffc0;
    int32_t temp = len&0x3f;
    __mmask64 mask0;
    mask0 = ((__mmask64)1<<temp) - 1;
    __m512i ymm512_0, ymm512_1;

#pragma unroll
    for (i=0; i<byteStep_64; i+=64) {
        ymm512_0 = _mm512_loadu_si512 ((__m512i const *) pIn);
        ymm512_1 = _mm512_loadu_si512 ((__m512i const *) pOut);
        ymm512_0 = _mm512_adds_epi8 (ymm512_0, ymm512_1);
        ymm512_1 = _mm512_max_epi8(LLR_VMIN_512, ymm512_0);
        ymm512_0 = _mm512_min_epi8(LLR_VMAX_512, ymm512_1);
        pIn = pIn + 64;
        _mm512_storeu_si512 ((__m512i *) pOut, ymm512_0);
        pOut = pOut + 64;
    }

    ymm512_0 = _mm512_maskz_loadu_epi8 (mask0, (__m512i const *) pIn);
    ymm512_1 = _mm512_maskz_loadu_epi8 (mask0, (__m512i const *) pOut);
    ymm512_0 = _mm512_adds_epi8 (ymm512_0, ymm512_1);
    ymm512_1 = _mm512_max_epi8(LLR_VMIN_512, ymm512_0);
    ymm512_0 = _mm512_min_epi8(LLR_VMAX_512, ymm512_1);
    _mm512_mask_storeu_epi8 ((void*) pOut, mask0, ymm512_0);
}

int32_t harq_combine_avx512(struct bblib_rate_dematching_5gnr_request *request,
    struct bblib_rate_dematching_5gnr_response *response, int8_t *pDeInterleave)
{
    __declspec (align(64)) int8_t alignedBuffer[40 * 1024];
    int8_t *pHarqTmp, *pHarq = (int8_t *)request->p_harq;
    int32_t ncbStep_64 = 0, ncbTmp = 0;
    __m512i ymm0_avx512;
    get_k0(request);
    int32_t ncb_, length, offset_e = 0, offset_ncb = request->k0;

    /* first transmition should fill zero to harq buffer */
    if (request->isretx == 0) {
        /* fill zero into harq buffer */
        ncbStep_64 = request->ncb & 0xffffffc0;
        ncbTmp = request->ncb & 0x3f;
        __mmask64 mask0;
        mask0 = (1 << ncbTmp) - 1;
        pHarqTmp = pHarq;
        ymm0_avx512 = _mm512_set1_epi8 (0);
        for (int32_t i = 0; i < ncbStep_64; i += 64) {
            _mm512_storeu_si512 ((__m512i *) pHarqTmp, ymm0_avx512);
            pHarqTmp = pHarqTmp + 64;
        }
        _mm512_mask_storeu_epi8 ((void*) pHarqTmp, mask0, ymm0_avx512);
    }

    ncb_ = request->ncb - request->num_of_null;
    if (offset_ncb > request->start_null_index)
        offset_ncb -= request->num_of_null;

    while (offset_e < request->e) {
        length = MIN(request->e - offset_e, ncb_ - offset_ncb);
        if (offset_ncb > 0)
            ippsSet_8u(0, (uint8_t *)alignedBuffer, offset_ncb);
        if ((offset_e + offset_ncb) > 0) {
            ippsCopy_8u((uint8_t *) pDeInterleave + offset_e,
                    (uint8_t *) alignedBuffer + offset_ncb, length);
            wrapAdd2HarqBuffer_avx512(alignedBuffer, pHarq, length + offset_ncb);
        } else
            wrapAdd2HarqBuffer_avx512(pDeInterleave, pHarq, length + offset_ncb);
        offset_ncb = length + offset_ncb;
        if (offset_ncb == ncb_)
            offset_ncb = 0;
        offset_e += length;
    }

    return 0;
}

#if 0
/**
 * @brief This function implements deinterlaver for LDPC rate dematching
 * @param[in] pHarq output of HARQ combine, and input of deinterleaver
 * @param[in] the pointer of rate dematching struct
 * @param[out] pDeInterleave output of deinterleaver, and as an input of bit deselection
 *
 */
__m512i permute_64QAM_0 = _mm512_set_epi16 (0,0,32,29,26,23,20,17,14,8,5,2,28,25,22,19,16,13,10,7,4,1,27,24,21,18,15,12,9,6,3,0);
__m512i QAM64_60byte_deint(__m512i x0)
{
    __m256i x0_256,x1_256;
    x0 = _mm512_permutexvar_epi16(permute_64QAM_0,x0);//01 ... 01, 23 ... 23, 45 ... 45
    x0_256 = _mm512_cvtepi16_epi8 (x0);//0...0 2...2 4...4
    x0 = _mm512_srli_epi16 (x0, 8);
    x1_256 = _mm512_cvtepi16_epi8 (x0);//1...1 3...3 5...5
    x0 = _mm512_broadcast_i32x8 (x0_256);
    x0 = _mm512_inserti64x4 (x0, x1_256, 1);//0...0 2...2 4...4 x 1...1 3...3 5...5 x
    return x0;
}

__m512i permute_64QAM_1 = _mm512_set_epi16 (0,0,41,40,39,38,37,9,8,7,6,5,52,51,50,49,48,20,19,18,17,16,36,35,34,33,32,4,3,2,1,0);
__m512i permute_64QAM_2 = _mm512_set_epi16 (0,0,62,61,60,59,58,30,29,28,27,26,47,46,45,44,43,15,14,13,12,11,57,56,55,54,53,25,24,23,22,21);


void QAM64_120byte_deint(__m512i x0,__m512i x1,__m512i *py0,__m512i *py1)
{
    x0 = QAM64_60byte_deint(x0);//0...0 2...2 4...4 x 1...1 3...3 5...5 x
    x1 = QAM64_60byte_deint(x1);//0...0 2...2 4...4 x 1...1 3...3 5...5 x
    *py0 = _mm512_permutex2var_epi16 (x0, permute_64QAM_1, x1);//0...00...0 1...11...1 2...22...2 xx
    *py1 = _mm512_permutex2var_epi16 (x0, permute_64QAM_2, x1);//3...33...3 4...44...4 5...55...5 xx
}

void printm512(__m512i ymmTemp0) {
    uint32_t *pBytes = (uint32_t *) &ymmTemp0;
    printf("%08x %08x %08x %08x %08x %08x %08x %08x\n",
            pBytes[0], pBytes[1], pBytes[2], pBytes[3],
            pBytes[4], pBytes[5], pBytes[6], pBytes[7]);
    printf("%08x %08x %08x %08x %08x %08x %08x %08x\n",
            pBytes[8], pBytes[9], pBytes[10], pBytes[11],
            pBytes[12], pBytes[13], pBytes[14], pBytes[15]);
}
#endif

void deInterleave_avx512(uint8_t *pCbIn, int8_t *pDeInterleave, bblib_rate_dematching_5gnr_request *pRM)
{
    int32_t i=0, j=0;
    int32_t  e = pRM->e;
    int32_t modQ = pRM->modulation_order;

    int32_t eDiv2 = (e>>1), eDiv4=0, eDiv6=0, eDiv8=0;
    int32_t len0=0, len1=0;

    const __m128i idxHalf128L = _mm_set_epi64x(0x2, 0x0);
    const __m128i idxHalf128H = _mm_set_epi64x(0x3, 0x1);

    const __m128i idxInter16_L128 = _mm_set_epi16(0xB, 0x3, 0xA, 0x2, 0x9, 0x1, 0x8, 0x0);
    const __m128i idxInter16_H128 = _mm_set_epi16(0xF,0x7, 0xE,0x6, 0xD,0x5, 0xC, 0x4);

    const __m128i idxInter32_L128 = _mm_set_epi32(0x5, 0x1, 0x4, 0x0);
    const __m128i idxInter32_H128 = _mm_set_epi32(0x7, 0x3, 0x6, 0x2);

    __m512i *pCbInVec512 = (__m512i *)pCbIn;
    /* deinterleaving according to 38.212 bit interleaving in LDPC rate matching */
    switch (modQ)
    {
        case BBLIB_QPSK:
        {
            const __m512i idxEven64_512 = _mm512_set_epi64(0xE, 0xC, 0xA, 0x8, 0x6, 0x4, 0x2, 0x0);
            const __m512i idxOdd64_512 = _mm512_set_epi64(0xF, 0xD, 0xB, 0x9, 0x7, 0x5, 0x3, 0x1);
            const __m512i idxEvenOdd8_512 = _mm512_set_epi32(
                                                            0x0F0D0B09, 0x07050301, 0x0E0C0A08, 0x06040200,
                                                            0x0F0D0B09, 0x07050301, 0x0E0C0A08, 0x06040200,
                                                            0x0F0D0B09, 0x07050301, 0x0E0C0A08, 0x06040200,
                                                            0x0F0D0B09, 0x07050301, 0x0E0C0A08, 0x06040200);
            const __m128i idxEvenOdd_128 = _mm_set_epi32(0xF0D0B09, 0x7050301, 0xE0C0A08, 0x6040200);

            len0 = eDiv2 & 0xffc0; //64*x bytes
            len1 = eDiv2 & 0x3f;  //left 0~63 bytes

            for (i=0, j=0; i<len0; i+=64, j+=2) {
                //a0 a2, ...a12,a14 #a1,a3..,a13,a15 #a16,..a30 #a17,..a31, #a32,...,a46 #a33, ..a47 #a48,a50,...a60 a62 #a49 a51...a61 a63
                __m512i ymmTemp0 = _mm512_shuffle_epi8 (pCbInVec512[j], idxEvenOdd8_512);
                //b0 b2 ... ##b1 b3, ....#b48,...b60 b62 #b49,...b61 b63
                __m512i ymmTemp1 = _mm512_shuffle_epi8 (pCbInVec512[j+1], idxEvenOdd8_512);
                __m512i ymm2 = _mm512_permutex2var_epi64(ymmTemp0, idxEven64_512, ymmTemp1);
                _mm512_storeu_si512 ((__m512i *) pDeInterleave, ymm2);

                __m512i ymm3 =  _mm512_permutex2var_epi64(ymmTemp0, idxOdd64_512, ymmTemp1);
                _mm512_storeu_si512 ((__m512i *) (pDeInterleave+eDiv2), ymm3);
                pDeInterleave += 64;
            }
            pCbIn = pCbIn + 2*len0;

            if (len1 >= 16) {
                __m128i *pCbInVec128 = (__m128i *)pCbIn;
                len0 = (len1&0x30);  //x*16 bytes
                len1 = (len1&0xf); //left 0~15 bytes

                for (i = 0, j = 0; i < len0; i += 16, j += 2) {
                    //a0 a2, ..., a14, a1, a3, .., a15
                    __m128i xmmTemp0 = _mm_shuffle_epi8 (pCbInVec128[j], idxEvenOdd_128);
                    //b0 b2, ..., b14, b1, b3, .., b15
                    __m128i xmmTemp1 = _mm_shuffle_epi8 (pCbInVec128[j+1], idxEvenOdd_128);
                    //a0 a2, ..., a14, b0 b2, ..., b14
                    __m128i xmmTemp2 = _mm_permutex2var_epi64(xmmTemp0, idxHalf128L, xmmTemp1);
                    _mm_storeu_si128 ((__m128i *) pDeInterleave, xmmTemp2);
                    //a1, a3, .., a15,  b1, b3, .., b15
                    __m128i xmmTemp3 = _mm_permutex2var_epi64(xmmTemp0, idxHalf128H, xmmTemp1);
                    _mm_storeu_si128 ((__m128i *) (pDeInterleave+eDiv2), xmmTemp3);
                    pDeInterleave += 16;
                }
                pCbIn = pCbIn + 2 * len0;
            }

            for (i = 0; i < len1; i++) {
                *pDeInterleave = *pCbIn++;
                *(pDeInterleave+eDiv2) = *pCbIn++;
                pDeInterleave++;
            }
            break;
        }
        case BBLIB_QAM16:
        {
            const __m512i idxEvenOdd4_512 = _mm512_set_epi32(
                                                            0x0F0B0703, 0x0E0A0602, 0x0D090501, 0x0C080400,
                                                            0x0F0B0703, 0x0E0A0602, 0x0D090501, 0x0C080400,
                                                            0x0F0B0703, 0x0E0A0602, 0x0D090501, 0x0C080400,
                                                            0x0F0B0703, 0x0E0A0602, 0x0D090501, 0x0C080400);

            const __m512i idx0Inter32_512 = _mm512_set_epi32(
                                                            0x1D, 0x19, 0x15, 0x11, 0x0D, 0x09, 0x05, 0x01,
                                                            0x1C, 0x18, 0x14, 0x10, 0x0C, 0x08, 0x04, 0x00);
            const __m512i idx1Inter32_512 =  _mm512_set_epi32(
                                                            0x1F, 0x1B, 0x17, 0x13, 0x0F, 0x0B, 0x07, 0x03,
                                                            0x1E, 0x1A, 0x16, 0x12, 0x0E, 0x0A, 0x06, 0x02);

            const __m128i idxInter4_128_shf = _mm_set_epi32(0x0F0B0703, 0x0E0A0602, 0x0D090501, 0x0C080400);

            eDiv4 = eDiv2 / 2;
            len0 = eDiv4 & 0x3fc0; //64*x bytes
            len1 = eDiv4 & 0x3f;  //left 0~63 bytes


            for (i = 0, j = 0; i < len0; i += 64, j += 4) {
                //a0 a4, a8,a12 #a1,a5,a9,a13 #a2,..a14 #a3,..a15, #a16,...,a28 #a17, ..a29 #a18,... a30 #a19,...,a31
                //a32, a36,..,a44  #a33,a37,...,a45 #a34,...,a46 #a35,..,a47, #a48,....
                __m512i ymmTemp0 = _mm512_shuffle_epi8 (pCbInVec512[j], idxEvenOdd4_512);
                //b0 b4, b8,b12 #b1,b5,b9,b13 #b2,..b14 #b3,..b15, #b16,...,b28 #b17, ..b29 #b18,... b30 #b19,...,b31
                //b32, b36,..,a44  #b33,b37,...,b45 #b34,...,b46 #b35,..,b47, #b48,....
                __m512i ymmTemp1 = _mm512_shuffle_epi8 (pCbInVec512[j+1], idxEvenOdd4_512);
                //c0...
                __m512i ymmTemp2 = _mm512_shuffle_epi8 (pCbInVec512[j+2], idxEvenOdd4_512);
                //d0...
                __m512i ymmTemp3 = _mm512_shuffle_epi8 (pCbInVec512[j+3], idxEvenOdd4_512);
                //a0 a4, a8,a12 #a16,...,a28 #a32, a36,..,a44  #a48,....,a60 ##b0 b4, b8,b12 #b16,...,b28 #b32, b36,..,b44  #b48,....,b60
                //a1 a5, a9,a13 #a17,...,a29 #a33, a37,..,a45  #a49,....,a61 ##b1 b5, b9,b13 #b17,...,b29 #b33, b37,..,b45  #b49,....,b61
                __m512i ymm0 = _mm512_permutex2var_epi32(ymmTemp0, idx0Inter32_512, ymmTemp1);
                //a2 a6, a10,a14 #a18,...
                //a3 a7, a11,a15 #a19,...
                __m512i ymm1 = _mm512_permutex2var_epi32(ymmTemp0, idx1Inter32_512, ymmTemp1);
                //c0 c4, c8,c12... #d0...
                //c1 c5, c9,c13... #d1...
                __m512i ymm2 = _mm512_permutex2var_epi32(ymmTemp2, idx0Inter32_512, ymmTemp3);
                //c2 c6, c10,c14... #d2...
                //c3 c7, c11,c15... #d3...
                __m512i ymm3 = _mm512_permutex2var_epi32(ymmTemp2, idx1Inter32_512, ymmTemp3);
                // a0 a4 ...a60, b0, b4, ..., b60, c0 c4 ...c60, d0, d4, ..., d60
                ymmTemp0 = _mm512_shuffle_i64x2(ymm0, ymm2, 0x44);
                _mm512_storeu_si512 ((__m512i *) pDeInterleave, ymmTemp0);
                //  a1 a5.. a61, b1, b5,..., b61, c1 c5.. c61, d1, d5,..., d61
                ymmTemp1 = _mm512_shuffle_i64x2(ymm0, ymm2, 0xEE);
                _mm512_storeu_si512 ((__m512i *) (pDeInterleave+eDiv4), ymmTemp1);
                // a2 a6 ...a62, b2, b6, ..., b62, c2 c6 ...c62, d2, d6, ..., d62
                ymmTemp2 = _mm512_shuffle_i64x2(ymm1, ymm3, 0x44);
                _mm512_storeu_si512 ((__m512i *)  (pDeInterleave+eDiv2), ymmTemp2);
                // a3 a7.. a63, b3, b7,..., b63, c3 c7.. c63, d3, d7,..., d63
                ymmTemp3 = _mm512_shuffle_i64x2(ymm1, ymm3, 0xEE);
                _mm512_storeu_si512 ((__m512i *) (pDeInterleave+eDiv2+eDiv4), ymmTemp3);
                pDeInterleave += 64;
            }
            pCbIn = pCbIn + 4*len0;

            if (len1 >= 16) {
                __m128i *pCbInVec128 = (__m128i *)pCbIn;
                len0 = (len1&0x30);  //x*16 bytes
                len1 = (len1&0xf); //left 0~15 bytes

                for (i=0, j=0; i<len0; i+=16, j+=4) {
                    //a0 a4, a8,a12 #a1,a5,a9,a13 #a2,..a14 #a3,..a15, #a16,...,a28 #a17, ..a29 #a18,... a30 #a19,...,a31
                    //a0,a4,a8,a12  #a1,a5.., a13  #a2,a6,a10,a14  #a3,a7, .., a15
                    __m128i xmmTemp0 = _mm_shuffle_epi8(pCbInVec128[j], idxInter4_128_shf);
                    //b0,b4,b8,b12  #b1,b5.., b13  #b2,b6,b10,b14  #b3,b7, .., b15
                    __m128i xmmTemp1 = _mm_shuffle_epi8(pCbInVec128[j+1], idxInter4_128_shf);
                    //c0,c4,c8,c12  #c1,c5.., c13  #c2,c6,c10,c14  #c3,c7, .., c15
                    __m128i xmmTemp2 = _mm_shuffle_epi8(pCbInVec128[j+2], idxInter4_128_shf);
                    //d0,d4,d8,d12  #d1,d5.., d13  #d2,d6,d10,d14  #d3,d7, .., d15
                    __m128i xmmTemp3 = _mm_shuffle_epi8(pCbInVec128[j+3], idxInter4_128_shf);


                    //a0,a4,a8,a12  b0,b4,b8,b12  #a1,a5.., a13  b1,b5.., b13
                    __m128i xmm0 = _mm_permutex2var_epi32(xmmTemp0, idxInter32_L128, xmmTemp1);
                    //a2,a6,a10,a14  b2,b6,b10,b14 #a3,a7, .., a15  b3,b7, .., b15
                    __m128i xmm1 = _mm_permutex2var_epi32(xmmTemp0, idxInter32_H128, xmmTemp1);
                    //c0,c4,c8,c12  d0,d4,d8,d12  #c1,c5.., c13  d1,d5.., d13
                    __m128i xmm2 = _mm_permutex2var_epi32(xmmTemp2, idxInter32_L128, xmmTemp3);
                    //c2,c6,c10,c14  d2,d6,d10,d14  #c3,c7, .., c15  d3,d7, .., d15
                    __m128i xmm3 = _mm_permutex2var_epi32(xmmTemp2, idxInter32_H128, xmmTemp3);

                    // a0 a4, a8, a12, b0 b4, b8, b12, c0 c4, c8, c12, d0 d4, d8, d12
                    xmmTemp0 = _mm_permutex2var_epi64(xmm0, idxHalf128L, xmm2);
                    _mm_storeu_si128 ((__m128i *) pDeInterleave, xmmTemp0);
                    // a1, a5.., a13,  b1, b5.., b13, c1, c5.., c13, d1, d5.., d13
                    xmmTemp1 = _mm_permutex2var_epi64(xmm0, idxHalf128H, xmm2);
                    _mm_storeu_si128 ((__m128i *) (pDeInterleave+eDiv4), xmmTemp1);
                    // a2, a6, a10,a14, b2, b6, ..,b14, c2, c6, c10,c14, d2, d6, ..,d14
                    xmmTemp2 = _mm_permutex2var_epi64(xmm1, idxHalf128L, xmm3);
                    _mm_storeu_si128 ((__m128i *) (pDeInterleave+eDiv2), xmmTemp2);
                    // a3, a7, .., a15,  b3, b7, .., b15,  c3, c7, .., c15, d3, d7, .., d15
                    xmmTemp3 = _mm_permutex2var_epi64(xmm1, idxHalf128H, xmm3);
                    _mm_storeu_si128 ((__m128i *) (pDeInterleave+eDiv2+eDiv4), xmmTemp3);

                    pDeInterleave += 16;
                }
                pCbIn = pCbIn + 4*len0;
            }

            for (i = 0; i < len1; i++) {
                *pDeInterleave = *pCbIn++;
                *(pDeInterleave+eDiv4) = *pCbIn++;
                *(pDeInterleave+eDiv2) = *pCbIn++;
                *(pDeInterleave+eDiv2+eDiv4) = *pCbIn++;
                pDeInterleave++;
            }

            break;
        }
        case BBLIB_QAM64:
        {
#if 1
            const __m128i idxInter6_128_shf = _mm_set_epi32(0x0, 0x0B050A04, 0x09030802, 0x07010600);
            eDiv6 = eDiv2 / 3;
            len0 = eDiv6 & 0x1ff0; //16*x bytes
            len1 = eDiv6 & 0xf;

            for (i=0; i<len0; i+=16) {
                __m128i xmm0 = _mm_loadu_si128 ((__m128i *) pCbIn);// a0, a1, ..., a11
                pCbIn = pCbIn + 12;
                __m128i xmm1 = _mm_loadu_si128 ((__m128i *) pCbIn);// b0, b1, ..., b11
                pCbIn = pCbIn + 12;
                __m128i xmm2 = _mm_loadu_si128 ((__m128i *) pCbIn);// c0, c1, ..., c11
                pCbIn = pCbIn + 12;
                __m128i xmm3 = _mm_loadu_si128 ((__m128i *) pCbIn);// d0, d1, ..., d11
                pCbIn = pCbIn + 12;
                __m128i xmm4 = _mm_loadu_si128 ((__m128i *) pCbIn);// e0, e1, ..., e11
                pCbIn = pCbIn + 12;
                __m128i xmm5 = _mm_loadu_si128 ((__m128i *) pCbIn);// f0, f1, ..., f11
                pCbIn = pCbIn + 12;
                __m128i xmm6 = _mm_loadu_si128 ((__m128i *) pCbIn);// g0, g1, ..., g11
                pCbIn = pCbIn + 12;
                __m128i xmm7 = _mm_loadu_si128 ((__m128i *) pCbIn);// h0, h1, ..., h11
                pCbIn = pCbIn + 12;

                //a0,a6 #a1,a7 #a2,a8#a3,a9 #a4,a10#a5,a11 unused(4bytes)
                __m128i xmmTemp0 = _mm_shuffle_epi8(xmm0, idxInter6_128_shf);
                //b0,b6 #b1,b7  #b2,b8 #b3,b9 #b4,b10#b5,b11 unused(4bytes)
                __m128i xmmTemp1 = _mm_shuffle_epi8(xmm1, idxInter6_128_shf);
                //c0,...
                __m128i xmmTemp2 = _mm_shuffle_epi8(xmm2, idxInter6_128_shf);
                //d0,...
                __m128i xmmTemp3 = _mm_shuffle_epi8(xmm3, idxInter6_128_shf);
                //e0,...
                __m128i xmmTemp4 = _mm_shuffle_epi8(xmm4, idxInter6_128_shf);
                //f0,...
                __m128i xmmTemp5 = _mm_shuffle_epi8(xmm5, idxInter6_128_shf);
                //g0,...
                __m128i xmmTemp6 = _mm_shuffle_epi8(xmm6, idxInter6_128_shf);
                //h0,...
                __m128i xmmTemp7 = _mm_shuffle_epi8(xmm7, idxInter6_128_shf);

                //a0,a6 #b0,b6 #a1,a7 #b1,b7 #a2,a8 #b2,b8 #a3,a9 #b3,b9
                xmm0 = _mm_permutex2var_epi16(xmmTemp0, idxInter16_L128, xmmTemp1);
                //a4,a10 #b4,b10 #a5,a11 #b5,b11 #unused(8bytes)
                xmm1 = _mm_permutex2var_epi16(xmmTemp0, idxInter16_H128, xmmTemp1);

                //c0,c6 #d0,d6 #c1,c7 #d1,d7 #c2,c8 #d2,d8 #c3,c9 #d3,d9
                xmm2 = _mm_permutex2var_epi16(xmmTemp2, idxInter16_L128, xmmTemp3);
                //c4,c10 #d4,d10 #c5,c11 #d5,d11 #unused(8bytes)
                xmm3 = _mm_permutex2var_epi16(xmmTemp2, idxInter16_H128, xmmTemp3);

                //e0,e6 #f0,f6 #e1,e7 #f1,f7 #e2,e8 #f2,f8 #e3,e9 #f3,f9
                xmm4 = _mm_permutex2var_epi16(xmmTemp4, idxInter16_L128, xmmTemp5);
                //e4,e10 #f4,f10 #e5,e11 #f5,f11 #unused(8bytes)
                xmm5 = _mm_permutex2var_epi16(xmmTemp4, idxInter16_H128, xmmTemp5);

                //g0,g6 #h0,h6 #g1,g7 #h1,h7 #g2,g8 #h2,h8 #g3,g9 #h3,h9
                xmm6 = _mm_permutex2var_epi16(xmmTemp6, idxInter16_L128, xmmTemp7);
                //g4,g10 #h4,h10 #g5,g11 #h5,h11 #unused(8bytes)
                xmm7 = _mm_permutex2var_epi16(xmmTemp6, idxInter16_H128, xmmTemp7);

                xmmTemp0 = _mm_permutex2var_epi32(xmm0, idxInter32_L128, xmm2);//a0,a6 b0,b6 #c0,c6 d0,d6 #a1,a7 b1,b7#c1,c7 d1,d7
                xmmTemp1 = _mm_permutex2var_epi32(xmm0, idxInter32_H128, xmm2);//a2,a8  b2,b8 #c2,c8  d2,d8 #a3,a9 b3,b9 #c3,c9 d3,d9
                xmmTemp2 = _mm_permutex2var_epi32(xmm4, idxInter32_L128, xmm6);//e0,e6 f0,f6 #g0,g6 h0,h6 #e1,e7 f1,f7 #g1,g7 h1,h7
                xmmTemp3 = _mm_permutex2var_epi32(xmm4, idxInter32_H128, xmm6);//e2,e8  f2,f8 #g2,g8  h2,h8 #e3,e9 f3,f9 #g3,g9 h3,h9
                xmmTemp4 = _mm_permutex2var_epi32(xmm1, idxInter32_L128, xmm3);//a4,a10 b4,b10 #c4,c10 d4,d10 #a5,a11 b5,b11 #c5,c11 d5,d11
                xmmTemp6 = _mm_permutex2var_epi32(xmm5, idxInter32_L128, xmm7);//e4,e10 f4,f10 #g4,g10 h4,h10 #e5,e11 f5,f11#g5,g11 h5,h11

                //a0,a6 b0,b6 #c0,c6 d0,d6 #e0,e6 f0,f6 #g0,g6 h0,h6
                xmm0 = _mm_permutex2var_epi64(xmmTemp0, idxHalf128L, xmmTemp2);
                _mm_storeu_si128 ((__m128i *) pDeInterleave, xmm0);
                //a1,a7 b1,b7#c1,c7 d1,d7 #e1,e7 f1,f7 #g1,g7 h1,h7
                xmm1 = _mm_permutex2var_epi64(xmmTemp0, idxHalf128H, xmmTemp2);
                _mm_storeu_si128 ((__m128i *) (pDeInterleave+eDiv6), xmm1);

                //a2,a8  b2,b8 #c2,c8  d2,d8 #e2,e8  f2,f8 #g2,g8  h2,h8
                xmm2 = _mm_permutex2var_epi64(xmmTemp1, idxHalf128L, xmmTemp3);
                _mm_storeu_si128 ((__m128i *)  (pDeInterleave+2*eDiv6), xmm2);
                //a3,a9 b3,b9 #c3,c9 d3,d9 #e3,e9 f3,f9 #g3,g9 h3,h9
                xmm3 = _mm_permutex2var_epi64(xmmTemp1, idxHalf128H, xmmTemp3);
                _mm_storeu_si128 ((__m128i *)  (pDeInterleave+eDiv2), xmm3);

                //a4,a10 b4,b10 #c4,c10 d4,d10 #e4,e10 f4,f10 #g4,g10 h4,h10
                xmm4 = _mm_permutex2var_epi64(xmmTemp4, idxHalf128L, xmmTemp6);
                _mm_storeu_si128 ((__m128i *)  (pDeInterleave+eDiv2+eDiv6), xmm4);
                //a5,a11 b5,b11 #c5,c11 d5,d11 #e5,e11 f5,f11#g5,g11 h5,h11
                xmm5 = _mm_permutex2var_epi64(xmmTemp4, idxHalf128H, xmmTemp6);
                _mm_storeu_si128 ((__m128i *) (pDeInterleave+eDiv2+2*eDiv6), xmm5);

                pDeInterleave += 16;

            }

            //left 0~15bytes
            for (i = 0; i < len1; i++) {
                *pDeInterleave = *pCbIn++;
                *(pDeInterleave+eDiv6) = *pCbIn++;
                *(pDeInterleave+2*eDiv6) = *pCbIn++;
                *(pDeInterleave+eDiv2) = *pCbIn++;
                *(pDeInterleave+eDiv2+eDiv6) = *pCbIn++;
                *(pDeInterleave+eDiv2+2*eDiv6) = *pCbIn++;
                pDeInterleave++;
            }
#else

            eDiv6 = eDiv2/3;
            int32_t eDiv3 = eDiv6*2;
            int32_t eMul2Div3 = eDiv6*4;
            int32_t eMul5Div6 = eDiv6*5;
            int32_t e0 = (e/120)*120;
            int32_t eMod120 = e-e0;
            int32_t eMod120Div6 = eMod120/6;
            __mmask64 mask0 = ((__mmask64)1<<eMod120Div6)-1;
            __m512i y0,y1;
            __m512i vindex0 = _mm512_setr_epi32 (0, 4, 8, 12, 16,
                    eDiv6, eDiv6+4,eDiv6+8, eDiv6+12, eDiv6+16,
                    eDiv3, eDiv3+4,eDiv3+8, eDiv3+12, eDiv3+16,0);
            __m512i vindex1 = _mm512_setr_epi32 (eDiv2, eDiv2+4, eDiv2+8, eDiv2+12, eDiv2+16,
                                eMul2Div3, eMul2Div3+4, eMul2Div3+8, eMul2Div3+12, eMul2Div3+16,
                                eMul5Div6, eMul5Div6+4,eMul5Div6+8, eMul5Div6+12, eMul5Div6+16,0);
            int8_t *pDeInterleaveOffset = pDeInterleave;
#if 0
            for (i=0; i<e0; i+=120)
            {
                __m512i xmm0 = _mm512_loadu_si512 ((__m512i *) pCbIn);//
                pCbIn = pCbIn + 60;
                __m512i xmm1 = _mm512_loadu_si512 ((__m512i *) pCbIn);//
                pCbIn = pCbIn + 60;
                //0...00...0 1...11...1 2...22...2 xx
                //3...33...3 4...44...4 5...55...5 xx
                QAM64_120byte_deint(xmm0,xmm1,&y0,&y1);

                _mm512_mask_i32scatter_epi32 (pDeInterleaveOffset, 0x7fff, vindex0, y0, 1);
                _mm512_mask_i32scatter_epi32 (pDeInterleaveOffset, 0x7fff, vindex1, y1, 1);
                pDeInterleaveOffset = pDeInterleaveOffset + 20;
            }
            if (eMod120!=0)
            {
                __m512i xmm0 = _mm512_loadu_si512 ((__m512i *) pCbIn);//
                pCbIn = pCbIn + 60;
                __m512i xmm1 = _mm512_loadu_si512 ((__m512i *) pCbIn);//
                pCbIn = pCbIn + 60;
                //0...00...0 1...11...1 2...22...2 xx
                //3...33...3 4...44...4 5...55...5 xx
                QAM64_120byte_deint(xmm0,xmm1,&y0,&y1);
                _mm512_mask_storeu_epi8(pDeInterleaveOffset,mask0,y0);
                _mm512_mask_storeu_epi8(pDeInterleaveOffset+eDiv6-eMod120Div6,mask0<<eMod120Div6,y0);
                _mm512_mask_storeu_epi8(pDeInterleaveOffset+eDiv3-eMod120Div6*2,mask0<<(eMod120Div6*2),y0);

                _mm512_mask_storeu_epi8(pDeInterleaveOffset+eDiv2,mask0,y1);
                _mm512_mask_storeu_epi8(pDeInterleaveOffset+eMul2Div3-eMod120Div6,mask0<<eMod120Div6,y1);
                _mm512_mask_storeu_epi8(pDeInterleaveOffset+eMul5Div6-eMod120Div6*2,mask0<<(eMod120Div6*2),y1);
            }
#else
            for (int32_t j=0; j<6;j++)
            for (int32_t i=0; i<eDiv6;i++)
            {
                *pDeInterleaveOffset++ = *(pCbIn+6*i+j);
            }
#endif
#endif

            break;
        }

        case BBLIB_QAM256:
        {
            const __m512i idxInter128_L512 = _mm512_set_epi64(0xD, 0xC, 0x5, 0x4, 0x9, 0x8, 0x1, 0x0);
            const __m512i idxInter128_H512 = _mm512_set_epi64(0xF, 0xE, 0x7, 0x6, 0xB, 0xA, 0x3, 0x2);
#if 0
            const __m128i idxInter8_128_shf = _mm_set_epi8(0xF, 0x7, 0xE, 0x6, 0xD, 0x5,0xC, 0x4, 0xB, 0x3, 0xA, 0x2, 0x9, 0x1, 0x8, 0x0);
#endif
            const __m512i idxInter8_512 = _mm512_set_epi32(
                                                            0x0F070E06, 0x0D050C04, 0x0B030A02, 0x09010800,
                                                            0x0F070E06, 0x0D050C04, 0x0B030A02, 0x09010800,
                                                            0x0F070E06, 0x0D050C04, 0x0B030A02, 0x09010800,
                                                            0x0F070E06, 0x0D050C04, 0x0B030A02, 0x09010800);

            const __m512i idxInter16_L512 = _mm512_set_epi16(
                                                            0x3B, 0x33, 0x2B, 0x23, 0x1B, 0x13, 0x0B, 0x03,
                                                            0x3A, 0x32, 0x2A, 0x22, 0x1A, 0x12, 0x0A, 0x02,
                                                            0x39, 0x31, 0x29, 0x21, 0x19, 0x11, 0x09, 0x01,
                                                            0x38, 0x30, 0x28, 0x20, 0x18, 0x10, 0x08, 0x00);
            const __m512i idxInter16_H512 =  _mm512_set_epi16(
                                                            0x3F, 0x37, 0x2F, 0x27, 0x1F, 0x17, 0x0F, 0x07,
                                                            0x3E, 0x36, 0x2E, 0x26, 0x1E, 0x16, 0x0E, 0x06,
                                                            0x3D, 0x35, 0x2D, 0x25, 0x1D, 0x15, 0x0D, 0x05,
                                                            0x3C, 0x34, 0x2C, 0x24, 0x1C, 0x14, 0x0C, 0x04);

            eDiv4 = eDiv2 / 2;
            eDiv8 = eDiv2 / 4;
            len0 = eDiv8 & 0x1fc0; //64*x bytes
            len1 = eDiv8 & 0x3f;  //left 0~63 bytes

            for (i=0, j=0; i<len0; i+=64, j+=8) {
                //a0,a8,a1,a9,a2,a10,a3,a11,a4,a12,a5,a13,a6,a14,a7,a15#a16,a24....,a31#a32,a40,..,a47 #a48,a56,...,a63
                __m512i ymmTemp0 = _mm512_shuffle_epi8 (pCbInVec512[j], idxInter8_512);
                //b0,b8,b1,b9,b2,b10,b3,b11,b4,b12,b5,b13,b6,b14,b7,b15#b16,b24....,b31#b32,b40,..,b47 #b48,b56,...,b63
                __m512i ymmTemp1 = _mm512_shuffle_epi8 (pCbInVec512[j+1], idxInter8_512);
                //c0,c8,c1,c9,c2,c10,...
                __m512i ymmTemp2 = _mm512_shuffle_epi8 (pCbInVec512[j+2], idxInter8_512);
                //d0,d8,d1,d9,d2,d10,...
                __m512i ymmTemp3 = _mm512_shuffle_epi8 (pCbInVec512[j+3], idxInter8_512);
                //e0,e8,e1,e9,e2,e10,...
                __m512i ymmTemp4 = _mm512_shuffle_epi8 (pCbInVec512[j+4], idxInter8_512);
                //f0,f8,f1,f9,f2,f10,...
                __m512i ymmTemp5 = _mm512_shuffle_epi8 (pCbInVec512[j+5], idxInter8_512);
                //g0,g8,g1,g9,g2,g10,...
                __m512i ymmTemp6 = _mm512_shuffle_epi8 (pCbInVec512[j+6], idxInter8_512);
                //h0,h8,h1,h9,h2,h10,...
                __m512i ymmTemp7 = _mm512_shuffle_epi8 (pCbInVec512[j+7], idxInter8_512);

                //a0,a8,a16,a24,a32,a40,a48,a56 #b0,b8,b16,b24,b32,b40,b48,b56  #a1,a9,...b1,b9... #a2,a10,...b2,b10... #a3,a11,...b3,b11...
                __m512i ymm0 = _mm512_permutex2var_epi16(ymmTemp0, idxInter16_L512, ymmTemp1);
                //a4,a12,a20,a28,a36,a44,a52,a60 #b4,b12,b20,b28,b36,b44,b52,b60  #a5,a13,...b5,b13... #a6,a14,...b6,b14... #a7,a15,...b7,b15...
                __m512i ymm1 = _mm512_permutex2var_epi16(ymmTemp0, idxInter16_H512, ymmTemp1);
                //c0,c8,c16,c24,c32,c40,c48,c56 #d0,d8,d16,d24,d32,d40,d48,d56  #c1,c9,...d1,d9... #c2,c10,...d2,d10... #c3,c11,...d3,d11...
                __m512i ymm2 = _mm512_permutex2var_epi16(ymmTemp2, idxInter16_L512, ymmTemp3);
                //c4,c12,c20,c28,c36,c44,c52,c60 #d4,d12,d20,d28,d36,d44,d52,d60  #c5,c13,...d5,d13... #c6,c14,...d6,d14... #c7,c15,...d7,d15...
                __m512i ymm3 = _mm512_permutex2var_epi16(ymmTemp2, idxInter16_H512, ymmTemp3);
                //e0,e8,... #f0,f8, ...
                __m512i ymm4 = _mm512_permutex2var_epi16(ymmTemp4, idxInter16_L512, ymmTemp5);
                //e4,e12,... #f4,f12,...
                __m512i ymm5 = _mm512_permutex2var_epi16(ymmTemp4, idxInter16_H512, ymmTemp5);
                //g0,g8,... #h0,h8, ...
                __m512i ymm6 = _mm512_permutex2var_epi16(ymmTemp6, idxInter16_L512, ymmTemp7);
                //g4,g12,... #h4,h12,...
                __m512i ymm7 = _mm512_permutex2var_epi16(ymmTemp6, idxInter16_H512, ymmTemp7);

                //a0,a8,...,a56 #b0,b8,...,b56 #c0,c8,...,c56 #d0,d8,...,d56 #a2,a10,...,a58 #b2,b10,...,b58 #c2,c10,...,c58 #d2,d10,...,d58
                ymmTemp0 = _mm512_permutex2var_epi64 (ymm0, idxInter128_L512, ymm2);
                //a1,a9,...,a57 #b1,b9,...,b57 #c1,c9,...,c57 #d1,d9,...,d57 #a3,a11,...,a59 #b3,b11,...,b59 #c3,c11,...,c59 #d3,d11,...,d59
                ymmTemp1 = _mm512_permutex2var_epi64 (ymm0, idxInter128_H512, ymm2);
                //e0,e8,...,e56 #f0,f8,...,f56 #g0,g8,...,g56 #h0,h8,...,h56 #e2,e10,...,e58 #f2,f10,...,f58 #g2,g10,...,g58 #h2,h10,...,h58
                ymmTemp2 = _mm512_permutex2var_epi64 (ymm4, idxInter128_L512, ymm6);
                //e1,e9,...,e57 #f1,f9,...,f57 #g1,g9,...,g57 #h1,h9,...,h57 #e3,e11,...,e59 #f3,f11,...,f59 #g3,g11,...,g59 #h3,h11,...,h59
                ymmTemp3 = _mm512_permutex2var_epi64 (ymm4, idxInter128_H512, ymm6);
                //a4,a12,...,a60 #b4,b12,...,b60 #c4,c12,...,c60 #d4,d12,...,d60 #a6,a14,...,a62 #b6,b14,...,b62 #c6,c14,...,c62 #d6,d14,...,d62
                ymmTemp4 = _mm512_permutex2var_epi64 (ymm1, idxInter128_L512, ymm3);
                //a5,a13,...,a61 #b5,b13,...,b61 #c5,c13,...,c61 #d5,d13,...,d61 #a7,a15,...,a63 #b7,b15,...,b63 c7,c15,...,c63 #d7,d15,...,d63
                ymmTemp5 = _mm512_permutex2var_epi64 (ymm1, idxInter128_H512, ymm3);
                //e4,e12,...,e60 #f4,f12,...,f60 #g4,g12,...,g60 #h4,h12,...,h60 #e6,e14,...,e62 #d6,d14,...,d62 #g6,g14,...,g62 #h6,h14,...,h62
                ymmTemp6 = _mm512_permutex2var_epi64 (ymm5, idxInter128_L512, ymm7);
                //e5,e13,...,e61 #f5,f13,...,f61 #g5,g13,...,g61 #h5,h13,...,h61 #e7,e15,...,e63 #f7,f15,...,f63 #g7,g15,...,g63 #h7,h15,...,h63
                ymmTemp7 = _mm512_permutex2var_epi64 (ymm5, idxInter128_H512, ymm7);

                //a0, a8, a16, ..., a56, b0, b8, ..., b56, c0, c8, c16, ..., c56, d0, d8, ..., d56,  e0, e8, e16, ..., e56, f0, f8, ..., f56,g0, g8, g16, ..., g56, h0, h8, ..., h56
                ymm0 = _mm512_shuffle_i64x2(ymmTemp0, ymmTemp2, 0x44);
                _mm512_storeu_si512 ((__m512i *) pDeInterleave, ymm0);
                //a1, a9, ..., a57, b1, b9,..., b57, c1, c9, ..., c57, d1, d9,..., d57, e1, e9, ..., e57,  f1, f9,..., f57, g1, g9, ..., g57, h1, h9,..., h57
                ymm1 = _mm512_shuffle_i64x2(ymmTemp1, ymmTemp3, 0x44);
                _mm512_storeu_si512 ((__m512i *) (pDeInterleave+eDiv8), ymm1);
                //a2,a10,...,a58 #b2,b10,...,b58 #c2,c10,...,c58 #d2,d10,...,d58 e2,e10,...,e58 #f2,f10,...,f58 #g2,g10,...,g58 #h2,h10,...,h58
                ymm2 = _mm512_shuffle_i64x2(ymmTemp0, ymmTemp2, 0xEE);
                _mm512_storeu_si512 ((__m512i *)  (pDeInterleave+eDiv4), ymm2);
                //a3,a11,...,a59 #b3,b11,...,b59 #c3,c11,...,c59 #d3,d11,...,d59 e3,e11,...,e59 #f3,f11,...,f59 #g3,g11,...,g59 #h3,h11,...,h59
                ymm3 = _mm512_shuffle_i64x2(ymmTemp1, ymmTemp3, 0xEE);
                _mm512_storeu_si512 ((__m512i *) (pDeInterleave+eDiv4+eDiv8), ymm3);
                //a4,a12,...,a60 #b4,b12,...,b60 #c4,c12,...,c60 #d4,d12,...,d60 e4,e12,...,e60 #f4,f12,...,f60 #g4,g12,...,g60
                ymm4 = _mm512_shuffle_i64x2(ymmTemp4, ymmTemp6, 0x44);
                _mm512_storeu_si512 ((__m512i *)  (pDeInterleave+eDiv2), ymm4);
                //a5,a13,...,a61 #b5,b13,...,b61 #c5,c13,...,c61 #d5,d13,...,d61 e5,e13,...,e61 #f5,f13,...,f61 #g5,g13,...,g61 #h5,h13,...,h61
                ymm5 = _mm512_shuffle_i64x2(ymmTemp5, ymmTemp7, 0x44);
                _mm512_storeu_si512 ((__m512i *) (pDeInterleave+eDiv2+eDiv8), ymm5);
                //a6,a14,...,a62 #b6,b14,...,b62 #c6,c14,...,c62 #d6,d14,...,d62 e6,e14,...,e62 #d6,d14,...,d62 #g6,g14,...,g62 #h6,h14,...,h62
                ymm6 = _mm512_shuffle_i64x2(ymmTemp4, ymmTemp6, 0xEE);
                _mm512_storeu_si512 ((__m512i *)  (pDeInterleave+eDiv2+eDiv4), ymm6);
                //a7,a15,...,a63 #b7,b15,...,b63 c7,c15,...,c63 #d7,d15,...,d63 e7,e15,...,e63 #f7,f15,...,f63 #g7,g15,...,g63 #h7,h15,...,h63
                ymm7 = _mm512_shuffle_i64x2(ymmTemp5, ymmTemp7, 0xEE);
                _mm512_storeu_si512 ((__m512i *) (pDeInterleave+eDiv2+eDiv4+eDiv8), ymm7);

                pDeInterleave += 64;
            }
            pCbIn = pCbIn + 8*len0;

#if 0
            if (len1>16) // FIXME For now not required
            {
                __m128i *pCbInVec128 = (__m128i *)pCbIn;
                len0 = (len1&0x30);  //x*16 bytes
                len1 = (len1&0xf); //left 0~15 bytes
                for (i=0, j=0; i<len0; i+=16, j+=8) {
                    //a0,a8 a1,a9 a2,a10 a3,a11 a4,a12 a5,a13 a6,a14 a7,a15
                    __m128i xmmTemp0 = _mm_shuffle_epi8(pCbInVec128[j], idxInter8_128_shf);
                    //b0,b8 b1,b9 b2,b10 b3,b11 b4,b12 b5,b13 b6,b14 b7,b15
                    __m128i xmmTemp1 = _mm_shuffle_epi8(pCbInVec128[j+1], idxInter8_128_shf);
                    //c0,c8 ...
                    __m128i xmmTemp2 = _mm_shuffle_epi8(pCbInVec128[j+2], idxInter8_128_shf);
                    //d0,d8 ...
                    __m128i xmmTemp3 = _mm_shuffle_epi8(pCbInVec128[j+3], idxInter8_128_shf);
                    //e0,e8 ...
                    __m128i xmmTemp4 = _mm_shuffle_epi8(pCbInVec128[j+4], idxInter8_128_shf);
                    //f0,f8 ...
                    __m128i xmmTemp5 = _mm_shuffle_epi8(pCbInVec128[j+5], idxInter8_128_shf);
                    //g0,g8 ...
                    __m128i xmmTemp6 = _mm_shuffle_epi8(pCbInVec128[j+6], idxInter8_128_shf);
                    //h0,h8 ...
                    __m128i xmmTemp7 = _mm_shuffle_epi8(pCbInVec128[j+7], idxInter8_128_shf);

                    //a0,a8 b0,b8 #a1,a9 b1,b9 #a2,a10 b2,b10 #a3,a11 b3,b11
                    __m128i xmm0 = _mm_permutex2var_epi16(xmmTemp0, idxInter16_L128, xmmTemp1);
                    //a4,a12 b4,b12 #a5,a13 b5,b13 #a6,a14 b6,b14 #a7,a15 b7,b15
                    __m128i xmm1 = _mm_permutex2var_epi16(xmmTemp0, idxInter16_H128, xmmTemp1);
                    //c0,c8 d0,d8 #c1,c9 d1,d9 #c2,c10 d2,d10 #c3,c11 d3,d11
                    __m128i xmm2 = _mm_permutex2var_epi16(xmmTemp2, idxInter16_L128, xmmTemp3);
                    //c4,c12 d4,d12 #c5,c13 d5,d13 #c6,c14 d6,d14 #c7,c15 d7,d15
                    __m128i xmm3 = _mm_permutex2var_epi16(xmmTemp2, idxInter16_H128, xmmTemp3);
                    //e0,e8 f0,f8 ...
                    __m128i xmm4 = _mm_permutex2var_epi16(xmmTemp4, idxInter16_L128, xmmTemp5);
                    //e4,e12 f4,f12 ...
                    __m128i xmm5 = _mm_permutex2var_epi16(xmmTemp4, idxInter16_H128, xmmTemp5);
                    //g0,g8 h0,h8 ...
                    __m128i xmm6 = _mm_permutex2var_epi16(xmmTemp6, idxInter16_L128, xmmTemp7);
                    //g4,g12 h4,h12 ...
                    __m128i xmm7 = _mm_permutex2var_epi16(xmmTemp6, idxInter16_H128, xmmTemp7);

                    //a0 a8, b0 b8, c0 c8, d0 d8, a1, a9, b1, b9, c1, c9, d1, d9
                    xmm0 = _mm_permutex2var_epi32(xmmTemp0, idxInter32_L128, xmmTemp2);
                    //a2, a10,b2, b10, c2, c10,d2, d10, a3, a11,b3, b11, c3, c11,d3, d11
                    xmm1 = _mm_permutex2var_epi32(xmmTemp0, idxInter32_H128, xmmTemp2);
                    //e0 e8, f0 f8, g0 g8, h0 h8,e1, e9, f1, f9, g1, g9, h1, h9
                    xmm2 = _mm_permutex2var_epi32(xmmTemp4, idxInter32_L128, xmmTemp6);
                    //e2, e10, f2, f10, g2, g10, h2, h10, e3,e11, f3, f1, g3,g11, h3, h11
                    xmm3 = _mm_permutex2var_epi32(xmmTemp4, idxInter32_H128, xmmTemp6);
                    //a4,a12,b4,b12, c4,c12,d4,d12, a5,a13, b5,b13, c5,c13, d5,d13
                    xmm4 = _mm_permutex2var_epi32(xmmTemp1, idxInter32_L128, xmmTemp3);
                    //a6,a14,b6,b14, c6,c14,d6,d14,  a7,a15,b7,b15, c7,c15,d7,d15
                    xmm5 = _mm_permutex2var_epi32(xmmTemp1, idxInter32_H128, xmmTemp3);
                    //e4,e12, f4,f12, g4,g12, h4,h12, e5,e13, f5,f13, g5,g13, h5,h13
                    xmm6 = _mm_permutex2var_epi32(xmmTemp5, idxInter32_L128, xmmTemp7);
                    // e6,e14, f6,f14, g6,g14, h6,h14, e7,e15, f7,f15, g7,g15, h7,h15
                    xmm7 = _mm_permutex2var_epi32(xmmTemp5, idxInter32_H128, xmmTemp7);

                    //a0 a8,b0, b8, c0, c8, d0, d8, e0,e8, f0,f8, g0, g8, h0, h8
                    xmmTemp0 = _mm_permutex2var_epi64(xmm0, idxHalf128L, xmm2);
                    _mm_storeu_si128 ((__m128i *) pDeInterleave, xmmTemp0);
                    //a1, a9, b1, b9, c1, c9, d1, d9, e1, e9, f1, f9, g1, g9, h1, h9
                    xmmTemp1 = _mm_permutex2var_epi64(xmm0, idxHalf128H, xmm2);
                    _mm_storeu_si128 ((__m128i *) (pDeInterleave+eDiv8), xmmTemp1);

                    //a2, a10,b2, b10, c2, c10,d2, d10, e2, e10, f2, f10, g2, g10, h2, h10
                    xmmTemp2 = _mm_permutex2var_epi64(xmm1, idxHalf128L, xmm3);
                    _mm_storeu_si128 ((__m128i *)  (pDeInterleave+eDiv4), xmmTemp2);
                    //a3, a11,b3, b11, c3, c11,d3, d11, e3,e11, f3, f1, g3,g11, h3, h11
                    xmmTemp3 = _mm_permutex2var_epi64(xmm1, idxHalf128H, xmm3);
                    _mm_storeu_si128 ((__m128i *)  (pDeInterleave+eDiv4+eDiv8), xmmTemp3);

                    //a4,a12,b4,b12, c4,c12,d4,d12, e4,e12, f4,f12, g4,g12, h4,h12
                    xmmTemp4 = _mm_permutex2var_epi64(xmm4, idxHalf128L, xmm6);
                    _mm_storeu_si128 ((__m128i *)  (pDeInterleave+eDiv2), xmmTemp4);
                    //a5,a13, b5,b13, c5,c13, d5,d13, e5,e13, f5,f13, g5,g13, h5,h13
                    xmmTemp5 = _mm_permutex2var_epi64(xmm4, idxHalf128H, xmm6);
                    _mm_storeu_si128 ((__m128i *) (pDeInterleave+eDiv2+eDiv8), xmmTemp5);

                    //a6,a14,b6,b14, c6,c14,d6,d14, 6,e14, f6,f14, g6,g14, h6,h14
                    xmmTemp6 = _mm_permutex2var_epi64(xmm5, idxHalf128L, xmm7);
                    _mm_storeu_si128 ((__m128i *) (pDeInterleave+eDiv2+eDiv4), xmmTemp6);
                    //a7,a15,b7,b15, c7,c15,d7,d15, e7,e15, f7,f15, g7,g15, h7,h15
                    xmmTemp7 = _mm_permutex2var_epi64(xmm5, idxHalf128H, xmm7);
                    _mm_storeu_si128 ((__m128i *) (pDeInterleave+eDiv2+eDiv4+eDiv8), xmmTemp7);

                    pDeInterleave += 16;
                }
                pCbIn = pCbIn + 8*len0;
            }
#endif

            //left 0~7bytes
            for (i=0; i<len1; i++) {
                *pDeInterleave = *pCbIn++;
                *(pDeInterleave+eDiv8) = *pCbIn++;
                *(pDeInterleave+eDiv4) = *pCbIn++;
                *(pDeInterleave+eDiv4+eDiv8) = *pCbIn++;
                *(pDeInterleave+eDiv2) = *pCbIn++;
                *(pDeInterleave+eDiv2+eDiv8) = *pCbIn++;
                *(pDeInterleave+eDiv2+eDiv4) = *pCbIn++;
                *(pDeInterleave+eDiv2+eDiv4+eDiv8) = *pCbIn++;
                pDeInterleave++;
            }

            break;
        }
    }

}
/**
 * @brief Implements rate dematching with AVX512
 * @param [in] request Structure containing the configuration, input data
 * @param [out] response Structure containing the output data.
**/
void bblib_rate_dematching_5gnr_avx512(struct bblib_rate_dematching_5gnr_request *req,
struct bblib_rate_dematching_5gnr_response *resp)
{
    __declspec (align(64)) int8_t internalBuffer[65536];
    deInterleave_avx512((uint8_t *) req->p_in, internalBuffer, req);
    harq_combine_avx512(req, resp, internalBuffer);
}
