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

/**
 * @file   phy_crc_sse.cpp
 * @brief  Implementation of LTE CRC24A/CRC24B and the corresponding
 *         CRC generation, check functions
 */

/**
 * Include public/global header files
 */
#include <tmmintrin.h> // SSSE 3

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "phy_crc_internal.h"
#include "phy_crc.h"
#if defined(_BBLIB_SSE4_2_) || defined(_BBLIB_AVX2_) || defined(_BBLIB_AVX512_)
struct init_crc_sse
{
    init_crc_sse()
    {

        bblib_print_crc_version();

    }
};

init_crc_sse do_constructor_crc_sse;


void bblib_lte_crc24a_gen_sse(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    uint8_t *data = request->data;
    uint8_t *dataOut = response->data;

    // len is passed in as bits, so turn into bytes
    uint32_t len_bytes = request->len / 8;

    /* record the start of input data */
    int32_t    i = 0;
    /* A= B mod C => A*K = B*K mod C*K, set K = 2^8, then compute CRC-32, the most significant */
    /* 24 bits is final 24 bits CRC. */
    const static uint64_t  CRC24APOLY = 0x1864CFB;   //CRC-24A polynomial
    const static uint64_t  CRC24APLUS8 = CRC24APOLY << 8;

    /* some pre-computed key constants */
    const static uint32_t k192   = 0x2c8c9d00;   //t=128+64, x^192 mod CRC24APLUS8, verified
    const static uint32_t k128   = 0x64e4d700;   //t=128, x^128 mod CRC24APLUS8, verified
    const static uint32_t k96    = 0xfd7e0c00;   //t=96, x^96 mod CRC24APLUS8, verified
    const static uint32_t k64    = 0xd9fe8c00;   //t=64, x^64 mod CRC24APLUS8, verified
    const static uint64_t u      = 0x1f845fe24;  //u for crc24A * 256, floor(x^64 / CRC24APLUS8), verified
    const static __m128i ENDIA_SHUF_MASK = _mm_set_epi8(0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                                  0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F);

    __m128i xmm3, xmm2, xmm1, xmm0;

    /* 1. fold by 128bit. remaining length <=2*128bits. */
    xmm3 = _mm_set_epi32(0, k192, 0, k128);
    xmm1 = _mm_load_si128((__m128i *)data); data += 16;
    xmm1 = _mm_shuffle_epi8(xmm1, ENDIA_SHUF_MASK);

    for (i=(len_bytes>>4)-1; i>0; i--){
        xmm2 = xmm1;
        xmm0 = _mm_load_si128((__m128i *)data);  data += 16;
        xmm0 = _mm_shuffle_epi8(xmm0, ENDIA_SHUF_MASK);
        xmm1 = _mm_clmulepi64_si128(xmm1, xmm3, 0x00);
        xmm2 = _mm_clmulepi64_si128(xmm2, xmm3, 0x11);
        xmm0 = _mm_xor_si128(xmm1, xmm0);
        xmm1 = _mm_xor_si128(xmm2, xmm0);
    }


    /* 2. if remaining length > 128 bits, then pad zero to the most-significant bit to grow to 256bits length,
     * then fold once to 128 bits. */
    if (len_bytes>16){
        xmm0 = _mm_load_si128((__m128i *)data); //load remaining len%16 bytes and maybe some garbage bytes.
        xmm0 = _mm_shuffle_epi8(xmm0, ENDIA_SHUF_MASK);
        for (i=15-len_bytes%16; i>=0; i--){
            xmm0 = _mm_srli_si128(xmm0, 1);
            xmm0 = _mm_insert_epi8(xmm0, _mm_extract_epi8(xmm1, 0), 15);
            xmm1 = _mm_srli_si128(xmm1, 1);
        }
        xmm2 = xmm1;
        xmm1 = _mm_clmulepi64_si128(xmm1, xmm3, 0x00);
        xmm2 = _mm_clmulepi64_si128(xmm2, xmm3, 0x11);
        xmm0 = _mm_xor_si128(xmm1, xmm0);
        xmm0 = _mm_xor_si128(xmm2, xmm0);
    }
    else{
        xmm0 = xmm1;
        for (i=16-len_bytes; i>0; i--){
            xmm0 = _mm_srli_si128(xmm0, 1);
        }
    }


    /* 3. Apply 64 bits fold to 64 bits + 32 bits crc(32 bits zero) */
    xmm3 =  _mm_set_epi32(0, 0, 0, k96);
    xmm1 = _mm_clmulepi64_si128(xmm0, xmm3, 0x01);
    xmm2 = _mm_slli_si128(xmm0, 8);
    xmm2 = _mm_srli_si128(xmm2, 4);
    xmm0 = _mm_xor_si128(xmm1, xmm2);


    /* 4. Apply 32 bits fold to 32 bits + 32 bits crc(32 bits zero) */
    xmm3 =  _mm_set_epi32(0, 0, 0, k64);
    xmm1 = _mm_clmulepi64_si128(xmm0, xmm3, 0x01);
    xmm0 = _mm_slli_si128(xmm0, 8);
    xmm0 = _mm_srli_si128(xmm0, 8);
    xmm0 = _mm_xor_si128(xmm1, xmm0);

    /* 5. Use Barrett Reduction Algorithm to calculate the 32 bits crc.
     * Output: C(x)  = R(x) mod P(x)
     * Step 1: T1(x) = floor(R(x)/x^32)) * u
     * Step 2: T2(x) = floor(T1(x)/x^32)) * P(x)
     * Step 3: C(x)  = R(x) xor T2(x) mod x^32 */
    xmm1 = _mm_set_epi32(0, 0, 1, (u & 0xFFFFFFFF));
    xmm2 = _mm_srli_si128(xmm0, 4);
    xmm1 = _mm_clmulepi64_si128(xmm1, xmm2, 0x00);
    xmm1 = _mm_srli_si128(xmm1, 4);
    xmm2 = _mm_set_epi32(0, 0, 1, (CRC24APLUS8 & 0xFFFFFFFF));
    xmm1 = _mm_clmulepi64_si128(xmm1, xmm2, 0x00);
    xmm0 = _mm_xor_si128(xmm0, xmm1);
    xmm1 = _mm_set_epi32(0, 0, 0, 0xFFFFFFFF);
    xmm0 = _mm_and_si128 (xmm0, xmm1);


    /* 6. Update Result
    /* add crc to last 3 bytes. */
    dataOut[len_bytes]   =  _mm_extract_epi8(xmm0, 3);
    dataOut[len_bytes+1] =  _mm_extract_epi8(xmm0, 2);
    dataOut[len_bytes+2] =  _mm_extract_epi8(xmm0, 1);
    response->len = (len_bytes + 3)*8;

    /* the most significant 24 bits of the 32 bits crc is the finial 24 bits crc. */
    response->crc_value = (((uint32_t)_mm_extract_epi32(xmm0, 0)) >> 8);
}



void bblib_lte_crc24b_gen_sse(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    uint8_t *data = request->data;
    uint8_t *dataOut = response->data;

    // len is passed in as bits, so turn into bytes
    uint32_t len_bytes = request->len / 8;

    /* record the start of input data */
    int32_t    i = 0;

    /* A= B mod C => A*K = B*K mod C*K, set K = 2^8, then compute CRC-32, the most significant
     * 24 bits is final 24 bits CRC. */
    const static uint64_t  CRC24BPOLY = 0x1800063;    //CRC24B Polynomial
    const static uint64_t  CRC24BPLUS8 = CRC24BPOLY << 8;

    /* some pre-computed key constants */
    const static uint32_t k192   = 0x42000100;   //t=128+64, x^192 mod CRC24BPLUS8, verified
    const static uint32_t k128   = 0x80140500;   //t=128, x^128 mod CRC24BPLUS8, verified
    const static uint32_t k96    = 0x09000200;   //t=96, x^96 mod CRC24BPLUS8, verified
    const static uint32_t k64    = 0x90042100;   //t=64, x^64 mod CRC24BPLUS8, verified
    const static uint64_t u      = 0x1ffff83ff;  //u for crc24A * 256, floor(x^64 / CRC24BPULS8), verified
    const static __m128i ENDIA_SHUF_MASK = _mm_set_epi8(0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                                  0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F);
    /* variables */
    __m128i xmm3, xmm2, xmm1, xmm0;

    /* 1. fold by 128bit. remaining length <=2*128bits. */
    xmm3 = _mm_set_epi32(0, k192, 0, k128);
    xmm1 = _mm_load_si128((__m128i *)data); data += 16;
    xmm1 = _mm_shuffle_epi8(xmm1, ENDIA_SHUF_MASK);

    for (i=(len_bytes>>4)-1; i>0; i--){
        xmm2 = xmm1;
        xmm0 = _mm_load_si128((__m128i *)data);  data += 16;
        xmm0 = _mm_shuffle_epi8(xmm0, ENDIA_SHUF_MASK);
        xmm1 = _mm_clmulepi64_si128(xmm1, xmm3, 0x00);
        xmm2 = _mm_clmulepi64_si128(xmm2, xmm3, 0x11);
        xmm0 = _mm_xor_si128(xmm1, xmm0);
        xmm1 = _mm_xor_si128(xmm2, xmm0);
    }

    /* 2. if remaining length > 128 bits, then pad zero to the most-significant bit to grow to 256bits length,
     *   then fold once to 128 bits. */
    if (16 < len_bytes){
        xmm0 = _mm_load_si128((__m128i *)data); //load remaining len%16 bytes and maybe some garbage bytes.
        xmm0 = _mm_shuffle_epi8(xmm0, ENDIA_SHUF_MASK);
        for (i=15-len_bytes%16; i>=0; i--){
            xmm0 = _mm_srli_si128(xmm0, 1);
            xmm0 = _mm_insert_epi8(xmm0, _mm_extract_epi8(xmm1, 0), 15);
            xmm1 = _mm_srli_si128(xmm1, 1);
        }
        xmm2 = xmm1;
        xmm1 = _mm_clmulepi64_si128(xmm1, xmm3, 0x00);
        xmm2 = _mm_clmulepi64_si128(xmm2, xmm3, 0x11);
        xmm0 = _mm_xor_si128(xmm1, xmm0);
        xmm0 = _mm_xor_si128(xmm2, xmm0);
    }
    else{
        xmm0 = xmm1;
        for (i=16-len_bytes; i>0; i--){
            xmm0 = _mm_srli_si128(xmm0, 1);
        }
    }

    /* 3. Apply 64 bits fold to 64 bits + 32 bits crc(32 bits zero) */
    xmm3 =  _mm_set_epi32(0, 0, 0, k96);
    xmm1 = _mm_clmulepi64_si128(xmm0, xmm3, 0x01);
    xmm2 = _mm_slli_si128(xmm0, 8);
    xmm2 = _mm_srli_si128(xmm2, 4);
    xmm0 = _mm_xor_si128(xmm1, xmm2);

    /* 4. Apply 32 bits fold to 32 bits + 32 bits crc(32 bits zero) */
    xmm3 =  _mm_set_epi32(0, 0, 0, k64);
    xmm1 = _mm_clmulepi64_si128(xmm0, xmm3, 0x01);
    xmm0 = _mm_slli_si128(xmm0, 8);
    xmm0 = _mm_srli_si128(xmm0, 8);
    xmm0 = _mm_xor_si128(xmm1, xmm0);

    /* 5. Use Barrett Reduction Algorithm to calculate the 32 bits crc.
     * Output: C(x)  = R(x) mod P(x)
     * Step 1: T1(x) = floor(R(x)/x^32)) * u
     * Step 2: T2(x) = floor(T1(x)/x^32)) * P(x)
     * Step 3: C(x)  = R(x) xor T2(x) mod x^32 */
    xmm1 = _mm_set_epi32(0, 0, 1, (u & 0xFFFFFFFF));
    xmm2 = _mm_srli_si128(xmm0, 4);
    xmm1 = _mm_clmulepi64_si128(xmm1, xmm2, 0x00);
    xmm1 = _mm_srli_si128(xmm1, 4);
    xmm2 =  _mm_set_epi32(0, 0, 1, (CRC24BPLUS8 & 0xFFFFFFFF));
    xmm1 = _mm_clmulepi64_si128(xmm1, xmm2, 0x00);
    xmm0 = _mm_xor_si128(xmm0, xmm1);
    xmm1 = _mm_set_epi32(0, 0, 0, 0xFFFFFFFF);
    xmm0 = _mm_and_si128 (xmm0, xmm1);


    /* 6. Update Result
    /* add crc to last 3 bytes. */
    dataOut[len_bytes]   =  _mm_extract_epi8(xmm0, 3);
    dataOut[len_bytes+1] =  _mm_extract_epi8(xmm0, 2);
    dataOut[len_bytes+2] =  _mm_extract_epi8(xmm0, 1);
    response->len = (len_bytes + 3)*8;

    /* the most significant 24 bits of the 32 bits crc is the finial 24 bits crc. */
    response->crc_value = (((uint32_t)_mm_extract_epi32(xmm0, 0)) >> 8);

}



void bblib_lte_crc24a_check_sse(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    if (request->data == NULL){
        printf("bblib_lte_crc24a_check input / output address error \n");
        response->check_passed = false;
        return;
    }

    // len is passed in as bits, so turn into bytes
    uint32_t len_bytes = request->len / 8;

    /* CRC in the original sequence */
    uint32_t CRC_orig = 0;

    CRC_orig = ((request->data[len_bytes]<<16)&0x00FF0000) +
               ((request->data[len_bytes+1]<<8)&0x0000FF00) +
               (request->data[len_bytes+2]&0x000000FF);

    bblib_lte_crc24a_gen_sse(request, response);

    if (response->crc_value != CRC_orig)
        response->check_passed = false;
    else
        response->check_passed = true;
}



void bblib_lte_crc24b_check_sse(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    if (request->data == NULL){
        printf("bblib_lte_crc24b_check input / output address error \n");
        response->check_passed = false;
        return;
    }

    // len is passed in as bits, so turn into bytes
    uint32_t len_bytes = request->len / 8;

    /* CRC in the original sequence */
    uint32_t CRC_orig = 0;

    CRC_orig = ((request->data[len_bytes]<<16)&0x00FF0000) +
               ((request->data[len_bytes+1]<<8)&0x0000FF00) +
               (request->data[len_bytes+2]&0x000000FF);
    bblib_lte_crc24b_gen_sse(request, response);

    if (response->crc_value != CRC_orig)
        response->check_passed = false;
    else
        response->check_passed = true;
}



void bblib_lte_crc16_gen_sse(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    uint8_t *data = request->data;
    uint8_t *dataOut = response->data;

    // len is passed in as bits, so turn into bytes
    uint32_t len_bytes = request->len / 8;

    /* record the start of input data */
    int32_t    i = 0;

    /* A= B mod C => A*K = B*K mod C*K, set K = 2^8, then compute CRC-32, the most significant
     * 16 bits is final 16 bit CRC. */
    const static uint64_t  CRC16POLY = 0x11021;    //CRC16 Polynomial
    const static uint64_t  CRC16PLUS16 = CRC16POLY << 16;  //pads poly to 32bits

    /* some pre-computed key constants */
    const static uint32_t k192   = 0xd5f60000;   //t=128+64, x^192 mod CRC16PLUS16, verified
    const static uint32_t k128   = 0x45630000;   //t=128, x^128 mod CRC16PLUS16, verified
    const static uint32_t k96    = 0xeb230000;   //t=96, x^96 mod CRC16PLUS16, verified
    const static uint32_t k64    = 0xaa510000;   //t=64, x^64 mod CRC16PLUS16, verified
    const static uint64_t u      = 0x111303471;  //u for crc16 * 256, floor(x^64 / CRC16PULS16), verified
    const static __m128i ENDIA_SHUF_MASK = _mm_set_epi8(0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                                  0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F);
    /* variables */
    __m128i xmm3, xmm2, xmm1, xmm0;

    /* 1. fold by 128bit. remaining length <=2*128bits. */
    xmm3 = _mm_set_epi32(0, k192, 0, k128);
    xmm1 = _mm_load_si128((__m128i *)data); data += 16;
    xmm1 = _mm_shuffle_epi8(xmm1, ENDIA_SHUF_MASK);

    for (i=(len_bytes>>4)-1; i>0; i--){
        xmm2 = xmm1;
        xmm0 = _mm_load_si128((__m128i *)data);  data += 16;
        xmm0 = _mm_shuffle_epi8(xmm0, ENDIA_SHUF_MASK);
        xmm1 = _mm_clmulepi64_si128(xmm1, xmm3, 0x00);
        xmm2 = _mm_clmulepi64_si128(xmm2, xmm3, 0x11);
        xmm0 = _mm_xor_si128(xmm1, xmm0);
        xmm1 = _mm_xor_si128(xmm2, xmm0);
    }

    /* 2. if remaining length > 128 bits, then pad zero to the most-significant bit to grow to 256bits length,
         *   then fold once to 128 bits. */
    if (16 < len_bytes){
        xmm0 = _mm_load_si128((__m128i *)data); //load remaining len%16 bytes and maybe some garbage bytes.
        xmm0 = _mm_shuffle_epi8(xmm0, ENDIA_SHUF_MASK);
        for (i=15-len_bytes%16; i>=0; i--){
            xmm0 = _mm_srli_si128(xmm0, 1);
            xmm0 = _mm_insert_epi8(xmm0, _mm_extract_epi8(xmm1, 0), 15);
            xmm1 = _mm_srli_si128(xmm1, 1);
        }
        xmm2 = xmm1;
        xmm1 = _mm_clmulepi64_si128(xmm1, xmm3, 0x00);
        xmm2 = _mm_clmulepi64_si128(xmm2, xmm3, 0x11);
        xmm0 = _mm_xor_si128(xmm1, xmm0);
        xmm0 = _mm_xor_si128(xmm2, xmm0);
    }
    else{
        xmm0 = xmm1;
        for (i=16-len_bytes; i>0; i--){
            xmm0 = _mm_srli_si128(xmm0, 1);
        }
    }

    /* 3. Apply 64 bits fold to 64 bits + 32 bits crc(32 bits zero) */
    xmm3 =  _mm_set_epi32(0, 0, 0, k96);
    xmm1 = _mm_clmulepi64_si128(xmm0, xmm3, 0x01);
    xmm2 = _mm_slli_si128(xmm0, 8);
    xmm2 = _mm_srli_si128(xmm2, 4);
    xmm0 = _mm_xor_si128(xmm1, xmm2);

    /* 4. Apply 32 bits fold to 32 bits + 32 bits crc(32 bits zero) */
    xmm3 =  _mm_set_epi32(0, 0, 0, k64);
    xmm1 = _mm_clmulepi64_si128(xmm0, xmm3, 0x01);
    xmm0 = _mm_slli_si128(xmm0, 8);
    xmm0 = _mm_srli_si128(xmm0, 8);
    xmm0 = _mm_xor_si128(xmm1, xmm0);


    /* 5. Use Barrett Reduction Algorithm to calculate the 32 bits crc.
     * Output: C(x)  = R(x) mod P(x)
     * Step 1: T1(x) = floor(R(x)/x^32)) * u
     * Step 2: T2(x) = floor(T1(x)/x^32)) * P(x)
     * Step 3: C(x)  = R(x) xor T2(x) mod x^32 */
    xmm1 = _mm_set_epi32(0, 0, 1, (u & 0xFFFFFFFF));
    xmm2 = _mm_srli_si128(xmm0, 4);
    xmm1 = _mm_clmulepi64_si128(xmm1, xmm2, 0x00);
    xmm1 = _mm_srli_si128(xmm1, 4);
    xmm2 =  _mm_set_epi32(0, 0, 1, (CRC16PLUS16 & 0xFFFFFFFF));
    xmm1 = _mm_clmulepi64_si128(xmm1, xmm2, 0x00);
    xmm0 = _mm_xor_si128(xmm0, xmm1);
    xmm1 = _mm_set_epi32(0, 0, 0, 0xFFFFFFFF);
    xmm0 = _mm_and_si128 (xmm0, xmm1);


    /* 6. Update Result
    /* add crc to last 2 bytes. */
    dataOut[len_bytes]   =  _mm_extract_epi8(xmm0, 3);
    dataOut[len_bytes+1] =  _mm_extract_epi8(xmm0, 2);
    response->len = (len_bytes + 2)*8;

    /* the most significant 24 bits of the 32 bits crc is the finial 24 bits crc. */
    response->crc_value = (((uint32_t)_mm_extract_epi32(xmm0, 0)) >> 16);

}



void bblib_lte_crc16_check_sse(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    if (request->data == NULL){
        printf("bblib_lte_crc16_check input / output address error \n");
        response->check_passed = false;
        return;
    }

    // len is passed in as bits, so turn into bytes
    uint32_t len_bytes = request->len / 8;

    /* CRC in the original sequence */
    uint32_t CRC_orig = 0;

    CRC_orig = ((request->data[len_bytes]<<8)&0x0000FF00) +
               (request->data[len_bytes+1]&0x000000FF);
    bblib_lte_crc16_gen_sse(request, response);

    if (response->crc_value != CRC_orig)
        response->check_passed = false;
    else
        response->check_passed = true;
}



void bblib_lte_crc11_gen_sse(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    _mm256_zeroupper();
    uint8_t *data = request->data;
    uint8_t *dataOut = response->data;

    // len is passed in as bits, so turn to bytes
    uint32_t len_bytes = request->len / 8;

    /* record the start of input data */
    int32_t    i = 0;

    /* A= B mod C => A*K = B*K mod C*K, set K = 2^8, then compute CRC-32, the most significant
     * 11 bits is final 11 bit CRC. */
    const static uint64_t  CRC11POLY = 0xe21;    //CRC11 Polynomial
    const static uint64_t  CRC11PLUS21 = CRC11POLY << 21;  //pads poly to 32bits

    /* some pre-computed key constants */
    const static uint32_t k192   = 0x8ea00000;   //t=128+64, x^192 mod CRC11PLUS21, verified
    const static uint32_t k128   = 0x47600000;   //t=128, x^128 mod CRC11PLUS21, verified
    const static uint32_t k96    = 0x5e600000;   //t=96, x^96 mod CRC11PLUS21, verified
    const static uint32_t k64    = 0xc9000000;   //t=64, x^64 mod CRC11PLUS21, verified
    const static uint64_t u      = 0x1b3fa1f48;  //u for crc11 * 256, floor(x^64 / CRC11PLUS21), verified
    const static __m128i ENDIA_SHUF_MASK = _mm_set_epi8(0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                                  0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F);

    /* variables */
    __m128i xmm3, xmm2, xmm1, xmm0;

    /* 1. fold by 128bit. remaining length <=2*128bits. */
    xmm3 = _mm_set_epi32(0, k192, 0, k128);
    xmm1 = _mm_load_si128((__m128i *)data); data += 16;
    xmm1 = _mm_shuffle_epi8(xmm1, ENDIA_SHUF_MASK);

    for (i=(len_bytes>>4)-1; i>0; i--){
        xmm2 = xmm1;
        xmm0 = _mm_load_si128((__m128i *)data);  data += 16;
        xmm0 = _mm_shuffle_epi8(xmm0, ENDIA_SHUF_MASK);
        xmm1 = _mm_clmulepi64_si128(xmm1, xmm3, 0x00);
        xmm2 = _mm_clmulepi64_si128(xmm2, xmm3, 0x11);
        xmm0 = _mm_xor_si128(xmm1, xmm0);
        xmm1 = _mm_xor_si128(xmm2, xmm0);
    }

    /* 2. if remaining length > 128 bits, then pad zero to the most-significant bit to grow to 256bits length,
        *   then fold once to 128 bits. */
    if (16 < len_bytes){
        xmm0 = _mm_load_si128((__m128i *)data); //load remaining len%16 bytes and maybe some garbage bytes.
        xmm0 = _mm_shuffle_epi8(xmm0, ENDIA_SHUF_MASK);
        for (i=15-len_bytes%16; i>=0; i--){
            xmm0 = _mm_srli_si128(xmm0, 1);
            xmm0 = _mm_insert_epi8(xmm0, _mm_extract_epi8(xmm1, 0), 15);
            xmm1 = _mm_srli_si128(xmm1, 1);
        }
        xmm2 = xmm1;
        xmm1 = _mm_clmulepi64_si128(xmm1, xmm3, 0x00);
        xmm2 = _mm_clmulepi64_si128(xmm2, xmm3, 0x11);
        xmm0 = _mm_xor_si128(xmm1, xmm0);
        xmm0 = _mm_xor_si128(xmm2, xmm0);
    }
    else{
        xmm0 = xmm1;
        for (i=16-len_bytes; i>0; i--){
            xmm0 = _mm_srli_si128(xmm0, 1);
        }
    }

    /* 3. Apply 64 bits fold to 64 bits + 32 bits crc(32 bits zero) */
    xmm3 =  _mm_set_epi32(0, 0, 0, k96);
    xmm1 = _mm_clmulepi64_si128(xmm0, xmm3, 0x01);
    xmm2 = _mm_slli_si128(xmm0, 8);
    xmm2 = _mm_srli_si128(xmm2, 4);
    xmm0 = _mm_xor_si128(xmm1, xmm2);

    /* 4. Apply 32 bits fold to 32 bits + 32 bits crc(32 bits zero) */
    xmm3 =  _mm_set_epi32(0, 0, 0, k64);
    xmm1 = _mm_clmulepi64_si128(xmm0, xmm3, 0x01);
    xmm0 = _mm_slli_si128(xmm0, 8);
    xmm0 = _mm_srli_si128(xmm0, 8);
    xmm0 = _mm_xor_si128(xmm1, xmm0);


    /* 5. Use Barrett Reduction Algorithm to calculate the 32 bits crc.
     * Output: C(x)  = R(x) mod P(x)
     * Step 1: T1(x) = floor(R(x)/x^32)) * u
     * Step 2: T2(x) = floor(T1(x)/x^32)) * P(x)
     * Step 3: C(x)  = R(x) xor T2(x) mod x^32 */
    xmm1 = _mm_set_epi32(0, 0, 1, (u & 0xFFFFFFFF));
    xmm2 = _mm_srli_si128(xmm0, 4);
    xmm1 = _mm_clmulepi64_si128(xmm1, xmm2, 0x00);
    xmm1 = _mm_srli_si128(xmm1, 4);
    xmm2 =  _mm_set_epi32(0, 0, 1, (CRC11PLUS21 & 0xFFFFFFFF));
    xmm1 = _mm_clmulepi64_si128(xmm1, xmm2, 0x00);
    xmm0 = _mm_xor_si128(xmm0, xmm1);
    xmm1 = _mm_set_epi32(0, 0, 0, 0xFFFFFFFF);
    xmm0 = _mm_and_si128 (xmm0, xmm1);


    /* 6. Update Result
    /* add crc to last byte. */
    dataOut[len_bytes]   =  _mm_extract_epi8(xmm0, 3);
    dataOut[len_bytes+1] =  _mm_extract_epi8(xmm0, 2);
    response->len = ((len_bytes + 2)*8) - 5;

    /* the most significant 11 bits of the 32 bits crc is the final 11 bits crc. */
    response->crc_value = (((uint32_t)_mm_extract_epi32(xmm0, 0)) >> 21);
    _mm256_zeroupper();
}



void bblib_lte_crc11_check_sse(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    if (request->data == NULL) {
        printf("bblib_lte_crc11_check input / output address error \n");
        response->check_passed = false;
        return;
    }
    // len is passed in as bits, so turn into bytes
    uint32_t len_bytes = request->len / 8;

    /* CRC in the original sequence */
    uint32_t CRC_orig = 0;

    CRC_orig = (((request->data[len_bytes]<<8)&0x0000FF00) +
               ((request->data[len_bytes+1])&0x000000FF)) >> 5;
    bblib_lte_crc11_gen_sse(request, response);

    if (response->crc_value != CRC_orig)
        response->check_passed = false;
    else
        response->check_passed = true;
}



void bblib_lte_crc6_gen_sse(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    _mm256_zeroupper();
    uint8_t *data = request->data;
    uint8_t *dataOut = response->data;

    // len is passed in as bits, so turn to bytes
    uint32_t len_bytes = request->len / 8;

    /* record the start of input data */
    int32_t    i = 0;

    /* A= B mod C => A*K = B*K mod C*K, set K = 2^8, then compute CRC-32, the most significant
     * 6 bits is final 6 bit CRC. */
    const static uint64_t  CRC6POLY = 0x61;    //CRC6 Polynomial
    const static uint64_t  CRC6PLUS26 = CRC6POLY << 26;  //pads poly to 32bits

    /* some pre-computed key constants */
    const static uint32_t k192   = 0x38000000;   //t=128+64, x^192 mod CRC6PLUS26, verified
    const static uint32_t k128   = 0x1c000000;   //t=128, x^128 mod CRC6PLUS26, verified
    const static uint32_t k96    = 0x8c000000;   //t=96, x^96 mod CRC6PLUS26, verified
    const static uint32_t k64    = 0xcc000000;   //t=64, x^64 mod CRC6PLUS26, verified
    const static uint64_t u      = 0x1fab37693;  //u for crc11 * 256, floor(x^64 / CRC6PLUS26), verified
    const static __m128i ENDIA_SHUF_MASK = _mm_set_epi8(0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                                  0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F);

    /* variables */
    __m128i xmm3, xmm2, xmm1, xmm0;

    /* 1. fold by 128bit. remaining length <=2*128bits. */
    xmm3 = _mm_set_epi32(0, k192, 0, k128);
    xmm1 = _mm_load_si128((__m128i *)data); data += 16;
    xmm1 = _mm_shuffle_epi8(xmm1, ENDIA_SHUF_MASK);

    for (i=(len_bytes>>4)-1; i>0; i--){
        xmm2 = xmm1;
        xmm0 = _mm_load_si128((__m128i *)data);  data += 16;
        xmm0 = _mm_shuffle_epi8(xmm0, ENDIA_SHUF_MASK);
        xmm1 = _mm_clmulepi64_si128(xmm1, xmm3, 0x00);
        xmm2 = _mm_clmulepi64_si128(xmm2, xmm3, 0x11);
        xmm0 = _mm_xor_si128(xmm1, xmm0);
        xmm1 = _mm_xor_si128(xmm2, xmm0);
    }

    /* 2. if remaining length > 128 bits, then pad zero to the most-significant bit to grow to 256bits length,
         *   then fold once to 128 bits. */
    if (16 < len_bytes){
        xmm0 = _mm_load_si128((__m128i *)data); //load remaining len%16 bytes and maybe some garbage bytes.
        xmm0 = _mm_shuffle_epi8(xmm0, ENDIA_SHUF_MASK);
        for (i=15-len_bytes%16; i>=0; i--){
            xmm0 = _mm_srli_si128(xmm0, 1);
            xmm0 = _mm_insert_epi8(xmm0, _mm_extract_epi8(xmm1, 0), 15);
            xmm1 = _mm_srli_si128(xmm1, 1);
        }
        xmm2 = xmm1;
        xmm1 = _mm_clmulepi64_si128(xmm1, xmm3, 0x00);
        xmm2 = _mm_clmulepi64_si128(xmm2, xmm3, 0x11);
        xmm0 = _mm_xor_si128(xmm1, xmm0);
        xmm0 = _mm_xor_si128(xmm2, xmm0);
    }
    else{
        xmm0 = xmm1;
        for (i=16-len_bytes; i>0; i--){
            xmm0 = _mm_srli_si128(xmm0, 1);
        }
    }

    /* 3. Apply 64 bits fold to 64 bits + 32 bits crc(32 bits zero) */
    xmm3 =  _mm_set_epi32(0, 0, 0, k96);
    xmm1 = _mm_clmulepi64_si128(xmm0, xmm3, 0x01);
    xmm2 = _mm_slli_si128(xmm0, 8);
    xmm2 = _mm_srli_si128(xmm2, 4);
    xmm0 = _mm_xor_si128(xmm1, xmm2);

    /* 4. Apply 32 bits fold to 32 bits + 32 bits crc(32 bits zero) */
    xmm3 =  _mm_set_epi32(0, 0, 0, k64);
    xmm1 = _mm_clmulepi64_si128(xmm0, xmm3, 0x01);
    xmm0 = _mm_slli_si128(xmm0, 8);
    xmm0 = _mm_srli_si128(xmm0, 8);
    xmm0 = _mm_xor_si128(xmm1, xmm0);


    /* 5. Use Barrett Reduction Algorithm to calculate the 32 bits crc.
     * Output: C(x)  = R(x) mod P(x)
     * Step 1: T1(x) = floor(R(x)/x^32)) * u
     * Step 2: T2(x) = floor(T1(x)/x^32)) * P(x)
     * Step 3: C(x)  = R(x) xor T2(x) mod x^32 */
    xmm1 = _mm_set_epi32(0, 0, 1, (u & 0xFFFFFFFF));
    xmm2 = _mm_srli_si128(xmm0, 4);
    xmm1 = _mm_clmulepi64_si128(xmm1, xmm2, 0x00);
    xmm1 = _mm_srli_si128(xmm1, 4);
    xmm2 =  _mm_set_epi32(0, 0, 1, (CRC6PLUS26 & 0xFFFFFFFF));
    xmm1 = _mm_clmulepi64_si128(xmm1, xmm2, 0x00);
    xmm0 = _mm_xor_si128(xmm0, xmm1);
    xmm1 = _mm_set_epi32(0, 0, 0, 0xFFFFFFFF);
    xmm0 = _mm_and_si128 (xmm0, xmm1);


    /* 6. Update Result
    /* add crc to last byte. */
    dataOut[len_bytes]   =  _mm_extract_epi8(xmm0, 3);
    response->len = ((len_bytes + 1)*8) - 2;

    /* the most significant 6 bits of the 32 bits crc is the final 6 bits crc. */
    response->crc_value = (((uint32_t)_mm_extract_epi32(xmm0, 0)) >> 26);
    _mm256_zeroupper();
}



void bblib_lte_crc6_check_sse(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    if (request->data == NULL) {
        printf("bblib_lte_crc6_check input / output address error \n");
        response->check_passed = false;
        return;
    }
    // len is passed in as bits, so turn into bytes
    uint32_t len_bytes = request->len / 8;

    /* CRC in the original sequence */
    uint32_t CRC_orig = 0;

    CRC_orig = ((request->data[len_bytes])&0x000000FF)>>2;
    bblib_lte_crc6_gen_sse(request, response);

    if (response->crc_value != CRC_orig)
        response->check_passed = false;
    else
        response->check_passed = true;
}



#else
void bblib_lte_crc24a_gen_sse(struct bblib_crc_request *request, struct bblib_crc_response *response){
    printf("bblib_crc requires at least SSE4.2 ISA support to run\n");
    exit(-1);
}
void bblib_lte_crc24b_gen_sse(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    printf("bblib_crc requires at least SSE4.2 ISA support to run\n");
    exit(-1);
}
void bblib_lte_crc16_gen_sse(struct bblib_crc_request *request, struct bblib_crc_response *response){
    printf("bblib_crc requires at least SSE4.2 ISA support to run\n");
    exit(-1);
}
void bblib_lte_crc11_gen_sse(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    printf("bblib_crc requires at least SSE4.2 ISA support to run\n");
    exit(-1);
}
void bblib_lte_crc6_gen_sse(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    printf("bblib_crc requires at least SSE4.2 ISA support to run\n");
    exit(-1);
}
void bblib_lte_crc24a_check_sse(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    printf("bblib_crc requires at least SSE4.2 ISA support to run\n");
    exit(-1);
}
void bblib_lte_crc24b_check_sse(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    printf("bblib_crc requires at least SSE4.2 ISA support to run\n");
    exit(-1);
}
void bblib_lte_crc16_check_sse(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    printf("bblib_crc requires at least SSE4.2 ISA support to run\n");
    exit(-1);
}
void bblib_lte_crc11_check_sse(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    printf("bblib_crc requires at least SSE4.2 ISA support to run\n");
    exit(-1);
}
void bblib_lte_crc6_check_sse(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    printf("bblib_crc requires at least SSE4.2 ISA support to run\n");
    exit(-1);
}
#endif
