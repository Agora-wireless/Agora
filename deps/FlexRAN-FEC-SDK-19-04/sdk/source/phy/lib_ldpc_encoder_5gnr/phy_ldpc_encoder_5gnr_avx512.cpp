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
 *  @file   phy_ldpc_encoder_5gnr_avx512.cpp
 *  @brief  AVX512 code for 5GNR LDPC Encoder functions.
 */

#include <stdlib.h>
#include <string.h>
#include <ipp.h>
#include <ipps.h>
#include <immintrin.h>  /* AVX512 */
#include "phy_ldpc_encoder_5gnr.h"
#include "ldpc_encoder_cycshift.h"
#include "phy_ldpc_encoder_5gnr_internal.h"

#include "common_typedef_sdk.h"

/**
 *  @brief Adapter function for scattering/gathering data when Zs > 256 (ie. one way)
 *  @param [in] data input
 *  @param [out] data output
 *  @param [in] lifting factor
 *  @param [in] Size of the data
 *  @param [in] direction for scatter/gather
 *  @return void
**/
void adapter_from288to384(int8_t **pBuff0, int8_t *pBuff1, uint16_t zcSize, uint32_t cbLen, int8_t direct)
{
    int8_t *pBuff0Offset,*pBuff1Offset;
    __mmask64 mask0;
    int16_t byteNum;
    __m512i x0;

    pBuff0Offset = pBuff0[0];
    pBuff1Offset = pBuff1;
    byteNum = (zcSize>>3);
    mask0 = ((__mmask64)1<<(byteNum)) - 1;
    if (1 == direct) {
        for(int16_t i = 0; i < cbLen >> 3; i = i + byteNum) {
            x0 = _mm512_maskz_loadu_epi8 (mask0, pBuff0Offset + i);
            _mm512_storeu_si512 (pBuff1Offset, x0);
            pBuff1Offset = pBuff1Offset + PROC_BYTES;
        }
    } else {
        for(int16_t i=0; i<cbLen>>3; i=i+byteNum) {
            x0 = _mm512_loadu_si512 (pBuff1Offset);
            _mm512_mask_storeu_epi8  (pBuff0Offset, mask0, x0);
            pBuff1Offset = pBuff1Offset + PROC_BYTES;
            pBuff0Offset = pBuff0Offset + byteNum;
        }
    }
}

#define BITMASKU8(x) ((1U << (x)) - 1)
#define MIN(a,b) (((a)<(b))?(a):(b))

// Unoptimized implementationt but functional for any bit alignment
void scatter_slow(uint8_t* dst, const uint8_t* src, unsigned num_bits, uint8_t src_offbits)
{
    //Process byte by byte
    while(num_bits != 0) {
        unsigned num_bits_inB = MIN(8, num_bits);
        uint8_t newB;
        if (src_offbits == 0)
            newB = src[0];
        else
            newB = ((src[0] & 0xFF) >> src_offbits) | ((src[1] & 0xFF) << (8 - src_offbits));
        dst[0] = newB & BITMASKU8(num_bits_inB);
        num_bits -= num_bits_inB;
        dst++;
        src++;
    }
}

// Unoptimized implementationt but functional for any bit alignment
void gather_slow(uint8_t* dst, const uint8_t* src, unsigned num_bits, uint8_t dst_offbits)
{
    //Process byte by byte
    bool firstByte= true;
    while(num_bits != 0) {
        unsigned num_bits_inB = MIN(8, num_bits);
        uint8_t newB;
        if (dst_offbits == 0) {
            // simple copy
            newB   = src[0] & BITMASKU8(num_bits_inB);
            src++;
        } else {
            if (firstByte) {
                newB = (dst[0] & BITMASKU8(dst_offbits)) | (src[0] & 0xFF) << dst_offbits;
                num_bits_inB = 8 - dst_offbits;
                firstByte = false;
            } else {
                newB = ((src[0] & 0xFF) >> (8 - dst_offbits) | (src[1] & 0xFF) << dst_offbits) & BITMASKU8(num_bits_inB);
                src++;
            }
        }
       dst[0] = newB;
       num_bits -= num_bits_inB;
       dst++;
    }
}

/**
 *  @brief Plain C and slow implementation of adapter function for scattering/gathering data
 *  @param [in] data input
 *  @param [out] data output
 *  @param [in] lifting factor
 *  @param [in] Size of the data
 *  @param [in] direction for scatter/gather
 *  @return void
**/
void adapter_lowSpeed(int8_t **pBuff0, int8_t *pBuff1, uint16_t zcSize, uint32_t cbLen, int8_t direct)
{
    int8_t *pBuff0Offset,*pBuff1Offset;
    pBuff0Offset = pBuff0[0];
    pBuff1Offset = pBuff1;
    uint8_t dst_offbits = 0, src_offbits = 0;
    if (1 == direct) {
        for(int16_t i = 0; i < cbLen / zcSize; i++) {
            scatter_slow((uint8_t*)pBuff1Offset, (uint8_t*)pBuff0Offset, zcSize, src_offbits);
            int32_t * pBytes0 = (int32_t *) pBuff0Offset;
            int32_t * pBytes1 = (int32_t *) pBuff1Offset;
            uint8_t byteOffset = (src_offbits + zcSize) >> 3;
            src_offbits = (src_offbits + zcSize) - (byteOffset << 3);
            pBuff0Offset = pBuff0Offset + byteOffset;
            pBuff1Offset = pBuff1Offset + PROC_BYTES;
        }
    } else {
        for(int16_t i = 0; i < cbLen / zcSize; i++) {
            gather_slow((uint8_t*)pBuff0Offset, (uint8_t*)pBuff1Offset, zcSize, dst_offbits);
            int32_t * pBytes0 = (int32_t *) pBuff0Offset;
            int32_t * pBytes1 = (int32_t *) pBuff1Offset;
            uint8_t byteOffset = (dst_offbits + zcSize) >> 3;
            dst_offbits = (dst_offbits + zcSize) - (byteOffset << 3);
            pBuff0Offset = pBuff0Offset + byteOffset;
            pBuff1Offset = pBuff1Offset + PROC_BYTES;
        }
    }
}

/**
 *  @brief adapter function based 
 *  @param [in] input stream
 *  @param [out] output stream
 *  @param [in] input stream
 *  @param [in] input stream
 *  @param [in] input stream
 *  @return actual function
**/
void adapter_2ways_from144to256(int8_t **pbuff0, int8_t *pbuff1, uint16_t zcSize, uint32_t cbLen, int8_t direct)//2ways
{
    int8_t *pBuff0Offset[WAYS_144to256],*pBuff1Offset;
    __mmask64 mask0;
    __mmask32 mask1,mask2;
    uint16_t zcSizeMul2,shortNum,zc2WayByteNum;
    __m512i x0,x1,x2,x3;
    __m512i swapIdx0,swapIdx1;
    int16_t adapterPermuteTableShort[64] = {
        0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,
        32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63
    };

    pBuff0Offset[0] = pbuff0[0];
    pBuff0Offset[1] = pbuff0[1];
    pBuff1Offset = pbuff1;

    // Prevent buffer overflow
    if (zcSize > ZC_MAX)
        return;

    shortNum = (zcSize>>4);
    zc2WayByteNum = (zcSize>>2);
    mask0 = ((__mmask64)1<<(zc2WayByteNum)) - 1;
    if (64 == zc2WayByteNum)
        {mask0 = 0xffffffffffffffff;}
    zcSizeMul2 = (zcSize<<1);

    mask1 = ((__mmask32)1<<shortNum) - 1;
    if (1== direct) {
        mask2 = mask1<<16|mask1;
        swapIdx0 = _mm512_loadu_si512 ((void const*)(adapterPermuteTableShort));//x0's first part
        swapIdx0 = _mm512_mask_loadu_epi16 (swapIdx0, mask1 << 16, adapterPermuteTableShort+16);//x1's first part
        swapIdx1 = _mm512_loadu_si512 ((void const*)(adapterPermuteTableShort+shortNum));//x0's seconde part
        swapIdx1 = _mm512_mask_loadu_epi16 (swapIdx1, mask1 << 16, 
            adapterPermuteTableShort+16+shortNum);//x1's first part
        for(int16_t i=0; i<cbLen; i=i+zcSizeMul2) {
            x0 = _mm512_maskz_loadu_epi8 (mask0, pBuff0Offset[0]);
            x1 = _mm512_maskz_loadu_epi8 (mask0, pBuff0Offset[1]);
            pBuff0Offset[0]= pBuff0Offset[0] + zc2WayByteNum;
            pBuff0Offset[1]= pBuff0Offset[1] + zc2WayByteNum;
            x2 = _mm512_mask_permutex2var_epi16 (x0, mask2, swapIdx0, x1);
            x3 = _mm512_mask_permutex2var_epi16 (x0, mask2, swapIdx1, x1);
            _mm512_storeu_si512 (pBuff1Offset, x2);
            pBuff1Offset = pBuff1Offset + PROC_BYTES;
            _mm512_storeu_si512 (pBuff1Offset, x3);
            pBuff1Offset = pBuff1Offset + PROC_BYTES;
        }
    } else {
        mask2 = (mask1<<(shortNum));
        swapIdx0 = _mm512_loadu_si512 ((void const*)(adapterPermuteTableShort));
        swapIdx0 = _mm512_mask_loadu_epi16 (swapIdx0, mask2,
            ((void const*)(adapterPermuteTableShort + 32-shortNum)));
        swapIdx1 = _mm512_loadu_si512 ((void const*)(adapterPermuteTableShort+16));
        swapIdx1 = _mm512_mask_loadu_epi16 (swapIdx1, mask2,
            ((void const*)(adapterPermuteTableShort + 48-shortNum)));

        for(int16_t i=0; i<=cbLen; i=i+zcSize) {
            x0 = _mm512_loadu_si512 (pBuff1Offset);
            pBuff1Offset = pBuff1Offset + PROC_BYTES;
            x1 = _mm512_loadu_si512 (pBuff1Offset);
            pBuff1Offset = pBuff1Offset + PROC_BYTES;
            x2 = _mm512_mask_permutex2var_epi16 (x0, mask2|mask1, swapIdx0, x1);
            x3 = _mm512_mask_permutex2var_epi16 (x0, mask2|mask1, swapIdx1, x1);
            _mm512_mask_storeu_epi8  (pBuff0Offset[0],mask0, x2);
            _mm512_mask_storeu_epi8  (pBuff0Offset[1],mask0, x3);
            pBuff0Offset[0]= pBuff0Offset[0] + zc2WayByteNum;
            pBuff0Offset[1]= pBuff0Offset[1] + zc2WayByteNum;
        }
    }
}

/**
 *  @brief Select adapter function based 
 *  @param [in] Zs lifting factor size
 *  @param [in] number of code blocks provided
 *  @return actual function
**/
LDPC_ADAPTER_P ldpc_select_adapter_func(uint16_t zcSize, uint8_t num_ways)
{
    if (zcSize < 64 || zcSize == 72 || zcSize == 88 || zcSize == 104 || zcSize == 120) 
        return adapter_lowSpeed;
    else if (zcSize >= 288 || num_ways == 1)
        return adapter_from288to384;
    else
        return adapter_2ways_from144to256;
}

/**
 *  @brief Actual Encoding for LDPC with BG1 .
 *  @param [in] input data after adapter
 *  @param [out] output data before adapter
 *  @param [in] Matrix const LUTs structure
 *  @param [in] zcSize Lifting factor size
 *  @return void
**/
void ldpc_encoder_bg1(int8_t *pDataIn, int8_t *pDataOut, const int16_t *pMatrixNumPerCol,
        const int16_t *pAddr, const int16_t *pShiftMatrix, int16_t zcSize, uint8_t i_LS)
{
    const int16_t *pTempAddr,  *pTempMatrix;
    int8_t *pTempIn,*pTempOut;
    int16_t addrOffset = 0;
    int32_t i = 0;
    __mmask64 zcLSMask;
    __m512i x1,x2,x3,x4,x5,x6,x7,x8,x9;
    CYCLE_BIT_LEFT_SHIFT cycle_bit_left_shift_p = ldpc_select_left_shift_func(zcSize);

    zcLSMask = (1<<(zcSize>>3))-1;
    for(int32_t j=0; j<BG1_ROW_TOTAL; j++)
        _mm512_storeu_si512 (pDataOut+PROC_BYTES*j, _mm512_set1_epi8 (0));
    pTempAddr = pAddr;
    pTempMatrix = pShiftMatrix;
    pTempIn = pDataIn;
    pTempOut = pDataOut;

    x1 = _mm512_loadu_si512 (pTempIn + i * PROC_BYTES);
    for(int32_t j = 0; j < *(pMatrixNumPerCol + i); j++) {
        x2 = cycle_bit_left_shift_p(x1, *pTempMatrix++, zcSize);
        _mm512_storeu_si512 (pTempOut + (*pTempAddr++), x2);
    }
    i = 1;
    for(; i < BG1_COL_INF_NUM; i++) {
        x1 = _mm512_loadu_si512 (pTempIn + i * PROC_BYTES);
        for(int32_t j = 0; j<*(pMatrixNumPerCol + i); j++) {
            addrOffset = *pTempAddr++;
            x2 = cycle_bit_left_shift_p(x1, *pTempMatrix++, zcSize);
            x3 = _mm512_loadu_si512  (pTempOut + addrOffset);
            x4 = _mm512_xor_epi32 (x2, x3);
            _mm512_storeu_si512 (pTempOut + addrOffset, x4);
        }
    }
    // Row Transform to resolve the small 4x4 parity matrix
    x1 = _mm512_loadu_si512 (pTempOut);
    x2 = _mm512_loadu_si512 (pTempOut + PROC_BYTES);
    x3 = _mm512_loadu_si512 (pTempOut + (PROC_BYTES * 2));
    x4 = _mm512_loadu_si512 ( pTempOut + PROC_BYTES * 3);
    //first 384
    x5 = _mm512_xor_epi32 (x1, x2);
    x5 = _mm512_xor_epi32 (x5, x3);
    x5 = _mm512_xor_epi32 (x5, x4);
    // Special case for circulant
    if (6 == i_LS) {
        x5 = cycle_bit_left_shift_p(x5, 103, zcSize);
        _mm512_storeu_si512 (pDataOut, x5);
    } else
        _mm512_storeu_si512 (pDataOut, x5);
    //second 384
    if (6 == i_LS)
        x6 = x5;
    else
        x6 = cycle_bit_left_shift_p(x5, 1, zcSize);

    x7 = _mm512_xor_epi32 (x1, x6);
    _mm512_storeu_si512 (pDataOut + PROC_BYTES, x7);
    //fourth 384
    x8 = _mm512_xor_epi32 (x4, x6);
    _mm512_storeu_si512 (pDataOut + PROC_BYTES * 3, x8);
    //third 384
    x9 = _mm512_xor_epi32 (x3, x8);
    _mm512_storeu_si512 (pDataOut + (PROC_BYTES * 2), x9);

    // Rest of parity based on identity matrix
    for(; i < 4 + BG1_COL_INF_NUM; i++) {
        x1 = _mm512_loadu_si512 (pDataOut + (i - BG1_COL_INF_NUM) * PROC_BYTES);
        for(int32_t j = 0; j < *(pMatrixNumPerCol + i); j++) {
            addrOffset = *pTempAddr++;
            x2 = cycle_bit_left_shift_p(x1, *pTempMatrix++, zcSize);
            x3 = _mm512_loadu_si512 (pTempOut + addrOffset);
            x4 = _mm512_xor_epi32 (x2, x3);
            _mm512_storeu_si512 (pTempOut + addrOffset, x4);
        }
    }
}

/**
 *  @brief Actual Encoding for LDPC with BG2 .
 *  @param [in] input data after adapter
 *  @param [out] output data before adapter
 *  @param [in] Matrix const LUTs structure
 *  @param [in] zcSize Lifting factor size
 *  @return void
**/
void ldpc_encoder_bg2(int8_t *pDataIn, int8_t *pDataOut, const int16_t *pMatrixNumPerCol,
        const int16_t *pAddr, const int16_t *pShiftMatrix, int16_t zcSize, uint8_t i_LS)
{
    const int16_t *pTempAddr,  *pTempMatrix;
    int8_t *pTempIn,*pTempOut;
    int16_t addrOffset = 0;
    int32_t i = 0;
    __mmask64 zcLSMask;
    __m512i x1,x2,x3,x4,x5,x6,x7,x8,x9;
    CYCLE_BIT_LEFT_SHIFT cycle_bit_left_shift_p = ldpc_select_left_shift_func(zcSize);

    zcLSMask = (1 << (zcSize >> 3)) - 1;
    for(int32_t j = 0; j < BG2_ROW_TOTAL; j++)
        _mm512_storeu_si512 (pDataOut + PROC_BYTES * j, _mm512_set1_epi8(0));
    pTempAddr = pAddr;
    pTempMatrix = pShiftMatrix;
    pTempIn = pDataIn;
    pTempOut = pDataOut;

    x1 = _mm512_loadu_si512 (pTempIn + i * PROC_BYTES);
    for(int32_t j = 0; j<*(pMatrixNumPerCol + i); j++) {
        x2 = cycle_bit_left_shift_p(x1, *pTempMatrix++, zcSize);
        _mm512_storeu_si512 (pTempOut + (*pTempAddr++), x2);
    }
    i = 1; 
    for(; i<BG2_COL_INF_NUM; i++) {
        x1 = _mm512_loadu_si512 (pTempIn + i * PROC_BYTES);
        for(int32_t j = 0; j<*(pMatrixNumPerCol + i); j++) {
            addrOffset = *pTempAddr++;
            x2 = cycle_bit_left_shift_p(x1, *pTempMatrix++, zcSize);
            x3 = _mm512_loadu_si512  (pTempOut + addrOffset);
            x4 = _mm512_xor_epi32 (x2, x3);
            _mm512_storeu_si512 (pTempOut + addrOffset, x4);
        }
    }

    // Row Transform to resolve the small 4x4 parity matrix
    x1 = _mm512_loadu_si512 (pTempOut);
    x2 = _mm512_loadu_si512 (pTempOut + PROC_BYTES);
    x3 = _mm512_loadu_si512 (pTempOut + (PROC_BYTES * 2));
    x4 = _mm512_loadu_si512 ( pTempOut + PROC_BYTES * 3);

    //first 384
    x5 = _mm512_xor_epi32 (x1, x2);
    x5 = _mm512_xor_epi32 (x5, x3);
    x5 = _mm512_xor_epi32 (x5, x4);

    // Special case for the circulant
    if ((i_LS == 3) || (i_LS == 7))
        _mm512_storeu_si512 (pDataOut, x5);
    else {
        x5 = cycle_bit_left_shift_p(x5, (zcSize-1),zcSize);
        _mm512_storeu_si512 (pDataOut, x5);
    }

    //second 384
    if ((i_LS == 3) || (i_LS == 7))
        x6 = cycle_bit_left_shift_p(x5, 1, zcSize);
    else
        x6 = x5;
    x7 = _mm512_xor_epi32 (x1, x6);
    _mm512_storeu_si512 (pDataOut + PROC_BYTES, x7);

    //third 384 - c2(x2)+w2(x7)=w3(x8)
    x8 = _mm512_xor_epi32 (x2, x7);
    _mm512_storeu_si512 (pDataOut + (PROC_BYTES * 2), x8);
    //fourth 384 - c4(x4)+w1_0(x6)=w4(x9)
    x9 = _mm512_xor_epi32 (x4, x6);
    _mm512_storeu_si512 (pDataOut + PROC_BYTES * 3, x9);

    // Rest of parity based on identity matrix
    for(; i < 4 + BG2_COL_INF_NUM; i++) {
        x1 = _mm512_loadu_si512 (pDataOut + (i - BG2_COL_INF_NUM) * PROC_BYTES);
        for(int32_t j = 0; j<*(pMatrixNumPerCol + i); j++) {
            addrOffset = *pTempAddr++;
            x2 = cycle_bit_left_shift_p(x1, *pTempMatrix++, zcSize);
            x3 = _mm512_loadu_si512 (pTempOut + addrOffset);
            x4 = _mm512_xor_epi32 (x2, x3);
            _mm512_storeu_si512 (pTempOut + addrOffset, x4);
        }
    }
}




//-------------------------------------------------------------------------------------------
/**
 *  @brief Encoding for LDPC in 5GNR.
 *  @param [in] request Structure containing configuration information and input data.
 *  @param [out] response Structure containing kernel outputs.
 *  @return Success: return 0, else: return -1.
**/
int32_t bblib_ldpc_encoder_5gnr_avx512(struct bblib_ldpc_encoder_5gnr_request *request, struct bblib_ldpc_encoder_5gnr_response *response)
{
    if ( (request->numberCodeblocks > 2) || ((request->numberCodeblocks == 2) && ( (request->Zc > 256) || (request->Zc <= 128) )) ) {
        printf("bblib_ldpc_encoder_5gnr_avx512 Number of code blocks invalid \n");
        return(-1);
    }
    /* internal processing buffer allocated internally */
    __declspec (align(64)) int8_t internalBuffer0[BG1_ROW_TOTAL * PROC_BYTES];
    __declspec (align(64)) int8_t internalBuffer1[BG1_ROW_TOTAL * PROC_BYTES];
    const int16_t *pShiftMatrix;
    LDPC_ADAPTER_P ldpc_adapter_func;
    uint32_t cbEncLen, cbLen;

    /* Find i_Ls based on lifting factor size as defined in 38.212 Table 5.3.2-1*/
    uint8_t i_LS;
    if ((request->Zc % 15) == 0)
        i_LS = 7;
    else if ((request->Zc % 13) == 0)
        i_LS = 6;
    else if ((request->Zc % 11) == 0)
        i_LS = 5;
    else if ((request->Zc % 9) == 0)
        i_LS = 4;
    else if ((request->Zc % 7) == 0)
        i_LS = 3;
    else if ((request->Zc % 5) == 0)
        i_LS = 2;
    else if ((request->Zc % 3) == 0)
        i_LS = 1;
    else 
        i_LS = 0;
    if (request->baseGraph == 1) {
        pShiftMatrix = Bg1HShiftMatrix + i_LS * BG1_NONZERO_NUM;
        cbLen = BG1_COL_INF_NUM * request->Zc;
    } else {
        pShiftMatrix = Bg2HShiftMatrix + i_LS * BG2_NONZERO_NUM;
        cbLen = BG2_COL_INF_NUM * request->Zc;
    }

    cbEncLen = request->nRows * request->Zc;
    /* Adapter function to scatter the data into internal buffer as 64B chunks */
    ldpc_adapter_func = ldpc_select_adapter_func(request->Zc, request->numberCodeblocks);
    ldpc_adapter_func(request->input, internalBuffer0, request->Zc, cbLen, 1);
    /* Actual processing */
    if (request->baseGraph == 1)
        ldpc_encoder_bg1(internalBuffer0, internalBuffer1, Bg1MatrixNumPerCol, Bg1Address, pShiftMatrix, (int16_t) request->Zc, i_LS);
    else
        ldpc_encoder_bg2(internalBuffer0, internalBuffer1, Bg2MatrixNumPerCol, Bg2Address, pShiftMatrix, (int16_t) request->Zc, i_LS);
    /* Adapter function to gather back the data */
    ldpc_adapter_func(response->output, internalBuffer1, request->Zc, cbEncLen, 0);

    return 0;
}
