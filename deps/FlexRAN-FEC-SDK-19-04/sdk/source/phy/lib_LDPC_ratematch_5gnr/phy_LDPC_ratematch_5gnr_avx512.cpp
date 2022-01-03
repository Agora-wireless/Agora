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
 *  @file   phy_LDPC_ratematch_5gnr_avx512.cpp
 *  @brief  AVX512 code for 5GNR Rate Matching functions.
 */

#include <stdlib.h>
#include <string.h>
#include <ipp.h>
#include <ipps.h>
#include <immintrin.h>  /* AVX512 */
#include "phy_LDPC_ratematch_5gnr.h"
#include "common_typedef_sdk.h"

constexpr auto MAX_LENGTH=8448;
constexpr __mmask64 tailMask[] = {
0x0, 0x3, 0xF, 0x3F, 0xFF, 0x3FF, 0xFFF, 0x3FFF,
0xFFFF, 0x3FFFF, 0xFFFFF, 0x3FFFFF, 0xFFFFFF, 0x3FFFFFF, 0xFFFFFFF, 0x3FFFFFFF,
0xFFFFFFFF, 0x3FFFFFFFF, 0xFFFFFFFFF, 0x3FFFFFFFFF, 0xFFFFFFFFFF, 0x3FFFFFFFFFF, 0xFFFFFFFFFFF, 0x3FFFFFFFFFFF,
0xFFFFFFFFFFFF, 0x3FFFFFFFFFFFF, 0xFFFFFFFFFFFFF, 0x3FFFFFFFFFFFFF, 0xFFFFFFFFFFFFFF, 0x3FFFFFFFFFFFFFF, 0xFFFFFFFFFFFFFFF, 0x3FFFFFFFFFFFFFFF};

//! @{
/*! \brief bit select function for 5GNR LDPC rate matching.
    \param [in] request Structure containing configuration information and input data.
    \param [out] e Temporary pointer to the output after bit slection.
*/
inline void bitSelect(const struct bblib_LDPC_ratematch_5gnr_request *request, uint8_t *e)
{
    int32_t cb = request->Ncb;
    int32_t zc = request->Zc;
    int32_t E = request->E;
    int32_t Q = request->Qm;
    int32_t rv = request->rvidx;
    int32_t graph = request->baseGraph;
    int32_t ni = request->nullIndex;
    int32_t nl = request->nLen;
    uint8_t *in = request->input;

    int32_t k0;
    if (graph == 1)
    {
        switch (rv)
        {
            case 0:
                k0 = 0;
                break;
            case 1:
                k0 = (17 * cb) / (66 * zc) * zc;
                break;
            case 2:
                k0 = (33 * cb) / (66 * zc) * zc;
                break;
            case 3:
                k0 = (56 * cb) / (66 * zc) * zc;
                break;
            default:
                printf("Wrong parameter for rvidx. It should be 0/1/2/3\n");
                exit(-1);
                break;
        }
    }
    else if (graph == 2)
    {
        switch (rv)
        {
            case 0:
                k0 = 0;
                break;
            case 1:
                k0 = (13 * cb) / (50 * zc) * zc;
                break;
            case 2:
                k0 = (25 * cb) / (50 * zc) * zc;
                break;
            case 3:
                k0 = (43 * cb) / (50 * zc) * zc;
                break;
            default:
                printf("Wrong parameter for rvidx. It should be 0/1/2/3\n");
                exit(-1);
                break;
        }
    }
    else
    {
        printf("Wrong parameter for Base Graph. It should be 1/2\n");
        exit(-1);
    }

    int32_t startBit, srcOffBit, desOffBit;
    int32_t srcByte, srcBits, desByte, desBits;
    int32_t bitLeft = E/Q;
    startBit = ( (k0<ni) || (k0>ni+nl) ) ? k0 : (ni+nl);

    for (int i = 0; i < Q; i++) {
        srcOffBit = startBit;
        desOffBit = 8 * i * ((bitLeft - 1) / 8 + 1 );

        srcByte = srcOffBit >> 3;
        srcBits = srcOffBit & 0x7;
        desByte = desOffBit >> 3;
        desBits = desOffBit & 0x7;

        /* If data between start bit and null bit is enough for bit select, copy continuous bits to output */
        if ((startBit < ni) && (startBit + bitLeft <= ni)) {
            ippsCopyBE_1u( (Ipp8u*)(in + srcByte), srcBits, (Ipp8u*)(e + desByte), desBits, bitLeft );
            srcOffBit += bitLeft;
        }
        /* If data between start bit and null bit is not enough for bit select, copy all bits between them and copy remain bits from end of null bits */
        else if ((startBit < ni) && (startBit + bitLeft + nl <= cb))
        {
            ippsCopyBE_1u( (Ipp8u*)(in+srcByte), srcBits, (Ipp8u*)(e+desByte), desBits, ni-startBit );

            desOffBit += ni - startBit;
            desByte = desOffBit >> 3;
            desBits = desOffBit & 0x7;
            srcOffBit = ni + nl;
            srcByte = srcOffBit >> 3;
            srcBits = srcOffBit & 0x7;

            ippsCopyBE_1u( (Ipp8u*)(in+srcByte), srcBits, (Ipp8u*)(e+desByte), desBits, bitLeft-ni+startBit );
            srcOffBit += bitLeft-ni+startBit;
        }
        /* If data between start bit and null bit is not enough for bit select, copy all bits between them and copy remain bits from end of null bits
           If we reach the end of input, get back to the beginning and copy all except null bits until we get enough bits */
        else if ( startBit<ni )
        {
            ippsCopyBE_1u( (Ipp8u*)(in+srcByte), srcBits, (Ipp8u*)(e+desByte), desBits, ni-startBit );

            desOffBit += ni - startBit;
            desByte = desOffBit >> 3;
            desBits = desOffBit & 0x7;
            srcOffBit = ni+nl;
            srcByte = srcOffBit >> 3;
            srcBits = srcOffBit & 0x7;

            ippsCopyBE_1u( (Ipp8u*)(in+srcByte), srcBits, (Ipp8u*)(e+desByte), desBits, cb-ni-nl );

            desOffBit += cb - ni - nl;
            desByte = desOffBit >> 3;
            desBits = desOffBit & 0x7;
            bitLeft -= cb - startBit - nl;

            while (bitLeft > 0)
            {
                if ( bitLeft>ni )
                {
                    ippsCopyBE_1u( (Ipp8u*)in, 0, (Ipp8u*)(e+desByte), desBits, ni );
                    bitLeft -= ni;
                    desOffBit += ni;
                    desByte = desOffBit >> 3;
                    desBits = desOffBit & 0x7;
                }
                else
                {
                    ippsCopyBE_1u( (Ipp8u*)in, 0, (Ipp8u*)(e+desByte), desBits, bitLeft );
                    srcOffBit = bitLeft;
                    bitLeft = 0;
                    break;
                }

                srcOffBit = ni + nl;
                srcByte = srcOffBit >> 3;
                srcBits = srcOffBit & 0x7;
                if ( bitLeft>(cb-ni-nl) )
                {
                    ippsCopyBE_1u( (Ipp8u*)(in+srcByte), srcBits, (Ipp8u*)(e+desByte), desBits, cb-ni-nl );
                    bitLeft -= cb - ni - nl;
                    desOffBit += cb - ni - nl;
                    desByte = desOffBit >> 3;
                    desBits = desOffBit & 0x7;
                }
                else
                {
                    ippsCopyBE_1u( (Ipp8u*)(in+srcByte), srcBits, (Ipp8u*)(e+desByte), desBits, bitLeft );
                    srcOffBit += bitLeft;
                    bitLeft = 0;
                    break;
                }
            }
        }
        /* If we start at the end of null bits, and the remaining bits is enough */
        else if (startBit+bitLeft<=cb)
        {
            ippsCopyBE_1u( (Ipp8u*)(in+srcByte), srcBits, (Ipp8u*)(e+desByte), desBits, bitLeft );
            srcOffBit += bitLeft;
        }
        /* If we start at the end of null bits, and the remaining bits is not enough, we get back to the beginning and copy all except null bits until we get enough bits */
        else
        {
            ippsCopyBE_1u( (Ipp8u*)(in+srcByte), srcBits, (Ipp8u*)(e+desByte), desBits, cb-startBit );
            bitLeft -= cb-startBit;
            desOffBit += cb-startBit;
            desByte = desOffBit >> 3;
            desBits = desOffBit & 0x7;

            while (bitLeft > 0)
            {
                if ( bitLeft>ni )
                {
                    ippsCopyBE_1u( (Ipp8u*)in, 0, (Ipp8u*)(e+desByte), desBits, ni );
                    bitLeft -= ni;
                    desOffBit += ni;
                    desByte = desOffBit >> 3;
                    desBits = desOffBit & 0x7;
                }
                else
                {
                    ippsCopyBE_1u( (Ipp8u*)in, 0, (Ipp8u*)(e+desByte), desBits, bitLeft );
                    srcOffBit = bitLeft;
                    bitLeft = 0;
                    break;
                }

                srcOffBit = ni + nl;
                srcByte = srcOffBit >> 3;
                srcBits = srcOffBit & 0x7;
                if ( bitLeft>(cb-ni-nl) )
                {
                    ippsCopyBE_1u( (Ipp8u*)(in+srcByte), srcBits, (Ipp8u*)(e+desByte), desBits, cb-ni-nl );
                    bitLeft -= cb - ni - nl;
                    desOffBit += cb - ni - nl;
                    desByte = desOffBit >> 3;
                    desBits = desOffBit & 0x7;
                }
                else
                {
                    ippsCopyBE_1u( (Ipp8u*)(in+srcByte), srcBits, (Ipp8u*)(e+desByte), desBits, bitLeft );
                    srcOffBit += bitLeft;
                    bitLeft = 0;
                    break;
                }
            }
        }
        bitLeft = E/Q;
        /* In case where are just at the boundary, skip the filler bits */
        if (srcOffBit == ni)
            srcOffBit += nl;
        startBit = srcOffBit;
    }
}

//! @{
/*! \brief bit interleave function for 5GNR LDPC rate matching.
    \param [in] request Structure containing configuration information.
    \param [out] response Structure containing output pointer.
    \param [in] e Temporary pointer to the output after bit slection, as input of bit interleave.
*/
inline void bitInterleave_avx512(const struct bblib_LDPC_ratematch_5gnr_request *request, struct bblib_LDPC_ratematch_5gnr_response *response, uint8_t *e)
{
    int32_t Q = request->Qm;
    int32_t len = request->E;
    int32_t jMax = len / Q;
    uint8_t *output = response->output;
    unsigned char *tmp[8];
    int bitOffset, byteOffset, bitTail, byteTail;
    int lenInByte = jMax>>3;
    __m512i inter = _mm512_setzero_si512();
    __m512i res;
    __mmask64 val1, val2, val3, val4;
    __mmask64 left;
    unsigned __int64 in[8];
    unsigned __int64 out[8];

    for (int i = 0;i < Q;i++)
    {
        tmp[i] = e + i * ( ((len/Q-1)>>3)+1 );
    }

    __m512i tmp0_QPSK, tmp1_QPSK;
    __m512i vidxIn1_QPSK = _mm512_set_epi16( 31, 15, 30, 14,
                                             29, 13, 28, 12,
                                             27, 11, 26, 10,
                                             25, 9, 24, 8,
                                             23, 7, 22, 6,
                                             21, 5, 20, 4,
                                             19, 3, 18, 2,
                                             17, 1, 16, 0);
    __m512i vidxIn2_QPSK = _mm512_set_epi8(15, 13, 14, 12, 11, 9, 10, 8,
                                           7, 5, 6, 4, 3, 1, 2, 0,
                                           15, 13, 14, 12, 11, 9, 10, 8,
                                           7, 5, 6, 4, 3, 1, 2, 0,
                                           15, 13, 14, 12, 11, 9, 10, 8,
                                           7, 5, 6, 4, 3, 1, 2, 0,
                                           15, 13, 14, 12, 11, 9, 10, 8,
                                           7, 5, 6, 4, 3, 1, 2, 0);
    __m512i vidxOut1_QPSK = _mm512_set_epi16(31, 27, 23, 19, 15, 11, 7, 3,
                                             30, 26, 22, 18, 14, 10, 6, 2,
                                             29, 25, 21, 17, 13, 9, 5, 1,
                                             28, 24, 20, 16, 12, 8, 4, 0);
    __m512i vidxOut2_QPSK = _mm512_set_epi8(15, 13, 11, 9, 7, 5, 3, 1,
                                            14, 12, 10, 8, 6, 4, 2, 0,
                                            15, 13, 11, 9, 7, 5, 3, 1,
                                            14, 12, 10, 8, 6, 4, 2, 0,
                                            15, 13, 11, 9, 7, 5, 3, 1,
                                            14, 12, 10, 8, 6, 4, 2, 0,
                                            15, 13, 11, 9, 7, 5, 3, 1,
                                            14, 12, 10, 8, 6, 4, 2, 0);

    __m512i tmp0_16QAM, tmp1_16QAM, tmp2_16QAM, tmp3_16QAM;
    __m512i vidxIn1_16QAM = _mm512_set_epi16( 31, 23, 15, 7,
                                              30, 22, 14, 6,
                                              29, 21, 13, 5,
                                              28, 20, 12, 4,
                                              27, 19, 11, 3,
                                              26, 18, 10, 2,
                                              25, 17, 9, 1,
                                              24, 16, 8, 0);
    __m512i vidxIn2_16QAM = _mm512_set_epi8(15, 13, 11, 9, 14, 12, 10, 8,
                                            7, 5, 3, 1, 6, 4, 2, 0,
                                            15, 13, 11, 9, 14, 12, 10, 8,
                                            7, 5, 3, 1, 6, 4, 2, 0,
                                            15, 13, 11, 9, 14, 12, 10, 8,
                                            7, 5, 3, 1, 6, 4, 2, 0,
                                            15, 13, 11, 9, 14, 12, 10, 8,
                                            7, 5, 3, 1, 6, 4, 2, 0);
    __m512i vidxOut1_16QAM = _mm512_set_epi16(31, 27, 23, 19, 15, 11, 7, 3,
                                              30, 26, 22, 18, 14, 10, 6, 2,
                                              29, 25, 21, 17, 13, 9, 5, 1,
                                              28, 24, 20, 16, 12, 8, 4, 0);
    __m512i vidxOut2_16QAM = _mm512_set_epi8(15, 13, 11, 9, 7, 5, 3, 1,
                                             14, 12, 10, 8, 6, 4, 2, 0,
                                             15, 13, 11, 9, 7, 5, 3, 1,
                                             14, 12, 10, 8, 6, 4, 2, 0,
                                             15, 13, 11, 9, 7, 5, 3, 1,
                                             14, 12, 10, 8, 6, 4, 2, 0,
                                             15, 13, 11, 9, 7, 5, 3, 1,
                                             14, 12, 10, 8, 6, 4, 2, 0);
    __m512i vidxIn1_64QAM = _mm512_set_epi16( 31, 30, 29, 28, 27, 26, 25, 24,
                                              23, 19, 15, 11, 7, 3,
                                              22, 18, 14, 10, 6, 2,
                                              21, 17, 13, 9, 5, 1,
                                              20, 16, 12, 8, 4, 0);
    __m512i vidxIn2_64QAM = _mm512_set_epi8(15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0,
                                            15, 13, 11, 9, 7, 5, 14, 12, 10, 8, 6, 4, 3, 1, 2, 0,
                                            11, 9, 15, 13, 14, 12, 10, 8, 7, 5, 3, 1, 2, 0, 6, 4,
                                            15, 13, 14, 12, 11, 9, 7, 5, 3, 1, 10, 8, 6, 4, 2, 0);
    __m512i vidxIn3_64QAM = _mm512_set_epi16( 31, 30, 29, 28, 27, 26, 25, 24,
                                              23, 22, 21, 20, 19, 18, 17, 14,
                                              15, 16, 13, 12, 11, 10, 7, 8,
                                              9, 6, 5, 4, 3, 2, 1, 0);
    __m512i vidxOut1_64QAM = _mm512_set_epi16(31, 27, 23, 19, 15, 11, 7, 3,
                                              8, 9, 21, 30, 14, 26, 10, 22,
                                              6, 29, 13, 25, 18, 2, 28, 12,
                                              24, 5, 17, 1, 20, 4, 16, 0);
    __m512i vidxOut2_64QAM = _mm512_set_epi8(15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0,
                                             15, 14, 10, 12, 11, 13, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0,
                                             15, 11, 13, 9, 14, 10, 12, 8, 7, 6, 5, 4, 3, 2, 1, 0,
                                             15, 11, 13, 9, 14, 10, 12, 8, 7, 6, 5, 4, 3, 2, 1, 0);
    __m512i vidxOut3_64QAM = _mm512_set_epi32(15, 14, 13, 12,
                                              7, 10, 9, 8,
                                              3, 6, 5, 4,
                                              11, 2, 1, 0);
    __m512i vidxOut4_64QAM = _mm512_set_epi8(15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0,
                                             9, 8, 13, 7, 6, 14, 5, 4, 12, 3, 2, 10, 1, 0, 11, 15,
                                             11, 13, 7, 6, 14, 5, 4, 12, 9, 3, 2, 10, 1, 0, 8, 15,
                                             11, 12, 15, 14, 13, 7, 6, 9, 5, 4, 10, 3, 2, 8, 1, 0);

    __m512i vidxIn1_256QAM = _mm512_set_epi16( 31, 27, 23, 19, 15, 11, 7, 3,
                                               30, 26, 22, 18, 14, 10, 6, 2,
                                               29, 25, 21, 17, 13, 9, 5, 1,
                                               28, 24, 20, 16, 12, 8, 4, 0);
    __m512i vidxIn2_256QAM = _mm512_set_epi8(15, 13, 11, 9, 7, 5, 3, 1,
                                             14, 12, 10, 8, 6, 4, 2, 0,
                                             15, 13, 11, 9, 7, 5, 3, 1,
                                             14, 12, 10, 8, 6, 4, 2, 0,
                                             15, 13, 11, 9, 7, 5, 3, 1,
                                             14, 12, 10, 8, 6, 4, 2, 0,
                                             15, 13, 11, 9, 7, 5, 3, 1,
                                             14, 12, 10, 8, 6, 4, 2, 0);
    __m512i vidxOut1_256QAM = _mm512_set_epi16(31, 27, 23, 19, 15, 11, 7, 3,
                                               30, 26, 22, 18, 14, 10, 6, 2,
                                               29, 25, 21, 17, 13, 9, 5, 1,
                                               28, 24, 20, 16, 12, 8, 4, 0);
    __m512i vidxOut2_256QAM = _mm512_set_epi8(15, 13, 11, 9, 7, 5, 3, 1,
                                              14, 12, 10, 8, 6, 4, 2, 0,
                                              15, 13, 11, 9, 7, 5, 3, 1,
                                              14, 12, 10, 8, 6, 4, 2, 0,
                                              15, 13, 11, 9, 7, 5, 3, 1,
                                              14, 12, 10, 8, 6, 4, 2, 0,
                                              15, 13, 11, 9, 7, 5, 3, 1,
                                              14, 12, 10, 8, 6, 4, 2, 0);
    switch (Q)
    {
        case 1:
            bitTail = jMax & 0x7;
            memcpy(output, e, lenInByte);
            if (bitTail > 0)
            {
                ippsCopyBE_1u( e+lenInByte, 0, output+lenInByte, 0, bitTail );
            }
            break;
        case 2:
            bitTail = jMax & 0x7;
            byteTail = lenInByte & 0x3F;

            for (int j = 0;j <= lenInByte-64;j+=64)
            {
                tmp0_QPSK = _mm512_loadu_si512((__m512i *)(tmp[0]));
                tmp1_QPSK = _mm512_loadu_si512((__m512i *)(tmp[1]));
                tmp[0] += 64;
                tmp[1] += 64;

                /* order in byte:
                b0-0  b0-1  ... ... b0-15  // 256 bits
                b1-0  b1-1  ... ... b1-15
                */
                inter = _mm512_shuffle_i32x4(tmp0_QPSK, tmp1_QPSK, 0x44);
                //inter = _mm512_inserti64x4(inter, tmp1_QPSK, 1);

                /* order in byte:
                b0-0  b1-0  b0-1  b1-1  ... b0-7  b1-7  // 256 bits
                b0-8  b1-8  b0-9  b1-9  ... b0-15 b1-15 // 256 bits
                */
                inter = _mm512_permutex2var_epi16(inter, vidxIn1_QPSK, inter);
                inter = _mm512_shuffle_epi8(inter, vidxIn2_QPSK);

                /* order in bit, lower bit to higher bit
                val1:   14 15, 30 31, 46 47, 62 63, ... , 462 463, 478 479, 494 495, 510 511
                val2:   12 13, 28 29, 44 45, 60 61, ... , 460 461, 476 477, 492 493, 508 509
                val3:   10 11, 26 27, 42 43, 58 59, ... , 458 459, 474 475, 490 491, 506 507
                val4:   8  9 , 24 25, 40 41, 56 57, ... , 456 457, 472 473, 488 489, 504 505
                order in byte,
                out[1]: 1, 9,  17, 25, 33, 41, 49, 57
                out[3]: 3, 11, 19, 27, 35, 43, 51, 59
                out[5]: 5, 13, 21, 29, 37, 45, 53, 61
                out[7]: 7, 15, 23, 31, 39, 47, 55, 63
                */
                val1 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val2 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val3 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val4 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                out[1] = (unsigned __int64)( ( (val1&0x0303030303030303)<<6 ) | ( (val2&0x0303030303030303)<<4 ) | ( (val3&0x0303030303030303)<<2) | (val4&0x0303030303030303) );
                out[3] = (unsigned __int64)( ( (val1&0x0C0C0C0C0C0C0C0C)<<4 ) | ( (val2&0x0C0C0C0C0C0C0C0C)<<2 ) | (val3&0x0C0C0C0C0C0C0C0C) | ( (val4&0x0C0C0C0C0C0C0C0C)>>2) );
                out[5] = (unsigned __int64)( ( (val1&0x3030303030303030)<<2 ) | (val2&0x3030303030303030) | ( (val3&0x3030303030303030)>>2 ) | ( (val4&0x3030303030303030)>>4) );
                out[7] = (unsigned __int64)( (val1&0xC0C0C0C0C0C0C0C0) | ( (val2&0xC0C0C0C0C0C0C0C0)>>2 ) | ( (val3&0xC0C0C0C0C0C0C0C0)>>4 ) | ( (val4&0xC0C0C0C0C0C0C0C0)>>6) );

                /*
                order in byte,
                out[0]: 0, 8,  16, 24, 32, 40, 48, 56
                out[2]: 2, 10, 18, 26, 34, 42, 50, 58
                out[4]: 4, 12, 20, 28, 36, 44, 52, 60
                out[6]: 6, 14, 22, 30, 38, 46, 54, 62
                */
                val1 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val2 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val3 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val4 = _mm512_movepi8_mask(inter);
                out[0] = (unsigned __int64)( ( (val1&0x0303030303030303)<<6 ) | ( (val2&0x0303030303030303)<<4 ) | ( (val3&0x0303030303030303)<<2) | (val4&0x0303030303030303) );
                out[2] = (unsigned __int64)( ( (val1&0x0C0C0C0C0C0C0C0C)<<4 ) | ( (val2&0x0C0C0C0C0C0C0C0C)<<2 ) | (val3&0x0C0C0C0C0C0C0C0C) | ( (val4&0x0C0C0C0C0C0C0C0C)>>2) );
                out[4] = (unsigned __int64)( ( (val1&0x3030303030303030)<<2 ) | (val2&0x3030303030303030) | ( (val3&0x3030303030303030)>>2 ) | ( (val4&0x3030303030303030)>>4) );
                out[6] = (unsigned __int64)( (val1&0xC0C0C0C0C0C0C0C0) | ( (val2&0xC0C0C0C0C0C0C0C0)>>2 ) | ( (val3&0xC0C0C0C0C0C0C0C0)>>4 ) | ( (val4&0xC0C0C0C0C0C0C0C0)>>6) );

                /* Shuffle to right order */
                res = _mm512_loadu_si512(out);
                res = _mm512_permutex2var_epi16(res, vidxOut1_QPSK, res);
                res = _mm512_shuffle_epi8(res, vidxOut2_QPSK);

                _mm512_storeu_si512(output, res);
                output += 64;

                inter = _mm512_shuffle_i32x4(tmp0_QPSK, tmp1_QPSK, 0xEE);
                //inter = _mm512_inserti64x4(inter, tmp1_QPSK, 1);

                /* order in byte:
                b0-0  b1-0  b0-1  b1-1  ... b0-7  b1-7  // 256 bits
                b0-8  b1-8  b0-9  b1-9  ... b0-15 b1-15 // 256 bits
                */
                inter = _mm512_permutex2var_epi16(inter, vidxIn1_QPSK, inter);
                inter = _mm512_shuffle_epi8(inter, vidxIn2_QPSK);

                /* order in bit, lower bit to higher bit
                val1:   14 15, 30 31, 46 47, 62 63, ... , 462 463, 478 479, 494 495, 510 511
                val2:   12 13, 28 29, 44 45, 60 61, ... , 460 461, 476 477, 492 493, 508 509
                val3:   10 11, 26 27, 42 43, 58 59, ... , 458 459, 474 475, 490 491, 506 507
                val4:   8  9 , 24 25, 40 41, 56 57, ... , 456 457, 472 473, 488 489, 504 505
                order in byte,
                out[1]: 1, 9,  17, 25, 33, 41, 49, 57
                out[3]: 3, 11, 19, 27, 35, 43, 51, 59
                out[5]: 5, 13, 21, 29, 37, 45, 53, 61
                out[7]: 7, 15, 23, 31, 39, 47, 55, 63
                */
                val1 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val2 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val3 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val4 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                out[1] = (unsigned __int64)( ( (val1&0x0303030303030303)<<6 ) | ( (val2&0x0303030303030303)<<4 ) | ( (val3&0x0303030303030303)<<2) | (val4&0x0303030303030303) );
                out[3] = (unsigned __int64)( ( (val1&0x0C0C0C0C0C0C0C0C)<<4 ) | ( (val2&0x0C0C0C0C0C0C0C0C)<<2 ) | (val3&0x0C0C0C0C0C0C0C0C) | ( (val4&0x0C0C0C0C0C0C0C0C)>>2) );
                out[5] = (unsigned __int64)( ( (val1&0x3030303030303030)<<2 ) | (val2&0x3030303030303030) | ( (val3&0x3030303030303030)>>2 ) | ( (val4&0x3030303030303030)>>4) );
                out[7] = (unsigned __int64)( (val1&0xC0C0C0C0C0C0C0C0) | ( (val2&0xC0C0C0C0C0C0C0C0)>>2 ) | ( (val3&0xC0C0C0C0C0C0C0C0)>>4 ) | ( (val4&0xC0C0C0C0C0C0C0C0)>>6) );

                /*
                order in byte,
                out[0]: 0, 8,  16, 24, 32, 40, 48, 56
                out[2]: 2, 10, 18, 26, 34, 42, 50, 58
                out[4]: 4, 12, 20, 28, 36, 44, 52, 60
                out[6]: 6, 14, 22, 30, 38, 46, 54, 62
                */
                val1 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val2 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val3 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val4 = _mm512_movepi8_mask(inter);
                out[0] = (unsigned __int64)( ( (val1&0x0303030303030303)<<6 ) | ( (val2&0x0303030303030303)<<4 ) | ( (val3&0x0303030303030303)<<2) | (val4&0x0303030303030303) );
                out[2] = (unsigned __int64)( ( (val1&0x0C0C0C0C0C0C0C0C)<<4 ) | ( (val2&0x0C0C0C0C0C0C0C0C)<<2 ) | (val3&0x0C0C0C0C0C0C0C0C) | ( (val4&0x0C0C0C0C0C0C0C0C)>>2) );
                out[4] = (unsigned __int64)( ( (val1&0x3030303030303030)<<2 ) | (val2&0x3030303030303030) | ( (val3&0x3030303030303030)>>2 ) | ( (val4&0x3030303030303030)>>4) );
                out[6] = (unsigned __int64)( (val1&0xC0C0C0C0C0C0C0C0) | ( (val2&0xC0C0C0C0C0C0C0C0)>>2 ) | ( (val3&0xC0C0C0C0C0C0C0C0)>>4 ) | ( (val4&0xC0C0C0C0C0C0C0C0)>>6) );

                /* Shuffle to right order */
                res = _mm512_loadu_si512(out);
                res = _mm512_permutex2var_epi16(res, vidxOut1_QPSK, res);
                res = _mm512_shuffle_epi8(res, vidxOut2_QPSK);

                _mm512_storeu_si512(output, res);
                output += 64;
            }

            if (byteTail >= 32)
            {
                tmp0_QPSK = _mm512_loadu_si512((__m512i *)(tmp[0]));
                tmp1_QPSK = _mm512_loadu_si512((__m512i *)(tmp[1]));

                /* order in byte:
                b0-0  b0-1  ... ... b0-15  // 256 bits
                b1-0  b1-1  ... ... b1-15
                */
                inter = _mm512_shuffle_i32x4(tmp0_QPSK, tmp1_QPSK, 0x44);
                //inter = _mm512_inserti64x4(inter, tmp1_QPSK, 1);

                /* order in byte:
                b0-0  b1-0  b0-1  b1-1  ... b0-7  b1-7  // 256 bits
                b0-8  b1-8  b0-9  b1-9  ... b0-15 b1-15 // 256 bits
                */
                inter = _mm512_permutex2var_epi16(inter, vidxIn1_QPSK, inter);
                inter = _mm512_shuffle_epi8(inter, vidxIn2_QPSK);

                /* order in bit, lower bit to higher bit
                val1:   14 15, 30 31, 46 47, 62 63, ... , 462 463, 478 479, 494 495, 510 511
                val2:   12 13, 28 29, 44 45, 60 61, ... , 460 461, 476 477, 492 493, 508 509
                val3:   10 11, 26 27, 42 43, 58 59, ... , 458 459, 474 475, 490 491, 506 507
                val4:   8  9 , 24 25, 40 41, 56 57, ... , 456 457, 472 473, 488 489, 504 505
                order in byte,
                out[1]: 1, 9,  17, 25, 33, 41, 49, 57
                out[3]: 3, 11, 19, 27, 35, 43, 51, 59
                out[5]: 5, 13, 21, 29, 37, 45, 53, 61
                out[7]: 7, 15, 23, 31, 39, 47, 55, 63
                */
                val1 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val2 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val3 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val4 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                out[1] = (unsigned __int64)( ( (val1&0x0303030303030303)<<6 ) | ( (val2&0x0303030303030303)<<4 ) | ( (val3&0x0303030303030303)<<2) | (val4&0x0303030303030303) );
                out[3] = (unsigned __int64)( ( (val1&0x0C0C0C0C0C0C0C0C)<<4 ) | ( (val2&0x0C0C0C0C0C0C0C0C)<<2 ) | (val3&0x0C0C0C0C0C0C0C0C) | ( (val4&0x0C0C0C0C0C0C0C0C)>>2) );
                out[5] = (unsigned __int64)( ( (val1&0x3030303030303030)<<2 ) | (val2&0x3030303030303030) | ( (val3&0x3030303030303030)>>2 ) | ( (val4&0x3030303030303030)>>4) );
                out[7] = (unsigned __int64)( (val1&0xC0C0C0C0C0C0C0C0) | ( (val2&0xC0C0C0C0C0C0C0C0)>>2 ) | ( (val3&0xC0C0C0C0C0C0C0C0)>>4 ) | ( (val4&0xC0C0C0C0C0C0C0C0)>>6) );

                /*
                order in byte,
                out[0]: 0, 8,  16, 24, 32, 40, 48, 56
                out[2]: 2, 10, 18, 26, 34, 42, 50, 58
                out[4]: 4, 12, 20, 28, 36, 44, 52, 60
                out[6]: 6, 14, 22, 30, 38, 46, 54, 62
                */
                val1 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val2 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val3 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val4 = _mm512_movepi8_mask(inter);
                out[0] = (unsigned __int64)( ( (val1&0x0303030303030303)<<6 ) | ( (val2&0x0303030303030303)<<4 ) | ( (val3&0x0303030303030303)<<2) | (val4&0x0303030303030303) );
                out[2] = (unsigned __int64)( ( (val1&0x0C0C0C0C0C0C0C0C)<<4 ) | ( (val2&0x0C0C0C0C0C0C0C0C)<<2 ) | (val3&0x0C0C0C0C0C0C0C0C) | ( (val4&0x0C0C0C0C0C0C0C0C)>>2) );
                out[4] = (unsigned __int64)( ( (val1&0x3030303030303030)<<2 ) | (val2&0x3030303030303030) | ( (val3&0x3030303030303030)>>2 ) | ( (val4&0x3030303030303030)>>4) );
                out[6] = (unsigned __int64)( (val1&0xC0C0C0C0C0C0C0C0) | ( (val2&0xC0C0C0C0C0C0C0C0)>>2 ) | ( (val3&0xC0C0C0C0C0C0C0C0)>>4 ) | ( (val4&0xC0C0C0C0C0C0C0C0)>>6) );

                /* Shuffle to right order */
                res = _mm512_loadu_si512(out);
                res = _mm512_permutex2var_epi16(res, vidxOut1_QPSK, res);
                res = _mm512_shuffle_epi8(res, vidxOut2_QPSK);

                _mm512_storeu_si512(output, res);

                tmp[0] += 32;
                tmp[1] += 32;
                output += 64;
                byteTail -= 32;
            }

            if (byteTail > 0)
            {
                tmp0_QPSK = _mm512_loadu_si512((__m512i *)(tmp[0]));
                tmp1_QPSK = _mm512_loadu_si512((__m512i *)(tmp[1]));

                /* order in byte:
                b0-0  b0-1  ... ... b0-15  // 256 bits
                b1-0  b1-1  ... ... b1-15
                */
                inter = _mm512_shuffle_i32x4(tmp0_QPSK, tmp1_QPSK, 0x44);
                //inter = _mm512_inserti64x4(inter, tmp1_QPSK, 1);

                /* order in byte:
                b0-0  b1-0  b0-1  b1-1  ... b0-7  b1-7  // 256 bits
                b0-8  b1-8  b0-9  b1-9  ... b0-15 b1-15 // 256 bits
                */
                inter = _mm512_permutex2var_epi16(inter, vidxIn1_QPSK, inter);
                inter = _mm512_shuffle_epi8(inter, vidxIn2_QPSK);

                /* order in bit, lower bit to higher bit
                val1:   14 15, 30 31, 46 47, 62 63, ... , 462 463, 478 479, 494 495, 510 511
                val2:   12 13, 28 29, 44 45, 60 61, ... , 460 461, 476 477, 492 493, 508 509
                val3:   10 11, 26 27, 42 43, 58 59, ... , 458 459, 474 475, 490 491, 506 507
                val4:   8  9 , 24 25, 40 41, 56 57, ... , 456 457, 472 473, 488 489, 504 505
                order in byte,
                out[1]: 1, 9,  17, 25, 33, 41, 49, 57
                out[3]: 3, 11, 19, 27, 35, 43, 51, 59
                out[5]: 5, 13, 21, 29, 37, 45, 53, 61
                out[7]: 7, 15, 23, 31, 39, 47, 55, 63
                */
                val1 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val2 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val3 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val4 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                out[1] = (unsigned __int64)( ( (val1&0x0303030303030303)<<6 ) | ( (val2&0x0303030303030303)<<4 ) | ( (val3&0x0303030303030303)<<2) | (val4&0x0303030303030303) );
                out[3] = (unsigned __int64)( ( (val1&0x0C0C0C0C0C0C0C0C)<<4 ) | ( (val2&0x0C0C0C0C0C0C0C0C)<<2 ) | (val3&0x0C0C0C0C0C0C0C0C) | ( (val4&0x0C0C0C0C0C0C0C0C)>>2) );
                out[5] = (unsigned __int64)( ( (val1&0x3030303030303030)<<2 ) | (val2&0x3030303030303030) | ( (val3&0x3030303030303030)>>2 ) | ( (val4&0x3030303030303030)>>4) );
                out[7] = (unsigned __int64)( (val1&0xC0C0C0C0C0C0C0C0) | ( (val2&0xC0C0C0C0C0C0C0C0)>>2 ) | ( (val3&0xC0C0C0C0C0C0C0C0)>>4 ) | ( (val4&0xC0C0C0C0C0C0C0C0)>>6) );

                /*
                order in byte,
                out[0]: 0, 8,  16, 24, 32, 40, 48, 56
                out[2]: 2, 10, 18, 26, 34, 42, 50, 58
                out[4]: 4, 12, 20, 28, 36, 44, 52, 60
                out[6]: 6, 14, 22, 30, 38, 46, 54, 62
                */
                val1 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val2 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val3 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val4 = _mm512_movepi8_mask(inter);
                out[0] = (unsigned __int64)( ( (val1&0x0303030303030303)<<6 ) | ( (val2&0x0303030303030303)<<4 ) | ( (val3&0x0303030303030303)<<2) | (val4&0x0303030303030303) );
                out[2] = (unsigned __int64)( ( (val1&0x0C0C0C0C0C0C0C0C)<<4 ) | ( (val2&0x0C0C0C0C0C0C0C0C)<<2 ) | (val3&0x0C0C0C0C0C0C0C0C) | ( (val4&0x0C0C0C0C0C0C0C0C)>>2) );
                out[4] = (unsigned __int64)( ( (val1&0x3030303030303030)<<2 ) | (val2&0x3030303030303030) | ( (val3&0x3030303030303030)>>2 ) | ( (val4&0x3030303030303030)>>4) );
                out[6] = (unsigned __int64)( (val1&0xC0C0C0C0C0C0C0C0) | ( (val2&0xC0C0C0C0C0C0C0C0)>>2 ) | ( (val3&0xC0C0C0C0C0C0C0C0)>>4 ) | ( (val4&0xC0C0C0C0C0C0C0C0)>>6) );

                /* Shuffle to right order */
                res = _mm512_loadu_si512(out);
                res = _mm512_permutex2var_epi16(res, vidxOut1_QPSK, res);
                res = _mm512_shuffle_epi8(res, vidxOut2_QPSK);

                left = (__mmask64)(tailMask[byteTail]);
                _mm512_mask_storeu_epi8(output, left, res);

                tmp[0] += byteTail;
                tmp[1] += byteTail;
                output += byteTail * 2;
            }

            byteOffset = 0;
            bitOffset = 0;
            for (int i=0;i < bitTail;i++)
            {
                ippsCopyBE_1u( (Ipp8u*)tmp[0], i, output+byteOffset, bitOffset+0, 1 );
                ippsCopyBE_1u( (Ipp8u*)tmp[1], i, output+byteOffset, bitOffset+1, 1 );
                bitOffset += 2;
                byteOffset += bitOffset >> 3;
                bitOffset = bitOffset & 0x7;
            }
            break;
        case 4:
            bitTail = jMax & 0x7;
            byteTail = lenInByte & 0x3F;

            for (int j = 0;j <= lenInByte-64;j+=64)
            {
                tmp0_16QAM = _mm512_loadu_si512((__m512i *)(tmp[0]));
                tmp1_16QAM = _mm512_loadu_si512((__m512i *)(tmp[1]));
                tmp2_16QAM = _mm512_loadu_si512((__m512i *)(tmp[2]));
                tmp3_16QAM = _mm512_loadu_si512((__m512i *)(tmp[3]));
                tmp[0] += 64;
                tmp[1] += 64;
                tmp[2] += 64;
                tmp[3] += 64;

                /* order in byte:
                b0-0  b0-1  ... ... b0-16  // 128bits
                b1-0  b1-1  ... ... b1-16
                b2-0  b2-1  ... ... b2-16
                b3-0  b3-1  ... ... b3-16
                */
                inter = _mm512_mask_shuffle_i64x2(inter, 0x33, tmp0_16QAM, tmp2_16QAM, 0x0);
                inter = _mm512_mask_shuffle_i64x2(inter, 0xCC, tmp1_16QAM, tmp3_16QAM, 0x0);

                /* order in byte:
                b0-0  b1-0  b2-0  b3-0  b0-1  b1-1  b2-1  b3-1  ... b0-3  b1-3  b2-3  b3-3  // 128bits
                b0-4  b1-4  b2-4  b3-4  b0-5  b1-5  b2-5  b3-5  ... b0-7  b1-7  b2-7  b3-7  // 128bits
                b0-8  b1-8  b2-8  b3-8  b0-9  b1-9  b2-9  b3-9  ... b0-11 b1-11 b2-11 b3-11 // 128bits
                b0-12 b1-12 b2-12 b3-12 b0-13 b1-13 b2-13 b3-13 ... b0-15 b1-15 b2-15 b3-15 // 128bits
                */
                inter = _mm512_permutex2var_epi16(inter, vidxIn1_16QAM, inter);
                inter = _mm512_shuffle_epi8(inter, vidxIn2_16QAM);

                /* order in bit, lower bit to higher bit
                val1:   28 29 30 31, 60 61 62 63, ... , 476 477 478 479, 508 509 510 511
                val2:   24 25 26 27, 56 57 58 59, ... , 472 473 474 475, 504 505 506 507
                out[3]: 24 24 26 27, 28 29 30 31, ... , 472 473 474 475, 476 477 478 479
                out[7]: 56 57 58 59, 60 61 62 63, ... , 504 505 506 507, 508 509 510 511
                order in byte,
                out[3]: 3, 11, 19, 27, 35, 43, 51, 59
                out[7]: 7, 15, 23, 31, 39, 47, 55, 63
                */
                val1 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val2 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                out[3] = (unsigned __int64)( ( (val1 & 0x0F0F0F0F0F0F0F0F)<<4 ) | ( val2 & 0x0F0F0F0F0F0F0F0F ) );
                out[7] = (unsigned __int64)( ( val1 & 0xF0F0F0F0F0F0F0F0 ) | ( (val2 & 0xF0F0F0F0F0F0F0F0)>>4 ) );

                /*
                order in byte,
                out[2]: 2, 10, 18, 26, 34, 42, 50, 58
                out[6]: 6, 14, 22, 30, 38, 46, 54, 62
                */
                val1 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val2 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                out[2] = (unsigned __int64)( ( (val1 & 0x0F0F0F0F0F0F0F0F)<<4 ) | ( val2 & 0x0F0F0F0F0F0F0F0F ) );
                out[6] = (unsigned __int64)( ( val1 & 0xF0F0F0F0F0F0F0F0 ) | ( (val2 & 0xF0F0F0F0F0F0F0F0)>>4 ) );

                /*
                order in byte,
                out[1]: 1, 9,  17, 25, 33, 41, 49, 57
                out[5]: 5, 13, 21, 29, 37, 45, 53, 61
                */
                val1 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val2 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                out[1] = (unsigned __int64)( ( (val1 & 0x0F0F0F0F0F0F0F0F)<<4 ) | ( val2 & 0x0F0F0F0F0F0F0F0F ) );
                out[5] = (unsigned __int64)( ( val1 & 0xF0F0F0F0F0F0F0F0 ) | ( (val2 & 0xF0F0F0F0F0F0F0F0)>>4 ) );

                /*
                order in byte,
                out[0]: 0, 8,  16, 24, 32, 40, 48, 56
                out[4]: 4, 12, 20, 28, 36, 44, 52, 60
                */
                val1 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val2 = _mm512_movepi8_mask(inter);
                out[0] = (unsigned __int64)( ( (val1 & 0x0F0F0F0F0F0F0F0F)<<4 ) | ( val2 & 0x0F0F0F0F0F0F0F0F ) );
                out[4] = (unsigned __int64)( ( val1 & 0xF0F0F0F0F0F0F0F0 ) | ( (val2 & 0xF0F0F0F0F0F0F0F0)>>4 ) );

                /* Shuffle to right order */
                res = _mm512_loadu_si512(out);
                res = _mm512_permutex2var_epi16(res, vidxOut1_16QAM, res);
                res = _mm512_shuffle_epi8(res, vidxOut2_16QAM);

                _mm512_storeu_si512(output, res);
                output += 64;

                inter = _mm512_mask_shuffle_i64x2(inter, 0x33, tmp0_16QAM, tmp2_16QAM, 0x55);
                inter = _mm512_mask_shuffle_i64x2(inter, 0xCC, tmp1_16QAM, tmp3_16QAM, 0x55);

                /* order in byte:
                b0-0  b1-0  b2-0  b3-0  b0-1  b1-1  b2-1  b3-1  ... b0-3  b1-3  b2-3  b3-3  // 128bits
                b0-4  b1-4  b2-4  b3-4  b0-5  b1-5  b2-5  b3-5  ... b0-7  b1-7  b2-7  b3-7  // 128bits
                b0-8  b1-8  b2-8  b3-8  b0-9  b1-9  b2-9  b3-9  ... b0-11 b1-11 b2-11 b3-11 // 128bits
                b0-12 b1-12 b2-12 b3-12 b0-13 b1-13 b2-13 b3-13 ... b0-15 b1-15 b2-15 b3-15 // 128bits
                */
                inter = _mm512_permutex2var_epi16(inter, vidxIn1_16QAM, inter);
                inter = _mm512_shuffle_epi8(inter, vidxIn2_16QAM);

                /* order in bit, lower bit to higher bit
                val1:   28 29 30 31, 60 61 62 63, ... , 476 477 478 479, 508 509 510 511
                val2:   24 25 26 27, 56 57 58 59, ... , 472 473 474 475, 504 505 506 507
                out[3]: 24 24 26 27, 28 29 30 31, ... , 472 473 474 475, 476 477 478 479
                out[7]: 56 57 58 59, 60 61 62 63, ... , 504 505 506 507, 508 509 510 511
                order in byte,
                out[3]: 3, 11, 19, 27, 35, 43, 51, 59
                out[7]: 7, 15, 23, 31, 39, 47, 55, 63
                */
                val1 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val2 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                out[3] = (unsigned __int64)( ( (val1 & 0x0F0F0F0F0F0F0F0F)<<4 ) | ( val2 & 0x0F0F0F0F0F0F0F0F ) );
                out[7] = (unsigned __int64)( ( val1 & 0xF0F0F0F0F0F0F0F0 ) | ( (val2 & 0xF0F0F0F0F0F0F0F0)>>4 ) );

                /*
                order in byte,
                out[2]: 2, 10, 18, 26, 34, 42, 50, 58
                out[6]: 6, 14, 22, 30, 38, 46, 54, 62
                */
                val1 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val2 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                out[2] = (unsigned __int64)( ( (val1 & 0x0F0F0F0F0F0F0F0F)<<4 ) | ( val2 & 0x0F0F0F0F0F0F0F0F ) );
                out[6] = (unsigned __int64)( ( val1 & 0xF0F0F0F0F0F0F0F0 ) | ( (val2 & 0xF0F0F0F0F0F0F0F0)>>4 ) );

                /*
                order in byte,
                out[1]: 1, 9,  17, 25, 33, 41, 49, 57
                out[5]: 5, 13, 21, 29, 37, 45, 53, 61
                */
                val1 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val2 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                out[1] = (unsigned __int64)( ( (val1 & 0x0F0F0F0F0F0F0F0F)<<4 ) | ( val2 & 0x0F0F0F0F0F0F0F0F ) );
                out[5] = (unsigned __int64)( ( val1 & 0xF0F0F0F0F0F0F0F0 ) | ( (val2 & 0xF0F0F0F0F0F0F0F0)>>4 ) );

                /*
                order in byte,
                out[0]: 0, 8,  16, 24, 32, 40, 48, 56
                out[4]: 4, 12, 20, 28, 36, 44, 52, 60
                */
                val1 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val2 = _mm512_movepi8_mask(inter);
                out[0] = (unsigned __int64)( ( (val1 & 0x0F0F0F0F0F0F0F0F)<<4 ) | ( val2 & 0x0F0F0F0F0F0F0F0F ) );
                out[4] = (unsigned __int64)( ( val1 & 0xF0F0F0F0F0F0F0F0 ) | ( (val2 & 0xF0F0F0F0F0F0F0F0)>>4 ) );

                /* Shuffle to right order */
                res = _mm512_loadu_si512(out);
                res = _mm512_permutex2var_epi16(res, vidxOut1_16QAM, res);
                res = _mm512_shuffle_epi8(res, vidxOut2_16QAM);

                _mm512_storeu_si512(output, res);
                output += 64;

                inter = _mm512_mask_shuffle_i64x2(inter, 0x33, tmp0_16QAM, tmp2_16QAM, 0xAA);
                inter = _mm512_mask_shuffle_i64x2(inter, 0xCC, tmp1_16QAM, tmp3_16QAM, 0xAA);

                /* order in byte:
                b0-0  b1-0  b2-0  b3-0  b0-1  b1-1  b2-1  b3-1  ... b0-3  b1-3  b2-3  b3-3  // 128bits
                b0-4  b1-4  b2-4  b3-4  b0-5  b1-5  b2-5  b3-5  ... b0-7  b1-7  b2-7  b3-7  // 128bits
                b0-8  b1-8  b2-8  b3-8  b0-9  b1-9  b2-9  b3-9  ... b0-11 b1-11 b2-11 b3-11 // 128bits
                b0-12 b1-12 b2-12 b3-12 b0-13 b1-13 b2-13 b3-13 ... b0-15 b1-15 b2-15 b3-15 // 128bits
                */
                inter = _mm512_permutex2var_epi16(inter, vidxIn1_16QAM, inter);
                inter = _mm512_shuffle_epi8(inter, vidxIn2_16QAM);

                /* order in bit, lower bit to higher bit
                val1:   28 29 30 31, 60 61 62 63, ... , 476 477 478 479, 508 509 510 511
                val2:   24 25 26 27, 56 57 58 59, ... , 472 473 474 475, 504 505 506 507
                out[3]: 24 24 26 27, 28 29 30 31, ... , 472 473 474 475, 476 477 478 479
                out[7]: 56 57 58 59, 60 61 62 63, ... , 504 505 506 507, 508 509 510 511
                order in byte,
                out[3]: 3, 11, 19, 27, 35, 43, 51, 59
                out[7]: 7, 15, 23, 31, 39, 47, 55, 63
                */
                val1 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val2 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                out[3] = (unsigned __int64)( ( (val1 & 0x0F0F0F0F0F0F0F0F)<<4 ) | ( val2 & 0x0F0F0F0F0F0F0F0F ) );
                out[7] = (unsigned __int64)( ( val1 & 0xF0F0F0F0F0F0F0F0 ) | ( (val2 & 0xF0F0F0F0F0F0F0F0)>>4 ) );

                /*
                order in byte,
                out[2]: 2, 10, 18, 26, 34, 42, 50, 58
                out[6]: 6, 14, 22, 30, 38, 46, 54, 62
                */
                val1 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val2 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                out[2] = (unsigned __int64)( ( (val1 & 0x0F0F0F0F0F0F0F0F)<<4 ) | ( val2 & 0x0F0F0F0F0F0F0F0F ) );
                out[6] = (unsigned __int64)( ( val1 & 0xF0F0F0F0F0F0F0F0 ) | ( (val2 & 0xF0F0F0F0F0F0F0F0)>>4 ) );

                /*
                order in byte,
                out[1]: 1, 9,  17, 25, 33, 41, 49, 57
                out[5]: 5, 13, 21, 29, 37, 45, 53, 61
                */
                val1 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val2 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                out[1] = (unsigned __int64)( ( (val1 & 0x0F0F0F0F0F0F0F0F)<<4 ) | ( val2 & 0x0F0F0F0F0F0F0F0F ) );
                out[5] = (unsigned __int64)( ( val1 & 0xF0F0F0F0F0F0F0F0 ) | ( (val2 & 0xF0F0F0F0F0F0F0F0)>>4 ) );

                /*
                order in byte,
                out[0]: 0, 8,  16, 24, 32, 40, 48, 56
                out[4]: 4, 12, 20, 28, 36, 44, 52, 60
                */
                val1 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val2 = _mm512_movepi8_mask(inter);
                out[0] = (unsigned __int64)( ( (val1 & 0x0F0F0F0F0F0F0F0F)<<4 ) | ( val2 & 0x0F0F0F0F0F0F0F0F ) );
                out[4] = (unsigned __int64)( ( val1 & 0xF0F0F0F0F0F0F0F0 ) | ( (val2 & 0xF0F0F0F0F0F0F0F0)>>4 ) );

                /* Shuffle to right order */
                res = _mm512_loadu_si512(out);
                res = _mm512_permutex2var_epi16(res, vidxOut1_16QAM, res);
                res = _mm512_shuffle_epi8(res, vidxOut2_16QAM);

                _mm512_storeu_si512(output, res);
                output += 64;

                inter = _mm512_mask_shuffle_i64x2(inter, 0x33, tmp0_16QAM, tmp2_16QAM, 0xFF);
                inter = _mm512_mask_shuffle_i64x2(inter, 0xCC, tmp1_16QAM, tmp3_16QAM, 0xFF);

                /* order in byte:
                b0-0  b1-0  b2-0  b3-0  b0-1  b1-1  b2-1  b3-1  ... b0-3  b1-3  b2-3  b3-3  // 128bits
                b0-4  b1-4  b2-4  b3-4  b0-5  b1-5  b2-5  b3-5  ... b0-7  b1-7  b2-7  b3-7  // 128bits
                b0-8  b1-8  b2-8  b3-8  b0-9  b1-9  b2-9  b3-9  ... b0-11 b1-11 b2-11 b3-11 // 128bits
                b0-12 b1-12 b2-12 b3-12 b0-13 b1-13 b2-13 b3-13 ... b0-15 b1-15 b2-15 b3-15 // 128bits
                */
                inter = _mm512_permutex2var_epi16(inter, vidxIn1_16QAM, inter);
                inter = _mm512_shuffle_epi8(inter, vidxIn2_16QAM);

                /* order in bit, lower bit to higher bit
                val1:   28 29 30 31, 60 61 62 63, ... , 476 477 478 479, 508 509 510 511
                val2:   24 25 26 27, 56 57 58 59, ... , 472 473 474 475, 504 505 506 507
                out[3]: 24 24 26 27, 28 29 30 31, ... , 472 473 474 475, 476 477 478 479
                out[7]: 56 57 58 59, 60 61 62 63, ... , 504 505 506 507, 508 509 510 511
                order in byte,
                out[3]: 3, 11, 19, 27, 35, 43, 51, 59
                out[7]: 7, 15, 23, 31, 39, 47, 55, 63
                */
                val1 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val2 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                out[3] = (unsigned __int64)( ( (val1 & 0x0F0F0F0F0F0F0F0F)<<4 ) | ( val2 & 0x0F0F0F0F0F0F0F0F ) );
                out[7] = (unsigned __int64)( ( val1 & 0xF0F0F0F0F0F0F0F0 ) | ( (val2 & 0xF0F0F0F0F0F0F0F0)>>4 ) );

                /*
                order in byte,
                out[2]: 2, 10, 18, 26, 34, 42, 50, 58
                out[6]: 6, 14, 22, 30, 38, 46, 54, 62
                */
                val1 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val2 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                out[2] = (unsigned __int64)( ( (val1 & 0x0F0F0F0F0F0F0F0F)<<4 ) | ( val2 & 0x0F0F0F0F0F0F0F0F ) );
                out[6] = (unsigned __int64)( ( val1 & 0xF0F0F0F0F0F0F0F0 ) | ( (val2 & 0xF0F0F0F0F0F0F0F0)>>4 ) );

                /*
                order in byte,
                out[1]: 1, 9,  17, 25, 33, 41, 49, 57
                out[5]: 5, 13, 21, 29, 37, 45, 53, 61
                */
                val1 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val2 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                out[1] = (unsigned __int64)( ( (val1 & 0x0F0F0F0F0F0F0F0F)<<4 ) | ( val2 & 0x0F0F0F0F0F0F0F0F ) );
                out[5] = (unsigned __int64)( ( val1 & 0xF0F0F0F0F0F0F0F0 ) | ( (val2 & 0xF0F0F0F0F0F0F0F0)>>4 ) );

                /*
                order in byte,
                out[0]: 0, 8,  16, 24, 32, 40, 48, 56
                out[4]: 4, 12, 20, 28, 36, 44, 52, 60
                */
                val1 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val2 = _mm512_movepi8_mask(inter);
                out[0] = (unsigned __int64)( ( (val1 & 0x0F0F0F0F0F0F0F0F)<<4 ) | ( val2 & 0x0F0F0F0F0F0F0F0F ) );
                out[4] = (unsigned __int64)( ( val1 & 0xF0F0F0F0F0F0F0F0 ) | ( (val2 & 0xF0F0F0F0F0F0F0F0)>>4 ) );

                /* Shuffle to right order */
                res = _mm512_loadu_si512(out);
                res = _mm512_permutex2var_epi16(res, vidxOut1_16QAM, res);
                res = _mm512_shuffle_epi8(res, vidxOut2_16QAM);

                _mm512_storeu_si512(output, res);
                output += 64;
            }

            while (byteTail >= 16)
            {
                tmp0_16QAM = _mm512_loadu_si512((__m512i *)(tmp[0]));
                tmp1_16QAM = _mm512_loadu_si512((__m512i *)(tmp[1]));
                tmp2_16QAM = _mm512_loadu_si512((__m512i *)(tmp[2]));
                tmp3_16QAM = _mm512_loadu_si512((__m512i *)(tmp[3]));

                /* order in byte:
                b0-0  b0-1  ... ... b0-16  // 128bits
                b1-0  b1-1  ... ... b1-16
                b2-0  b2-1  ... ... b2-16
                b3-0  b3-1  ... ... b3-16
                */
                inter = _mm512_mask_shuffle_i64x2(inter, 0x33, tmp0_16QAM, tmp2_16QAM, 0x0);
                inter = _mm512_mask_shuffle_i64x2(inter, 0xCC, tmp1_16QAM, tmp3_16QAM, 0x0);

                /* order in byte:
                b0-0  b1-0  b2-0  b3-0  b0-1  b1-1  b2-1  b3-1  ... b0-3  b1-3  b2-3  b3-3  // 128bits
                b0-4  b1-4  b2-4  b3-4  b0-5  b1-5  b2-5  b3-5  ... b0-7  b1-7  b2-7  b3-7  // 128bits
                b0-8  b1-8  b2-8  b3-8  b0-9  b1-9  b2-9  b3-9  ... b0-11 b1-11 b2-11 b3-11 // 128bits
                b0-12 b1-12 b2-12 b3-12 b0-13 b1-13 b2-13 b3-13 ... b0-15 b1-15 b2-15 b3-15 // 128bits
                */
                inter = _mm512_permutex2var_epi16(inter, vidxIn1_16QAM, inter);
                inter = _mm512_shuffle_epi8(inter, vidxIn2_16QAM);

                /* order in bit, lower bit to higher bit
                val1:   28 29 30 31, 60 61 62 63, ... , 476 477 478 479, 508 509 510 511
                val2:   24 25 26 27, 56 57 58 59, ... , 472 473 474 475, 504 505 506 507
                out[3]: 24 24 26 27, 28 29 30 31, ... , 472 473 474 475, 476 477 478 479
                out[7]: 56 57 58 59, 60 61 62 63, ... , 504 505 506 507, 508 509 510 511
                order in byte,
                out[3]: 3, 11, 19, 27, 35, 43, 51, 59
                out[7]: 7, 15, 23, 31, 39, 47, 55, 63
                */
                val1 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val2 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                out[3] = (unsigned __int64)( ( (val1 & 0x0F0F0F0F0F0F0F0F)<<4 ) | ( val2 & 0x0F0F0F0F0F0F0F0F ) );
                out[7] = (unsigned __int64)( ( val1 & 0xF0F0F0F0F0F0F0F0 ) | ( (val2 & 0xF0F0F0F0F0F0F0F0)>>4 ) );

                /*
                order in byte,
                out[2]: 2, 10, 18, 26, 34, 42, 50, 58
                out[6]: 6, 14, 22, 30, 38, 46, 54, 62
                */
                val1 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val2 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                out[2] = (unsigned __int64)( ( (val1 & 0x0F0F0F0F0F0F0F0F)<<4 ) | ( val2 & 0x0F0F0F0F0F0F0F0F ) );
                out[6] = (unsigned __int64)( ( val1 & 0xF0F0F0F0F0F0F0F0 ) | ( (val2 & 0xF0F0F0F0F0F0F0F0)>>4 ) );

                /*
                order in byte,
                out[1]: 1, 9,  17, 25, 33, 41, 49, 57
                out[5]: 5, 13, 21, 29, 37, 45, 53, 61
                */
                val1 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val2 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                out[1] = (unsigned __int64)( ( (val1 & 0x0F0F0F0F0F0F0F0F)<<4 ) | ( val2 & 0x0F0F0F0F0F0F0F0F ) );
                out[5] = (unsigned __int64)( ( val1 & 0xF0F0F0F0F0F0F0F0 ) | ( (val2 & 0xF0F0F0F0F0F0F0F0)>>4 ) );

                /*
                order in byte,
                out[0]: 0, 8,  16, 24, 32, 40, 48, 56
                out[4]: 4, 12, 20, 28, 36, 44, 52, 60
                */
                val1 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val2 = _mm512_movepi8_mask(inter);
                out[0] = (unsigned __int64)( ( (val1 & 0x0F0F0F0F0F0F0F0F)<<4 ) | ( val2 & 0x0F0F0F0F0F0F0F0F ) );
                out[4] = (unsigned __int64)( ( val1 & 0xF0F0F0F0F0F0F0F0 ) | ( (val2 & 0xF0F0F0F0F0F0F0F0)>>4 ) );

                /* Shuffle to right order */
                res = _mm512_loadu_si512(out);
                res = _mm512_permutex2var_epi16(res, vidxOut1_16QAM, res);
                res = _mm512_shuffle_epi8(res, vidxOut2_16QAM);

                _mm512_storeu_si512(output, res);

                tmp[0] += 16;
                tmp[1] += 16;
                tmp[2] += 16;
                tmp[3] += 16;
                output += 64;
                byteTail -= 16;
            }

            if (byteTail > 0)
            {
                tmp0_16QAM = _mm512_loadu_si512((__m512i *)(tmp[0]));
                tmp1_16QAM = _mm512_loadu_si512((__m512i *)(tmp[1]));
                tmp2_16QAM = _mm512_loadu_si512((__m512i *)(tmp[2]));
                tmp3_16QAM = _mm512_loadu_si512((__m512i *)(tmp[3]));

                /* order in byte:
                b0-0  b0-1  ... ... b0-16  // 128bits
                b1-0  b1-1  ... ... b1-16
                b2-0  b2-1  ... ... b2-16
                b3-0  b3-1  ... ... b3-16
                */
                inter = _mm512_mask_shuffle_i64x2(inter, 0x33, tmp0_16QAM, tmp2_16QAM, 0x0);
                inter = _mm512_mask_shuffle_i64x2(inter, 0xCC, tmp1_16QAM, tmp3_16QAM, 0x0);

                /* order in byte:
                b0-0  b1-0  b2-0  b3-0  b0-1  b1-1  b2-1  b3-1  ... b0-3  b1-3  b2-3  b3-3  // 128bits
                b0-4  b1-4  b2-4  b3-4  b0-5  b1-5  b2-5  b3-5  ... b0-7  b1-7  b2-7  b3-7  // 128bits
                b0-8  b1-8  b2-8  b3-8  b0-9  b1-9  b2-9  b3-9  ... b0-11 b1-11 b2-11 b3-11 // 128bits
                b0-12 b1-12 b2-12 b3-12 b0-13 b1-13 b2-13 b3-13 ... b0-15 b1-15 b2-15 b3-15 // 128bits
                */
                inter = _mm512_permutex2var_epi16(inter, vidxIn1_16QAM, inter);
                inter = _mm512_shuffle_epi8(inter, vidxIn2_16QAM);

                /* order in bit, lower bit to higher bit
                val1:   28 29 30 31, 60 61 62 63, ... , 476 477 478 479, 508 509 510 511
                val2:   24 25 26 27, 56 57 58 59, ... , 472 473 474 475, 504 505 506 507
                out[3]: 24 24 26 27, 28 29 30 31, ... , 472 473 474 475, 476 477 478 479
                out[7]: 56 57 58 59, 60 61 62 63, ... , 504 505 506 507, 508 509 510 511
                order in byte,
                out[3]: 3, 11, 19, 27, 35, 43, 51, 59
                out[7]: 7, 15, 23, 31, 39, 47, 55, 63
                */
                val1 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val2 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                out[3] = (unsigned __int64)( ( (val1 & 0x0F0F0F0F0F0F0F0F)<<4 ) | ( val2 & 0x0F0F0F0F0F0F0F0F ) );
                out[7] = (unsigned __int64)( ( val1 & 0xF0F0F0F0F0F0F0F0 ) | ( (val2 & 0xF0F0F0F0F0F0F0F0)>>4 ) );

                /*
                order in byte,
                out[2]: 2, 10, 18, 26, 34, 42, 50, 58
                out[6]: 6, 14, 22, 30, 38, 46, 54, 62
                */
                val1 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val2 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                out[2] = (unsigned __int64)( ( (val1 & 0x0F0F0F0F0F0F0F0F)<<4 ) | ( val2 & 0x0F0F0F0F0F0F0F0F ) );
                out[6] = (unsigned __int64)( ( val1 & 0xF0F0F0F0F0F0F0F0 ) | ( (val2 & 0xF0F0F0F0F0F0F0F0)>>4 ) );

                /*
                order in byte,
                out[1]: 1, 9,  17, 25, 33, 41, 49, 57
                out[5]: 5, 13, 21, 29, 37, 45, 53, 61
                */
                val1 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val2 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                out[1] = (unsigned __int64)( ( (val1 & 0x0F0F0F0F0F0F0F0F)<<4 ) | ( val2 & 0x0F0F0F0F0F0F0F0F ) );
                out[5] = (unsigned __int64)( ( val1 & 0xF0F0F0F0F0F0F0F0 ) | ( (val2 & 0xF0F0F0F0F0F0F0F0)>>4 ) );

                /*
                order in byte,
                out[0]: 0, 8,  16, 24, 32, 40, 48, 56
                out[4]: 4, 12, 20, 28, 36, 44, 52, 60
                */
                val1 = _mm512_movepi8_mask(inter);
                inter = _mm512_slli_epi64(inter, 1);
                val2 = _mm512_movepi8_mask(inter);
                out[0] = (unsigned __int64)( ( (val1 & 0x0F0F0F0F0F0F0F0F)<<4 ) | ( val2 & 0x0F0F0F0F0F0F0F0F ) );
                out[4] = (unsigned __int64)( ( val1 & 0xF0F0F0F0F0F0F0F0 ) | ( (val2 & 0xF0F0F0F0F0F0F0F0)>>4 ) );

                /* Shuffle to right order */
                res = _mm512_loadu_si512(out);
                res = _mm512_permutex2var_epi16(res, vidxOut1_16QAM, res);
                res = _mm512_shuffle_epi8(res, vidxOut2_16QAM);

                left = (__mmask64)(tailMask[byteTail*2]);
                _mm512_mask_storeu_epi8(output, left, res);

                tmp[0] += byteTail;
                tmp[1] += byteTail;
                tmp[2] += byteTail;
                tmp[3] += byteTail;
                output += byteTail * 4;
            }

            byteOffset = 0;
            bitOffset = 0;
            for (int i=0;i < bitTail;i++)
            {
                ippsCopyBE_1u( (Ipp8u*)tmp[0], i, output+byteOffset, bitOffset+0, 1 );
                ippsCopyBE_1u( (Ipp8u*)tmp[1], i, output+byteOffset, bitOffset+1, 1 );
                ippsCopyBE_1u( (Ipp8u*)tmp[2], i, output+byteOffset, bitOffset+2, 1 );
                ippsCopyBE_1u( (Ipp8u*)tmp[3], i, output+byteOffset, bitOffset+3, 1 );
                bitOffset += 4;
                byteOffset += bitOffset >> 3;
                bitOffset = bitOffset & 0x7;
            }
            break;
        case 6:
            bitTail = jMax & 7;
            byteTail = lenInByte & 7;

            in[6] = 0;
            in[7] = 0;

            for (int j = 0;j <= lenInByte-8;j+=8)
            {
                in[0] = *(unsigned __int64 *)(tmp[0]);
                in[1] = *(unsigned __int64 *)(tmp[1]);
                in[2] = *(unsigned __int64 *)(tmp[2]);
                in[3] = *(unsigned __int64 *)(tmp[3]);
                in[4] = *(unsigned __int64 *)(tmp[4]);
                in[5] = *(unsigned __int64 *)(tmp[5]);

                inter = _mm512_loadu_si512(in);

                inter = _mm512_permutex2var_epi16(inter, vidxIn1_64QAM, inter);
                inter = _mm512_shuffle_epi8(inter, vidxIn2_64QAM);
                inter = _mm512_permutex2var_epi16(inter, vidxIn3_64QAM, inter);

                val1 = (unsigned __int64)(_mm512_movepi8_mask(inter));
                inter = _mm512_slli_epi64(inter, 1);
                val2 = (unsigned __int64)(_mm512_movepi8_mask(inter));
                inter = _mm512_slli_epi64(inter, 1);
                val3 = (unsigned __int64)(_mm512_movepi8_mask(inter));
                inter = _mm512_slli_epi64(inter, 1);
                val4 = (unsigned __int64)(_mm512_movepi8_mask(inter));
                inter = _mm512_slli_epi64(inter, 1);

                out[4] = (val4&0x00003F00003F) | ( (val3&0x00003F00003F)<<6  ) | ( (val2&0x00003F00003F)<<12 ) | ( (val1&0x00003F00003F)<<18 );
                out[5] = ( (val4&0x000FC0000FC0)>>6  ) | (val3&0x000FC0000FC0) | ( (val2&0x000FC0000FC0)<<6  ) | ( (val1&0x000FC0000FC0)<<12 );
                out[6] = ( (val4&0x03F00003F000)>>12 ) | ( (val3&0x03F00003F000)>>6  ) | (val2&0x03F00003F000) | ( (val1&0x03F00003F000)<<6  );
                out[7] = ( (val4&0xFC0000FC0000)>>18 ) | ( (val3&0xFC0000FC0000)>>12 ) | ( (val2&0xFC0000FC0000)>>6  ) | (val1&0xFC0000FC0000);

                val1 = (unsigned __int64)(_mm512_movepi8_mask(inter));
                inter = _mm512_slli_epi64(inter, 1);
                val2 = (unsigned __int64)(_mm512_movepi8_mask(inter));
                inter = _mm512_slli_epi64(inter, 1);
                val3 = (unsigned __int64)(_mm512_movepi8_mask(inter));
                inter = _mm512_slli_epi64(inter, 1);
                val4 = (unsigned __int64)(_mm512_movepi8_mask(inter));

                out[0] = (val4&0x00003F00003F) | ( (val3&0x00003F00003F)<<6  ) | ( (val2&0x00003F00003F)<<12 ) | ( (val1&0x00003F00003F)<<18 );
                out[1] = ( (val4&0x000FC0000FC0)>>6  ) | (val3&0x000FC0000FC0) | ( (val2&0x000FC0000FC0)<<6  ) | ( (val1&0x000FC0000FC0)<<12 );
                out[2] = ( (val4&0x03F00003F000)>>12 ) | ( (val3&0x03F00003F000)>>6  ) | (val2&0x03F00003F000) | ( (val1&0x03F00003F000)<<6  );
                out[3] = ( (val4&0xFC0000FC0000)>>18 ) | ( (val3&0xFC0000FC0000)>>12 ) | ( (val2&0xFC0000FC0000)>>6  ) | (val1&0xFC0000FC0000);

                res = _mm512_loadu_si512(out);

                res = _mm512_permutex2var_epi16(res, vidxOut1_64QAM, res);
                res = _mm512_shuffle_epi8(res, vidxOut2_64QAM);
                res = _mm512_permutex2var_epi32(res, vidxOut3_64QAM, res);
                res = _mm512_shuffle_epi8(res, vidxOut4_64QAM);
                /* Shuffle to right order */

                _mm512_storeu_si512(output, res);

                tmp[0] += 8;
                tmp[1] += 8;
                tmp[2] += 8;
                tmp[3] += 8;
                tmp[4] += 8;
                tmp[5] += 8;
                output += 48;
            }

            if (byteTail > 0)
            {
                in[0] = *(unsigned __int64 *)(tmp[0]);
                in[1] = *(unsigned __int64 *)(tmp[1]);
                in[2] = *(unsigned __int64 *)(tmp[2]);
                in[3] = *(unsigned __int64 *)(tmp[3]);
                in[4] = *(unsigned __int64 *)(tmp[4]);
                in[5] = *(unsigned __int64 *)(tmp[5]);

                inter = _mm512_loadu_si512(in);

                inter = _mm512_permutex2var_epi16(inter, vidxIn1_64QAM, inter);
                inter = _mm512_shuffle_epi8(inter, vidxIn2_64QAM);
                inter = _mm512_permutex2var_epi16(inter, vidxIn3_64QAM, inter);

                val1 = (unsigned __int64)(_mm512_movepi8_mask(inter));
                inter = _mm512_slli_epi64(inter, 1);
                val2 = (unsigned __int64)(_mm512_movepi8_mask(inter));
                inter = _mm512_slli_epi64(inter, 1);
                val3 = (unsigned __int64)(_mm512_movepi8_mask(inter));
                inter = _mm512_slli_epi64(inter, 1);
                val4 = (unsigned __int64)(_mm512_movepi8_mask(inter));
                inter = _mm512_slli_epi64(inter, 1);

                out[4] = (val4&0x00003F00003F) | ( (val3&0x00003F00003F)<<6  ) | ( (val2&0x00003F00003F)<<12 ) | ( (val1&0x00003F00003F)<<18 );
                out[5] = ( (val4&0x000FC0000FC0)>>6  ) | (val3&0x000FC0000FC0) | ( (val2&0x000FC0000FC0)<<6  ) | ( (val1&0x000FC0000FC0)<<12 );
                out[6] = ( (val4&0x03F00003F000)>>12 ) | ( (val3&0x03F00003F000)>>6  ) | (val2&0x03F00003F000) | ( (val1&0x03F00003F000)<<6  );
                out[7] = ( (val4&0xFC0000FC0000)>>18 ) | ( (val3&0xFC0000FC0000)>>12 ) | ( (val2&0xFC0000FC0000)>>6  ) | (val1&0xFC0000FC0000);

                val1 = (unsigned __int64)(_mm512_movepi8_mask(inter));
                inter = _mm512_slli_epi64(inter, 1);
                val2 = (unsigned __int64)(_mm512_movepi8_mask(inter));
                inter = _mm512_slli_epi64(inter, 1);
                val3 = (unsigned __int64)(_mm512_movepi8_mask(inter));
                inter = _mm512_slli_epi64(inter, 1);
                val4 = (unsigned __int64)(_mm512_movepi8_mask(inter));

                out[0] = (val4&0x00003F00003F) | ( (val3&0x00003F00003F)<<6  ) | ( (val2&0x00003F00003F)<<12 ) | ( (val1&0x00003F00003F)<<18 );
                out[1] = ( (val4&0x000FC0000FC0)>>6  ) | (val3&0x000FC0000FC0) | ( (val2&0x000FC0000FC0)<<6  ) | ( (val1&0x000FC0000FC0)<<12 );
                out[2] = ( (val4&0x03F00003F000)>>12 ) | ( (val3&0x03F00003F000)>>6  ) | (val2&0x03F00003F000) | ( (val1&0x03F00003F000)<<6  );
                out[3] = ( (val4&0xFC0000FC0000)>>18 ) | ( (val3&0xFC0000FC0000)>>12 ) | ( (val2&0xFC0000FC0000)>>6  ) | (val1&0xFC0000FC0000);

                res = _mm512_loadu_si512(out);

                res = _mm512_permutex2var_epi16(res, vidxOut1_64QAM, res);
                res = _mm512_shuffle_epi8(res, vidxOut2_64QAM);
                res = _mm512_permutex2var_epi32(res, vidxOut3_64QAM, res);
                res = _mm512_shuffle_epi8(res, vidxOut4_64QAM);
                /* Shuffle to right order */

                left = (__mmask64)(tailMask[byteTail*3]);
                _mm512_mask_storeu_epi8(output, left, res);

                tmp[0] += byteTail;
                tmp[1] += byteTail;
                tmp[2] += byteTail;
                tmp[3] += byteTail;
                tmp[4] += byteTail;
                tmp[5] += byteTail;
                output += byteTail * 6;
            }

            byteOffset = 0;
            bitOffset = 0;
            for (int i=0;i < bitTail;i++)
            {
                ippsCopyBE_1u( (Ipp8u*)tmp[0], i, output+byteOffset, bitOffset+0, 1 );
                ippsCopyBE_1u( (Ipp8u*)tmp[1], i, output+byteOffset, bitOffset+1, 1 );
                bitOffset += 2;
                byteOffset += bitOffset >> 3;
                bitOffset = bitOffset & 0x7;
                ippsCopyBE_1u( (Ipp8u*)tmp[2], i, output+byteOffset, bitOffset+0, 1 );
                ippsCopyBE_1u( (Ipp8u*)tmp[3], i, output+byteOffset, bitOffset+1, 1 );
                bitOffset += 2;
                byteOffset += bitOffset >> 3;
                bitOffset = bitOffset & 0x7;
                ippsCopyBE_1u( (Ipp8u*)tmp[4], i, output+byteOffset, bitOffset+0, 1 );
                ippsCopyBE_1u( (Ipp8u*)tmp[5], i, output+byteOffset, bitOffset+1, 1 );
                bitOffset += 2;
                byteOffset += bitOffset >> 3;
                bitOffset = bitOffset & 0x7;
            }
            break;
        case 8:
            bitTail = jMax & 0x7;
            byteTail = lenInByte & 0x7;

            for (int j = 0;j <= lenInByte-8;j+=8)
            {
                in[0] = *(unsigned __int64 *)(tmp[0]);
                in[1] = *(unsigned __int64 *)(tmp[1]);
                in[2] = *(unsigned __int64 *)(tmp[2]);
                in[3] = *(unsigned __int64 *)(tmp[3]);
                in[4] = *(unsigned __int64 *)(tmp[4]);
                in[5] = *(unsigned __int64 *)(tmp[5]);
                in[6] = *(unsigned __int64 *)(tmp[6]);
                in[7] = *(unsigned __int64 *)(tmp[7]);

                inter = _mm512_loadu_si512(in);

                inter = _mm512_permutex2var_epi16(inter, vidxIn1_256QAM, inter);
                inter = _mm512_shuffle_epi8(inter, vidxIn2_256QAM);

                out[7] = (unsigned __int64)(_mm512_movepi8_mask(inter));
                inter = _mm512_slli_epi64(inter, 1);
                out[6] = (unsigned __int64)(_mm512_movepi8_mask(inter));
                inter = _mm512_slli_epi64(inter, 1);
                out[5] = (unsigned __int64)(_mm512_movepi8_mask(inter));
                inter = _mm512_slli_epi64(inter, 1);
                out[4] = (unsigned __int64)(_mm512_movepi8_mask(inter));
                inter = _mm512_slli_epi64(inter, 1);
                out[3] = (unsigned __int64)(_mm512_movepi8_mask(inter));
                inter = _mm512_slli_epi64(inter, 1);
                out[2] = (unsigned __int64)(_mm512_movepi8_mask(inter));
                inter = _mm512_slli_epi64(inter, 1);
                out[1] = (unsigned __int64)(_mm512_movepi8_mask(inter));
                inter = _mm512_slli_epi64(inter, 1);
                out[0] = (unsigned __int64)(_mm512_movepi8_mask(inter));

                res = _mm512_loadu_si512(out);
                res = _mm512_permutex2var_epi16(res, vidxOut1_256QAM, res);
                res = _mm512_shuffle_epi8(res, vidxOut2_256QAM);
                /* Shuffle to right order */

                _mm512_storeu_si512(output, res);

                tmp[0] += 8;
                tmp[1] += 8;
                tmp[2] += 8;
                tmp[3] += 8;
                tmp[4] += 8;
                tmp[5] += 8;
                tmp[6] += 8;
                tmp[7] += 8;
                output += 64;
            }

            if (byteTail > 0)
            {
                in[0] = *(unsigned __int64 *)(tmp[0]);
                in[1] = *(unsigned __int64 *)(tmp[1]);
                in[2] = *(unsigned __int64 *)(tmp[2]);
                in[3] = *(unsigned __int64 *)(tmp[3]);
                in[4] = *(unsigned __int64 *)(tmp[4]);
                in[5] = *(unsigned __int64 *)(tmp[5]);
                in[6] = *(unsigned __int64 *)(tmp[6]);
                in[7] = *(unsigned __int64 *)(tmp[7]);

                inter = _mm512_loadu_si512(in);

                inter = _mm512_permutex2var_epi16(inter, vidxIn1_256QAM, inter);
                inter = _mm512_shuffle_epi8(inter, vidxIn2_256QAM);

                out[7] = (unsigned __int64)(_mm512_movepi8_mask(inter));
                inter = _mm512_slli_epi64(inter, 1);
                out[6] = (unsigned __int64)(_mm512_movepi8_mask(inter));
                inter = _mm512_slli_epi64(inter, 1);
                out[5] = (unsigned __int64)(_mm512_movepi8_mask(inter));
                inter = _mm512_slli_epi64(inter, 1);
                out[4] = (unsigned __int64)(_mm512_movepi8_mask(inter));
                inter = _mm512_slli_epi64(inter, 1);
                out[3] = (unsigned __int64)(_mm512_movepi8_mask(inter));
                inter = _mm512_slli_epi64(inter, 1);
                out[2] = (unsigned __int64)(_mm512_movepi8_mask(inter));
                inter = _mm512_slli_epi64(inter, 1);
                out[1] = (unsigned __int64)(_mm512_movepi8_mask(inter));
                inter = _mm512_slli_epi64(inter, 1);
                out[0] = (unsigned __int64)(_mm512_movepi8_mask(inter));

                res = _mm512_loadu_si512(out);
                res = _mm512_permutex2var_epi16(res, vidxOut1_256QAM, res);
                res = _mm512_shuffle_epi8(res, vidxOut2_256QAM);
                /* Shuffle to right order */

                left = (__mmask64)(tailMask[byteTail*4]);
                _mm512_mask_storeu_epi8(output, left, res);

                tmp[0] += byteTail;
                tmp[1] += byteTail;
                tmp[2] += byteTail;
                tmp[3] += byteTail;
                tmp[4] += byteTail;
                tmp[5] += byteTail;
                tmp[6] += byteTail;
                tmp[7] += byteTail;
                output += byteTail * 8;
            }

            for (int i=0;i < bitTail;i++)
            {
                ippsCopyBE_1u( (Ipp8u*)tmp[0], i, output, 0, 1 );
                ippsCopyBE_1u( (Ipp8u*)tmp[1], i, output, 1, 1 );
                ippsCopyBE_1u( (Ipp8u*)tmp[2], i, output, 2, 1 );
                ippsCopyBE_1u( (Ipp8u*)tmp[3], i, output, 3, 1 );
                ippsCopyBE_1u( (Ipp8u*)tmp[4], i, output, 4, 1 );
                ippsCopyBE_1u( (Ipp8u*)tmp[5], i, output, 5, 1 );
                ippsCopyBE_1u( (Ipp8u*)tmp[6], i, output, 6, 1 );
                ippsCopyBE_1u( (Ipp8u*)tmp[7], i, output, 7, 1 );
                output++;
            }
            break;
        default:
            printf("Wrong parameter for modulation type. It should be 1/2/4/6/8\n");
            exit(-1);
            break;
    }
}

//-------------------------------------------------------------------------------------------
/**
 *  @brief rate matching for LDPC in 5GNR.
 *  @param [in] request Structure containing configuration information and input data.
 *  @param [out] response Structure containing kernel outputs.
 *  @return Success: return 0, else: return -1.
**/
int32_t bblib_LDPC_ratematch_5gnr_avx512(const struct bblib_LDPC_ratematch_5gnr_request *request, struct bblib_LDPC_ratematch_5gnr_response *response)
{
    uint8_t e[MAX_LENGTH];
    memset(e, 0, MAX_LENGTH);

    bitSelect(request, e);

    bitInterleave_avx512(request, response, e);
    return 0;
}
