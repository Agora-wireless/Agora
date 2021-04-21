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
 * @file   rmatching_5gnr_c.cpp
 * @brief  Source code of External API for scrambling and descrambling functions
*/
#include <cstdio>
#include <cstdint>
#include <functional>

//#include "phy_LDPC_ratematch_5gnr.h"
#include "sdk_version.h"
#include "rmatching_5gnr_c.h"
#include <string.h>

constexpr auto MAX_LENGTH=8448;


// Return the number of bytes needed to store n_bits bits
static inline size_t bits_to_bytes(size_t n_bits) { return (n_bits + 7) / 8; }

//! @{
/*! \brief Copy contents of one bit vector into another.
    \param [in] pSrc pointer
    \param [in] len in bits
    \param [out] pDest Temporary pointer to the output after bit slection.

*/

void memcpy_bitOffset(uint8_t* pSrc, int32_t srcBitOffset, uint8_t* pDest, int32_t dstBitOffset, int32_t len)
{
    // FIXME - "Complete" but not fully tested
    size_t nbytes = bits_to_bytes(len);
    uint32_t mask = ((1U << dstBitOffset) - 1U) << (8-dstBitOffset);

    if (srcBitOffset == 0)
    {
        for (size_t i = 0; i < nbytes; ++i)
        {
            uint8_t srcBits = (pSrc[i] >> dstBitOffset);
            pDest[i] = (pDest[i] & mask) | srcBits;

            uint8_t bitLeft = pSrc[i] << (8 - dstBitOffset);
            
            if (dstBitOffset > 0)
                pDest[i+1] = bitLeft;
        }        
    }
    else if (len > 0)
    {
        uint8_t v0 = pSrc[0];
        uint8_t tmp[nbytes];
        for (size_t i = 0; i < nbytes; ++i)
        {
            uint8_t v1 = pSrc[i + 1];
            tmp[i] = (v0 << srcBitOffset) | (v1 >> (8 - srcBitOffset));
            v0 = v1;

            uint8_t srcBits = (tmp[i] >> dstBitOffset);
            pDest[i] = (pDest[i] & mask) | srcBits;

            uint8_t bitLeft = tmp[i] << (8 - dstBitOffset);

            if (dstBitOffset > 0)
                pDest[i+1] = bitLeft;
        }
    }
}

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
            // memcpy_bitOffset( (in + srcByte), srcBits, (e + desByte), desBits, bitLeft );
            srcOffBit += bitLeft;
        }
        /* If data between start bit and null bit is not enough for bit select, copy all bits between them and copy remain bits from end of null bits */
        else if ((startBit < ni) && (startBit + bitLeft + nl <= cb))
        {
            ippsCopyBE_1u( (Ipp8u*)(in+srcByte), srcBits, (Ipp8u*)(e+desByte), desBits, ni-startBit );
            // memcpy_bitOffset( (in + srcByte), srcBits, (e + desByte), desBits, ni-startBit );
            desOffBit += ni - startBit;
            desByte = desOffBit >> 3;
            desBits = desOffBit & 0x7;
            srcOffBit = ni + nl;
            srcByte = srcOffBit >> 3;
            srcBits = srcOffBit & 0x7;

            ippsCopyBE_1u( (Ipp8u*)(in+srcByte), srcBits, (Ipp8u*)(e+desByte), desBits, bitLeft-ni+startBit );
            // memcpy_bitOffset( (in + srcByte), srcBits, (e + desByte), desBits, bitLeft-ni+startBit );
            srcOffBit += bitLeft-ni+startBit;
        }
        /* If data between start bit and null bit is not enough for bit select, copy all bits between them and copy remain bits from end of null bits
           If we reach the end of input, get back to the beginning and copy all except null bits until we get enough bits */
        else if ( startBit<ni )
        {
            ippsCopyBE_1u( (Ipp8u*)(in+srcByte), srcBits, (Ipp8u*)(e+desByte), desBits, ni-startBit );
            // memcpy_bitOffset( (in + srcByte), srcBits, (e + desByte), desBits, ni-startBit );
            desOffBit += ni - startBit;
            desByte = desOffBit >> 3;
            desBits = desOffBit & 0x7;
            srcOffBit = ni+nl;
            srcByte = srcOffBit >> 3;
            srcBits = srcOffBit & 0x7;

            ippsCopyBE_1u( (Ipp8u*)(in+srcByte), srcBits, (Ipp8u*)(e+desByte), desBits, cb-ni-nl );
            // memcpy_bitOffset( (in + srcByte), srcBits, (e + desByte), desBits, cb-ni-nl );
            desOffBit += cb - ni - nl;
            desByte = desOffBit >> 3;
            desBits = desOffBit & 0x7;
            bitLeft -= cb - startBit - nl;

            while (bitLeft > 0)
            {
                if ( bitLeft>ni )
                {
                    ippsCopyBE_1u( (Ipp8u*)in, 0, (Ipp8u*)(e+desByte), desBits, ni );
                    // memcpy_bitOffset( (in + srcByte), srcBits, (e + desByte), desBits, ni );
                    bitLeft -= ni;
                    desOffBit += ni;
                    desByte = desOffBit >> 3;
                    desBits = desOffBit & 0x7;
                }
                else
                {
                    ippsCopyBE_1u( (Ipp8u*)in, 0, (Ipp8u*)(e+desByte), desBits, bitLeft );
            	    // memcpy_bitOffset( (in + srcByte), srcBits, (e + desByte), desBits, bitLeft );
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
		            // memcpy_bitOffset( (in + srcByte), srcBits, (e + desByte), desBits, cb-ni-nl );
                    bitLeft -= cb - ni - nl;
                    desOffBit += cb - ni - nl;
                    desByte = desOffBit >> 3;
                    desBits = desOffBit & 0x7;
                }
                else
                {
                    ippsCopyBE_1u( (Ipp8u*)(in+srcByte), srcBits, (Ipp8u*)(e+desByte), desBits, bitLeft );
		            // memcpy_bitOffset( (in + srcByte), srcBits, (e + desByte), desBits, bitLeft );
                    srcOffBit += bitLeft;
                    bitLeft = 0;
                    break;
                }
            }
        }
        /* If we start at the end of null bits, and the remaining bits is enough */
        else if (startBit+bitLeft<=cb)
        {
            // printf("==== OBCH 3 ==== srcB: %d, srcb: %d, dstB: %d, dstb: %d, bitLeft: %d, len: %d \n ", srcByte, srcBits, desByte, desBits, bitLeft, bitLeft);
            ippsCopyBE_1u( (Ipp8u*)(in+srcByte), srcBits, (Ipp8u*)(e+desByte), desBits, bitLeft );
	        // memcpy_bitOffset( (in + srcByte), srcBits, (e + desByte), desBits, bitLeft );
            srcOffBit += bitLeft;
        }
        /* If we start at the end of null bits, and the remaining bits is not enough, we get back to the beginning and copy all except null bits until we get enough bits */
        else
        {
            ippsCopyBE_1u( (Ipp8u*)(in+srcByte), srcBits, (Ipp8u*)(e+desByte), desBits, cb-startBit );
	        // memcpy_bitOffset( (in + srcByte), srcBits, (e + desByte), desBits, cb-startBit );
            bitLeft -= cb-startBit;
            desOffBit += cb-startBit;
            desByte = desOffBit >> 3;
            desBits = desOffBit & 0x7;

            while (bitLeft > 0)
            {
                if ( bitLeft>ni )
                {
                    ippsCopyBE_1u( (Ipp8u*)in, 0, (Ipp8u*)(e+desByte), desBits, ni );
		            // memcpy_bitOffset( (in + srcByte), srcBits, (e + desByte), desBits, ni );
                    bitLeft -= ni;
                    desOffBit += ni;
                    desByte = desOffBit >> 3;
                    desBits = desOffBit & 0x7;
                }
                else
                {
                    ippsCopyBE_1u( (Ipp8u*)in, 0, (Ipp8u*)(e+desByte), desBits, bitLeft );
		            // memcpy_bitOffset( (in + srcByte), srcBits, (e + desByte), desBits, bitLeft );
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
		            // memcpy_bitOffset( (in + srcByte), srcBits, (e + desByte), desBits, cb-ni-nl );
                    bitLeft -= cb - ni - nl;
                    desOffBit += cb - ni - nl;
                    desByte = desOffBit >> 3;
                    desBits = desOffBit & 0x7;
                }
                else
                {
                    ippsCopyBE_1u( (Ipp8u*)(in+srcByte), srcBits, (Ipp8u*)(e+desByte), desBits, bitLeft );
		            // memcpy_bitOffset( (in + srcByte), srcBits, (e + desByte), desBits, bitLeft );
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
    \param [in] e Temporary pointer to the output after bit slection, as input to bit interleaver.
*/
inline void bitInterleave(const struct bblib_LDPC_ratematch_5gnr_request *request, struct bblib_LDPC_ratematch_5gnr_response *response, uint8_t *e)
{
    // TODO: Interleave bits instead of bytes
    int32_t EB = request->E / 8;
    int32_t Q = request->Qm;
 
    // Specified in TS 38212-5.4.2.2
    for (int32_t byte = 0; byte < EB/Q; byte++)
    {
        for (int32_t mod = 0; mod < Q; mod++)
        {
	        response->output[mod+byte*Q] = e[mod*(EB/Q)+byte];
	    }
    }
}



//-------------------------------------------------------------------------------------------
/**
 *  @brief rate matching for LDPC in 5GNR.
 *  @param [in] request Structure containing configuration information and input data.
 *  @param [out] response Structure containing kernel outputs.
 *  @return Success: return 0, else: return -1.
**/
int32_t bblib_LDPC_ratematch_5gnr_c(const struct bblib_LDPC_ratematch_5gnr_request *request, struct bblib_LDPC_ratematch_5gnr_response *response)
{
    uint8_t e[MAX_LENGTH];
    memset(e, 0, MAX_LENGTH);

    bitSelect(request, e);
    bitInterleave(request, response, e);
    return 0;
}

