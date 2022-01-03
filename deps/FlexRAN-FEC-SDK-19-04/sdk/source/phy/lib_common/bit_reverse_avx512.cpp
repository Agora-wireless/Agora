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
 * @file   bit_reverse_avx512.cpp
 * @brief  Source code of conversion between float and int16, with agc gain, with AVX512 instructions.
*/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <malloc.h>
#include <stdint.h>

#include <immintrin.h>  /* AVX */

#include "bit_reverse.h"

#if defined (_BBLIB_AVX512_)

static inline __m512i BitBlend (__m512i a, __m512i b, int8_t choose) {
    return _mm512_ternarylogic_epi32(a, b, _mm512_set1_epi8(choose), 0xd8);
}

static inline __m512i shr(__m512i a, int offset) { return _mm512_srli_epi16(a, offset); }
static inline __m512i shl(__m512i a, int offset) { return _mm512_slli_epi16(a, offset); }

static __m512i bitInByteRev(__m512i bytes)
{
    const auto swapNibbles = BitBlend(shr(bytes, 4),       shl(bytes, 4),       0b11110000);
    const auto swapPairs =   BitBlend(shr(swapNibbles, 2), shl(swapNibbles, 2), 0b11001100);
    const auto swapBits =    BitBlend(shr(swapPairs, 1),   shl(swapPairs, 1),   0b10101010);
    return swapBits;
}


//! @{
/*! \brief Bit Reversion.
    \param [in] input Input buffer
    \param [in] num_data Number of data for conversion.
    \param [out] output Output buffer
    \return Return 0 for success, and -1 for error.
    \note Input/output is aligned with 512 bits. Only handles the tail processing
*/
void bblib_bit_reverse_avx512(int8_t* pInOut, int32_t bitLen)
{
    uint32_t bitmod512, bitDiv512;
    __m512i * pInOutOffset512 = (__m512i *)pInOut;
    __m512i temp;
    bitDiv512 = bitLen >> 9;
    bitmod512 = (bitLen - (bitDiv512 << 9) + 7) >> 3;
    for (uint32_t i = 0; i < bitDiv512; i++) {
        *pInOutOffset512 = bitInByteRev(*pInOutOffset512);
        pInOutOffset512++;
    }
    if (bitmod512 != 0) {
        temp = bitInByteRev(*pInOutOffset512);
        const auto tail_mask = ((uint64_t) 1 << bitmod512 ) - 1;
        _mm512_mask_storeu_epi8 (pInOut + (bitDiv512 << 6),
                tail_mask, temp);
    }
    return;
}

#else
void bblib_bit_reverse_avx512(int8_t* output, int32_t num_data)
{
    printf("This version of bblib_common requires AVX2 ISA support to run\n");
    exit(-1);
}

#endif
