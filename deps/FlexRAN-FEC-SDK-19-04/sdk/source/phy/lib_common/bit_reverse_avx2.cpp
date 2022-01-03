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
 * @file   bit_reverse_avx2.cpp
 * @brief  Source code of conversion between float and int16, with agc gain, with AVX2 instructions.
*/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <malloc.h>
#include <stdint.h>
#include <immintrin.h>  /* AVX */
#include "bit_reverse.h"

#if defined (_BBLIB_AVX2_) || defined (_BBLIB_AVX512_)

// reverse bits in each byte of 256 bit register
static inline __m256i reverse_bits_in_byte(const __m256i x)
{
    const auto luthigh = _mm256_setr_epi8(0, 8, 4, 12, 2, 10, 6, 14, 1, 9, 5, 13, 3, 11, 7, 15,
                                          0, 8, 4, 12, 2, 10, 6, 14, 1, 9, 5, 13, 3, 11, 7, 15);

    const auto lutlow  = _mm256_slli_epi16(luthigh, 4);
    const auto lowmask = _mm256_set1_epi8(15);
    const auto high    = _mm256_shuffle_epi8(lutlow, _mm256_and_si256(x, lowmask));
    const auto low     = _mm256_shuffle_epi8(luthigh, _mm256_and_si256(_mm256_srli_epi16(x, 4), lowmask));
    return _mm256_or_si256(low, high);
}

//! @{
/*! \brief Bit Reversion.
    \param [in] input Input buffer
    \param [in] num_data Number of data for conversion.
    \param [out] output Output buffer
    \return Return 0 for success, and -1 for error.
    \note Input and output is aligned with 512 bits.
*/
void bblib_bit_reverse_avx2(int8_t* pInOut, int32_t bitLen)
{
    uint32_t bitDiv32, bitMod32, byte;
    __m256i *pInOutOffset = (__m256i *)pInOut;
    uint8_t *pTail, *pTmp;
    __m256i temp;
    bitDiv32 = bitLen >> 5;
    bitMod32 = (bitLen - (bitDiv32 << 5) + 7) >> 3;
    for(uint32_t i = 0; i < bitDiv32; i ++) {
        *pInOutOffset = reverse_bits_in_byte(*pInOutOffset);
        pInOutOffset++;
    }
    if(bitMod32!=0) {
        temp = reverse_bits_in_byte(*pInOutOffset);
        pTail = (uint8_t *)pInOutOffset;
        pTmp = (uint8_t *) &temp;
        for (byte = 0; byte < bitMod32; byte++)
            pTail[byte] = pTmp[byte];
    }
    return;
}


#else
void bblib_bit_reverse_avx2(int8_t* output, int32_t num_data)
{
    printf("This version of bblib_common requires AVX2 ISA support to run\n");
    exit(-1);
}

#endif
