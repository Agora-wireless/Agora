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
 * @file   float_int16_convert_agc_avx512.cpp
 * @brief  Source code of conversion between float and int16, with agc gain, with AVX512 instructions.
*/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <malloc.h>
#include <stdint.h>

#include <immintrin.h>  /* AVX */

#include "float_int16_convert_agc.h"

#if defined (_BBLIB_AVX512_)
#define LEN_PACKET 512
#define LEN_FLOAT 32
#define LEN_INT16 16
#define NFLOAT_IN_PACKET (LEN_PACKET/LEN_FLOAT)
#define NINT16_IN_PACKET (LEN_PACKET/LEN_INT16)

/**
 * @brief conversion from float to int16, with float gain, with AVX512 instructions
 * @param[in] input input buffer for float
 * @param[in] num_data number of data for conversion
 * @param[in] gain gain for agc
 * @param[out] output output buffer for int16
 * @note input and output is aligned with 512 bits
 * @note (input data*gain) should be in the rage of -32768~32767, for the range of int16
 */
void bblib_float_to_int16_agc_avx512( int16_t* output, float* input, int32_t num_data, float gain )
{
    /* temp variable for conversion */
    __m512 temp_float;
    __m512i temp_int32;
    __m256i temp_int16;
    __m256i temp_int16_shuffle;

    /* index for shuffle, after int32-->int16 packing */
    __m256i index_shuffle = _mm256_setr_epi16(0,1,2,3,8,9,10,11,4,5,6,7,12,13,14,15);

    /* float gain vector  */
    __m512 temp_gain = _mm512_set1_ps(gain);

    /* number of loops in instruction operation */
    int32_t num_loop = num_data/NFLOAT_IN_PACKET;

    /* pointer and temp variable for switch of 2 * 256 bit vector in loop */
    __m256i* pm256i_0 = (__m256i*)(&temp_int32);
    __m256i* pm256i_1 = pm256i_0+1;

    /* temp for result after gain out of loop */
    float temp_mul = 0;

    /* deal with loop with 256 bits bus instructions */
    for( int32_t index_loop = 0; index_loop<num_loop; index_loop++ )
    {
       /* pick 256 bits from input float buffer, multiplex with gain, and convert to int32 */
       temp_float = _mm512_load_ps( input+index_loop*NFLOAT_IN_PACKET );
       temp_float = _mm512_mul_ps( temp_float, temp_gain );
       temp_int32 = _mm512_cvtps_epi32( temp_float );

       /* pack into int16 vector, and come back to the original order */
       temp_int16 = _mm256_packs_epi32(*pm256i_0, *pm256i_1);
       temp_int16_shuffle = _mm256_permutexvar_epi16( index_shuffle, temp_int16 );
       /* store to output int16 buffer */
       _mm256_store_si256((__m256i *)(output+index_loop*NFLOAT_IN_PACKET), temp_int16_shuffle);
    }

    /* deal with data in tail, out of loop */
    for( int32_t index_tail = num_loop*NFLOAT_IN_PACKET; index_tail<num_data; index_tail++ )
    {
        temp_mul = input[index_tail]*gain;
        if( temp_mul > 32767 )
            temp_mul = 32767;
        if( temp_mul < -32768 )
            temp_mul = -32768;

        output[index_tail] = (int16_t)temp_mul;
    }

    return;
}

/**
 * @brief conversion from float to int16, with float gain and int16 threshold, with AVX2 instructions
 * @param[in] input input buffer for float
 * @param[in] num_data number of data for conversion
 * @param[in] gain gain for agc
 * @param[in] threshold threshold after agc, which should be >=0
 * @param[out] output output buffer for int16
 * @return 0 for success, and -1 for error
 * @note input and output is aligned with 512 bits
 */
int16_t bblib_float_to_int16_agc_threshold_avx512( int16_t* output, float* input, int32_t num_data, float gain, int16_t threshold )
{
    /* temp variable for conversion */
    __m512 temp_float;
    __m512i temp_int32;
    __m256i temp_int16_preshuffle;
    __m256i temp_int16;

    /* index for shuffle, after int32-->int16 packing */
    __m256i index_shuffle = _mm256_setr_epi16(0,1,2,3,8,9,10,11,4,5,6,7,12,13,14,15);

    /* float gain vector  */
    __m512 temp_gain = _mm512_set1_ps(gain);

    /* number of loops in instruction operation */
    int32_t num_loop = num_data/NFLOAT_IN_PACKET;

    /* pointer and temp variable for switch of 2 * 256 bit vector in loop */
    __m256i* pm256i_0 = (__m256i*)(&temp_int32);
    __m256i* pm256i_1 = pm256i_0+1;

    /* negative threshold */
    int16_t neg_threshold = -1*threshold;

    /* Compare result with threshold and negative threshold */
    __m256i cmp_result_threshold;
    __m256i cmp_result_neg_threshold;

    /* temp for result after gain out of loop */
    float temp_mul = 0;

    if( threshold < 0 )
    {
        printf( "threshold in function float_to_int16_agc_threshold_avx2 should be >=0" );
        return -1;
    }

    if( num_data <= 0 )
    {
        printf( "num_data in function float_to_int16_agc_threshold_avx2 should be >0" );
        return -1;
    }

    /* deal with loop with 256 bits bus instructions */
    for( int32_t index_loop = 0; index_loop<num_loop; index_loop++ )
    {
        /* init temp_threshold & temp_neg_threshold in each rotation*/
        __m256i temp_threshold = _mm256_set1_epi16(threshold);
        __m256i temp_neg_threshold = _mm256_set1_epi16(neg_threshold);

       /* pick 256 bits from input float buffer, multiplex with gain, and convert to int32 */
       temp_float = _mm512_load_ps( input+index_loop*NFLOAT_IN_PACKET );
       temp_float = _mm512_mul_ps( temp_float, temp_gain );
       temp_int32 = _mm512_cvtps_epi32( temp_float );

       /* pack into int16 vector, and come back to the original order */
       temp_int16_preshuffle = _mm256_packs_epi32 (*pm256i_0, *pm256i_1);
       temp_int16 = _mm256_permutexvar_epi16( index_shuffle, temp_int16_preshuffle );

       /* compare with threshold and neg threshold */
       cmp_result_threshold = _mm256_cmpgt_epi16( temp_int16, temp_threshold );
       temp_threshold = _mm256_and_si256( cmp_result_threshold, temp_threshold );
       temp_int16 = _mm256_andnot_si256( cmp_result_threshold, temp_int16 );

       cmp_result_neg_threshold = _mm256_cmpgt_epi16 ( temp_neg_threshold, temp_int16 );
       temp_neg_threshold = _mm256_and_si256( cmp_result_neg_threshold, temp_neg_threshold );
       temp_int16 = _mm256_andnot_si256( cmp_result_neg_threshold, temp_int16 );

       temp_int16 = _mm256_or_si256( temp_int16, temp_threshold );
       temp_int16 = _mm256_or_si256( temp_int16, temp_neg_threshold );

       /* store to output int16 buffer */
       _mm256_store_si256((__m256i *)(output+index_loop*NFLOAT_IN_PACKET), temp_int16);
    }

    /* deal with data in tail, out of loop */
    for( int32_t index_tail = num_loop*NFLOAT_IN_PACKET; index_tail<num_data; index_tail++ )
    {
        temp_mul = input[index_tail]*gain;
        if( temp_mul > threshold )
            temp_mul = threshold;
        if( temp_mul < neg_threshold )
            temp_mul = neg_threshold;

        output[index_tail] = (int16_t)temp_mul;
    }

    return 0;
}

/**
 * @brief conversion from int16 to float, with float gain, with AVX512 instructions
 * @param[in] input input buffer for int16
 * @param[in] num_data number of data for conversion
 * @param[in] gain gain for agc
 * @param[out] output output buffer for float
 * @note input and output is aligned with 512 bits
 */
void bblib_int16_to_float_agc_avx512( float* output, int16_t* input, int32_t num_data, float gain )
{
    /* temp variable for conversion */
    __m256i temp_int16;
    __m512i temp_int32;
    __m512 temp_float;

    /* float gain vector  */
    __m512 temp_gain = _mm512_set1_ps(gain);

    /* number of loops in instruction operation */
    int32_t num_loop = num_data/NFLOAT_IN_PACKET;

    /* deal with loop with 256 bits bus instructions */
    for( int32_t index_loop = 0; index_loop<num_loop; index_loop++ )
    {
       //pick 256 bits from input int16 buffer, convert to int32, float, finally multiplex with gain
       temp_int16 = _mm256_load_si256( (__m256i*)(input+index_loop*NFLOAT_IN_PACKET) );
       temp_int32 = _mm512_cvtepi16_epi32(temp_int16);
       temp_float = _mm512_cvtepi32_ps(temp_int32);
       temp_float = _mm512_mul_ps( temp_float, temp_gain );

       /* store to output int16 buffer */
       _mm512_store_ps( output+index_loop*NFLOAT_IN_PACKET, temp_float );
    }

    /* deal with data in tail, out of loop */
    for( int32_t index_tail = num_loop*NFLOAT_IN_PACKET; index_tail<num_data; index_tail++ )
    {
        output[index_tail] = ((float)(input[index_tail]))*gain;
    }

    return;
}

/**
* @brief conversion from int16 to int16, with float gain, with AVX512 instructions
* @param[in] input input buffer for int16
* @param[in] num_data number of data for conversion
* @param[in] gain gain for agc
* @param[out] output output buffer for int16
* @note input and output is aligned with 512 bits
* @note (input data*gain) should be in the rage of -32768~32767, for the range of int16
*/
void bblib_int16_to_int16_agc_avx512(int16_t* output, int16_t* input, int32_t num_data, float gain)
{
    /* temp variable for conversion */
    __m256i temp_int16_in;
    __m512 temp_float;
    __m512i temp_int32;
    __m256i temp_int16_out;
    __m256i temp_int16_shuffle_out;

    /* index for shuffle, after int32-->int16 packing */
    __m256i index_shuffle = _mm256_setr_epi16(0,1,2,3,8,9,10,11,4,5,6,7,12,13,14,15);

    /* float gain vector  */
    __m512 temp_gain = _mm512_set1_ps(gain);

    /* number of loops in instruction operation */
    int32_t num_loop = num_data / NFLOAT_IN_PACKET;

    /* pointer and temp variable for switch of 2 * 256 bit vector in loop */
    __m256i* pm256i_0 = (__m256i*)(&temp_int32);
    __m256i* pm256i_1 = pm256i_0 + 1;

    /* temp for result after gain out of loop */
    float temp_mul = 0;

    /* deal with loop with 256 bits bus instructions */
    for (int32_t index_loop = 0; index_loop<num_loop; index_loop++)
    {
        //pick 128 bits from input int16 buffer, convert to int32, float, finally multiplex with gain
        temp_int16_in = _mm256_load_si256((__m256i*)(input + index_loop*NFLOAT_IN_PACKET));
        temp_int32 = _mm512_cvtepi16_epi32(temp_int16_in);
        temp_float = _mm512_cvtepi32_ps(temp_int32);
        temp_float = _mm512_mul_ps(temp_float, temp_gain);

        /* convert float to int32 */
        temp_int32 = _mm512_cvtps_epi32(temp_float);

       /* pack into int16 vector, and come back to the original order */
       temp_int16_out = _mm256_packs_epi32 (*pm256i_0, *pm256i_1);
       temp_int16_shuffle_out = _mm256_permutexvar_epi16( index_shuffle, temp_int16_out );
       /* store to output int16 buffer */
       _mm256_store_si256((__m256i *)(output+index_loop*NFLOAT_IN_PACKET), temp_int16_shuffle_out);
    }

    /* deal with data in tail, out of loop */
    for (int32_t index_tail = num_loop*NFLOAT_IN_PACKET; index_tail<num_data; index_tail++)
    {
        temp_mul = (float)(input[index_tail]) * gain;
        if (temp_mul > 32767)
            temp_mul = 32767;
        if (temp_mul < -32768)
            temp_mul = -32768;

        output[index_tail] = (int16_t)temp_mul;
    }

    return;
}

void bblib_int16_to_int16_fxp_scale_avx512(int16_t *scaleOut, int16_t *scaleIn, int32_t num_samples, int16_t scale16)
{
    /* number of loops in instruction operation */
    int32_t num_loop = num_samples / NINT16_IN_PACKET;
    __m512i temp_gain = _mm512_set1_epi16(scale16);

    /* deal with loop with 512 bits bus instructions */
    for (int32_t index_loop = 0; index_loop<num_loop; index_loop++)
    {
        //pick 512 bits from input int16 buffer
        __m512i temp_int16_in = _mm512_load_si512((__m512i*)(scaleIn + index_loop*NINT16_IN_PACKET));
        // multiply with scale16 and >>16
        __m512i temp_int16_out = _mm512_mulhi_epi16( temp_int16_in, temp_gain );
        // store to output int16 buffer
        _mm512_store_si512((__m512 *)(scaleOut+index_loop*NINT16_IN_PACKET), temp_int16_out);
    }

    // deal with tail data
    if( num_samples > num_loop*NINT16_IN_PACKET )
    {
        for (int32_t index_tail = num_loop*NFLOAT_IN_PACKET; index_tail<num_samples; index_tail++)
        {
            int32_t temp = scaleIn[index_tail] * scale16;
            scaleOut[index_tail] = (int16_t)(temp >> 16);
        }
    }

    return;
}

#else
void bblib_float_to_int16_agc_avx512( int16_t* output, float* input, int32_t num_data, float gain )
{
    printf("This version of bblib_common requires AVX2 ISA support to run\n");
    exit(-1);
}
int16_t bblib_float_to_int16_agc_threshold_avx512( int16_t* output, float* input, int32_t num_data, float gain, int16_t threshold )
{
    printf("This version of bblib_common requires AVX2 ISA support to run\n");
    exit(-1);
}
void bblib_int16_to_float_agc_avx512( float* output, int16_t* input, int32_t num_data, float gain )
{
    printf("This version of bblib_common requires AVX2 ISA support to run\n");
    exit(-1);
}
void bblib_int16_to_int16_agc_avx512(int16_t* output, int16_t* input, int32_t num_data, float gain)
{
    printf("This version of bblib_common requires AVX2 ISA support to run\n");
    exit(-1);
}

void bblib_int16_to_int16_fxp_scale_avx512(int16_t *scaleOut, int16_t *scaleIn, int32_t num_samples, int16_t scale16)
{
    printf("This version of bblib_common requires AVX512 ISA support to run\n");
    exit(-1);
}
#endif
