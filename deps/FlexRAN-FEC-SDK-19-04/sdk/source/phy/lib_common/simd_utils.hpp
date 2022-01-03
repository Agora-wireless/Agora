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

#pragma once

#include <math.h>
#include <stdlib.h>
#include <immintrin.h>
#include <dvec.h>

#include <type_traits>
#include "common_typedef_sdk.h"
#include "common_typedef_simd.hpp"

/// These functions pack and unpack data from N sequential blocks of data into a single block of
/// data where each element is Nx bigger, or vice versa. That is, given data which is laid out as:
///
///    aaaaaaaabbbbbbbbccccccccddddddddeeeeeeeeffffffffgggggggghhhhhhhh
///
///  All the first elements are gathered together, all the second elements and so on:
///
///    abcdefghabcdefghabcdefghabcdefghabcdefghabcdefghabcdefghabcdefgh
///
/// The UnpackSimdN reverse the operation, splitting the packed blocks of elements back into their
/// respective positions in the original blocks.
///
/// These functions are used to allow kernels which operate on SIMD8 packed data to transform the
/// data to and from more regular blocks of sequential data.
///
/// \param input The original input data. This will be unmodified.
/// \param output The output data. This will be written to.
/// \param numElements The total number of elements in the data sets. This must be divisible by the SIMD vector length (4,8,16).
///@{


template<typename T>
void PackSimd8(const T *input, T *output, size_t numElements);

template<typename T>
void UnpackSimd8(const T *input, T *output, size_t numElements);


template<typename T>
void PackSimd16(const T *input, T *output, size_t numElements);

template<typename T>
void UnpackSimd16(const T *input, T *output, size_t numElements);
///@}

/// Copy the inverted sign of the `from' values into the `to' values.
inline F32vec16 CopyInvertedSign(F32vec16 from, F32vec16 to) {
    const auto t = _mm512_ternarylogic_epi32(_mm512_castps_si512(from),
                                             _mm512_castps_si512(to),
                                             _mm512_set1_epi32(0x80000000),
                                             0x4e);
    return _mm512_castsi512_ps(t);
}

inline F32vec8 CopyInvertedSign(F32vec8 from, F32vec8 to) {
    return _mm256_or_ps(_mm256_andnot_ps(from, _mm256_set1_ps(-0.0f)),
                        _mm256_andnot_ps(_mm256_set1_ps(-0.0f), to));
}

inline I16vec32 CopyInvertedSign(I16vec32 from, I16vec32 to) {
    return _mm512_mask_subs_epi16(to, _mm512_cmpgt_epi16_mask(from, _mm512_setzero_si512()),
                                      _mm512_setzero_si512(), to);
}

inline I16vec16 CopyInvertedSign(I16vec16 from, I16vec16 to) {
    const auto mask = _mm256_cmpgt_epi16(from, _mm256_setzero_si256());
    const auto negate = _mm256_subs_epi16(_mm256_setzero_si256(), to);
    return _mm256_blendv_epi8(to, negate, mask);
}

/// Utility function to compute the negative of the absolute value of all elements. Any positive
/// values are negated, and any negative values are left unchanged.
inline F32vec16 NegativeAbs(F32vec16 value) {
    return _mm512_or_ps(_mm512_set1_ps(-0.0f), value);
}

inline F32vec8 NegativeAbs(F32vec8 value) {
    return _mm256_or_ps(_mm256_set1_ps(-0.0f), value);
}

inline I16vec32 NegativeAbs(I16vec32 value) {
    return _mm512_sub_epi16(_mm512_setzero_si512(), _mm512_abs_epi16(value));
}

inline I16vec16 NegativeAbs(I16vec16 value) {
    return _mm256_sub_epi16(_mm256_setzero_si256(), _mm256_abs_epi16(value));
}



/*! \brief common function to store register using a mask
 *
 * Works for both AVX2 and AVX512, fixed point and single precision float.
 * For AVX2 requires more operations, especially for fixed point.
 */
inline void store_mask(I16vec32 * mem_addr, __mmask32 mask, I16vec32 a) 
{ 
    _mm512_mask_storeu_epi16((void*)mem_addr, mask, a);
    return;
}
inline void store_mask(I16vec16 * mem_addr, __mmask32 mask, I16vec16 a)
{
    auto dest = _mm256_load_si256(( __m256i*)mem_addr);
    __m256i vec_mask = _mm256_set_epi8((char)(mask >> 8), (char)(mask >> 8), (char)(mask >> 7), (char)(mask >> 7),
                                       (char)(mask >> 6), (char)(mask >> 6), (char)(mask >> 5), (char)(mask >> 5),
                                       (char)(mask >> 4), (char)(mask >> 4), (char)(mask >> 3), (char)(mask >> 3),
                                       (char)(mask >> 2), (char)(mask >> 2), (char)(mask >> 1), (char)(mask >> 1),
                                       (char)mask, (char)mask, (char)(mask << 1), (char)(mask << 1),
                                       (char)(mask << 2), (char)(mask << 2), (char)(mask << 3), (char)(mask << 3),
                                       (char)(mask << 4), (char)(mask << 4), (char)(mask << 5), (char)(mask << 5),
                                       (char)(mask << 6), (char)(mask << 6), (char)(mask << 7), (char)(mask << 7));
    auto blend_a = _mm256_blendv_epi8(dest, a, vec_mask);
    _mm256_storeu_si256((__m256i*)mem_addr, blend_a);
    return;
}
inline void store_mask(F32vec16 * mem_addr, __mmask16 mask, F32vec16 a) 
{ 
    _mm512_mask_storeu_ps((void*)mem_addr, mask, a);
    return;
}
inline void store_mask(F32vec8 * mem_addr, __mmask8 mask, F32vec8 a)
{
    __m256i vec_mask = _mm256_set_epi32((int)mask << 24, (int)mask << 25, (int)mask << 26, (int)mask << 27,
                                        (int)mask << 28, (int)mask << 29, (int)mask << 30, (int)mask << 31);
    _mm256_maskstore_ps((float*)mem_addr, vec_mask, a);
    return;
}

/*! \brief utility function to Multiply packed 16-bit integers
 *
 * Multiply packed 16-bit integers in a and b, producing intermediate signed
 * 32-bit integers. Truncate each intermediate integer to the 18 most
 * significant bits, round by adding 1, and store bits [16:1] to dst.
 * Supports AVX2, AVX512 and C
 */
inline int16_t mulhrs(const int16_t a, const int16_t b) {

    // convert to 32bit int for multiplication
    int tmpA = (int) a;
    int tmpB = (int) b;

    int tmpC = tmpA * tmpB;

    // perform scale and rounding and return result
    tmpC = (tmpC >>14) +1;

    return ((int16_t) (tmpC >>1));
}
inline I16vec32 mulhrs(const I16vec32 a, const I16vec32 b) { return (_mm512_mulhrs_epi16( a, b)); }
inline I16vec16 mulhrs(const I16vec16 a, const I16vec16 b) { return (_mm256_mulhrs_epi16( a, b)); }

/*! \brief utility function to Multiply packed 16-bit integers
 *
 * Multiply packed 16-bit integers in a and b, producing intermediate signed
 * 32-bit integers. Store the high 16 bits to dst.
 * Supports AVX2, AVX512 and C
 */
inline int16_t mulhi(const int16_t a, const int16_t b) {

    // convert to 32bit int for multiplication
    int tmpA = (int) a;
    int tmpB = (int) b;

    int tmpC = tmpA * tmpB;

    return ((int16_t) (tmpC >>16));
}
inline I16vec32 mulhi(const I16vec32 a, const I16vec32 b) { return (_mm512_mulhi_epi16( a, b)); }
inline I16vec16 mulhi(const I16vec16 a, const I16vec16 b) { return (_mm256_mulhi_epi16( a, b)); }

inline I16vec32 mulhi_epu(const I16vec32 a, const I16vec32 b) { return (_mm512_mulhi_epu16( a, b)); }
inline I16vec16 mulhi_epu(const I16vec16 a, const I16vec16 b) { return (_mm256_mulhi_epu16( a, b)); }
inline I16vec8 mulhi_epu(const I16vec8 a, const I16vec8 b) { return (_mm_mulhi_epu16( a, b)); }


/*! \brief utility function to perform fused multiply add instruction.
 *
 * Supports floating point AVX2 and AVX512
 */
inline F32vec16 fmadd(const F32vec16 a, const F32vec16 b, const F32vec16 c) { return (_mm512_fmadd_ps(a,b,c)); }
inline F32vec8 fmadd(const F32vec8 a, const F32vec8 b, const F32vec8 c) { return (_mm256_fmadd_ps(a,b,c)); }

/*! \brief utility function to perform fused multiply add subtract instruction.
 *
 * Supports floating point AVX2 and AVX512
 */
inline F32vec16 fmaddsub(const F32vec16 a, const F32vec16 b, const F32vec16 c) { return (_mm512_fmaddsub_ps(a,b,c));}
inline F32vec8 fmaddsub(const F32vec8 a, const F32vec8 b, const F32vec8 c) { return (_mm256_fmaddsub_ps(a,b,c));}


/*! \brief utility function to switch floating point the IQ values to QI.
 *
 * Supports single precision floating point AVX2 and AVX512
 */
inline F32vec16 switch_iq(const F32vec16 a) {
    const int permute_mask_cdab = 177;
    return (_mm512_permute_ps(a, permute_mask_cdab));
}
inline F32vec8 switch_iq(const F32vec8 a) {
    const int permute_mask_cdab = 177;
    return (_mm256_permute_ps(a, permute_mask_cdab));
}


/*! \brief utility function to switch 16-bit IQ values to QI and negate imaginary
 *
 * Supports 16-bit fixed point AVX2 and AVX512
 */
inline I16vec32 switch_iq_and_negate(const I16vec32 a) {
    I16vec32 b = _mm512_rol_epi32(a, 16);
    I32vec16 zero = I32vec16(0);
    return (_mm512_mask_sub_epi16(b, 0x55555555, zero, b));
}
inline I16vec16 switch_iq_and_negate(const I16vec16 a) {
    const __m256i xSwitchReIm_Avx = _mm256_setr_epi8(0x02, 0x03, 0x00, 0x01, 0x06, 0x07, 0x04,
                                                     0x05, 0x0a, 0x0b, 0x08, 0x09, 0x0e, 0x0f,
                                                     0x0c, 0x0d, 0x02, 0x03, 0x00, 0x01, 0x06,
                                                     0x07, 0x04, 0x05, 0x0a, 0x0b, 0x08, 0x09,
                                                     0x0e, 0x0f, 0x0c, 0x0d);
    const __m256i xMaskOne_Avx = _mm256_setr_epi16(-1, 1, -1, 1, -1, 1, -1, 1,-1, 1, -1, 1, -1, 1,
                                                   -1, 1);
    I16vec16 b = _mm256_shuffle_epi8(a, xSwitchReIm_Avx);
    return (_mm256_sign_epi16(b, xMaskOne_Avx));
}

/*! \brief utility function to duplicate the real (I) samples in register
 *
 * a.r a.i b.r b.i  => a.r a.r b.r b.r
 * Supports 16-bit fixed point, 32-bit fixed point, single precision floating point in both AVX2
 * and AVX512
 */
inline I16vec32 duplicateReals(I16vec32 a) {
    I16vec32 duplicateReals_Mask = I8vec64(13,12,13,12,9,8,9,8,5,4,5,4,1,0,1,0,
                                               13,12,13,12,9,8,9,8,5,4,5,4,1,0,1,0,
                                               13,12,13,12,9,8,9,8,5,4,5,4,1,0,1,0,
                                               13,12,13,12,9,8,9,8,5,4,5,4,1,0,1,0);

    return (_mm512_shuffle_epi8(a, duplicateReals_Mask));
}
inline I16vec16 duplicateReals(I16vec16 a) {

    I16vec16 duplicateReals_Mask = _mm256_setr_epi8(0, 1, 0, 1, 4, 5, 4, 5, 8, 9, 8, 9, 12, 13, 12, 13,
                                                0, 1, 0, 1, 4, 5, 4, 5, 8, 9, 8, 9, 12, 13, 12, 13);
    return (_mm256_shuffle_epi8(a, duplicateReals_Mask));
}
inline I16vec32 duplicateReals(I32vec16 a) {
    I16vec32 duplicateReals_Mask = I8vec64(13,12,13,12,9,8,9,8,5,4,5,4,1,0,1,0,
                                               13,12,13,12,9,8,9,8,5,4,5,4,1,0,1,0,
                                               13,12,13,12,9,8,9,8,5,4,5,4,1,0,1,0,
                                               13,12,13,12,9,8,9,8,5,4,5,4,1,0,1,0);
    return (_mm512_shuffle_epi8(a, duplicateReals_Mask));
}
inline I16vec16 duplicateReals(I32vec8 a) {

    I16vec16 duplicateReals_Mask = _mm256_setr_epi8(0, 1, 0, 1, 4, 5, 4, 5, 8, 9, 8, 9, 12, 13, 12, 13,
                                                0, 1, 0, 1, 4, 5, 4, 5, 8, 9, 8, 9, 12, 13, 12, 13);
    return (_mm256_shuffle_epi8(a, duplicateReals_Mask));
}
inline F32vec16 duplicateReals(F32vec16 a) { return (_mm512_moveldup_ps(a));}
inline F32vec8 duplicateReals(F32vec8 a) { return (_mm256_moveldup_ps(a));}

/*! \brief utility function to duplicate the real (I) samples in register
 *
 * a.r a.i b.r b.i  => a.i a.i b.i b.i
 * Supports 16-bit fixed point, single precision floating point in both AVX2
 * and AVX512
 */
inline I16vec32 duplicateImag(I16vec32 a) {
    I16vec32 duplicateImag_Mask = I8vec64(15,14,15,14,11,10,11,10,7,6,7,6,3,2,3,2,
            15,14,15,14,11,10,11,10,7,6,7,6,3,2,3,2,
            15,14,15,14,11,10,11,10,7,6,7,6,3,2,3,2,
            15,14,15,14,11,10,11,10,7,6,7,6,3,2,3,2);

    return (_mm512_shuffle_epi8(a, duplicateImag_Mask));
}
inline I16vec16 duplicateImag(I16vec16 a) {
    I16vec16 duplicateImag_Mask = _mm256_setr_epi8(2, 3, 2, 3, 6, 7, 6, 7, 10, 11, 10, 11, 14, 15, 14, 15,
            2, 3, 2, 3, 6, 7, 6, 7, 10, 11, 10, 11, 14, 15, 14, 15);

    return (_mm256_shuffle_epi8(a, duplicateImag_Mask));
}
inline F32vec16 duplicateImag(F32vec16 a) { return (_mm512_movehdup_ps(a));}
inline F32vec8 duplicateImag(F32vec8 a) { return (_mm256_movehdup_ps(a));}

inline I16vec32 slli(I16vec32 a, unsigned int count) {return ( _mm512_slli_epi16(a, count)); }
inline I16vec16 slli(I16vec16 a, unsigned int count) {return ( _mm256_slli_epi16(a, count)); }

inline I16vec32 srai(I16vec32 a, unsigned int count) {return ( _mm512_srai_epi16(a, count)); }
inline I16vec16 srai(I16vec16 a, unsigned int count) {return ( _mm256_srai_epi16(a, count)); }

inline I16vec32 srli(I16vec32 a, unsigned int count) {return ( _mm512_srli_epi16(a, count)); }
inline I16vec16 srli(I16vec16 a, unsigned int count) {return ( _mm256_srli_epi16(a, count)); }

inline I16vec32 add(I16vec32 a, I16vec32 b) { return _mm512_add_epi16(a, b); }
inline I16vec16 add(I16vec16 a, I16vec16 b) { return _mm256_add_epi16(a, b); }

inline I16vec32 adds(I16vec32 a, I16vec32 b) { return _mm512_adds_epi16(a, b); }
inline I16vec16 adds(I16vec16 a, I16vec16 b) { return _mm256_adds_epi16(a, b); }

inline I16vec32 adds_epu(I16vec32 a, I16vec32 b) { return _mm512_adds_epu16(a, b); }
inline I16vec16 adds_epu(I16vec16 a, I16vec16 b) { return _mm256_adds_epu16(a, b); }

inline I16vec32 sub(I16vec32 a, I16vec32 b) { return _mm512_sub_epi16(a, b); }
inline I16vec16 sub(I16vec16 a, I16vec16 b) { return _mm256_sub_epi16(a, b); }

inline I16vec32 subs(I16vec32 a, I16vec32 b) { return _mm512_subs_epi16(a, b); }
inline I16vec16 subs(I16vec16 a, I16vec16 b) { return _mm256_subs_epi16(a, b); }

inline I16vec32 abs(I16vec32 a) { return _mm512_abs_epi16(a); }
inline I16vec16 abs(I16vec16 a) { return _mm256_abs_epi16(a); }

inline I16vec32 mullo(I16vec32 a, I16vec32 b) { return _mm512_mullo_epi16(a, b); }
inline I16vec16 mullo(I16vec16 a, I16vec16 b) { return _mm256_mullo_epi16(a, b); }

/*! \brief utility function to perform bitwise left shift */
inline I16vec32 left_shift(I16vec32 a, unsigned int count) {return ( _mm512_slli_epi16(a, count)); }
inline I16vec16 left_shift(I16vec16 a, unsigned int count) {return ( _mm256_slli_epi16(a, count)); }

inline I32vec16 mul_addI32(I16vec32 a, I16vec32 b) { return _mm512_madd_epi16(a, b); }
inline I32vec8 mul_addI32(I16vec16 a, I16vec16 b) { return _mm256_madd_epi16(a, b); }

inline I32vec16 addI32(I32vec16 a, I32vec16 b) { return _mm512_add_epi32(a, b); }
inline I32vec8 addI32(I32vec8 a, I32vec8 b) { return _mm256_add_epi32(a, b); }

inline I32vec16 convertF32ToI32(const F32vec16 &a) { return _mm512_cvtps_epi32(a); }
inline I32vec8 convertF32ToI32(const F32vec8 &a) { return _mm256_cvtps_epi32(a); }

inline F32vec16 convertI32ToF32(const I32vec16 &a) { return _mm512_cvtepi32_ps(a); }
inline F32vec8 convertI32ToF32(const I32vec8 &a) { return _mm256_cvtepi32_ps(a); }

inline F32vec16 fused_negate_mul_add(F32vec16 a, F32vec16 b, F32vec16 c) { return _mm512_fnmadd_ps(a, b, c); }
inline F32vec8 fused_negate_mul_add(F32vec8 a, F32vec8 b, F32vec8 c) { return _mm256_fnmadd_ps(a, b, c); }

inline F32vec16 fused_mul_add(F32vec16 a, F32vec16 b, F32vec16 c) { return _mm512_fmadd_ps(a, b, c); }
inline F32vec8 fused_mul_add(F32vec8 a, F32vec8 b, F32vec8 c) { return _mm256_fmadd_ps(a, b, c); }

inline F32vec16 subF32(F32vec16 a, F32vec16 b) { return _mm512_sub_ps(a, b); }
inline F32vec8 subF32(F32vec8 a, F32vec8 b) { return _mm256_sub_ps(a, b); }

inline F32vec16 mulF32(F32vec16 a, F32vec16 b) { return _mm512_mul_ps(a, b); }
inline F32vec8 mulF32(F32vec8 a, F32vec8 b) { return _mm256_mul_ps(a, b); }

inline F32vec16 addF32(F32vec16 a, F32vec16 b) { return _mm512_add_ps(a, b); }
inline F32vec8 addF32(F32vec8 a, F32vec8 b) { return _mm256_add_ps(a, b); }

inline F32vec16 rcpF32(F32vec16 a) { return _mm512_rcp14_ps(a); }
inline F32vec8 rcpF32(F32vec8 a) { return _mm256_rcp_ps(a); }

/// Utility function to get the absolute value of a variable
inline int32_t get_absolute(int32_t a) { return (abs(a));}
inline float get_absolute(float a) {return (fabs(a));}

/*! \brief Calculate sum of vector with complex numbers in format of packed 16-bit integers
    \param [in] values Vector with complex numbers placed one by one:
                       [Num1.Re Num1.Im Num2.Re Num2.Im ... NumN.Re NumN.Im]
    \return vector with calculated sum as complex number: real and imaginary part are 16b numbers
    \note Summing is done using 16b numbers, overflow is not checked.
          Supports AVX2 and AVX512.
*/
inline I16vec16 sum_complex_vec_elem_16b_acc(I16vec16 values)
{
    auto sum_16b = _mm256_permute2f128_si256(values, values, 0x1); /* 0x1 swaps 128 bit lanes. */ // 3/1
    sum_16b = _mm256_add_epi16(values, sum_16b);

    auto sum_16b_sw = _mm256_shuffle_epi32(sum_16b, 0x4E); /* 0x4E swaps f0<->f2 and f1<->f3 within a 128 bit lanes. */ // 1/1
    sum_16b = _mm256_add_epi16(sum_16b, sum_16b_sw);

    sum_16b_sw = _mm256_shuffle_epi32(sum_16b, 0xB1); /* 0xB1 swaps f0<->f1 and f2<->f3 within a 128 bit lanes. */ // 1/1
    sum_16b = _mm256_add_epi16(sum_16b, sum_16b_sw);

    return sum_16b;
}
inline I16vec32 sum_complex_vec_elem_16b_acc(I16vec32 values)
{
    auto sum_16b = _mm512_shuffle_i64x2(values, values, 0x4E); /* 0x4E swaps 256 bits in 512 vector */
    sum_16b = _mm512_add_epi16(values, sum_16b);

    auto sum_16b_sw = _mm512_shuffle_i64x2(sum_16b, sum_16b, 0xB1); /* 0xB1 swaps 128 bit lanes (0<->1 and 2<->3) */
    sum_16b = _mm512_add_epi16(sum_16b, sum_16b_sw);

    sum_16b_sw = _mm512_shuffle_epi32(sum_16b, 0x4E); /*0x4E swap 64b bits in each lane of 512 vector */
    sum_16b = _mm512_add_epi16(sum_16b, sum_16b_sw);

    sum_16b_sw = _mm512_rol_epi64(sum_16b, 32); /* swap 32b bits in each lane of 512 vector */

    sum_16b = _mm512_add_epi16(sum_16b, sum_16b_sw);

    return sum_16b;
}

/*! \brief Calculate sum of vector with complex numbers in format of packed 16-bit integers
    \param [in] values Vector with complex numbers placed one by one:
                       [Num1.Re Num1.Im Num2.Re Num2.Im ... NumN.Re NumN.Im]
    \return vector with calculated sum as complex number: real and imaginary part are 32b numbers
    \note First summing is done using 16b numbers. Than conversion to 32b numbers is done.
          Than rest of summing is done with 32b numbers (32b accumulators are needed to
          prevent overflow of fxp numbers).
          Supports AVX2 and AVX512.
*/
inline I16vec16 sum_complex_vec_elem_32b_acc(I16vec16 values)
{
    auto sum_16b = _mm256_permute2f128_si256(values, values, 0x1); /* 0x1 swaps 128 bit lanes. */ // 3/1
    sum_16b = _mm256_add_epi16(values, sum_16b);

    /* Conversion to 32bits before further summing, 32b long accumulators needed. */
    auto sum_32b = _mm256_cvtepi16_epi32(_mm256_extractf128_si256(sum_16b, 0));

    auto sum_32b_sw = _mm256_permute2f128_si256(sum_32b, sum_32b, 0x1); /* 0x1 swaps 128 bit lanes. */
    sum_32b = _mm256_add_epi32(sum_32b, sum_32b_sw);

    sum_32b_sw = _mm256_shuffle_epi32(sum_32b, 0x4E); /* 0x4E swaps 64 bits (f0<->f2 and f1<->f3) within a 128 bit lanes. */ // 1/1
    sum_32b = _mm256_add_epi32(sum_32b, sum_32b_sw);

    return sum_32b;
}
inline I16vec32 sum_complex_vec_elem_32b_acc(I16vec32 values)
{
    auto sum_16b = _mm512_shuffle_i64x2(values, values, 0x4E); /* 0x4E swaps 256 bits in 512 vector */
    sum_16b = _mm512_add_epi16(values, sum_16b);

    /* Conversion to 32bits before further summing, 32b long accumulators needed. */
    auto sum_32b = _mm512_cvtepi16_epi32(_mm512_extracti64x4_epi64(sum_16b, 0));

    auto sum_32b_sw = _mm512_shuffle_i64x2(sum_32b, sum_32b, 0x4E); /* 0x4E swaps 256 bits in 512 vector */
    sum_32b = _mm512_add_epi32(sum_32b, sum_32b_sw);

    sum_32b_sw = _mm512_shuffle_i64x2(sum_32b, sum_32b, 0xB1); /* 0xB1 swaps 128 bit lanes (0<->1 and 2<->3) */
    sum_32b = _mm512_add_epi32(sum_32b, sum_32b_sw);

    sum_32b_sw = _mm512_shuffle_epi32(sum_32b, 0x4E); /*0x4E swap 64b bits in each lane of 512 vector */
    sum_32b = _mm512_add_epi32(sum_32b, sum_32b_sw);

    return sum_32b;
}
/*! \brief Calculate sum of vector with complex numbers in format of floating point numbers
    \param [in] values Vector with complex numbers placed one by one:
                       [Num1.Re Num1.Im Num2.Re Num2.Im ... NumN.Re NumN.Im]
    \return vector with calculated sum as complex number
    \note Supports AVX2.
*/
inline F32vec8 sum_complex_vec_elem(F32vec8 values)
{
    auto sum = _mm256_permute2f128_ps(values, values, 0x1); /* 0x1  swaps 128 bit lanes. */
    sum = _mm256_add_ps(values, sum);

    auto sum_sw = _mm256_permute_ps(sum, 0x4E); /* 0x4E swaps f0<->f2 and f1<->f3 within a 128 bit lanes. */
    sum = _mm256_add_ps(sum_sw, sum);
    return sum;
}

/*! \brief utility function to division packed 16-bit or 32-bit integers
 *
 * Divide packed 16-bit or 32-bit integers in a by packed elements in b, and store the truncated results in dst.
 * Supports AVX2, AVX512
 */
inline I16vec32 Div(const I16vec32 a, const I16vec32 b) { return (_mm512_div_epi16(a, b)); }
inline I16vec16 Div(const I16vec16 a, const I16vec16 b) { return (_mm256_div_epi16(a, b)); }
inline I32vec16 Div(const I32vec16 a, const I32vec16 b) { return (_mm512_div_epi32(a, b)); }
inline I32vec8  Div(const I32vec8 a,  const I32vec8 b)  { return (_mm256_div_epi32(a, b)); }


/*! \brief utility function to set packed 16-bit or 32-bit integers
 *
 * Broadcast 16-bit or 32-bit integer a to all elements of dst.
 * Supports AVX2, AVX512
 */
inline I16vec32 Set(short a, const I16vec32 b) { return (_mm512_set1_epi16(a)); }
inline I16vec16 Set(short a, const I16vec16 b) { return (_mm256_set1_epi16(a)); }
inline I32vec16 Set(int a,   const I32vec16 b) { return (_mm512_set1_epi32(a)); }
inline I32vec8  Set(int a,   const I32vec8 b)  { return (_mm256_set1_epi32(a)); }

/*! \brief utility function to logical rigth shift packed 16-bit or 32-bit integers
 *
 * Shift packed 16-bit or 32-bit integers in a right by imm8 while shifting in zeros, and store the results in dst.
 * Supports AVX2, AVX512
 */
inline I16vec32 LogicRightShift(const I16vec32 a, unsigned int count) { return (_mm512_srli_epi16(a, count)); }
inline I16vec16 LogicRightShift(const I16vec16 a, unsigned int count) { return (_mm256_srli_epi16(a, count)); }
inline I32vec16 LogicRightShift(const I32vec16 a, unsigned int count) { return (_mm512_srli_epi32(a, count)); }
inline I32vec8  LogicRightShift(const I32vec8  a, unsigned int count) { return (_mm256_srli_epi32(a, count)); }

/*! \brief utility function to switch 16-bit IQ values to QI.
 *
 * Supports 16-bit fixed point AVX2 and AVX512
 */
inline I16vec32 SwitchIq(const I16vec32 a) { return (_mm512_rol_epi32(a, 16));}
inline I16vec16 SwitchIq(const I16vec16 a) {
    const __m256i xSwitchReIm_Avx = _mm256_setr_epi8(0x02, 0x03, 0x00, 0x01, 0x06, 0x07, 0x04,
                                                     0x05, 0x0a, 0x0b, 0x08, 0x09, 0x0e, 0x0f,
                                                     0x0c, 0x0d, 0x02, 0x03, 0x00, 0x01, 0x06,
                                                     0x07, 0x04, 0x05, 0x0a, 0x0b, 0x08, 0x09,
                                                     0x0e, 0x0f, 0x0c, 0x0d);
    return (_mm256_shuffle_epi8(a, xSwitchReIm_Avx));
}

/*! \brief utility function to switch 16-bit IQ values to QI and negate real and imag
 *
 * Supports 16-bit fixed point AVX2 and AVX512
 */
inline I16vec32 SwitchIqNegBoth(const I16vec32 a) {
    I16vec32 b = _mm512_rol_epi32(a, 16);
    const __m512i zero_512_1 = _mm512_setzero_si512();
    return (_mm512_sub_epi16(zero_512_1, b));
}
inline I16vec16 SwitchIqNegBoth(const I16vec16 a) {
    const __m256i xSwitchReIm_Avx = _mm256_setr_epi8(0x02, 0x03, 0x00, 0x01, 0x06, 0x07, 0x04,
                                                     0x05, 0x0a, 0x0b, 0x08, 0x09, 0x0e, 0x0f,
                                                     0x0c, 0x0d, 0x02, 0x03, 0x00, 0x01, 0x06,
                                                     0x07, 0x04, 0x05, 0x0a, 0x0b, 0x08, 0x09,
                                                     0x0e, 0x0f, 0x0c, 0x0d);
    const __m256i zero_256_1 = _mm256_setzero_si256();
    I16vec16 b = _mm256_shuffle_epi8(a, xSwitchReIm_Avx);
    return (_mm256_sub_epi16(zero_256_1, b));
}

/*! \brief utility function to switch 16-bit IQ values to QI and negate imag
 *
 * Supports 16-bit fixed point AVX2 and AVX512
 */
inline I16vec32 SwitchIqNegImag(const I16vec32 a) {
    I16vec32 b = _mm512_rol_epi32(a, 16);
    I32vec16 zero = I32vec16(0);
    return (_mm512_mask_sub_epi16(b, 0x55555555, zero, b));
}
inline I16vec16 SwitchIqNegImag(const I16vec16 a) {
    const __m256i xSwitchReIm_Avx = _mm256_setr_epi8(0x02, 0x03, 0x00, 0x01, 0x06, 0x07, 0x04,
                                                     0x05, 0x0a, 0x0b, 0x08, 0x09, 0x0e, 0x0f,
                                                     0x0c, 0x0d, 0x02, 0x03, 0x00, 0x01, 0x06,
                                                     0x07, 0x04, 0x05, 0x0a, 0x0b, 0x08, 0x09,
                                                     0x0e, 0x0f, 0x0c, 0x0d);
    const __m256i xMaskOne_Avx = _mm256_setr_epi16(-1, 1, -1, 1, -1, 1, -1, 1,-1, 1, -1, 1, -1, 1,
                                                   -1, 1);
    I16vec16 b = _mm256_shuffle_epi8(a, xSwitchReIm_Avx);
    return (_mm256_sign_epi16(b, xMaskOne_Avx));
}

/*! \brief utility function to duplicate the real (I) samples in register and negate even
 *
 * a.r a.i b.r b.i  => a.r -a.r b.r -b.r
 * Supports 16-bit fixed point, single precision floating point in both AVX2
 * and AVX512
 */
inline I16vec32 DupRealNegEven(I16vec32 a) {
    I16vec32 duplicateReals_Mask = I8vec64(13,12,13,12,9,8,9,8,5,4,5,4,1,0,1,0,
                                               13,12,13,12,9,8,9,8,5,4,5,4,1,0,1,0,
                                               13,12,13,12,9,8,9,8,5,4,5,4,1,0,1,0,
                                               13,12,13,12,9,8,9,8,5,4,5,4,1,0,1,0);
    const __m512i zero_512_2 = _mm512_setzero_si512();
    I16vec32 b = _mm512_shuffle_epi8(a, duplicateReals_Mask);
    return (_mm512_mask_sub_epi16(b, 0xAAAAAAAA, zero_512_2, b));
}
inline I16vec16 DupRealNegEven(I16vec16 a) {
    I16vec16 duplicateReals_Mask = _mm256_setr_epi8(0, 1, 0, 1, 4, 5, 4, 5, 8, 9, 8, 9, 12, 13, 12, 13,
                                                0, 1, 0, 1, 4, 5, 4, 5, 8, 9, 8, 9, 12, 13, 12, 13);
    const __m256i xMaskOne_Avx_1 = _mm256_setr_epi16(1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1,
                                                   1, -1);
    I16vec16 b = _mm256_shuffle_epi8(a, duplicateReals_Mask);
    return (_mm256_sign_epi16(b, xMaskOne_Avx_1));
}

/*! \brief utility function to duplicate the imag (Q) samples in register and negate even
 *
 * a.r a.i b.r b.i  => a.i -a.i b.i -b.i
 * Supports 16-bit fixed point in both AVX2 and AVX512
 */
inline I16vec32 DupImagNegEven(I16vec32 a) {
    I16vec32 duplicateImag_Mask = I8vec64(15,14,15,14,11,10,11,10,7,6,7,6,3,2,3,2,
            15,14,15,14,11,10,11,10,7,6,7,6,3,2,3,2,
            15,14,15,14,11,10,11,10,7,6,7,6,3,2,3,2,
            15,14,15,14,11,10,11,10,7,6,7,6,3,2,3,2);
    const __m512i zero_512_3 = _mm512_setzero_si512();
    I16vec32 b = _mm512_shuffle_epi8(a, duplicateImag_Mask);
    return (_mm512_mask_sub_epi16(b, 0xAAAAAAAA, zero_512_3, b));
}
inline I16vec16 DupImagNegEven(I16vec16 a) {
    I16vec16 duplicateImag_Mask = _mm256_setr_epi8(2, 3, 2, 3, 6, 7, 6, 7, 10, 11, 10, 11, 14, 15, 14, 15,
            2, 3, 2, 3, 6, 7, 6, 7, 10, 11, 10, 11, 14, 15, 14, 15);
    const __m256i xMaskOne_Avx_2 = _mm256_setr_epi16(1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1,
                                                   1, -1);
    I16vec16 b = _mm256_shuffle_epi8(a, duplicateImag_Mask);
    return (_mm256_sign_epi16(b, xMaskOne_Avx_2));
}


/*! \brief Utility function to implement complex multiplication
 * multiply complex: (A + iB)*(C+iD) = AC-BD + i(AD+BC) */
inline void bblib_complex_mult(__m512i input0, __m512i input1, __m512i *outPtr) {

  const __m512i m512_sw_r = _mm512_set_epi8(13,12,13,12,9,8,9,8,5,4,5,4,1,0,1,0,
                                   13,12,13,12,9,8,9,8,5,4,5,4,1,0,1,0,
                                   13,12,13,12,9,8,9,8,5,4,5,4,1,0,1,0,
                                   13,12,13,12,9,8,9,8,5,4,5,4,1,0,1,0);

  const __m512i m512_sw_i  = _mm512_set_epi8(15,14,15,14,11,10,11,10,7,6,7,6,3,2,3,2,
                                   15,14,15,14,11,10,11,10,7,6,7,6,3,2,3,2,
                                   15,14,15,14,11,10,11,10,7,6,7,6,3,2,3,2,
                                   15,14,15,14,11,10,11,10,7,6,7,6,3,2,3,2);

  const __m512i m512IQ_switch = _mm512_set_epi8(13,12,15,14,9,8,11,10,5,4,7,6,1,0,3,2,
                                       13,12,15,14,9,8,11,10,5,4,7,6,1,0,3,2,
                                       13,12,15,14,9,8,11,10,5,4,7,6,1,0,3,2,
                                       13,12,15,14,9,8,11,10,5,4,7,6,1,0,3,2);

  const __m512i m512Neg_I = _mm512_set_epi16(0x0001,0xffff, 0x0001, 0xffff,
                                        0x0001,0xffff, 0x0001, 0xffff,
                                        0x0001,0xffff, 0x0001, 0xffff,
                                        0x0001,0xffff, 0x0001, 0xffff,
                                        0x0001,0xffff, 0x0001, 0xffff,
                                        0x0001,0xffff, 0x0001, 0xffff,
                                        0x0001,0xffff, 0x0001, 0xffff,
                                        0x0001,0xffff, 0x0001, 0xffff);


    __m512i ReRe, ImIm, negImPosRe;
    __m512i tmp1, tmp2, result;
    ReRe = _mm512_shuffle_epi8(input0, m512_sw_r);
    ImIm = _mm512_shuffle_epi8(input0, m512_sw_i);
    tmp1 = _mm512_shuffle_epi8(input1, m512IQ_switch);
    negImPosRe = _mm512_mullo_epi16(tmp1, m512Neg_I);

    tmp1 = _mm512_mulhrs_epi16(ReRe, input1);
    tmp2 = _mm512_mulhrs_epi16(ImIm, negImPosRe);
    result = _mm512_adds_epi16(tmp1, tmp2);
    _mm512_store_si512((__m512i *) outPtr, result);
}



