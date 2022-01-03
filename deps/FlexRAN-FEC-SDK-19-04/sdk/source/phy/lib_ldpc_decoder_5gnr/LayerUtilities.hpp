// INTEL CONFIDENTIAL
// Copyright 2009-2019 Intel Corporation All Rights Reserved.
// 
// The source code contained or described herein and all documents related to the
// source code ("Material") are owned by Intel Corporation or its suppliers or
// licensors. Title to the Material remains with Intel Corporation or its
// suppliers and licensors. The Material may contain trade secrets and proprietary
// and confidential information of Intel Corporation and its suppliers and
// licensors, and is protected by worldwide copyright and trade secret laws and
// treaty provisions. No part of the Material may be used, copied, reproduced,
// modified, published, uploaded, posted, transmitted, distributed, or disclosed
// in any way without Intel's prior express written permission.
// 
// No license under any patent, copyright, trade secret or other intellectual
// property right is granted to or conferred upon you by disclosure or delivery
// of the Materials, either expressly, by implication, inducement, estoppel or
// otherwise. Any license under such intellectual property rights must be
// express and approved by Intel in writing.
// 
// Unless otherwise agreed by Intel in writing, you may not remove or alter this
// notice or any other notice embedded in Materials by Intel or Intel's suppliers
// or licensors in any way.
// 
//  version: SDK-jenkins-FlexRAN-SDK-REL-448-g3be238

#pragma once

#include <dvec.h>
#include <cstdint>

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Dvec Extensions
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Dvec doesn't provide a Is16vec16 class, which is an issue. Reported to the compiler team as issue
// CMPLRIL0-30948 in JIRA.
class Is16vec16
{
public:

  Is16vec16() = default;

  Is16vec16(int16_t v) : value(_mm256_set1_epi16(v)) { }
  Is16vec16(__m256i v) : value(v) { }

  operator __m256i() const { return value; }

  friend Is16vec16 abs(Is16vec16 v) { return _mm256_abs_epi16(v); }
  friend Is16vec16 sat_add(Is16vec16 lhs, Is16vec16 rhs) { return _mm256_adds_epi16(lhs, rhs); }
  friend Is16vec16 sat_sub(Is16vec16 lhs, Is16vec16 rhs) { return _mm256_subs_epi16(lhs, rhs); }
  friend Is16vec16 sat_sub_unsigned(Is16vec16 lhs, Is16vec16 rhs) { return _mm256_subs_epu16(lhs, rhs); }

  Is16vec16 operator>>(int count) const { return _mm256_srai_epi16(value, count); }

  __m256i value;
};

inline Is16vec16 select_lt(Is16vec16 a, Is16vec16 b, Is16vec16 c, Is16vec16 d)
{
  return _mm256_blendv_epi8(d, c, _mm256_cmpgt_epi16(b, a));
}

// Some functions are missing from dvec for AVX-512.
inline Is16vec32 abs(Is16vec32 v) { return _mm512_abs_epi16(v); }
inline Is16vec32 sat_sub_unsigned(Is16vec32 lhs, Is16vec32 rhs) { return _mm512_subs_epu16(lhs, rhs); }

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Misc
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Equivalent to Ceil(a/b)
static inline int RoundUpDiv(int a, int b)
{
  return ((a + b - 1) / b);
}

/// Expand the set of bits into complete 16-bit elements which are either all 1 or all 0.
static Is16vec16 ExpandMsb(int16_t msb)
{
  // There are 16 bits, each of which needs to be expanded into a complete 16-bit integer of 1's and
  // 0's.

  // Start by broadcasting all bits to every position.
  const auto dupBits = _mm256_set1_epi16(msb);

  // AND with a mask bit for each position. 1 for the first element, 2 for the next, 4 and so
  // on. That effectively creates a single bit in each element; its the MSB but in the wrong place.
  const auto k_bitForElementMask =
    _mm256_setr_epi16(0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0020, 0x0040, 0x0080,
                      0x0100, 0x0200, 0x0400, 0x0800, 0x1000, 0x2000, 0x4000, 0x8000);
  const auto justElementBits = _mm256_and_si256(dupBits, k_bitForElementMask);

  // Now that each element has just one bit that is set or cleared, comparing back against the same
  // bit mask that was used above will turn that single bit into a complete set of 0's and 1's which
  // fills the element.
  return _mm256_cmpeq_epi16(justElementBits, k_bitForElementMask);
}

template<typename SIMD_TYPE>
static int GetNumAlignedSimdLoops(int zExpansion)
{
  // The number of 16-bit elements in the given SIMD type.
  constexpr int k_numElements = sizeof(SIMD_TYPE) / sizeof(int16_t);
  const int numLoops = (zExpansion + (k_numElements - 1)) / k_numElements;

  return numLoops;
}

/// Broadcast a single int16 to the given type. Unfortunately this is needed because Is16vec32 from
/// dvec doesn't provide its own scalar broadcast.
template<typename T> T BroadcastInt16(int16_t);
template<> Is16vec16 BroadcastInt16(int16_t v) { return _mm256_set1_epi16(v); }
template<> Is16vec32 BroadcastInt16(int16_t v) { return _mm512_set1_epi16(v); }

//////////////////////////////////////////////////////////////////////////////////////////////////////
// AVX2 / AVX512 Overloaded functions
//////////////////////////////////////////////////////////////////////////////////////////////////////

/// Complete horizontal addition of a vector
static int32_t GetHorizontalSum(Is16vec16 v)
{
  //16-->8
  auto sum1 = _mm256_hadds_epi16(v, _mm256_setzero_si256());
  //8-->4
  sum1 = _mm256_hadds_epi16(sum1, _mm256_setzero_si256());
  //4-->2
  sum1 = _mm256_hadds_epi16(sum1, _mm256_setzero_si256());

  int32_t result = (int32_t)(((int16_t *)&sum1)[0]) + (int32_t)(((int16_t *)&sum1)[8]);
  return result;
}

static int32_t GetHorizontalSum(Is16vec32 v)
{
  auto sum1 = _mm512_reduce_add_epi32(_mm512_cvtepi16_epi32(((__m256i *)&v)[0]));
  auto sum2 = _mm512_reduce_add_epi32(_mm512_cvtepi16_epi32(((__m256i *)&v)[1]));
  return sum1 + sum2;
}

/// Get a mask of the negative elements. One bit for each element.
static int16_t GetNegativeMask(Is16vec16 simd)
{
  // Extract all the MSBs of each byte. There is no 16-bit equivalent
  // so this 8-bit version will give too many bits to start with.
  const auto byteMsbs = _mm256_movemask_epi8(simd);

  // Reduce the number of bits down by extracting every other bit, which corresponds to just the
  // epi16 MSBs.
  const auto bits = _pext_u32(byteMsbs, 0b10101010101010101010101010101010);

  return int16_t(bits);
}

/// Get a mask of the negative elements. One bit for each element.
static int32_t GetNegativeMask(Is16vec32 simd)
{
  // Read the MSB by comparing to zero. This is a 3/0.5 instruction. The alternative is to use
  // movepi16_mask instead, which extracts the MSB bit into a mask directly. That instruction is
  // 1/1. For LDPC we are very compute limited though, so the higher throughput of the comparison
  // below gives better performance (3% at time of writing).
  return _mm512_cmp_epi16_mask(simd, _mm512_setzero_si512(), _MM_CMPINT_LT);
}

/// Apply a parity correction. All values whose corresponding bits are zero will be negated.
static Is16vec16 ApplyParityCorrection(int16_t parity, Is16vec16 value)
{
  // Note that individual parity bits can be expanded into a complete set of 0's which forces
  // sign_epi16 to zero the result too. A single 1 is ORed into the parity bit to avoid this.
  const auto parityV = ExpandMsb(parity);
  const __m256i parityTieBreak = _mm256_or_si256(parityV, _mm256_set1_epi16(1));
  return _mm256_sign_epi16(value, parityTieBreak);
}

static Is16vec32 ApplyParityCorrection(int32_t parity, Is16vec32 value)
{
  return _mm512_mask_sub_epi16(value, parity, _mm512_setzero_si512(), value);
}

/// Perform an insertion of a new value into the best two values found so far, which are sorted in order.
static void InsertSort(Is16vec16& min1, Is16vec16& min2, Is16vec16& minPos, int newPos, Is16vec16 value)
{
  // If the new values are better than the current best, update the index.
  minPos = select_lt(value, min1, _mm256_set1_epi16(short(newPos)), minPos);

  // Sort-insertion network.
  const auto t = _mm256_max_epi16(min1, value);
  min2 = _mm256_min_epi16(t, min2);
  min1 = _mm256_min_epi16(min1, value);
}

static void InsertSort(Is16vec32& min1, Is16vec32& min2, Is16vec32& minPos, int newPos, Is16vec32 value)
{
  // Is the original min unchanged?
  const auto isOriginalStillTheMin = _mm512_cmp_epi16_mask(min1, value, _CMP_LE_OS);

  // If the new value is better than the original minimum, update the index.
  minPos = _mm512_mask_blend_epi16(isOriginalStillTheMin, _mm512_set1_epi16(short(newPos)), minPos);

  // Three-way minimum of the new value and the two original lowest values! If the new value is
  // smaller than the original minimum then the original minimum gets displaced to become the second
  // minimum, simply by passing through this mask instruction. If the original minimum wasn't
  // displaced, then choose between the minimum of the new value, and the so-far untested second
  // minimum.
  min2 = _mm512_mask_min_epi16(min1, isOriginalStillTheMin, value, min2);

  // This could be done using a min_epi16 instruction to allow it to be scheduled earlier, but that
  // min would take up the only execution port. Better to reuse the mask and allow the blend to use
  // any port to give higher throughput.
  min1 = _mm512_mask_blend_epi16(isOriginalStillTheMin, value, min1);
}

static Is16vec16 UpdateScheduleNorm(Is16vec16 scheduleNorm, Is16vec16 vnIn, Is16vec16 deltaSigned)
{
  Is16vec16 normSignProduct = _mm256_xor_si256(vnIn, deltaSigned);
  Is16vec16 normPlusMinus = _mm256_sign_epi16(_mm256_set1_epi16(1), normSignProduct);
  return sat_add(scheduleNorm, normPlusMinus);
}

static Is16vec32 UpdateScheduleNorm(Is16vec32 scheduleNorm, Is16vec32 vnIn, Is16vec32 deltaSigned)
{
  Is16vec32 normSignProduct = _mm512_xor_si512(vnIn, deltaSigned);
  auto product_mask = _mm512_cmp_epi16_mask(normSignProduct, _mm512_setzero_si512(), _CMP_LE_OS);
  Is16vec32 normPlusMinus = _mm512_mask_blend_epi16(product_mask, _mm512_set1_epi16(0x1), _mm512_set1_epi16(0xFFFF) );
  return sat_add(scheduleNorm, normPlusMinus);
}

// The compiler's own implementation of this function is wrong (issue CMPLRLIBS-2780). Provide a
// corrected replacement until it has been fixed.
static Is16vec16 SelectEqWorkaround(Is16vec16 a, Is16vec16 b, Is16vec16 c, Is16vec16 d)
{
  return _mm256_blendv_epi8(d, c, _mm256_cmpeq_epi16(a, b));
}

static Is16vec32 SelectEqWorkaround(Is16vec32 a, Is16vec32 b, Is16vec32 c, Is16vec32 d)
{
  const auto mask = _mm512_cmp_epi16_mask(a, b, _MM_CMPINT_EQ);
  return _mm512_mask_blend_epi16(mask, d, c);
}



