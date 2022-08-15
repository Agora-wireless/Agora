/**
 * @file simd_types.h
 * @brief Aligned types for SIMD compatibility
 */
#ifndef SIMD_TYPES_H_
#define SIMD_TYPES_H_

#include <boost/align/aligned_allocator.hpp>
#include <complex>
#include <cstddef>
#include <vector>

//Safe for both AVX512 and AVX2
constexpr size_t kSimdAlignment = 64u;

using SimdAlignByteVector =
    std::vector<std::byte,
                boost::alignment::aligned_allocator<std::byte, kSimdAlignment>>;

using SimdAlignFltVector =
    std::vector<float,
                boost::alignment::aligned_allocator<float, kSimdAlignment>>;

using SimdAlignShrtVector =
    std::vector<short,
                boost::alignment::aligned_allocator<short, kSimdAlignment>>;

using SimdAlignCxFltVector = std::vector<
    std::complex<float>,
    boost::alignment::aligned_allocator<std::complex<float>, kSimdAlignment>>;

using SimdAlignCxShrtVector = std::vector<
    std::complex<short>,
    boost::alignment::aligned_allocator<std::complex<short>, kSimdAlignment>>;

#endif  // SIMD_TYPES_H_