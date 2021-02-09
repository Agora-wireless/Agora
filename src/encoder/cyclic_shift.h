#ifndef _CYCLIC_SHIFT_H_
#define _CYCLIC_SHIFT_H_

#include <immintrin.h>
#include <stdexcept>

namespace avx2enc {
inline __m256i CycleBitShift2to64(__m256i data, int16_t cyc_shift, int16_t zc);
inline __m256i CycleBitShift72to128(
    __m256i data, int16_t cyc_shift, int16_t zc);
inline __m256i CycleBitShift144to256(
    __m256i data, int16_t cyc_shift, int16_t zc);

typedef __m256i (*CYCLIC_BIT_SHIFT)(__m256i, int16_t, int16_t);
CYCLIC_BIT_SHIFT LdpcSelectShiftFunc(int16_t zcSize);
} // namespace avx2enc

#endif
