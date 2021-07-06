/**
 * @file cyclic_shift.cc
 * @brief Cyclic right bit shift. The shift values are defined by the 5gnr
 * ldpc standard in TS38212 5.3.2
 */
#include "cyclic_shift.h"

#include <cstring> /* std::strerror, std::memset, std::memcpy */

namespace avx2enc {
inline __m256i CycleBitShift2to64(__m256i data, int16_t cyc_shift, int16_t zc) {
  __m256i x1;
  __m256i x2;
  __m256i bit_mask;
  cyc_shift = cyc_shift % zc;
  __int64_t e0;

  if (zc >= 64) {
    e0 = 0xffffffffffffffff;
  } else {
    e0 = (1UL << zc) - 1;
  }

  bit_mask = _mm256_set_epi64x(0, 0, 0, e0);
  data = _mm256_and_si256(data, bit_mask);

  x1 = _mm256_srli_epi64(data, cyc_shift);
  x2 = _mm256_slli_epi64(data, zc - cyc_shift);

  x1 = _mm256_or_si256(x1, x2);
  x1 = _mm256_and_si256(x1, bit_mask);

  return x1;
}

inline __m256i CycleBitShift72to128(__m256i data, int16_t cyc_shift,
                                    int16_t zc) {
  /* when zc is 88 or 104 or 120 */
  int8_t shuffle_table[9][32] = {
      {0, 1, 2,  3,  4,  5,  6,  7,  8,  0,  1,  2,  3,  4,  5,  6,
       7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},  // 72
      {0, 1, 2, 3, 4,  5,  6,  7,  8,  9,  0,  1,  2,  3,  4,  5,
       6, 7, 8, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},  // 80
      {0, 1, 2, 3, 4, 5,  6,  7,  8,  9,  10, 0,  1,  2,  3,  4,
       5, 6, 7, 8, 9, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},  // 88
      {0, 1, 2, 3, 4, 5, 6,  7,  8,  9,  10, 11, 0,  1,  2,  3,
       4, 5, 6, 7, 8, 9, 10, 11, -1, -1, -1, -1, -1, -1, -1, -1},  // 96
      {0, 1, 2, 3, 4, 5, 6, 7,  8,  9,  10, 11, 12, 0,  1,  2,
       3, 4, 5, 6, 7, 8, 9, 10, 11, 12, -1, -1, -1, -1, -1, -1},  // 104
      {0, 1, 2, 3, 4, 5, 6, 7, 8,  9,  10, 11, 12, 13, 0,  1,
       2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, -1, -1, -1, -1},  // 112
      {0, 1, 2, 3, 4, 5, 6, 7, 8, 9,  10, 11, 12, 13, 14, 0,
       1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, -1, -1},  // 120
      {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
       0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15},  // 128
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // padding to avoid memory
                                                      // overrun
  };

  __m256i x0;
  __m256i x2;
  __m256i bit_mask;
  __m256i shift_mask0;
  int64_t e0;
  int zc_in_bytes = zc >> 3;
  auto* p_out = reinterpret_cast<uint8_t*>(&x2);

  int right_shift = cyc_shift % zc;
  int byte_shift = right_shift >> 3;
  int bit_shift = right_shift - (byte_shift << 3);
  int zc_index = (zc >> 3) - 9;

  shift_mask0 =
      _mm256_loadu_si256((__m256i*)(shuffle_table[zc_index] + byte_shift));

  // shift by bytes
  x0 = _mm256_shuffle_epi8(data, shift_mask0);

  // shift remaining memcpybits
  auto* p_data = reinterpret_cast<uint8_t*>(&x0);
  p_out[zc_in_bytes - 1] =
      (p_data[zc_in_bytes - 1] >> bit_shift) | (p_data[0] << (8 - bit_shift));
  for (int i = 0; i < zc_in_bytes - 1; i++) {
    p_out[i] = (p_data[0] >> bit_shift) | (p_data[1] << (8 - bit_shift));
    p_data++;
  }

  // zero out the bits outside zc range in the output
  if (zc >= 128) {
    e0 = 0xffffffffffffffff;
  } else {
    e0 = (1UL << (zc - 64)) - 1;
  }
  bit_mask = _mm256_set_epi64x(0, 0, e0, 0xffffffffffffffff);
  x2 = _mm256_and_si256(x2, bit_mask);

  return x2;
}

inline __m256i CycleBitShift144to256(__m256i data, int16_t cyc_shift,
                                     int16_t zc) {
  /* zc in this range is always a multiple of 16 */

  __m256i x0;
  __m256i x1;
  __m256i x2;
  __m256i bit_mask;
  int64_t e0;
  int64_t e1;
  cyc_shift = cyc_shift % zc;
  int packed_shift = cyc_shift >> 4;
  int bit_shift = cyc_shift & 0xf;
  int zc_in_shorts = zc >> 4;

  auto* p_out_0 = reinterpret_cast<uint16_t*>(&x0);
  auto* p_out_1 = reinterpret_cast<uint16_t*>(&x1);
  auto* p_in = reinterpret_cast<uint16_t*>(&data);

  // manual shift in units of 2 bytes
  // for(int i=0; i<(zc_in_shorts-packed_shift); i++){
  //     p_out_0[i] = p_in[i+packed_shift];
  // }
  // for(int i=zc_in_shorts-packed_shift; i<zc_in_shorts; i++){
  //     p_out_0[i] = p_in[i-zc_in_shorts+packed_shift];
  // }

  std::memcpy(p_out_0, p_in + packed_shift,
              (zc_in_shorts - packed_shift) * sizeof(uint16_t));
  std::memcpy(p_out_0 + zc_in_shorts - packed_shift, p_in,
              packed_shift * sizeof(uint16_t));
  std::memset(p_out_0 + zc_in_shorts, 0, 32 - zc_in_shorts * sizeof(uint16_t));

  x1 = _mm256_srli_si256(x0, 2);

  std::memcpy(p_out_1 + 7, p_out_0 + 8, 2);
  std::memcpy(p_out_1 + zc_in_shorts - 1, p_out_0, 2);
  // p_out_1[7] = p_out_0[8];
  // p_out_1[zc_in_shorts-1] = p_out_0[0];

  // shift the remaining bits
  x2 = _mm256_srli_epi16(x0, bit_shift);
  x0 = _mm256_slli_epi16(x1, 16 - bit_shift);
  x1 = _mm256_or_si256(x0, x2);

  // zero out bits outside zc range in the output
  if (zc >= 256) {
    e0 = 0xffffffffffffffff;
    e1 = 0xffffffffffffffff;
  }
  if (zc >= 192) {
    e0 = 0xffffffffffffffff;
    e1 = (1UL << (zc - 192)) - 1;
  } else if (zc >= 128) {
    e0 = (1UL << (zc - 128)) - 1;
    e1 = 0;
  }
  // !! add exception for error
  bit_mask = _mm256_set_epi64x(e1, e0, 0xffffffffffffffff, 0xffffffffffffffff);

  x1 = _mm256_and_si256(x1, bit_mask);

  return x1;
}

CYCLIC_BIT_SHIFT LdpcSelectShiftFunc(int16_t zcSize) {
  if (zcSize <= 64) {
    return CycleBitShift2to64;
  } else if (zcSize <= 128) {
    return CycleBitShift72to128;
  } else if (zcSize <= 256) {
    return CycleBitShift144to256;
  } else {
    throw std::invalid_argument(
        "cyclic shifter for zc larger than 256 has not been implemented");
  }
}
}  // namespace avx2enc