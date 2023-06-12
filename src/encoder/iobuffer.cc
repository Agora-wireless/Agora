/**
 * @file iobuffer.cc
 * @brief Read the input files into z-bit segments; regroup the output from
 * z-bit segments to bytes
 */
#include "iobuffer.h"

#include <cassert>
#include <cstdio>

#include "common_typedef_sdk.h"
#include "encoder.h"

namespace avx2enc {
static constexpr bool kPrintAdapterDebug = false;

void ScatterSlow(uint8_t* dst, const uint8_t* src, unsigned num_bits,
                 uint8_t src_offbits) {
  // Process byte by byte
  while (num_bits != 0) {
    unsigned num_bits_in_b = MIN(8, num_bits);
    uint8_t new_b;
    if (src_offbits == 0) {
      new_b = src[0];
    } else {
      new_b = ((src[0] & 0xFF) >> src_offbits) |
              ((src[1] & 0xFF) << (8 - src_offbits));
    }
    dst[0] = new_b & BITMASKU8(num_bits_in_b);
    num_bits -= num_bits_in_b;

    dst++;
    src++;
  }
}

void GatherSlow(uint8_t* dst, const uint8_t* src, int16_t num_bits,
                uint8_t dst_offbits) {
  // Process byte by byte
  bool first_byte = true;
  while (num_bits > 0) {
    unsigned num_bits_in_b = MIN(8, num_bits);
    uint8_t new_b;
    if (dst_offbits == 0) {
      // simple copy
      new_b = src[0] & BITMASKU8(num_bits_in_b);
      src++;
    } else {
      if (first_byte) {
        new_b = (dst[0] & BITMASKU8(dst_offbits)) | (src[0] & 0xFF)
                                                        << dst_offbits;
        num_bits_in_b = 8 - dst_offbits;
        first_byte = false;
      } else {
        new_b = ((src[0] & 0xFF) >> (8 - dst_offbits) | (src[1] & 0xFF)
                                                            << dst_offbits) &
                BITMASKU8(num_bits_in_b);
        src++;
      }
    }
    dst[0] = new_b;
    num_bits -= num_bits_in_b;
    dst++;
  }
}

void Adapter2to64(int8_t* ptr_buff_0, int8_t* ptr_buff_1, uint16_t zc_size,
                  uint32_t cb_len_bits, int8_t direct) {
  int8_t* p_buff_0;
  int8_t* p_buff_1;
  uint8_t dst_offbits = 0;
  uint8_t src_offbits = 0;
  p_buff_0 = ptr_buff_0;
  p_buff_1 = ptr_buff_1;

  if (1 == direct) {
    /* parsing the input
    p_buff_0 is the input, p_buff_1 is the buffer for barrel shifter */
    for (size_t i = 0; i < cb_len_bits / zc_size; i++) {
      ScatterSlow((uint8_t*)p_buff_1, (uint8_t*)p_buff_0, zc_size, src_offbits);
      uint8_t byte_offset = (src_offbits + zc_size) >> 3;
      src_offbits = (src_offbits + zc_size) - (byte_offset << 3);
      p_buff_0 = p_buff_0 + byte_offset;
      p_buff_1 = p_buff_1 + kProcBytes;
    }
  } else {
    /* storing encoded bits into output buffer
    p_buff_0 is the output, p_buff_1 is the buffer for processing data*/
    for (size_t i = 0; i < cb_len_bits / zc_size; i++) {
      GatherSlow((uint8_t*)p_buff_0, (uint8_t*)p_buff_1, (int16_t)zc_size,
                 dst_offbits);
      uint8_t byte_offset = (dst_offbits + zc_size) >> 3;
      dst_offbits = (dst_offbits + zc_size) - (byte_offset << 3);
      p_buff_0 = p_buff_0 + byte_offset;
      p_buff_1 = p_buff_1 + kProcBytes;
    }
  }
}

void Print256Epi8(__m256i var) {
  auto* val = reinterpret_cast<int8_t*>(&var);
  std::printf(
      "Numerical int8_t: %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i "
      "%i %i %i %i %i %i %i %i %i %i %i %i %i %i\n",
      val[0], val[1], val[2], val[3], val[4], val[5], val[6], val[7], val[8],
      val[9], val[10], val[11], val[12], val[13], val[14], val[15], val[16],
      val[17], val[18], val[19], val[20], val[21], val[22], val[23], val[24],
      val[25], val[26], val[27], val[28], val[29], val[30], val[31]);
}

/**
 * @param ptr_buff_0 must be ((cb_len_bits / zc_size) -1) * (zc_size / 8) +
 * kProcBytes (32) bytes large
 * @param ptr_buff_1 must be (cb_len_bits / zc_size) * kProcBytes (32)
 * large
 * @param zc_size  zc value
 * @param cb_len_bits codeblock length
 * @param direct  1 = scatter, otherwise gather
 */
void Adapter64to256(int8_t* ptr_buff_0, int8_t* ptr_buff_1, uint16_t zc_size,
                    uint32_t cb_len_bits, int8_t direct) {
  /* after 64, z is always a multiple of 8 so no need for shifting bytes*/
  assert(zc_size <= 256);

  // __m256d, and __m256i local and global data to 32-byte boundaries on the
  // stack
  __m256i bit_mask;
  __m256i x0;
  __m256i x1;
  int64_t e0 = 0xffffffffffffffff;
  int64_t e1;
  int64_t e2;
  int64_t e3;
  int16_t byte_num = zc_size >> 3;  // Bits to bytes (ie /8)

  if (zc_size >= 256) {
    e1 = 0xffffffffffffffff;
    e2 = 0xffffffffffffffff;
    e3 = 0xffffffffffffffff;
  } else if (zc_size >= 192) {
    e1 = 0xffffffffffffffff;
    e2 = 0xffffffffffffffff;
    e3 = (1UL << (zc_size - 192)) - 1;
  } else if (zc_size >= 128) {
    e1 = 0xffffffffffffffff;
    e2 = (1UL << (zc_size - 128)) - 1;
    e3 = 0;
  } else if (zc_size >= 64) {
    e1 = (1UL << (zc_size - 64)) - 1;
    e2 = 0;
    e3 = 0;
  }
  bit_mask = _mm256_set_epi64x(e3, e2, e1, e0);

  int8_t** read_buffer = nullptr;
  int8_t** write_buffer = nullptr;

  int8_t* p_buff_0 = ptr_buff_0;
  int8_t* p_buff_1 = ptr_buff_1;

  if (kPrintAdapterDebug) {
    std::printf(
        "Adapter64to256 scatter / gather %d, cb length: %d, zc_size: %d, "
        "iterations %d\n",
        static_cast<int>(direct == 1), cb_len_bits, zc_size,
        (cb_len_bits / zc_size));
  }

  // scatter
  if (1 == direct) {
    read_buffer = &p_buff_0;
    write_buffer = &p_buff_1;
  }
  // gather
  else {
    read_buffer = &p_buff_1;
    write_buffer = &p_buff_0;
  }

  // Do the scatter / gather
  for (size_t i = 0; i < (cb_len_bits / zc_size); i++) {
    x0 = _mm256_loadu_si256(reinterpret_cast<const __m256i*>(*read_buffer));
    x1 = _mm256_and_si256(x0, bit_mask);
    _mm256_storeu_si256(reinterpret_cast<__m256i*>(*write_buffer), x1);

    if (kPrintAdapterDebug) {
      std::printf("before: ");
      Print256Epi8(x0);
      std::printf("after: ");
      Print256Epi8(x1);
    }
    p_buff_1 = (p_buff_1 + kProcBytes);
    p_buff_0 = (p_buff_0 + byte_num);
  }
}

void Adapter288to384(int8_t* ptr_buff_0, int8_t* ptr_buff_1, uint16_t zc_size,
                     uint32_t cb_len_bits, int8_t direct) {
  /* use two __m256i to store one segment of length zc */
  int8_t* p_buff_in;
  int8_t* p_buff_out;
  __m256i x0;
  __m256i bit_mask;

  p_buff_in = ptr_buff_0;
  p_buff_out = ptr_buff_1;
  int xtra_byte_num = (zc_size - 256) >> 3;

  bit_mask = _mm256_set_epi32(0, 0, 0, 0, -(xtra_byte_num - 15),
                              -(xtra_byte_num - 11), -(xtra_byte_num - 7), -1);

  if (1 == direct) {
    for (size_t i = 0; i < cb_len_bits / zc_size; i++) {
      // read the first 256 bits
      x0 = _mm256_loadu_si256((__m256i*)p_buff_in);
      _mm256_storeu_si256((__m256i*)p_buff_out, x0);
      p_buff_out = p_buff_out + kProcBytes;
      p_buff_in = p_buff_in + kProcBytes;
      // read the remaining bits
      x0 = _mm256_loadu_si256((__m256i*)p_buff_in);
      _mm256_maskstore_epi32((int*)p_buff_out, bit_mask, x0);
      p_buff_out = p_buff_out + kProcBytes;
      p_buff_in = p_buff_in + xtra_byte_num;
    }
  }
}

LDPC_ADAPTER_P LdpcSelectAdapterFunc(uint16_t zc_size) {
  if (zc_size < 64) {
    return Adapter2to64;
  } else if (zc_size <= 256) {
    return Adapter64to256;
  } else {
    return Adapter288to384;
  }
}
}  // namespace avx2enc
