/**
 * @file  scrambler.cc
 * @brief Implmentation file for the Scramble Class
 */
#include "scrambler.h"

namespace Scrambler {

static const size_t kBitsInitArraySize = 7u;

void ConvertBytesToBits(const int8_t* in_byte_buffer, size_t byte_buffer_size,
                        int8_t* out_bit_buffer) {
  for (size_t i = 0; i < byte_buffer_size; i++) {
    for (size_t j = 0; j < 8; j++) {
      out_bit_buffer[i * 8 + j] =
          (in_byte_buffer[i] & (1 << (7 - j))) >> (7 - j);
    }
  }
}

void ConvertBitsToBytes(const int8_t* in_bit_buffer, size_t byte_buffer_size,
                        int8_t* out_byte_buffer) {
  for (size_t i = 0; i < byte_buffer_size; i++) {
    out_byte_buffer[i] = 0;
    for (size_t j = 0; j < 8; j++) {
      out_byte_buffer[i] <<= 1;
      out_byte_buffer[i] += in_bit_buffer[i * 8 + j];
    }
  }
}

void WlanScrambler(void* byte_buffer, size_t byte_buffer_size) {
  auto* byte_buffer_ptr = reinterpret_cast<int8_t*>(byte_buffer);
  int8_t b_scrambler_init_bits[kBitsInitArraySize];
  int8_t scrambler_init_bits[kBitsInitArraySize];
  int8_t tmp;
  size_t j;
  size_t buff_size;
  int8_t res_xor;
  auto* scram_seq_data = new int8_t[kScramblerlength];
  auto* bit_buffer = new int8_t[byte_buffer_size * 8];

  ConvertBytesToBits(&byte_buffer_ptr[0], byte_buffer_size, &bit_buffer[0]);

  // Do scrambling
  if (byte_buffer_size != 0) {
    // Generate scrambler initial state array
    for (int8_t& scrambler_init_bit : scrambler_init_bits) {
      scrambler_init_bit = 0;
    }
    j = 1;
    tmp = kScramblerInitState;
    while ((j <= 7) && (tmp > 0)) {
      scrambler_init_bits[j - 1] = tmp % 2;
      tmp /= 2;
      j++;
    }

    // Inverse the initial state array
    for (size_t i = 0; i < 7; i++) {
      b_scrambler_init_bits[i] = scrambler_init_bits[6 - i];
    }
    for (size_t i = 0; i < 7; i++) {
      scrambler_init_bits[i] = b_scrambler_init_bits[i];
    }

    // Generate the scrambling sequence using the generator polynomial
    buff_size = byte_buffer_size * 8;
    if (buff_size > 127) {
      buff_size = 127;
    }
    for (j = 0; j < buff_size; j++) {
      //  x7 xor x4
      res_xor = static_cast<int8_t>((scrambler_init_bits[0] != 0) !=
                                    (scrambler_init_bits[3] != 0));
      scram_seq_data[j] = res_xor;
      //  Left-shift
      for (size_t i = 0; i < 6; i++) {
        scrambler_init_bits[i] = scrambler_init_bits[i + 1];
      }
      //  Update x1
      scrambler_init_bits[6] = res_xor;
    }

    // Generate scrambled sequence by xor-ing input to the scrambling sequence
    j = 0;
    for (size_t i = 0; i < byte_buffer_size * 8; i++) {
      if (j == buff_size) {
        j = 1;
      } else {
        j++;
      }

      bit_buffer[i] = static_cast<signed char>((bit_buffer[i] != 0) !=
                                               (scram_seq_data[j - 1] != 0));
    }
  }

  ConvertBitsToBytes(&bit_buffer[0], byte_buffer_size, &byte_buffer_ptr[0]);

  delete[] scram_seq_data;
  delete[] bit_buffer;
}

void WlanScramble(void* byte_buffer, size_t byte_buffer_size) {
  WlanScrambler(byte_buffer, byte_buffer_size);
}

void WlanDescramble(void* byte_buffer, size_t byte_buffer_size) {
  WlanScrambler(byte_buffer, byte_buffer_size);
}

};  // end namespace Scrambler