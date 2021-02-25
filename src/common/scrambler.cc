/**
 * @file  scrambler.cc
 * @brief Implmentation file for the Scramble Class
 */
#include "scrambler.h"

namespace AgoraScrambler {

static const size_t kBitsInitArraySize = 7u;
static const size_t kStartingVectorSize = (100 * 8);

Scrambler::Scrambler()
    : scram_buffer_(kScramblerlength), bit_buffer_(kStartingVectorSize) {}

void Scrambler::ConvertBytesToBits(const int8_t* in_byte_buffer,
                                   size_t byte_buffer_size,
                                   int8_t* out_bit_buffer) {
  for (size_t i = 0; i < byte_buffer_size; i++) {
    for (size_t j = 0; j < 8; j++) {
      out_bit_buffer[i * 8 + j] =
          (in_byte_buffer[i] & (1 << (7 - j))) >> (7 - j);
    }
  }
}

void Scrambler::ConvertBitsToBytes(const int8_t* in_bit_buffer,
                                   size_t byte_buffer_size,
                                   int8_t* out_byte_buffer) {
  for (size_t i = 0; i < byte_buffer_size; i++) {
    out_byte_buffer[i] = 0;
    for (size_t j = 0; j < 8; j++) {
      out_byte_buffer[i] <<= 1;
      out_byte_buffer[i] += in_bit_buffer[i * 8 + j];
    }
  }
}

void Scrambler::WlanScrambler(void* byte_buffer, size_t byte_buffer_size,
                              std::vector<int8_t>& scram_buffer,
                              std::vector<int8_t>& bit_buffer) {
  auto* byte_buffer_ptr = reinterpret_cast<int8_t*>(byte_buffer);
  std::array<int8_t, kBitsInitArraySize> b_scrambler_init_bits;
  std::array<int8_t, kBitsInitArraySize> scrambler_init_bits;
  int8_t tmp;
  size_t j;
  size_t buff_size;
  int8_t res_xor;

  // Make sure there is enough room in each of the vectors
  // reserve will save the size of the largest call to prevent
  // memory reallocation (high water mark) 0 init vectors
  scram_buffer.reserve(kScramblerlength);
  scram_buffer.resize(kScramblerlength);
  std::fill(scram_buffer.begin(), scram_buffer.end(), 0);

  size_t bit_bufffer_size = byte_buffer_size * 8;
  bit_buffer.reserve(bit_bufffer_size);
  bit_buffer.resize(bit_bufffer_size);
  std::fill(bit_buffer.begin(), bit_buffer.end(), 0);

  ConvertBytesToBits(&byte_buffer_ptr[0], byte_buffer_size, bit_buffer.data());

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
      b_scrambler_init_bits.at(i) = scrambler_init_bits.at(6 - i);
    }
    for (size_t i = 0; i < 7; i++) {
      scrambler_init_bits.at(i) = b_scrambler_init_bits.at(i);
    }

    // Generate the scrambling sequence using the generator polynomial
    buff_size = byte_buffer_size * 8;
    if (buff_size > 127) {
      buff_size = 127;
    }
    for (j = 0; j < buff_size; j++) {
      //  x7 xor x4
      res_xor = static_cast<int8_t>((scrambler_init_bits.at(0) != 0) !=
                                    (scrambler_init_bits.at(3) != 0));
      scram_buffer.at(j) = res_xor;
      //  Left-shift
      for (size_t i = 0; i < 6; i++) {
        scrambler_init_bits.at(i) = scrambler_init_bits.at(i + 1);
      }
      //  Update x1
      scrambler_init_bits.at(6) = res_xor;
    }

    // Generate scrambled sequence by xor-ing input to the scrambling sequence
    j = 0;
    for (size_t i = 0; i < byte_buffer_size * 8; i++) {
      if (j == buff_size) {
        j = 1;
      } else {
        j++;
      }

      bit_buffer.at(i) = static_cast<signed char>(
          (bit_buffer.at(i) != 0) != (scram_buffer.at(j - 1) != 0));
    }
  }
  ConvertBitsToBytes(bit_buffer.data(), byte_buffer_size, &byte_buffer_ptr[0]);
}

void Scrambler::Scramble(void* byte_buffer, size_t byte_buffer_size) {
  WlanScrambler(byte_buffer, byte_buffer_size, this->scram_buffer_,
                this->bit_buffer_);
}

void Scrambler::Descramble(void* byte_buffer, size_t byte_buffer_size) {
  WlanScrambler(byte_buffer, byte_buffer_size, this->scram_buffer_,
                this->bit_buffer_);
}

};  // namespace AgoraScrambler