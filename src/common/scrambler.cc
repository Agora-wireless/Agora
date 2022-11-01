/**
 * @file  scrambler.cc
 * @brief Implmentation file for the Scramble Class
 */
#include "scrambler.h"

#include <array>
#include <bitset>

#include "logger.h"

namespace AgoraScrambler {

static constexpr size_t kBitsInByte = 8u;
static constexpr size_t kBitsInitArraySize = 7u;
static constexpr size_t kStartingVectorSize = (64 * 125);

Scrambler::Scrambler()
    : scram_buffer_(kScramblerlength), bit_buffer_(kStartingVectorSize) {}

// in byte 0xA5
// out_bits[7] = msb
void Scrambler::ConvertBytesToBits(const std::byte* in_byte_buffer,
                                   size_t byte_buffer_size,
                                   std::byte* out_bit_buffer) {
  for (size_t byte_num = 0; byte_num < byte_buffer_size; byte_num++) {
    const size_t out_byte_offset = (byte_num * kBitsInByte);
    std::byte convert_byte = in_byte_buffer[byte_num];
    AGORA_LOG_TRACE("byte to convert %0x", static_cast<uint8_t>(convert_byte));
    for (size_t bit_num = 1; bit_num <= kBitsInByte; bit_num++) {
      const size_t bit_out_loc = out_byte_offset + (kBitsInByte - bit_num);
      out_bit_buffer[bit_out_loc] = convert_byte & std::byte(0x01);
      AGORA_LOG_TRACE("Convert %0x bit %zu value %d at loc %zu",
                      static_cast<uint8_t>(convert_byte), bit_num,
                      static_cast<uint8_t>(out_bit_buffer[bit_out_loc]),
                      bit_out_loc);
      convert_byte = convert_byte >> 1;
    }
  }
}

void Scrambler::ConvertBitsToBytes(const std::byte* in_bit_buffer,
                                   size_t byte_buffer_size,
                                   std::byte* out_byte_buffer) {
  for (size_t byte_num = 0; byte_num < byte_buffer_size; byte_num++) {
    const size_t byte_offset = (byte_num * kBitsInByte);
    out_byte_buffer[byte_num] = std::byte(0x0);
    for (size_t bit_num = 0; bit_num < kBitsInByte; bit_num++) {
      const size_t bit_in_loc = byte_offset + bit_num;
      out_byte_buffer[byte_num] =
          (in_bit_buffer[bit_in_loc] | (out_byte_buffer[byte_num] << 1));
      AGORA_LOG_TRACE("Convert %0x bit %zu value %d at loc %zu",
                      static_cast<uint8_t>(out_byte_buffer[byte_num]), bit_num,
                      static_cast<uint8_t>(in_bit_buffer[bit_in_loc]),
                      bit_in_loc);
    }
  }
}

void Scrambler::WlanScrambler(void* output_buffer, const void* input_buffer,
                              size_t num_bytes,
                              std::bitset<kScramblerlength>& scram_buffer,
                              std::vector<std::byte>& bit_buffer) {
  const auto* input_buffer_ptr =
      reinterpret_cast<const std::byte*>(input_buffer);
  auto* output_buffer_ptr = reinterpret_cast<std::byte*>(output_buffer);

  std::bitset<kBitsInitArraySize> scrambler_init_bits{kScramblerInitState};

  size_t buff_size;
  std::bitset<1> res_xor;

  //scram_buffer.reset();

  // Make sure there is enough room in each of the vectors
  // reserve will save the size of the largest call to prevent
  // memory reallocation (high water mark) 0 init vectors
  const size_t bit_bufffer_size = num_bytes * kBitsInByte;
  bit_buffer.reserve(bit_bufffer_size);
  bit_buffer.resize(bit_bufffer_size);

  ConvertBytesToBits(input_buffer_ptr, num_bytes, bit_buffer.data());

  // Do scrambling
  if (num_bytes != 0) {
    // Generate scrambler initial state array
    // Inverse the initial state array?  Skip?
    //scrambler_init_bits;

    // Generate the scrambling sequence using the generator polynomial
    buff_size = num_bytes * kBitsInByte;
    if (buff_size > scram_buffer.size()) {
      buff_size = scram_buffer.size();
    }
    for (size_t i = 0; i < buff_size; i++) {
      //  x7 xor x4
      res_xor[0] = scrambler_init_bits[0] ^ scrambler_init_bits[3];
      scram_buffer[i] = res_xor[0];
      scrambler_init_bits = scrambler_init_bits >> 1;
      //  Update x1
      scrambler_init_bits[6] = res_xor[0];
    }

    // Generate scrambled sequence by xor-ing input to the scrambling sequence
    size_t j = 0;
    for (size_t i = 0; i < bit_bufffer_size; i++) {
      if (j == buff_size) {
        j = 1;
      } else {
        j++;
      }
      bit_buffer.at(i) =
          bit_buffer.at(i) ^ std::byte(static_cast<int>(std::bitset<1>(1)[0] &
                                                        scram_buffer[j - 1]));
    }
  }
  ConvertBitsToBytes(bit_buffer.data(), num_bytes, output_buffer_ptr);
}

void Scrambler::Scramble(void* scrambled, const void* to_scramble,
                         size_t bytes_to_scramble) {
  WlanScrambler(scrambled, to_scramble, bytes_to_scramble, scram_buffer_,
                bit_buffer_);
}

void Scrambler::Scramble(void* inout_bytes, size_t bytes_to_scramble) {
  WlanScrambler(inout_bytes, inout_bytes, bytes_to_scramble, scram_buffer_,
                bit_buffer_);
}

void Scrambler::Descramble(void* descrambled, const void* scrambled,
                           size_t bytes_to_descramble) {
  WlanScrambler(descrambled, scrambled, bytes_to_descramble, scram_buffer_,
                bit_buffer_);
}
void Scrambler::Descramble(void* inout_bytes, size_t bytes_to_descramble) {
  WlanScrambler(inout_bytes, inout_bytes, bytes_to_descramble, scram_buffer_,
                bit_buffer_);
}

};  // namespace AgoraScrambler