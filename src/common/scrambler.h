/**
 * @file  scrambler.h
 * @brief Scramble Class and helper functions
 */
#ifndef SCRAMBLER_H_
#define SCRAMBLER_H_

#include <cstdio>
#include <functional>
#include <iostream>
#include <vector>

namespace AgoraScrambler {
static constexpr int8_t kScramblerInitState = 93;  // [1, 127]
static constexpr int8_t kScramblerlength = 127;

class Scrambler {
 public:
  Scrambler();
  ~Scrambler() = default;

  void Scramble(void* byte_buffer, size_t byte_buffer_size);
  void Descramble(void* byte_buffer, size_t byte_buffer_size);

 private:
  /**
   * @brief                        WLAN Scrambler of IEEE 802.11-2012
   *
   * Section 18.3.5.5. The same scrambler is used to both scramble bits at the
   * transmitter and descramble at the receiver.
   *
   * The input is scrambled with a length-127 frame-synchronous scrambler using
   * the generator polynomial s(x) = x7 + x4 + 1 and a pseudorandom nonzero
   * initial state (default 0x5D), which is an integer picked in the range
   * [1,127]. The mapping of the seed to the generator is Bit0 ~ Bit6 to x1 ~
   * x7. The output is the scrambld data of the same size and type as the input.
   *
   * @param  byte_buffer           Byte array for both input and scrambled data
   * @param  byte_buffer_size      Byte array size
   * @param  scram_init            Scamber initial state
   */
  void WlanScrambler(void* byte_buffer, size_t byte_buffer_size,
                     std::vector<int8_t>& scram_buffer,
                     std::vector<int8_t>& bit_buffer);

  /**
   * @brief                        Convert a byte array to a bit array. MSB
   * first
   *
   * @param  in_byte_buffer        Input byte array
   * @param  byte_buffer_size      Input byte array size
   * @param  out_bit_buffer        Output bit array
   */
  void ConvertBitsToBytes(const int8_t* in_bit_buffer, size_t byte_buffer_size,
                          int8_t* out_byte_buffer);

  /**
   * @brief                        Convert a bit array to a byte array. MSB
   * first
   *
   * @param  in_bit_buffer         Input bit array
   * @param  byte_buffer_size      Output byte array size
   * @param  out_byte_buffer       Output byte array
   */
  void ConvertBytesToBits(const int8_t* in_byte_buffer, size_t byte_buffer_size,
                          int8_t* out_bit_buffer);

  std::vector<int8_t> scram_buffer_;
  std::vector<int8_t> bit_buffer_;
};  // class Scrambler

};  // namespace AgoraScrambler

#endif  // SCRAMBLER_H_