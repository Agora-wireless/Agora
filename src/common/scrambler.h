/**
 * @file  scrambler.h
 * @brief Scramble Class and helper functions
 */
#ifndef SCRAMBLER_H_
#define SCRAMBLER_H_

#include <bitset>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace AgoraScrambler {
// [1, 127] (93)
static constexpr uint8_t kScramblerInitState = 0x5D;
static constexpr uint8_t kScramblerlength = 127;

class Scrambler {
 public:
  Scrambler();
  ~Scrambler() = default;

  void Scramble(void* scrambled, const void* to_scramble,
                size_t bytes_to_scramble);
  void Scramble(void* inout_bytes, size_t bytes_to_scramble);
  void Descramble(void* descrambled, const void* scrambled,
                  size_t bytes_to_descramble);
  void Descramble(void* inout_bytes, size_t bytes_to_descramble);

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
   * @param  output_buffer         Byte array for output scrambled data (can the the same as input)
   * @param  input_buffer          Byte array for input to be scrambled
   * @param  num_bytes             Byte array size - number of bytes to scramble / descramble
   * @param  scram_buffer          Scratch memory
   * @param  bit_buffer            Scratch memory
   */
  static void WlanScrambler(void* output_buffer, const void* input_buffer,
                            size_t num_bytes,
                            std::bitset<kScramblerlength>& scram_buffer,
                            std::vector<std::byte>& bit_buffer);

  /**
   * @brief                        Convert a byte array to a bit array. MSB
   * first
   *
   * @param  in_byte_buffer        Input byte array
   * @param  byte_buffer_size      Input byte array size
   * @param  out_bit_buffer        Output bit array
   */
  static void ConvertBytesToBits(const std::byte* in_byte_buffer,
                                 size_t byte_buffer_size,
                                 std::byte* out_bit_buffer);

  /**
   * @brief                        Convert a bit array to a byte array. MSB
   * first
   *
   * @param  in_bit_buffer         Input bit array
   * @param  byte_buffer_size      Output byte array size
   * @param  out_byte_buffer       Output byte array
   */
  static void ConvertBitsToBytes(const std::byte* in_bit_buffer,
                                 size_t byte_buffer_size,
                                 std::byte* out_byte_buffer);

  std::bitset<kScramblerlength> scram_buffer_;
  std::vector<std::byte> bit_buffer_;
};  // class Scrambler

};  // namespace AgoraScrambler

#endif  // SCRAMBLER_H_