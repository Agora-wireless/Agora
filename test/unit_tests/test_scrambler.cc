/**
 * @file test_scrambler.cc
 * @brief Unit tests for WLAN scrmabler and descrambler
 */

#include <gtest/gtest.h>
#include <time.h>
#include <vector>
#include "utils_ldpc.hpp"

static constexpr size_t kNumInputBytes = 125;

/**
 * @brief  Construct a new TEST object
 *
 * The test data and the expected output are hardcoded in byte_buffer[] and
 * expect[].
 */
TEST(Scrambler, fixed_input_scramble_only) {
  int8_t byte_buffer[kNumInputBytes] = {
      -121, 104, 66,  23,   126, -69,  -50,  111,  -120, 36,   -13,  67,   -107,
      30,   118, 89,  -61,  -21, -127, -127, -69,  -22,  -100, 108,  85,   102,
      -101, 104, -39, -122, 81,  6,    -118, -79,  -122, -83,  -105, 40,   -21,
      78,   -82, 59,  -109, 108, 50,   -22,  7,    -99,  -8,   20,   35,   -9,
      -91,  96,  100, -45,  -21, 116,  -111, -112, 2,    -105, 125,  -46,  -76,
      75,   -32, -68, 121,  -77, -21,  -84,  -57,  -84,  -50,  28,   -112, 85,
      16,   63,  59,  -61,  34,  104,  6,    -10,  -90,  94,   22,   42,   -82,
      99,   119, 126, -25,  -67, 26,   -65,  77,   -113, -4,   64,   -10,  21,
      -78,  125, 82,  28,   39,  -86,  -106, -42,  94,   -29,  -53,  102,  -40,
      -100, -35, 87,  -104, -7,  -56,  19,   -94};
  int8_t expect[kNumInputBytes] = {
      -21,  113,  -21,  -40,  22,  -18,  58,   -52, -7,  -40,  -56, -120, -79,
      22,   -18,  -29,  27,   -40, -46,  31,   107, 65,  117,  42,  -74,  -98,
      -20,  -2,   -111, -105, 96,  115,  58,   -41, 33,  -112, 54,  127,  57,
      -61,  105,  -53,  124,  64,  -94,  -56,  101, 118, -104, -39, 109,  -116,
      -25,  -49,  -63,  -56,  100, -107, 79,   -55, 34,  -45,  -72, 4,    117,
      -47,  124,  74,   -4,   -20, -95,  -101, -40, 111, 114,  -82, -48,  -36,
      -101, -110, -72,  -10,  27,  -123, 12,   72,  50,  48,   41,  -83,  -41,
      7,    -10,  109,  -16,  -26, 28,   -43,  62,  85,  -23,  61,  -34,  -55,
      -51,  115,  -96,  -43,  37,  -116, -72,  96,  82,  55,   44,  -46,  -14,
      102,  -116, -17,  102,  -28, 45,   -127, -90};

  WlanScramble(byte_buffer, kNumInputBytes, kScramblerInitState);

  for (size_t i = 0; i < kNumInputBytes; i++)
    ASSERT_EQ(byte_buffer[i], expect[i]);
}

/**
 * @brief  Construct a new TEST object
 *
 * The test data is generated inside the test as random integers of 1 byte.
 *
 * The expected output is the same as the test data, which is scrambled and
 * then descrambled.
 */
TEST(Scrambler, random_input_scramble_descramble) {
  int8_t* byte_buffer;
  int8_t* byte_buffer_orig;
  byte_buffer = (int8_t*)std::calloc(kNumInputBytes, sizeof(int8_t));
  byte_buffer_orig = (int8_t*)std::calloc(kNumInputBytes, sizeof(int8_t));

  srand(time(0));
  for (size_t i = 0; i < kNumInputBytes; i++) {
    byte_buffer[i] = rand() % 128 - 127;
    byte_buffer_orig[i] = byte_buffer[i];
  }

  // Scramble
  WlanScramble(byte_buffer, kNumInputBytes, kScramblerInitState);

  // Descramble
  WlanScramble(byte_buffer, kNumInputBytes, kScramblerInitState);

  for (size_t i = 0; i < kNumInputBytes; i++)
    ASSERT_EQ(byte_buffer[i], byte_buffer_orig[i]);

  std::free(byte_buffer);
  std::free(byte_buffer_orig);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}