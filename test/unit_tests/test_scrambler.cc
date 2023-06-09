/**
 * @file test_scrambler.cc
 * @brief Unit tests for WLAN scrmabler and descrambler
 */

#include <gtest/gtest.h>

#include <ctime>
#include <vector>

#include "scrambler.h"
#include "utils_ldpc.h"

static constexpr size_t kNumInputBytes = 125;

/**
 * @brief  Construct a new TEST object
 *
 * The test data and the expected output are hardcoded in byte_buffer[] and
 * expect[] as int8_t.
 */
TEST(WLAN_Scrambler, fixed_input_scramble_int8_t) {
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

  auto scrambler = std::make_unique<AgoraScrambler::Scrambler>();
  scrambler->Scramble(byte_buffer, kNumInputBytes);

  for (size_t i = 0; i < kNumInputBytes; i++) {
    ASSERT_EQ(byte_buffer[i], expect[i]);
  }
}

/**
 * @brief  Construct a new TEST object
 *
 * The test data and the expected output are hardcoded in byte_buffer[] and
 * expect[] as uint8_t.
 */
TEST(WLAN_Scrambler, fixed_input_scramble_uint8_t) {
  uint8_t byte_buffer[kNumInputBytes] = {
      1,   2,   3,   4,   5,   6,   7,   8,   9,   10,  11,  12,  13,  14,
      15,  16,  17,  18,  19,  20,  21,  22,  23,  24,  25,  26,  27,  28,
      29,  30,  31,  32,  33,  34,  35,  36,  37,  38,  39,  40,  41,  42,
      43,  44,  45,  46,  47,  48,  49,  50,  51,  52,  53,  54,  55,  56,
      57,  58,  59,  60,  61,  62,  63,  64,  65,  66,  67,  68,  69,  70,
      71,  72,  73,  74,  75,  76,  77,  78,  79,  80,  81,  82,  83,  84,
      85,  86,  87,  88,  89,  90,  91,  92,  93,  94,  95,  96,  97,  98,
      99,  100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112,
      113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125};
  uint8_t expect[kNumInputBytes] = {
      109, 27,  170, 203, 109, 83,  243, 171, 120, 246, 48,  199, 41,  6,
      151, 170, 201, 33,  64,  138, 197, 189, 254, 94,  250, 226, 108, 138,
      85,  15,  46,  85,  145, 68,  132, 25,  132, 113, 245, 165, 238, 218,
      196, 0,   189, 12,  77,  219, 81,  255, 125, 79,  119, 153, 146, 35,
      182, 219, 229, 101, 29,  122, 250, 150, 128, 216, 223, 178, 192, 25,
      13,  127, 86,  137, 247, 254, 13,  199, 196, 253, 210, 103, 106, 185,
      95,  232, 195, 54,  102, 221, 34,  56,  220, 77,  72,  59,  103, 8,
      16,  190, 112, 27,  79,  180, 22,  100, 153, 165, 111, 72,  65,  198,
      125, 166, 148, 192, 95,  140, 38,  192, 135, 103, 158, 238, 121};

  auto scrambler = std::make_unique<AgoraScrambler::Scrambler>();
  scrambler->Scramble(byte_buffer, kNumInputBytes);

  for (size_t i = 0; i < kNumInputBytes; i++) {
    ASSERT_EQ(byte_buffer[i], expect[i]);
  }
}

/**
 * @brief  Construct a new TEST object
 *
 * The test data is generated inside the test as random integers of 1 byte.
 *
 * The expected output is the same as the test data, which is scrambled and
 * then descrambled.
 */
TEST(WLAN_Scrambler, random_input_scramble_descramble_inplace) {
  int8_t* byte_buffer;
  int8_t* byte_buffer_orig;
  byte_buffer = (int8_t*)std::calloc(kNumInputBytes, sizeof(int8_t));
  byte_buffer_orig = (int8_t*)std::calloc(kNumInputBytes, sizeof(int8_t));

  auto scrambler = std::make_unique<AgoraScrambler::Scrambler>();

  srand(time(nullptr));
  for (size_t i = 0; i < kNumInputBytes; i++) {
    byte_buffer[i] = rand() % 128 - 127;
    byte_buffer_orig[i] = byte_buffer[i];
  }

  // Scramble
  scrambler->Scramble(byte_buffer, kNumInputBytes);

  // Descramble
  scrambler->Descramble(byte_buffer, kNumInputBytes);

  for (size_t i = 0; i < kNumInputBytes; i++) {
    ASSERT_EQ(byte_buffer[i], byte_buffer_orig[i]);
  }

  std::free(byte_buffer);
  std::free(byte_buffer_orig);
}

/**
 * @brief  random_input_scramble_descramble
 *
 * The test data is generated inside the test as random integers of 1 byte.
 *
 * The expected output is the same as the test data, which is scrambled and
 * then descrambled.
 */
TEST(WLAN_Scrambler, random_input_scramble_descramble) {
  int8_t* byte_buffer;
  int8_t* byte_buffer_orig;
  byte_buffer = (int8_t*)std::calloc(kNumInputBytes, sizeof(int8_t));
  byte_buffer_orig = (int8_t*)std::calloc(kNumInputBytes, sizeof(int8_t));

  auto scrambler = std::make_unique<AgoraScrambler::Scrambler>();

  srand(time(nullptr));
  for (size_t i = 0; i < kNumInputBytes; i++) {
    byte_buffer_orig[i] = rand() % 128 - 127;
  }

  // Scramble (don't modify the input)
  scrambler->Scramble(byte_buffer, (const int8_t*)byte_buffer_orig,
                      kNumInputBytes);

  // Descramble (inplace)
  scrambler->Descramble(byte_buffer, kNumInputBytes);

  for (size_t i = 0; i < kNumInputBytes; i++) {
    ASSERT_EQ(byte_buffer[i], byte_buffer_orig[i]);
  }

  std::free(byte_buffer);
  std::free(byte_buffer_orig);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}