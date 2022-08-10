/**
 * @file test_datatype_conversion.cc
 * @brief Unit tests for data type and bit-level conversions
 */
#include <gtest/gtest.h>

#include <bitset>

#include "comms-lib.h"
#include "datatype_conversion.h"
#include "utils_ldpc.h"

static constexpr size_t kSIMDTestNum = 1024;

TEST(Modulation, adapt_bits_for_mod_one) {
  static constexpr auto kModOrder = CommsLib::ModulationOrder::kQaM64;
  static constexpr size_t kInputBytes = 8;
  std::vector<uint8_t> input(kInputBytes);
  std::vector<uint8_t> output(std::ceil(kInputBytes * 8.0 / kModOrder));
  for (size_t i = 0; i < kInputBytes; i++) {
    input[i] = 0b11111111;
    output[i] = 0;
  }

  AdaptBitsForMod(&input[0], &output[0], kInputBytes, kModOrder);

  std::printf("adapt_bits_for_mod test input (%zu B): ", kInputBytes);
  for (size_t i = 0; i < kInputBytes; i++) {
    std::printf("%s ", std::bitset<8>(input[i]).to_string().c_str());
  }
  std::printf("\noutput (%zu B): ", output.size());
  for (unsigned char i : output) {
    std::printf("%s ", std::bitset<8>(i).to_string().c_str());
  }

  std::vector<uint8_t> regen_input(kInputBytes);
  AdaptBitsFromMod(&output[0], &regen_input[0], output.size(), kModOrder);

  std::printf("\nregenerated input (%zu B): ", kInputBytes);
  for (size_t i = 0; i < kInputBytes; i++) {
    std::printf("%s ", std::bitset<8>(regen_input[i]).to_string().c_str());
  }
  std::printf("\n");

  ASSERT_EQ(input, regen_input);
}

TEST(Modulation, adapt_bits_for_mod_stress) {
  std::vector<size_t> modulations = {CommsLib::ModulationOrder::kQaM16,
                                     CommsLib::ModulationOrder::kQaM64};

  for (size_t mod_type : modulations) {
    for (size_t iter = 0; iter < 1000; iter++) {
      const size_t num_input_bytes = rand() % 10000;
      std::vector<uint8_t> input(num_input_bytes);
      std::vector<uint8_t> regen_input(num_input_bytes);
      std::vector<uint8_t> output(std::ceil(num_input_bytes * 8.0 / mod_type));
      for (size_t i = 0; i < num_input_bytes; i++) {
        input[i] = rand();
        output[i] = 0;
      }

      AdaptBitsForMod(&input[0], &output[0], num_input_bytes, mod_type);

      // Sanity check: Input and output must have same number of set bits
      size_t set_bits_in_input = 0;
      size_t set_bits_in_output = 0;
      for (uint8_t& i : input) {
        set_bits_in_input += __builtin_popcount(i);
      }
      for (uint8_t& o : output) {
        set_bits_in_output += __builtin_popcount(o);
      }
      ASSERT_EQ(set_bits_in_input, set_bits_in_output);

      // Sanity check: Output bytes must have at most mod_type bits set
      for (uint8_t& o : output) {
        ASSERT_LE(o, ((1 << mod_type) - 1));
      }

      AdaptBitsFromMod(&output[0], &regen_input[0], output.size(), mod_type);

      ASSERT_EQ(input, regen_input);
    }
  }
}

TEST(SIMD, float_32_to_16) {
  constexpr float kAllowedError = 1e-3;
  auto* in_buf = static_cast<float*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, kSIMDTestNum * sizeof(float)));
  for (size_t i = 0; i < kSIMDTestNum; i++) {
    in_buf[i] = static_cast<float>(rand()) / (RAND_MAX * 1.0);
  }

  auto* medium = static_cast<float*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, kSIMDTestNum / 2 * sizeof(float)));
  SimdConvertFloat32ToFloat16(medium, in_buf, kSIMDTestNum);

  auto* out_buf = static_cast<float*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, kSIMDTestNum * sizeof(float)));
  SimdConvertFloat16ToFloat32(out_buf, medium, kSIMDTestNum);

  for (size_t i = 0; i < kSIMDTestNum; i++) {
    ASSERT_LE(abs(in_buf[i] - out_buf[i]), kAllowedError);
  }

  std::free(in_buf);
  std::free(medium);
  std::free(out_buf);
}

TEST(SIMD, int16_to_float) {
  //For avx512 the arrays must be multiples of 512bits
  const size_t array_size_bytes = 64;
  const size_t int16_elements = array_size_bytes / sizeof(int16_t);
  const size_t float_eq_elements = array_size_bytes / sizeof(float);
  const size_t float_array_expanded =
      (int16_elements / float_eq_elements) * array_size_bytes;
  //For avx512 the arrays must be multiples of 512bits
  auto* short_buf = static_cast<int16_t*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, array_size_bytes));

  auto* check_short = static_cast<int16_t*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, array_size_bytes));

  auto* float_buf = static_cast<float*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, float_array_expanded));

  auto* check_float = static_cast<float*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, float_array_expanded));

  auto* reconstruct = static_cast<int16_t*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, array_size_bytes));

  int16_t value = SHRT_MIN;
  const size_t simd_interations =
      (static_cast<size_t>(SHRT_MAX - SHRT_MIN) / int16_elements) + 1;

  //Convert to Float
  for (size_t i = 0; i < simd_interations; i++) {
    //Load Input
    for (size_t j = 0; j < int16_elements; j++) {
      short_buf[j] = value++;
    }
    //Convert
    SimdConvertShortToFloat(short_buf, float_buf, int16_elements);
    ConvertShortToFloat(short_buf, check_float, int16_elements);
    for (size_t j = 0; j < int16_elements; j++) {
      if (float_buf[j] != check_float[j]) {
        std::printf("Value Mismatch ShortToFloat - Simd %f, Conv %f\n",
                    float_buf[j], check_float[j]);
      }
    }

    //Convert Back
    SimdConvertFloatToShort(float_buf, reconstruct, int16_elements, 0, 1);
    ConvertFloatToShort(float_buf, check_short, int16_elements);
    for (size_t j = 0; j < int16_elements; j++) {
      if (reconstruct[j] != check_short[j]) {
        std::printf("Value Mismatch FloatToShort - Simd %d, Conv %d\n",
                    reconstruct[j], check_short[j]);
      }
    }

    //Check Output
    for (size_t j = 0; j < int16_elements; j++) {
      if (short_buf[j] != reconstruct[j]) {
        std::printf("Value Mismatch - Orig %d, Float %f, Conv %d\n",
                    short_buf[j], float_buf[j], reconstruct[j]);
      }
    }
  }
  std::free(short_buf);
  std::free(float_buf);
  std::free(reconstruct);
  std::free(check_float);
  std::free(check_short);
}

TEST(SIMD, int16_to_float_cplen) {
  const int16_t allowed_error = 1;
  //For avx512 the arrays must be multiples of 512bits
  const size_t test_elements = 128;
  const size_t test_cp_len = 32;
  const size_t test_scale = 512;
  //For avx512 the arrays must be multiples of 512bits
  auto* short_buf = static_cast<int16_t*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, test_elements * sizeof(short)));

  auto* check_short = static_cast<int16_t*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64,
      (test_elements + test_cp_len) * sizeof(short)));

  auto* float_buf = static_cast<float*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, test_elements * sizeof(float)));

  auto* check_float = static_cast<float*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, test_elements * sizeof(float)));

  auto* reconstruct = static_cast<int16_t*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64,
      (test_elements + test_cp_len) * sizeof(short)));

  int16_t value = SHRT_MIN;
  const size_t simd_interations =
      (static_cast<size_t>(SHRT_MAX - SHRT_MIN) / test_elements) + 1;

  //Convert to Float
  for (size_t i = 0; i < simd_interations; i++) {
    //Load Input
    for (size_t j = 0; j < test_elements; j++) {
      short_buf[j] = value++;
    }
    //Convert
    SimdConvertShortToFloat(short_buf, float_buf, test_elements);
    ConvertShortToFloat(short_buf, check_float, test_elements);
    for (size_t j = 0; j < test_elements; j++) {
      if (float_buf[j] != check_float[j]) {
        std::printf(
            "Value Mismatch ShortToFloat[%zu] - Simd %f, Conv %f, Diff %f\n", j,
            float_buf[j], check_float[j], float_buf[j] - check_float[j]);
      }
    }

    //Convert Back
    SimdConvertFloatToShort(float_buf, reconstruct, test_elements, test_cp_len,
                            test_scale);
    ConvertFloatToShort(float_buf, check_short, test_elements, test_cp_len,
                        test_scale);
    for (size_t j = 0; j < (test_elements + test_cp_len); j++) {
      const int16_t naive = check_short[j];
      const int16_t simd = reconstruct[j];
      if ((naive >= simd + allowed_error) && (naive <= simd - allowed_error)) {
        std::printf(
            "Value Mismatch FloatToShort[%zu] - Simd %d, Conv %d, Diff %d\n", j,
            simd, naive, simd - naive);
      }
    }

    //Check e2e conversions Output
    for (size_t j = 0; j < test_elements; j++) {
      const int16_t orig_vale = short_buf[j] / test_scale;
      const int16_t reconstructed = reconstruct[j + test_cp_len];
      if ((orig_vale >= reconstructed + allowed_error) &&
          (orig_vale <= reconstructed - allowed_error)) {
        std::printf(
            "Value Mismatch[%zu] - Orig %d, Float %f, Conv %d, Diff %d\n", j,
            orig_vale, float_buf[j], reconstructed, orig_vale - reconstructed);
      }
    }

    for (size_t j = 0; j < test_cp_len; j++) {
      const int16_t orig_vale =
          short_buf[j + (test_elements - test_cp_len)] / test_scale;
      const int16_t reconstructed = reconstruct[j];
      if ((orig_vale >= reconstructed + allowed_error) &&
          (orig_vale <= reconstructed - allowed_error)) {
        std::printf(
            "Value Mismatch[%zu] CPlen - Orig %d, Float %f, Conv %d, Diff "
            "%d\n",
            j, orig_vale, float_buf[j], reconstruct[j],
            orig_vale - reconstruct[j]);
      }
    }
  }
  std::free(short_buf);
  std::free(float_buf);
  std::free(reconstruct);
  std::free(check_float);
  std::free(check_short);
}

TEST(SIMD, float_to_int_saturate) {
  //For avx512 the arrays must be multiples of 512bits
  const size_t test_elements = 32;

  //For avx512 the arrays must be multiples of 512bits
  auto* check_smd = static_cast<int16_t*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, test_elements * sizeof(short)));

  auto* check = static_cast<int16_t*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, test_elements * sizeof(short)));

  auto* float_buf = static_cast<float*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, test_elements * sizeof(float)));

  //setup the input array
  std::memset(float_buf, 0, test_elements * sizeof(float));
  float_buf[0] = FLT_MIN;
  float_buf[1] = SHRT_MIN - 1;
  float_buf[2] = SHRT_MIN;
  float_buf[3] = -10;
  float_buf[4] = -2;
  float_buf[5] = -1;
  float_buf[6] = -0.999999;
  float_buf[7] = -0.5;
  float_buf[8] = 0;
  float_buf[9] = 0.5;
  float_buf[10] = 0.9999999;
  float_buf[11] = 1;
  float_buf[12] = 2;
  float_buf[13] = 10;
  float_buf[14] = static_cast<float>(SHRT_MAX);
  float_buf[15] = static_cast<float>(SHRT_MAX) + 1;
  float_buf[16] = static_cast<float>(INT_MAX);
  float_buf[17] = static_cast<float>(INT_MAX) + 1;
  float_buf[18] = FLT_MAX;

  //Convert to Short
  SimdConvertFloatToShort(float_buf, check_smd, test_elements, 0, 1);
  ConvertFloatToShort(float_buf, check, test_elements, 0, 1);
  for (size_t j = 0; j < test_elements; j++) {
    if (check_smd[j] != check[j]) {
      std::printf("Value Mismatch FloatToShort - Float %f,  Simd %d, Conv %d\n",
                  float_buf[j], check_smd[j], check[j]);
    }
  }
  std::free(float_buf);
  std::free(check_smd);
  std::free(check);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}