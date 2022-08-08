/**
 * @file test_datatype_conversion.cc
 * @brief Unit tests for data type and bit-level conversions
 */
#include <gtest/gtest.h>

#include <bitset>

#include "comms-lib.h"
#include "datatype_conversion.h"
#include "utils_ldpc.h"

//Should define
#define DATATYPE_MEMORY_CHECK

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
  //const size_t float_elements = int16_elements;
  //For avx512 the arrays must be multiples of 512bits
  auto* in_buf = static_cast<int16_t*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, array_size_bytes));

  auto* float_buf = static_cast<float*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, float_array_expanded));

  auto* regen_buf = static_cast<int16_t*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, array_size_bytes));

  std::printf("Elements (%zu) Bytes (%zu:%zu)\n", int16_elements,
              array_size_bytes, float_array_expanded);

  int16_t value = SHRT_MIN;
  const size_t simd_interations =
      (static_cast<size_t>(SHRT_MAX - SHRT_MIN) / int16_elements) + 1;
  //const size_t simd_interations = 1;

  //Convert to Float
  for (size_t i = 0; i < simd_interations; i++) {
    //Load Input
    for (size_t j = 0; j < int16_elements; j++) {
      in_buf[j] = value++;
    }
    //Convert
    SimdConvertShortToFloat(in_buf, float_buf, int16_elements);

    //Check Output
    for (size_t j = 0; j < int16_elements; j++) {
      if (i % 128 == 0) {
        //std::printf("In %d, Out %f\n", in_buf[j], float_buf[j]);
      }
    }

    //Convert Back
    SimdConvertFloatToShort(float_buf, regen_buf, int16_elements, 0, 1);

    //Check Output
    for (size_t j = 0; j < int16_elements; j++) {
      if (in_buf[j] != regen_buf[j]) {
        std::printf("Value Mismatch - Orig %d, Float %f, Conv %d\n", in_buf[j],
                    float_buf[j], regen_buf[j]);
      }
      //std::printf("Orig %d, Float %f, Conv %d\n", in_buf[j], float_buf[j],
      //            regen_buf[j]);
    }
  }

  //for (size_t j = 0; j < int16_elements; j++) {
  //  std::printf("In %d, Out %f\n", in_buf[j], out_buf[j]);
  //}
  std::free(in_buf);
  std::free(float_buf);
  std::free(regen_buf);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}