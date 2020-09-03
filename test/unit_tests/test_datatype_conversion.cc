/**
 * @file test_datatype_conversion.cc
 * @brief Unit tests for data type and bit-level conversions
 */

#include "comms-lib.h"
#include "datatype_conversion.h"
#include "utils_ldpc.hpp"
#include <bitset>
#include <gtest/gtest.h>
#include <malloc.h>

static constexpr size_t kSIMDTestNum = 1024;

TEST(Modulation, adapt_bits_for_mod_one)
{
    static constexpr auto kModOrder = CommsLib::ModulationOrder::QAM64;
    static constexpr size_t kInputBytes = 8;
    std::vector<uint8_t> input(kInputBytes);
    std::vector<uint8_t> output(std::ceil(kInputBytes * 8.0 / kModOrder));
    for (size_t i = 0; i < kInputBytes; i++) {
        input[i] = 0b11111111;
        output[i] = 0;
    }

    adapt_bits_for_mod(&input[0], &output[0], kInputBytes, kModOrder);

    printf("adapt_bits_for_mod test input (%zu B): ", kInputBytes);
    for (size_t i = 0; i < kInputBytes; i++) {
        printf("%s ", std::bitset<8>(input[i]).to_string().c_str());
    }
    printf("\noutput (%zu B): ", output.size());
    for (size_t i = 0; i < output.size(); i++) {
        printf("%s ", std::bitset<8>(output[i]).to_string().c_str());
    }

    std::vector<uint8_t> regen_input(kInputBytes);
    adapt_bits_from_mod(&output[0], &regen_input[0], output.size(), kModOrder);

    printf("\nregenerated input (%zu B): ", kInputBytes);
    for (size_t i = 0; i < kInputBytes; i++) {
        printf("%s ", std::bitset<8>(regen_input[i]).to_string().c_str());
    }
    printf("\n");

    ASSERT_EQ(input, regen_input);
}

TEST(Modulation, adapt_bits_for_mod_stress)
{
    std::vector<size_t> modulations = { CommsLib::ModulationOrder::QAM16,
        CommsLib::ModulationOrder::QAM64 };

    for (size_t mod_type : modulations) {
        for (size_t iter = 0; iter < 1000; iter++) {
            const size_t num_input_bytes = rand() % 10000;
            std::vector<uint8_t> input(num_input_bytes);
            std::vector<uint8_t> regen_input(num_input_bytes);
            std::vector<uint8_t> output(
                std::ceil(num_input_bytes * 8.0 / mod_type));
            for (size_t i = 0; i < num_input_bytes; i++) {
                input[i] = rand();
                output[i] = 0;
            }

            adapt_bits_for_mod(
                &input[0], &output[0], num_input_bytes, mod_type);

            // Sanity check: Input and output must have same number of set bits
            size_t set_bits_in_input = 0;
            size_t set_bits_in_output = 0;
            for (uint8_t& i : input)
                set_bits_in_input += __builtin_popcount(i);
            for (uint8_t& o : output)
                set_bits_in_output += __builtin_popcount(o);
            ASSERT_EQ(set_bits_in_input, set_bits_in_output);

            // Sanity check: Output bytes must have at most mod_type bits set
            for (uint8_t& o : output) {
                ASSERT_LE(o, ((1 << mod_type) - 1));
            }

            adapt_bits_from_mod(
                &output[0], &regen_input[0], output.size(), mod_type);

            ASSERT_EQ(input, regen_input);
        }
    }
}

TEST(SIMD, float_32_to_16)
{
    constexpr float allowed_error = 1e-3;
    float* in_buf
        = reinterpret_cast<float*>(memalign(64, kSIMDTestNum * sizeof(float)));
    for (size_t i = 0; i < kSIMDTestNum; i++) {
        in_buf[i] = static_cast<float>(rand()) / (RAND_MAX * 1.0);
    }

    float* medium = reinterpret_cast<float*>(
        memalign(64, kSIMDTestNum / 2 * sizeof(float)));
    simd_convert_float32_to_float16(medium, in_buf, kSIMDTestNum);

    float* out_buf
        = reinterpret_cast<float*>(memalign(64, kSIMDTestNum * sizeof(float)));
    simd_convert_float16_to_float32(out_buf, medium, kSIMDTestNum);

    for (size_t i = 0; i < kSIMDTestNum; i++) {
        ASSERT_LE(abs(in_buf[i] - out_buf[i]), allowed_error);
    }

    free(in_buf);
    free(medium);
    free(out_buf);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}