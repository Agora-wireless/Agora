#include <gtest/gtest.h>

#include "comms-lib.h"
#include "gettime.h"

#ifdef __AVX512F__
#define NUM_RUNS 1000000  // number of runs, for benchmarking

TEST(TestComplexMul, Multiply) {
  float values[32] __attribute((aligned(64)));
  float out256[16] __attribute((aligned(64)));
  float out512[16] __attribute((aligned(64)));
  __m256 result_256_lower __attribute((aligned(64)));
  __m256 result_256_upper __attribute((aligned(64)));
  __m512 result_512 __attribute((aligned(64)));
  uint64_t ticks;
  for (float& value : values) {
    // Set each float to a random value between -1 and 1
    value = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
  }
  /* Load the generated values */
  __m256 values0_lower_256 =
      _mm256_load_ps(reinterpret_cast<const float*>(values));
  __m256 values0_upper_256 =
      _mm256_load_ps(reinterpret_cast<const float*>(values + 8));
  __m256 values1_lower_256 =
      _mm256_load_ps(reinterpret_cast<const float*>(values + 16));
  __m256 values1_upper_256 =
      _mm256_load_ps(reinterpret_cast<const float*>(values + 24));
  __m512 values0_512 = _mm512_load_ps(reinterpret_cast<const float*>(values));
  __m512 values1_512 =
      _mm512_load_ps(reinterpret_cast<const float*>(values + 16));
  /* Do the multiplication */
  ticks = GetTime::Rdtsc();
  for (int i = 0; i < NUM_RUNS; i++) {
    result_256_lower = CommsLib::M256ComplexCf32Mult(values0_lower_256,
                                                     values1_lower_256, false);
    result_256_upper = CommsLib::M256ComplexCf32Mult(values0_upper_256,
                                                     values1_upper_256, false);
  }
  ticks = GetTime::Rdtsc() - ticks;
  std::cout << "AVX256 Multiplication took " << ticks << "\n";
  ticks = GetTime::Rdtsc();
  for (int i = 0; i < NUM_RUNS; i++) {
    result_512 = CommsLib::M512ComplexCf32Mult(values0_512, values1_512, false);
  }
  ticks = GetTime::Rdtsc() - ticks;
  std::cout << "AVX512 Multiplication took " << ticks << "\n";
  /* Extract the results into output buffers */
  _mm256_stream_ps(reinterpret_cast<float*>(out256), result_256_lower);
  _mm256_stream_ps(reinterpret_cast<float*>(out256 + 8), result_256_upper);
  _mm512_stream_ps(reinterpret_cast<float*>(out512), result_512);
  ASSERT_EQ(memcmp(out512, out256, sizeof(out512)), 0)
      << "AVX512 and AVX256 multiplication differ";
}

TEST(TestComplexMul, ConjMultiply) {
  float values[32] __attribute((aligned(64)));
  float out256[16] __attribute((aligned(64)));
  float out512[16] __attribute((aligned(64)));
  __m256 result_256_lower __attribute((aligned(64)));
  __m256 result_256_upper __attribute((aligned(64)));
  __m512 result_512 __attribute((aligned(64)));
  uint64_t ticks;
  for (float& value : values) {
    // Set each float to a random value between -1 and 1
    value = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
  }
  /* Load the generated values */
  __m256 values0_lower_256 =
      _mm256_load_ps(reinterpret_cast<const float*>(values));
  __m256 values0_upper_256 =
      _mm256_load_ps(reinterpret_cast<const float*>(values + 8));
  __m256 values1_lower_256 =
      _mm256_load_ps(reinterpret_cast<const float*>(values + 16));
  __m256 values1_upper_256 =
      _mm256_load_ps(reinterpret_cast<const float*>(values + 24));
  __m512 values0_512 = _mm512_load_ps(reinterpret_cast<const float*>(values));
  __m512 values1_512 =
      _mm512_load_ps(reinterpret_cast<const float*>(values + 16));
  /* Do the multiplication */
  ticks = GetTime::Rdtsc();
  for (int i = 0; i < NUM_RUNS; i++) {
    result_256_lower = CommsLib::M256ComplexCf32Mult(values0_lower_256,
                                                     values1_lower_256, true);
    result_256_upper = CommsLib::M256ComplexCf32Mult(values0_upper_256,
                                                     values1_upper_256, true);
  }
  ticks = GetTime::Rdtsc() - ticks;
  std::cout << "AVX256 Conj Multiplication took " << ticks << "\n";
  ticks = GetTime::Rdtsc();
  for (int i = 0; i < NUM_RUNS; i++) {
    result_512 = CommsLib::M512ComplexCf32Mult(values0_512, values1_512, true);
  }
  ticks = GetTime::Rdtsc() - ticks;
  std::cout << "AVX512 Conj Multiplication took " << ticks << "\n";
  /* Extract the results into output buffers */
  _mm256_stream_ps(reinterpret_cast<float*>(out256), result_256_lower);
  _mm256_stream_ps(reinterpret_cast<float*>(out256 + 8), result_256_upper);
  _mm512_stream_ps(reinterpret_cast<float*>(out512), result_512);
  ASSERT_EQ(memcmp(out512, out256, sizeof(out512)), 0)
      << "AVX512 and AVX256 conjugate multiplication differ";
}

#endif

int main(int argc, char** argv) {
#ifdef __AVX512F__
  testing::InitGoogleTest(&argc, argv);
  std::cout << "---- CommsLib AVX512 Channel Estimation ----\n";
  return RUN_ALL_TESTS();
#else
  (void)argc;
  (void)argv;
  std::cout << "Platform does not support AVX512!\n";
  return 0;
#endif
}
