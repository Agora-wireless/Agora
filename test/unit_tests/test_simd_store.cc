#include <gtest/gtest.h>

#include <cstdio>
#include <cstring>
#include <random>

#include "dodemul.h"
#include "gettime.h"

TEST(TestSimdStore, StorageTest) {
  size_t ant_test_number = 32;
  size_t ant_num_simd_512 = 0;
  size_t ant_num_simd_256 = 0;

  static const size_t memory_size_bytes =
      kSCsPerCacheline * kMaxAntennas * sizeof(complex_float);

  float* data_gather_buffer_ =
      static_cast<float*>(Agora_memory::PaddedAlignedAlloc(
          Agora_memory::Alignment_t::kAlign64, memory_size_bytes));

  float* dst_512 = static_cast<float*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, memory_size_bytes));

  float* dst_256 = static_cast<float*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, memory_size_bytes));

  std::default_random_engine generator;
  std::uniform_real_distribution<float> distribution(-1.0, 1.0);

  //Fill Randomly
  for (size_t i = 0; i < (memory_size_bytes / sizeof(float)); i++) {
    data_gather_buffer_[i] = distribution(generator);
  }
  std::memset(dst_512, 0u, memory_size_bytes);
  std::memset(dst_256, 0u, memory_size_bytes);

  ant_num_simd_512 = StoreData::StoreRxDataAVX512(data_gather_buffer_, dst_512,
                                                  ant_test_number);
  ant_num_simd_256 =
      StoreData::StoreRxDataAVX2(data_gather_buffer_, dst_256, ant_test_number);
  //ant_num_simd = StoreData::StoreRxDataLoop(data_gather_buffer_, dst, ant_test_number);

  //ASSERT_EQ(ant_num_simd_512, ant_num_simd_256)
  //    << "AVX512 and AVX256 store same value";

  ASSERT_EQ(std::memcmp(dst_512, dst_256, memory_size_bytes), 0)
      << "AVX512 and AVX256 did not return the same result";
  std::free(data_gather_buffer_);
  std::free(dst_512);
  std::free(dst_256);
}

int main(int argc, char** argv) {
#ifdef __AVX512F__
  testing::InitGoogleTest(&argc, argv);
  std::cout << "---- Simd Storage ----\n";
  return RUN_ALL_TESTS();
#endif
}
