#include <gtest/gtest.h>

#include <cstdio>
#include <cstring>
#include <random>

#include "dodemul.h"
#include "gettime.h"

TEST(TestSimdStore, SingleMethodTest) {
  size_t ant_num_done_512 = 0;
  size_t ant_num_done_256 = 0;
  size_t ant_num_done_avxloop = 0;
  Config cfg = Config("../data/bs-ul-sim.json");
  ASSERT_TRUE(cfg.BsAntNum() >= 8) << "Too few antennas";
  if (cfg.BsAntNum() % 8) return;

  static const size_t memory_size_bytes =
      kSCsPerCacheline * kMaxAntennas * sizeof(complex_float);

  auto* src = static_cast<complex_float*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, memory_size_bytes));

  auto* dst_512 = static_cast<complex_float*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, memory_size_bytes));

  auto* dst_256 = static_cast<complex_float*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, memory_size_bytes));

  auto* dst_avxloop = static_cast<complex_float*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, memory_size_bytes));

  auto* dst_genloop = static_cast<complex_float*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, memory_size_bytes));

  auto* dst_loop = static_cast<complex_float*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, memory_size_bytes));

  std::memset(dst_512, 0u, memory_size_bytes);
  std::memset(dst_256, 0u, memory_size_bytes);
  std::memset(dst_avxloop, 0u, memory_size_bytes);
  std::memset(dst_genloop, 0u, memory_size_bytes);
  std::memset(dst_loop, 0u, memory_size_bytes);

  std::default_random_engine generator;
  std::uniform_real_distribution<float> distribution(-1.0, 1.0);

  for (size_t i = 0; i < (memory_size_bytes / sizeof(complex_float)); i++) {
    //Fill Randomly
    src[i].re = distribution(generator);
    src[i].im = distribution(generator);
  }

  for (size_t i = 0; i < cfg.DemulBlockSize(); i += kSCsPerCacheline) {
    ant_num_done_512 = StoreData::StoreRxDataAVX512(dst_512, src, &cfg, i);
    ant_num_done_256 = StoreData::StoreRxDataAVX2(dst_256, src, &cfg, i);
    ant_num_done_avxloop = StoreData::StoreRxDataAVXLoop(dst_avxloop, src, &cfg, i);
    StoreData::StoreRxDataGeneralLoop(dst_genloop, src, &cfg, i, 0);
    StoreData::StoreRxDataLoop(dst_loop, src, &cfg, i, 0);
  }

  ASSERT_TRUE(ant_num_done_512 == cfg.BsAntNum()
      && ant_num_done_256 == cfg.BsAntNum()
      && ant_num_done_avxloop == cfg.BsAntNum())
          << " AntNum: " << cfg.BsAntNum()
          << " AVX512: " << ant_num_done_512
          << " AVX2: " << ant_num_done_256;

  ASSERT_TRUE(std::memcmp(dst_512, dst_loop, memory_size_bytes) == 0)
      << "AVX512 and Loop did not output the same result";
  ASSERT_TRUE(std::memcmp(dst_256, dst_loop, memory_size_bytes) == 0)
      << "AVX2 and Loop did not output the same result";
  ASSERT_TRUE(std::memcmp(dst_avxloop, dst_loop, memory_size_bytes) == 0)
      << "AVXLoop and Loop did not output the same result";
  ASSERT_TRUE(std::memcmp(dst_genloop, dst_loop, memory_size_bytes) == 0)
      << "GeneralLoop and Loop did not output the same result";
  
  std::free(src);
  std::free(dst_512);
  std::free(dst_256);
  std::free(dst_avxloop);
  std::free(dst_genloop);
  std::free(dst_loop);
}

TEST(TestSimdStore, CombinedMethodTest) {
  size_t ant_num_done = 0;
  Config cfg = Config("../data/bs-ul-sim.json");

  static const size_t memory_size_bytes =
      kSCsPerCacheline * kMaxAntennas * sizeof(complex_float);

  auto* src = static_cast<complex_float*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, memory_size_bytes));

  auto* dst_512 = static_cast<complex_float*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, memory_size_bytes));

  auto* dst_256 = static_cast<complex_float*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, memory_size_bytes));

  auto* dst_avxloop = static_cast<complex_float*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, memory_size_bytes));

  auto* dst_genloop = static_cast<complex_float*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, memory_size_bytes));

  auto* dst_loop = static_cast<complex_float*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, memory_size_bytes));

  std::memset(dst_512, 0u, memory_size_bytes);
  std::memset(dst_256, 0u, memory_size_bytes);
  std::memset(dst_avxloop, 0u, memory_size_bytes);
  std::memset(dst_genloop, 0u, memory_size_bytes);
  std::memset(dst_loop, 0u, memory_size_bytes);

  std::default_random_engine generator;
  std::uniform_real_distribution<float> distribution(-1.0, 1.0);

  for (size_t i = 0; i < (memory_size_bytes / sizeof(complex_float)); i++) {
    //Fill Randomly
    src[i].re = distribution(generator);
    src[i].im = distribution(generator);
  }

  for (size_t i = 0; i < cfg.DemulBlockSize(); i += kSCsPerCacheline) {
    ant_num_done = StoreData::StoreRxDataAVX512(dst_512, src, &cfg, i);
    if (ant_num_done < cfg.BsAntNum()) {
      StoreData::StoreRxDataGeneralLoop(dst_512, src, &cfg, i, ant_num_done);
    }
    ant_num_done = StoreData::StoreRxDataAVX2(dst_256, src, &cfg, i);
    if (ant_num_done < cfg.BsAntNum()) {
      StoreData::StoreRxDataGeneralLoop(dst_256, src, &cfg, i, ant_num_done);
    }
    ant_num_done = StoreData::StoreRxDataAVXLoop(dst_avxloop, src, &cfg, i);
    if (ant_num_done < cfg.BsAntNum()) {
      StoreData::StoreRxDataGeneralLoop(dst_avxloop, src, &cfg, i, ant_num_done);
    }
    StoreData::StoreRxDataGeneralLoop(dst_genloop, src, &cfg, i, 0);
    StoreData::StoreRxDataLoop(dst_loop, src, &cfg, i, 0);
  }

  ASSERT_TRUE(std::memcmp(dst_genloop, dst_loop, memory_size_bytes) == 0)
      << "GeneralLoop and Loop did not output the same result";
  ASSERT_TRUE(std::memcmp(dst_avxloop, dst_loop, memory_size_bytes) == 0)
      << "AVXLoop and Loop did not output the same result";
  ASSERT_TRUE(std::memcmp(dst_512, dst_256, memory_size_bytes) == 0)
      << "AVX512 and AVX2 did not output the same result";
  ASSERT_TRUE(std::memcmp(dst_256, dst_loop, memory_size_bytes) == 0)
      << "AVX2 and Loop did not output the same result";

  std::free(src);
  std::free(dst_512);
  std::free(dst_256);
  std::free(dst_avxloop);
  std::free(dst_genloop);
  std::free(dst_loop);
}

int main(int argc, char** argv) {
#ifdef __AVX512F__
  testing::InitGoogleTest(&argc, argv);
  std::cout << "---- Simd Storage ----\n";
  return RUN_ALL_TESTS();
#endif
}
