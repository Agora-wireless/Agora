#include <gtest/gtest.h>

#include <cstdio>
#include <cstring>
#include <random>

#include "dodemul.h"
#include "gettime.h"

TEST(TestSimdStore, CombinedMethodTest) {
  Config cfg = Config("../data/bs-ul-sim.json");
  for (size_t base_sc_id = 0; base_sc_id < 512; base_sc_id++) {
  for (size_t ant_num = 1; ant_num <= 48; ant_num++) {
    static const size_t memory_size_bytes =
        kSCsPerCacheline * kMaxAntennas * sizeof(complex_float);
    auto* src = static_cast<complex_float*>(Agora_memory::PaddedAlignedAlloc(
        Agora_memory::Alignment_t::kAlign64, memory_size_bytes));
    auto* dst_avx = static_cast<complex_float*>(Agora_memory::PaddedAlignedAlloc(
        Agora_memory::Alignment_t::kAlign64, memory_size_bytes));
    auto* dst_loop = static_cast<complex_float*>(Agora_memory::PaddedAlignedAlloc(
        Agora_memory::Alignment_t::kAlign64, memory_size_bytes));
    auto* dst_org = static_cast<complex_float*>(Agora_memory::PaddedAlignedAlloc(
        Agora_memory::Alignment_t::kAlign64, memory_size_bytes));

    std::memset(dst_avx, 0u, memory_size_bytes);
    std::memset(dst_loop, 0u, memory_size_bytes);
    std::memset(dst_org, 0u, memory_size_bytes);

    std::default_random_engine generator;
    std::uniform_real_distribution<float> distribution(-1.0, 1.0);

    for (size_t i = 0; i < (memory_size_bytes / sizeof(complex_float)); i++) {
      //Fill Randomly
      src[i].re = distribution(generator);
      src[i].im = distribution(generator);
    }

    for (size_t i = 0; i < cfg.DemulBlockSize(); i += kSCsPerCacheline) {
      StoreData::StoreRxDataAVX(dst_avx, src, base_sc_id + i, ant_num);
    }
    for (size_t i = 0; i < cfg.DemulBlockSize(); i += kSCsPerCacheline) {
      StoreData::StoreRxData(dst_loop, src, base_sc_id + i, ant_num);
    }
    for (size_t i = 0; i < cfg.DemulBlockSize(); i += kSCsPerCacheline) {
      StoreData::StoreRxDataOrg(dst_org, src, base_sc_id + i, ant_num, cfg.OfdmDataNum());
    }

    ASSERT_TRUE(std::memcmp(dst_org, dst_loop, memory_size_bytes) == 0)
        << "Org and Loop did not output the same result when ant_num = " << ant_num;
    ASSERT_TRUE(std::memcmp(dst_avx, dst_loop, memory_size_bytes) == 0)
        << "AVX and Loop did not output the same result when ant_num = " << ant_num;

    std::free(src);
    std::free(dst_avx);
    std::free(dst_loop);
    std::free(dst_org);
  }
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  std::cout << "---- Simd Storage ----\n";
#ifdef __AVX512F__
  std::cout << "AVX512 is supported\n";
#else
  std::cout << "AVX512 is NOT supported\n";
#endif
  printf("kTransposeBlockSize = %lu\n", kTransposeBlockSize);
  printf("kSCsPerCacheline = %lu\n", kSCsPerCacheline);
  printf("kUsePartialTrans = %u\n", kUsePartialTrans);
  return RUN_ALL_TESTS();
}
