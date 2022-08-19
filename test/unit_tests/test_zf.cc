#include <gtest/gtest.h>
// For some reason, gtest include order matters
#include "buffer.h"
#include "concurrentqueue.h"
#include "config.h"
#include "dobeamweights.h"
#include "gettime.h"
#include "message.h"
#include "utils.h"

/// Measure performance of zeroforcing
TEST(TestZF, Perf) {
  static constexpr size_t kNumIters = 10000;
  auto cfg = std::make_unique<Config>("files/config/ci/tddconfig-sim-ul.json");
  cfg->GenData();

  int tid = 0;

  auto buffer = std::make_unique<AgoraBuffer>(cfg.get());
  auto phy_stats = std::make_unique<PhyStats>(cfg.get(), Direction::kUplink);
  auto stats = std::make_unique<Stats>(cfg.get());

  auto compute_zf = std::make_unique<DoBeamWeights>(
      cfg.get(), tid, buffer.get(), phy_stats.get(), stats.get());

  FastRand fast_rand;
  size_t start_tsc = GetTime::Rdtsc();
  for (size_t i = 0; i < kNumIters; i++) {
    uint32_t frame_id = fast_rand.NextU32();
    size_t base_sc_id =
        (fast_rand.NextU32() % (cfg->OfdmDataNum() / cfg->BeamBlockSize())) *
        cfg->BeamBlockSize();
    compute_zf->Launch(gen_tag_t::FrmSc(frame_id, base_sc_id).tag_);
  }
  double ms = GetTime::CyclesToMs(GetTime::Rdtsc() - start_tsc, cfg->FreqGhz());

  std::printf("Time per zeroforcing iteration = %.4f ms\n", ms / kNumIters);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
