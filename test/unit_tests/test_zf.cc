#include <gtest/gtest.h>
// For some reason, gtest include order matters
#include "concurrentqueue.h"
#include "config.h"
#include "dobeamweights.h"
#include "gettime.h"
#include "utils.h"

/// Measure performance of zeroforcing
TEST(TestZF, Perf) {
  static constexpr size_t kNumIters = 10000;
  auto cfg = std::make_unique<Config>("files/config/ci/tddconfig-sim-ul.json");
  cfg->GenData();

  int tid = 0;

  PtrGrid<kFrameWnd, kMaxUEs, complex_float> csi_buffers;
  csi_buffers.RandAllocCxFloat(cfg->BsAntNum() * cfg->OfdmDataNum());

  PtrGrid<kFrameWnd, kMaxDataSCs, complex_float> ul_zf_matrices(
      cfg->BsAntNum() * cfg->UeAntNum());
  PtrGrid<kFrameWnd, kMaxDataSCs, complex_float> dl_zf_matrices(
      cfg->UeAntNum() * cfg->BsAntNum());

  Table<complex_float> calib_dl_msum_buffer;
  calib_dl_msum_buffer.RandAllocCxFloat(kFrameWnd,
                                        cfg->OfdmDataNum() * cfg->BsAntNum(),
                                        Agora_memory::Alignment_t::kAlign64);

  Table<complex_float> calib_ul_msum_buffer;
  calib_ul_msum_buffer.RandAllocCxFloat(kFrameWnd,
                                        cfg->OfdmDataNum() * cfg->BsAntNum(),
                                        Agora_memory::Alignment_t::kAlign64);

  Table<complex_float> calib_dl_buffer;
  calib_dl_buffer.RandAllocCxFloat(kFrameWnd,
                                   cfg->OfdmDataNum() * cfg->BsAntNum(),
                                   Agora_memory::Alignment_t::kAlign64);

  Table<complex_float> calib_ul_buffer;
  calib_ul_buffer.RandAllocCxFloat(kFrameWnd,
                                   cfg->OfdmDataNum() * cfg->BsAntNum(),
                                   Agora_memory::Alignment_t::kAlign64);

  auto phy_stats = std::make_unique<PhyStats>(cfg.get(), Direction::kUplink);
  auto stats = std::make_unique<Stats>(cfg.get());

  auto compute_zf = std::make_unique<DoBeamWeights>(
      cfg.get(), tid, csi_buffers, calib_dl_msum_buffer, calib_ul_msum_buffer,
      calib_dl_buffer, calib_ul_buffer, ul_zf_matrices, dl_zf_matrices,
      phy_stats.get(), stats.get());

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

  calib_dl_msum_buffer.Free();
  calib_ul_msum_buffer.Free();
  calib_dl_buffer.Free();
  calib_ul_buffer.Free();
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
