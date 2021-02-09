#include <gtest/gtest.h>
// For some reason, gtest include order matters
#include "concurrentqueue.h"
#include "config.h"
#include "dozf.h"
#include "gettime.h"
#include "utils.h"

/// Measure performance of zeroforcing
TEST(TestZF, Perf) {
  static constexpr size_t kNumIters = 10000;
  auto* cfg = new Config("data/tddconfig-sim-ul.json");
  cfg->GenData();

  int tid = 0;

  PtrGrid<kFrameWnd, kMaxUEs, complex_float> csi_buffers;
  csi_buffers.RandAllocCxFloat(cfg->bs_ant_num_ * cfg->ofdm_data_num_);

  PtrGrid<kFrameWnd, kMaxDataSCs, complex_float> ul_zf_matrices(
      cfg->bs_ant_num_ * cfg->ue_num_);
  PtrGrid<kFrameWnd, kMaxDataSCs, complex_float> dl_zf_matrices(
      cfg->ue_num_ * cfg->bs_ant_num_);

  Table<complex_float> calib_dl_buffer;
  calib_dl_buffer.RandAllocCxFloat(kFrameWnd,
                                   cfg->ofdm_data_num_ * cfg->bs_ant_num_,
                                   Agora_memory::Alignment_t::kK64Align);

  Table<complex_float> calib_ul_buffer;
  calib_ul_buffer.RandAllocCxFloat(kFrameWnd,
                                   cfg->ofdm_data_num_ * cfg->bs_ant_num_,
                                   Agora_memory::Alignment_t::kK64Align);

  auto* stats = new Stats(cfg);

  auto* compute_zf =
      new DoZF(cfg, tid, csi_buffers, calib_dl_buffer, calib_ul_buffer,
               ul_zf_matrices, dl_zf_matrices, stats);

  FastRand fast_rand;
  size_t start_tsc = Rdtsc();
  for (size_t i = 0; i < kNumIters; i++) {
    uint32_t frame_id = fast_rand.NextU32();
    size_t base_sc_id =
        (fast_rand.NextU32() % (cfg->ofdm_data_num_ / cfg->zf_block_size_)) *
        cfg->zf_block_size_;
    compute_zf->Launch(gen_tag_t::FrmSc(frame_id, base_sc_id).tag_);
  }
  double ms = CyclesToMs(Rdtsc() - start_tsc, cfg->freq_ghz_);

  std::printf("Time per zeroforcing iteration = %.4f ms\n", ms / kNumIters);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
