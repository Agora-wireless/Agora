#include <gtest/gtest.h>
// For some reason, gtest include order matters
#include "concurrentqueue.h"
#include "config.hpp"
#include "dozf.hpp"
#include "gettime.h"
#include "utils.h"

/// Measure performance of zeroforcing
TEST(TestZF, Perf)
{
    static constexpr size_t kNumIters = 10000;
    auto* cfg = new Config("data/tddconfig-sim-ul.json");
    cfg->GenData();

    int tid = 0;

    PtrGrid<kFrameWnd, kMaxUEs, complex_float> csi_buffers;
    csi_buffers.rand_alloc_cx_float(cfg->bs_ant_num() * cfg->ofdm_data_num());

    PtrGrid<kFrameWnd, kMaxDataSCs, complex_float> ul_zf_matrices(
        cfg->bs_ant_num() * cfg->ue_num());
    PtrGrid<kFrameWnd, kMaxDataSCs, complex_float> dl_zf_matrices(
        cfg->ue_num() * cfg->bs_ant_num());

    Table<complex_float> calib_dl_buffer;
    calib_dl_buffer.rand_alloc_cx_float(
        kFrameWnd, cfg->ofdm_data_num() * cfg->bs_ant_num(), Agora_memory::Alignment_t::k64Align);

    Table<complex_float> calib_ul_buffer;
    calib_ul_buffer.rand_alloc_cx_float(
        kFrameWnd, cfg->ofdm_data_num() * cfg->bs_ant_num(), Agora_memory::Alignment_t::k64Align);

    auto stats = new Stats(cfg);

    auto computeZF = new DoZF(cfg, tid, csi_buffers, calib_dl_buffer,
        calib_ul_buffer, ul_zf_matrices, dl_zf_matrices, stats);

    FastRand fast_rand;
    size_t start_tsc = rdtsc();
    for (size_t i = 0; i < kNumIters; i++) {
        uint32_t frame_id = fast_rand.next_u32();
        size_t base_sc_id
            = (fast_rand.next_u32() % (cfg->ofdm_data_num() / cfg->zf_block_size()))
            * cfg->zf_block_size();
        computeZF->launch(gen_tag_t::frm_sc(frame_id, base_sc_id)._tag);
    }
    double ms = cycles_to_ms(rdtsc() - start_tsc, cfg->freq_ghz());

    std::printf("Time per zeroforcing iteration = %.4f ms\n", ms / kNumIters);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
