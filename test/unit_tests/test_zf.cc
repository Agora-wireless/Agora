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
    auto* cfg = new Config("data/tddconfig-sim-ul-distributed.json");
    cfg->genData();

    int tid = 0;
    double freq_ghz = measure_rdtsc_freq();

    auto event_queue = moodycamel::ConcurrentQueue<Event_data>(2 * kNumIters);
    auto comp_queue = moodycamel::ConcurrentQueue<Event_data>(2 * kNumIters);
    auto ptok = new moodycamel::ProducerToken(comp_queue);

    PtrGrid<kFrameWnd, kMaxUEs, complex_float> csi_buffers;
    csi_buffers.rand_alloc_cx_float(cfg->BS_ANT_NUM * cfg->OFDM_DATA_NUM);

    PtrGrid<kFrameWnd, kMaxDataSCs, complex_float> ul_zf_matrices(
        cfg->BS_ANT_NUM * cfg->UE_NUM);
    PtrGrid<kFrameWnd, kMaxDataSCs, complex_float> dl_zf_matrices(
        cfg->UE_NUM * cfg->BS_ANT_NUM);

    Table<complex_float> calib_buffer;
    calib_buffer.rand_alloc_cx_float(
        TASK_BUFFER_FRAME_NUM, cfg->OFDM_DATA_NUM * cfg->BS_ANT_NUM, 64);

    auto stats = new Stats(cfg, kMaxStatBreakdown, freq_ghz);

    auto computeZF = new DoZF(cfg, tid, freq_ghz, event_queue, comp_queue, ptok,
        csi_buffers, calib_buffer, ul_zf_matrices, dl_zf_matrices, stats);

    FastRand fast_rand;
    size_t start_tsc = rdtsc();
    for (size_t i = 0; i < kNumIters; i++) {
        uint32_t frame_id = fast_rand.next_u32();
        size_t base_sc_id
            = (fast_rand.next_u32() % (cfg->OFDM_DATA_NUM / cfg->zf_block_size))
            * cfg->zf_block_size;
        computeZF->launch(gen_tag_t::frm_sc(frame_id, base_sc_id)._tag);
    }
    double ms = cycles_to_ms(rdtsc() - start_tsc, freq_ghz);

    printf("Time per zeroforcing iteration = %.4f ms\n", ms / kNumIters);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
