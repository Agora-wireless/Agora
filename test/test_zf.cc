#include <algorithm>
#include <gtest/gtest.h>
#include <time.h>
#include <vector>

#include "concurrentqueue.h"
#include "config.hpp"
#include "dozf.hpp"
#include "gettime.h"

/// Measure performance of zeroforcing
TEST(TestZF, Perf)
{
    static constexpr size_t kNumIters = 1024;
    size_t x = 0;
    ASSERT_EQ(x, 0);

    auto* cfg = new Config("data/tddconfig-sim-ul.json");
    cfg->genData();

    int tid = 0;
    double freq_ghz = measure_rdtsc_freq();

    auto event_queue = moodycamel::ConcurrentQueue<Event_data>(2 * kNumIters);
    auto comp_queue = moodycamel::ConcurrentQueue<Event_data>(2 * kNumIters);
    auto ptok = new moodycamel::ProducerToken(comp_queue);

    Table<complex_float> csi_buffer_, ul_precoder_buffer_, dl_precoder_buffer_,
        recip_buffer_;
    csi_buffer_.malloc(cfg->pilot_symbol_num_perframe * TASK_BUFFER_FRAME_NUM,
        cfg->BS_ANT_NUM * cfg->OFDM_DATA_NUM, 64);
    ul_precoder_buffer_.malloc(cfg->OFDM_DATA_NUM * TASK_BUFFER_FRAME_NUM,
        cfg->BS_ANT_NUM * cfg->UE_NUM, 64);
    dl_precoder_buffer_.calloc(cfg->OFDM_DATA_NUM * TASK_BUFFER_FRAME_NUM,
        cfg->UE_NUM * cfg->BS_ANT_NUM, 64);
    recip_buffer_.calloc(
        TASK_BUFFER_FRAME_NUM, cfg->OFDM_DATA_NUM * cfg->BS_ANT_NUM, 64);

    auto stats = new Stats(cfg, kMaxStatBreakdown, cfg->worker_thread_num,
        cfg->fft_thread_num, cfg->zf_thread_num, cfg->demul_thread_num,
        freq_ghz);

    auto computeZF = new DoZF(cfg, tid, freq_ghz, event_queue, comp_queue, ptok,
        csi_buffer_, recip_buffer_, ul_precoder_buffer_, dl_precoder_buffer_,
        stats);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
