#include <gtest/gtest.h>
// For some reason, gtest include order matters
#include "concurrentqueue.h"
#include "config.hpp"
#include "dodemul.hpp"
#include "gettime.h"
#include "phy_stats.hpp"
#include "utils.h"
#include <thread>

static constexpr size_t kNumWorkers = 14;
static constexpr size_t kMaxTestNum = 100;
static constexpr size_t kMaxItrNum = (1 << 30);
static constexpr size_t kModTestNum = 3;
static constexpr size_t mod_bits_nums[kModTestNum] = { 4, 6, 4 };
static constexpr size_t frame_offsets[kModTestNum] = { 0, 20, 30 };
// A spinning barrier to synchronize the start of worker threads
std::atomic<size_t> num_workers_ready_atomic;

void MasterToWorkerDynamic_master(Config* cfg,
    moodycamel::ConcurrentQueue<EventData>& event_queue,
    moodycamel::ConcurrentQueue<EventData>& complete_task_queue)
{
    pin_to_core_with_offset(ThreadType::kMaster, cfg->core_offset, 0);
    // Wait for all worker threads to be ready
    while (num_workers_ready_atomic != kNumWorkers) {
        // Wait
    }

    for (size_t bs_ant_idx = 0; bs_ant_idx < kModTestNum; bs_ant_idx++) {
        cfg->update_mod_cfgs(mod_bits_nums[bs_ant_idx]);
        for (size_t i = 0; i < kMaxTestNum; i++) {
            uint32_t frame_id = i
                    / (cfg->demul_events_per_symbol
                          * cfg->ul_data_symbol_num_perframe)
                + frame_offsets[bs_ant_idx];
            uint32_t symbol_id = (i / cfg->demul_events_per_symbol)
                % cfg->ul_data_symbol_num_perframe;
            size_t base_sc_id
                = (i % cfg->demul_events_per_symbol) * cfg->zf_block_size;
            event_queue.enqueue(EventData(EventType::kZF,
                gen_tag_t::frm_sym_sc(frame_id, symbol_id, base_sc_id)._tag));
        }

        // Dequeue all events in queue to avoid overflow
        size_t num_finished_events = 0;
        while (num_finished_events < kMaxTestNum) {
            EventData event;
            int ret = complete_task_queue.try_dequeue(event);
            if (ret)
                num_finished_events++;
        }
    }
}

void MasterToWorkerDynamic_worker(Config* cfg, size_t worker_id,
    double freq_ghz, moodycamel::ConcurrentQueue<EventData>& event_queue,
    moodycamel::ConcurrentQueue<EventData>& complete_task_queue,
    moodycamel::ProducerToken* ptok, Table<complex_float>& data_buffer,
    PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& ul_zf_matrices,
    Table<complex_float>& ue_spec_pilot_buffer,
    Table<complex_float>& equal_buffer,
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& demod_buffers_,
    PhyStats* phy_stats, Stats* stats)
{
    pin_to_core_with_offset(
        ThreadType::kWorker, cfg->core_offset + 1, worker_id);

    // Wait for all threads (including master) to start runnung
    num_workers_ready_atomic++;
    while (num_workers_ready_atomic != kNumWorkers) {
        // Wait
    }

    auto computeDemul = new DoDemul(cfg, worker_id, freq_ghz, event_queue,
        complete_task_queue, ptok, data_buffer, ul_zf_matrices,
        ue_spec_pilot_buffer, equal_buffer, demod_buffers_, phy_stats, stats);

    size_t start_tsc = rdtsc();
    size_t num_tasks = 0;
    EventData req_event;
    size_t max_frame_id_wo_offset
        = (kMaxTestNum - 1) / (cfg->OFDM_DATA_NUM / cfg->zf_block_size);
    for (size_t i = 0; i < kMaxItrNum; i++) {
        if (event_queue.try_dequeue(req_event)) {
            num_tasks++;
            size_t frame_offset_id = 0;
            size_t cur_frame_id = gen_tag_t(req_event.tags_[0]).frame_id;
            if (cur_frame_id >= frame_offsets[1]
                and cur_frame_id - frame_offsets[1] <= max_frame_id_wo_offset) {
                frame_offset_id = 1;
            } else if (cur_frame_id >= frame_offsets[2]
                and cur_frame_id - frame_offsets[2] <= max_frame_id_wo_offset) {
                frame_offset_id = 2;
            }
            ASSERT_EQ(cfg->mod_order_bits, mod_bits_nums[frame_offset_id]);
            EventData resp_event = computeDemul->launch(req_event.tags_[0]);
            TryEnqueueFallback(&complete_task_queue, ptok, resp_event);
        }
    }
    double ms = cycles_to_ms(rdtsc() - start_tsc, freq_ghz);

    printf("Worker %zu: %zu tasks, time per task = %.4f ms\n", worker_id,
        num_tasks, ms / num_tasks);
}

/// Test correctness of BS_ANT_NUM values in multi-threaded DoDemul
/// when BS_ANT_NUM varies in runtime
TEST(TestDemul, VaryingConfig)
{
    static constexpr size_t kNumIters = 10000;
    auto* cfg = new Config("data/tddconfig-sim-ul.json");
    cfg->genData();

    double freq_ghz = measure_rdtsc_freq();

    auto event_queue = moodycamel::ConcurrentQueue<EventData>(2 * kNumIters);
    moodycamel::ProducerToken* ptoks[kNumWorkers];
    auto complete_task_queue
        = moodycamel::ConcurrentQueue<EventData>(2 * kNumIters);
    for (size_t i = 0; i < kNumWorkers; i++) {
        ptoks[i] = new moodycamel::ProducerToken(complete_task_queue);
    }

    Table<complex_float> data_buffer, ue_spec_pilot_buffer, equal_buffer;
    data_buffer.rand_alloc_cx_float(
        cfg->ul_data_symbol_num_perframe * TASK_BUFFER_FRAME_NUM,
        kMaxAntennas * kMaxDataSCs, 64);
    PtrGrid<kFrameWnd, kMaxDataSCs, complex_float> ul_zf_matrices(
        kMaxAntennas * kMaxUEs);
    equal_buffer.calloc(
        cfg->ul_data_symbol_num_perframe * TASK_BUFFER_FRAME_NUM,
        kMaxDataSCs * kMaxUEs, 64);
    ue_spec_pilot_buffer.calloc(
        TASK_BUFFER_FRAME_NUM, cfg->UL_PILOT_SYMS * kMaxUEs, 64);
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t> demod_buffers(kFrameWnd,
        cfg->symbol_num_perframe, cfg->UE_NUM,
        kMaxModType * cfg->OFDM_DATA_NUM);
    printf(
        "Size of [data_buffer, ul_zf_matrices, equal_buffer, "
        "ue_spec_pilot_buffer, demod_soft_buffer]: [%.1f %.1f %.1f %.1f %.1f] "
        "MB\n",
        cfg->ul_data_symbol_num_perframe * TASK_BUFFER_FRAME_NUM * kMaxAntennas
            * kMaxDataSCs * 4 * 1.0f / 1024 / 1024,
        kMaxDataSCs * TASK_BUFFER_FRAME_NUM * kMaxUEs * kMaxAntennas * 4 * 1.0f
            / 1024 / 1024,
        cfg->ul_data_symbol_num_perframe * TASK_BUFFER_FRAME_NUM * kMaxDataSCs
            * kMaxUEs * 4 * 1.0f / 1024 / 1024,
        TASK_BUFFER_FRAME_NUM * cfg->UL_PILOT_SYMS * kMaxUEs * 4 * 1.0f / 1024
            / 1024,
        cfg->ul_data_symbol_num_perframe * TASK_BUFFER_FRAME_NUM * kMaxModType
            * kMaxDataSCs * kMaxUEs * 1.0f / 1024 / 1024);

    auto stats = new Stats(cfg, kMaxStatBreakdown, freq_ghz);
    auto phy_stats = new PhyStats(cfg);

    auto master = std::thread(MasterToWorkerDynamic_master, cfg,
        std::ref(event_queue), std::ref(complete_task_queue));
    std::thread workers[kNumWorkers];
    for (size_t i = 0; i < kNumWorkers; i++) {
        workers[i] = std::thread(MasterToWorkerDynamic_worker, cfg, i, freq_ghz,
            std::ref(event_queue), std::ref(complete_task_queue), ptoks[i],
            std::ref(data_buffer), std::ref(ul_zf_matrices),
            std::ref(equal_buffer), std::ref(ue_spec_pilot_buffer),
            std::ref(demod_buffers), phy_stats, stats);
    }
    master.join();
    for (auto& w : workers)
        w.join();
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
