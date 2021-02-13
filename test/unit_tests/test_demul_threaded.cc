#include <gtest/gtest.h>
// For some reason, gtest include order matters
#include <thread>

#include "concurrentqueue.h"
#include "config.h"
#include "dodemul.h"
#include "gettime.h"
#include "phy_stats.h"
#include "utils.h"

static constexpr size_t kNumWorkers = 14;
static constexpr size_t kMaxTestNum = 100;
static constexpr size_t kMaxItrNum = (1 << 30);
static constexpr size_t kModTestNum = 3;
static constexpr size_t kModBitsNums[kModTestNum] = {4, 6, 4};
static constexpr size_t kFrameOffsets[kModTestNum] = {0, 20, 30};
// A spinning barrier to synchronize the start of worker threads
std::atomic<size_t> num_workers_ready_atomic;

void MasterToWorkerDynamicMaster(
    Config* cfg, moodycamel::ConcurrentQueue<EventData>& event_queue,
    moodycamel::ConcurrentQueue<EventData>& complete_task_queue) {
  PinToCoreWithOffset(ThreadType::kMaster, cfg->CoreOffset(), 0);
  // Wait for all worker threads to be ready
  while (num_workers_ready_atomic != kNumWorkers) {
    // Wait
  }

  for (size_t bs_ant_idx = 0; bs_ant_idx < kModTestNum; bs_ant_idx++) {
    cfg->UpdateModCfgs(kModBitsNums[bs_ant_idx]);
    for (size_t i = 0; i < kMaxTestNum; i++) {
      uint32_t frame_id =
          i / (cfg->DemulEventsPerSymbol() * cfg->Frame().NumULSyms()) +
          kFrameOffsets[bs_ant_idx];
      uint32_t symbol_id =
          (i / cfg->DemulEventsPerSymbol()) % cfg->Frame().NumULSyms();
      size_t base_sc_id =
          (i % cfg->DemulEventsPerSymbol()) * cfg->ZfBlockSize();
      event_queue.enqueue(
          EventData(EventType::kZF,
                    gen_tag_t::FrmSymSc(frame_id, symbol_id, base_sc_id).tag_));
    }

    // Dequeue all events in queue to avoid overflow
    size_t num_finished_events = 0;
    while (num_finished_events < kMaxTestNum) {
      EventData event;
      int ret = static_cast<int>(complete_task_queue.try_dequeue(event));
      if (ret != 0) {
        num_finished_events++;
      }
    }
  }
}

void MasterToWorkerDynamicWorker(
    Config* cfg, size_t worker_id,
    moodycamel::ConcurrentQueue<EventData>& event_queue,
    moodycamel::ConcurrentQueue<EventData>& complete_task_queue,
    moodycamel::ProducerToken* ptok, Table<complex_float>& data_buffer,
    PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& ul_zf_matrices,
    Table<complex_float>& ue_spec_pilot_buffer,
    Table<complex_float>& equal_buffer,
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& demod_buffers_,
    PhyStats* phy_stats, Stats* stats) {
  PinToCoreWithOffset(ThreadType::kWorker, cfg->CoreOffset() + 1, worker_id);

  // Wait for all threads (including master) to start runnung
  num_workers_ready_atomic++;
  while (num_workers_ready_atomic != kNumWorkers) {
    // Wait
  }

  auto* compute_demul = new DoDemul(cfg, worker_id, data_buffer, ul_zf_matrices,
                                    ue_spec_pilot_buffer, equal_buffer,
                                    demod_buffers_, phy_stats, stats);

  size_t start_tsc = Rdtsc();
  size_t num_tasks = 0;
  EventData req_event;
  size_t max_frame_id_wo_offset =
      (kMaxTestNum - 1) / (cfg->OfdmDataNum() / cfg->ZfBlockSize());
  for (size_t i = 0; i < kMaxItrNum; i++) {
    if (event_queue.try_dequeue(req_event)) {
      num_tasks++;
      size_t frame_offset_id = 0;
      size_t cur_frame_id = gen_tag_t(req_event.tags_[0]).frame_id_;
      if (cur_frame_id >= kFrameOffsets[1] and
          cur_frame_id - kFrameOffsets[1] <= max_frame_id_wo_offset) {
        frame_offset_id = 1;
      } else if (cur_frame_id >= kFrameOffsets[2] and
                 cur_frame_id - kFrameOffsets[2] <= max_frame_id_wo_offset) {
        frame_offset_id = 2;
      }
      ASSERT_EQ(cfg->ModOrderBits(), kModBitsNums[frame_offset_id]);
      EventData resp_event = compute_demul->Launch(req_event.tags_[0]);
      TryEnqueueFallback(&complete_task_queue, ptok, resp_event);
    }
  }
  double ms = CyclesToMs(Rdtsc() - start_tsc, cfg->FreqGhz());

  std::printf("Worker %zu: %zu tasks, time per task = %.4f ms\n", worker_id,
              num_tasks, ms / num_tasks);
}

/// Test correctness of BS_ANT_NUM values in multi-threaded DoDemul
/// when BS_ANT_NUM varies in runtime
TEST(TestDemul, VaryingConfig) {
  static constexpr size_t kNumIters = 10000;
  auto* cfg = new Config("data/tddconfig-sim-ul.json");
  cfg->GenData();

  auto event_queue = moodycamel::ConcurrentQueue<EventData>(2 * kNumIters);
  moodycamel::ProducerToken* ptoks[kNumWorkers];
  auto complete_task_queue =
      moodycamel::ConcurrentQueue<EventData>(2 * kNumIters);
  for (auto& ptok : ptoks) {
    ptok = new moodycamel::ProducerToken(complete_task_queue);
  }

  Table<complex_float> data_buffer;
  Table<complex_float> ue_spec_pilot_buffer;
  Table<complex_float> equal_buffer;
  data_buffer.RandAllocCxFloat(cfg->Frame().NumULSyms() * kFrameWnd,
                               kMaxAntennas * kMaxDataSCs,
                               Agora_memory::Alignment_t::kAlign64);
  PtrGrid<kFrameWnd, kMaxDataSCs, complex_float> ul_zf_matrices(kMaxAntennas *
                                                                kMaxUEs);
  equal_buffer.Calloc(cfg->Frame().NumULSyms() * kFrameWnd,
                      kMaxDataSCs * kMaxUEs,
                      Agora_memory::Alignment_t::kAlign64);
  ue_spec_pilot_buffer.Calloc(kFrameWnd,
                              cfg->Frame().ClientUlPilotSymbols() * kMaxUEs,
                              Agora_memory::Alignment_t::kAlign64);
  PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t> demod_buffers(
      kFrameWnd, cfg->Frame().NumTotalSyms(), cfg->UeNum(),
      kMaxModType * cfg->OfdmDataNum());
  std::printf(
      "Size of [data_buffer, ul_zf_matrices, equal_buffer, "
      "ue_spec_pilot_buffer, demod_soft_buffer]: [%.1f %.1f %.1f %.1f %.1f] "
      "MB\n",
      cfg->Frame().NumULSyms() * kFrameWnd * kMaxAntennas * kMaxDataSCs * 4 *
          1.0f / 1024 / 1024,
      kMaxDataSCs * kFrameWnd * kMaxUEs * kMaxAntennas * 4 * 1.0f / 1024 / 1024,
      cfg->Frame().NumULSyms() * kFrameWnd * kMaxDataSCs * kMaxUEs * 4 * 1.0f /
          1024 / 1024,
      kFrameWnd * cfg->Frame().ClientUlPilotSymbols() * kMaxUEs * 4 * 1.0f /
          1024 / 1024,
      cfg->Frame().NumULSyms() * kFrameWnd * kMaxModType * kMaxDataSCs *
          kMaxUEs * 1.0f / 1024 / 1024);

  auto* stats = new Stats(cfg);
  auto* phy_stats = new PhyStats(cfg);

  auto master =
      std::thread(MasterToWorkerDynamicMaster, cfg, std::ref(event_queue),
                  std::ref(complete_task_queue));
  std::thread workers[kNumWorkers];
  for (size_t i = 0; i < kNumWorkers; i++) {
    workers[i] =
        std::thread(MasterToWorkerDynamicWorker, cfg, i, std::ref(event_queue),
                    std::ref(complete_task_queue), ptoks[i],
                    std::ref(data_buffer), std::ref(ul_zf_matrices),
                    std::ref(equal_buffer), std::ref(ue_spec_pilot_buffer),
                    std::ref(demod_buffers), phy_stats, stats);
  }
  master.join();
  for (auto& w : workers) {
    w.join();
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
