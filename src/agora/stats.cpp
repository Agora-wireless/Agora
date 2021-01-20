#include "stats.hpp"

#include <typeinfo>

Stats::Stats(Config* cfg)
    : config_(cfg),
      task_thread_num_(cfg->worker_thread_num()),
      fft_thread_num_(cfg->fft_thread_num()),
      zf_thread_num_(cfg->zf_thread_num()),
      demul_thread_num_(cfg->demul_thread_num()),
      decode_thread_num_(cfg->decode_thread_num()),
      freq_ghz_(cfg->freq_ghz()),
      creation_tsc_(rdtsc()),
      doer_us_(),
      doer_breakdown_us_() {
  frame_start_.calloc(config_->socket_thread_num(), kNumStatsFrames,
                      Agora_memory::Alignment_t::k64Align);
}

Stats::~Stats() { frame_start_.free(); }

void Stats::PopulateSummary(FrameSummary* frame_summary, size_t thread_id,
                            DoerType doer_type) {
  DurationStat* ds = GetDurationStat(doer_type, thread_id);
  DurationStat* ds_old = GetDurationStatOld(doer_type, thread_id);

  frame_summary->count_this_thread = ds->task_count - ds_old->task_count;
  frame_summary->count_all_threads += frame_summary->count_this_thread;

  for (size_t j = 0; j < break_down_num_; j++) {
    frame_summary->us_this_thread.at(j) = cycles_to_us(
        ds->task_duration.at(j) - ds_old->task_duration.at(j), freq_ghz_);
    frame_summary->us_avg_threads.at(j) += frame_summary->us_this_thread.at(j);
  }
  *ds_old = *ds;
}

void Stats::ComputeAvgOverThreads(FrameSummary* frame_summary,
                                  size_t thread_num, size_t break_down_num_) {
  for (size_t j = 0; j < break_down_num_; j++) {
    frame_summary->us_avg_threads.at(j) =
        frame_summary->us_avg_threads.at(j) / thread_num;
  }
}

void Stats::PrintPerThreadPerTask(std::string const& doer_string,
                                  FrameSummary const& s) {
  if (s.count_this_thread > 0) {
    std::printf("%s: %zu tasks %.1f us (~", doer_string.c_str(),
                s.count_this_thread,
                s.us_this_thread.at(0u) / s.count_this_thread);

    for (size_t i = 1u; i < s.us_this_thread.size(); i++) {
      if (i != 1) {
        std::printf("+ ");
      }
      std::printf(" %.1f ", s.us_this_thread.at(i) / s.count_this_thread);
    }
    std::printf("us), ");
  }
}

void Stats::PrintPerFrame(std::string const& doer_string,
                          FrameSummary const& frame_summary) {
  if (frame_summary.count_all_threads > 0) {
    std::printf("%s (%zu tasks): %.3f ms (~", doer_string.c_str(),
                frame_summary.count_all_threads,
                (frame_summary.us_avg_threads.at(0u) / 1000.0));

    for (size_t i = 1u; i < frame_summary.us_avg_threads.size(); i++) {
      if (i != 1) {
        std::printf("+ ");
      }
      std::printf("%.4f ", frame_summary.us_avg_threads.at(i) / 1000.0);
    }
    std::printf("ms), ");
  }
}

void Stats::UpdateStats(size_t frame_id) {
  this->last_frame_id_ = frame_id;
  size_t frame_slot = (frame_id % kNumStatsFrames);

  if (kIsWorkerTimingEnabled == true) {
    std::vector<FrameSummary> work_summary(kAllDoerTypes.size());
    for (size_t i = 0u; i < task_thread_num_; i++) {
      for (size_t j = 0u; j < kAllDoerTypes.size(); j++) {
        PopulateSummary(&work_summary.at(j), i, kAllDoerTypes.at(j));
      }

      if (kDebugPrintStatsPerThread == true) {
        std::printf("In frame %zu, thread %zu, \t", frame_id, i);
        double sum_us_this_frame_this_thread = 0;
        for (size_t j = 0u; j < kAllDoerTypes.size(); j++) {
          PrintPerThreadPerTask(kDoerNames.at(kAllDoerTypes.at(j)),
                                work_summary.at(j));
          sum_us_this_frame_this_thread +=
              work_summary.at(j).us_this_thread.at(0);
        }
        std::printf("sum: %.3f\n", sum_us_this_frame_this_thread);
      }
    }
    for (auto& summary : work_summary) {
      ComputeAvgOverThreads(&summary, task_thread_num_, break_down_num_);
    }

    double sum_us = 0.0f;
    for (size_t i = 0u; i < this->doer_us_.size(); i++) {
      double us_avg = work_summary.at(i).us_avg_threads.at(0u);
      this->doer_us_.at(i).at(frame_slot) = us_avg;
      sum_us += us_avg;
    }

    for (size_t i = 1; i < this->break_down_num_; i++) {
      for (size_t doer = 0; doer < work_summary.size(); doer++) {
        this->doer_breakdown_us_.at(doer).at(i - 1).at(frame_slot) =
            work_summary.at(doer).us_avg_threads.at(i);
      }
    }

    if (kStatsPrintFrameSummary == true) {
      std::printf("Frame %zu summary: ", frame_id);
      for (size_t i = 0u; i < kAllDoerTypes.size(); i++) {
        PrintPerFrame(kDoerNames.at(kAllDoerTypes.at(i)), work_summary.at(i));
      }
      std::printf("Total: %.2f ms\n", sum_us / 1000);
    }
  }
}

void Stats::SaveToFile(void) {
  std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
  std::string filename = cur_directory + "/data/timeresult.txt";
  std::printf("Stats: Saving master timestamps to %s\n", filename.c_str());
  FILE* fp_debug = std::fopen(filename.c_str(), "w");
  rt_assert(fp_debug != nullptr,
            std::string("Open file failed ") + std::to_string(errno));

  // For backwards compatibility, it is easier to make a new file format for
  // the combined case
  if ((config_->frame().NumDLSyms() > 0) &&
      (config_->frame().NumULSyms() > 0)) {
    std::fprintf(fp_debug,
                 "Pilot RX by socket threads (= reference time), "
                 "kPilotRX, kProcessingStarted, kPilotAllRX, kFFTPilotsDone, "
                 "kZFDone, kPrecodeDone, kIFFTDone, kEncodeDone, kDemulDone, "
                 "kDecodeDone, kRXDone, time in CSI, time in "
                 "FFT, time in ZF, time in Demul, time in Decode\n");
    for (size_t i = 0; i < this->last_frame_id_; i++) {
      size_t ref_tsc = SIZE_MAX;
      for (size_t j = 0; j < config_->socket_thread_num(); j++) {
        ref_tsc = std::min(ref_tsc, this->frame_start_[j][i]);
      }
      std::fprintf(
          fp_debug,
          "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f "
          "%.3f %.3f %.3f "
          "%.3f %.3f\n",
          cycles_to_us(ref_tsc - this->creation_tsc_, this->freq_ghz_),
          MasterGetUsFromRef(TsType::kPilotRX, i, ref_tsc),
          MasterGetUsFromRef(TsType::kProcessingStarted, i, ref_tsc),
          MasterGetUsFromRef(TsType::kPilotAllRX, i, ref_tsc),
          MasterGetUsFromRef(TsType::kFFTPilotsDone, i, ref_tsc),
          MasterGetUsFromRef(TsType::kZFDone, i, ref_tsc),
          MasterGetUsFromRef(TsType::kPrecodeDone, i, ref_tsc),
          MasterGetUsFromRef(TsType::kIFFTDone, i, ref_tsc),
          MasterGetUsFromRef(TsType::kEncodeDone, i, ref_tsc),
          MasterGetUsFromRef(TsType::kDemulDone, i, ref_tsc),
          MasterGetUsFromRef(TsType::kDecodeDone, i, ref_tsc),
          MasterGetUsFromRef(TsType::kRXDone, i, ref_tsc),
          this->doer_us_.at(static_cast<size_t>(DoerType::kCSI)).at(i),
          this->doer_us_.at(static_cast<size_t>(DoerType::kFFT)).at(i),
          this->doer_us_.at(static_cast<size_t>(DoerType::kZF)).at(i),
          this->doer_us_.at(static_cast<size_t>(DoerType::kDemul)).at(i),
          this->doer_us_.at(static_cast<size_t>(DoerType::kDecode)).at(i));
    }
  } else if (config_->frame().NumDLSyms() > 0) {
    std::fprintf(fp_debug,
                 "Pilot RX by socket threads (= reference time), "
                 "kPilotRX, kProcessingStarted, kPilotAllRX, kFFTPilotsDone, "
                 "kZFDone, kPrecodeDone, kIFFTDone, kEncodeDone, kRXDone\n");
    for (size_t i = 0; i < this->last_frame_id_; i++) {
      size_t ref_tsc = SIZE_MAX;
      for (size_t j = 0; j < config_->socket_thread_num(); j++) {
        ref_tsc = std::min(ref_tsc, this->frame_start_[j][i]);
      }
      std::fprintf(fp_debug,
                   "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f \n",
                   cycles_to_us(ref_tsc - this->creation_tsc_, this->freq_ghz_),
                   MasterGetUsFromRef(TsType::kPilotRX, i, ref_tsc),
                   MasterGetUsFromRef(TsType::kProcessingStarted, i, ref_tsc),
                   MasterGetUsFromRef(TsType::kPilotAllRX, i, ref_tsc),
                   MasterGetUsFromRef(TsType::kFFTPilotsDone, i, ref_tsc),
                   MasterGetUsFromRef(TsType::kZFDone, i, ref_tsc),
                   MasterGetUsFromRef(TsType::kPrecodeDone, i, ref_tsc),
                   MasterGetUsFromRef(TsType::kIFFTDone, i, ref_tsc),
                   MasterGetUsFromRef(TsType::kEncodeDone, i, ref_tsc),
                   MasterGetUsFromRef(TsType::kRXDone, i, ref_tsc));
    }
  } else if (config_->frame().NumULSyms() > 0) {
    // Print the header
    std::fprintf(
        fp_debug,
        "Pilot RX by socket threads (= reference time), "
        "kPilotRX, kProcessingStarted, kPilotAllRX, kFFTPilotsDone, "
        "kZFDone, kDemulDone, kDecodeDone, kRXDone, time in CSI, time in "
        "FFT, time in ZF, time in Demul, time in Decode\n");
    for (size_t i = 0; i < this->last_frame_id_; i++) {
      size_t ref_tsc = SIZE_MAX;
      for (size_t j = 0; j < config_->socket_thread_num(); j++) {
        ref_tsc = std::min(ref_tsc, this->frame_start_[j][i]);
      }
      std::fprintf(
          fp_debug,
          "%.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.3f %.3f %.3f "
          "%.3f %.3f\n",
          cycles_to_us(ref_tsc - this->creation_tsc_, this->freq_ghz_),
          MasterGetUsFromRef(TsType::kPilotRX, i, ref_tsc),
          MasterGetUsFromRef(TsType::kProcessingStarted, i, ref_tsc),
          MasterGetUsFromRef(TsType::kPilotAllRX, i, ref_tsc),
          MasterGetUsFromRef(TsType::kFFTPilotsDone, i, ref_tsc),
          MasterGetUsFromRef(TsType::kZFDone, i, ref_tsc),
          MasterGetUsFromRef(TsType::kDemulDone, i, ref_tsc),
          MasterGetUsFromRef(TsType::kDecodeDone, i, ref_tsc),
          MasterGetUsFromRef(TsType::kRXDone, i, ref_tsc),
          this->doer_us_.at(static_cast<size_t>(DoerType::kCSI)).at(i),
          this->doer_us_.at(static_cast<size_t>(DoerType::kFFT)).at(i),
          this->doer_us_.at(static_cast<size_t>(DoerType::kZF)).at(i),
          this->doer_us_.at(static_cast<size_t>(DoerType::kDemul)).at(i),
          this->doer_us_.at(static_cast<size_t>(DoerType::kDecode)).at(i));
    }
  } else {
    // Shouldn't happen
    rt_assert(false,
              std::string("No uplink or downlink symbols in the frame\n"));
  }

  std::fclose(fp_debug);

  if (kIsWorkerTimingEnabled == true) {
    std::string filename_detailed =
        cur_directory + "/data/timeresult_detail.txt";
    std::printf("Stats: Printing detailed results to %s\n",
                filename_detailed.c_str());

    FILE* fp_debug_detailed = std::fopen(filename_detailed.c_str(), "w");
    rt_assert(fp_debug_detailed != nullptr,
              std::string("Open file failed ") + std::to_string(errno));
    // Print the header
    std::fprintf(
        fp_debug_detailed,
        "fft_0, fft_1, fft_2, zf_0, zf_1, zf_2, demul_0, demul_1, demul_2, "
        "decode_0, decode_1, decode_2\n");

    for (size_t i = 0; i < this->last_frame_id_; i++) {
      std::fprintf(
          fp_debug_detailed,
          "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",
          this->doer_breakdown_us_.at(static_cast<size_t>(DoerType::kFFT))
                  .at(0)
                  .at(i) +
              this->doer_breakdown_us_.at(static_cast<size_t>(DoerType::kCSI))
                  .at(0)
                  .at(i),
          this->doer_breakdown_us_.at(static_cast<size_t>(DoerType::kFFT))
                  .at(1)
                  .at(i) +
              this->doer_breakdown_us_.at(static_cast<size_t>(DoerType::kCSI))
                  .at(1)
                  .at(i),
          this->doer_breakdown_us_.at(static_cast<size_t>(DoerType::kFFT))
                  .at(2)
                  .at(i) +
              this->doer_breakdown_us_.at(static_cast<size_t>(DoerType::kCSI))
                  .at(2)
                  .at(i),
          this->doer_breakdown_us_.at(static_cast<size_t>(DoerType::kZF))
              .at(0)
              .at(i),
          this->doer_breakdown_us_.at(static_cast<size_t>(DoerType::kZF))
              .at(1)
              .at(i),
          this->doer_breakdown_us_.at(static_cast<size_t>(DoerType::kZF))
              .at(2)
              .at(i),
          this->doer_breakdown_us_.at(static_cast<size_t>(DoerType::kDemul))
              .at(0)
              .at(i),
          this->doer_breakdown_us_.at(static_cast<size_t>(DoerType::kDemul))
              .at(1)
              .at(i),
          this->doer_breakdown_us_.at(static_cast<size_t>(DoerType::kDemul))
              .at(2)
              .at(i),
          this->doer_breakdown_us_.at(static_cast<size_t>(DoerType::kDecode))
              .at(0)
              .at(i),
          this->doer_breakdown_us_.at(static_cast<size_t>(DoerType::kDecode))
              .at(1)
              .at(i),
          this->doer_breakdown_us_.at(static_cast<size_t>(DoerType::kDecode))
              .at(2)
              .at(i));
    }
    std::fclose(fp_debug_detailed);
  }
}

size_t Stats::GetTotalTaskCount(DoerType doer_type, size_t thread_num) {
  size_t total_count = 0;
  for (size_t i = 0; i < thread_num; i++) {
    total_count = total_count + GetDurationStat(doer_type, i)->task_count;
  }
  return total_count;
}

void Stats::PrintSummary(void) {
  std::printf("Stats: total processed frames %zu\n", this->last_frame_id_ + 1);
  if (kIsWorkerTimingEnabled == false) {
    std::printf("Stats: Worker timing is disabled. Not printing summary\n");
  } else {
    std::vector<size_t> num_tasks;

    for (size_t j = 0u; j < kAllDoerTypes.size(); j++) {
      num_tasks.push_back(
          GetTotalTaskCount(kAllDoerTypes.at(j), task_thread_num_));
    }

    double csi_frames =
        (static_cast<double>(
            num_tasks.at(static_cast<size_t>(DoerType::kCSI)))) /
        (this->config_->bs_ant_num() * this->config_->frame().NumPilotSyms());
    double zf_frames = (static_cast<double>(
                           num_tasks.at(static_cast<size_t>(DoerType::kZF)))) /
                       this->config_->zf_events_per_symbol();

    if (config_->frame().NumDLSyms() > 0) {
      double precode_frames =
          (static_cast<double>(
              num_tasks.at(static_cast<size_t>(DoerType::kPrecode)))) /
          (this->config_->ofdm_data_num() * this->config_->frame().NumDLSyms());
      double ifft_frames =
          (static_cast<double>(
              num_tasks.at(static_cast<size_t>(DoerType::kIFFT)))) /
          (this->config_->bs_ant_num() * this->config_->frame().NumDLSyms());
      double encode_frames =
          (static_cast<double>(
              num_tasks.at(static_cast<size_t>(DoerType::kEncode)))) /
          (this->config_->ldpc_config().num_blocks_in_symbol() *
           this->config_->ue_num() * this->config_->frame().NumDLSyms());
      std::printf("Downlink totals (tasks, frames): ");
      std::printf("CSI (%zu, %.2f), ",
                  num_tasks.at(static_cast<size_t>(DoerType::kCSI)),
                  csi_frames);
      std::printf("ZF (%zu, %.2f), ",
                  num_tasks.at(static_cast<size_t>(DoerType::kZF)), zf_frames);
      std::printf("Encode (%zu, %.2f), ",
                  num_tasks.at(static_cast<size_t>(DoerType::kEncode)),
                  encode_frames);
      std::printf("Precode (%zu, %.2f), ",
                  num_tasks.at(static_cast<size_t>(DoerType::kPrecode)),
                  precode_frames);
      std::printf("IFFT (%zu, %.2f)",
                  num_tasks.at(static_cast<size_t>(DoerType::kIFFT)),
                  ifft_frames);
      std::printf("\n");
    }  // config_->frame().NumDLSyms() > 0

    if (config_->frame().NumULSyms() > 0) {
      double fft_frames =
          (static_cast<double>(
              num_tasks.at(static_cast<size_t>(DoerType::kFFT)))) /
          (this->config_->bs_ant_num() * this->config_->frame().NumULSyms());
      double demul_frames =
          (static_cast<double>(
              num_tasks.at(static_cast<size_t>(DoerType::kDemul)))) /
          (this->config_->ofdm_data_num() * this->config_->frame().NumULSyms());
      double decode_frames =
          (static_cast<double>(
              num_tasks.at(static_cast<size_t>(DoerType::kDecode)))) /
          (this->config_->ldpc_config().num_blocks_in_symbol() *
           this->config_->ue_num() * this->config_->frame().NumULSyms());
      std::printf("Uplink totals (tasks, frames): ");
      std::printf("CSI (%zu, %.2f), ",
                  num_tasks.at(static_cast<size_t>(DoerType::kCSI)),
                  csi_frames);
      std::printf("ZF (%zu, %.2f), ",
                  num_tasks.at(static_cast<size_t>(DoerType::kZF)), zf_frames);
      std::printf("FFT (%zu, %.2f), ",
                  num_tasks.at(static_cast<size_t>(DoerType::kFFT)),
                  fft_frames);
      std::printf("Demul (%zu, %.2f), ",
                  num_tasks.at(static_cast<size_t>(DoerType::kDemul)),
                  demul_frames);
      std::printf("Decode (%zu, %.2f)",
                  num_tasks.at(static_cast<size_t>(DoerType::kDecode)),
                  decode_frames);
      std::printf("\n");
    }  // config_->frame().NumULSyms() > 0

    for (size_t i = 0; i < task_thread_num_; i++) {
      std::printf("Thread %zu performed (tasks, fraction of tasks): ", i);
      for (size_t j = 0u; j < kAllDoerTypes.size(); j++) {
        size_t duration_stat =
            GetDurationStat(kAllDoerTypes.at(j), i)->task_count;
        if (duration_stat > 0) {
          double percent_stat =
              (static_cast<double>(duration_stat) * 100.0f) / num_tasks.at(j);
          std::printf("%s (%zu, %.2f%%), ",
                      kDoerNames.at(kAllDoerTypes.at(j)).c_str(), duration_stat,
                      percent_stat);
        }
      }
      std::printf("\n");
    }
  }  // kIsWorkerTimingEnabled == true
}
