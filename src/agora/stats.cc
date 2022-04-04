/**
 * @file stats.cc
 * @brief Implmentation file for the Stats class.
 */
#include "stats.h"

#include <typeinfo>

#include "logger.h"

Stats::Stats(const Config* const cfg)
    : config_(cfg),
      task_thread_num_(cfg->WorkerThreadNum()),
      fft_thread_num_(cfg->FftThreadNum()),
      zf_thread_num_(cfg->ZfThreadNum()),
      demul_thread_num_(cfg->DemulThreadNum()),
      decode_thread_num_(cfg->DecodeThreadNum()),
      freq_ghz_(cfg->FreqGhz()),
      creation_tsc_(GetTime::Rdtsc()) {
  frame_start_.Calloc(config_->SocketThreadNum(), kNumStatsFrames,
                      Agora_memory::Alignment_t::kAlign64);
}

Stats::~Stats() { frame_start_.Free(); }

void Stats::PopulateSummary(FrameSummary* frame_summary, size_t thread_id,
                            DoerType doer_type) {
  DurationStat* ds = GetDurationStat(doer_type, thread_id);
  DurationStat* ds_old = GetDurationStatOld(doer_type, thread_id);

  frame_summary->count_this_thread_ = ds->task_count_ - ds_old->task_count_;
  frame_summary->count_all_threads_ += frame_summary->count_this_thread_;

  for (size_t j = 0; j < break_down_num_; j++) {
    frame_summary->us_this_thread_.at(j) = GetTime::CyclesToUs(
        ds->task_duration_.at(j) - ds_old->task_duration_.at(j), freq_ghz_);
    frame_summary->us_avg_threads_.at(j) +=
        frame_summary->us_this_thread_.at(j);
  }
  *ds_old = *ds;
}

void Stats::ComputeAvgOverThreads(FrameSummary* frame_summary,
                                  size_t thread_num, size_t break_down_num_) {
  for (size_t j = 0; j < break_down_num_; j++) {
    frame_summary->us_avg_threads_.at(j) =
        frame_summary->us_avg_threads_.at(j) / thread_num;
  }
}

void Stats::PrintPerThreadPerTask(std::string const& doer_string,
                                  FrameSummary const& s) {
  if (s.count_this_thread_ > 0) {
    std::printf("%s: %zu tasks %.1f us (~", doer_string.c_str(),
                s.count_this_thread_,
                s.us_this_thread_.at(0u) / s.count_this_thread_);

    for (size_t i = 1u; i < s.us_this_thread_.size(); i++) {
      if (i != 1) {
        std::printf("+ ");
      }
      std::printf(" %.1f ", s.us_this_thread_.at(i) / s.count_this_thread_);
    }
    std::printf("us), ");
  }
}

std::string Stats::PrintPerFrame(std::string const& doer_string,
                                 FrameSummary const& frame_summary) {
  std::stringstream output;
  if (frame_summary.count_all_threads_ > 0) {
    output << doer_string.c_str() << " (" << frame_summary.count_all_threads_
           << " tasks): " << (frame_summary.us_avg_threads_.at(0u) / 1000.0f)
           << " ms (~";

    for (size_t i = 1u; i < frame_summary.us_avg_threads_.size(); i++) {
      if (i != 1) {
        output << "+ ";
      }
      output << frame_summary.us_avg_threads_.at(i) / 1000.0f << " ";
    }
    output << "ms), ";
  }
  return output.str();
}

void Stats::UpdateStats(size_t frame_id) {
  this->last_frame_id_ = frame_id;
  size_t frame_slot = (frame_id % kNumStatsFrames);

  if (kIsWorkerTimingEnabled) {
    std::vector<FrameSummary> work_summary(kAllDoerTypes.size());
    for (size_t i = 0u; i < task_thread_num_; i++) {
      for (size_t j = 0u; j < kAllDoerTypes.size(); j++) {
        PopulateSummary(&work_summary.at(j), i, kAllDoerTypes.at(j));
      }

      if (kDebugPrintStatsPerThread) {
        std::printf("In frame %zu, thread %zu, \t", frame_id, i);
        double sum_us_this_frame_this_thread = 0;
        for (size_t j = 0u; j < kAllDoerTypes.size(); j++) {
          PrintPerThreadPerTask(kDoerNames.at(kAllDoerTypes.at(j)),
                                work_summary.at(j));
          sum_us_this_frame_this_thread +=
              work_summary.at(j).us_this_thread_.at(0);
        }
        std::printf("sum: %.3f\n", sum_us_this_frame_this_thread);
      }
    }
    for (auto& summary : work_summary) {
      ComputeAvgOverThreads(&summary, task_thread_num_, break_down_num_);
    }

    double sum_us = 0.0f;
    for (size_t i = 0u; i < this->doer_us_.size(); i++) {
      double us_avg = work_summary.at(i).us_avg_threads_.at(0u);
      this->doer_us_.at(i).at(frame_slot) = us_avg;
      sum_us += us_avg;
    }

    for (size_t i = 1; i < this->break_down_num_; i++) {
      for (size_t doer = 0; doer < work_summary.size(); doer++) {
        this->doer_breakdown_us_.at(doer).at(i - 1).at(frame_slot) =
            work_summary.at(doer).us_avg_threads_.at(i);
      }
    }

    if (kStatsPrintFrameSummary) {
      std::string print_summary =
          "Frame " + std::to_string(frame_id) + " Summary: ";

      for (size_t i = 0u; i < kAllDoerTypes.size(); i++) {
        print_summary += PrintPerFrame(kDoerNames.at(kAllDoerTypes.at(i)),
                                       work_summary.at(i));
      }
      print_summary += "Total: " + std::to_string(sum_us / 1000.0f) + " ms\n";
      AGORA_LOG_INFO("%s", print_summary.c_str());
    }
  }
}

void Stats::SaveToFile() {
  const std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
  const std::string filename = cur_directory + "/data/timeresult.txt";
  AGORA_LOG_INFO("Stats: Saving master timestamps to %s\n", filename.c_str());
  FILE* fp_debug = std::fopen(filename.c_str(), "w");
  RtAssert(fp_debug != nullptr,
           std::string("Open file failed ") + std::to_string(errno));

  size_t first_frame_idx = 0;
  size_t last_frame_idx = this->last_frame_id_;
  size_t total_stat_frames = this->last_frame_id_;
  if (total_stat_frames > kNumStatsFrames) {
    last_frame_idx = last_frame_idx % kNumStatsFrames;
    first_frame_idx = (last_frame_idx + 1) % kNumStatsFrames;
    total_stat_frames = kNumStatsFrames;
  }

  // For backwards compatibility, it is easier to make a new file format for
  // the combined case
  if ((config_->Frame().NumDLSyms() > 0) &&
      (config_->Frame().NumULSyms() > 0)) {
    std::fprintf(fp_debug,
                 "Pilot RX by socket threads (= reference time), "
                 "kPilotRX, kProcessingStarted, kPilotAllRX, kFFTPilotsDone, "
                 "kZFDone, kPrecodeDone, kIFFTDone, kEncodeDone, kDemulDone, "
                 "kDecodeDone, kRXDone, time in CSI, time in "
                 "FFT, time in ZF, time in Demul, time in Decode\n");

    for (size_t frame = 0; frame < total_stat_frames; frame++) {
      const size_t i = (first_frame_idx + frame) % kNumStatsFrames;
      size_t ref_tsc = SIZE_MAX;
      for (size_t j = 0; j < config_->SocketThreadNum(); j++) {
        ref_tsc = std::min(ref_tsc, this->frame_start_[j][i]);
      }
      std::fprintf(
          fp_debug,
          "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f "
          "%.3f %.3f %.3f %.3f %.3f\n",
          GetTime::CyclesToUs(ref_tsc - this->creation_tsc_, this->freq_ghz_),
          MasterGetUsFromRef(TsType::kFirstSymbolRX, i, ref_tsc),
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
  } else if (config_->Frame().NumDLSyms() > 0) {
    std::fprintf(fp_debug,
                 "Pilot RX by socket threads (= reference time), "
                 "kPilotRX, kProcessingStarted, kPilotAllRX, kFFTPilotsDone, "
                 "kZFDone, kPrecodeDone, kIFFTDone, kEncodeDone, kRXDone\n");
    for (size_t frame = 0; frame < total_stat_frames; frame++) {
      const size_t i = (first_frame_idx + frame) % kNumStatsFrames;
      size_t ref_tsc = SIZE_MAX;
      for (size_t j = 0; j < config_->SocketThreadNum(); j++) {
        ref_tsc = std::min(ref_tsc, this->frame_start_[j][i]);
      }
      std::fprintf(
          fp_debug, "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f \n",
          GetTime::CyclesToUs(ref_tsc - this->creation_tsc_, this->freq_ghz_),
          MasterGetUsFromRef(TsType::kFirstSymbolRX, i, ref_tsc),
          MasterGetUsFromRef(TsType::kProcessingStarted, i, ref_tsc),
          MasterGetUsFromRef(TsType::kPilotAllRX, i, ref_tsc),
          MasterGetUsFromRef(TsType::kFFTPilotsDone, i, ref_tsc),
          MasterGetUsFromRef(TsType::kZFDone, i, ref_tsc),
          MasterGetUsFromRef(TsType::kPrecodeDone, i, ref_tsc),
          MasterGetUsFromRef(TsType::kIFFTDone, i, ref_tsc),
          MasterGetUsFromRef(TsType::kEncodeDone, i, ref_tsc),
          MasterGetUsFromRef(TsType::kRXDone, i, ref_tsc));
    }
  } else if (config_->Frame().NumULSyms() > 0) {
    // Print the header
    std::fprintf(
        fp_debug,
        "Pilot RX by socket threads (= reference time), "
        "kPilotRX, kProcessingStarted, kPilotAllRX, kFFTPilotsDone, "
        "kZFDone, kDemulDone, kDecodeDone, kRXDone, time in CSI, time in "
        "FFT, time in ZF, time in Demul, time in Decode\n");
    for (size_t frame = 0; frame < total_stat_frames; frame++) {
      const size_t i = (first_frame_idx + frame) % kNumStatsFrames;
      size_t ref_tsc = SIZE_MAX;
      for (size_t j = 0; j < config_->SocketThreadNum(); j++) {
        ref_tsc = std::min(ref_tsc, this->frame_start_[j][i]);
      }
      std::fprintf(
          fp_debug,
          "%.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.3f %.3f %.3f "
          "%.3f %.3f\n",
          GetTime::CyclesToUs(ref_tsc - this->creation_tsc_, this->freq_ghz_),
          MasterGetUsFromRef(TsType::kFirstSymbolRX, i, ref_tsc),
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
    RtAssert(false,
             std::string("No uplink or downlink symbols in the frame\n"));
  }

  std::fclose(fp_debug);

  if (kIsWorkerTimingEnabled == true) {
    std::string filename_detailed =
        cur_directory + "/data/timeresult_detail.txt";
    AGORA_LOG_INFO("Stats: Printing detailed results to %s\n",
                   filename_detailed.c_str());

    FILE* fp_debug_detailed = std::fopen(filename_detailed.c_str(), "w");
    RtAssert(fp_debug_detailed != nullptr,
             std::string("Open file failed ") + std::to_string(errno));
    // Print the header
    std::fprintf(
        fp_debug_detailed,
        "fft_0, fft_1, fft_2, zf_0, zf_1, zf_2, demul_0, demul_1, demul_2, "
        "decode_0, decode_1, decode_2\n");

    for (size_t frame = 0; frame < total_stat_frames; frame++) {
      const size_t i = (first_frame_idx + frame) % kNumStatsFrames;
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
    total_count = total_count + GetDurationStat(doer_type, i)->task_count_;
  }
  return total_count;
}

void Stats::PrintSummary() {
  AGORA_LOG_INFO("Stats: total processed frames %zu\n",
                 this->last_frame_id_ + 1);
  if (kIsWorkerTimingEnabled == false) {
    AGORA_LOG_INFO("Stats: Worker timing is disabled. Not printing summary\n");
  } else {
    std::vector<size_t> num_tasks;

    num_tasks.reserve(kAllDoerTypes.size());
    for (auto k_all_doer_type : kAllDoerTypes) {
      num_tasks.push_back(GetTotalTaskCount(k_all_doer_type, task_thread_num_));
    }

    double csi_frames =
        (static_cast<double>(
            num_tasks.at(static_cast<size_t>(DoerType::kCSI)))) /
        (this->config_->BsAntNum() * this->config_->Frame().NumPilotSyms());
    double zf_frames = (static_cast<double>(
                           num_tasks.at(static_cast<size_t>(DoerType::kZF)))) /
                       this->config_->ZfEventsPerSymbol();

    if (config_->Frame().NumDLSyms() > 0) {
      double precode_frames =
          (static_cast<double>(
              num_tasks.at(static_cast<size_t>(DoerType::kPrecode)))) /
          (this->config_->OfdmDataNum() * this->config_->Frame().NumDLSyms());
      double ifft_frames =
          (static_cast<double>(
              num_tasks.at(static_cast<size_t>(DoerType::kIFFT)))) /
          (this->config_->BsAntNum() * this->config_->Frame().NumDLSyms());
      double encode_frames =
          (static_cast<double>(
              num_tasks.at(static_cast<size_t>(DoerType::kEncode)))) /
          (this->config_->LdpcConfig(Direction::kDownlink).NumBlocksInSymbol() *
           this->config_->UeAntNum() * this->config_->Frame().NumDLSyms());
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

    if (config_->Frame().NumULSyms() > 0) {
      double fft_frames =
          (static_cast<double>(
              num_tasks.at(static_cast<size_t>(DoerType::kFFT)))) /
          (this->config_->BsAntNum() * this->config_->Frame().NumULSyms());
      double demul_frames =
          (static_cast<double>(
              num_tasks.at(static_cast<size_t>(DoerType::kDemul)))) /
          (this->config_->OfdmDataNum() * this->config_->Frame().NumULSyms());
      double decode_frames =
          (static_cast<double>(
              num_tasks.at(static_cast<size_t>(DoerType::kDecode)))) /
          (this->config_->LdpcConfig(Direction::kUplink).NumBlocksInSymbol() *
           this->config_->UeAntNum() * this->config_->Frame().NumULSyms());
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
            GetDurationStat(kAllDoerTypes.at(j), i)->task_count_;
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
