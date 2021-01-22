#include "stats.hpp"

#include <typeinfo>

Stats::Stats(const Config* const cfg)
    : kConfig(cfg),
      kTaskThreadNum(cfg->WorkerThreadNum()),
      kFftThreadNum(cfg->FftThreadNum()),
      kZfThreadNum(cfg->ZfThreadNum()),
      kDemulThreadNum(cfg->DemulThreadNum()),
      kDecodeThreadNum(cfg->DecodeThreadNum()),
      kFreqGhz(cfg->FreqGhz()),
      kCreationTsc(Rdtsc()),
      doer_us_(),
      doer_breakdown_us_() {
  frame_start_.Calloc(kConfig->SocketThreadNum(), kNumStatsFrames,
                      Agora_memory::Alignment_t::k64Align);
}

Stats::~Stats() { frame_start_.Free(); }

void Stats::PopulateSummary(FrameSummary* frame_summary, size_t thread_id,
                            DoerType doer_type) {
  DurationStat* ds = GetDurationStat(doer_type, thread_id);
  DurationStat* ds_old = GetDurationStatOld(doer_type, thread_id);

  frame_summary->count_this_thread_ = ds->task_count_ - ds_old->task_count_;
  frame_summary->count_all_threads_ += frame_summary->count_this_thread_;

  for (size_t j = 0; j < kBreakDownNum; j++) {
    frame_summary->us_this_thread_.at(j) = CyclesToUs(
        ds->task_duration_.at(j) - ds_old->task_duration_.at(j), kFreqGhz);
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

void Stats::PrintPerFrame(std::string const& doer_string,
                          FrameSummary const& frame_summary) {
  if (frame_summary.count_all_threads_ > 0) {
    std::printf("%s (%zu tasks): %.3f ms (~", doer_string.c_str(),
                frame_summary.count_all_threads_,
                (frame_summary.us_avg_threads_.at(0u) / 1000.0));

    for (size_t i = 1u; i < frame_summary.us_avg_threads_.size(); i++) {
      if (i != 1) {
        std::printf("+ ");
      }
      std::printf("%.4f ", frame_summary.us_avg_threads_.at(i) / 1000.0);
    }
    std::printf("ms), ");
  }
}

void Stats::UpdateStats(size_t frame_id) {
  this->last_frame_id_ = frame_id;
  size_t frame_slot = (frame_id % kNumStatsFrames);

  if (kIsWorkerTimingEnabled == true) {
    std::vector<FrameSummary> work_summary(kAllDoerTypes.size());
    for (size_t i = 0u; i < kTaskThreadNum; i++) {
      for (size_t j = 0u; j < kAllDoerTypes.size(); j++) {
        PopulateSummary(&work_summary.at(j), i, kAllDoerTypes.at(j));
      }

      if (kDebugPrintStatsPerThread == true) {
        std::printf("In frame %zu, thread %zu, \t", frame_id, i);
        double sum_us_this_frame_this_thread = 0;
        for (size_t j = 0u; j < kAllDoerTypes.size(); j++) {
          PrintPerThreadPerTask(k_doer_names.at(kAllDoerTypes.at(j)),
                                work_summary.at(j));
          sum_us_this_frame_this_thread +=
              work_summary.at(j).us_this_thread_.at(0);
        }
        std::printf("sum: %.3f\n", sum_us_this_frame_this_thread);
      }
    }
    for (auto& summary : work_summary) {
      ComputeAvgOverThreads(&summary, kTaskThreadNum, kBreakDownNum);
    }

    double sum_us = 0.0f;
    for (size_t i = 0u; i < this->doer_us_.size(); i++) {
      double us_avg = work_summary.at(i).us_avg_threads_.at(0u);
      this->doer_us_.at(i).at(frame_slot) = us_avg;
      sum_us += us_avg;
    }

    for (size_t i = 1; i < this->kBreakDownNum; i++) {
      for (size_t doer = 0; doer < work_summary.size(); doer++) {
        this->doer_breakdown_us_.at(doer).at(i - 1).at(frame_slot) =
            work_summary.at(doer).us_avg_threads_.at(i);
      }
    }

    if (kStatsPrintFrameSummary == true) {
      std::printf("Frame %zu summary: ", frame_id);
      for (size_t i = 0u; i < kAllDoerTypes.size(); i++) {
        PrintPerFrame(k_doer_names.at(kAllDoerTypes.at(i)), work_summary.at(i));
      }
      std::printf("Total: %.2f ms\n", sum_us / 1000);
    }
  }
}

void Stats::SaveToFile() {
  std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
  std::string filename = cur_directory + "/data/timeresult.txt";
  std::printf("Stats: Saving master timestamps to %s\n", filename.c_str());
  FILE* fp_debug = std::fopen(filename.c_str(), "w");
  RtAssert(fp_debug != nullptr,
           std::string("Open file failed ") + std::to_string(errno));

  // For backwards compatibility, it is easier to make a new file format for
  // the combined case
  if ((kConfig->Frame().NumDLSyms() > 0) &&
      (kConfig->Frame().NumULSyms() > 0)) {
    std::fprintf(fp_debug,
                 "Pilot RX by socket threads (= reference time), "
                 "kPilotRX, kProcessingStarted, kPilotAllRX, kFFTPilotsDone, "
                 "kZFDone, kPrecodeDone, kIFFTDone, kEncodeDone, kDemulDone, "
                 "kDecodeDone, kRXDone, time in CSI, time in "
                 "FFT, time in ZF, time in Demul, time in Decode\n");
    for (size_t i = 0; i < this->last_frame_id_; i++) {
      size_t ref_tsc = SIZE_MAX;
      for (size_t j = 0; j < kConfig->SocketThreadNum(); j++) {
        ref_tsc = std::min(ref_tsc, this->frame_start_[j][i]);
      }
      std::fprintf(
          fp_debug,
          "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f "
          "%.3f %.3f %.3f "
          "%.3f %.3f\n",
          CyclesToUs(ref_tsc - this->kCreationTsc, this->kFreqGhz),
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
  } else if (kConfig->Frame().NumDLSyms() > 0) {
    std::fprintf(fp_debug,
                 "Pilot RX by socket threads (= reference time), "
                 "kPilotRX, kProcessingStarted, kPilotAllRX, kFFTPilotsDone, "
                 "kZFDone, kPrecodeDone, kIFFTDone, kEncodeDone, kRXDone\n");
    for (size_t i = 0; i < this->last_frame_id_; i++) {
      size_t ref_tsc = SIZE_MAX;
      for (size_t j = 0; j < kConfig->SocketThreadNum(); j++) {
        ref_tsc = std::min(ref_tsc, this->frame_start_[j][i]);
      }
      std::fprintf(fp_debug,
                   "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f \n",
                   CyclesToUs(ref_tsc - this->kCreationTsc, this->kFreqGhz),
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
  } else if (kConfig->Frame().NumULSyms() > 0) {
    // Print the header
    std::fprintf(
        fp_debug,
        "Pilot RX by socket threads (= reference time), "
        "kPilotRX, kProcessingStarted, kPilotAllRX, kFFTPilotsDone, "
        "kZFDone, kDemulDone, kDecodeDone, kRXDone, time in CSI, time in "
        "FFT, time in ZF, time in Demul, time in Decode\n");
    for (size_t i = 0; i < this->last_frame_id_; i++) {
      size_t ref_tsc = SIZE_MAX;
      for (size_t j = 0; j < kConfig->SocketThreadNum(); j++) {
        ref_tsc = std::min(ref_tsc, this->frame_start_[j][i]);
      }
      std::fprintf(
          fp_debug,
          "%.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.3f %.3f %.3f "
          "%.3f %.3f\n",
          CyclesToUs(ref_tsc - this->kCreationTsc, this->kFreqGhz),
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
    RtAssert(false,
             std::string("No uplink or downlink symbols in the frame\n"));
  }

  std::fclose(fp_debug);

  if (kIsWorkerTimingEnabled == true) {
    std::string filename_detailed =
        cur_directory + "/data/timeresult_detail.txt";
    std::printf("Stats: Printing detailed results to %s\n",
                filename_detailed.c_str());

    FILE* fp_debug_detailed = std::fopen(filename_detailed.c_str(), "w");
    RtAssert(fp_debug_detailed != nullptr,
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
    total_count = total_count + GetDurationStat(doer_type, i)->task_count_;
  }
  return total_count;
}

void Stats::PrintSummary() {
  std::printf("Stats: total processed frames %zu\n", this->last_frame_id_ + 1);
  if (kIsWorkerTimingEnabled == false) {
    std::printf("Stats: Worker timing is disabled. Not printing summary\n");
  } else {
    std::vector<size_t> num_tasks;

    num_tasks.reserve(kAllDoerTypes.size());
    for (auto kAllDoerType : kAllDoerTypes) {
      num_tasks.push_back(GetTotalTaskCount(kAllDoerType, kTaskThreadNum));
    }

    double csi_frames =
        (static_cast<double>(
            num_tasks.at(static_cast<size_t>(DoerType::kCSI)))) /
        (this->kConfig->BsAntNum() * this->kConfig->Frame().NumPilotSyms());
    double zf_frames = (static_cast<double>(
                           num_tasks.at(static_cast<size_t>(DoerType::kZF)))) /
                       this->kConfig->ZfEventsPerSymbol();

    if (kConfig->Frame().NumDLSyms() > 0) {
      double precode_frames =
          (static_cast<double>(
              num_tasks.at(static_cast<size_t>(DoerType::kPrecode)))) /
          (this->kConfig->OfdmDataNum() * this->kConfig->Frame().NumDLSyms());
      double ifft_frames =
          (static_cast<double>(
              num_tasks.at(static_cast<size_t>(DoerType::kIFFT)))) /
          (this->kConfig->BsAntNum() * this->kConfig->Frame().NumDLSyms());
      double encode_frames =
          (static_cast<double>(
              num_tasks.at(static_cast<size_t>(DoerType::kEncode)))) /
          (this->kConfig->LdpcConfig().NumBlocksInSymbol() *
           this->kConfig->UeNum() * this->kConfig->Frame().NumDLSyms());
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

    if (kConfig->Frame().NumULSyms() > 0) {
      double fft_frames =
          (static_cast<double>(
              num_tasks.at(static_cast<size_t>(DoerType::kFFT)))) /
          (this->kConfig->BsAntNum() * this->kConfig->Frame().NumULSyms());
      double demul_frames =
          (static_cast<double>(
              num_tasks.at(static_cast<size_t>(DoerType::kDemul)))) /
          (this->kConfig->OfdmDataNum() * this->kConfig->Frame().NumULSyms());
      double decode_frames =
          (static_cast<double>(
              num_tasks.at(static_cast<size_t>(DoerType::kDecode)))) /
          (this->kConfig->LdpcConfig().NumBlocksInSymbol() *
           this->kConfig->UeNum() * this->kConfig->Frame().NumULSyms());
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

    for (size_t i = 0; i < kTaskThreadNum; i++) {
      std::printf("Thread %zu performed (tasks, fraction of tasks): ", i);
      for (size_t j = 0u; j < kAllDoerTypes.size(); j++) {
        size_t duration_stat =
            GetDurationStat(kAllDoerTypes.at(j), i)->task_count_;
        if (duration_stat > 0) {
          double percent_stat =
              (static_cast<double>(duration_stat) * 100.0f) / num_tasks.at(j);
          std::printf("%s (%zu, %.2f%%), ",
                      k_doer_names.at(kAllDoerTypes.at(j)).c_str(),
                      duration_stat, percent_stat);
        }
      }
      std::printf("\n");
    }
  }  // kIsWorkerTimingEnabled == true
}
