/**
 * @file stats.h
 * @brief Declaration file for the Stats class. Includes definations for
 * DurationStat and FrameSummery types.
 */
#ifndef STATS_H_
#define STATS_H_

#include <array>
#include <cstddef>
#include <string>

#include "config.h"
#include "gettime.h"
#include "memory_manage.h"
#include "message.h"
#include "symbols.h"

static constexpr size_t kMaxStatBreakdown = 4;

// Accumulated task duration for all tracked frames in each worker thread
struct DurationStat {
  std::array<size_t, kMaxStatBreakdown> task_duration_;  // Unit = TSC cycles
  size_t task_count_;
  DurationStat() { Reset(); }
  void Reset() { std::memset(this, 0, sizeof(DurationStat)); }
};

// Temporary summary statistics assembled from per-thread runtime stats
struct FrameSummary {
  std::array<double, kMaxStatBreakdown> us_this_thread_;
  size_t count_this_thread_ = 0;
  std::array<double, kMaxStatBreakdown> us_avg_threads_;
  size_t count_all_threads_ = 0;
  FrameSummary() { std::memset(this, 0, sizeof(FrameSummary)); }
};

// Type of timestamps recorded at the master for a framePhyStats
// TODO: Add definitions of what each event means
enum class TsType : size_t {
  kFirstSymbolRX,      // First symbol in packet received
  kProcessingStarted,  // Signal processing started on a pilot symbol
  kPilotAllRX,         // All pilot packets received
  kRCAllRX,            // All Reciprocity Calibration Symbols received
  kFFTPilotsDone,      // Completed FFT for all pilots in this frame
  kBeamDone,           // Completed zeroforcing for this frame
  kDemulDone,          // Completed demodulation for this frame
  kRXDone,             // All packets of a frame received
  kRCDone,             // Recirocity Calibration Computation done
  kEncodeDone,
  kDecodeDone,  // Completed all LDPC decoding for this frame
  kPrecodeDone,
  kIFFTDone,
  kTXProcessedFirst,
  kTXDone,
  kModulDone,
  kFFTDone,
  kTsTypeEnd
};
static constexpr size_t kNumTimestampTypes =
    static_cast<size_t>(TsType::kTsTypeEnd);

class Stats {
 public:
  explicit Stats(const Config* const cfg);
  ~Stats();

  /// If worker stats collection is enabled, combine and update per-worker
  /// stats for all uplink and donwlink Doer types. Else return immediately.
  void UpdateStats(size_t frame_id);

  /// Save master timestamps to a file. If worker stats collection is enabled,
  /// also save detailed worker timing info to a file.
  void SaveToFile();

  /// If worker stats collection is enabled, prsize_t a summary of stats
  void PrintSummary();

  /// From the master, set the RDTSC timestamp for a frame ID and timestamp
  /// type
  void MasterSetTsc(TsType timestamp_type, size_t frame_id) {
    this->master_timestamps_.at(static_cast<size_t>(timestamp_type))
        .at(frame_id % kNumStatsFrames) = GetTime::Rdtsc();
  }

  /// From the master, get the RDTSC timestamp for a frame ID and timestamp
  /// type
  size_t MasterGetTsc(TsType timestamp_type, size_t frame_id) const {
    return this->master_timestamps_.at(static_cast<size_t>(timestamp_type))
        .at((frame_id % kNumStatsFrames));
  }

  /// From the master, get the millisecond elapsed since the timestamp of
  /// timestamp_type was taken for frame_id
  double MasterGetMsSince(TsType timestamp_type, size_t frame_id) const {
    return GetTime::CyclesToMs(
        GetTime::Rdtsc() - MasterGetTsc(timestamp_type, frame_id),
        this->freq_ghz_);
  }

  /// From the master, get the microseconds elapsed since the timestamp of
  /// timestamp_type was taken for frame_id
  double MasterGetUsSince(TsType timestamp_type, size_t frame_id) const {
    return GetTime::CyclesToUs(
        GetTime::Rdtsc() - MasterGetTsc(timestamp_type, frame_id),
        this->freq_ghz_);
  }

  /// From the master, get the microseconds between when the timestamp of
  /// timestamp_type was taken for frame_id, and reference_tsc
  double MasterGetUsFromRef(TsType timestamp_type, size_t frame_id,
                            size_t reference_tsc) const {
    return GetTime::CyclesToUs(
        MasterGetTsc(timestamp_type, frame_id) - reference_tsc,
        this->freq_ghz_);
  }

  /// From the master, for a frame ID, get the millisecond difference
  /// between two timestamp types
  double MasterGetDeltaMs(TsType timestamp_type_1, TsType timestamp_type_2,
                          size_t frame_id) const {
    return GetTime::CyclesToMs(MasterGetTsc(timestamp_type_1, frame_id) -
                                   MasterGetTsc(timestamp_type_2, frame_id),
                               this->freq_ghz_);
  }

  /// From the master, for a frame ID, get the microsecond difference
  /// between two timestamp types
  double MasterGetDeltaUs(TsType timestamp_type_1, TsType timestamp_type_2,
                          size_t frame_id) const {
    return GetTime::CyclesToUs(MasterGetTsc(timestamp_type_1, frame_id) -
                                   MasterGetTsc(timestamp_type_2, frame_id),
                               this->freq_ghz_);
  }

  /// From the master, get the microsecond difference between the times that
  /// a timestamp type was taken for two frames
  double MasterGetDeltaMs(TsType timestamp_type, size_t frame_id_1,
                          size_t frame_id_2) const {
    return GetTime::CyclesToMs(MasterGetTsc(timestamp_type, frame_id_1) -
                                   MasterGetTsc(timestamp_type, frame_id_2),
                               this->freq_ghz_);
  }

  /// From the master, get the microsecond difference between the times that
  /// a timestamp type was taken for two frames
  double MasterGetDeltaUs(TsType timestamp_type, size_t frame_id_1,
                          size_t frame_id_2) const {
    return GetTime::CyclesToUs(MasterGetTsc(timestamp_type, frame_id_1) -
                                   MasterGetTsc(timestamp_type, frame_id_2),
                               this->freq_ghz_);
  }

  void PrintPerFrameDone(PrintType print_type, size_t frame_id) const;
  void PrintPerSymbolDone(PrintType print_type, size_t frame_id,
                          size_t symbol_id, size_t sub_count) const;
  void PrintPerTaskDone(PrintType print_type, size_t frame_id, size_t symbol_id,
                        size_t ant_or_sc_id, size_t task_count) const;

  /// Get the DurationStat object used by thread thread_id for DoerType
  /// doer_type
  DurationStat* GetDurationStat(DoerType doer_type, size_t thread_id) {
    return &this->worker_durations_[thread_id]
                .duration_stat_[static_cast<size_t>(doer_type)];
  }

  /// The master thread uses a stale copy of DurationStats to compute
  /// differences. This gets the DurationStat object for thread thread_id
  /// for DoerType doer_type.
  DurationStat* GetDurationStatOld(DoerType doer_type, size_t thread_id) {
    return &this->worker_durations_old_[thread_id]
                .duration_stat_[static_cast<size_t>(doer_type)];
  }

  inline size_t LastFrameId() const { return this->last_frame_id_; }
  /// Dimensions = number of packet RX threads x kNumStatsFrames.
  /// frame_start[i][j] is the RDTSC timestamp taken by thread i when it
  /// starts receiving frame j.
  inline Table<size_t>& FrameStart() { return this->frame_start_; };

 private:
  // Fill in running time summary stats for the current frame for this
  // thread and Doer type
  void PopulateSummary(FrameSummary* frame_summary, size_t thread_id,
                       DoerType doer_type);

  static void ComputeAvgOverThreads(FrameSummary* frame_summary,
                                    size_t thread_num, size_t break_down_num);
  static void PrintPerThreadPerTask(std::string const& doer_string,
                                    FrameSummary const& s);
  static std::string PrintPerFrame(std::string const& doer_string,
                                   FrameSummary const& frame_summary);

  size_t GetTotalTaskCount(DoerType doer_type, size_t thread_num);

  const Config* const config_;

  const size_t task_thread_num_;
  const size_t fft_thread_num_;
  const size_t beam_thread_num_;
  const size_t demul_thread_num_;
  const size_t decode_thread_num_;
  const size_t break_down_num_ = kMaxStatBreakdown;
  const double freq_ghz_;
  const size_t creation_tsc_;  // TSC at which this object was created

  /// Timestamps taken by the master thread at different points in a frame's
  /// processing
  std::array<std::array<double, kNumStatsFrames>, kNumTimestampTypes>
      master_timestamps_;

  /// Running time duration statistics. Each worker thread has one
  /// DurationStat object for every Doer type. The master thread keeps stale
  /// ("old") copies of all DurationStat objects.
  struct TimeDurationsStats {
    std::array<DurationStat, kNumDoerTypes> duration_stat_;
    std::array<uint8_t, 64> false_sharing_padding_;
  };

  std::array<TimeDurationsStats, kMaxThreads> worker_durations_;
  std::array<TimeDurationsStats, kMaxThreads> worker_durations_old_;

  std::array<std::array<double, kNumStatsFrames>, kNumDoerTypes> doer_us_;
  std::array<std::array<std::array<double, kNumStatsFrames>, kMaxStatBreakdown>,
             kNumDoerTypes>
      doer_breakdown_us_;

  size_t last_frame_id_;

  /// Dimensions = number of packet RX threads x kNumStatsFrames.
  /// frame_start[i][j] is the RDTSC timestamp taken by thread i when it
  /// starts receiving frame j.
  Table<size_t> frame_start_;
};

#endif  // STATS_H_
