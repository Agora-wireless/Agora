#ifndef STATS
#define STATS

#include <iostream>

#include "config.h"
#include "gettime.h"
#include "memory_manage.h"
#include "symbols.h"

static constexpr size_t kMaxStatBreakdown = 4;

// Accumulated task duration for all tracked frames in each worker thread
struct DurationStat {
  size_t task_duration_[kMaxStatBreakdown];  // Unit = TSC cycles
  size_t task_count_;
  DurationStat() { Reset(); }
  void Reset() { std::memset(this, 0, sizeof(DurationStat)); }
};

// Temporary summary statistics assembled from per-thread runtime stats
struct FrameSummary {
  double us_this_thread_[kMaxStatBreakdown];
  size_t count_this_thread_ = 0;
  double us_avg_threads_[kMaxStatBreakdown];
  size_t count_all_threads_ = 0;
  FrameSummary() { std::memset(this, 0, sizeof(FrameSummary)); }
};

// Type of timestamps recorded at the master for a frame
// TODO: Add definitions of what each event means
enum class TsType : size_t {
  kPilotRX,            // First pilot packet received
  kProcessingStarted,  // Signal processing started on a pilot symbol
  kPilotAllRX,         // All pilot packets received
  kRCAllRX,            // All Reciprocity Calibration Symbols received
  kFFTPilotsDone,      // Completed FFT for all pilots in this frame
  kZFDone,             // Completed zeroforcing for this frame
  kDemulDone,          // Completed demodulation for this frame
  kRXDone,             // All packets of a frame received
  kRCDone,             // Recirocity Calibration Computation done
  kEncodeDone,
  kDecodeDone,  // Completed all LDPC decoding for this frame
  kPrecodeDone,
  kIFFTDone,
  kTXProcessedFirst,
  kTXDone,
};
static constexpr size_t kNumTimestampTypes =
    static_cast<size_t>(TsType::kTXDone) + 1;

class Stats {
 public:
  Stats(Config* cfg);
  ~Stats();

  /// If worker stats collection is enabled, combine and update per-worker
  /// stats for all uplink Doer types. Else return immediately.
  void UpdateStatsInFunctionsUplink(size_t frame_id);

  /// If worker stats collection is enabled, combine and update per-worker
  /// stats for all downlink Doer types. Else return immediately.
  void UpdateStatsInFunctionsDownlink(size_t frame_id);

  /// Save master timestamps to a file. If worker stats collection is enabled,
  /// also save detailed worker timing info to a file.
  void SaveToFile();

  /// If worker stats collection is enabled, prsize_t a summary of stats
  void PrintSummary();

  size_t last_frame_id_;

  /// From the master, set the RDTSC timestamp for a frame ID and timestamp
  /// type
  void MasterSetTsc(TsType timestamp_type, size_t frame_id) {
    master_timestamps_[static_cast<size_t>(timestamp_type)]
                      [frame_id % kNumStatsFrames] = Rdtsc();
  }

  /// From the master, get the RDTSC timestamp for a frame ID and timestamp
  /// type
  size_t MasterGetTsc(TsType timestamp_type, size_t frame_id) {
    return master_timestamps_[static_cast<size_t>(timestamp_type)]
                             [frame_id % kNumStatsFrames];
  }

  /// From the master, get the millisecond elapsed since the timestamp of
  /// timestamp_type was taken for frame_id
  double MasterGetMsSince(TsType timestamp_type, size_t frame_id) {
    return CyclesToMs(Rdtsc() - MasterGetTsc(timestamp_type, frame_id),
                      freq_ghz_);
  }

  /// From the master, get the microseconds elapsed since the timestamp of
  /// timestamp_type was taken for frame_id
  double MasterGetUsSince(TsType timestamp_type, size_t frame_id) {
    return CyclesToUs(Rdtsc() - MasterGetTsc(timestamp_type, frame_id),
                      freq_ghz_);
  }

  /// From the master, get the microseconds between when the timestamp of
  /// timestamp_type was taken for frame_id, and reference_tsc
  double MasterGetUsFromRef(TsType timestamp_type, size_t frame_id,
                            size_t reference_tsc) {
    return CyclesToUs(MasterGetTsc(timestamp_type, frame_id) - reference_tsc,
                      freq_ghz_);
  }

  /// From the master, for a frame ID, get the millisecond difference
  /// between two timestamp types
  double MasterGetDeltaMs(TsType timestamp_type_1, TsType timestamp_type_2,
                          size_t frame_id) {
    return CyclesToMs(MasterGetTsc(timestamp_type_1, frame_id) -
                          MasterGetTsc(timestamp_type_2, frame_id),
                      freq_ghz_);
  }

  /// From the master, for a frame ID, get the microsecond difference
  /// between two timestamp types
  double MasterGetDeltaUs(TsType timestamp_type_1, TsType timestamp_type_2,
                          size_t frame_id) {
    return CyclesToUs(MasterGetTsc(timestamp_type_1, frame_id) -
                          MasterGetTsc(timestamp_type_2, frame_id),
                      freq_ghz_);
  }

  /// From the master, get the microsecond difference between the times that
  /// a timestamp type was taken for two frames
  double MasterGetDeltaMs(TsType timestamp_type, size_t frame_id_1,
                          size_t frame_id_2) {
    return CyclesToMs(MasterGetTsc(timestamp_type, frame_id_1) -
                          MasterGetTsc(timestamp_type, frame_id_2),
                      freq_ghz_);
  }

  /// From the master, get the microsecond difference between the times that
  /// a timestamp type was taken for two frames
  double MasterGetDeltaUs(TsType timestamp_type, size_t frame_id_1,
                          size_t frame_id_2) {
    return CyclesToUs(MasterGetTsc(timestamp_type, frame_id_1) -
                          MasterGetTsc(timestamp_type, frame_id_2),
                      freq_ghz_);
  }

  /// Get the DurationStat object used by thread thread_id for DoerType
  /// doer_type
  DurationStat* GetDurationStat(DoerType doer_type, size_t thread_id) {
    return &worker_durations_[thread_id]
                .duration_stat_[static_cast<size_t>(doer_type)];
  }

  /// The master thread uses a stale copy of DurationStats to compute
  /// differences. This gets the DurationStat object for thread thread_id
  /// for DoerType doer_type.
  DurationStat* GetDurationStatOld(DoerType doer_type, size_t thread_id) {
    return &worker_durations_old_[thread_id]
                .duration_stat_[static_cast<size_t>(doer_type)];
  }

  /// Dimensions = number of packet RX threads x kNumStatsFrames.
  /// frame_start[i][j] is the RDTSC timestamp taken by thread i when it
  /// starts receiving frame j.
  Table<size_t> frame_start_;

 private:
  // Fill in running time summary stats for the current frame for this
  // thread and Doer type
  void PopulateSummary(FrameSummary* frame_summary, size_t thread_id,
                       DoerType doer_type);

  static void ComputeAvgOverThreads(FrameSummary* frame_summary,
                                    size_t thread_num, size_t break_down_num);
  static void PrintPerThreadPerTask(FrameSummary s);
  static void PrintPerFrame(const char* doer_string,
                            FrameSummary frame_summary);

  size_t GetTotalTaskCount(DoerType doer_type, size_t thread_num);

  /* stats for the worker threads */
  void UpdateStatsInDofftBigstation(size_t frame_id, size_t thread_num,
                                    size_t thread_num_offset,
                                    FrameSummary* fft_frame_summary,
                                    FrameSummary* csi_frame_summary);
  void UpdateStatsInDozfBigstation(size_t frame_id, size_t thread_num,
                                   size_t thread_num_offset,
                                   FrameSummary* zf_frame_summary);
  void UpdateStatsInDodemulBigstation(size_t frame_id, size_t thread_num,
                                      size_t thread_num_offset,
                                      FrameSummary* demul_frame_summary);
  void UpdateStatsInDodecodeBigstation(size_t frame_id, size_t thread_num,
                                       size_t thread_num_offset,
                                       FrameSummary* decode_frame_summary);
  void UpdateStatsInDoifftBigstation(size_t frame_id, size_t thread_num,
                                     size_t thread_num_offset,
                                     FrameSummary* ifft_frame_summary,
                                     FrameSummary* csi_frame_summary);
  void UpdateStatsInDoprecodeBigstation(size_t frame_id, size_t thread_num,
                                        size_t thread_num_offset,
                                        FrameSummary* precode_frame_summary);
  void UpdateStatsInDoencodeBigstation(size_t frame_id, size_t thread_num,
                                       size_t thread_num_offset,
                                       FrameSummary* encode_frame_summary);

  void UpdateStatsInFunctionsUplinkBigstation(
      size_t frame_id, FrameSummary* fft_frame_summary,
      FrameSummary* csi_frame_summary, FrameSummary* zf_frame_summary,
      FrameSummary* demul_frame_summary, FrameSummary* decode_frame_summary);

  void UpdateStatsInFunctionsUplinkAgora(size_t frame_id,
                                         FrameSummary* fft_frame_summary,
                                         FrameSummary* csi_frame_summary,
                                         FrameSummary* zf_frame_summary,
                                         FrameSummary* demul_frame_summary,
                                         FrameSummary* decode_frame_summary);

  void UpdateStatsInFunctionsDownlinkBigstation(
      size_t frame_id, FrameSummary* ifft_frame_summary,
      FrameSummary* csi_frame_summary, FrameSummary* zf_frame_summary,
      FrameSummary* precode_frame_summary, FrameSummary* encode_frame_summary);

  void UpdateStatsInFunctionsDownlinkAgora(size_t frame_id,
                                           FrameSummary* ifft_frame_summary,
                                           FrameSummary* csi_frame_summary,
                                           FrameSummary* zf_frame_summary,
                                           FrameSummary* precode_frame_summary,
                                           FrameSummary* encode_frame_summary);

  Config* config_;
  const size_t task_thread_num_;
  const size_t fft_thread_num_;
  const size_t zf_thread_num_;
  const size_t demul_thread_num_;
  const size_t decode_thread_num_;
  const size_t break_down_num_ = kMaxStatBreakdown;
  const double freq_ghz_;
  const size_t creation_tsc_;  // TSC at which this object was created

  /// Timestamps taken by the master thread at different points in a frame's
  /// processing
  double master_timestamps_[kNumTimestampTypes][kNumStatsFrames];

  /// Running time duration statistics. Each worker thread has one
  /// DurationStat object for every Doer type. The master thread keeps stale
  /// ("old") copies of all DurationStat objects.
  struct {
    DurationStat duration_stat_[kNumDoerTypes];
    uint8_t false_sharing_padding_[64];
  } worker_durations_[kMaxThreads], worker_durations_old_[kMaxThreads];

  double fft_us_[kNumStatsFrames];
  double csi_us_[kNumStatsFrames];
  double zf_us_[kNumStatsFrames];
  double demul_us_[kNumStatsFrames];
  double ifft_us_[kNumStatsFrames];
  double precode_us_[kNumStatsFrames];
  double decode_us_[kNumStatsFrames];
  double encode_us_[kNumStatsFrames];
  double rc_us_[kNumStatsFrames];

  double fft_breakdown_us_[kMaxStatBreakdown][kNumStatsFrames];
  double csi_breakdown_us_[kMaxStatBreakdown][kNumStatsFrames];
  double zf_breakdown_us_[kMaxStatBreakdown][kNumStatsFrames];
  double demul_breakdown_us_[kMaxStatBreakdown][kNumStatsFrames];
  double decode_breakdown_us_[kMaxStatBreakdown][kNumStatsFrames];
};

#endif
