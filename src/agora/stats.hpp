#ifndef STATS
#define STATS

#include "Symbols.hpp"
#include "config.hpp"
#include "gettime.h"
#include "memory_manage.h"
#include <iostream>

static constexpr size_t kMaxStatBreakdown = 4;

// Accumulated task duration for all tracked frames in each worker thread
struct DurationStat {
    size_t task_duration[kMaxStatBreakdown]; // Unit = TSC cycles
    size_t task_count;
    DurationStat() { reset(); }
    void reset() { std::memset(this, 0, sizeof(DurationStat)); }
};

// Temporary summary statistics assembled from per-thread runtime stats
struct FrameSummary {
    double us_this_thread[kMaxStatBreakdown];
    size_t count_this_thread = 0;
    double us_avg_threads[kMaxStatBreakdown];
    size_t count_all_threads = 0;
    FrameSummary() { std::memset(this, 0, sizeof(FrameSummary)); }
};

// Type of timestamps recorded at the master for a frame
// TODO: Add definitions of what each event means
enum class TsType : size_t {
    kPilotRX, // First pilot packet received
    kProcessingStarted, // Signal processing started on a pilot symbol
    kPilotAllRX, // All pilot packets received
    kRCAllRX, // All Reciprocity Calibration Symbols received
    kFFTPilotsDone, // Completed FFT for all pilots in this frame
    kZFDone, // Completed zeroforcing for this frame
    kDemulDone, // Completed demodulation for this frame
    kRXDone, // All packets of a frame received
    kRCDone, // Recirocity Calibration Computation done
    kEncodeDone,
    kDecodeDone, // Completed all LDPC decoding for this frame
    kPrecodeDone,
    kIFFTDone,
    kTXProcessedFirst,
    kTXDone,
};
static constexpr size_t kNumTimestampTypes
    = static_cast<size_t>(TsType::kTXDone) + 1;

class Stats {
public:
    Stats(Config* cfg);
    ~Stats();

    /// If worker stats collection is enabled, combine and update per-worker
    /// stats for all uplink Doer types. Else return immediately.
    void update_stats_in_functions_uplink(size_t frame_id);

    /// If worker stats collection is enabled, combine and update per-worker
    /// stats for all downlink Doer types. Else return immediately.
    void update_stats_in_functions_downlink(size_t frame_id);

    /// Save master timestamps to a file. If worker stats collection is enabled,
    /// also save detailed worker timing info to a file.
    void save_to_file();

    /// If worker stats collection is enabled, prsize_t a summary of stats
    void print_summary();

    size_t last_frame_id( void ) const { return this->last_frame_id_; }

    /// From the master, set the RDTSC timestamp for a frame ID and timestamp
    /// type
    void master_set_tsc(TsType timestamp_type, size_t frame_id)
    {
        master_timestamps[static_cast<size_t>(timestamp_type)]
                         [frame_id % kNumStatsFrames]
            = rdtsc();
    }

    /// From the master, get the RDTSC timestamp for a frame ID and timestamp
    /// type
    size_t master_get_tsc(TsType timestamp_type, size_t frame_id)
    {
        return master_timestamps[static_cast<size_t>(timestamp_type)]
                                [frame_id % kNumStatsFrames];
    }

    /// From the master, get the millisecond elapsed since the timestamp of
    /// timestamp_type was taken for frame_id
    double master_get_ms_since(TsType timestamp_type, size_t frame_id)
    {
        return cycles_to_ms(
            rdtsc() - master_get_tsc(timestamp_type, frame_id), freq_ghz);
    }

    /// From the master, get the microseconds elapsed since the timestamp of
    /// timestamp_type was taken for frame_id
    double master_get_us_since(TsType timestamp_type, size_t frame_id)
    {
        return cycles_to_us(
            rdtsc() - master_get_tsc(timestamp_type, frame_id), freq_ghz);
    }

    /// From the master, get the microseconds between when the timestamp of
    /// timestamp_type was taken for frame_id, and reference_tsc
    double master_get_us_from_ref(
        TsType timestamp_type, size_t frame_id, size_t reference_tsc)
    {
        return cycles_to_us(
            master_get_tsc(timestamp_type, frame_id) - reference_tsc, freq_ghz);
    }

    /// From the master, for a frame ID, get the millisecond difference
    /// between two timestamp types
    double master_get_delta_ms(
        TsType timestamp_type_1, TsType timestamp_type_2, size_t frame_id)
    {
        return cycles_to_ms(master_get_tsc(timestamp_type_1, frame_id)
                - master_get_tsc(timestamp_type_2, frame_id),
            freq_ghz);
    }

    /// From the master, for a frame ID, get the microsecond difference
    /// between two timestamp types
    double master_get_delta_us(
        TsType timestamp_type_1, TsType timestamp_type_2, size_t frame_id)
    {
        return cycles_to_us(master_get_tsc(timestamp_type_1, frame_id)
                - master_get_tsc(timestamp_type_2, frame_id),
            freq_ghz);
    }

    /// From the master, get the microsecond difference between the times that
    /// a timestamp type was taken for two frames
    double master_get_delta_ms(
        TsType timestamp_type, size_t frame_id_1, size_t frame_id_2)
    {
        return cycles_to_ms(master_get_tsc(timestamp_type, frame_id_1)
                - master_get_tsc(timestamp_type, frame_id_2),
            freq_ghz);
    }

    /// From the master, get the microsecond difference between the times that
    /// a timestamp type was taken for two frames
    double master_get_delta_us(
        TsType timestamp_type, size_t frame_id_1, size_t frame_id_2)
    {
        return cycles_to_us(master_get_tsc(timestamp_type, frame_id_1)
                - master_get_tsc(timestamp_type, frame_id_2),
            freq_ghz);
    }

    /// Get the DurationStat object used by thread thread_id for DoerType
    /// doer_type
    DurationStat* get_duration_stat(DoerType doer_type, size_t thread_id)
    {
        return &worker_durations[thread_id]
                    .duration_stat[static_cast<size_t>(doer_type)];
    }

    /// The master thread uses a stale copy of DurationStats to compute
    /// differences. This gets the DurationStat object for thread thread_id
    /// for DoerType doer_type.
    DurationStat* get_duration_stat_old(DoerType doer_type, size_t thread_id)
    {
        return &worker_durations_old[thread_id]
                    .duration_stat[static_cast<size_t>(doer_type)];
    }

    /// Dimensions = number of packet RX threads x kNumStatsFrames.
    /// frame_start[i][j] is the RDTSC timestamp taken by thread i when it
    /// starts receiving frame j.
    Table<size_t> frame_start;

private:
    // Fill in running time summary stats for the current frame for this
    // thread and Doer type
    void populate_summary(
        FrameSummary* frame_summary, size_t thread_id, DoerType doer_type);

    static void compute_avg_over_threads(
        FrameSummary* frame_summary, size_t thread_num, size_t break_down_num);
    static void print_per_thread_per_task(FrameSummary frame_summary);
    static void print_per_frame(
        const char* doer_string, FrameSummary frame_summary);

    size_t get_total_task_count(DoerType doer_type, size_t thread_num);

    /* stats for the worker threads */
    void update_stats_in_dofft_bigstation(size_t frame_id, size_t thread_num,
        size_t thread_num_offset, FrameSummary* frame_summary_fft,
        FrameSummary* frame_summary_csi);
    void update_stats_in_dozf_bigstation(size_t frame_id, size_t thread_num,
        size_t thread_num_offset, FrameSummary* frame_summary_zf);
    void update_stats_in_dodemul_bigstation(size_t frame_id, size_t thread_num,
        size_t thread_num_offset, FrameSummary* frame_summary_demul);
    void update_stats_in_dodecode_bigstation(size_t frame_id, size_t thread_num,
        size_t thread_num_offset, FrameSummary* frame_summary_demul);
    void update_stats_in_doifft_bigstation(size_t frame_id, size_t thread_num,
        size_t thread_num_offset, FrameSummary* frame_summary_ifft,
        FrameSummary* frame_summary_csi);
    void update_stats_in_doprecode_bigstation(size_t frame_id,
        size_t thread_num, size_t thread_num_offset,
        FrameSummary* frame_summary_precode);
    void update_stats_in_doencode_bigstation(size_t frame_id, size_t thread_num,
        size_t thread_num_offset, FrameSummary* frame_summary_precode);

    void update_stats_in_functions_uplink_bigstation(size_t frame_id,
        FrameSummary* frame_summary_fft, FrameSummary* frame_summary_csi,
        FrameSummary* frame_summary_zf, FrameSummary* frame_summary_demul,
        FrameSummary* frame_summary_decode);

    void update_stats_in_functions_uplink_agora(size_t frame_id,
        FrameSummary* frame_summary_fft, FrameSummary* frame_summary_csi,
        FrameSummary* frame_summary_zf, FrameSummary* frame_summary_demul,
        FrameSummary* frame_summary_decode);

    void update_stats_in_functions_downlink_bigstation(size_t frame_id,
        FrameSummary* frame_summary_ifft, FrameSummary* frame_summary_csi,
        FrameSummary* frame_summary_zf, FrameSummary* frame_summary_precode,
        FrameSummary* frame_summary_encode);

    void update_stats_in_functions_downlink_agora(size_t frame_id,
        FrameSummary* frame_summary_ifft, FrameSummary* frame_summary_csi,
        FrameSummary* frame_summary_zf, FrameSummary* frame_summary_precode,
        FrameSummary* frame_summary_encode);

    Config* config_;
    const size_t task_thread_num;
    const size_t fft_thread_num;
    const size_t zf_thread_num;
    const size_t demul_thread_num;
    const size_t decode_thread_num;
    const size_t break_down_num = kMaxStatBreakdown;
    const double freq_ghz;
    const size_t creation_tsc; // TSC at which this object was created

    /// Timestamps taken by the master thread at different points in a frame's
    /// processing
    double master_timestamps[kNumTimestampTypes][kNumStatsFrames];

    /// Running time duration statistics. Each worker thread has one
    /// DurationStat object for every Doer type. The master thread keeps stale
    /// ("old") copies of all DurationStat objects.
    struct {
        DurationStat duration_stat[kNumDoerTypes];
        uint8_t false_sharing_padding[64];
    } worker_durations[kMaxThreads], worker_durations_old[kMaxThreads];

    double fft_us[kNumStatsFrames];
    double csi_us[kNumStatsFrames];
    double zf_us[kNumStatsFrames];
    double demul_us[kNumStatsFrames];
    double ifft_us[kNumStatsFrames];
    double precode_us[kNumStatsFrames];
    double decode_us[kNumStatsFrames];
    double encode_us[kNumStatsFrames];
    double rc_us[kNumStatsFrames];

    double fft_breakdown_us[kMaxStatBreakdown][kNumStatsFrames];
    double csi_breakdown_us[kMaxStatBreakdown][kNumStatsFrames];
    double zf_breakdown_us[kMaxStatBreakdown][kNumStatsFrames];
    double demul_breakdown_us[kMaxStatBreakdown][kNumStatsFrames];
    double decode_breakdown_us[kMaxStatBreakdown][kNumStatsFrames];

    size_t last_frame_id_;
};

#endif
