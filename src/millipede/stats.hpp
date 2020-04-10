/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */

#ifndef STATS
#define STATS

#include "Symbols.hpp"
#include "config.hpp"
#include "gettime.h"
#include "memory_manage.h"
#include <iostream>
#include <stdio.h>
#include <string.h>

static constexpr size_t kMaxStatBreakdown = 4;

/* Accumulated task duration for all frames in each worker thread */
struct DurationStat {
    double task_duration[kMaxStatBreakdown];
    size_t task_count;
    DurationStat() { memset(this, 0, sizeof(DurationStat)); }
};

struct Stats_worker_per_frame {
    double duration_this_thread[kMaxStatBreakdown];
    double duration_this_thread_per_task[kMaxStatBreakdown];
    int count_this_thread = 0;
    double duration_avg_threads[kMaxStatBreakdown];
    int count_all_threads = 0;
    Stats_worker_per_frame()
    {
        memset(this, 0, sizeof(Stats_worker_per_frame));
    }
};

// Type of timestamps recorded
// TODO: Add definitions of what each event means
enum class TsType : size_t {
    kPilotRX,
    kPilotAllRX,
    kProcessingStarted,
    kRXDone,
    kFFTDone,
    kDemulDone,
    kZFDone,
    kRCDone,
    kEncodeDone,
    kDecodeDone,
    kPrecodeDone,
    kIFFTDone,
    kTXProcessedFirst,
    kTXDone,
};
static constexpr size_t kNumTimestampTypes
    = static_cast<size_t>(TsType::kTXDone) + 1;

// Types of Millipede Doers
enum class DoerType : size_t {
    kFFT,
    kCSI,
    kZF,
    kDemul,
    kDecode,
    kEncode,
    kIFFT,
    kPrecode,
    kRC
};
static constexpr size_t kNumDoerTypes = static_cast<size_t>(DoerType::kRC) + 1;

class Stats {
public:
    Stats(Config* cfg, int break_down_num, int task_thread_num,
        int fft_thread_num, int zf_thread_num, int demul_thread_num);
    ~Stats();

    void update_stats_for_breakdowns(Stats_worker_per_frame* stats_per_frame,
        const DurationStat* duration_stat, DurationStat* stats_in_worker_old,
        int break_down_num);

    void compute_avg_over_threads(Stats_worker_per_frame* stats_per_frame,
        int thread_num, int break_down_num);
    void print_per_thread_per_task(Stats_worker_per_frame stats_per_frame);
    void print_per_frame(Stats_worker_per_frame stats_per_frame);

    void update_stats_in_functions_uplink(int frame_id);
    void update_stats_in_functions_downlink(int frame_id);
    void save_to_file();

    int get_total_task_count(DoerType doer_type, int thread_num);
    void print_summary();

    size_t last_frame_id;

    void master_set_timestamp(TsType timestamp_type, int frame_id)
    {
        master_timestamps[static_cast<size_t>(timestamp_type)]
                         [frame_id % kNumStatsFrames]
            = get_time();
    }

    double master_get_timestamp(TsType timestamp_type, int frame_id)
    {
        return master_timestamps[static_cast<size_t>(timestamp_type)]
                                [frame_id % kNumStatsFrames];
    }

    double master_get_timestamp_delta(
        TsType timestamp_type_1, TsType timestamp_type_2, int frame_id)
    {
        return master_timestamps[static_cast<size_t>(timestamp_type_1)]
                                [frame_id % kNumStatsFrames]
            - master_timestamps[static_cast<size_t>(timestamp_type_2)]
                               [frame_id % kNumStatsFrames];
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

    Table<double> frame_start;

private:
    /* stats for the worker threads */
    void update_stats_in_dofft_bigstation(int frame_id, int thread_num,
        int thread_num_offset, Stats_worker_per_frame* fft_stats_per_frame,
        Stats_worker_per_frame* csi_stats_per_frame);
    void update_stats_in_dozf_bigstation(int frame_id, int thread_num,
        int thread_num_offset, Stats_worker_per_frame* zf_stats_per_frame);
    void update_stats_in_dodemul_bigstation(int frame_id, int thread_num,
        int thread_num_offset, Stats_worker_per_frame* demul_stats_per_frame);
    void update_stats_in_doifft_bigstation(int frame_id, int thread_num,
        int thread_num_offset, Stats_worker_per_frame* ifft_stats_per_frame,
        Stats_worker_per_frame* csi_stats_per_frame);
    void update_stats_in_doprecode_bigstation(int frame_id, int thread_num,
        int thread_num_offset, Stats_worker_per_frame* precode_stats_per_frame);

    void update_stats_in_functions_uplink_bigstation(int frame_id,
        Stats_worker_per_frame* fft_stats_per_frame,
        Stats_worker_per_frame* csi_stats_per_frame,
        Stats_worker_per_frame* zf_stats_per_frame,
        Stats_worker_per_frame* demul_stats_per_frame,
        Stats_worker_per_frame* decode_stats_per_frame);

    void update_stats_in_functions_uplink_millipede(int frame_id,
        Stats_worker_per_frame* fft_stats_per_frame,
        Stats_worker_per_frame* csi_stats_per_frame,
        Stats_worker_per_frame* zf_stats_per_frame,
        Stats_worker_per_frame* demul_stats_per_frame,
        Stats_worker_per_frame* decode_stats_per_frame);

    void update_stats_in_functions_downlink_bigstation(int frame_id,
        Stats_worker_per_frame* ifft_stats_per_frame,
        Stats_worker_per_frame* csi_stats_per_frame,
        Stats_worker_per_frame* zf_stats_per_frame,
        Stats_worker_per_frame* precode_stats_per_frame,
        Stats_worker_per_frame* encode_stats_per_frame);

    void update_stats_in_functions_downlink_millipede(int frame_id,
        Stats_worker_per_frame* ifft_stats_per_frame,
        Stats_worker_per_frame* csi_stats_per_frame,
        Stats_worker_per_frame* zf_stats_per_frame,
        Stats_worker_per_frame* precode_stats_per_frame,
        Stats_worker_per_frame* encode_stats_per_frame);

    Config* config_;

    int task_thread_num;
    int fft_thread_num;
    int zf_thread_num;
    int demul_thread_num;
    int break_down_num;

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

    double csi_time_in_function[kNumStatsFrames];
    double fft_time_in_function[kNumStatsFrames];
    double zf_time_in_function[kNumStatsFrames];
    double demul_time_in_function[kNumStatsFrames];
    double ifft_time_in_function[kNumStatsFrames];
    double precode_time_in_function[kNumStatsFrames];
    double decode_time_in_function[kNumStatsFrames];
    double encode_time_in_function[kNumStatsFrames];
    double rc_time_in_function[kNumStatsFrames];

#if DEBUG_UPDATE_STATS_DETAILED
    Table<double> csi_time_in_function_details;
    Table<double> fft_time_in_function_details;
    Table<double> zf_time_in_function_details;
    Table<double> demul_time_in_function_details;
#endif
};

#endif
