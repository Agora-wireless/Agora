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

// Number of cellular frames tracked by statistics
static constexpr size_t kNumStatsFrames = 10000;

struct Stats_worker {
    /* accumulated task duration for all frames in each worker thread*/
    Table<double> task_duration;
    /* accumulated task count for all frames in each worker thread*/
    int* task_count;
};

struct Stats_worker_per_frame {
    double* duration_this_thread;
    double* duration_this_thread_per_task;
    int count_this_thread = 0;
    double* duration_avg_threads;
    int count_all_threads = 0;
};

class Stats {
public:
    Stats(Config* cfg, int in_break_down_num, int in_task_thread_num,
        int in_fft_thread_num, int in_zf_thread_num, int in_demul_thread_num);
    ~Stats();

    void init_stats_worker(
        Stats_worker* stats_in_worker, int thread_num, int break_down_num);
    void init_stats_worker_per_frame(
        Stats_worker_per_frame* stats_in_worker, int break_down_num);
    void free_stats_worker(Stats_worker* stats_in_worker, int thread_num);
    void free_stats_worker_per_frame(Stats_worker_per_frame* stats_in_worker);
    void reset_stats_worker_per_frame(
        Stats_worker_per_frame* stats_in_worker, int break_down_num);
    void update_stats_for_breakdowns(Stats_worker_per_frame* stats_per_frame,
        Stats_worker stats_in_worker, Stats_worker* stats_in_worker_old,
        int thread_id, int break_down_num);
    void compute_avg_over_threads(Stats_worker_per_frame* stats_per_frame,
        int thread_num, int break_down_num);
    void print_per_thread_per_task(Stats_worker_per_frame stats_per_frame);
    void print_per_frame(Stats_worker_per_frame stats_per_frame);

    void update_stats_in_functions_uplink(int frame_id);
    void update_stats_in_functions_uplink_bigstation(int frame_id);
    void update_stats_in_functions_uplink_millipede(int frame_id);
    void update_stats_in_functions_downlink(int frame_id);
    void update_stats_in_functions_downlink_bigstation(int frame_id);
    void update_stats_in_functions_downlink_millipede(int frame_id);
    void save_to_file();

    int compute_total_count(Stats_worker stats_in_worker, int thread_num);
    double compute_count_percentage(
        Stats_worker stats_in_worker, int total_count, int thread_id);
    void print_summary();

    size_t last_frame_id;

    /* stats for the master thread */
    void update_pilot_received(int frame_id)
    {
        pilot_received[frame_id % kNumStatsFrames] = get_time();
    };
    double get_pilot_received(int frame_id)
    {
        return pilot_received[frame_id % kNumStatsFrames];
    };

    void update_pilot_all_received(int frame_id)
    {
        pilot_all_received[frame_id % kNumStatsFrames] = get_time();
    };
    double get_pilot_all_received(int frame_id)
    {
        return pilot_all_received[frame_id % kNumStatsFrames];
    };

    void update_processing_started(int frame_id)
    {
        processing_started[frame_id % kNumStatsFrames] = get_time();
    };
    double get_processing_started(int frame_id)
    {
        return processing_started[frame_id % kNumStatsFrames];
    };

    void update_rx_processed(int frame_id)
    {
        rx_processed[frame_id % kNumStatsFrames] = get_time();
    };
    double get_rx_processed(int frame_id)
    {
        return rx_processed[frame_id % kNumStatsFrames];
    };

    void update_fft_processed(int frame_id)
    {
        fft_processed[frame_id % kNumStatsFrames] = get_time();
    };
    double get_fft_processed(int frame_id)
    {
        return fft_processed[frame_id % kNumStatsFrames];
    };

    void update_demul_processed(int frame_id)
    {
        demul_processed[frame_id % kNumStatsFrames] = get_time();
    };
    double get_demul_processed(int frame_id)
    {
        return demul_processed[frame_id % kNumStatsFrames];
    };

    void update_zf_processed(int frame_id)
    {
        zf_processed[frame_id % kNumStatsFrames] = get_time();
    };
    double get_zf_processed(int frame_id)
    {
        return zf_processed[frame_id % kNumStatsFrames];
    };

    void update_rc_processed(int frame_id)
    {
        rc_processed[frame_id % kNumStatsFrames] = get_time();
    };
    double get_rc_processed(int frame_id)
    {
        return rc_processed[frame_id % kNumStatsFrames];
    };

    void update_decode_processed(int frame_id)
    {
        decode_processed[frame_id % kNumStatsFrames] = get_time();
    };
    double get_decode_processed(int frame_id)
    {
        return decode_processed[frame_id % kNumStatsFrames];
    };

    void update_encode_processed(int frame_id)
    {
        encode_processed[frame_id % kNumStatsFrames] = get_time();
    };
    double get_encode_processed(int frame_id)
    {
        return encode_processed[frame_id % kNumStatsFrames];
    };

    void update_precode_processed(int frame_id)
    {
        precode_processed[frame_id % kNumStatsFrames] = get_time();
    };
    double get_precode_processed(int frame_id)
    {
        return precode_processed[frame_id % kNumStatsFrames];
    };

    void update_ifft_processed(int frame_id)
    {
        ifft_processed[frame_id % kNumStatsFrames] = get_time();
    };
    double get_ifft_processed(int frame_id)
    {
        return ifft_processed[frame_id % kNumStatsFrames];
    };

    void update_tx_processed_first(int frame_id)
    {
        tx_processed_first[frame_id % kNumStatsFrames] = get_time();
    };
    double get_tx_processed_first(int frame_id)
    {
        return tx_processed_first[frame_id % kNumStatsFrames];
    };

    void update_tx_processed(int frame_id)
    {
        tx_processed[frame_id % kNumStatsFrames] = get_time();
    };
    double get_tx_processed(int frame_id)
    {
        return tx_processed[frame_id % kNumStatsFrames];
    };

    /* stats for the worker threads */
    void update_stats_in_dofft(
        int frame_id, int thread_num, int thread_num_offset);
    double get_csi_time_in_dofft(int frame_id)
    {
        return csi_time_in_function[frame_id % kNumStatsFrames];
    };

    // void update_fft_in_function(int frame_id);
    double get_fft_time_in_dofft(int frame_id)
    {
        return fft_time_in_function[frame_id % kNumStatsFrames];
    };

    void update_stats_in_dozf(
        int frame_id, int thread_num, int thread_num_offset);
    double get_time_in_dozf(int frame_id)
    {
        return zf_time_in_function[frame_id % kNumStatsFrames];
    };

    void update_stats_in_dodemul(
        int frame_id, int thread_num, int thread_num_offset);
    double get_time_in_dodemul(int frame_id)
    {
        return demul_time_in_function[frame_id % kNumStatsFrames];
    };

    void update_stats_in_doifft(
        int frame_id, int thread_num, int thread_num_offset);
    double get_time_in_doifft(int frame_id)
    {
        return ifft_time_in_function[frame_id % kNumStatsFrames];
    };

    void update_stats_in_doprecode(
        int frame_id, int thread_num, int thread_num_offset);
    double get_time_in_doprecode(int frame_id)
    {
        return precode_time_in_function[frame_id % kNumStatsFrames];
    };

    void update_stats_in_rc(
        int frame_id, int thread_num, int thread_num_offset);
    double get_time_in_rc(int frame_id)
    {
        return zf_time_in_function[frame_id % kNumStatsFrames];
    };

    /* accumulated task duration for all frames in each worker thread*/
    Stats_worker csi_stats_worker;
    Stats_worker fft_stats_worker;
    Stats_worker zf_stats_worker;
    Stats_worker demul_stats_worker;
    Stats_worker decode_stats_worker;
    Stats_worker encode_stats_worker;
    Stats_worker ifft_stats_worker;
    Stats_worker precode_stats_worker;
    Stats_worker rc_stats_worker;
    Table<double> frame_start;

private:
    Config* config_;

    int task_thread_num;
    int fft_thread_num;
    int zf_thread_num;
    int demul_thread_num;
    int break_down_num;

    double pilot_received[kNumStatsFrames] __attribute__((aligned(4096)));
    double pilot_all_received[kNumStatsFrames] __attribute__((aligned(4096)));
    double processing_started[kNumStatsFrames] __attribute__((aligned(4096)));
    double rx_processed[kNumStatsFrames] __attribute__((aligned(4096)));
    double fft_processed[kNumStatsFrames] __attribute__((aligned(4096)));
    double demul_processed[kNumStatsFrames] __attribute__((aligned(4096)));
    double zf_processed[kNumStatsFrames] __attribute__((aligned(4096)));
    double decode_processed[kNumStatsFrames] __attribute__((aligned(4096)));

    double encode_processed[kNumStatsFrames] __attribute__((aligned(4096)));
    double precode_processed[kNumStatsFrames] __attribute__((aligned(4096)));
    double ifft_processed[kNumStatsFrames] __attribute__((aligned(4096)));
    double tx_processed_first[kNumStatsFrames] __attribute__((aligned(4096)));
    double tx_processed[kNumStatsFrames] __attribute__((aligned(4096)));
    double rc_processed[kNumStatsFrames] __attribute__((aligned(4096)));

    double csi_time_in_function[kNumStatsFrames] __attribute__((aligned(4096)));
    double fft_time_in_function[kNumStatsFrames] __attribute__((aligned(4096)));
    double zf_time_in_function[kNumStatsFrames] __attribute__((aligned(4096)));
    double demul_time_in_function[kNumStatsFrames]
        __attribute__((aligned(4096)));
    double ifft_time_in_function[kNumStatsFrames]
        __attribute__((aligned(4096)));
    double precode_time_in_function[kNumStatsFrames]
        __attribute__((aligned(4096)));
    double decode_time_in_function[kNumStatsFrames]
        __attribute__((aligned(4096)));
    double encode_time_in_function[kNumStatsFrames]
        __attribute__((aligned(4096)));
    double rc_time_in_function[kNumStatsFrames] __attribute__((aligned(4096)));

#if DEBUG_UPDATE_STATS_DETAILED
    Table<double> csi_time_in_function_details;
    Table<double> fft_time_in_function_details;
    Table<double> zf_time_in_function_details;
    Table<double> demul_time_in_function_details;
#endif

    Stats_worker csi_stats_worker_old;
    Stats_worker fft_stats_worker_old;
    Stats_worker zf_stats_worker_old;
    Stats_worker demul_stats_worker_old;
    Stats_worker decode_stats_worker_old;
    Stats_worker encode_stats_worker_old;
    Stats_worker ifft_stats_worker_old;
    Stats_worker precode_stats_worker_old;
    Stats_worker rc_stats_worker_old;

    Stats_worker_per_frame csi_stats_per_frame;
    Stats_worker_per_frame fft_stats_per_frame;
    Stats_worker_per_frame zf_stats_per_frame;
    Stats_worker_per_frame demul_stats_per_frame;
    Stats_worker_per_frame decode_stats_per_frame;
    Stats_worker_per_frame encode_stats_per_frame;
    Stats_worker_per_frame ifft_stats_per_frame;
    Stats_worker_per_frame precode_stats_per_frame;
    Stats_worker_per_frame rc_stats_per_frame;
};

#endif
