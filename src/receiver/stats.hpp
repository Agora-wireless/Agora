#ifndef STATS
#define STATS

#include <iostream>
#include <stdio.h>  /* for fprintf */
#include <string.h> /* for memcpy */
#include "gettime.h"
#include "Symbols.hpp"
#include "memory_manage.h"


class Stats
{
public:
    Stats(double **in_CSI_task_duration, int *in_CSI_task_count, double **in_FFT_task_duration, int *in_FFT_task_count, 
          double **in_ZF_task_duration, int *in_ZF_task_count, double **in_Demul_task_duration, int *in_Demul_task_count,
          double **in_IFFT_task_duration, int *in_IFFT_task_count, double **in_Precode_task_duration, int *in_Precode_task_count,
          double **in_frame_start, 
          int in_task_duration_dim1, int in_task_duration_dim2, int in_task_count_dim,
          int in_task_thread_num, int in_fft_thread_num, int in_zf_thread_num, int in_demul_thread_num);
    ~Stats(){};


    void update_stats_in_functions_uplink(int frame_id);
    void update_stats_in_functions_uplink_bigstation(int frame_id);
    void update_stats_in_functions_uplink_millipede(int frame_id);
    void update_stats_in_functions_downlink(int frame_id);
    void update_stats_in_functions_downlink_bigstation(int frame_id);
    void update_stats_in_functions_downlink_millipede(int frame_id);
    void save_to_file(int last_frame_id, int socket_rx_thread_num);
    void save_to_file_details(int last_frame_id);
    void print_summary(int last_frame_id);

    /* stats for the master thread */
    void update_pilot_received(int frame_id) {pilot_received[frame_id % 10000] = get_time();};
    double get_pilot_received(int frame_id) {return pilot_received[frame_id % 10000];};

    void update_pilot_all_received(int frame_id){pilot_all_received[frame_id % 10000] = get_time();};
    double get_pilot_all_received(int frame_id) {return pilot_all_received[frame_id % 10000];};

    void update_processing_started(int frame_id) {processing_started[frame_id % 10000] = get_time();};
    double get_processing_started(int frame_id) {return processing_started[frame_id % 10000];};

    void update_rx_processed(int frame_id) {rx_processed[frame_id % 10000] = get_time();};
    double get_rx_processed(int frame_id) {return rx_processed[frame_id % 10000];};

    void update_fft_processed(int frame_id) {fft_processed[frame_id % 10000] = get_time();};
    double get_fft_processed(int frame_id) {return fft_processed[frame_id % 10000];};

    void update_demul_processed(int frame_id) {demul_processed[frame_id % 10000] = get_time();};
    double get_demul_processed(int frame_id) {return demul_processed[frame_id % 10000];};

    void update_zf_processed(int frame_id) {zf_processed[frame_id % 10000] = get_time();};
    double get_zf_processed(int frame_id) {return zf_processed[frame_id % 10000];};

    void update_precode_processed(int frame_id) {precode_processed[frame_id % 10000] = get_time();};
    double get_precode_processed(int frame_id) {return precode_processed[frame_id % 10000];};

    void update_ifft_processed(int frame_id) {ifft_processed[frame_id % 10000] = get_time();};
    double get_ifft_processed(int frame_id) {return ifft_processed[frame_id % 10000];};

    void update_tx_processed_first(int frame_id) {tx_processed_first[frame_id % 10000] = get_time();};
    double get_tx_processed_first(int frame_id) {return tx_processed_first[frame_id % 10000];};

    void update_tx_processed(int frame_id) {tx_processed[frame_id % 10000] = get_time();};
    double get_tx_processed(int frame_id) {return tx_processed[frame_id % 10000];};

    /* stats for the worker threads */
    void update_stats_in_dofft(int frame_id, int thread_num, int thread_num_offset);
    double get_csi_time_in_dofft(int frame_id) {return csi_time_in_function[frame_id % 10000];};

    // void update_fft_in_function(int frame_id);
    double get_fft_time_in_dofft(int frame_id) {return fft_time_in_function[frame_id % 10000];};

    void update_stats_in_dozf(int frame_id, int thread_num, int thread_num_offset);
    double get_time_in_dozf(int frame_id) {return zf_time_in_function[frame_id % 10000];};

    void update_stats_in_dodemul(int frame_id, int thread_num, int thread_num_offset);
    double get_time_in_dodemul(int frame_id) {return demul_time_in_function[frame_id % 10000];};

    void update_stats_in_doifft(int frame_id, int thread_num, int thread_num_offset);
    double get_time_in_doifft(int frame_id) {return ifft_time_in_function[frame_id % 10000];};

    void update_stats_in_doprecode(int frame_id, int thread_num, int thread_num_offset);
    double get_time_in_doprecode(int frame_id) {return precode_time_in_function[frame_id % 10000];};
 
private:
    int task_thread_num;
    int fft_thread_num;
    int zf_thread_num;
    int demul_thread_num;
    int task_duration_dim1;
    int task_duration_dim2;
    int task_count_dim;


    double pilot_received[10000] __attribute__( ( aligned (4096) ) ) ;
    double pilot_all_received[10000] __attribute__( ( aligned (4096) ) ) ;
    double processing_started[10000] __attribute__( ( aligned (4096) ) ) ;
    double rx_processed[10000] __attribute__( ( aligned (4096) ) ) ;
    double fft_processed[10000] __attribute__( ( aligned (4096) ) ) ;
    double demul_processed[10000] __attribute__( ( aligned (4096) ) ) ;
    double zf_processed[10000] __attribute__( ( aligned (4096) ) ) ;

    double precode_processed[10000] __attribute__( ( aligned (4096) ) ) ;
    double ifft_processed[10000] __attribute__( ( aligned (4096) ) ) ;
    double tx_processed_first[10000] __attribute__( ( aligned (4096) ) ) ;
    double tx_processed[10000] __attribute__( ( aligned (4096) ) ) ;

    double csi_time_in_function[10000] __attribute__( ( aligned (4096) ) ) ;
    double fft_time_in_function[10000] __attribute__( ( aligned (4096) ) ) ;
    double zf_time_in_function[10000] __attribute__( ( aligned (4096) ) ) ;
    double demul_time_in_function[10000] __attribute__( ( aligned (4096) ) ) ;
    double ifft_time_in_function[10000] __attribute__( ( aligned (4096) ) ) ;
    double precode_time_in_function[10000] __attribute__( ( aligned (4096) ) ) ;

#if DEBUG_UPDATE_STATS_DETAILED
    double **csi_time_in_function_details;
    double **fft_time_in_function_details;
    double **zf_time_in_function_details;
    double **demul_time_in_function_details;
#endif

    /* accumulated task duration for all frames in each worker thread*/
    double **CSI_task_duration;
    double **FFT_task_duration;
    double **ZF_task_duration; 
    double **Demul_task_duration; 
    double **IFFT_task_duration; 
    double **Precode_task_duration; 

    /* accumulated task count for all frames in each worker thread*/
    int *CSI_task_count;
    int *FFT_task_count;
    int *ZF_task_count; 
    int *Demul_task_count;
    int *IFFT_task_count; 
    int *Precode_task_count;

    double **frame_start;


    int csi_count_this_frame_this_thread = 0;
    int fft_count_this_frame_this_thread = 0;
    int zf_count_this_frame_this_thread = 0;
    int demul_count_this_frame_this_thread = 0;
    int ifft_count_this_frame_this_thread = 0;
    int precode_count_this_frame_this_thread = 0;

    int csi_count_this_frame = 0;
    int fft_count_this_frame = 0;
    int zf_count_this_frame = 0;
    int demul_count_this_frame = 0;
    int ifft_count_this_frame = 0;
    int precode_count_this_frame = 0;

    double *csi_time_this_frame_this_thread;
    double *fft_time_this_frame_this_thread;
    double *zf_time_this_frame_this_thread;
    double *demul_time_this_frame_this_thread;
    double *ifft_time_this_frame_this_thread;
    double *precode_time_this_frame_this_thread;

    double *csi_time_this_frame_this_thread_per_task;
    double *fft_time_this_frame_this_thread_per_task;
    double *zf_time_this_frame_this_thread_per_task;
    double *demul_time_this_frame_this_thread_per_task;
    double *ifft_time_this_frame_this_thread_per_task;
    double *precode_time_this_frame_this_thread_per_task;

    double *csi_time_this_frame;
    double *fft_time_this_frame;
    double *zf_time_this_frame;
    double *demul_time_this_frame;
    double *ifft_time_this_frame;
    double *precode_time_this_frame;

    int *CSI_task_count_prev_frame_each_thread;
    int *FFT_task_count_prev_frame_each_thread;
    int *ZF_task_count_prev_frame_each_thread;
    int *Demul_task_count_prev_frame_each_thread;
    int *IFFT_task_count_prev_frame_each_thread;
    int *Precode_task_count_prev_frame_each_thread;

    double **CSI_task_duration_prev_frame_each_thread;
    double **FFT_task_duration_prev_frame_each_thread;
    double **ZF_task_duration_prev_frame_each_thread;
    double **Demul_task_duration_prev_frame_each_thread;
    double **IFFT_task_duration_prev_frame_each_thread;
    double **Precode_task_duration_prev_frame_each_thread;

};


#endif
