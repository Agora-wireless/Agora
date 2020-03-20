/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#include "reciprocity.hpp"
#include "Consumer.hpp"

using namespace arma;
Reciprocity::Reciprocity(Config* in_config, int in_tid,
    moodycamel::ConcurrentQueue<Event_data>& in_task_queue,
    Consumer& in_consumer, Table<complex_float>& in_calib_buffer,
    Table<complex_float>& in_recip_buffer, Stats* in_stats_manager)
    : Doer(in_config, in_tid, in_task_queue, in_consumer)
    , calib_buffer_(in_calib_buffer)
    , recip_buffer_(in_recip_buffer)
    , RC_task_duration(&in_stats_manager->rc_stats_worker.task_duration)
    , RC_task_count(in_stats_manager->rc_stats_worker.task_count)
{
    bs_ant_num = cfg->bs_ant_num;
    ofdm_data_num = cfg->ofdm_data_num;

    calib_gather_buffer = (complex_float*)aligned_alloc(
        64, bs_ant_num * ofdm_data_num * sizeof(complex_float));
}

Reciprocity::~Reciprocity() { free(calib_gather_buffer); }

void Reciprocity::launch(int offset)
{
    int ref_ant = cfg->ref_ant;

#if DEBUG_PRINT_IN_TASK
    printf("In doRecip thread %d: frame: %d, \n", tid, offset);
#endif

#if DEBUG_UPDATE_STATS
    double start_time1 = get_time();
#endif

    cx_float* ptr_in = (cx_float*)calib_buffer_[offset];
    cx_fmat mat_input(ptr_in, ofdm_data_num, bs_ant_num, false);
    cx_fvec vec_calib_ref = mat_input.col(ref_ant);
    cx_float* ptr_out = (cx_float*)calib_gather_buffer;
    cx_fmat mat_output(ptr_out, bs_ant_num, ofdm_data_num, false);
    complex_float* recip_buff = recip_buffer_[offset];

    for (int ant_id = 0; ant_id < bs_ant_num; ant_id++) {
        cx_fvec vec_calib = mat_input.col(ant_id);
        cx_fvec recipFactor = vec_calib_ref / vec_calib;
        mat_output.row(ant_id) = recipFactor;
        for (int sc_id = ant_id; sc_id < ofdm_data_num; sc_id += bs_ant_num) {
            // TODO: interpolate here
            for (int i = 0; i < bs_ant_num; i++) {
                recip_buff[(sc_id + i) * bs_ant_num + ant_id].re
                    = mat_output.at(ant_id, sc_id).real();
                recip_buff[(sc_id + i) * bs_ant_num + ant_id].im
                    = mat_output.at(ant_id, sc_id).imag();
            }
        }
    }

#if DEBUG_UPDATE_STATS_DETAILED
    double duration1 = get_time() - start_time1;
    (*RC_task_duration)[tid * 8][1] += duration1;
    // double start_time2 = get_time();
#endif

#if DEBUG_UPDATE_STATS_DETAILED
    double start_time2 = get_time();
    double duration2 = start_time2 - start_time1;
    (*RC_task_duration)[tid * 8][2] += duration2;
    // double start_time2 = get_time();
#endif

    // cout<<"Precoder:" <<mat_output<<endl;
#if DEBUG_UPDATE_STATS_DETAILED
    double duration3 = get_time() - start_time2;
    (*RC_task_duration)[tid * 8][3] += duration3;
    // double start_time3 = get_time();
#endif

#if DEBUG_UPDATE_STATS
    RC_task_count[tid * 16] = RC_task_count[tid * 16] + 1;
    double duration = get_time() - start_time1;
    (*RC_task_duration)[tid * 8][0] += duration;
    if (duration > 500) {
        printf("Thread %d RC takes %.2f\n", tid, duration);
    }
#endif

    /* Inform main thread */
    Event_data RC_finish_event(EventType::kRC, offset);
    consumer_.handle(RC_finish_event);
}
