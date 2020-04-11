/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#include "reciprocity.hpp"
#include "concurrent_queue_wrapper.hpp"

using namespace arma;
Reciprocity::Reciprocity(Config* in_config, int in_tid, double freq_ghz,
    moodycamel::ConcurrentQueue<Event_data>& in_task_queue,
    moodycamel::ConcurrentQueue<Event_data>& complete_task_queue,
    moodycamel::ProducerToken* worker_producer_token,
    Table<complex_float>& in_calib_buffer,
    Table<complex_float>& in_recip_buffer, Stats* in_stats_manager)
    : Doer(in_config, in_tid, freq_ghz, in_task_queue, complete_task_queue,
          worker_producer_token)
    , calib_buffer_(in_calib_buffer)
    , recip_buffer_(in_recip_buffer)
{
    duration_stat = in_stats_manager->get_duration_stat(DoerType::kRC, in_tid);
    BS_ANT_NUM = cfg->BS_ANT_NUM;
    OFDM_DATA_NUM = cfg->OFDM_DATA_NUM;

    calib_gather_buffer = (complex_float*)aligned_alloc(
        64, BS_ANT_NUM * OFDM_DATA_NUM * sizeof(complex_float));
}

Reciprocity::~Reciprocity() { free(calib_gather_buffer); }

Event_data Reciprocity::launch(int offset)
{
#if DEBUG_PRINT_IN_TASK
    printf("In doRecip thread %d: frame: %d, \n", tid, offset);
#endif

    size_t start_tsc1 = worker_rdtsc();

    cx_float* ptr_in = (cx_float*)calib_buffer_[offset];
    cx_fmat mat_input(ptr_in, OFDM_DATA_NUM, BS_ANT_NUM, false);
    cx_fvec vec_calib_ref = mat_input.col(cfg->ref_ant);
    cx_float* ptr_out = (cx_float*)calib_gather_buffer;
    cx_fmat mat_output(ptr_out, BS_ANT_NUM, OFDM_DATA_NUM, false);
    complex_float* recip_buff = recip_buffer_[offset];

    for (int ant_id = 0; ant_id < BS_ANT_NUM; ant_id++) {
        cx_fvec vec_calib = mat_input.col(ant_id);
        cx_fvec recipFactor = vec_calib_ref / vec_calib;
        mat_output.row(ant_id) = recipFactor;
        for (int sc_id = ant_id; sc_id < OFDM_DATA_NUM; sc_id += BS_ANT_NUM) {
            // TODO: interpolate here
            for (int i = 0; i < BS_ANT_NUM; i++) {
                recip_buff[(sc_id + i) * BS_ANT_NUM + ant_id].re
                    = mat_output.at(ant_id, sc_id).real();
                recip_buff[(sc_id + i) * BS_ANT_NUM + ant_id].im
                    = mat_output.at(ant_id, sc_id).imag();
            }
        }
    }
    // cout<<"Precoder:" <<mat_output<<endl;

    duration_stat->task_duration[1] += worker_rdtsc() - start_tsc1;
    size_t start_tsc2 = worker_rdtsc();
    duration_stat->task_duration[2] += start_tsc2 - start_tsc1;
    duration_stat->task_duration[3] += worker_rdtsc() - start_tsc2;

    double duration = worker_rdtsc() - start_tsc1;
    duration_stat->task_duration[0] += duration;
    if (cycles_to_us(duration, freq_ghz) > 500) {
        printf(
            "Thread %d RC takes %.2f\n", tid, cycles_to_us(duration, freq_ghz));
    }
    duration_stat->task_count++;

    return Event_data(EventType::kRC, offset);
}
