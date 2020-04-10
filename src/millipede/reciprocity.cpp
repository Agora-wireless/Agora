/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#include "reciprocity.hpp"
#include "concurrent_queue_wrapper.hpp"

using namespace arma;
Reciprocity::Reciprocity(Config* in_config, int in_tid,
    moodycamel::ConcurrentQueue<Event_data>& in_task_queue,
    moodycamel::ConcurrentQueue<Event_data>& complete_task_queue,
    moodycamel::ProducerToken* worker_producer_token,
    Table<complex_float>& in_calib_buffer,
    Table<complex_float>& in_recip_buffer, Stats* in_stats_manager)
    : Doer(in_config, in_tid, in_task_queue, complete_task_queue,
          worker_producer_token)
    , calib_buffer_(in_calib_buffer)
    , recip_buffer_(in_recip_buffer)
{
    duration_stat = in_stats_manager->get_duration_stat(DoerType::kRC, in_tid);
    BS_ANT_NUM = cfg->BS_ANT_NUM;
    OFDM_DATA_NUM = cfg->OFDM_DATA_NUM;
}

Reciprocity::~Reciprocity() {}

Event_data Reciprocity::launch(int offset)
{
#if DEBUG_PRINT_IN_TASK
    printf("In doRecip thread %d: frame: %d, \n", tid, offset);
#endif

#if DEBUG_UPDATE_STATS
    double start_time1 = get_time();
#endif

    cx_float* ptr_in = (cx_float*)calib_buffer_[offset];
    cx_fmat mat_input(ptr_in, OFDM_DATA_NUM, BS_ANT_NUM, false);
    cx_fvec vec_calib_ref = mat_input.col(cfg->ref_ant);
    cx_float* recip_buff = (cx_float*)recip_buffer_[offset];
    cx_fmat calib_mat = mat_input.each_col() / vec_calib_ref;
    cx_fmat recip_mat(recip_buff, BS_ANT_NUM, OFDM_DATA_NUM, false);
    recip_mat = calib_mat.st();
    for (int i = 0; i < OFDM_DATA_NUM; i += BS_ANT_NUM) {
        // TODO: interpolate instead of steps
        recip_mat.cols(i, std::min(i + BS_ANT_NUM - 1, OFDM_DATA_NUM))
            .each_col()
            = recip_mat.col(i);
    }

#if DEBUG_UPDATE_STATS_DETAILED
    duration_stat->task_duration[1] += get_time() - start_time1;
#endif

#if DEBUG_UPDATE_STATS_DETAILED
    double start_time2 = get_time();
    duration_stat->task_duration[2] += start_time2 - start_time1;
#endif

#if DEBUG_UPDATE_STATS_DETAILED
    duration_stat->task_duration[3] += get_time() - start_time2;
#endif

#if DEBUG_UPDATE_STATS
    double duration = get_time() - start_time1;
    duration_stat->task_duration[0] += duration;
    if (duration > 500) {
        printf("Thread %d RC takes %.2f\n", tid, duration);
    }
    duration_stat->task_count++;
#endif

    return Event_data(EventType::kRC, offset);
}
