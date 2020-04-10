/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#include "doprecode.hpp"
#include "concurrent_queue_wrapper.hpp"

using namespace arma;

DoPrecode::DoPrecode(Config* in_config, int in_tid,
    moodycamel::ConcurrentQueue<Event_data>& in_task_queue,
    moodycamel::ConcurrentQueue<Event_data>& complete_task_queue,
    moodycamel::ProducerToken* worker_producer_token,
    Table<complex_float>& in_precoder_buffer,
    Table<complex_float>& in_dl_ifft_buffer,
#ifdef USE_LDPC
    Table<int8_t>& in_dl_encoded_data,
#else
    Table<int8_t>& in_dl_raw_data,
#endif
    Stats* in_stats_manager)
    : Doer(in_config, in_tid, in_task_queue, complete_task_queue,
          worker_producer_token)
    , precoder_buffer_(in_precoder_buffer)
    , dl_ifft_buffer_(in_dl_ifft_buffer)
#ifdef USE_LDPC
    , dl_raw_data(in_dl_encoded_data)
#else
    , dl_raw_data(in_dl_raw_data)
#endif
{
    duration_stat
        = in_stats_manager->get_duration_stat(DoerType::kPrecode, in_tid);
    init_modulation_table(qam_table, cfg->mod_type);

    alloc_buffer_1d(&modulated_buffer_temp, cfg->UE_NUM, 64, 0);
    alloc_buffer_1d(
        &precoded_buffer_temp, cfg->demul_block_size * cfg->BS_ANT_NUM, 64, 0);
}

DoPrecode::~DoPrecode()
{
    free_buffer_1d(&modulated_buffer_temp);
    free_buffer_1d(&precoded_buffer_temp);
}

Event_data DoPrecode::launch(int offset)
{
    int sc_id = offset % cfg->demul_block_num * cfg->demul_block_size;
    int total_data_subframe_id = offset / cfg->demul_block_num;
    int data_subframe_num_perframe = cfg->data_symbol_num_perframe;
    int frame_id = total_data_subframe_id / data_subframe_num_perframe;
    int current_data_subframe_id
        = total_data_subframe_id % data_subframe_num_perframe;

#if DEBUG_UPDATE_STATS
    double start_time = get_time();
#endif
#if DEBUG_PRINT_IN_TASK
    printf("In doPrecode thread %d: frame: %d, subframe: %d, subcarrier: %d\n",
        tid, frame_id, current_data_subframe_id, sc_id);
#endif

    __m256i index = _mm256_setr_epi64x(
        0, cfg->BS_ANT_NUM, cfg->BS_ANT_NUM * 2, cfg->BS_ANT_NUM * 3);
    int max_sc_ite
        = std::min(cfg->demul_block_size, cfg->OFDM_DATA_NUM - sc_id);

    for (int i = 0; i < max_sc_ite; i = i + 4) {
#if DEBUG_UPDATE_STATS_DETAILED
        double start_time1 = get_time();
#endif
        for (int j = 0; j < 4; j++) {
#if DEBUG_UPDATE_STATS_DETAILED
            double start_time2 = get_time();
#endif
            int cur_sc_id = sc_id + i + j;
            int precoder_offset = frame_id * cfg->OFDM_DATA_NUM + cur_sc_id;
            if (cfg->freq_orthogonal_pilot)
                precoder_offset = precoder_offset - cur_sc_id % cfg->UE_NUM;

            complex_float* data_ptr = modulated_buffer_temp;
            if ((unsigned)current_data_subframe_id
                <= cfg->dl_data_symbol_start - 1 + DL_PILOT_SYMS) {
                for (size_t user_id = 0; user_id < cfg->UE_NUM; user_id++)
                    data_ptr[user_id]
                        = { cfg->pilots_[cur_sc_id + cfg->OFDM_DATA_START], 0 };
            } else {
                int subframe_id_in_buffer
                    = current_data_subframe_id - cfg->dl_data_symbol_start;

                for (size_t user_id = 0; user_id < cfg->UE_NUM; user_id++) {
#ifdef USE_LDPC
                    int8_t* raw_data_ptr
                        = &dl_raw_data[total_data_subframe_id][cur_sc_id
                            + cfg->OFDM_DATA_NUM * user_id];
#else
                    int8_t* raw_data_ptr
                        = &dl_raw_data[subframe_id_in_buffer][cur_sc_id
                            + cfg->OFDM_DATA_NUM * user_id];
#endif
                    data_ptr[user_id] = mod_single_uint8(
                        (uint8_t) * (raw_data_ptr), qam_table);
                }
            }

            cx_float* precoder_ptr
                = (cx_float*)precoder_buffer_[precoder_offset];
            cx_fmat mat_precoder(
                precoder_ptr, cfg->UE_NUM, cfg->BS_ANT_NUM, false);
            cx_fmat mat_data((cx_float*)data_ptr, 1, cfg->UE_NUM, false);
            cx_float* precoded_ptr
                = (cx_float*)precoded_buffer_temp + (i + j) * cfg->BS_ANT_NUM;
            cx_fmat mat_precoded(precoded_ptr, 1, cfg->BS_ANT_NUM, false);

#if DEBUG_UPDATE_STATS_DETAILED
            duration_stat->task_duration[1] += get_time() - start_time2;
#endif
            mat_precoded = mat_data * mat_precoder;

            // printf("In doPrecode thread %d: frame: %d, subframe: %d, "
            //        "subcarrier: % d\n ",
            //     tid, frame_id, current_data_subframe_id, sc_id);
            // cout << "Precoder: \n" << mat_precoder << endl;
            // cout << "Data: \n" << mat_data << endl;
            // cout << "Precoded data: \n" << mat_precoded << endl;
        }
#if DEBUG_UPDATE_STATS_DETAILED
        duration_stat->task_duration[2] += get_time() - start_time1;
#endif
    }

#if DEBUG_UPDATE_STATS_DETAILED
    double start_time3 = get_time();
#endif

    float* precoded_ptr = (float*)precoded_buffer_temp;
    for (size_t ant_id = 0; ant_id < cfg->BS_ANT_NUM; ant_id++) {
        int ifft_buffer_offset
            = ant_id + cfg->BS_ANT_NUM * total_data_subframe_id;
        float* ifft_ptr = (float*)&dl_ifft_buffer_[ifft_buffer_offset][sc_id
            + cfg->OFDM_DATA_START];
        for (size_t i = 0; i < cfg->demul_block_size / 4; i++) {
            float* input_shifted_ptr
                = precoded_ptr + 4 * i * 2 * cfg->BS_ANT_NUM + ant_id * 2;
            __m256d t_data
                = _mm256_i64gather_pd((double*)input_shifted_ptr, index, 8);
            _mm256_stream_pd((double*)(ifft_ptr + i * 8), t_data);
        }
    }
#if DEBUG_UPDATE_STATS_DETAILED
    duration_stat->task_duration[3] += get_time() - start_time3;
#endif

#if DEBUG_UPDATE_STATS
    duration_stat->task_duration[0] += get_time() - start_time;
    duration_stat->task_count++;
#endif

#if DEBUG_PRINT_IN_TASK
    printf("In doPrecode thread %d: finished frame: %d, subframe: %d, "
           "subcarrier: %d , offset: %d\n",
        tid, frame_id, current_data_subframe_id, sc_id, offset);
#endif
    return Event_data(EventType::kPrecode, offset);
}
