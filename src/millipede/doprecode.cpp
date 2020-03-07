/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#include "doprecode.hpp"
#include "Consumer.hpp"

using namespace arma;

DoPrecode::DoPrecode(Config* in_config, int in_tid,
    moodycamel::ConcurrentQueue<Event_data>& in_task_queue,
    Consumer& in_consumer, Table<complex_float>& in_precoder_buffer,
    Table<complex_float>& in_dl_ifft_buffer,
#ifdef USE_LDPC
    Table<int8_t>& in_dl_encoded_data,
#else
    Table<int8_t>& in_dl_IQ_data,
#endif
    Stats* in_stats_manager)
    : Doer(in_config, in_tid, in_task_queue, in_consumer)
    , precoder_buffer_(in_precoder_buffer)
    , dl_ifft_buffer_(in_dl_ifft_buffer)
#ifdef USE_LDPC
    , dl_IQ_data(in_dl_encoded_data)
#else
    , dl_IQ_data(in_dl_IQ_data)
#endif
    , Precode_task_duration(
          in_stats_manager->precode_stats_worker.task_duration)
{

    Precode_task_count = in_stats_manager->precode_stats_worker.task_count;

    size_t mod_type = config_->mod_type;
    init_modulation_table(qam_table, mod_type);

    int BS_ANT_NUM = config_->BS_ANT_NUM;
    int UE_NUM = config_->UE_NUM;
    int demul_block_size = config_->demul_block_size;
    alloc_buffer_1d(&modulated_buffer_temp, UE_NUM, 64, 0);
    alloc_buffer_1d(
        &precoded_buffer_temp, demul_block_size * BS_ANT_NUM, 64, 0);
}

DoPrecode::~DoPrecode()
{
    free_buffer_1d(&modulated_buffer_temp);
    free_buffer_1d(&precoded_buffer_temp);
}

void DoPrecode::launch(int offset)
{
    int OFDM_DATA_NUM = config_->OFDM_DATA_NUM;
    int demul_block_size = config_->demul_block_size;
    int sc_id = offset % config_->demul_block_num * demul_block_size;
    int total_data_subframe_id = offset / config_->demul_block_num;
    int data_subframe_num_perframe = config_->data_symbol_num_perframe;
    int frame_id = total_data_subframe_id / data_subframe_num_perframe;
    int current_data_subframe_id
        = total_data_subframe_id % data_subframe_num_perframe;

    int BS_ANT_NUM = config_->BS_ANT_NUM;
    int UE_NUM = config_->UE_NUM;
    int OFDM_DATA_START = config_->OFDM_DATA_START;
#if DEBUG_UPDATE_STATS
    double start_time = get_time();
#endif
#if DEBUG_PRINT_IN_TASK
    printf("In doPrecode thread %d: frame: %d, subframe: %d, subcarrier: %d\n",
        tid, frame_id, current_data_subframe_id, sc_id);
#endif

    __m256i index
        = _mm256_setr_epi64x(0, BS_ANT_NUM, BS_ANT_NUM * 2, BS_ANT_NUM * 3);
    int precoder_cache_line_num = UE_NUM * BS_ANT_NUM * sizeof(double) / 64;
    int max_sc_ite = std::min(demul_block_size, OFDM_DATA_NUM - sc_id);

    for (int i = 0; i < max_sc_ite; i = i + 4) {
#if DEBUG_UPDATE_STATS_DETAILED
        double start_time1 = get_time();
#endif
        for (int j = 0; j < 4; j++) {
#if DEBUG_UPDATE_STATS_DETAILED
            double start_time2 = get_time();
#endif
            int cur_sc_id = sc_id + i + j;
            int precoder_offset = frame_id * OFDM_DATA_NUM + cur_sc_id;
            if (config_->freq_orthogonal_pilot)
                precoder_offset = precoder_offset - cur_sc_id % UE_NUM;

            for (int line_idx = 0; line_idx < precoder_cache_line_num;
                 line_idx++) {
                _mm_prefetch(
                    (char*)(precoder_buffer_[precoder_offset] + line_idx * 8),
                    _MM_HINT_T0);
            }

            complex_float* data_ptr = modulated_buffer_temp;
            if ((unsigned)current_data_subframe_id
                == config_->dl_data_symbol_start - 1 + DL_PILOT_SYMS) {
                for (int user_id = 0; user_id < UE_NUM; user_id++)
                    data_ptr[user_id] = { config_->pilots_[cur_sc_id], 0 };
            } else {
                int subframe_id_in_buffer
                    = current_data_subframe_id - config_->dl_data_symbol_start;
                _mm_prefetch(
                    (char*)(dl_IQ_data[subframe_id_in_buffer] + cur_sc_id),
                    _MM_HINT_T0);
                // printf("In doPrecode thread %d: frame: %d, subframe: %d,
                // subcarrier: %d\n", tid, frame_id, current_data_subframe_id,
                // cur_sc_id); printf("raw data: \n");
                for (int user_id = 0; user_id < UE_NUM - 1; user_id++) {
                    int8_t* raw_data_ptr
                        = &dl_IQ_data[subframe_id_in_buffer]
                                     [cur_sc_id + OFDM_DATA_NUM * user_id];
                    int8_t* next_raw_data_ptr
                        = &dl_IQ_data[subframe_id_in_buffer][cur_sc_id
                            + OFDM_DATA_NUM * (user_id + 1)];
                    _mm_prefetch((char*)next_raw_data_ptr, _MM_HINT_T0);
                    // printf("%u ", *raw_data_ptr);
                    data_ptr[user_id] = mod_single_uint8(
                        (uint8_t) * (raw_data_ptr), qam_table);
                }
                // printf("\n");
                int8_t* raw_data_ptr
                    = &dl_IQ_data[subframe_id_in_buffer]
                                 [cur_sc_id + OFDM_DATA_NUM * (UE_NUM - 1)];
                data_ptr[UE_NUM - 1]
                    = mod_single_uint8((uint8_t) * (raw_data_ptr), qam_table);
            }

            cx_float* precoder_ptr
                = (cx_float*)precoder_buffer_[precoder_offset];
            cx_fmat mat_precoder(precoder_ptr, UE_NUM, BS_ANT_NUM, false);
            cx_fmat mat_data((cx_float*)data_ptr, 1, UE_NUM, false);
            cx_float* precoded_ptr
                = (cx_float*)precoded_buffer_temp + (i + j) * BS_ANT_NUM;
            cx_fmat mat_precoded(precoded_ptr, 1, BS_ANT_NUM, false);

#if DEBUG_UPDATE_STATS_DETAILED
            double duration1 = get_time() - start_time2;
            Precode_task_duration[tid * 8][1] += duration1;
#endif
            mat_precoded = mat_data * mat_precoder;
            // cout<<"Precoder: \n"<<mat_precoder<<endl;
            // cout<<"Data: \n"<<mat_data<<endl;
            // printf("In doPrecode thread %d: frame: %d, subframe: %d,
            // subcarrier: %d\n", tid, frame_id, current_data_subframe_id,
            // cur_sc_id); cout << "Precoded data:" ; for (int j = 0; j <
            // BS_ANT_NUM; j++) {
            //     cout <<*((float *)(precoded_ptr+j)) << "+j"<<*((float
            //     *)(precoded_ptr+j)+1)<<",   ";
            // }
            // cout<<endl;
        }
#if DEBUG_UPDATE_STATS_DETAILED
        double duration2 = get_time() - start_time1;
        Precode_task_duration[tid * 8][2] += duration2;
#endif
    }

#if DEBUG_UPDATE_STATS_DETAILED
    double start_time3 = get_time();
#endif

    float* precoded_ptr = (float*)precoded_buffer_temp;
    for (int ant_id = 0; ant_id < BS_ANT_NUM; ant_id++) {
        int ifft_buffer_offset = ant_id + BS_ANT_NUM * total_data_subframe_id;
        float* ifft_ptr = (float*)&dl_ifft_buffer_[ifft_buffer_offset]
                                                  [sc_id + OFDM_DATA_START];
        for (int i = 0; i < demul_block_size / 4; i++) {
            float* input_shifted_ptr
                = precoded_ptr + 4 * i * 2 * BS_ANT_NUM + ant_id * 2;
            __m256d t_data
                = _mm256_i64gather_pd((double*)input_shifted_ptr, index, 8);
            _mm256_stream_pd((double*)(ifft_ptr + i * 8), t_data);
        }
    }
#if DEBUG_UPDATE_STATS_DETAILED
    double duration3 = get_time() - start_time3;
    Precode_task_duration[tid * 8][3] += duration3;
#endif

#if DEBUG_UPDATE_STATS
    Precode_task_count[tid * 16] = Precode_task_count[tid * 16] + max_sc_ite;
    Precode_task_duration[tid * 8][0] += get_time() - start_time;
#endif
    /* inform main thread */
    Event_data precode_finish_event;
    precode_finish_event.event_type = EVENT_PRECODE;
    precode_finish_event.data = offset;
    consumer_.handle(precode_finish_event);
#if DEBUG_PRINT_IN_TASK
    printf("In doPrecode thread %d: finished frame: %d, subframe: %d, "
           "subcarrier: %d , offset: %d\n",
        tid, frame_id, current_data_subframe_id, sc_id, offset);
#endif
}
