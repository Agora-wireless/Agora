/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 * 
 */
#include "doprecode.hpp"
#include "Consumer.hpp"

using namespace arma;

DoPrecode::DoPrecode(Config* in_config, int in_tid,
    moodycamel::ConcurrentQueue<Event_data>& in_task_queue, Consumer& in_consumer,
    Table<complex_float>& in_precoder_buffer,
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
    , Precode_task_duration(in_stats_manager->precode_stats_worker.task_duration)
{
    int BS_ANT_NUM = config_->BS_ANT_NUM;
    int UE_NUM = config_->UE_NUM;

    size_t mod_type = config_->mod_type;
    init_modulation_table(qam_table, mod_type);

    Precode_task_count = in_stats_manager->precode_stats_worker.task_count;
    // Precode_task_duration = in_Precode_task_duration;
    // Precode_task_count = in_Precode_task_count;

    modulated_buffer_temp = (complex_float*)aligned_alloc(64, UE_NUM * sizeof(complex_float));
    int demul_block_size = config_->demul_block_size;
    precoded_buffer_temp = (complex_float*)aligned_alloc(64, demul_block_size * BS_ANT_NUM * sizeof(complex_float));
    // precoded_buffer_temp = (complex_float **)aligned_alloc(64, demul_block_size * sizeof(complex_float *));
    // for (int i = 0; i < demul_block_size; i++) {
    //     precoded_buffer_temp[i] =  (complex_float *)aligned_alloc(64, BS_ANT_NUM * sizeof(complex_float))
    // }
}

DoPrecode::~DoPrecode()
{
}

void DoPrecode::launch(int offset)
{
    int OFDM_DATA_NUM = config_->OFDM_DATA_NUM;
    int demul_block_size = config_->demul_block_size;
    int demul_block_num = 1 + (OFDM_DATA_NUM - 1) / demul_block_size;
    int sc_id = offset % demul_block_num * demul_block_size;
    int total_data_subframe_id = offset / demul_block_num;
    int data_subframe_num_perframe = config_->data_symbol_num_perframe;
    int frame_id = total_data_subframe_id / data_subframe_num_perframe;
    int current_data_subframe_id = total_data_subframe_id % data_subframe_num_perframe;

    int BS_ANT_NUM = config_->BS_ANT_NUM;
    int UE_NUM = config_->UE_NUM;
    int OFDM_DATA_START = config_->OFDM_DATA_START;
#if DEBUG_UPDATE_STATS
    double start_time = get_time();
#endif

    __m256i index = _mm256_setr_epi64x(0, BS_ANT_NUM, BS_ANT_NUM * 2, BS_ANT_NUM * 3);

    int precoder_cache_line_num = UE_NUM * BS_ANT_NUM * sizeof(double) / 64;

    // double start_time = get_time();
    int max_sc_ite = std::min(demul_block_size, OFDM_DATA_NUM - sc_id);
    // printf("In doPrecode thread %d: frame: %d, subframe: %d, subcarrier: %d, max_sc_ite: %d\n", tid, frame_id, current_data_subframe_id, sc_id, max_sc_ite);

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

            for (int line_idx = 0; line_idx < precoder_cache_line_num; line_idx++) {
                _mm_prefetch((char*)(precoder_buffer_[precoder_offset] + line_idx * 8), _MM_HINT_T0);
            }

            complex_float* data_ptr = modulated_buffer_temp;
            if ((unsigned)current_data_subframe_id == config_->dl_data_symbol_start - 1 + DL_PILOT_SYMS) {
                for (int user_id = 0; user_id < UE_NUM; user_id++)
                    data_ptr[user_id] = { config_->pilots_[cur_sc_id], 0 };
            } else {
                _mm_prefetch((char*)(dl_IQ_data[current_data_subframe_id] + cur_sc_id), _MM_HINT_T0);
                for (int user_id = 0; user_id < UE_NUM - 1; user_id++) {
                    // int *raw_data_ptr = &dl_IQ_data[current_data_subframe_id * UE_NUM + user_id][cur_sc_id];
                    int8_t* raw_data_ptr = &dl_IQ_data[current_data_subframe_id][cur_sc_id + OFDM_DATA_NUM * user_id];
                    // cout<<*raw_data_ptr<<", ";
                    _mm_prefetch((char*)dl_IQ_data[current_data_subframe_id][cur_sc_id + OFDM_DATA_NUM * (user_id + 1)], _MM_HINT_T0);
                    data_ptr[user_id] = mod_single_uint8((uint8_t) * (raw_data_ptr), qam_table);
                    // cout << data_ptr[user_id].real << "+" << data_ptr[user_id].imag << "j, ";
                }
                // cout<<endl;

                int8_t* raw_data_ptr = &dl_IQ_data[current_data_subframe_id][cur_sc_id + OFDM_DATA_NUM * (UE_NUM - 1)];
                data_ptr[UE_NUM - 1] = mod_single_uint8((uint8_t) * (raw_data_ptr), qam_table);
            }

            // mat_precoder size: UE_NUM \times BS_ANT_NUM
            cx_float* precoder_ptr = (cx_float*)precoder_buffer_[precoder_offset];
            cx_fmat mat_precoder(precoder_ptr, UE_NUM, BS_ANT_NUM, false);

            // mat_data size: UE_NUM \times 1
            // cx_fmat mat_data((cx_float *)data_ptr, UE_NUM, 1, false);
            cx_fmat mat_data((cx_float*)data_ptr, 1, UE_NUM, false);
            // cout << "Frame: "<< frame_id<<", subframe: "<< current_data_subframe_id<<", SC: " << sc_id+i << ", data: " << real(mat_data).st() << endl;

            // mat_precoded size: BS_ANT_NUM \times 1
            cx_float* precoded_ptr = (cx_float*)precoded_buffer_temp + (i + j) * BS_ANT_NUM;

            // cx_fmat mat_precoded(precoded_ptr, BS_ANT_NUM, 1, false);
            cx_fmat mat_precoded(precoded_ptr, 1, BS_ANT_NUM, false);

#if DEBUG_UPDATE_STATS_DETAILED
            double duration1 = get_time() - start_time2;
            Precode_task_duration[tid * 8][1] += duration1;
#endif
            mat_precoded = mat_data * mat_precoder;
            // mat_precoded = mat_precoder.st() * mat_data;
            // cout<<"Precoder: \n"<<mat_precoder<<endl;
            // cout<<"Precoder transposed: \n"<<mat_precoder.st()<<endl;
            // cout<<"Data: "<<mat_data<<endl;
            // cout << "Frame: "<< frame_id<<", subframe: "<< current_data_subframe_id<<", SC: " << sc_id+i << ", data: " << real(mat_precoded).st() << endl;
            // cout << "Precoded data:" ;
            // for (int j = 0; j < BS_ANT_NUM; j++) {
            //     cout <<*((float *)(precoded_ptr+j)) << "+j"<<*((float *)(precoded_ptr+j)+1)<<",   ";
            // }
            // cout<<endl;
        }
#if DEBUG_UPDATE_STATS_DETAILED
        double duration2 = get_time() - start_time1;
        Precode_task_duration[tid * 8][2] += duration2;
        //double start_time3 = get_time();
#endif

        //         /* copy data to ifft input, 4 subcarriers per iteration */
        //         double *precoded_ptr = (double *)precoded_buffer_temp + i * BS_ANT_NUM;
        //         for (int ant_id = 0; ant_id < BS_ANT_NUM; ant_id++) {
        //             int ifft_buffer_offset = generateOffset3d(BS_ANT_NUM, frame_id, current_data_subframe_id, ant_id);
        //             double* ifft_ptr = (double *)&dl_ifft_buffer_[ifft_buffer_offset][sc_id + i + OFDM_DATA_START];

        //             double *input_shifted_ptr = precoded_ptr + ant_id;
        //             __m256d t_data = _mm256_i64gather_pd(input_shifted_ptr, index, 8);
        //             _mm256_stream_pd(ifft_ptr, t_data);
        //         }
        // #if DEBUG_UPDATE_STATS_DETAILED
        //         double duration3 = get_time() - start_time3;
        //         Precode_task_duration[tid * 8][3] += duration3;
        // #endif
    }

#if DEBUG_UPDATE_STATS_DETAILED
    double start_time3 = get_time();
#endif

    float* precoded_ptr = (float*)precoded_buffer_temp;
    for (int ant_id = 0; ant_id < BS_ANT_NUM; ant_id++) {
        int ifft_buffer_offset = ant_id + BS_ANT_NUM * total_data_subframe_id;
        float* ifft_ptr = (float*)&dl_ifft_buffer_[ifft_buffer_offset][sc_id + OFDM_DATA_START];
        for (int i = 0; i < demul_block_size / 4; i++) {
            float* input_shifted_ptr = precoded_ptr + 4 * i * 2 * BS_ANT_NUM + ant_id * 2;
            __m256d t_data = _mm256_i64gather_pd((double*)input_shifted_ptr, index, 8);
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
    // inform main thread
    Event_data precode_finish_event;
    precode_finish_event.event_type = EVENT_PRECODE;
    precode_finish_event.data = offset;
    consumer_.handle(precode_finish_event);
#if DEBUG_PRINT_IN_TASK
    printf("In doPrecode thread %d: finished frame: %d, subframe: %d, subcarrier: %d , offset: %d\n", tid,
        frame_id, current_data_subframe_id, sc_id, offset);
#endif
}
