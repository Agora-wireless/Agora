/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 * 
 */
#include "doprecode.hpp"

using namespace arma;

DoPrecode::DoPrecode(int in_tid, int in_demul_block_size, int in_transpose_block_size,
        moodycamel::ConcurrentQueue<Event_data> *in_complete_task_queue, moodycamel::ProducerToken *in_task_ptok,
        complex_float **in_dl_modulated_buffer, complex_float **in_precoder_buffer, complex_float **in_dl_precoded_data_buffer, 
        complex_float **in_dl_ifft_buffer, int **in_dl_IQ_data, 
        double **in_Precode_task_duration, int *in_Precode_task_count)
{
    tid = in_tid;
    demul_block_size = in_demul_block_size;
    transpose_block_size = in_transpose_block_size;
    complete_task_queue_ = in_complete_task_queue;
    task_ptok = in_task_ptok;

    dl_modulated_buffer_ = in_dl_modulated_buffer;
    precoder_buffer_ = in_precoder_buffer;
    dl_precoded_data_buffer_ = in_dl_precoded_data_buffer;
    dl_ifft_buffer_ = in_dl_ifft_buffer;
    dl_IQ_data = in_dl_IQ_data;
    // qam16_table = in_qam16_table;
    qam16_table = (float **)malloc(2 * sizeof(float *));
    for (int i = 0; i < 2; i++) 
        qam16_table[i] = (float *)aligned_alloc(64, 16 * sizeof(float));


    Precode_task_duration = in_Precode_task_duration;
    Precode_task_count = in_Precode_task_count;

    modulated_buffer_temp = (complex_float *)aligned_alloc(64, UE_NUM * sizeof(complex_float));
    precoded_buffer_temp =  (complex_float *)aligned_alloc(64, demul_block_size * BS_ANT_NUM * sizeof(complex_float));
    // precoded_buffer_temp = (complex_float **)aligned_alloc(64, demul_block_size * sizeof(complex_float *));
    // for (int i = 0; i < demul_block_size; i++) {
    //     precoded_buffer_temp[i] =  (complex_float *)aligned_alloc(64, BS_ANT_NUM * sizeof(complex_float))
    // }

}

DoPrecode::~DoPrecode() 
{

}


void DoPrecode::Precode(int offset) 
{
#if DEBUG_UPDATE_STATS
    double start_time = get_time();
#endif
    int frame_id, total_data_subframe_id, current_data_subframe_id, sc_id;
    interpreteOffset3d(OFDM_DATA_NUM, offset, &frame_id, &total_data_subframe_id, &current_data_subframe_id, &sc_id);
    __m256i index = _mm256_setr_epi64x(0, BS_ANT_NUM, BS_ANT_NUM * 2, BS_ANT_NUM * 3);

    int precoder_cache_line_num = UE_NUM * BS_ANT_NUM * sizeof(double) / 64;
    
    // double start_time = get_time();
    int max_sc_ite;
    if (sc_id + demul_block_size <= OFDM_DATA_NUM) 
        max_sc_ite = demul_block_size;
    else
        max_sc_ite = OFDM_DATA_NUM - sc_id;
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


            for (int line_idx = 0; line_idx < precoder_cache_line_num; line_idx ++) {
                _mm_prefetch((char *)(precoder_buffer_[precoder_offset] + line_idx * 8), _MM_HINT_T0);
            }

            // complex_float *data_ptr = &dl_modulated_buffer_[total_data_subframe_id][UE_NUM * cur_sc_id];
            _mm_prefetch((char *)(dl_IQ_data[current_data_subframe_id * UE_NUM]+cur_sc_id), _MM_HINT_T0);
            complex_float *data_ptr = modulated_buffer_temp;
            for (int user_id = 0; user_id < UE_NUM - 1; user_id ++) {
                int *raw_data_ptr = &dl_IQ_data[current_data_subframe_id * UE_NUM + user_id][cur_sc_id];
                // cout<<*raw_data_ptr<<", ";
                _mm_prefetch((char *)dl_IQ_data[current_data_subframe_id * UE_NUM + user_id + 1], _MM_HINT_T0);
                *(data_ptr + user_id) = mod_16qam_single(*(raw_data_ptr), qam16_table);
                
                // cout<<(*(data_ptr + user_id)).real<<"+"<<(*(data_ptr + user_id)).imag<<"j, ";
            }
            // cout<<endl;

            int *raw_data_ptr = &dl_IQ_data[current_data_subframe_id * UE_NUM + UE_NUM - 1][cur_sc_id];
            *(data_ptr + UE_NUM - 1) = mod_16qam_single(*(raw_data_ptr), qam16_table);


            
            // mat_precoder size: UE_NUM \times BS_ANT_NUM        
            cx_float* precoder_ptr = (cx_float *)precoder_buffer_[precoder_offset];
            cx_fmat mat_precoder(precoder_ptr, UE_NUM, BS_ANT_NUM, false);

            // mat_data size: UE_NUM \times 1
            // cx_float* data_ptr = (cx_float *)(&dl_modulated_buffer_.data[total_data_subframe_id][UE_NUM * (sc_id+i)]);
            // cx_fmat mat_data((cx_float *)data_ptr, UE_NUM, 1, false);
            cx_fmat mat_data((cx_float *)data_ptr, 1, UE_NUM, false);
            // cout << "Frame: "<< frame_id<<", subframe: "<< current_data_subframe_id<<", SC: " << sc_id+i << ", data: " << real(mat_data).st() << endl;

            // mat_precoded size: BS_ANT_NUM \times 1
            cx_float *precoded_ptr = (cx_float *)precoded_buffer_temp + (i+j) * BS_ANT_NUM;
            // cx_float* precoded_ptr = (cx_float *)(&dl_precoded_data_buffer_[total_data_subframe_id][cur_sc_id * BS_ANT_NUM]);
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
        double start_time3 = get_time();
#endif

//         /* copy data to ifft input, 4 subcarriers per iteration */ 
//         double *precoded_ptr = (double *)precoded_buffer_temp + i * BS_ANT_NUM;
//         // double* precoded_ptr = (double *)&dl_precoded_data_buffer_[total_data_subframe_id][(sc_id + i) * BS_ANT_NUM];
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

    float *precoded_ptr = (float *)precoded_buffer_temp;
    // float* precoded_ptr = (float *)&dl_precoded_data_buffer_[total_data_subframe_id][sc_id * BS_ANT_NUM];
    for (int ant_id = 0; ant_id < BS_ANT_NUM; ant_id++) {
        int ifft_buffer_offset = generateOffset3d(BS_ANT_NUM, frame_id, current_data_subframe_id, ant_id);
        float* ifft_ptr = (float *)&dl_ifft_buffer_[ifft_buffer_offset][sc_id + OFDM_DATA_START];
        for (int i = 0; i< demul_block_size/4; i++) {
            float *input_shifted_ptr = precoded_ptr + 4 * i * 2 * BS_ANT_NUM + ant_id * 2;
            __m256d t_data = _mm256_i64gather_pd((double *)input_shifted_ptr, index, 8);
            _mm256_stream_pd((double *)(ifft_ptr + i * 8), t_data);
        }
    }
#if DEBUG_UPDATE_STATS_DETAILED   
        double duration3 = get_time() - start_time3;
        Precode_task_duration[tid * 8][3] += duration3;
#endif

#if DEBUG_UPDATE_STATS
    Precode_task_count[tid*16] = Precode_task_count[tid*16] + max_sc_ite;
    Precode_task_duration[tid*8][0] += get_time() - start_time;
#endif
    // inform main thread
    Event_data precode_finish_event;
    precode_finish_event.event_type = EVENT_PRECODE;
    precode_finish_event.data = offset;
    
    if ( !complete_task_queue_->enqueue(*task_ptok, precode_finish_event ) ) {
        printf("Precoding message enqueue failed\n");
        exit(0);
    }
#if DEBUG_PRINT_IN_TASK
    printf("In doPrecode thread %d: finished frame: %d, subframe: %d, subcarrier: %d , offset: %d\n", tid, 
        frame_id, current_data_subframe_id, sc_id, offset);
#endif

}