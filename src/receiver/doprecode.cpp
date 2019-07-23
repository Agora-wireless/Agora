#include "doprecode.hpp"

using namespace arma;

DoPrecode::DoPrecode(int in_tid, int in_demul_block_size, int in_transpose_block_size,
        moodycamel::ConcurrentQueue<Event_data> *in_complete_task_queue, moodycamel::ProducerToken *in_task_ptok,
        complex_float **in_dl_modulated_buffer, complex_float **in_precoder_buffer, complex_float **in_dl_precoded_data_buffer, 
        complex_float **in_dl_ifft_buffer, int **in_dl_IQ_data, float **in_qam16_table,
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
    qam16_table = in_qam16_table;


    Precode_task_duration = in_Precode_task_duration;
    Precode_task_count = in_Precode_task_count;

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

    
    // double start_time = get_time();
    int max_sc_ite;
    if (sc_id + demul_block_size <= OFDM_DATA_NUM) 
        max_sc_ite = demul_block_size;
    else
        max_sc_ite = OFDM_DATA_NUM - sc_id;
    // printf("In doPrecode thread %d: frame: %d, subframe: %d, subcarrier: %d, max_sc_ite: %d\n", tid, frame_id, current_data_subframe_id, sc_id, max_sc_ite);
    for (int i = 0; i < max_sc_ite; i++) { 
        int cur_sc_id = sc_id + i;

        complex_float *data_ptr = &dl_modulated_buffer_[total_data_subframe_id][UE_NUM * cur_sc_id];
        for (int user_id = 0; user_id < UE_NUM; user_id ++) {
            int *raw_data_ptr = &dl_IQ_data[current_data_subframe_id * UE_NUM + user_id][cur_sc_id];
            // cout<<*raw_data_ptr<<", ";
            *(data_ptr + user_id) = mod_16qam_single(*(raw_data_ptr), qam16_table);
            // cout<<(*(data_ptr + user_id)).real<<"+"<<(*(data_ptr + user_id)).imag<<"j, ";
        }
        // cout<<endl;

        int precoder_offset = frame_id * OFDM_DATA_NUM + cur_sc_id;
        // mat_precoder size: UE_NUM \times BS_ANT_NUM        
        cx_float* precoder_ptr = (cx_float *)precoder_buffer_[precoder_offset];
        cx_fmat mat_precoder(precoder_ptr, UE_NUM, BS_ANT_NUM, false);

        // mat_data size: UE_NUM \times 1
        // cx_float* data_ptr = (cx_float *)(&dl_modulated_buffer_.data[total_data_subframe_id][UE_NUM * (sc_id+i)]);
        cx_fmat mat_data((cx_float *)data_ptr, UE_NUM, 1, false);
        // cout << "Frame: "<< frame_id<<", subframe: "<< current_data_subframe_id<<", SC: " << sc_id+i << ", data: " << real(mat_data).st() << endl;

        // mat_precoded size: BS_ANT_NUM \times 1
        cx_float* precoded_ptr = (cx_float *)(&dl_precoded_data_buffer_[total_data_subframe_id][cur_sc_id * BS_ANT_NUM]);
        cx_fmat mat_precoded(precoded_ptr, BS_ANT_NUM, 1, false);

        mat_precoded = mat_precoder.st() * mat_data;
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

    // copy data to ifft input
    // int cache_line_num = transpose_block_size / 8;
    // int iteration_per_page = 64 / cache_line_num;
    // int offset_in_page = OFDM_DATA_START / 8;
    // int block_num = OFDM_DATA_NUM / transpose_block_size;

    // for(int c2 = 0; c2 < block_num; c2++) {
    //     // c3 = 0, 1, ..., transpose_block_size/8 -1 = 7
    //     // c3*8 = 0, 8, ..., 64-8
    //     if (c2 % iteration_per_page == 0 && c2 < block_num - iteration_per_page)
    //         float temp = *(src_ptr + 1024);
    //     for(int c3 = 0; c3 < cache_line_num; c3++) {
    //         // data: 256 bits = 32 bytes = 8 float values = 4 subcarriers

    //         // __m256 data = _mm256_load_ps(src_ptr);
    //         // original data order: SCs of ant1, SCs of ant2, ..., SCs of ant 96
    //         // transposed data order: SC1-32 of ants, SC33-64 of ants, ..., SC993-1024 of ants (32 blocks each with 32 subcarriers)
    //         // prefetch a cache line
    //         _mm_prefetch((char*)(src_ptr + 16), _MM_HINT_T0);
    //         float *tar_ptr_cur = tar_ptr + (c2 * BS_ANT_NUM + ant_id)* transpose_block_size * 2 + c3 * 16;
    //         _mm256_stream_ps(tar_ptr_cur, _mm256_load_ps(src_ptr));
    //         _mm256_stream_ps(tar_ptr_cur + 8, _mm256_load_ps(src_ptr + 8));
    //         // printf("In deFFT thread %d: frame %d, subframe %d, subcarrier %d %d, address offset: %d\n", tid, frame_id, subframe_id, c2, c3, tar_ptr_cur - src_ptr);
    //         src_ptr += 16;
    //     }            
    // } 
    __m256i index = _mm256_setr_epi64x(0, BS_ANT_NUM, BS_ANT_NUM * 2, BS_ANT_NUM * 3);
    float* precoded_ptr = (float *)&dl_precoded_data_buffer_[total_data_subframe_id][sc_id * BS_ANT_NUM];
    for (int ant_id = 0; ant_id < BS_ANT_NUM; ant_id++) {
        int ifft_buffer_offset = generateOffset3d(BS_ANT_NUM, frame_id, current_data_subframe_id, ant_id);
        float* ifft_ptr = (float *)&dl_ifft_buffer_[ifft_buffer_offset][sc_id + OFDM_DATA_START];
        for (int i = 0; i< demul_block_size/4; i++) {
            float *input_shifted_ptr = precoded_ptr + 4 * i * 2 * BS_ANT_NUM + ant_id * 2;
            __m256d t_data = _mm256_i64gather_pd((double *)input_shifted_ptr, index, 8);
            _mm256_stream_pd((double *)(ifft_ptr + i * 8), t_data);
        }
    }

#if DEBUG_UPDATE_STATS
    Precode_task_count[tid*16] = Precode_task_count[tid*16]+1;
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