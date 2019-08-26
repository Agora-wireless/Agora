/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 * 
 */
#include "dofft.hpp"

DoFFT::DoFFT(int in_tid, int in_transpose_block_size, 
    moodycamel::ConcurrentQueue<Event_data> *in_complete_task_queue, moodycamel::ProducerToken *in_task_ptok,
    char **in_socket_buffer, int **in_socket_buffer_status, complex_float **in_data_buffer_, complex_float **in_csi_buffer, float *in_pilots,
    complex_float **in_dl_ifft_buffer, char *in_dl_socket_buffer, 
    double **in_FFT_task_duration, double **in_CSI_task_duration, int *in_FFT_task_count, int *in_CSI_task_count,
    double **in_IFFT_task_duration, int *in_IFFT_task_count) 
{
    tid = in_tid;
    transpose_block_size = in_transpose_block_size;
    complete_task_queue_ = in_complete_task_queue;
    task_ptok = in_task_ptok;

    socket_buffer_ = in_socket_buffer;
    socket_buffer_status_ = in_socket_buffer_status;
    data_buffer_ = in_data_buffer_;
    csi_buffer_ = in_csi_buffer;
    pilots_ = in_pilots;

    dl_socket_buffer_ = in_dl_socket_buffer;
    dl_ifft_buffer_ = in_dl_ifft_buffer;


    FFT_task_duration = in_FFT_task_duration;
    CSI_task_duration = in_CSI_task_duration;
    FFT_task_count = in_FFT_task_count;
    CSI_task_count = in_CSI_task_count;
    IFFT_task_duration = in_IFFT_task_duration;
    IFFT_task_count = in_IFFT_task_count;


    int FFT_buffer_block_num = 1;
    fft_buffer_.FFT_inputs = (complex_float **)malloc(FFT_buffer_block_num * sizeof(complex_float *)); 
    for (int i = 0; i < FFT_buffer_block_num; i++) {
        fft_buffer_.FFT_inputs[i] = (complex_float *)aligned_alloc(64, OFDM_CA_NUM * sizeof(complex_float));
        memset(fft_buffer_.FFT_inputs[i], 0, sizeof(OFDM_CA_NUM * sizeof(complex_float)));
    }

    fft_buffer_.FFT_outputs = (complex_float **)malloc(FFT_buffer_block_num * sizeof(complex_float *));
    for (int i = 0; i < FFT_buffer_block_num; i++) {
        fft_buffer_.FFT_outputs[i] = (complex_float *)aligned_alloc(64, OFDM_CA_NUM * sizeof(complex_float));
    }

    mkl_status = DftiCreateDescriptor(&mkl_handle, DFTI_SINGLE, DFTI_COMPLEX, 1, OFDM_CA_NUM);
    // mkl_status = DftiSetValue(mkl_handle, DFTI_PLACEMENT, DFTI_NOT_INPLACE);
    mkl_status = DftiCommitDescriptor(mkl_handle);

    mkl_status_dl = DftiCreateDescriptor(&mkl_handle_dl, DFTI_SINGLE, DFTI_COMPLEX, 1, OFDM_CA_NUM);
    mkl_status_dl = DftiCommitDescriptor(mkl_handle_dl);

    // muplans_ = mufft_create_plan_1d_c2c(OFDM_CA_NUM, MUFFT_FORWARD, MUFFT_FLAG_CPU_ANY);

}


DoFFT::~DoFFT()
{
    int FFT_buffer_block_num = 1;
    for (int i = 0; i < FFT_buffer_block_num; i++) {
        free(fft_buffer_.FFT_inputs[i]);
        free(fft_buffer_.FFT_outputs[i]);
    }
    DftiFreeDescriptor(&mkl_handle);
    DftiFreeDescriptor(&mkl_handle_dl);
}

void DoFFT::FFT(int offset)
{
#if DEBUG_UPDATE_STATS
    double start_time = get_time();
#endif
    
    int socket_thread_id, cur_offset;
    interpreteOffset2d_setbits(offset, &socket_thread_id, &cur_offset, 28);
    offset = cur_offset;
    // int socket_thread_id = offset / buffer_subframe_num_;
    // offset = offset - socket_thread_id * buffer_subframe_num;
    // offset = offset % buffer_subframe_num_;
    // printf("In doFFT: socket_thread: %d offset %d\n", socket_thread_id, offset);
    // read info of one frame
    char *cur_buffer_ptr = socket_buffer_[socket_thread_id] + (long long) offset * package_length;

    int ant_id, frame_id, subframe_id, cell_id;
    frame_id = *((int *)cur_buffer_ptr);
    subframe_id = *((int *)cur_buffer_ptr + 1);
    cell_id = *((int *)cur_buffer_ptr + 2);
    ant_id = *((int *)cur_buffer_ptr + 3);
    // printf("thread %d process frame_id %d, subframe_id %d, cell_id %d, ant_id %d\n", tid, frame_id, subframe_id, cell_id, ant_id);
    // remove CP, do FFT
    int delay_offset = 0;
    // int FFT_buffer_target_id = getFFTBufferIndex(frame_id, subframe_id, ant_id);
    // int FFT_buffer_target_id = (frame_id % TASK_BUFFER_FRAME_NUM) * (subframe_num_perframe) + subframe_id;


    // transfer ushort to float
    short *cur_buffer_ptr_ushort = (short *)(cur_buffer_ptr + 64 + OFDM_PREFIX_LEN * 2);
    // float *cur_fft_buffer_float = (float *)fft_buffer_.FFT_inputs[FFT_buffer_target_id];
    // float *cur_fft_buffer_float = (float *)(fft_buffer_.FFT_inputs[FFT_buffer_target_id] + ant_id * OFDM_CA_NUM);
    // float *cur_fft_buffer_float = (float *)(fft_buffer_.FFT_inputs[tid] + ant_id * OFDM_CA_NUM);
    float *cur_fft_buffer_float = (float *)(fft_buffer_.FFT_inputs[0]);

    int pilot_symbol = -1;
    if (isPilot(subframe_id))
        pilot_symbol = 1;
    else if (isData(subframe_id))
        pilot_symbol = 0;


    // Use SIMD
    // reference: https://stackoverflow.com/questions/50597764/convert-signed-short-to-float-in-c-simd
    // 0x4380'8000
    const __m256 magic = _mm256_set1_ps(float((1<<23) + (1<<15))/32768.f);
    const __m256i magic_i = _mm256_castps_si256(magic);
    for (int i = 0; i < OFDM_CA_NUM * 2; i += 16) {
        // get input:
        __m128i val = _mm_load_si128((__m128i*)(cur_buffer_ptr_ushort + i)); // port 2,3

        __m128i val1 = _mm_load_si128((__m128i*)(cur_buffer_ptr_ushort + i + 8)); 
        // interleave with 0x0000
        __m256i val_unpacked = _mm256_cvtepu16_epi32(val); // port 5
        /// convert by xor-ing and subtracting magic value:
        // VPXOR avoids port5 bottlenecks on Intel CPUs before SKL
        __m256i val_f_int = _mm256_xor_si256(val_unpacked, magic_i); // port 0,1,5
        __m256 val_f = _mm256_castsi256_ps(val_f_int);  // no instruction
        __m256 converted = _mm256_sub_ps(val_f, magic); // port 1,5 ?
        // store:
        // __m256 converted = _mm256_set1_ps(0); 
        _mm256_store_ps(cur_fft_buffer_float + i, converted); // port 2,3,4,7
        // _mm256_load_ps((cur_fft_buffer_float + i));


        __m256i val_unpacked1 = _mm256_cvtepu16_epi32(val1); // port 5
        /// convert by xor-ing and subtracting magic value:
        // VPXOR avoids port5 bottlenecks on Intel CPUs before SKL
        __m256i val_f_int1 = _mm256_xor_si256(val_unpacked1, magic_i); // port 0,1,5
        __m256 val_f1 = _mm256_castsi256_ps(val_f_int1);  // no instruction
        __m256 converted1 = _mm256_sub_ps(val_f1, magic); // port 1,5 ?
        // store:
        // __m256 converted = _mm256_set1_ps(0); 
        _mm256_store_ps(cur_fft_buffer_float + i + 8, converted1); // port 2,3,4,7
        // _mm256_load_ps((cur_fft_buffer_float + i));

    }

#if DEBUG_PLOT
    if (subframe_id == 0 && frame_id == 100 && ant_id == 1)
    {
        std::vector<float> rx(cur_fft_buffer_float, cur_fft_buffer_float+2304*2);
        std::vector<double> rx_I(2304);
        for (int i = 0; i < 2304; i++) rx_I[i] = (double)rx[2*i];
        FILE *fi = fopen("rx100_0_1.bin","wb");
        fwrite(rx_I.data(), sizeof(float), 2304*2, fi);
        fclose(fi);
    }
#endif

    // printf("In doFFT thread %d: frame: %d, subframe: %d, ant: %d\n", tid, frame_id%TASK_BUFFER_FRAME_NUM, subframe_id, ant_id);
    // printf("FFT input\n");
    // for ( int i = 0; i< OFDM_CA_NUM; i++) {
    //     // std::cout <<"("<<i<<", "<<(*(fft_buffer_.FFT_inputs[0] +i)).real<<","<<(*(fft_buffer_.FFT_inputs[0] +i)).imag<<") ";
    //     std::cout <<" "<<(*(fft_buffer_.FFT_inputs[0] +i)).real<<"+"<<(*(fft_buffer_.FFT_inputs[0] +i)).imag<<"*1j";
    //     // cout <<"("<<(*(fft_buffer_.FFT_inputs[FFT_buffer_target_id]+i)).real<<","<<(*(fft_buffer_.FFT_inputs[FFT_buffer_target_id]+i)).imag<<") ";
    //     // printf("(%.4f, %.4f) ", *((float *)(fft_buffer_.FFT_inputs[FFT_buffer_target_id] + ant_id * OFDM_CA_NUM+i)), *((float *)(fft_buffer_.FFT_inputs[FFT_buffer_target_id] + ant_id * OFDM_CA_NUM+i)+1));
    // }
    // printf("\n size of float _Complex: %d", sizeof(float _Complex));
    // printf("\n");


#if DEBUG_UPDATE_STATS_DETAILED
    double start_time1 = get_time();
    double duration1 = start_time1 - start_time;
    if (pilot_symbol == 0)
        FFT_task_duration[tid * 8][1] += duration1;
    else 
        CSI_task_duration[tid * 8][1] += duration1;
#endif
    // for(int i = 0; i < (OFDM_CA_NUM - delay_offset) * 2; i++)
    //     cur_fft_buffer_float[i] = cur_ptr_buffer_ushort[OFDM_PREFIX_LEN + delay_offset + i] * csi_format_offset;
    
    // append zero
    // if(delay_offset > 0) 
    //     memset((char *)fft_buffer_.FFT_inputs[FFT_buffer_target_id] 
    //         + (OFDM_CA_NUM - delay_offset) * 2 * sizeof(float), 0, sizeof(float) * 2 * delay_offset);
    // mufft_execute_plan_1d(muplans_[tid], fft_buffer_.FFT_outputs[tid] + ant_id * OFDM_CA_NUM, 
    //     fft_buffer_.FFT_inputs[tid] + ant_id * OFDM_CA_NUM);
    // mufft_execute_plan_1d(muplans_, fft_buffer_.FFT_outputs[0], fft_buffer_.FFT_inputs[0]);

    DftiComputeForward(mkl_handle, fft_buffer_.FFT_inputs[0]);
    // DftiComputeForward(mkl_handle, fft_buffer_.FFT_inputs[0], fft_buffer_.FFT_outputs[0]);


#if DEBUG_UPDATE_STATS_DETAILED
    double start_time2 = get_time();
    double duration2 = start_time2 - start_time1;
    if (pilot_symbol == 0)
        FFT_task_duration[tid * 8][2] += duration2;
    else
        CSI_task_duration[tid * 8][2] += duration2;
#endif
    // mufft_execute_plan_1d(muplans_[tid], fft_buffer_.FFT_outputs[FFT_buffer_target_id] + ant_id * OFDM_CA_NUM, 
    //     fft_buffer_.FFT_inputs[FFT_buffer_target_id] + ant_id * OFDM_CA_NUM);
    // printf("In doFFT thread %d: frame: %d, subframe: %d, ant: %d\n", tid, frame_id%TASK_BUFFER_FRAME_NUM, subframe_id, ant_id);
    // printf("FFT output\n");
    // for ( int i = 0; i< OFDM_CA_NUM; i++) {
    //     std::cout <<"("<<i<<", "<<(*(fft_buffer_.FFT_outputs[0] +i)).real<<","<<(*(fft_buffer_.FFT_outputs[0] +i)).imag<<") ";
    //     // cout <<"("<<(*(fft_buffer_.FFT_inputs[FFT_buffer_target_id]+i)).real<<","<<(*(fft_buffer_.FFT_inputs[FFT_buffer_target_id]+i)).imag<<") ";
    //     // printf("(%.4f, %.4f) ", *((float *)(fft_buffer_.FFT_inputs[FFT_buffer_target_id] + ant_id * OFDM_CA_NUM+i)), *((float *)(fft_buffer_.FFT_inputs[FFT_buffer_target_id] + ant_id * OFDM_CA_NUM+i)+1));
    // }
    // printf("\n");
    // printf("\n FFT output: \n");
    // for ( int i = 0; i< OFDM_CA_NUM; i++) {
    //     printf("(%.4f, %.4f) ", *((float *)(fft_buffer_.FFT_outputs[tid]+i)), *((float *)(fft_buffer_.FFT_outputs[tid]+i)+1));
    //      // printf("(%.4f, %.4f) ", *((float *)(fft_buffer_.FFT_outputs[FFT_buffer_target_id] + ant_id * OFDM_CA_NUM+i)), *((float *)(fft_buffer_.FFT_outputs[FFT_buffer_target_id] + ant_id * OFDM_CA_NUM+i)+1));
    // }
    // printf("\n");


#if DEBUG_PRINT_IN_TASK
        printf("In doFFT thread %d: frame: %d, subframe: %d, ant: %d\n", tid, frame_id%TASK_BUFFER_FRAME_NUM, subframe_id, ant_id);
#endif
#if DEBUG_UPDATE_STATS_DETAILED
    double start_time_part3 = get_time();
#endif
    // if it is pilot part, do CE
    if(pilot_symbol == 1) {
        int UE_id = subframe_id;
        // int ca_offset = (frame_id % TASK_BUFFER_FRAME_NUM) * OFDM_CA_NUM;
        // int csi_offset = ant_id + UE_id * BS_ANT_NUM;
        int subframe_offset = (frame_id % TASK_BUFFER_FRAME_NUM) * UE_NUM + UE_id;
        // int csi_offset = UE_id + ant_id * UE_NUM;

        // float* cur_fft_buffer_float_output = (float*)(fft_buffer_.FFT_outputs[tid] + ant_id * OFDM_CA_NUM);
        float* cur_fft_buffer_float_output = (float*)(fft_buffer_.FFT_inputs[0]); //+ OFDM_DATA_START * 2;

        // Use SIMD
        float* csi_buffer_ptr = (float*)(csi_buffer_[subframe_offset]);
        int sc_idx = OFDM_DATA_START;
        int dst_idx = 0;

        int cache_line_num = transpose_block_size / 8;
        int iteration_per_page = 64 / cache_line_num;
        int offset_in_page = OFDM_DATA_START / 8;
        int block_num = OFDM_DATA_NUM / transpose_block_size;
        _mm_prefetch((char*)pilots_, _MM_HINT_T0);
        for (int block_idx = 0; block_idx < block_num; block_idx ++) {
            // if (block_idx % iteration_per_page == 0 && block_idx < block_num - iteration_per_page)
            //     float temp = *(cur_fft_buffer_float_output + sc_idx * 2 + 1024);
            for (int sc_inblock_idx = 0; sc_inblock_idx < transpose_block_size; sc_inblock_idx += 8) {
                // load 8 floats (4 bytes) / 4 complex floats
                float *src_ptr_cur = cur_fft_buffer_float_output + sc_idx * 2;
                // _mm_prefetch((char*)(src_ptr_cur + 16), _MM_HINT_T0);
                // _mm_prefetch((char*)(pilots_ + sc_idx * 2 + 16), _MM_HINT_T0);
                __m256 pilot_rx = _mm256_load_ps(src_ptr_cur);
                __m256 pilot_tx = _mm256_set_ps(pilots_[sc_idx+3], pilots_[sc_idx+3], pilots_[sc_idx+2], pilots_[sc_idx+2], 
                                                pilots_[sc_idx+1], pilots_[sc_idx+1], pilots_[sc_idx], pilots_[sc_idx]);
                __m256 csi_est = _mm256_mul_ps(pilot_rx, pilot_tx);

                __m256 pilot_rx1 = _mm256_load_ps(src_ptr_cur + 8);
                __m256 pilot_tx1 = _mm256_set_ps(pilots_[sc_idx+7], pilots_[sc_idx+7], pilots_[sc_idx+6], pilots_[sc_idx+6], 
                                                pilots_[sc_idx+5], pilots_[sc_idx+5], pilots_[sc_idx+4], pilots_[sc_idx+4]);
                __m256 csi_est1 = _mm256_mul_ps(pilot_rx1, pilot_tx1);

                float *tar_ptr_cur = csi_buffer_ptr + (block_idx * BS_ANT_NUM + ant_id) * transpose_block_size * 2 + sc_inblock_idx * 2;
                _mm256_stream_ps(tar_ptr_cur, csi_est);
                _mm256_stream_ps(tar_ptr_cur + 8, csi_est1);
                // printf("subcarrier index: %d, pilot: %.2f, %.2f, %.2f, %2.f\n", sc_idx, pilots_[sc_idx], pilots_[sc_idx+1], pilots_[sc_idx+2], pilots_[sc_idx+3]);
                // float *temp = (float *) &csi_est;
                // float *temp_rx = (float *)&pilot_rx;
                // float *temp_tx = (float *)&pilot_tx;
                // printf("Pilot_rx: %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f\n ", temp_rx[0],temp_rx[1],temp_rx[2],temp_rx[3],temp_rx[4],temp_rx[5],temp_rx[6],temp_rx[7]);
                // printf("Pilot_tx: %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f\n ", temp_tx[0],temp_tx[1],temp_tx[2],temp_tx[3],temp_tx[4],temp_tx[5],temp_tx[6],temp_tx[7]);
                // printf("CSI: %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f\n ", temp[0],temp[1],temp[2],temp[3],temp[4],temp[5],temp[6],temp[7]);
                sc_idx += 8;
                
            }
        }
        
        // std::cout<<"Before: "<<std::endl;
        // for (int i =0;i<OFDM_CA_NUM; i++) {
        //     std::cout<<"("<<i<<", "<<fft_buffer_.FFT_inputs[tid][i].real<<","<<fft_buffer_.FFT_inputs[tid][i].imag<<") ";
        // }
        // std::cout<<std::endl;
        // printf("In doFFT thread %d: frame: %d, subframe: %d, ant: %d\n", tid, frame_id%TASK_BUFFER_FRAME_NUM, subframe_id, ant_id);
        // std::cout<<"After: "<<std::endl;
        // for (int i =0;i<OFDM_CA_NUM; i++) {
        //     cout<<"("<<i<<", "<<fft_buffer_.FFT_outputs[tid][i].real*pilots_[i]<<","<<fft_buffer_.FFT_outputs[tid][i].imag*pilots_[i]<<") ";
        // }
        // for (int i =0;i<OFDM_DATA_NUM*BS_ANT_NUM; i++) {
        //     std::cout<<"("<<i<<", "<<csi_buffer_[subframe_offset][i].real<<"+j"<<csi_buffer_[subframe_offset][i].imag<<") ";
        // }
        // std::cout<<std::endl;

    }
    else if (pilot_symbol == 0) {
        
        int data_subframe_id = subframe_id - UE_NUM;
        int frame_offset = (frame_id % TASK_BUFFER_FRAME_NUM) * data_subframe_num_perframe + data_subframe_id;
        
        // cx_fmat mat_fft_cx_output((cx_float *)fft_buffer_.FFT_outputs[FFT_buffer_target_id], OFDM_CA_NUM, 1, false);


        /* //naive transpose
        for(int j = 0; j < OFDM_CA_NUM; j++)
        {
            data_buffer_.data[frame_offset][ant_id + j * BS_ANT_NUM] = fft_buffer_.FFT_outputs[FFT_buffer_target_id][j];
        }
        */
        // block transpose
        // src_ptr: point to the start of subframe (subframe size: OFDM_CA_NUM) 
        // 2048 float values
        // cx_float *src_ptr = (cx_float *)&fft_buffer_.FFT_outputs[FFT_buffer_target_id][0];
        // cx_mat mat_data_buffer(src_ptr, BS_ANT_NUM, )


        // float *src_ptr = (float *)&fft_buffer_.FFT_outputs[FFT_buffer_target_id][0];
        // float *src_ptr = (float *)&fft_buffer_.FFT_outputs[tid][ant_id*OFDM_CA_NUM] + OFDM_DATA_START * 2;
        float *src_ptr = (float *)fft_buffer_.FFT_inputs[0] + OFDM_DATA_START * 2;

        // printf("FFT output: \n");
        // for ( int i = 0; i< OFDM_CA_NUM; i++) {
        //      printf("(%.4f, %.4f) ", *(src_ptr+i*2), *(src_ptr+i*2+1));
        // }
        // printf("\n");

        // tar_ptr: point to the start of subframe with size BS_ANT_NUM * OFDM_CA_NUM
        // 96*1024*2 float values
        float *tar_ptr = (float *)&data_buffer_[frame_offset][0];
        // copy data from fft_outputs to data_buffer
        // 1024*2/8 = 256 iterations, copy 8 bytes every time
        // c2 = 0, 1, ..., 1024/64*2-1 = 31
        // c2*transpose_block_size = 0, 64, 128, ..., 2048-64
        int cache_line_num = transpose_block_size / 8;
        int iteration_per_page = 64 / cache_line_num;
        int offset_in_page = OFDM_DATA_START / 8;
        int block_num = OFDM_DATA_NUM / transpose_block_size;
        for(int c2 = 0; c2 < block_num; c2++) {
            // c3 = 0, 1, ..., transpose_block_size/8 -1 = 7
            // c3*8 = 0, 8, ..., 64-8
            // if (c2 % iteration_per_page == 0 && c2 < block_num - iteration_per_page)
            //     float temp = *(src_ptr + 1024);
            for(int c3 = 0; c3 < cache_line_num; c3++) {
                // data: 256 bits = 32 bytes = 8 float values = 4 subcarriers

                // __m256 data = _mm256_load_ps(src_ptr);
                // original data order: SCs of ant1, SCs of ant2, ..., SCs of ant 96
                // transposed data order: SC1-32 of ants, SC33-64 of ants, ..., SC993-1024 of ants (32 blocks each with 32 subcarriers)
                // prefetch a cache line
                // _mm_prefetch((char*)(src_ptr + 16), _MM_HINT_T0);
                float *tar_ptr_cur = tar_ptr + (c2 * BS_ANT_NUM + ant_id)* transpose_block_size * 2 + c3 * 16;
                _mm256_stream_ps(tar_ptr_cur, _mm256_load_ps(src_ptr));
                _mm256_stream_ps(tar_ptr_cur + 8, _mm256_load_ps(src_ptr + 8));
                // printf("In deFFT thread %d: frame %d, subframe %d, subcarrier %d %d, address offset: %d\n", tid, frame_id, subframe_id, c2, c3, tar_ptr_cur - src_ptr);
                src_ptr += 16;
            }            
        }        
    }
#if DEBUG_UPDATE_STATS_DETAILED
    double end_time = get_time();
    double duration3 = end_time - start_time_part3;
    if (pilot_symbol == 0)
        FFT_task_duration[tid * 8][3] += duration3;
    else
        CSI_task_duration[tid * 8][3] += duration2;
#endif    

    // after finish
    socket_buffer_status_[socket_thread_id][offset] = 0; // now empty
    // printf("In doFFT: emptied socket buffer frame: %d, subframe: %d, ant: %d, offset: %d\n",frame_id, subframe_id, ant_id, offset);
    // inform main thread
#if DEBUG_UPDATE_STATS
    double duration = get_time() - start_time;
    if (pilot_symbol == 0) {
        FFT_task_count[tid * 16] = FFT_task_count[tid * 16]+1;
        FFT_task_duration[tid * 8][0] += duration;
        // if (duration > 500) {
        //     printf("Thread %d FFT takes %.2f\n", tid, duration);
        // }
    }
    else {
        CSI_task_count[tid * 16] = CSI_task_count[tid * 16]+1;
        CSI_task_duration[tid * 8][0] += duration;
        // if (duration > 500) {
        //     printf("Thread %d pilot FFT takes %.2f\n", tid, duration);
        // }
    }
#endif
    Event_data fft_finish_event;
    fft_finish_event.event_type = EVENT_FFT;
    fft_finish_event.data = generateOffset2d((frame_id % TASK_BUFFER_FRAME_NUM), subframe_id);
    // fft_finish_event.data = generateOffset2d(subframe_num_perframe, frame_id, subframe_id);
    // getSubframeBufferIndex(frame_id, subframe_id);
    
    

    // if ( !complete_task_queue_.enqueue(*task_ptok[tid], fft_finish_event ) ) {
    if ( !complete_task_queue_->enqueue(*task_ptok, fft_finish_event ) ) {
        printf("fft message enqueue failed\n");
        exit(0);
    }

}



void DoFFT::IFFT(int offset)
{
#if DEBUG_UPDATE_STATS
    double start_time = get_time();
#endif
    int frame_id, total_data_subframe_id, current_data_subframe_id, ant_id;
    interpreteOffset3d(offset, &frame_id, &current_data_subframe_id, &ant_id);
    int offset_in_buffer = ant_id + BS_ANT_NUM * (current_data_subframe_id + frame_id * data_subframe_num_perframe);
    // interpreteOffset3d(BS_ANT_NUM, offset, &frame_id, &total_data_subframe_id, &current_data_subframe_id, &ant_id);
#if DEBUG_PRINT_IN_TASK
        printf("In doIFFT thread %d: frame: %d, subframe: %d, antenna: %d\n", tid, frame_id, current_data_subframe_id, ant_id);
#endif

    // cout << "In ifft: frame: "<< frame_id<<", subframe: "<< current_data_subframe_id<<", ant: " << ant_id << ", input data: ";
    // for (int j = 0; j <OFDM_CA_NUM; j++) {
    //     cout << dl_ifft_buffer_.IFFT_inputs[offset][j].real << "+" << dl_ifft_buffer_.IFFT_inputs[offset][j].imag << "j,   ";
    // }
    // cout<<"\n\n"<<endl;
    // mufft_execute_plan_1d(muplans_ifft_[tid], dl_ifft_buffer_.IFFT_outputs[offset], 
    //     dl_ifft_buffer_.IFFT_inputs[offset]);

    DftiComputeBackward(mkl_handle_dl, dl_ifft_buffer_[offset_in_buffer]);

    // cout << "In ifft: frame: "<< frame_id<<", subframe: "<< current_data_subframe_id<<", ant: " << ant_id <<", offset: "<<offset <<", output data: ";
    // for (int j = 0; j <OFDM_CA_NUM; j++) {
    //     cout << dl_ifft_buffer_.IFFT_outputs[offset][j].real << "+" << dl_ifft_buffer_.IFFT_outputs[offset][j].imag << "j,   ";
    // }
    // cout<<"\n\n"<<endl;

    // calculate data for downlink socket buffer 
    float *ifft_output_ptr = (float *)(&dl_ifft_buffer_[offset_in_buffer][0]);
    int socket_subframe_offset = (frame_id % SOCKET_BUFFER_FRAME_NUM) * data_subframe_num_perframe + current_data_subframe_id;
    char *socket_ptr = &dl_socket_buffer_[socket_subframe_offset * BS_ANT_NUM * package_length];
    int socket_offset = sizeof(int) * 16 + ant_id * package_length;

    // for (int sc_id = 0; sc_id < OFDM_CA_NUM; sc_id++) {
    //     float *shifted_input_ptr = (float *)(ifft_output_ptr + 2 * sc_id);
    //     // ifft scaled results by 2048, 16 = 2^15/2048
    //     *((short *)(socket_ptr + socket_offset) + 2 * sc_id ) = (short)(*shifted_input_ptr * 16);
    //     *((short *)(socket_ptr + socket_offset) + 2 * sc_id + 1 ) = (short)(*(shifted_input_ptr+1) * 16);
    // }

    
    socket_ptr = socket_ptr + socket_offset;
    for (int sc_id = 0; sc_id < OFDM_CA_NUM; sc_id += 8) {
        __m256 scale_factor = _mm256_set1_ps(16);
        __m256 ifft1 = _mm256_load_ps(ifft_output_ptr + 2 * sc_id);
        __m256 ifft2 = _mm256_load_ps(ifft_output_ptr + 2 * sc_id + 8);
        __m256 scaled_ifft1 = _mm256_mul_ps(ifft1, scale_factor);
        __m256 scaled_ifft2 = _mm256_mul_ps(ifft2, scale_factor);
        __m256i integer1 = _mm256_cvtps_epi32(scaled_ifft1);
        __m256i integer2 = _mm256_cvtps_epi32(scaled_ifft2);
        integer1 = _mm256_packs_epi32(integer1, integer2);
        integer1 = _mm256_permute4x64_epi64(integer1, 0xD8);
        _mm256_stream_si256((__m256i *) (socket_ptr + sc_id * 4), integer1);
    }



    // cout << "In ifft: frame: "<< frame_id<<", subframe: "<< current_data_subframe_id<<", ant: " << ant_id << ", data: ";
    // for (int j = 0; j <OFDM_CA_NUM; j++) {
    //     int socket_offset = sizeof(int) * 16 + ant_id * PackageReceiver::package_length;
    //     cout <<*((short *)(socket_ptr + socket_offset) + 2 * j)  << "+j"<<*((short *)(socket_ptr + socket_offset) + 2 * j + 1 )<<",   ";
    // }
    // cout<<"\n\n"<<endl;

#if DEBUG_UPDATE_STATS   
    IFFT_task_count[tid*16] = IFFT_task_count[tid*16]+1;
    IFFT_task_duration[tid*8][0] += get_time() - start_time;
#endif

    // inform main thread
    Event_data ifft_finish_event;
    ifft_finish_event.event_type = EVENT_IFFT;
    ifft_finish_event.data = offset;

    if ( !complete_task_queue_->enqueue(*task_ptok, ifft_finish_event ) ) {
        printf("IFFT message enqueue failed\n");
        exit(0);
    }

}
