/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 * 
 */
#include "dofft.hpp"
#include "Consumer.hpp"

DoFFT::DoFFT(Config* in_config, int in_tid,
    moodycamel::ConcurrentQueue<Event_data>& in_task_queue, Consumer& in_consumer,
    Table<char>& in_socket_buffer, Table<int>& in_socket_buffer_status,
    Table<complex_float>& in_data_buffer, Table<complex_float>& in_csi_buffer,
    Table<complex_float>& in_calib_buffer,
    Stats* in_stats_manager)
    : Doer(in_config, in_tid, in_task_queue, in_consumer)
    , socket_buffer_(in_socket_buffer)
    , socket_buffer_status_(in_socket_buffer_status)
    , data_buffer_(in_data_buffer)
    , csi_buffer_(in_csi_buffer)
    , calib_buffer_(in_calib_buffer)
    , FFT_task_duration(&in_stats_manager->fft_stats_worker.task_duration)
    , FFT_task_count(in_stats_manager->fft_stats_worker.task_count)
    , CSI_task_duration(&in_stats_manager->csi_stats_worker.task_duration)
    , CSI_task_count(in_stats_manager->csi_stats_worker.task_count)
{
    (void)DftiCreateDescriptor(&mkl_handle, DFTI_SINGLE, DFTI_COMPLEX,
        1, config_->OFDM_CA_NUM);
    // mkl_status = DftiSetValue(mkl_handle, DFTI_PLACEMENT, DFTI_NOT_INPLACE);
    (void)DftiCommitDescriptor(mkl_handle);

    int FFT_buffer_block_num = 1;
    fft_buffer_.FFT_inputs.calloc(FFT_buffer_block_num, config_->OFDM_CA_NUM, 64);
    fft_buffer_.FFT_outputs.calloc(FFT_buffer_block_num, config_->OFDM_CA_NUM, 64);
}

DoFFT::~DoFFT()
{
    DftiFreeDescriptor(&mkl_handle);
    fft_buffer_.FFT_inputs.free();
    fft_buffer_.FFT_outputs.free();
}

void DoFFT::launch(int offset)
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
    int packet_length = config_->packet_length;
    char* cur_buffer_ptr = socket_buffer_[socket_thread_id] + (long long)offset * packet_length;
    struct Packet* pkt = (struct Packet*)cur_buffer_ptr;
    size_t frame_id = pkt->frame_id % 10000;
    size_t subframe_id = pkt->symbol_id;
    size_t ant_id = pkt->ant_id;
    // printf("thread %d process frame_id %d, subframe_id %d, ant_id %d\n", tid, frame_id, subframe_id, ant_id);
    // remove CP, do FFT
    // int delay_offset = 0;
    // int FFT_buffer_target_id = getFFTBufferIndex(frame_id, subframe_id, ant_id);

    // transfer ushort to float
    size_t OFDM_PREFIX_LEN = config_->OFDM_PREFIX_LEN;
    size_t OFDM_CA_NUM = config_->OFDM_CA_NUM;
    size_t OFDM_DATA_NUM = config_->OFDM_DATA_NUM;
    size_t OFDM_DATA_START = config_->OFDM_DATA_START;
    short* cur_buffer_ptr_ushort = &pkt->data[2 * OFDM_PREFIX_LEN];
    // float *cur_fft_buffer_float = (float *)fft_buffer_.FFT_inputs[FFT_buffer_target_id];
    // float *cur_fft_buffer_float = (float *)(fft_buffer_.FFT_inputs[FFT_buffer_target_id] + ant_id * OFDM_CA_NUM);
    // float *cur_fft_buffer_float = (float *)(fft_buffer_.FFT_inputs[tid] + ant_id * OFDM_CA_NUM);
    float* cur_fft_buffer_float = (float*)(fft_buffer_.FFT_inputs[0]);

    int cur_symbol_type = UNKNOWN;
    if (config_->isUplink(frame_id, subframe_id))
        cur_symbol_type = UL;
    else if (config_->isPilot(frame_id, subframe_id))
        cur_symbol_type = PILOT;
    else if (config_->isCalDlPilot(frame_id, subframe_id))
        cur_symbol_type = CAL_DL;
    else if (config_->isCalUlPilot(frame_id, subframe_id))
        cur_symbol_type = CAL_UL;

    // Use SIMD
    // reference: https://stackoverflow.com/questions/50597764/convert-signed-short-to-float-in-c-simd
    // 0x4380'8000
    const __m256 magic = _mm256_set1_ps(float((1 << 23) + (1 << 15)) / 32768.f);
    const __m256i magic_i = _mm256_castps_si256(magic);
    for (size_t i = 0; i < OFDM_CA_NUM * 2; i += 16) {
        // get input:
        __m128i val = _mm_load_si128((__m128i*)(cur_buffer_ptr_ushort + i)); // port 2,3

        __m128i val1 = _mm_load_si128((__m128i*)(cur_buffer_ptr_ushort + i + 8));
        // interleave with 0x0000
        __m256i val_unpacked = _mm256_cvtepu16_epi32(val); // port 5
        /// convert by xor-ing and subtracting magic value:
        // VPXOR avoids port5 bottlenecks on Intel CPUs before SKL
        __m256i val_f_int = _mm256_xor_si256(val_unpacked, magic_i); // port 0,1,5
        __m256 val_f = _mm256_castsi256_ps(val_f_int); // no instruction
        __m256 converted = _mm256_sub_ps(val_f, magic); // port 1,5 ?
        // store:
        // __m256 converted = _mm256_set1_ps(0);
        _mm256_store_ps(cur_fft_buffer_float + i, converted); // port 2,3,4,7
        // _mm256_load_ps((cur_fft_buffer_float + i));

        __m256i val_unpacked1 = _mm256_cvtepu16_epi32(val1); // port 5
        /// convert by xor-ing and subtracting magic value:
        // VPXOR avoids port5 bottlenecks on Intel CPUs before SKL
        __m256i val_f_int1 = _mm256_xor_si256(val_unpacked1, magic_i); // port 0,1,5
        __m256 val_f1 = _mm256_castsi256_ps(val_f_int1); // no instruction
        __m256 converted1 = _mm256_sub_ps(val_f1, magic); // port 1,5 ?
        // store:
        // __m256 converted = _mm256_set1_ps(0);
        _mm256_store_ps(cur_fft_buffer_float + i + 8, converted1); // port 2,3,4,7
        // _mm256_load_ps((cur_fft_buffer_float + i));
    }

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
    if (cur_symbol_type == UL)
        (*FFT_task_duration)[tid * 8][1] += duration1;
    else if (cur_symbol_type == PILOT)
        (*CSI_task_duration)[tid * 8][1] += duration1;
        //else if (cur_symbol_type == CAL_DL || cur_symbol_type == CAL_UL)
        //    (*RC_task_duration)[tid * 8][1] += duration1;
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
    if (cur_symbol_type == UL)
        (*FFT_task_duration)[tid * 8][2] += duration2;
    else if (cur_symbol_type == PILOT)
        (*CSI_task_duration)[tid * 8][2] += duration2;
        //else if (cur_symbol_type == CAL_DL || cur_symbol_type == CAL_UL)
        //    RC_task_duration[tid * 8][2] += duration2;
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
    printf("In doFFT thread %d: frame: %d, subframe: %d, ant: %d\n", tid, frame_id % TASK_BUFFER_FRAME_NUM, subframe_id, ant_id);
#endif
#if DEBUG_UPDATE_STATS_DETAILED
    double start_time_part3 = get_time();
#endif
    // if it is pilot part, do CE
    size_t BS_ANT_NUM = config_->BS_ANT_NUM;
    int transpose_block_size = config_->transpose_block_size;
    if (cur_symbol_type == PILOT) {
        int pilot_id = config_->getPilotSFIndex(frame_id, subframe_id); //subframe_id;
        // int ca_offset = (frame_id % TASK_BUFFER_FRAME_NUM) * OFDM_CA_NUM;
        // int csi_offset = ant_id + UE_id * BS_ANT_NUM;
        int subframe_offset = (frame_id % TASK_BUFFER_FRAME_NUM) * config_->pilot_symbol_num_perframe + pilot_id;
        // int csi_offset = UE_id + ant_id * UE_NUM;

        int sc_idx = config_->OFDM_DATA_START;
        // float* cur_fft_buffer_float_output = (float*)(fft_buffer_.FFT_outputs[tid] + ant_id * OFDM_CA_NUM);
        float* cur_fft_buffer_float_output = (float*)(fft_buffer_.FFT_inputs[0]); //+ sc_idx * 2;

        // Use SIMD
        float* csi_buffer_ptr = (float*)(csi_buffer_[subframe_offset]);
        //int dst_idx = 0;

        // int cache_line_num = transpose_block_size / 8;
        // int iteration_per_page = 64 / cache_line_num;
        // int offset_in_page = config_->OFDM_DATA_START / 8;
        int OFDM_DATA_NUM = config_->OFDM_DATA_NUM;
        int block_num = OFDM_DATA_NUM / transpose_block_size;
        float* pilots_ = config_->pilots_;
        _mm_prefetch((char*)pilots_, _MM_HINT_T0);
        for (int block_idx = 0; block_idx < block_num; block_idx++) {
            // if (block_idx % iteration_per_page == 0 && block_idx < block_num - iteration_per_page)
            //     float temp = *(cur_fft_buffer_float_output + sc_idx * 2 + 1024);
            for (int sc_inblock_idx = 0; sc_inblock_idx < transpose_block_size; sc_inblock_idx += 8) {
                // load 8 floats (4 bytes) / 4 complex floats
                float* src_ptr_cur = cur_fft_buffer_float_output + sc_idx * 2;
                // _mm_prefetch((char*)(src_ptr_cur + 16), _MM_HINT_T0);
                // _mm_prefetch((char*)(pilots_ + sc_idx * 2 + 16), _MM_HINT_T0);
                __m256 pilot_rx = _mm256_load_ps(src_ptr_cur);
                __m256 pilot_tx = _mm256_set_ps(pilots_[sc_idx + 3], pilots_[sc_idx + 3], pilots_[sc_idx + 2], pilots_[sc_idx + 2],
                    pilots_[sc_idx + 1], pilots_[sc_idx + 1], pilots_[sc_idx], pilots_[sc_idx]);
                __m256 csi_est = _mm256_mul_ps(pilot_rx, pilot_tx);

                __m256 pilot_rx1 = _mm256_load_ps(src_ptr_cur + 8);
                __m256 pilot_tx1 = _mm256_set_ps(pilots_[sc_idx + 7], pilots_[sc_idx + 7], pilots_[sc_idx + 6], pilots_[sc_idx + 6],
                    pilots_[sc_idx + 5], pilots_[sc_idx + 5], pilots_[sc_idx + 4], pilots_[sc_idx + 4]);
                __m256 csi_est1 = _mm256_mul_ps(pilot_rx1, pilot_tx1);

                float* tar_ptr_cur = csi_buffer_ptr + (block_idx * BS_ANT_NUM + ant_id) * transpose_block_size * 2 + sc_inblock_idx * 2;
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
    } else if (cur_symbol_type == UL) {

        int data_subframe_id = config_->getUlSFIndex(frame_id, subframe_id); //subframe_id - UE_NUM;
        int data_subframe_num_perframe = config_->data_symbol_num_perframe;
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
        // float *src_ptr = (float *)&fft_buffer_.FFT_outputs[tid][ant_id*OFDM_CA_NUM] + config_->OFDM_DATA_START * 2;
        float* src_ptr = (float*)fft_buffer_.FFT_inputs[0] + config_->OFDM_DATA_START * 2;

        // printf("FFT output: \n");
        // for ( int i = 0; i< OFDM_CA_NUM; i++) {
        //      printf("(%.4f, %.4f) ", *(src_ptr+i*2), *(src_ptr+i*2+1));
        // }
        // printf("\n");

        // tar_ptr: point to the start of subframe with size BS_ANT_NUM * OFDM_CA_NUM
        // 96*1024*2 float values
        float* tar_ptr = (float*)&data_buffer_[frame_offset][0];
        // copy data from fft_outputs to data_buffer
        // 1024*2/8 = 256 iterations, copy 8 bytes every time
        // c2 = 0, 1, ..., 1024/64*2-1 = 31
        // c2*transpose_block_size = 0, 64, 128, ..., 2048-64
        int cache_line_num = transpose_block_size / 8;
        // int iteration_per_page = 64 / cache_line_num;
        // int offset_in_page = config_->OFDM_DATA_START / 8;
        int block_num = config_->OFDM_DATA_NUM / transpose_block_size;
        for (int c2 = 0; c2 < block_num; c2++) {
            // c3 = 0, 1, ..., transpose_block_size/8 -1 = 7
            // c3*8 = 0, 8, ..., 64-8
            // if (c2 % iteration_per_page == 0 && c2 < block_num - iteration_per_page)
            //     float temp = *(src_ptr + 1024);
            for (int c3 = 0; c3 < cache_line_num; c3++) {
                // data: 256 bits = 32 bytes = 8 float values = 4 subcarriers

                // __m256 data = _mm256_load_ps(src_ptr);
                // original data order: SCs of ant1, SCs of ant2, ..., SCs of ant 96
                // transposed data order: SC1-32 of ants, SC33-64 of ants, ..., SC993-1024 of ants (32 blocks each with 32 subcarriers)
                // prefetch a cache line
                // _mm_prefetch((char*)(src_ptr + 16), _MM_HINT_T0);
                float* tar_ptr_cur = tar_ptr + (c2 * BS_ANT_NUM + ant_id) * transpose_block_size * 2 + c3 * 16;
                _mm256_stream_ps(tar_ptr_cur, _mm256_load_ps(src_ptr));
                _mm256_stream_ps(tar_ptr_cur + 8, _mm256_load_ps(src_ptr + 8));
                // printf("In deFFT thread %d: frame %d, subframe %d, subcarrier %d %d, address offset: %d\n", tid, frame_id, subframe_id, c2, c3, tar_ptr_cur - src_ptr);
                src_ptr += 16;
            }
        }
    } else if (((cur_symbol_type == CAL_DL) && (ant_id == config_->ref_ant)) || ((cur_symbol_type == CAL_UL) && (ant_id != config_->ref_ant))) {
        int frame_offset = (frame_id % TASK_BUFFER_FRAME_NUM);
        int ant_offset = ant_id * OFDM_DATA_NUM;
        float* src_ptr = (float*)fft_buffer_.FFT_inputs[0] + OFDM_DATA_START * 2;
        float* tar_ptr = (float*)&calib_buffer_[frame_offset][ant_offset];
        int cache_line_num = transpose_block_size / 8;
        int block_num = OFDM_DATA_NUM / transpose_block_size;
        for (int c2 = 0; c2 < block_num; c2++) {
            for (int c3 = 0; c3 < cache_line_num; c3++) {
                float* tar_ptr_cur = tar_ptr + c2 * transpose_block_size * 2 + c3 * 16;
                _mm256_stream_ps(tar_ptr_cur, _mm256_load_ps(src_ptr));
                _mm256_stream_ps(tar_ptr_cur + 8, _mm256_load_ps(src_ptr + 8));
                src_ptr += 16;
            }
        }
    } else {
        printf("unkown or unsupported symbol type.\n");
    }
#if DEBUG_UPDATE_STATS_DETAILED
    double end_time = get_time();
    double duration3 = end_time - start_time_part3;
    if (cur_symbol_type == 0)
        (*FFT_task_duration)[tid * 8][3] += duration3;
    else
        (*CSI_task_duration)[tid * 8][3] += duration2;
#endif

    // after finish
    socket_buffer_status_[socket_thread_id][offset] = 0; // now empty
    // printf("In doFFT: emptied socket buffer frame: %d, subframe: %d, ant: %d, offset: %d\n",frame_id, subframe_id, ant_id, offset);
    // inform main thread
#if DEBUG_UPDATE_STATS
    double duration = get_time() - start_time;
    if (cur_symbol_type == 0) {
        FFT_task_count[tid * 16] = FFT_task_count[tid * 16] + 1;
        (*FFT_task_duration)[tid * 8][0] += duration;
    } else {
        CSI_task_count[tid * 16] = CSI_task_count[tid * 16] + 1;
        (*CSI_task_duration)[tid * 8][0] += duration;
    }
#endif
    Event_data fft_finish_event;
    fft_finish_event.event_type = EVENT_FFT;
    int subframe_num_perframe = config_->symbol_num_perframe;
    fft_finish_event.data = frame_id % TASK_BUFFER_FRAME_NUM * subframe_num_perframe + subframe_id;

    consumer_.handle(fft_finish_event);
}

DoIFFT::DoIFFT(Config* in_config, int in_tid,
    moodycamel::ConcurrentQueue<Event_data>& in_task_queue, Consumer& in_consumer,
    Table<complex_float>& in_dl_ifft_buffer, char* in_dl_socket_buffer,
    Stats* in_stats_manager)
    : Doer(in_config, in_tid, in_task_queue, in_consumer)
    , dl_ifft_buffer_(in_dl_ifft_buffer)
    , dl_socket_buffer_(in_dl_socket_buffer)
    , task_duration(&in_stats_manager->ifft_stats_worker.task_duration)
    , task_count(in_stats_manager->ifft_stats_worker.task_count)
{
    (void)DftiCreateDescriptor(&mkl_handle, DFTI_SINGLE, DFTI_COMPLEX,
        1, config_->OFDM_CA_NUM);
    (void)DftiCommitDescriptor(mkl_handle);
}

DoIFFT::~DoIFFT()
{
    DftiFreeDescriptor(&mkl_handle);
}

void DoIFFT::launch(int offset)
{
#if DEBUG_UPDATE_STATS
    double start_time = get_time();
#endif
    int BS_ANT_NUM = config_->BS_ANT_NUM;
    int TX_PREFIX_LEN = config_->TX_PREFIX_LEN;
    int OFDM_CA_NUM = config_->OFDM_CA_NUM;
    int CP_LEN = config_->CP_LEN;
    int packet_header_offset = config_->packet_header_offset;
    int data_subframe_num_perframe = config_->data_symbol_num_perframe;

    int ant_id = offset % BS_ANT_NUM;
#if DEBUG_PRINT_IN_TASK
    int total_data_subframe_id = offset / BS_ANT_NUM;
    int frame_id = total_data_subframe_id / data_subframe_num_perframe;
    int current_data_subframe_id = total_data_subframe_id % data_subframe_num_perframe;
    printf("In doIFFT thread %d: frame: %d, subframe: %d, antenna: %d\n", tid, frame_id, current_data_subframe_id, ant_id);
#endif

    // cout << "In ifft: frame: "<< frame_id<<", subframe: "<< current_data_subframe_id<<", ant: " << ant_id << ", input data: ";
    // for (int j = 0; j <OFDM_CA_NUM; j++) {
    //     cout << dl_ifft_buffer_.IFFT_inputs[offset][j].real << "+" << dl_ifft_buffer_.IFFT_inputs[offset][j].imag << "j,   ";
    // }
    // cout<<"\n\n"<<endl;
    // mufft_execute_plan_1d(muplans_ifft_[tid], dl_ifft_buffer_.IFFT_outputs[offset],
    //     dl_ifft_buffer_.IFFT_inputs[offset]);

    DftiComputeBackward(mkl_handle, dl_ifft_buffer_[offset]);

    // cout << "In ifft: frame: "<< frame_id<<", subframe: "<< current_data_subframe_id<<", ant: " << ant_id <<", offset: "<<offset <<", output data: ";
    // for (int j = 0; j <OFDM_CA_NUM; j++) {
    //     cout << dl_ifft_buffer_.IFFT_outputs[offset][j].real << "+" << dl_ifft_buffer_.IFFT_outputs[offset][j].imag << "j,   ";
    // }
    // cout<<"\n\n"<<endl;

    // calculate data for downlink socket buffer
    float* ifft_output_ptr = (float*)(&dl_ifft_buffer_[offset][0]);
    int socket_subframe_offset = offset % (SOCKET_BUFFER_FRAME_NUM * data_subframe_num_perframe * BS_ANT_NUM);
    int packet_length = config_->packet_length;
    char* socket_ptr = &dl_socket_buffer_[socket_subframe_offset * packet_length];
    //int socket_offset = sizeof(int) * 16 + ant_id * packet_length;
    socket_ptr += packet_header_offset + ant_id * packet_length + TX_PREFIX_LEN * sizeof(short) * 2;

    // for (int sc_id = 0; sc_id < OFDM_CA_NUM; sc_id++) {
    //     float *shifted_input_ptr = (float *)(ifft_output_ptr + 2 * sc_id);
    //     // ifft scaled results by 2048, 16 = 2^15/2048
    //     *((short *)socket_ptr + 2 * sc_id ) = (short)(*shifted_input_ptr * 16);
    //     *((short *)socket_ptr + 2 * sc_id + 1 ) = (short)(*(shifted_input_ptr+1) * 16);
    // }

    for (int sc_id = 0; sc_id < OFDM_CA_NUM; sc_id += 8) {
        //__m256 scale_factor = _mm256_set1_ps(16);
        __m256 scale_factor = _mm256_set1_ps(32768.0 / OFDM_CA_NUM);
        __m256 ifft1 = _mm256_load_ps(ifft_output_ptr + 2 * sc_id);
        __m256 ifft2 = _mm256_load_ps(ifft_output_ptr + 2 * sc_id + 8);
        __m256 scaled_ifft1 = _mm256_mul_ps(ifft1, scale_factor);
        __m256 scaled_ifft2 = _mm256_mul_ps(ifft2, scale_factor);
        __m256i integer1 = _mm256_cvtps_epi32(scaled_ifft1);
        __m256i integer2 = _mm256_cvtps_epi32(scaled_ifft2);
        integer1 = _mm256_packs_epi32(integer1, integer2);
        integer1 = _mm256_permute4x64_epi64(integer1, 0xD8);
        //_mm256_stream_si256((__m256i*)(socket_ptr + sc_id * 4), integer1);
        _mm256_stream_si256((__m256i*)(socket_ptr + (sc_id + CP_LEN) * 4), integer1);
        if (sc_id >= OFDM_CA_NUM - CP_LEN) // add CP
            _mm256_stream_si256((__m256i*)(socket_ptr + (sc_id + CP_LEN - OFDM_CA_NUM) * 4), integer1);
    }

    // cout << "In ifft: frame: "<< frame_id<<", subframe: "<< current_data_subframe_id<<", ant: " << ant_id << ", data: ";
    // for (int j = 0; j <OFDM_CA_NUM; j++) {
    //     int socket_offset = sizeof(int) * 16 + ant_id * packetReceiver::packet_length;
    //     cout <<*((short *)(socket_ptr + socket_offset) + 2 * j)  << "+j"<<*((short *)(socket_ptr + socket_offset) + 2 * j + 1 )<<",   ";
    // }
    // cout<<"\n\n"<<endl;

#if DEBUG_UPDATE_STATS
    task_count[tid * 16] = task_count[tid * 16] + 1;
    (*task_duration)[tid * 8][0] += get_time() - start_time;
#endif

    // inform main thread
    Event_data ifft_finish_event;
    ifft_finish_event.event_type = EVENT_IFFT;
    ifft_finish_event.data = offset;
    consumer_.handle(ifft_finish_event);
}
