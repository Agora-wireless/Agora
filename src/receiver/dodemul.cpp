/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 * 
 */
#include "dodemul.hpp"

using namespace arma;
DoDemul::DoDemul(int in_tid, int in_demul_block_size, int in_transpose_block_size,
        moodycamel::ConcurrentQueue<Event_data> *in_complete_task_queue, moodycamel::ProducerToken *in_task_ptok,
        complex_float **in_data_buffer, complex_float **in_precoder_buffer, complex_float **in_equal_buffer, uint8_t **in_demul_hard_buffer,
        double **in_Demul_task_duration, int *in_Demul_task_count)
{
    tid = in_tid;
    demul_block_size = in_demul_block_size;
    transpose_block_size = in_transpose_block_size;
    complete_task_queue_ = in_complete_task_queue;
    task_ptok = in_task_ptok;

    data_buffer_ = in_data_buffer;
    precoder_buffer_ = in_precoder_buffer;
    equal_buffer_ = in_equal_buffer;
    demul_hard_buffer_ = in_demul_hard_buffer;


    Demul_task_duration = in_Demul_task_duration;
    Demul_task_count = in_Demul_task_count;

    spm_buffer = (complex_float *)aligned_alloc(64, 8 * BS_ANT_NUM * sizeof(complex_float));
    equaled_buffer_temp = (complex_float *)aligned_alloc(64, demul_block_size * UE_NUM * sizeof(complex_float));

}

DoDemul::~DoDemul() 
{
    free(spm_buffer);
    free(equaled_buffer_temp);
}



void DoDemul::Demul(int offset)
{

    int frame_id, total_data_subframe_id, current_data_subframe_id, sc_id;
    interpreteOffset3d(OFDM_DATA_NUM, offset, &frame_id, &total_data_subframe_id, &current_data_subframe_id, &sc_id);
    int subframe_offset = subframe_num_perframe * frame_id + UE_NUM + current_data_subframe_id;

#if DEBUG_UPDATE_STATS    
    double start_time = get_time();
#endif
    

#if DEBUG_PRINT_IN_TASK
        printf("In doDemul thread %d: frame: %d, subframe: %d, subcarrier: %d \n", tid, frame_id, current_data_subframe_id,sc_id);
#endif

    __m256i index = _mm256_setr_epi32(0, 1, transpose_block_size * 2, transpose_block_size * 2 + 1, transpose_block_size * 4, transpose_block_size * 4 + 1, transpose_block_size * 6, transpose_block_size * 6 + 1);
    int gather_step_size = 8 * transpose_block_size;

    int mat_elem = UE_NUM * BS_ANT_NUM;
    int cache_line_num = mat_elem / 8;
    int ue_data_cache_line_num = UE_NUM/8;
    int max_sc_ite;
    if (sc_id + demul_block_size <= OFDM_DATA_NUM) 
        max_sc_ite = demul_block_size;
    else
        max_sc_ite = OFDM_DATA_NUM - sc_id;
    /* i = 0, 1, ..., 32/8
     * iterate through cache lines (each cache line has 8 subcarriers) */
    for (int i = 0; i < max_sc_ite/8; i++) {        
        /* for a cache line size of 64 bytes, each read load 8 subcarriers
         * use spatial locality to reduce latency */
    #if DEBUG_UPDATE_STATS_DETAILED    
        double start_time1 = get_time();
    #endif

        // for (int j = 0; j < 1; j++) {
        //     int cur_sc_id = i * 8 + j + sc_id;
        //     int precoder_offset = frame_id * OFDM_DATA_NUM + cur_sc_id;
        //     cx_float* precoder_ptr = (cx_float *)precoder_buffer_.precoder[precoder_offset];
        //     for (int line_idx = 0; line_idx < cache_line_num; line_idx ++) {
        //         _mm_prefetch((char *)(precoder_ptr + 8 * line_idx), _MM_HINT_T2);
        //     }
        // }


        int cur_block_id = (sc_id + i * 8) / transpose_block_size;
        int sc_inblock_idx = (i * 8) % transpose_block_size;
        int cur_sc_offset = cur_block_id * transpose_block_size * BS_ANT_NUM + sc_inblock_idx;
        float *tar_data_ptr = (float *)spm_buffer;        
        float *src_data_ptr = (float *)data_buffer_[total_data_subframe_id] + cur_sc_offset * 2;

        /* gather data for all antennas and 8 subcarriers in the same cache line */
        for (int ant_idx = 0; ant_idx < BS_ANT_NUM; ant_idx += 4) {
            /* 1 subcarrier and 4 ants per iteration */
            for (int j = 0; j < 8; j++) {
                __m256 data_rx = _mm256_i32gather_ps(src_data_ptr + j * 2, index, 4);
                _mm256_store_ps(tar_data_ptr + j * BS_ANT_NUM * 2, data_rx);
                // printf("Frame %d, sc: %d, UE %d, ant %d, data: %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n", frame_id, sc_id, ue_idx, ant_idx, *((float *)tar_csi_ptr), *((float *)tar_csi_ptr+1), 
                //         *((float *)tar_csi_ptr+2), *((float *)tar_csi_ptr+3),  *((float *)tar_csi_ptr+4), *((float *)tar_csi_ptr+5));   
            }
            src_data_ptr += gather_step_size;
            tar_data_ptr += 8;
        }
    #if DEBUG_UPDATE_STATS_DETAILED   
        double duration1 = get_time() - start_time1;
        Demul_task_duration[tid * 8][1] += duration1;
    #endif

        // Demul_task_duration_part1[tid] += get_time() - start_time1;
        
        /* perform computation for 8 subcarriers */
        for (int j = 0; j < 8; j++) {
            /* create input data matrix */
            cx_float* data_ptr = (cx_float *)(spm_buffer + j * BS_ANT_NUM);
            cx_fmat mat_data(data_ptr, BS_ANT_NUM, 1, false);

            /* create input precoder matrix */
            int cur_sc_id = i * 8 + j + sc_id;
            int precoder_offset = frame_id * OFDM_DATA_NUM + cur_sc_id;
            cx_float* precoder_ptr = (cx_float *)precoder_buffer_[precoder_offset];
            // for (int line_idx = 0; line_idx < cache_line_num; line_idx ++) {
            //     _mm_prefetch((char *)(precoder_ptr + 8 * line_idx), _MM_HINT_NTA);
            // }
            cx_fmat mat_precoder(precoder_ptr, UE_NUM, BS_ANT_NUM, false);


            // cout<<"Precoder: "<< mat_precoder<<endl;

            /* create output matrix for equalization */ 
#if EXPORT_CONSTELLATION
            cx_float* equal_ptr = (cx_float *)(&equal_buffer_[total_data_subframe_id][cur_sc_id * UE_NUM]);
#else
            cx_float* equal_ptr = (cx_float *)(&equaled_buffer_temp[(cur_sc_id-sc_id) * UE_NUM]);
#endif
            cx_fmat mat_equaled(equal_ptr, UE_NUM, 1, false);


#if DEBUG_UPDATE_STATS_DETAILED    
    double start_time2 = get_time();
#endif      
            /* perform computation for equalization */
            mat_equaled = mat_precoder * mat_data;
            // cout<<mat_equaled.st()<<endl;


#if DEBUG_UPDATE_STATS_DETAILED   
    double start_time3 = get_time(); 
    double duration2 = get_time() - start_time2;   
    Demul_task_duration[tid * 8][2] += duration2;
#endif
            // printf("In doDemul thread %d: frame: %d, subframe: %d, subcarrier: %d, sc_id: %d \n", tid, frame_id, current_data_subframe_id,cur_sc_id, sc_id);
            // cout << "Equaled data: "<<mat_equaled.st()<<endl;
            
#if !ENABLE_DECODE 
            /* decode with hard decision */
            uint8_t *demul_ptr = (&demul_hard_buffer_[total_data_subframe_id][cur_sc_id * UE_NUM]);
            // if (i * 8 + j < max_sc_ite -1)
            //     _mm_prefetch((char *)(demul_ptr+UE_NUM), _MM_HINT_T1);
            demod_16qam_loop((float *)equal_ptr, demul_ptr, UE_NUM);
            // demod_16qam_loop((float *)equal_ptr, (uint8_t *)data_ptr, UE_NUM);
#if DEBUG_UPDATE_STATS_DETAILED   
    double duration3 = get_time() - start_time3;   
    Demul_task_duration[tid * 8][3] += duration3;
#endif

             // printf("In doDemul thread %d: frame: %d, subframe: %d, subcarrier: %d, sc_id: %d \n", tid, frame_id, current_data_subframe_id,cur_sc_id, sc_id);
             // cout<< "Demuled data: ";
             // for (int ue_idx = 0; ue_idx < UE_NUM; ue_idx++) {
             //     // cout<<demul_hard_buffer_[total_data_subframe_id][cur_sc_id * UE_NUM+ue_idx]<<" "<<endl;
             //     cout<<+*(demul_ptr+ue_idx)<<"  ";
             // }
             // cout<<endl; 
#endif      
        #if DEBUG_UPDATE_STATS    
            Demul_task_count[tid * 16] = Demul_task_count[tid * 16] + 1;
        #endif
        }

        // if (frame_id==0) 
        // {
        //     FILE* fp_debug = fopen("tmpresult.txt", "a");
        //     if (fp_debug==NULL) {
        //         printf("open file faild");
        //         std::cerr << "Error: " << strerror(errno) << std::endl;
        //         exit(0);
        //     }
        //     for(int ii = 0; ii < UE_NUM; ii++)
        //     {
        //         // printf("User %d: %d, ", ii,demul_ptr2(ii));
        //         fprintf(fp_debug, "%d\n", mat_demuled2(ii));
        //     }
        //     // printf("\n");
        //     // fwrite(mat_demuled2.memptr(), sizeof(int),sizeof(mat_demuled), fp_debug);
        //     fclose(fp_debug);
        // }
    }



//     for (int i = 0; i < max_sc_ite; i++) {        
//     #if DEBUG_UPDATE_STATS_DETAILED    
//         double start_time1 = get_time();
//     #endif
//         int cur_block_id = (sc_id + i) / transpose_block_size;
//         int sc_inblock_idx = i % transpose_block_size;
//         int cur_sc_offset = cur_block_id * transpose_block_size * BS_ANT_NUM + sc_inblock_idx;
//         float *tar_data_ptr = (float *)spm_buffer[tid];        
//         float *src_data_ptr = (float *)data_buffer_.data[total_data_subframe_id] + cur_sc_offset * 2;
//         for (int c2 = 0; c2 < BS_ANT_NUM / 4; c2++) {
//             __m256 data_rx = _mm256_i32gather_ps(src_data_ptr, index, 4);
//             _mm256_store_ps(tar_data_ptr, data_rx);
//             // printf("Frame %d, sc: %d, UE %d, ant %d, data: %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n", frame_id, sc_id, ue_idx, ant_idx, *((float *)tar_csi_ptr), *((float *)tar_csi_ptr+1), 
//                 //         *((float *)tar_csi_ptr+2), *((float *)tar_csi_ptr+3),  *((float *)tar_csi_ptr+4), *((float *)tar_csi_ptr+5));           
//             src_data_ptr += gather_step_size;
//             tar_data_ptr += 8;
//         }
       
//     #if DEBUG_UPDATE_STATS_DETAILED   
//         double start_time2 = get_time();
//         double duration1 = start_time2 - start_time1;
//         Demul_task_duration[tid * 8][1] += duration1;
//     #endif

//         // Demul_task_duration_part1[tid] += get_time() - start_time1;
        
//         /* perform computation for 8 subcarriers */
        
// // #if DEBUG_UPDATE_STATS_DETAILED    
// //     double start_time2 = get_time();
// // #endif

//         /* create input data matrix */
//         cx_float* data_ptr = (cx_float *)(spm_buffer[tid]);
//         cx_fmat mat_data(data_ptr, BS_ANT_NUM, 1, false);

//         /* create input precoder matrix */
//         int cur_sc_id = i + sc_id;
//         int precoder_offset = frame_id * OFDM_DATA_NUM + cur_sc_id;
//         cx_float* precoder_ptr = (cx_float *)precoder_buffer_.precoder[precoder_offset];
//         // for (int line_idx = 0; line_idx < cache_line_num; line_idx ++) {
//         //     _mm_prefetch((char *)(precoder_ptr + 8 * line_idx), _MM_HINT_NTA);
//         // }
//         cx_fmat mat_precoder(precoder_ptr, UE_NUM, BS_ANT_NUM, false);
//         // cout<<"Precoder: "<< mat_precoder<<endl;

//         /* create output matrix for equalization */ 
// #if EXPORT_CONSTELLATION
//         cx_float* equal_ptr = (cx_float *)(&equal_buffer_.data[total_data_subframe_id][cur_sc_id * UE_NUM]);
// #else
//         cx_float* equal_ptr = (cx_float *)(&equaled_buffer_temp[tid][(cur_sc_id-sc_id) * UE_NUM]);
// #endif
//         cx_fmat mat_equaled(equal_ptr, UE_NUM, 1, false);
      
//         /* perform computation for equalization */
//         mat_equaled = mat_precoder * mat_data;

// #if DEBUG_UPDATE_STATS_DETAILED   
//     double start_time3 = get_time(); 
//     double duration2 = start_time3 - start_time2;   
//     Demul_task_duration[tid * 8][2] += duration2;
// #endif
// #if !ENABLE_DECODE 
//         /* decode with hard decision */
//         uint8_t *demul_ptr = (&demul_hard_buffer_[total_data_subframe_id][cur_sc_id * UE_NUM]);
//         demod_16qam_loop((float *)equal_ptr, demul_ptr, UE_NUM);
// #if DEBUG_UPDATE_STATS_DETAILED   
//     double duration3 = get_time() - start_time3;   
//     Demul_task_duration[tid * 8][3] += duration3;
// #endif
//         // printf("In doDemul thread %d: frame: %d, subframe: %d, subcarrier: %d, sc_id: %d \n", tid, frame_id, current_data_subframe_id,cur_sc_id, sc_id);
//         // cout<< "Demuled data: ";
//         // for (int ue_idx = 0; ue_idx < UE_NUM; ue_idx++) {
//         //     // cout<<demul_hard_buffer_[total_data_subframe_id][cur_sc_id * UE_NUM+ue_idx]<<" "<<endl;
//         //     cout<<+*(demul_ptr+ue_idx)<<"  ";
//         // }
//         // cout<<endl; 
// #endif      
//         #if DEBUG_UPDATE_STATS    
//             Demul_task_count[tid * 16] = Demul_task_count[tid * 16] + 1;
//         #endif
        
//     }


    // inform main thread
#if DEBUG_UPDATE_STATS   
    double duration = get_time() - start_time;
    Demul_task_duration[tid * 8][0] += duration;
    if (duration > 500) {
            printf("Thread %d Demul takes %.2f\n", tid, duration);
    }
#endif
    Event_data demul_finish_event;
    demul_finish_event.event_type = EVENT_DEMUL;
    demul_finish_event.data = offset;
    

    if ( !complete_task_queue_->enqueue(*task_ptok, demul_finish_event ) ) {
        printf("Demuliplexing message enqueue failed\n");
        exit(0);
    }  

      
}

void DoDemul::DemulSingleSC(int offset)
{
    double start_time = get_time();
    int frame_id, total_data_subframe_id, current_data_subframe_id, sc_id;
    interpreteOffset3d(OFDM_DATA_NUM, offset, &frame_id, &total_data_subframe_id, &current_data_subframe_id, &sc_id);
    int subframe_offset = subframe_num_perframe * frame_id + UE_NUM + current_data_subframe_id;
    
    int gather_step_size = 8 * transpose_block_size;

#if DEBUG_PRINT_IN_TASK
        printf("In doDemul thread %d: frame: %d, subframe: %d, subcarrier: %d \n", tid, frame_id, current_data_subframe_id,sc_id);
#endif

    __m256i index = _mm256_setr_epi32(0, 1, transpose_block_size * 2, transpose_block_size * 2 + 1, transpose_block_size * 4, transpose_block_size * 4 + 1, transpose_block_size * 6, transpose_block_size * 6 + 1);



    int cur_block_id = sc_id / transpose_block_size;
    int sc_inblock_idx = sc_id % transpose_block_size;
    int cur_sc_offset = cur_block_id * transpose_block_size * BS_ANT_NUM + sc_inblock_idx;
    float *tar_data_ptr = (float *)spm_buffer;        
    float *src_data_ptr = (float *)data_buffer_[total_data_subframe_id] + cur_sc_offset * 2;
    for (int ant_idx = 0; ant_idx < BS_ANT_NUM; ant_idx += 4) {
        __m256 data_rx = _mm256_i32gather_ps(src_data_ptr, index, 4);
        _mm256_store_ps(tar_data_ptr, data_rx);     
        src_data_ptr += gather_step_size;
        tar_data_ptr += 8;
    }

    // mat_data size: BS_ANT_NUM \times 1
    cx_float* data_ptr = (cx_float *)(spm_buffer);
    cx_fmat mat_data(data_ptr, BS_ANT_NUM, 1, false);
    // cout<< "Raw data: " << mat_data.st()<<endl;

    // mat_precoder size: UE_NUM \times BS_ANT_NUM
    int precoder_offset = frame_id * OFDM_DATA_NUM + sc_id;
    cx_float* precoder_ptr = (cx_float *)precoder_buffer_[precoder_offset];


    cx_fmat mat_precoder(precoder_ptr, UE_NUM, BS_ANT_NUM, false);
    // cout<<"Precoder: "<< mat_precoder<<endl;

    // mat_demuled size: UE_NUM \times 1
    cx_float* equal_ptr = (cx_float *)(&equal_buffer_[total_data_subframe_id][sc_id * UE_NUM]);
    cx_fmat mat_equaled(equal_ptr, UE_NUM, 1, false);

    // Demodulation
    // sword* demul_ptr = (sword *)(&demul_hard_buffer_.data[total_data_subframe_id][sc_id * UE_NUM]);
    // imat mat_demuled(demul_ptr, UE_NUM, 1, false);
    uint8_t *demul_ptr = (&demul_hard_buffer_[total_data_subframe_id][sc_id * UE_NUM]);
    
    // Equalization
    mat_equaled = mat_precoder * mat_data;
    // cout << "Equaled data: "<<mat_equaled.st()<<endl;

    

    // Hard decision
    demod_16qam_loop((float *)equal_ptr, demul_ptr, UE_NUM);
    printf("In doDemul thread %d: frame: %d, subframe: %d, subcarrier: %d \n", tid, frame_id, current_data_subframe_id,sc_id);
    cout<< "Demuled data: ";
    for (int ue_idx = 0; ue_idx < UE_NUM; ue_idx++) {
         cout<<*(demul_ptr+ue_idx)<<"  ";
     }
     cout<<endl;

    // inform main thread
    double duration3 = get_time() - start_time;
    Demul_task_duration[tid][1] += duration3;
    Event_data demul_finish_event;
    demul_finish_event.event_type = EVENT_DEMUL;
    demul_finish_event.data = offset;
    Demul_task_count[tid] = Demul_task_count[tid]+1;

    if ( !complete_task_queue_->enqueue(*task_ptok, demul_finish_event ) ) {
        printf("Demuliplexing message enqueue failed\n");
        exit(0);
    }  
    double duration = get_time() - start_time;
    Demul_task_duration[tid][0] += duration;
      
}
