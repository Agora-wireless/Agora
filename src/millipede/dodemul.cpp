/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#include "dodemul.hpp"
#include "concurrent_queue_wrapper.hpp"

using namespace arma;
DoDemul::DoDemul(Config* in_config, int in_tid,
    moodycamel::ConcurrentQueue<Event_data>& in_task_queue,
    moodycamel::ConcurrentQueue<Event_data>& complete_task_queue,
    moodycamel::ProducerToken* worker_producer_token,
    Table<complex_float>& in_data_buffer,
    Table<complex_float>& in_precoder_buffer,
    Table<complex_float>& in_equal_buffer, Table<uint8_t>& in_demod_hard_buffer,
    Table<int8_t>& in_demod_soft_buffer, Stats* in_stats_manager)
    : Doer(in_config, in_tid, in_task_queue, complete_task_queue,
          worker_producer_token)
    , data_buffer_(in_data_buffer)
    , precoder_buffer_(in_precoder_buffer)
    , equal_buffer_(in_equal_buffer)
    , demod_hard_buffer_(in_demod_hard_buffer)
    , demod_soft_buffer_(in_demod_soft_buffer)
    , Demul_task_duration(in_stats_manager->demul_stats_worker.task_duration)
{
    Demul_task_count = in_stats_manager->demul_stats_worker.task_count;

    alloc_buffer_1d(&spm_buffer, 8 * cfg->BS_ANT_NUM, 64, 0);
    alloc_buffer_1d(
        &equaled_buffer_temp, cfg->demul_block_size * cfg->UE_NUM, 64, 0);
    alloc_buffer_1d(&equaled_buffer_temp_transposed,
        cfg->demul_block_size * cfg->UE_NUM, 64, 0);

    ue_num_simd256 = cfg->UE_NUM / double_num_in_simd256;
}

DoDemul::~DoDemul()
{
    free_buffer_1d(&spm_buffer);
    free_buffer_1d(&equaled_buffer_temp);
    free_buffer_1d(&equaled_buffer_temp_transposed);
}

Event_data DoDemul::launch(int offset)
{
    int sc_id = offset % cfg->demul_block_num * cfg->demul_block_size;
    int total_data_subframe_id = offset / cfg->demul_block_num;
    int data_subframe_num_perframe = cfg->ul_data_symbol_num_perframe;
    int frame_id = total_data_subframe_id / data_subframe_num_perframe;

#if DEBUG_UPDATE_STATS
    double start_time = get_time();
#endif

#if DEBUG_PRINT_IN_TASK
    int current_data_subframe_id
        = total_data_subframe_id % data_subframe_num_perframe;
    printf("In doDemul thread %d: frame: %d, subframe: %d, subcarrier: %d \n",
        tid, frame_id, current_data_subframe_id, sc_id);
#endif

    int transpose_block_size = cfg->transpose_block_size;
    int gather_step_size = 8 * transpose_block_size;
    __m256i index = _mm256_setr_epi32(0, 1, transpose_block_size * 2,
        transpose_block_size * 2 + 1, transpose_block_size * 4,
        transpose_block_size * 4 + 1, transpose_block_size * 6,
        transpose_block_size * 6 + 1);

    int max_sc_ite
        = std::min(cfg->demul_block_size, cfg->OFDM_DATA_NUM - sc_id);
    /* Iterate through cache lines (each cache line has 8 subcarriers) */
    for (int i = 0; i < max_sc_ite / 8; i++) {
#if DEBUG_UPDATE_STATS_DETAILED
        double start_time1 = get_time();
#endif
        /* gather data for all antennas and 8 subcarriers in the same cache line
         * 1 subcarrier and 4 ants per iteration */
        int cur_block_id = (sc_id + i * 8) / transpose_block_size;
        int sc_inblock_idx = (sc_id + i * 8) % transpose_block_size;
        int cur_sc_offset
            = cur_block_id * transpose_block_size * cfg->BS_ANT_NUM
            + sc_inblock_idx;
        float* src_data_ptr
            = (float*)data_buffer_[total_data_subframe_id] + cur_sc_offset * 2;
        float* tar_data_ptr = (float*)spm_buffer;
        for (size_t ant_idx = 0; ant_idx < cfg->BS_ANT_NUM; ant_idx += 4) {
            for (int j = 0; j < 8; j++) {
                __m256 data_rx
                    = _mm256_i32gather_ps(src_data_ptr + j * 2, index, 4);
                _mm256_store_ps(
                    tar_data_ptr + j * cfg->BS_ANT_NUM * 2, data_rx);
            }
            src_data_ptr += gather_step_size;
            tar_data_ptr += 8;
        }
#if DEBUG_UPDATE_STATS_DETAILED
        double duration1 = get_time() - start_time1;
        Demul_task_duration[tid * 8][1] += duration1;
#endif

        /* computation for 8 subcarriers */
        for (int j = 0; j < 8; j++) {
            /* create input data matrix */
            cx_float* data_ptr = (cx_float*)(spm_buffer + j * cfg->BS_ANT_NUM);
            cx_fmat mat_data(data_ptr, cfg->BS_ANT_NUM, 1, false);

            /* create input precoder matrix */
            int cur_sc_id = i * 8 + j + sc_id;
            int precoder_offset = frame_id * cfg->OFDM_DATA_NUM + cur_sc_id;
            if (cfg->freq_orthogonal_pilot)
                precoder_offset = precoder_offset - cur_sc_id % cfg->UE_NUM;
            cx_float* precoder_ptr
                = (cx_float*)precoder_buffer_[precoder_offset];
            cx_fmat mat_precoder(
                precoder_ptr, cfg->UE_NUM, cfg->BS_ANT_NUM, false);
            // cout<<"Precoder: "<< mat_precoder<<endl;
            // cout << "Raw data: " << mat_data.st() <<endl;

#if EXPORT_CONSTELLATION
            cx_float* equal_ptr
                = (cx_float*)(&equal_buffer_[total_data_subframe_id]
                                            [cur_sc_id * UE_NUM]);
#else
            cx_float* equal_ptr
                = (cx_float*)(&equaled_buffer_temp[(cur_sc_id - sc_id)
                    * cfg->UE_NUM]);
#endif
            /* create output matrix for equalization */
            cx_fmat mat_equaled(equal_ptr, cfg->UE_NUM, 1, false);

#if DEBUG_UPDATE_STATS_DETAILED
            double start_time2 = get_time();
#endif
            /* perform computation for equalization */
            mat_equaled = mat_precoder * mat_data;

#if DEBUG_UPDATE_STATS_DETAILED
            double start_time3 = get_time();
            double duration2 = get_time() - start_time2;
            Demul_task_duration[tid * 8][2] += duration2;
#endif
            // cout << "Equaled data sc "<<cur_sc_id<<": "<<mat_equaled.st();

#ifndef USE_LDPC
            /* decode with hard decision */
            uint8_t* demul_ptr = (&demod_hard_buffer_[total_data_subframe_id]
                                                     [cur_sc_id * cfg->UE_NUM]);
            demod_16qam_hard_avx2((float*)equal_ptr, demul_ptr, cfg->UE_NUM);
            // cout<< "Demuled data:";
            // for (int ue_idx = 0; ue_idx < cfg->UE_NUM; ue_idx++)
            //     cout<<+*(demul_ptr+ue_idx)<<" ";
            // cout<<endl;
            // cout<<endl;
#endif

#if DEBUG_UPDATE_STATS_DETAILED
            double duration3 = get_time() - start_time3;
            Demul_task_duration[tid * 8][3] += duration3;
#endif

#if DEBUG_UPDATE_STATS
            Demul_task_count[tid * 16] = Demul_task_count[tid * 16] + 1;
#endif
        }
    }

#ifdef USE_LDPC
    __m256i index2 = _mm256_setr_epi32(0, 1, cfg->UE_NUM * 2,
        cfg->UE_NUM * 2 + 1, cfg->UE_NUM * 4, cfg->UE_NUM * 4 + 1,
        cfg->UE_NUM * 6, cfg->UE_NUM * 6 + 1);
    float* equal_T_ptr = (float*)(equaled_buffer_temp_transposed);
    for (int i = 0; i < cfg->UE_NUM; i++) {
        float* equal_ptr = (float*)(equaled_buffer_temp + i);
        int8_t* demul_ptr
            = (&demod_soft_buffer_[total_data_subframe_id]
                                  [(cfg->OFDM_DATA_NUM * i + sc_id)
                                      * cfg->mod_type]);
        for (int j = 0; j < max_sc_ite / double_num_in_simd256; j++) {
            __m256 equal_T_temp = _mm256_i32gather_ps(equal_ptr, index2, 4);
            _mm256_store_ps(equal_T_ptr, equal_T_temp);
            equal_T_ptr += 8;
            equal_ptr += cfg->UE_NUM * double_num_in_simd256 * 2;
        }
        int num_sc_avx2 = (max_sc_ite / 16) * 16;
        int rest = max_sc_ite % 16;
        // demod_16qam_soft_sse((equal_T_ptr - max_sc_ite * 2), demul_ptr,
        // max_sc_ite);
        demod_16qam_soft_avx2(
            (equal_T_ptr - max_sc_ite * 2), demul_ptr, num_sc_avx2);
        if (rest > 0)
            demod_16qam_soft_sse(
                (equal_T_ptr - max_sc_ite * 2 + num_sc_avx2 * 2),
                demul_ptr + cfg->mod_type * num_sc_avx2, rest);
        // printf("In doDemul thread %d: frame: %d, subframe: %d, sc_id: %d \n",
        // tid, frame_id, current_data_subframe_id, sc_id); cout<< "Demuled
        // data: \n"; cout<<"UE "<<i<<": "; for (int k = 0; k < max_sc_ite *
        // cfg->mod_order; k++)
        //     printf("%i ", demul_ptr[k]);
        // cout<<endl;
    }
#endif

#if DEBUG_UPDATE_STATS
    double duration = get_time() - start_time;
    Demul_task_duration[tid * 8][0] += duration;
    // if (duration > 500)
    //     printf("Thread %d Demul takes %.2f\n", tid, duration);
#endif

    /* Inform main thread */
    Event_data demul_finish_event(EventType::kDemul, offset);
    // consumer_.handle(demul_finish_event);
    return demul_finish_event;
}

Event_data DoDemul::DemulSingleSC(int offset)
{
    double start_time = get_time();

    int sc_id = offset % cfg->demul_block_num * cfg->demul_block_size;
    int total_data_subframe_id = offset / cfg->demul_block_num;
    int data_subframe_num_perframe = cfg->data_symbol_num_perframe;
    int frame_id = total_data_subframe_id / data_subframe_num_perframe;
    int current_data_subframe_id
        = total_data_subframe_id % data_subframe_num_perframe;
#if DEBUG_PRINT_IN_TASK
    printf("In doDemul thread %d: frame: %d, subframe: %d, subcarrier: %d \n",
        tid, frame_id, current_data_subframe_id, sc_id);
#endif
    // int subframe_offset = subframe_num_perframe * frame_id + UE_NUM +
    // current_data_subframe_id;

    int transpose_block_size = cfg->transpose_block_size;
    int gather_step_size = 8 * transpose_block_size;

    __m256i index = _mm256_setr_epi32(0, 1, transpose_block_size * 2,
        transpose_block_size * 2 + 1, transpose_block_size * 4,
        transpose_block_size * 4 + 1, transpose_block_size * 6,
        transpose_block_size * 6 + 1);

    int cur_block_id = sc_id / transpose_block_size;
    int sc_inblock_idx = sc_id % transpose_block_size;
    int cur_sc_offset = cur_block_id * transpose_block_size * cfg->BS_ANT_NUM
        + sc_inblock_idx;
    float* tar_data_ptr = (float*)spm_buffer;
    float* src_data_ptr
        = (float*)data_buffer_[total_data_subframe_id] + cur_sc_offset * 2;
    for (size_t ant_idx = 0; ant_idx < cfg->BS_ANT_NUM; ant_idx += 4) {
        __m256 data_rx = _mm256_i32gather_ps(src_data_ptr, index, 4);
        _mm256_store_ps(tar_data_ptr, data_rx);
        src_data_ptr += gather_step_size;
        tar_data_ptr += 8;
    }

    // mat_data size: cfg->BS_ANT_NUM \times 1
    cx_float* data_ptr = (cx_float*)(spm_buffer);
    cx_fmat mat_data(data_ptr, cfg->BS_ANT_NUM, 1, false);
    // cout<< "Raw data: " << mat_data.st()<<endl;

    // mat_precoder size: cfg->UE_NUM \times cfg->BS_ANT_NUM
    int precoder_offset = frame_id * cfg->OFDM_DATA_NUM + sc_id;
    cx_float* precoder_ptr = (cx_float*)precoder_buffer_[precoder_offset];

    cx_fmat mat_precoder(precoder_ptr, cfg->UE_NUM, cfg->BS_ANT_NUM, false);
    // cout<<"Precoder: "<< mat_precoder<<endl;

    // mat_demuled size: cfg->UE_NUM \times 1
    cx_float* equal_ptr = (cx_float*)(&equal_buffer_[total_data_subframe_id]
                                                    [sc_id * cfg->UE_NUM]);
    cx_fmat mat_equaled(equal_ptr, cfg->UE_NUM, 1, false);

    // Demodulation
    uint8_t* demul_ptr
        = (&demod_hard_buffer_[total_data_subframe_id][sc_id * cfg->UE_NUM]);

    // Equalization
    mat_equaled = mat_precoder * mat_data;
    // cout << "Equaled data: "<<mat_equaled.st()<<endl;

    // Hard decision
    demod_16qam_hard_loop((float*)equal_ptr, demul_ptr, cfg->UE_NUM);
    printf("In doDemul thread %d: frame: %d, subframe: %d, subcarrier: %d \n",
        tid, frame_id, current_data_subframe_id, sc_id);
    cout << "Demuled data: ";
    for (size_t ue_idx = 0; ue_idx < cfg->UE_NUM; ue_idx++) {
        cout << *(demul_ptr + ue_idx) << "  ";
    }
    cout << endl;

    // inform main thread
    double duration3 = get_time() - start_time;
    Demul_task_duration[tid][1] += duration3;

    Demul_task_count[tid] = Demul_task_count[tid] + 1;
    double duration = get_time() - start_time;
    Demul_task_duration[tid][0] += duration;

    return Event_data(EventType::kDemul, offset);
}
