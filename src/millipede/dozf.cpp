/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#include "dozf.hpp"
#include "Consumer.hpp"
#include "doer.hpp"

using namespace arma;
DoZF::DoZF(Config* in_config, int in_tid,
    moodycamel::ConcurrentQueue<Event_data>& in_task_queue,
    Consumer& in_consumer, Table<complex_float>& in_csi_buffer,
    Table<complex_float>& in_recip_buffer,
    Table<complex_float>& in_ul_precoder_buffer,
    Table<complex_float>& in_dl_precoder_buffer, Stats* in_stats_manager)
    : Doer(in_config, in_tid, in_task_queue, in_consumer)
    , csi_buffer_(in_csi_buffer)
    , recip_buffer_(in_recip_buffer)
    , ul_precoder_buffer_(in_ul_precoder_buffer)
    , dl_precoder_buffer_(in_dl_precoder_buffer)
{
    ZF_task_duration = &in_stats_manager->zf_stats_worker.task_duration;
    ZF_task_count = in_stats_manager->zf_stats_worker.task_count;
    int BS_ANT_NUM = config_->BS_ANT_NUM;
    int UE_NUM = config_->UE_NUM;
    alloc_buffer_1d(&pred_csi_buffer, BS_ANT_NUM * UE_NUM, 64, 0);
    alloc_buffer_1d(&csi_gather_buffer, BS_ANT_NUM * UE_NUM, 64, 0);
}

DoZF::~DoZF()
{
    free_buffer_1d(&csi_gather_buffer);
    free_buffer_1d(&pred_csi_buffer);
}

Event_data DoZF::launch(int offset)
{
    if (config_->freq_orthogonal_pilot)
        ZF_freq_orthogonal(offset);
    else
        ZF_time_orthogonal(offset);
    // finish(offset);
    Event_data ZF_finish_event(EventType::kZF, offset);
    return ZF_finish_event;
}

void DoZF::finish(int offset)
{
    // inform main thread
    Event_data ZF_finish_event(EventType::kZF, offset);
    consumer_.handle(ZF_finish_event);
}

void DoZF::precoder(void* mat_input_ptr, int frame_id, int sc_id, int offset,
    bool downlink_mode)
{
    cx_fmat& mat_input = *(cx_fmat*)mat_input_ptr;
    int BS_ANT_NUM = config_->BS_ANT_NUM;
    int UE_NUM = config_->UE_NUM;
    cx_float* ptr_ul_out = (cx_float*)ul_precoder_buffer_[offset];
    cx_fmat mat_ul_precoder(ptr_ul_out, UE_NUM, BS_ANT_NUM, false);
    pinv(mat_ul_precoder, mat_input, 1e-1, "dc");
    if (downlink_mode) {
        cx_float* ptr_dl_out = (cx_float*)dl_precoder_buffer_[offset];
        cx_fmat mat_dl_precoder(ptr_dl_out, UE_NUM, BS_ANT_NUM, false);
        if (config_->recipCalEn) {
            cx_float* calib
                = (cx_float*)(&recip_buffer_[frame_id][sc_id * BS_ANT_NUM]);
            cx_fvec vec_calib(calib, BS_ANT_NUM, false);
            cx_fmat mat_calib(BS_ANT_NUM, BS_ANT_NUM);
            mat_calib = diagmat(vec_calib);
            mat_dl_precoder = inv(mat_calib) * mat_ul_precoder;
        } else
            mat_dl_precoder = mat_ul_precoder;
    }
}

void DoZF::ZF_time_orthogonal(int offset)
{
    int OFDM_DATA_NUM = config_->OFDM_DATA_NUM;
    int zf_block_size = config_->zf_block_size;

    int frame_id = offset / config_->zf_block_num;
    int sc_id = offset % config_->zf_block_num * zf_block_size;
#if DEBUG_PRINT_IN_TASK
    printf(
        "In doZF thread %d: frame: %d, subcarrier: %d\n", tid, frame_id, sc_id);
#endif
    int offset_in_buffer = frame_id * OFDM_DATA_NUM + sc_id;
    int max_sc_ite = std::min(zf_block_size, OFDM_DATA_NUM - sc_id);
    int BS_ANT_NUM = config_->BS_ANT_NUM;
    int UE_NUM = config_->UE_NUM;
    for (int i = 0; i < max_sc_ite; i++) {

#if DEBUG_UPDATE_STATS
        double start_time1 = get_time();
#endif
        int cur_sc_id = sc_id + i;
        int cur_offset = offset_in_buffer + i;
        int transpose_block_size = config_->transpose_block_size;
        __m256i index = _mm256_setr_epi32(0, 1, transpose_block_size * 2,
            transpose_block_size * 2 + 1, transpose_block_size * 4,
            transpose_block_size * 4 + 1, transpose_block_size * 6,
            transpose_block_size * 6 + 1);
        int transpose_block_id = cur_sc_id / transpose_block_size;
        int sc_inblock_idx = cur_sc_id % transpose_block_size;
        int offset_in_csi_buffer
            = transpose_block_id * BS_ANT_NUM * transpose_block_size
            + sc_inblock_idx;
        int subframe_offset = frame_id * UE_NUM;
        float* tar_csi_ptr = (float*)csi_gather_buffer;

        // printf("In doZF thread %d: frame: %d, subcarrier: %d\n", tid,
        // frame_id, sc_id); int mat_elem = UE_NUM * BS_ANT_NUM; int
        // cache_line_num = mat_elem / 8; for (int line_idx = 0; line_idx <
        // cache_line_num; line_idx ++) {
        //     _mm_prefetch((char *)precoder_buffer_.precoder[cur_offset + 8 *
        //     line_idx], _MM_HINT_ET1);
        //     // _mm_prefetch((char *)(tar_csi_ptr + 16 * line_idx),
        //     _MM_HINT_ET1);
        // }

        /* gather csi matrix of all users and antennas */
        for (int ue_idx = 0; ue_idx < UE_NUM; ue_idx++) {
            float* src_csi_ptr = (float*)csi_buffer_[subframe_offset + ue_idx]
                + offset_in_csi_buffer * 2;
            for (int ant_idx = 0; ant_idx < BS_ANT_NUM; ant_idx += 4) {
                /* fetch 4 complex floats for 4 ants */
                __m256 t_csi = _mm256_i32gather_ps(src_csi_ptr, index, 4);
                _mm256_store_ps(tar_csi_ptr, t_csi);
                // printf("UE %d, ant %d, data: %.4f, %.4f, %.4f, %.4f, %.4f,
                // %.4f\n", ue_idx, ant_idx, *((float *)tar_csi_ptr), *((float
                // *)tar_csi_ptr+1),
                //         *((float *)tar_csi_ptr+2), *((float *)tar_csi_ptr+3),
                //         *((float *)tar_csi_ptr+4), *((float
                //         *)tar_csi_ptr+5));
                src_csi_ptr += 8 * transpose_block_size;
                tar_csi_ptr += 8;
            }
        }

#if DEBUG_UPDATE_STATS_DETAILED
        double duration1 = get_time() - start_time1;
        (*ZF_task_duration)[tid * 8][1] += duration1;
#endif
        cx_float* ptr_in = (cx_float*)csi_gather_buffer;
        cx_fmat mat_input(ptr_in, BS_ANT_NUM, UE_NUM, false);
        // cout<<"CSI matrix"<<endl;
        // cout<<mat_input.st()<<endl;
        precoder(&mat_input, frame_id, cur_sc_id, cur_offset,
            config_->dl_data_symbol_num_perframe > 0);

#if DEBUG_UPDATE_STATS_DETAILED
        double start_time2 = get_time();
        double duration2 = start_time2 - start_time1;
        (*ZF_task_duration)[tid * 8][2] += duration2;
#endif

        // cout<<"Precoder:" <<mat_output<<endl;
#if DEBUG_UPDATE_STATS_DETAILED
        double duration3 = get_time() - start_time2;
        (*ZF_task_duration)[tid * 8][3] += duration3;
#endif
        // // int mat_elem = UE_NUM * BS_ANT_NUM;
        // // int cache_line_num = mat_elem / 8;
        // for (int line_idx = 0; line_idx < cache_line_num; line_idx++) {
        //     _mm256_stream_ps(tar_ptr, _mm256_load_ps(src_ptr));
        //     _mm256_stream_ps(tar_ptr + 8, _mm256_load_ps(src_ptr + 8));
        //     tar_ptr += 16;
        //     src_ptr += 16;
        // }

#if DEBUG_UPDATE_STATS
        ZF_task_count[tid * 16] = ZF_task_count[tid * 16] + 1;
        double duration = get_time() - start_time1;
        (*ZF_task_duration)[tid * 8][0] += duration;
        if (duration > 500) {
            printf("Thread %d ZF takes %.2f\n", tid, duration);
        }
#endif
    }
}

void DoZF::ZF_freq_orthogonal(int offset)
{
    int OFDM_DATA_NUM = config_->OFDM_DATA_NUM;
    int zf_block_size = config_->zf_block_size;
    int UE_NUM = config_->UE_NUM;
    int frame_id = offset / config_->zf_block_num;
    int sc_id = offset % config_->zf_block_num * zf_block_size;
#if DEBUG_PRINT_IN_TASK
    printf("In doZF thread %d: frame: %d, subcarrier: %d, block: %d\n", tid,
        frame_id, sc_id, sc_id / UE_NUM);
#endif
    int offset_in_buffer = frame_id * OFDM_DATA_NUM + sc_id;

#if DEBUG_UPDATE_STATS
    double start_time1 = get_time();
#endif
    int transpose_block_size = config_->transpose_block_size;
    int BS_ANT_NUM = config_->BS_ANT_NUM;
    for (int i = 0; i < UE_NUM; i++) {
        int cur_sc_id = sc_id + i;
        __m256i index = _mm256_setr_epi32(0, 1, transpose_block_size * 2,
            transpose_block_size * 2 + 1, transpose_block_size * 4,
            transpose_block_size * 4 + 1, transpose_block_size * 6,
            transpose_block_size * 6 + 1);

        int transpose_block_id = cur_sc_id / transpose_block_size;
        int sc_inblock_idx = cur_sc_id % transpose_block_size;
        int offset_in_csi_buffer
            = transpose_block_id * BS_ANT_NUM * transpose_block_size
            + sc_inblock_idx;
        int subframe_offset = frame_id;
        float* tar_csi_ptr = (float*)csi_gather_buffer + BS_ANT_NUM * i * 2;

        float* src_csi_ptr
            = (float*)csi_buffer_[subframe_offset] + offset_in_csi_buffer * 2;
        for (int ant_idx = 0; ant_idx < BS_ANT_NUM; ant_idx += 4) {
            // fetch 4 complex floats for 4 ants
            __m256 t_csi = _mm256_i32gather_ps(src_csi_ptr, index, 4);
            _mm256_store_ps(tar_csi_ptr, t_csi);
            // printf("UE %d, ant %d, data: %.4f, %.4f, %.4f, %.4f, %.4f,
            // %.4f\n", ue_idx, ant_idx, *((float *)tar_csi_ptr), *((float
            // *)tar_csi_ptr+1),
            //         *((float *)tar_csi_ptr+2), *((float *)tar_csi_ptr+3),
            //         *((float *)tar_csi_ptr+4), *((float *)tar_csi_ptr+5));
            src_csi_ptr += 8 * transpose_block_size;
            tar_csi_ptr += 8;
        }
    }

#if DEBUG_UPDATE_STATS_DETAILED
    double duration1 = get_time() - start_time1;
    (*ZF_task_duration)[tid * 8][1] += duration1;
#endif
    cx_float* ptr_in = (cx_float*)csi_gather_buffer;
    cx_fmat mat_input(ptr_in, BS_ANT_NUM, UE_NUM, false);
    // cout<<"CSI matrix"<<endl;
    // cout<<mat_input.st()<<endl;
    precoder(&mat_input, frame_id, sc_id, offset_in_buffer,
        config_->dl_data_symbol_num_perframe > 0);

#if DEBUG_UPDATE_STATS_DETAILED
    double start_time2 = get_time();
    double duration2 = start_time2 - start_time1;
    (*ZF_task_duration)[tid * 8][2] += duration2;
#endif

    // cout<<"Precoder:" <<mat_output<<endl;
#if DEBUG_UPDATE_STATS_DETAILED
    double duration3 = get_time() - start_time2;
    (*ZF_task_duration)[tid * 8][3] += duration3;
#endif

#if DEBUG_UPDATE_STATS
    ZF_task_count[tid * 16] = ZF_task_count[tid * 16] + 1;
    double duration = get_time() - start_time1;
    (*ZF_task_duration)[tid * 8][0] += duration;
    if (duration > 500) {
        printf("Thread %d ZF takes %.2f\n", tid, duration);
    }
#endif
}

void DoZF::Predict(int offset)
{
    int OFDM_DATA_NUM = config_->OFDM_DATA_NUM;
    int zf_block_size = config_->zf_block_size;
    int frame_id = offset / config_->zf_block_num;
    int sc_id = offset % config_->zf_block_num * zf_block_size;

    // Use stale CSI as predicted CSI
    // TODO: add prediction algorithm
    int offset_in_buffer = frame_id * OFDM_DATA_NUM + sc_id;
    cx_float* ptr_in = (cx_float*)pred_csi_buffer;
    int BS_ANT_NUM = config_->BS_ANT_NUM;
    int UE_NUM = config_->UE_NUM;
    memcpy(ptr_in, (cx_float*)csi_buffer_[offset_in_buffer],
        sizeof(cx_float) * BS_ANT_NUM * UE_NUM);
    cx_fmat mat_input(ptr_in, BS_ANT_NUM, UE_NUM, false);
    int offset_next_frame
        = ((frame_id + 1) % TASK_BUFFER_FRAME_NUM) * OFDM_DATA_NUM + sc_id;
    precoder(&mat_input, frame_id, sc_id, offset_next_frame,
        config_->dl_data_symbol_num_perframe > 0);

    // inform main thread
    finish(offset_next_frame);
}
