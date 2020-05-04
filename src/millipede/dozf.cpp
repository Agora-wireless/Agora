/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#include "dozf.hpp"
#include "concurrent_queue_wrapper.hpp"
#include "doer.hpp"
#include <malloc.h>

DoZF::DoZF(Config* config, int tid, double freq_ghz,
    moodycamel::ConcurrentQueue<Event_data>& task_queue,
    moodycamel::ConcurrentQueue<Event_data>& complete_task_queue,
    moodycamel::ProducerToken* worker_producer_token,
    Table<complex_float>& csi_buffer, Table<complex_float>& recip_buffer,
    Table<complex_float>& ul_precoder_buffer,
    Table<complex_float>& dl_precoder_buffer, Stats* stats_manager)
    : Doer(config, tid, freq_ghz, task_queue, complete_task_queue,
          worker_producer_token)
    , csi_buffer_(csi_buffer)
    , recip_buffer_(recip_buffer)
    , ul_precoder_buffer_(ul_precoder_buffer)
    , dl_precoder_buffer_(dl_precoder_buffer)
{
    duration_stat = stats_manager->get_duration_stat(DoerType::kZF, tid);
    pred_csi_buffer = reinterpret_cast<complex_float*>(
        memalign(64, cfg->BS_ANT_NUM * cfg->UE_NUM * sizeof(complex_float)));
    csi_gather_buffer = reinterpret_cast<complex_float*>(
        memalign(64, cfg->BS_ANT_NUM * cfg->UE_NUM * sizeof(complex_float)));
}

DoZF::~DoZF()
{
    free(pred_csi_buffer);
    free(csi_gather_buffer);
}

Event_data DoZF::launch(size_t tag)
{
    if (cfg->freq_orthogonal_pilot)
        ZF_freq_orthogonal(tag);
    else
        ZF_time_orthogonal(tag);

    return Event_data(EventType::kZF, tag);
}

void DoZF::precoder(void* mat_input_ptr, size_t frame_id, size_t sc_id,
    size_t offset, bool downlink_mode)
{
    auto& mat_input = *(arma::cx_fmat*)mat_input_ptr;
    auto* ptr_ul_out = (arma::cx_float*)ul_precoder_buffer_[offset];
    arma::cx_fmat mat_ul_precoder(
        ptr_ul_out, cfg->UE_NUM, cfg->BS_ANT_NUM, false);
    pinv(mat_ul_precoder, mat_input, 1e-2, "dc");
    if (downlink_mode) {
        auto* ptr_dl_out = (arma::cx_float*)dl_precoder_buffer_[offset];
        arma::cx_fmat mat_dl_precoder(
            ptr_dl_out, cfg->UE_NUM, cfg->BS_ANT_NUM, false);
        if (cfg->recipCalEn) {
            auto* calib = (arma::cx_float*)(&recip_buffer_[frame_id
                % TASK_BUFFER_FRAME_NUM][sc_id * cfg->BS_ANT_NUM]);
            arma::cx_fvec vec_calib(calib, cfg->BS_ANT_NUM, false);
            arma::cx_fmat mat_calib(cfg->BS_ANT_NUM, cfg->BS_ANT_NUM);
            mat_calib = diagmat(vec_calib);
            mat_dl_precoder = mat_ul_precoder * inv(mat_calib);
        } else
            mat_dl_precoder = mat_ul_precoder;
    }
}

void DoZF::ZF_time_orthogonal(size_t tag)
{
    size_t frame_id = gen_tag_t(tag).frame_id;
    size_t base_sc_id = gen_tag_t(tag).sc_id;
    if (kDebugPrintInTask) {
        printf("In doZF thread %d: frame: %zu, base subcarrier: %zu\n", tid,
            frame_id, base_sc_id);
    }
    const size_t offset_in_buffer
        = ((frame_id % TASK_BUFFER_FRAME_NUM) * cfg->OFDM_DATA_NUM)
        + base_sc_id;
    size_t num_subcarriers
        = std::min(cfg->zf_block_size, cfg->OFDM_DATA_NUM - base_sc_id);
    for (size_t i = 0; i < num_subcarriers; i++) {
        size_t start_tsc1 = worker_rdtsc();
        size_t cur_sc_id = base_sc_id + i;
        int cur_offset = offset_in_buffer + i;
        int transpose_block_size = cfg->transpose_block_size;
        __m256i index = _mm256_setr_epi32(0, 1, transpose_block_size * 2,
            transpose_block_size * 2 + 1, transpose_block_size * 4,
            transpose_block_size * 4 + 1, transpose_block_size * 6,
            transpose_block_size * 6 + 1);
        int transpose_block_id = cur_sc_id / transpose_block_size;
        int sc_inblock_idx = cur_sc_id % transpose_block_size;
        int offset_in_csi_buffer
            = transpose_block_id * cfg->BS_ANT_NUM * transpose_block_size
            + sc_inblock_idx;
        const size_t symbol_offset
            = (frame_id % TASK_BUFFER_FRAME_NUM) * cfg->UE_NUM;
        float* tar_csi_ptr = (float*)csi_gather_buffer;

        /* Gather csi matrix of all users and antennas */
        for (size_t ue_idx = 0; ue_idx < cfg->UE_NUM; ue_idx++) {
            float* src_csi_ptr = (float*)csi_buffer_[symbol_offset + ue_idx]
                + offset_in_csi_buffer * 2;
            for (size_t ant_idx = 0; ant_idx < cfg->BS_ANT_NUM; ant_idx += 4) {
                /* Fetch 4 complex floats for 4 ants */
                __m256 t_csi = _mm256_i32gather_ps(src_csi_ptr, index, 4);
                _mm256_store_ps(tar_csi_ptr, t_csi);
                // printf("UE %d, ant %d, data: %.4f, %.4f, %.4f, %.4f, %.4f,
                // %.4f\n", ue_idx, ant_idx, *((float *)tar_csi_ptr), *((float
                // *)tar_csi_ptr+1),
                //         *((float *)tar_csi_ptr+2), *((float *)tar_csi_ptr+3),
                //         *((float *)tar_csi_ptr+4), *((float
                //         *)tar_csi_ptr+5));
                src_csi_ptr += 8 * cfg->transpose_block_size;
                tar_csi_ptr += 8;
            }
        }

        duration_stat->task_duration[1] += worker_rdtsc() - start_tsc1;
        auto* ptr_in = (arma::cx_float*)csi_gather_buffer;
        arma::cx_fmat mat_input(ptr_in, cfg->BS_ANT_NUM, cfg->UE_NUM, false);
        // cout<<"CSI matrix"<<endl;
        // cout<<mat_input.st()<<endl;
        precoder(&mat_input, frame_id, cur_sc_id, cur_offset,
            cfg->dl_data_symbol_num_perframe > 0);

        double start_tsc2 = worker_rdtsc();
        duration_stat->task_duration[2] += start_tsc2 - start_tsc1;

        // cout<<"Precoder:" <<mat_output<<endl;
        double duration3 = worker_rdtsc() - start_tsc2;
        duration_stat->task_duration[3] += duration3;

        duration_stat->task_count++;
        duration_stat->task_duration[0] += worker_rdtsc() - start_tsc1;
        // if (duration > 500) {
        //     printf("Thread %d ZF takes %.2f\n", tid, duration);
        // }
    }
}

void DoZF::ZF_freq_orthogonal(size_t tag)
{
    size_t frame_id = gen_tag_t(tag).frame_id;
    size_t base_sc_id = gen_tag_t(tag).sc_id;
    if (kDebugPrintInTask) {
        printf("In doZF thread %d: frame: %zu, subcarrier: %zu, block: %zu\n",
            tid, frame_id, base_sc_id, base_sc_id / cfg->UE_NUM);
    }
    const size_t offset_in_buffer
        = ((frame_id % TASK_BUFFER_FRAME_NUM) * cfg->OFDM_DATA_NUM)
        + base_sc_id;

    double start_tsc1 = worker_rdtsc();
    for (size_t i = 0; i < cfg->UE_NUM; i++) {
        int cur_sc_id = base_sc_id + i;
        __m256i index = _mm256_setr_epi32(0, 1, cfg->transpose_block_size * 2,
            cfg->transpose_block_size * 2 + 1, cfg->transpose_block_size * 4,
            cfg->transpose_block_size * 4 + 1, cfg->transpose_block_size * 6,
            cfg->transpose_block_size * 6 + 1);

        int transpose_block_id = cur_sc_id / cfg->transpose_block_size;
        int sc_inblock_idx = cur_sc_id % cfg->transpose_block_size;
        int offset_in_csi_buffer
            = transpose_block_id * cfg->BS_ANT_NUM * cfg->transpose_block_size
            + sc_inblock_idx;
        const size_t symbol_offset = frame_id % TASK_BUFFER_FRAME_NUM;
        float* tar_csi_ptr
            = (float*)csi_gather_buffer + cfg->BS_ANT_NUM * i * 2;

        float* src_csi_ptr
            = (float*)csi_buffer_[symbol_offset] + offset_in_csi_buffer * 2;
        for (size_t ant_idx = 0; ant_idx < cfg->BS_ANT_NUM; ant_idx += 4) {
            // fetch 4 complex floats for 4 ants
            __m256 t_csi = _mm256_i32gather_ps(src_csi_ptr, index, 4);
            _mm256_store_ps(tar_csi_ptr, t_csi);
            // printf("UE %d, ant %d, data: %.4f, %.4f, %.4f, %.4f, %.4f,
            // %.4f\n", ue_idx, ant_idx, *((float *)tar_csi_ptr), *((float
            // *)tar_csi_ptr+1),
            //         *((float *)tar_csi_ptr+2), *((float *)tar_csi_ptr+3),
            //         *((float *)tar_csi_ptr+4), *((float *)tar_csi_ptr+5));
            src_csi_ptr += 8 * cfg->transpose_block_size;
            tar_csi_ptr += 8;
        }
    }

    duration_stat->task_duration[1] += worker_rdtsc() - start_tsc1;
    auto* ptr_in = (arma::cx_float*)csi_gather_buffer;
    arma::cx_fmat mat_input(ptr_in, cfg->BS_ANT_NUM, cfg->UE_NUM, false);
    // cout<<"CSI matrix"<<endl;
    // cout<<mat_input.st()<<endl;
    precoder(&mat_input, frame_id, base_sc_id, offset_in_buffer,
        cfg->dl_data_symbol_num_perframe > 0);

    double start_tsc2 = worker_rdtsc();
    duration_stat->task_duration[2] += start_tsc2 - start_tsc1;

    // cout<<"Precoder:" <<mat_output<<endl;
    duration_stat->task_duration[3] += worker_rdtsc() - start_tsc2;
    duration_stat->task_count++;
    duration_stat->task_duration[0] += worker_rdtsc() - start_tsc1;

    // if (duration > 500) {
    //     printf("Thread %d ZF takes %.2f\n", tid, duration);
    // }
}

// Currently unused
void DoZF::Predict(size_t tag)
{
    size_t frame_id = gen_tag_t(tag).frame_id;
    size_t base_sc_id = gen_tag_t(tag).sc_id;

    // Use stale CSI as predicted CSI
    // TODO: add prediction algorithm
    const size_t offset_in_buffer
        = ((frame_id % TASK_BUFFER_FRAME_NUM) * cfg->OFDM_DATA_NUM)
        + base_sc_id;
    auto* ptr_in = (arma::cx_float*)pred_csi_buffer;
    memcpy(ptr_in, (arma::cx_float*)csi_buffer_[offset_in_buffer],
        sizeof(arma::cx_float) * cfg->BS_ANT_NUM * cfg->UE_NUM);
    arma::cx_fmat mat_input(ptr_in, cfg->BS_ANT_NUM, cfg->UE_NUM, false);
    const size_t offset_next_frame
        = ((frame_id + 1) % TASK_BUFFER_FRAME_NUM) * cfg->OFDM_DATA_NUM
        + base_sc_id;
    precoder(&mat_input, frame_id, base_sc_id, offset_next_frame,
        cfg->dl_data_symbol_num_perframe > 0);
}
