#include "dyprecode.hpp"
#include "concurrent_queue_wrapper.hpp"

using namespace arma;

DyPrecode::DyPrecode(Config* in_config, int in_tid, double freq_ghz,
    Table<int8_t>& encoded_buffer_to_precode,
    PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& dl_zf_matrices,
    Table<complex_float>& dl_ifft_buffer,
    std::vector<std::vector<ControlInfo>>& control_info_table,
    std::vector<size_t>& control_idx_list)
    : Doer(in_config, in_tid, freq_ghz)
    , dl_zf_matrices_(dl_zf_matrices)
    , dl_ifft_buffer_(dl_ifft_buffer)
    , encoded_buffer_to_precode_(encoded_buffer_to_precode)
    , control_info_table_(control_info_table)
    , control_idx_list_(control_idx_list)
{
    alloc_buffer_1d(&modulated_buffer_temp_, cfg_->UE_NUM, 64, 0);
    alloc_buffer_1d(
        &precoded_buffer_temp_, cfg_->demul_block_size * cfg_->BS_ANT_NUM, 64, 0);

    MKL_Complex8 alpha = { 1, 0 };
    MKL_Complex8 beta = { 0, 0 };

    for (size_t i = 1; i <= cfg_->UE_NUM; i ++) {
        mkl_jit_status_t status = mkl_jit_create_cgemm(&jitter[i], MKL_COL_MAJOR,
            MKL_NOTRANS, MKL_NOTRANS, cfg_->BS_ANT_NUM, 1, i, &alpha,
            cfg_->BS_ANT_NUM, i, &beta, cfg_->BS_ANT_NUM);
        if (MKL_JIT_ERROR == status) {
            fprintf(stderr,
                "Error: insufficient memory to JIT and store the DGEMM kernel\n");
            exit(1);
        }
        mkl_jit_cgemm[i] = mkl_jit_get_cgemm_ptr(jitter[i]);
    }
}

DyPrecode::~DyPrecode()
{
    free_buffer_1d(&modulated_buffer_temp_);
    free_buffer_1d(&precoded_buffer_temp_);
}

void DyPrecode::Launch(
    size_t frame_id, size_t symbol_id_dl, size_t base_sc_id, size_t sc_block_size)
{
    // size_t data_symbol_idx_dl = cfg_->get_dl_symbol_idx(frame_id, symbol_id);
    size_t data_symbol_idx_dl = symbol_id_dl;
    size_t total_data_symbol_idx
        = cfg_->get_total_data_symbol_idx_dl(frame_id, data_symbol_idx_dl);

    if (kDebugPrintInTask) {
        printf("In doPrecode TID %d: frame %zu, symbol %zu, subcarrier %zu\n",
            tid_, frame_id, symbol_id_dl, base_sc_id);
    }

    __m256i index = _mm256_setr_epi64x(
        0, cfg_->BS_ANT_NUM, cfg_->BS_ANT_NUM * 2, cfg_->BS_ANT_NUM * 3);
    // int max_sc_ite
    //     = std::min(cfg_->demul_block_size, (cfg_->bs_server_addr_idx + 1) * cfg_->get_num_sc_per_server() - base_sc_id);
    // int max_sc_ite = std::min(cfg_->demul_block_size, 
    //     std::min(cfg_->subcarrier_start + (tid_ + 1) * cfg_->subcarrier_block_size, cfg_->subcarrier_end) - base_sc_id);
    // assert(max_sc_ite % kSCsPerCacheline == 0);
    size_t max_sc_ite = sc_block_size;

    // Begin Debug
    // if (frame_id == 200)
    // printf("[Precoder %d] (%zu->%zu)\n", tid_, base_sc_id, base_sc_id + max_sc_ite - 1);
    // End Debug

    for (size_t i = 0; i < max_sc_ite; i ++) {
        // for (int j = 0; j < 4; j++) {
        size_t cur_sc_id = base_sc_id + i;

        complex_float* data_ptr = modulated_buffer_temp_;
        for (size_t user_id = 0; user_id < cfg_->UE_NUM; user_id++) {
            int8_t* raw_data_ptr
                = cfg_->get_encoded_buf(encoded_buffer_to_precode_, frame_id,
                symbol_id_dl, user_id) + cur_sc_id * cfg_->mod_order_bits;
            if (cur_sc_id % cfg_->OFDM_PILOT_SPACING == 0)
                data_ptr[user_id]
                    = cfg_->ue_specific_pilot[user_id][cur_sc_id];
            else
                data_ptr[user_id] = mod_single_uint8(
                    (uint8_t) * (raw_data_ptr), cfg_->mod_table);
        }

        // Begin Debug
        // printf("(%lf %lf) ", data_ptr[0].re, data_ptr[0].im);
        // if (frame_id == 800 && symbol_id_dl == 7 && base_sc_id == 304) {
        //     printf("| ");
        //     for (size_t k = 0; k < cfg_->UE_NUM; k ++) {
        //         printf("(%lf %lf) ", data_ptr[k].re, data_ptr[k].im);
        //     }
        //     printf("| ");
        // }
        // End Debug

        auto* precoder_ptr = reinterpret_cast<cx_float*>(
            dl_zf_matrices_[frame_id % kFrameWnd]
                            [cfg_->get_zf_sc_id(cur_sc_id)]);

        cx_fmat mat_precoder(
            precoder_ptr, cfg_->UE_NUM, cfg_->BS_ANT_NUM, false);
        cx_fmat mat_data((cx_float*)data_ptr, 1, cfg_->UE_NUM, false);
        cx_float* precoded_ptr
            = (cx_float*)precoded_buffer_temp_ + i * cfg_->BS_ANT_NUM;
        cx_fmat mat_precoded(precoded_ptr, 1, cfg_->BS_ANT_NUM, false);

        mkl_jit_cgemm[cfg_->UE_NUM](jitter[cfg_->UE_NUM], (MKL_Complex8*)precoder_ptr, (MKL_Complex8*)data_ptr,
            (MKL_Complex8*)precoded_ptr);

            // mat_precoded = mat_data * mat_precoder;

            // printf("In doPrecode thread %d: frame: %d, symbol: %d, "
            //        "subcarrier: % d\n ",
            //     tid, frame_id, current_data_symbol_id, cur_sc_id);
            // cout << "Precoder: \n" << mat_precoder << endl;
            // cout << "Data: \n" << mat_data << endl;
            // cout << "Precoded data: \n" << mat_precoded << endl;
        // }
    }

    // Begin Debug
    // printf("\nPrecoded data base sc %lu:\n", base_sc_id);
    // for (size_t i = 0; i < max_sc_ite; i ++) {
    //     printf("(%lf %lf) ", precoded_buffer_temp[i * cfg_->BS_ANT_NUM].re, precoded_buffer_temp[i * cfg_->BS_ANT_NUM].im);
    // }
    // printf("\n");
    // printf("Precoder:\n");
    // for (size_t i = 0; i < cfg_->UE_NUM; i ++) {
    //     for (size_t j = 0; j < cfg_->BS_ANT_NUM; j ++) {
    //         printf("(%lf %lf) ", dl_zf_matrices_[0][base_sc_id][j * cfg_->UE_NUM + i].re, dl_zf_matrices_[0][base_sc_id][j * cfg_->UE_NUM + i].im);
    //     }
    //     printf("\n");
    // }
    // End Debug

    float* precoded_ptr = (float*)precoded_buffer_temp_;
    for (size_t ant_id = 0; ant_id < cfg_->BS_ANT_NUM; ant_id++) {
        int ifft_buffer_offset
            = ant_id + cfg_->BS_ANT_NUM * total_data_symbol_idx;
        float* ifft_ptr
            = (float*)&dl_ifft_buffer_[ifft_buffer_offset][base_sc_id];
        // for (size_t i = 0; i < cfg_->demul_block_size / 4; i++) {
        size_t pre_sc_off = 0;
        if (base_sc_id % 4 > 0) {
            pre_sc_off = 4 - base_sc_id % 4;
            for (size_t i = 0; i < pre_sc_off; i ++) {
                memcpy(ifft_ptr + i * 2, 
                    precoded_ptr + i * 2 * cfg_->BS_ANT_NUM + ant_id * 2,
                    2 * sizeof(float));
            }
        }
        // rt_assert(max_sc_ite > pre_sc_off, "Invalid subcarrier allocation (too small)!");
        if (max_sc_ite < pre_sc_off) {
            printf("Invalid subcarrier allocation (too small: %zu %zu %zu)!", base_sc_id, sc_block_size, max_sc_ite);
            exit(0);
        }
        for (size_t i = 0; i < (max_sc_ite - pre_sc_off) / 4; i ++) {
            float* input_shifted_ptr
                = precoded_ptr + (4 * i + pre_sc_off) * 2 * cfg_->BS_ANT_NUM + ant_id * 2;
            __m256d t_data
                = _mm256_i64gather_pd((double*)input_shifted_ptr, index, 8);
            _mm256_store_pd((double*)(ifft_ptr + pre_sc_off * 2 + i * 8), t_data);
        }
        if ((max_sc_ite - pre_sc_off) % 4 > 0) {
            for (size_t i = 0; i < (max_sc_ite - pre_sc_off) % 4; i ++) {
                memcpy(ifft_ptr + pre_sc_off * 2 + (max_sc_ite - pre_sc_off) / 4 * 8 + i * 2, 
                    precoded_ptr + (((max_sc_ite - pre_sc_off) / 4) * 4 + i + pre_sc_off) * 2 * cfg_->BS_ANT_NUM + ant_id * 2,
                    2 * sizeof(float));
            }
        }
    }

    if (frame_id >= 200) {
        task_count_ += max_sc_ite;
    }
    // Begin Debug
    // printf("\nPrecoded data base sc %lu:\n", base_sc_id);
    // for (size_t i = 0; i < max_sc_ite; i ++) {
    //     printf("(%lf %lf) ", dl_ifft_buffer_[0][base_sc_id + i - cfg_->subcarrier_start].re, dl_ifft_buffer_[0][base_sc_id + i - cfg_->subcarrier_start].im);
    // }
    // printf("\n");
    // End Debug

    if (kDebugPrintInTask) {
        printf("In doPrecode thread %d: finished frame: %zu, symbol: %zu, "
               "subcarrier: %zu\n",
            tid_, frame_id, symbol_id_dl, base_sc_id);
    }
}