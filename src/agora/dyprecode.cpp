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
}

DyPrecode::~DyPrecode()
{
    free_buffer_1d(&modulated_buffer_temp_);
    free_buffer_1d(&precoded_buffer_temp_);
}

void DyPrecode::Launch(
    size_t frame_id, size_t symbol_id_dl, size_t base_sc_id)
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
    int max_sc_ite = std::min(cfg_->demul_block_size, 
        std::min(cfg_->subcarrier_start + (tid_ + 1) * cfg_->subcarrier_block_size, cfg_->subcarrier_end) - base_sc_id);
    assert(max_sc_ite % kSCsPerCacheline == 0);

    // Begin Debug
    // printf("DL mod data base sc %u:\n", base_sc_id);
    // End Debug

    for (int i = 0; i < max_sc_ite; i = i + 4) {
        for (int j = 0; j < 4; j++) {
            int cur_sc_id = base_sc_id + i + j;

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
                = (cx_float*)precoded_buffer_temp_ + (i + j) * cfg_->BS_ANT_NUM;
            cx_fmat mat_precoded(precoded_ptr, 1, cfg_->BS_ANT_NUM, false);

            mat_precoded = mat_data * mat_precoder;

            // printf("In doPrecode thread %d: frame: %d, symbol: %d, "
            //        "subcarrier: % d\n ",
            //     tid, frame_id, current_data_symbol_id, cur_sc_id);
            // cout << "Precoder: \n" << mat_precoder << endl;
            // cout << "Data: \n" << mat_data << endl;
            // cout << "Precoded data: \n" << mat_precoded << endl;
        }
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
        for (size_t i = 0; i < cfg_->demul_block_size / 4; i++) {
            float* input_shifted_ptr
                = precoded_ptr + 4 * i * 2 * cfg_->BS_ANT_NUM + ant_id * 2;
            __m256d t_data
                = _mm256_i64gather_pd((double*)input_shifted_ptr, index, 8);
            _mm256_store_pd((double*)(ifft_ptr + i * 8), t_data);
        }
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
