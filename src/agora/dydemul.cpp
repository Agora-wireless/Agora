#include "dydemul.hpp"
#include "concurrent_queue_wrapper.hpp"
#include "datatype_conversion.h"
#include <malloc.h>

static constexpr bool kUseSIMDGather = true;

DyDemul::DyDemul(Config* config, int tid, double freq_ghz,
    Table<char>& freq_domain_iq_buffer,
    PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& ul_zf_matrices,
    Table<complex_float>& equal_buffer,
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& demod_buffer_to_send,
    std::vector<std::vector<ControlInfo>>& control_info_table,
    std::vector<size_t>& control_idx_list)
    : Doer(config, tid, freq_ghz)
    , ul_zf_matrices_(ul_zf_matrices)
    , equal_buffer_(equal_buffer)
    , demod_buffer_to_send_(demod_buffer_to_send)
    , freq_domain_iq_buffer_(freq_domain_iq_buffer)
    , control_info_table_(control_info_table)
    , control_idx_list_(control_idx_list)
{
    data_gather_buffer_ = reinterpret_cast<complex_float*>(memalign(
        64, cfg_->OFDM_DATA_NUM * kMaxAntennas * sizeof(complex_float)));
    equaled_buffer_temp_ = reinterpret_cast<complex_float*>(
        memalign(64, cfg_->demul_block_size * kMaxUEs * sizeof(complex_float)));
    equaled_buffer_temp_transposed_ = reinterpret_cast<complex_float*>(
        memalign(64, cfg_->demul_block_size * kMaxUEs * sizeof(complex_float)));

#if USE_MKL_JIT
    MKL_Complex8 alpha = { 1, 0 };
    MKL_Complex8 beta = { 0, 0 };

    for (size_t i = 1; i <= cfg_->UE_NUM; i ++) {
        mkl_jit_status_t status = mkl_jit_create_cgemm(&jitter[i], MKL_COL_MAJOR,
            MKL_NOTRANS, MKL_NOTRANS, i, 1, cfg_->BS_ANT_NUM, &alpha,
            i, cfg_->BS_ANT_NUM, &beta, i);
        if (MKL_JIT_ERROR == status) {
            fprintf(stderr,
                "Error: insufficient memory to JIT and store the DGEMM kernel\n");
            exit(1);
        }
        mkl_jit_cgemm[i] = mkl_jit_get_cgemm_ptr(jitter[i]);
    }
#endif
}

DyDemul::~DyDemul()
{
    free(data_gather_buffer_);
    free(equaled_buffer_temp_);
    free(equaled_buffer_temp_transposed_);
}

void DyDemul::Launch(
    size_t frame_id, size_t symbol_idx_ul, size_t base_sc_id, size_t sc_block_size)
{
    const size_t total_data_symbol_idx_ul
        = cfg_->get_total_data_symbol_idx_ul(frame_id, symbol_idx_ul);
    const size_t frame_slot = frame_id % kFrameWnd;

    size_t start_tsc = worker_rdtsc();

    if (kDebugPrintInTask) {
        printf("In doDemul tid %d: frame: %zu, symbol: %zu, subcarrier: %zu \n",
            tid_, frame_id, symbol_idx_ul, base_sc_id);
    }

    size_t max_sc_ite = sc_block_size;
    // max_sc_ite = std::min(cfg_->demul_block_size, 
    //     std::min(cfg_->subcarrier_start + (tid_ + 1) * cfg_->subcarrier_block_size, cfg_->subcarrier_end) - base_sc_id);
    // assert(max_sc_ite % kSCsPerCacheline == 0);

    size_t pre_sc_off = 0;
    if (base_sc_id % kSCsPerCacheline > 0) {
        pre_sc_off = kSCsPerCacheline - base_sc_id % kSCsPerCacheline;
    }

    complex_float tmp[kSCsPerCacheline];
    if (pre_sc_off > 0) {
        size_t sc_chunk_size = pre_sc_off;
        for (size_t j = 0; j < cfg_->BS_ANT_NUM; j++) {
            float* src;
            src = reinterpret_cast<float*>(freq_domain_iq_buffer_[j]
                + frame_slot * (cfg_->symbol_num_perframe * cfg_->packet_length)
                + (symbol_idx_ul + cfg_->pilot_symbol_num_perframe)
                    * cfg_->packet_length
                + Packet::kOffsetOfData
                + 2 * sizeof(short) * (base_sc_id / kSCsPerCacheline * kSCsPerCacheline + cfg_->OFDM_DATA_START));
            simd_convert_float16_to_float32(
                reinterpret_cast<float*>(tmp), src, kSCsPerCacheline * 2);
            for (size_t t = 0; t < sc_chunk_size; t++) {
                complex_float* dst = data_gather_buffer_
                    + (base_sc_id + t) * cfg_->BS_ANT_NUM + j;
                *dst = tmp[t + base_sc_id % kSCsPerCacheline];
            }
        }
    }
    for (size_t i = 0; i < max_sc_ite - pre_sc_off; i += kSCsPerCacheline) {
        size_t sc_chunk_size = std::min(kSCsPerCacheline, max_sc_ite - pre_sc_off - i);
        for (size_t j = 0; j < cfg_->BS_ANT_NUM; j++) {
            float* src;
            src = reinterpret_cast<float*>(freq_domain_iq_buffer_[j]
                + frame_slot * (cfg_->symbol_num_perframe * cfg_->packet_length)
                + (symbol_idx_ul + cfg_->pilot_symbol_num_perframe)
                    * cfg_->packet_length
                + Packet::kOffsetOfData
                + 2 * sizeof(short) * (i + base_sc_id + pre_sc_off + cfg_->OFDM_DATA_START));
            simd_convert_float16_to_float32(
                reinterpret_cast<float*>(tmp), src, kSCsPerCacheline * 2);
            for (size_t t = 0; t < sc_chunk_size; t++) {
                complex_float* dst = data_gather_buffer_
                    + (base_sc_id + i + pre_sc_off + t) * cfg_->BS_ANT_NUM + j;
                *dst = tmp[t];
            }
        }
    }

    preprocess_cycles_ += worker_rdtsc() - start_tsc;

    for (size_t i = 0; i < max_sc_ite; i++) {
        size_t cur_sc_id = base_sc_id + i;

        size_t ue_num = 0;
        std::vector<ControlInfo>& control_list = control_info_table_[control_idx_list_[frame_id]];
        for (size_t j = 0; j < control_list.size(); j ++) {
            if (control_list[j].sc_start <= cur_sc_id && control_list[j].sc_end > cur_sc_id) {
                ue_num ++;
            }
        }
        
        if (ue_num == 0) {
            continue;
        }

        cx_float* equal_ptr = nullptr;
        equal_ptr = (cx_float*)(&equaled_buffer_temp_[(cur_sc_id - base_sc_id)
            * cfg_->UE_NUM]);
        cx_fmat mat_equaled(equal_ptr, ue_num, 1, false);

        cx_float* data_ptr = reinterpret_cast<cx_float*>(
            &data_gather_buffer_[cur_sc_id * cfg_->BS_ANT_NUM]);
        cx_float* ul_zf_ptr = reinterpret_cast<cx_float*>(
            ul_zf_matrices_[frame_slot][cfg_->get_zf_sc_id(cur_sc_id)]);

        size_t start_tsc2 = worker_rdtsc();

#if USE_MKL_JIT
        mkl_jit_cgemm[ue_num](jitter[ue_num], (MKL_Complex8*)ul_zf_ptr, (MKL_Complex8*)data_ptr,
            (MKL_Complex8*)equal_ptr);
#else
        const cx_fmat mat_data(data_ptr, cfg_->BS_ANT_NUM, 1, false);
        const cx_fmat mat_ul_zf(ul_zf_ptr, ue_num, cfg_->BS_ANT_NUM, false);
        mat_equaled = mat_ul_zf * mat_data;
#endif

        size_t start_tsc3 = worker_rdtsc();

        equal_cycles_ += start_tsc3 - start_tsc2;
        equal_count_ ++;
    }

    size_t start_tsc3 = worker_rdtsc();
    __m256i index2 = _mm256_setr_epi32(0, 1, cfg_->UE_NUM * 2,
        cfg_->UE_NUM * 2 + 1, cfg_->UE_NUM * 4, cfg_->UE_NUM * 4 + 1,
        cfg_->UE_NUM * 6, cfg_->UE_NUM * 6 + 1);
    size_t kNumDoubleInSIMD256 = sizeof(__m256) / sizeof(double); // == 4
    float* equal_T_ptr = (float*)(equaled_buffer_temp_transposed_) + (base_sc_id % kSCsPerCacheline) * 2;

    std::vector<ControlInfo>& info_list = control_info_table_[control_idx_list_[frame_id]];
    for (size_t i = 0; i < info_list.size(); i ++) {
        equal_T_ptr = (float*)(equaled_buffer_temp_transposed_) + (base_sc_id % kSCsPerCacheline) * 2;
        ControlInfo& info = info_list[i];
        float* equal_ptr = nullptr;
        if (kExportConstellation) {
            equal_ptr = (float*)(&equal_buffer_[total_data_symbol_idx_ul]
                                               [base_sc_id * cfg_->UE_NUM + info.ue_id]);
        } else {
            equal_ptr = (float*)(equaled_buffer_temp_ + info.ue_id);
        }
        if (pre_sc_off > 0) {
            for (size_t j = 0; j < pre_sc_off; j ++) {
                memcpy(equal_T_ptr + j * 2, 
                    equal_ptr + j * 2 * cfg_->UE_NUM + info.ue_id * 2,
                    2 * sizeof(float));
            }
        }
        // if (base_sc_id == 210) {
        //     printf("Demul: equal ptr %p->%p, pre sc off %zu, origin %p\n", equal_T_ptr, 
        //         equal_T_ptr + pre_sc_off * 2, pre_sc_off, equaled_buffer_temp_transposed_);
        // }
        equal_T_ptr += pre_sc_off * 2;     
        equal_ptr += cfg_->UE_NUM * pre_sc_off * 2;
        for (size_t j = 0; j < (max_sc_ite - pre_sc_off) / kNumDoubleInSIMD256; j++) {
            if (info.sc_start <= j * kNumDoubleInSIMD256 + base_sc_id + pre_sc_off && 
                info.sc_end > j * kNumDoubleInSIMD256 + base_sc_id + pre_sc_off) {
                __m256 equal_T_temp = _mm256_i32gather_ps(equal_ptr, index2, 4);
                _mm256_store_ps(equal_T_ptr, equal_T_temp);
            }
            equal_T_ptr += 8;
            equal_ptr += cfg_->UE_NUM * kNumDoubleInSIMD256 * 2;
        }
        if ((max_sc_ite - pre_sc_off) % kNumDoubleInSIMD256 > 0) {
            for (size_t j = 0; j < (max_sc_ite - pre_sc_off) % kNumDoubleInSIMD256; j ++) {
                memcpy(equal_T_ptr + j * 2, equal_ptr + cfg_->UE_NUM * j * 2, 2 * sizeof(float));
            }
        }
        equal_T_ptr = (float*)(equaled_buffer_temp_transposed_) + (base_sc_id % kSCsPerCacheline) * 2;

        int8_t* demul_ptr = demod_buffer_to_send_[frame_slot][symbol_idx_ul][i]
            + (cfg_->mod_order_bits * base_sc_id);

        switch (cfg_->mod_order_bits) {
        case (CommsLib::QAM16):
            if (pre_sc_off > 0) {
                demod_16qam_hard_loop(equal_T_ptr, (uint8_t*)demul_ptr, pre_sc_off);
                equal_T_ptr += pre_sc_off * 2;
                demul_ptr += cfg_->mod_order_bits * pre_sc_off;
            }
            demod_16qam_soft_avx2(equal_T_ptr, demul_ptr, max_sc_ite - pre_sc_off);
            break;
        case (CommsLib::QAM64):
            if (pre_sc_off > 0) {
                demod_64qam_hard_loop(equal_T_ptr, (uint8_t*)demul_ptr, pre_sc_off);
                equal_T_ptr += pre_sc_off * 2;
                demul_ptr += cfg_->mod_order_bits * pre_sc_off;
            }
            demod_64qam_soft_avx2(equal_T_ptr, demul_ptr, max_sc_ite - pre_sc_off);
            break;
        default:
            printf("Demodulation: modulation type %s not supported!\n",
                cfg_->modulation.c_str());
        }
    }

    demod_cycles_ += worker_rdtsc() - start_tsc3;
    demod_count_ += max_sc_ite;
    total_cycles_ += worker_rdtsc() - start_tsc;
    return;
}
