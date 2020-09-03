/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#include "dodemul.hpp"
#include "concurrent_queue_wrapper.hpp"
#include <malloc.h>

static constexpr bool kUseSIMDGather = true;

DoDemul::DoDemul(Config* config, int tid, double freq_ghz,
    moodycamel::ConcurrentQueue<Event_data>& task_queue,
    moodycamel::ConcurrentQueue<Event_data>& complete_task_queue,
    moodycamel::ProducerToken* worker_producer_token,
    Table<complex_float>& data_buffer,
    PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& ul_zf_matrices,
    Table<complex_float>& ue_spec_pilot_buffer,
    Table<complex_float>& equal_buffer,
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& demod_buffers,
    PhyStats* in_phy_stats, Stats* stats_manager)
    : Doer(config, tid, freq_ghz, task_queue, complete_task_queue,
          worker_producer_token)
    , data_buffer_(data_buffer)
    , ul_zf_matrices_(ul_zf_matrices)
    , ue_spec_pilot_buffer_(ue_spec_pilot_buffer)
    , equal_buffer_(equal_buffer)
    , demod_buffers_(demod_buffers)
    , phy_stats(in_phy_stats)
{
    duration_stat = stats_manager->get_duration_stat(DoerType::kDemul, tid);

    spm_buffer = reinterpret_cast<complex_float*>(memalign(
        64, kSCsPerCacheline * cfg->BS_ANT_NUM * sizeof(complex_float)));
    equaled_buffer_temp = reinterpret_cast<complex_float*>(memalign(
        64, cfg->demul_block_size * cfg->UE_NUM * sizeof(complex_float)));
    equaled_buffer_temp_transposed = reinterpret_cast<complex_float*>(memalign(
        64, cfg->demul_block_size * cfg->UE_NUM * sizeof(complex_float)));

    // phase offset calibration data
    cx_float* ue_pilot_ptr = (cx_float*)cfg->ue_specific_pilot[0];
    cx_fmat mat_pilot_data(
        ue_pilot_ptr, cfg->OFDM_DATA_NUM, cfg->UE_ANT_NUM, false);
    ue_pilot_data = mat_pilot_data.st();

#if USE_MKL_JIT
    MKL_Complex8 alpha = { 1, 0 };
    MKL_Complex8 beta = { 0, 0 };

    mkl_jit_status_t status = mkl_jit_create_cgemm(&jitter, MKL_COL_MAJOR,
        MKL_NOTRANS, MKL_NOTRANS, cfg->UE_NUM, 1, cfg->BS_ANT_NUM, &alpha,
        cfg->UE_NUM, cfg->BS_ANT_NUM, &beta, cfg->UE_NUM);
    if (MKL_JIT_ERROR == status) {
        fprintf(stderr,
            "Error: insufficient memory to JIT and store the DGEMM kernel\n");
        exit(1);
    }
    mkl_jit_cgemm = mkl_jit_get_cgemm_ptr(jitter);
#endif
}

DoDemul::~DoDemul()
{
    free(spm_buffer);
    free(equaled_buffer_temp);
    free(equaled_buffer_temp_transposed);
}

Event_data DoDemul::launch(size_t tag)
{
    const size_t frame_id = gen_tag_t(tag).frame_id;
    const size_t symbol_idx_ul = gen_tag_t(tag).symbol_id;
    const size_t base_sc_id = gen_tag_t(tag).sc_id;
    const size_t total_data_symbol_idx_ul
        = cfg->get_total_data_symbol_idx_ul(frame_id, symbol_idx_ul);
    const complex_float* data_buf = data_buffer_[total_data_symbol_idx_ul];

    const size_t frame_slot = frame_id % kFrameWnd;
    size_t start_tsc = worker_rdtsc();

    if (kDebugPrintInTask) {
        printf("In doDemul tid %d: frame: %zu, symbol: %zu, subcarrier: %zu \n",
            tid, frame_id, symbol_idx_ul, base_sc_id);
    }

    size_t max_sc_ite
        = std::min(cfg->demul_block_size, cfg->OFDM_DATA_NUM - base_sc_id);
    assert(max_sc_ite % kSCsPerCacheline == 0);
    // Iterate through cache lines
    for (size_t i = 0; i < max_sc_ite; i += kSCsPerCacheline) {
        size_t start_tsc0 = worker_rdtsc();

        // Step 1: Populate spm_buffer as a row-major matrix with
        // kSCsPerCacheline rows and BS_ANT_NUM columns

        // Since kSCsPerCacheline divides demul_block_size and
        // kTransposeBlockSize, all subcarriers (base_sc_id + i) lie in the
        // same partial transpose block.
        const size_t partial_transpose_block_base
            = ((base_sc_id + i) / kTransposeBlockSize)
            * (kTransposeBlockSize * cfg->BS_ANT_NUM);

        if (kUseSIMDGather) {
            __m256i index = _mm256_setr_epi32(0, 1, kTransposeBlockSize * 2,
                kTransposeBlockSize * 2 + 1, kTransposeBlockSize * 4,
                kTransposeBlockSize * 4 + 1, kTransposeBlockSize * 6,
                kTransposeBlockSize * 6 + 1);
            // Gather data for all antennas and 8 subcarriers in the same cache
            // line, 1 subcarrier and 4 ants per iteration
            size_t cur_sc_offset = partial_transpose_block_base
                + (base_sc_id + i) % kTransposeBlockSize;
            auto* src = (const float*)&data_buf[cur_sc_offset];
            auto* dst = (float*)spm_buffer;
            for (size_t ant_i = 0; ant_i < cfg->BS_ANT_NUM; ant_i += 4) {
                for (size_t j = 0; j < kSCsPerCacheline; j++) {
                    __m256 data_rx = _mm256_i32gather_ps(src + j * 2, index, 4);
                    _mm256_store_ps(dst + j * cfg->BS_ANT_NUM * 2, data_rx);
                }
                src += (kSCsPerCacheline * kTransposeBlockSize);
                dst += 8;
            }
        } else {
            complex_float* dst = spm_buffer;
            for (size_t j = 0; j < kSCsPerCacheline; j++) {
                for (size_t ant_i = 0; ant_i < cfg->BS_ANT_NUM; ant_i++) {
                    *dst++ = data_buf[partial_transpose_block_base
                        + (ant_i * kTransposeBlockSize)
                        + ((base_sc_id + i + j) % kTransposeBlockSize)];
                }
            }
        }
        duration_stat->task_duration[1] += worker_rdtsc() - start_tsc0;

        // Step 2: For each subcarrier, perform equalization by multiplying the
        // subcarrier's data from each antenna with the subcarrier's precoder
        for (size_t j = 0; j < kSCsPerCacheline; j++) {
            // size_t start_tsc1 = worker_rdtsc();
            const size_t cur_sc_id = base_sc_id + i + j;

            cx_float* equal_ptr = nullptr;
            if (kExportConstellation) {
                equal_ptr
                    = (cx_float*)(&equal_buffer_[total_data_symbol_idx_ul]
                                                [cur_sc_id * cfg->UE_NUM]);
            } else {
                equal_ptr
                    = (cx_float*)(&equaled_buffer_temp[(cur_sc_id - base_sc_id)
                        * cfg->UE_NUM]);
            }
            cx_fmat mat_equaled(equal_ptr, cfg->UE_NUM, 1, false);

            auto* data_ptr
                = reinterpret_cast<cx_float*>(&spm_buffer[j * cfg->BS_ANT_NUM]);
            auto* ul_zf_ptr = reinterpret_cast<cx_float*>(
                ul_zf_matrices_[frame_slot][cfg->get_zf_sc_id(cur_sc_id)]);

            size_t start_tsc2 = worker_rdtsc();
#if USE_MKL_JIT
            mkl_jit_cgemm(jitter, (MKL_Complex8*)ul_zf_ptr,
                (MKL_Complex8*)data_ptr, (MKL_Complex8*)equal_ptr);
#else
            const cx_fmat mat_data(data_ptr, cfg->BS_ANT_NUM, 1, false);
            const cx_fmat mat_ul_zf(
                ul_zf_ptr, cfg->UE_NUM, cfg->BS_ANT_NUM, false);
            mat_equaled = mat_ul_zf * mat_data;
#endif

            if (symbol_idx_ul < cfg->UL_PILOT_SYMS) { // Calc new phase shift
                if (symbol_idx_ul == 0 && cur_sc_id == 0) {
                    // Reset previous frame
                    cx_float* phase_shift_ptr
                        = (cx_float*)ue_spec_pilot_buffer_[(frame_id - 1)
                            % TASK_BUFFER_FRAME_NUM];
                    cx_fmat mat_phase_shift(phase_shift_ptr, cfg->UE_NUM,
                        cfg->UL_PILOT_SYMS, false);
                    mat_phase_shift.fill(0);
                }
                cx_float* phase_shift_ptr
                    = (cx_float*)&ue_spec_pilot_buffer_[frame_id
                        % TASK_BUFFER_FRAME_NUM][symbol_idx_ul * cfg->UE_NUM];
                cx_fmat mat_phase_shift(phase_shift_ptr, cfg->UE_NUM, 1, false);
                cx_fmat shift_sc
                    = sign(mat_equaled % conj(ue_pilot_data.col(cur_sc_id)));
                mat_phase_shift += shift_sc;
            } else if (cfg->UL_PILOT_SYMS
                > 0) { // apply previously calc'ed phase shift to data
                cx_float* pilot_corr_ptr = (cx_float*)
                    ue_spec_pilot_buffer_[frame_id % TASK_BUFFER_FRAME_NUM];
                cx_fmat pilot_corr_mat(
                    pilot_corr_ptr, cfg->UE_NUM, cfg->UL_PILOT_SYMS, false);
                fmat theta_mat = arg(pilot_corr_mat);
                fmat theta_inc = zeros<fmat>(cfg->UE_NUM, 1);
                for (size_t s = 1; s < cfg->UL_PILOT_SYMS; s++) {
                    fmat theta_diff = theta_mat.col(s) - theta_mat.col(s - 1);
                    theta_inc += theta_diff;
                }
                theta_inc /= (float)std::max(1, (int)cfg->UL_PILOT_SYMS - 1);
                fmat cur_theta = theta_mat.col(0) + (symbol_idx_ul * theta_inc);
                cx_fmat mat_phase_correct = zeros<cx_fmat>(size(cur_theta));
                mat_phase_correct.set_real(cos(-cur_theta));
                mat_phase_correct.set_imag(sin(-cur_theta));
                mat_equaled %= mat_phase_correct;

                // Measure EVM from ground truth
                if (symbol_idx_ul == cfg->UL_PILOT_SYMS) {
                    phy_stats->update_evm_stats(
                        frame_id, cur_sc_id, mat_equaled);
                    if (kPrintPhyStats && cur_sc_id == 0) {
                        phy_stats->print_evm_stats(frame_id - 1);
                    }
                }
            }

            size_t start_tsc3 = worker_rdtsc();
            duration_stat->task_duration[2] += start_tsc3 - start_tsc2;
            duration_stat->task_count++;
        }
    }

    size_t start_tsc3 = worker_rdtsc();
    __m256i index2 = _mm256_setr_epi32(0, 1, cfg->UE_NUM * 2,
        cfg->UE_NUM * 2 + 1, cfg->UE_NUM * 4, cfg->UE_NUM * 4 + 1,
        cfg->UE_NUM * 6, cfg->UE_NUM * 6 + 1);
    float* equal_T_ptr = (float*)(equaled_buffer_temp_transposed);
    for (size_t i = 0; i < cfg->UE_NUM; i++) {
        float* equal_ptr = nullptr;
        if (kExportConstellation) {
            equal_ptr = (float*)(&equal_buffer_[total_data_symbol_idx_ul]
                                               [base_sc_id * cfg->UE_NUM + i]);
        } else {
            equal_ptr = (float*)(equaled_buffer_temp + i);
        }
        size_t kNumDoubleInSIMD256 = sizeof(__m256) / sizeof(double); // == 4
        for (size_t j = 0; j < max_sc_ite / kNumDoubleInSIMD256; j++) {
            __m256 equal_T_temp = _mm256_i32gather_ps(equal_ptr, index2, 4);
            _mm256_store_ps(equal_T_ptr, equal_T_temp);
            equal_T_ptr += 8;
            equal_ptr += cfg->UE_NUM * kNumDoubleInSIMD256 * 2;
        }
        equal_T_ptr = (float*)(equaled_buffer_temp_transposed);
        int8_t* demul_ptr = demod_buffers_[frame_id][symbol_idx_ul][i]
            + (cfg->mod_type * base_sc_id);

        switch (cfg->mod_type) {
        case (CommsLib::QAM16):
            demod_16qam_soft_avx2(equal_T_ptr, demul_ptr, max_sc_ite);
            break;
        case (CommsLib::QAM64):
            demod_64qam_soft_avx2(equal_T_ptr, demul_ptr, max_sc_ite);
            break;
        default:
            printf("Demodulation: modulation type %s not supported!\n",
                cfg->modulation.c_str());
        }
        // printf("In doDemul thread %d: frame: %d, symbol: %d, sc_id: %d \n",
        //     tid, frame_id, symbol_idx_ul, base_sc_id);
        // cout << "Demuled data : \n ";
        // cout << " UE " << i << ": ";
        // for (int k = 0; k < max_sc_ite * cfg->mod_type; k++)
        //     printf("%i ", demul_ptr[k]);
        // cout << endl;
    }

    duration_stat->task_duration[3] += worker_rdtsc() - start_tsc3;
    duration_stat->task_duration[0] += worker_rdtsc() - start_tsc;
    return Event_data(EventType::kDemul, tag);
}
