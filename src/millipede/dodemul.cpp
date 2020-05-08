/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#include "dodemul.hpp"
#include "concurrent_queue_wrapper.hpp"
#include <malloc.h>

static constexpr bool kUseSIMDGather = false;

using namespace arma;
DoDemul::DoDemul(Config* config, int tid, double freq_ghz,
    moodycamel::ConcurrentQueue<Event_data>& task_queue,
    moodycamel::ConcurrentQueue<Event_data>& complete_task_queue,
    moodycamel::ProducerToken* worker_producer_token,
    Table<complex_float>& data_buffer, Table<complex_float>& precoder_buffer,
    Table<complex_float>& equal_buffer, Table<uint8_t>& demod_hard_buffer,
    Table<int8_t>& demod_soft_buffer, Stats* stats_manager)
    : Doer(config, tid, freq_ghz, task_queue, complete_task_queue,
          worker_producer_token)
    , data_buffer_(data_buffer)
    , precoder_buffer_(precoder_buffer)
    , equal_buffer_(equal_buffer)
    , demod_hard_buffer_(demod_hard_buffer)
    , demod_soft_buffer_(demod_soft_buffer)
{
    duration_stat = stats_manager->get_duration_stat(DoerType::kDemul, tid);
    spm_buffer = reinterpret_cast<complex_float*>(memalign(
        64, kSCsPerCacheline * cfg->BS_ANT_NUM * sizeof(complex_float)));
    equaled_buffer_temp = reinterpret_cast<complex_float*>(memalign(
        64, cfg->demul_block_size * cfg->UE_NUM * sizeof(complex_float)));
    equaled_buffer_temp_transposed = reinterpret_cast<complex_float*>(memalign(
        64, cfg->demul_block_size * cfg->UE_NUM * sizeof(complex_float)));
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
    const size_t symbol_id = gen_tag_t(tag).symbol_id;
    const size_t base_sc_id = gen_tag_t(tag).sc_id;
    const size_t total_data_symbol_idx
        = cfg->get_total_data_symbol_idx_ul(frame_id, symbol_id);
    const complex_float* data_buf = data_buffer_[total_data_symbol_idx];

    size_t start_tsc = worker_rdtsc();

    if (kDebugPrintInTask) {
        printf("In doDemul tid %d: frame: %zu, symbol: %zu, subcarrier: %zu \n",
            tid, frame_id, symbol_id, base_sc_id);
    }

    size_t max_sc_ite
        = std::min(cfg->demul_block_size, cfg->OFDM_DATA_NUM - base_sc_id);
    assert(max_sc_ite % kSCsPerCacheline == 0);
    // Iterate through cache lines
    for (size_t i = 0; i < max_sc_ite; i += kSCsPerCacheline) {
        size_t start_tsc1 = worker_rdtsc();
        if (kUseSIMDGather) {
            size_t transpose_block_size = cfg->transpose_block_size;
            __m256i index = _mm256_setr_epi32(0, 1, transpose_block_size * 2,
                transpose_block_size * 2 + 1, transpose_block_size * 4,
                transpose_block_size * 4 + 1, transpose_block_size * 6,
                transpose_block_size * 6 + 1);
            // Gather data for all antennas and 8 subcarriers in the same cache
            // line,  1 subcarrier and 4 ants per iteration
            size_t cur_sc_offset = (((base_sc_id + i) / transpose_block_size)
                                       * transpose_block_size * cfg->BS_ANT_NUM)
                + (base_sc_id + i) % transpose_block_size;
            auto* src = (const float*)&data_buf[cur_sc_offset];
            auto* dst = (float*)spm_buffer;
            for (size_t ant_idx = 0; ant_idx < cfg->BS_ANT_NUM; ant_idx += 4) {
                for (size_t j = 0; j < kSCsPerCacheline; j++) {
                    __m256 data_rx = _mm256_i32gather_ps(src + j * 2, index, 4);
                    _mm256_store_ps(dst + j * cfg->BS_ANT_NUM * 2, data_rx);
                }
                src += (kSCsPerCacheline * transpose_block_size);
                dst += 8;
            }
        } else {
            complex_float* dst = spm_buffer;
            const size_t block_base_offset
                = ((base_sc_id + i) / cfg->transpose_block_size)
                * (cfg->transpose_block_size * cfg->BS_ANT_NUM);
            for (size_t j = 0; j < kSCsPerCacheline; j++) {
                size_t cur_sc_id = base_sc_id + i + j;
                for (size_t ant_i = 0; ant_i < cfg->BS_ANT_NUM; ant_i++) {
                    *dst++ = data_buf[block_base_offset
                        + (ant_i * cfg->transpose_block_size)
                        + (cur_sc_id % cfg->transpose_block_size)];
                }
            }
        }
        duration_stat->task_duration[1] += worker_rdtsc() - start_tsc1;

        for (size_t j = 0; j < kSCsPerCacheline; j++) {
            // Create input data matrix
            cx_float* data_ptr = (cx_float*)(spm_buffer + j * cfg->BS_ANT_NUM);
            cx_fmat mat_data(data_ptr, cfg->BS_ANT_NUM, 1, false);

            // Create input precoder matrix
            size_t cur_sc_id = base_sc_id + i + j;
            size_t precoder_offset
                = (frame_id % TASK_BUFFER_FRAME_NUM) * cfg->OFDM_DATA_NUM
                + cur_sc_id;
            if (cfg->freq_orthogonal_pilot)
                precoder_offset = precoder_offset - cur_sc_id % cfg->UE_NUM;
            cx_float* precoder_ptr
                = (cx_float*)precoder_buffer_[precoder_offset];
            cx_fmat mat_precoder(
                precoder_ptr, cfg->UE_NUM, cfg->BS_ANT_NUM, false);
            // cout<<"Precoder: "<< mat_precoder<<endl;
            // cout << "Raw data: " << mat_data.st() <<endl;

            cx_float* equal_ptr = nullptr;
            if (kExportConstellation) {
                equal_ptr
                    = (cx_float*)(&equal_buffer_[total_data_symbol_idx]
                                                [cur_sc_id * cfg->UE_NUM]);
            } else {
                equal_ptr
                    = (cx_float*)(&equaled_buffer_temp[(cur_sc_id - base_sc_id)
                        * cfg->UE_NUM]);
            }
            // Equalization
            cx_fmat mat_equaled(equal_ptr, cfg->UE_NUM, 1, false);
            size_t start_tsc2 = worker_rdtsc();
            mat_equaled = mat_precoder * mat_data;
            size_t start_tsc3 = worker_rdtsc();
            duration_stat->task_duration[2] += start_tsc3 - start_tsc2;
            // cout << "Equaled data sc "<<cur_sc_id<<": "<<mat_equaled.st();

            if (!kUseLDPC) {
                // Decode with hard decision
                uint8_t* demul_ptr
                    = (&demod_hard_buffer_[total_data_symbol_idx]
                                          [cur_sc_id * cfg->UE_NUM]);
                demod_16qam_hard_avx2(
                    (float*)equal_ptr, demul_ptr, cfg->UE_NUM);
                // cout<< "Demuled data:";
                // for (int ue_idx = 0; ue_idx < cfg->UE_NUM; ue_idx++)
                //     cout<<+*(demul_ptr+ue_idx)<<" ";
                // cout<<endl;
                // cout<<endl;
            }

            duration_stat->task_duration[3] += worker_rdtsc() - start_tsc3;
            duration_stat->task_count++;
        }
    }

    if (kUseLDPC) {
        __m256i index2 = _mm256_setr_epi32(0, 1, cfg->UE_NUM * 2,
            cfg->UE_NUM * 2 + 1, cfg->UE_NUM * 4, cfg->UE_NUM * 4 + 1,
            cfg->UE_NUM * 6, cfg->UE_NUM * 6 + 1);
        float* equal_T_ptr = (float*)(equaled_buffer_temp_transposed);
        for (size_t i = 0; i < cfg->UE_NUM; i++) {
            float* equal_ptr = (float*)(equaled_buffer_temp + i);
            int8_t* demul_ptr
                = (&demod_soft_buffer_[total_data_symbol_idx]
                                      [(cfg->OFDM_DATA_NUM * i + base_sc_id)
                                          * cfg->mod_type]);
            size_t kNumDoubleInSIMD256
                = sizeof(__m256) / sizeof(double); // == 4
            for (size_t j = 0; j < max_sc_ite / kNumDoubleInSIMD256; j++) {
                __m256 equal_T_temp = _mm256_i32gather_ps(equal_ptr, index2, 4);
                _mm256_store_ps(equal_T_ptr, equal_T_temp);
                equal_T_ptr += 8;
                equal_ptr += cfg->UE_NUM * kNumDoubleInSIMD256 * 2;
            }
            int num_sc_avx2 = (max_sc_ite / 16) * 16;
            int rest = max_sc_ite % 16;
            demod_16qam_soft_avx2(
                (equal_T_ptr - max_sc_ite * 2), demul_ptr, num_sc_avx2);
            if (rest > 0)
                demod_16qam_soft_sse(
                    (equal_T_ptr - max_sc_ite * 2 + num_sc_avx2 * 2),
                    demul_ptr + cfg->mod_type * num_sc_avx2, rest);
            /*
            printf("In doDemul thread %d: frame: %d, symbol: %d, sc_id: %d \n",
                tid, frame_id, current_data_symbol_id, sc_id);
            cout << "Demuled data : \n ";
            cout << " UE " << i << ": ";
            for (int k = 0; k < max_sc_ite * cfg->mod_order; k++)
                printf("%i ", demul_ptr[k]);
            cout << endl;
            */
        }
    }

    duration_stat->task_duration[0] += worker_rdtsc() - start_tsc;
    // if (duration > 500)
    //     printf("Thread %d Demul takes %.2f\n", tid, duration);

    return Event_data(EventType::kDemul, tag);
}
