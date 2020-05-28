/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#include "doprecode.hpp"
#include "concurrent_queue_wrapper.hpp"

using namespace arma;

DoPrecode::DoPrecode(Config* in_config, int in_tid, double freq_ghz,
    moodycamel::ConcurrentQueue<Event_data>& in_task_queue,
    moodycamel::ConcurrentQueue<Event_data>& complete_task_queue,
    moodycamel::ProducerToken* worker_producer_token,
    Table<complex_float>& in_precoder_buffer,
    Table<complex_float>& in_dl_ifft_buffer,
#ifdef USE_LDPC
    Table<int8_t>& in_dl_encoded_data,
#else
    Table<int8_t>& in_dl_raw_data,
#endif
    Stats* in_stats_manager)
    : Doer(in_config, in_tid, freq_ghz, in_task_queue, complete_task_queue,
          worker_producer_token)
    , precoder_buffer_(in_precoder_buffer)
    , dl_ifft_buffer_(in_dl_ifft_buffer)
#ifdef USE_LDPC
    , dl_raw_data(in_dl_encoded_data)
#else
    , dl_raw_data(in_dl_raw_data)
#endif
{
    duration_stat
        = in_stats_manager->get_duration_stat(DoerType::kPrecode, in_tid);
    init_modulation_table(qam_table, cfg->mod_type);

    alloc_buffer_1d(&modulated_buffer_temp, cfg->UE_NUM, 64, 0);
    alloc_buffer_1d(
        &precoded_buffer_temp, cfg->demul_block_size * cfg->BS_ANT_NUM, 64, 0);
}

DoPrecode::~DoPrecode()
{
    free_buffer_1d(&modulated_buffer_temp);
    free_buffer_1d(&precoded_buffer_temp);
}

Event_data DoPrecode::launch(size_t tag)
{
    size_t frame_id = gen_tag_t(tag).frame_id;
    size_t base_sc_id = gen_tag_t(tag).sc_id;
    size_t data_symbol_idx_dl = gen_tag_t(tag).symbol_id;
    size_t total_data_symbol_idx
        = cfg->get_total_data_symbol_idx(frame_id, data_symbol_idx_dl);

    size_t start_tsc = worker_rdtsc();
    if (kDebugPrintInTask) {
        printf("In doPrecode TID %d: frame %zu, symbol %zu, subcarrier %zu\n",
            tid, frame_id, data_symbol_idx_dl, base_sc_id);
    }

    __m256i index = _mm256_setr_epi64x(
        0, cfg->BS_ANT_NUM, cfg->BS_ANT_NUM * 2, cfg->BS_ANT_NUM * 3);
    int max_sc_ite
        = std::min(cfg->demul_block_size, cfg->OFDM_DATA_NUM - base_sc_id);

    for (int i = 0; i < max_sc_ite; i = i + 4) {
        size_t start_tsc1 = worker_rdtsc();
        for (int j = 0; j < 4; j++) {
            size_t start_tsc2 = worker_rdtsc();
            int cur_sc_id = base_sc_id + i + j;
            int precoder_offset
                = ((frame_id % TASK_BUFFER_FRAME_NUM) * cfg->OFDM_DATA_NUM)
                + cur_sc_id;
            if (cfg->freq_orthogonal_pilot)
                precoder_offset = precoder_offset - (cur_sc_id % cfg->UE_NUM);

            complex_float* data_ptr = modulated_buffer_temp;
            if (data_symbol_idx_dl
                <= cfg->dl_data_symbol_start - 1 + cfg->DL_PILOT_SYMS) {
                for (size_t user_id = 0; user_id < cfg->UE_NUM; user_id++)
                    data_ptr[user_id]
                        = cfg->ue_specific_pilot[user_id][cur_sc_id];
            } else {
                int symbol_id_in_buffer
                    = data_symbol_idx_dl - cfg->dl_data_symbol_start;

                for (size_t user_id = 0; user_id < cfg->UE_NUM; user_id++) {
                    int8_t* raw_data_ptr
                        = &dl_raw_data[kUseLDPC ? total_data_symbol_idx
                                                : symbol_id_in_buffer][cur_sc_id
                            + cfg->OFDM_DATA_NUM * user_id];
                    if (cur_sc_id % cfg->OFDM_PILOT_SPACING == 0)
                        data_ptr[user_id]
                            = cfg->ue_specific_pilot[user_id][cur_sc_id];
                    else
                        data_ptr[user_id] = mod_single_uint8(
                            (uint8_t) * (raw_data_ptr), qam_table);
                }
            }

            cx_float* precoder_ptr
                = (cx_float*)precoder_buffer_[precoder_offset];
            cx_fmat mat_precoder(
                precoder_ptr, cfg->UE_NUM, cfg->BS_ANT_NUM, false);
            cx_fmat mat_data((cx_float*)data_ptr, 1, cfg->UE_NUM, false);
            cx_float* precoded_ptr
                = (cx_float*)precoded_buffer_temp + (i + j) * cfg->BS_ANT_NUM;
            cx_fmat mat_precoded(precoded_ptr, 1, cfg->BS_ANT_NUM, false);

            duration_stat->task_duration[1] += worker_rdtsc() - start_tsc2;
            mat_precoded = mat_data * mat_precoder;

            // printf("In doPrecode thread %d: frame: %d, symbol: %d, "
            //        "subcarrier: % d\n ",
            //     tid, frame_id, current_data_symbol_id, cur_sc_id);
            // cout << "Precoder: \n" << mat_precoder << endl;
            // cout << "Data: \n" << mat_data << endl;
            // cout << "Precoded data: \n" << mat_precoded << endl;
            duration_stat->task_count++;
        }
        duration_stat->task_duration[2] += worker_rdtsc() - start_tsc1;
    }

    size_t start_tsc3 = worker_rdtsc();

    float* precoded_ptr = (float*)precoded_buffer_temp;
    for (size_t ant_id = 0; ant_id < cfg->BS_ANT_NUM; ant_id++) {
        int ifft_buffer_offset
            = ant_id + cfg->BS_ANT_NUM * total_data_symbol_idx;
        float* ifft_ptr
            = (float*)&dl_ifft_buffer_[ifft_buffer_offset]
                                      [base_sc_id + cfg->OFDM_DATA_START];
        for (size_t i = 0; i < cfg->demul_block_size / 4; i++) {
            float* input_shifted_ptr
                = precoded_ptr + 4 * i * 2 * cfg->BS_ANT_NUM + ant_id * 2;
            __m256d t_data
                = _mm256_i64gather_pd((double*)input_shifted_ptr, index, 8);
            _mm256_stream_pd((double*)(ifft_ptr + i * 8), t_data);
        }
    }
    duration_stat->task_duration[3] += worker_rdtsc() - start_tsc3;
    duration_stat->task_duration[0] += worker_rdtsc() - start_tsc;
    // duration_stat->task_count++;

    if (kDebugPrintInTask) {
        printf("In doPrecode thread %d: finished frame: %zu, symbol: %zu, "
               "subcarrier: %zu\n",
            tid, frame_id, data_symbol_idx_dl, base_sc_id);
    }
    return Event_data(EventType::kPrecode, tag);
}
