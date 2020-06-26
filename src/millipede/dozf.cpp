/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#include "dozf.hpp"
#include "concurrent_queue_wrapper.hpp"
#include "doer.hpp"
#include <malloc.h>
#include "rpc_sock.hpp"

static constexpr bool kUseSIMDGather = true;

// Calculate the zeroforcing receiver using the formula W_zf = inv(H' * H) * H'.
// This is faster but less accurate than using an SVD-based pseudoinverse.
static constexpr size_t kUseInverseForZF = true;

extern RPCContext **ctx_list;

DoZF::DoZF(Config* config, int tid, double freq_ghz,
    moodycamel::ConcurrentQueue<Event_data>& task_queue,
    moodycamel::ConcurrentQueue<Event_data>& complete_task_queue,
    moodycamel::ProducerToken* worker_producer_token,
    Table<complex_float>& csi_buffer, Table<complex_float>& recip_buffer,
    Table<complex_float>& ul_zf_buffer, Table<complex_float>& dl_zf_buffer,
    Stats* stats_manager)
    : Doer(config, tid, freq_ghz, task_queue, complete_task_queue,
          worker_producer_token)
    , csi_buffer_(csi_buffer)
    , recip_buffer_(recip_buffer)
    , ul_zf_buffer_(ul_zf_buffer)
    , dl_zf_buffer_(dl_zf_buffer)
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

void DoZF::compute_precoder(const arma::cx_fmat& mat_csi,
    const complex_float* recip_ptr, complex_float* _mat_ul_zf,
    complex_float* _mat_dl_zf)
{
    arma::cx_fmat mat_ul_zf(reinterpret_cast<arma::cx_float*>(_mat_ul_zf),
        cfg->UE_NUM, cfg->BS_ANT_NUM, false);
    if (kUseInverseForZF) {
        try {
            mat_ul_zf = arma::inv_sympd(mat_csi.t() * mat_csi) * mat_csi.t();
        } catch (std::runtime_error) {
            MLPD_WARN(
                "Failed to invert channel matrix, falling back to pinv()\n");
            arma::pinv(mat_ul_zf, mat_csi, 1e-2, "dc");
        }
    } else {
        arma::pinv(mat_ul_zf, mat_csi, 1e-2, "dc");
    }

    if (cfg->dl_data_symbol_num_perframe > 0) {
        arma::cx_fmat mat_dl_zf(reinterpret_cast<arma::cx_float*>(_mat_dl_zf),
            cfg->UE_NUM, cfg->BS_ANT_NUM, false);
        if (cfg->recipCalEn) {
            auto* _recip_ptr = const_cast<arma::cx_float*>(
                reinterpret_cast<const arma::cx_float*>(recip_ptr));
            arma::cx_fvec vec_calib(_recip_ptr, cfg->BS_ANT_NUM, false);
            arma::cx_fmat mat_calib(cfg->BS_ANT_NUM, cfg->BS_ANT_NUM);
            mat_calib = arma::diagmat(vec_calib);
            mat_dl_zf = mat_ul_zf * arma::inv(mat_calib);
        } else
            mat_dl_zf = mat_ul_zf;
    }
}

void DoZF::ZF_time_orthogonal(size_t tag)
{
    const size_t frame_id = gen_tag_t(tag).frame_id;
    const size_t base_sc_id = gen_tag_t(tag).sc_id;
    const size_t frame_slot = frame_id % TASK_BUFFER_FRAME_NUM;
    if (kDebugPrintInTask) {
        printf("In doZF thread %d: frame: %zu, base subcarrier: %zu\n", tid,
            frame_id, base_sc_id);
    }
    size_t num_subcarriers
        = std::min(cfg->zf_block_size, cfg->OFDM_DATA_NUM - base_sc_id);

    // Handle each subcarrier one by one
    for (size_t i = 0; i < num_subcarriers; i++) {
        size_t start_tsc1 = worker_rdtsc();
        const size_t cur_sc_id = base_sc_id + i;

        // Gather CSI matrices of each pilot from partially-transposed CSIs.
        // The SIMD and non-SIMD methods are equivalent.
        if (kUseSIMDGather) {
            __m256i index = _mm256_setr_epi32(0, 1, kTransposeBlockSize * 2,
                kTransposeBlockSize * 2 + 1, kTransposeBlockSize * 4,
                kTransposeBlockSize * 4 + 1, kTransposeBlockSize * 6,
                kTransposeBlockSize * 6 + 1);
            int transpose_block_id = cur_sc_id / kTransposeBlockSize;
            int sc_inblock_idx = cur_sc_id % kTransposeBlockSize;
            int offset_in_csi_buffer
                = transpose_block_id * cfg->BS_ANT_NUM * kTransposeBlockSize
                + sc_inblock_idx;
            const size_t symbol_offset
                = (frame_id % TASK_BUFFER_FRAME_NUM) * cfg->UE_NUM;
            float* tar_csi_ptr = (float*)csi_gather_buffer;

            /* Gather csi matrix of all users and antennas */
            for (size_t ue_idx = 0; ue_idx < cfg->UE_NUM; ue_idx++) {
                float* src_csi_ptr = (float*)csi_buffer_[symbol_offset + ue_idx]
                    + offset_in_csi_buffer * 2;
                for (size_t ant_idx = 0; ant_idx < cfg->BS_ANT_NUM;
                     ant_idx += 4) {
                    /* Fetch 4 complex floats for 4 ants */
                    __m256 t_csi = _mm256_i32gather_ps(src_csi_ptr, index, 4);
                    _mm256_store_ps(tar_csi_ptr, t_csi);
                    src_csi_ptr += 8 * kTransposeBlockSize;
                    tar_csi_ptr += 8;
                }
            }
        } else {
            const size_t pt_base_offset = (cur_sc_id / kTransposeBlockSize)
                * (kTransposeBlockSize * cfg->BS_ANT_NUM);

            size_t gather_idx = 0;
            for (size_t p_i = 0; p_i < cfg->pilot_symbol_num_perframe; p_i++) {
                const complex_float* csi_buf
                    = csi_buffer_[(frame_slot * cfg->pilot_symbol_num_perframe)
                        + p_i];
                for (size_t ant_i = 0; ant_i < cfg->BS_ANT_NUM; ant_i++) {
                    csi_gather_buffer[gather_idx++]
                        = csi_buf[pt_base_offset + (ant_i * kTransposeBlockSize)
                            + (cur_sc_id % kTransposeBlockSize)];
                }
            }
        }

        duration_stat->task_duration[1] += worker_rdtsc() - start_tsc1;
        arma::cx_fmat mat_csi((arma::cx_float*)csi_gather_buffer,
            cfg->BS_ANT_NUM, cfg->UE_NUM, false);
        // cout<<"CSI matrix"<<endl;
        // cout<<mat_input.st()<<endl;
        compute_precoder(mat_csi,
            cfg->get_calib_buffer(recip_buffer_, frame_id, cur_sc_id),
            cfg->get_ul_zf_mat(ul_zf_buffer_, frame_id, cur_sc_id),
            cfg->get_dl_zf_mat(dl_zf_buffer_, frame_id, cur_sc_id));

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
    const size_t frame_id = gen_tag_t(tag).frame_id;
    const size_t base_sc_id = gen_tag_t(tag).sc_id;
    const size_t frame_slot = frame_id % TASK_BUFFER_FRAME_NUM;
    if (kDebugPrintInTask) {
        printf("In doZF thread %d: frame: %zu, subcarrier: %zu, block: %zu\n",
            tid, frame_id, base_sc_id, base_sc_id / cfg->UE_NUM);
    }

    double start_tsc1 = worker_rdtsc();

    // Gather CSIs from partially-transposed CSIs
    size_t gather_idx = 0;
    for (size_t i = 0; i < cfg->UE_NUM; i++) {
        const size_t cur_sc_id = base_sc_id + i;

        // The SIMD and non-SIMD methods are equivalent.
        if (kUseSIMDGather) {
            __m256i index = _mm256_setr_epi32(0, 1, kTransposeBlockSize * 2,
                kTransposeBlockSize * 2 + 1, kTransposeBlockSize * 4,
                kTransposeBlockSize * 4 + 1, kTransposeBlockSize * 6,
                kTransposeBlockSize * 6 + 1);

            int transpose_block_id = cur_sc_id / kTransposeBlockSize;
            int sc_inblock_idx = cur_sc_id % kTransposeBlockSize;
            int offset_in_csi_buffer
                = transpose_block_id * cfg->BS_ANT_NUM * kTransposeBlockSize
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
                src_csi_ptr += 8 * kTransposeBlockSize;
                tar_csi_ptr += 8;
            }
        } else {
            const size_t pt_base_offset = (cur_sc_id / kTransposeBlockSize)
                * (kTransposeBlockSize * cfg->BS_ANT_NUM);

            for (size_t ant_i = 0; ant_i < cfg->BS_ANT_NUM; ant_i++) {
                csi_gather_buffer[gather_idx++]
                    = csi_buffer_[frame_slot]
                                 [pt_base_offset + (ant_i * kTransposeBlockSize)
                                     + (cur_sc_id % kTransposeBlockSize)];
            }
        }
    }

    duration_stat->task_duration[1] += worker_rdtsc() - start_tsc1;
    arma::cx_fmat mat_csi(reinterpret_cast<arma::cx_float*>(csi_gather_buffer),
        cfg->BS_ANT_NUM, cfg->UE_NUM, false);
    // cout<<"CSI matrix"<<endl;
    // cout<<mat_input.st()<<endl;
    compute_precoder(mat_csi,
        cfg->get_calib_buffer(recip_buffer_, frame_id, base_sc_id),
        cfg->get_ul_zf_mat(ul_zf_buffer_, frame_id, base_sc_id),
        cfg->get_dl_zf_mat(dl_zf_buffer_, frame_id, base_sc_id));

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

    // Input matrix and calibration are for current frame, output precoders are
    // for the next frame
    compute_precoder(mat_input,
        cfg->get_calib_buffer(recip_buffer_, frame_id, base_sc_id),
        cfg->get_ul_zf_mat(ul_zf_buffer_, frame_id + 1, base_sc_id),
        cfg->get_dl_zf_mat(dl_zf_buffer_, frame_id + 1, base_sc_id));
}
