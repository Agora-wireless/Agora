#include "dozf.hpp"
#include "concurrent_queue_wrapper.hpp"
#include "doer.hpp"

static constexpr bool kUseSIMDGather = true;
// Calculate the zeroforcing receiver using the formula W_zf = inv(H' * H) * H'.
// This is faster but less accurate than using an SVD-based pseudoinverse.
static constexpr size_t kUseInverseForZF = true;

DoZF::DoZF(Config* config, int tid,
    PtrGrid<kFrameWnd, kMaxUEs, complex_float>& csi_buffers,
    Table<complex_float>& calib_dl_buffer,
    Table<complex_float>& calib_ul_buffer,
    PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& ul_zf_matrices,
    PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& dl_zf_matrices,
    Stats* stats_manager)
    : Doer(config, tid)
    , csi_buffers_(csi_buffers)
    , calib_dl_buffer_(calib_dl_buffer)
    , calib_ul_buffer_(calib_ul_buffer)
    , ul_zf_matrices_(ul_zf_matrices)
    , dl_zf_matrices_(dl_zf_matrices)
{
    duration_stat = stats_manager->get_duration_stat(DoerType::kZF, tid);
    pred_csi_buffer = static_cast<complex_float*>(
        Agora_memory::padded_aligned_alloc(Agora_memory::Alignment_t::k64Align,
            kMaxAntennas * kMaxUEs * sizeof(complex_float)));
    csi_gather_buffer = static_cast<complex_float*>(
        Agora_memory::padded_aligned_alloc(Agora_memory::Alignment_t::k64Align,
            kMaxAntennas * kMaxUEs * sizeof(complex_float)));
    calib_gather_buffer = static_cast<complex_float*>(
        Agora_memory::padded_aligned_alloc(Agora_memory::Alignment_t::k64Align,
            kMaxAntennas * sizeof(complex_float)));
}

DoZF::~DoZF()
{
    std::free(pred_csi_buffer);
    std::free(csi_gather_buffer);
    std::free(calib_gather_buffer);
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
    complex_float* calib_ptr, complex_float* _mat_ul_zf,
    complex_float* _mat_dl_zf)
{
    arma::cx_fmat mat_ul_zf(reinterpret_cast<arma::cx_float*>(_mat_ul_zf),
        cfg->UE_NUM, cfg->BS_ANT_NUM, false);
    arma::cx_fmat mat_ul_zf_tmp;
    if (kUseInverseForZF) {
        try {
            mat_ul_zf_tmp
                = arma::inv_sympd(mat_csi.t() * mat_csi) * mat_csi.t();
        } catch (std::runtime_error&) {
            MLPD_WARN(
                "Failed to invert channel matrix, falling back to pinv()\n");
            arma::pinv(mat_ul_zf_tmp, mat_csi, 1e-2, "dc");
        }
    } else {
        arma::pinv(mat_ul_zf_tmp, mat_csi, 1e-2, "dc");
    }

    if (cfg->dl_data_symbol_num_perframe > 0) {
        arma::cx_fvec vec_calib(reinterpret_cast<arma::cx_float*>(calib_ptr),
            cfg->BF_ANT_NUM, false);

        arma::cx_fmat mat_csi_dl = arma::diagmat(vec_calib) * mat_csi;
        arma::cx_fmat mat_dl_zf_tmp;
        try {
            mat_dl_zf_tmp
                = arma::inv_sympd(mat_csi_dl.t() * mat_csi_dl) * mat_csi_dl.t();
        } catch (std::runtime_error&) {
            MLPD_WARN("Failed to invert reference channel matrix, skip "
                      "applying it\n");
            Utils::print_vec(vec_calib, "vec_calib");
            mat_dl_zf_tmp = mat_ul_zf_tmp;
        }

        // We should be scaling the beamforming matrix, so the IFFT
        // output can be scaled with OFDM_CA_NUM across all antennas.
        // See Argos paper (Mobicom 2012) Sec. 3.4 for details.
        float scale = 1 / (abs(mat_dl_zf_tmp).max());
        mat_dl_zf_tmp *= scale;

        if (cfg->external_ref_node) {
            mat_dl_zf_tmp.insert_cols(cfg->ref_ant,
                arma::cx_fmat(cfg->UE_NUM, cfg->nChannels, arma::fill::zeros));
        }
        arma::cx_fmat mat_dl_zf(reinterpret_cast<arma::cx_float*>(_mat_dl_zf),
            cfg->BS_ANT_NUM, cfg->UE_NUM, false);
        mat_dl_zf = mat_dl_zf_tmp.st();
    }
    if (cfg->external_ref_node) {
        mat_ul_zf_tmp.insert_cols(cfg->ref_ant,
            arma::cx_fmat(cfg->UE_NUM, cfg->nChannels, arma::fill::zeros));
    }
    mat_ul_zf = mat_ul_zf_tmp;
}

// Gather data of one symbol from partially-transposed buffer
// produced by dofft
static inline void partial_transpose_gather(
    size_t cur_sc_id, float* src, float*& dst, size_t bs_ant_num)
{
    // The SIMD and non-SIMD methods are equivalent.
    size_t ant_start = 0;
    if (kUseSIMDGather and bs_ant_num >= 4) {
        __m256i index = _mm256_setr_epi32(0, 1, kTransposeBlockSize * 2,
            kTransposeBlockSize * 2 + 1, kTransposeBlockSize * 4,
            kTransposeBlockSize * 4 + 1, kTransposeBlockSize * 6,
            kTransposeBlockSize * 6 + 1);

        const size_t transpose_block_id = cur_sc_id / kTransposeBlockSize;
        const size_t sc_inblock_idx = cur_sc_id % kTransposeBlockSize;
        const size_t offset_in_src_buffer
            = transpose_block_id * bs_ant_num * kTransposeBlockSize
            + sc_inblock_idx;

        src = src + offset_in_src_buffer * 2;
        for (size_t ant_idx = 0; ant_idx < bs_ant_num; ant_idx += 4) {
            // fetch 4 complex floats for 4 ants
            __m256 t = _mm256_i32gather_ps(src, index, 4);
            _mm256_storeu_ps(dst, t);
            src += 8 * kTransposeBlockSize;
            dst += 8;
        }
        // Set the of the remaining antennas to use non-SIMD gather
        ant_start = bs_ant_num / 4 * 4;
    }
    if (ant_start < bs_ant_num) {
        const size_t pt_base_offset = (cur_sc_id / kTransposeBlockSize)
            * (kTransposeBlockSize * bs_ant_num);
        complex_float* cx_src = (complex_float*)src;
        complex_float* cx_dst = (complex_float*)dst + ant_start;
        for (size_t ant_i = ant_start; ant_i < bs_ant_num; ant_i++) {
            *cx_dst = cx_src[pt_base_offset + (ant_i * kTransposeBlockSize)
                + (cur_sc_id % kTransposeBlockSize)];
            cx_dst++;
        }
    }
}

// Gather data of one symbol from partially-transposed buffer
// produced by dofft
static inline void transpose_gather(size_t cur_sc_id, float* src, float*& dst,
    size_t bs_ant_num, size_t ofdm_data_num)
{
    complex_float* cx_src = (complex_float*)src;
    complex_float* cx_dst = (complex_float*)dst;
    for (size_t ant_i = 0; ant_i < bs_ant_num; ant_i++) {
        *cx_dst = cx_src[ant_i * ofdm_data_num + cur_sc_id];
        cx_dst++;
    }
}

void DoZF::ZF_time_orthogonal(size_t tag)
{
    const size_t frame_id = gen_tag_t(tag).frame_id;
    const size_t base_sc_id = gen_tag_t(tag).sc_id;
    const size_t frame_slot = frame_id % kFrameWnd;
    if (kDebugPrintInTask) {
        std::printf("In doZF thread %d: frame: %zu, base subcarrier: %zu\n",
            tid, frame_id, base_sc_id);
    }
    size_t num_subcarriers
        = std::min(cfg->zf_block_size, cfg->OFDM_DATA_NUM - base_sc_id);

    // Handle each subcarrier one by one
    for (size_t i = 0; i < num_subcarriers; i++) {
        size_t start_tsc1 = worker_rdtsc();
        const size_t cur_sc_id = base_sc_id + i;

        // Gather CSI matrices of each pilot from partially-transposed CSIs.
        for (size_t ue_idx = 0; ue_idx < cfg->UE_NUM; ue_idx++) {
            float* dst_csi_ptr
                = (float*)(csi_gather_buffer + cfg->BS_ANT_NUM * ue_idx);
            if (kUsePartialTrans)
                partial_transpose_gather(cur_sc_id,
                    (float*)csi_buffers_[frame_slot][ue_idx], dst_csi_ptr,
                    cfg->BS_ANT_NUM);
            else
                transpose_gather(cur_sc_id,
                    (float*)csi_buffers_[frame_slot][ue_idx], dst_csi_ptr,
                    cfg->BS_ANT_NUM, cfg->OFDM_DATA_NUM);
        }

        duration_stat->task_duration[1] += worker_rdtsc() - start_tsc1;
        arma::cx_fmat mat_csi((arma::cx_float*)csi_gather_buffer,
            cfg->BS_ANT_NUM, cfg->UE_NUM, false);

        if (cfg->dl_data_symbol_num_perframe > 0) {
            arma::cx_fvec calib_vec(
                reinterpret_cast<arma::cx_float*>(calib_gather_buffer),
                cfg->BF_ANT_NUM, false);
            size_t frame_cal_slot = kFrameWnd - 1;
            size_t frame_cal_slot_prev = kFrameWnd - 1;
            if (cfg->recipCalEn && frame_id >= TX_FRAME_DELTA) {
                size_t frame_grp_id
                    = (frame_id - TX_FRAME_DELTA) / cfg->ant_group_num;

                // use the previous window which has a full set of calibration results
                frame_cal_slot = (frame_grp_id + kFrameWnd - 1) % kFrameWnd;
                if (frame_id >= TX_FRAME_DELTA + cfg->ant_group_num)
                    frame_cal_slot_prev
                        = (frame_grp_id + kFrameWnd - 2) % kFrameWnd;
            }
            arma::cx_fmat calib_dl_mat(reinterpret_cast<arma::cx_float*>(
                                           calib_dl_buffer_[frame_cal_slot]),
                cfg->OFDM_DATA_NUM, cfg->BF_ANT_NUM, false);
            arma::cx_fmat calib_ul_mat(reinterpret_cast<arma::cx_float*>(
                                           calib_ul_buffer_[frame_cal_slot]),
                cfg->OFDM_DATA_NUM, cfg->BF_ANT_NUM, false);
            arma::cx_fmat calib_dl_mat_prev(
                reinterpret_cast<arma::cx_float*>(
                    calib_dl_buffer_[frame_cal_slot_prev]),
                cfg->OFDM_DATA_NUM, cfg->BF_ANT_NUM, false);
            arma::cx_fmat calib_ul_mat_prev(
                reinterpret_cast<arma::cx_float*>(
                    calib_ul_buffer_[frame_cal_slot_prev]),
                cfg->OFDM_DATA_NUM, cfg->BF_ANT_NUM, false);
            arma::cx_fmat calib_dl_mat_mean
                = (calib_dl_mat + calib_dl_mat_prev) / 2;
            arma::cx_fmat calib_ul_mat_mean
                = (calib_ul_mat + calib_ul_mat_prev) / 2;
            arma::cx_fvec calib_dl_vec = calib_dl_mat_mean.row(cur_sc_id).st();
            arma::cx_fvec calib_ul_vec = calib_ul_mat_mean.row(cur_sc_id).st();
            calib_vec = calib_dl_vec / calib_ul_vec;

            if (cfg->external_ref_node) {
                mat_csi.shed_rows(
                    cfg->ref_ant, cfg->ref_ant + cfg->nChannels - 1);
            }
        }

        double start_tsc2 = worker_rdtsc();
        duration_stat->task_duration[2] += start_tsc2 - start_tsc1;
        compute_precoder(mat_csi, calib_gather_buffer,
            ul_zf_matrices_[frame_slot][cur_sc_id],
            dl_zf_matrices_[frame_slot][cur_sc_id]);

        // cout<<"Precoder:" <<mat_output<<endl;
        double duration3 = worker_rdtsc() - start_tsc2;
        duration_stat->task_duration[3] += duration3;
        duration_stat->task_count++;
        duration_stat->task_duration[0] += worker_rdtsc() - start_tsc1;
        // if (duration > 500) {
        //     std::printf("Thread %d ZF takes %.2f\n", tid, duration);
        // }
    }
}

void DoZF::ZF_freq_orthogonal(size_t tag)
{
    const size_t frame_id = gen_tag_t(tag).frame_id;
    const size_t base_sc_id = gen_tag_t(tag).sc_id;
    const size_t frame_slot = frame_id % kFrameWnd;
    if (kDebugPrintInTask) {
        std::printf(
            "In doZF thread %d: frame: %zu, subcarrier: %zu, block: %zu, "
            "BS_ANT_NUM: %zu\n",
            tid, frame_id, base_sc_id, base_sc_id / cfg->UE_NUM,
            cfg->BS_ANT_NUM);
    }

    double start_tsc1 = worker_rdtsc();

    // Gather CSIs from partially-transposed CSIs
    for (size_t i = 0; i < cfg->UE_NUM; i++) {
        const size_t cur_sc_id = base_sc_id + i;
        float* dst_csi_ptr = (float*)(csi_gather_buffer + cfg->BS_ANT_NUM * i);
        partial_transpose_gather(cur_sc_id, (float*)csi_buffers_[frame_slot][0],
            dst_csi_ptr, cfg->BS_ANT_NUM);
    }
    if (cfg->dl_data_symbol_num_perframe > 0) {
        arma::cx_fvec calib_vec(
            reinterpret_cast<arma::cx_float*>(calib_gather_buffer),
            cfg->BF_ANT_NUM, false);
        size_t frame_cal_slot = kFrameWnd - 1;
        size_t frame_cal_slot_prev = kFrameWnd - 1;
        if (cfg->recipCalEn && frame_id >= TX_FRAME_DELTA) {
            size_t frame_grp_id
                = (frame_id - TX_FRAME_DELTA) / cfg->ant_group_num;

            // use the previous window which has a full set of calibration results
            frame_cal_slot = (frame_grp_id + kFrameWnd - 1) % kFrameWnd;
            if (frame_id >= TX_FRAME_DELTA + cfg->ant_group_num)
                frame_cal_slot_prev
                    = (frame_grp_id + kFrameWnd - 2) % kFrameWnd;
        }
        arma::cx_fmat calib_dl_mat(
            reinterpret_cast<arma::cx_float*>(calib_dl_buffer_[frame_cal_slot]),
            cfg->OFDM_DATA_NUM, cfg->BF_ANT_NUM, false);
        arma::cx_fvec calib_ul_mat(
            reinterpret_cast<arma::cx_float*>(calib_ul_buffer_[frame_cal_slot]),
            cfg->OFDM_DATA_NUM, cfg->BF_ANT_NUM, false);

        arma::cx_fmat calib_dl_mat_prev(
            reinterpret_cast<arma::cx_float*>(
                calib_dl_buffer_[frame_cal_slot_prev]),
            cfg->OFDM_DATA_NUM, cfg->BF_ANT_NUM, false);
        arma::cx_fmat calib_ul_mat_prev(
            reinterpret_cast<arma::cx_float*>(
                calib_ul_buffer_[frame_cal_slot_prev]),
            cfg->OFDM_DATA_NUM, cfg->BF_ANT_NUM, false);
        arma::cx_fmat calib_dl_mat_mean
            = (calib_dl_mat + calib_dl_mat_prev) / 2;
        arma::cx_fmat calib_ul_mat_mean
            = (calib_ul_mat + calib_ul_mat_prev) / 2;
        arma::cx_fvec calib_dl_vec = calib_dl_mat_mean.row(base_sc_id).st();
        arma::cx_fvec calib_ul_vec = calib_ul_mat_mean.row(base_sc_id).st();
        calib_vec = calib_dl_vec / calib_ul_vec;
    }

    duration_stat->task_duration[1] += worker_rdtsc() - start_tsc1;
    arma::cx_fmat mat_csi(reinterpret_cast<arma::cx_float*>(csi_gather_buffer),
        cfg->BS_ANT_NUM, cfg->UE_NUM, false);

    compute_precoder(mat_csi, calib_gather_buffer,
        ul_zf_matrices_[frame_slot][cfg->get_zf_sc_id(base_sc_id)],
        dl_zf_matrices_[frame_slot][cfg->get_zf_sc_id(base_sc_id)]);

    double start_tsc2 = worker_rdtsc();
    duration_stat->task_duration[2] += start_tsc2 - start_tsc1;

    // cout<<"Precoder:" <<mat_output<<endl;
    duration_stat->task_duration[3] += worker_rdtsc() - start_tsc2;
    duration_stat->task_count++;
    duration_stat->task_duration[0] += worker_rdtsc() - start_tsc1;

    // if (duration > 500) {
    //     std::printf("Thread %d ZF takes %.2f\n", tid, duration);
    // }
}

// Currently unused
/*
void DoZF::Predict(size_t tag)
{
    size_t frame_id = gen_tag_t(tag).frame_id;
    size_t base_sc_id = gen_tag_t(tag).sc_id;

    // Use stale CSI as predicted CSI
    // TODO: add prediction algorithm
    const size_t offset_in_buffer
        = ((frame_id % kFrameWnd) * cfg->OFDM_DATA_NUM)
        + base_sc_id;
    auto* ptr_in = (arma::cx_float*)pred_csi_buffer;
    std::memcpy(ptr_in, (arma::cx_float*)csi_buffer_[offset_in_buffer],
        sizeof(arma::cx_float) * cfg->BS_ANT_NUM * cfg->UE_NUM);
    arma::cx_fmat mat_input(ptr_in, cfg->BS_ANT_NUM, cfg->UE_NUM, false);

    // Input matrix and calibration are for current frame, output precoders are
    // for the next frame
    compute_precoder(mat_input,
        cfg->get_calib_buffer(calib_buffer_, frame_id, base_sc_id),
        cfg->get_ul_zf_mat(ul_zf_buffer_, frame_id + 1, base_sc_id),
        cfg->get_dl_zf_mat(dl_zf_buffer_, frame_id + 1, base_sc_id));
}
*/
