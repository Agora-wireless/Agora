#include "dyzf.hpp"
#include "concurrent_queue_wrapper.hpp"
#include "doer.hpp"
#include <malloc.h>

static constexpr bool kUseSIMDGather = true;
// Calculate the zeroforcing receiver using the formula W_zf = inv(H' * H) * H'.
// This is faster but less accurate than using an SVD-based pseudoinverse.
static constexpr size_t kUseInverseForZF = true;

DyZF::DyZF(Config* config, int tid, double freq_ghz,
    PtrGrid<kFrameWnd, kMaxUEs, complex_float>& csi_buffers,
    Table<complex_float>& calib_buffer,
    PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& ul_zf_matrices,
    PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& dl_zf_matrices,
    std::vector<std::vector<ControlInfo>>& control_info_table,
    std::vector<size_t>& control_idx_list)
    : Doer(config, tid, freq_ghz)
    , csi_buffers_(csi_buffers)
    , calib_buffer_(calib_buffer)
    , ul_zf_matrices_(ul_zf_matrices)
    , dl_zf_matrices_(dl_zf_matrices)
    , control_info_table_(control_info_table)
    , control_idx_list_(control_idx_list)
{
    csi_gather_buffer_ = reinterpret_cast<complex_float*>(
        memalign(64, kMaxAntennas * kMaxUEs * sizeof(complex_float)));
    calib_gather_buffer_ = reinterpret_cast<complex_float*>(
        memalign(64, kMaxAntennas * sizeof(complex_float)));
}

DyZF::~DyZF()
{
    free(csi_gather_buffer_);
    free(calib_gather_buffer_);
}

EventData DyZF::Launch(size_t tag)
{
    if (cfg_->freq_orthogonal_pilot) {
        ZFFreqOrthogonal(tag);
    } else {
        ZFTimeOrthogonal(tag);
    }
    return EventData(EventType::kZF, tag);
}

void DyZF::computePrecoder(const arma::cx_fmat& mat_csi,
    complex_float* calib_ptr, complex_float* _mat_ul_zf,
    complex_float* _mat_dl_zf, size_t ue_num)
{
    if (ue_num == 0) {
        ue_num = cfg_->UE_NUM;
    }
    arma::cx_fmat mat_ul_zf(reinterpret_cast<arma::cx_float*>(_mat_ul_zf),
        ue_num, cfg_->BS_ANT_NUM, false);
    if (kUseInverseForZF) {
        try {
            mat_ul_zf = arma::inv_sympd(mat_csi.t() * mat_csi) * mat_csi.t();
        } catch (std::runtime_error) {
            MLPD_WARN(
                "Failed to invert channel matrix, falling back to pinv()\n");
            // std::cout << mat_csi << std::endl;
            rt_assert(false);
            arma::pinv(mat_ul_zf, mat_csi, 1e-2, "dc");
        }
    } else {
        arma::pinv(mat_ul_zf, mat_csi, 1e-2, "dc");
    }

    if (cfg_->dl_data_symbol_num_perframe > 0) {
        arma::cx_fmat mat_dl_zf(reinterpret_cast<arma::cx_float*>(_mat_dl_zf),
            ue_num, cfg_->BS_ANT_NUM, false);
        if (cfg_->recipCalEn) {
            arma::cx_fvec vec_calib(
                reinterpret_cast<arma::cx_float*>(calib_ptr), cfg_->BS_ANT_NUM,
                false);

            vec_calib = vec_calib / vec_calib(cfg_->ref_ant);
            arma::cx_fmat mat_calib(cfg_->BS_ANT_NUM, cfg_->BS_ANT_NUM);
            mat_calib = arma::diagmat(vec_calib);
            mat_dl_zf = mat_ul_zf * arma::inv(mat_calib);
        } else
            mat_dl_zf = mat_ul_zf;
        // We should be scaling the beamforming matrix, so the IFFT
        // output can be scaled with OFDM_CA_NUM across all antennas.
        // See Argos paper (Mobicom 2012) Sec. 3.4 for details.
        mat_dl_zf /= abs(mat_dl_zf).max();
    }
}

void DyZF::computeULPrecoder(const arma::cx_fmat& mat_csi,
    complex_float* calib_ptr, complex_float* _mat_ul_zf)
{
    arma::cx_fmat mat_ul_zf(reinterpret_cast<arma::cx_float*>(_mat_ul_zf),
        cfg_->UE_NUM, cfg_->BS_ANT_NUM, false);
    if (kUseInverseForZF) {
        try {
            mat_ul_zf = arma::inv_sympd(mat_csi.t() * mat_csi) * mat_csi.t();
        } catch (std::runtime_error) {
            MLPD_WARN(
                "Failed to invert channel matrix, falling back to pinv()\n");
            // std::cout << mat_csi << std::endl;
            rt_assert(false);
            arma::pinv(mat_ul_zf, mat_csi, 1e-2, "dc");
        }
    } else {
        arma::pinv(mat_ul_zf, mat_csi, 1e-2, "dc");
    }
}

void DyZF::computeDLPrecoder(const arma::cx_fmat& mat_csi,
    complex_float* calib_ptr, complex_float* _mat_dl_zf)
{
    arma::cx_fmat mat_dl_zf(reinterpret_cast<arma::cx_float*>(_mat_dl_zf),
        cfg_->UE_NUM, cfg_->BS_ANT_NUM, false);
    if (kUseInverseForZF) {
        try {
            mat_dl_zf = arma::inv_sympd(mat_csi.t() * mat_csi) * mat_csi.t();
        } catch (std::runtime_error) {
            MLPD_WARN(
                "Failed to invert channel matrix, falling back to pinv()\n");
            // std::cout << mat_csi << std::endl;
            rt_assert(false);
            arma::pinv(mat_dl_zf, mat_csi, 1e-2, "dc");
        }
    } else {
        arma::pinv(mat_dl_zf, mat_csi, 1e-2, "dc");
    }
}

// Gather data of one symbol from partially-transposed buffer
// produced by dofft
static inline void partialTransposeGather(
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

void DyZF::ZFTimeOrthogonal(size_t tag)
{
    const size_t frame_id = gen_tag_t(tag).frame_id;
    const size_t base_sc_id = gen_tag_t(tag).sc_id;
    const size_t frame_slot = frame_id % TASK_BUFFER_FRAME_NUM;
    if (kDebugPrintInTask) {
        printf("In doZF thread %d: frame: %zu, base subcarrier: %zu\n", tid_,
            frame_id, base_sc_id);
    }
    // const size_t num_subcarriers = std::min(
    //     cfg_->zf_block_size, (cfg_->bs_server_addr_idx + 1) * cfg_->get_num_sc_per_server() - base_sc_id);
    const size_t num_subcarriers = std::min(
        cfg_->zf_block_size, cfg_->subcarrier_end - base_sc_id);

    // Handle each subcarrier one by one
    for (size_t i = 0; i < num_subcarriers; i++) {
        const size_t cur_sc_id = base_sc_id + i;

        // Gather CSI matrices of each pilot from partially-transposed CSIs.
        for (size_t ue_idx = 0; ue_idx < cfg_->UE_NUM; ue_idx++) {
            float* dst_csi_ptr
                = (float*)(csi_gather_buffer_ + cfg_->BS_ANT_NUM * ue_idx);
            partialTransposeGather(cur_sc_id,
                (float*)csi_buffers_[frame_slot][ue_idx], dst_csi_ptr,
                cfg_->BS_ANT_NUM);
        }
        if (cfg_->recipCalEn) {
            // Gather reciprocal calibration data from partially-transposed buffer
            float* dst_calib_ptr = (float*)calib_gather_buffer_;
            partialTransposeGather(cur_sc_id,
                (float*)calib_buffer_[frame_slot], dst_calib_ptr,
                cfg_->BS_ANT_NUM);
        }

        arma::cx_fmat mat_csi((arma::cx_float*)csi_gather_buffer_,
            cfg_->BS_ANT_NUM, cfg_->UE_NUM, false);

        computePrecoder(mat_csi, calib_gather_buffer_,
            ul_zf_matrices_[frame_slot][cur_sc_id],
            dl_zf_matrices_[frame_slot][cur_sc_id]);
    }
}

void DyZF::ZFFreqOrthogonal(size_t tag)
{
    const size_t frame_id = gen_tag_t(tag).frame_id;
    const size_t base_sc_id = gen_tag_t(tag).sc_id;
    const size_t frame_slot = frame_id % TASK_BUFFER_FRAME_NUM;
    if (kDebugPrintInTask) {
        printf("In doZF thread %d: frame: %zu, subcarrier: %zu, block: %zu, "
               "BS_ANT_NUM: %zu\n",
            tid_, frame_id, base_sc_id, base_sc_id / cfg_->UE_NUM,
            cfg_->BS_ANT_NUM);
    }

    double start_tsc1 = worker_rdtsc();

    // Gather CSIs from partially-transposed CSIs
    std::vector<ControlInfo>& info_list = control_info_table_[control_idx_list_[frame_id]];
    size_t total_ue_sc = 0;
    for (size_t i = 0; i < info_list.size(); i ++) {
        size_t ue_id = info_list[i].ue_id;
        if (info_list[i].sc_start > base_sc_id || info_list[i].sc_end <= base_sc_id) {
            continue;
        }
        const size_t cur_sc_id = base_sc_id + ue_id;
        float* dst_csi_ptr = (float*)(csi_gather_buffer_ + cfg_->BS_ANT_NUM * total_ue_sc);
        partialTransposeGather(cur_sc_id, (float*)csi_buffers_[frame_slot][0],
            dst_csi_ptr, cfg_->BS_ANT_NUM);
        total_ue_sc ++;
    }
    if (cfg_->recipCalEn) {
        // Gather reciprocal calibration data from partially-transposed buffer
        float* dst_calib_ptr = (float*)calib_gather_buffer_;
        partialTransposeGather(base_sc_id, (float*)calib_buffer_[frame_slot],
            dst_calib_ptr, cfg_->BS_ANT_NUM);
    }

    if (total_ue_sc == 0) {
        return;
    }

    arma::cx_fmat mat_csi(reinterpret_cast<arma::cx_float*>(csi_gather_buffer_),
        cfg_->BS_ANT_NUM, total_ue_sc, false);

    computePrecoder(mat_csi, calib_gather_buffer_,
        ul_zf_matrices_[frame_slot][cfg_->get_zf_sc_id(base_sc_id)],
        dl_zf_matrices_[frame_slot][cfg_->get_zf_sc_id(base_sc_id)],
        total_ue_sc);

    double start_tsc2 = worker_rdtsc();
    zf_tsc_ += start_tsc2 - start_tsc1;
    zf_count_ ++;
}

void DyZF::ZFFreqOrthogonalStatic(size_t tag)
{
    const size_t frame_id = gen_tag_t(tag).frame_id;
    const size_t base_sc_id = gen_tag_t(tag).sc_id;
    const size_t frame_slot = frame_id % TASK_BUFFER_FRAME_NUM;
    if (kDebugPrintInTask) {
        printf("In doZF thread %d: frame: %zu, subcarrier: %zu, block: %zu, "
               "BS_ANT_NUM: %zu\n",
            tid_, frame_id, base_sc_id, base_sc_id / cfg_->UE_NUM,
            cfg_->BS_ANT_NUM);
    }

    double start_tsc1 = worker_rdtsc();

    // Gather CSIs from partially-transposed CSIs
    for (size_t ue_id = 0; ue_id < cfg_->UE_NUM; ue_id ++) {
        const size_t cur_sc_id = base_sc_id + ue_id;
        float* dst_csi_ptr = (float*)(csi_gather_buffer_ + cfg_->BS_ANT_NUM * ue_id);
        partialTransposeGather(cur_sc_id, (float*)csi_buffers_[frame_slot][0],
            dst_csi_ptr, cfg_->BS_ANT_NUM);
    }
    if (cfg_->recipCalEn) {
        // Gather reciprocal calibration data from partially-transposed buffer
        float* dst_calib_ptr = (float*)calib_gather_buffer_;
        partialTransposeGather(base_sc_id, (float*)calib_buffer_[frame_slot],
            dst_calib_ptr, cfg_->BS_ANT_NUM);
    }

    arma::cx_fmat mat_csi(reinterpret_cast<arma::cx_float*>(csi_gather_buffer_),
        cfg_->BS_ANT_NUM, cfg_->UE_NUM, false);

    if (cfg_->downlink_mode) {
        computeDLPrecoder(mat_csi, calib_gather_buffer_,
            dl_zf_matrices_[frame_slot][cfg_->get_zf_sc_id(base_sc_id)]);
    } else {
        computeULPrecoder(mat_csi, calib_gather_buffer_,
            ul_zf_matrices_[frame_slot][cfg_->get_zf_sc_id(base_sc_id)]);
    }

    double start_tsc2 = worker_rdtsc();
    zf_tsc_ += start_tsc2 - start_tsc1;
    zf_count_ ++;
}
