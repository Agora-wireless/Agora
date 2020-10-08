#include "dofft.hpp"
#include "concurrent_queue_wrapper.hpp"
#include "datatype_conversion.h"
#include <malloc.h>

using namespace arma;

static constexpr bool kPrintFFTInput = false;

DoFFT::DoFFT(Config* config, int tid, double freq_ghz,
    moodycamel::ConcurrentQueue<Event_data>& task_queue,
    moodycamel::ConcurrentQueue<Event_data>& complete_task_queue,
    moodycamel::ProducerToken* worker_producer_token,
    Table<char>& socket_buffer, Table<int>& socket_buffer_status,
    Table<complex_float>& data_buffer,
    PtrGrid<TASK_BUFFER_FRAME_NUM, kMaxUEs, complex_float>& csi_buffers,
    Table<complex_float>& calib_buffer, PhyStats* in_phy_stats,
    Stats* stats_manager)
    : Doer(config, tid, freq_ghz, task_queue, complete_task_queue,
          worker_producer_token)
    , socket_buffer_(socket_buffer)
    , socket_buffer_status_(socket_buffer_status)
    , data_buffer_(data_buffer)
    , csi_buffers_(csi_buffers)
    , calib_buffer_(calib_buffer)
    , phy_stats(in_phy_stats)
{
    duration_stat_fft = stats_manager->get_duration_stat(DoerType::kFFT, tid);
    duration_stat_csi = stats_manager->get_duration_stat(DoerType::kCSI, tid);
    DftiCreateDescriptor(
        &mkl_handle, DFTI_SINGLE, DFTI_COMPLEX, 1, cfg->OFDM_CA_NUM);
    DftiCommitDescriptor(mkl_handle);

    // Aligned for SIMD
    fft_inout = reinterpret_cast<complex_float*>(
        memalign(64, cfg->OFDM_CA_NUM * sizeof(complex_float)));
}

DoFFT::~DoFFT()
{
    DftiFreeDescriptor(&mkl_handle);
    free(fft_inout);
}

Event_data DoFFT::launch(size_t tag)
{
    size_t socket_thread_id = fft_req_tag_t(tag).tid;
    size_t buf_offset = fft_req_tag_t(tag).offset;
    size_t start_tsc = worker_rdtsc();
    auto* pkt = (Packet*)(socket_buffer_[socket_thread_id]
        + buf_offset * cfg->packet_length);
    size_t frame_id = pkt->frame_id;
    size_t frame_slot = frame_id % TASK_BUFFER_FRAME_NUM;
    size_t symbol_id = pkt->symbol_id;
    size_t ant_id = pkt->ant_id;

    if (cfg->fft_in_rru) {
        simd_convert_float16_to_float32(reinterpret_cast<float*>(fft_inout),
            reinterpret_cast<float*>(
                &pkt->data[2 * cfg->ofdm_rx_zero_prefix_bs_]),
            cfg->OFDM_CA_NUM * 2);
    } else {
        simd_convert_short_to_float(
            &pkt->data[2 * cfg->ofdm_rx_zero_prefix_bs_],
            reinterpret_cast<float*>(fft_inout), cfg->OFDM_CA_NUM * 2);

        if (kDebugPrintInTask) {
            printf("In doFFT thread %d: frame: %zu, symbol: %zu, ant: %zu\n",
                tid, frame_id, symbol_id, ant_id);
            if (kPrintFFTInput) {
                printf("FFT input\n");
                for (size_t i = 0; i < cfg->OFDM_CA_NUM; i++) {
                    printf("%.4f+%.4fi ", fft_inout[i].re, fft_inout[i].im);
                }
                printf("\n");
            }
        }
    }

    DurationStat dummy_duration_stat; // TODO: timing for calibration symbols
    DurationStat* duration_stat = nullptr;
    SymbolType sym_type = cfg->get_symbol_type(frame_id, symbol_id);
    if (sym_type == SymbolType::kUL) {
        duration_stat = duration_stat_fft;
    } else if (sym_type == SymbolType::kPilot) {
        duration_stat = duration_stat_csi;
    } else {
        duration_stat = &dummy_duration_stat; // For calibration symbols
    }

    size_t start_tsc1 = worker_rdtsc();
    duration_stat->task_duration[1] += start_tsc1 - start_tsc;

    if (!cfg->fft_in_rru) {
        DftiComputeForward(mkl_handle,
            reinterpret_cast<float*>(fft_inout)); // Compute FFT in-place
    }

    size_t start_tsc2 = worker_rdtsc();
    duration_stat->task_duration[2] += start_tsc2 - start_tsc1;

    if (sym_type == SymbolType::kPilot) {
        if (kCollectPhyStats) {
            phy_stats->update_pilot_snr(frame_id,
                cfg->get_pilot_symbol_idx(frame_id, symbol_id), fft_inout);
        }

        const size_t frame_slot = frame_id % TASK_BUFFER_FRAME_NUM;
        const size_t ue_id = cfg->get_pilot_symbol_idx(frame_id, symbol_id);
        partial_transpose(
            csi_buffers_[frame_slot][ue_id], ant_id, SymbolType::kPilot);
    } else if (sym_type == SymbolType::kUL) {
        partial_transpose(cfg->get_data_buf(data_buffer_, frame_id, symbol_id),
            ant_id, SymbolType::kUL);
    } else if ((sym_type == SymbolType::kCalDL and ant_id == cfg->ref_ant)
        or (sym_type == SymbolType::kCalUL and ant_id != cfg->ref_ant)) {
        partial_transpose(
            calib_buffer_[frame_slot], ant_id, SymbolType::kCalUL);
    } else {
        rt_assert(false, "Unknown or unsupported symbol type");
    }

    duration_stat->task_duration[3] += worker_rdtsc() - start_tsc2;
    socket_buffer_status_[socket_thread_id][buf_offset] = 0; // Reset sock buf
    duration_stat->task_count++;
    duration_stat->task_duration[0] += worker_rdtsc() - start_tsc;
    return Event_data(EventType::kFFT,
        gen_tag_t::frm_sym(pkt->frame_id, pkt->symbol_id)._tag);
}

void DoFFT::partial_transpose(
    complex_float* out_buf, size_t ant_id, SymbolType symbol_type) const
{
    // We have OFDM_DATA_NUM % kTransposeBlockSize == 0
    const size_t num_blocks = cfg->OFDM_DATA_NUM / kTransposeBlockSize;
    // Do the 1st step of 2-step reciprocal calibration
    // The 2nd step will be performed in dozf
    if (symbol_type == SymbolType::kCalDL
        or symbol_type == SymbolType::kCalUL) {
        for (size_t i = 0; i < cfg->OFDM_DATA_NUM; i += cfg->BS_ANT_NUM)
            for (size_t j = 0; j < cfg->BS_ANT_NUM - 1; j++)
                fft_inout[std::min(i + j, cfg->OFDM_DATA_NUM - 1)
                    + cfg->OFDM_DATA_START]
                    = fft_inout[i + cfg->OFDM_DATA_START];
    }

    for (size_t block_idx = 0; block_idx < num_blocks; block_idx++) {
        const size_t block_base_offset
            = block_idx * (kTransposeBlockSize * cfg->BS_ANT_NUM);
        // We have kTransposeBlockSize % kSCsPerCacheline == 0
        for (size_t sc_j = 0; sc_j < kTransposeBlockSize;
             sc_j += kSCsPerCacheline) {
            const size_t sc_idx = (block_idx * kTransposeBlockSize) + sc_j;
            const complex_float* src
                = &fft_inout[sc_idx + cfg->OFDM_DATA_START];

            complex_float* dst = &out_buf[block_base_offset
                + (ant_id * kTransposeBlockSize) + sc_j];

            // With either of AVX-512 or AVX2, load one cacheline =
            // 16 float values = 8 subcarriers = kSCsPerCacheline

#if 0
            // AVX-512. Disabled for now because we don't have a working
            // complex multiply for __m512 type.
            __m512 fft_result
                = _mm512_load_ps(reinterpret_cast<const float*>(src));
            if (symbol_type == SymbolType::kPilot) {
                __m512 pilot_tx = _mm512_set_ps(cfg->pilots_sgn_[sc_idx + 7].im,
                    cfg->pilots_sgn_[sc_idx + 7].re,
                    cfg->pilots_sgn_[sc_idx + 6].im,
                    cfg->pilots_sgn_[sc_idx + 6].re,
                    cfg->pilots_sgn_[sc_idx + 5].im,
                    cfg->pilots_sgn_[sc_idx + 5].re,
                    cfg->pilots_sgn_[sc_idx + 4].im,
                    cfg->pilots_sgn_[sc_idx + 4].re,
                    cfg->pilots_sgn_[sc_idx + 3].im,
                    cfg->pilots_sgn_[sc_idx + 3].re,
                    cfg->pilots_sgn_[sc_idx + 2].im,
                    cfg->pilots_sgn_[sc_idx + 2].re,
                    cfg->pilots_sgn_[sc_idx + 1].im,
                    cfg->pilots_sgn_[sc_idx + 1].re,
                    cfg->pilots_sgn_[sc_idx].im, cfg->pilots_sgn_[sc_idx].re);
                fft_result = _mm512_mul_ps(fft_result, pilot_tx);
            }
            _mm512_stream_ps(reinterpret_cast<float*>(dst), fft_result);
#else
            __m256 fft_result0
                = _mm256_load_ps(reinterpret_cast<const float*>(src));
            __m256 fft_result1
                = _mm256_load_ps(reinterpret_cast<const float*>(src + 4));
            if (symbol_type == SymbolType::kPilot) {
                __m256 pilot_tx0 = _mm256_set_ps(
                    cfg->pilots_sgn_[sc_idx + 3].im,
                    cfg->pilots_sgn_[sc_idx + 3].re,
                    cfg->pilots_sgn_[sc_idx + 2].im,
                    cfg->pilots_sgn_[sc_idx + 2].re,
                    cfg->pilots_sgn_[sc_idx + 1].im,
                    cfg->pilots_sgn_[sc_idx + 1].re,
                    cfg->pilots_sgn_[sc_idx].im, cfg->pilots_sgn_[sc_idx].re);
                fft_result0 = CommsLib::__m256_complex_cf32_mult(
                    fft_result0, pilot_tx0, true);

                __m256 pilot_tx1
                    = _mm256_set_ps(cfg->pilots_sgn_[sc_idx + 7].im,
                        cfg->pilots_sgn_[sc_idx + 7].re,
                        cfg->pilots_sgn_[sc_idx + 6].im,
                        cfg->pilots_sgn_[sc_idx + 6].re,
                        cfg->pilots_sgn_[sc_idx + 5].im,
                        cfg->pilots_sgn_[sc_idx + 5].re,
                        cfg->pilots_sgn_[sc_idx + 4].im,
                        cfg->pilots_sgn_[sc_idx + 4].re);
                fft_result1 = CommsLib::__m256_complex_cf32_mult(
                    fft_result1, pilot_tx1, true);
            }
            _mm256_store_ps(reinterpret_cast<float*>(dst), fft_result0);
            _mm256_store_ps(reinterpret_cast<float*>(dst + 4), fft_result1);
#endif
        }
    }
}

DoIFFT::DoIFFT(Config* in_config, int in_tid, double freq_ghz,
    moodycamel::ConcurrentQueue<Event_data>& in_task_queue,
    moodycamel::ConcurrentQueue<Event_data>& complete_task_queue,
    moodycamel::ProducerToken* worker_producer_token,
    Table<complex_float>& in_dl_ifft_buffer, char* in_dl_socket_buffer,
    Stats* in_stats_manager)
    : Doer(in_config, in_tid, freq_ghz, in_task_queue, complete_task_queue,
          worker_producer_token)
    , dl_ifft_buffer_(in_dl_ifft_buffer)
    , dl_socket_buffer_(in_dl_socket_buffer)
{
    duration_stat
        = in_stats_manager->get_duration_stat(DoerType::kIFFT, in_tid);
    (void)DftiCreateDescriptor(
        &mkl_handle, DFTI_SINGLE, DFTI_COMPLEX, 1, cfg->OFDM_CA_NUM);
    (void)DftiCommitDescriptor(mkl_handle);
}

DoIFFT::~DoIFFT() { DftiFreeDescriptor(&mkl_handle); }

Event_data DoIFFT::launch(size_t tag)
{
    size_t start_tsc = worker_rdtsc();
    size_t ant_id = gen_tag_t(tag).ant_id;
    size_t frame_id = gen_tag_t(tag).frame_id;
    size_t symbol_id = gen_tag_t(tag).symbol_id;
    size_t data_symbol_idx_dl = cfg->get_dl_symbol_idx(frame_id, symbol_id);

    if (kDebugPrintInTask) {
        printf("In doIFFT thread %d: frame: %zu, symbol: %zu, antenna: %zu\n",
            tid, frame_id, symbol_id, ant_id);
    }

    size_t offset
        = (cfg->get_total_data_symbol_idx_dl(frame_id, data_symbol_idx_dl)
              * cfg->BS_ANT_NUM)
        + ant_id;

    size_t start_tsc1 = worker_rdtsc();
    duration_stat->task_duration[1] += start_tsc1 - start_tsc;

    float* ifft_buf_ptr = (float*)dl_ifft_buffer_[offset];
    memset(ifft_buf_ptr, 0, sizeof(float) * cfg->OFDM_DATA_START * 2);
    memset(ifft_buf_ptr + (cfg->OFDM_DATA_STOP) * 2, 0,
        sizeof(float) * cfg->OFDM_DATA_START * 2);

    DftiComputeBackward(mkl_handle, ifft_buf_ptr);
    // printf("data after ifft\n");
    // for (size_t i = 0; i < cfg->OFDM_CA_NUM; i++)
    //     printf("%.1f+%.1fj ", dl_ifft_buffer_[buffer_symbol_offset][i].re,
    //         dl_ifft_buffer_[buffer_symbol_offset][i].im);
    // printf("\n");

    cx_fmat mat_data((cx_float*)ifft_buf_ptr, 1, cfg->OFDM_CA_NUM, false);
    float post_scale = cfg->OFDM_CA_NUM;
    mat_data /= post_scale;

    size_t start_tsc2 = worker_rdtsc();
    duration_stat->task_duration[2] += start_tsc2 - start_tsc1;

    int dl_socket_buffer_status_size = cfg->BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM
        * cfg->dl_data_symbol_num_perframe;
    int socket_symbol_offset = offset % dl_socket_buffer_status_size;
    int packet_length = cfg->packet_length;
    struct Packet* pkt = (struct Packet*)&dl_socket_buffer_[socket_symbol_offset
        * packet_length];
    short* socket_ptr = &pkt->data[2 * cfg->ofdm_tx_zero_prefix_];

    for (size_t sc_id = 0; sc_id < cfg->OFDM_CA_NUM; sc_id += 8) {
        /* ifft scaled results by OFDM_CA_NUM */
        __m256 scale_factor = _mm256_set1_ps(32768.0);
        __m256 ifft1 = _mm256_load_ps(ifft_buf_ptr + 2 * sc_id);
        __m256 ifft2 = _mm256_load_ps(ifft_buf_ptr + 2 * sc_id + 8);
        __m256 scaled_ifft1 = _mm256_mul_ps(ifft1, scale_factor);
        __m256 scaled_ifft2 = _mm256_mul_ps(ifft2, scale_factor);
        __m256i integer1 = _mm256_cvtps_epi32(scaled_ifft1);
        __m256i integer2 = _mm256_cvtps_epi32(scaled_ifft2);
        integer1 = _mm256_packs_epi32(integer1, integer2);
        integer1 = _mm256_permute4x64_epi64(integer1, 0xD8);
        //_mm256_stream_si256((__m256i*)&socket_ptr[2 * sc_id], integer1);
        _mm256_stream_si256(
            (__m256i*)&socket_ptr[2 * (sc_id + cfg->CP_LEN)], integer1);
        if (sc_id >= cfg->OFDM_CA_NUM - cfg->CP_LEN) // add CP
            _mm256_stream_si256((__m256i*)&socket_ptr[2
                                    * (sc_id + cfg->CP_LEN - cfg->OFDM_CA_NUM)],
                integer1);
    }

    duration_stat->task_duration[2] += worker_rdtsc() - start_tsc2;

    // cout << "In ifft: frame: "<< frame_id<<", symbol: "<<
    // current_data_symbol_id<<", ant: " << ant_id << ", data: "; for (int j =
    // 0; j <OFDM_CA_NUM; j++) {
    //     int socket_offset = sizeof(int) * 16 + ant_id *
    //     packetReceiver::packet_length; cout <<*((short *)(socket_ptr +
    //     socket_offset) + 2 * j)  << "+j"<<*((short *)(socket_ptr +
    //     socket_offset) + 2 * j + 1 )<<",   ";
    // }
    // cout<<"\n\n"<<endl;

    duration_stat->task_count++;
    duration_stat->task_duration[0] += worker_rdtsc() - start_tsc;
    return Event_data(EventType::kIFFT, tag);
}
