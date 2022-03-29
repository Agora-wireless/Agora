#include "dofft.hpp"
#include "concurrent_queue_wrapper.hpp"
#include "datatype_conversion.h"
#include "shared_counters.hpp"
#include "signalHandler.hpp"
#include <malloc.h>

#define TRIGGER_TIMER(stmt) if(likely(state_trigger)){stmt;}

using namespace arma;

static constexpr bool kPrintFFTInput = false;

DoFFT::DoFFT(Config* config, int tid, double freq_ghz, Range ant_range,
    Table<char>& time_domain_iq_buffer,
    Table<char>& freq_domain_iq_buffer_to_send, 
    SharedState* shared_state_)
    : Doer(config, tid, freq_ghz)
    , time_domain_iq_buffer_(time_domain_iq_buffer)
    , freq_domain_iq_buffer_to_send_(freq_domain_iq_buffer_to_send)
    , ant_range_(ant_range)
    , shared_state_(shared_state_)
{
    DftiCreateDescriptor(
        &mkl_handle, DFTI_SINGLE, DFTI_COMPLEX, 1, cfg_->OFDM_CA_NUM);
    DftiCommitDescriptor(mkl_handle);

    // Aligned for SIMD
    fft_inout = reinterpret_cast<complex_float*>(
        memalign(64, cfg_->OFDM_CA_NUM * sizeof(complex_float)));
}

DoFFT::~DoFFT()
{
    DftiFreeDescriptor(&mkl_handle);
    free(fft_inout);
}

void DoFFT::Launch(size_t frame_id, size_t symbol_id, size_t ant_id)
{
    size_t frame_slot = frame_id % kFrameWnd;

    simd_convert_short_to_float(reinterpret_cast<short*>(time_domain_iq_buffer_[ant_id] +
        (frame_slot * cfg_->symbol_num_perframe * cfg_->packet_length)
        + symbol_id * cfg_->packet_length + Packet::kOffsetOfData),
        reinterpret_cast<float*>(fft_inout), cfg_->OFDM_CA_NUM * 2);

    DftiComputeForward(mkl_handle, reinterpret_cast<float*>(fft_inout));

    simd_convert_float32_to_float16(reinterpret_cast<float*>(freq_domain_iq_buffer_to_send_[ant_id] + 
        (frame_slot * cfg_->symbol_num_perframe * cfg_->packet_length)
        + symbol_id * cfg_->packet_length),
        reinterpret_cast<float*>(fft_inout), cfg_->OFDM_CA_NUM * 2);

    return;
}

void DoFFT::StartWork()
{
    cur_idx_ = tid_;
    size_t start_tsc = 0;
    size_t work_tsc_duration = 0;
    size_t fft_tsc_duration = 0;
    size_t state_operation_duration = 0;
    size_t loop_count = 0;
    size_t work_count = 0;
    size_t fft_task_count = 0;
    size_t fft_start_tsc;
    bool state_trigger = false;

    while (cfg_->running && !SignalHandler::gotExitSignal()) {
        TRIGGER_TIMER(loop_count ++);
        size_t work_start_tsc, state_start_tsc;
        size_t cur_symbol = cur_idx_ / cfg_->get_num_ant_to_process();
        size_t cur_ant = cur_idx_ % cfg_->get_num_ant_to_process() + cfg_->ant_start;

        TRIGGER_TIMER({
            work_start_tsc = rdtsc();
            state_start_tsc = rdtsc();
        });
        bool ret = shared_state_->received_all_time_iq_pkts(cur_frame_, cur_symbol);
        TRIGGER_TIMER({
            state_operation_duration += rdtsc() - state_start_tsc;
            work_tsc_duration += rdtsc() - work_start_tsc;
        });

        if (ret) {
            if (unlikely(!state_trigger && cur_frame_ >= 200)) {
                loop_count ++;
                start_tsc = rdtsc();
                state_trigger = true;
            }
            TRIGGER_TIMER({
                work_start_tsc = rdtsc();
                work_count ++;
                fft_start_tsc = rdtsc();
            });
            Launch(cur_frame_, cur_symbol, cur_ant);
            TRIGGER_TIMER({
                fft_tsc_duration += rdtsc() - fft_start_tsc;
                fft_task_count += ant_range_.end - ant_range_.start;
                state_start_tsc = rdtsc();
            });
            shared_state_->fft_done(cur_frame_, cur_symbol);
            TRIGGER_TIMER({
                state_operation_duration += rdtsc() - state_start_tsc;
                work_tsc_duration += rdtsc() - work_start_tsc;
            });
            cur_idx_ += cfg_->fft_thread_num;
            if (cur_idx_ >= cfg_->get_num_ant_to_process() * cfg_->symbol_num_perframe) {
                cur_idx_ = tid_;
                cur_frame_ ++;
                if (unlikely(cur_frame_ == cfg_->frames_to_test)) {
                    break;
                }
            }
        }
    }

    size_t whole_duration = rdtsc() - start_tsc;
    size_t idle_duration = whole_duration - work_tsc_duration;
    printf("DoFFT Thread %d duration stats: total time used %.2lfms, "
        "fft %.2lfms (%zu, %.2lf%%), stating %.2lfms (%.2lf%%), idle %.2lfms (%.2lf%%), "
        "working proportions (%zu/%zu: %.2lf%%)\n",
        tid_, cycles_to_ms(whole_duration, freq_ghz_),
        cycles_to_ms(fft_tsc_duration, freq_ghz_), fft_task_count, fft_tsc_duration * 100.0f / whole_duration,
        cycles_to_ms(state_operation_duration, freq_ghz_), state_operation_duration * 100.0f / whole_duration,
        cycles_to_ms(idle_duration, freq_ghz_), idle_duration * 100.0f / whole_duration,
        work_count, loop_count, loop_count == 0 ? 0 : work_count * 100.0f / loop_count);
}

void DoFFT::partial_transpose(
    complex_float* out_buf, size_t ant_id, SymbolType symbol_type) const
{
    // We have OFDM_DATA_NUM % kTransposeBlockSize == 0
    const size_t num_blocks = cfg_->OFDM_DATA_NUM / kTransposeBlockSize;
    // Do the 1st step of 2-step reciprocal calibration
    // The 2nd step will be performed in dozf
    if (symbol_type == SymbolType::kCalDL
        or symbol_type == SymbolType::kCalUL) {
        for (size_t i = 0; i < cfg_->OFDM_DATA_NUM; i += cfg_->BS_ANT_NUM)
            for (size_t j = 0; j < cfg_->BS_ANT_NUM - 1; j++)
                fft_inout[std::min(i + j, cfg_->OFDM_DATA_NUM - 1)
                    + cfg_->OFDM_DATA_START]
                    = fft_inout[i + cfg_->OFDM_DATA_START];
    }

    for (size_t block_idx = 0; block_idx < num_blocks; block_idx++) {
        const size_t block_base_offset
            = block_idx * (kTransposeBlockSize * cfg_->BS_ANT_NUM);
        // We have kTransposeBlockSize % kSCsPerCacheline == 0
        for (size_t sc_j = 0; sc_j < kTransposeBlockSize;
             sc_j += kSCsPerCacheline) {
            const size_t sc_idx = (block_idx * kTransposeBlockSize) + sc_j;
            const complex_float* src
                = &fft_inout[sc_idx + cfg_->OFDM_DATA_START];

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
                    cfg_->pilots_sgn_[sc_idx + 3].im,
                    cfg_->pilots_sgn_[sc_idx + 3].re,
                    cfg_->pilots_sgn_[sc_idx + 2].im,
                    cfg_->pilots_sgn_[sc_idx + 2].re,
                    cfg_->pilots_sgn_[sc_idx + 1].im,
                    cfg_->pilots_sgn_[sc_idx + 1].re,
                    cfg_->pilots_sgn_[sc_idx].im, cfg_->pilots_sgn_[sc_idx].re);
                fft_result0 = CommsLib::__m256_complex_cf32_mult(
                    fft_result0, pilot_tx0, true);

                __m256 pilot_tx1
                    = _mm256_set_ps(cfg_->pilots_sgn_[sc_idx + 7].im,
                        cfg_->pilots_sgn_[sc_idx + 7].re,
                        cfg_->pilots_sgn_[sc_idx + 6].im,
                        cfg_->pilots_sgn_[sc_idx + 6].re,
                        cfg_->pilots_sgn_[sc_idx + 5].im,
                        cfg_->pilots_sgn_[sc_idx + 5].re,
                        cfg_->pilots_sgn_[sc_idx + 4].im,
                        cfg_->pilots_sgn_[sc_idx + 4].re);
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
    Table<complex_float>& in_dl_ifft_buffer, char* in_dl_socket_buffer)
    : Doer(in_config, in_tid, freq_ghz)
    , dl_ifft_buffer_(in_dl_ifft_buffer)
    , dl_socket_buffer_(in_dl_socket_buffer)
{
    (void)DftiCreateDescriptor(
        &mkl_handle, DFTI_SINGLE, DFTI_COMPLEX, 1, cfg_->OFDM_CA_NUM);
    (void)DftiCommitDescriptor(mkl_handle);
}

DoIFFT::~DoIFFT() { DftiFreeDescriptor(&mkl_handle); }

void DoIFFT::Launch(size_t frame_id, size_t symbol_id_dl, size_t ant_id)
{
    size_t data_symbol_idx_dl = symbol_id_dl;

    if (kDebugPrintInTask) {
        printf("In doIFFT thread %d: frame: %zu, symbol: %zu, antenna: %zu\n",
            tid_, frame_id, symbol_id_dl, ant_id);
    }

    size_t offset
        = (cfg_->get_total_data_symbol_idx_dl(frame_id, data_symbol_idx_dl)
              * cfg_->BS_ANT_NUM)
        + ant_id;

    float* ifft_buf_ptr = (float*)dl_ifft_buffer_[offset];
    memset(ifft_buf_ptr, 0, sizeof(float) * cfg_->OFDM_DATA_START * 2);
    memset(ifft_buf_ptr + (cfg_->OFDM_DATA_STOP) * 2, 0,
        sizeof(float) * cfg_->OFDM_DATA_START * 2);

    DftiComputeBackward(mkl_handle, ifft_buf_ptr);
    // printf("data after ifft\n");
    // for (size_t i = 0; i < cfg->OFDM_CA_NUM; i++)
    //     printf("%.1f+%.1fj ", dl_ifft_buffer_[buffer_symbol_offset][i].re,
    //         dl_ifft_buffer_[buffer_symbol_offset][i].im);
    // printf("\n");

    cx_fmat mat_data((cx_float*)ifft_buf_ptr, 1, cfg_->OFDM_CA_NUM, false);
    float post_scale = cfg_->OFDM_CA_NUM;
    mat_data /= post_scale;

    int dl_socket_buffer_status_size = cfg_->BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM
        * cfg_->dl_data_symbol_num_perframe;
    int socket_symbol_offset = offset % dl_socket_buffer_status_size;
    int packet_length = cfg_->packet_length;
    struct Packet* pkt = (struct Packet*)&dl_socket_buffer_[socket_symbol_offset
        * packet_length];
    short* socket_ptr = &pkt->data_[2 * cfg_->TX_PREFIX_LEN];

    for (size_t sc_id = 0; sc_id < cfg_->OFDM_CA_NUM; sc_id += 8) {
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
            (__m256i*)&socket_ptr[2 * (sc_id + cfg_->CP_LEN)], integer1);
        if (sc_id >= cfg_->OFDM_CA_NUM - cfg_->CP_LEN) // add CP
            _mm256_stream_si256((__m256i*)&socket_ptr[2
                                    * (sc_id + cfg_->CP_LEN - cfg_->OFDM_CA_NUM)],
                integer1);
    }

    // cout << "In ifft: frame: "<< frame_id<<", symbol: "<<
    // current_data_symbol_id<<", ant: " << ant_id << ", data: "; for (int j =
    // 0; j <OFDM_CA_NUM; j++) {
    //     int socket_offset = sizeof(int) * 16 + ant_id *
    //     packetReceiver::packet_length; cout <<*((short *)(socket_ptr +
    //     socket_offset) + 2 * j)  << "+j"<<*((short *)(socket_ptr +
    //     socket_offset) + 2 * j + 1 )<<",   ";
    // }
    // cout<<"\n\n"<<endl;
}
