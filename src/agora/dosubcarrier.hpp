#pragma once

#include "Symbols.hpp"
#include "buffer.hpp"
#include "comms-lib.h"
#include "concurrentqueue.h"
#include "config.hpp"
#include "datatype_conversion.h"
#include "dodemul.hpp"
#include "doer.hpp"
#include "doprecode.hpp"
#include "dozf.hpp"
#include "gettime.h"
#include "modulation.hpp"
#include "phy_stats.hpp"
#include "shared_counters.hpp"
#include "signalHandler.hpp"
#include "stats.hpp"
#include <armadillo>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <vector>

using namespace arma;

/**
 * @brief A worker class that handles all subcarrier-parallel processing tasks.
 *
 * Currently, this worker class contains the following functionality:
 * @li DoZF
 * @li DoDemul
 * @li DoPrecode
 * @li Reciprocity (?? TBD)
 *
 * ## General usage ##
 * One instance of this class should handle the computation for one `block_size`
 * range of subcarrier frequencies, so we should spawn `num_events` instances of
 * this class.
 * For example, see `Config::demul_events_per_symbol and
 * `Config::demul_block_size`, or the similar ones for zeroforcing,
 * `zf_events_per_symbol` and `zf_block_size`.
 *
 * Upon receiving an event, it executes the specific doer for that event type,
 * consisting of one of the following:
 * @li zeroforcing (`DoZF`)
 * @li demodulation (`DoDemul`) for uplink
 * @li precoding (`DoPrecode`) for downlink.
 *
 * FIXME: The biggest issue is how buffers are going to be allocated, shared,
 * and accessed. Currently, the rest of Agora expects single instance of all
 * buffers, but with this redesign, we are allocating per-DoSubcarrier buffers.
 * While this probably is okay for intermediate internal buffers, perhaps we
 * should require (at least initially) that all input and output buffers are
 * shared across all `DoSubcarrier` instances.
 *
 * ## Buffer ownership and management ##
 * The general buffer ownership policy is to accept *references* to input
 * buffers, and to own both intermediate buffers for internal usage as well as
 * output buffers that are shared with others.
 * This means that the constructor/destructor of this class is responsible for
 * allocating/deallocating the intermediate buffers and output buffers but not
 * the input buffers.
 * 
 * FIXME: Currently, the output buffers are still owned by the core `Agora`
 * instance. We should eventually move them into here.
 */
class DoSubcarrier : public Doer {
public:
    /// Construct a new Do Subcarrier object
    DoSubcarrier(Config* config, int tid, double freq_ghz,
        /// The range of subcarriers handled by this subcarrier doer.
        Range sc_range,
        // input buffers
        Table<char>& socket_buffer,
        PtrGrid<kFrameWnd, kMaxUEs, complex_float>& csi_buffers,
        Table<complex_float>& calib_buffer, Table<int8_t>& dl_encoded_buffer,
        // output buffers
        PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& demod_buffers,
        Table<complex_float>& dl_ifft_buffer,
        // intermediate buffers owned by SubcarrierManager
        Table<complex_float>& ue_spec_pilot_buffer,
        Table<complex_float>& equal_buffer,
        PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& ul_zf_matrices,
        PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& dl_zf_matrices,
        PhyStats* phy_stats, Stats* stats, RxStatus* rx_status = nullptr,
        DemulStatus* demul_status = nullptr, PrecodeStatus* precode_status = nullptr)
        : Doer(config, tid, freq_ghz, dummy_conq_, dummy_conq_,
              nullptr /* tok */)
        , sc_range_(sc_range)
        , socket_buffer_(socket_buffer)
        , csi_buffers_(csi_buffers)
        , calib_buffer_(calib_buffer)
        , dl_encoded_buffer_(dl_encoded_buffer)
        , demod_buffers_(demod_buffers)
        , dl_ifft_buffer_(dl_ifft_buffer)
        , ue_spec_pilot_buffer_(ue_spec_pilot_buffer)
        , equal_buffer_(equal_buffer)
        , ul_zf_matrices_(ul_zf_matrices)
        , dl_zf_matrices_(dl_zf_matrices)
        , rx_status_(rx_status)
        , demul_status_(demul_status)
        , precode_status_(precode_status)
    {
        // Create the requisite Doers
        do_zf_ = new DoZF(this->cfg, tid, freq_ghz, dummy_conq_, dummy_conq_,
            nullptr /* ptok */, csi_buffers_, calib_buffer, ul_zf_matrices_,
            dl_zf_matrices_, stats);

        do_demul_ = new DoDemul(this->cfg, tid, freq_ghz, dummy_conq_,
            dummy_conq_, nullptr /* ptok */, ul_zf_matrices_,
            ue_spec_pilot_buffer_, equal_buffer_, demod_buffers_, phy_stats,
            stats, &socket_buffer_);

        do_precode_ = new DoPrecode(this->cfg, tid, freq_ghz, dummy_conq_,
            dummy_conq_, nullptr /* ptok */, dl_zf_matrices_, dl_ifft_buffer_,
            dl_encoded_buffer_, stats);

        // Init internal states
        demul_cur_sym_ul_ = 0;
    }

    ~DoSubcarrier()
    {
        delete do_zf_;
        delete do_demul_;
        delete do_precode_;
        // delete computeReciprocity_;
    }

    void start_work()
    {
        const size_t n_zf_tasks_reqd
            = (sc_range_.end - sc_range_.start) / cfg->zf_block_size;
        const size_t n_demul_tasks_reqd
            = (sc_range_.end - sc_range_.start) / cfg->demul_block_size;
        const size_t n_precode_tasks_reqd
            = (sc_range_.end - sc_range_.start) / cfg->demul_block_size;
        
        printf("Range [%u:%u] starts to work\n", sc_range_.start, sc_range_.end);

        size_t start_tsc = 0;
        size_t work_tsc_duration = 0;
        size_t csi_tsc_duration = 0;
        size_t zf_tsc_duration = 0;
        size_t demod_tsc_duration = 0;
        size_t precode_tsc_duration = 0;
        size_t print_tsc_duration = 0;
        size_t state_operation_duration = 0;
        size_t loop_count = 0;
        size_t work_count = 0;

        size_t work_start_tsc, state_start_tsc;

        while (cfg->running && !SignalHandler::gotExitSignal()) {
            size_t worked = 0;

            if (zf_cur_frame_ > demul_cur_frame_) {

                work_start_tsc = rdtsc();
                state_start_tsc = rdtsc();
                bool ret = rx_status_->is_demod_ready(
                       demul_cur_frame_, demul_cur_sym_ul_);
                state_operation_duration += rdtsc() - state_start_tsc;
                work_tsc_duration += rdtsc() - work_start_tsc;

                if (ret) {
                    work_start_tsc = rdtsc();
                    worked = 1;

                    size_t demod_start_tsc = rdtsc();
                    do_demul_->launch(demul_cur_frame_,
                        demul_cur_sym_ul_,
                        sc_range_.start
                            + (n_demul_tasks_done_ * cfg->demul_block_size));
                    demod_tsc_duration += rdtsc() - demod_start_tsc;

                    n_demul_tasks_done_++;
                    if (n_demul_tasks_done_ == n_demul_tasks_reqd) {
                        n_demul_tasks_done_ = 0;

                        state_start_tsc = rdtsc();
                        if (cfg->test_mode != 1) {
                            demul_status_->demul_complete(
                                demul_cur_frame_, demul_cur_sym_ul_, n_demul_tasks_reqd);
                        }
                        state_operation_duration += rdtsc() - state_start_tsc;

                        demul_cur_sym_ul_++;
                        if (demul_cur_sym_ul_ == cfg->ul_data_symbol_num_perframe) {
                            demul_cur_sym_ul_ = 0;

                            demod_start_tsc = rdtsc();
                            printf("Main thread: Demodulation done frame: %lu "
                                "(%lu UL symbols)\n",
                                demul_cur_frame_,
                                cfg->ul_data_symbol_num_perframe);
                            print_tsc_duration += rdtsc() - demod_start_tsc;
                            
                            if (cfg->test_mode == 1) {
                                rx_status_->decode_done(demul_cur_frame_);
                            }

                            demul_cur_frame_++;
                            if (unlikely(demul_cur_frame_ == cfg->frames_to_test)) {
                                work_tsc_duration += rdtsc() - work_start_tsc;
                                break;
                            }
                        }
                    }

                    work_tsc_duration += rdtsc() - work_start_tsc;
                    continue;
                }
                
            }

            if (precode_status_->received_all_encoded_data(precode_cur_frame_, precode_cur_sym_dl_) 
                && zf_cur_frame_ > precode_cur_frame_) {
                size_t work_start_tsc = rdtsc();
                worked = 1;
                size_t base_sc_id = n_precode_tasks_done_ * cfg->demul_block_size + sc_range_.start;
                // printf("Precode frame %u symbol %u subcarrier %u\n", precode_cur_frame_, 
                    // precode_cur_sym_dl_, base_sc_id);
                size_t precode_start_tsc = rdtsc();
                do_precode_->launch(gen_tag_t::frm_sym_sc(precode_cur_frame_, precode_cur_sym_dl_, base_sc_id)._tag);
                precode_tsc_duration += rdtsc() - precode_start_tsc;
                n_precode_tasks_done_ ++;
                if (n_precode_tasks_done_ == n_precode_tasks_reqd) {
                    n_precode_tasks_done_ = 0;
                    precode_cur_sym_dl_ ++;
                    if (precode_cur_sym_dl_ == cfg->dl_data_symbol_num_perframe) {
                        precode_cur_sym_dl_ = 0;
                        // printf("Main thread: Precode done frame: %lu "
                        //        "(%lu UL symbols)\n",
                        //     precode_cur_frame_,
                        //     cfg->dl_data_symbol_num_perframe);
                        precode_start_tsc = rdtsc();
                        rx_status_->precode_done(precode_cur_frame_);
                        state_operation_duration += rdtsc() - precode_start_tsc;
                        precode_cur_frame_ ++;
                        if (unlikely(precode_cur_frame_ == cfg->frames_to_test)) {
                            work_tsc_duration += rdtsc() - work_start_tsc;
                            break;
                        }
                    }
                }
                work_tsc_duration += rdtsc() - work_start_tsc;
                continue;
            }

            if (csi_cur_frame_ > zf_cur_frame_) {
                work_start_tsc = rdtsc();
                worked = 1;

                size_t zf_start_tsc = rdtsc();
                do_zf_->launch(gen_tag_t::frm_sym_sc(zf_cur_frame_, 0,
                    sc_range_.start + n_zf_tasks_done_ * cfg->zf_block_size)
                                   ._tag);
                zf_tsc_duration += rdtsc() - zf_start_tsc;

                n_zf_tasks_done_++;
                if (n_zf_tasks_done_ == n_zf_tasks_reqd) {
                    n_zf_tasks_done_ = 0;

                    zf_start_tsc = rdtsc();
                    printf("Main thread: ZF done frame: %lu\n", zf_cur_frame_);
                    print_tsc_duration += rdtsc() - zf_start_tsc;

                    zf_cur_frame_++;
                }

                work_tsc_duration += rdtsc() - work_start_tsc;
                continue;
            }

            if (likely(start_tsc > 0)) {
                loop_count ++;
                work_start_tsc = rdtsc();
                state_start_tsc = rdtsc();
            }
            bool ret = rx_status_->received_all_pilots(csi_cur_frame_);
            if (likely(start_tsc > 0)) {
                state_operation_duration += rdtsc() - state_start_tsc;
                work_tsc_duration += rdtsc() - work_start_tsc;
            }

            if (ret) {
                if (unlikely(start_tsc == 0)) {
                    start_tsc = rdtsc();
                }

                work_start_tsc = rdtsc();
                worked = 1;

                size_t csi_start_tsc = rdtsc();
                run_csi(csi_cur_frame_, sc_range_.start);
                csi_tsc_duration += rdtsc() - csi_start_tsc;

                csi_start_tsc = rdtsc();
                printf(
                    "Main thread: pilot frame: %lu, finished CSI for all pilot "
                    "symbols\n",
                    csi_cur_frame_);
                print_tsc_duration += rdtsc() - csi_start_tsc;

                csi_cur_frame_++;
                work_tsc_duration += rdtsc() - work_start_tsc;
            }

            work_count += worked;
        }

        size_t whole_duration = rdtsc() - start_tsc;
        size_t idle_duration = whole_duration - work_tsc_duration;
        printf("DoSubcarrier Thread %u duration stats: total time used %.2lfms, "
            "csi %.2lfms (%.2lf\%), zf %.2lfms (%.2lf\%), demod %.2lfms (%.2lf\%), "
            "precode %.2lfms (%.2lf\%), print %.2lfms (%.2lf\%), stating "
            "%.2lfms (%.2lf\%), idle %.2lfms (%.2lf\%), working rate (%u/%u: %.2lf\%)\n", 
            tid, cycles_to_ms(whole_duration, freq_ghz),
            cycles_to_ms(csi_tsc_duration, freq_ghz), csi_tsc_duration * 100.0f / whole_duration,
            cycles_to_ms(zf_tsc_duration, freq_ghz), zf_tsc_duration * 100.0f / whole_duration,
            cycles_to_ms(demod_tsc_duration, freq_ghz), demod_tsc_duration * 100.0f / whole_duration, 
            cycles_to_ms(precode_tsc_duration, freq_ghz), precode_tsc_duration * 100.0f / whole_duration,
            cycles_to_ms(print_tsc_duration, freq_ghz), print_tsc_duration * 100.0f / whole_duration,
            cycles_to_ms(state_operation_duration, freq_ghz), state_operation_duration * 100.0f / whole_duration,
            cycles_to_ms(idle_duration, freq_ghz), idle_duration * 100.0f / whole_duration,
            work_count, loop_count, work_count * 100.0f / loop_count);
    }

private:
    void run_csi(size_t frame_id, size_t base_sc_id)
    {
        const size_t frame_slot = frame_id % kFrameWnd;
        rt_assert(base_sc_id == sc_range_.start, "Invalid SC in run_csi!");

        complex_float converted_sc[kSCsPerCacheline];

        for (size_t i = 0; i < cfg->pilot_symbol_num_perframe; i++) {
            for (size_t j = 0; j < cfg->BS_ANT_NUM; j++) {
                auto* pkt = reinterpret_cast<Packet*>(socket_buffer_[j]
                    + (frame_slot * cfg->symbol_num_perframe
                          * cfg->packet_length)
                    + i * cfg->packet_length);

                // Subcarrier ranges should be aligned with kTransposeBlockSize
                for (size_t block_idx = sc_range_.start / kTransposeBlockSize;
                     block_idx < sc_range_.end / kTransposeBlockSize;
                     block_idx++) {

                    const size_t block_base_offset
                        = block_idx * (kTransposeBlockSize * cfg->BS_ANT_NUM);

                    for (size_t sc_j = 0; sc_j < kTransposeBlockSize;
                         sc_j += kSCsPerCacheline) {
                        const size_t sc_idx
                            = (block_idx * kTransposeBlockSize) + sc_j;

                        simd_convert_float16_to_float32(
                            reinterpret_cast<float*>(converted_sc),
                            reinterpret_cast<float*>(pkt->data
                                + (cfg->OFDM_DATA_START + sc_idx) * 2),
                            kSCsPerCacheline * 2);

                        const complex_float* src = converted_sc;
                        complex_float* dst = csi_buffers_[frame_slot][i]
                            + block_base_offset + (j * kTransposeBlockSize)
                            + sc_j;

                        // With either of AVX-512 or AVX2, load one cacheline =
                        // 16 float values = 8 subcarriers = kSCsPerCacheline
                        // TODO: AVX512 complex multiply support below
                        // size_t pilots_sgn_offset = cfg->bs_server_addr_idx
                        //     * cfg->get_num_sc_per_server();
                        size_t pilots_sgn_offset = 0;

                        __m256 fft_result0 = _mm256_load_ps(
                            reinterpret_cast<const float*>(src));
                        __m256 fft_result1 = _mm256_load_ps(
                            reinterpret_cast<const float*>(src + 4));
                        __m256 pilot_tx0 = _mm256_set_ps(
                            cfg->pilots_sgn_[sc_idx + 3 + pilots_sgn_offset].im,
                            cfg->pilots_sgn_[sc_idx + 3 + pilots_sgn_offset].re,
                            cfg->pilots_sgn_[sc_idx + 2 + pilots_sgn_offset].im,
                            cfg->pilots_sgn_[sc_idx + 2 + pilots_sgn_offset].re,
                            cfg->pilots_sgn_[sc_idx + 1 + pilots_sgn_offset].im,
                            cfg->pilots_sgn_[sc_idx + 1 + pilots_sgn_offset].re,
                            cfg->pilots_sgn_[sc_idx + pilots_sgn_offset].im,
                            cfg->pilots_sgn_[sc_idx + pilots_sgn_offset].re);
                        fft_result0 = CommsLib::__m256_complex_cf32_mult(
                            fft_result0, pilot_tx0, true);

                        __m256 pilot_tx1 = _mm256_set_ps(
                            cfg->pilots_sgn_[sc_idx + 7 + pilots_sgn_offset].im,
                            cfg->pilots_sgn_[sc_idx + 7 + pilots_sgn_offset].re,
                            cfg->pilots_sgn_[sc_idx + 6 + pilots_sgn_offset].im,
                            cfg->pilots_sgn_[sc_idx + 6 + pilots_sgn_offset].re,
                            cfg->pilots_sgn_[sc_idx + 5 + pilots_sgn_offset].im,
                            cfg->pilots_sgn_[sc_idx + 5 + pilots_sgn_offset].re,
                            cfg->pilots_sgn_[sc_idx + 4 + pilots_sgn_offset].im,
                            cfg->pilots_sgn_[sc_idx + 4 + pilots_sgn_offset]
                                .re);
                        fft_result1 = CommsLib::__m256_complex_cf32_mult(
                            fft_result1, pilot_tx1, true);
                        _mm256_stream_ps(
                            reinterpret_cast<float*>(dst), fft_result0);
                        _mm256_stream_ps(
                            reinterpret_cast<float*>(dst + 4), fft_result1);
                    }
                }
            }
        }
    }

    /// The subcarrier range handled by this subcarrier doer.
    struct Range sc_range_;

    DoZF* do_zf_;
    DoDemul* do_demul_;
    DoPrecode* do_precode_;

    // Input buffers

    Table<char>& socket_buffer_;

    PtrGrid<kFrameWnd, kMaxUEs, complex_float>& csi_buffers_;
    Table<complex_float>& calib_buffer_;
    Table<int8_t>& dl_encoded_buffer_;

    // Output buffers

    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& demod_buffers_;
    Table<complex_float>& dl_ifft_buffer_;

    // Intermediate buffers

    Table<complex_float>& ue_spec_pilot_buffer_;
    Table<complex_float>& equal_buffer_;
    PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& ul_zf_matrices_;
    PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& dl_zf_matrices_;

    // Shared states with TXRX threads
    RxStatus* rx_status_;

    // Internal CSI states
    size_t csi_cur_frame_ = 0;

    // Internal ZF states
    size_t zf_cur_frame_ = 0; // Current frame waiting for CSI matrix
    size_t n_zf_tasks_done_ = 0;

    // Internal Demul states
    size_t demul_cur_frame_; // Current frame waiting for ZF matrix
    size_t demul_cur_sym_ul_ = 0; // Current data symbol wait to process
    size_t n_demul_tasks_done_ = 0;

    // Internal Precode states
    size_t precode_cur_frame_ = 0;
    size_t precode_cur_sym_dl_ = 0;
    size_t n_precode_tasks_done_ = 0;

    // Shared status with Decode threads
    DemulStatus* demul_status_;

    // Shared status with TXRX threads
    PrecodeStatus* precode_status_;

    moodycamel::ConcurrentQueue<Event_data> dummy_conq_;
};
