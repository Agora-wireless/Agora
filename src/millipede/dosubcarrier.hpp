/// Author: Kevin Boos
/// Email: kevinaboos@gmail.com
///
/// @see DoSubcarrier

#ifndef DOSUBCARRIER_HPP
#define DOSUBCARRIER_HPP

// [Kevin]: unsure if all of these includes are needed.
#include "Symbols.hpp"
#include "buffer.hpp"
#include "concurrentqueue.h"
#include "config.hpp"
#include "datatype_conversion.h"
#include "dodemul.hpp"
#include "doer.hpp"
#include "doprecode.hpp"
#include "dozf.hpp"
#include "reciprocity.hpp"

#include "gettime.h"
#include "modulation.hpp"
#include "phy_stats.hpp"
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
 * @li reciprocity (`Reciprocity)
 * @li demodulation (`DoDemul`) for uplink
 * @li precoding (`DoPrecode`) for downlink.
 *
 * FIXME: The biggest issue is how buffers are going to be allocated, shared,
 * and accessed. Currently, the rest of Millipede expects single instance of all
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
 * FIXME: Currently, the output buffers are still owned by the core `Millipede`
 * instance. We should eventually move them into here.
 */
class DoSubcarrier : public Doer {
public:
    /// Construct a new Do Subcarrier object
    DoSubcarrier(Config* config, int tid, double freq_ghz,
        moodycamel::ConcurrentQueue<Event_data>& task_queue,
        moodycamel::ConcurrentQueue<Event_data>& complete_task_queue,
        moodycamel::ProducerToken* worker_producer_token,
        /// The range of subcarriers handled by this subcarrier doer.
        struct Range subcarrier_range,
        // input buffers
        Table<char>& socket_buffer, Table<int>& socket_buffer_status,
        Table<complex_float>& csi_buffer, Table<complex_float>& recip_buffer,
        Table<complex_float>& calib_buffer, Table<int8_t>& dl_encoded_buffer,
        Table<complex_float>& data_buffer,
        // output buffers
        Table<int8_t>& demod_soft_buffer, Table<complex_float>& dl_ifft_buffer,
        // intermediate buffers owned by SubcarrierManager
        Table<complex_float>& ue_spec_pilot_buffer,
        Table<complex_float>& equal_buffer, Table<complex_float>& ul_zf_buffer,
        Table<complex_float>& dl_zf_buffer, PhyStats* phy_stats, Stats* stats,
        RxStats* rx_stats)
        : Doer(config, tid, freq_ghz, task_queue, complete_task_queue,
              worker_producer_token)
        , subcarrier_range_(subcarrier_range)
        , socket_buffer_(socket_buffer)
        , socket_buffer_status_(socket_buffer_status)
        , csi_buffer_(csi_buffer)
        , recip_buffer_(recip_buffer)
        , calib_buffer_(calib_buffer)
        , dl_encoded_buffer_(dl_encoded_buffer)
        , data_buffer_(data_buffer)
        , demod_soft_buffer_(demod_soft_buffer)
        , dl_ifft_buffer_(dl_ifft_buffer)
        , ue_spec_pilot_buffer_(ue_spec_pilot_buffer)
        , equal_buffer_(equal_buffer)
        , ul_zf_buffer_(ul_zf_buffer)
        , dl_zf_buffer_(dl_zf_buffer)
        , rx_stats_(rx_stats)
    {

        // Create the requisite Doers
        computeZF_ = new DoZF(this->cfg, tid, freq_ghz, this->task_queue_,
            this->complete_task_queue, this->worker_producer_token, csi_buffer_,
            recip_buffer_, ul_zf_buffer_, dl_zf_buffer_, stats);

        computeDemul_ = new DoDemul(this->cfg, tid, freq_ghz, this->task_queue_,
            this->complete_task_queue, this->worker_producer_token,
            data_buffer_, ul_zf_buffer_, ue_spec_pilot_buffer_, equal_buffer_,
            demod_soft_buffer_, phy_stats, stats);

        computePrecode_
            = new DoPrecode(this->cfg, tid, freq_ghz, this->task_queue_,
                this->complete_task_queue, this->worker_producer_token,
                dl_zf_buffer_, dl_ifft_buffer_, dl_encoded_buffer_, stats);

        // computeReciprocity_ = new Reciprocity(this->cfg, tid, freq_ghz,
        //     this->task_queue_, this->complete_task_queue,
        //     this->worker_producer_token, calib_buffer_, recip_buffer_, stats);

        // Init internal states
        csi_cur_frame = 0;
        zf_cur_frame = 0;
        num_zf_task_completed = 0;
        demul_cur_frame = 0;
        demul_cur_symbol_to_process = cfg->pilot_symbol_num_perframe;
        num_demul_task_completed = 0;
    }

    ~DoSubcarrier()
    {
        delete computeZF_;
        delete computeDemul_;
        delete computePrecode_;
        // delete computeReciprocity_;
    }

    Event_data launch(size_t tag, EventType event_type)
    {
        rt_assert(subcarrier_range_.contains(gen_tag_t(tag).sc_id),
            std::string("BUG: DoSubcarrier for ")
                + subcarrier_range_.to_string()
                + " tried to handle wrong subcarrier ID: "
                + std::to_string(gen_tag_t(tag).sc_id) + ", event_type "
                + std::to_string((int)event_type));

        switch (event_type) {
        case EventType::kCSI: {
            return run_csi(tag);
        } break;
        case EventType::kZF: {
            return computeZF_->launch(tag);
        } break;
        case EventType::kDemul: {
            return computeDemul_->launch(tag);
        } break;
        case EventType::kPrecode: {
            return computePrecode_->launch(tag);
        } break;
        /// TODO: move reciprocity into Subcarrier doers.
        // case EventType::kRc: {
        //     return computeReciprocity_->launch(tag, event_type);
        // } break;

        // Should never reach below here!
        default:
            break;
        }

        std::cerr << "[DoSubcarrier::launch] error, unexpected event type "
                  << (int)event_type << std::endl;
        rt_assert(
            false, "[DoSubcarrier::launch] error, unexpected event type.");

        return Event_data();
    }

    // Returns the range of subcarrier IDs handled by this subcarrier doer.
    Range& subcarrier_range() { return subcarrier_range_; }

private:
    static inline __m256 __m256_complex_cf32_mult(
        __m256 data1, __m256 data2, bool conj)
    {
        __m256 prod0 __attribute__((aligned(32)));
        __m256 prod1 __attribute__((aligned(32)));
        __m256 res __attribute__((aligned(32)));

        // https://stackoverflow.com/questions/39509746
        const __m256 neg0
            = _mm256_setr_ps(1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0);
        const __m256 neg1
            = _mm256_set_ps(1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0);
        prod0 = _mm256_mul_ps(data1, data2); // q1*q2, i1*i2, ...

        /* Step 2: Negate the imaginary elements of vec2 */
        data2 = _mm256_mul_ps(data2, conj ? neg0 : neg1);

        /* Step 3: Switch the real and imaginary elements of vec2 */
        data2 = _mm256_permute_ps(data2, 0xb1);

        /* Step 4: Multiply vec1 and the modified vec2 */
        prod1 = _mm256_mul_ps(data1, data2); // i2*q1, -i1*q2, ...

        /* Horizontally add the elements in vec3 and vec4 */
        res = conj
            ? _mm256_hadd_ps(prod0, prod1)
            : _mm256_hsub_ps(prod0, prod1); // i2*q1+-i1*q2, i1*i2+-q1*q2, ...
        res = _mm256_permute_ps(res, 0xd8);

        return res;
    }

    void start_work()
    {
        num_zf_task_required = (subcarrier_range_.end - subcarrier_range_.start)
            / cfg->zf_block_size;
        num_demul_task_required
            = (subcarrier_range_.end - subcarrier_range_.start)
            / cfg->demul_block_size;

        // TODO
        // 1. Integrate some statements into functions in RxStats
        // 2. Change the meaning of cur_frame in RxStats
        while (cfg->running && !SignalHandler::gotExitSignal()) {
            if (rx_counters_
                    ->num_pilot_pkts[csi_cur_frame % TASK_BUFFER_FRAME_NUM]
                == rx_counters_->num_pilot_pkts_per_frame) {
                run_csi(gen_tag_t::frm_sym_sc(
                    csi_cur_frame, 0, subcarrier_range_.start)
                            ._tag);
                csi_cur_frame++;
            }
            if (csi_cur_frame > zf_cur_frame) {
                computeZF_->launch(
                    gen_tag_t::frm_sym_sc(zf_cur_frame, 0,
                        subcarrier_range_.start + num_zf_task_completed)
                        ._tag,
                    EventType::kZF);
                num_zf_task_completed += cfg->zf_block_size;
                if (num_zf_task_completed == num_zf_task_required) {
                    num_zf_task_completed = 0;
                    zf_cur_frame++;
                }
            }
            if (zf_cur_frame > demul_cur_frame
                && demul_cur_frame == rx_stats_->cur_frame) {
                if (rx_stats_->next_data_symbol > demul_cur_symbol_to_process) {
                    computeDemul_->launch(
                        gen_tag_t::frm_sym_sc(demul_cur_frame,
                            demul_cur_symbol_to_process,
                            subcarrier_range_.start + num_demul_task_completed)
                            ._tag,
                        EventType::kDemul);
                    num_demul_task_completed += cfg->demul_block_size;
                    if (num_demul_task_completed == num_demul_task_required) {
                        num_demul_task_completed = 0;
                        demul_cur_symbol_to_process++;
                        if (demul_cur_symbol_to_process
                            == cfg->symbol_num_perframe) {
                            demul_cur_symbol_to_process
                                = cfg->pilot_symbol_num_perframe;
                            demul_cur_frame++;
                        }
                    }
                }
            }
        }
    }

    Event_data run_csi(size_t tag)
    {
        const size_t frame_id = gen_tag_t(tag).frame_id;
        const size_t base_sc_id = gen_tag_t(tag).sc_id;
        const size_t frame_slot = frame_id % TASK_BUFFER_FRAME_NUM;

        rt_assert(base_sc_id == subcarrier_range_.start,
            "Received wrong task for CSI!");

        complex_float converted_sc[kSCsPerCacheline];

        for (size_t i = 0; i < cfg->pilot_symbol_num_perframe; i++) {
            for (size_t j = 0; j < cfg->BS_ANT_NUM; j++) {
                auto* pkt = reinterpret_cast<Packet*>(socket_buffer_[j]
                    + (frame_slot * cfg->symbol_num_perframe
                          * cfg->packet_length)
                    + i * cfg->packet_length); // TODO: find out the packet
                // Subcarrier ranges should be aligned with kTransposeBlockSize
                size_t block_idx_start
                    = subcarrier_range_.start / kTransposeBlockSize;
                size_t block_idx_end
                    = subcarrier_range_.end / kTransposeBlockSize;
                for (size_t block_idx = block_idx_start;
                     block_idx < block_idx_end; block_idx++) {
                    const size_t block_base_offset
                        = block_idx * (kTransposeBlockSize * cfg->BS_ANT_NUM);

                    for (size_t sc_j = 0; sc_j < kTransposeBlockSize;
                         sc_j += kSCsPerCacheline) {
                        const size_t sc_idx
                            = (block_idx * kTransposeBlockSize) + sc_j;

                        simd_convert_float16_to_float32(
                            reinterpret_cast<float*>(pkt->data + sc_idx * 2),
                            reinterpret_cast<float*>(converted_sc),
                            kSCsPerCacheline * 2);

                        const complex_float* src
                            = converted_sc; // TODO: find src pointer from pkt

                        complex_float* dst
                            = cfg->get_csi_mat(csi_buffer_, frame_id, i);
                        dst = &dst[block_base_offset + (j * kTransposeBlockSize)
                            + sc_j];

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
                        __m256 fft_result0 = _mm256_load_ps(
                            reinterpret_cast<const float*>(src));
                        __m256 fft_result1 = _mm256_load_ps(
                            reinterpret_cast<const float*>(src + 4));
                        __m256 pilot_tx0
                            = _mm256_set_ps(cfg->pilots_sgn_[sc_idx + 3].im,
                                cfg->pilots_sgn_[sc_idx + 3].re,
                                cfg->pilots_sgn_[sc_idx + 2].im,
                                cfg->pilots_sgn_[sc_idx + 2].re,
                                cfg->pilots_sgn_[sc_idx + 1].im,
                                cfg->pilots_sgn_[sc_idx + 1].re,
                                cfg->pilots_sgn_[sc_idx].im,
                                cfg->pilots_sgn_[sc_idx].re);
                        fft_result0 = __m256_complex_cf32_mult(
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
                        fft_result1 = __m256_complex_cf32_mult(
                            fft_result1, pilot_tx1, true);
                        _mm256_stream_ps(
                            reinterpret_cast<float*>(dst), fft_result0);
                        _mm256_stream_ps(
                            reinterpret_cast<float*>(dst + 4), fft_result1);
#endif
                    }
                }
            }
        }
        return Event_data(EventType::kCSI, tag);
    }

    /// The subcarrier range handled by this subcarrier doer.
    struct Range subcarrier_range_;

    // TODO: We should use owned objects here instead of pointers, but these
    // buffers depend on some Tables beine malloc-ed
    DoZF* computeZF_;
    DoDemul* computeDemul_;
    DoPrecode* computePrecode_;
    // Reciprocity*  computeReciprocity_;

    // For the following buffers, see the `SubcarrierManager`'s documentation.

    // Input buffers

    Table<char>& socket_buffer_;
    Table<int>& socket_buffer_status_;
    Table<complex_float>& csi_buffer_;
    Table<complex_float>& recip_buffer_;
    Table<complex_float>& calib_buffer_;
    Table<int8_t>& dl_encoded_buffer_;
    Table<complex_float>& data_buffer_;

    // Output buffers

    Table<int8_t>& demod_soft_buffer_;
    Table<complex_float>& dl_ifft_buffer_;

    // Intermediate buffers

    Table<complex_float>& ue_spec_pilot_buffer_;
    Table<complex_float>& equal_buffer_;
    Table<complex_float>& ul_zf_buffer_;
    Table<complex_float>& dl_zf_buffer_;

    // Shared states with TXRX threads
    RxStats* rx_stats_;

    // Internal CSI states
    size_t csi_cur_frame;

    // Internal ZF states
    size_t zf_cur_frame; // Current frame waiting for CSI matrix
    size_t num_zf_task_completed;

    // Internal Demul states
    size_t demul_cur_frame; // Current frame waiting for ZF matrix
    size_t demul_cur_symbol_to_process; // Current data symbol wait to process
    size_t num_demul_task_completed;
};

#endif // DOSUBCARRIER_HPP
