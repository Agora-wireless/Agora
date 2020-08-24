/// Author: Kevin Boos
/// Email: kevinaboos@gmail.com
///
/// @see DoSubcarrier

#ifndef DOSUBCARRIER_HPP
#define DOSUBCARRIER_HPP

// [Kevin]: unsure if all of these includes are needed.
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
#include "reciprocity.hpp"
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
        RxStatus* rx_status = nullptr, DemulStatus* demul_status = nullptr)
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
        , rx_status_(rx_status)
        , demul_status_(demul_status)
    {

        // Create the requisite Doers
        computeZF_ = new DoZF(this->cfg, tid, freq_ghz, this->task_queue_,
            this->complete_task_queue, this->worker_producer_token, csi_buffer_,
            recip_buffer_, ul_zf_buffer_, dl_zf_buffer_, stats);

        computeDemul_ = new DoDemul(this->cfg, tid, freq_ghz, this->task_queue_,
            this->complete_task_queue, this->worker_producer_token,
            data_buffer_, ul_zf_buffer_, ue_spec_pilot_buffer_, equal_buffer_,
            demod_soft_buffer_, phy_stats, stats, &socket_buffer_);

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
        case EventType::kCSI:
            return run_csi(tag);
        case EventType::kZF:
            return computeZF_->launch(tag);
        case EventType::kDemul:
            return computeDemul_->launch(tag);
        case EventType::kPrecode:
            return computePrecode_->launch(tag);
        /// TODO: move reciprocity into Subcarrier doers.
        // case EventType::kRc: {
        //     return computeReciprocity_->launch(tag, event_type);
        // } break;

        // Should never reach below here!
        default:
            break;
        }

        rt_assert(false,
            std::string("[DoSubcarrier::launch] error, unexpected event type ")
                + std::to_string(static_cast<int>(event_type)));
        return Event_data();
    }

    // Returns the range of subcarrier IDs handled by this subcarrier doer.
    Range& subcarrier_range() { return subcarrier_range_; }

    void start_work()
    {
        const size_t num_zf_task_required
            = (subcarrier_range_.end - subcarrier_range_.start)
            / cfg->zf_block_size;
        const size_t num_demul_task_required
            = (subcarrier_range_.end - subcarrier_range_.start)
            / cfg->demul_block_size;

        while (cfg->running && !SignalHandler::gotExitSignal()) {
            if (rx_status_->is_pilot_ready(csi_cur_frame)) {
                run_csi(gen_tag_t::frm_sym_sc(
                    csi_cur_frame, 0, subcarrier_range_.start)
                            ._tag);
                // exit(0);
                printf(
                    "Main thread: pilot frame: %lu, finished CSI for all pilot "
                    "symbols\n",
                    csi_cur_frame);
                csi_cur_frame++;
            }
            if (csi_cur_frame > zf_cur_frame) {
                computeZF_->launch(gen_tag_t::frm_sym_sc(zf_cur_frame, 0,
                    subcarrier_range_.start
                        + num_zf_task_completed * cfg->zf_block_size)
                                       ._tag);
                // exit(0);
                num_zf_task_completed++;
                if (num_zf_task_completed == num_zf_task_required) {
                    num_zf_task_completed = 0;
                    printf("Main thread: ZF done frame: %lu\n", zf_cur_frame);
                    zf_cur_frame++;
                }
            }
            if (zf_cur_frame > demul_cur_frame
                && rx_status_->is_demod_ready(
                       demul_cur_frame, demul_cur_symbol_to_process)) {
                // if (demul_cur_frame == 0) {
                //     printf("Run demod on frame %lu symbol %lu sc %lu\n",
                //         demul_cur_frame, demul_cur_symbol_to_process,
                //         subcarrier_range_.start
                //             + num_demul_task_completed * cfg->demul_block_size);
                // }
                // computeDemul_->launch(gen_tag_t::frm_sym_sc(demul_cur_frame,
                //     demul_cur_symbol_to_process
                //         - cfg->pilot_symbol_num_perframe,
                //     subcarrier_range_.start
                //         + num_demul_task_completed * cfg->demul_block_size)
                //                           ._tag);
                computeDemul_->independent_launch(
                    gen_tag_t::frm_sym_sc(demul_cur_frame,
                        demul_cur_symbol_to_process
                            - cfg->pilot_symbol_num_perframe,
                        subcarrier_range_.start
                            + num_demul_task_completed * cfg->demul_block_size)
                        ._tag);
                num_demul_task_completed++;
                if (num_demul_task_completed == num_demul_task_required) {
                    num_demul_task_completed = 0;
                    demul_status_->demul_complete(demul_cur_frame,
                        demul_cur_symbol_to_process, num_demul_task_required);
                    demul_cur_symbol_to_process++;
                    if (demul_cur_symbol_to_process
                        == cfg->symbol_num_perframe) {
                        demul_cur_symbol_to_process
                            = cfg->pilot_symbol_num_perframe;
                        printf("Main thread: Demodulation done frame: %lu "
                               "(%lu "
                               "UL symbols)\n",
                            demul_cur_frame,
                            cfg->symbol_num_perframe
                                - cfg->pilot_symbol_num_perframe);
                        demul_cur_frame++;
                        // for (size_t i = 0; i < cfg->UE_ANT_NUM; i++) {
                        //     rx_status_->decode_done(demul_cur_frame - 1);
                        // }
                    }
                }
            }
        }
    }

private:
    Event_data run_csi(size_t tag)
    {
        const size_t frame_id = gen_tag_t(tag).frame_id;
        const size_t base_sc_id = gen_tag_t(tag).sc_id;
        const size_t frame_slot = frame_id % TASK_BUFFER_FRAME_NUM;

        rt_assert(base_sc_id == subcarrier_range_.start,
            "Received wrong task for CSI!");

        complex_float converted_sc[kSCsPerCacheline];

        // printf("Run CSI: (%x %x)\n",
        //     *(reinterpret_cast<short*>(&socket_buffer_[0][cfg->packet_length
        //         + Packet::kOffsetOfData])),
        //     *(reinterpret_cast<short*>(&socket_buffer_[0][cfg->packet_length
        //         + Packet::kOffsetOfData + 2])));

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

                        // if (j == 0 && i == 1 && sc_idx == 0) {
                        //     for (size_t t = 0; t < kSCsPerCacheline; t++) {
                        //         printf("(%x %x) ", pkt->data[(sc_idx + t) * 2],
                        //             pkt->data[(sc_idx + t) * 2 + 1]);
                        //     }
                        //     printf("\n");
                        // }

                        simd_convert_float16_to_float32(
                            reinterpret_cast<float*>(converted_sc),
                            reinterpret_cast<float*>(pkt->data
                                + (sc_idx + cfg->OFDM_DATA_START) * 2),
                            kSCsPerCacheline * 2);

                        // if (i == 0 && j == 0) {
                        //     for (size_t t = 0; t < kSCsPerCacheline; t++) {
                        //         printf("(%f %f) ", converted_sc[t].re,
                        //             converted_sc[t].im);
                        //     }
                        //     printf("\n");
                        //     // exit(0);
                        // }

                        const complex_float* src
                            = converted_sc; // TODO: find src pointer from pkt

                        // if (j == 0 && i == 0 && sc_idx == 0) {
                        //     printf("src data: (%f %f)\n", src[0].re, src[0].im);
                        // }

                        complex_float* dst
                            = cfg->get_csi_mat(csi_buffer_, frame_id, i);
                        dst = &dst[block_base_offset + (j * kTransposeBlockSize)
                            + sc_j];

                        // With either of AVX-512 or AVX2, load one cacheline =
                        // 16 float values = 8 subcarriers = kSCsPerCacheline
                        // TODO: AVX512 complex multiply support below

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
                        _mm256_stream_ps(
                            reinterpret_cast<float*>(dst), fft_result0);
                        _mm256_stream_ps(
                            reinterpret_cast<float*>(dst + 4), fft_result1);
                        // if (i == 0 && j == 0) {
                        //     for (size_t t = 0; t < kSCsPerCacheline; t++) {
                        //         printf("(%f %f) ", dst[t].re, dst[t].im);
                        //     }
                        //     printf("\n");
                        //     exit(0);
                        // }
                    }
                }
            }
        }
        // printf("(%f %f)\n", csi_buffer_[1][0].re, csi_buffer_[1][0].im);
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
    RxStatus* rx_status_;

    // Internal CSI states
    size_t csi_cur_frame;

    // Internal ZF states
    size_t zf_cur_frame; // Current frame waiting for CSI matrix
    size_t num_zf_task_completed;

    // Internal Demul states
    size_t demul_cur_frame; // Current frame waiting for ZF matrix
    size_t demul_cur_symbol_to_process; // Current data symbol wait to process
    size_t num_demul_task_completed;

    // Shared status with Decode threads
    DemulStatus* demul_status_;
};

#endif // DOSUBCARRIER_HPP
