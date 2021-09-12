#pragma once

#include "Symbols.hpp"
#include "buffer.hpp"
#include "comms-lib.h"
#include "concurrentqueue.h"
#include "config.hpp"
#include "datatype_conversion.h"
#include "dydemul.hpp"
#include "doer.hpp"
#include "doprecode.hpp"
#include "dyzf.hpp"
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
 * buffers, but with this redesign, we are allocating per-DySubcarrier buffers.
 * While this probably is okay for intermediate internal buffers, perhaps we
 * should require (at least initially) that all input and output buffers are
 * shared across all `DySubcarrier` instances.
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
class DySubcarrier : public Doer {
public:
    /// Construct a new Do Subcarrier object
    DySubcarrier(Config* config, int tid, double freq_ghz,
        moodycamel::ConcurrentQueue<Event_data>& task_queue,
        moodycamel::ConcurrentQueue<Event_data>& complete_queue,
        moodycamel::ProducerToken* complete_queue_token,
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
        std::vector<std::vector<ControlInfo>>& control_info_table,
        std::vector<size_t>& control_idx_list,
        PhyStats* phy_stats, Stats* stats, RxStatus* rx_status = nullptr,
        DemulStatus* demul_status = nullptr, PrecodeStatus* precode_status = nullptr)
        : Doer(config, tid, freq_ghz, dummy_conq_, dummy_conq_,
              nullptr /* tok */)
        , task_queue_(task_queue)
        , complete_queue_(complete_queue)
        , complete_queue_token_(complete_queue_token)
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
        , control_info_table_(control_info_table)
        , control_idx_list_(control_idx_list)
        , rx_status_(rx_status)
        , demul_status_(demul_status)
        , precode_status_(precode_status)
    {
        // Create the requisite Doers
        do_zf_ = new DyZF(this->cfg, tid, freq_ghz, dummy_conq_, dummy_conq_,
            nullptr /* ptok */, csi_buffers_, calib_buffer, ul_zf_matrices_,
            dl_zf_matrices_, control_info_table_, control_idx_list_, stats);

        do_demul_ = new DyDemul(this->cfg, tid, freq_ghz, dummy_conq_,
            dummy_conq_, nullptr /* ptok */, ul_zf_matrices_,
            ue_spec_pilot_buffer_, equal_buffer_, demod_buffers_, 
            control_info_table_, control_idx_list_, phy_stats,
            stats, &socket_buffer_);

        do_precode_ = new DoPrecode(this->cfg, tid, freq_ghz, dummy_conq_,
            dummy_conq_, nullptr /* ptok */, dl_zf_matrices_, dl_ifft_buffer_,
            dl_encoded_buffer_, stats);

        // Init internal states
        demul_cur_sym_ul_ = 0;
    }

    ~DySubcarrier()
    {
        delete do_zf_;
        delete do_demul_;
        delete do_precode_;
        // delete computeReciprocity_;
    }

/*
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

        size_t csi_count = 0;
        size_t zf_count = 0;
        size_t demod_count = 0;

        size_t demod_max = 0;
        size_t zf_max = 0;
        size_t csi_max = 0;

        size_t work_start_tsc, state_start_tsc;

        while (cfg->running && !SignalHandler::gotExitSignal()) {
            size_t worked = 0;
            if (likely(start_tsc > 0)) {
                loop_count ++;
            }

            if (cfg->test_mode > 1) {
                continue;
            }

            if (zf_cur_frame_ > demul_cur_frame_) {

                // work_start_tsc = rdtsc();
                state_start_tsc = rdtsc();
                bool ret = rx_status_->is_demod_ready(
                       demul_cur_frame_, demul_cur_sym_ul_);
                size_t state_tsc_usage = rdtsc() - state_start_tsc;
                if (likely(start_tsc > 0)) {
                    state_operation_duration += state_tsc_usage;
                }
                // work_tsc_duration += rdtsc() - work_start_tsc;

                if (ret) {
                    if (likely(start_tsc > 0)) {
                        work_tsc_duration += state_tsc_usage;
                    }
                    work_start_tsc = rdtsc();
                    worked = 1;

                    size_t demod_start_tsc = rdtsc();
                    do_demul_->launch(demul_cur_frame_,
                        demul_cur_sym_ul_,
                        sc_range_.start
                            + (n_demul_tasks_done_ * cfg->demul_block_size));
                    if (likely(start_tsc > 0)) {
                        size_t demod_tmp_tsc = rdtsc() - demod_start_tsc;
                        demod_tsc_duration += demod_tmp_tsc;
                        demod_max = demod_max < demod_tmp_tsc ? demod_tmp_tsc : demod_max;
                        demod_count ++;
                    }

                    n_demul_tasks_done_++;
                    if (n_demul_tasks_done_ == n_demul_tasks_reqd) {
                        n_demul_tasks_done_ = 0;

                        // state_start_tsc = rdtsc();
                        if (cfg->test_mode != 1) {
                            demul_status_->demul_complete(
                                demul_cur_frame_, demul_cur_sym_ul_, n_demul_tasks_reqd);
                        }
                        // state_operation_duration += rdtsc() - state_start_tsc;

                        demul_cur_sym_ul_++;
                        if (demul_cur_sym_ul_ == cfg->ul_data_symbol_num_perframe) {
                            demul_cur_sym_ul_ = 0;

                            demod_start_tsc = rdtsc();
                            MLPD_INFO("Main thread (%u): Demodulation done frame: %lu "
                                "(%lu UL symbols)\n",
                                tid,
                                demul_cur_frame_,
                                cfg->ul_data_symbol_num_perframe);
                            if (likely(start_tsc > 0)) {
                                print_tsc_duration += rdtsc() - demod_start_tsc;
                            }
                            
                            if (cfg->test_mode == 1) {
                                rx_status_->decode_done(demul_cur_frame_);
                            }

                            demul_cur_frame_++;
                            if (unlikely(demul_cur_frame_ == cfg->frames_to_test)) {
                                if (likely(start_tsc > 0)) {
                                    work_tsc_duration += rdtsc() - work_start_tsc;
                                }
                                break;
                            }

                            if (cfg->sleep_mode && should_sleep(control_info_table_[control_idx_list_[demul_cur_frame_]])) {
                                std::this_thread::sleep_for(std::chrono::microseconds(600));
                            }
                        }
                    }

                    if (likely(start_tsc > 0)) {
                        work_tsc_duration += rdtsc() - work_start_tsc;
                    }
                    continue;
                }
                
            }

            // if (precode_status_->received_all_encoded_data(precode_cur_frame_, precode_cur_sym_dl_) 
            //     && zf_cur_frame_ > precode_cur_frame_) {
            //     size_t work_start_tsc = rdtsc();
            //     worked = 1;
            //     size_t base_sc_id = n_precode_tasks_done_ * cfg->demul_block_size + sc_range_.start;
            //     // printf("Precode frame %u symbol %u subcarrier %u\n", precode_cur_frame_, 
            //         // precode_cur_sym_dl_, base_sc_id);
            //     size_t precode_start_tsc = rdtsc();
            //     do_precode_->launch(gen_tag_t::frm_sym_sc(precode_cur_frame_, precode_cur_sym_dl_, base_sc_id)._tag);
            //     precode_tsc_duration += rdtsc() - precode_start_tsc;
            //     n_precode_tasks_done_ ++;
            //     if (n_precode_tasks_done_ == n_precode_tasks_reqd) {
            //         n_precode_tasks_done_ = 0;
            //         precode_cur_sym_dl_ ++;
            //         if (precode_cur_sym_dl_ == cfg->dl_data_symbol_num_perframe) {
            //             precode_cur_sym_dl_ = 0;
            //             // printf("Main thread: Precode done frame: %lu "
            //             //        "(%lu UL symbols)\n",
            //             //     precode_cur_frame_,
            //             //     cfg->dl_data_symbol_num_perframe);
            //             precode_start_tsc = rdtsc();
            //             rx_status_->precode_done(precode_cur_frame_);
            //             state_operation_duration += rdtsc() - precode_start_tsc;
            //             precode_cur_frame_ ++;
            //             if (unlikely(precode_cur_frame_ == cfg->frames_to_test)) {
            //                 work_tsc_duration += rdtsc() - work_start_tsc;
            //                 break;
            //             }
            //         }
            //     }
            //     work_tsc_duration += rdtsc() - work_start_tsc;
            //     continue;
            // }

            if (csi_cur_frame_ > zf_cur_frame_) {
                work_start_tsc = rdtsc();
                worked = 1;

                size_t zf_start_tsc = rdtsc();
                do_zf_->launch(gen_tag_t::frm_sym_sc(zf_cur_frame_, 0,
                    sc_range_.start + n_zf_tasks_done_ * cfg->zf_block_size)
                                   ._tag);
                if (likely(start_tsc > 0)) {
                    size_t zf_tmp_tsc = rdtsc() - zf_start_tsc;
                    zf_tsc_duration += zf_tmp_tsc;
                    zf_max = zf_max < zf_tmp_tsc ? zf_tmp_tsc : zf_max;
                    zf_count ++;
                }

                n_zf_tasks_done_++;
                if (n_zf_tasks_done_ == n_zf_tasks_reqd) {
                    n_zf_tasks_done_ = 0;

                    zf_start_tsc = rdtsc();
                    MLPD_INFO("Main thread (%u): ZF done frame: %lu\n", tid, zf_cur_frame_);
                    if (likely(start_tsc > 0)) {
                        print_tsc_duration += rdtsc() - zf_start_tsc;
                    }

                    zf_cur_frame_++;
                }

                if (likely(start_tsc > 0)) {
                    work_tsc_duration += rdtsc() - work_start_tsc;
                }
                continue;
            }

            size_t state_tsc_usage = 0;
            if (likely(start_tsc > 0)) {
                // work_start_tsc = rdtsc();
                state_start_tsc = rdtsc();
            }
            bool ret = rx_status_->received_all_pilots(csi_cur_frame_);
            if (likely(start_tsc > 0)) {
                state_tsc_usage = rdtsc() - state_start_tsc;
                state_operation_duration += state_tsc_usage;
                // work_tsc_duration += rdtsc() - work_start_tsc;
            }

            if (ret) {
                if (unlikely(start_tsc == 0 && csi_cur_frame_ >= 200)) {
                    start_tsc = rdtsc();
                }

                if (likely(start_tsc > 0)) {
                    work_tsc_duration += state_tsc_usage;
                }
                work_start_tsc = rdtsc();
                worked = 1;

                size_t csi_start_tsc = rdtsc();
                run_csi(csi_cur_frame_, sc_range_.start);
                if (likely(start_tsc > 0)) {
                    size_t csi_tmp_tsc = rdtsc() - csi_start_tsc;
                    csi_tsc_duration += csi_tmp_tsc;
                    csi_max = csi_max < csi_tmp_tsc ? csi_tmp_tsc : csi_max;
                    csi_count ++;
                }

                csi_start_tsc = rdtsc();
                MLPD_INFO(
                    "Main thread (%u): pilot frame: %lu, finished CSI for all pilot "
                    "symbols\n", tid,
                    csi_cur_frame_);
                if (likely(start_tsc > 0)) {
                    print_tsc_duration += rdtsc() - csi_start_tsc;
                }

                csi_cur_frame_++;
                // while (should_sleep(control_info_table_[control_idx_list_[csi_cur_frame_]])) {
                //     csi_cur_frame_ ++;
                //     usleep(800); // TODO: decide the sleeping length
                // }
                if (likely(start_tsc > 0)) {
                    work_tsc_duration += rdtsc() - work_start_tsc;
                }
            }

            if (likely(start_tsc > 0)) {
                work_count += worked;
            }
        }

        size_t whole_duration = rdtsc() - start_tsc;
        size_t idle_duration = whole_duration - work_tsc_duration;
        printf("DySubcarrier Thread %u duration stats: total time used %.2lfms, "
            "csi %.2lfms (%.2lf\%, %u, %.2lfus), zf %.2lfms (%.2lf\%, %u, %.2lfus), demod %.2lfms (%.2lf\%, %u, %.2lfus), "
            "precode %.2lfms (%.2lf\%), print %.2lfms (%.2lf\%), stating "
            "%.2lfms (%.2lf\%), idle %.2lfms (%.2lf\%), working rate (%u/%u: %.2lf\%)\n", 
            tid, cycles_to_ms(whole_duration, freq_ghz),
            cycles_to_ms(csi_tsc_duration, freq_ghz), csi_tsc_duration * 100.0f / whole_duration, csi_count, cycles_to_us(csi_max, freq_ghz),
            cycles_to_ms(zf_tsc_duration, freq_ghz), zf_tsc_duration * 100.0f / whole_duration, zf_count, cycles_to_us(zf_max, freq_ghz),
            cycles_to_ms(demod_tsc_duration, freq_ghz), demod_tsc_duration * 100.0f / whole_duration, demod_count, cycles_to_us(demod_max, freq_ghz),
            cycles_to_ms(precode_tsc_duration, freq_ghz), precode_tsc_duration * 100.0f / whole_duration,
            cycles_to_ms(print_tsc_duration, freq_ghz), print_tsc_duration * 100.0f / whole_duration,
            cycles_to_ms(state_operation_duration, freq_ghz), state_operation_duration * 100.0f / whole_duration,
            cycles_to_ms(idle_duration, freq_ghz), idle_duration * 100.0f / whole_duration,
            work_count, loop_count, work_count * 100.0f / loop_count);
        // do_demul_->print_overhead();

        std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
        std::string filename = cur_directory + "/data/performance_dysubcarrier.txt";
        FILE* fp = fopen(filename.c_str(), "a");
        fprintf(fp, "%u %u %u %u %u %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %u %u %u %.2lf %.2lf %.2lf\n", cfg->BS_ANT_NUM, cfg->UE_NUM,
            sc_range_.end - sc_range_.start, cfg->demul_block_size, cfg->mod_order_bits,
            csi_tsc_duration * 100.0f / whole_duration, zf_tsc_duration * 100.0f / whole_duration,
            demod_tsc_duration * 100.0f / whole_duration, precode_tsc_duration * 100.0f / whole_duration,
            print_tsc_duration * 100.0f / whole_duration, state_operation_duration * 100.0f / whole_duration,
            idle_duration * 100.0f / whole_duration, cycles_to_ms(whole_duration, freq_ghz),
            csi_count, zf_count, demod_count, 
            cycles_to_us(csi_max, freq_ghz), cycles_to_us(zf_max, freq_ghz), cycles_to_us(demod_max, freq_ghz));
        fclose(fp);
    }
    */

    void start_work() {
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

        size_t csi_count = 0;
        size_t zf_count = 0;
        size_t demod_count = 0;

        size_t demod_max = 0;
        size_t zf_max = 0;
        size_t csi_max = 0;

        size_t work_start_tsc, state_start_tsc;
        size_t csi_start_tsc, zf_start_tsc, demod_start_tsc;

        while (cfg->running && !SignalHandler::gotExitSignal()) {
            Event_data event, resp;
            size_t tag;
            size_t slot_id;
            size_t sc_id;
            size_t symbol_id_ul;
            if (task_queue_.try_dequeue(event)) {
                work_start_tsc = rdtsc();
                switch (event.event_type) {
                case EventType::kCSI:
                    tag = event.tags[0];
                    slot_id = gen_tag_t(tag).frame_id;
                    sc_id = gen_tag_t(tag).sc_id;
                    if (unlikely(start_tsc == 0 && slot_id >= 200)) {
                        start_tsc = rdtsc();
                    }
                    csi_start_tsc = rdtsc();
                    run_csi(slot_id, sc_id);
                    if (likely(start_tsc > 0)) {
                        size_t csi_tmp_tsc = rdtsc() - csi_start_tsc;
                        csi_tsc_duration += csi_tmp_tsc;
                        csi_max = csi_max < csi_tmp_tsc ? csi_tmp_tsc : csi_max;
                        csi_count ++;
                    }
                    resp = Event_data(EventType::kCSI);
                    // printf("[Thread %u] Producer token: %p\n", tid, complete_queue_token_);
                    if (likely(start_tsc > 0)) {
                        state_start_tsc = rdtsc();
                    }
                    try_enqueue_fallback(&complete_queue_, complete_queue_token_, resp);
                    if (likely(start_tsc > 0)) {
                        state_operation_duration += rdtsc() - state_start_tsc;
                    }
                    break;
                case EventType::kZF:
                    tag = event.tags[0];
                    slot_id = gen_tag_t(tag).frame_id;
                    zf_start_tsc = rdtsc();
                    for (sc_id = sc_range_.start; sc_id < sc_range_.end; sc_id += cfg->zf_block_size) {
                        do_zf_->launch(gen_tag_t::frm_sym_sc(zf_cur_frame_, 0, sc_id)._tag);
                    }
                    if (likely(start_tsc > 0)) {
                        size_t zf_tmp_tsc = rdtsc() - zf_start_tsc;
                        zf_tsc_duration += zf_tmp_tsc;
                        zf_max = zf_max < zf_tmp_tsc ? zf_tmp_tsc : zf_max;
                        zf_count ++;
                    }
                    resp = Event_data(EventType::kZF);
                    // printf("[Thread %u] Producer token: %p\n", tid, complete_queue_token_);
                    if (likely(start_tsc > 0)) {
                        state_start_tsc = rdtsc();
                    }
                    try_enqueue_fallback(&complete_queue_, complete_queue_token_, resp);
                    if (likely(start_tsc > 0)) {
                        state_operation_duration += rdtsc() - state_start_tsc;
                    }
                    break;
                case EventType::kDemul:
                    tag = event.tags[0];
                    slot_id = gen_tag_t(tag).frame_id;
                    symbol_id_ul = gen_tag_t(tag).symbol_id;
                    sc_id = gen_tag_t(tag).sc_id;
                    demod_start_tsc = rdtsc();
                    do_demul_->launch(slot_id, symbol_id_ul, sc_id);
                    if (likely(start_tsc > 0)) {
                        size_t demod_tmp_tsc = rdtsc() - demod_start_tsc;
                        demod_tsc_duration += demod_tmp_tsc;
                        demod_max = demod_max < demod_tmp_tsc ? demod_tmp_tsc : demod_max;
                        demod_count ++;
                    }
                    resp = Event_data(EventType::kDemul, gen_tag_t::frm_sym_sc(slot_id, symbol_id_ul, sc_id)._tag);
                    // printf("[Thread %u] Producer token: %p\n", tid, complete_queue_token_);
                    if (likely(start_tsc > 0)) {
                        state_start_tsc = rdtsc();
                    }
                    try_enqueue_fallback(&complete_queue_, complete_queue_token_, resp);
                    if (likely(start_tsc > 0)) {
                        state_operation_duration += rdtsc() - state_start_tsc;
                    }
                    break;
                }
                if (likely(start_tsc > 0)) {
                    work_tsc_duration += rdtsc() - work_start_tsc;
                }
            }
        }

        size_t whole_duration = rdtsc() - start_tsc;
        size_t idle_duration = whole_duration - work_tsc_duration;
        printf("DySubcarrier Thread %u duration stats: total time used %.2lfms, "
            "csi %.2lfms (%.2lf\%, %u, %.2lfus), zf %.2lfms (%.2lf\%, %u, %.2lfus), demod %.2lfms (%.2lf\%, %u, %.2lfus), "
            "precode %.2lfms (%.2lf\%), print %.2lfms (%.2lf\%), stating "
            "%.2lfms (%.2lf\%), idle %.2lfms (%.2lf\%), working rate (%u/%u: %.2lf\%)\n", 
            tid, cycles_to_ms(whole_duration, freq_ghz),
            cycles_to_ms(csi_tsc_duration, freq_ghz), csi_tsc_duration * 100.0f / whole_duration, csi_count, cycles_to_us(csi_max, freq_ghz),
            cycles_to_ms(zf_tsc_duration, freq_ghz), zf_tsc_duration * 100.0f / whole_duration, zf_count, cycles_to_us(zf_max, freq_ghz),
            cycles_to_ms(demod_tsc_duration, freq_ghz), demod_tsc_duration * 100.0f / whole_duration, demod_count, cycles_to_us(demod_max, freq_ghz),
            cycles_to_ms(precode_tsc_duration, freq_ghz), precode_tsc_duration * 100.0f / whole_duration,
            cycles_to_ms(print_tsc_duration, freq_ghz), print_tsc_duration * 100.0f / whole_duration,
            cycles_to_ms(state_operation_duration, freq_ghz), state_operation_duration * 100.0f / whole_duration,
            cycles_to_ms(idle_duration, freq_ghz), idle_duration * 100.0f / whole_duration,
            work_count, loop_count, work_count * 100.0f / loop_count);
    }

// private:
    void run_csi(size_t frame_id, size_t base_sc_id)
    {
        const size_t frame_slot = frame_id % kFrameWnd;
        // rt_assert(base_sc_id == sc_range_.start, "Invalid SC in run_csi!");

        size_t sc_start = base_sc_id;
        size_t sc_end = base_sc_id + cfg->zf_block_size;
        sc_end = sc_end > cfg->subcarrier_end ? cfg->subcarrier_end : sc_end;

        complex_float converted_sc[kSCsPerCacheline];

        for (size_t i = 0; i < cfg->pilot_symbol_num_perframe; i++) {
            for (size_t j = 0; j < cfg->BS_ANT_NUM; j++) {
                auto* pkt = reinterpret_cast<Packet*>(socket_buffer_[j]
                    + (frame_slot * cfg->symbol_num_perframe
                          * cfg->packet_length)
                    + i * cfg->packet_length);

                // Subcarrier ranges should be aligned with kTransposeBlockSize
                // for (size_t block_idx = sc_range_.start / kTransposeBlockSize;
                //      block_idx < sc_range_.end / kTransposeBlockSize;
                //      block_idx++) {
                for (size_t block_idx = sc_start / kTransposeBlockSize;
                     block_idx < sc_end / kTransposeBlockSize;
                     block_idx++) {

                    const size_t block_base_offset
                        = block_idx * (kTransposeBlockSize * cfg->BS_ANT_NUM);

                    for (size_t sc_j = 0; sc_j < kTransposeBlockSize;
                         sc_j += kSCsPerCacheline) {
                        const size_t sc_idx
                            = (block_idx * kTransposeBlockSize) + sc_j;

                        memcpy(converted_sc, pkt->data
                                + (cfg->OFDM_DATA_START + sc_idx) * 2,
                            kSCsPerCacheline * 2 * sizeof(short));
                        memcpy(converted_sc + kSCsPerCacheline / 2,
                            pkt->data + (cfg->OFDM_DATA_START + sc_idx) * 2,
                            kSCsPerCacheline * 2 * sizeof(short));
                        for (size_t i = 0; i < kSCsPerCacheline; i ++) {
                            converted_sc[i].re ++;
                            converted_sc[i].im ++;
                        }

                        const complex_float* src = converted_sc;
                        complex_float* dst = csi_buffers_[frame_slot][i]
                            + block_base_offset + (j * kTransposeBlockSize)
                            + sc_j;

                        memcpy(dst, converted_sc, kSCsPerCacheline * sizeof(complex_float));
                    }
                }
            }
        }
    }

    inline bool should_sleep(std::vector<ControlInfo>& control_list) {
        for (size_t i = 0; i < control_list.size(); i ++) {
            if (!(control_list[i].sc_end < sc_range_.start || control_list[i].sc_start >= sc_range_.end)) {
                return false;
            }
        }
        return true;
    }

    /// The subcarrier range handled by this subcarrier doer.
    struct Range sc_range_;

    DyZF* do_zf_;
    DyDemul* do_demul_;
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

    moodycamel::ConcurrentQueue<Event_data>& task_queue_;
    moodycamel::ConcurrentQueue<Event_data>& complete_queue_;
    moodycamel::ProducerToken* complete_queue_token_;

    // Control info
    std::vector<std::vector<ControlInfo>>& control_info_table_;
    std::vector<size_t>& control_idx_list_;
};
