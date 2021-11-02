#include "dysubcarrier.hpp"

#define TRIGGER_TIMER(stmt) if(likely(start_tsc>0)){stmt;}

DySubcarrier::DySubcarrier(Config* config, int tid, double freq_ghz_,
    /// The range of subcarriers handled by this subcarrier doer.
    Range sc_range,
    // input buffers
    Table<char>& freq_domain_iq_buffer,
    PtrGrid<kFrameWnd, kMaxUEs, complex_float>& csi_buffer,
    Table<complex_float>& calib_buffer, Table<int8_t>& dl_encoded_buffer,
    // output buffers
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& demod_buffer_to_send,
    Table<complex_float>& dl_ifft_buffer,
    // intermediate buffers owned by SubcarrierManager
    Table<complex_float>& equal_buffer,
    PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& ul_zf_matrices,
    PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& dl_zf_matrices,
    std::vector<std::vector<ControlInfo>>& control_info_table,
    std::vector<size_t>& control_idx_list,
    SharedState* shared_state)
    : Doer(config, tid, freq_ghz_)
    , sc_range_(sc_range)
    , freq_domain_iq_buffer_(freq_domain_iq_buffer)
    , csi_buffer_(csi_buffer)
    , calib_buffer_(calib_buffer)
    , dl_encoded_buffer_(dl_encoded_buffer)
    , demod_buffer_to_send_(demod_buffer_to_send)
    , dl_ifft_buffer_(dl_ifft_buffer)
    , equal_buffer_(equal_buffer)
    , ul_zf_matrices_(ul_zf_matrices)
    , dl_zf_matrices_(dl_zf_matrices)
    , shared_state_(shared_state)
    , control_info_table_(control_info_table)
    , control_idx_list_(control_idx_list)
{
    // Create the requisite Doers
    do_zf_ = new DyZF(cfg_, tid_, freq_ghz_, csi_buffer_, calib_buffer, ul_zf_matrices_,
        dl_zf_matrices_, control_info_table_, control_idx_list_);

    do_demul_ = new DyDemul(cfg_, tid_, freq_ghz_, freq_domain_iq_buffer_, ul_zf_matrices_,
        equal_buffer_, demod_buffer_to_send_, 
        control_info_table_, control_idx_list_);

    // Init internal states
    demul_cur_sym_ul_ = 0;
}

DySubcarrier::~DySubcarrier()
{
    delete do_zf_;
    delete do_demul_;
}

void DySubcarrier::StartWork()
{
    const size_t n_zf_tasks_reqd
        = (sc_range_.end - sc_range_.start) / cfg_->zf_block_size;
    const size_t n_demul_tasks_reqd
        = (sc_range_.end - sc_range_.start) / cfg_->demul_block_size;
    
    printf("Range [%zu:%zu] starts to work\n", sc_range_.start, sc_range_.end);

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

    while (cfg_->running && !SignalHandler::gotExitSignal()) {
        size_t worked = 0;
        TRIGGER_TIMER(loop_count ++);

        if (zf_cur_frame_ > demul_cur_frame_) {
            state_start_tsc = rdtsc();
            bool ret = shared_state_->received_all_data_pkts(
                    demul_cur_frame_, demul_cur_sym_ul_);
            size_t state_tsc_usage = rdtsc() - state_start_tsc;
            TRIGGER_TIMER(state_operation_duration += state_tsc_usage);

            if (ret) {
                TRIGGER_TIMER(work_tsc_duration += state_tsc_usage);
                work_start_tsc = rdtsc();
                worked = 1;

                size_t demod_start_tsc = rdtsc();
                do_demul_->Launch(demul_cur_frame_,
                    demul_cur_sym_ul_,
                    sc_range_.start
                        + (n_demul_tasks_done_ * cfg_->demul_block_size));
                TRIGGER_TIMER({
                    size_t demod_tmp_tsc = rdtsc() - demod_start_tsc;
                    demod_tsc_duration += demod_tmp_tsc;
                    demod_max = demod_max < demod_tmp_tsc ? demod_tmp_tsc : demod_max;
                    demod_count ++;
                });

                n_demul_tasks_done_++;
                if (n_demul_tasks_done_ == n_demul_tasks_reqd) {
                    n_demul_tasks_done_ = 0;

                    shared_state_->demul_done(
                        demul_cur_frame_, demul_cur_sym_ul_, n_demul_tasks_reqd);

                    demul_cur_sym_ul_++;
                    if (demul_cur_sym_ul_ == cfg_->ul_data_symbol_num_perframe) {
                        demul_cur_sym_ul_ = 0;

                        demod_start_tsc = rdtsc();
                        MLPD_INFO("Main thread (%u): Demodulation done frame: %lu "
                            "(%lu UL symbols)\n",
                            tid_,
                            demul_cur_frame_,
                            cfg_->ul_data_symbol_num_perframe);
                        TRIGGER_TIMER(print_tsc_duration += rdtsc() - demod_start_tsc);

                        demul_cur_frame_++;
                        if (unlikely(demul_cur_frame_ == cfg_->frames_to_test)) {
                            TRIGGER_TIMER(work_tsc_duration += rdtsc() - work_start_tsc);
                            break;
                        }

                        if (cfg_->sleep_mode && shouldSleep(control_info_table_[control_idx_list_[demul_cur_frame_]])) {
                            std::this_thread::sleep_for(std::chrono::microseconds(600));
                        }
                    }
                }

                TRIGGER_TIMER(work_tsc_duration += rdtsc() - work_start_tsc);
                continue;
            }
        }

        if (csi_cur_frame_ > zf_cur_frame_) {
            work_start_tsc = rdtsc();
            worked = 1;

            size_t zf_start_tsc = rdtsc();
            do_zf_->Launch(gen_tag_t::frm_sym_sc(zf_cur_frame_, 0,
                sc_range_.start + n_zf_tasks_done_ * cfg_->zf_block_size)
                            ._tag);
            TRIGGER_TIMER({
                size_t zf_tmp_tsc = rdtsc() - zf_start_tsc;
                zf_tsc_duration += zf_tmp_tsc;
                zf_max = zf_max < zf_tmp_tsc ? zf_tmp_tsc : zf_max;
                zf_count ++;
            });

            n_zf_tasks_done_++;
            if (n_zf_tasks_done_ == n_zf_tasks_reqd) {
                n_zf_tasks_done_ = 0;

                zf_start_tsc = rdtsc();
                MLPD_INFO("Main thread (%u): ZF done frame: %lu\n", tid_, zf_cur_frame_);
                TRIGGER_TIMER(print_tsc_duration += rdtsc() - zf_start_tsc);

                zf_cur_frame_++;
            }

            TRIGGER_TIMER(work_tsc_duration += rdtsc() - work_start_tsc);
            continue;
        }

        size_t state_tsc_usage = 0;
        TRIGGER_TIMER(state_start_tsc = rdtsc());
        bool ret = shared_state_->received_all_pilots(csi_cur_frame_);
        TRIGGER_TIMER({
            state_tsc_usage = rdtsc() - state_start_tsc;
            state_operation_duration += state_tsc_usage;
        });

        if (ret) {
            if (unlikely(start_tsc == 0 && csi_cur_frame_ >= 200)) {
                start_tsc = rdtsc();
            }

            TRIGGER_TIMER(work_tsc_duration += state_tsc_usage);
            work_start_tsc = rdtsc();
            worked = 1;

            size_t csi_start_tsc = rdtsc();
            runCsi(csi_cur_frame_, sc_range_.start);
            TRIGGER_TIMER({
                size_t csi_tmp_tsc = rdtsc() - csi_start_tsc;
                csi_tsc_duration += csi_tmp_tsc;
                csi_max = csi_max < csi_tmp_tsc ? csi_tmp_tsc : csi_max;
                csi_count ++;
            });

            csi_start_tsc = rdtsc();
            MLPD_INFO(
                "Main thread (%u): pilot frame: %lu, finished CSI for all pilot "
                "symbols\n", tid_,
                csi_cur_frame_);
            TRIGGER_TIMER(print_tsc_duration += rdtsc() - csi_start_tsc);

            csi_cur_frame_++;
            TRIGGER_TIMER(work_tsc_duration += rdtsc() - work_start_tsc);
        }

        TRIGGER_TIMER(work_count += worked);
    }

    size_t whole_duration = rdtsc() - start_tsc;
    size_t idle_duration = whole_duration - work_tsc_duration;
    printf("DySubcarrier Thread %u duration stats: total time used %.2lfms, "
        "csi %.2lfms (%.2lf%%, %zu, %.2lfus), zf %.2lfms (%.2lf%%, %zu, %.2lfus, %.2lfus), demod %.2lfms (%.2lf%%, %zu, %.2lfus), "
        "precode %.2lfms (%.2lf%%), print %.2lfms (%.2lf%%), stating "
        "%.2lfms (%.2lf%%), idle %.2lfms (%.2lf%%), working rate (%zu/%zu: %.2lf%%)\n", 
        tid_, cycles_to_ms(whole_duration, freq_ghz_),
        cycles_to_ms(csi_tsc_duration, freq_ghz_), csi_tsc_duration * 100.0f / whole_duration, csi_count, cycles_to_us(csi_max, freq_ghz_),
        cycles_to_ms(zf_tsc_duration, freq_ghz_), zf_tsc_duration * 100.0f / whole_duration, zf_count, cycles_to_us(zf_max, freq_ghz_), do_zf_->GetZfTscPerTask(),
        cycles_to_ms(demod_tsc_duration, freq_ghz_), demod_tsc_duration * 100.0f / whole_duration, demod_count, cycles_to_us(demod_max, freq_ghz_),
        cycles_to_ms(precode_tsc_duration, freq_ghz_), precode_tsc_duration * 100.0f / whole_duration,
        cycles_to_ms(print_tsc_duration, freq_ghz_), print_tsc_duration * 100.0f / whole_duration,
        cycles_to_ms(state_operation_duration, freq_ghz_), state_operation_duration * 100.0f / whole_duration,
        cycles_to_ms(idle_duration, freq_ghz_), idle_duration * 100.0f / whole_duration,
        work_count, loop_count, work_count * 100.0f / loop_count);
    
    if (kDebugPrintDemulStats) {
        do_demul_->PrintOverhead();
    }

    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string filename = cur_directory + "/data/performance_dysubcarrier.txt";
    FILE* fp = fopen(filename.c_str(), "a");
    fprintf(fp, "%zu %zu %zu %zu %zu %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %zu %zu %zu %.2lf %.2lf %.2lf\n", cfg_->BS_ANT_NUM, cfg_->UE_NUM,
        sc_range_.end - sc_range_.start, cfg_->demul_block_size, cfg_->mod_order_bits,
        csi_tsc_duration * 100.0f / whole_duration, zf_tsc_duration * 100.0f / whole_duration,
        demod_tsc_duration * 100.0f / whole_duration, precode_tsc_duration * 100.0f / whole_duration,
        print_tsc_duration * 100.0f / whole_duration, state_operation_duration * 100.0f / whole_duration,
        idle_duration * 100.0f / whole_duration, cycles_to_ms(whole_duration, freq_ghz_),
        csi_count, zf_count, demod_count, 
        cycles_to_us(csi_max, freq_ghz_), cycles_to_us(zf_max, freq_ghz_), cycles_to_us(demod_max, freq_ghz_));
    fclose(fp);
}

void DySubcarrier::runCsi(size_t frame_id, size_t base_sc_id)
{
    const size_t frame_slot = frame_id % kFrameWnd;
    // rt_assert(base_sc_id == sc_range_.start, "Invalid SC in runCsi!");

    complex_float converted_sc[kSCsPerCacheline];

    for (size_t i = 0; i < cfg_->pilot_symbol_num_perframe; i++) {
        for (size_t j = 0; j < cfg_->BS_ANT_NUM; j++) {
            auto* pkt = reinterpret_cast<Packet*>(freq_domain_iq_buffer_[j]
                + (frame_slot * cfg_->symbol_num_perframe
                        * cfg_->packet_length)
                + i * cfg_->packet_length);

            // Subcarrier ranges should be aligned with kTransposeBlockSize
            for (size_t block_idx = sc_range_.start / kTransposeBlockSize;
                    block_idx < sc_range_.end / kTransposeBlockSize;
                    block_idx++) {

                const size_t block_base_offset
                    = block_idx * (kTransposeBlockSize * cfg_->BS_ANT_NUM);

                for (size_t sc_j = 0; sc_j < kTransposeBlockSize;
                        sc_j += kSCsPerCacheline) {
                    const size_t sc_idx
                        = (block_idx * kTransposeBlockSize) + sc_j;

                    simd_convert_float16_to_float32(
                        reinterpret_cast<float*>(converted_sc),
                        reinterpret_cast<float*>(pkt->data_
                            + (cfg_->OFDM_DATA_START + sc_idx) * 2),
                        kSCsPerCacheline * 2);

                    const complex_float* src = converted_sc;
                    complex_float* dst = csi_buffer_[frame_slot][i]
                        + block_base_offset + (j * kTransposeBlockSize)
                        + sc_j;

                    // With either of AVX-512 or AVX2, load one cacheline =
                    // 16 float values = 8 subcarriers = kSCsPerCacheline
                    // TODO: AVX512 complex multiply support below
                    // size_t pilots_sgn_offset = cfg_->bs_server_addr_idx
                    //     * cfg_->get_num_sc_per_server();
                    size_t pilots_sgn_offset = 0;

                    __m256 fft_result0 = _mm256_load_ps(
                        reinterpret_cast<const float*>(src));
                    __m256 fft_result1 = _mm256_load_ps(
                        reinterpret_cast<const float*>(src + 4));
                    __m256 pilot_tx0 = _mm256_set_ps(
                        cfg_->pilots_sgn_[sc_idx + 3 + pilots_sgn_offset].im,
                        cfg_->pilots_sgn_[sc_idx + 3 + pilots_sgn_offset].re,
                        cfg_->pilots_sgn_[sc_idx + 2 + pilots_sgn_offset].im,
                        cfg_->pilots_sgn_[sc_idx + 2 + pilots_sgn_offset].re,
                        cfg_->pilots_sgn_[sc_idx + 1 + pilots_sgn_offset].im,
                        cfg_->pilots_sgn_[sc_idx + 1 + pilots_sgn_offset].re,
                        cfg_->pilots_sgn_[sc_idx + pilots_sgn_offset].im,
                        cfg_->pilots_sgn_[sc_idx + pilots_sgn_offset].re);
                    fft_result0 = CommsLib::__m256_complex_cf32_mult(
                        fft_result0, pilot_tx0, true);

                    __m256 pilot_tx1 = _mm256_set_ps(
                        cfg_->pilots_sgn_[sc_idx + 7 + pilots_sgn_offset].im,
                        cfg_->pilots_sgn_[sc_idx + 7 + pilots_sgn_offset].re,
                        cfg_->pilots_sgn_[sc_idx + 6 + pilots_sgn_offset].im,
                        cfg_->pilots_sgn_[sc_idx + 6 + pilots_sgn_offset].re,
                        cfg_->pilots_sgn_[sc_idx + 5 + pilots_sgn_offset].im,
                        cfg_->pilots_sgn_[sc_idx + 5 + pilots_sgn_offset].re,
                        cfg_->pilots_sgn_[sc_idx + 4 + pilots_sgn_offset].im,
                        cfg_->pilots_sgn_[sc_idx + 4 + pilots_sgn_offset]
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

inline bool DySubcarrier::shouldSleep(std::vector<ControlInfo>& control_list) {
    for (size_t i = 0; i < control_list.size(); i ++) {
        if (!(control_list[i].sc_end < sc_range_.start || control_list[i].sc_start >= sc_range_.end)) {
            return false;
        }
    }
    return true;
}

void DySubcarrier::StartWorkCentral() {
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

    while (cfg_->running && !SignalHandler::gotExitSignal()) {
        EventData event, resp;
        size_t tag;
        size_t slot_id;
        size_t sc_id;
        size_t symbol_id_ul;
        if (task_queue_->try_dequeue(event)) {
            work_start_tsc = rdtsc();
            switch (event.event_type_) {
            case EventType::kCSI:
                tag = event.tags_[0];
                slot_id = gen_tag_t(tag).frame_id;
                sc_id = gen_tag_t(tag).sc_id;
                if (unlikely(start_tsc == 0 && slot_id >= 200)) {
                    start_tsc = rdtsc();
                }
                csi_start_tsc = rdtsc();
                runCsi(slot_id, sc_id);
                if (likely(start_tsc > 0)) {
                    size_t csi_tmp_tsc = rdtsc() - csi_start_tsc;
                    csi_tsc_duration += csi_tmp_tsc;
                    csi_max = csi_max < csi_tmp_tsc ? csi_tmp_tsc : csi_max;
                    csi_count ++;
                }
                resp = EventData(EventType::kCSI);
                if (likely(start_tsc > 0)) {
                    state_start_tsc = rdtsc();
                }
                TryEnqueueFallback(complete_queue_, complete_queue_token_, resp);
                if (likely(start_tsc > 0)) {
                    state_operation_duration += rdtsc() - state_start_tsc;
                }
                break;
            case EventType::kZF:
                tag = event.tags_[0];
                slot_id = gen_tag_t(tag).frame_id;
                zf_start_tsc = rdtsc();
                for (sc_id = sc_range_.start; sc_id < sc_range_.end; sc_id += cfg_->zf_block_size) {
                    do_zf_->Launch(gen_tag_t::frm_sym_sc(zf_cur_frame_, 0, sc_id)._tag);
                }
                if (likely(start_tsc > 0)) {
                    size_t zf_tmp_tsc = rdtsc() - zf_start_tsc;
                    zf_tsc_duration += zf_tmp_tsc;
                    zf_max = zf_max < zf_tmp_tsc ? zf_tmp_tsc : zf_max;
                    zf_count ++;
                }
                resp = EventData(EventType::kZF);
                if (likely(start_tsc > 0)) {
                    state_start_tsc = rdtsc();
                }
                TryEnqueueFallback(complete_queue_, complete_queue_token_, resp);
                if (likely(start_tsc > 0)) {
                    state_operation_duration += rdtsc() - state_start_tsc;
                }
                break;
            case EventType::kDemul:
                tag = event.tags_[0];
                slot_id = gen_tag_t(tag).frame_id;
                symbol_id_ul = gen_tag_t(tag).symbol_id;
                sc_id = gen_tag_t(tag).sc_id;
                demod_start_tsc = rdtsc();
                do_demul_->Launch(slot_id, symbol_id_ul, sc_id);
                if (likely(start_tsc > 0)) {
                    size_t demod_tmp_tsc = rdtsc() - demod_start_tsc;
                    demod_tsc_duration += demod_tmp_tsc;
                    demod_max = demod_max < demod_tmp_tsc ? demod_tmp_tsc : demod_max;
                    demod_count ++;
                }
                resp = EventData(EventType::kDemul, gen_tag_t::frm_sym_sc(slot_id, symbol_id_ul, sc_id)._tag);
                if (likely(start_tsc > 0)) {
                    state_start_tsc = rdtsc();
                }
                TryEnqueueFallback(complete_queue_, complete_queue_token_, resp);
                if (likely(start_tsc > 0)) {
                    state_operation_duration += rdtsc() - state_start_tsc;
                }
                break;
            default:
                perror("Wrong Event Type in DoSubcarrier!");
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
        "csi %.2lfms (%.2lf%%, %zu, %.2lfus), zf %.2lfms (%.2lf%%, %zu, %.2lfus), demod %.2lfms (%.2lf%%, %zu, %.2lfus), "
        "precode %.2lfms (%.2lf%%), print %.2lfms (%.2lf%%), stating "
        "%.2lfms (%.2lf%%), idle %.2lfms (%.2lf%%), working rate (%zu/%zu: %.2lf%%)\n", 
        tid_, cycles_to_ms(whole_duration, freq_ghz_),
        cycles_to_ms(csi_tsc_duration, freq_ghz_), csi_tsc_duration * 100.0f / whole_duration, csi_count, cycles_to_us(csi_max, freq_ghz_),
        cycles_to_ms(zf_tsc_duration, freq_ghz_), zf_tsc_duration * 100.0f / whole_duration, zf_count, cycles_to_us(zf_max, freq_ghz_),
        cycles_to_ms(demod_tsc_duration, freq_ghz_), demod_tsc_duration * 100.0f / whole_duration, demod_count, cycles_to_us(demod_max, freq_ghz_),
        cycles_to_ms(precode_tsc_duration, freq_ghz_), precode_tsc_duration * 100.0f / whole_duration,
        cycles_to_ms(print_tsc_duration, freq_ghz_), print_tsc_duration * 100.0f / whole_duration,
        cycles_to_ms(state_operation_duration, freq_ghz_), state_operation_duration * 100.0f / whole_duration,
        cycles_to_ms(idle_duration, freq_ghz_), idle_duration * 100.0f / whole_duration,
        work_count, loop_count, work_count * 100.0f / loop_count);
}