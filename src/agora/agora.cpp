#include "agora.hpp"
using namespace std;

Agora::Agora(Config* cfg)
    : freq_ghz(measure_rdtsc_freq())
    , base_worker_core_offset(cfg->core_offset + 1 + cfg->socket_thread_num)
    , csi_buffers_(kFrameWnd, cfg->UE_NUM, cfg->BS_ANT_NUM * cfg->OFDM_DATA_NUM)
    , ul_zf_matrices_(
          kFrameWnd, cfg->OFDM_DATA_NUM, cfg->BF_ANT_NUM * cfg->UE_NUM)
    , demod_buffers_(kFrameWnd, cfg->symbol_num_perframe, cfg->UE_NUM,
          kMaxModType * cfg->OFDM_DATA_NUM)
    , decoded_buffer_(kFrameWnd, cfg->symbol_num_perframe, cfg->UE_NUM,
          cfg->LDPC_config.nblocksInSymbol * roundup<64>(cfg->num_bytes_per_cb))
    , dl_zf_matrices_(
          kFrameWnd, cfg->OFDM_DATA_NUM, cfg->UE_NUM * cfg->BF_ANT_NUM)
{
    std::string directory = TOSTRING(PROJECT_DIRECTORY);
    printf("Agora: project directory [%s], RDTSC frequency = %.2f GHz\n",
        directory.c_str(), freq_ghz);

    this->config_ = cfg;

    pin_to_core_with_offset(
        ThreadType::kMaster, cfg->core_offset, 0, false /* quiet */);
    initialize_queues();
    initialize_uplink_buffers();

    if (config_->dl_data_symbol_num_perframe > 0) {
        printf("Agora: Initializing downlink buffers\n");
        initialize_downlink_buffers();
    }

    stats = new Stats(cfg, kMaxStatBreakdown, freq_ghz);
    phy_stats = new PhyStats(cfg);

    /* Initialize TXRX threads */
    packet_tx_rx_.reset(
        new PacketTXRX(cfg, cfg->core_offset + 1, &message_queue_,
            get_conq(EventType::kPacketTX), rx_ptoks_ptr, tx_ptoks_ptr));

    if (kEnableMac) {
        const size_t mac_cpu_core = cfg->core_offset + cfg->socket_thread_num
            + cfg->worker_thread_num + 1;
        mac_thread_ = new MacThread(MacThread::Mode::kServer, cfg, mac_cpu_core,
            decoded_buffer_, nullptr /* ul bits */,
            nullptr /* ul bits status */, &dl_bits_buffer_,
            &dl_bits_buffer_status_, &mac_request_queue_, &mac_response_queue_);

        mac_std_thread_ = std::thread(&MacThread::run_event_loop, mac_thread_);
    }

    /* Create worker threads */
    if (config_->bigstation_mode) {
        create_threads(pthread_fun_wrapper<Agora, &Agora::worker_fft>, 0,
            cfg->fft_thread_num);
        create_threads(pthread_fun_wrapper<Agora, &Agora::worker_zf>,
            cfg->fft_thread_num, cfg->fft_thread_num + cfg->zf_thread_num);
        create_threads(pthread_fun_wrapper<Agora, &Agora::worker_demul>,
            cfg->fft_thread_num + cfg->zf_thread_num,
            cfg->fft_thread_num + cfg->zf_thread_num + cfg->demul_thread_num);
        create_threads(pthread_fun_wrapper<Agora, &Agora::worker_decode>,
            cfg->fft_thread_num + cfg->zf_thread_num + cfg->demul_thread_num,
            cfg->worker_thread_num);
    } else {
        create_threads(pthread_fun_wrapper<Agora, &Agora::worker>, 0,
            cfg->worker_thread_num);
    }

    printf("Master thread core %zu, TX/RX thread cores %zu--%zu, worker thread "
           "cores %zu--%zu\n",
        cfg->core_offset, cfg->core_offset + 1,
        cfg->core_offset + 1 + cfg->socket_thread_num - 1,
        base_worker_core_offset,
        base_worker_core_offset + cfg->worker_thread_num - 1);
}

Agora::~Agora()
{
    free_uplink_buffers();
    /* Downlink */
    if (config_->dl_data_symbol_num_perframe > 0)
        free_downlink_buffers();

    if (kEnableMac)
        mac_std_thread_.join();
    delete mac_thread_;
}

void Agora::stop()
{
    std::cout << "Agora: stopping threads" << std::endl;
    config_->running = false;
    usleep(1000);
    packet_tx_rx_.reset();
}

void Agora::send_snr_report(
    EventType event_type, size_t frame_id, size_t symbol_id)
{
    assert(event_type == EventType::kSNRReport);
    auto base_tag = gen_tag_t::frm_sym_ue(frame_id, symbol_id, 0);
    for (size_t i = 0; i < config_->UE_NUM; i++) {
        Event_data snr_report(EventType::kSNRReport, base_tag._tag);
        snr_report.num_tags = 2;
        float snr = phy_stats->get_evm_snr(frame_id, i);
        memcpy(&snr_report.tags[1], &snr, sizeof(float));
        try_enqueue_fallback(&mac_request_queue_, snr_report);
        base_tag.ue_id++;
    }
}

void Agora::schedule_antennas(
    EventType event_type, size_t frame_id, size_t symbol_id)
{
    assert(event_type == EventType::kFFT or event_type == EventType::kIFFT);
    auto base_tag = gen_tag_t::frm_sym_ant(frame_id, symbol_id, 0);

    size_t num_blocks = config_->BS_ANT_NUM / config_->fft_block_size;
    size_t num_remainder = config_->BS_ANT_NUM % config_->fft_block_size;
    if (num_remainder > 0)
        num_blocks++;
    Event_data event;
    event.num_tags = config_->fft_block_size;
    event.event_type = event_type;
    for (size_t i = 0; i < num_blocks; i++) {
        if ((i == num_blocks - 1) && num_remainder > 0)
            event.num_tags = num_remainder;
        for (size_t j = 0; j < event.num_tags; j++) {
            event.tags[j] = base_tag._tag;
            base_tag.ant_id++;
        }
        try_enqueue_fallback(get_conq(event_type), get_ptok(event_type), event);
    }
}

void Agora::schedule_subcarriers(
    EventType event_type, size_t frame_id, size_t symbol_id)
{
    auto base_tag = gen_tag_t::frm_sym_sc(frame_id, symbol_id, 0);
    size_t num_events = SIZE_MAX;
    size_t block_size = SIZE_MAX;

    switch (event_type) {
    case EventType::kDemul:
    case EventType::kPrecode:
        num_events = config_->demul_events_per_symbol;
        block_size = config_->demul_block_size;
        break;
    case EventType::kZF:
        num_events = config_->zf_events_per_symbol;
        block_size = config_->zf_block_size;
        break;
    default:
        assert(false);
    }

    if (event_type == EventType::kZF) {
        Event_data event;
        event.event_type = event_type;
        event.num_tags = config_->zf_batch_size;
        size_t num_blocks = num_events / event.num_tags;
        size_t num_remainder = num_events % event.num_tags;
        if (num_remainder > 0)
            num_blocks++;
        for (size_t i = 0; i < num_blocks; i++) {
            if ((i == num_blocks - 1) && num_remainder > 0)
                event.num_tags = num_remainder;
            for (size_t j = 0; j < event.num_tags; j++) {
                event.tags[j] = gen_tag_t::frm_sym_sc(
                    frame_id, symbol_id, block_size * (i * event.num_tags + j))
                                    ._tag;
            }
            try_enqueue_fallback(
                get_conq(event_type), get_ptok(event_type), event);
        }
    } else {
        for (size_t i = 0; i < num_events; i++) {
            try_enqueue_fallback(get_conq(event_type), get_ptok(event_type),
                Event_data(event_type, base_tag._tag));
            base_tag.sc_id += block_size;
        }
    }
}

void Agora::schedule_codeblocks(
    EventType event_type, size_t frame_id, size_t symbol_idx)
{
    auto base_tag = gen_tag_t::frm_sym_cb(frame_id, symbol_idx, 0);

    // for (size_t i = 0;
    //      i < config_->UE_NUM * config_->LDPC_config.nblocksInSymbol; i++) {
    //     try_enqueue_fallback(get_conq(event_type), get_ptok(event_type),
    //         Event_data(event_type, base_tag._tag));
    //     base_tag.cb_id++;
    // }
    size_t num_tasks = config_->UE_NUM * config_->LDPC_config.nblocksInSymbol;
    size_t num_blocks = num_tasks / config_->encode_block_size;
    size_t num_remainder = num_tasks % config_->encode_block_size;
    if (num_remainder > 0)
        num_blocks++;
    Event_data event;
    event.num_tags = config_->encode_block_size;
    event.event_type = event_type;
    for (size_t i = 0; i < num_blocks; i++) {
        if ((i == num_blocks - 1) && num_remainder > 0)
            event.num_tags = num_remainder;
        for (size_t j = 0; j < event.num_tags; j++) {
            event.tags[j] = base_tag._tag;
            base_tag.cb_id++;
        }
        try_enqueue_fallback(get_conq(event_type), get_ptok(event_type), event);
    }
}

void Agora::schedule_users(
    EventType event_type, size_t frame_id, size_t symbol_id)
{
    assert(event_type == EventType::kPacketToMac);
    auto base_tag = gen_tag_t::frm_sym_ue(frame_id, symbol_id, 0);

    for (size_t i = 0; i < config_->UE_NUM; i++) {
        try_enqueue_fallback(&mac_request_queue_,
            Event_data(EventType::kPacketToMac, base_tag._tag));
        base_tag.ue_id++;
    }
}

void Agora::move_events_between_queues(
    EventType event_type1, EventType event_type2)
{
    auto q1 = get_conq(event_type1);
    auto q2 = get_conq(event_type2);
    Event_data events_list[16];
    // printf("%zu elements in decode queue\n", q1->size_approx());
    while (q1->size_approx() > 0) {
        size_t num_events = q1->try_dequeue_bulk(events_list, 16);
        q2->try_enqueue_bulk(events_list, num_events);
    }
}

void Agora::start()
{
    auto& cfg = config_;

    // Start packet I/O
    if (!packet_tx_rx_->startTXRX(socket_buffer_, socket_buffer_status_,
            socket_buffer_status_size_, stats->frame_start, dl_socket_buffer_,
            calib_dl_buffer_, calib_ul_buffer_)) {
        this->stop();
        return;
    }

    pin_to_core_with_offset(
        ThreadType::kMaster, cfg->core_offset, 0, false /* quiet */);

    // Agora processes a frame after processing for previous frames is
    // complete. cur_frame_id is the frame that is currently being processed.
    size_t cur_frame_id = 0;

    // Counters for printing summary
    size_t demul_count = 0;
    size_t tx_count = 0;
    double demul_begin = get_time_us();
    double tx_begin = get_time_us();

    bool is_turn_to_dequeue_from_io = true;
    const size_t max_events_needed = std::max(
        kDequeueBulkSizeTXRX * (cfg->socket_thread_num + 1 /* MAC */),
        kDequeueBulkSizeWorker * cfg->worker_thread_num);
    Event_data events_list[max_events_needed];

    while (config_->running && !SignalHandler::gotExitSignal()) {
        // Get a batch of events
        size_t num_events = 0;
        if (is_turn_to_dequeue_from_io) {
            for (size_t i = 0; i < cfg->socket_thread_num; i++) {
                num_events += message_queue_.try_dequeue_bulk_from_producer(
                    *(rx_ptoks_ptr[i]), events_list + num_events,
                    kDequeueBulkSizeTXRX);
            }
            num_events += mac_response_queue_.try_dequeue_bulk(
                events_list + num_events, kDequeueBulkSizeTXRX);
        } else {
            if (!cfg->downlink_mode)
                num_events += complete_decode_task_queue_.try_dequeue_bulk(
                    events_list + num_events, max_events_needed);
            if (num_events == 0) {
                num_events += complete_task_queue_.try_dequeue_bulk(
                    events_list + num_events, max_events_needed);
            }
        }
        is_turn_to_dequeue_from_io = !is_turn_to_dequeue_from_io;

        // Handle each event
        for (size_t ev_i = 0; ev_i < num_events; ev_i++) {
            Event_data& event = events_list[ev_i];

            // FFT processing is scheduled after falling through the switch
            switch (event.event_type) {
            case EventType::kPacketRX: {
                size_t socket_thread_id = rx_tag_t(event.tags[0]).tid;
                size_t sock_buf_offset = rx_tag_t(event.tags[0]).offset;
                auto* pkt = (Packet*)(socket_buffer_[socket_thread_id]
                    + (sock_buf_offset * cfg->packet_length));

                if (pkt->frame_id >= cur_frame_id + kFrameWnd) {
                    printf("Error: Received packet for future frame %u beyond "
                           "frame window (= %zu + %zu). This can happen if "
                           "Agora is running slowly, e.g., in debug mode\n",
                        pkt->frame_id, cur_frame_id, kFrameWnd);
                    cfg->running = false;
                    break;
                }

                update_rx_counters(pkt->frame_id, pkt->symbol_id);
                fft_queue_arr[pkt->frame_id % kFrameWnd].push(
                    fft_req_tag_t(event.tags[0]));
            } break;

            case EventType::kFFT: {
                for (size_t i = 0; i < event.num_tags; i++) {
                    handle_event_fft(event.tags[i]);
                }
            } break;

            case EventType::kZF: {
                for (size_t tag_id = 0; tag_id < event.num_tags; tag_id++) {
                    size_t frame_id = gen_tag_t(event.tags[tag_id]).frame_id;
                    print_per_task_done(PrintType::kZF, frame_id, 0,
                        zf_stats_.get_symbol_count(frame_id));
                    if (zf_stats_.last_symbol(frame_id)) {
                        stats->master_set_tsc(TsType::kZFDone, frame_id);
                        zf_stats_.coded_frame = frame_id;
                        print_per_frame_done(PrintType::kZF, frame_id);

                        // If all the data in a frame has arrived when ZF is done
                        for (size_t i = 0; i < cfg->ul_data_symbol_num_perframe;
                             i++) {
                            if (fft_stats_.cur_frame_for_symbol[i]
                                == frame_id) {
                                schedule_subcarriers(
                                    EventType::kDemul, frame_id, i);
                            }
                        }
                        // Schedule precoding for downlink symbols
                        for (size_t i = 0; i < cfg->dl_data_symbol_num_perframe;
                             i++) {
                            if (encode_stats_.cur_frame_for_symbol[i]
                                == frame_id) {
                                schedule_subcarriers(EventType::kPrecode,
                                    frame_id, cfg->DLSymbols[0][i]);
                            }
                        }
                    }
                }
            } break;

            case EventType::kDemul: {
                size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
                size_t symbol_idx_ul = gen_tag_t(event.tags[0]).symbol_id;
                size_t base_sc_id = gen_tag_t(event.tags[0]).sc_id;

                print_per_task_done(
                    PrintType::kDemul, frame_id, symbol_idx_ul, base_sc_id);
                /* If this symbol is ready */
                if (demul_stats_.last_task(frame_id, symbol_idx_ul)) {
                    if (demul_stats_.get_symbol_count(frame_id)
                        < demul_stats_.max_symbol_count - 1)
                        schedule_codeblocks(
                            EventType::kDecode, frame_id, symbol_idx_ul);
                    print_per_symbol_done(
                        PrintType::kDemul, frame_id, symbol_idx_ul);
                    if (demul_stats_.last_symbol(frame_id)) {
                        max_equaled_frame = frame_id;
                        if (!cfg->bigstation_mode) {
                            assert(cur_frame_id == frame_id);
                            cur_frame_id++;
                            move_events_between_queues(
                                EventType::kDecode, EventType::kDecodeLast);
                            schedule_codeblocks(EventType::kDecodeLast,
                                frame_id, symbol_idx_ul);
                        } else {
                            schedule_codeblocks(
                                EventType::kDecode, frame_id, symbol_idx_ul);
                        }
                        stats->master_set_tsc(TsType::kDemulDone, frame_id);
                        print_per_frame_done(PrintType::kDemul, frame_id);
                    }

                    demul_count++;
                    if (demul_count == demul_stats_.max_symbol_count * 9000) {
                        demul_count = 0;
                        double diff = get_time_us() - demul_begin;
                        int samples_num_per_UE = cfg->OFDM_DATA_NUM
                            * demul_stats_.max_symbol_count * 1000;
                        printf(
                            "Frame %zu: RX %d samples (per-client) from %zu "
                            "clients in %f secs, throughtput %f bps per-client "
                            "(16QAM), current task queue length %zu\n",
                            frame_id, samples_num_per_UE, cfg->UE_NUM, diff,
                            samples_num_per_UE * log2(16.0f) / diff,
                            get_conq(EventType::kFFT)->size_approx());
                        demul_begin = get_time_us();
                    }
                }
            } break;

            case EventType::kDecode: {
                size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
                size_t symbol_idx_ul = gen_tag_t(event.tags[0]).symbol_id;

                if (decode_stats_.last_task(frame_id, symbol_idx_ul)) {
                    if (kEnableMac) {
                        schedule_users(
                            EventType::kPacketToMac, frame_id, symbol_idx_ul);
                    }
                    print_per_symbol_done(
                        PrintType::kDecode, frame_id, symbol_idx_ul);
                    if (decode_stats_.last_symbol(frame_id)) {
                        stats->master_set_tsc(TsType::kDecodeDone, frame_id);
                        print_per_frame_done(PrintType::kDecode, frame_id);
                        if (!kEnableMac) {
                            // assert(cur_frame_id == frame_id);
                            // cur_frame_id++;
                            stats->update_stats_in_functions_uplink(frame_id);
                            if (stats->last_frame_id == cfg->frames_to_test - 1)
                                goto finish;
                        }
                    }
                }
            } break;

            case EventType::kRANUpdate: {
                RanConfig rc;
                rc.n_antennas = event.tags[0];
                rc.mod_order_bits = event.tags[1];
                rc.frame_id = event.tags[2];
                update_ran_config(rc);
            } break;

            case EventType::kPacketToMac: {
                size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
                size_t symbol_idx_ul = gen_tag_t(event.tags[0]).symbol_id;
                if (tomac_stats_.last_task(frame_id, symbol_idx_ul)) {
                    print_per_symbol_done(
                        PrintType::kPacketToMac, frame_id, symbol_idx_ul);
                    if (tomac_stats_.last_symbol(frame_id)) {
                        assert(cur_frame_id == frame_id);
                        cur_frame_id++;
                        // stats->master_set_tsc(TsType::kMacTXDone, frame_id);
                        print_per_frame_done(PrintType::kPacketToMac, frame_id);
                        stats->update_stats_in_functions_uplink(frame_id);
                        if (stats->last_frame_id == cfg->frames_to_test - 1)
                            goto finish;
                    }
                }

            } break;

            case EventType::kEncode: {
                for (size_t i = 0; i < event.num_tags; i++) {
                    size_t frame_id = gen_tag_t(event.tags[i]).frame_id;
                    size_t symbol_id = gen_tag_t(event.tags[i]).symbol_id;
                    size_t symbol_idx_dl
                        = cfg->get_dl_symbol_idx(frame_id, symbol_id);
                    if (encode_stats_.last_task(frame_id, symbol_idx_dl)) {
                        encode_stats_.cur_frame_for_symbol[symbol_idx_dl]
                            = frame_id;
                        // If precoder of the current frame exists
                        if (zf_stats_.coded_frame == frame_id) {
                            schedule_subcarriers(
                                EventType::kPrecode, frame_id, symbol_id);
                        }
                        print_per_symbol_done(
                            PrintType::kEncode, frame_id, symbol_idx_dl);
                        if (encode_stats_.last_symbol(frame_id)) {
                            stats->master_set_tsc(
                                TsType::kEncodeDone, frame_id);
                            print_per_frame_done(PrintType::kEncode, frame_id);
                        }
                    }
                }
            } break;

            case EventType::kPrecode: {
                /* Precoding is done, schedule ifft */
                size_t sc_id = gen_tag_t(event.tags[0]).sc_id;
                size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
                size_t symbol_id = gen_tag_t(event.tags[0]).symbol_id;
                size_t symbol_idx_dl
                    = cfg->get_dl_symbol_idx(frame_id, symbol_id);
                print_per_task_done(
                    PrintType::kPrecode, frame_id, symbol_idx_dl, sc_id);
                if (precode_stats_.last_task(frame_id, symbol_idx_dl)) {
                    schedule_antennas(EventType::kIFFT, frame_id, symbol_id);
                    print_per_symbol_done(
                        PrintType::kPrecode, frame_id, symbol_idx_dl);
                    if (precode_stats_.last_symbol(frame_id)) {
                        stats->master_set_tsc(TsType::kPrecodeDone, frame_id);
                        print_per_frame_done(PrintType::kPrecode, frame_id);
                    }
                }
            } break;

            case EventType::kIFFT: {
                for (size_t i = 0; i < event.num_tags; i++) {
                    /* IFFT is done, schedule data transmission */
                    size_t ant_id = gen_tag_t(event.tags[i]).ant_id;
                    size_t frame_id = gen_tag_t(event.tags[i]).frame_id;
                    size_t symbol_id = gen_tag_t(event.tags[i]).symbol_id;
                    size_t symbol_idx_dl
                        = cfg->get_dl_symbol_idx(frame_id, symbol_id);
                    try_enqueue_fallback(get_conq(EventType::kPacketTX),
                        tx_ptoks_ptr[ant_id % cfg->socket_thread_num],
                        Event_data(EventType::kPacketTX, event.tags[0]));
                    print_per_task_done(
                        PrintType::kIFFT, frame_id, symbol_idx_dl, ant_id);

                    if (ifft_stats_.last_task(frame_id, symbol_idx_dl)) {
                        print_per_symbol_done(
                            PrintType::kIFFT, frame_id, symbol_idx_dl);
                        if (ifft_stats_.last_symbol(frame_id)) {
                            stats->master_set_tsc(TsType::kIFFTDone, frame_id);
                            print_per_frame_done(PrintType::kIFFT, frame_id);
                            assert(frame_id == cur_frame_id);
                            cur_frame_id++;
                            stats->update_stats_in_functions_downlink(frame_id);
                            if (stats->last_frame_id == cfg->frames_to_test - 1)
                                goto finish;
                        }
                    }
                }
            } break;

            case EventType::kPacketTX: {
                /* Data is sent */
                size_t ant_id = gen_tag_t(event.tags[0]).ant_id;
                size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
                size_t symbol_id = gen_tag_t(event.tags[0]).symbol_id;
                size_t symbol_idx_dl
                    = cfg->get_dl_symbol_idx(frame_id, symbol_id);
                print_per_task_done(
                    PrintType::kPacketTX, frame_id, symbol_idx_dl, ant_id);
                if (tx_stats_.last_task(frame_id, symbol_idx_dl)) {
                    print_per_symbol_done(
                        PrintType::kPacketTX, frame_id, symbol_idx_dl);
                    /* If tx of the first symbol is done */
                    if (symbol_id == cfg->DLSymbols[0].front()) {
                        stats->master_set_tsc(
                            TsType::kTXProcessedFirst, frame_id);
                        print_per_frame_done(
                            PrintType::kPacketTXFirst, frame_id);
                    }
                    if (tx_stats_.last_symbol(frame_id)) {
                        stats->master_set_tsc(TsType::kTXDone, frame_id);
                        print_per_frame_done(PrintType::kPacketTX, frame_id);
                        if (stats->last_frame_id == cfg->frames_to_test - 1)
                            goto finish;
                    }

                    tx_count++;
                    if (tx_count == tx_stats_.max_symbol_count * 9000) {
                        tx_count = 0;
                        double diff = get_time_us() - tx_begin;
                        int samples_num_per_UE = cfg->OFDM_DATA_NUM
                            * tx_stats_.max_symbol_count * 1000;

                        printf("TX %d samples (per-client) to %zu clients "
                               "in %f secs, throughtput %f bps per-client "
                               "(16QAM), current tx queue length %zu\n",
                            samples_num_per_UE, cfg->UE_NUM, diff,
                            samples_num_per_UE * log2(16.0f) / diff,
                            get_conq(EventType::kPacketTX)->size_approx());
                        tx_begin = get_time_us();
                    }
                }
            } break;
            default:
                printf("Wrong event type in message queue!");
                exit(0);
            } /* End of switch */

            // We schedule FFT processing if the event handling above results in
            // either (a) sufficient packets received for the current frame,
            // or (b) the current frame being updated.
            std::queue<fft_req_tag_t>& cur_fftq
                = fft_queue_arr[cur_frame_id % kFrameWnd];
            if (cur_fftq.size() >= config_->fft_block_size) {
                size_t num_fft_blocks
                    = cur_fftq.size() / config_->fft_block_size;
                for (size_t i = 0; i < num_fft_blocks; i++) {
                    Event_data do_fft_task;
                    do_fft_task.num_tags = config_->fft_block_size;
                    do_fft_task.event_type = EventType::kFFT;

                    for (size_t j = 0; j < config_->fft_block_size; j++) {
                        do_fft_task.tags[j] = cur_fftq.front()._tag;
                        cur_fftq.pop();
                        if (fft_created_count++ == 0) {
                            stats->master_set_tsc(
                                TsType::kProcessingStarted, cur_frame_id);
                        } else if (fft_created_count
                            == rx_counters_.num_pkts_per_frame) {
                            fft_created_count = 0;
                            if (cfg->bigstation_mode)
                                cur_frame_id++;
                        }
                    }
                    try_enqueue_fallback(get_conq(EventType::kFFT),
                        get_ptok(EventType::kFFT), do_fft_task);
                }
            }
        } /* End of for */
    } /* End of while */

finish:

    printf("Agora: printing stats and saving to file\n");
    stats->print_summary();
    stats->save_to_file();
    if (flags.enable_save_decode_data_to_file) {
        save_decode_data_to_file(stats->last_frame_id);
    }
    if (flags.enable_save_tx_data_to_file)
        save_tx_data_to_file(stats->last_frame_id);

    // Calculate and print per-user BER
    if (!kEnableMac && kPrintPhyStats) {
        phy_stats->print_phy_stats();
    }
    this->stop();
}

void Agora::handle_event_fft(size_t tag)
{
    size_t frame_id = gen_tag_t(tag).frame_id;
    size_t symbol_id = gen_tag_t(tag).symbol_id;
    size_t frame_slot = frame_id % kFrameWnd;
    SymbolType sym_type = config_->get_symbol_type(frame_id, symbol_id);

    if (sym_type == SymbolType::kPilot) {
        if (fft_stats_.last_task(frame_id, symbol_id)) {
            print_per_symbol_done(PrintType::kFFTPilots, frame_id, symbol_id);
            if (!config_->downlink_mode
                || (config_->downlink_mode && !config_->recipCalEn)
                || (config_->downlink_mode && config_->recipCalEn
                       && rc_stats_.last_frame == frame_id)) {
                /* If CSI of all UEs is ready, schedule ZF/prediction */
                if (fft_stats_.last_symbol(frame_id)) {
                    stats->master_set_tsc(TsType::kFFTPilotsDone, frame_id);
                    print_per_frame_done(PrintType::kFFTPilots, frame_id);
                    if (kPrintPhyStats)
                        phy_stats->print_snr_stats(frame_id);
                    if (kEnableMac)
                        send_snr_report(
                            EventType::kSNRReport, frame_id, symbol_id);
                    schedule_subcarriers(EventType::kZF, frame_id, 0);
                }
            }
        }
    } else if (sym_type == SymbolType::kUL) {
        if (fft_stats_.last_task(frame_id, symbol_id)) {
            size_t symbol_idx_ul
                = config_->get_ul_symbol_idx(frame_id, symbol_id);
            fft_stats_.cur_frame_for_symbol[symbol_idx_ul] = frame_id;
            print_per_symbol_done(PrintType::kFFTData, frame_id, symbol_id);
            /* If precoder exist, schedule demodulation */
            if (zf_stats_.coded_frame == frame_id) {
                schedule_subcarriers(
                    EventType::kDemul, frame_id, symbol_idx_ul);
            }
        }
    } else if (sym_type == SymbolType::kCalDL
        or sym_type == SymbolType::kCalUL) {
        if (++fft_stats_.symbol_rc_count[frame_slot]
            == fft_stats_.max_symbol_rc_count) {
            print_per_frame_done(PrintType::kFFTCal, frame_id);
            // TODO: rc_stats_.max_task_count appears uninitalized
            stats->master_set_tsc(TsType::kRCDone, frame_id);
            fft_stats_.symbol_rc_count[frame_slot] = 0;
            rc_stats_.last_frame = frame_id;
        }
    }
}

void* Agora::worker(int tid)
{
    pin_to_core_with_offset(
        ThreadType::kWorker, base_worker_core_offset, tid, false /* quiet */);

    /* Initialize operators */
    auto computeFFT = new DoFFT(config_, tid, freq_ghz,
        *get_conq(EventType::kFFT), complete_task_queue_, worker_ptoks_ptr[tid],
        socket_buffer_, socket_buffer_status_, data_buffer_, csi_buffers_,
        calib_dl_buffer_, calib_ul_buffer_, phy_stats, stats);

    auto computeIFFT = new DoIFFT(config_, tid, freq_ghz,
        *get_conq(EventType::kIFFT), complete_task_queue_,
        worker_ptoks_ptr[tid], dl_ifft_buffer_, dl_socket_buffer_, stats);

    auto computeZF = new DoZF(config_, tid, freq_ghz, *get_conq(EventType::kZF),
        complete_task_queue_, worker_ptoks_ptr[tid], csi_buffers_,
        calib_dl_buffer_, calib_ul_buffer_, ul_zf_matrices_, dl_zf_matrices_,
        stats);

    auto computeDemul = new DoDemul(config_, tid, freq_ghz,
        *get_conq(EventType::kDemul), complete_task_queue_,
        worker_ptoks_ptr[tid], data_buffer_, ul_zf_matrices_,
        ue_spec_pilot_buffer_, equal_buffer_, demod_buffers_, phy_stats, stats);

    auto computePrecode
        = new DoPrecode(config_, tid, freq_ghz, *get_conq(EventType::kPrecode),
            complete_task_queue_, worker_ptoks_ptr[tid], dl_zf_matrices_,
            dl_ifft_buffer_, dl_encoded_buffer_, stats);

    auto computeEncoding = new DoEncode(config_, tid, freq_ghz,
        *get_conq(EventType::kEncode), complete_task_queue_,
        worker_ptoks_ptr[tid], config_->dl_bits, dl_encoded_buffer_, stats);

    auto computeDecoding
        = new DoDecode(config_, tid, freq_ghz, *get_conq(EventType::kDecode),
            complete_task_queue_, worker_ptoks_ptr[tid], demod_buffers_,
            decoded_buffer_, phy_stats, stats);

    auto computeDecodingLast = new DoDecode(config_, tid, freq_ghz,
        *get_conq(EventType::kDecodeLast), complete_decode_task_queue_,
        decode_ptoks_ptr[tid], demod_buffers_, decoded_buffer_, phy_stats,
        stats);

    std::vector<Doer*> computers_vec;
    if (config_->dl_data_symbol_num_perframe > 0)
        computers_vec = { computeZF, computeFFT, computeIFFT, computePrecode,
            computeEncoding };
    else
        computers_vec = { computeDecodingLast, computeZF, computeFFT,
            computeDecoding, computeDemul };

    while (true) {
        for (size_t i = 0; i < computers_vec.size(); i++) {
            if (computers_vec[i]->try_launch())
                break;
        }
    }
}

void* Agora::worker_fft(int tid)
{
    pin_to_core_with_offset(
        ThreadType::kWorkerFFT, base_worker_core_offset, tid);

    /* Initialize IFFT operator */
    auto computeFFT = new DoFFT(config_, tid, freq_ghz,
        *get_conq(EventType::kFFT), complete_task_queue_, worker_ptoks_ptr[tid],
        socket_buffer_, socket_buffer_status_, data_buffer_, csi_buffers_,
        calib_dl_buffer_, calib_ul_buffer_, phy_stats, stats);
    auto computeIFFT = new DoIFFT(config_, tid, freq_ghz,
        *get_conq(EventType::kIFFT), complete_task_queue_,
        worker_ptoks_ptr[tid], dl_ifft_buffer_, dl_socket_buffer_, stats);

    while (true) {
        if (computeFFT->try_launch()) {
        } else if (config_->dl_data_symbol_num_perframe > 0
            && computeIFFT->try_launch()) {
        }
    }
}

void* Agora::worker_zf(int tid)
{
    pin_to_core_with_offset(
        ThreadType::kWorkerZF, base_worker_core_offset, tid);

    /* Initialize ZF operator */
    auto computeZF = new DoZF(config_, tid, freq_ghz, *get_conq(EventType::kZF),
        complete_task_queue_, worker_ptoks_ptr[tid], csi_buffers_,
        calib_dl_buffer_, calib_ul_buffer_, ul_zf_matrices_, dl_zf_matrices_,
        stats);

    while (true) {
        computeZF->try_launch();
    }
}

void* Agora::worker_demul(int tid)
{
    pin_to_core_with_offset(
        ThreadType::kWorkerDemul, base_worker_core_offset, tid);

    auto computeDemul = new DoDemul(config_, tid, freq_ghz,
        *get_conq(EventType::kDemul), complete_task_queue_,
        worker_ptoks_ptr[tid], data_buffer_, ul_zf_matrices_,
        ue_spec_pilot_buffer_, equal_buffer_, demod_buffers_, phy_stats, stats);

    /* Initialize Precode operator */
    auto computePrecode
        = new DoPrecode(config_, tid, freq_ghz, *get_conq(EventType::kPrecode),
            complete_task_queue_, worker_ptoks_ptr[tid], dl_zf_matrices_,
            dl_ifft_buffer_, dl_encoded_buffer_, stats);

    while (true) {
        if (config_->dl_data_symbol_num_perframe > 0) {
            computePrecode->try_launch();
        } else {
            computeDemul->try_launch();
        }
    }
}

void* Agora::worker_decode(int tid)
{
    pin_to_core_with_offset(
        ThreadType::kWorkerDecode, base_worker_core_offset, tid);

    auto computeEncoding = new DoEncode(config_, tid, freq_ghz,
        *get_conq(EventType::kEncode), complete_task_queue_,
        worker_ptoks_ptr[tid], config_->dl_bits, dl_encoded_buffer_, stats);

    auto computeDecoding
        = new DoDecode(config_, tid, freq_ghz, *get_conq(EventType::kDecode),
            complete_task_queue_, worker_ptoks_ptr[tid], demod_buffers_,
            decoded_buffer_, phy_stats, stats);

    while (true) {
        if (config_->dl_data_symbol_num_perframe > 0) {
            computeEncoding->try_launch();
        } else {
            computeDecoding->try_launch();
        }
    }
}

void Agora::create_threads(void* (*worker)(void*), int tid_start, int tid_end)
{
    int ret;
    for (int i = tid_start; i < tid_end; i++) {
        auto context = new EventHandlerContext<Agora>;
        context->obj_ptr = this;
        context->id = i;
        ret = pthread_create(&task_threads[i], NULL, worker, context);
        if (ret != 0) {
            perror("task thread create failed");
            exit(0);
        }
    }
}

void Agora::update_ran_config(RanConfig rc)
{
    config_->update_mod_cfgs(rc.mod_order_bits);
}

void Agora::update_rx_counters(size_t frame_id, size_t symbol_id)
{
    const size_t frame_slot = frame_id % kFrameWnd;
    if (config_->isPilot(frame_id, symbol_id)) {
        rx_counters_.num_pilot_pkts[frame_slot]++;
        if (rx_counters_.num_pilot_pkts[frame_slot]
            == rx_counters_.num_pilot_pkts_per_frame) {
            rx_counters_.num_pilot_pkts[frame_slot] = 0;
            stats->master_set_tsc(TsType::kPilotAllRX, frame_id);
            print_per_frame_done(PrintType::kPacketRXPilots, frame_id);
        }
    } else if (config_->isCalDlPilot(frame_id, symbol_id)
        or config_->isCalUlPilot(frame_id, symbol_id)) {
        if (++rx_counters_.num_reciprocity_pkts[frame_slot]
            == rx_counters_.num_reciprocity_pkts_per_frame) {
            rx_counters_.num_reciprocity_pkts[frame_slot] = 0;
            stats->master_set_tsc(TsType::kRCAllRX, frame_id);
        }
    }
    // Receive first packet in a frame
    if (rx_counters_.num_pkts[frame_slot] == 0) {
        // schedule this frame's encoding
        for (size_t i = 0; i < config_->dl_data_symbol_num_perframe; i++)
            schedule_codeblocks(
                EventType::kEncode, frame_id, config_->DLSymbols[0][i]);
        stats->master_set_tsc(TsType::kPilotRX, frame_id);
        if (kDebugPrintPerFrameStart) {
            const size_t prev_frame_slot
                = (frame_slot + kFrameWnd - 1) % kFrameWnd;
            printf("Main [frame %zu + %.2f ms since last frame]: Received "
                   "first packet. Remaining packets in prev frame: %zu\n",
                frame_id,
                stats->master_get_delta_ms(
                    TsType::kPilotRX, frame_id, frame_id - 1),
                rx_counters_.num_pkts[prev_frame_slot]);
        }
    }

    rx_counters_.num_pkts[frame_slot]++;
    if (rx_counters_.num_pkts[frame_slot] == rx_counters_.num_pkts_per_frame) {
        stats->master_set_tsc(TsType::kRXDone, frame_id);
        print_per_frame_done(PrintType::kPacketRX, frame_id);
        rx_counters_.num_pkts[frame_slot] = 0;
    }
}

void Agora::print_per_frame_done(PrintType print_type, size_t frame_id)
{
    if (!kDebugPrintPerFrameDone)
        return;
    switch (print_type) {
    case (PrintType::kPacketRXPilots):
        printf("Main [frame %zu + %.2f ms]: Received all pilots\n", frame_id,
            stats->master_get_delta_ms(
                TsType::kPilotAllRX, TsType::kPilotRX, frame_id));
        break;
    case (PrintType::kPacketRX): {
        printf("Main [frame %zu + %.2f ms]: Received all packets\n", frame_id,
            stats->master_get_delta_ms(
                TsType::kRXDone, TsType::kPilotRX, frame_id));
    } break;
    case (PrintType::kFFTPilots):
        printf("Main [frame %zu + %.2f ms]: FFT-ed all pilots\n", frame_id,
            stats->master_get_delta_ms(
                TsType::kFFTPilotsDone, TsType::kPilotRX, frame_id));
        break;
    case (PrintType::kFFTCal):
        printf("Main [frame %zu + %.2f ms]: FFT-ed all calibration symbols\n",
            frame_id,
            stats->master_get_us_since(TsType::kRCAllRX, frame_id) / 1000.0);
        break;
    case (PrintType::kZF):
        printf("Main [frame %zu + %.2f ms]: Completed zero-forcing\n", frame_id,
            stats->master_get_delta_ms(
                TsType::kZFDone, TsType::kPilotRX, frame_id));
        break;
    case (PrintType::kDemul):
        printf("Main [frame %zu + %.2f ms]: Completed demodulation\n", frame_id,
            stats->master_get_delta_ms(
                TsType::kDemulDone, TsType::kPilotRX, frame_id));
        break;
    case (PrintType::kDecode):
        printf("Main [frame %zu + %.2f ms]: Completed LDPC decoding (%zu UL "
               "symbols)\n",
            frame_id,
            stats->master_get_delta_ms(
                TsType::kDecodeDone, TsType::kPilotRX, frame_id),
            config_->ul_data_symbol_num_perframe);
        break;
    case (PrintType::kEncode):
        printf("Main [frame %zu + %.2f ms]: Completed LDPC encoding\n",
            frame_id,
            stats->master_get_delta_ms(
                TsType::kEncodeDone, TsType::kPilotRX, frame_id));
        break;
    case (PrintType::kPrecode):
        printf("Main [frame %zu + %.2f ms]: Completed precoding\n", frame_id,
            stats->master_get_delta_ms(
                TsType::kPrecodeDone, TsType::kPilotRX, frame_id));
        break;
    case (PrintType::kIFFT):
        printf("Main [frame %zu + %.2f ms]: Completed IFFT\n", frame_id,
            stats->master_get_delta_ms(
                TsType::kIFFTDone, TsType::kPilotRX, frame_id));
        break;
    case (PrintType::kPacketTXFirst):
        printf("Main [frame %zu + %.2f ms]: Completed TX of first symbol\n",
            frame_id,
            stats->master_get_delta_ms(
                TsType::kTXProcessedFirst, TsType::kPilotRX, frame_id));
        break;
    case (PrintType::kPacketTX):
        printf("Main [frame %zu + %.2f ms]: Completed TX (%zu DL symbols)\n",
            frame_id,
            stats->master_get_delta_ms(
                TsType::kTXDone, TsType::kPilotRX, frame_id),
            config_->dl_data_symbol_num_perframe);
        break;
    case (PrintType::kPacketToMac):
        printf("Main [frame %zu + %.2f ms]: Completed MAC TX \n", frame_id,
            stats->master_get_ms_since(TsType::kPilotRX, frame_id));
        break;
    default:
        printf("Wrong task type in frame done print!");
    }
}

void Agora::print_per_symbol_done(
    PrintType print_type, size_t frame_id, size_t symbol_id)
{
    if (!kDebugPrintPerSymbolDone)
        return;
    switch (print_type) {
    case (PrintType::kFFTPilots):
        printf("Main [frame %zu symbol %zu + %.3f ms]: FFT-ed pilot symbol, "
               "%zu symbols done\n",
            frame_id, symbol_id,
            stats->master_get_ms_since(TsType::kPilotRX, frame_id),
            fft_stats_.get_symbol_count(frame_id) + 1);
        break;
    case (PrintType::kFFTData):
        printf("Main [frame %zu symbol %zu + %.3f ms]: FFT-ed data symbol, "
               "precoder status: %d\n",
            frame_id, symbol_id,
            stats->master_get_ms_since(TsType::kPilotRX, frame_id),
            zf_stats_.coded_frame == frame_id);
        break;
    case (PrintType::kDemul):
        printf("Main [frame %zu symbol %zu + %.3f ms]: Completed demodulation, "
               "%zu symbols done\n",
            frame_id, symbol_id,
            stats->master_get_ms_since(TsType::kPilotRX, frame_id),
            demul_stats_.get_symbol_count(frame_id) + 1);
        break;
    case (PrintType::kDecode):
        printf("Main [frame %zu symbol %zu + %.3f ms]: Completed decoding, "
               "%zu symbols done\n",
            frame_id, symbol_id,
            stats->master_get_ms_since(TsType::kPilotRX, frame_id),
            decode_stats_.get_symbol_count(frame_id) + 1);
        break;
    case (PrintType::kEncode):
        printf("Main [frame %zu symbol %zu + %.3f ms]: Completed encoding, "
               "%zu symbols done\n",
            frame_id, symbol_id,
            stats->master_get_ms_since(TsType::kPilotRX, frame_id),
            encode_stats_.get_symbol_count(frame_id) + 1);
        break;
    case (PrintType::kPrecode):
        printf("Main [frame %zu symbol %zu + %.3f ms]: Completed precoding, "
               "%zu symbols done\n",
            frame_id, symbol_id,
            stats->master_get_ms_since(TsType::kPilotRX, frame_id),
            precode_stats_.get_symbol_count(frame_id) + 1);
        break;
    case (PrintType::kIFFT):
        printf("Main [frame %zu symbol %zu + %.3f ms]: Completed IFFT, "
               "%zu symbols done\n",
            frame_id, symbol_id,
            stats->master_get_ms_since(TsType::kPilotRX, frame_id),
            ifft_stats_.get_symbol_count(frame_id) + 1);
        break;
    case (PrintType::kPacketTX):
        printf("Main [frame %zu symbol %zu + %.3f ms]: Completed TX, "
               "%zu symbols done\n",
            frame_id, symbol_id,
            stats->master_get_ms_since(TsType::kPilotRX, frame_id),
            tx_stats_.get_symbol_count(frame_id) + 1);
        break;
    case (PrintType::kPacketToMac):
        printf("Main [frame %zu symbol %zu + %.3f ms]: Completed MAC TX, "
               "%zu symbols done\n",
            frame_id, symbol_id,
            stats->master_get_ms_since(TsType::kPilotRX, frame_id),
            tomac_stats_.get_symbol_count(frame_id) + 1);
        break;
    default:
        printf("Wrong task type in symbol done print!");
    }
}

void Agora::print_per_task_done(PrintType print_type, size_t frame_id,
    size_t symbol_id, size_t ant_or_sc_id)
{
    if (!kDebugPrintPerTaskDone)
        return;
    switch (print_type) {
    case (PrintType::kZF):
        printf("Main thread: ZF done frame: %zu, subcarrier %zu\n", frame_id,
            ant_or_sc_id);
        break;
    case (PrintType::kRC):
        printf("Main thread: RC done frame: %zu, subcarrier %zu\n", frame_id,
            ant_or_sc_id);
        break;
    case (PrintType::kDemul):
        printf("Main thread: Demodulation done frame: %zu, symbol: %zu, sc: "
               "%zu, num blocks done: %zu\n",
            frame_id, symbol_id, ant_or_sc_id,
            demul_stats_.get_task_count(frame_id, symbol_id));
        break;
    case (PrintType::kDecode):
        printf("Main thread: Decoding done frame: %zu, symbol: %zu, sc: %zu, "
               "num blocks done: %zu\n",
            frame_id, symbol_id, ant_or_sc_id,
            decode_stats_.get_task_count(frame_id, symbol_id));
        break;
    case (PrintType::kPrecode):
        printf("Main thread: Precoding done frame: %zu, symbol: %zu, "
               "subcarrier: %zu, total SCs: %zu\n",
            frame_id, symbol_id, ant_or_sc_id,
            precode_stats_.get_task_count(frame_id, symbol_id));
        break;
    case (PrintType::kIFFT):
        printf("Main thread: IFFT done frame: %zu, symbol: %zu, antenna: %zu, "
               "total ants: %zu\n",
            frame_id, symbol_id, ant_or_sc_id,
            ifft_stats_.get_task_count(frame_id, symbol_id));
        break;
    case (PrintType::kPacketTX):
        printf("Main thread: TX done frame: %zu, symbol: %zu, antenna: %zu, "
               "total packets: %zu\n",
            frame_id, symbol_id, ant_or_sc_id,
            tx_stats_.get_task_count(frame_id, symbol_id));
        break;
    default:
        printf("Wrong task type in task done print!");
    }
}

void Agora::initialize_queues()
{
    using mt_queue_t = moodycamel::ConcurrentQueue<Event_data>;

    int data_symbol_num_perframe = config_->data_symbol_num_perframe;
    message_queue_ = mt_queue_t(512 * data_symbol_num_perframe);
    complete_task_queue_ = mt_queue_t(512 * data_symbol_num_perframe * 4);
    complete_decode_task_queue_ = mt_queue_t(2048);

    // Create concurrent queues for each Doer
    for (sched_info_t& s : sched_info_arr) {
        s.concurrent_q = mt_queue_t(512 * data_symbol_num_perframe * 4);
        s.ptok = new moodycamel::ProducerToken(s.concurrent_q);
    }

    for (size_t i = 0; i < config_->socket_thread_num; i++) {
        rx_ptoks_ptr[i] = new moodycamel::ProducerToken(message_queue_);
        tx_ptoks_ptr[i]
            = new moodycamel::ProducerToken(*get_conq(EventType::kPacketTX));
    }

    for (size_t i = 0; i < config_->worker_thread_num; i++) {
        worker_ptoks_ptr[i]
            = new moodycamel::ProducerToken(complete_task_queue_);
        decode_ptoks_ptr[i]
            = new moodycamel::ProducerToken(complete_decode_task_queue_);
    }
}

void Agora::initialize_uplink_buffers()
{
    auto& cfg = config_;
    const size_t task_buffer_symbol_num_ul
        = cfg->ul_data_symbol_num_perframe * kFrameWnd;

    alloc_buffer_1d(&task_threads, cfg->worker_thread_num, 64, 0);

    socket_buffer_status_size_
        = cfg->BS_ANT_NUM * kFrameWnd * cfg->symbol_num_perframe;
    socket_buffer_size_ = cfg->packet_length * socket_buffer_status_size_;

    socket_buffer_.malloc(
        cfg->socket_thread_num /* RX */, socket_buffer_size_, 64);
    socket_buffer_status_.calloc(
        cfg->socket_thread_num /* RX */, socket_buffer_status_size_, 64);

    data_buffer_.malloc(
        task_buffer_symbol_num_ul, cfg->OFDM_DATA_NUM * cfg->BS_ANT_NUM, 64);

    equal_buffer_.malloc(
        task_buffer_symbol_num_ul, cfg->OFDM_DATA_NUM * cfg->UE_NUM, 64);
    ue_spec_pilot_buffer_.calloc(
        kFrameWnd, cfg->UL_PILOT_SYMS * cfg->UE_NUM, 64);

    rx_counters_.num_pkts_per_frame = cfg->BS_ANT_NUM
        * (cfg->pilot_symbol_num_perframe + cfg->ul_data_symbol_num_perframe
              + cfg->recip_pilot_symbol_num_perframe);
    rx_counters_.num_pilot_pkts_per_frame
        = cfg->BS_ANT_NUM * cfg->pilot_symbol_num_perframe;
    rx_counters_.num_reciprocity_pkts_per_frame = cfg->BS_ANT_NUM;

    fft_created_count = 0;
    fft_stats_.init(cfg->BS_ANT_NUM, cfg->pilot_symbol_num_perframe,
        cfg->symbol_num_perframe);
    fft_stats_.max_symbol_data_count = cfg->ul_data_symbol_num_perframe;
    fft_stats_.symbol_rc_count.fill(0);
    fft_stats_.max_symbol_rc_count = cfg->BS_ANT_NUM;
    fft_stats_.cur_frame_for_symbol
        = std::vector<size_t>(cfg->ul_data_symbol_num_perframe, SIZE_MAX);

    zf_stats_.init(config_->zf_events_per_symbol);

    demul_stats_.init(config_->demul_events_per_symbol,
        cfg->ul_data_symbol_num_perframe, cfg->data_symbol_num_perframe);

    decode_stats_.init(config_->LDPC_config.nblocksInSymbol * cfg->UE_NUM,
        cfg->ul_data_symbol_num_perframe, cfg->data_symbol_num_perframe);

    tomac_stats_.init(cfg->UE_NUM, cfg->ul_data_symbol_num_perframe,
        cfg->data_symbol_num_perframe);
}

void Agora::initialize_downlink_buffers()
{
    auto& cfg = config_;
    const size_t task_buffer_symbol_num
        = cfg->dl_data_symbol_num_perframe * kFrameWnd;

    size_t dl_socket_buffer_status_size
        = cfg->BS_ANT_NUM * kFrameWnd * cfg->dl_data_symbol_num_perframe;
    size_t dl_socket_buffer_size
        = cfg->dl_packet_length * dl_socket_buffer_status_size;
    alloc_buffer_1d(&dl_socket_buffer_, dl_socket_buffer_size, 64, 0);
    alloc_buffer_1d(
        &dl_socket_buffer_status_, dl_socket_buffer_status_size, 64, 1);

    dl_bits_buffer_.calloc(
        task_buffer_symbol_num, cfg->OFDM_DATA_NUM * cfg->UE_NUM, 64);
    size_t dl_bits_buffer_status_size
        = task_buffer_symbol_num * cfg->LDPC_config.nblocksInSymbol;
    dl_bits_buffer_status_.calloc(cfg->UE_NUM, dl_bits_buffer_status_size, 64);

    dl_ifft_buffer_.calloc(
        cfg->BS_ANT_NUM * task_buffer_symbol_num, cfg->OFDM_CA_NUM, 64);
    calib_dl_buffer_.calloc(
        kFrameWnd, cfg->BF_ANT_NUM * cfg->OFDM_DATA_NUM, 64);
    calib_ul_buffer_.calloc(
        kFrameWnd, cfg->BF_ANT_NUM * cfg->OFDM_DATA_NUM, 64);
    // initialize the content of the last window to 1
    for (size_t i = 0; i < cfg->OFDM_DATA_NUM * cfg->BF_ANT_NUM; i++) {
        calib_dl_buffer_[kFrameWnd - 1][i] = { 1, 0 };
        calib_ul_buffer_[kFrameWnd - 1][i] = { 1, 0 };
    }
    dl_encoded_buffer_.calloc(task_buffer_symbol_num,
        roundup<64>(cfg->OFDM_DATA_NUM) * cfg->UE_NUM, 64);

    frommac_stats_.init(config_->UE_NUM, cfg->dl_data_symbol_num_perframe,
        cfg->data_symbol_num_perframe);
    encode_stats_.init(config_->LDPC_config.nblocksInSymbol * cfg->UE_NUM,
        cfg->dl_data_symbol_num_perframe, cfg->data_symbol_num_perframe);
    encode_stats_.cur_frame_for_symbol
        = std::vector<size_t>(cfg->dl_data_symbol_num_perframe, SIZE_MAX);
    precode_stats_.init(config_->demul_events_per_symbol,
        cfg->dl_data_symbol_num_perframe, cfg->data_symbol_num_perframe);
    ifft_stats_.init(cfg->BS_ANT_NUM, cfg->dl_data_symbol_num_perframe,
        cfg->data_symbol_num_perframe);
    tx_stats_.init(cfg->BS_ANT_NUM, cfg->dl_data_symbol_num_perframe,
        cfg->data_symbol_num_perframe);
}

void Agora::free_uplink_buffers()
{
    socket_buffer_.free();
    socket_buffer_status_.free();
    data_buffer_.free();
    equal_buffer_.free();

    fft_stats_.fini();
    demul_stats_.fini();
    decode_stats_.fini();
}

void Agora::free_downlink_buffers()
{
    free_buffer_1d(&dl_socket_buffer_);
    free_buffer_1d(&dl_socket_buffer_status_);

    dl_ifft_buffer_.free();
    calib_dl_buffer_.free();
    calib_ul_buffer_.free();
    dl_encoded_buffer_.free();

    encode_stats_.fini();
    precode_stats_.fini();
    ifft_stats_.fini();
    tx_stats_.fini();
}

void Agora::save_decode_data_to_file(int frame_id)
{
    auto& cfg = config_;
    const size_t num_decoded_bytes
        = cfg->num_bytes_per_cb * cfg->LDPC_config.nblocksInSymbol;

    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string filename = cur_directory + "/data/decode_data.bin";
    printf("Saving decode data to %s\n", filename.c_str());
    FILE* fp = fopen(filename.c_str(), "wb");

    for (size_t i = 0; i < cfg->ul_data_symbol_num_perframe; i++) {
        for (size_t j = 0; j < cfg->UE_NUM; j++) {
            uint8_t* ptr = decoded_buffer_[frame_id % kFrameWnd][i][j];
            fwrite(ptr, num_decoded_bytes, sizeof(uint8_t), fp);
        }
    }
    fclose(fp);
}

void Agora::save_tx_data_to_file(UNUSED int frame_id)
{
    auto& cfg = config_;

    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string filename = cur_directory + "/data/tx_data.bin";
    printf("Saving TX data to %s\n", filename.c_str());
    FILE* fp = fopen(filename.c_str(), "wb");

    for (size_t i = 0; i < cfg->dl_data_symbol_num_perframe; i++) {
        size_t total_data_symbol_id
            = cfg->get_total_data_symbol_idx_dl(frame_id, i);

        for (size_t ant_id = 0; ant_id < cfg->BS_ANT_NUM; ant_id++) {
            size_t offset = total_data_symbol_id * cfg->BS_ANT_NUM + ant_id;
            struct Packet* pkt = (struct Packet*)(&dl_socket_buffer_[offset
                * cfg->dl_packet_length]);
            short* socket_ptr = pkt->data;
            fwrite(socket_ptr, cfg->sampsPerSymbol * 2, sizeof(short), fp);
        }
    }
    fclose(fp);
}

void Agora::getEqualData(float** ptr, int* size)
{
    auto& cfg = config_;
    auto offset = cfg->get_total_data_symbol_idx_ul(
        max_equaled_frame, cfg->UL_PILOT_SYMS);
    *ptr = (float*)&equal_buffer_[offset][0];
    *size = cfg->UE_NUM * cfg->OFDM_DATA_NUM * 2;
}

extern "C" {
EXPORT Agora* Agora_new(Config* cfg)
{
    // printf("Size of Agora: %d\n",sizeof(Agora *));
    auto* agora = new Agora(cfg);

    return agora;
}
EXPORT void Agora_start(Agora* agora) { agora->start(); }
EXPORT void Agora_stop(/*Agora *agora*/)
{
    SignalHandler::setExitSignal(true); /*agora->stop();*/
}
EXPORT void Agora_destroy(Agora* agora) { delete agora; }
EXPORT void Agora_getEqualData(Agora* agora, float** ptr, int* size)
{
    return agora->getEqualData(ptr, size);
}
}
