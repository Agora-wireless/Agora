#include "agora.hpp"

static const size_t kDefaultMessageQueueSize = 512;
static const size_t kDefaultWorkerQueueSize  = 256;

Agora::Agora(Config* cfg)
    : base_worker_core_offset(cfg->core_offset() + 1 + cfg->socket_thread_num())
    , csi_buffers_(kFrameWnd, cfg->ue_num(), cfg->bs_ant_num() * cfg->ofdm_data_num())
    , ul_zf_matrices_(
          kFrameWnd, cfg->ofdm_data_num(), cfg->bs_ant_num() * cfg->ue_num())
    , demod_buffers_(kFrameWnd, cfg->frame().NumTotalSyms(), cfg->ue_num(),
          kMaxModType * cfg->ofdm_data_num())
    , decoded_buffer_(kFrameWnd, cfg->frame().NumTotalSyms(), cfg->ue_num(),
          cfg->ldpc_config().num_blocks_in_symbol() * roundup<64>(cfg->num_bytes_per_cb()))
    , dl_zf_matrices_(
          kFrameWnd, cfg->ofdm_data_num(), cfg->ue_num() * cfg->bs_ant_num())
{
    std::string directory = TOSTRING(PROJECT_DIRECTORY);
    std::printf("Agora: project directory [%s], RDTSC frequency = %.2f GHz\n",
        directory.c_str(), cfg->freq_ghz());

    this->config_ = cfg;

    pin_to_core_with_offset(
        ThreadType::kMaster, cfg->core_offset(), 0, false /* quiet */);
    initialize_queues();
    initialize_uplink_buffers();
    initialize_downlink_buffers();

    stats_.reset( new Stats(cfg) );
    phy_stats_.reset( new PhyStats(cfg) );

    /* Initialize TXRX threads */
    packet_tx_rx_.reset(
        new PacketTXRX(cfg, cfg->core_offset() + 1, &message_queue_,
            get_conq(EventType::kPacketTX, 0), rx_ptoks_ptr, tx_ptoks_ptr));

    if (kEnableMac == true) {
        const size_t mac_cpu_core = cfg->core_offset() + cfg->socket_thread_num()
            + cfg->worker_thread_num() + 1;
        mac_thread_ = new MacThread(MacThread::Mode::kServer, cfg, mac_cpu_core,
            decoded_buffer_, nullptr /* ul bits */,
            nullptr /* ul bits status */, &dl_bits_buffer_,
            &dl_bits_buffer_status_, &mac_request_queue_, &mac_response_queue_);

        mac_std_thread_ = std::thread(&MacThread::run_event_loop, mac_thread_);
    }

    /* Create worker threads */
    create_threads();

    std::printf(
        "Master thread core %zu, TX/RX thread cores %zu--%zu, worker thread "
        "cores %zu--%zu\n",
        cfg->core_offset(), cfg->core_offset() + 1,
        cfg->core_offset() + 1 + cfg->socket_thread_num() - 1,
        base_worker_core_offset,
        base_worker_core_offset + cfg->worker_thread_num() - 1);
}

Agora::~Agora(void)
{
    //std::printf( "Agora: destructing\n" );
    free_uplink_buffers();
    /* Downlink */
    free_downlink_buffers();
    //std::printf( "Buffers freed\n" );
    if (kEnableMac == true) {
        //std::printf( "Joining mac thread\n" );
        mac_std_thread_.join();
        delete mac_thread_;
    }
    int i = 0;
    for ( auto&& worker_thread : workers_ ) {
        std::printf( "Joining worker thread: %d\n", i++);
        worker_thread.join();
    }

    stats_.reset();
    phy_stats_.reset();
    free_queues();
}

void Agora::stop()
{
    std::cout << "Agora: stopping threads" << std::endl;
    config_->running( false );
    usleep(1000);
    packet_tx_rx_.reset();
}

void Agora::send_snr_report(
    EventType event_type, size_t frame_id, size_t symbol_id)
{
    assert(event_type == EventType::kSNRReport);
    auto base_tag = gen_tag_t::frm_sym_ue(frame_id, symbol_id, 0);
    for (size_t i = 0; i < config_->ue_num(); i++) {
        Event_data snr_report(EventType::kSNRReport, base_tag._tag);
        snr_report.num_tags = 2;
        float snr = this->phy_stats_->get_evm_snr(frame_id, i);
        std::memcpy(&snr_report.tags[1], &snr, sizeof(float));
        try_enqueue_fallback(&mac_request_queue_, snr_report);
        base_tag.ue_id++;
    }
}

void Agora::schedule_antennas(
    EventType event_type, size_t frame_id, size_t symbol_id)
{
    assert(event_type == EventType::kFFT or event_type == EventType::kIFFT);
    auto base_tag = gen_tag_t::frm_sym_ant(frame_id, symbol_id, 0);

    size_t num_blocks = config_->bs_ant_num() / config_->fft_block_size();
    size_t num_remainder = config_->bs_ant_num() % config_->fft_block_size();
    if (num_remainder > 0)
        num_blocks++;
    Event_data event;
    event.num_tags = config_->fft_block_size();
    event.event_type = event_type;
    size_t qid = frame_id & 0x1;
    for (size_t i = 0; i < num_blocks; i++) {
        if ((i == num_blocks - 1) && num_remainder > 0)
            event.num_tags = num_remainder;
        for (size_t j = 0; j < event.num_tags; j++) {
            event.tags[j] = base_tag._tag;
            base_tag.ant_id++;
        }
        try_enqueue_fallback(
            get_conq(event_type, qid), get_ptok(event_type, qid), event);
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
        num_events = config_->demul_events_per_symbol();
        block_size = config_->demul_block_size();
        break;
    case EventType::kZF:
        num_events = config_->zf_events_per_symbol();
        block_size = config_->zf_block_size();
        break;
    default:
        assert(false);
    }

    size_t qid = frame_id & 0x1;
    if (event_type == EventType::kZF) {
        Event_data event;
        event.event_type = event_type;
        event.num_tags = config_->zf_batch_size();
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
                get_conq(event_type, qid), get_ptok(event_type, qid), event);
        }
    } else {
        for (size_t i = 0; i < num_events; i++) {
            try_enqueue_fallback(get_conq(event_type, qid),
                get_ptok(event_type, qid),
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
    //      i < config_->ue_num() * config_->ldpc_config().num_blocks_in_symbol(); i++) {
    //     try_enqueue_fallback(get_conq(event_type), get_ptok(event_type),
    //         Event_data(event_type, base_tag._tag));
    //     base_tag.cb_id++;
    // }
    size_t num_tasks = config_->ue_num() * config_->ldpc_config().num_blocks_in_symbol();
    size_t num_blocks = num_tasks / config_->encode_block_size();
    size_t num_remainder = num_tasks % config_->encode_block_size();
    if (num_remainder > 0)
        num_blocks++;
    Event_data event;
    event.num_tags = config_->encode_block_size();
    event.event_type = event_type;
    size_t qid = frame_id & 0x1;
    for (size_t i = 0; i < num_blocks; i++) {
        if ((i == num_blocks - 1) && num_remainder > 0)
            event.num_tags = num_remainder;
        for (size_t j = 0; j < event.num_tags; j++) {
            event.tags[j] = base_tag._tag;
            base_tag.cb_id++;
        }
        try_enqueue_fallback(
            get_conq(event_type, qid), get_ptok(event_type, qid), event);
    }
}

void Agora::schedule_users(
    EventType event_type, size_t frame_id, size_t symbol_id)
{
    assert(event_type == EventType::kPacketToMac);
    auto base_tag = gen_tag_t::frm_sym_ue(frame_id, symbol_id, 0);

    for (size_t i = 0; i < config_->ue_num(); i++) {
        try_enqueue_fallback(&mac_request_queue_,
            Event_data(EventType::kPacketToMac, base_tag._tag));
        base_tag.ue_id++;
    }
}

void Agora::start()
{
    const auto& cfg = this->config_;

    // Start packet I/O
    if (packet_tx_rx_->startTXRX(socket_buffer_, socket_buffer_status_,
            socket_buffer_status_size_, this->stats_->frame_start, dl_socket_buffer_,
            calib_dl_buffer_, calib_ul_buffer_) == false) {
        this->stop();
        return;
    }

    pin_to_core_with_offset(
        ThreadType::kMaster, cfg->core_offset(), 0, false /* quiet */);

    // Counters for printing summary
    size_t tx_count = 0;
    double tx_begin = get_time_us();

    bool is_turn_to_dequeue_from_io = true;
    const size_t max_events_needed = std::max(
        kDequeueBulkSizeTXRX * (cfg->socket_thread_num() + 1 /* MAC */),
        kDequeueBulkSizeWorker * cfg->worker_thread_num());
    Event_data events_list[max_events_needed];

    while ((config_->running() == true) && (SignalHandler::gotExitSignal() == false)) {
        // Get a batch of events
        size_t num_events = 0;
        if (is_turn_to_dequeue_from_io) {
            for (size_t i = 0; i < cfg->socket_thread_num(); i++) {
                num_events += message_queue_.try_dequeue_bulk_from_producer(
                    *(rx_ptoks_ptr[i]), events_list + num_events,
                    kDequeueBulkSizeTXRX);
            }
            num_events += mac_response_queue_.try_dequeue_bulk(
                events_list + num_events, kDequeueBulkSizeTXRX);
        } else {
            num_events += complete_task_queue_[cur_proc_frame_id & 0x1]
                              .try_dequeue_bulk(
                                  events_list + num_events, max_events_needed);
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
                    + (sock_buf_offset * cfg->packet_length()));

                if (pkt->frame_id >= cur_sche_frame_id + kFrameWnd) {
                    std::printf(
                        "Error: Received packet for future frame %u beyond "
                        "frame window (= %zu + %zu). This can happen if "
                        "Agora is running slowly, e.g., in debug mode\n",
                        pkt->frame_id, cur_sche_frame_id, kFrameWnd);
                    cfg->running( false );
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
                for (size_t tag_id = 0; (tag_id < event.num_tags); tag_id++) {
                    size_t frame_id = gen_tag_t(event.tags[tag_id]).frame_id;
                    print_per_task_done(PrintType::kZF, frame_id, 0, zf_counters_.GetTaskCount(frame_id));
                    
                    bool last_zf_task = zf_counters_.CompleteTask(frame_id);
                    if (last_zf_task == true) {
                        this->stats_->master_set_tsc(TsType::kZFDone, frame_id);
                        zf_last_frame = frame_id;
                        print_per_frame_done(PrintType::kZF, frame_id);

                        // If all the data in a frame has arrived when ZF is done
                        std::printf("kZF last task, frame id %zu\n", frame_id);

                        for (size_t i = 0; i < cfg->frame().NumULSyms(); i++) {
                            if (this->fft_cur_frame_for_symbol_.at(i) == frame_id) {
                                //std::printf("---kZF kDemul %zu %zu\n\n\n\n", frame_id, i);
                                schedule_subcarriers(
                                    EventType::kDemul, frame_id, cfg->frame().GetULSymbol(i)); //*************************************
                            }
                        }
                        // Schedule precoding for downlink symbols
                        for (size_t i = 0; i < cfg->frame().NumDLSyms(); i++) {
                            if (this->encode_cur_frame_for_symbol_.at(i) == frame_id) {
                                //std::printf("---kZF kPrecode %zu %zu %zu\n", frame_id, i, cfg->frame().GetDLSymbol(i));
                                schedule_subcarriers(EventType::kPrecode,
                                    frame_id, cfg->frame().GetDLSymbol(i));
                            }
                        }
                    } // end if (zf_counters_.last_task(frame_id) == true)
                }
            } break;

            case EventType::kDemul: {
                size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
                size_t symbol_idx_ul = gen_tag_t(event.tags[0]).symbol_id;
                size_t base_sc_id = gen_tag_t(event.tags[0]).sc_id;
                //std::printf("---Schedule kDemul %zu %zu %zu\n", frame_id, symbol_idx_ul, base_sc_id);

                print_per_task_done(PrintType::kDemul, frame_id, symbol_idx_ul, base_sc_id);
                bool last_demul_task = this->demul_counters_.CompleteTask(frame_id, symbol_idx_ul);

                if (last_demul_task == true) {
                    schedule_codeblocks(
                        EventType::kDecode, frame_id, symbol_idx_ul);
                    print_per_symbol_done(
                        PrintType::kDemul, frame_id, symbol_idx_ul);
                    bool last_demul_symbol = this->demul_counters_.CompleteSymbol(frame_id);
                    if (last_demul_symbol == true) {
                        max_equaled_frame = frame_id;
                        if (cfg->bigstation_mode() == false) {
                            assert(cur_sche_frame_id == frame_id);
                            cur_sche_frame_id++;
                        } else {
                            schedule_codeblocks(
                                EventType::kDecode, frame_id, symbol_idx_ul);
                        }
                        this->stats_->master_set_tsc(TsType::kDemulDone, frame_id);
                        print_per_frame_done(PrintType::kDemul, frame_id);
                    }
                }
            } break;

            case EventType::kDecode: {
                size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
                size_t symbol_idx_ul = gen_tag_t(event.tags[0]).symbol_id;

                //std::printf("---Schedule kDecode %zu %zu\n", frame_id, symbol_idx_ul);
                bool last_decode_task = this->decode_counters_.CompleteTask(frame_id, symbol_idx_ul);
                if (last_decode_task == true) {
                    if (kEnableMac == true) {
                        schedule_users(
                            EventType::kPacketToMac, frame_id, symbol_idx_ul);
                    }
                    print_per_symbol_done(
                        PrintType::kDecode, frame_id, symbol_idx_ul);
                    bool last_decode_symbol = this->decode_counters_.CompleteSymbol(frame_id);
                    if (last_decode_symbol == true) {
                        this->stats_->master_set_tsc(TsType::kDecodeDone, frame_id);
                        print_per_frame_done(PrintType::kDecode, frame_id);
                        if (kEnableMac == false) {
                            assert(cur_proc_frame_id == frame_id);
                            cur_proc_frame_id++;

                            this->stats_->update_stats_in_functions_uplink(frame_id);
                            bool work_finished = this->CheckWorkComplete(frame_id);
                            if (work_finished == true) {
                                std::printf("---kDecode finished---\n");
                                goto finish;
                            }
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

                bool last_tomac_task = this->tomac_counters_.CompleteTask(frame_id, symbol_idx_ul);
                if (last_tomac_task == true) {
                    print_per_symbol_done(
                        PrintType::kPacketToMac, frame_id, symbol_idx_ul);

                    bool last_tomac_symbol = this->tomac_counters_.CompleteSymbol(frame_id);
                    if (last_tomac_symbol == true) {
                        assert(cur_proc_frame_id == frame_id);
                        cur_proc_frame_id++;
                        // this->stats_->master_set_tsc(TsType::kMacTXDone, frame_id);
                        print_per_frame_done(PrintType::kPacketToMac, frame_id);
                        this->stats_->update_stats_in_functions_uplink(frame_id);
                        bool work_finished = this->CheckWorkComplete(frame_id);
                        if (work_finished == true) {
                            std::printf("---kPacketToMac finished---\n");
                            goto finish;
                        }
                    }
                }

            } break;

            case EventType::kEncode: {
                for (size_t i = 0; i < event.num_tags; i++) {
                    size_t frame_id = gen_tag_t(event.tags[i]).frame_id;
                    size_t symbol_id = gen_tag_t(event.tags[i]).symbol_id;
                    size_t symbol_idx_dl
                        = cfg->GetDLSymbolIdx(frame_id, symbol_id);
                    
                    bool last_encode_task = encode_counters_.CompleteTask(frame_id, symbol_idx_dl);
                    if (last_encode_task == true) {
                        //std::printf("encode_cur_frame_for_symbol %zu %zu %zu\n", symbol_idx_dl, symbol_id, frame_id);
                        this->encode_cur_frame_for_symbol_.at(symbol_idx_dl) = frame_id;
                        // If precoder of the current frame exists
                        if (zf_last_frame == frame_id) {
                            schedule_subcarriers(
                                EventType::kPrecode, frame_id, symbol_id);
                        }
                        print_per_symbol_done(
                            PrintType::kEncode, frame_id, symbol_idx_dl);
                        
                        bool last_encode_symbol = encode_counters_.CompleteSymbol(frame_id);
                        if (last_encode_symbol == true) {
                            this->stats_->master_set_tsc(
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
                    = cfg->GetDLSymbolIdx(frame_id, symbol_id);
                print_per_task_done(
                    PrintType::kPrecode, frame_id, symbol_idx_dl, sc_id);
                bool last_precode_task = this->precode_counters_.CompleteTask(frame_id, symbol_idx_dl);

                //std::printf("Precode %d %zu %zu\n", last_precode_task, this->precode_counters_.GetTaskCount(frame_id, symbol_idx_dl), this->precode_counters_.max_task_count());
                if (last_precode_task == true) {
                    schedule_antennas(EventType::kIFFT, frame_id, symbol_id);
                    print_per_symbol_done(
                        PrintType::kPrecode, frame_id, symbol_idx_dl);

                    bool last_precode_symbol = this->precode_counters_.CompleteSymbol(frame_id);
                    if (last_precode_symbol == true) {
                        this->stats_->master_set_tsc(TsType::kPrecodeDone, frame_id);
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
                        = cfg->GetDLSymbolIdx(frame_id, symbol_id);
                    try_enqueue_fallback(get_conq(EventType::kPacketTX, 0),
                        tx_ptoks_ptr[ant_id % cfg->socket_thread_num()],
                        Event_data(EventType::kPacketTX, event.tags[0]));
                    print_per_task_done(
                        PrintType::kIFFT, frame_id, symbol_idx_dl, ant_id);

                    bool last_ifft_task = this->ifft_counters_.CompleteTask(frame_id, symbol_idx_dl);
                    if (last_ifft_task == true) {
                        print_per_symbol_done(
                            PrintType::kIFFT, frame_id, symbol_idx_dl);

                        bool last_ifft_symbol = this->ifft_counters_.CompleteSymbol(frame_id);
                        if (last_ifft_symbol == true) {
                            this->stats_->master_set_tsc(TsType::kIFFTDone, frame_id);
                            print_per_frame_done(PrintType::kIFFT, frame_id);
                            assert(frame_id == cur_proc_frame_id);
                            cur_proc_frame_id++;
                            cur_sche_frame_id++;
                            this->stats_->update_stats_in_functions_downlink(frame_id);

                            bool work_finished = this->CheckWorkComplete(frame_id);
                            if (work_finished == true) {
                                std::printf("---kIFFT finished---\n");
                                goto finish;
                            }
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
                    = cfg->GetDLSymbolIdx(frame_id, symbol_id);
                print_per_task_done(
                    PrintType::kPacketTX, frame_id, symbol_idx_dl, ant_id);

                bool last_tx_task = this->tx_counters_.CompleteTask(frame_id, symbol_idx_dl);
                if (last_tx_task == true) {
                    print_per_symbol_done(
                        PrintType::kPacketTX, frame_id, symbol_idx_dl);
                    /* If tx of the first symbol is done */
                    if (symbol_id == cfg->frame().GetDLSymbol(0)) {
                        this->stats_->master_set_tsc(
                            TsType::kTXProcessedFirst, frame_id);
                        print_per_frame_done(
                            PrintType::kPacketTXFirst, frame_id);
                    }

                    bool last_tx_symbol = this->tx_counters_.CompleteSymbol(frame_id);
                    if (last_tx_symbol == true) {
                        this->stats_->master_set_tsc(TsType::kTXDone, frame_id);
                        print_per_frame_done(PrintType::kPacketTX, frame_id);

                        bool work_finished = this->CheckWorkComplete(frame_id);
                        if (work_finished == true) {
                            std::printf("---nkPacketTX finished---\n");
                            goto finish;
                        }
                    }

                    tx_count++;
                    if (tx_count == tx_counters_.max_symbol_count() * 9000) {
                        tx_count = 0;
                        double diff = get_time_us() - tx_begin;
                        int samples_num_per_UE = cfg->ofdm_data_num()
                            * tx_counters_.max_symbol_count() * 1000;

                        std::printf("TX %d samples (per-client) to %zu clients "
                                    "in %f secs, throughtput %f bps per-client "
                                    "(16QAM), current tx queue length %zu\n",
                            samples_num_per_UE, cfg->ue_num(), diff,
                            samples_num_per_UE * log2(16.0f) / diff,
                            get_conq(EventType::kPacketTX, 0)->size_approx());
                        tx_begin = get_time_us();
                    }
                }
            } break;
            default:
                std::printf("Wrong event type in message queue!");
                std::exit(0);
            } /* End of switch */

            // We schedule FFT processing if the event handling above results in
            // either (a) sufficient packets received for the current frame,
            // or (b) the current frame being updated.
            std::queue<fft_req_tag_t>& cur_fftq
                = fft_queue_arr[cur_sche_frame_id % kFrameWnd];
            size_t qid = cur_sche_frame_id & 0x1;
            if (cur_fftq.size() >= config_->fft_block_size()) {
                size_t num_fft_blocks
                    = cur_fftq.size() / config_->fft_block_size();
                for (size_t i = 0; i < num_fft_blocks; i++) {
                    Event_data do_fft_task;
                    do_fft_task.num_tags = config_->fft_block_size();
                    do_fft_task.event_type = EventType::kFFT;

                    for (size_t j = 0; j < config_->fft_block_size(); j++) {
                        do_fft_task.tags[j] = cur_fftq.front()._tag;
                        cur_fftq.pop();
                        if (fft_created_count++ == 0) {
                            this->stats_->master_set_tsc(
                                TsType::kProcessingStarted, cur_sche_frame_id);
                        } else if (fft_created_count
                            == rx_counters_.num_pkts_per_frame) {
                            fft_created_count = 0;
                            if (cfg->bigstation_mode() == true) {
                                cur_sche_frame_id++;
							}
                        }
                    }
                    try_enqueue_fallback(get_conq(EventType::kFFT, qid),
                        get_ptok(EventType::kFFT, qid), do_fft_task);
                }
            }
        } /* End of for */
    } /* End of while */

finish:
    std::printf("Agora: printing stats and saving to file\n");
    this->stats_->print_summary();
    this->stats_->save_to_file();
    if (flags.enable_save_decode_data_to_file == true) {
        save_decode_data_to_file(this->stats_->last_frame_id());
    }
    if (flags.enable_save_tx_data_to_file == true) {
        save_tx_data_to_file(this->stats_->last_frame_id());
    }

    // Calculate and print per-user BER
    if ((kEnableMac == false) && (kPrintPhyStats == true)) {
        this->phy_stats_->print_phy_stats();
    }
    this->stop();
}

void Agora::handle_event_fft(size_t tag)
{
    size_t frame_id = gen_tag_t(tag).frame_id;
    size_t symbol_id = gen_tag_t(tag).symbol_id;
    SymbolType sym_type = config_->GetSymbolType(frame_id, symbol_id);
    //std::printf("***** handle_event_fft %d\n", (int)sym_type);

    if (sym_type == SymbolType::kPilot) {
        bool last_fft_task = fft_counters_.CompleteTask(frame_id, symbol_id);
        if (last_fft_task == true) {
            print_per_symbol_done(PrintType::kFFTPilots, frame_id, symbol_id);
            if ((config_->frame().NumDLSyms() == 0)
                || ((config_->frame().NumDLSyms() > 0) && (config_->frame().IsRecCalEnabled() == false))
                || ((config_->frame().NumDLSyms() > 0) && (config_->frame().IsRecCalEnabled() == true) && (rc_last_frame == frame_id))) {
                /* If CSI of all UEs is ready, schedule ZF/prediction */
                bool last_fft_symbol = fft_counters_.CompleteSymbol(frame_id);
                if (last_fft_symbol == true) {
                    this->stats_->master_set_tsc(TsType::kFFTPilotsDone, frame_id);
                    print_per_frame_done(PrintType::kFFTPilots, frame_id);
                    if (kPrintPhyStats == true) {
                        this->phy_stats_->print_snr_stats(frame_id);
					}
                    if (kEnableMac == true) {
                        send_snr_report(
                            EventType::kSNRReport, frame_id, symbol_id);
					}
                    schedule_subcarriers(EventType::kZF, frame_id, 0);
                }
            }
        }
    } else if (sym_type == SymbolType::kUL) {
        // TODO, look into. (fft_counters_.Init(cfg->frame().NumPilotSyms(), cfg->bs_ant_num());\)
        bool last_fft_task = fft_counters_.CompleteTask(frame_id, symbol_id); /* Max NumPilotSyms = 8 */
        if (last_fft_task == true) {
            size_t symbol_idx_ul
                = config_->GetULSymbolIdx(frame_id, symbol_id);
            fft_cur_frame_for_symbol_.at(symbol_idx_ul) = frame_id;

            //std::printf("fft_cur_frame_for_symbol_ %zu %zu %zu %zu\n", symbol_idx_ul, symbol_id, frame_id, fft_counters_.GetTaskCount(frame_id, symbol_id));
            print_per_symbol_done(PrintType::kFFTData, frame_id, symbol_id);
            /* If precoder exist, schedule demodulation */
            if (zf_last_frame == frame_id) {
                std::printf("HELP %zu %zu\n\n", symbol_idx_ul, symbol_id);
                schedule_subcarriers(
                    EventType::kDemul, frame_id, symbol_idx_ul);
            }
        }
    } else if ((sym_type == SymbolType::kCalDL) || (sym_type == SymbolType::kCalUL)) {
        print_per_symbol_done(PrintType::kFFTCal, frame_id, symbol_id);
        
        bool last_rc_task = this->rc_counters_.CompleteTask(frame_id);
        if (last_rc_task == true) {
            print_per_frame_done(PrintType::kFFTCal, frame_id);
            this->stats_->master_set_tsc(TsType::kRCDone, frame_id);
            rc_last_frame = frame_id;
        }
    }
}

void Agora::worker(int tid)
{
    pin_to_core_with_offset(
        ThreadType::kWorker, base_worker_core_offset, tid, false /* quiet */);

    /* Initialize operators */
    std::unique_ptr<DoZF> computeZF ( new DoZF(this->config_, tid, this->csi_buffers_, this->calib_dl_buffer_,
        this->calib_ul_buffer_, this->ul_zf_matrices_, this->dl_zf_matrices_, this->stats_.get()));

    std::unique_ptr<DoFFT> computeFFT ( new DoFFT(this->config_, tid, this->socket_buffer_,
        this->socket_buffer_status_, this->data_buffer_, this->csi_buffers_, this->calib_dl_buffer_,
        this->calib_ul_buffer_, this->phy_stats_.get(), this->stats_.get()) );

    //Downlink workers
    std::unique_ptr<DoIFFT> computeIFFT ( new DoIFFT(this->config_, tid, this->dl_ifft_buffer_, this->dl_socket_buffer_, this->stats_.get()) );

    std::unique_ptr<DoPrecode> computePrecode ( new DoPrecode(this->config_, tid, this->dl_zf_matrices_,
        this->dl_ifft_buffer_, this->dl_encoded_buffer_, this->stats_.get()));

    std::unique_ptr<DoEncode> computeEncoding ( new DoEncode(
        this->config_, tid, this->config_->dl_bits(), this->dl_encoded_buffer_, this->stats_.get()));

    //Uplink workers
    std::unique_ptr<DoDecode> computeDecoding ( new DoDecode(
        this->config_, tid, this->demod_buffers_, this->decoded_buffer_, this->phy_stats_.get(), this->stats_.get()));

    std::unique_ptr<DoDemul> computeDemul( new DoDemul(this->config_, tid, this->data_buffer_, this->ul_zf_matrices_,
        this->ue_spec_pilot_buffer_, this->equal_buffer_, this->demod_buffers_, this->phy_stats_.get(), this->stats_.get()));

    std::vector<Doer*> computers_vec;
    std::vector<EventType> events_vec;
    ///*************************
    computers_vec.push_back( computeZF.get() );
    computers_vec.push_back( computeFFT.get() );
    events_vec.push_back(EventType::kZF);
    events_vec.push_back(EventType::kFFT);

    if (config_->frame().NumULSyms() > 0) {
        computers_vec.push_back( computeDecoding.get() );
        computers_vec.push_back( computeDemul.get() );
        events_vec.push_back(EventType::kDecode);
        events_vec.push_back(EventType::kDemul);
    }

    if (config_->frame().NumDLSyms() > 0) {
        computers_vec.push_back( computeIFFT.get() );
        computers_vec.push_back( computePrecode.get() );
        computers_vec.push_back( computeEncoding.get() );
        events_vec.push_back(EventType::kIFFT);
        events_vec.push_back(EventType::kPrecode);
        events_vec.push_back(EventType::kEncode);
    }
    
    /*
    if (config_->frame().NumULSyms() > 0) {
        computers_vec.push_back( computeDecoding.get() );
        computers_vec.push_back( computeDemul.get() );
        events_vec.push_back(EventType::kDecode);
        events_vec.push_back(EventType::kDemul);
    }*/

    size_t cur_qid = 0;
    size_t empty_qeueu_itrs = 0;
    bool empty_queue = true;
    while (this->config_->running() == true) {
        for (size_t i = 0; i < computers_vec.size(); i++) {
            if (computers_vec.at(i)->try_launch(*get_conq(events_vec.at(i), cur_qid),
                    complete_task_queue_[cur_qid],
                    worker_ptoks_ptr[tid][cur_qid])) {
                empty_queue = false;
                break;
            }
        }
        // If all queues in this set are empty for 5 iterations,
        // check the other set of qeueus
        if (empty_queue == true) {
            empty_qeueu_itrs++;
            if (empty_qeueu_itrs == 5) {
                if (cur_sche_frame_id != cur_proc_frame_id) {
                    cur_qid ^= 0x1;
                }
                else {
                    cur_qid = cur_sche_frame_id & 0x1;
                }
                empty_qeueu_itrs = 0;
            }
        } else {
            empty_queue = true;
        }
    }
    std::printf( "Agora worker %d exit\n", tid);
}

void Agora::worker_fft(int tid)
{
    pin_to_core_with_offset(
        ThreadType::kWorkerFFT, base_worker_core_offset, tid);

    /* Initialize IFFT operator */
    std::unique_ptr<DoFFT> computeFFT( new DoFFT(config_, tid, socket_buffer_,
        socket_buffer_status_, data_buffer_, csi_buffers_, calib_dl_buffer_,
        calib_ul_buffer_, this->phy_stats_.get(), this->stats_.get()));
    std::unique_ptr<DoIFFT> computeIFFT
        ( new DoIFFT(config_, tid, dl_ifft_buffer_, dl_socket_buffer_, this->stats_.get()) );

    while (this->config_->running() == true) {
        //TODO refactor the if / else 
        if (computeFFT->try_launch(*get_conq(EventType::kFFT, 0),
                complete_task_queue_[0], worker_ptoks_ptr[tid][0]) == true) {
            //Do nothing
        } else if ( (config_->frame().NumDLSyms() > 0)
            && (computeIFFT->try_launch(*get_conq(EventType::kIFFT, 0),
                   complete_task_queue_[0], worker_ptoks_ptr[tid][0]) == true) ) {
            //Do nothing
        }
    }
}

void Agora::worker_zf(int tid)
{
    pin_to_core_with_offset(
        ThreadType::kWorkerZF, base_worker_core_offset, tid);

    /* Initialize ZF operator */
    std::unique_ptr<DoZF> computeZF ( new DoZF(config_, tid, csi_buffers_, calib_dl_buffer_,
        calib_ul_buffer_, ul_zf_matrices_, dl_zf_matrices_, this->stats_.get()) );

    while (this->config_->running() == true) {
        computeZF->try_launch(*get_conq(EventType::kZF, 0),
            complete_task_queue_[0], worker_ptoks_ptr[tid][0]);
    }
}

//TODO rework to decouple kDemul or kPrecode
void Agora::worker_demul(int tid)
{
    pin_to_core_with_offset(
        ThreadType::kWorkerDemul, base_worker_core_offset, tid);

    std::unique_ptr<DoDemul> computeDemul ( new DoDemul(config_, tid, data_buffer_, ul_zf_matrices_,
        ue_spec_pilot_buffer_, equal_buffer_, demod_buffers_, this->phy_stats_.get(), this->stats_.get()) );

    /* Initialize Precode operator */
    std::unique_ptr<DoPrecode> computePrecode ( new DoPrecode(config_, tid, dl_zf_matrices_,
        dl_ifft_buffer_, dl_encoded_buffer_, this->stats_.get()) );

    //std::printf("!!!!!!!!!!!!! demul worker !!!!!!!!!!!!!!!!!1 \n");
    assert( false );

    while (this->config_->running() == true) {
        if (config_->frame().NumDLSyms() > 0) {
            computePrecode->try_launch(*get_conq(EventType::kDemul, 0),
                complete_task_queue_[0], worker_ptoks_ptr[tid][0]);
        } else {
            computeDemul->try_launch(*get_conq(EventType::kPrecode, 0),
                complete_task_queue_[0], worker_ptoks_ptr[tid][0]);
        }
    }
}

//TODO rework to decouple kEncode and kDecode
void Agora::worker_decode(int tid)
{
    pin_to_core_with_offset(
        ThreadType::kWorkerDecode, base_worker_core_offset, tid);

    std::unique_ptr<DoEncode> computeEncoding ( new DoEncode(
        config_, tid, config_->dl_bits(), dl_encoded_buffer_, this->stats_.get()) );

    std::unique_ptr<DoDecode> computeDecoding ( new DoDecode(
        config_, tid, demod_buffers_, decoded_buffer_, this->phy_stats_.get(), this->stats_.get()));

    while (this->config_->running() == true) {
        if (config_->frame().NumDLSyms() > 0) {
            computeEncoding->try_launch(*get_conq(EventType::kEncode, 0),
                complete_task_queue_[0], worker_ptoks_ptr[tid][0]);
        } else {
            computeDecoding->try_launch(*get_conq(EventType::kDecode, 0),
                complete_task_queue_[0], worker_ptoks_ptr[tid][0]);
        }
    }
}

void Agora::create_threads()
{
    auto& cfg = config_;
    if (cfg->bigstation_mode() == true) {
        for (size_t i = 0; i < cfg->fft_thread_num(); i++) {
            workers_.push_back( std::thread(&Agora::worker_fft, this, i) );
		}
        for (size_t i = cfg->fft_thread_num();
             i < cfg->fft_thread_num() + cfg->zf_thread_num(); i++) {
            workers_.push_back( std::thread(&Agora::worker_zf, this, i) );
		}
        for (size_t i = cfg->fft_thread_num() + cfg->zf_thread_num(); i
             < cfg->fft_thread_num() + cfg->zf_thread_num() + cfg->demul_thread_num();
             i++) {
            workers_.push_back( std::thread(&Agora::worker_demul, this, i) );
		}
        for (size_t i
             = cfg->fft_thread_num() + cfg->zf_thread_num() + cfg->demul_thread_num();
             i < cfg->worker_thread_num(); i++) {
            workers_.push_back( std::thread(&Agora::worker_decode, this, i) );
		}
    } else {
        //printf("Agora: creating %zu workers\n", cfg->worker_thread_num());
        for (size_t i = 0; i < cfg->worker_thread_num(); i++) {
            workers_.push_back( std::thread(&Agora::worker, this, i) );
		}
    }
}

void Agora::update_ran_config(RanConfig rc)
{
    config_->UpdateModCfgs(rc.mod_order_bits);
}

void Agora::update_rx_counters(size_t frame_id, size_t symbol_id)
{
    const size_t frame_slot = frame_id % kFrameWnd;
    if (config_->IsPilot(frame_id, symbol_id)) {
        rx_counters_.num_pilot_pkts[frame_slot]++;
        if (rx_counters_.num_pilot_pkts[frame_slot]
            == rx_counters_.num_pilot_pkts_per_frame) {
            rx_counters_.num_pilot_pkts[frame_slot] = 0;
            this->stats_->master_set_tsc(TsType::kPilotAllRX, frame_id);
            print_per_frame_done(PrintType::kPacketRXPilots, frame_id);
        }
    } else if (config_->IsCalDlPilot(frame_id, symbol_id)
        or config_->IsCalUlPilot(frame_id, symbol_id)) {
        if (++rx_counters_.num_reciprocity_pkts[frame_slot]
            == rx_counters_.num_reciprocity_pkts_per_frame) {
            rx_counters_.num_reciprocity_pkts[frame_slot] = 0;
            this->stats_->master_set_tsc(TsType::kRCAllRX, frame_id);
        }
    }
    // Receive first packet in a frame
    if (rx_counters_.num_pkts[frame_slot] == 0) {
        // schedule this frame's encoding
        for (size_t i = 0; i < config_->frame().NumDLSyms(); i++)
            schedule_codeblocks(
                EventType::kEncode, frame_id, config_->frame().GetDLSymbol(i));
        this->stats_->master_set_tsc(TsType::kPilotRX, frame_id);
        if (kDebugPrintPerFrameStart) {
            const size_t prev_frame_slot
                = (frame_slot + kFrameWnd - 1) % kFrameWnd;
            std::printf("Main [frame %zu + %.2f ms since last frame]: Received "
                        "first packet. Remaining packets in prev frame: %zu\n",
                frame_id,
                this->stats_->master_get_delta_ms(
                    TsType::kPilotRX, frame_id, frame_id - 1),
                rx_counters_.num_pkts[prev_frame_slot]);
        }
    }

    rx_counters_.num_pkts[frame_slot]++;
    if (rx_counters_.num_pkts[frame_slot] == rx_counters_.num_pkts_per_frame) {
        this->stats_->master_set_tsc(TsType::kRXDone, frame_id);
        print_per_frame_done(PrintType::kPacketRX, frame_id);
        rx_counters_.num_pkts[frame_slot] = 0;
    }
}

void Agora::print_per_frame_done(PrintType print_type, size_t frame_id)
{
    if (kDebugPrintPerFrameDone == false) {
        return;
    }

    switch (print_type) {
    case (PrintType::kPacketRXPilots):
        std::printf("Main [frame %zu + %.2f ms]: Received all pilots\n",
            frame_id,
            this->stats_->master_get_delta_ms(
                TsType::kPilotAllRX, TsType::kPilotRX, frame_id));
        break;
    case (PrintType::kPacketRX): {
        std::printf("Main [frame %zu + %.2f ms]: Received all packets\n",
            frame_id,
            this->stats_->master_get_delta_ms(
                TsType::kRXDone, TsType::kPilotRX, frame_id));
    } break;
    case (PrintType::kFFTPilots):
        std::printf("Main [frame %zu + %.2f ms]: FFT-ed all pilots\n", frame_id,
            this->stats_->master_get_delta_ms(
                TsType::kFFTPilotsDone, TsType::kPilotRX, frame_id));
        break;
    case (PrintType::kFFTCal):
        std::printf(
            "Main [frame %zu + %.2f ms]: FFT-ed all calibration symbols\n",
            frame_id,
            this->stats_->master_get_us_since(TsType::kRCAllRX, frame_id) / 1000.0);
        break;
    case (PrintType::kZF):
        std::printf("Main [frame %zu + %.2f ms]: Completed zero-forcing\n",
            frame_id,
            this->stats_->master_get_delta_ms(
                TsType::kZFDone, TsType::kPilotRX, frame_id));
        break;
    case (PrintType::kDemul):
        std::printf("Main [frame %zu + %.2f ms]: Completed demodulation\n",
            frame_id,
            this->stats_->master_get_delta_ms(
                TsType::kDemulDone, TsType::kPilotRX, frame_id));
        break;
    case (PrintType::kDecode):
        std::printf(
            "Main [frame %zu + %.2f ms]: Completed LDPC decoding (%zu UL "
            "symbols)\n",
            frame_id,
            this->stats_->master_get_delta_ms(
                TsType::kDecodeDone, TsType::kPilotRX, frame_id),
            config_->frame().NumULSyms());
        break;
    case (PrintType::kEncode):
        std::printf("Main [frame %zu + %.2f ms]: Completed LDPC encoding\n",
            frame_id,
            this->stats_->master_get_delta_ms(
                TsType::kEncodeDone, TsType::kPilotRX, frame_id));
        break;
    case (PrintType::kPrecode):
        std::printf("Main [frame %zu + %.2f ms]: Completed precoding\n",
            frame_id,
            this->stats_->master_get_delta_ms(
                TsType::kPrecodeDone, TsType::kPilotRX, frame_id));
        break;
    case (PrintType::kIFFT):
        std::printf("Main [frame %zu + %.2f ms]: Completed IFFT\n", frame_id,
            this->stats_->master_get_delta_ms(
                TsType::kIFFTDone, TsType::kPilotRX, frame_id));
        break;
    case (PrintType::kPacketTXFirst):
        std::printf(
            "Main [frame %zu + %.2f ms]: Completed TX of first symbol\n",
            frame_id,
            this->stats_->master_get_delta_ms(
                TsType::kTXProcessedFirst, TsType::kPilotRX, frame_id));
        break;
    case (PrintType::kPacketTX):
        std::printf(
            "Main [frame %zu + %.2f ms]: Completed TX (%zu DL symbols)\n",
            frame_id,
            this->stats_->master_get_delta_ms(
                TsType::kTXDone, TsType::kPilotRX, frame_id),
            config_->frame().NumDLSyms());
        break;
    case (PrintType::kPacketToMac):
        std::printf("Main [frame %zu + %.2f ms]: Completed MAC TX \n", frame_id,
            this->stats_->master_get_ms_since(TsType::kPilotRX, frame_id));
        break;
    default:
        std::printf("Wrong task type in frame done print!");
    }
}

void Agora::print_per_symbol_done(
    PrintType print_type, size_t frame_id, size_t symbol_id)
{
    if (kDebugPrintPerSymbolDone == false) {
        return;
    }
    switch (print_type) {
    case (PrintType::kFFTPilots):
        std::printf(
            "Main [frame %zu symbol %zu + %.3f ms]: FFT-ed pilot symbol, "
            "%zu symbols done\n",
            frame_id, symbol_id,
            this->stats_->master_get_ms_since(TsType::kPilotRX, frame_id),
            fft_counters_.GetSymbolCount(frame_id) + 1);
        break;
    case (PrintType::kFFTData):
        std::printf(
            "Main [frame %zu symbol %zu + %.3f ms]: FFT-ed data symbol, "
            "precoder status: %d\n",
            frame_id, symbol_id,
            this->stats_->master_get_ms_since(TsType::kPilotRX, frame_id),
            zf_last_frame == frame_id);
        break;
    case (PrintType::kDemul):
        std::printf(
            "Main [frame %zu symbol %zu + %.3f ms]: Completed demodulation, "
            "%zu symbols done\n",
            frame_id, symbol_id,
            this->stats_->master_get_ms_since(TsType::kPilotRX, frame_id),
            demul_counters_.GetSymbolCount(frame_id) + 1);
        break;
    case (PrintType::kDecode):
        std::printf(
            "Main [frame %zu symbol %zu + %.3f ms]: Completed decoding, "
            "%zu symbols done\n",
            frame_id, symbol_id,
            this->stats_->master_get_ms_since(TsType::kPilotRX, frame_id),
            decode_counters_.GetSymbolCount(frame_id) + 1);
        break;
    case (PrintType::kEncode):
        std::printf(
            "Main [frame %zu symbol %zu + %.3f ms]: Completed encoding, "
            "%zu symbols done\n",
            frame_id, symbol_id,
            this->stats_->master_get_ms_since(TsType::kPilotRX, frame_id),
            encode_counters_.GetSymbolCount(frame_id) + 1);
        break;
    case (PrintType::kPrecode):
        std::printf(
            "Main [frame %zu symbol %zu + %.3f ms]: Completed precoding, "
            "%zu symbols done\n",
            frame_id, symbol_id,
            this->stats_->master_get_ms_since(TsType::kPilotRX, frame_id),
            precode_counters_.GetSymbolCount(frame_id) + 1);
        break;
    case (PrintType::kIFFT):
        std::printf("Main [frame %zu symbol %zu + %.3f ms]: Completed IFFT, "
                    "%zu symbols done\n",
            frame_id, symbol_id,
            this->stats_->master_get_ms_since(TsType::kPilotRX, frame_id),
            ifft_counters_.GetSymbolCount(frame_id) + 1);
        break;
    case (PrintType::kPacketTX):
        std::printf("Main [frame %zu symbol %zu + %.3f ms]: Completed TX, "
                    "%zu symbols done\n",
            frame_id, symbol_id,
            this->stats_->master_get_ms_since(TsType::kPilotRX, frame_id),
            tx_counters_.GetSymbolCount(frame_id) + 1);
        break;
    case (PrintType::kPacketToMac):
        std::printf("Main [frame %zu symbol %zu + %.3f ms]: Completed MAC TX, "
                    "%zu symbols done\n",
            frame_id, symbol_id,
            this->stats_->master_get_ms_since(TsType::kPilotRX, frame_id),
            tomac_counters_.GetSymbolCount(frame_id) + 1);
        break;
    default:
        std::printf("Wrong task type in symbol done print!");
    }
}

void Agora::print_per_task_done(PrintType print_type, size_t frame_id,
    size_t symbol_id, size_t ant_or_sc_id)
{
    if (kDebugPrintPerTaskDone == true) {
        switch (print_type) {
        case (PrintType::kZF):
            //std::printf("Main thread: ZF done frame: %zu, subcarrier %zu\n",
            //    frame_id, ant_or_sc_id);
            break;
        case (PrintType::kRC):
            std::printf("Main thread: RC done frame: %zu, subcarrier %zu\n",
                frame_id, ant_or_sc_id);
            break;
        case (PrintType::kDemul):
            std::printf(
                "Main thread: Demodulation done frame: %zu, symbol: %zu, sc: "
                "%zu, num blocks done: %zu\n",
                frame_id, symbol_id, ant_or_sc_id,
                demul_counters_.GetTaskCount(frame_id, symbol_id));
            break;
        case (PrintType::kDecode):
            std::printf(
                "Main thread: Decoding done frame: %zu, symbol: %zu, sc: %zu, "
                "num blocks done: %zu\n",
                frame_id, symbol_id, ant_or_sc_id,
                decode_counters_.GetTaskCount(frame_id, symbol_id));
            break;
        case (PrintType::kPrecode):
            std::printf("Main thread: Precoding done frame: %zu, symbol: %zu, "
                        "subcarrier: %zu, total SCs: %zu\n",
                frame_id, symbol_id, ant_or_sc_id,
                precode_counters_.GetTaskCount(frame_id, symbol_id));
            break;
        case (PrintType::kIFFT):
            std::printf(
                "Main thread: IFFT done frame: %zu, symbol: %zu, antenna: %zu, "
                "total ants: %zu\n",
                frame_id, symbol_id, ant_or_sc_id,
                ifft_counters_.GetTaskCount(frame_id, symbol_id));
            break;
        case (PrintType::kPacketTX):
            std::printf(
                "Main thread: TX done frame: %zu, symbol: %zu, antenna: %zu, "
                "total packets: %zu\n",
                frame_id, symbol_id, ant_or_sc_id,
                tx_counters_.GetTaskCount(frame_id, symbol_id));
            break;
        default:
            std::printf("Wrong task type in task done print!");
        }
    }
}

void Agora::initialize_queues()
{
    using mt_queue_t = moodycamel::ConcurrentQueue<Event_data>;

    int data_symbol_num_perframe = config_->frame().NumDataSyms();
    message_queue_ = mt_queue_t(kDefaultMessageQueueSize * data_symbol_num_perframe);
    for (auto& c : complete_task_queue_) {
        c = mt_queue_t(kDefaultWorkerQueueSize * data_symbol_num_perframe);
    }
    // Create concurrent queues for each Doer
    for (auto& vec : sched_info_arr) {
        for (auto& s : vec) {
            s.concurrent_q = mt_queue_t(kDefaultWorkerQueueSize * data_symbol_num_perframe);
            s.ptok = new moodycamel::ProducerToken(s.concurrent_q);
        }
    }

    for (size_t i = 0; i < config_->socket_thread_num(); i++) {
        rx_ptoks_ptr[i] = new moodycamel::ProducerToken(message_queue_);
        tx_ptoks_ptr[i]
            = new moodycamel::ProducerToken(*get_conq(EventType::kPacketTX, 0));
    }

    for (size_t i = 0; i < config_->worker_thread_num(); i++) {
        for (size_t j = 0; j < 2; j++) {
            worker_ptoks_ptr[i][j]
                = new moodycamel::ProducerToken(complete_task_queue_[j]);
        }
    }
}

void Agora::free_queues()
{
    // remove tokens for each doer
    for (auto& vec : sched_info_arr) {
        for (auto& s : vec) {
            delete s.ptok;
        }
    }

    for (size_t i = 0; i < config_->socket_thread_num(); i++) {
        delete rx_ptoks_ptr[i];
        delete tx_ptoks_ptr[i];
    }

    for (size_t i = 0; i < config_->worker_thread_num(); i++) {
        for (size_t j = 0; j < 2; j++) {
            delete worker_ptoks_ptr[i][j];
        }
    }
}

void Agora::initialize_uplink_buffers()
{
    auto& cfg = config_;
    const size_t task_buffer_symbol_num_ul = cfg->frame().NumULSyms() * kFrameWnd;

    socket_buffer_status_size_
        = cfg->bs_ant_num() * kFrameWnd * cfg->frame().NumTotalSyms();
    socket_buffer_size_ = cfg->packet_length() * socket_buffer_status_size_;

    socket_buffer_.malloc(cfg->socket_thread_num() /* RX */, socket_buffer_size_,
        Agora_memory::Alignment_t::k64Align);
    socket_buffer_status_.calloc(cfg->socket_thread_num() /* RX */,
        socket_buffer_status_size_, Agora_memory::Alignment_t::k64Align);

    data_buffer_.malloc(task_buffer_symbol_num_ul,
        cfg->ofdm_data_num() * cfg->bs_ant_num(),
        Agora_memory::Alignment_t::k64Align);

    equal_buffer_.malloc(task_buffer_symbol_num_ul,
        cfg->ofdm_data_num() * cfg->ue_num(), Agora_memory::Alignment_t::k64Align);
    ue_spec_pilot_buffer_.calloc(kFrameWnd, cfg->frame().client_ul_pilot_symbols() * cfg->ue_num(),
        Agora_memory::Alignment_t::k64Align);

    rx_counters_.num_pkts_per_frame = cfg->bs_ant_num()
        * (cfg->frame().NumPilotSyms() + cfg->frame().NumULSyms()
              + static_cast<size_t>(cfg->frame().IsRecCalEnabled()));
    rx_counters_.num_pilot_pkts_per_frame
        = cfg->bs_ant_num() * cfg->frame().NumPilotSyms();
    rx_counters_.num_reciprocity_pkts_per_frame = cfg->bs_ant_num();

    fft_created_count = 0;
    fft_counters_.Init(cfg->frame().NumPilotSyms(), cfg->bs_ant_num());
    fft_cur_frame_for_symbol_
        = std::vector<size_t>(cfg->frame().NumULSyms(), SIZE_MAX);

    rc_counters_.Init(cfg->bs_ant_num());

    zf_counters_.Init(config_->zf_events_per_symbol());

    demul_counters_.Init(
        cfg->frame().NumULSyms(), config_->demul_events_per_symbol());

    decode_counters_.Init(cfg->frame().NumULSyms(),
        config_->ldpc_config().num_blocks_in_symbol() * cfg->ue_num());

    tomac_counters_.Init(cfg->frame().NumULSyms(), cfg->ue_num());
}

void Agora::initialize_downlink_buffers( void )
{
    if (config_->frame().NumDLSyms() > 0) {
        std::printf("Agora: Initializing downlink buffers\n");

        const size_t task_buffer_symbol_num
            = config_->frame().NumDLSyms() * kFrameWnd;

        size_t dl_socket_buffer_status_size
            = config_->bs_ant_num() * kFrameWnd * config_->frame().NumDLSyms();
        size_t dl_socket_buffer_size
            = config_->dl_packet_length() * dl_socket_buffer_status_size;
        alloc_buffer_1d(&dl_socket_buffer_, dl_socket_buffer_size,
            Agora_memory::Alignment_t::k64Align, 0);
        alloc_buffer_1d(&dl_socket_buffer_status_, dl_socket_buffer_status_size,
            Agora_memory::Alignment_t::k64Align, 1);

        this->dl_bits_buffer_.calloc(task_buffer_symbol_num,
            config_->ofdm_data_num() * config_->ue_num(), Agora_memory::Alignment_t::k64Align);
        size_t dl_bits_buffer_status_size
            = task_buffer_symbol_num * config_->ldpc_config().num_blocks_in_symbol();
        this->dl_bits_buffer_status_.calloc(config_->ue_num(), dl_bits_buffer_status_size,
            Agora_memory::Alignment_t::k64Align);

        dl_ifft_buffer_.calloc(config_->bs_ant_num() * task_buffer_symbol_num,
            config_->ofdm_ca_num(), Agora_memory::Alignment_t::k64Align);
        calib_dl_buffer_.calloc(kFrameWnd, config_->bf_ant_num() * config_->ofdm_data_num(),
            Agora_memory::Alignment_t::k64Align);
        calib_ul_buffer_.calloc(kFrameWnd, config_->bf_ant_num() * config_->ofdm_data_num(),
            Agora_memory::Alignment_t::k64Align);
        // initialize the content of the last window to 1
        for (size_t i = 0; i < config_->ofdm_data_num() * config_->bf_ant_num(); i++) {
            calib_dl_buffer_[kFrameWnd - 1][i] = { 1, 0 };
            calib_ul_buffer_[kFrameWnd - 1][i] = { 1, 0 };
        }
        dl_encoded_buffer_.calloc(task_buffer_symbol_num,
            roundup<64>(config_->ofdm_data_num()) * config_->ue_num(),
            Agora_memory::Alignment_t::k64Align);

        encode_counters_.Init(config_->frame().NumDLSyms(),
            config_->ldpc_config().num_blocks_in_symbol() * config_->ue_num());
        encode_cur_frame_for_symbol_
            = std::vector<size_t>(config_->frame().NumDLSyms(), SIZE_MAX);
        precode_counters_.Init(
            config_->frame().NumDLSyms(), config_->demul_events_per_symbol());
        ifft_counters_.Init(config_->frame().NumDLSyms(), config_->bs_ant_num());
        tx_counters_.Init(config_->frame().NumDLSyms(), config_->bs_ant_num());
    }
}

void Agora::free_uplink_buffers( void )
{
    socket_buffer_.free();
    socket_buffer_status_.free();
    data_buffer_.free();
    equal_buffer_.free();
    ue_spec_pilot_buffer_.free();
}

void Agora::free_downlink_buffers( void )
{
    if (config_->frame().NumDLSyms() > 0) {

        free_buffer_1d(&dl_socket_buffer_);
        free_buffer_1d(&dl_socket_buffer_status_);

        dl_ifft_buffer_.free();
        calib_dl_buffer_.free();
        calib_ul_buffer_.free();
        dl_encoded_buffer_.free();
        dl_bits_buffer_.free();
        dl_bits_buffer_status_.free();
    }
}

void Agora::save_decode_data_to_file(int frame_id)
{
    auto& cfg = config_;
    const size_t num_decoded_bytes
        = cfg->num_bytes_per_cb() * cfg->ldpc_config().num_blocks_in_symbol();

    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string filename = cur_directory + "/data/decode_data.bin";
    std::printf("Saving decode data to %s\n", filename.c_str());
    FILE* fp = std::fopen(filename.c_str(), "wb");

    for (size_t i = 0; i < cfg->frame().NumULSyms(); i++) {
        for (size_t j = 0; j < cfg->ue_num(); j++) {
            uint8_t* ptr = decoded_buffer_[frame_id % kFrameWnd][i][j];
            fwrite(ptr, num_decoded_bytes, sizeof(uint8_t), fp);
        }
    }
    std::fclose(fp);
}

void Agora::save_tx_data_to_file(UNUSED int frame_id)
{
    auto& cfg = config_;

    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string filename = cur_directory + "/data/tx_data.bin";
    std::printf("Saving TX data to %s\n", filename.c_str());
    FILE* fp = std::fopen(filename.c_str(), "wb");

    for (size_t i = 0; i < cfg->frame().NumDLSyms(); i++) {
        size_t total_data_symbol_id
            = cfg->GetTotalDataSymbolIdxDl(frame_id, i);

        for (size_t ant_id = 0; ant_id < cfg->bs_ant_num(); ant_id++) {
            size_t offset = total_data_symbol_id * cfg->bs_ant_num() + ant_id;
            struct Packet* pkt = (struct Packet*)(&dl_socket_buffer_[offset
                * cfg->dl_packet_length()]);
            short* socket_ptr = pkt->data;
            fwrite(socket_ptr, cfg->samps_per_symbol() * 2, sizeof(short), fp);
        }
    }
    std::fclose(fp);
}

void Agora::getEqualData(float** ptr, int* size)
{
    auto& cfg = config_;
    auto offset = cfg->GetTotalDataSymbolIdxUl(
        max_equaled_frame, cfg->frame().client_ul_pilot_symbols());
    *ptr = (float*)&equal_buffer_[offset][0];
    *size = cfg->ue_num() * cfg->ofdm_data_num() * 2;
}

bool Agora::CheckWorkComplete( size_t frame_id ) const
{
    bool finished = false;

    std::printf("\nChecking work complete %zu, ifft %d, tx %d, decode %d, tomac %d\n\n", frame_id, this->ifft_counters_.IsLastSymbol( frame_id ), this->tx_counters_.IsLastSymbol( frame_id )
    , this->decode_counters_.IsLastSymbol( frame_id ), this->tomac_counters_.IsLastSymbol( frame_id ));

    // Complete if last frame and ifft / decode complete
    if ( (frame_id == (this->config_->frames_to_test() - 1))     && 
         (true == this->ifft_counters_.IsLastSymbol( frame_id )) && 
         (true == this->tx_counters_.IsLastSymbol( frame_id ))   &&
         ( ((kEnableMac == false) && (true == this->decode_counters_.IsLastSymbol( frame_id ))) || 
           ((kEnableMac == true)  && (true == this->tomac_counters_.IsLastSymbol(frame_id))) )
          ) {
        finished = true;
    }
    return finished;
}


extern "C" {
EXPORT Agora* Agora_new(Config* cfg)
{
    // std::printf("Size of Agora: %d\n",sizeof(Agora *));
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
