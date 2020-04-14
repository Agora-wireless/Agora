/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#include "millipede.hpp"
using namespace std;

Millipede::Millipede(Config* cfg)
{
    std::string directory = TOSTRING(PROJECT_DIRECTORY);
    printf("Millipede: project directory %s\n", directory.c_str());
    // setenv("MKL_THREADING_LAYER", "sequential", true /* overwrite */);
    // std::cout << "MKL_THREADING_LAYER =  " << getenv("MKL_THREADING_LAYER")
    // << std::endl; openblas_set_num_threads(1);

    this->config_ = cfg;
#if DEBUG_PRINT_PILOT
    cout << "Millipede: Pilot data: " << endl;
    for (size_t i = 0; i < cfg->OFDM_CA_NUM; i++)
        cout << config_->pilots_[i] << ",";
    cout << endl;
#endif

    freq_ghz = measure_rdtsc_freq();
    printf("Measured RDTSC frequency = %.2f\n", freq_ghz);

    pin_to_core_with_offset(ThreadType::kMaster, cfg->core_offset, 0);
    initialize_queues();
    initialize_uplink_buffers();

    if (config_->dl_data_symbol_num_perframe > 0) {
        printf("Millipede: Initializing downlink buffers\n");
        initialize_downlink_buffers();
    }

    stats = new Stats(config_, kMaxStatBreakdown, cfg->worker_thread_num,
        cfg->fft_thread_num, cfg->zf_thread_num, cfg->demul_thread_num,
        freq_ghz);

    /* Initialize TXRX threads*/
    receiver_.reset(
        new PacketTXRX(config_, cfg->socket_thread_num, cfg->core_offset + 1,
            &message_queue_, &tx_queue_, rx_ptoks_ptr, tx_ptoks_ptr));

    /* Create worker threads */
    if (config_->bigstation_mode) {
        create_threads(pthread_fun_wrapper<Millipede, &Millipede::worker_fft>,
            0, cfg->fft_thread_num);
        create_threads(pthread_fun_wrapper<Millipede, &Millipede::worker_zf>,
            cfg->fft_thread_num, cfg->fft_thread_num + cfg->zf_thread_num);
        create_threads(pthread_fun_wrapper<Millipede, &Millipede::worker_demul>,
            cfg->fft_thread_num + cfg->zf_thread_num, cfg->worker_thread_num);
    } else {
        create_threads(pthread_fun_wrapper<Millipede, &Millipede::worker>, 0,
            cfg->worker_thread_num);
    }
}

Millipede::~Millipede()
{
    free_uplink_buffers();
    /* Downlink */
    if (config_->dl_data_symbol_num_perframe > 0)
        free_downlink_buffers();
}

void Millipede::stop()
{
    std::cout << "Millipede: stopping threads" << std::endl;
    config_->running = false;
    usleep(1000);
    receiver_.reset();
}

/// Enqueue a batch of task_set_size tasks starting from task index
/// (task_set_size * task_set_id)
static void schedule_task_set(EventType task_type, int task_set_size,
    int task_set_id, moodycamel::ConcurrentQueue<Event_data>& task_queue,
    moodycamel::ProducerToken& producer_token)
{
    Event_data task(task_type, task_set_size * task_set_id);
    for (int i = 0; i < task_set_size; i++) {
        try_enqueue_fallback(task_queue, producer_token, task);
        task.data++;
    }
}

void Millipede::start()
{
    auto& cfg = config_;

    /* start txrx receiver */
    if (!receiver_->startTXRX(socket_buffer_, socket_buffer_status_,
            socket_buffer_status_size_, socket_buffer_size_, stats->frame_start,
            dl_socket_buffer_, dl_socket_buffer_status_,
            dl_socket_buffer_status_size_, dl_socket_buffer_size_)) {
        this->stop();
        return;
    }

    /* Tokens used for dequeue */
    moodycamel::ConsumerToken ctok(message_queue_);
    moodycamel::ConsumerToken ctok_complete(complete_task_queue_);

    size_t cur_frame_id = 0;

    /* Counters for printing summary */
    int demul_count = 0;
    int tx_count = 0;
    double demul_begin = get_time_us();
    double tx_begin = get_time_us();

    bool is_turn_to_dequeue_from_io = true;
    const size_t max_events_needed
        = std::max(kDequeueBulkSizeWorker * cfg->socket_thread_num,
            kDequeueBulkSizeTXRX * cfg->worker_thread_num);
    Event_data events_list[max_events_needed];

    while (config_->running && !SignalHandler::gotExitSignal()) {
        /* Get a batch of events */
        int num_events = 0;
        if (is_turn_to_dequeue_from_io) {
            for (size_t i = 0; i < config_->socket_thread_num; i++) {
                num_events += message_queue_.try_dequeue_bulk_from_producer(
                    *(rx_ptoks_ptr[i]), events_list + num_events,
                    kDequeueBulkSizeTXRX);
            }
        } else {
            for (size_t i = 0; i < config_->worker_thread_num; i++) {
                num_events
                    += complete_task_queue_.try_dequeue_bulk_from_producer(
                        *(worker_ptoks_ptr[i]), events_list + num_events,
                        kDequeueBulkSizeWorker);
            }
            // ret = complete_task_queue_.try_dequeue_bulk(
            //     ctok_complete, events_list, dequeue_bulk_size_single);
        }
        is_turn_to_dequeue_from_io = !is_turn_to_dequeue_from_io;

        /* Handle each event */
        int frame_count = 0;
        for (int ev_i = 0; ev_i < num_events; ev_i++) {
            Event_data& event = events_list[ev_i];

            // FFT processing is scheduled after falling through the switch
            switch (event.event_type) {
            case EventType::kPacketRX: {
                int offset_in_current_buffer = rx_tag_t(event.data).offset;
                int socket_thread_id = rx_tag_t(event.data).tid;

                char* socket_buffer_ptr = socket_buffer_[socket_thread_id]
                    + (long long)offset_in_current_buffer * cfg->packet_length;
                struct Packet* pkt = (struct Packet*)socket_buffer_ptr;

                frame_count = pkt->frame_id % kNumStatsFrames;
                size_t frame_id = frame_count % TASK_BUFFER_FRAME_NUM;
                int symbol_id = pkt->symbol_id;

                update_rx_counters(frame_count, frame_id, symbol_id);
                if (config_->bigstation_mode) {
                    /* In BigStation, schedule FFT whenever a packet is RX */
                    if (cur_frame_id != frame_id) {
                        stats->master_set_tsc(
                            TsType::kProcessingStarted, frame_count);
                        cur_frame_id = frame_id;
                    }
                }

                fft_queue_arr[frame_id].push(fft_req_tag_t(event.data));
            } break;

            case EventType::kFFT: {
                if (event.num_offsets == 0) {
                    handle_event_fft(event.data);
                } else {
                    for (int i = 0; i < event.num_offsets; i++) {
                        handle_event_fft(event.offsets[i]);
                    }
                }
            } break;

            case EventType::kRC: {
                int frame_id = event.data;
                stats->master_set_tsc(TsType::kRCDone, rc_stats_.frame_count);
                print_per_frame_done(PRINT_RC, rc_stats_.frame_count, frame_id);
                fft_stats_.symbol_cal_count[frame_id] = 0;
                rc_stats_.update_frame_count();
                rc_stats_.last_frame = frame_id;
            } break;

            case EventType::kZF: {
                int offset = event.data;
                int frame_id = offset / zf_stats_.max_symbol_count;

                print_per_task_done(
                    PRINT_ZF, frame_id, 0, zf_stats_.symbol_count[frame_id]);
                if (zf_stats_.last_symbol(frame_id)) {
                    stats->master_set_tsc(
                        TsType::kZFDone, zf_stats_.frame_count);
                    zf_stats_.coded_frame = frame_id;
                    print_per_frame_done(
                        PRINT_ZF, zf_stats_.frame_count, frame_id);
                    zf_stats_.update_frame_count();

                    /* If all the data in a frame has arrived when ZF is done */
                    schedule_demul_task(
                        frame_id, 0, fft_stats_.max_symbol_data_count);
                    if (config_->dl_data_symbol_num_perframe > 0) {
                        /* If downlink data transmission is enabled, schedule
                         * downlink encode/modulation for the first data
                         * symbol */
                        int total_data_symbol_id
                            = frame_id * cfg->data_symbol_num_perframe
                            + cfg->dl_data_symbol_start;
#if USE_LDPC
                        schedule_task_set(EventType::kEncode,
                            config_->LDPC_config.nblocksInSymbol * cfg->UE_NUM,
                            total_data_symbol_id, encode_queue_, ptok_encode);
#else
                        schedule_task_set(EventType::kPrecode,
                            config_->demul_block_num, total_data_symbol_id,
                            precode_queue_, *ptok_precode);
#endif
                    }
                }
            } break;

            case EventType::kDemul: {
                int offset = event.data;
                int block_size = config_->demul_block_size;
                int block_num = demul_stats_.max_task_count;
                int sc_id = offset % block_num * block_size;
                int total_data_symbol_id = offset / block_num;
                int frame_id
                    = total_data_symbol_id / cfg->ul_data_symbol_num_perframe;
                int data_symbol_id
                    = total_data_symbol_id % cfg->ul_data_symbol_num_perframe;

                print_per_task_done(
                    PRINT_DEMUL, frame_id, data_symbol_id, sc_id);
                /* If this symbol is ready */
                if (demul_stats_.last_task(frame_id, data_symbol_id)) {
                    max_equaled_frame = frame_id;
#ifdef USE_LDPC
                    consumer_decode.schedule_task_set(total_data_symbol_id);
#endif
                    print_per_symbol_done(PRINT_DEMUL, demul_stats_.frame_count,
                        frame_id, data_symbol_id);
                    if (demul_stats_.last_symbol(frame_id)) {
                        /* Schedule fft for the next frame if there are delayed
                         * fft tasks */
#ifndef USE_LDPC
                        cur_frame_id = (frame_id + 1) % TASK_BUFFER_FRAME_NUM;
                        frame_count = demul_stats_.frame_count + 1;
                        stats->update_stats_in_functions_uplink(
                            demul_stats_.frame_count);
                        if (stats->last_frame_id == config_->tx_frame_num - 1)
                            goto finish;
#endif
                        stats->master_set_tsc(
                            TsType::kDemulDone, demul_stats_.frame_count);
                        print_per_frame_done(
                            PRINT_DEMUL, demul_stats_.frame_count, frame_id);

                        demul_stats_.update_frame_count();
                    }

                    demul_count++;
                    if (demul_count == demul_stats_.max_symbol_count * 9000) {
                        demul_count = 0;
                        double diff = get_time_us() - demul_begin;
                        int samples_num_per_UE = cfg->OFDM_DATA_NUM
                            * demul_stats_.max_symbol_count * 1000;
                        printf(
                            "Frame %d: RX %d samples (per-client) from %zu "
                            "clients in %f secs, throughtput %f bps per-client "
                            "(16QAM), current task queue length %zu\n",
                            demul_stats_.frame_count, samples_num_per_UE,
                            cfg->UE_NUM, diff,
                            samples_num_per_UE * log2(16.0f) / diff,
                            fft_queue_.size_approx());
                        demul_begin = get_time_us();
                    }
                }
            } break;

#ifdef USE_LDPC
            case EventType::kDecode: {
                int offset = event.data;
                int num_code_blocks = decode_stats_.max_task_count;
                int total_data_symbol_id = offset / num_code_blocks;
                int frame_id
                    = total_data_symbol_id / cfg->ul_data_symbol_num_perframe;
                int data_symbol_id
                    = total_data_symbol_id % cfg->ul_data_symbol_num_perframe;

                if (decode_stats_.last_task(frame_id, data_symbol_id)) {
                    print_per_symbol_done(PRINT_DECODE,
                        decode_stats_.frame_count, frame_id, data_symbol_id);
                    if (decode_stats_.last_symbol(frame_id)) {
                        cur_frame_id = (frame_id + 1) % TASK_BUFFER_FRAME_NUM;
                        frame_count = decode_stats_.frame_count + 1;
                        stats->master_set_tsc(
                            TsType::kDecodeDone, decode_stats_.frame_count);
                        print_per_frame_done(
                            PRINT_DECODE, decode_stats_.frame_count, frame_id);
                        stats->update_stats_in_functions_uplink(
                            decode_stats_.frame_count);
                        if (stats->last_frame_id == config_->tx_frame_num - 1)
                            goto finish;
                        decode_stats_.update_frame_count();
                    }
                }
            } break;

            case EventType::kEncode: {
                int offset = event.data;
                int num_code_blocks = encode_stats_.max_task_count;
                int total_data_symbol_id = offset / num_code_blocks;
                int frame_id
                    = total_data_symbol_id / cfg->data_symbol_num_perframe;
                int data_symbol_id
                    = total_data_symbol_id % cfg->data_symbol_num_perframe;

                if (encode_stats_.last_task(frame_id, data_symbol_id)) {
                    consumer_precode.schedule_task_set(total_data_symbol_id);
                    print_per_symbol_done(PRINT_ENCODE,
                        encode_stats_.frame_count, frame_id, data_symbol_id);
                    if (encode_stats_.last_symbol(frame_id)) {
                        stats->master_set_tsc(
                            TsType::kEncodeDone, encode_stats_.frame_count);
                        print_per_frame_done(
                            PRINT_ENCODE, encode_stats_.frame_count, frame_id);
                        encode_stats_.update_frame_count();
                    }
                }
            } break;
#endif /* defined(USE_LDPC) */

            case EventType::kPrecode: {
                /* Precoding is done, schedule ifft */
                int offset = event.data;
                int block_size = config_->demul_block_size;
                int block_num = precode_stats_.max_task_count;
                int sc_id = offset % block_num * block_size;
                int total_data_symbol_id = offset / block_num;
                int data_symbol_id
                    = total_data_symbol_id % cfg->data_symbol_num_perframe;
                int frame_id
                    = total_data_symbol_id / cfg->data_symbol_num_perframe;

                print_per_task_done(
                    PRINT_PRECODE, frame_id, data_symbol_id, sc_id);
                if (precode_stats_.last_task(frame_id, data_symbol_id)) {
                    // TODO: offset below should have a descriptive name
                    int offset = precode_stats_.frame_count
                            * cfg->data_symbol_num_perframe
                        + data_symbol_id;
                    schedule_task_set(EventType::kIFFT, cfg->BS_ANT_NUM, offset,
                        ifft_queue_, *ptok_ifft);
                    if (data_symbol_id < (int)cfg->dl_data_symbol_end - 1) {
#ifdef USE_LDPC
                        schedule_task_set(EventType::kEncode,
                            config_->LDPC_config.nblocksInSymbol * cfg->UE_NUM,
                            total_data_symbol_id + 1, encode_queue_,
                            ptok_encode);
#else
                        schedule_task_set(EventType::kPrecode,
                            config_->demul_block_num, total_data_symbol_id + 1,
                            precode_queue_, *ptok_precode);
#endif
                    }

                    print_per_symbol_done(PRINT_PRECODE,
                        precode_stats_.frame_count, frame_id, data_symbol_id);
                    if (precode_stats_.last_symbol(frame_id)) {
                        stats->master_set_tsc(
                            TsType::kPrecodeDone, precode_stats_.frame_count);
                        print_per_frame_done(PRINT_PRECODE,
                            precode_stats_.frame_count, frame_id);
                        precode_stats_.update_frame_count();
                    }
                }
            } break;

            case EventType::kIFFT: {
                /* IFFT is done, schedule data transmission */
                int offset = event.data;
                int ant_id = offset % cfg->BS_ANT_NUM;
                int total_data_symbol_id = offset / cfg->BS_ANT_NUM;
                int frame_id = total_data_symbol_id
                    / cfg->data_symbol_num_perframe % TASK_BUFFER_FRAME_NUM;
                int data_symbol_id
                    = total_data_symbol_id % cfg->data_symbol_num_perframe;
                int ptok_id = ant_id % cfg->socket_thread_num; /* RX */

                Event_data tx_event(EventType::kPacketTX, offset);
                try_enqueue_fallback(
                    tx_queue_, *tx_ptoks_ptr[ptok_id], tx_event);

                print_per_task_done(
                    PRINT_IFFT, frame_id, data_symbol_id, ant_id);

                if (ifft_stats_.last_task(frame_id, data_symbol_id)) {
                    if (ifft_stats_.last_symbol(frame_id)) {
                        cur_frame_id = (frame_id + 1) % TASK_BUFFER_FRAME_NUM;
                        frame_count = ifft_stats_.frame_count + 1;
                        stats->master_set_tsc(
                            TsType::kIFFTDone, ifft_stats_.frame_count);
                        print_per_frame_done(
                            PRINT_IFFT, ifft_stats_.frame_count, frame_id);
                        ifft_stats_.update_frame_count();
                    }
                }
            } break;

            case EventType::kPacketTX: {
                /* Data is sent */
                int offset = event.data;
                int ant_id = offset % cfg->BS_ANT_NUM;
                int total_data_symbol_id = offset / cfg->BS_ANT_NUM;
                int frame_id
                    = total_data_symbol_id / cfg->data_symbol_num_perframe;
                int data_symbol_id
                    = total_data_symbol_id % cfg->data_symbol_num_perframe;
                // printf("In main thread: tx finished for ",
                //     "frame %d symbol %d ant %d\n",
                //     frame_id, data_symbol_id, ant_id);
                frame_id = frame_id % TASK_BUFFER_FRAME_NUM;

                print_per_task_done(PRINT_TX, frame_id, data_symbol_id, ant_id);
                if (tx_stats_.last_task(frame_id, data_symbol_id)) {
                    print_per_symbol_done(PRINT_TX, tx_stats_.frame_count,
                        frame_id, data_symbol_id);
                    /* If tx of the first symbol is done */
                    if (data_symbol_id == (int)cfg->dl_data_symbol_start) {
                        stats->master_set_tsc(
                            TsType::kTXProcessedFirst, tx_stats_.frame_count);
                        print_per_frame_done(
                            PRINT_TX_FIRST, tx_stats_.frame_count, frame_id);
                    }
                    if (tx_stats_.last_symbol(frame_id)) {
                        stats->master_set_tsc(
                            TsType::kTXDone, tx_stats_.frame_count);
                        print_per_frame_done(
                            PRINT_TX, tx_stats_.frame_count, frame_id);
                        stats->update_stats_in_functions_downlink(
                            tx_stats_.frame_count);
                        if (stats->last_frame_id == config_->tx_frame_num - 1)
                            goto finish;
                        tx_stats_.update_frame_count();
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
                            tx_queue_.size_approx());
                        tx_begin = get_time_us();
                    }
                }
            } break;
            default:
                printf("Wrong event type in message queue!");
                exit(0);
            } /* End of switch */

            /* Schedule multiple fft tasks as one event */
            if (fft_queue_arr[cur_frame_id].size() >= config_->fft_block_size) {
                size_t fft_block_num = fft_queue_arr[cur_frame_id].size()
                    / config_->fft_block_size;

                for (size_t i = 0; i < fft_block_num; i++) {
                    Event_data do_fft_task;
                    do_fft_task.num_offsets = config_->fft_block_size;
                    do_fft_task.event_type = EventType::kFFT;

                    for (size_t j = 0; j < config_->fft_block_size; j++) {
                        do_fft_task.offsets[j]
                            = fft_queue_arr[cur_frame_id].front()._tag;
                        fft_queue_arr[cur_frame_id].pop();

                        if (!config_->bigstation_mode) {
                            if (fft_created_count++ == 0) {
                                stats->master_set_tsc(
                                    TsType::kProcessingStarted, frame_count);
                            } else if (fft_created_count
                                == rx_stats_.max_task_count) {
                                fft_created_count = 0;
                            }
                        }
                    }
                    try_enqueue_fallback(fft_queue_, *ptok_fft, do_fft_task);
                }
            }
        } /* End of for */
    } /* End of while */

finish:

    printf("Millipede: printing stats and saving to file\n");
    stats->print_summary();
    stats->save_to_file();
#ifdef USE_LDPC
    save_decode_data_to_file(stats->last_frame_id);
#else
    save_demul_data_to_file(stats->last_frame_id);
#endif

    save_tx_data_to_file(stats->last_frame_id);
    this->stop();
}

void Millipede::handle_event_fft(int tag)
{
    int frame_id = fft_resp_tag_t(tag).frame_id;
    int symbol_id = fft_resp_tag_t(tag).symbol_id;

    if (fft_stats_.last_task(frame_id, symbol_id)) {
        if (config_->isPilot(frame_id, symbol_id)) {
            print_per_symbol_done(
                PRINT_FFT_PILOTS, fft_stats_.frame_count, frame_id, symbol_id);
            if (!config_->downlink_mode
                || (config_->downlink_mode && !config_->recipCalEn)
                || (config_->downlink_mode && config_->recipCalEn
                       && rc_stats_.last_frame == frame_id)) {
                /* If CSI of all UEs is ready, schedule ZF/prediction */
                if (fft_stats_.last_symbol(frame_id)) {
                    stats->master_set_tsc(
                        TsType::kFFTDone, fft_stats_.frame_count);
                    print_per_frame_done(
                        PRINT_FFT_PILOTS, fft_stats_.frame_count, frame_id);
                    fft_stats_.update_frame_count();
                    schedule_task_set(EventType::kZF, config_->zf_block_num,
                        frame_id, zf_queue_, *ptok_zf);
                }
            }
        } else if (config_->isUplink(frame_id, symbol_id)) {
            int data_symbol_id = config_->getUlSFIndex(frame_id, symbol_id);
            fft_stats_.cur_frame_for_symbol[data_symbol_id] = frame_id;
            print_per_symbol_done(PRINT_FFT_DATA, fft_stats_.frame_count - 1,
                frame_id, symbol_id);
            /* If precoder exist, schedule demodulation */
            if (zf_stats_.coded_frame == frame_id)
                schedule_demul_task(
                    frame_id, data_symbol_id, data_symbol_id + 1);
        } else if (config_->isCalDlPilot(frame_id, symbol_id)
            || config_->isCalUlPilot(frame_id, symbol_id)) {
            print_per_symbol_done(
                PRINT_FFT_CAL, fft_stats_.frame_count - 1, frame_id, symbol_id);
            if (++fft_stats_.symbol_cal_count[frame_id]
                == fft_stats_.max_symbol_cal_count) {
                print_per_frame_done(
                    PRINT_FFT_CAL, fft_stats_.frame_count - 1, frame_id);
                // TODO: rc_stats_.max_task_count appears uninitalized
                schedule_task_set(EventType::kRC, rc_stats_.max_task_count,
                    frame_id, rc_queue_, *ptok_rc);
            }
        }
    }
}

static void pin_worker(ThreadType thread_type, int tid, Config* config_)
{
    int socket_rx_thread_num = config_->socket_thread_num;
    pin_to_core_with_offset(
        thread_type, config_->core_offset + socket_rx_thread_num + 1, tid);
}

void* Millipede::worker(int tid)
{
    pin_worker(ThreadType::kWorker, tid, config_);

    /* Initialize operators */
    auto computeFFT = new DoFFT(config_, tid, freq_ghz, fft_queue_,
        complete_task_queue_, worker_ptoks_ptr[tid], socket_buffer_,
        socket_buffer_status_, data_buffer_, csi_buffer_, calib_buffer_, stats);

    auto computeIFFT
        = new DoIFFT(config_, tid, freq_ghz, ifft_queue_, complete_task_queue_,
            worker_ptoks_ptr[tid], dl_ifft_buffer_, dl_socket_buffer_, stats);

    auto computeZF = new DoZF(config_, tid, freq_ghz, zf_queue_,
        complete_task_queue_, worker_ptoks_ptr[tid], csi_buffer_, recip_buffer_,
        precoder_buffer_, dl_precoder_buffer_, stats);

    auto computeDemul = new DoDemul(config_, tid, freq_ghz, demul_queue_,
        complete_task_queue_, worker_ptoks_ptr[tid], data_buffer_,
        precoder_buffer_, equal_buffer_, demod_hard_buffer_, demod_soft_buffer_,
        stats);

    auto computePrecode = new DoPrecode(config_, tid, freq_ghz, precode_queue_,
        complete_task_queue_, worker_ptoks_ptr[tid], dl_precoder_buffer_,
        dl_ifft_buffer_,
#ifdef USE_LDPC
        dl_encoded_buffer_,
#else
        config_->dl_IQ_data,
#endif
        stats);

#ifdef USE_LDPC
    auto* computeEncoding = new DoEncode(config_, tid, freq_ghz, encode_queue_,
        complete_task_queue_, worker_ptoks_ptr[tid], config_->dl_IQ_data,
        dl_encoded_buffer_, stats);
    auto* computeDecoding = new DoDecode(config_, tid, freq_ghz, decode_queue_,
        complete_task_queue_, worker_ptoks_ptr[tid], demod_soft_buffer_,
        decoded_buffer_, stats);
#endif
    auto* computeReciprocity = new Reciprocity(config_, tid, freq_ghz,
        rc_queue_, complete_task_queue_, worker_ptoks_ptr[tid], calib_buffer_,
        recip_buffer_, stats);

    Doer* computers[] = {
        computeIFFT,
        computePrecode,
#ifdef USE_LDPC
        computeEncoding,
#endif
        computeZF,
        computeReciprocity,
        computeFFT,
        computeDemul,
#ifdef USE_LDPC
        computeDecoding,
#endif
    };

#define NITEMS(a) (sizeof(a) / sizeof(*a))

    while (true) {
        for (size_t i = 0; i < NITEMS(computers); i++) {
            if (computers[i]->try_launch())
                break;
        }
    }
}

void* Millipede::worker_fft(int tid)
{
    pin_worker(ThreadType::kWorkerFFT, tid, config_);

    /* Initialize IFFT operator */
    auto computeFFT = new DoFFT(config_, tid, freq_ghz, fft_queue_,
        complete_task_queue_, worker_ptoks_ptr[tid], socket_buffer_,
        socket_buffer_status_, data_buffer_, csi_buffer_, calib_buffer_, stats);
    auto computeIFFT
        = new DoIFFT(config_, tid, freq_ghz, ifft_queue_, complete_task_queue_,
            worker_ptoks_ptr[tid], dl_ifft_buffer_, dl_socket_buffer_, stats);

    while (true) {
        if (computeFFT->try_launch()) {
        } else if (config_->dl_data_symbol_num_perframe > 0
            && computeIFFT->try_launch()) {
        }
    }
}

void* Millipede::worker_zf(int tid)
{
    pin_worker(ThreadType::kWorkerZF, tid, config_);

    /* Initialize ZF operator */
    auto computeZF = new DoZF(config_, tid, freq_ghz, zf_queue_,
        complete_task_queue_, worker_ptoks_ptr[tid], csi_buffer_, recip_buffer_,
        precoder_buffer_, dl_precoder_buffer_, stats);

    while (true) {
        computeZF->try_launch();
    }
}

void* Millipede::worker_demul(int tid)
{
    pin_worker(ThreadType::kWorkerDemul, tid, config_);

    auto computeDemul = new DoDemul(config_, tid, freq_ghz, demul_queue_,
        complete_task_queue_, worker_ptoks_ptr[tid], data_buffer_,
        precoder_buffer_, equal_buffer_, demod_hard_buffer_, demod_soft_buffer_,
        stats);

    /* Initialize Precode operator */
    auto computePrecode = new DoPrecode(config_, tid, freq_ghz, precode_queue_,
        complete_task_queue_, worker_ptoks_ptr[tid], dl_precoder_buffer_,
        dl_ifft_buffer_,
#ifdef USE_LDPC
        dl_encoded_buffer_,
#else
        config_->dl_IQ_data,
#endif
        stats);

    while (true) {
        if (config_->dl_data_symbol_num_perframe > 0) {
            computePrecode->try_launch();
        } else {
            computeDemul->try_launch();
        }
    }
}

void Millipede::create_threads(
    void* (*worker)(void*), int tid_start, int tid_end)
{
    int ret;
    for (int i = tid_start; i < tid_end; i++) {
        auto context = new EventHandlerContext<Millipede>;
        context->obj_ptr = this;
        context->id = i;
        ret = pthread_create(&task_threads[i], NULL, worker, context);
        if (ret != 0) {
            perror("task thread create failed");
            exit(0);
        }
    }
}

void Millipede::schedule_demul_task(
    int frame_id, int start_symbol_id, int end_symbol_id)
{
    int data_symbol_num_perframe = config_->ul_data_symbol_num_perframe;
    for (int data_symbol_id = start_symbol_id; data_symbol_id < end_symbol_id;
         data_symbol_id++) {
        if (fft_stats_.cur_frame_for_symbol[data_symbol_id] == frame_id) {
            /* Schedule demodulation task for subcarrier blocks */
            int total_data_symbol_id
                = frame_id * data_symbol_num_perframe + data_symbol_id;
            schedule_task_set(EventType::kDemul, config_->demul_block_num,
                total_data_symbol_id, demul_queue_, *ptok_demul);
#if DEBUG_PRINT_PER_SUBFRAME_ENTER_QUEUE
            printf("Main thread: created Demodulation task for frame: %d, "
                   "start symbol: %d, current symbol: %d, in %.2f\n",
                frame_id, start_symbol_id, data_symbol_id,
                stats->master_get_us_since(
                    TsType::kPilotRX, demul_stats_.frame_count));
#endif
        }
    }
}

void Millipede::update_rx_counters(int frame_count, int frame_id, int symbol_id)
{
    if (config_->isPilot(frame_count, symbol_id)) {
        if (++rx_stats_.task_pilot_count[frame_id]
            == rx_stats_.max_task_pilot_count) {
            rx_stats_.task_pilot_count[frame_id] = 0;
            stats->master_set_tsc(TsType::kPilotAllRX, frame_count);
            print_per_frame_done(PRINT_RX_PILOTS, frame_count, frame_id);
        }
    }
    if (rx_stats_.task_count[frame_id]++ == 0) {
        stats->master_set_tsc(TsType::kPilotRX, frame_count);
#if DEBUG_PRINT_PER_FRAME_START
        int prev_frame_id
            = (frame_count + TASK_BUFFER_FRAME_NUM - 1) % TASK_BUFFER_FRAME_NUM;
        printf("Main thread: data received from frame %d, symbol %d, in %.2f "
               "us. RX in prev frame: %d\n",
            frame_count, symbol_id,
            stats->master_get_delta_us(
                TsType::kPilotRX, frame_count, frame_count - 1),
            rx_stats_.task_count[prev_frame_id]);
#endif
    } else if (rx_stats_.task_count[frame_id] == rx_stats_.max_task_count) {
        stats->master_set_tsc(TsType::kRXDone, frame_count);
        print_per_frame_done(PRINT_RX, frame_count, frame_id);
        rx_stats_.task_count[frame_id] = 0;
    }
}

void Millipede::print_per_frame_done(
    int task_type, int frame_count, int frame_id)
{
    int dl_data_symbol_num_perframe = config_->dl_data_symbol_num_perframe;
    int ul_data_symbol_num_perframe = config_->ul_data_symbol_num_perframe;
#if DEBUG_PRINT_PER_FRAME_DONE
    switch (task_type) {
    case (PRINT_RX): {
        int prev_frame_count = (frame_count - 1) % TASK_BUFFER_FRAME_NUM;
        printf("Main thread: received all packets in frame: %d, frame buffer: "
               "%d in %.2f us, demul: %d done, rx in prev frame: %d\n",
            frame_count, frame_id,
            stats->master_get_delta_us(
                TsType::kRXDone, TsType::kPilotRX, frame_count),
            demul_stats_.symbol_count[frame_id],
            rx_stats_.task_count[prev_frame_count]);
    } break;
    case (PRINT_RX_PILOTS):
        printf("Main thread: received all pilots in frame: %d, frame buffer: "
               "%d in %.2f us\n",
            frame_count, frame_id,
            stats->master_get_delta_us(
                TsType::kPilotAllRX, TsType::kPilotRX, frame_count));
        break;
    case (PRINT_FFT_PILOTS):
        printf("Main thread: pilot frame: %d, %d, finished FFT for all pilot "
               "symbols in %.2f us, pilot all received: %.2f\n",
            frame_count, frame_id,
            stats->master_get_delta_us(
                TsType::kFFTDone, TsType::kPilotRX, frame_count),
            stats->master_get_delta_us(
                TsType::kPilotAllRX, TsType::kPilotRX, frame_count));
        break;
    case (PRINT_FFT_DATA):
        printf("Main thread: data frame: %d, %d, finished FFT for all data "
               "symbols in %.2f us\n",
            frame_count, frame_id,
            stats->master_get_us_since(TsType::kPilotRX, frame_count));
        break;
    case (PRINT_FFT_CAL):
        printf("Main thread: cal frame: %d, %d, finished FFT for all cal "
               "symbols in %.2f us\n",
            frame_count, frame_id,
            stats->master_get_us_since(TsType::kPilotRX, frame_count));
        break;
    case (PRINT_ZF):
        printf("Main thread: ZF done frame: %d, %d in %.2f us since pilot FFT "
               "done, total: %.2f us, FFT queue %zu\n",
            frame_count, frame_id,
            stats->master_get_delta_us(
                TsType::kZFDone, TsType::kFFTDone, frame_count),
            stats->master_get_delta_us(
                TsType::kZFDone, TsType::kPilotRX, frame_count),
            fft_queue_.size_approx());
        break;
    case (PRINT_DEMUL):
        printf("Main thread: Demodulation done frame: %d, %d (%d UL symbols) "
               "in %.2f us since ZF done, total %.2f us\n",
            frame_count, frame_id, ul_data_symbol_num_perframe,
            stats->master_get_delta_us(
                TsType::kDemulDone, TsType::kZFDone, frame_count),
            stats->master_get_delta_us(
                TsType::kDemulDone, TsType::kPilotRX, frame_count));
        break;
    case (PRINT_DECODE):
        printf("Main thread: Decoding done frame: %d, %d (%d UL symbols) in "
               "%.2f us since ZF done, total %.2f us\n",
            frame_count, frame_id, ul_data_symbol_num_perframe,
            stats->master_get_delta_us(
                TsType::kDecodeDone, TsType::kZFDone, frame_count),
            stats->master_get_delta_us(
                TsType::kDecodeDone, TsType::kPilotRX, frame_count));
        break;
    case (PRINT_ENCODE):
        printf("Main thread: Encoding done frame: %d, %d in %.2f us since ZF "
               "done, total %.2f us\n",
            frame_count, frame_id,
            stats->master_get_delta_us(
                TsType::kEncodeDone, TsType::kZFDone, frame_count),
            stats->master_get_delta_us(
                TsType::kEncodeDone, TsType::kPilotRX, frame_count));
        break;
    case (PRINT_PRECODE):
        printf("Main thread: Precoding done frame: %d, %d in %.2f us since ZF "
               "done, total: %.2f us\n",
            frame_count, frame_id,
            stats->master_get_delta_us(
                TsType::kPrecodeDone, TsType::kZFDone, frame_count),
            stats->master_get_delta_us(
                TsType::kPrecodeDone, TsType::kPilotRX, frame_count));
        break;
    case (PRINT_RC):
        printf("Main thread: Reciprocity Calculation done frame: %d, %d in "
               "%.2f us since pilot FFT done, total: %.2f us\n",
            frame_count, frame_id,
            stats->master_get_delta_us(
                TsType::kRCDone, TsType::kFFTDone, frame_count),
            stats->master_get_delta_us(
                TsType::kRCDone, TsType::kPilotRX, frame_count));
        break;
    case (PRINT_IFFT):
        printf("Main thread: IFFT done frame: %d, %d in %.2f us since precode "
               "done, total: %.2f us\n",
            frame_count, frame_id,
            stats->master_get_delta_us(
                TsType::kIFFTDone, TsType::kPrecodeDone, frame_count),
            stats->master_get_delta_us(
                TsType::kIFFTDone, TsType::kPilotRX, frame_count));
        break;
    case (PRINT_TX_FIRST):
        printf("Main thread: TX of first symbol done frame: %d, %d in %.2f "
               "us since ZF done, total: %.2f us\n",
            frame_count, frame_id,
            stats->master_get_delta_us(
                TsType::kTXProcessedFirst, TsType::kZFDone, frame_count),
            stats->master_get_delta_us(
                TsType::kTXProcessedFirst, TsType::kPilotRX, frame_count));
        break;
    case (PRINT_TX):
        printf("Main thread: TX done frame: %d %d (%d DL symbols) in %.2f us "
               "since ZF done, total: %.2f us\n",
            frame_count, frame_id, dl_data_symbol_num_perframe,
            stats->master_get_delta_us(
                TsType::kTXDone, TsType::kZFDone, frame_count),
            stats->master_get_delta_us(
                TsType::kTXDone, TsType::kPilotRX, frame_count));
        break;
    default:
        printf("Wrong task type in frame done print!");
    }
#endif
}

void Millipede::print_per_symbol_done(UNUSED int task_type,
    UNUSED int frame_count, UNUSED int frame_id, UNUSED int symbol_id)
{
#if DEBUG_PRINT_PER_SUBFRAME_DONE
    switch (task_type) {
    case (PRINT_FFT_PILOTS):
        printf("Main thread: pilot FFT done frame: %d, %d, symbol: %d, num "
               "symbols done: %d\n",
            frame_count, frame_id, symbol_id,
            fft_stats_.symbol_count[frame_id]);
        break;
    case (PRINT_FFT_DATA):
        printf(
            "Main thread: data FFT done frame %d, %d, symbol %d, precoder "
            "status: %d, fft queue: %zu, zf queue: %zu, demul queue: %zu, in "
            "%.2f\n",
            frame_count, frame_id, symbol_id, zf_stats_.coded_frame == frame_id,
            fft_queue_.size_approx(), zf_queue_.size_approx(),
            demul_queue_.size_approx(),
            stats->master_get_us_since(TsType::kPilotRX, frame_count));
        break;
    case (PRINT_RC):
        printf("Main thread: cal symbol FFT done frame: %d, %d, symbol: %d, "
               "num symbols done: %d\n",
            frame_count, frame_id, symbol_id,
            fft_stats_.symbol_cal_count[frame_id]);
        break;
    case (PRINT_DEMUL):
        printf("Main thread: Demodulation done frame %d %d, symbol: %d, num "
               "symbols done: %d in %.2f\n",
            frame_count, frame_id, symbol_id,
            demul_stats_.symbol_count[frame_id],
            stats->master_get_us_since(TsType::kPilotRX, frame_count));
        break;
#ifdef USE_LDPC
    case (PRINT_DECODE):
        printf("Main thread: Decoding done frame %d %d, symbol: %d, num "
               "symbols done: %d\n",
            frame_count, frame_id, symbol_id,
            decode_stats_.symbol_count[frame_id]);
        break;
    case (PRINT_ENCODE):
        printf("Main thread: Encoding done frame %d %d, symbol: %d, num "
               "symbols done: %d\n",
            frame_count, frame_id, symbol_id,
            encode_stats_.symbol_count[frame_id]);
        break;
#endif
    case (PRINT_PRECODE):
        printf("Main thread: Precoding done frame: %d %d, symbol: %d in %.2f "
               "us\n",
            frame_count, frame_id, symbol_id,
            stats->master_get_us_since(TsType::kPilotRX, frame_count));
        break;
    case (PRINT_TX):
        printf("Main thread: TX done frame: %d %d, symbol: %d in %.2f us\n",
            frame_count, frame_id, symbol_id,
            stats->master_get_us_since(TsType::kPilotRX, frame_count));
        break;
    default:
        printf("Wrong task type in frame done print!");
    }
#endif
}

void Millipede::print_per_task_done(UNUSED int task_type, UNUSED int frame_id,
    UNUSED int symbol_id, UNUSED int ant_or_sc_id)
{
#if DEBUG_PRINT_PER_TASK_DONE
    switch (task_type) {
    case (PRINT_ZF):
        printf("Main thread: ZF done frame: %d, subcarrier %d\n", frame_id,
            ant_or_sc_id);
        break;
    case (PRINT_RC):
        printf("Main thread: RC done frame: %d, subcarrier %d\n", frame_id,
            ant_or_sc_id);
        break;
    case (PRINT_DEMUL):
        printf("Main thread: Demodulation done frame: %d, symbol: %d, sc: "
               "%d, num blocks done: %d\n",
            frame_id, symbol_id, ant_or_sc_id,
            demul_stats_.task_count[frame_id][symbol_id]);
        break;
#ifdef USE_LDPC
    case (PRINT_DECODE):
        printf("Main thread: Decoding done frame: %d, symbol: %d, sc: %d, "
               "num blocks done: %d\n",
            frame_id, symbol_id, ant_or_sc_id,
            decode_stats_.task_count[frame_id][symbol_id]);
        break;
#endif
    case (PRINT_PRECODE):
        printf("Main thread: Precoding done frame: %d, symbol: %d, "
               "subcarrier: %d, total SCs: %d\n",
            frame_id, symbol_id, ant_or_sc_id,
            precode_stats_.task_count[frame_id][symbol_id]);
        break;
    case (PRINT_IFFT):
        printf("Main thread: IFFT done frame: %d, symbol: %d, antenna: %d, "
               "total ants: %d\n",
            frame_id, symbol_id, ant_or_sc_id,
            ifft_stats_.task_count[frame_id][symbol_id]);
        break;
    case (PRINT_TX):
        printf("Main thread: TX done frame: %d, symbol: %d, antenna: %d, "
               "total packets: %d\n",
            frame_id, symbol_id, ant_or_sc_id,
            tx_stats_.task_count[frame_id][symbol_id]);
        break;
    default:
        printf("Wrong task type in frame done print!");
    }

#endif
}

void Millipede::initialize_queues()
{
    using mt_queue_t = moodycamel::ConcurrentQueue<Event_data>;

    int data_symbol_num_perframe = config_->data_symbol_num_perframe;
    message_queue_ = mt_queue_t(512 * data_symbol_num_perframe);
    complete_task_queue_ = mt_queue_t(512 * data_symbol_num_perframe * 4);

    fft_queue_ = mt_queue_t(512 * data_symbol_num_perframe * 4);
    ptok_fft = new moodycamel::ProducerToken(fft_queue_);

    zf_queue_ = mt_queue_t(512 * data_symbol_num_perframe * 4);
    ptok_zf = new moodycamel::ProducerToken(zf_queue_);

    rc_queue_ = mt_queue_t(512 * 2 * 4);
    ptok_rc = new moodycamel::ProducerToken(rc_queue_);

    demul_queue_ = mt_queue_t(512 * data_symbol_num_perframe * 4);
    ptok_demul = new moodycamel::ProducerToken(demul_queue_);

#ifdef USE_LDPC
    decode_queue_ = mt_queue_t(512 * data_symbol_num_perframe * 4);
    ptok_decode = moodycamel::ProducerToken(decode_queue_);
#endif

    ifft_queue_ = mt_queue_t(512 * data_symbol_num_perframe * 4);
    ptok_ifft = new moodycamel::ProducerToken(ifft_queue_);
#ifdef USE_LDPC
    encode_queue_ = mt_queue_t(512 * data_symbol_num_perframe * 4);
    ptok_encode = new moodycamel::ProducerToken(encode_queue_);
#endif
    precode_queue_ = mt_queue_t(512 * data_symbol_num_perframe * 4);
    ptok_precode = new moodycamel::ProducerToken(precode_queue_);
    tx_queue_ = mt_queue_t(512 * data_symbol_num_perframe * 4);

    rx_ptoks_ptr = (moodycamel::ProducerToken**)aligned_alloc(
        64, config_->socket_thread_num * sizeof(moodycamel::ProducerToken*));
    for (size_t i = 0; i < config_->socket_thread_num; i++)
        rx_ptoks_ptr[i] = new moodycamel::ProducerToken(message_queue_);

    tx_ptoks_ptr = (moodycamel::ProducerToken**)aligned_alloc(
        64, config_->socket_thread_num * sizeof(moodycamel::ProducerToken*));
    for (size_t i = 0; i < config_->socket_thread_num; i++)
        tx_ptoks_ptr[i] = new moodycamel::ProducerToken(tx_queue_);

    worker_ptoks_ptr = (moodycamel::ProducerToken**)aligned_alloc(
        64, config_->worker_thread_num * sizeof(moodycamel::ProducerToken*));
    for (size_t i = 0; i < config_->worker_thread_num; i++)
        worker_ptoks_ptr[i]
            = new moodycamel::ProducerToken(complete_task_queue_);
}

void Millipede::initialize_uplink_buffers()
{
    auto& cfg = config_;
    int TASK_BUFFER_SUBFRAME_NUM
        = cfg->ul_data_symbol_num_perframe * TASK_BUFFER_FRAME_NUM;

    alloc_buffer_1d(&task_threads, cfg->worker_thread_num, 64, 0);

    socket_buffer_status_size_
        = cfg->BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM * cfg->symbol_num_perframe;
    socket_buffer_size_
        = (long long)cfg->packet_length * socket_buffer_status_size_;

    printf("Millipede: Initializing uplink buffers: socket buffer size %lld, "
           "socket buffer status size %d\n",
        socket_buffer_size_, socket_buffer_status_size_);

    socket_buffer_.malloc(
        cfg->socket_thread_num /* RX */, socket_buffer_size_, 64);
    socket_buffer_status_.calloc(
        cfg->socket_thread_num /* RX */, socket_buffer_status_size_, 64);

    csi_buffer_.malloc(cfg->pilot_symbol_num_perframe * TASK_BUFFER_FRAME_NUM,
        cfg->BS_ANT_NUM * cfg->OFDM_DATA_NUM, 64);
    data_buffer_.malloc(
        TASK_BUFFER_SUBFRAME_NUM, cfg->BS_ANT_NUM * cfg->OFDM_DATA_NUM, 64);
    precoder_buffer_.malloc(cfg->OFDM_DATA_NUM * TASK_BUFFER_FRAME_NUM,
        cfg->BS_ANT_NUM * cfg->UE_NUM, 64);

    equal_buffer_.malloc(
        TASK_BUFFER_SUBFRAME_NUM, cfg->OFDM_DATA_NUM * cfg->UE_NUM, 64);
    demod_hard_buffer_.malloc(
        TASK_BUFFER_SUBFRAME_NUM, cfg->OFDM_DATA_NUM * cfg->UE_NUM, 64);
    size_t mod_type = config_->mod_type;
    demod_soft_buffer_.malloc(TASK_BUFFER_SUBFRAME_NUM,
        mod_type * cfg->OFDM_DATA_NUM * cfg->UE_NUM, 64);
    size_t num_decoded_bytes
        = (cfg->LDPC_config.cbLen + 7) >> 3 * cfg->LDPC_config.nblocksInSymbol;
    decoded_buffer_.calloc(
        TASK_BUFFER_SUBFRAME_NUM, num_decoded_bytes * cfg->UE_NUM, 64);

    int max_packet_num_per_frame = cfg->BS_ANT_NUM
        * (cfg->pilot_symbol_num_perframe + cfg->ul_data_symbol_num_perframe);
    rx_stats_.max_task_count = max_packet_num_per_frame;
    rx_stats_.max_task_pilot_count
        = cfg->BS_ANT_NUM * cfg->pilot_symbol_num_perframe;
    alloc_buffer_1d(&(rx_stats_.task_count), TASK_BUFFER_FRAME_NUM, 64, 1);
    alloc_buffer_1d(
        &(rx_stats_.task_pilot_count), TASK_BUFFER_FRAME_NUM, 64, 1);
    fft_created_count = 0;
    fft_stats_.init(cfg->BS_ANT_NUM, cfg->pilot_symbol_num_perframe,
        TASK_BUFFER_FRAME_NUM, cfg->symbol_num_perframe, 64);
    fft_stats_.max_symbol_data_count = cfg->ul_data_symbol_num_perframe;
    alloc_buffer_1d(
        &(fft_stats_.symbol_cal_count), TASK_BUFFER_FRAME_NUM, 64, 1);
    fft_stats_.max_symbol_cal_count = 2;
    alloc_buffer_1d(&fft_stats_.cur_frame_for_symbol,
        cfg->ul_data_symbol_num_perframe, 64, 0);
    for (size_t i = 0; i < cfg->ul_data_symbol_num_perframe; ++i)
        fft_stats_.cur_frame_for_symbol[i] = -1;

    zf_stats_.init(config_->zf_block_num, TASK_BUFFER_FRAME_NUM, 1);

    demul_stats_.init(config_->demul_block_num,
        cfg->ul_data_symbol_num_perframe, TASK_BUFFER_FRAME_NUM,
        cfg->data_symbol_num_perframe, 64);

#ifdef USE_LDPC
    decode_stats_.init(config_->LDPC_config.nblocksInSymbol * cfg->UE_NUM,
        cfg->ul_data_symbol_num_perframe, TASK_BUFFER_FRAME_NUM,
        cfg->data_symbol_num_perframe, 64);
#endif
}

void Millipede::initialize_downlink_buffers()
{
    auto& cfg = config_;
    int TASK_BUFFER_SUBFRAME_NUM
        = cfg->data_symbol_num_perframe * TASK_BUFFER_FRAME_NUM;

    dl_socket_buffer_status_size_ = cfg->BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM
        * cfg->data_symbol_num_perframe;
    dl_socket_buffer_size_
        = (long long)cfg->packet_length * dl_socket_buffer_status_size_;
    alloc_buffer_1d(&dl_socket_buffer_, dl_socket_buffer_size_, 64, 0);
    alloc_buffer_1d(
        &dl_socket_buffer_status_, dl_socket_buffer_status_size_, 64, 1);
    dl_ifft_buffer_.calloc(
        cfg->BS_ANT_NUM * TASK_BUFFER_SUBFRAME_NUM, cfg->OFDM_CA_NUM, 64);
    dl_precoder_buffer_.malloc(cfg->OFDM_DATA_NUM * TASK_BUFFER_FRAME_NUM,
        cfg->UE_NUM * cfg->BS_ANT_NUM, 64);
    recip_buffer_.malloc(
        TASK_BUFFER_FRAME_NUM, cfg->OFDM_DATA_NUM * cfg->BS_ANT_NUM, 64);
    calib_buffer_.malloc(
        TASK_BUFFER_FRAME_NUM, cfg->OFDM_DATA_NUM * cfg->BS_ANT_NUM, 64);
#ifdef USE_LDPC
    dl_encoded_buffer_.malloc(
        TASK_BUFFER_SUBFRAME_NUM, cfg->OFDM_DATA_NUM * cfg->UE_NUM, 64);

    encode_stats_.init(config_->LDPC_config.nblocksInSymbol * cfg->UE_NUM,
        cfg->dl_data_symbol_num_perframe, TASK_BUFFER_FRAME_NUM,
        cfg->data_symbol_num_perframe, 64);
#endif
    precode_stats_.init(config_->demul_block_num,
        cfg->dl_data_symbol_num_perframe, TASK_BUFFER_FRAME_NUM,
        cfg->data_symbol_num_perframe, 64);

    ifft_stats_.init(cfg->BS_ANT_NUM, cfg->dl_data_symbol_num_perframe,
        TASK_BUFFER_FRAME_NUM, cfg->data_symbol_num_perframe, 64);

    tx_stats_.init(cfg->BS_ANT_NUM, cfg->dl_data_symbol_num_perframe,
        TASK_BUFFER_FRAME_NUM, cfg->data_symbol_num_perframe, 64);
}

void Millipede::free_uplink_buffers()
{
    socket_buffer_.free();
    socket_buffer_status_.free();
    csi_buffer_.free();
    data_buffer_.free();
    precoder_buffer_.free();
    equal_buffer_.free();
    demod_hard_buffer_.free();
    demod_soft_buffer_.free();
    decoded_buffer_.free();

    free_buffer_1d(&(rx_stats_.task_count));
    free_buffer_1d(&(rx_stats_.task_pilot_count));
    fft_stats_.fini();
    free_buffer_1d(&fft_stats_.symbol_cal_count);
    free_buffer_1d(&fft_stats_.cur_frame_for_symbol);
    zf_stats_.fini();
    demul_stats_.fini();
#ifdef USE_LDPC
    decode_stats_.fini();
#endif
}

void Millipede::free_downlink_buffers()
{
    free_buffer_1d(&dl_socket_buffer_);
    free_buffer_1d(&dl_socket_buffer_status_);

    dl_ifft_buffer_.free();
    recip_buffer_.free();
    calib_buffer_.free();
    dl_precoder_buffer_.free();
    dl_encoded_buffer_.free();

#ifdef USE_LDPC
    encode_stats_.fini();
#endif
    precode_stats_.fini();
    ifft_stats_.fini();
    tx_stats_.fini();
}

void Millipede::save_demul_data_to_file(UNUSED int frame_id)
{
#ifdef WRITE_DEMUL
    auto& cfg = config_;
    printf("Saving demul data to data/demul_data.txt\n");

    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string filename = cur_directory + "/data/demul_data.bin";

    FILE* fp = fopen(filename.c_str(), "wb");
    for (size_t i = 0; i < cfg->ul_data_symbol_num_perframe; i++) {
        int total_data_symbol_id = (frame_id % TASK_BUFFER_FRAME_NUM)
                * cfg->ul_data_symbol_num_perframe
            + i;

        for (size_t sc = 0; sc < cfg->OFDM_DATA_NUM; sc++) {
            uint8_t* ptr
                = &demod_hard_buffer_[total_data_symbol_id][sc * cfg->UE_NUM];
            fwrite(ptr, cfg->UE_NUM, sizeof(uint8_t), fp);
        }
    }
    fclose(fp);
#endif
}

void Millipede::save_decode_data_to_file(UNUSED int frame_id)
{
#ifdef WRITE_DEMUL
    auto& cfg = config_;
    printf("Saving decode data to data/decode_data.bin\n");
    size_t num_decoded_bytes
        = (cfg->LDPC_config.cbLen + 7) >> 3 * cfg->LDPC_config.nblocksInSymbol;

    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string filename = cur_directory + "/data/decode_data.bin";
    FILE* fp = fopen(filename.c_str(), "wb");

    for (size_t i = 0; i < cfg->ul_data_symbol_num_perframe; i++) {
        int total_data_symbol_id = (frame_id % TASK_BUFFER_FRAME_NUM)
                * cfg->ul_data_symbol_num_perframe
            + i;
        uint8_t* ptr = decoded_buffer_[total_data_symbol_id];
        fwrite(ptr, cfg->UE_NUM * num_decoded_bytes, sizeof(uint8_t), fp);
    }
    fclose(fp);
#endif
}

void Millipede::save_tx_data_to_file(UNUSED int frame_id)
{
#ifdef WRITE_TX
    auto& cfg = config_;
    printf("Saving TX data to data/tx_data.bin\n");

    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string filename = cur_directory + "/data/tx_data.bin";
    FILE* fp = fopen(filename.c_str(), "wb");

    for (size_t i = 0; i < cfg->dl_data_symbol_num_perframe; i++) {
        int total_data_symbol_id
            = (frame_id % TASK_BUFFER_FRAME_NUM) * cfg->data_symbol_num_perframe
            + i + cfg->dl_data_symbol_start;

        for (size_t ant_id = 0; ant_id < cfg->BS_ANT_NUM; ant_id++) {
            int offset = total_data_symbol_id * cfg->BS_ANT_NUM + ant_id;
            int packet_length = config_->packet_length;
            struct Packet* pkt
                = (struct Packet*)(&dl_socket_buffer_[offset * packet_length]);
            short* socket_ptr = pkt->data;
            fwrite(socket_ptr, cfg->sampsPerSymbol * 2, sizeof(short), fp);
        }
    }
    fclose(fp);
#endif
}

void Millipede::getDemulData(int** ptr, int* size)
{
    int ul_data_symbol_num_perframe = config_->ul_data_symbol_num_perframe;
    size_t OFDM_DATA_NUM = config_->OFDM_DATA_NUM;
    int UE_NUM = config_->UE_NUM;
    *ptr = (int*)&demod_hard_buffer_[max_equaled_frame
        * ul_data_symbol_num_perframe][0];
    *size = UE_NUM * OFDM_DATA_NUM;
}

void Millipede::getEqualData(float** ptr, int* size)
{
    auto& cfg = config_;
    *ptr = (float*)&equal_buffer_[max_equaled_frame
        * cfg->ul_data_symbol_num_perframe][0];
    // *ptr = equal_output;
    *size = cfg->UE_NUM * cfg->OFDM_DATA_NUM * 2;

    // printf("In getEqualData()\n");
    // for(int ii = 0; ii < UE_NUM*OFDM_DATA_NUM; ii++)
    //{
    //    // printf("User %d: %d, ", ii,demul_ptr2(ii));
    //    printf("[%.4f+j%.4f] ", *(*ptr+ii*UE_NUM*2), *(*ptr+ii*UE_NUM*2+1));
    //}
    // printf("\n");
    // printf("\n");
}

extern "C" {
EXPORT Millipede* Millipede_new(Config* cfg)
{
    // printf("Size of Millipede: %d\n",sizeof(Millipede *));
    auto* millipede = new Millipede(cfg);

    return millipede;
}
EXPORT void Millipede_start(Millipede* millipede) { millipede->start(); }
EXPORT void Millipede_stop(/*Millipede *millipede*/)
{
    SignalHandler::setExitSignal(true); /*millipede->stop();*/
}
EXPORT void Millipede_destroy(Millipede* millipede) { delete millipede; }
EXPORT void Millipede_getEqualData(Millipede* millipede, float** ptr, int* size)
{
    return millipede->getEqualData(ptr, size);
}
EXPORT void Millipede_getDemulData(Millipede* millipede, int** ptr, int* size)
{
    return millipede->getDemulData(ptr, size);
}
}
