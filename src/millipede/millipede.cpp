/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#include "millipede.hpp"
#include "Consumer.hpp"
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

    pin_to_core_with_offset(ThreadType::kMaster, cfg->core_offset, 0);
    initialize_queues();
    initialize_uplink_buffers();

    if (config_->dl_data_symbol_num_perframe > 0) {
        printf("Millipede: Initializing downlink buffers\n");
        initialize_downlink_buffers();
    }

    stats_manager_ = new Stats(config_, 4, cfg->worker_thread_num,
        cfg->fft_thread_num, cfg->zf_thread_num, cfg->demul_thread_num);

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

void Millipede::start()
{
    auto& cfg = config_;

    /* start txrx receiver */
    if (!receiver_->startTXRX(socket_buffer_, socket_buffer_status_,
            socket_buffer_status_size_, socket_buffer_size_,
            stats_manager_->frame_start, dl_socket_buffer_,
            dl_socket_buffer_status_, dl_socket_buffer_status_size_,
            dl_socket_buffer_size_)) {
        this->stop();
        return;
    }

    /* Tokens used for enqueue */
    /* Uplink */
    moodycamel::ProducerToken ptok_fft(fft_queue_);
    Consumer consumer_fft(
        fft_queue_, ptok_fft, fft_stats_.max_task_count, EventType::kFFT);

    moodycamel::ProducerToken ptok_zf(zf_queue_);
    Consumer consumer_zf(
        zf_queue_, ptok_zf, zf_stats_.max_task_count, EventType::kZF);

    moodycamel::ProducerToken ptok_demul(demul_queue_);
    Consumer consumer_demul(demul_queue_, ptok_demul,
        demul_stats_.max_task_count, EventType::kDemul);

#ifdef USE_LDPC
    moodycamel::ProducerToken ptok_decode(decode_queue_);
    Consumer consumer_decode(decode_queue_, ptok_decode,
        decode_stats_.max_task_count, EventType::kDecode);
#endif

    /* Downlink */
#ifdef USE_LDPC
    moodycamel::ProducerToken ptok_encode(encode_queue_);
    Consumer consumer_encode(encode_queue_, ptok_encode,
        encode_stats_.max_task_count, EventType::kEncode);
#endif

    moodycamel::ProducerToken ptok_ifft(ifft_queue_);
    Consumer consumer_ifft(
        ifft_queue_, ptok_ifft, ifft_stats_.max_task_count, EventType::kIFFT);

    moodycamel::ProducerToken ptok_rc(rc_queue_);
    Consumer consumer_rc(
        rc_queue_, ptok_rc, rc_stats_.max_task_count, EventType::kRC);

    moodycamel::ProducerToken ptok_precode(precode_queue_);
    Consumer consumer_precode(precode_queue_, ptok_precode,
        precode_stats_.max_task_count, EventType::kPrecode);

    /* Tokens used for dequeue */
    moodycamel::ConsumerToken ctok(message_queue_);
    moodycamel::ConsumerToken ctok_complete(complete_task_queue_);

    size_t cur_frame_id = 0;

    /* Counters for printing summary */
    int demul_count = 0;
    int tx_count = 0;
    double demul_begin = get_time();
    double tx_begin = get_time();

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
                // TODOs: Should it be complete_task_queue_?
                num_events += message_queue_.try_dequeue_bulk_from_producer(
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
                int offset = event.data;
                int socket_thread_id, offset_in_current_buffer;
                interpreteOffset2d_setbits(
                    offset, &socket_thread_id, &offset_in_current_buffer, 28);
                char* socket_buffer_ptr = socket_buffer_[socket_thread_id]
                    + (long long)offset_in_current_buffer * cfg->packet_length;
                struct Packet* pkt = (struct Packet*)socket_buffer_ptr;

                frame_count = pkt->frame_id % 10000;
                size_t frame_id = frame_count % TASK_BUFFER_FRAME_NUM;
                int subframe_id = pkt->symbol_id;

                update_rx_counters(frame_count, frame_id, subframe_id);
                if (config_->bigstation_mode) {
                    /* In BigStation, schedule FFT whenever a packet is RX */
                    if (cur_frame_id != frame_id) {
                        stats_manager_->update_processing_started(frame_count);
                        cur_frame_id = frame_id;
                    }
                }

                fft_queue_arr[frame_id].push(offset);
            } break;

            case EventType::kFFT: {
                if (event.num_offsets == 0) {
                    handle_event_fft(
                        event.data, consumer_zf, consumer_demul, consumer_rc);
                } else {
                    for (int i = 0; i < event.num_offsets; i++) {
                        handle_event_fft(event.offsets[i], consumer_zf,
                            consumer_demul, consumer_rc);
                    }
                }
            } break;

            case EventType::kRC: {
                int frame_id = event.data;
                stats_manager_->update_rc_processed(rc_stats_.frame_count);
                print_per_frame_done(PRINT_RC, rc_stats_.frame_count, frame_id);
                fft_stats_.symbol_cal_count[frame_id] = 0;
                rc_stats_.update_frame_count();
            } break;

            case EventType::kZF: {
                int offset = event.data;
                int frame_id = offset / zf_stats_.max_symbol_count;

                print_per_task_done(
                    PRINT_ZF, frame_id, 0, zf_stats_.symbol_count[frame_id]);
                if (zf_stats_.last_symbol(frame_id)) {
                    stats_manager_->update_zf_processed(zf_stats_.frame_count);
                    zf_stats_.coded_frame = frame_id;
                    print_per_frame_done(
                        PRINT_ZF, zf_stats_.frame_count, frame_id);
                    zf_stats_.update_frame_count();

                    /* If all the data in a frame has arrived when ZF is done */
                    schedule_demul_task(frame_id, 0,
                        fft_stats_.max_symbol_data_count, consumer_demul);
                    if (config_->dl_data_symbol_num_perframe > 0) {
                        /* If downlink data transmission is enabled, schedule
                         * downlink encode/modulation for the first data
                         * subframe */
                        int total_data_subframe_id
                            = frame_id * cfg->data_symbol_num_perframe
                            + cfg->dl_data_symbol_start;
#ifdef USE_LDPC
                        consumer_encode.schedule_task_set(
                            total_data_subframe_id);
#else
                        consumer_precode.schedule_task_set(
                            total_data_subframe_id);
#endif
                    }
                }
            } break;

            case EventType::kDemul: {
                int offset = event.data;
                int block_size = config_->demul_block_size;
                int block_num = demul_stats_.max_task_count;
                int sc_id = offset % block_num * block_size;
                int total_data_subframe_id = offset / block_num;
                int frame_id
                    = total_data_subframe_id / cfg->ul_data_symbol_num_perframe;
                int data_subframe_id
                    = total_data_subframe_id % cfg->ul_data_symbol_num_perframe;

                print_per_task_done(
                    PRINT_DEMUL, frame_id, data_subframe_id, sc_id);
                /* If this subframe is ready */
                if (demul_stats_.last_task(frame_id, data_subframe_id)) {
                    max_equaled_frame = frame_id;
#ifdef USE_LDPC
                    consumer_decode.schedule_task_set(total_data_subframe_id);
#endif
                    print_per_subframe_done(PRINT_DEMUL,
                        demul_stats_.frame_count, frame_id, data_subframe_id);
                    if (demul_stats_.last_symbol(frame_id)) {
                        /* Schedule fft for the next frame if there are delayed
                         * fft tasks */
#ifndef USE_LDPC
                        cur_frame_id = (frame_id + 1) % TASK_BUFFER_FRAME_NUM;
                        frame_count = demul_stats_.frame_count + 1;
                        stats_manager_->update_stats_in_functions_uplink(
                            demul_stats_.frame_count);
                        if (stats_manager_->last_frame_id
                            == config_->tx_frame_num - 1)
                            goto finish;
#endif
                        stats_manager_->update_demul_processed(
                            demul_stats_.frame_count);
                        print_per_frame_done(
                            PRINT_DEMUL, demul_stats_.frame_count, frame_id);

                        demul_stats_.update_frame_count();
                    }

                    demul_count++;
                    if (demul_count == demul_stats_.max_symbol_count * 9000) {
                        demul_count = 0;
                        double diff = get_time() - demul_begin;
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
                        demul_begin = get_time();
                    }
                }
            } break;

#ifdef USE_LDPC
            case EventType::kDecode: {
                int offset = event.data;
                int num_code_blocks = decode_stats_.max_task_count;
                int total_data_subframe_id = offset / num_code_blocks;
                int frame_id
                    = total_data_subframe_id / ul_data_subframe_num_perframe;
                int data_subframe_id
                    = total_data_subframe_id % ul_data_subframe_num_perframe;

                if (decode_stats_.last_task(frame_id, data_subframe_id)) {
                    print_per_subframe_done(PRINT_DECODE,
                        decode_stats_.frame_count, frame_id, data_subframe_id);
                    if (decode_stats_.last_symbol(frame_id)) {
                        cur_frame_id = (frame_id + 1) % TASK_BUFFER_FRAME_NUM;
                        frame_count = decode_stats_.frame_count + 1;
                        stats_manager_->update_decode_processed(
                            decode_stats_.frame_count);
                        print_per_frame_done(
                            PRINT_DECODE, decode_stats_.frame_count, frame_id);
                        stats_manager_->update_stats_in_functions_uplink(
                            decode_stats_.frame_count);
                        if (stats_manager_->last_frame_id
                            == config_->tx_frame_num - 1)
                            goto finish;
                        decode_stats_.update_frame_count();
                    }
                }
            } break;

            case EventType::kEncode: {
                int offset = event.data;
                int num_code_blocks = encode_stats_.max_task_count;
                int total_data_subframe_id = offset / num_code_blocks;
                int frame_id
                    = total_data_subframe_id / data_subframe_num_perframe;
                int data_subframe_id
                    = total_data_subframe_id % data_subframe_num_perframe;

                if (encode_stats_.last_task(frame_id, data_subframe_id)) {
                    consumer_precode.schedule_task_set(total_data_subframe_id);
                    print_per_subframe_done(PRINT_ENCODE,
                        encode_stats_.frame_count, frame_id, data_subframe_id);
                    if (encode_stats_.last_symbol(frame_id)) {
                        stats_manager_->update_encode_processed(
                            encode_stats_.frame_count);
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
                int total_data_subframe_id = offset / block_num;
                int data_subframe_id
                    = total_data_subframe_id % cfg->data_symbol_num_perframe;
                int frame_id
                    = total_data_subframe_id / cfg->data_symbol_num_perframe;

                print_per_task_done(
                    PRINT_PRECODE, frame_id, data_subframe_id, sc_id);
                if (precode_stats_.last_task(frame_id, data_subframe_id)) {
                    int offset = precode_stats_.frame_count
                            * cfg->data_symbol_num_perframe
                        + data_subframe_id;
                    consumer_ifft.schedule_task_set(offset);
                    if (data_subframe_id < (int)cfg->dl_data_symbol_end - 1) {
#ifdef USE_LDPC
                        consumer_encode.schedule_task_set(
                            total_data_subframe_id + 1);
#else
                        consumer_precode.schedule_task_set(
                            total_data_subframe_id + 1);
#endif
                    }

                    print_per_subframe_done(PRINT_PRECODE,
                        precode_stats_.frame_count, frame_id, data_subframe_id);
                    if (precode_stats_.last_symbol(frame_id)) {
                        stats_manager_->update_precode_processed(
                            precode_stats_.frame_count);
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
                int total_data_subframe_id = offset / cfg->BS_ANT_NUM;
                int frame_id = total_data_subframe_id
                    / cfg->data_symbol_num_perframe % TASK_BUFFER_FRAME_NUM;
                int data_subframe_id
                    = total_data_subframe_id % cfg->data_symbol_num_perframe;
                int ptok_id = ant_id % cfg->socket_thread_num; /* RX */

                Consumer consumer_tx(
                    tx_queue_, *tx_ptoks_ptr[ptok_id], 1, EventType::kPacketTX);

                consumer_tx.schedule_task_set(offset);
                print_per_task_done(
                    PRINT_IFFT, frame_id, data_subframe_id, ant_id);

                if (ifft_stats_.last_task(frame_id, data_subframe_id)) {
                    if (ifft_stats_.last_symbol(frame_id)) {
                        cur_frame_id = (frame_id + 1) % TASK_BUFFER_FRAME_NUM;
                        frame_count = ifft_stats_.frame_count + 1;
                        stats_manager_->update_ifft_processed(
                            ifft_stats_.frame_count);
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
                int total_data_subframe_id = offset / cfg->BS_ANT_NUM;
                int frame_id
                    = total_data_subframe_id / cfg->data_symbol_num_perframe;
                int data_subframe_id
                    = total_data_subframe_id % cfg->data_symbol_num_perframe;
                // printf("In main thread: tx finished for ",
                //     "frame %d subframe %d ant %d\n",
                //     frame_id, data_subframe_id, ant_id);
                frame_id = frame_id % TASK_BUFFER_FRAME_NUM;

                print_per_task_done(
                    PRINT_TX, frame_id, data_subframe_id, ant_id);
                if (tx_stats_.last_task(frame_id, data_subframe_id)) {
                    print_per_subframe_done(PRINT_TX, tx_stats_.frame_count,
                        frame_id, data_subframe_id);
                    /* If tx of the first symbol is done */
                    if (data_subframe_id == (int)cfg->dl_data_symbol_start) {
                        stats_manager_->update_tx_processed_first(
                            tx_stats_.frame_count);
                        print_per_frame_done(
                            PRINT_TX_FIRST, tx_stats_.frame_count, frame_id);
                    }
                    if (tx_stats_.last_symbol(frame_id)) {
                        stats_manager_->update_tx_processed(
                            tx_stats_.frame_count);
                        print_per_frame_done(
                            PRINT_TX, tx_stats_.frame_count, frame_id);
                        stats_manager_->update_stats_in_functions_downlink(
                            tx_stats_.frame_count);
                        if (stats_manager_->last_frame_id
                            == config_->tx_frame_num - 1)
                            goto finish;
                        tx_stats_.update_frame_count();
                    }

                    tx_count++;
                    if (tx_count == tx_stats_.max_symbol_count * 9000) {
                        tx_count = 0;
                        double diff = get_time() - tx_begin;
                        int samples_num_per_UE = cfg->OFDM_DATA_NUM
                            * tx_stats_.max_symbol_count * 1000;

                        printf("TX %d samples (per-client) to %zu clients "
                               "in %f secs, throughtput %f bps per-client "
                               "(16QAM), current tx queue length %zu\n",
                            samples_num_per_UE, cfg->UE_NUM, diff,
                            samples_num_per_UE * log2(16.0f) / diff,
                            tx_queue_.size_approx());
                        tx_begin = get_time();
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
                            = fft_queue_arr[cur_frame_id].front();
                        fft_queue_arr[cur_frame_id].pop();

                        if (!config_->bigstation_mode) {
                            if (fft_created_count++ == 0) {
                                stats_manager_->update_processing_started(
                                    frame_count);
                            } else if (fft_created_count
                                == rx_stats_.max_task_count) {
                                fft_created_count = 0;
                            }
                        }
                    }
                    consumer_fft.try_handle(do_fft_task);
                }
            }
        } /* End of for */
    } /* End of while */

finish:

    printf("Millipede: printing stats\n");
    int last_frame_id = stats_manager_->last_frame_id;
    stats_manager_->save_to_file(last_frame_id);
    stats_manager_->print_summary(last_frame_id);

#ifdef USE_LDPC
    save_decode_data_to_file(last_frame_id);
#else
    save_demul_data_to_file(last_frame_id);
#endif

    save_ifft_data_to_file(last_frame_id);
    this->stop();
    // exit(0);
}

void Millipede::handle_event_fft(int offset, Consumer& consumer_zf,
    Consumer& consumer_demul, Consumer& consumer_rc)
{
    int frame_id = offset / config_->symbol_num_perframe;
    int subframe_id = offset % config_->symbol_num_perframe;

    if (fft_stats_.last_task(frame_id, subframe_id)) {
        if (config_->isPilot(frame_id, subframe_id)) {
            print_per_subframe_done(PRINT_FFT_PILOTS, fft_stats_.frame_count,
                frame_id, subframe_id);
            /* If CSI of all UEs is ready, schedule ZF/prediction */
            if (fft_stats_.last_symbol(frame_id)) {
                stats_manager_->update_fft_processed(fft_stats_.frame_count);
                print_per_frame_done(
                    PRINT_FFT_PILOTS, fft_stats_.frame_count, frame_id);
                fft_stats_.update_frame_count();
                consumer_zf.schedule_task_set(frame_id);
            }
        } else if (config_->isUplink(frame_id, subframe_id)) {
            int data_subframe_id = config_->getUlSFIndex(frame_id, subframe_id);
            fft_stats_.cur_frame_for_symbol[data_subframe_id] = frame_id;
            print_per_subframe_done(PRINT_FFT_DATA, fft_stats_.frame_count - 1,
                frame_id, subframe_id);
            /* If precoder exist, schedule demodulation */
            if (zf_stats_.coded_frame == frame_id)
                schedule_demul_task(frame_id, data_subframe_id,
                    data_subframe_id + 1, consumer_demul);
        } else if (config_->isCalDlPilot(frame_id, subframe_id)
            || config_->isCalUlPilot(frame_id, subframe_id)) {
            print_per_subframe_done(PRINT_FFT_CAL, fft_stats_.frame_count - 1,
                frame_id, subframe_id);
            if (++fft_stats_.symbol_cal_count[frame_id]
                == fft_stats_.max_symbol_cal_count) {
                print_per_frame_done(
                    PRINT_FFT_CAL, fft_stats_.frame_count - 1, frame_id);
                consumer_rc.schedule_task_set(frame_id);
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
    // moodycamel::ProducerToken ptok_complete(complete_task_queue_);
    // Consumer consumer(complete_task_queue_, ptok_complete);
    Consumer consumer(complete_task_queue_, *worker_ptoks_ptr[tid]);

    /* Initialize operators */
    auto computeFFT = new DoFFT(config_, tid, fft_queue_, consumer,
        socket_buffer_, socket_buffer_status_, data_buffer_, csi_buffer_,
        calib_buffer_, stats_manager_);

    auto computeIFFT = new DoIFFT(config_, tid, ifft_queue_, consumer,
        dl_ifft_buffer_, dl_socket_buffer_, stats_manager_);

    auto computeZF = new DoZF(config_, tid, zf_queue_, consumer, csi_buffer_,
        recip_buffer_, precoder_buffer_, dl_precoder_buffer_, stats_manager_);

    auto computeDemul = new DoDemul(config_, tid, demul_queue_, consumer,
        data_buffer_, precoder_buffer_, equal_buffer_, demod_hard_buffer_,
        demod_soft_buffer_, stats_manager_);

    auto computePrecode = new DoPrecode(config_, tid, precode_queue_, consumer,
        dl_precoder_buffer_, dl_ifft_buffer_,
#ifdef USE_LDPC
        dl_encoded_buffer_,
#else
        config_->dl_IQ_data,
#endif
        stats_manager_);

#ifdef USE_LDPC
    auto* computeEncoding = new DoEncode(config_, tid, encode_queue_, consumer,
        config_->dl_IQ_data, dl_encoded_buffer_, stats_manager_);
    auto* computeDecoding = new DoDecode(config_, tid, decode_queue_, consumer,
        demod_soft_buffer_, decoded_buffer_, stats_manager_);
#endif
    auto* computeReciprocity = new Reciprocity(config_, tid, rc_queue_,
        consumer, calib_buffer_, recip_buffer_, stats_manager_);

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

    moodycamel::ProducerToken ptok_complete(complete_task_queue_);
    Consumer consumer(complete_task_queue_, ptok_complete);

    /* Initialize IFFT operator */
    auto computeFFT = new DoFFT(config_, tid, fft_queue_, consumer,
        socket_buffer_, socket_buffer_status_, data_buffer_, csi_buffer_,
        calib_buffer_, stats_manager_);
    auto computeIFFT = new DoIFFT(config_, tid, ifft_queue_, consumer,
        dl_ifft_buffer_, dl_socket_buffer_, stats_manager_);

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

    moodycamel::ProducerToken ptok_complete(complete_task_queue_);
    Consumer consumer(complete_task_queue_, ptok_complete);

    /* Initialize ZF operator */
    auto computeZF = new DoZF(config_, tid, zf_queue_, consumer, csi_buffer_,
        recip_buffer_, precoder_buffer_, dl_precoder_buffer_, stats_manager_);

    while (true) {
        computeZF->try_launch();
    }
}

void* Millipede::worker_demul(int tid)
{
    pin_worker(ThreadType::kWorkerDemul, tid, config_);

    moodycamel::ProducerToken ptok_complete(complete_task_queue_);
    Consumer consumer(complete_task_queue_, ptok_complete);

    /* Initialize Demul operator */
    auto computeDemul = new DoDemul(config_, tid, demul_queue_, consumer,
        data_buffer_, precoder_buffer_, equal_buffer_, demod_hard_buffer_,
        demod_soft_buffer_, stats_manager_);

    /* Initialize Precode operator */
    auto computePrecode = new DoPrecode(config_, tid, precode_queue_, consumer,
        dl_precoder_buffer_, dl_ifft_buffer_,
#ifdef USE_LDPC
        dl_encoded_buffer_,
#else
        config_->dl_IQ_data,
#endif
        stats_manager_);

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

void Millipede::schedule_demul_task(int frame_id, int start_subframe_id,
    int end_subframe_id, Consumer const& consumer)
{
    int data_subframe_num_perframe = config_->ul_data_symbol_num_perframe;
    for (int data_subframe_id = start_subframe_id;
         data_subframe_id < end_subframe_id; data_subframe_id++) {
        if (fft_stats_.cur_frame_for_symbol[data_subframe_id] == frame_id) {
            /* Schedule demodulation task for subcarrier blocks */
            int total_data_subframe_id
                = frame_id * data_subframe_num_perframe + data_subframe_id;
            consumer.schedule_task_set(total_data_subframe_id);
#if DEBUG_PRINT_PER_SUBFRAME_ENTER_QUEUE
            printf("Main thread: created Demodulation task for frame: %d, "
                   "start subframe: %d, current subframe: %d, in %.2f\n",
                frame_id, start_subframe_id, data_subframe_id,
                get_time()
                    - stats_manager_->get_pilot_received(
                          demul_stats_.frame_count));
#endif
        }
    }
}

void Millipede::update_rx_counters(
    int frame_count, int frame_id, int subframe_id)
{
    if (config_->isPilot(frame_count, subframe_id)) {
        if (++rx_stats_.task_pilot_count[frame_id]
            == rx_stats_.max_task_pilot_count) {
            rx_stats_.task_pilot_count[frame_id] = 0;
            stats_manager_->update_pilot_all_received(frame_count);
            print_per_frame_done(PRINT_RX_PILOTS, frame_count, frame_id);
        }
    }
    if (rx_stats_.task_count[frame_id]++ == 0) {
        stats_manager_->update_pilot_received(frame_count);
#if DEBUG_PRINT_PER_FRAME_START
        int prev_frame_id
            = (frame_count + TASK_BUFFER_FRAME_NUM - 1) % TASK_BUFFER_FRAME_NUM;
        printf("Main thread: data received from frame %d, subframe %d, in %.2f "
               "us. RX in prev frame: %d\n",
            frame_count, subframe_id,
            stats_manager_->get_pilot_received(frame_count)
                - stats_manager_->get_pilot_received(frame_count - 1),
            rx_stats_.task_count[prev_frame_id]);
#endif
    } else if (rx_stats_.task_count[frame_id] == rx_stats_.max_task_count) {
        stats_manager_->update_rx_processed(frame_count);
        print_per_frame_done(PRINT_RX, frame_count, frame_id);
        rx_stats_.task_count[frame_id] = 0;
    }
}

void Millipede::print_per_frame_done(
    int task_type, int frame_count, int frame_id)
{
    int dl_data_subframe_num_perframe = config_->dl_data_symbol_num_perframe;
    int ul_data_subframe_num_perframe = config_->ul_data_symbol_num_perframe;
#if DEBUG_PRINT_PER_FRAME_DONE
    switch (task_type) {
    case (PRINT_RX): {
        int prev_frame_count = (frame_count - 1) % TASK_BUFFER_FRAME_NUM;
        printf("Main thread: received all packets in frame: %d, frame buffer: "
               "%d in %.2f us, demul: %d done, rx in prev frame: %d\n",
            frame_count, frame_id,
            stats_manager_->get_rx_processed(frame_count)
                - stats_manager_->get_pilot_received(frame_count),
            demul_stats_.symbol_count[frame_id],
            rx_stats_.task_count[prev_frame_count]);
    } break;
    case (PRINT_RX_PILOTS):
        printf("Main thread: received all pilots in frame: %d, frame buffer: "
               "%d in %.2f us\n",
            frame_count, frame_id,
            stats_manager_->get_pilot_all_received(frame_count)
                - stats_manager_->get_pilot_received(frame_count));
        break;
    case (PRINT_FFT_PILOTS):
        printf("Main thread: pilot frame: %d, %d, finished FFT for all pilot "
               "subframes in %.2f us, pilot all received: %.2f\n",
            frame_count, frame_id,
            stats_manager_->get_fft_processed(frame_count)
                - stats_manager_->get_pilot_received(frame_count),
            stats_manager_->get_pilot_all_received(frame_count)
                - stats_manager_->get_pilot_received(frame_count));
        break;
    case (PRINT_FFT_DATA):
        printf("Main thread: data frame: %d, %d, finished FFT for all data "
               "subframes in %.2f us\n",
            frame_count, frame_id,
            get_time() - stats_manager_->get_pilot_received(frame_count));
        break;
    case (PRINT_FFT_CAL):
        printf("Main thread: cal frame: %d, %d, finished FFT for all cal "
               "subframes in %.2f us\n",
            frame_count, frame_id,
            get_time() - stats_manager_->get_pilot_received(frame_count));
        break;
    case (PRINT_ZF):
        printf("Main thread: ZF done frame: %d, %d in %.2f us since pilot FFT "
               "done, total: %.2f us, FFT queue %zu\n",
            frame_count, frame_id,
            stats_manager_->get_zf_processed(frame_count)
                - stats_manager_->get_fft_processed(frame_count),
            stats_manager_->get_zf_processed(frame_count)
                - stats_manager_->get_pilot_received(frame_count),
            fft_queue_.size_approx());
        break;
    case (PRINT_DEMUL):
        printf("Main thread: Demodulation done frame: %d, %d (%d UL subframes) "
               "in %.2f us since ZF done, total %.2f us\n",
            frame_count, frame_id, ul_data_subframe_num_perframe,
            stats_manager_->get_demul_processed(frame_count)
                - stats_manager_->get_zf_processed(frame_count),
            stats_manager_->get_demul_processed(frame_count)
                - stats_manager_->get_pilot_received(frame_count));
        break;
    case (PRINT_DECODE):
        printf("Main thread: Decoding done frame: %d, %d (%d UL subframes) in "
               "%.2f us since ZF done, total %.2f us\n",
            frame_count, frame_id, ul_data_subframe_num_perframe,
            stats_manager_->get_decode_processed(frame_count)
                - stats_manager_->get_zf_processed(frame_count),
            stats_manager_->get_decode_processed(frame_count)
                - stats_manager_->get_pilot_received(frame_count));
        break;
    case (PRINT_ENCODE):
        printf("Main thread: Encoding done frame: %d, %d in %.2f us since ZF "
               "done, total %.2f us\n",
            frame_count, frame_id,
            stats_manager_->get_encode_processed(frame_count)
                - stats_manager_->get_zf_processed(frame_count),
            stats_manager_->get_encode_processed(frame_count)
                - stats_manager_->get_pilot_received(frame_count));
        break;
    case (PRINT_PRECODE):
        printf("Main thread: Precoding done frame: %d, %d in %.2f us since ZF "
               "done, total: %.2f us\n",
            frame_count, frame_id,
            stats_manager_->get_precode_processed(frame_count)
                - stats_manager_->get_zf_processed(frame_count),
            stats_manager_->get_precode_processed(frame_count)
                - stats_manager_->get_pilot_received(frame_count));
        break;
    case (PRINT_RC):
        printf("Main thread: Reciprocity Calculation done frame: %d, %d in "
               "%.2f us since pilot FFT done, total: %.2f us\n",
            frame_count, frame_id,
            stats_manager_->get_rc_processed(frame_id)
                - stats_manager_->get_fft_processed(frame_id),
            stats_manager_->get_rc_processed(frame_id)
                - stats_manager_->get_pilot_received(frame_id));
        break;
    case (PRINT_IFFT):
        printf("Main thread: IFFT done frame: %d, %d in %.2f us since precode "
               "done, total: %.2f us\n",
            frame_count, frame_id,
            stats_manager_->get_ifft_processed(frame_count)
                - stats_manager_->get_precode_processed(frame_count),
            stats_manager_->get_ifft_processed(frame_count)
                - stats_manager_->get_pilot_received(frame_count));
        break;
    case (PRINT_TX_FIRST):
        printf("Main thread: TX of first subframe done frame: %d, %d in %.2f "
               "us since ZF done, total: %.2f us\n",
            frame_count, frame_id,
            stats_manager_->get_tx_processed_first(frame_count)
                - stats_manager_->get_zf_processed(frame_count),
            stats_manager_->get_tx_processed_first(frame_count)
                - stats_manager_->get_pilot_received(frame_count));
        break;
    case (PRINT_TX):
        printf("Main thread: TX done frame: %d %d (%d DL subframes) in %.2f us "
               "since ZF done, total: %.2f us\n",
            frame_count, frame_id, dl_data_subframe_num_perframe,
            stats_manager_->get_tx_processed(frame_count)
                - stats_manager_->get_zf_processed(frame_count),
            stats_manager_->get_tx_processed(frame_count)
                - stats_manager_->get_pilot_received(frame_count));
        break;
    default:
        printf("Wrong task type in frame done print!");
    }
#endif
}

void Millipede::print_per_subframe_done(UNUSED int task_type,
    UNUSED int frame_count, UNUSED int frame_id, UNUSED int subframe_id)
{
#if DEBUG_PRINT_PER_SUBFRAME_DONE
    switch (task_type) {
    case (PRINT_FFT_PILOTS):
        printf("Main thread: pilot FFT done frame: %d, %d, subframe: %d, num "
               "subframes done: %d\n",
            frame_count, frame_id, subframe_id,
            fft_stats_.symbol_count[frame_id]);
        break;
    case (PRINT_FFT_DATA):
        printf(
            "Main thread: data FFT done frame %d, %d, subframe %d, precoder "
            "status: %d, fft queue: %zu, zf queue: %zu, demul queue: %zu, in "
            "%.2f\n",
            frame_count, frame_id, subframe_id,
            zf_stats_.coded_frame == frame_id, fft_queue_.size_approx(),
            zf_queue_.size_approx(), demul_queue_.size_approx(),
            get_time() - stats_manager_->get_pilot_received(frame_count));
        break;
    case (PRINT_RC):
        printf("Main thread: cal symbol FFT done frame: %d, %d, subframe: %d, "
               "num subframes done: %d\n",
            frame_count, frame_id, subframe_id,
            fft_stats_.symbol_cal_count[frame_id]);
        break;
    case (PRINT_DEMUL):
        printf("Main thread: Demodulation done frame %d %d, subframe: %d, num "
               "subframes done: %d in %.2f\n",
            frame_count, frame_id, subframe_id,
            demul_stats_.symbol_count[frame_id],
            get_time() - stats_manager_->get_pilot_received(frame_count));
        break;
#ifdef USE_LDPC
    case (PRINT_DECODE):
        printf("Main thread: Decoding done frame %d %d, subframe: %d, num "
               "subframes done: %d\n",
            frame_count, frame_id, subframe_id,
            decode_stats_.symbol_count[frame_id]);
        break;
    case (PRINT_ENCODE):
        printf("Main thread: Encoding done frame %d %d, subframe: %d, num "
               "subframes done: %d\n",
            frame_count, frame_id, subframe_id,
            encode_stats_.symbol_count[frame_id]);
        break;
#endif
    case (PRINT_PRECODE):
        printf("Main thread: Precoding done frame: %d %d, subframe: %d in %.2f "
               "us\n",
            frame_count, frame_id, subframe_id,
            get_time() - stats_manager_->get_pilot_received(frame_count));
        break;
    case (PRINT_TX):
        printf("Main thread: TX done frame: %d %d, subframe: %d in %.2f us\n",
            frame_count, frame_id, subframe_id,
            get_time() - stats_manager_->get_pilot_received(frame_count));
        break;
    default:
        printf("Wrong task type in frame done print!");
    }
#endif
}

void Millipede::print_per_task_done(UNUSED int task_type, UNUSED int frame_id,
    UNUSED int subframe_id, UNUSED int ant_or_sc_id)
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
        printf("Main thread: Demodulation done frame: %d, subframe: %d, sc: "
               "%d, num blocks done: %d\n",
            frame_id, subframe_id, ant_or_sc_id,
            demul_stats_.task_count[frame_id][subframe_id]);
        break;
#ifdef USE_LDPC
    case (PRINT_DECODE):
        printf("Main thread: Decoding done frame: %d, subframe: %d, sc: %d, "
               "num blocks done: %d\n",
            frame_id, subframe_id, ant_or_sc_id,
            decode_stats_.task_count[frame_id][subframe_id]);
        break;
#endif
    case (PRINT_PRECODE):
        printf("Main thread: Precoding done frame: %d, subframe: %d, "
               "subcarrier: %d, total SCs: %d\n",
            frame_id, subframe_id, ant_or_sc_id,
            precode_stats_.task_count[frame_id][subframe_id]);
        break;
    case (PRINT_IFFT):
        printf("Main thread: IFFT done frame: %d, subframe: %d, antenna: %d, "
               "total ants: %d\n",
            frame_id, subframe_id, ant_or_sc_id,
            ifft_stats_.task_count[frame_id][subframe_id]);
        break;
    case (PRINT_TX):
        printf("Main thread: TX done frame: %d, subframe: %d, antenna: %d, "
               "total packets: %d\n",
            frame_id, subframe_id, ant_or_sc_id,
            tx_stats_.task_count[frame_id][subframe_id]);
        break;
    default:
        printf("Wrong task type in frame done print!");
    }

#endif
}

void Millipede::initialize_queues()
{
    using mt_queue_t = moodycamel::ConcurrentQueue<Event_data>;

    int data_subframe_num_perframe = config_->data_symbol_num_perframe;
    message_queue_ = mt_queue_t(512 * data_subframe_num_perframe);
    complete_task_queue_ = mt_queue_t(512 * data_subframe_num_perframe * 4);

    fft_queue_ = mt_queue_t(512 * data_subframe_num_perframe * 4);
    zf_queue_ = mt_queue_t(512 * data_subframe_num_perframe * 4);

    rc_queue_ = mt_queue_t(512 * 2 * 4);

    demul_queue_ = mt_queue_t(512 * data_subframe_num_perframe * 4);
#ifdef USE_LDPC
    decode_queue_ = mt_queue_t(512 * data_subframe_num_perframe * 4);
#endif

    ifft_queue_ = mt_queue_t(512 * data_subframe_num_perframe * 4);
#ifdef USE_LDPC
    encode_queue_ = mt_queue_t(512 * data_subframe_num_perframe * 4);
#endif
    precode_queue_ = mt_queue_t(512 * data_subframe_num_perframe * 4);
    tx_queue_ = mt_queue_t(512 * data_subframe_num_perframe * 4);

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
        ul_data_subframe_num_perframe, TASK_BUFFER_FRAME_NUM,
        data_subframe_num_perframe, 64);
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
    dl_encoded_buffer_.malloc(
        TASK_BUFFER_SUBFRAME_NUM, cfg->OFDM_DATA_NUM * cfg->UE_NUM, 64);
    recip_buffer_.malloc(
        TASK_BUFFER_FRAME_NUM, cfg->OFDM_DATA_NUM * cfg->BS_ANT_NUM, 64);
    calib_buffer_.malloc(
        TASK_BUFFER_FRAME_NUM, cfg->OFDM_DATA_NUM * cfg->BS_ANT_NUM, 64);

#ifdef USE_LDPC
    encode_stats_.init(config_->LDPC_config.nblocksInSymbol * UE_NUM,
        dl_data_subframe_num_perframe, TASK_BUFFER_FRAME_NUM,
        data_subframe_num_perframe, 64);
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
        int total_data_subframe_id = (frame_id % TASK_BUFFER_FRAME_NUM)
                * cfg->ul_data_symbol_num_perframe
            + i;

        for (size_t sc = 0; sc < cfg->OFDM_DATA_NUM; sc++) {
            uint8_t* ptr
                = &demod_hard_buffer_[total_data_subframe_id][sc * cfg->UE_NUM];
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
        int total_data_subframe_id = (frame_id % TASK_BUFFER_FRAME_NUM)
                * cfg->ul_data_symbol_num_perframe
            + i;
        uint8_t* ptr = decoded_buffer_[total_data_subframe_id];
        fwrite(ptr, cfg->UE_NUM * num_decoded_bytes, sizeof(uint8_t), fp);
    }
    fclose(fp);
#endif
}

void Millipede::save_ifft_data_to_file(UNUSED int frame_id)
{
#ifdef WRITE_IFFT
    auto& cfg = config_;
    printf("Saving IFFT data to data/ifft_data.bin\n");

    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string filename = cur_directory + "/data/ifft_data.bin";
    FILE* fp = fopen(filename.c_str(), "wb");

    for (size_t i = 0; i < cfg->dl_data_symbol_num_perframe; i++) {
        int total_data_subframe_id = (frame_id % TASK_BUFFER_FRAME_NUM)
                * cfg->dl_data_symbol_num_perframe
            + i + cfg->dl_data_symbol_start;

        for (size_t ant_id = 0; ant_id < cfg->BS_ANT_NUM; ant_id++) {
            int offset = total_data_subframe_id * cfg->BS_ANT_NUM + ant_id;
            float* ptr = (float*)dl_ifft_buffer_[offset];
            fwrite(ptr, cfg->OFDM_CA_NUM * 2, sizeof(float), fp);
        }
    }
    fclose(fp);
#endif
}

void Millipede::getDemulData(int** ptr, int* size)
{
    int ul_data_subframe_num_perframe = config_->ul_data_symbol_num_perframe;
    size_t OFDM_DATA_NUM = config_->OFDM_DATA_NUM;
    int UE_NUM = config_->UE_NUM;
    *ptr = (int*)&demod_hard_buffer_[max_equaled_frame
        * ul_data_subframe_num_perframe][0];
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
