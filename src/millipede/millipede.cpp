/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 * 
 */
#include "millipede.hpp"
#include "Consumer.hpp"
using namespace std;
// typedef cx_float COMPLEX;

Millipede::Millipede(Config* cfg)
{
    std::string directory = TOSTRING(PROJECT_DIRECTORY);
    printf("PROJECT_DIRECTORY: %s\n", directory.c_str());
    printf("Main thread: on core %d\n", sched_getcpu());
    putenv("MKL_THREADING_LAYER=sequential");
    // char thread_cmd[] = "MKL_THREADING_LAYER=sequential";
    // putenv(thread_cmd);
    std::cout << "MKL_THREADING_LAYER =  " << getenv("MKL_THREADING_LAYER") << std::endl;
    // openblas_set_num_threads(1);
    printf("enter constructor\n");

    this->config_ = cfg;
#if DEBUG_PRINT_PILOT
    size_t OFDM_CA_NUM = config_->OFDM_CA_NUM;
    cout << "Pilot data" << endl;
    for (size_t i = 0; i < OFDM_CA_NUM; i++)
        cout << config_->pilots_[i] << ",";
    cout << endl;
#endif

    int TASK_THREAD_NUM = cfg->worker_thread_num;
    int SOCKET_RX_THREAD_NUM = cfg->socket_thread_num;
    int SOCKET_TX_THREAD_NUM = cfg->socket_thread_num;
    int FFT_THREAD_NUM = cfg->fft_thread_num;
    int DEMUL_THREAD_NUM = cfg->demul_thread_num;
    int ZF_THREAD_NUM = cfg->zf_thread_num;
    int CORE_OFFSET = cfg->core_offset;
    pin_to_core_with_offset(Master, CORE_OFFSET, 0);

    initialize_queues();

    printf("initialize uplink buffers\n");
    initialize_uplink_buffers();

    if (config_->downlink_mode) {
        printf("initialize downlink buffers\n");
        initialize_downlink_buffers();
    }
    stats_manager_ = new Stats(config_, 4, TASK_THREAD_NUM, FFT_THREAD_NUM, ZF_THREAD_NUM, DEMUL_THREAD_NUM);

    /* initialize TXRX threads*/
    printf("new TXRX\n");
    receiver_.reset(new PacketTXRX(config_, SOCKET_RX_THREAD_NUM, SOCKET_TX_THREAD_NUM, CORE_OFFSET + 1,
        &message_queue_, &tx_queue_, rx_ptoks_ptr, tx_ptoks_ptr));

    /* create worker threads */
#if BIGSTATION
    create_threads(Worker_FFT, 0, FFT_THREAD_NUM);
    create_threads(Worker_ZF, FFT_THREAD_NUM, FFT_THREAD_NUM + ZF_THREAD_NUM);
    create_threads(Worker_Demul, FFT_THREAD_NUM + ZF_THREAD_NUM, TASK_THREAD_NUM);
#else
    create_threads(Worker, 0, TASK_THREAD_NUM);
#endif
    // stats_manager_.reset(new Stats(config_, 4, TASK_THREAD_NUM, FFT_THREAD_NUM, ZF_THREAD_NUM, DEMUL_THREAD_NUM));
}

Millipede::~Millipede()
{
    free_uplink_buffers();
    /* downlink */
    if (config_->downlink_mode)
        free_downlink_buffers();
}

void Millipede::stop()
{
    std::cout << "stopping threads " << std::endl;
    config_->running = false;
    usleep(1000);
    receiver_.reset();
}

void Millipede::start()
{
    /* start uplink receiver */
    std::vector<pthread_t> rx_threads = receiver_->startRecv(socket_buffer_,
        socket_buffer_status_, socket_buffer_status_size_, socket_buffer_size_, stats_manager_->frame_start);
#ifdef USE_ARGOS
    if (rx_threads.size() == 0) {
        this->stop();
        return;
    }
#endif

    /* tokens used for enqueue */
    /* uplink */
    moodycamel::ProducerToken ptok_fft(fft_queue_);
    Consumer consumer_fft(fft_queue_, ptok_fft, fft_stats_.max_task_count, TASK_FFT);
    moodycamel::ProducerToken ptok_zf(zf_queue_);
    Consumer consumer_zf(zf_queue_, ptok_zf, zf_stats_.max_task_count, TASK_ZF);
    moodycamel::ProducerToken ptok_demul(demul_queue_);
    Consumer consumer_demul(demul_queue_, ptok_demul, demul_stats_.max_task_count, TASK_DEMUL);
#ifdef USE_LDPC
    moodycamel::ProducerToken ptok_decode(decode_queue_);
    Consumer consumer_decode(decode_queue_, ptok_decode, decode_stats_.max_task_count, TASK_DECODE);
#endif
    /* downlink */
#ifdef USE_LDPC
    moodycamel::ProducerToken ptok_encode(encode_queue_);
    Consumer consumer_encode(encode_queue_, ptok_encode, encode_stats_.max_task_count, TASK_ENCODE);
#endif
    moodycamel::ProducerToken ptok_ifft(ifft_queue_);
    Consumer consumer_ifft(ifft_queue_, ptok_ifft, ifft_stats_.max_task_count, TASK_IFFT);
    moodycamel::ProducerToken ptok_rc(rc_queue_);
    Consumer consumer_rc(rc_queue_, ptok_rc, rc_stats_.max_task_count, TASK_RC);
    moodycamel::ProducerToken ptok_precode(precode_queue_);
    Consumer consumer_precode(precode_queue_, ptok_precode, precode_stats_.max_task_count, TASK_PRECODE);

    /* tokens used for dequeue */
    moodycamel::ConsumerToken ctok(message_queue_);
    moodycamel::ConsumerToken ctok_complete(complete_task_queue_);

    std::vector<pthread_t> tx_threads;
    if (config_->downlink_mode) {
        /* start downlink transmitter */
        tx_threads = receiver_->startTX(dl_socket_buffer_, dl_socket_buffer_status_,
            dl_socket_buffer_status_size_, dl_socket_buffer_size_);
    }

#if !BIGSTATION
    int cur_frame_id = 0;
#endif

    /* counters for printing summary */
    int demul_count = 0;
    int tx_count = 0;
    double demul_begin = get_time();
    double tx_begin = get_time();

    bool prev_demul_scheduled = false;
    int data_subframe_num_perframe = config_->data_symbol_num_perframe;
    int ul_data_subframe_num_perframe = config_->ul_data_symbol_num_perframe;
    int subframe_num_perframe = config_->symbol_num_perframe;
    int BS_ANT_NUM = config_->BS_ANT_NUM;
    int UE_NUM = config_->UE_NUM;
    //int PILOT_NUM = config_->pilot_symbol_num_perframe;
    int OFDM_DATA_NUM = config_->OFDM_DATA_NUM;
    int dl_data_subframe_start = config_->dl_data_symbol_start;
    int dl_data_subframe_end = config_->dl_data_symbol_end;
    int packet_length = config_->packet_length;
    int SOCKET_RX_THREAD_NUM = config_->socket_thread_num;

    int last_dequeue = 0;
    int ret = 0;
    Event_data events_list[dequeue_bulk_size];
    int miss_count = 0;
    int total_count = 0;

    while (config_->running && !SignalHandler::gotExitSignal()) {
        /* get a bulk of events */
        if (last_dequeue == 0) {
            // #ifdef USE_ARGOS
            //             ret = message_queue_.try_dequeue_bulk(ctok, events_list, dequeue_bulk_size_single);
            // #else
            ret = 0;
            for (int rx_itr = 0; rx_itr < SOCKET_RX_THREAD_NUM; rx_itr++)
                ret += message_queue_.try_dequeue_bulk_from_producer(*(rx_ptoks_ptr[rx_itr]), events_list + ret, dequeue_bulk_size_single);
            // #endif
            last_dequeue = 1;
        } else {
            ret = complete_task_queue_.try_dequeue_bulk(ctok_complete, events_list, dequeue_bulk_size_single);
            last_dequeue = 0;
        }
        total_count++;
        if (total_count == 1e9) {
            //printf("message dequeue miss rate %f\n", (float)miss_count / total_count);
            total_count = 0;
            miss_count = 0;
        }
        if (ret == 0) {
            miss_count++;
            continue;
        }

        /* handle each event */
        for (int bulk_count = 0; bulk_count < ret; bulk_count++) {
            Event_data& event = events_list[bulk_count];
            switch (event.event_type) {
            case EVENT_PACKET_RECEIVED: {
                int offset = event.data;
                int socket_thread_id, offset_in_current_buffer;
                interpreteOffset2d_setbits(offset, &socket_thread_id, &offset_in_current_buffer, 28);
                char* socket_buffer_ptr = socket_buffer_[socket_thread_id] + (long long)offset_in_current_buffer * packet_length;
                struct Packet* pkt = (struct Packet*)socket_buffer_ptr;

                int frame_count = pkt->frame_id % 10000;
                int frame_id = frame_count % TASK_BUFFER_FRAME_NUM;
                int subframe_id = pkt->symbol_id;
                int ant_id = pkt->ant_id;

                update_rx_counters(frame_count, frame_id, pkt->symbol_id);
#if BIGSTATION
                /* in BigStation, schedule FFT whenever a packet is received */
                schedule_fft_task(offset, frame_count, frame_id, subframe_id, ant_id, consumer_fft);
#else
                /* if all previous frames are processed, schedule FFT for this packet */
                if (frame_id == cur_frame_id) {
                    schedule_fft_task(offset, frame_count, frame_id, subframe_id, ant_id, consumer_fft);
                } else {
                    /* if the previous frame is not finished, store offset in queue */
                    delay_fft_queue[frame_id][delay_fft_queue_cnt[frame_id]++] = offset;
                }
#endif
            } break;
            case EVENT_FFT: {
                int offset = event.data;
                int frame_id = offset / subframe_num_perframe;
                int subframe_id = offset % subframe_num_perframe;
                if (fft_stats_.last_task(frame_id, subframe_id)) {
                    if (config_->isPilot(frame_id, subframe_id)) {
                        print_per_subframe_done(PRINT_FFT_PILOTS, fft_stats_.frame_count, frame_id, subframe_id);
                        /* if csi of all UEs is ready, schedule ZF or prediction */
                        if (fft_stats_.last_symbol(frame_id)) {
                            stats_manager_->update_fft_processed(fft_stats_.frame_count);
                            print_per_frame_done(PRINT_FFT_PILOTS, fft_stats_.frame_count, frame_id);
                            fft_stats_.update_frame_count();
                            consumer_zf.schedule_task_set(frame_id);
                        }
                    } else if (config_->isUplink(frame_id, subframe_id)) {
                        int data_subframe_id = config_->getUlSFIndex(frame_id, subframe_id);
                        fft_stats_.data_exist_in_symbol[frame_id][data_subframe_id] = true;
                        fft_stats_.symbol_data_count[frame_id]++;
                        print_per_subframe_done(PRINT_FFT_DATA, fft_stats_.frame_count - 1, frame_id, subframe_id);
                        if (fft_stats_.symbol_data_count[frame_id] == fft_stats_.max_symbol_data_count) {
                            print_per_frame_done(PRINT_FFT_DATA, fft_stats_.frame_count - 1, frame_id);
                            prev_demul_scheduled = false;
                        }
                        /* if precoder exist, schedule demodulation */
                        if (zf_stats_.precoder_exist_in_frame[frame_id]) {
                            int start_data_subframe_id, end_data_subframe_id;
                            start_data_subframe_id = data_subframe_id;
                            end_data_subframe_id = fft_stats_.symbol_data_count[frame_id];
                            if (end_data_subframe_id < start_data_subframe_id)
                                end_data_subframe_id = start_data_subframe_id + 1;
                            if (!prev_demul_scheduled) {
                                start_data_subframe_id = 0;
                                prev_demul_scheduled = true;
                            }
                            schedule_demul_task(frame_id, start_data_subframe_id, end_data_subframe_id, consumer_demul);
                        }
                    } else if (config_->isCalDlPilot(frame_id, subframe_id) || config_->isCalUlPilot(frame_id, subframe_id)) {
                        fft_stats_.symbol_cal_count[frame_id]++;
                        print_per_subframe_done(PRINT_FFT_CAL, fft_stats_.frame_count - 1, frame_id, subframe_id);
                        if (fft_stats_.symbol_cal_count[frame_id] == fft_stats_.max_symbol_cal_count) {
                            print_per_frame_done(PRINT_FFT_CAL, fft_stats_.frame_count - 1, frame_id);
                            consumer_rc.schedule_task_set(frame_id);
                        }
                    }
                }
            } break;
            case EVENT_RC: {
                int frame_id = event.data;
                stats_manager_->update_rc_processed(rc_stats_.frame_count);
                print_per_frame_done(PRINT_RC, rc_stats_.frame_count, frame_id);
                rc_stats_.update_frame_count();
            } break;
            case EVENT_UP_ZF:
            case EVENT_DN_ZF: {
                int offset = event.data;

                int frame_id = offset / zf_stats_.max_symbol_count;
                print_per_task_done(PRINT_ZF, frame_id, 0, zf_stats_.symbol_count[frame_id]);
                if (zf_stats_.last_symbol(frame_id)) {
                    stats_manager_->update_zf_processed(zf_stats_.frame_count);
                    zf_stats_.precoder_exist_in_frame[frame_id] = true;
                    print_per_frame_done(PRINT_ZF, zf_stats_.frame_count, frame_id);
                    zf_stats_.update_frame_count();
                    //int subframe_num_perframe = config_->symbol_num_perframe;
                    /* if all the data in a frame has arrived when ZF is done */
                    if (fft_stats_.symbol_data_count[frame_id] == fft_stats_.max_symbol_data_count)
                        schedule_demul_task(frame_id, 0, fft_stats_.max_symbol_data_count, consumer_demul);
                    if (event.event_type == EVENT_DN_ZF) {
                        /* if downlink data transmission is enabled, schedule downlink encode/modulation for the first data subframe */
                        int total_data_subframe_id = frame_id * data_subframe_num_perframe + dl_data_subframe_start;
#ifdef USE_LDPC
                        consumer_encode.schedule_task_set(total_data_subframe_id);
#else
                        consumer_precode.schedule_task_set(total_data_subframe_id);
#endif
                    }
                }
            } break;

            case EVENT_DEMUL: {
                int offset = event.data;
                int block_size = config_->demul_block_size;
                int block_num = demul_stats_.max_task_count;
                int sc_id = offset % block_num * block_size;
                int total_data_subframe_id = offset / block_num;
                int frame_id = total_data_subframe_id / ul_data_subframe_num_perframe;
                int data_subframe_id = total_data_subframe_id % ul_data_subframe_num_perframe;

                print_per_task_done(PRINT_DEMUL, frame_id, data_subframe_id, sc_id);
                /* if this subframe is ready */
                if (demul_stats_.last_task(frame_id, data_subframe_id)) {
                    max_equaled_frame = frame_id;
#ifdef USE_LDPC
                    consumer_decode.schedule_task_set(total_data_subframe_id);
#endif
                    print_per_subframe_done(PRINT_DEMUL, demul_stats_.frame_count, frame_id, data_subframe_id);
                    if (demul_stats_.last_symbol(frame_id)) {
                        /* schedule fft for the next frame if there are delayed fft tasks */
#ifndef USE_LDPC
#if !BIGSTATION
                        if (!schedule_delayed_fft_tasks(demul_stats_.frame_count, frame_id, data_subframe_id, consumer_fft))
                            cur_frame_id = (frame_id + 1) % TASK_BUFFER_FRAME_NUM;
#endif
                        stats_manager_->update_stats_in_functions_uplink(demul_stats_.frame_count);
                        if (stats_manager_->last_frame_id == config_->tx_frame_num - 1)
                            goto finish;
#endif
                        stats_manager_->update_demul_processed(demul_stats_.frame_count);
                        zf_stats_.precoder_exist_in_frame[frame_id] = false;
                        fft_stats_.symbol_data_count[frame_id] = 0;
                        print_per_frame_done(PRINT_DEMUL, demul_stats_.frame_count, frame_id);

                        demul_stats_.update_frame_count();
                    }
                    demul_count++;
                    if (demul_count == demul_stats_.max_symbol_count * 9000) {
                        demul_count = 0;
                        double diff = get_time() - demul_begin;
                        int samples_num_per_UE = OFDM_DATA_NUM * demul_stats_.max_symbol_count * 1000;
                        printf("Frame %d: Receive %d samples (per-client) from %d clients in %f secs, throughtput %f bps per-client (16QAM), current task queue length %zu\n",
                            demul_stats_.frame_count, samples_num_per_UE, UE_NUM, diff, samples_num_per_UE * log2(16.0f) / diff, fft_queue_.size_approx());
                        demul_begin = get_time();
                    }
                }
            } break;

#ifdef USE_LDPC
            case EVENT_DECODE: {
                int offset = event.data;
                int num_code_blocks = decode_stats_.max_task_count;
                int total_data_subframe_id = offset / num_code_blocks;
                int frame_id = total_data_subframe_id / ul_data_subframe_num_perframe;
                int data_subframe_id = total_data_subframe_id % ul_data_subframe_num_perframe;
                if (decode_stats_.last_task(frame_id, data_subframe_id)) {
                    print_per_subframe_done(PRINT_DECODE, decode_stats_.frame_count, frame_id, data_subframe_id);
                    if (decode_stats_.last_symbol(frame_id)) {
#if !BIGSTATION
                        if (!schedule_delayed_fft_tasks(decode_stats_.frame_count, frame_id, data_subframe_id, consumer_fft))
                            cur_frame_id = (frame_id + 1) % TASK_BUFFER_FRAME_NUM;
#endif
                        stats_manager_->update_decode_processed(decode_stats_.frame_count);
                        print_per_frame_done(PRINT_DECODE, decode_stats_.frame_count, frame_id);
                        stats_manager_->update_stats_in_functions_uplink(decode_stats_.frame_count);
                        if (stats_manager_->last_frame_id == config_->tx_frame_num - 1)
                            goto finish;
                        decode_stats_.update_frame_count();
                    }
                }
            } break;

            case EVENT_ENCODE: {
                int offset = event.data;
                int total_data_subframe_id = offset % TASK_BUFFER_SUBFRAME_NUM;
                int frame_id = total_data_subframe_id / data_subframe_num_perframe;
                int data_subframe_id = total_data_subframe_id % data_subframe_num_perframe;

                if (encode_stats_.last_task(frame_id, data_subframe_id)) {
                    consumer_precode.schedule_task_set(total_data_subframe_id);
                    print_per_subframe_done(PRINT_ENCODE, encode_stats_.frame_count, frame_id, data_subframe_id);
                    if (encode_stats_.last_symbol(frame_id)) {
                        stats_manager_->update_encode_processed(encode_stats_.frame_count);
                        print_per_frame_done(PRINT_ENCODE, encode_stats_.frame_count, frame_id);
                        encode_stats_.update_frame_count();
                    }
                }
            } break;
#endif /* defined(USE_LDPC) */

            case EVENT_PRECODE: {
                /* Precoding is done, schedule ifft */
                int offset = event.data;
                int block_size = config_->demul_block_size;
                int block_num = precode_stats_.max_task_count;
                int sc_id = offset % block_num * block_size;
                int total_data_subframe_id = offset / block_num;
                int data_subframe_id = total_data_subframe_id % data_subframe_num_perframe;
                int frame_id = total_data_subframe_id / data_subframe_num_perframe;

                print_per_task_done(PRINT_PRECODE, frame_id, data_subframe_id, sc_id);
                if (precode_stats_.last_task(frame_id, data_subframe_id)) {
                    int offset = precode_stats_.frame_count * data_subframe_num_perframe + data_subframe_id;
                    consumer_ifft.schedule_task_set(offset);
                    if (data_subframe_id < dl_data_subframe_end - 1) {
#ifdef USE_LDPC
                        consumer_encode.schedule_task_set(total_data_subframe_id);
#else
                        consumer_precode.schedule_task_set(total_data_subframe_id + 1);
#endif
                    }

                    print_per_subframe_done(PRINT_PRECODE, precode_stats_.frame_count, frame_id, data_subframe_id);
                    if (precode_stats_.last_symbol(frame_id)) {
                        stats_manager_->update_precode_processed(precode_stats_.frame_count);
                        print_per_frame_done(PRINT_PRECODE, precode_stats_.frame_count, frame_id);
                        precode_stats_.update_frame_count();
                    }
                }
            } break;
            case EVENT_IFFT: {
                /* IFFT is done, schedule data transmission */
                int offset = event.data;
                int ant_id = offset % BS_ANT_NUM;
                int total_data_subframe_id = offset / BS_ANT_NUM;
                int frame_id = total_data_subframe_id / data_subframe_num_perframe % TASK_BUFFER_FRAME_NUM;
                int data_subframe_id = total_data_subframe_id % data_subframe_num_perframe;
                int ptok_id = ant_id % SOCKET_RX_THREAD_NUM;
                Consumer consumer_tx(tx_queue_, *tx_ptoks_ptr[ptok_id], 1, TASK_SEND);
                consumer_tx.schedule_task_set(offset);
                print_per_task_done(PRINT_IFFT, frame_id, data_subframe_id, ant_id);

                if (ifft_stats_.last_task(frame_id, data_subframe_id)) {
                    if (ifft_stats_.last_symbol(frame_id)) {
#if !BIGSTATION
                        /* schedule fft for next frame */
                        if (!schedule_delayed_fft_tasks(ifft_stats_.frame_count, frame_id, data_subframe_id, consumer_fft))
                            cur_frame_id = (frame_id + 1) % TASK_BUFFER_FRAME_NUM;
#endif
                        stats_manager_->update_ifft_processed(ifft_stats_.frame_count);
                        print_per_frame_done(PRINT_IFFT, ifft_stats_.frame_count, frame_id);
                        ifft_stats_.update_frame_count();
                    }
                }
            } break;
            case EVENT_PACKET_SENT: {
                /* Data is sent */
                int offset = event.data;
                int ant_id = offset % BS_ANT_NUM;
                int total_data_subframe_id = offset / BS_ANT_NUM;
                int frame_id = total_data_subframe_id / data_subframe_num_perframe;
                int data_subframe_id = total_data_subframe_id % data_subframe_num_perframe;
                // printf("In main thread: tx finished for frame %d subframe %d ant %d\n", frame_id, data_subframe_id, ant_id);
                frame_id = frame_id % TASK_BUFFER_FRAME_NUM;

                print_per_task_done(PRINT_TX, frame_id, data_subframe_id, ant_id);
                if (tx_stats_.last_task(frame_id, data_subframe_id)) {
                    print_per_subframe_done(PRINT_TX, tx_stats_.frame_count, frame_id, data_subframe_id);
                    /* if tx of the first symbol is done */
                    if (data_subframe_id == dl_data_subframe_start) {
                        stats_manager_->update_tx_processed_first(tx_stats_.frame_count);
                        print_per_frame_done(PRINT_TX_FIRST, tx_stats_.frame_count, frame_id);
                    }
                    if (tx_stats_.last_symbol(frame_id)) {
                        stats_manager_->update_tx_processed(tx_stats_.frame_count);
                        print_per_frame_done(PRINT_TX, tx_stats_.frame_count, frame_id);
                        stats_manager_->update_stats_in_functions_downlink(tx_stats_.frame_count);
                        if (stats_manager_->last_frame_id == config_->tx_frame_num - 1)
                            goto finish;
                        tx_stats_.update_frame_count();
                    }
                    tx_count++;
                    if (tx_count == tx_stats_.max_symbol_count * 9000) {
                        tx_count = 0;
                        double diff = get_time() - tx_begin;
                        int samples_num_per_UE = OFDM_DATA_NUM * tx_stats_.max_symbol_count * 1000;
                        printf("Transmit %d samples (per-client) to %d clients in %f secs, throughtput %f bps per-client (16QAM), current tx queue length %zu\n",
                            samples_num_per_UE, UE_NUM, diff, samples_num_per_UE * log2(16.0f) / diff, tx_queue_.size_approx());
                        tx_begin = get_time();
                    }
                }
            } break;
            default:
                printf("Wrong event type in message queue!");
                exit(0);
            } /* end of switch */
        } /* end of for */
    } /* end of while */
finish:
    this->stop();
    printf("Total dequeue trials: %d, missed %d\n", total_count, miss_count);
    int last_frame_id = stats_manager_->last_frame_id;
    stats_manager_->save_to_file(last_frame_id, SOCKET_RX_THREAD_NUM);
    stats_manager_->print_summary(last_frame_id);
    save_demul_data_to_file(last_frame_id);
    //exit(0);
}

static void
pin_worker(thread_type thread, int tid, Config* config_)
{
    int SOCKET_RX_THREAD_NUM = config_->socket_thread_num;
    int CORE_OFFSET = config_->core_offset;
    int core_offset = SOCKET_RX_THREAD_NUM + CORE_OFFSET + 1;
    pin_to_core_with_offset(thread, core_offset, tid);
}

#if !BIGSTATION

void* Millipede::worker(int tid)
{
    pin_worker(Worker, tid, config_);
    moodycamel::ProducerToken ptok_complete(complete_task_queue_);
    Consumer consumer(complete_task_queue_, ptok_complete);

    /* initialize operators */
    auto computeFFT = new DoFFT(config_, tid, fft_queue_, consumer,
        socket_buffer_, socket_buffer_status_, data_buffer_, csi_buffer_, calib_buffer_,
        stats_manager_);

    auto computeIFFT = new DoIFFT(config_, tid, ifft_queue_, consumer,
        dl_ifft_buffer_, dl_socket_buffer_, stats_manager_);

    auto computeUpZF = new DoUpZF(config_, tid, zf_queue_, consumer,
        csi_buffer_, precoder_buffer_, stats_manager_);

    auto computeDnZF = new DoDnZF(config_, tid, zf_queue_, consumer,
        csi_buffer_, recip_buffer_, dl_precoder_buffer_, stats_manager_);

    auto computeDemul = new DoDemul(config_, tid, demul_queue_, consumer,
        data_buffer_, precoder_buffer_, equal_buffer_, demod_hard_buffer_, demod_soft_buffer_, stats_manager_);

    auto computePrecode = new DoPrecode(config_, tid, precode_queue_, consumer, dl_precoder_buffer_, dl_ifft_buffer_,
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
    auto* computeReciprocity = new Reciprocity(config_, tid, rc_queue_, consumer,
        calib_buffer_, recip_buffer_, stats_manager_);

    Doer* computers[] = {
        computeIFFT,
        computePrecode,
#ifdef USE_LDPC
        computeEncoding,
#endif
        computeDnZF,
        computeUpZF,
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

#else /* BIGSTATION */
void* Millipede::worker_fft(int tid)
{
    pin_worker(Worker_FFT, tid, config_);
    moodycamel::ProducerToken ptok_complete(complete_task_queue_);
    Consumer consumer(complete_task_queue_, ptok_complete);

    /* initialize IFFT operator */
    auto computeFFT = new DoFFT(config_, tid, fft_queue_, consumer,
        socket_buffer_, socket_buffer_status_, data_buffer_, csi_buffer_, calib_buffer_,
        stats_manager_);
    auto computeIFFT = new DoIFFT(config_, tid, ifft_queue_, consumer,
        dl_ifft_buffer_, dl_socket_buffer_, stats_manager_);

    while (true) {
        if (computeFFT->try_launch()) {
        } else if (config_->downlink_mode && computeIFFT->try_launch()) {
        }
    }
}

void* Millipede::worker_zf(int tid)
{
    pin_worker(Worker_ZF, tid, config_);
    moodycamel::ProducerToken ptok_complete(complete_task_queue_);
    Consumer consumer(complete_task_queue_, ptok_complete);

    /* initialize ZF operator */
    auto computeUpZF = new DoUpZF(config_, tid, zf_queue_, consumer,
        csi_buffer_, precoder_buffer_, stats_manager_);

    while (true) {
        computeUpZF->try_launch();
    }
}

void* Millipede::worker_demul(int tid)
{
    pin_worker(Worker_Demul, tid, config_);
    moodycamel::ProducerToken ptok_complete(complete_task_queue_);
    Consumer consumer(complete_task_queue_, ptok_complete);

    /* initialize Demul operator */
    auto computeDemul = new DoDemul(config_, tid, demul_queue_, consumer,
        data_buffer_, precoder_buffer_, equal_buffer_, demod_hard_buffer_, demod_soft_buffer_, stats_manager_);

    /* initialize Precode operator */
    auto computePrecode = new DoPrecode(config_, tid, precode_queue_, consumer,
        dl_precoder_buffer_, dl_ifft_buffer_,
#ifdef USE_LDPC
        dl_encoded_buffer_,
#else
        config_->dl_IQ_data,
#endif
        stats_manager_);

    // int cur_frame_id = 0;

    while (true) {
        if (config_->downlink_mode) {
            computePrecode->try_launch();
        } else {
            // int ul_data_subframe_num_perframe = config_->ul_data_symbol_num_perframe;
            // int frame_id = event.data / (OFDM_CA_NUM * ul_data_subframe_num_perframe);
            // // check precoder status for the current frame
            // if (frame_id > cur_frame_id || frame_id == 0) {
            //     while (!precoder_status_[frame_id]);
            // }
            computeDemul->try_launch();
        }
    }
}
#endif /* !BIGSTATION */

void Millipede::create_threads(thread_type thread, int tid_start, int tid_end)
{
    int ret;
    for (int i = tid_start; i < tid_end; i++) {
        auto context = new EventHandlerContext<Millipede>;
        context->obj_ptr = this;
        context->id = i;
        switch (thread) {
#if !BIGSTATION
        case Worker:
            ret = pthread_create(&task_threads[i], NULL, pthread_fun_wrapper<Millipede, &Millipede::worker>, context);
            break;
#else
        case Worker_FFT:
            ret = pthread_create(&task_threads[i], NULL, pthread_fun_wrapper<Millipede, &Millipede::worker_fft>, context);
            break;
        case Worker_ZF:
            ret = pthread_create(&task_threads[i], NULL, pthread_fun_wrapper<Millipede, &Millipede::worker_zf>, context);
            break;
        case Worker_Demul:
            ret = pthread_create(&task_threads[i], NULL, pthread_fun_wrapper<Millipede, &Millipede::worker_demul>, context);
            break;
#endif
        default:
            printf("ERROR: Wrong thread type to create workers\n");
            exit(0);
        }
        if (ret != 0) {
            perror("task thread create failed");
            exit(0);
        }
    }
}

void Millipede::schedule_fft_task(int offset, int frame_count,
    UNUSED int frame_id, UNUSED int subframe_id, UNUSED int ant_id,
    Consumer const& consumer)
{
    Event_data do_fft_task;
    do_fft_task.event_type = TASK_FFT;
    do_fft_task.data = offset;
    consumer.try_handle(do_fft_task);
#if DEBUG_PRINT_PER_TASK_ENTER_QUEUE
    printf("Main thread: created FFT tasks for frame: %d, frame buffer: %d, subframe: %d, ant: %d\n",
        frame_count, frame_id, subframe_id, ant_id);
#endif
    if (rx_stats_.fft_created_count++ == 0) {
        stats_manager_->update_processing_started(frame_count);
    } else if (rx_stats_.fft_created_count == rx_stats_.max_task_count) {
        rx_stats_.fft_created_count = 0;
#if DEBUG_PRINT_PER_FRAME_ENTER_QUEUE
        printf("Main thread: created FFT tasks for all packets in frame: %d, frame buffer: %d in %.5f us\n",
            frame_count, frame_id, get_time() - stats_manager_->get_pilot_received(frame_count));
#endif
    }
}

#if !BIGSTATION
bool Millipede::schedule_delayed_fft_tasks(int frame_count, int frame_id, int data_subframe_id,
    Consumer const& consumer)
{
    frame_id = (frame_id + 1) % TASK_BUFFER_FRAME_NUM;
    if (delay_fft_queue_cnt[frame_id] > 0) {
        for (int i = 0; i < delay_fft_queue_cnt[frame_id]; i++) {
            int offset_rx = delay_fft_queue[frame_id][i];
            schedule_fft_task(offset_rx, frame_count + 1, frame_id, data_subframe_id, 0, consumer);
        }
        delay_fft_queue_cnt[frame_id] = 0;
#if DEBUG_PRINT_PER_FRAME_ENTER_QUEUE
        if (config_->downlink_mode)
            printf("Main thread in IFFT: schedule fft for %d packets for frame %d is done\n", delay_fft_queue_cnt[frame_id], frame_id);
        else
            printf("Main thread in demul: schedule fft for %d packets for frame %d is done\n", delay_fft_queue_cnt[frame_id], frame_id);
#endif
        return (rx_stats_.fft_created_count == 0);
    }
    return (false);
}
#endif /* !BIGSTATION */

void Millipede::schedule_demul_task(int frame_id, int start_subframe_id, int end_subframe_id, Consumer const& consumer)
{
    int data_subframe_num_perframe = config_->ul_data_symbol_num_perframe;
    for (int data_subframe_id = start_subframe_id; data_subframe_id < end_subframe_id; data_subframe_id++) {
        if (fft_stats_.data_exist_in_symbol[frame_id][data_subframe_id]) {
            /* schedule demodulation task for subcarrier blocks */
            int total_data_subframe_id = frame_id * data_subframe_num_perframe + data_subframe_id;
            consumer.schedule_task_set(total_data_subframe_id);
#if DEBUG_PRINT_PER_SUBFRAME_ENTER_QUEUE
            printf("Main thread: created Demodulation task for frame: %d,, start subframe: %d, current subframe: %d\n",
                frame_id, start_subframe_id, data_subframe_id);
#endif
            /* clear data status after scheduling */
            fft_stats_.data_exist_in_symbol[frame_id][data_subframe_id] = false;
        }
    }
}

void Millipede::update_rx_counters(int frame_count, int frame_id, int subframe_id)
{
    if (config_->isPilot(frame_count, subframe_id)) {
        if (++rx_stats_.task_pilot_count[frame_id] == rx_stats_.max_task_pilot_count) {
            rx_stats_.task_pilot_count[frame_id] = 0;
            stats_manager_->update_pilot_all_received(frame_count);
            print_per_frame_done(PRINT_RX_PILOTS, frame_count, frame_id);
        }
    }
    if (rx_stats_.task_count[frame_id]++ == 0) {
        stats_manager_->update_pilot_received(frame_count);
#if DEBUG_PRINT_PER_FRAME_START
        int prev_frame_id = (frame_count + TASK_BUFFER_FRAME_NUM - 1) % TASK_BUFFER_FRAME_NUM;
        printf("Main thread: data received from frame %d, subframe %d, in %.5f us, rx in prev frame: %d\n",
            frame_count, subframe_id,
            stats_manager_->get_pilot_received(frame_count) - stats_manager_->get_pilot_received(frame_count - 1),
            rx_stats_.task_count[prev_frame_id]);
#endif
    } else if (rx_stats_.task_count[frame_id] == rx_stats_.max_task_count) {
        stats_manager_->update_rx_processed(frame_count);
        print_per_frame_done(PRINT_RX, frame_count, frame_id);
        rx_stats_.task_count[frame_id] = 0;
    }
}

void Millipede::print_per_frame_done(int task_type, int frame_count, int frame_id)
{
    int dl_data_subframe_num_perframe = config_->dl_data_symbol_num_perframe;
    int ul_data_subframe_num_perframe = config_->ul_data_symbol_num_perframe;
#if DEBUG_PRINT_PER_FRAME_DONE
    switch (task_type) {
    case (PRINT_RX): {
        int prev_frame_count = (frame_count - 1) % TASK_BUFFER_FRAME_NUM;
        int fft_sym = fft_stats_.symbol_data_count[frame_id];
        int UE_NUM = config_->UE_NUM;
        printf("Main thread: received all packets in frame: %d, frame buffer: %d in %.2f us, demul: %d done, FFT: %d,%d, rx in prev frame: %d\n",
            frame_count, frame_id, stats_manager_->get_rx_processed(frame_count) - stats_manager_->get_pilot_received(frame_count),
            demul_stats_.symbol_count[frame_id],
            fft_sym, fft_stats_.task_count[frame_id][fft_sym + UE_NUM],
            rx_stats_.task_count[prev_frame_count]);
    } break;
    case (PRINT_RX_PILOTS):
        printf("Main thread: received all pilots in frame: %d, frame buffer: %d in %.2f us\n", frame_count, frame_id,
            stats_manager_->get_pilot_all_received(frame_count) - stats_manager_->get_pilot_received(frame_count));
        break;
    case (PRINT_FFT_PILOTS):
        printf("Main thread: pilot frame: %d, %d, finished FFT for all pilot subframes in %.2f us, pilot all received: %.2f\n",
            frame_count, frame_id,
            stats_manager_->get_fft_processed(frame_count) - stats_manager_->get_pilot_received(frame_count),
            stats_manager_->get_pilot_all_received(frame_count) - stats_manager_->get_pilot_received(frame_count));
        break;
    case (PRINT_FFT_DATA):
        printf("Main thread: data frame: %d, %d, finished FFT for all data subframes in %.2f us\n",
            frame_count, frame_id,
            get_time() - stats_manager_->get_pilot_received(frame_count));
        break;
    case (PRINT_FFT_CAL):
        printf("Main thread: cal frame: %d, %d, finished FFT for all cal subframes in %.2f us\n",
            frame_count, frame_id,
            get_time() - stats_manager_->get_pilot_received(frame_count));
        break;
    case (PRINT_ZF):
        printf("Main thread: ZF done frame: %d, %d in %.2f us since pilot FFT done, total: %.2f us\n",
            frame_count, frame_id,
            stats_manager_->get_zf_processed(frame_count) - stats_manager_->get_fft_processed(frame_count),
            stats_manager_->get_zf_processed(frame_count) - stats_manager_->get_pilot_received(frame_count));
        break;
    case (PRINT_DEMUL):
        printf("Main thread: Demodulation done frame: %d, %d (%d UL subframes) in %.2f us since ZF done, total %.2f us\n",
            frame_count, frame_id, ul_data_subframe_num_perframe,
            stats_manager_->get_demul_processed(frame_count) - stats_manager_->get_zf_processed(frame_count),
            stats_manager_->get_demul_processed(frame_count) - stats_manager_->get_pilot_received(frame_count));
        break;
    case (PRINT_DECODE):
        printf("Main thread: Decoding done frame: %d, %d (%d UL subframes) in %.2f us since ZF done, total %.2f us\n",
            frame_count, frame_id, ul_data_subframe_num_perframe,
            stats_manager_->get_decode_processed(frame_count) - stats_manager_->get_zf_processed(frame_count),
            stats_manager_->get_decode_processed(frame_count) - stats_manager_->get_pilot_received(frame_count));
        break;
    case (PRINT_ENCODE):
        printf("Main thread: Encoding done frame: %d, %d in %.2f us since ZF done, total %.2f us\n",
            frame_count, frame_id,
            stats_manager_->get_encode_processed(frame_count) - stats_manager_->get_zf_processed(frame_count),
            stats_manager_->get_encode_processed(frame_count) - stats_manager_->get_pilot_received(frame_count));
        break;
    case (PRINT_PRECODE):
        printf("Main thread: Precoding done frame: %d, %d in %.2f us since ZF done, total: %.2f us\n",
            frame_count, frame_id,
            stats_manager_->get_precode_processed(frame_count) - stats_manager_->get_zf_processed(frame_count),
            stats_manager_->get_precode_processed(frame_count) - stats_manager_->get_pilot_received(frame_count));
        break;
    case (PRINT_RC):
        printf("Main thread: Reciprocity Calculation done frame: %d, %d in %.2f us since pilot FFT done, total: %.2f us\n",
            frame_count, frame_id,
            stats_manager_->get_rc_processed(frame_id) - stats_manager_->get_fft_processed(frame_id),
            stats_manager_->get_rc_processed(frame_id) - stats_manager_->get_pilot_received(frame_id));
        break;
    case (PRINT_IFFT):
        printf("Main thread: IFFT done frame: %d, %d in %.2f us since precode done, total: %.2f us\n",
            frame_count, frame_id,
            stats_manager_->get_ifft_processed(frame_count) - stats_manager_->get_precode_processed(frame_count),
            stats_manager_->get_ifft_processed(frame_count) - stats_manager_->get_pilot_received(frame_count));
        break;
    case (PRINT_TX_FIRST):
        printf("Main thread: TX of first subframe done frame: %d, %d in %.2f us since ZF done, total: %.2f us\n",
            frame_count, frame_id,
            stats_manager_->get_tx_processed_first(frame_count) - stats_manager_->get_zf_processed(frame_count),
            stats_manager_->get_tx_processed_first(frame_count) - stats_manager_->get_pilot_received(frame_count));
        break;
    case (PRINT_TX):
        printf("Main thread: TX done frame: %d %d (%d DL subframes) in %.2f us since ZF done, total: %.2f us\n",
            frame_count, frame_id, dl_data_subframe_num_perframe,
            stats_manager_->get_tx_processed(frame_count) - stats_manager_->get_zf_processed(frame_count),
            stats_manager_->get_tx_processed(frame_count) - stats_manager_->get_pilot_received(frame_count));
        break;
    default:
        printf("Wrong task type in frame done print!");
    }
#endif
}

void Millipede::print_per_subframe_done(UNUSED int task_type, UNUSED int frame_count, UNUSED int frame_id, UNUSED int subframe_id)
{
#if DEBUG_PRINT_PER_SUBFRAME_DONE
    switch (task_type) {
    case (PRINT_FFT_PILOTS):
        printf("Main thread: pilot FFT done frame: %d, %d, subframe: %d, num sumbframes done: %d\n",
            frame_count, frame_id, subframe_id, fft_stats_.symbol_count[frame_id]);
        break;
    case (PRINT_FFT_DATA):
        printf("Main thread: data FFT done frame %d, %d, subframe %d, precoder status: %d, fft queue: %d, zf queue: %d, demul queue: %d, in %.2f\n",
            frame_count, frame_id, subframe_id, zf_stats_.precoder_exist_in_frame[frame_id],
            fft_queue_.size_approx(), zf_queue_.size_approx(), demul_queue_.size_approx(),
            get_time() - stats_manager_->get_pilot_received(frame_count));
        break;
    case (PRINT_RC):
        printf("Main thread: cal symbol FFT done frame: %d, %d, subframe: %d, num sumbframes done: %d\n",
            frame_count, frame_id, subframe_id, fft_stats_.symbol_cal_count[frame_id]);
        break;
    case (PRINT_DEMUL):
        printf("Main thread: Demodulation done frame %d %d, subframe: %d, num sumbframes done: %d\n",
            frame_count, frame_id, subframe_id, demul_stats_.symbol_count[frame_id]);
        break;
#ifdef USE_LDPC
    case (PRINT_DECODE):
        printf("Main thread: Decoding done frame %d %d, subframe: %d, num sumbframes done: %d\n",
            frame_count, frame_id, subframe_id, decode_stats_.symbol_count[frame_id]);
        break;
    case (PRINT_ENCODE):
        printf("Main thread: Encoding done frame %d %d, subframe: %d, num sumbframes done: %d\n",
            frame_count, frame_id, subframe_id, encode_stats_.symbol_count[frame_id]);
        break;
#endif
    case (PRINT_PRECODE):
        printf("Main thread: Precoding done frame: %d %d, subframe: %d in %.2f us\n",
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

void Millipede::print_per_task_done(UNUSED int task_type, UNUSED int frame_id, UNUSED int subframe_id, UNUSED int ant_or_sc_id)
{
#if DEBUG_PRINT_PER_TASK_DONE
    switch (task_type) {
    case (PRINT_ZF):
        printf("Main thread: ZF done frame: %d, subcarrier %d\n", frame_id, ant_or_sc_id);
        break;
    case (PRINT_RC):
        printf("Main thread: RC done frame: %d, subcarrier %d\n", frame_id, ant_or_sc_id);
        break;
    case (PRINT_DEMUL):
        printf("Main thread: Demodulation done frame: %d, subframe: %d, sc: %d, num blocks done: %d\n",
            frame_id, subframe_id, ant_or_sc_id, demul_stats_.task_count[frame_id][subframe_id]);
        break;
#ifdef USE_LDPC
    case (PRINT_DECODE):
        printf("Main thread: Decoding done frame: %d, subframe: %d, sc: %d, num blocks done: %d\n",
            frame_id, subframe_id, ant_or_sc_id, decode_stats_.task_count[frame_id][subframe_id]);
        break;
#endif
    case (PRINT_PRECODE):
        printf("Main thread: Precoding done frame: %d, subframe: %d, subcarrier: %d, total SCs: %d\n",
            frame_id, subframe_id, ant_or_sc_id, precode_stats_.task_count[frame_id][subframe_id]);
        break;
    case (PRINT_IFFT):
        printf("Main thread: IFFT done frame: %d, subframe: %d, antenna: %d, total ants: %d\n",
            frame_id, subframe_id, ant_or_sc_id, ifft_stats_.task_count[frame_id][subframe_id]);
        break;
    case (PRINT_TX):
        printf("Main thread: TX done frame: %d, subframe: %d, antenna: %d, total packets: %d\n",
            frame_id, subframe_id, ant_or_sc_id, tx_stats_.task_count[frame_id][subframe_id]);
        break;
    default:
        printf("Wrong task type in frame done print!");
    }

#endif
}

void Millipede::initialize_queues()
{
    int data_subframe_num_perframe = config_->data_symbol_num_perframe;
    message_queue_ = moodycamel::ConcurrentQueue<Event_data>(512 * data_subframe_num_perframe);
    complete_task_queue_ = moodycamel::ConcurrentQueue<Event_data>(512 * data_subframe_num_perframe * 4);

    fft_queue_ = moodycamel::ConcurrentQueue<Event_data>(512 * data_subframe_num_perframe * 4);
    zf_queue_ = moodycamel::ConcurrentQueue<Event_data>(512 * data_subframe_num_perframe * 4);
    ;
    rc_queue_ = moodycamel::ConcurrentQueue<Event_data>(512 * 2 * 4);
    ;
    demul_queue_ = moodycamel::ConcurrentQueue<Event_data>(512 * data_subframe_num_perframe * 4);
#ifdef USE_LDPC
    decode_queue_ = moodycamel::ConcurrentQueue<Event_data>(512 * data_subframe_num_perframe * 4);
#endif

    ifft_queue_ = moodycamel::ConcurrentQueue<Event_data>(512 * data_subframe_num_perframe * 4);
    // modulate_queue_ = moodycamel::ConcurrentQueue<Event_data>(512 * data_subframe_num_perframe * 4);
#ifdef USE_LDPC
    encode_queue_ = moodycamel::ConcurrentQueue<Event_data>(512 * data_subframe_num_perframe * 4);
#endif
    precode_queue_ = moodycamel::ConcurrentQueue<Event_data>(512 * data_subframe_num_perframe * 4);
    tx_queue_ = moodycamel::ConcurrentQueue<Event_data>(512 * data_subframe_num_perframe * 4);

    int SOCKET_RX_THREAD_NUM = config_->socket_thread_num;
    rx_ptoks_ptr = (moodycamel::ProducerToken**)aligned_alloc(64, SOCKET_RX_THREAD_NUM * sizeof(moodycamel::ProducerToken*));
    for (int i = 0; i < SOCKET_RX_THREAD_NUM; i++)
        rx_ptoks_ptr[i] = new moodycamel::ProducerToken(message_queue_);

    tx_ptoks_ptr = (moodycamel::ProducerToken**)aligned_alloc(64, SOCKET_RX_THREAD_NUM * sizeof(moodycamel::ProducerToken*));
    for (int i = 0; i < SOCKET_RX_THREAD_NUM; i++)
        tx_ptoks_ptr[i] = new moodycamel::ProducerToken(tx_queue_);
}

void Millipede::initialize_uplink_buffers()
{
    int data_subframe_num_perframe = config_->data_symbol_num_perframe;
    int ul_data_subframe_num_perframe = config_->ul_data_symbol_num_perframe;
    int TASK_BUFFER_SUBFRAME_NUM = ul_data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM;

    int TASK_THREAD_NUM = config_->worker_thread_num;
    alloc_buffer_1d(&task_threads, TASK_THREAD_NUM, 64, 0);
    // task_threads = (pthread_t *)malloc(TASK_THREAD_NUM * sizeof(pthread_t));
    // context = (EventHandlerContext *)malloc(TASK_THREAD_NUM * sizeof(EventHandlerContext));

    int BS_ANT_NUM = config_->BS_ANT_NUM;
    int packet_length = config_->packet_length;
    int subframe_num_perframe = config_->symbol_num_perframe;
    int SOCKET_RX_THREAD_NUM = config_->socket_thread_num;
    int PILOT_NUM = config_->pilot_symbol_num_perframe;
    int OFDM_DATA_NUM = config_->OFDM_DATA_NUM;
    int UE_NUM = config_->UE_NUM;

    socket_buffer_status_size_ = BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM * subframe_num_perframe;
    socket_buffer_size_ = (long long)packet_length * socket_buffer_status_size_;
    printf("socket_buffer_size %lld, socket_buffer_status_size %d\n", socket_buffer_size_, socket_buffer_status_size_);
    socket_buffer_.malloc(SOCKET_RX_THREAD_NUM, socket_buffer_size_, 64);
    socket_buffer_status_.calloc(SOCKET_RX_THREAD_NUM, socket_buffer_status_size_, 64);

    csi_buffer_.malloc(PILOT_NUM * TASK_BUFFER_FRAME_NUM, BS_ANT_NUM * OFDM_DATA_NUM, 64);
    data_buffer_.malloc(TASK_BUFFER_SUBFRAME_NUM, BS_ANT_NUM * OFDM_DATA_NUM, 64);
    precoder_buffer_.malloc(OFDM_DATA_NUM * TASK_BUFFER_FRAME_NUM, BS_ANT_NUM * UE_NUM, 64);

    equal_buffer_.malloc(TASK_BUFFER_SUBFRAME_NUM, OFDM_DATA_NUM * UE_NUM, 64);
    demod_hard_buffer_.malloc(TASK_BUFFER_SUBFRAME_NUM, OFDM_DATA_NUM * UE_NUM, 64);
    size_t mod_type = config_->mod_type;
    demod_soft_buffer_.malloc(TASK_BUFFER_SUBFRAME_NUM, mod_type * OFDM_DATA_NUM * UE_NUM, 64);
    decoded_buffer_.malloc(TASK_BUFFER_SUBFRAME_NUM, OFDM_DATA_NUM * UE_NUM, 64);

    int max_packet_num_per_frame = BS_ANT_NUM * (PILOT_NUM + (config_->downlink_mode ? 0 : ul_data_subframe_num_perframe));
    rx_stats_.max_task_count = max_packet_num_per_frame;
    rx_stats_.max_task_pilot_count = BS_ANT_NUM * PILOT_NUM;
    alloc_buffer_1d(&(rx_stats_.task_count), TASK_BUFFER_FRAME_NUM, 64, 1);
    alloc_buffer_1d(&(rx_stats_.task_pilot_count), TASK_BUFFER_FRAME_NUM, 64, 1);
    rx_stats_.fft_created_count = 0;
    fft_stats_.init(BS_ANT_NUM, PILOT_NUM,
        TASK_BUFFER_FRAME_NUM, subframe_num_perframe, 64);
    alloc_buffer_1d(&(fft_stats_.symbol_data_count), TASK_BUFFER_FRAME_NUM, 64, 1);
    fft_stats_.max_symbol_data_count = ul_data_subframe_num_perframe;
    alloc_buffer_1d(&(fft_stats_.symbol_cal_count), TASK_BUFFER_FRAME_NUM, 64, 1);
    fft_stats_.max_symbol_cal_count = 2;
    fft_stats_.data_exist_in_symbol.calloc(TASK_BUFFER_FRAME_NUM, ul_data_subframe_num_perframe, 64);

    zf_stats_.init(config_->zf_block_num, TASK_BUFFER_FRAME_NUM, 1);

    demul_stats_.init(config_->demul_block_num, ul_data_subframe_num_perframe,
        TASK_BUFFER_FRAME_NUM, data_subframe_num_perframe, 64);

#ifdef USE_LDPC
    decode_stats_.init(config_->LDPC_config.nblocksInSymbol * UE_NUM, ul_data_subframe_num_perframe,
        TASK_BUFFER_FRAME_NUM, data_subframe_num_perframe, 64);
#endif

#if !BIGSTATION
    delay_fft_queue.calloc(TASK_BUFFER_FRAME_NUM, subframe_num_perframe * BS_ANT_NUM, 32);
    alloc_buffer_1d(&delay_fft_queue_cnt, TASK_BUFFER_FRAME_NUM, 32, 1);
#endif
}

void Millipede::initialize_downlink_buffers()
{
    int data_subframe_num_perframe = config_->data_symbol_num_perframe;
    int TASK_BUFFER_SUBFRAME_NUM = data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM;
    int dl_data_subframe_num_perframe = config_->dl_data_symbol_num_perframe;
    int BS_ANT_NUM = config_->BS_ANT_NUM;
    int packet_length = config_->packet_length;
    size_t OFDM_CA_NUM = config_->OFDM_CA_NUM;
    int OFDM_DATA_NUM = config_->OFDM_DATA_NUM;
    int UE_NUM = config_->UE_NUM;

    dl_socket_buffer_status_size_ = BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM * data_subframe_num_perframe;
    dl_socket_buffer_size_ = (long long)packet_length * dl_socket_buffer_status_size_;
    alloc_buffer_1d(&dl_socket_buffer_, dl_socket_buffer_size_, 64, 0);
    alloc_buffer_1d(&dl_socket_buffer_status_, dl_socket_buffer_status_size_, 64, 1);
    dl_ifft_buffer_.calloc(BS_ANT_NUM * TASK_BUFFER_SUBFRAME_NUM, OFDM_CA_NUM, 64);
    dl_precoder_buffer_.malloc(OFDM_DATA_NUM * TASK_BUFFER_FRAME_NUM, UE_NUM * BS_ANT_NUM, 64);
    dl_encoded_buffer_.malloc(TASK_BUFFER_SUBFRAME_NUM, OFDM_DATA_NUM * UE_NUM, 64);
    recip_buffer_.malloc(TASK_BUFFER_FRAME_NUM, OFDM_DATA_NUM * BS_ANT_NUM, 64);
    calib_buffer_.malloc(TASK_BUFFER_FRAME_NUM, OFDM_DATA_NUM * BS_ANT_NUM, 64);

#ifdef USE_LDPC
    encode_stats_.init(config_->LDPC_config.nblocksInSymbol * UE_NUM, dl_data_subframe_num_perframe,
        TASK_BUFFER_FRAME_NUM, data_subframe_num_perframe, 64);
#endif
    precode_stats_.init(config_->demul_block_num, dl_data_subframe_num_perframe,
        TASK_BUFFER_FRAME_NUM, data_subframe_num_perframe, 64);

    ifft_stats_.init(BS_ANT_NUM, dl_data_subframe_num_perframe,
        TASK_BUFFER_FRAME_NUM, data_subframe_num_perframe, 64);

    tx_stats_.init(BS_ANT_NUM, dl_data_subframe_num_perframe,
        TASK_BUFFER_FRAME_NUM, data_subframe_num_perframe, 64);
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
    fft_stats_.data_exist_in_symbol.free();
    free_buffer_1d(&(fft_stats_.symbol_data_count));
    zf_stats_.fini();
    demul_stats_.fini();
#ifdef USE_LDPC
    decode_stats_.fini();
#endif

#if !BIGSTATION
    delay_fft_queue.free();
    free_buffer_1d(&delay_fft_queue_cnt);
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
    int data_subframe_num_perframe = config_->ul_data_symbol_num_perframe;
    int UE_NUM = config_->UE_NUM;
    int OFDM_DATA_NUM = config_->OFDM_DATA_NUM;
    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string filename = cur_directory + "/data/demul_data.bin";
    FILE* fp = fopen(filename.c_str(), "wb");
    for (int i = 0; i < data_subframe_num_perframe; i++) {
        int total_data_subframe_id = (frame_id % TASK_BUFFER_FRAME_NUM) * data_subframe_num_perframe + i;
        for (int sc = 0; sc < OFDM_DATA_NUM; sc++) {
            uint8_t* ptr = &demod_hard_buffer_[total_data_subframe_id][sc * UE_NUM];
            fwrite(ptr, UE_NUM, sizeof(uint8_t), fp);
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
    *ptr = (int*)&demod_hard_buffer_[max_equaled_frame * ul_data_subframe_num_perframe][0];
    *size = UE_NUM * OFDM_DATA_NUM;
}

void Millipede::getEqualData(float** ptr, int* size)
{
    int ul_data_subframe_num_perframe = config_->ul_data_symbol_num_perframe;
    int OFDM_DATA_NUM = config_->OFDM_DATA_NUM;
    int UE_NUM = config_->UE_NUM;
    // max_equaled_frame = 0;
    *ptr = (float*)&equal_buffer_[max_equaled_frame * ul_data_subframe_num_perframe][0];
    // *ptr = equal_output;
    *size = UE_NUM * OFDM_DATA_NUM * 2;

    //printf("In getEqualData()\n");
    //for(int ii = 0; ii < UE_NUM*OFDM_DATA_NUM; ii++)
    //{
    //    // printf("User %d: %d, ", ii,demul_ptr2(ii));
    //    printf("[%.4f+j%.4f] ", *(*ptr+ii*UE_NUM*2), *(*ptr+ii*UE_NUM*2+1));
    //}
    //printf("\n");
    //printf("\n");
}

extern "C" {
EXPORT Millipede* Millipede_new(Config* cfg)
{
    // printf("Size of Millipede: %d\n",sizeof(Millipede *));
    Millipede* millipede = new Millipede(cfg);

    return millipede;
}
EXPORT void Millipede_start(Millipede* millipede) { millipede->start(); }
EXPORT void Millipede_stop(/*Millipede *millipede*/) { SignalHandler::setExitSignal(true); /*millipede->stop();*/ }
EXPORT void Millipede_destroy(Millipede* millipede) { delete millipede; }
EXPORT void Millipede_getEqualData(Millipede* millipede, float** ptr, int* size) { return millipede->getEqualData(ptr, size); }
EXPORT void Millipede_getDemulData(Millipede* millipede, int** ptr, int* size) { return millipede->getDemulData(ptr, size); }
}
