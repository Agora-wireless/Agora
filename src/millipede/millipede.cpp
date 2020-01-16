/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 * 
 */
#include "millipede.hpp"
#include "Consumer.hpp"

// typedef cx_float COMPLEX;
bool keep_running = true;

void intHandler(int)
{
    std::cout << "will exit..." << std::endl;
    keep_running = false;
}

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
    initialize_vars_from_cfg(config_);

    pin_to_core_with_offset(Master, CORE_OFFSET, 0);

    initialize_queues();

    printf("initialize uplink buffers\n");
    initialize_uplink_buffers();

    if (downlink_mode) {
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
    if (downlink_mode)
        free_downlink_buffers();
}

void Millipede::stop()
{
    std::cout << "stopping threads " << std::endl;
    config_->running = false;
    usleep(1000);
    receiver_.reset();
}

static void
schedule_task_set(int task_type, int num_tasks, int total_data_subframe_id,
    int TASK_BUFFER_SUBFRAME_NUM, Consumer const& consumer)
{
    Event_data do_task;
    do_task.event_type = task_type;
    do_task.data = total_data_subframe_id;
    for (int i = 0; i < num_tasks; i++) {
        consumer.try_handle(do_task);
        do_task.data += TASK_BUFFER_SUBFRAME_NUM;
    }
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

    /* start downlink transmitter */
    std::vector<pthread_t> tx_threads;
    if (downlink_mode) {
        std::vector<pthread_t> tx_threads = receiver_->startTX(dl_socket_buffer_,
            dl_socket_buffer_status_, dl_socket_buffer_status_size_, dl_socket_buffer_size_);
    }

    /* tokens used for enqueue */
    /* uplink */
    moodycamel::ProducerToken ptok_fft(fft_queue_);
    Consumer consumer_fft(fft_queue_, ptok_fft);
    moodycamel::ProducerToken ptok_zf(zf_queue_);
    Consumer consumer_zf(zf_queue_, ptok_zf);
    moodycamel::ProducerToken ptok_demul(demul_queue_);
    Consumer consumer_demul(demul_queue_, ptok_demul);
    moodycamel::ProducerToken ptok_decode(decode_queue_);
    Consumer consumer_decode(decode_queue_, ptok_decode);
    /* downlink */
    moodycamel::ProducerToken ptok_encode(encode_queue_);
    Consumer consumer_encode(encode_queue_, ptok_encode);
    moodycamel::ProducerToken ptok_ifft(ifft_queue_);
    Consumer consumer_ifft(ifft_queue_, ptok_ifft);
    moodycamel::ProducerToken ptok_precode(precode_queue_);
    Consumer consumer_precode(precode_queue_, ptok_precode);

    /* tokens used for dequeue */
    moodycamel::ConsumerToken ctok(message_queue_);
    moodycamel::ConsumerToken ctok_complete(complete_task_queue_);

#ifdef USE_LDPC
    prev_frame_counter = downlink_mode ? ifft_stats_.symbol_count : decode_stats_.symbol_count;
#else
    prev_frame_counter = downlink_mode ? ifft_stats_.symbol_count : demul_stats_.symbol_count;
#endif
    prev_frame_counter_max = downlink_mode ? ifft_stats_.max_symbol_count : demul_stats_.max_symbol_count;

    /* counters for printing summary */
    int demul_count = 0;
    int tx_count = 0;
    double demul_begin = get_time();
    double tx_begin = get_time();

    bool prev_demul_scheduled = false;
    int data_subframe_num_perframe = config_->data_symbol_num_perframe;
    int TASK_BUFFER_SUBFRAME_NUM = data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM;

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
                bool previous_frame_done = prev_frame_counter[(frame_id + TASK_BUFFER_FRAME_NUM - 1) % TASK_BUFFER_FRAME_NUM] == prev_frame_counter_max;
                /* if this is the first frame or the previous frame is all processed, schedule FFT for this packet */
                if ((frame_count == 0 && fft_stats_.frame_count < 100) || (fft_stats_.frame_count > 0 && previous_frame_done)) {
                    schedule_fft_task(offset, frame_count, frame_id, subframe_id, ant_id, consumer_fft);
                } else {
                    /* if the previous frame is not finished, store offset in queue */
                    delay_fft_queue[frame_id][delay_fft_queue_cnt[frame_id]] = offset;
                    delay_fft_queue_cnt[frame_id]++;
                }
#endif
            } break;
            case EVENT_FFT: {
                int offset = event.data;
                int frame_id = offset / data_subframe_num_perframe;
                int subframe_id = offset % data_subframe_num_perframe;
                if (fft_stats_.last_task(frame_id, subframe_id)) {
                    if (config_->isPilot(frame_id, subframe_id)) {
                        print_per_subframe_done(PRINT_FFT_PILOTS, fft_stats_.frame_count, frame_id, subframe_id);
                        /* if csi of all UEs is ready, schedule ZF or prediction */
                        if (fft_stats_.last_symbol(frame_id)) {
                            stats_manager_->update_fft_processed(fft_stats_.frame_count);
                            print_per_frame_done(PRINT_FFT_PILOTS, fft_stats_.frame_count, frame_id);
                            update_frame_count(&(fft_stats_.frame_count));
                            schedule_zf_task(frame_id, consumer_zf);
                        }
                    } else if (config_->isUplink(frame_id, subframe_id)) {
                        fft_stats_.data_exist_in_symbol[frame_id][subframe_id - PILOT_NUM] = true;
                        fft_stats_.symbol_data_count[frame_id]++;
                        print_per_subframe_done(PRINT_FFT_DATA, fft_stats_.frame_count - 1, frame_id, subframe_id);
                        if (fft_stats_.symbol_data_count[frame_id] == fft_stats_.max_symbol_data_count) {
                            print_per_frame_done(PRINT_FFT_DATA, fft_stats_.frame_count - 1, frame_id);
                            prev_demul_scheduled = false;
                        }
                        /* if precoder exist, schedule demodulation */
                        if (zf_stats_.precoder_exist_in_frame[frame_id]) {
                            int start_subframe_id, end_subframe_id;
                            start_subframe_id = subframe_id - PILOT_NUM;
                            end_subframe_id = fft_stats_.symbol_data_count[frame_id];
                            if (end_subframe_id < start_subframe_id)
                                end_subframe_id = start_subframe_id + 1;
                            if (!prev_demul_scheduled) {
                                start_subframe_id = 0;
                                prev_demul_scheduled = true;
                            }
                            schedule_demul_task(frame_id, start_subframe_id, end_subframe_id, consumer_demul);
                        }
                    }
                }
            } break;
            case EVENT_ZF: {
                int offset = event.data;
                int frame_id = offset / data_subframe_num_perframe;
                // print_per_task_done(PRINT_ZF, frame_id, 0, sc_id);
                if (zf_stats_.last_task(frame_id)) {
                    stats_manager_->update_zf_processed(zf_stats_.frame_count);
                    zf_stats_.precoder_exist_in_frame[frame_id] = true;
                    print_per_frame_done(PRINT_ZF, zf_stats_.frame_count, frame_id);
                    update_frame_count(&(zf_stats_.frame_count));
                    /* if all the data in a frame has arrived when ZF is done */
                    if (fft_stats_.symbol_data_count[frame_id] == fft_stats_.max_symbol_data_count)
                        schedule_demul_task(frame_id, 0, config_->symbol_num_perframe - PILOT_NUM, consumer_demul);
                    if (downlink_mode) {
/* if downlink data transmission is enabled, schedule downlink encode/modulation for the first data subframe */
#ifdef USE_LDPC
                        int num_tasks = UE_NUM * LDPC_config.nblocksInSymbol;
                        schedule_task_set(TASK_ENCODE, num_tasks,
                            frame_id * data_subframe_num_perframe + data_subframe_id,
                            TASK_BUFFER_SUBFRAME_NUM, consumer_encode);
#else
                        int demul_block_num = 1 + (OFDM_DATA_NUM - 1) / config_->demul_block_size;
                        schedule_task_set(TASK_PRECODE, demul_block_num,
                            frame_id * data_subframe_num_perframe + dl_data_subframe_start,
                            TASK_BUFFER_SUBFRAME_NUM, consumer_precode);
#endif
                    }
                }
            } break;

            case EVENT_DEMUL: {
                int offset = event.data;
                int sc_id = offset / TASK_BUFFER_SUBFRAME_NUM;
                int total_data_subframe_id = offset % TASK_BUFFER_SUBFRAME_NUM;
                int frame_id = total_data_subframe_id / data_subframe_num_perframe;
                int data_subframe_id = total_data_subframe_id % data_subframe_num_perframe;
                print_per_task_done(PRINT_DEMUL, frame_id, data_subframe_id, sc_id);
                /* if this subframe is ready */
                if (demul_stats_.last_task(frame_id, data_subframe_id)) {
                    max_equaled_frame = frame_id;
#ifdef USE_LDPC
                    int num_tasks = UE_NUM * LDPC_config.nblocksInSymbol;
                    schedule_task_set(TASK_DECODE, num_tasks,
                        total_data_subframe_id, TASK_BUFFER_SUBFRAME_NUM, consumer_decode);
#endif
                    print_per_subframe_done(PRINT_DEMUL, demul_stats_.frame_count, frame_id, data_subframe_id);
                    if (++demul_stats_.symbol_count[frame_id] == demul_stats_.max_symbol_count) {
                        /* schedule fft for the next frame if there are delayed fft tasks */
#ifndef USE_LDPC
                        schedule_delayed_fft_tasks(demul_stats_.frame_count, frame_id, data_subframe_id, consumer_fft);
#if BIGSTATION
                        demul_stats_.symbol_count[frame_id] = 0;
#endif
                        stats_manager_->update_stats_in_functions_uplink(demul_stats_.frame_count);
#else
                        demul_stats_.symbol_count[frame_id] = 0;
#endif
                        stats_manager_->update_demul_processed(demul_stats_.frame_count);
                        zf_stats_.precoder_exist_in_frame[frame_id] = false;
                        fft_stats_.symbol_data_count[frame_id] = 0;
                        print_per_frame_done(PRINT_DEMUL, demul_stats_.frame_count, frame_id);

                        update_frame_count(&demul_stats_.frame_count);
                    }
                    // save_demul_data_to_file(frame_id, data_subframe_id);
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

            case EVENT_DECODE: {
                int offset = event.data;
                int total_data_subframe_id = offset % TASK_BUFFER_SUBFRAME_NUM;
                int frame_id = total_data_subframe_id / data_subframe_num_perframe;
                int data_subframe_id = total_data_subframe_id % data_subframe_num_perframe;
                if (decode_stats_.last_task(frame_id, data_subframe_id)) {
                    print_per_subframe_done(PRINT_DECODE, decode_stats_.frame_count, frame_id, data_subframe_id);
                    if (++decode_stats_.symbol_count[frame_id] == decode_stats_.max_symbol_count) {
                        schedule_delayed_fft_tasks(decode_stats_.frame_count, frame_id, data_subframe_id, consumer_fft);
#if BIGSTATION
                        prev_frame_counter[frame_id] = 0;
#endif
                        stats_manager_->update_decode_processed(decode_stats_.frame_count);
                        print_per_frame_done(PRINT_DECODE, decode_stats_.frame_count, frame_id);
                        stats_manager_->update_stats_in_functions_uplink(decode_stats_.frame_count);
                        update_frame_count(&decode_stats_.frame_count);
                    }
                }
            } break;

            case EVENT_ENCODE: {
                int offset = event.data;
                int total_data_subframe_id = offset % TASK_BUFFER_SUBFRAME_NUM;
                int frame_id = total_data_subframe_id / data_subframe_num_perframe;
                int data_subframe_id = total_data_subframe_id % data_subframe_num_perframe;
                int demul_block_num = 1 + (OFDM_DATA_NUM - 1) / config_->demul_block_size;

                if (encode_stats_.last_task(frame_id, data_subframe_id)) {
                    schedule_task_set(TASK_PRECODE, demul_block_num,
                        total_data_subframe_id, TASK_BUFFER_SUBFRAME_NUM, consumer_precode);
                    print_per_subframe_done(PRINT_ENCODE, encode_stats_.frame_count, frame_id, data_subframe_id);
                    if (encode_stats_.last_symbol(frame_id)) {
                        stats_manager_->update_encode_processed(encode_stats_.frame_count);
                        print_per_frame_done(PRINT_ENCODE, encode_stats_.frame_count, frame_id);
                        update_frame_count(&encode_stats_.frame_count);
                    }
                }
            } break;

            case EVENT_PRECODE: {
                /* Precoding is done, schedule ifft */
                int offset = event.data;
                int sc_id = offset / TASK_BUFFER_SUBFRAME_NUM;
                int total_data_subframe_id = offset % TASK_BUFFER_SUBFRAME_NUM;
                int frame_id = total_data_subframe_id / data_subframe_num_perframe;
                int data_subframe_id = total_data_subframe_id % data_subframe_num_perframe;

                print_per_task_done(PRINT_PRECODE, frame_id, data_subframe_id, sc_id);
                if (precode_stats_.last_task(frame_id, data_subframe_id)) {
                    schedule_task_set(TASK_IFFT, BS_ANT_NUM,
                        total_data_subframe_id, TASK_BUFFER_SUBFRAME_NUM, consumer_ifft);
                    if (data_subframe_id < dl_data_subframe_end - 1) {
#ifdef USE_LDPC
                        int num_tasks = UE_NUM * LDPC_config.nblocksInSymbol;
                        schedule_task_set(TASK_ENCODE, num_tasks,
                            total_data_subframe_id, TASK_BUFFER_SUBFRAME_NUM, consumer_encode);
#else
                        int demul_block_num = 1 + (OFDM_DATA_NUM - 1) / config_->demul_block_size;
                        schedule_task_set(TASK_PRECODE, demul_block_num,
                            total_data_subframe_id + 1, TASK_BUFFER_SUBFRAME_NUM, consumer_precode);
#endif
                    }

                    print_per_subframe_done(PRINT_PRECODE, precode_stats_.frame_count, frame_id, data_subframe_id);
                    if (precode_stats_.last_symbol(frame_id)) {
                        stats_manager_->update_precode_processed(precode_stats_.frame_count);
                        print_per_frame_done(PRINT_PRECODE, precode_stats_.frame_count, frame_id);
                        update_frame_count(&precode_stats_.frame_count);
                    }
                }
            } break;
            case EVENT_IFFT: {
                /* IFFT is done, schedule data transmission */
                int offset = event.data;
                int ant_id = offset / TASK_BUFFER_SUBFRAME_NUM;
                int total_data_subframe_id = offset % TASK_BUFFER_SUBFRAME_NUM;
                int frame_id = total_data_subframe_id / data_subframe_num_perframe;
                int data_subframe_id = total_data_subframe_id % data_subframe_num_perframe;

                Event_data do_tx_task;
                do_tx_task.event_type = TASK_SEND;
                do_tx_task.data = offset;
                int ptok_id = ant_id % SOCKET_RX_THREAD_NUM;
                Consumer consumer_tx(tx_queue_, *tx_ptoks_ptr[ptok_id]);
                consumer_tx.try_handle(do_tx_task);

                frame_id = frame_id % TASK_BUFFER_FRAME_NUM;
                print_per_task_done(PRINT_IFFT, frame_id, data_subframe_id, ant_id);

                if (ifft_stats_.last_task(frame_id, data_subframe_id)) {
                    if (++ifft_stats_.symbol_count[frame_id] == ifft_stats_.max_symbol_count) {
                        /* schedule fft for next frame */
                        schedule_delayed_fft_tasks(ifft_stats_.frame_count, frame_id, data_subframe_id, consumer_fft);
                        stats_manager_->update_ifft_processed(ifft_stats_.frame_count);
                        print_per_frame_done(PRINT_IFFT, ifft_stats_.frame_count, frame_id);
                        update_frame_count(&ifft_stats_.frame_count);
                    }
                }
            } break;
            case EVENT_PACKET_SENT: {
                /* Data is sent */
                int offset = event.data;
                int ant_id = offset / TASK_BUFFER_SUBFRAME_NUM;
                int total_data_subframe_id = offset % TASK_BUFFER_SUBFRAME_NUM;
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
                        update_frame_count(&tx_stats_.frame_count);
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
    this->stop();
    printf("Total dequeue trials: %d, missed %d\n", total_count, miss_count);
    int last_frame_id = downlink_mode ? tx_stats_.frame_count : demul_stats_.frame_count;
    stats_manager_->save_to_file(last_frame_id, SOCKET_RX_THREAD_NUM);
    stats_manager_->print_summary(last_frame_id);
    //exit(0);
}

void* Millipede::worker(int tid)
{
    int core_offset = SOCKET_RX_THREAD_NUM + CORE_OFFSET + 1;
    pin_to_core_with_offset(Worker, core_offset, tid);
    Consumer consumer(complete_task_queue_, *task_ptoks_ptr[tid]);

    /* initialize operators */
    auto computeFFT = new DoFFT(config_, tid, consumer,
        socket_buffer_, socket_buffer_status_, data_buffer_, csi_buffer_,
        stats_manager_);

    auto computeIFFT = new DoIFFT(config_, tid, consumer,
        dl_ifft_buffer_, dl_socket_buffer_, stats_manager_);

    auto computeZF = new DoZF(config_, tid, consumer,
        csi_buffer_, precoder_buffer_, dl_precoder_buffer_, stats_manager_);

    auto computeDemul = new DoDemul(config_, tid, consumer,
        data_buffer_, precoder_buffer_, equal_buffer_, demod_hard_buffer_, demod_soft_buffer_, stats_manager_);

    auto computePrecode = new DoPrecode(config_, tid, consumer, dl_precoder_buffer_, dl_ifft_buffer_,
#ifdef USE_LDPC
        dl_encoded_buffer_,
#else
        *dl_IQ_data,
#endif
        stats_manager_);

#ifdef USE_LDPC
    auto* computeCoding = new DoCoding(config_, tid, consumer,
        *dl_IQ_data, dl_encoded_buffer_, demod_soft_buffer_, decoded_buffer_,
        stats_manager_);
#endif

    Event_data event;
    bool ret = false;

    int queue_num;
    int* dequeue_order;
#ifdef USE_LDPC
    int dequeue_order_DL_LDPC[] = { TASK_IFFT, TASK_PRECODE, TASK_ENCODE, TASK_ZF, TASK_FFT };
    int dequeue_order_UL_LDPC[] = { TASK_ZF, TASK_FFT, TASK_DEMUL, TASK_DECODE };
#else
    int dequeue_order_DL[] = { TASK_IFFT, TASK_PRECODE, TASK_ZF, TASK_FFT };
    int dequeue_order_UL[] = { TASK_ZF, TASK_FFT, TASK_DEMUL };
#endif

#ifdef USE_LDPC
    if (downlink_mode) {
        queue_num = sizeof(dequeue_order_DL_LDPC) / sizeof(dequeue_order_DL_LDPC[0]);
        dequeue_order = dequeue_order_DL_LDPC;
    } else {
        queue_num = sizeof(dequeue_order_UL_LDPC) / sizeof(dequeue_order_UL_LDPC[0]);
        ;
        dequeue_order = dequeue_order_UL_LDPC;
    }
#else
    if (downlink_mode) {
        queue_num = sizeof(dequeue_order_DL) / sizeof(dequeue_order_DL[0]);
        ;
        dequeue_order = dequeue_order_DL;
    } else {
        queue_num = sizeof(dequeue_order_UL) / sizeof(dequeue_order_UL[0]);
        dequeue_order = dequeue_order_UL;
    }
#endif

    int dequeue_idx = 0;

    while (true) {
        switch (dequeue_order[dequeue_idx]) {
        case TASK_IFFT:
            ret = ifft_queue_.try_dequeue(event);
            if (ret)
                computeIFFT->IFFT(event.data);
            break;
        case TASK_PRECODE:
            ret = precode_queue_.try_dequeue(event);
            if (ret)
                computePrecode->Precode(event.data);
            break;
#ifdef USE_LDPC
        case TASK_ENCODE:
            ret = encode_queue_.try_dequeue(event);
            if (ret)
                computeCoding->Encode(event.data);
            break;
        case TASK_DECODE:
            ret = decode_queue_.try_dequeue(event);
            if (ret)
                computeCoding->Decode(event.data);
            break;
#endif
        case TASK_ZF:
            ret = zf_queue_.try_dequeue(event);
            if (ret)
                computeZF->ZF(event.data);
            break;
        case TASK_FFT:
            ret = fft_queue_.try_dequeue(event);
            if (ret)
                computeFFT->FFT(event.data);
            break;
        case TASK_DEMUL:
            ret = demul_queue_.try_dequeue(event);
            if (ret)
                computeDemul->Demul(event.data);
            break;
        default:
            printf("ERROR: unsupported task type in dequeue\n");
            exit(0);
        }
        if (ret)
            dequeue_idx = 0;
        else
            dequeue_idx = (dequeue_idx + 1) % queue_num;
    }
}

void* Millipede::worker_fft(int tid)
{
    int core_offset = SOCKET_RX_THREAD_NUM + CORE_OFFSET + 1;
    pin_to_core_with_offset(Worker_FFT, core_offset, tid);
    Consumer consumer(complete_task_queue_, *task_ptoks_ptr[tid]);

    /* initialize IFFT operator */
    auto computeFFT = new DoFFT(config_, tid, consumer,
        socket_buffer_, socket_buffer_status_, data_buffer_, csi_buffer_,
        stats_manager_);
    auto computeIFFT = new DoIFFT(config_, tid, consumer,
        dl_ifft_buffer_, dl_socket_buffer_, stats_manager_);

    Event_data event;

    while (true) {
        if (fft_queue_.try_dequeue(event))
            computeFFT->FFT(event.data);
        else if (downlink_mode && ifft_queue_.try_dequeue(event))
            computeIFFT->IFFT(event.data);
    }
}

void* Millipede::worker_zf(int tid)
{

    int core_offset = SOCKET_RX_THREAD_NUM + CORE_OFFSET + 1;
    pin_to_core_with_offset(Worker_ZF, core_offset, tid);

    Consumer consumer(complete_task_queue_, *task_ptoks_ptr[tid]);

    /* initialize ZF operator */
    auto computeZF = new DoZF(config_, tid, consumer,
        csi_buffer_, precoder_buffer_, dl_precoder_buffer_, stats_manager_);

    Event_data event;

    while (true) {
        if (zf_queue_.try_dequeue(event))
            computeZF->ZF(event.data);
    }
}

void* Millipede::worker_demul(int tid)
{

    int core_offset = SOCKET_RX_THREAD_NUM + CORE_OFFSET + 1;
    pin_to_core_with_offset(Worker_Demul, core_offset, tid);
    Consumer consumer(complete_task_queue_, *task_ptoks_ptr[tid]);

    /* initialize Demul operator */
    auto computeDemul = new DoDemul(config_, tid, consumer,
        data_buffer_, precoder_buffer_, equal_buffer_, demod_hard_buffer_, demod_soft_buffer_, stats_manager_);

    /* initialize Precode operator */
    auto computePrecode = new DoPrecode(config_, tid, consumer,
        dl_precoder_buffer_, dl_ifft_buffer_,
#ifdef USE_LDPC
        dl_encoded_buffer_,
#else
        *dl_IQ_data,
#endif
        stats_manager_);

    Event_data event;
    // int cur_frame_id = 0;

    while (true) {
        if (downlink_mode) {
            if (precode_queue_.try_dequeue(event))
                computePrecode->Precode(event.data);
        } else if (demul_queue_.try_dequeue(event)) {
            // int ul_data_subframe_num_perframe = cfg->ul_data_symbol_num_perframe;
            // int frame_id = event.data / (OFDM_CA_NUM * ul_data_subframe_num_perframe);
            // // check precoder status for the current frame
            // if (frame_id > cur_frame_id || frame_id == 0) {
            //     while (!precoder_status_[frame_id]);
            // }
            computeDemul->Demul(event.data);
        }
    }
}

void Millipede::create_threads(thread_type thread, int tid_start, int tid_end)
{
    int ret;
    for (int i = tid_start; i < tid_end; i++) {
        context[i].obj_ptr = this;
        context[i].id = i;
        switch (thread) {
        case Worker:
            ret = pthread_create(&task_threads[i], NULL, pthread_fun_wrapper<Millipede, &Millipede::worker>, &context[i]);
            break;
        case Worker_FFT:
            ret = pthread_create(&task_threads[i], NULL, pthread_fun_wrapper<Millipede, &Millipede::worker_fft>, &context[i]);
            break;
        case Worker_ZF:
            ret = pthread_create(&task_threads[i], NULL, pthread_fun_wrapper<Millipede, &Millipede::worker_zf>, &context[i]);
            break;
        case Worker_Demul:
            ret = pthread_create(&task_threads[i], NULL, pthread_fun_wrapper<Millipede, &Millipede::worker_demul>, &context[i]);
            break;
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

inline void Millipede::update_frame_count(int* frame_count)
{
    *frame_count = *frame_count + 1;
    if (*frame_count == 1e9)
        *frame_count = 0;
}

void Millipede::schedule_fft_task(UNUSED int offset, UNUSED int frame_count,
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
    rx_stats_.fft_created_count[frame_id]++;
    if (rx_stats_.fft_created_count[frame_id] == 1) {
        stats_manager_->update_processing_started(frame_count);
    } else if (rx_stats_.fft_created_count[frame_id] == rx_stats_.max_task_count) {
        rx_stats_.fft_created_count[frame_id] = 0;
#if DEBUG_PRINT_PER_FRAME_ENTER_QUEUE
        printf("Main thread: created FFT tasks for all packets in frame: %d, frame buffer: %d in %.5f us\n",
            frame_count, frame_id, get_time() - stats_manager_->get_pilot_received(frame_count));
#endif
#if !BIGSTATION
        prev_frame_counter[(frame_id + TASK_BUFFER_FRAME_NUM - 1) % TASK_BUFFER_FRAME_NUM] = 0;
#endif
    }
}

void Millipede::schedule_delayed_fft_tasks(int frame_count, int frame_id, int data_subframe_id,
    Consumer const& consumer)
{
    frame_id = (frame_id + 1) % TASK_BUFFER_FRAME_NUM;
    if (delay_fft_queue_cnt[frame_id] > 0) {
        for (int i = 0; i < delay_fft_queue_cnt[frame_id]; i++) {
            int offset_rx = delay_fft_queue[frame_id][i];
            schedule_fft_task(offset_rx, frame_count + 1, frame_id, data_subframe_id + UE_NUM, 0, consumer);
        }
        delay_fft_queue_cnt[frame_id] = 0;
#if DEBUG_PRINT_PER_FRAME_ENTER_QUEUE
        if (downlink_mode)
            printf("Main thread in IFFT: schedule fft for %d packets for frame %d is done\n", delay_fft_queue_cnt[frame_id], frame_id);
        else
            printf("Main thread in demul: schedule fft for %d packets for frame %d is done\n", delay_fft_queue_cnt[frame_id], frame_id);
#endif
    }
}

void Millipede::schedule_zf_task(int frame_id, Consumer const& consumer)
{
    /* schedule normal ZF for all data subcarriers */
    int data_subframe_num_perframe = config_->data_symbol_num_perframe;
    Event_data do_zf_task;
    do_zf_task.event_type = TASK_ZF;
    do_zf_task.data = frame_id * data_subframe_num_perframe;
    int zf_block_num = 1 + (OFDM_DATA_NUM - 1) / config_->zf_block_size;
    for (int i = 0; i < zf_block_num; i++) {
        consumer.try_handle(do_zf_task);
        do_zf_task.data++;
    }
#if DEBUG_PRINT_PER_FRAME_ENTER_QUEUE
    printf("Main thread: created ZF tasks for frame: %d\n", frame_id);
#endif
}

void Millipede::schedule_demul_task(int frame_id, int start_subframe_id, int end_subframe_id, Consumer const& consumer)
{
    int data_subframe_num_perframe = config_->data_symbol_num_perframe;
    int TASK_BUFFER_SUBFRAME_NUM = data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM;
    int demul_block_num = 1 + (OFDM_DATA_NUM - 1) / config_->demul_block_size;
    for (int data_subframe_id = start_subframe_id; data_subframe_id < end_subframe_id; data_subframe_id++) {
        if (fft_stats_.data_exist_in_symbol[frame_id][data_subframe_id]) {
            /* schedule demodulation task for subcarrier blocks */
            schedule_task_set(TASK_DEMUL, demul_block_num,
                frame_id * data_subframe_num_perframe + data_subframe_id, TASK_BUFFER_SUBFRAME_NUM, consumer);
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
    case (PRINT_DEMUL):
        printf("Main thread: Demodulation done frame %d %d, subframe: %d, num sumbframes done: %d\n",
            frame_count, frame_id, subframe_id, demul_stats_.symbol_count[frame_id]);
        break;
    case (PRINT_DECODE):
        printf("Main thread: Decoding done frame %d %d, subframe: %d, num sumbframes done: %d\n",
            frame_count, frame_id, subframe_id, decode_stats_.symbol_count[frame_id]);
        break;
    case (PRINT_ENCODE):
        printf("Main thread: Encoding done frame %d %d, subframe: %d, num sumbframes done: %d\n",
            frame_count, frame_id, subframe_id, encode_stats_.symbol_count[frame_id]);
        break;
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
    case (PRINT_DEMUL):
        printf("Main thread: Demodulation done frame: %d, subframe: %d, sc: %d, num blocks done: %d\n",
            frame_id, subframe_id, ant_or_sc_id, demul_stats_.task_count[frame_id][subframe_id]);
        break;
    case (PRINT_DEMUL):
        printf("Main thread: Decoding done frame: %d, subframe: %d, sc: %d, num blocks done: %d\n",
            frame_id, subframe_id, ant_or_sc_id, decode_stats.task_count[frame_id][subframe_id]);
        break;
    case (PRINT_PRECODE):
        printf("Main thread: Precoding done frame: %d, subframe: %d, subcarrier: %d, total SCs: %d\n",
            frame_id, subframe_id, ant_or_sc_id, precode_stats_.task_count[frame_id][subframe_id]);
        break;
    case (PRINT_IFFT):
        printf("Main thread: IFFT done frame: %d, subframe: %d, antenna: %d, total ants: %d\n",
            frame_id, subframe_id, ant_or_sc_id, ifft_stats_.task_count[frame_id]);
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

void Millipede::initialize_vars_from_cfg(Config* cfg)
{
    dl_IQ_data = &cfg->dl_IQ_data;

#if DEBUG_PRINT_PILOT
    cout << "Pilot data" << endl;
    for (int i = 0; i < OFDM_CA_NUM; i++)
        cout << cfg->pilots_[i] << ",";
    cout << endl;
#endif

    BS_ANT_NUM = cfg->BS_ANT_NUM;
    UE_NUM = cfg->UE_NUM;
    PILOT_NUM = cfg->pilot_symbol_num_perframe;
    OFDM_CA_NUM = cfg->OFDM_CA_NUM;
    OFDM_DATA_NUM = cfg->OFDM_DATA_NUM;
    downlink_mode = config_->downlink_mode;
    dl_data_subframe_start = cfg->dl_data_symbol_start;
    dl_data_subframe_end = cfg->dl_data_symbol_end;
    packet_length = cfg->packet_length;

    TASK_THREAD_NUM = cfg->worker_thread_num;
    SOCKET_RX_THREAD_NUM = cfg->socket_thread_num;
    SOCKET_TX_THREAD_NUM = cfg->socket_thread_num;
    FFT_THREAD_NUM = cfg->fft_thread_num;
    DEMUL_THREAD_NUM = cfg->demul_thread_num;
    ZF_THREAD_NUM = cfg->zf_thread_num;
    CORE_OFFSET = cfg->core_offset;

    LDPC_config = cfg->LDPC_config;
    mod_type = cfg->mod_type;
}

void Millipede::initialize_queues()
{
    int data_subframe_num_perframe = config_->data_symbol_num_perframe;
    message_queue_ = moodycamel::ConcurrentQueue<Event_data>(512 * data_subframe_num_perframe);
    complete_task_queue_ = moodycamel::ConcurrentQueue<Event_data>(512 * data_subframe_num_perframe * 4);

    fft_queue_ = moodycamel::ConcurrentQueue<Event_data>(512 * data_subframe_num_perframe * 4);
    zf_queue_ = moodycamel::ConcurrentQueue<Event_data>(512 * data_subframe_num_perframe * 4);
    ;
    demul_queue_ = moodycamel::ConcurrentQueue<Event_data>(512 * data_subframe_num_perframe * 4);
    decode_queue_ = moodycamel::ConcurrentQueue<Event_data>(512 * data_subframe_num_perframe * 4);

    ifft_queue_ = moodycamel::ConcurrentQueue<Event_data>(512 * data_subframe_num_perframe * 4);
    // modulate_queue_ = moodycamel::ConcurrentQueue<Event_data>(512 * data_subframe_num_perframe * 4);
    encode_queue_ = moodycamel::ConcurrentQueue<Event_data>(512 * data_subframe_num_perframe * 4);
    precode_queue_ = moodycamel::ConcurrentQueue<Event_data>(512 * data_subframe_num_perframe * 4);
    tx_queue_ = moodycamel::ConcurrentQueue<Event_data>(512 * data_subframe_num_perframe * 4);

    rx_ptoks_ptr = (moodycamel::ProducerToken**)aligned_alloc(64, SOCKET_RX_THREAD_NUM * sizeof(moodycamel::ProducerToken*));
    for (int i = 0; i < SOCKET_RX_THREAD_NUM; i++)
        rx_ptoks_ptr[i] = new moodycamel::ProducerToken(message_queue_);

    tx_ptoks_ptr = (moodycamel::ProducerToken**)aligned_alloc(64, SOCKET_RX_THREAD_NUM * sizeof(moodycamel::ProducerToken*));
    for (int i = 0; i < SOCKET_RX_THREAD_NUM; i++)
        tx_ptoks_ptr[i] = new moodycamel::ProducerToken(tx_queue_);

    task_ptoks_ptr = (moodycamel::ProducerToken**)aligned_alloc(64, TASK_THREAD_NUM * sizeof(moodycamel::ProducerToken*));
    for (int i = 0; i < TASK_THREAD_NUM; i++)
        task_ptoks_ptr[i] = new moodycamel::ProducerToken(complete_task_queue_);
}

void Millipede::initialize_uplink_buffers()
{
    int data_subframe_num_perframe = config_->data_symbol_num_perframe;
    int TASK_BUFFER_SUBFRAME_NUM = data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM;
    int ul_data_subframe_num_perframe = config_->ul_data_symbol_num_perframe;

    alloc_buffer_1d(&task_threads, TASK_THREAD_NUM, 64, 0);
    alloc_buffer_1d(&context, TASK_THREAD_NUM, 64, 0);
    // task_threads = (pthread_t *)malloc(TASK_THREAD_NUM * sizeof(pthread_t));
    // context = (EventHandlerContext *)malloc(TASK_THREAD_NUM * sizeof(EventHandlerContext));

    socket_buffer_size_ = (long long)packet_length * config_->symbol_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM;
    socket_buffer_status_size_ = config_->symbol_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM;
    printf("socket_buffer_size %lld, socket_buffer_status_size %d\n", socket_buffer_size_, socket_buffer_status_size_);
    socket_buffer_.malloc(SOCKET_RX_THREAD_NUM, socket_buffer_size_, 64);
    socket_buffer_status_.calloc(SOCKET_RX_THREAD_NUM, socket_buffer_status_size_, 64);

    csi_buffer_.malloc(PILOT_NUM * TASK_BUFFER_FRAME_NUM, BS_ANT_NUM * OFDM_DATA_NUM, 64);
    data_buffer_.malloc(TASK_BUFFER_SUBFRAME_NUM, BS_ANT_NUM * OFDM_DATA_NUM, 64);
    precoder_buffer_.malloc(OFDM_DATA_NUM * TASK_BUFFER_FRAME_NUM, BS_ANT_NUM * UE_NUM, 64);

    equal_buffer_.malloc(TASK_BUFFER_SUBFRAME_NUM, OFDM_DATA_NUM * UE_NUM, 64);
    demod_hard_buffer_.malloc(TASK_BUFFER_SUBFRAME_NUM, OFDM_DATA_NUM * UE_NUM, 64);
    demod_soft_buffer_.malloc(TASK_BUFFER_SUBFRAME_NUM, mod_type * OFDM_DATA_NUM * UE_NUM, 64);
    decoded_buffer_.malloc(TASK_BUFFER_SUBFRAME_NUM, OFDM_DATA_NUM * UE_NUM, 64);

    int max_packet_num_per_frame = downlink_mode ? (BS_ANT_NUM * PILOT_NUM) : (BS_ANT_NUM * (ul_data_subframe_num_perframe + PILOT_NUM));
    rx_stats_.max_task_count = max_packet_num_per_frame;
    rx_stats_.max_task_pilot_count = BS_ANT_NUM * PILOT_NUM;
    alloc_buffer_1d(&(rx_stats_.task_count), TASK_BUFFER_FRAME_NUM, 64, 1);
    alloc_buffer_1d(&(rx_stats_.task_pilot_count), TASK_BUFFER_FRAME_NUM, 64, 1);
    alloc_buffer_1d(&(rx_stats_.fft_created_count), TASK_BUFFER_FRAME_NUM, 64, 1);

    fft_stats_.init(BS_ANT_NUM, PILOT_NUM,
        TASK_BUFFER_FRAME_NUM, config_->symbol_num_perframe, 64);
    alloc_buffer_1d(&(fft_stats_.symbol_data_count), TASK_BUFFER_FRAME_NUM, 64, 1);
    fft_stats_.max_symbol_data_count = ul_data_subframe_num_perframe;
    fft_stats_.data_exist_in_symbol.calloc(TASK_BUFFER_FRAME_NUM, data_subframe_num_perframe, 64);

    int zf_block_num = 1 + (OFDM_DATA_NUM - 1) / config_->zf_block_size;
    zf_stats_.init(zf_block_num, TASK_BUFFER_FRAME_NUM, 64, 1);

    int demul_block_num = 1 + (OFDM_DATA_NUM - 1) / config_->demul_block_size;
    demul_stats_.init(demul_block_num, ul_data_subframe_num_perframe,
        TASK_BUFFER_FRAME_NUM, data_subframe_num_perframe, 64);

    decode_stats_.init(LDPC_config.nblocksInSymbol * UE_NUM, ul_data_subframe_num_perframe,
        TASK_BUFFER_FRAME_NUM, data_subframe_num_perframe, 64);

    delay_fft_queue.calloc(TASK_BUFFER_FRAME_NUM, config_->symbol_num_perframe * BS_ANT_NUM, 32);
    alloc_buffer_1d(&delay_fft_queue_cnt, TASK_BUFFER_FRAME_NUM, 32, 1);
}

void Millipede::initialize_downlink_buffers()
{
    int data_subframe_num_perframe = config_->data_symbol_num_perframe;
    int TASK_BUFFER_SUBFRAME_NUM = data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM;
    int dl_data_subframe_num_perframe = config_->dl_data_symbol_num_perframe;

    dl_socket_buffer_size_ = (long long)data_subframe_num_perframe * SOCKET_BUFFER_FRAME_NUM * packet_length * BS_ANT_NUM;
    dl_socket_buffer_status_size_ = data_subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM;
    alloc_buffer_1d(&dl_socket_buffer_, dl_socket_buffer_size_, 64, 0);
    alloc_buffer_1d(&dl_socket_buffer_status_, dl_socket_buffer_status_size_, 64, 1);
    dl_ifft_buffer_.calloc(BS_ANT_NUM * TASK_BUFFER_SUBFRAME_NUM, OFDM_CA_NUM, 64);
    dl_precoder_buffer_.malloc(OFDM_DATA_NUM * TASK_BUFFER_FRAME_NUM, UE_NUM * BS_ANT_NUM, 64);
    dl_encoded_buffer_.malloc(TASK_BUFFER_SUBFRAME_NUM, OFDM_DATA_NUM * UE_NUM, 64);

    encode_stats_.init(LDPC_config.nblocksInSymbol * UE_NUM, dl_data_subframe_num_perframe,
        TASK_BUFFER_FRAME_NUM, data_subframe_num_perframe, 64);

    int demul_block_num = 1 + (OFDM_DATA_NUM - 1) / config_->demul_block_size;
    precode_stats_.init(demul_block_num, dl_data_subframe_num_perframe,
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
    free_buffer_1d(&(rx_stats_.fft_created_count));
    fft_stats_.fini();
    fft_stats_.data_exist_in_symbol.free();
    free_buffer_1d(&(fft_stats_.symbol_data_count));
    zf_stats_.fini();
    demul_stats_.fini();
    decode_stats_.fini();

    delay_fft_queue.free();
    free_buffer_1d(&delay_fft_queue_cnt);
}

void Millipede::free_downlink_buffers()
{
    free_buffer_1d(&dl_socket_buffer_);
    free_buffer_1d(&dl_socket_buffer_status_);

    dl_ifft_buffer_.free();

    encode_stats_.fini();
    precode_stats_.fini();
    ifft_stats_.fini();
    tx_stats_.fini();
}

void Millipede::save_demul_data_to_file(UNUSED int frame_id, UNUSED int data_subframe_id)
{
#if WRITE_DEMUL
    int data_subframe_num_perframe = config_->data_symbol_num_perframe;
    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string filename = cur_directory + "/data/demul_data.txt";
    FILE* fp = fopen(filename.c_str(), "a");
    int total_data_subframe_id = frame_id * data_subframe_num_perframe + data_subframe_id;
    for (int cc = 0; cc < OFDM_DATA_NUM; cc++) {
        int* cx = &demod_hard_buffer_[total_data_subframe_id][cc * UE_NUM];
        fprintf(fp, "SC: %d, Frame %d, subframe: %d, ", cc, frame_id, data_subframe_id);
        for (int kk = 0; kk < UE_NUM; kk++)
            fprintf(fp, "%d ", cx[kk]);
        fprintf(fp, "\n");
    }
    fclose(fp);
#endif
}

void Millipede::getDemulData(int** ptr, int* size)
{
    int data_subframe_num_perframe = config_->data_symbol_num_perframe;
    *ptr = (int*)&equal_buffer_[max_equaled_frame * data_subframe_num_perframe][0];
    *size = UE_NUM * OFDM_CA_NUM;
}

void Millipede::getEqualData(float** ptr, int* size)
{
    int data_subframe_num_perframe = config_->data_symbol_num_perframe;
    // max_equaled_frame = 0;
    *ptr = (float*)&equal_buffer_[max_equaled_frame * data_subframe_num_perframe][0];
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
