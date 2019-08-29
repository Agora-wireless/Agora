/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 * 
 */
#include "millipede.hpp"

typedef cx_float COMPLEX;
bool keep_running = true;

void intHandler(int) {
    std::cout << "will exit..." << std::endl;
    keep_running = false;
}


Millipede::Millipede(Config *cfg)
{
    std::string directory = TOSTRING(PROJECT_DIRECTORY);
    printf("PROJECT_DIRECTORY: %s\n", directory.c_str());
    printf("Main thread: on core %d\n", sched_getcpu());
    putenv( "MKL_THREADING_LAYER=sequential" );
    std::cout << "MKL_THREADING_LAYER =  " << getenv("MKL_THREADING_LAYER") << std::endl; 
    // openblas_set_num_threads(1);
    printf("enter constructor\n");


    this->cfg_ = cfg;
    initialize_vars_from_cfg(cfg_);

#ifdef ENABLE_CPU_ATTACH
    int main_core_id = CORE_OFFSET + 1;
    if(stick_this_thread_to_core(main_core_id) != 0) {
        printf("Main thread: stitch main thread to core %d failed\n", main_core_id);
        exit(0);
    }
    else {
        printf("Main thread: stitch main thread to core %d succeeded\n", main_core_id);
    }
#endif

    initialize_queues();

    printf("initialize uplink buffers\n");
    initialize_uplink_buffers();

#if ENABLE_DOWNLINK
    printf("initialize downlink buffers\n");
    initialize_downlink_buffers();
#endif  

    
    /* initialize packageReceiver*/
    printf("new PackageReceiver\n");
    receiver_.reset(new PackageReceiver(cfg_, SOCKET_RX_THREAD_NUM, SOCKET_TX_THREAD_NUM, CORE_OFFSET+1, 
                    &message_queue_, &tx_queue_, rx_ptoks_ptr, tx_ptoks_ptr));

    /* create worker threads */
#if BIGSTATION
    for(int i = 0; i < FFT_THREAD_NUM; i++) {
        context[i].obj_ptr = this;
        context[i].id = i;
        if(pthread_create( &task_threads[i], NULL, Millipede::fftThread, &context[i]) != 0) {
            perror("task thread create failed");
            exit(0);
        }
    }
    for(int i = FFT_THREAD_NUM; i < FFT_THREAD_NUM + ZF_THREAD_NUM; i++) {
        context[i].obj_ptr = this;
        context[i].id = i;
        if(pthread_create( &task_threads[i], NULL, Millipede::zfThread, &context[i]) != 0) {
            perror("ZF thread create failed");
            exit(0);
        }
    }
    for(int i = FFT_THREAD_NUM + ZF_THREAD_NUM; i < TASK_THREAD_NUM; i++) {
        context[i].obj_ptr = this;
        context[i].id = i;
        if(pthread_create( &task_threads[i], NULL, Millipede::demulThread, &context[i]) != 0) {
            perror("Demul thread create failed");
            exit(0);
        }
    }
#else
    for(int i = 0; i < TASK_THREAD_NUM; i++) {
        context[i].obj_ptr = this;
        context[i].id = i;
        if(pthread_create( &task_threads[i], NULL, Millipede::taskThread, &context[i]) != 0) {
            perror("task thread create failed");
            exit(0);
        }
    }
#endif
    stats_manager_.reset(new Stats(cfg_, CSI_task_duration, CSI_task_count, FFT_task_duration, FFT_task_count, 
          ZF_task_duration, ZF_task_count, Demul_task_duration, Demul_task_count,
          IFFT_task_duration, IFFT_task_count, Precode_task_duration, Precode_task_count,
          frame_start, 
          TASK_THREAD_NUM * 8, 4, TASK_THREAD_NUM * 16,
          TASK_THREAD_NUM, FFT_THREAD_NUM, ZF_THREAD_NUM, DEMUL_THREAD_NUM));
}

Millipede::~Millipede()
{
    free_uplink_buffers();
    /* downlink */
#if ENABLE_DOWNLINK
    free_downlink_buffers();
#endif
}

void Millipede::stop()
{
    std::cout << "stopping threads " << std::endl;
    cfg_->running = false;
    usleep(1000);
    receiver_.reset();
}

void Millipede::start()
{
// #ifdef ENABLE_CPU_ATTACH
//     int main_core_id = CORE_OFFSET + 1;
//     if(stick_this_thread_to_core(main_core_id) != 0) {
//         printf("Main thread: stitch main thread to core %d failed\n", main_core_id);
//         exit(0);
//     }
//     else {
//         printf("Main thread: stitch main thread to core %d succeeded\n", main_core_id);
//     }
// #endif
    /* start uplink receiver */
    std::vector<pthread_t> rx_threads = receiver_->startRecv(socket_buffer_, 
        socket_buffer_status_, socket_buffer_status_size_, socket_buffer_size_, frame_start);
    /* start downlink transmitter */
#if ENABLE_DOWNLINK
    std::vector<pthread_t> tx_threads = receiver_->startTX(dl_socket_buffer_, 
        dl_socket_buffer_status_, dl_socket_buffer_status_size_, dl_socket_buffer_size_);
#endif
    /* tokens used for enqueue */
    /* uplink */
    moodycamel::ProducerToken ptok(fft_queue_);
    moodycamel::ProducerToken ptok_zf(zf_queue_);
    moodycamel::ProducerToken ptok_demul(demul_queue_);
    moodycamel::ProducerToken ptok_decode(decode_queue_);
    /* downlink */
    moodycamel::ProducerToken ptok_ifft(ifft_queue_);
    moodycamel::ProducerToken ptok_modul(modulate_queue_);
    moodycamel::ProducerToken ptok_precode(precode_queue_);
    moodycamel::ProducerToken ptok_tx(tx_queue_);
    /* tokens used for dequeue */
    moodycamel::ConsumerToken ctok(message_queue_);
    moodycamel::ConsumerToken ctok_complete(complete_task_queue_);


    buffer_frame_num = subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM;
    max_packet_num_per_frame = ENABLE_DOWNLINK ? (BS_ANT_NUM * UE_NUM) : (BS_ANT_NUM * subframe_num_perframe);

    /* counters for printing summary */
    int demul_count = 0;
    int tx_count = 0;
    double demul_begin = get_time();
    double tx_begin = get_time();
    
    int frame_count_pilot_fft = 0;
    int frame_count_zf = 0;
    int frame_count_demul = 0;
    int frame_count_decode = 0;
    int frame_count_precode = 0;
    int frame_count_ifft = 0;
    int frame_count_tx = 0;
    
    bool prev_demul_scheduled = false;

    int last_dequeue = 0;
    int ret = 0;
    Event_data events_list[dequeue_bulk_size];
    int miss_count = 0;
    int total_count = 0;
    
    while (cfg_->running && !SignalHandler::gotExitSignal()) {
        /* get a bulk of events */
        if (last_dequeue == 0) {
// #ifdef USE_ARGOS
//             ret = message_queue_.try_dequeue_bulk(ctok, events_list, dequeue_bulk_size_single);
// #else
            ret = 0;
            for (int rx_itr = 0; rx_itr < SOCKET_RX_THREAD_NUM; rx_itr ++)             
                ret += message_queue_.try_dequeue_bulk_from_producer(*(rx_ptoks_ptr[rx_itr]), events_list + ret, dequeue_bulk_size_single);
// #endif
            last_dequeue = 1;
        }
        else {   
            ret = complete_task_queue_.try_dequeue_bulk(ctok_complete, events_list, dequeue_bulk_size_single);
            last_dequeue = 0;
        }
        total_count++;
        if(total_count == 1e9) {
            //printf("message dequeue miss rate %f\n", (float)miss_count / total_count);
            total_count = 0;
            miss_count = 0;
        }
        if(ret == 0) {
            miss_count++;
            continue;
        }

        /* handle each event */
        for(int bulk_count = 0; bulk_count < ret; bulk_count++) {
            Event_data& event = events_list[bulk_count];
            switch(event.event_type) {
                case EVENT_PACKAGE_RECEIVED: {         
                    int offset = event.data;    
                    int socket_thread_id, offset_in_current_buffer;
                    interpreteOffset2d_setbits(offset, &socket_thread_id, &offset_in_current_buffer, 28);                
                    char *socket_buffer_ptr = socket_buffer_[socket_thread_id] + (long long) offset_in_current_buffer * package_length;
                    int frame_id = *((int *)socket_buffer_ptr) % 10000;
                    int subframe_id = *((int *)socket_buffer_ptr + 1);                                    
                    int ant_id = *((int *)socket_buffer_ptr + 3);
                    int frame_id_in_buffer = (frame_id % TASK_BUFFER_FRAME_NUM);
                    int prev_frame_id = (frame_id - 1) % TASK_BUFFER_FRAME_NUM;
                    
                    update_rx_counters(frame_id, frame_id_in_buffer, subframe_id, ant_id); 
#if BIGSTATION 
                    /* in BigStation, schedule FFT whenever a packet is received */
                    schedule_fft_task(offset, frame_id, frame_id_in_buffer, subframe_id, ant_id, prev_frame_id, ptok);
#else
                    /* if this is the first frame or the previous frame is all processed, schedule FFT for this packet */
    #if ENABLE_DOWNLINK
                    bool previous_frame_done = ifft_checker_[prev_frame_id] == BS_ANT_NUM * dl_data_subframe_num_perframe;
    #else
                    bool previous_frame_done = demul_counter_subframes_[prev_frame_id] == ul_data_subframe_num_perframe;
    #endif
                    if ((frame_id == 0 && frame_count_pilot_fft < 100) || (frame_count_pilot_fft > 0 && previous_frame_done)) {
                        schedule_fft_task(offset, frame_id, frame_id_in_buffer, subframe_id, ant_id, prev_frame_id, ptok);
                    }
                    else {
                        /* if the previous frame is not finished, store offset in queue */
                        delay_fft_queue[frame_id_in_buffer][delay_fft_queue_cnt[frame_id_in_buffer]] = offset;
                        delay_fft_queue_cnt[frame_id_in_buffer]++;
                    }
#endif                                            
                }                
                break;        
                case EVENT_FFT: {
                    int offset_fft = event.data;
                    int frame_id, subframe_id;
                    interpreteOffset2d(offset_fft, &frame_id, &subframe_id);
                    fft_counter_ants_[frame_id][subframe_id] ++;
                    if (fft_counter_ants_[frame_id][subframe_id] == BS_ANT_NUM) {
                        fft_counter_ants_[frame_id][subframe_id] = 0;
                        if (cfg_->isPilot(frame_id, subframe_id)) {   
                            csi_counter_users_[frame_id] ++;
                            print_per_subframe_done(PRINT_FFT_PILOTS, frame_count_pilot_fft, frame_id, subframe_id);
                            /* if csi of all UEs is ready, schedule ZF or prediction */
                            if (csi_counter_users_[frame_id] == UE_NUM) {
                                csi_counter_users_[frame_id] = 0; 
                                stats_manager_->update_fft_processed(frame_count_pilot_fft);     
                                print_per_frame_done(PRINT_FFT_PILOTS, frame_count_pilot_fft, frame_id);    
                                update_frame_count(&frame_count_pilot_fft);                          
                                schedule_zf_task(frame_id, ptok_zf);
                            }
                        }
                        else if (cfg_->isUplink(frame_id, subframe_id)) {                     
                            data_exist_in_subframe_[frame_id][cfg_->getUlSFIndex(frame_id, subframe_id)] = true;  
                            data_counter_subframes_[frame_id]++; 
                            print_per_subframe_done(PRINT_FFT_DATA, frame_count_pilot_fft - 1, frame_id, subframe_id);                 
                            if (data_counter_subframes_[frame_id] == ul_data_subframe_num_perframe) {      
                                print_per_frame_done(PRINT_FFT_DATA, frame_count_pilot_fft - 1, frame_id);                                
                                prev_demul_scheduled = false;
                            }                     
                            /* if precoder exist, schedule demodulation */
                            if (precoder_exist_in_frame_[frame_id]) {
                                int start_sche_id = subframe_id;
                                if (!prev_demul_scheduled) {
                                    start_sche_id = UE_NUM;
                                    prev_demul_scheduled = true;
                                }
                                int end_sche_id = UE_NUM + data_counter_subframes_[frame_id];
                                if (end_sche_id < subframe_id)
                                    end_sche_id = subframe_id + 1;
                                schedule_demul_task(frame_id, start_sche_id, end_sche_id, ptok_demul);
                            }                                      
                        }
                    }
                }
                break;          
                case EVENT_ZF: {
                    int offset_zf = event.data;
                    int frame_id, sc_id;
                    interpreteOffset2d(offset_zf, &frame_id, &sc_id);
                    precoder_counter_scs_[frame_id] ++;
                    // print_per_task_done(PRINT_ZF, frame_id, 0, sc_id);
                    if (precoder_counter_scs_[frame_id] == zf_block_num) { 
                        stats_manager_->update_zf_processed(frame_count_zf);
                        precoder_counter_scs_[frame_id] = 0;
                        precoder_exist_in_frame_[frame_id] = true;
                        print_per_frame_done(PRINT_ZF, frame_count_zf, frame_id); 
                        update_frame_count(&frame_count_zf);
                        /* if all the data in a frame has arrived when ZF is done */
                        if (data_counter_subframes_[frame_id] == ul_data_subframe_num_perframe) 
                            schedule_demul_task(frame_id, UE_NUM, subframe_num_perframe, ptok_demul);   
#if ENABLE_DOWNLINK
                        /* if downlink data transmission is enabled, schedule downlink modulation for the first data subframe */
                        schedule_precode_task(frame_id, dl_data_subframe_start, ptok_precode);               
#endif   
                    }
                }
                break;

                case EVENT_DEMUL: {
                    int offset_demul = event.data;                   
                    int frame_id, total_data_subframe_id, data_subframe_id, sc_id;
                    interpreteOffset3d(offset_demul, &frame_id, &data_subframe_id, &sc_id);
                    demul_counter_scs_[frame_id][data_subframe_id]++;
                    print_per_task_done(PRINT_DEMUL, frame_id, data_subframe_id, sc_id);
                    /* if this subframe is ready */
                    if (demul_counter_scs_[frame_id][data_subframe_id] == demul_block_num) {
                        max_equaled_frame = frame_id;
                        demul_counter_scs_[frame_id][data_subframe_id] = 0;
                        demul_counter_subframes_[frame_id]++;
                        print_per_subframe_done(PRINT_DEMUL, frame_count_demul, frame_id, data_subframe_id);
                        if (demul_counter_subframes_[frame_id] == ul_data_subframe_num_perframe) {
                            /* schedule fft for the next frame if there are delayed fft tasks */
                            schedule_delayed_fft_tasks(frame_count_demul, frame_id, data_subframe_id, ptok);
#if BIGSTATION
                            demul_counter_subframes_[frame_id] = 0;
#endif  
                            stats_manager_->update_demul_processed(frame_count_demul);
                            precoder_exist_in_frame_[frame_id] = false;
                            data_counter_subframes_[frame_id] = 0;
                            print_per_frame_done(PRINT_DEMUL, frame_count_demul, frame_id);
                            stats_manager_->update_stats_in_functions_uplink(frame_count_demul);
                            update_frame_count(&frame_count_demul);                    
                        }
                        save_demul_data_to_file();
                        demul_count++;
                        if (demul_count == ul_data_subframe_num_perframe * 9000) {
                            demul_count = 0;
                            double diff = get_time() - demul_begin;
                            int samples_num_per_UE = OFDM_DATA_NUM * ul_data_subframe_num_perframe * 1000;
                            printf("Frame %d: Receive %d samples (per-client) from %d clients in %f secs, throughtput %f bps per-client (16QAM), current task queue length %d\n", 
                                frame_count_demul, samples_num_per_UE, UE_NUM, diff, samples_num_per_UE * log2(16.0f) / diff, fft_queue_.size_approx());
                            demul_begin = get_time();
                        }                       
                    }
                }
                break;

                case EVENT_PRECODE: {
                    /* Precoding is done, schedule ifft */
                    int offset_precode = event.data;
                    int frame_id, total_data_subframe_id, current_data_subframe_id, sc_id;
                    interpreteOffset3d(offset_precode, &frame_id, &current_data_subframe_id, &sc_id);
                    dl_data_counter_scs_[frame_id][current_data_subframe_id]++;
                    print_per_task_done(PRINT_PRECODE, frame_id, current_data_subframe_id, sc_id);          
                    if (dl_data_counter_scs_[frame_id][current_data_subframe_id] == demul_block_num) {
                        schedule_ifft_task(frame_count_precode, current_data_subframe_id, ptok_ifft);
                        // schedule_ifft_task(frame_id, current_data_subframe_id, ptok_ifft);
                        if (current_data_subframe_id < dl_data_subframe_end - 1) 
                            schedule_precode_task(frame_id, current_data_subframe_id + 1, ptok_precode); 

                        dl_data_counter_scs_[frame_id][current_data_subframe_id] = 0;    
                        print_per_subframe_done(PRINT_PRECODE, frame_count_precode, frame_id, current_data_subframe_id);                   
                        dl_data_counter_subframes_[frame_id]++;
                        if (dl_data_counter_subframes_[frame_id] == dl_data_subframe_num_perframe) {
                            dl_data_counter_subframes_[frame_id] = 0;
                            stats_manager_->update_precode_processed(frame_count_precode);    
                            print_per_frame_done(PRINT_PRECODE, frame_count_precode, frame_id);       
                            update_frame_count(&frame_count_precode);                       
                        }
                    }
                }
                break;
                case EVENT_IFFT: {
                    /* IFFT is done, schedule data transmission */
                    int offset_ifft = event.data;
                    int frame_id, total_data_subframe_id, current_data_subframe_id, ant_id;
                    interpreteOffset3d(offset_ifft, &current_data_subframe_id, &ant_id, &frame_id);
 
                    Event_data do_tx_task;
                    do_tx_task.event_type = TASK_SEND;
                    do_tx_task.data = offset_ifft;      
                    int ptok_id = ant_id % SOCKET_RX_THREAD_NUM;          
                    schedule_task(do_tx_task, &tx_queue_, *tx_ptoks_ptr[ptok_id]);

                    frame_id = frame_id % TASK_BUFFER_FRAME_NUM;
                    ifft_checker_[frame_id] += 1;
                    print_per_task_done(PRINT_IFFT, frame_id, current_data_subframe_id, ant_id);
                    if (ifft_checker_[frame_id] == BS_ANT_NUM * dl_data_subframe_num_perframe) {
                        /* schedule fft for next frame */
                        schedule_delayed_fft_tasks(frame_count_ifft, frame_id, current_data_subframe_id, ptok);
                        stats_manager_->update_ifft_processed(frame_count_ifft);
                        print_per_frame_done(PRINT_IFFT, frame_count_ifft, frame_id);      
                        update_frame_count(&frame_count_ifft);            
                    }
                }
                break;
                case EVENT_PACKAGE_SENT: {
                    /* Data is sent */
                    int offset_tx = event.data;
                    int frame_id, total_data_subframe_id, current_data_subframe_id, ant_id;
                    interpreteOffset3d(offset_tx, &current_data_subframe_id, &ant_id, &frame_id);
                    // printf("In main thread: tx finished for frame %d subframe %d ant %d\n", frame_id, current_data_subframe_id, ant_id);
                    frame_id = frame_id % TASK_BUFFER_FRAME_NUM;
                    tx_counter_ants_[frame_id][current_data_subframe_id] += 1;
                    print_per_task_done(PRINT_TX, frame_id, current_data_subframe_id, ant_id);
                    if (tx_counter_ants_[frame_id][current_data_subframe_id] == BS_ANT_NUM) {
                        tx_counter_ants_[frame_id][current_data_subframe_id] = 0;
                        print_per_subframe_done(PRINT_TX, frame_count_tx, frame_id, current_data_subframe_id);
                        /* if tx of the first symbol is done */
                        if (current_data_subframe_id == dl_data_subframe_start) {
                            stats_manager_->update_tx_processed_first(frame_count_tx);
                            print_per_frame_done(PRINT_TX_FIRST, frame_count_tx, frame_id);
                        }
                        tx_counter_subframes_[frame_id]++;       
                        if (tx_counter_subframes_[frame_id] == dl_data_subframe_num_perframe) {
                            tx_counter_subframes_[frame_id] = 0; 
                            stats_manager_->update_tx_processed(frame_count_tx);    
                            print_per_frame_done(PRINT_TX, frame_count_tx, frame_id);                       
                            stats_manager_->update_stats_in_functions_downlink(frame_count_tx);
                            update_frame_count(&frame_count_tx); 
                        }
                        tx_count++;
                        if (tx_count == dl_data_subframe_num_perframe * 9000)
                        {
                            tx_count = 0;
                            double diff = get_time() - tx_begin;
                            int samples_num_per_UE = OFDM_DATA_NUM * dl_data_subframe_num_perframe * 1000;
                            printf("Transmit %d samples (per-client) to %d clients in %f secs, throughtput %f bps per-client (16QAM), current tx queue length %d\n", 
                                samples_num_per_UE, UE_NUM, diff, samples_num_per_UE * log2(16.0f) / diff, tx_queue_.size_approx());
                            tx_begin = get_time();
                        }
                    }                    
                }
                break;
                default:
                    printf("Wrong event type in message queue!");
                    exit(0);
            } /* end of switch */
        } /* end of for */
    } /* end of while */
    this->stop();
    printf("Total dequeue trials: %d, missed %d\n", total_count, miss_count);
    int last_frame_id = ENABLE_DOWNLINK ? frame_count_tx : frame_count_demul;
    stats_manager_->save_to_file(last_frame_id, SOCKET_RX_THREAD_NUM);
    stats_manager_->print_summary(last_frame_id);
    exit(0);
}


void* Millipede::taskThread(void* context)
{

    int tid = ((EventHandlerContext *)context)->id;
    printf("task thread %d starts\n", tid);
    Millipede* obj_ptr = ((EventHandlerContext *)context)->obj_ptr;
    
#ifdef ENABLE_CPU_ATTACH
    // int offset_id = SOCKET_RX_THREAD_NUM + SOCKET_TX_THREAD_NUM + CORE_OFFSET + 2;
    int offset_id = obj_ptr->SOCKET_RX_THREAD_NUM + obj_ptr->CORE_OFFSET + 2;

    int tar_core_id = tid + offset_id;
    if (tar_core_id>=36) 
        tar_core_id = tar_core_id - 36 + 1;
    if(stick_this_thread_to_core(tar_core_id) != 0) {
        printf("Task thread: stitch thread %d to core %d failed\n", tid, tar_core_id);
        exit(0);
    }
    else {
        printf("Task thread: stitch thread %d to core %d succeeded\n", tid, tar_core_id);
    }
#endif
    
    
    moodycamel::ConcurrentQueue<Event_data>* fft_queue_ = &(obj_ptr->fft_queue_);
    moodycamel::ConcurrentQueue<Event_data>* zf_queue_ = &(obj_ptr->zf_queue_);
    moodycamel::ConcurrentQueue<Event_data>* demul_queue_ = &(obj_ptr->demul_queue_);
    moodycamel::ConcurrentQueue<Event_data>* decode_queue_ = &(obj_ptr->decode_queue_);
    moodycamel::ConcurrentQueue<Event_data>* ifft_queue_ = &(obj_ptr->ifft_queue_);
    moodycamel::ConcurrentQueue<Event_data>* modulate_queue_ = &(obj_ptr->modulate_queue_);
    moodycamel::ConcurrentQueue<Event_data>* precode_queue_ = &(obj_ptr->precode_queue_);
    // moodycamel::ConcurrentQueue<Event_data>* tx_queue_ = &(obj_ptr->tx_queue_);

    // obj_ptr->task_ptok[tid].reset(new moodycamel::ProducerToken(obj_ptr->complete_task_queue_));
    moodycamel::ProducerToken *task_ptok_ptr = obj_ptr->task_ptoks_ptr[tid];
    // task_ptok_ptr = obj_ptr->task_ptok[tid].get();

    /* initialize operators */
    DoFFT *computeFFT = new DoFFT(obj_ptr->cfg_, tid, obj_ptr->transpose_block_size, &(obj_ptr->complete_task_queue_), task_ptok_ptr,
        obj_ptr->socket_buffer_, obj_ptr->socket_buffer_status_, obj_ptr->data_buffer_, obj_ptr->csi_buffer_, obj_ptr->pilots_,
        obj_ptr->dl_ifft_buffer_, obj_ptr->dl_socket_buffer_, 
        obj_ptr->FFT_task_duration, obj_ptr->CSI_task_duration, obj_ptr->FFT_task_count, obj_ptr->CSI_task_count,
        obj_ptr->IFFT_task_duration, obj_ptr->IFFT_task_count);

    DoZF *computeZF = new DoZF(obj_ptr->cfg_, tid, obj_ptr->zf_block_size, obj_ptr->transpose_block_size, &(obj_ptr->complete_task_queue_), task_ptok_ptr,
        obj_ptr->csi_buffer_, obj_ptr->precoder_buffer_, obj_ptr->pred_csi_buffer_, obj_ptr->ZF_task_duration, obj_ptr->ZF_task_count);

    DoDemul *computeDemul = new DoDemul(obj_ptr->cfg_, tid, obj_ptr->demul_block_size, obj_ptr->transpose_block_size, &(obj_ptr->complete_task_queue_), task_ptok_ptr,
        obj_ptr->data_buffer_, obj_ptr->precoder_buffer_, obj_ptr->equal_buffer_, obj_ptr->demul_hard_buffer_, obj_ptr->Demul_task_duration, obj_ptr->Demul_task_count);

    DoPrecode *computePrecode = new DoPrecode(obj_ptr->cfg_, tid, obj_ptr->demul_block_size, obj_ptr->transpose_block_size, &(obj_ptr->complete_task_queue_), task_ptok_ptr,
        obj_ptr->dl_modulated_buffer_, obj_ptr->precoder_buffer_, obj_ptr->dl_precoded_data_buffer_, obj_ptr->dl_ifft_buffer_, obj_ptr->dl_IQ_data, 
        obj_ptr->Precode_task_duration, obj_ptr->Precode_task_count);


    Event_data event;
    bool ret = false;
    bool ret_zf = false;
    bool ret_demul = false;
    bool ret_decode = false;
    bool ret_modul = false;
    bool ret_ifft = false;
    bool ret_precode = false;

    while(true) {
        if (ENABLE_DOWNLINK) {
            ret_ifft = ifft_queue_->try_dequeue(event);
            if (!ret_ifft) {
                ret_precode = precode_queue_->try_dequeue(event);
                if (!ret_precode) {
                    ret_zf = zf_queue_->try_dequeue(event);
                    if (!ret_zf) {
                        ret = fft_queue_->try_dequeue(event);
                        if (!ret) 
                            continue;
                        else
                            computeFFT->FFT(event.data);
                    }
                    else if (event.event_type == TASK_ZF) {
                        computeZF->ZF(event.data);
                    }
                    else if (event.event_type == TASK_PRED) {
                        computeZF->Predict(event.data);
                    }
                }
                else {
                    computePrecode->Precode(event.data);
                }
            }
            else {
                computeFFT->IFFT(event.data);
            }
        }
        else {
            ret_zf = zf_queue_->try_dequeue(event);
            if (!ret_zf) {
                ret = fft_queue_->try_dequeue(event);
                if (!ret) {   
                    ret_demul = demul_queue_->try_dequeue(event);
                    if (!ret_demul)
                        continue;
                    else 
                        computeDemul->Demul(event.data);
                }
                else {
                    computeFFT->FFT(event.data);
                }
            }
            else if (event.event_type == TASK_ZF) {
                computeZF->ZF(event.data);
            }
            else if (event.event_type == TASK_PRED) {
                computeZF->Predict(event.data);
            }
        } 
    }
}



void* Millipede::fftThread(void* context)
{
    
    Millipede* obj_ptr = ((EventHandlerContext *)context)->obj_ptr;
    moodycamel::ConcurrentQueue<Event_data>* fft_queue_ = &(obj_ptr->fft_queue_);
#if ENABLE_DOWNLINK
    moodycamel::ConcurrentQueue<Event_data>* ifft_queue_ = &(obj_ptr->ifft_queue_);
#endif
    int tid = ((EventHandlerContext *)context)->id;
    printf("FFT thread %d starts\n", tid);
    
#ifdef ENABLE_CPU_ATTACH
    // int offset_id = SOCKET_RX_THREAD_NUM + SOCKET_TX_THREAD_NUM + CORE_OFFSET + 2;
    int offset_id = obj_ptr->SOCKET_RX_THREAD_NUM + obj_ptr->CORE_OFFSET + 2;
    int tar_core_id = tid + offset_id;
    if (tar_core_id>=36) 
        tar_core_id = tar_core_id - 36 + 1;
    if(stick_this_thread_to_core(tar_core_id) != 0) {
        printf("FFT thread: stitch thread %d to core %d failed\n", tid, tar_core_id);
        exit(0);
    }
    else {
        printf("FFT thread: stitch thread %d to core %d succeeded\n", tid, tar_core_id);
    }
#endif
    moodycamel::ProducerToken *task_ptok_ptr = obj_ptr->task_ptoks_ptr[tid];
    // obj_ptr->task_ptok[tid].reset(new moodycamel::ProducerToken(obj_ptr->complete_task_queue_));
    // moodycamel::ProducerToken *task_ptok_ptr;
    // task_ptok_ptr = obj_ptr->task_ptok[tid].get();

    /* initialize FFT operator */
    DoFFT* computeFFT = new DoFFT(obj_ptr->cfg_, tid, obj_ptr->transpose_block_size, &(obj_ptr->complete_task_queue_), task_ptok_ptr,
        obj_ptr->socket_buffer_, obj_ptr->socket_buffer_status_, obj_ptr->data_buffer_, obj_ptr->csi_buffer_, obj_ptr->pilots_,
        obj_ptr->dl_ifft_buffer_, obj_ptr->dl_socket_buffer_, 
        obj_ptr->FFT_task_duration, obj_ptr->CSI_task_duration, obj_ptr->FFT_task_count, obj_ptr->CSI_task_count,
        obj_ptr->IFFT_task_duration, obj_ptr->IFFT_task_count);


    Event_data event;
    bool ret = false;

    while(true) {
        ret = fft_queue_->try_dequeue(event);
        if (!ret) {
#if ENABLE_DOWNLINK
            ret = ifft_queue_->try_dequeue(event);
            if (!ret)
                continue;
            else
                computeFFT->IFFT(event.data);
#else
            continue;
#endif
        }
        else
            computeFFT->FFT(event.data);
    }

}



void* Millipede::zfThread(void* context)
{
    
    Millipede* obj_ptr = ((EventHandlerContext *)context)->obj_ptr;
    moodycamel::ConcurrentQueue<Event_data>* zf_queue_ = &(obj_ptr->zf_queue_);
    int tid = ((EventHandlerContext *)context)->id;
    printf("ZF thread %d starts\n", tid);
    
#ifdef ENABLE_CPU_ATTACH
    // int offset_id = SOCKET_RX_THREAD_NUM + SOCKET_TX_THREAD_NUM + CORE_OFFSET + 2;
    int offset_id = obj_ptr->SOCKET_RX_THREAD_NUM + obj_ptr->CORE_OFFSET + 2;
    int tar_core_id = tid + offset_id;
    if (tar_core_id>=36) 
        tar_core_id = tar_core_id - 36 + 1;
    if(stick_this_thread_to_core(tar_core_id) != 0) {
        printf("ZF thread: stitch thread %d to core %d failed\n", tid, tar_core_id);
        exit(0);
    }
    else {
        printf("ZF thread: stitch thread %d to core %d succeeded\n", tid, tar_core_id);
    }
#endif
    moodycamel::ProducerToken *task_ptok_ptr = obj_ptr->task_ptoks_ptr[tid];
    // obj_ptr->task_ptok[tid].reset(new moodycamel::ProducerToken(obj_ptr->complete_task_queue_));
    // moodycamel::ProducerToken *task_ptok_ptr;
    // task_ptok_ptr = obj_ptr->task_ptok[tid].get();

    /* initialize ZF operator */
    DoZF *computeZF = new DoZF(obj_ptr->cfg_, tid, obj_ptr->zf_block_size, obj_ptr->transpose_block_size, &(obj_ptr->complete_task_queue_), task_ptok_ptr,
        obj_ptr->csi_buffer_, obj_ptr->precoder_buffer_, obj_ptr->pred_csi_buffer_, obj_ptr->ZF_task_duration, obj_ptr->ZF_task_count);

    Event_data event;
    bool ret_zf = false;

    while(true) {
        ret_zf = zf_queue_->try_dequeue(event);
        if (!ret_zf) 
            continue;
        else
            computeZF->ZF(event.data);
    }

}

void* Millipede::demulThread(void* context)
{
    
    Millipede* obj_ptr = ((EventHandlerContext *)context)->obj_ptr;
    moodycamel::ConcurrentQueue<Event_data>* demul_queue_ = &(obj_ptr->demul_queue_);
#if ENABLE_DOWNLINK
    moodycamel::ConcurrentQueue<Event_data>* precode_queue_ = &(obj_ptr->precode_queue_);
#endif
    int tid = ((EventHandlerContext *)context)->id;
    printf("Demul thread %d starts\n", tid);
    
#ifdef ENABLE_CPU_ATTACH
    // int offset_id = SOCKET_RX_THREAD_NUM + SOCKET_TX_THREAD_NUM + CORE_OFFSET + 2;
    int offset_id = obj_ptr->SOCKET_RX_THREAD_NUM + obj_ptr->CORE_OFFSET + 2;
    int tar_core_id = tid + offset_id;
    if (tar_core_id>=36) 
        tar_core_id = tar_core_id - 36 + 1;
    if(stick_this_thread_to_core(tar_core_id) != 0) {
        printf("Demul thread: stitch thread %d to core %d failed\n", tid, tar_core_id);
        exit(0);
    }
    else {
        printf("Demul thread: stitch thread %d to core %d succeeded\n", tid, tar_core_id);
    }
#endif

    moodycamel::ProducerToken *task_ptok_ptr = obj_ptr->task_ptoks_ptr[tid];
    // obj_ptr->task_ptok[tid].reset(new moodycamel::ProducerToken(obj_ptr->complete_task_queue_));
    // moodycamel::ProducerToken *task_ptok_ptr;
    // task_ptok_ptr = obj_ptr->task_ptok[tid].get();

    /* initialize Demul operator */
    DoDemul *computeDemul = new DoDemul(obj_ptr->cfg_, tid, obj_ptr->demul_block_size, obj_ptr->transpose_block_size, &(obj_ptr->complete_task_queue_), task_ptok_ptr,
        obj_ptr->data_buffer_, obj_ptr->precoder_buffer_, obj_ptr->equal_buffer_, obj_ptr->demul_hard_buffer_, obj_ptr->Demul_task_duration, obj_ptr->Demul_task_count);


    /* initialize Precode operator */
    DoPrecode *computePrecode = new DoPrecode(obj_ptr->cfg_, tid, obj_ptr->demul_block_size, obj_ptr->transpose_block_size, &(obj_ptr->complete_task_queue_), task_ptok_ptr,
        obj_ptr->dl_modulated_buffer_, obj_ptr->precoder_buffer_, obj_ptr->dl_precoded_data_buffer_, obj_ptr->dl_ifft_buffer_, obj_ptr->dl_IQ_data,  
        obj_ptr->Precode_task_duration, obj_ptr->Precode_task_count);


    Event_data event;
    bool ret_demul = false;
    bool ret_precode = false;
    int cur_frame_id = 0;

    while(true) {         
#if ENABLE_DOWNLINK
        ret_precode = precode_queue_->try_dequeue(event);
        if (!ret_precode)
            continue;
        else
            computePrecode->Precode(event.data);
#else
        ret_demul = demul_queue_->try_dequeue(event);
        if (!ret_demul) {
            continue;
        }
        else {
            // int frame_id = event.data / (OFDM_CA_NUM * ul_data_subframe_num_perframe);
            // // check precoder status for the current frame
            // if (frame_id > cur_frame_id || frame_id == 0) {
            //     while (!precoder_status_[frame_id]);
            // }
            computeDemul->Demul(event.data);
        }
#endif   
    }

}


inline void Millipede::update_frame_count(int *frame_count)
{
    *frame_count = *frame_count + 1;
    if(*frame_count == 1e9)
        *frame_count = 0;
}

void Millipede::schedule_task(Event_data do_task, moodycamel::ConcurrentQueue<Event_data> * in_queue, moodycamel::ProducerToken const& ptok) 
{
    if ( !in_queue->try_enqueue(ptok, do_task ) ) {
        printf("need more memory\n");
        if ( !in_queue->enqueue(ptok, do_task ) ) {
            printf("task enqueue failed\n");
            exit(0);
        }
    }
}

void Millipede::schedule_fft_task(int offset, int frame_id, int frame_id_in_buffer, int subframe_id, int ant_id, int prev_frame_id,
    moodycamel::ProducerToken const& ptok) 
{
    Event_data do_fft_task;
    do_fft_task.event_type = TASK_FFT;
    do_fft_task.data = offset;
    schedule_task(do_fft_task, &fft_queue_, ptok);
#if DEBUG_PRINT_PER_TASK_ENTER_QUEUE
    printf("Main thread: created FFT tasks for frame: %d, frame buffer: %d, subframe: %d, ant: %d\n", 
            frame_id, frame_id_in_buffer, subframe_id, ant_id);
#endif             
    fft_created_counter_packets_[frame_id_in_buffer]++;
    if (fft_created_counter_packets_[frame_id_in_buffer] == 1) {
        stats_manager_->update_processing_started(frame_id);
    }
    else if (fft_created_counter_packets_[frame_id_in_buffer] == max_packet_num_per_frame) {
        fft_created_counter_packets_[frame_id_in_buffer] = 0;
#if DEBUG_PRINT_PER_FRAME_ENTER_QUEUE                            
        printf("Main thread: created FFT tasks for all packets in frame: %d, frame buffer: %d in %.5f us\n", 
                frame_id, frame_id_in_buffer, get_time()-stats_manager_->get_pilot_received(frame_id));
#endif  
#if !BIGSTATION
    #if ENABLE_DOWNLINK 
        ifft_checker_[prev_frame_id] = 0;
    #else
        demul_counter_subframes_[prev_frame_id] = 0;
    #endif
#endif
    }
}

void Millipede::schedule_delayed_fft_tasks(int frame_id, int frame_id_in_buffer, int data_subframe_id, moodycamel::ProducerToken const& ptok)
{
    int frame_id_next = (frame_id_in_buffer + 1) % TASK_BUFFER_FRAME_NUM;
    if (delay_fft_queue_cnt[frame_id_next] > 0) {
        for (int i = 0; i < delay_fft_queue_cnt[frame_id_next]; i++) {
            int offset_rx = delay_fft_queue[frame_id_next][i];
            schedule_fft_task(offset_rx, frame_id + 1, frame_id_next, data_subframe_id + UE_NUM, 0, frame_id_in_buffer, ptok);
        }
        delay_fft_queue_cnt[frame_id_next] = 0;
#if DEBUG_PRINT_PER_FRAME_ENTER_QUEUE 
    #if ENABLE_DOWNLINK
        printf("Main thread in IFFT: schedule fft for %d packets for frame %d is done\n", delay_fft_queue_cnt[frame_id_next], frame_id_next);
    #else
        printf("Main thread in demul: schedule fft for %d packets for frame %d is done\n", delay_fft_queue_cnt[frame_id_next], frame_id_next);
    #endif
#endif                                
    } 
}


void Millipede::schedule_zf_task(int frame_id, moodycamel::ProducerToken const& ptok_zf) 
{
    /* schedule normal ZF for all data subcarriers */                                                                                      
    Event_data do_zf_task;
    do_zf_task.event_type = TASK_ZF;
    for (int i = 0; i < zf_block_num; i++) {
        do_zf_task.data = generateOffset2d(frame_id, i * zf_block_size);
        schedule_task(do_zf_task, &zf_queue_, ptok_zf);
    }
#if DEBUG_PRINT_PER_FRAME_ENTER_QUEUE
    printf("Main thread: created ZF tasks for frame: %d\n", frame_id);
#endif
}


void Millipede::schedule_demul_task(int frame_id, int start_sche_id, int end_sche_id, moodycamel::ProducerToken const& ptok_demul) 
{
    for (int sche_subframe_id = start_sche_id; sche_subframe_id < end_sche_id; sche_subframe_id++) {
        int data_subframe_id = cfg_->getUlSFIndex(frame_id, sche_subframe_id);
        if (data_exist_in_subframe_[frame_id][data_subframe_id]) {
            Event_data do_demul_task;
            do_demul_task.event_type = TASK_DEMUL;
            /* schedule demodulation task for subcarrier blocks */
            for(int i = 0; i < demul_block_num; i++) {
                do_demul_task.data = generateOffset3d(frame_id, data_subframe_id, i * demul_block_size);
                schedule_task(do_demul_task, &demul_queue_, ptok_demul);
            }
#if DEBUG_PRINT_PER_SUBFRAME_ENTER_QUEUE
            printf("Main thread: created Demodulation task for frame: %d,, start subframe: %d, current subframe: %d\n", 
                    frame_id, start_sche_id, data_subframe_id);
#endif                                         
            /* clear data status after scheduling */
            data_exist_in_subframe_[frame_id][data_subframe_id] = false;
        }
    }
}


void Millipede::schedule_precode_task(int frame_id, int data_subframe_id, moodycamel::ProducerToken const& ptok_precode) 
{
    Event_data do_precode_task;
    do_precode_task.event_type = TASK_PRECODE;
    for(int j = 0; j < demul_block_num; j++) {
        do_precode_task.data = generateOffset3d(frame_id, data_subframe_id, j * demul_block_size);
        schedule_task(do_precode_task, &precode_queue_, ptok_precode);
    }
}

void Millipede::schedule_ifft_task(int frame_id, int data_subframe_id, moodycamel::ProducerToken const& ptok_ifft) 
{
    Event_data do_ifft_task;
    do_ifft_task.event_type = TASK_IFFT;
    for (int i = 0; i < BS_ANT_NUM; i++) {
        do_ifft_task.data = generateOffset3d(data_subframe_id, i, frame_id);
        schedule_task(do_ifft_task, &ifft_queue_, ptok_ifft);
    }
}

void Millipede::update_rx_counters(int frame_id, int frame_id_in_buffer, int subframe_id, int ant_id)
{
    int prev_frame_id = (frame_id - 1) % TASK_BUFFER_FRAME_NUM;
    if (cfg_->isPilot(frame_id, subframe_id)) { 
        rx_counter_packets_pilots_[frame_id_in_buffer]++;
        if(rx_counter_packets_pilots_[frame_id_in_buffer] == BS_ANT_NUM * UE_NUM) {
            rx_counter_packets_pilots_[frame_id_in_buffer] = 0;
            stats_manager_->update_pilot_all_received(frame_id);
            print_per_frame_done(PRINT_RX_PILOTS, frame_id, frame_id_in_buffer);    
        }
    }
    rx_counter_packets_[frame_id_in_buffer]++;
    if (rx_counter_packets_[frame_id_in_buffer] == 1) {   
        stats_manager_->update_pilot_received(frame_id);
#if DEBUG_PRINT_PER_FRAME_START 
        printf("Main thread: data received from frame %d, subframe %d, ant %d, in %.5f us, rx in prev frame: %d\n", \
                frame_id, subframe_id, ant_id, stats_manager_->get_pilot_received(frame_id) - stats_manager_->get_pilot_received(frame_id-1),
                rx_counter_packets_[prev_frame_id]);
#endif
    }
    else if (rx_counter_packets_[frame_id_in_buffer] == max_packet_num_per_frame) {  
        stats_manager_->update_rx_processed(frame_id);
        print_per_frame_done(PRINT_RX, frame_id, frame_id_in_buffer);                        
        rx_counter_packets_[frame_id_in_buffer] = 0;                                  
    } 
}


void Millipede::print_per_frame_done(int task_type, int frame_id, int frame_id_in_buffer)
{
#if DEBUG_PRINT_PER_FRAME_DONE
    switch(task_type) {
        case(PRINT_RX): {
            int prev_frame_id = (frame_id - 1) % TASK_BUFFER_FRAME_NUM;
            printf("Main thread: received all packets in frame: %d, frame buffer: %d in %.2f us, demul: %d done, FFT: %d,%d, rx in prev frame: %d\n", 
                frame_id, frame_id_in_buffer, stats_manager_->get_rx_processed(frame_id) - stats_manager_->get_pilot_received(frame_id),
                demul_counter_subframes_[frame_id_in_buffer], data_counter_subframes_[frame_id_in_buffer], 
                fft_counter_ants_[frame_id_in_buffer][data_counter_subframes_[frame_id_in_buffer] + UE_NUM], 
                rx_counter_packets_[prev_frame_id]);
            }
            break;
        case(PRINT_RX_PILOTS):
            printf("Main thread: received all pilots in frame: %d, frame buffer: %d in %.2f us\n", frame_id, frame_id_in_buffer, 
                stats_manager_->get_pilot_all_received(frame_id) - stats_manager_->get_pilot_received(frame_id));
            break;
        case(PRINT_FFT_PILOTS):
            printf("Main thread: pilot frame: %d, %d, finished FFT for all pilot subframes in %.2f us, pilot all received: %.2f\n", 
                frame_id, frame_id_in_buffer,
                stats_manager_->get_fft_processed(frame_id) - stats_manager_->get_pilot_received(frame_id),
                stats_manager_->get_pilot_all_received(frame_id) - stats_manager_->get_pilot_received(frame_id));
            break;
        case(PRINT_FFT_DATA):
            printf("Main thread: data frame: %d, %d, finished FFT for all data subframes in %.2f us\n", 
                frame_id, frame_id_in_buffer, 
                get_time()-stats_manager_->get_pilot_received(frame_id));
            break;
        case(PRINT_ZF):
            printf("Main thread: ZF done frame: %d, %d in %.2f us since pilot FFT done, total: %.2f us\n", 
                frame_id, frame_id_in_buffer, 
                stats_manager_->get_zf_processed(frame_id) - stats_manager_->get_fft_processed(frame_id),
                stats_manager_->get_zf_processed(frame_id) - stats_manager_->get_pilot_received(frame_id));
            break;
        case(PRINT_DEMUL):
            printf("Main thread: Demodulation done frame: %d, %d in %.2f us (%d UL subframes) since ZF done, total %.2f us\n",
                frame_id, frame_id_in_buffer, ul_data_subframe_num_perframe,
                stats_manager_->get_demul_processed(frame_id) - stats_manager_->get_zf_processed(frame_id),
                stats_manager_->get_demul_processed(frame_id) - stats_manager_->get_pilot_received(frame_id));
            break;
        case(PRINT_PRECODE):
            printf("Main thread: Precoding done frame: %d, %d in %.2f us since ZF done, total: %.2f us\n", 
                frame_id, frame_id_in_buffer, 
                stats_manager_->get_precode_processed(frame_id) - stats_manager_->get_zf_processed(frame_id),
                stats_manager_->get_precode_processed(frame_id) - stats_manager_->get_pilot_received(frame_id)); 
            break;
        case(PRINT_IFFT):
            printf("Main thread: IFFT done frame: %d, %d in %.2f us since precode done, total: %.2f us\n", 
                frame_id, frame_id_in_buffer,
                stats_manager_->get_ifft_processed(frame_id) - stats_manager_->get_precode_processed(frame_id),
                stats_manager_->get_ifft_processed(frame_id) - stats_manager_->get_pilot_received(frame_id));
            break;
        case(PRINT_TX_FIRST):
            printf("Main thread: TX of first subframe done frame: %d, %d in %.2f us since ZF done, total: %.2f us\n", 
                frame_id, frame_id_in_buffer, 
                stats_manager_->get_tx_processed_first(frame_id) - stats_manager_->get_zf_processed(frame_id),
                stats_manager_->get_tx_processed_first(frame_id) - stats_manager_->get_pilot_received(frame_id));
            break;
        case(PRINT_TX):
            printf("Main thread: TX done frame: %d %d in %.2f us (%d DL subframes) since ZF done, total: %.2f us\n", 
                frame_id, frame_id_in_buffer, dl_data_subframe_num_perframe,
                stats_manager_->get_tx_processed(frame_id) - stats_manager_->get_zf_processed(frame_id),
                stats_manager_->get_tx_processed(frame_id) - stats_manager_->get_pilot_received(frame_id));
            break;
        default:
            printf("Wrong task type in frame done print!");
    }
#endif
}

void Millipede::print_per_subframe_done(int task_type, int frame_id, int frame_id_in_buffer, int subframe_id)
{
#if DEBUG_PRINT_PER_SUBFRAME_DONE
    switch(task_type) {
        case(PRINT_FFT_PILOTS):
            printf("Main thread: pilot FFT done frame: %d, %d, subframe: %d, num sumbframes done: %d\n", 
                frame_id, frame_id_in_buffer, subframe_id, csi_counter_users_[frame_id_in_buffer]);
            break;
        case(PRINT_FFT_DATA):
            printf("Main thread: data FFT done frame %d, %d, subframe %d, precoder status: %d, fft queue: %d, zf queue: %d, demul queue: %d, in %.2f\n", 
                frame_id, frame_id_in_buffer, subframe_id, precoder_exist_in_frame_[frame_id_in_buffer], 
                fft_queue_.size_approx(), zf_queue_.size_approx(), demul_queue_.size_approx(),
                get_time()-stats_manager_->get_pilot_received(frame_id));
            break;
        case(PRINT_DEMUL):
            printf("Main thread: Demodulation done frame %d %d, subframe: %d, num sumbframes done: %d\n", 
                frame_id, frame_id_in_buffer, subframe_id, demul_counter_subframes_[frame_id_in_buffer]);
            break;
        case(PRINT_PRECODE):
            printf("Main thread: Precoding done frame: %d %d, subframe: %d in %.2f us\n", 
                frame_id, frame_id_in_buffer, subframe_id,
                get_time()-stats_manager_->get_pilot_received(frame_id));
            break;
        case(PRINT_TX):
            printf("Main thread: TX done frame: %d %d, subframe: %d in %.2f us\n", 
                frame_id, frame_id_in_buffer, subframe_id,
                get_time()-stats_manager_->get_pilot_received(frame_id));
            break;
        default:
            printf("Wrong task type in frame done print!");
    }
#endif
}

void Millipede::print_per_task_done(int task_type, int frame_id, int subframe_id, int ant_or_sc_id) 
{
#if DEBUG_PRINT_PER_TASK_DONE
    switch(task_type) {
        case(PRINT_ZF):
            printf("Main thread: ZF done frame: %d, subcarrier %d\n", frame_id, ant_or_sc_id);
            break;
        case(PRINT_DEMUL):
            printf("Main thread: Demodulation done frame: %d, subframe: %d, sc: %d, num blocks done: %d\n", 
                frame_id, subframe_id, ant_or_sc_id, demul_counter_scs_[frame_id][subframe_id]);
            break;
        case(PRINT_PRECODE):
            printf("Main thread: Precoding done frame: %d, subframe: %d, subcarrier: %d, total SCs: %d\n", 
                frame_id, subframe_id, ant_or_sc_id, dl_data_counter_scs_[frame_id][subframe_id]);
            break;
        case(PRINT_IFFT):
            printf("Main thread: IFFT done frame: %d, subframe: %d, antenna: %d, total ants: %d\n", 
                frame_id, subframe_id, ant_or_sc_id, ifft_checker_[frame_id]);
            break;
        case(PRINT_TX):
            printf("Main thread: TX done frame: %d, subframe: %d, antenna: %d, total packets: %d\n", 
                frame_id, subframe_id, ant_or_sc_id, tx_counter_ants_[frame_id][subframe_id]);
            break;
        default:
            printf("Wrong task type in frame done print!");
    }

#endif
}


void Millipede::initialize_vars_from_cfg(Config *cfg)
{
    pilots_ = cfg->pilots_;
#if DEBUG_PRINT_PILOT
    cout<<"Pilot data"<<endl;
    for (int i = 0; i < OFDM_CA_NUM; i++) 
        cout<<pilots_[i]<<",";
    cout<<endl;
#endif
    BS_ANT_NUM = cfg->BS_ANT_NUM;
    UE_NUM = cfg->UE_NUM;
    OFDM_CA_NUM = cfg->OFDM_CA_NUM;
    OFDM_DATA_NUM = cfg->OFDM_DATA_NUM;
    subframe_num_perframe = cfg->symbol_num_perframe;
    data_subframe_num_perframe = cfg->data_symbol_num_perframe;
    ul_data_subframe_num_perframe = cfg->ul_data_symbol_num_perframe;
    dl_data_subframe_num_perframe = cfg->dl_data_symbol_num_perframe;
    dl_data_subframe_start = cfg->dl_data_symbol_start;
    dl_data_subframe_end = cfg->dl_data_symbol_end;
    package_length = cfg->package_length;

    TASK_THREAD_NUM = cfg->worker_thread_num;
    SOCKET_RX_THREAD_NUM = cfg->socket_thread_num;
    SOCKET_TX_THREAD_NUM = cfg->socket_thread_num;
    FFT_THREAD_NUM = cfg->fft_thread_num;
    DEMUL_THREAD_NUM = cfg->demul_thread_num;
    ZF_THREAD_NUM = cfg->zf_thread_num;
    CORE_OFFSET = cfg->core_offset;
    demul_block_size = cfg->demul_block_size;
    zf_block_size = cfg->zf_block_size;
    demul_block_num = OFDM_DATA_NUM / demul_block_size + (OFDM_DATA_NUM % demul_block_size == 0 ? 0 : 1);
    zf_block_num = OFDM_DATA_NUM/zf_block_size + (OFDM_DATA_NUM % zf_block_size == 0 ? 0 : 1);
}


void Millipede::initialize_queues() 
{
    message_queue_ = moodycamel::ConcurrentQueue<Event_data>(512 * data_subframe_num_perframe);
    complete_task_queue_ = moodycamel::ConcurrentQueue<Event_data>(512 * data_subframe_num_perframe * 4);

    fft_queue_ = moodycamel::ConcurrentQueue<Event_data>(512 * data_subframe_num_perframe * 4);
    zf_queue_ = moodycamel::ConcurrentQueue<Event_data>(512 * data_subframe_num_perframe * 4);;
    demul_queue_ = moodycamel::ConcurrentQueue<Event_data>(512 * data_subframe_num_perframe * 4);
    decode_queue_ = moodycamel::ConcurrentQueue<Event_data>(512 * data_subframe_num_perframe * 4);
    

    ifft_queue_ = moodycamel::ConcurrentQueue<Event_data>(512 * data_subframe_num_perframe * 4);
    modulate_queue_ = moodycamel::ConcurrentQueue<Event_data>(512 * data_subframe_num_perframe * 4);
    precode_queue_ = moodycamel::ConcurrentQueue<Event_data>(512 * data_subframe_num_perframe * 4);
    tx_queue_ = moodycamel::ConcurrentQueue<Event_data>(512 * data_subframe_num_perframe * 4);

    rx_ptoks_ptr = (moodycamel::ProducerToken **)aligned_alloc(64, SOCKET_RX_THREAD_NUM * sizeof(moodycamel::ProducerToken *));
    for (int i = 0; i < SOCKET_RX_THREAD_NUM; i++) 
        rx_ptoks_ptr[i] = new moodycamel::ProducerToken(message_queue_);

    tx_ptoks_ptr = (moodycamel::ProducerToken **)aligned_alloc(64, SOCKET_RX_THREAD_NUM * sizeof(moodycamel::ProducerToken *));
    for (int i = 0; i < SOCKET_RX_THREAD_NUM; i++) 
        tx_ptoks_ptr[i] = new moodycamel::ProducerToken(tx_queue_);

    task_ptoks_ptr = (moodycamel::ProducerToken **)aligned_alloc(64, TASK_THREAD_NUM * sizeof(moodycamel::ProducerToken *));
    for (int i = 0; i < TASK_THREAD_NUM; i++)
        task_ptoks_ptr[i] = new moodycamel::ProducerToken(complete_task_queue_);
}


void Millipede::initialize_uplink_buffers()
{
    alloc_buffer_1d(&task_threads, TASK_THREAD_NUM, 64, 0);
    alloc_buffer_1d(&context, TASK_THREAD_NUM, 64, 0);
    // task_threads = (pthread_t *)malloc(TASK_THREAD_NUM * sizeof(pthread_t));
    // context = (EventHandlerContext *)malloc(TASK_THREAD_NUM * sizeof(EventHandlerContext));

    socket_buffer_size_ = (long long) package_length * subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM; 
    socket_buffer_status_size_ = subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM;
    alloc_buffer_2d(&socket_buffer_, SOCKET_RX_THREAD_NUM, socket_buffer_size_, 64, 0);
    alloc_buffer_2d(&socket_buffer_status_, SOCKET_RX_THREAD_NUM, socket_buffer_status_size_, 64, 1);
    alloc_buffer_2d(&csi_buffer_ , UE_NUM * TASK_BUFFER_FRAME_NUM, BS_ANT_NUM * OFDM_DATA_NUM, 64, 0);
    alloc_buffer_2d(&data_buffer_ , data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM, BS_ANT_NUM * OFDM_DATA_NUM, 64, 0);
    alloc_buffer_2d(&pred_csi_buffer_ , OFDM_DATA_NUM, BS_ANT_NUM * UE_NUM, 64, 0);
    alloc_buffer_2d(&precoder_buffer_ , OFDM_DATA_NUM * TASK_BUFFER_FRAME_NUM, UE_NUM * BS_ANT_NUM, 64, 0);
    alloc_buffer_2d(&equal_buffer_ , data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM, OFDM_DATA_NUM * UE_NUM, 64, 0);
    alloc_buffer_2d(&demul_hard_buffer_ , data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM, OFDM_DATA_NUM * UE_NUM, 64, 0);
    alloc_buffer_2d(&decoded_buffer_ , data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM, NUM_BITS * ORIG_CODE_LEN * NUM_CODE_BLOCK * UE_NUM, 64, 0);

    /* initilize all uplink status checkers */
    alloc_buffer_1d(&rx_counter_packets_, TASK_BUFFER_FRAME_NUM, 64, 1);
    alloc_buffer_1d(&rx_counter_packets_pilots_, TASK_BUFFER_FRAME_NUM, 64, 1);
    alloc_buffer_1d(&csi_counter_users_, TASK_BUFFER_FRAME_NUM, 64, 1);
    alloc_buffer_1d(&data_counter_subframes_, TASK_BUFFER_FRAME_NUM, 64, 1);
    alloc_buffer_1d(&precoder_counter_scs_, TASK_BUFFER_FRAME_NUM, 64, 1);
    alloc_buffer_1d(&demul_counter_subframes_, TASK_BUFFER_FRAME_NUM, 64, 1);
    alloc_buffer_1d(&fft_created_counter_packets_, TASK_BUFFER_FRAME_NUM, 64, 1);
    alloc_buffer_1d(&precoder_exist_in_frame_, TASK_BUFFER_FRAME_NUM, 64, 1);
    alloc_buffer_1d(&decode_counter_subframes_, TASK_BUFFER_FRAME_NUM, 64, 1);

    alloc_buffer_2d(&fft_counter_ants_, TASK_BUFFER_FRAME_NUM, subframe_num_perframe, 64, 1);

    alloc_buffer_2d(&data_exist_in_subframe_, TASK_BUFFER_FRAME_NUM, data_subframe_num_perframe, 64, 1);
    alloc_buffer_2d(&demul_counter_scs_, TASK_BUFFER_FRAME_NUM, data_subframe_num_perframe, 64, 1);
    alloc_buffer_2d(&decode_counter_blocks_, TASK_BUFFER_FRAME_NUM, data_subframe_num_perframe, 64, 1);

    alloc_buffer_2d(&delay_fft_queue, TASK_BUFFER_FRAME_NUM, subframe_num_perframe * BS_ANT_NUM, 32, 1);
    alloc_buffer_1d(&delay_fft_queue_cnt, TASK_BUFFER_FRAME_NUM, 32, 1);

    /* initilize all timestamps and counters for worker threads */
    alloc_buffer_2d(&CSI_task_duration, TASK_THREAD_NUM * 8, 4, 64, 1);
    alloc_buffer_2d(&FFT_task_duration, TASK_THREAD_NUM * 8, 4, 64, 1);
    alloc_buffer_2d(&ZF_task_duration, TASK_THREAD_NUM * 8, 4, 64, 1);
    alloc_buffer_2d(&Demul_task_duration, TASK_THREAD_NUM * 8, 4, 64, 1);

    alloc_buffer_1d(&CSI_task_count, TASK_THREAD_NUM * 16, 64, 1);
    alloc_buffer_1d(&FFT_task_count, TASK_THREAD_NUM * 16, 64, 1);
    alloc_buffer_1d(&ZF_task_count, TASK_THREAD_NUM * 16, 64, 1);
    alloc_buffer_1d(&Demul_task_count, TASK_THREAD_NUM * 16, 64, 1);

    alloc_buffer_2d(&frame_start, SOCKET_RX_THREAD_NUM, 10240, 4096, 1);
}


void Millipede::initialize_downlink_buffers()
{
    alloc_buffer_2d(&dl_IQ_data , data_subframe_num_perframe * UE_NUM, OFDM_CA_NUM, 64, 0);
    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string filename1 = cur_directory + "/data/orig_data_2048_ant" + std::to_string(BS_ANT_NUM) + ".bin";
    FILE *fp = fopen(filename1.c_str(),"rb");
    if (fp==NULL) {
        printf("open file faild");
        std::cerr << "Error: " << strerror(errno) << std::endl;
    }
    for (int i = 0; i < data_subframe_num_perframe * UE_NUM; i++) {
        fread(dl_IQ_data[i], sizeof(int), OFDM_CA_NUM, fp);
    }
    fclose(fp);


    dl_socket_buffer_size_ = (long long) data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM * package_length * BS_ANT_NUM;
    dl_socket_buffer_status_size_ = data_subframe_num_perframe * BS_ANT_NUM * TASK_BUFFER_FRAME_NUM;
    alloc_buffer_1d(&dl_socket_buffer_, dl_socket_buffer_size_, 64, 0);
    alloc_buffer_1d(&dl_socket_buffer_status_, dl_socket_buffer_status_size_, 64, 1);
    alloc_buffer_2d(&dl_ifft_buffer_, BS_ANT_NUM * data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM, OFDM_CA_NUM, 64, 1);
    alloc_buffer_2d(&dl_precoded_data_buffer_, data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM, BS_ANT_NUM * OFDM_DATA_NUM, 64, 0);
    alloc_buffer_2d(&dl_modulated_buffer_, data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM, UE_NUM * OFDM_DATA_NUM, 64, 0);


    /* initilize all downlink status checkers */
    alloc_buffer_1d(&dl_data_counter_subframes_, TASK_BUFFER_FRAME_NUM, 64, 1);
    alloc_buffer_1d(&ifft_checker_, TASK_BUFFER_FRAME_NUM, 64, 1);
    alloc_buffer_1d(&tx_counter_subframes_, TASK_BUFFER_FRAME_NUM, 64, 1);

    alloc_buffer_2d(&dl_data_counter_scs_, TASK_BUFFER_FRAME_NUM, data_subframe_num_perframe, 64, 1);
    alloc_buffer_2d(&modulate_checker_, TASK_BUFFER_FRAME_NUM, data_subframe_num_perframe, 64, 1);
    alloc_buffer_2d(&tx_counter_ants_, TASK_BUFFER_FRAME_NUM, data_subframe_num_perframe, 64, 1);

    /* initilize all timestamps and counters for worker threads */
    alloc_buffer_2d(&IFFT_task_duration, TASK_THREAD_NUM * 8, 4, 64, 1);
    alloc_buffer_2d(&Precode_task_duration, TASK_THREAD_NUM * 8, 4, 64, 1);

    alloc_buffer_1d(&IFFT_task_count, TASK_THREAD_NUM * 16, 64, 1);
    alloc_buffer_1d(&Precode_task_count, TASK_THREAD_NUM * 16, 64, 1);
}


void Millipede::free_uplink_buffers()
{
    free_buffer_1d(&pilots_);
    free_buffer_2d(&socket_buffer_, SOCKET_RX_THREAD_NUM);
    free_buffer_2d(&socket_buffer_status_, SOCKET_RX_THREAD_NUM);
    free_buffer_2d(&csi_buffer_, UE_NUM * TASK_BUFFER_FRAME_NUM);
    free_buffer_2d(&data_buffer_, data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM);
    free_buffer_2d(&pred_csi_buffer_ , OFDM_DATA_NUM);
    free_buffer_2d(&precoder_buffer_ , OFDM_DATA_NUM * TASK_BUFFER_FRAME_NUM);
    free_buffer_2d(&equal_buffer_ , data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM);
    free_buffer_2d(&demul_hard_buffer_ , data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM);
    free_buffer_2d(&decoded_buffer_ , data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM);

    free_buffer_1d(&rx_counter_packets_);
    free_buffer_1d(&rx_counter_packets_pilots_);
    free_buffer_1d(&csi_counter_users_);
    free_buffer_1d(&data_counter_subframes_);
    free_buffer_1d(&precoder_counter_scs_);
    free_buffer_1d(&demul_counter_subframes_);
    free_buffer_1d(&fft_created_counter_packets_);
    free_buffer_1d(&precoder_exist_in_frame_);
    free_buffer_1d(&decode_counter_subframes_);

    free_buffer_2d(&fft_counter_ants_, TASK_BUFFER_FRAME_NUM);

    free_buffer_2d(&data_exist_in_subframe_, TASK_BUFFER_FRAME_NUM);
    free_buffer_2d(&demul_counter_scs_, TASK_BUFFER_FRAME_NUM);
    free_buffer_2d(&decode_counter_blocks_, TASK_BUFFER_FRAME_NUM);

    free_buffer_2d(&delay_fft_queue, TASK_BUFFER_FRAME_NUM);
    free_buffer_1d(&delay_fft_queue_cnt);


    free_buffer_2d(&CSI_task_duration, TASK_THREAD_NUM * 8);
    free_buffer_2d(&FFT_task_duration, TASK_THREAD_NUM * 8);
    free_buffer_2d(&ZF_task_duration, TASK_THREAD_NUM * 8);
    free_buffer_2d(&Demul_task_duration, TASK_THREAD_NUM * 8);

    free_buffer_1d(&CSI_task_count);
    free_buffer_1d(&FFT_task_count);
    free_buffer_1d(&ZF_task_count);
    free_buffer_1d(&Demul_task_count);
}

void Millipede::free_downlink_buffers()
{
    free_buffer_2d(&dl_IQ_data , data_subframe_num_perframe * UE_NUM);

    free_buffer_1d(&dl_socket_buffer_);
    free_buffer_1d(&dl_socket_buffer_status_);

    free_buffer_2d(&dl_ifft_buffer_, BS_ANT_NUM * data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM);
    free_buffer_2d(&dl_precoded_data_buffer_, data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM);
    free_buffer_2d(&dl_modulated_buffer_, data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM);


    free_buffer_1d(&dl_data_counter_subframes_);
    free_buffer_1d(&ifft_checker_);
    free_buffer_1d(&tx_counter_subframes_);

    free_buffer_2d(&dl_data_counter_scs_, TASK_BUFFER_FRAME_NUM);
    free_buffer_2d(&modulate_checker_, TASK_BUFFER_FRAME_NUM);
    free_buffer_2d(&tx_counter_ants_, TASK_BUFFER_FRAME_NUM);

    free_buffer_2d(&IFFT_task_duration, TASK_THREAD_NUM * 8);
    free_buffer_2d(&Precode_task_duration, TASK_THREAD_NUM * 8);

    free_buffer_1d(&IFFT_task_count);
    free_buffer_1d(&Precode_task_count);
}



void Millipede::save_demul_data_to_file()
{
#if WRITE_DEMUL
    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string filename = cur_directory + "/data/demul_data.txt";
    FILE* fp = fopen(filename.c_str(),"a");
    for (int cc = 0; cc < OFDM_DATA_NUM; cc++) {
        int *cx = &demul_hard_buffer_[total_data_subframe_id][cc * UE_NUM];
        fprintf(fp, "SC: %d, Frame %d, subframe: %d, ", cc, frame_id, data_subframe_id);
        for(int kk = 0; kk < UE_NUM; kk++)  
            fprintf(fp, "%d ", cx[kk]);
        fprintf(fp, "\n");
    }
    fclose(fp);
#endif    
}



void Millipede::getDemulData(int **ptr, int *size)
{
    *ptr = (int *)&equal_buffer_[max_equaled_frame * data_subframe_num_perframe][0];
    *size = UE_NUM * OFDM_CA_NUM;
}

void Millipede::getEqualData(float **ptr, int *size)
{
    // max_equaled_frame = 0;
    *ptr = (float *)&equal_buffer_[max_equaled_frame * data_subframe_num_perframe][0];
    // *ptr = equal_output;
    *size = UE_NUM*OFDM_DATA_NUM*2;
    
    //printf("In getEqualData()\n");
    //for(int ii = 0; ii < UE_NUM*OFDM_DATA_NUM; ii++)
    //{
    //    // printf("User %d: %d, ", ii,demul_ptr2(ii));
    //    printf("[%.4f+j%.4f] ", *(*ptr+ii*UE_NUM*2), *(*ptr+ii*UE_NUM*2+1));
    //}
    //printf("\n");
    //printf("\n");
    
}



extern "C"
{
    EXPORT Millipede* Millipede_new(Config *cfg) {
        // printf("Size of Millipede: %d\n",sizeof(Millipede *));
        Millipede *millipede = new Millipede(cfg);
        
        return millipede;
    }
    EXPORT void Millipede_start(Millipede *millipede) {millipede->start();}
    EXPORT void Millipede_stop(Millipede *millipede) {millipede->stop();}
    EXPORT void Millipede_destroy(Millipede *millipede) {delete millipede;}
    EXPORT void Millipede_getEqualData(Millipede *millipede, float **ptr, int *size) {return millipede->getEqualData(ptr, size);}
    EXPORT void Millipede_getDemulData(Millipede *millipede, int **ptr, int *size) {return millipede->getDemulData(ptr, size);}
}





