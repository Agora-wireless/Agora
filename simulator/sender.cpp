/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#include "sender.hpp"

bool keep_running = true;

void intHandler(int)
{
    std::cout << "will exit..." << std::endl;
    keep_running = false;
}

void delay_pause(size_t us)
{
    // double start_time = get_time();
    const uint64_t start = RDTSC();
    const uint64_t ticks = (uint64_t)us * CPU_FREQ / 1E6;
    while ((RDTSC() - start) < ticks)
        _mm_pause();
    // double duration = get_time() - start_time;
    // printf("duration: %.5f, us %d\n", duration, us);
}

Sender::Sender(
    Config* cfg, size_t in_thread_num, size_t in_core_offset, size_t in_delay)
    : cfg(cfg)
    , cur_ptr_(0)
    , ant_id(0)
    , frame_id(0)
    , subframe_id(0)
    , thread_num(in_thread_num)
    , socket_num(in_thread_num)
    , core_offset(in_core_offset)
    , delay(in_delay)
{
    printf("TX constructer: on core %d\n", sched_getcpu());

    size_t buffer_length = tx_buf_offset + cfg->packet_length;
    size_t max_subframe_id = cfg->downlink_mode ? cfg->pilot_symbol_num_perframe
                                                : cfg->symbol_num_perframe;

    size_t max_length_ = BUFFER_FRAME_NUM * max_subframe_id * cfg->BS_ANT_NUM;

    packet_count_per_subframe.calloc(BUFFER_FRAME_NUM, max_subframe_id, 64);
    alloc_buffer_1d(&packet_count_per_frame, BUFFER_FRAME_NUM, 64, 1);

    socket_ = new int[socket_num];
    for (size_t i = 0; i < socket_num; i++) {
#if USE_IPV4
        socket_[i] = setup_socket_ipv4(cfg->ue_tx_port + i, false, 0);
        setup_sockaddr_remote_ipv4(
            &servaddr_[i], cfg->bs_port + i, cfg->rx_addr.c_str());
        memset(servaddr_[i].sin_zero, 0, sizeof(servaddr_[i].sin_zero));
#else
        socket_[i] = setup_socket_ipv6(cfg->ue_tx_port + i, false, 0);
        setup_sockaddr_remote_ipv6(
            &servaddr_[i], cfg->bs_port + i, "fe80::f436:d735:b04a:864a");
#endif

#if !defined(USE_DPDK) && CONNECT_UDP
        if (connect(socket_[i], (struct sockaddr*)&servaddr_[i],
                sizeof(servaddr_[i]))
            != 0)
            perror("UDP socket connect failed");
        else
            printf("UDP socket %zu connected\n", i);
#endif
    }

    size_t IQ_data_size = cfg->symbol_num_perframe * cfg->BS_ANT_NUM;
    IQ_data.calloc(IQ_data_size, cfg->OFDM_FRAME_LEN * 2, 64);
    IQ_data_coded.calloc(IQ_data_size, cfg->OFDM_FRAME_LEN * 2, 64);
    trans_buffer_.calloc(max_length_, buffer_length, 64);

    /* read from file */
    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
#ifdef USE_LDPC
    std::string filename = cur_directory + "/data/LDPC_rx_data_2048_ant"
        + std::to_string(cfg->BS_ANT_NUM) + ".bin";
#else
    std::string filename = cur_directory + "/data/rx_data_2048_ant"
        + std::to_string(cfg->BS_ANT_NUM) + ".bin";
#endif

    FILE* fp = fopen(filename.c_str(), "rb");
    if (fp == NULL) {
        printf("open file failed: %s\n", filename.c_str());
        std::cerr << "Error: " << strerror(errno) << std::endl;
    }

    for (size_t i = 0; i < cfg->symbol_num_perframe * cfg->BS_ANT_NUM; i++) {
        size_t expect_num_bytes = cfg->OFDM_FRAME_LEN * 2;
        size_t actual_bytes
            = fread(IQ_data[i], sizeof(float), expect_num_bytes, fp);

        if (expect_num_bytes != actual_bytes) {
            printf("read file failed: %s\n", filename.c_str());
            printf("i: %zu, expected: %zu, actual: %zu\n", i, expect_num_bytes,
                actual_bytes);
            std::cerr << "Error: " << strerror(errno) << std::endl;
        }

        for (size_t j = 0; j < cfg->OFDM_FRAME_LEN * 2; j++) {
            IQ_data_coded[i][j] = (ushort)(IQ_data[i][j] * 32768);
            // printf("i:%d, j:%d, Coded: %d, orignal:
            // %.4f\n",i,j/2,IQ_data_coded[i][j],IQ_data[i][j]);
        }
    }
    fclose(fp);

    task_ptok = (moodycamel::ProducerToken**)aligned_alloc(
        64, thread_num * sizeof(moodycamel::ProducerToken*));
    for (size_t i = 0; i < thread_num; i++) {
        task_ptok[i] = new moodycamel::ProducerToken(task_queue_);
    }
}

Sender::~Sender()
{
    IQ_data_coded.free();
    IQ_data.free();

    delete[] socket_;
    delete cfg;
    pthread_mutex_destroy(&lock_);
}

void Sender::startTX()
{
    // printf("start sender\n");
    // double frame_start[10240] __attribute__( ( aligned (4096) ) );
    // double frame_end[10240] __attribute__( ( aligned (4096) ) ) ;
    alloc_buffer_1d(&frame_start, 10240, 4096, 1);
    alloc_buffer_1d(&frame_end, 10240, 4096, 1);
    /* create send threads */
    std::vector<pthread_t> created_threads;
    for (size_t i = 0; i < thread_num; i++) {
        pthread_t send_thread_;
        EventHandlerContext<Sender>* context
            = (EventHandlerContext<Sender>*)malloc(sizeof(*context));
        context->obj_ptr = this;
        context->id = i;
        if (pthread_create(&send_thread_, NULL,
                pthread_fun_wrapper<Sender, &Sender::loopSend>, context)
            != 0) {
            perror("socket send thread create failed");
            exit(0);
        }
        created_threads.push_back(send_thread_);
    }

    /* give some time for all threads to lock */
    sleep(1);
    printf("Master: Now releasing the condition\n");
    pthread_cond_broadcast(&cond);

    /* load data to buffer */
    size_t max_subframe_id = cfg->downlink_mode
        ? cfg->pilot_symbol_num_perframe
        : cfg->pilot_symbol_num_perframe + cfg->data_symbol_num_perframe;

    size_t max_length_ = BUFFER_FRAME_NUM * max_subframe_id * cfg->BS_ANT_NUM;
    size_t cell_id = 0;

    for (size_t i = 0; i < max_length_; i++) {
        cur_ptr_ = i;
        size_t data_index = subframe_id * cfg->BS_ANT_NUM + ant_id;
        auto* pkt = (struct Packet*)(trans_buffer_[cur_ptr_] + tx_buf_offset);

        pkt->frame_id = frame_id;
        pkt->symbol_id = (subframe_id < cfg->pilot_symbol_num_perframe)
            ? cfg->pilotSymbols[0][subframe_id]
            : cfg->ULSymbols[0][subframe_id - cfg->pilot_symbol_num_perframe];
        pkt->cell_id = cell_id;
        pkt->ant_id = ant_id;
        memcpy(pkt->data, (char*)IQ_data_coded[data_index],
            sizeof(ushort) * cfg->OFDM_FRAME_LEN * 2);

        ant_id++;
        if (ant_id == cfg->BS_ANT_NUM) {
            ant_id = 0;
            subframe_id++;
            if (subframe_id == max_subframe_id) {
                subframe_id = 0;
                frame_id++;
                if (frame_id == MAX_FRAME_ID)
                    frame_id = 0;
            }
        }
    }

#if DEBUG_SENDER
    double start_time = get_time();
#endif
    size_t tx_frame_count = 0;
    uint64_t ticks_5 = (uint64_t)500000 * CPU_FREQ / 1e6 / 70;
    uint64_t ticks_100 = (uint64_t)150000 * CPU_FREQ / 1e6 / 70;
    uint64_t ticks_200 = (uint64_t)20000 * CPU_FREQ / 1e6 / 70;
    uint64_t ticks_500 = (uint64_t)10000 * CPU_FREQ / 1e6 / 70;
    uint64_t ticks_all = (uint64_t)delay * CPU_FREQ / 1e6 / 70;

    // Push tasks of the first subframe into task queue
    for (size_t i = 0; i < cfg->BS_ANT_NUM; i++) {
        size_t ptok_id = i % thread_num;
        if (!task_queue_.enqueue(*task_ptok[ptok_id], i)) {
            printf("send task enqueue failed\n");
            exit(0);
        }
    }

    int ret;
    signal(SIGINT, intHandler);

    uint64_t tick_start = RDTSC();
    while (keep_running) {
        size_t data_ptr;
        ret = message_queue_.try_dequeue(data_ptr);
        if (!ret)
            continue;

        size_t tx_ant_id = data_ptr % cfg->BS_ANT_NUM;
        size_t data_index = subframe_id * cfg->BS_ANT_NUM + tx_ant_id;
        auto pkt = (struct Packet*)(trans_buffer_[data_ptr] + tx_buf_offset);

        pkt->frame_id = frame_id;
        pkt->symbol_id = (subframe_id < cfg->pilot_symbol_num_perframe)
            ? cfg->pilotSymbols[0][subframe_id]
            : cfg->ULSymbols[0][subframe_id - cfg->pilot_symbol_num_perframe];

        pkt->cell_id = cell_id;
        pkt->ant_id = ant_id;
        memcpy(pkt->data, (char*)IQ_data_coded[data_index],
            sizeof(ushort) * cfg->OFDM_FRAME_LEN * 2);

        size_t tx_total_subframe_id = data_ptr / cfg->BS_ANT_NUM;
        size_t tx_current_subframe_id = tx_total_subframe_id % max_subframe_id;
        size_t tx_frame_id = tx_total_subframe_id / max_subframe_id;
        packet_count_per_subframe[tx_frame_id][tx_current_subframe_id]++;

        // printf("data_ptr: %d, tx_frame_id: %d, tx_total_subframe_id: %d,
        // tx_current_subframe_id %d, tx_ant_id %d, max_subframe_id %d\n",
        // data_ptr,
        //   tx_frame_id, tx_total_subframe_id, tx_current_subframe_id,
        //   tx_ant_id, max_subframe_id);

        if (packet_count_per_subframe[tx_frame_id][tx_current_subframe_id]
            == cfg->BS_ANT_NUM) {
            packet_count_per_frame[tx_frame_id]++;
            // double cur_time = get_time();
            // printf("Finished transmit all antennas in frame: %d, subframe:
            // %d, at
            // %.5f in %.5f us\n", tx_frame_id, tx_current_subframe_id,
            // cur_time,cur_time-start_time);
            if (tx_frame_count <= 5) {
                while ((RDTSC() - tick_start) < ticks_5)
                    _mm_pause();
            } else if (tx_frame_count < 100) {
                while ((RDTSC() - tick_start) < ticks_100)
                    _mm_pause();
            } else if (tx_frame_count < 200) {
                while ((RDTSC() - tick_start) < ticks_200)
                    _mm_pause();
            } else if (tx_frame_count < 500) {
                while ((RDTSC() - tick_start) < ticks_500)
                    _mm_pause();
            } else {
                while ((RDTSC() - tick_start) < ticks_all)
                    _mm_pause();
            }

            tick_start = RDTSC();

            if (packet_count_per_frame[tx_frame_id] == max_subframe_id) {
                frame_end[tx_frame_count] = get_time();
                tx_frame_count++;
                if (tx_frame_count == (size_t)cfg->tx_frame_num)
                    break;
                packet_count_per_frame[tx_frame_id] = 0;
                if (cfg->downlink_mode) {
                    if (frame_id < 500) {
                        while ((RDTSC() - tick_start)
                            < 2 * cfg->data_symbol_num_perframe * ticks_all)
                            _mm_pause();
                    } else {
                        while ((RDTSC() - tick_start)
                            < cfg->data_symbol_num_perframe * ticks_all)
                            _mm_pause();
                    }
                }
#if DEBUG_SENDER
                printf("Finished transmit all antennas in frame: %d, "
                       "next scheduled: %d, in %.5f us\n",
                    tx_frame_count, frame_id, get_time() - start_time);
                start_time = get_time();
#endif
                tick_start = RDTSC();
            }
            packet_count_per_subframe[tx_frame_id][tx_current_subframe_id] = 0;
            size_t next_subframe_ptr
                = ((tx_total_subframe_id + 1) * cfg->BS_ANT_NUM) % max_length_;
            for (size_t i = 0; i < cfg->BS_ANT_NUM; i++) {
                size_t ptok_id = i % thread_num;
                if (!task_queue_.enqueue(
                        *task_ptok[ptok_id], i + next_subframe_ptr)) {
                    printf("send task enqueue failed\n");
                    exit(0);
                }
            }
        }

        ant_id++;
        if (ant_id == cfg->BS_ANT_NUM) {
            ant_id = 0;
            subframe_id++;
            if (subframe_id == max_subframe_id) {
                subframe_id = 0;
                frame_id++;
                if (frame_id == MAX_FRAME_ID)
                    frame_id = 0;
            }
        }
    }

    printf("printing sender results to file...\n");

    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string filename = cur_directory + "/matlab/tx_result.txt";
    FILE* fp_debug = fopen(filename.c_str(), "w");
    if (fp_debug == NULL) {
        printf("open file faild");
        std::cerr << "Error: " << strerror(errno) << std::endl;
        exit(0);
    }

    for (size_t ii = 0; ii < tx_frame_count; ii++) {
        fprintf(fp_debug, "%.5f\n", frame_end[ii]);
    }
    exit(0);
}

void Sender::startTXfromMain(double* in_frame_start, double* in_frame_end)
{
    printf("start sender\n");
    frame_start = in_frame_start;
    frame_end = in_frame_end;

    /* create send threads */
    std::vector<pthread_t> created_threads;
    for (size_t i = 0; i < thread_num; i++) {
        pthread_t send_thread_;
        EventHandlerContext<Sender>* context
            = (EventHandlerContext<Sender>*)malloc(sizeof(*context));
        context->obj_ptr = this;
        context->id = i;
        if (pthread_create(&send_thread_, NULL,
                pthread_fun_wrapper<Sender, &Sender::loopSend>, context)
            != 0) {
            // if(pthread_create( &send_thread_, NULL, Sender::loopSend, (void
            // *)(&context[i])) != 0) {
            perror("socket send thread create failed");
            exit(0);
        }
        created_threads.push_back(send_thread_);
    }

    /* give some time for all threads to lock */
    sleep(1);
    printf("Master: Now releasing the condition\n");
    pthread_cond_broadcast(&cond);

    pthread_t main_send_thread_;
    EventHandlerContext<Sender>* context
        = (EventHandlerContext<Sender>*)malloc(sizeof(*context));
    context->obj_ptr = this;
    context->id = thread_num;
    if (pthread_create(&main_send_thread_, NULL,
            pthread_fun_wrapper<Sender, &Sender::loopSend_main>, context)
        != 0) {
        // if(pthread_create( &main_send_thread_, NULL, Sender::loopSend_main,
        // (void
        // *)(&context[thread_num])) != 0) {
        perror("socket main send thread create failed");
        exit(0);
    }
    // printf("Created main tx thread\n");
    created_threads.push_back(main_send_thread_);
}

void* Sender::loopSend_main(int)
{
    pin_to_core_with_offset(ThreadType::kMasterTX, core_offset, 0);

    size_t max_subframe_id = cfg->downlink_mode
        ? cfg->pilot_symbol_num_perframe
        : cfg->pilot_symbol_num_perframe + cfg->data_symbol_num_perframe;
    size_t max_length_ = BUFFER_FRAME_NUM * max_subframe_id * cfg->BS_ANT_NUM;

    // double frame_start[10240] __attribute__( ( aligned (4096) ) );
    // double frame_end[10240] __attribute__( ( aligned (4096) ) ) ;

    /* load data to buffer */
    size_t cell_id = 0;
    for (size_t i = 0; i < max_length_; i++) {
        cur_ptr_ = i;
        size_t data_index = subframe_id * cfg->BS_ANT_NUM + ant_id;
        auto* pkt = (struct Packet*)(trans_buffer_[cur_ptr_] + tx_buf_offset);

        pkt->frame_id = frame_id;
        pkt->symbol_id = (subframe_id < cfg->pilot_symbol_num_perframe)
            ? cfg->pilotSymbols[0][subframe_id]
            : cfg->ULSymbols[0][subframe_id - cfg->pilot_symbol_num_perframe];
        pkt->cell_id = cell_id;
        pkt->ant_id = ant_id;
        memcpy(pkt->data, (char*)IQ_data_coded[data_index],
            sizeof(ushort) * cfg->OFDM_FRAME_LEN * 2);

        ant_id++;
        if (ant_id == cfg->BS_ANT_NUM) {
            ant_id = 0;
            subframe_id++;
            if (subframe_id == max_subframe_id) {
                subframe_id = 0;
                frame_id++;
                if (frame_id == MAX_FRAME_ID)
                    frame_id = 0;
            }
        }
    }

    // double start_time = get_time();
    size_t tx_frame_count = 0;
    uint64_t ticks_5 = (uint64_t)500000 * CPU_FREQ / 1e6 / 70;
    uint64_t ticks_100 = (uint64_t)150000 * CPU_FREQ / 1e6 / 70;
    uint64_t ticks_200 = (uint64_t)20000 * CPU_FREQ / 1e6 / 70;
    uint64_t ticks_500 = (uint64_t)10000 * CPU_FREQ / 1e6 / 70;
    uint64_t ticks_all = (uint64_t)delay * CPU_FREQ / 1e6 / 70;

    // push tasks of the first subframe into task queue
    for (size_t i = 0; i < cfg->BS_ANT_NUM; i++) {
        size_t ptok_id = i % thread_num;
        if (!task_queue_.enqueue(*task_ptok[ptok_id], i)) {
            printf("send task enqueue failed\n");
            exit(0);
        }
    }

    int ret;
    signal(SIGINT, intHandler);

    frame_start[0] = get_time();
    uint64_t tick_start = RDTSC();
    while (keep_running) {
        size_t data_ptr;
        ret = message_queue_.try_dequeue(data_ptr);
        if (!ret)
            continue;
        size_t tx_ant_id = data_ptr % cfg->BS_ANT_NUM;
        size_t data_index = subframe_id * cfg->BS_ANT_NUM + tx_ant_id;
        auto* pkt = (struct Packet*)(trans_buffer_[data_ptr] + tx_buf_offset);

        pkt->frame_id = frame_id;
        pkt->symbol_id = (subframe_id < cfg->pilot_symbol_num_perframe)
            ? cfg->pilotSymbols[0][subframe_id]
            : cfg->ULSymbols[0][subframe_id - cfg->pilot_symbol_num_perframe];
        pkt->cell_id = cell_id;
        pkt->ant_id = ant_id;
        memcpy(pkt->data, (char*)IQ_data_coded[data_index],
            sizeof(ushort) * cfg->OFDM_FRAME_LEN * 2);

        size_t tx_total_subframe_id = data_ptr / cfg->BS_ANT_NUM;
        size_t tx_current_subframe_id = tx_total_subframe_id % max_subframe_id;
        size_t tx_frame_id = tx_total_subframe_id / max_subframe_id;
        packet_count_per_subframe[tx_frame_id][tx_current_subframe_id]++;

        // printf("data_ptr: %d, tx_frame_id: %d, tx_total_subframe_id: %d,
        // tx_current_subframe_id %d, tx_ant_id %d, max_subframe_id %d\n",
        // data_ptr,
        //   tx_frame_id, tx_total_subframe_id, tx_current_subframe_id,
        //   tx_ant_id, max_subframe_id);
        if (packet_count_per_subframe[tx_frame_id][tx_current_subframe_id]
            == cfg->BS_ANT_NUM) {
            packet_count_per_frame[tx_frame_id]++;
            // double cur_time = get_time();
            // printf("Finished transmit all antennas in frame: %d, subframe:
            // %d, at
            // %.5f in %.5f us\n", tx_frame_id, tx_current_subframe_id,
            // cur_time,cur_time-start_time);
            if (tx_frame_count <= 5) {
                while ((RDTSC() - tick_start) < ticks_5)
                    _mm_pause();
            } else if (tx_frame_count < 100) {
                while ((RDTSC() - tick_start) < ticks_100)
                    _mm_pause();
            } else if (tx_frame_count < 200) {
                while ((RDTSC() - tick_start) < ticks_200)
                    _mm_pause();
            } else if (tx_frame_count < 500) {
                while ((RDTSC() - tick_start) < ticks_500)
                    _mm_pause();
            } else {
                while ((RDTSC() - tick_start) < ticks_all)
                    _mm_pause();
            }

            tick_start = RDTSC();

            if (packet_count_per_frame[tx_frame_id] == max_subframe_id) {
                frame_end[tx_frame_count] = get_time();
                packet_count_per_frame[tx_frame_id] = 0;
                if (cfg->downlink_mode) {
                    if (frame_id < 500) {
                        while ((RDTSC() - tick_start)
                            < 2 * cfg->data_symbol_num_perframe * ticks_all)
                            _mm_pause();
                    } else {
                        while ((RDTSC() - tick_start)
                            < cfg->data_symbol_num_perframe * ticks_all)
                            _mm_pause();
                    }
                }
                // printf("Finished transmit all antennas in frame: %d, next
                // scheduled: %d, in %.5f us\n", tx_frame_id, frame_id,
                // get_time()-start_time); start_time = get_time();
                tick_start = RDTSC();

                tx_frame_count++;
                if (tx_frame_count == (size_t)cfg->tx_frame_num)
                    break;
                frame_start[tx_frame_count] = get_time();
                // printf("Finished transmit all antennas in frame: %d, in %.5f
                // us\n", tx_frame_count -1, frame_end[tx_frame_count - 1] -
                // frame_start[tx_frame_count-1]);
            }
            packet_count_per_subframe[tx_frame_id][tx_current_subframe_id] = 0;
            size_t next_subframe_ptr
                = ((tx_total_subframe_id + 1) * cfg->BS_ANT_NUM) % max_length_;
            for (size_t i = 0; i < cfg->BS_ANT_NUM; i++) {
                size_t ptok_id = i % thread_num;
                if (!task_queue_.enqueue(
                        *task_ptok[ptok_id], i + next_subframe_ptr)) {
                    printf("send task enqueue failed\n");
                    exit(0);
                }
            }
        }

        ant_id++;
        if (ant_id == cfg->BS_ANT_NUM) {
            ant_id = 0;
            subframe_id++;
            if (subframe_id == max_subframe_id) {
                subframe_id = 0;
                frame_id++;
                if (frame_id == MAX_FRAME_ID)
                    frame_id = 0;
            }
        }
    }

    printf("printing sender results to file...\n");

    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string filename = cur_directory + "/matlab/tx_result.txt";
    FILE* fp_debug = fopen(filename.c_str(), "w");
    if (fp_debug == NULL) {
        printf("open file faild");
        std::cerr << "Error: " << strerror(errno) << std::endl;
        exit(0);
    }

    for (size_t ii = 0; ii < tx_frame_count; ii++) {
        fprintf(fp_debug, "%.5f\n", frame_end[ii]);
    }
    exit(0);
}

void* Sender::loopSend(int tid)
{
    pin_to_core_with_offset(ThreadType::kWorkerTX, core_offset + 1, tid);

    size_t buffer_length = tx_buf_offset + cfg->packet_length;
    size_t max_subframe_id = cfg->downlink_mode
        ? cfg->pilot_symbol_num_perframe
        : cfg->pilot_symbol_num_perframe + cfg->data_symbol_num_perframe;

    // Use mutex to sychronize data receiving across threads
    pthread_mutex_lock(&mutex);
    printf("Thread %zu: waiting for release\n", (size_t)tid);

    pthread_cond_wait(&cond, &mutex);
    pthread_mutex_unlock(&mutex); // unlocking for all other threads

    // auto begin = std::chrono::system_clock::now();
    double begin = get_time();
    size_t packet_count = 0;
    // std::iota(ant_seq.begin(), ant_seq.end(), 0);

    size_t used_socker_id = 0;
    size_t total_tx_packets = 0;
    int ret;

    printf("Max_subframe_id: %zu\n", max_subframe_id);
    size_t ant_num_this_thread = cfg->BS_ANT_NUM / thread_num
        + ((size_t)tid < cfg->BS_ANT_NUM % thread_num ? 1 : 0);
#if DEBUG_SENDER
    double start_time_send = get_time();
    double end_time_send = get_time();
    double end_time_prev = get_time();
#endif

    printf("In thread %zu, %zu antennas, BS_ANT_NUM: %zu, thread number: %zu\n",
        (size_t)tid, ant_num_this_thread, cfg->BS_ANT_NUM, thread_num);

    while (true) {
        size_t data_ptr;
        ret = task_queue_.try_dequeue_from_producer(
            *(task_ptok[tid]), data_ptr);
        if (!ret)
            continue;

        used_socker_id = data_ptr % socket_num;

        auto* pkt = (struct Packet*)trans_buffer_[data_ptr];
        size_t subframe_id = pkt->symbol_id;
#if DEBUG_SENDER
        start_time_send = get_time();
#endif
        if (!cfg->downlink_mode
            || subframe_id < cfg->pilot_symbol_num_perframe) {
            /* Send a message to the server */
#if defined(USE_DPDK) || !CONNECT_UDP
            if (sendto(socket_[used_socker_id], trans_buffer_[data_ptr],
                    buffer_length, 0,
                    (struct sockaddr*)&servaddr_[used_socker_id],
                    sizeof(servaddr_[used_socker_id]))
                < 0) {
                perror("socket sendto failed");
                exit(0);
            }
#else
            if (send(socket_[used_socker_id], trans_buffer_[data_ptr],
                    buffer_length, 0)
                < 0) {
                perror("Socket sendto failed");
                exit(0);
            }
#endif
        }

        if (!message_queue_.enqueue(data_ptr)) {
            printf("Send message enqueue failed\n");
            exit(0);
        }
        packet_count++;
        total_tx_packets++;

#if DEBUG_SENDER
        end_time_send = get_time();
        printf("Thread %d transmit frame %d, subframe %d, ant %d, total: %d, "
               "tx time: %.3f, since last iteration: %.3f\n",
            tid, pkt->frame_id, pkt->symbol_id, pkt->ant_id, total_tx_packets,
            end_time_send - start_time_send, end_time_send - end_time_prev);
        end_time_prev = get_time();
#endif

        if (total_tx_packets > 1e9)
            total_tx_packets = 0;
        if (packet_count == ant_num_this_thread * max_subframe_id * 1000) {
            double end = get_time();
            // double byte_len = sizeof(ushort) * OFDM_FRAME_LEN * 2 *
            // ant_num_this_thread * max_subframe_id * 1000;
            double byte_len = buffer_length * ant_num_this_thread
                * max_subframe_id * 1000.f;
            double diff = end - begin;
            printf(
                "Thread %zu send %zu frames in %f secs, throughput %f Mbps\n",
                (size_t)tid,
                total_tx_packets / (ant_num_this_thread * max_subframe_id),
                diff / 1e6, byte_len * 8 * 1e6 / diff / 1024 / 1024);
            begin = get_time();
            packet_count = 0;
        }
    }
}
