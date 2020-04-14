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

void delay_ticks(uint64_t start, uint64_t ticks)
{
    while ((rdtsc() - start) < ticks)
        _mm_pause();
    // printf("duration: %.5f\n", get_time() - start_time);
}

Sender::Sender(
    Config* cfg, size_t in_thread_num, size_t in_core_offset, size_t in_delay)
    : cfg(cfg)
    , ant_id(0)
    , frame_id(0)
    , symbol_id(0)
    , thread_num(in_thread_num)
    , socket_num(cfg->nRadios)
    , core_offset(in_core_offset)
    , delay(in_delay)
{
    printf("TX constructer: on core %d\n", sched_getcpu());

    ticks_all = (uint64_t)delay * CPU_FREQ / 1e6 / 70;
    size_t buffer_length = kTXBufOffset + cfg->packet_length;
    size_t max_symbol_id = get_max_symbol_id();
    printf("max_symbol_id: %zu\n", max_symbol_id);
    size_t max_length_
        = SOCKET_BUFFER_FRAME_NUM * max_symbol_id * cfg->BS_ANT_NUM;

    packet_count_per_symbol.calloc(SOCKET_BUFFER_FRAME_NUM, max_symbol_id, 64);
    alloc_buffer_1d(&packet_count_per_frame, SOCKET_BUFFER_FRAME_NUM, 64, 1);

    tx_buffer_.calloc(max_length_, buffer_length, 64);
    init_IQ_from_file();
    /* preload data to tx buffer */
    preload_tx_buffer();

    task_ptok = (moodycamel::ProducerToken**)aligned_alloc(
        64, thread_num * sizeof(moodycamel::ProducerToken*));
    for (size_t i = 0; i < thread_num; i++)
        task_ptok[i] = new moodycamel::ProducerToken(task_queue_);

    socket_ = new int[socket_num];
#if USE_IPV4
    servaddr_ = new struct sockaddr_in[socket_num];
#else
    servaddr_ = new struct sockaddr_in6[socket_num];
#endif

    for (size_t i = 0; i < socket_num; i++) {
#if USE_IPV4
        socket_[i] = setup_socket_ipv4(cfg->ue_tx_port + i, false, 0);
        setup_sockaddr_remote_ipv4(
            &servaddr_[i], cfg->bs_port + i, cfg->rx_addr.c_str());
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
}

Sender::~Sender()
{
    IQ_data_coded.free();
    IQ_data.free();
    tx_buffer_.free();
    packet_count_per_symbol.free();
    free_buffer_1d(&packet_count_per_frame);

    delete[] servaddr_;
    delete[] socket_;
    delete cfg;
    pthread_mutex_destroy(&lock_);
}

void Sender::startTX()
{
    printf("start sender\n");
    alloc_buffer_1d(&frame_start, kNumStatsFrames, 4096, 1);
    alloc_buffer_1d(&frame_end, kNumStatsFrames, 4096, 1);
    /* create tx threads */
    create_threads(
        pthread_fun_wrapper<Sender, &Sender::loopSend>, 0, thread_num);

    /* give some time for all threads to lock */
    sleep(1);
    printf("Master: Now releasing the condition\n");
    pthread_cond_broadcast(&cond);

    /* run while loop */
    loopMain(0);
}

void Sender::startTXfromMain(double* in_frame_start, double* in_frame_end)
{
    printf("start sender from simulator\n");
    frame_start = in_frame_start;
    frame_end = in_frame_end;

    /* create tx threads */
    create_threads(
        pthread_fun_wrapper<Sender, &Sender::loopSend>, 0, thread_num);

    /* give some time for all threads to lock */
    sleep(1);
    printf("Master: Now releasing the condition\n");
    pthread_cond_broadcast(&cond);

    /* create main thread */
    create_threads(pthread_fun_wrapper<Sender, &Sender::loopMain>, thread_num,
        thread_num + 1);
}

void* Sender::loopMain(int tid)
{
    pin_to_core_with_offset(ThreadType::kMasterTX, core_offset, 0);

    size_t max_symbol_id = get_max_symbol_id();
    size_t max_length_
        = SOCKET_BUFFER_FRAME_NUM * max_symbol_id * cfg->BS_ANT_NUM;

    /* push tasks of the first symbol into task queue */
    for (size_t i = 0; i < cfg->BS_ANT_NUM; i++) {
        int ptok_id = i % thread_num;
        if (!task_queue_.enqueue(*task_ptok[ptok_id], i)) {
            printf("send task enqueue failed\n");
            exit(0);
        }
    }

    signal(SIGINT, intHandler);
    size_t tx_frame_count = 0;
    int ret;
    frame_start[0] = get_time();
    uint64_t tick_start = rdtsc();
#if DEBUG_SENDER
    double start_time = get_time();
#endif
    while (keep_running) {
        size_t data_ptr;
        ret = message_queue_.try_dequeue(data_ptr);
        if (!ret)
            continue;
        update_tx_buffer(data_ptr);
        size_t tx_total_symbol_id = data_ptr / cfg->BS_ANT_NUM;
        size_t tx_current_symbol_id = tx_total_symbol_id % max_symbol_id;
        size_t tx_frame_id = tx_total_symbol_id / max_symbol_id;
        packet_count_per_symbol[tx_frame_id][tx_current_symbol_id]++;
        if (packet_count_per_symbol[tx_frame_id][tx_current_symbol_id]
            == cfg->BS_ANT_NUM) {
#if DEBUG_SENDER
            double cur_time = get_time();
            printf("Finished transmit all antennas in frame: %zu, symbol: %zu,"
                   " in %.5f us\n ",
                tx_frame_id, tx_current_symbol_id, cur_time - start_time);
#endif
            packet_count_per_frame[tx_frame_id]++;
            delay_for_symbol(tx_frame_count, tick_start);
            tick_start = rdtsc();

            if (packet_count_per_frame[tx_frame_id] == max_symbol_id) {
                frame_end[tx_frame_count] = get_time();
                packet_count_per_frame[tx_frame_id] = 0;

                delay_for_frame(tx_frame_count, tick_start);
#if DEBUG_SENDER
                printf("Finished transmit all antennas in frame: %zu, "
                       "next scheduled: %zu, in %.5f us\n",
                    tx_frame_count, frame_id, get_time() - start_time);
                start_time = get_time();
#endif
                tx_frame_count++;
                if (tx_frame_count == (size_t)cfg->tx_frame_num)
                    break;
                frame_start[tx_frame_count] = get_time();
                tick_start = rdtsc();
                // printf("Finished transmit all antennas in frame: %d, in %.5f
                // us\n", tx_frame_count -1, frame_end[tx_frame_count - 1] -
                // frame_start[tx_frame_count-1]);
            }
            packet_count_per_symbol[tx_frame_id][tx_current_symbol_id] = 0;
            size_t next_symbol_ptr
                = ((tx_total_symbol_id + 1) * cfg->BS_ANT_NUM) % max_length_;
            for (size_t i = 0; i < cfg->BS_ANT_NUM; i++) {
                size_t ptok_id = i % thread_num;
                if (!task_queue_.enqueue(
                        *task_ptok[ptok_id], i + next_symbol_ptr)) {
                    printf("send task enqueue failed\n");
                    exit(0);
                }
            }
        }
    }
    write_stats_to_file(tx_frame_count);
    exit(0);
}

void* Sender::loopSend(int tid)
{
    pin_to_core_with_offset(ThreadType::kWorkerTX, core_offset + 1, tid);

    size_t buffer_length = kTXBufOffset + cfg->packet_length;
    /* Use mutex to sychronize data receiving across threads */
    pthread_mutex_lock(&mutex);
    printf("Thread %zu: waiting for release\n", (size_t)tid);

    pthread_cond_wait(&cond, &mutex);
    /* unlock all other threads */
    pthread_mutex_unlock(&mutex);

    double begin = get_time();
    size_t packet_count = 0;
    size_t total_tx_packets = 0;
    size_t max_symbol_id = get_max_symbol_id();
    int radio_lo = tid * cfg->nRadios / thread_num;
    int radio_hi = (tid + 1) * cfg->nRadios / thread_num;
    size_t ant_num_this_thread = cfg->BS_ANT_NUM / thread_num
        + ((size_t)tid < cfg->BS_ANT_NUM % thread_num ? 1 : 0);
    printf("In thread %zu, %zu antennas, BS_ANT_NUM: %zu, thread number: %zu\n",
        (size_t)tid, ant_num_this_thread, cfg->BS_ANT_NUM, thread_num);
    int radio_id = radio_lo;
    while (true) {
        int data_ptr = dequeue_send(tid, radio_id);
        if (data_ptr == -1)
            continue;
        packet_count++;
        total_tx_packets++;
        if (total_tx_packets > 1e9)
            total_tx_packets = 0;
        if (packet_count == ant_num_this_thread * max_symbol_id * 1000) {
            double end = get_time();
            double byte_len
                = buffer_length * ant_num_this_thread * max_symbol_id * 1000.f;
            double diff = end - begin;
            printf(
                "Thread %zu send %zu frames in %f secs, throughput %f Mbps\n",
                (size_t)tid,
                total_tx_packets / (ant_num_this_thread * max_symbol_id),
                diff / 1e6, byte_len * 8 * 1e6 / diff / 1024 / 1024);
            begin = get_time();
            packet_count = 0;
        }

        if (++radio_id == radio_hi)
            radio_id = radio_lo;
    }
}

int Sender::dequeue_send(int tid, int radio_id)
{
    size_t data_ptr;
    if (!task_queue_.try_dequeue_from_producer(*(task_ptok[tid]), data_ptr))
        return -1;

    size_t buffer_length = kTXBufOffset + cfg->packet_length;
#if DEBUG_SENDER
    double start_time_send = get_time();
#endif
    /* send a message to the server */
    int ret = 0;
#if defined(USE_DPDK) || !CONNECT_UDP
    ret = sendto(socket_[radio_id], tx_buffer_[data_ptr],
            buffer_length, 0, (struct sockaddr*)&servaddr_[tid],
            sizeof(servaddr_[tid]);
#else
    ret = send(socket_[radio_id], tx_buffer_[data_ptr], buffer_length, 0);
#endif
    if (ret < 0) {
        perror("Socket sendto failed");
        exit(0);
    }

#if DEBUG_SENDER
    double tx_duration = get_time() - start_time_send;
    struct Packet* pkt = (struct Packet*)tx_buffer_[data_ptr];
    printf("Thread %d transmit frame %d, symbol %d, ant %d, data_ptr: %zu,"
        " tx time: %.3f\n", tid, pkt->frame_id, pkt->symbol_id, pkt->ant_id, 
        data_ptr, tx_duration);
#endif

    if (!message_queue_.enqueue(data_ptr)) {
        printf("Send message enqueue failed\n");
        exit(0);
    }
    return data_ptr;
}

size_t Sender::get_max_symbol_id()
{
    size_t max_symbol_id = cfg->downlink_mode
        ? cfg->pilot_symbol_num_perframe
        : cfg->pilot_symbol_num_perframe + cfg->data_symbol_num_perframe;
    return max_symbol_id;
}

void Sender::init_IQ_from_file()
{
    size_t IQ_data_size = cfg->symbol_num_perframe * cfg->BS_ANT_NUM;

    IQ_data.calloc(IQ_data_size, cfg->OFDM_FRAME_LEN * 2, 64);
    IQ_data_coded.calloc(IQ_data_size, cfg->OFDM_FRAME_LEN * 2, 64);
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
    for (size_t i = 0; i < IQ_data_size; i++) {
        size_t expect_bytes = cfg->OFDM_FRAME_LEN * 2;
        size_t actual_bytes
            = fread(IQ_data[i], sizeof(float), expect_bytes, fp);
        if (expect_bytes != actual_bytes) {
            printf("read file failed: %s\n", filename.c_str());
            printf("i: %zu, expected: %zu, actual: %zu\n", i, expect_bytes,
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
}

void Sender::update_ids(size_t max_ant_id, size_t max_symbol_id)
{
    ant_id++;
    if (ant_id == max_ant_id) {
        ant_id = 0;
        symbol_id++;
        if (symbol_id == max_symbol_id) {
            symbol_id = 0;
            frame_id++;
            if (frame_id == kNumStatsFrames)
                frame_id = 0;
        }
    }
}

void Sender::delay_for_symbol(size_t tx_frame_count, uint64_t tick_start)
{
    if (tx_frame_count <= 5) {
        delay_ticks(tick_start, ticks_5);
    } else if (tx_frame_count < 100) {
        delay_ticks(tick_start, ticks_100);
    } else if (tx_frame_count < 200) {
        delay_ticks(tick_start, ticks_200);
    } else if (tx_frame_count < 500) {
        delay_ticks(tick_start, ticks_500);
    } else {
        delay_ticks(tick_start, ticks_all);
    }
}

void Sender::delay_for_frame(size_t tx_frame_count, uint64_t tick_start)
{
    if (cfg->downlink_mode) {
        if (tx_frame_count < 500) {
            delay_ticks(
                tick_start, 2 * cfg->data_symbol_num_perframe * ticks_all);
        } else {
            delay_ticks(tick_start, cfg->data_symbol_num_perframe * ticks_all);
        }
    }
}

void Sender::preload_tx_buffer()
{
    size_t max_symbol_id = get_max_symbol_id();
    size_t max_length_
        = SOCKET_BUFFER_FRAME_NUM * max_symbol_id * cfg->BS_ANT_NUM;
    for (size_t i = 0; i < max_length_; i++) {
        update_tx_buffer(i);
    }
}

void Sender::update_tx_buffer(size_t data_ptr)
{
    size_t tx_ant_id = data_ptr % cfg->BS_ANT_NUM;
    size_t data_index = symbol_id * cfg->BS_ANT_NUM + tx_ant_id;
    struct Packet* pkt = (struct Packet*)(tx_buffer_[data_ptr] + kTXBufOffset);
    pkt->frame_id = frame_id;
    pkt->symbol_id = cfg->getSymbolId(symbol_id);
    pkt->cell_id = 0;
    pkt->ant_id = tx_ant_id;
    memcpy(pkt->data, (char*)IQ_data_coded[data_index],
        sizeof(ushort) * cfg->OFDM_FRAME_LEN * 2);
    update_ids(cfg->BS_ANT_NUM, get_max_symbol_id());
}

void Sender::create_threads(void* (*worker)(void*), int tid_start, int tid_end)
{
    int ret;
    for (int i = tid_start; i < tid_end; i++) {
        pthread_t thread;
        auto context = new EventHandlerContext<Sender>;
        context->obj_ptr = this;
        context->id = i;
        ret = pthread_create(&thread, NULL, worker, context);
        if (ret != 0) {
            perror("task thread create failed");
            exit(0);
        }
    }
}

void Sender::write_stats_to_file(size_t tx_frame_count)
{
    printf("printing sender results to file...\n");
    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string filename = cur_directory + "/data/tx_result.txt";
    FILE* fp_debug = fopen(filename.c_str(), "w");
    if (fp_debug == NULL) {
        printf("open file faild");
        std::cerr << "Error: " << strerror(errno) << std::endl;
        exit(0);
    }
    for (size_t ii = 0; ii < tx_frame_count; ii++) {
        fprintf(fp_debug, "%.5f\n", frame_end[ii]);
    }
}
