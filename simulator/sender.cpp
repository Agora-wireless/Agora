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
    // double start_time = get_time();
    while ((RDTSC() - start) < ticks)
        _mm_pause();
    // printf("duration: %.5f\n", get_time() - start_time);
}

Sender::Sender(Config* cfg, int in_thread_num, int in_core_offset, int in_delay)
    : ant_id(0)
    , frame_id(0)
    , subframe_id(0)
    , thread_num(in_thread_num)
    , socket_num(in_thread_num)
    , core_offset(in_core_offset)
    , delay(in_delay)
{
    printf("TX constructer: on core %d\n", sched_getcpu());

    ticks_all = (uint64_t)delay * CPU_FREQ / 1e6 / 70;
    config_ = cfg;
    int BS_ANT_NUM = config_->BS_ANT_NUM;
    int packet_length = config_->packet_length;
    int buffer_length = kTXBufOffset + packet_length;
    int max_subframe_id = get_max_subframe_id();
    printf("max_subframe_id: %d\n", max_subframe_id);
    int max_length_ = BUFFER_FRAME_NUM * max_subframe_id * BS_ANT_NUM;

    packet_count_per_subframe.calloc(BUFFER_FRAME_NUM, max_subframe_id, 64);
    alloc_buffer_1d(&packet_count_per_frame, BUFFER_FRAME_NUM, 64, 1);

    tx_buffer_.calloc(max_length_, buffer_length, 64);
    init_IQ_from_file();
    /* preload data to tx buffer */
    preload_tx_buffer();

    task_ptok = (moodycamel::ProducerToken**)aligned_alloc(
        64, thread_num * sizeof(moodycamel::ProducerToken*));
    for (int i = 0; i < thread_num; i++)
        task_ptok[i] = new moodycamel::ProducerToken(task_queue_);

    socket_ = new int[socket_num];
    for (int i = 0; i < socket_num; i++) {
#if USE_IPV4
        socket_[i] = setup_socket_ipv4(config_->ue_tx_port + i, false, 0);
        setup_sockaddr_remote_ipv4(
            &servaddr_[i], config_->bs_port + i, config_->rx_addr.c_str());
#else
        socket_[i] = setup_socket_ipv6(config_->ue_tx_port + i, false, 0);
        setup_sockaddr_remote_ipv6(
            &servaddr_[i], config_->bs_port + i, "fe80::f436:d735:b04a:864a");
#endif

#if !defined(USE_DPDK) && CONNECT_UDP
        if (connect(socket_[i], (struct sockaddr*)&servaddr_[i],
                sizeof(servaddr_[i]))
            != 0)
            perror("UDP socket connect failed");
        else
            printf("UDP socket %d connected\n", i);
#endif
    }
}

Sender::~Sender()
{
    IQ_data_coded.free();
    IQ_data.free();
    tx_buffer_.free();
    packet_count_per_subframe.free();
    free_buffer_1d(&packet_count_per_frame);

    delete[] socket_;
    delete config_;
    pthread_mutex_destroy(&lock_);
}

void Sender::startTX()
{
    printf("start sender\n");
    alloc_buffer_1d(&frame_start, 10240, 4096, 1);
    alloc_buffer_1d(&frame_end, 10240, 4096, 1);
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

    int BS_ANT_NUM = config_->BS_ANT_NUM;
    int max_subframe_id = get_max_subframe_id();
    int max_length_ = BUFFER_FRAME_NUM * max_subframe_id * BS_ANT_NUM;

    /* push tasks of the first subframe into task queue */
    for (int i = 0; i < BS_ANT_NUM; i++) {
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
    uint64_t tick_start = RDTSC();
#if DEBUG_SENDER
    double start_time = get_time();
#endif
    while (keep_running) {
        int data_ptr;
        ret = message_queue_.try_dequeue(data_ptr);
        if (!ret)
            continue;
        update_tx_buffer(data_ptr);
        int tx_total_subframe_id = data_ptr / BS_ANT_NUM;
        int tx_current_subframe_id = tx_total_subframe_id % max_subframe_id;
        int tx_frame_id = tx_total_subframe_id / max_subframe_id;
        packet_count_per_subframe[tx_frame_id][tx_current_subframe_id]++;
        if (packet_count_per_subframe[tx_frame_id][tx_current_subframe_id]
            == BS_ANT_NUM) {
            // double cur_time = get_time();
            // printf("Finished transmit all antennas in frame: %d, subframe:
            // %d, at
            // %.5f in %.5f us\n", tx_frame_id, tx_current_subframe_id,
            // cur_time,cur_time-start_time);
            packet_count_per_frame[tx_frame_id]++;
            delay_for_symbol(tx_frame_count, tick_start);
            tick_start = RDTSC();

            if (packet_count_per_frame[tx_frame_id] == max_subframe_id) {
                frame_end[tx_frame_count] = get_time();
                packet_count_per_frame[tx_frame_id] = 0;
                delay_for_frame(tx_frame_count, tick_start);
#if DEBUG_SENDER
                printf("Finished transmit all antennas in frame: %d, "
                       "next scheduled: %d, in %.5f us\n",
                    tx_frame_count, frame_id, get_time() - start_time);
                start_time = get_time();
#endif
                tx_frame_count++;
                if (tx_frame_count == config_->tx_frame_num)
                    break;
                frame_start[tx_frame_count] = get_time();
                tick_start = RDTSC();
                // printf("Finished transmit all antennas in frame: %d, in %.5f
                // us\n", tx_frame_count -1, frame_end[tx_frame_count - 1] -
                // frame_start[tx_frame_count-1]);
            }
            packet_count_per_subframe[tx_frame_id][tx_current_subframe_id] = 0;
            int next_subframe_ptr
                = ((tx_total_subframe_id + 1) * BS_ANT_NUM) % max_length_;
            for (int i = 0; i < BS_ANT_NUM; i++) {
                int ptok_id = i % thread_num;
                if (!task_queue_.enqueue(
                        *task_ptok[ptok_id], i + next_subframe_ptr)) {
                    printf("send task enqueue failed\n");
                    exit(0);
                }
            }
        }
        update_ids(BS_ANT_NUM, max_subframe_id);
    }

    write_stats_to_file(tx_frame_count);
    exit(0);
}

void* Sender::loopSend(int tid)
{
    pin_to_core_with_offset(ThreadType::kWorkerTX, core_offset + 1, tid);
    int buffer_length = kTXBufOffset + config_->packet_length;
    /* Use mutex to sychronize data receiving across threads */
    pthread_mutex_lock(&mutex);
    printf("Thread %d: waiting for release\n", tid);

    pthread_cond_wait(&cond, &mutex);
    /* unlock all other threads */
    pthread_mutex_unlock(&mutex);

    double begin = get_time();
    int packet_count = 0;
    int total_tx_packets = 0;
    int max_subframe_id = get_max_subframe_id();
    int BS_ANT_NUM = config_->BS_ANT_NUM;
    int ant_num_this_thread
        = BS_ANT_NUM / thread_num + (tid < BS_ANT_NUM % thread_num ? 1 : 0);
    printf("In thread %d, %d antennas, BS_ANT_NUM: %d, thread number: %d\n",
        tid, ant_num_this_thread, BS_ANT_NUM, thread_num);
    while (true) {
        int data_ptr = dequeue_send(tid);
        if (data_ptr == -1)
            continue;
        packet_count++;
        total_tx_packets++;
        if (total_tx_packets > 1e9)
            total_tx_packets = 0;
        if (packet_count == ant_num_this_thread * max_subframe_id * 1000) {
            double end = get_time();
            double byte_len = buffer_length * ant_num_this_thread
                * max_subframe_id * 1000.f;
            double diff = end - begin;
            printf("thread %d send %d frames in %f secs, throughput %f Mbps\n",
                tid, total_tx_packets / (ant_num_this_thread * max_subframe_id),
                diff / 1e6, byte_len * 8 * 1e6 / diff / 1024 / 1024);
            begin = get_time();
            packet_count = 0;
        }
    }
}

int Sender::dequeue_send(int tid)
{
    int data_ptr;
    if (!task_queue_.try_dequeue_from_producer(*(task_ptok[tid]), data_ptr))
        return -1;

    int buffer_length = kTXBufOffset + config_->packet_length;
#if DEBUG_SENDER
    double start_time_send = get_time();
#endif
    /* send a message to the server */
    int ret = 0;
#if defined(USE_DPDK) || !CONNECT_UDP
    ret = sendto(socket_[tid], tx_buffer_[data_ptr],
            buffer_length, 0, (struct sockaddr*)&servaddr_[tid],
            sizeof(servaddr_[tid]);
#else
    ret = send(socket_[tid], tx_buffer_[data_ptr], buffer_length, 0);
#endif
    if (ret < 0) {
        perror("socket sendto failed");
        exit(0);
    }

#if DEBUG_SENDER
    double end_time_send = get_time();
    struct Packet* pkt = (struct Packet*)tx_buffer_[data_ptr];
    printf("Thread %d transmit frame %d, subframe %d, ant %d, total: %d, "
           "tx time: %.3f\n",
        tid, pkt->frame_id, pkt->symbol_id, pkt->ant_id, total_tx_packets,
        end_time_send - start_time_send);
#endif

    if (!message_queue_.enqueue(data_ptr)) {
        printf("send message enqueue failed\n");
        exit(0);
    }
    return data_ptr;
}

int Sender::get_max_subframe_id()
{
    int pilot_subframe_num_perframe = config_->pilot_symbol_num_perframe;
    int data_subframe_num_perframe = config_->data_symbol_num_perframe;
    int downlink_mode = config_->downlink_mode;
    int max_subframe_id = downlink_mode
        ? pilot_subframe_num_perframe
        : pilot_subframe_num_perframe + data_subframe_num_perframe;
    return max_subframe_id;
}

void Sender::init_IQ_from_file()
{
    size_t BS_ANT_NUM = config_->BS_ANT_NUM;
    size_t subframe_num_perframe = config_->symbol_num_perframe;
    size_t OFDM_FRAME_LEN = config_->OFDM_FRAME_LEN;
    size_t IQ_data_size = subframe_num_perframe * BS_ANT_NUM;

    IQ_data.calloc(IQ_data_size, OFDM_FRAME_LEN * 2, 64);
    IQ_data_coded.calloc(IQ_data_size, OFDM_FRAME_LEN * 2, 64);
    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
#ifdef USE_LDPC
    std::string filename = cur_directory + "/data/LDPC_rx_data_2048_ant"
        + std::to_string(BS_ANT_NUM) + ".bin";
#else
    std::string filename = cur_directory + "/data/rx_data_2048_ant"
        + std::to_string(BS_ANT_NUM) + ".bin";
#endif
    FILE* fp = fopen(filename.c_str(), "rb");
    if (fp == NULL) {
        printf("open file failed: %s\n", filename.c_str());
        std::cerr << "Error: " << strerror(errno) << std::endl;
    }
    for (size_t i = 0; i < IQ_data_size; i++) {
        size_t expect_bytes = OFDM_FRAME_LEN * 2;
        size_t actual_bytes
            = fread(IQ_data[i], sizeof(float), expect_bytes, fp);
        if (expect_bytes != actual_bytes) {
            printf("read file failed: %s\n", filename.c_str());
            printf("i: %zu, expected: %zu, actual: %zu\n", i, expect_bytes,
                actual_bytes);
            std::cerr << "Error: " << strerror(errno) << std::endl;
        }
        for (size_t j = 0; j < OFDM_FRAME_LEN * 2; j++) {
            IQ_data_coded[i][j] = (ushort)(IQ_data[i][j] * 32768);
            // printf("i:%d, j:%d, Coded: %d, orignal:
            // %.4f\n",i,j/2,IQ_data_coded[i][j],IQ_data[i][j]);
        }
    }
    fclose(fp);
}

void Sender::update_ids(int max_ant_id, int max_subframe_id)
{
    ant_id++;
    if (ant_id == max_ant_id) {
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
    if (config_->downlink_mode) {
        if (tx_frame_count < 500) {
            delay_ticks(
                tick_start, 2 * config_->data_symbol_num_perframe * ticks_all);
        } else {
            delay_ticks(
                tick_start, config_->data_symbol_num_perframe * ticks_all);
        }
    }
}

void Sender::preload_tx_buffer()
{
    int max_subframe_id = get_max_subframe_id();
    int max_length_ = BUFFER_FRAME_NUM * max_subframe_id * config_->BS_ANT_NUM;
    for (int i = 0; i < max_length_; i++) {
        update_tx_buffer(i);
        update_ids(config_->BS_ANT_NUM, max_subframe_id);
    }
}

void Sender::update_tx_buffer(int data_ptr)
{
    int tx_ant_id = data_ptr % config_->BS_ANT_NUM;
    int data_index = subframe_id * config_->BS_ANT_NUM + tx_ant_id;
    struct Packet* pkt = (struct Packet*)(tx_buffer_[data_ptr] + kTXBufOffset);
    pkt->frame_id = frame_id;
    pkt->symbol_id = config_->getSymbolId(subframe_id);
    pkt->cell_id = 0;
    pkt->ant_id = ant_id;
    memcpy(pkt->data, (char*)IQ_data_coded[data_index],
        sizeof(ushort) * config_->OFDM_FRAME_LEN * 2);
    // fastMemcpy(tx_buffer_[data_ptr] + kTXBufOffset + data_offset,
    // (char
    // *)IQ_data_coded[data_index], sizeof(ushort) * OFDM_FRAME_LEN * 2);
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
