/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#include "sender.hpp"
#include <thread>

bool keep_running = true;

void interrupt_handler(int)
{
    std::cout << "Will exit..." << std::endl;
    keep_running = false;
}

void delay_ticks(uint64_t start, uint64_t ticks)
{
    while ((rdtsc() - start) < ticks)
        _mm_pause();
}

Sender::Sender(Config* cfg, size_t thread_num, size_t core_offset, size_t delay)
    : cfg(cfg)
    , ant_id(0)
    , frame_id(0)
    , symbol_id(0)
    , thread_num(thread_num)
    , socket_num(cfg->nRadios)
    , core_offset(core_offset)
    , delay(delay)
{
    rt_assert(socket_num <= kMaxNumSockets, "Too many network sockets");
    const double ticks_per_sec = measure_rdtsc_freq() * 1e9;

    ticks_all = delay * ticks_per_sec / 1e6 / 70;
    ticks_5 = 500000 * ticks_per_sec / 1e6 / 70;
    ticks_100 = 150000 * ticks_per_sec / 1e6 / 70;
    ticks_200 = 20000 * ticks_per_sec / 1e6 / 70;
    ticks_500 = 10000 * ticks_per_sec / 1e6 / 70;

    for (size_t i = 0; i < SOCKET_BUFFER_FRAME_NUM; i++) {
        packet_count_per_symbol[i] = new size_t[get_max_symbol_id()]();
    }
    memset(packet_count_per_frame, 0, SOCKET_BUFFER_FRAME_NUM * sizeof(size_t));

    tx_buffers_.calloc(
        SOCKET_BUFFER_FRAME_NUM * get_max_symbol_id() * cfg->BS_ANT_NUM,
        kTXBufOffset + cfg->packet_length, 64);
    init_IQ_from_file();

    // Preload packets to TX buffers
    const size_t num_packets
        = SOCKET_BUFFER_FRAME_NUM * get_max_symbol_id() * cfg->BS_ANT_NUM;
    for (size_t i = 0; i < num_packets; i++) {
        update_tx_buffer(i);
    }

    task_ptok = (moodycamel::ProducerToken**)aligned_alloc(
        64, thread_num * sizeof(moodycamel::ProducerToken*));
    for (size_t i = 0; i < thread_num; i++)
        task_ptok[i] = new moodycamel::ProducerToken(send_queue_);

    for (size_t i = 0; i < socket_num; i++) {
        if (kUseIPv4) {
            socket_[i] = setup_socket_ipv4(cfg->ue_tx_port + i, false, 0);
            setup_sockaddr_remote_ipv4(
                &servaddr_ipv4[i], cfg->bs_port + i, cfg->rx_addr.c_str());
        } else {
            socket_[i] = setup_socket_ipv6(cfg->ue_tx_port + i, false, 0);
            setup_sockaddr_remote_ipv6(&servaddr_ipv6[i], cfg->bs_port + i,
                "fe80::f436:d735:b04a:864a");
        }

        if (!kUseDPDK && kConnectUDP) {
            int ret = connect(socket_[i], (struct sockaddr*)&servaddr_ipv4[i],
                sizeof(servaddr_ipv4[i]));
            rt_assert(ret == 0, "UDP socket connect failed");
        } else {
            printf("UDP socket %zu connected\n", i);
        }
    }
}

Sender::~Sender()
{
    IQ_data_coded.free();
    IQ_data.free();
    tx_buffers_.free();
    for (size_t i = 0; i < SOCKET_BUFFER_FRAME_NUM; i++) {
        free(packet_count_per_symbol[i]);
    }
    delete cfg;
    pthread_mutex_destroy(&lock_);
}

void Sender::startTX()
{
    printf("Starting sender\n");
    frame_start = new double[kNumStatsFrames]();
    frame_end = new double[kNumStatsFrames]();
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
    signal(SIGINT, interrupt_handler);
    pin_to_core_with_offset(ThreadType::kMasterTX, core_offset, 0);

    const size_t max_symbol_id = get_max_symbol_id();
    const size_t max_num_packets
        = SOCKET_BUFFER_FRAME_NUM * max_symbol_id * cfg->BS_ANT_NUM;

    // Push tasks of the first symbol into task queue
    for (size_t i = 0; i < cfg->BS_ANT_NUM; i++) {
        rt_assert(send_queue_.enqueue(*task_ptok[i % thread_num], i),
            "Send task enqueue failed");
    }

    size_t tx_frame_count = 0;
    frame_start[0] = get_time();
    uint64_t tick_start = rdtsc();
    double start_time = kDebugSenderReceiver ? get_time() : -1.0;
    while (keep_running) {
        size_t data_ptr;
        int ret = completion_queue_.try_dequeue(data_ptr);
        if (!ret)
            continue;
        update_tx_buffer(data_ptr);
        size_t tx_total_symbol_id = data_ptr / cfg->BS_ANT_NUM;
        size_t tx_current_symbol_id = tx_total_symbol_id % max_symbol_id;
        size_t tx_frame_id = tx_total_symbol_id / max_symbol_id;
        packet_count_per_symbol[tx_frame_id][tx_current_symbol_id]++;
        if (packet_count_per_symbol[tx_frame_id][tx_current_symbol_id]
            == cfg->BS_ANT_NUM) {
            if (kDebugSenderReceiver) {
                printf("Finished transmit all antennas in frame: %zu, "
                       "symbol: %zu, in %.5f us\n ",
                    tx_frame_id, tx_current_symbol_id, get_time() - start_time);
            }
            packet_count_per_frame[tx_frame_id]++;
            delay_for_symbol(tx_frame_count, tick_start);
            tick_start = rdtsc();

            if (packet_count_per_frame[tx_frame_id] == max_symbol_id) {
                frame_end[tx_frame_count] = get_time();
                packet_count_per_frame[tx_frame_id] = 0;

                delay_for_frame(tx_frame_count, tick_start);
                if (kDebugSenderReceiver) {
                    printf("Finished transmit all antennas in frame: %zu, "
                           "next scheduled: %zu, in %.5f us\n",
                        tx_frame_count, frame_id, get_time() - start_time);
                    start_time = get_time();
                }
                tx_frame_count++;
                if (tx_frame_count == cfg->frames_to_test)
                    break;
                frame_start[tx_frame_count] = get_time();
                tick_start = rdtsc();
            }
            packet_count_per_symbol[tx_frame_id][tx_current_symbol_id] = 0;
            size_t next_symbol_ptr
                = ((tx_total_symbol_id + 1) * cfg->BS_ANT_NUM)
                % max_num_packets;
            for (size_t i = 0; i < cfg->BS_ANT_NUM; i++) {
                rt_assert(send_queue_.enqueue(
                              *task_ptok[i % thread_num], i + next_symbol_ptr),
                    "Send task enqueue failed");
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
    printf("In thread %zu, %zu antennas, BS_ANT_NUM: %zu, num threads %zu:\n",
        (size_t)tid, ant_num_this_thread, cfg->BS_ANT_NUM, thread_num);
    int radio_id = radio_lo;
    while (true) {
        int data_ptr = dequeue_send(tid, radio_id);
        if (data_ptr == -1)
            continue;
        packet_count++;
        total_tx_packets++;
        if (packet_count == ant_num_this_thread * max_symbol_id * 1000) {
            double end = get_time();
            double byte_len
                = buffer_length * ant_num_this_thread * max_symbol_id * 1000.f;
            double diff = end - begin;
            printf("Thread %zu send %zu frames in %f secs, tput %f Mbps\n",
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
    if (!send_queue_.try_dequeue_from_producer(*(task_ptok[tid]), data_ptr))
        return -1;

    size_t buffer_length = kTXBufOffset + cfg->packet_length;
    double start_time_send = kDebugSenderReceiver ? get_time() : -1.0;
    /* send a message to the server */
    int ret = 0;
    if (kUseDPDK or !kConnectUDP) {
        ret = sendto(socket_[radio_id], tx_buffers_[data_ptr], buffer_length, 0,
            (struct sockaddr*)&servaddr_ipv4[tid], sizeof(servaddr_ipv4[tid]));
    } else {
        ret = send(socket_[radio_id], tx_buffers_[data_ptr], buffer_length, 0);
    }
    rt_assert(ret >= 0, "Socket sendto() failed");

    if (kDebugSenderReceiver) {
        auto* pkt = reinterpret_cast<Packet*>(tx_buffers_[data_ptr]);
        printf("Thread %d transmit frame %d, symbol %d, ant %d, data_ptr: "
               "%zu, tx time: %.3f\n",
            tid, pkt->frame_id, pkt->symbol_id, pkt->ant_id, data_ptr,
            get_time() - start_time_send);
    }

    rt_assert(completion_queue_.enqueue(data_ptr), "Completion enqueue failed");
    return data_ptr;
}

size_t Sender::get_max_symbol_id() const
{
    size_t max_symbol_id = cfg->downlink_mode
        ? cfg->pilot_symbol_num_perframe
        : cfg->pilot_symbol_num_perframe + cfg->data_symbol_num_perframe;
    return max_symbol_id;
}

void Sender::init_IQ_from_file()
{
    const size_t packets_per_frame = cfg->symbol_num_perframe * cfg->BS_ANT_NUM;
    IQ_data.calloc(packets_per_frame, cfg->OFDM_FRAME_LEN * 2, 64);
    IQ_data_coded.calloc(packets_per_frame, cfg->OFDM_FRAME_LEN * 2, 64);

    const std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);

    std::string filename;
    if (kUseLDPC) {
        filename = cur_directory + "/data/LDPC_rx_data_2048_ant"
            + std::to_string(cfg->BS_ANT_NUM) + ".bin";
    } else {
        filename = cur_directory + "/data/rx_data_2048_ant"
            + std::to_string(cfg->BS_ANT_NUM) + ".bin";
    }

    FILE* fp = fopen(filename.c_str(), "rb");
    rt_assert(fp != nullptr, "Failed to open IQ data file");

    for (size_t i = 0; i < packets_per_frame; i++) {
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

void Sender::update_tx_buffer(size_t data_ptr)
{
    size_t tx_ant_id = data_ptr % cfg->BS_ANT_NUM;
    size_t data_index = symbol_id * cfg->BS_ANT_NUM + tx_ant_id;
    auto* pkt = (Packet*)(tx_buffers_[data_ptr] + kTXBufOffset);
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
        rt_assert(ret == 0, "pthread_create() failed");
    }
}

void Sender::write_stats_to_file(size_t tx_frame_count) const
{
    printf("Printing sender results to file...\n");
    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string filename = cur_directory + "/data/tx_result.txt";
    FILE* fp_debug = fopen(filename.c_str(), "w");
    rt_assert(fp_debug != nullptr, "Failed to open stats file");
    for (size_t i = 0; i < tx_frame_count; i++) {
        fprintf(fp_debug, "%.5f\n", frame_end[i]);
    }
}
