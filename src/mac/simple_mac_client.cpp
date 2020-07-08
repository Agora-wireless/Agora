/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#include "simple_mac_client.hpp"
#include <thread>

bool keep_running = true;

// A spinning barrier to synchronize the start of SimpleClientMac threads
std::atomic<size_t> num_threads_ready_atomic;

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

inline size_t SimpleClientMac::tag_to_tx_buffers_index(gen_tag_t tag) const
{
    const size_t frame_slot = tag.frame_id % SOCKET_BUFFER_FRAME_NUM;
    return (frame_slot * cfg->UE_ANT_NUM) + tag.ue_id;
}

SimpleClientMac::SimpleClientMac(Config* cfg, size_t core_offset, size_t delay)
    : cfg(cfg)
    , freq_ghz(measure_rdtsc_freq())
    , ticks_per_usec(freq_ghz * 1e3)
    , thread_num(cfg->UE_ANT_NUM)
    , socket_num(cfg->UE_ANT_NUM)
    , core_offset(core_offset)
    , delay(delay)
    , ticks_all(delay * ticks_per_usec / cfg->symbol_num_perframe)
    , ticks_5(500000 * ticks_per_usec / cfg->symbol_num_perframe)
    , ticks_100(150000 * ticks_per_usec / cfg->symbol_num_perframe)
    , ticks_200(20000 * ticks_per_usec / cfg->symbol_num_perframe)
    , ticks_500(10000 * ticks_per_usec / cfg->symbol_num_perframe)
{
    rt_assert(socket_num <= kMaxNumSockets, "Too many network sockets");
    for (size_t i = 0; i < SOCKET_BUFFER_FRAME_NUM; i++) {
        packet_count_per_symbol[i] = new size_t[get_max_symbol_id()]();
    }
    memset(packet_count_per_frame, 0, SOCKET_BUFFER_FRAME_NUM * sizeof(size_t));

    tx_buffers_.malloc(
        SOCKET_BUFFER_FRAME_NUM * cfg->UE_ANT_NUM, cfg->mac_packet_length, 64);
    // init_data_from_file();

    task_ptok = (moodycamel::ProducerToken**)aligned_alloc(
        64, thread_num * sizeof(moodycamel::ProducerToken*));
    for (size_t i = 0; i < thread_num; i++)
        task_ptok[i] = new moodycamel::ProducerToken(send_queue_);

    // Start a thread to update data buffer
    create_threads(pthread_fun_wrapper<SimpleClientMac,
                       &SimpleClientMac::data_update_thread>,
        0, 1);

    for (size_t i = 0; i < socket_num; i++) {
        if (kUseIPv4) {
            socket_[i] = setup_socket_ipv4(cfg->ue_tx_port + i, false, 0);
            setup_sockaddr_remote_ipv4(
                &servaddr_ipv4[i], cfg->mac_rx_port + i, cfg->rx_addr.c_str());
            printf("Set up UDP socket client listening to port %zu"
                   " with remote address %s:%zu  \n",
                cfg->ue_tx_port + i, cfg->rx_addr.c_str(),
                cfg->mac_rx_port + i);
        } else {
            socket_[i] = setup_socket_ipv6(cfg->ue_tx_port + i, false, 0);
            setup_sockaddr_remote_ipv6(&servaddr_ipv6[i], cfg->mac_rx_port + i,
                "fe80::f436:d735:b04a:864a");
        }

        // if (!kUseDPDK && kConnectUDP) {
        //     int ret = connect(socket_[i], (struct sockaddr*)&servaddr_ipv4[i],
        //         sizeof(servaddr_ipv4[i]));
        //     rt_assert(ret == 0, "UDP socket connect failed");
        //     printf("UDP socket %zu connect() call ret %d\n", i, ret);
        // }
    }
    num_threads_ready_atomic = 0;
}

SimpleClientMac::~SimpleClientMac()
{
    IQ_data_coded.free();
    IQ_data.free();
    tx_buffers_.free();
    for (size_t i = 0; i < SOCKET_BUFFER_FRAME_NUM; i++) {
        free(packet_count_per_symbol[i]);
    }
}

void SimpleClientMac::startTX()
{
    frame_start = new double[kNumStatsFrames]();
    frame_end = new double[kNumStatsFrames]();

    // Create worker threads
    create_threads(
        pthread_fun_wrapper<SimpleClientMac, &SimpleClientMac::worker_thread>,
        0, thread_num);
    master_thread(0); // Start the master thread
}

void SimpleClientMac::startTXfromMain(
    double* in_frame_start, double* in_frame_end)
{
    frame_start = in_frame_start;
    frame_end = in_frame_end;

    // Create worker threads
    create_threads(
        pthread_fun_wrapper<SimpleClientMac, &SimpleClientMac::worker_thread>,
        0, thread_num);

    // Create the master thread
    create_threads(
        pthread_fun_wrapper<SimpleClientMac, &SimpleClientMac::master_thread>,
        thread_num, thread_num + 1);
}

void* SimpleClientMac::master_thread(int tid)
{
    signal(SIGINT, interrupt_handler);
    pin_to_core_with_offset(ThreadType::kMasterTX, core_offset, 0);

    // Wait for all SimpleClientMac threads (including master) to start runnung
    num_threads_ready_atomic++;
    while (num_threads_ready_atomic != thread_num + 1) {
        // Wait
    }

    // Load data of the first frame
    // Schedule one task for all antennas to avoid queue overflow
    for (size_t i = 0; i < 2; i++) {
        auto req_tag = gen_tag_t::frm_sym(i, 0);
        data_update_queue_.try_enqueue(req_tag._tag);
        // update_tx_buffer(req_tag);
    }

    // add some delay to ensure data update is finished
    sleep(1);
    int ret = 0;
    char* start_msg[1024];
    socklen_t addrlen = sizeof(servaddr_ipv4[0]);
    while (keep_running && ret == 0) {
        ret = recvfrom(socket_[0], start_msg, 1024, 0,
            (struct sockaddr*)&servaddr_ipv4[0], &addrlen);
    }
    std::cout << "Received WAKEUP Message\n"
              << "Starting Packet Transmission.." << std::endl;

    // Push tasks of the first symbol into task queue
    for (size_t i = 0; i < cfg->UE_ANT_NUM; i++) {
        auto req_tag = gen_tag_t::frm_sym_ue(0, 0, i);
        rt_assert(send_queue_.enqueue(*task_ptok[i % thread_num], req_tag._tag),
            "Send task enqueue failed");
    }

    frame_start[0] = get_time();
    uint64_t tick_start = rdtsc();
    double start_time = get_time();
    while (keep_running) {
        gen_tag_t ctag(0); // The completion tag
        int ret = completion_queue_.try_dequeue(ctag._tag);
        if (!ret)
            continue;

        const size_t comp_frame_slot = ctag.frame_id % SOCKET_BUFFER_FRAME_NUM;
        packet_count_per_frame[comp_frame_slot]++;
        size_t next_frame_id;
        if (packet_count_per_frame[comp_frame_slot] == cfg->UE_ANT_NUM) {
            if (kDebugPrintPerFrameDone) {
                printf("Finished transmit all antennas in frame: %u, "
                       "next frame scheduled in %.1f us\n",
                    ctag.frame_id, get_time() - start_time);
                start_time = get_time();
            }

            next_frame_id = ctag.frame_id + 1;
            if (next_frame_id == cfg->frames_to_test)
                break;
            frame_end[ctag.frame_id] = get_time();
            packet_count_per_frame[comp_frame_slot] = 0;

            delay_for_frame(ctag.frame_id, tick_start);
            tick_start = rdtsc();
            frame_start[next_frame_id] = get_time();

            for (size_t i = 0; i < cfg->UE_ANT_NUM; i++) {
                auto req_tag = gen_tag_t::frm_sym_ue(next_frame_id, 0, i);
                // update_tx_buffer(req_tag);
                rt_assert(send_queue_.enqueue(
                              *task_ptok[i % thread_num], req_tag._tag),
                    "Send task enqueue failed");
            }
            auto req_tag_for_data = gen_tag_t::frm_sym(next_frame_id + 1, 0);
            data_update_queue_.try_enqueue(req_tag_for_data._tag);
        } else {
            next_frame_id = ctag.frame_id;
        }
    }
    write_stats_to_file(cfg->frames_to_test);
    exit(0);
}

void* SimpleClientMac::data_update_thread(int tid)
{
    // Sender get better performance when this thread is not pinned to core
    // pin_to_core_with_offset(ThreadType::kWorker, 13, 0);
    printf("Data update thread running on core %d\n", sched_getcpu());

    while (true) {
        size_t tag = 0;
        if (!data_update_queue_.try_dequeue(tag))
            continue;
        for (size_t i = 0; i < cfg->UE_ANT_NUM; i++) {
            auto tag_for_ue = gen_tag_t::frm_sym_ue(
                ((gen_tag_t)tag).frame_id, ((gen_tag_t)tag).symbol_id, i);
            update_tx_buffer(tag_for_ue);
        }
    }
}

void SimpleClientMac::update_tx_buffer(gen_tag_t tag)
{
    auto* pkt = (MacPacket*)(tx_buffers_[tag_to_tx_buffers_index(tag)]);
    pkt->frame_id = tag.frame_id;
    pkt->symbol_id = 0;
    pkt->cell_id = 0;
    pkt->ue_id = tag.ue_id;

    // https://stackoverflow.com/questions/12149593/how-can-i-create-an-array-of-random-numbers-in-c
    std::random_device r;
    std::seed_seq seed{ r(), r(), r(), r(), r(), r(), r(), r() };
    std::mt19937 eng(seed); // a source of random data

    std::uniform_int_distribution<char> dist;
    std::vector<char> v(cfg->mac_data_bytes_num_perframe);

    generate(begin(v), end(v), bind(dist, eng));
    memcpy(pkt->data, (char*)v.data(), cfg->mac_data_bytes_num_perframe);

    // Print MAC packet summary
    printf("time %0.2f sending packet for frame %d, symbol %d, ue %d, bytes "
           "%d\n",
        get_time(), pkt->frame_id, pkt->symbol_id, pkt->ue_id,
        cfg->mac_data_bytes_num_perframe);

    for (size_t i = 0; i < cfg->mac_data_bytes_num_perframe; i++) {
        printf("%i ", *((uint8_t*)pkt->data + i));
    }
    printf("\n");
}

void* SimpleClientMac::worker_thread(int tid)
{
    pin_to_core_with_offset(ThreadType::kWorkerTX, core_offset + 1, tid);

    // Wait for all SimpleClientMac threads (including master) to start runnung
    num_threads_ready_atomic++;
    while (num_threads_ready_atomic != thread_num + 1) {
        // Wait
    }

    const size_t buffer_length = cfg->mac_packet_length;
    double begin = get_time();
    size_t total_tx_packets = 0;
    size_t total_tx_packets_rolling = 0;
    size_t max_symbol_id = 1; // get_max_symbol_id();
    int radio_lo = tid * cfg->UE_ANT_NUM / thread_num;
    int radio_hi = (tid + 1) * cfg->UE_ANT_NUM / thread_num;
    size_t ant_num_this_thread = cfg->UE_ANT_NUM / thread_num
        + ((size_t)tid < cfg->UE_ANT_NUM % thread_num ? 1 : 0);
    printf("In thread %zu, %zu users (radio %d - %d ), UE_ANT_NUM: %zu, num "
           "threads %zu:\n",
        (size_t)tid, ant_num_this_thread, radio_lo, radio_hi - 1,
        cfg->UE_ANT_NUM, thread_num);
    int radio_id = radio_lo;
    while (true) {
        size_t tag;
        if (!send_queue_.try_dequeue_from_producer(*(task_ptok[tid]), tag))
            continue;
        const size_t tx_bufs_idx = tag_to_tx_buffers_index(tag);

        // size_t start_tsc_send = rdtsc();
        // Send a message to the server. We assume that the server is running.
        // if (kUseDPDK or !kConnectUDP) {
        int ret = sendto(socket_[radio_id], tx_buffers_[tx_bufs_idx],
            buffer_length, 0, (struct sockaddr*)&servaddr_ipv4[radio_id],
            sizeof(servaddr_ipv4[radio_id]));
        rt_assert(ret >= 0, "Worker: sendto() failed");
        // } else {
        //     int ret = send(
        //         socket_[radio_id], tx_buffers_[tx_bufs_idx], buffer_length, 0);
        //     if (ret < 0) {
        //         fprintf(stderr,
        //             "send() failed. Error = %s. Is a server running at "
        //             "%s:%d?\n",
        //             strerror(errno), cfg->rx_addr.c_str(),
        //             cfg->bs_port + radio_id);
        //         exit(-1);
        //     }
        // }

        rt_assert(completion_queue_.enqueue(tag), "Completion enqueue failed");

        total_tx_packets_rolling++;
        total_tx_packets++;
        if (total_tx_packets_rolling
            == ant_num_this_thread * max_symbol_id * 1000) {
            double end = get_time();
            double byte_len
                = buffer_length * ant_num_this_thread * max_symbol_id * 1000.f;
            double diff = end - begin;
            printf("Thread %zu send %zu frames in %f secs, tput %f Mbps\n",
                (size_t)tid,
                total_tx_packets / (ant_num_this_thread * max_symbol_id),
                diff / 1e6, byte_len * 8 * 1e6 / diff / 1024 / 1024);
            begin = get_time();
            total_tx_packets_rolling = 0;
        }

        if (++radio_id == radio_hi)
            radio_id = radio_lo;
    }
}

size_t SimpleClientMac::get_max_symbol_id() const
{
    size_t max_symbol_id = cfg->downlink_mode
        ? cfg->ul_data_symbol_num_perframe
        : cfg->ul_data_symbol_num_perframe - cfg->UL_PILOT_SYMS;
    return max_symbol_id;
}

void SimpleClientMac::init_data_from_file()
{
    const size_t packets_per_frame = cfg->symbol_num_perframe * cfg->UE_ANT_NUM;
    IQ_data.calloc(packets_per_frame, cfg->OFDM_FRAME_LEN * 2, 64);
    IQ_data_coded.calloc(packets_per_frame, cfg->OFDM_FRAME_LEN * 2, 64);

    const std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);

    std::string filename;
    if (kUseLDPC) {
        filename = cur_directory + "/data/LDPC_orig_data_2048_ant"
            + std::to_string(cfg->UE_ANT_NUM) + ".bin";
    } else {
        filename = cur_directory + "/data/orig_data_2048_ant"
            + std::to_string(cfg->UE_ANT_NUM) + ".bin";
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
            IQ_data_coded[i][j] = (unsigned short)(IQ_data[i][j] * 32768);
            // printf("i:%d, j:%d, Coded: %d, orignal:
            // %.4f\n",i,j/2,IQ_data_coded[i][j],IQ_data[i][j]);
        }
    }
    fclose(fp);
}

void SimpleClientMac::delay_for_symbol(
    size_t tx_frame_count, uint64_t tick_start)
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

void SimpleClientMac::delay_for_frame(
    size_t tx_frame_count, uint64_t tick_start)
{
    if (tx_frame_count < 500) {
        delay_ticks(tick_start, 2 * cfg->symbol_num_perframe * ticks_all);
    } else {
        delay_ticks(tick_start, cfg->symbol_num_perframe * ticks_all);
    }
}

void SimpleClientMac::create_threads(
    void* (*worker)(void*), int tid_start, int tid_end)
{
    int ret;
    for (int i = tid_start; i < tid_end; i++) {
        pthread_t thread;
        auto context = new EventHandlerContext<SimpleClientMac>;
        context->obj_ptr = this;
        context->id = i;
        ret = pthread_create(&thread, NULL, worker, context);
        rt_assert(ret == 0, "pthread_create() failed");
    }
}

void SimpleClientMac::write_stats_to_file(size_t tx_frame_count) const
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
