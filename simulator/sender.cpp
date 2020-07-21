/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#include "sender.hpp"
#include <thread>

bool keep_running = true;

// A spinning barrier to synchronize the start of Sender threads
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

inline size_t Sender::tag_to_tx_buffers_index(gen_tag_t tag) const
{
    const size_t frame_slot = tag.frame_id % SOCKET_BUFFER_FRAME_NUM;
    return (frame_slot * (get_max_symbol_id() * cfg->BS_ANT_NUM))
        + (tag.symbol_id * cfg->BS_ANT_NUM) + tag.ant_id;
}

Sender::Sender(Config* cfg, size_t thread_num, size_t core_offset, size_t delay,
    bool enable_slow_start, std::string server_mac_addr_str,
    bool create_thread_for_master)
    : cfg(cfg)
    , freq_ghz(measure_rdtsc_freq())
    , ticks_per_usec(freq_ghz * 1e3)
    , thread_num(thread_num)
    , socket_num(cfg->nRadios)
    , enable_slow_start(enable_slow_start)
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

    tx_buffers_.calloc(
        SOCKET_BUFFER_FRAME_NUM * get_max_symbol_id() * cfg->BS_ANT_NUM,
        kTXBufOffset + cfg->packet_length, 64);
    init_IQ_from_file();

    task_ptok = (moodycamel::ProducerToken**)aligned_alloc(
        64, thread_num * sizeof(moodycamel::ProducerToken*));
    for (size_t i = 0; i < thread_num; i++)
        task_ptok[i] = new moodycamel::ProducerToken(send_queue_);

    // Start a thread to update data buffer
    create_threads(
        pthread_fun_wrapper<Sender, &Sender::data_update_thread>, 0, 1);

    // Create a separate thread for master thread
    // when sender is started from simulator
    if (create_thread_for_master)
        create_threads(pthread_fun_wrapper<Sender, &Sender::master_thread>,
            thread_num, thread_num + 1);

    for (size_t i = 0; i < socket_num; i++) {
        if (kUseIPv4) {
            socket_[i] = setup_socket_ipv4(cfg->ue_tx_port + i, false, 0);
            setup_sockaddr_remote_ipv4(
                &servaddr_ipv4[i], cfg->bs_port + i, cfg->server_addr.c_str());
        } else {
            socket_[i] = setup_socket_ipv6(cfg->ue_tx_port + i, false, 0);
            setup_sockaddr_remote_ipv6(&servaddr_ipv6[i], cfg->bs_port + i,
                "fe80::f436:d735:b04a:864a");
        }

        if (!kUseDPDK && kConnectUDP) {
            int ret = connect(socket_[i], (struct sockaddr*)&servaddr_ipv4[i],
                sizeof(servaddr_ipv4[i]));
            rt_assert(ret == 0, "UDP socket connect failed");
            printf("UDP socket %zu connected\n", i);
        }
    }
    num_threads_ready_atomic = 0;
}

Sender::~Sender()
{
    IQ_data_coded.free();
    IQ_data.free();
    tx_buffers_.free();
    for (size_t i = 0; i < SOCKET_BUFFER_FRAME_NUM; i++) {
        free(packet_count_per_symbol[i]);
    }
}

void Sender::startTX()
{
    frame_start = new double[kNumStatsFrames]();
    frame_end = new double[kNumStatsFrames]();

    // Create worker threads
    create_threads(
        pthread_fun_wrapper<Sender, &Sender::worker_thread>, 0, thread_num);
    master_thread(0); // Start the master thread
}

void Sender::startTXfromMain(double* in_frame_start, double* in_frame_end)
{
    frame_start = in_frame_start;
    frame_end = in_frame_end;

    // Create worker threads
    create_threads(
        pthread_fun_wrapper<Sender, &Sender::worker_thread>, 0, thread_num);
}

void* Sender::master_thread(int tid)
{
    signal(SIGINT, interrupt_handler);
    pin_to_core_with_offset(ThreadType::kMasterTX, core_offset, 0);

    // Wait for all Sender threads (including master) to start runnung
    num_threads_ready_atomic++;
    while (num_threads_ready_atomic != thread_num + 1) {
        // Wait
    }

    const size_t max_symbol_id = get_max_symbol_id();
    // Load data of the first frame
    // Schedule one task for all antennas to avoid queue overflow
    for (size_t i = 0; i < max_symbol_id; i++) {
        auto req_tag = gen_tag_t::frm_sym(0, i);
        data_update_queue_.try_enqueue(req_tag._tag);
        // update_tx_buffer(req_tag);
    }

    // add some delay to ensure data update is finished
    sleep(1);
    // Push tasks of the first symbol into task queue
    for (size_t i = 0; i < cfg->BS_ANT_NUM; i++) {
        auto req_tag = gen_tag_t::frm_sym_ant(0, 0, i);
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

        packet_count_per_symbol[comp_frame_slot][ctag.symbol_id]++;
        if (packet_count_per_symbol[comp_frame_slot][ctag.symbol_id]
            == cfg->BS_ANT_NUM) {
            if (kDebugSenderReceiver) {
                printf("Finished transmit all antennas in frame: %u, "
                       "symbol: %u, in %.1f us\n ",
                    ctag.frame_id, ctag.symbol_id, get_time() - start_time);
            }

            packet_count_per_symbol[comp_frame_slot][ctag.symbol_id] = 0;
            packet_count_per_frame[comp_frame_slot]++;
            delay_for_symbol(ctag.frame_id, tick_start);
            tick_start = rdtsc();

            const size_t next_symbol_id = (ctag.symbol_id + 1) % max_symbol_id;
            size_t next_frame_id;
            if (packet_count_per_frame[comp_frame_slot] == max_symbol_id) {
                if (kDebugSenderReceiver || kDebugPrintPerFrameDone) {
                    printf("Finished transmit all antennas in frame: %u, "
                           "next frame scheduled in %.1f us\n",
                        ctag.frame_id, get_time() - start_time);
                    start_time = get_time();
                }

                next_frame_id = ctag.frame_id + 1;
                if (next_frame_id == cfg->frames_to_test)
                    break;
                frame_end[ctag.frame_id % kNumStatsFrames] = get_time();
                packet_count_per_frame[comp_frame_slot] = 0;

                delay_for_frame(ctag.frame_id, tick_start);
                tick_start = rdtsc();
                frame_start[next_frame_id % kNumStatsFrames] = get_time();
            } else {
                next_frame_id = ctag.frame_id;
            }

            for (size_t i = 0; i < cfg->BS_ANT_NUM; i++) {
                auto req_tag
                    = gen_tag_t::frm_sym_ant(next_frame_id, next_symbol_id, i);
                rt_assert(send_queue_.enqueue(
                              *task_ptok[i % thread_num], req_tag._tag),
                    "Send task enqueue failed");
            }
            auto req_tag_for_data
                = gen_tag_t::frm_sym(ctag.frame_id + 1, ctag.symbol_id);
            data_update_queue_.try_enqueue(req_tag_for_data._tag);
        }
    }
    write_stats_to_file(cfg->frames_to_test);
    exit(0);
}

void* Sender::data_update_thread(int tid)
{
    // Sender get better performance when this thread is not pinned to core
    // pin_to_core_with_offset(ThreadType::kWorker, 13, 0);
    printf("Data update thread running on core %d\n", sched_getcpu());

    while (true) {
        size_t tag = 0;
        if (!data_update_queue_.try_dequeue(tag))
            continue;
        for (size_t i = 0; i < cfg->BS_ANT_NUM; i++) {
            auto tag_for_ant = gen_tag_t::frm_sym_ant(
                ((gen_tag_t)tag).frame_id, ((gen_tag_t)tag).symbol_id, i);
            update_tx_buffer(tag_for_ant);
        }
    }
}

void Sender::update_tx_buffer(gen_tag_t tag)
{
    auto* pkt
        = (Packet*)(tx_buffers_[tag_to_tx_buffers_index(tag)] + kTXBufOffset);
    pkt->frame_id = tag.frame_id;
    pkt->symbol_id = cfg->getSymbolId(tag.symbol_id);
    pkt->cell_id = 0;
    pkt->ant_id = tag.ant_id;

    size_t data_index = (tag.symbol_id * cfg->BS_ANT_NUM) + tag.ant_id;
    memcpy(pkt->data, (char*)IQ_data_coded[data_index],
        cfg->OFDM_FRAME_LEN * sizeof(unsigned short) * 2);
}

void* Sender::worker_thread(int tid)
{
    pin_to_core_with_offset(ThreadType::kWorkerTX, core_offset + 1, tid);

    // Wait for all Sender threads (including master) to start runnung
    num_threads_ready_atomic++;
    while (num_threads_ready_atomic != thread_num + 1) {
        // Wait
    }

    const size_t buffer_length = kTXBufOffset + cfg->packet_length;
    double begin = get_time();
    size_t total_tx_packets = 0;
    size_t total_tx_packets_rolling = 0;
    size_t max_symbol_id = get_max_symbol_id();
    int radio_lo = tid * cfg->nRadios / thread_num;
    int radio_hi = (tid + 1) * cfg->nRadios / thread_num;
    size_t ant_num_this_thread = cfg->BS_ANT_NUM / thread_num
        + ((size_t)tid < cfg->BS_ANT_NUM % thread_num ? 1 : 0);
    printf("In thread %zu, %zu antennas, BS_ANT_NUM: %zu, num threads %zu:\n",
        (size_t)tid, ant_num_this_thread, cfg->BS_ANT_NUM, thread_num);
    int radio_id = radio_lo;
    while (true) {
        size_t tag;
        if (!send_queue_.try_dequeue_from_producer(*(task_ptok[tid]), tag))
            continue;
        const size_t tx_bufs_idx = tag_to_tx_buffers_index(tag);

        size_t start_tsc_send = rdtsc();
        // Send a message to the server. We assume that the server is running.
        if (kUseDPDK or !kConnectUDP) {
            int ret = sendto(socket_[radio_id], tx_buffers_[tx_bufs_idx],
                buffer_length, 0, (struct sockaddr*)&servaddr_ipv4[tid],
                sizeof(servaddr_ipv4[tid]));
            rt_assert(ret >= 0, "Worker: sendto() failed");
        } else {
            int ret = send(
                socket_[radio_id], tx_buffers_[tx_bufs_idx], buffer_length, 0);
            if (ret < 0) {
                fprintf(stderr,
                    "send() failed. Error = %s. Is a server running at %s?\n",
                    strerror(errno), cfg->server_addr.c_str());
                exit(-1);
            }
        }

        if (kDebugSenderReceiver) {
            auto* pkt = reinterpret_cast<Packet*>(tx_buffers_[tx_bufs_idx]);
            printf("Thread %d (tag = %s) transmit frame %d, symbol %d, ant %d, "
                   "TX buffer: %zu, TX time: %.3f us\n",
                tid, gen_tag_t(tag).to_string().c_str(), pkt->frame_id,
                pkt->symbol_id, pkt->ant_id, tx_bufs_idx,
                cycles_to_us(rdtsc() - start_tsc_send, freq_ghz));
        }

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

    std::string filename = cur_directory + "/data/LDPC_rx_data_2048_ant"
        + std::to_string(cfg->BS_ANT_NUM) + ".bin";

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

void Sender::delay_for_symbol(size_t tx_frame_count, uint64_t tick_start)
{
    if (enable_slow_start) {
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
    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string filename = cur_directory + "/data/tx_result.txt";
    printf("Printing sender results to file \"%s\"...\n", filename.c_str());
    FILE* fp_debug = fopen(filename.c_str(), "w");
    rt_assert(fp_debug != nullptr, "Failed to open stats file");
    for (size_t i = 0; i < tx_frame_count; i++) {
        fprintf(fp_debug, "%.5f\n", frame_end[i % kNumStatsFrames]);
    }
}
