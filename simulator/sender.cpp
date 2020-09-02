/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#include "sender.hpp"
#include "datatype_conversion.h"
#include "udp_client.h"
#include <thread>

bool keep_running = true;

// A spinning barrier to synchronize the start of worker threads
std::atomic<size_t> num_workers_ready_atomic;

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

Sender::Sender(Config* cfg, size_t num_worker_threads_, size_t core_offset,
    size_t delay, bool enable_slow_start, std::string server_mac_addr_str,
    bool create_thread_for_master)
    : cfg(cfg)
    , freq_ghz(measure_rdtsc_freq())
    , ticks_per_usec(freq_ghz * 1e3)
    , num_worker_threads_(num_worker_threads_)
    , enable_slow_start(enable_slow_start)
    , core_offset(core_offset)
    , delay(delay)
    , ticks_all(delay * ticks_per_usec / cfg->symbol_num_perframe)
    , ticks_5(500000 * ticks_per_usec / cfg->symbol_num_perframe)
    , ticks_100(150000 * ticks_per_usec / cfg->symbol_num_perframe)
    , ticks_200(20000 * ticks_per_usec / cfg->symbol_num_perframe)
    , ticks_500(10000 * ticks_per_usec / cfg->symbol_num_perframe)
{
    _unused(server_mac_addr_str);
    for (size_t i = 0; i < SOCKET_BUFFER_FRAME_NUM; i++) {
        packet_count_per_symbol[i] = new size_t[get_max_symbol_id()]();
    }
    memset(packet_count_per_frame, 0, SOCKET_BUFFER_FRAME_NUM * sizeof(size_t));

    init_iq_from_file(std::string(TOSTRING(PROJECT_DIRECTORY))
        + "/data/LDPC_rx_data_2048_ant" + std::to_string(cfg->BS_ANT_NUM)
        + ".bin");

    task_ptok = (moodycamel::ProducerToken**)aligned_alloc(
        64, num_worker_threads_ * sizeof(moodycamel::ProducerToken*));
    for (size_t i = 0; i < num_worker_threads_; i++)
        task_ptok[i] = new moodycamel::ProducerToken(send_queue_);

    // Create a master thread when started from simulator
    if (create_thread_for_master)
        create_threads(pthread_fun_wrapper<Sender, &Sender::master_thread>,
            num_worker_threads_, num_worker_threads_ + 1);

#ifdef USE_DPDK
    DpdkTransport::dpdk_init(core_offset, num_worker_threads_);
    mbuf_pool = DpdkTransport::create_mempool();

    uint16_t portid = 0; // For now, hard-code to port zero
    if (DpdkTransport::nic_init(portid, mbuf_pool, num_worker_threads_) != 0)
        rte_exit(EXIT_FAILURE, "Cannot init port %u\n", portid);

    // Parse IP addresses and MAC addresses
    int ret = inet_pton(AF_INET, cfg->bs_rru_addr.c_str(), &sender_addr);
    rt_assert(ret == 1, "Invalid sender IP address");
    ret = inet_pton(AF_INET, cfg->bs_server_addr.c_str(), &server_addr);
    rt_assert(ret == 1, "Invalid server IP address");

    ether_addr* parsed_mac = ether_aton(server_mac_addr_str.c_str());
    rt_assert(parsed_mac != NULL, "Invalid server mac address");
    memcpy(&server_mac_addr, parsed_mac, sizeof(ether_addr));

    ret = rte_eth_macaddr_get(portid, &sender_mac_addr);
    rt_assert(ret == 0, "Cannot get MAC address of the port");
    printf("Number of DPDK cores: %d\n", rte_lcore_count());
#endif
    num_workers_ready_atomic = 0;
}

Sender::~Sender()
{
    iq_data_short_.free();
    for (size_t i = 0; i < SOCKET_BUFFER_FRAME_NUM; i++) {
        free(packet_count_per_symbol[i]);
    }
}

void Sender::startTX()
{
    frame_start = new double[kNumStatsFrames]();
    frame_end = new double[kNumStatsFrames]();

    create_threads(pthread_fun_wrapper<Sender, &Sender::worker_thread>, 0,
        num_worker_threads_);
    master_thread(0); // Start the master thread
}

void Sender::startTXfromMain(double* in_frame_start, double* in_frame_end)
{
    frame_start = in_frame_start;
    frame_end = in_frame_end;

    create_threads(pthread_fun_wrapper<Sender, &Sender::worker_thread>, 0,
        num_worker_threads_);
}

void* Sender::master_thread(int)
{
    signal(SIGINT, interrupt_handler);
    pin_to_core_with_offset(ThreadType::kMasterTX, core_offset, 0);

    // Wait for all worker threads to be ready
    while (num_workers_ready_atomic != num_worker_threads_) {
        // Wait
    }

    const size_t max_symbol_id = get_max_symbol_id();

    // Push tasks of the first symbol into task queue
    for (size_t i = 0; i < cfg->BS_ANT_NUM; i++) {
        auto req_tag = gen_tag_t::frm_sym_ant(0, 0, i);
        rt_assert(send_queue_.enqueue(
                      *task_ptok[i % num_worker_threads_], req_tag._tag),
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

            // Add inter-symbol delay
            if (enable_slow_start) {
                if (ctag.frame_id <= 5) {
                    delay_ticks(tick_start, ticks_5);
                } else if (ctag.frame_id < 100) {
                    delay_ticks(tick_start, ticks_100);
                } else if (ctag.frame_id < 200) {
                    delay_ticks(tick_start, ticks_200);
                } else if (ctag.frame_id < 500) {
                    delay_ticks(tick_start, ticks_500);
                } else {
                    delay_ticks(tick_start, ticks_all);
                }
            } else {
                delay_ticks(tick_start, ticks_all);
            }

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

                // Add end-of-frame delay
                if (cfg->downlink_mode) {
                    if (ctag.frame_id < 500) {
                        delay_ticks(tick_start,
                            2 * cfg->data_symbol_num_perframe * ticks_all);
                    } else {
                        delay_ticks(tick_start,
                            cfg->data_symbol_num_perframe * ticks_all);
                    }
                }

                tick_start = rdtsc();
                frame_start[next_frame_id % kNumStatsFrames] = get_time();
            } else {
                next_frame_id = ctag.frame_id;
            }

            for (size_t i = 0; i < cfg->BS_ANT_NUM; i++) {
                auto req_tag
                    = gen_tag_t::frm_sym_ant(next_frame_id, next_symbol_id, i);
                rt_assert(
                    send_queue_.enqueue(
                        *task_ptok[i % num_worker_threads_], req_tag._tag),
                    "Send task enqueue failed");
            }
        }
    }
    write_stats_to_file(cfg->frames_to_test);
    exit(0);
}

void* Sender::worker_thread(int tid)
{
    pin_to_core_with_offset(ThreadType::kWorkerTX, core_offset + 1, tid);

    // Wait for all Sender threads (including master) to start runnung
    num_workers_ready_atomic++;
    while (num_workers_ready_atomic != num_worker_threads_) {
        // Wait
    }

    DFTI_DESCRIPTOR_HANDLE mkl_handle;
    DftiCreateDescriptor(
        &mkl_handle, DFTI_SINGLE, DFTI_COMPLEX, 1, cfg->OFDM_CA_NUM);
    DftiCommitDescriptor(mkl_handle);

    const size_t max_symbol_id = get_max_symbol_id();
    const size_t radio_lo = tid * cfg->nRadios / num_worker_threads_;
    const size_t radio_hi = (tid + 1) * cfg->nRadios / num_worker_threads_;
    const size_t ant_num_this_thread = cfg->BS_ANT_NUM / num_worker_threads_
        + ((size_t)tid < cfg->BS_ANT_NUM % num_worker_threads_ ? 1 : 0);

    UDPClient udp_client;
    auto fft_inout = reinterpret_cast<complex_float*>(
        memalign(64, cfg->OFDM_CA_NUM * sizeof(complex_float)));
    auto* socks_pkt_buf = reinterpret_cast<Packet*>(malloc(cfg->packet_length));

    double begin = get_time();
    size_t total_tx_packets = 0;
    size_t total_tx_packets_rolling = 0;
    size_t cur_radio = radio_lo;

    printf("In thread %zu, %zu antennas, BS_ANT_NUM: %zu, num threads %zu:\n",
        (size_t)tid, ant_num_this_thread, cfg->BS_ANT_NUM, num_worker_threads_);

    // We currently don't support zero-padding OFDM prefix and postfix
    rt_assert(cfg->packet_length
        == Packet::kOffsetOfData
            + 2 * sizeof(unsigned short) * (cfg->CP_LEN + cfg->OFDM_CA_NUM));

    while (true) {
        gen_tag_t tag = 0;
        if (!send_queue_.try_dequeue_from_producer(*(task_ptok[tid]), tag._tag))
            continue;

        size_t start_tsc_send = rdtsc();

        // Send a message to the server. We assume that the server is running.
        Packet* pkt = socks_pkt_buf;
#ifdef USE_DPDK
        rte_mbuf* tx_mbuf = DpdkTransport::alloc_udp(mbuf_pool, sender_mac_addr,
            server_mac_addr, sender_addr, server_addr, cfg->bs_rru_port,
            cfg->bs_server_port + tag.ant_id, cfg->packet_length);
        pkt = (Packet*)(rte_pktmbuf_mtod(tx_mbuf, uint8_t*) + kPayloadOffset);
#endif

        // Update the TX buffer
        pkt->frame_id = tag.frame_id;
        pkt->symbol_id = cfg->getSymbolId(tag.symbol_id);
        pkt->cell_id = 0;
        pkt->ant_id = tag.ant_id;
        memcpy(pkt->data,
            iq_data_short_[(tag.symbol_id * cfg->BS_ANT_NUM) + tag.ant_id],
            (cfg->CP_LEN + cfg->OFDM_CA_NUM) * sizeof(unsigned short) * 2);
        if (cfg->fft_in_rru) {
            run_fft(pkt, fft_inout, mkl_handle);
        }

#ifdef USE_DPDK
        rt_assert(rte_eth_tx_burst(0, tid, &tx_mbuf, 1) == 1,
            "rte_eth_tx_burst() failed");
#else
        udp_client.send(cfg->bs_server_addr, cfg->bs_server_port + cur_radio,
            reinterpret_cast<uint8_t*>(socks_pkt_buf), cfg->packet_length);
#endif

        if (kDebugSenderReceiver) {
            printf("Thread %d (tag = %s) transmit frame %d, symbol %d, ant %d, "
                   "TX time: %.3f us\n",
                tid, gen_tag_t(tag).to_string().c_str(), pkt->frame_id,
                pkt->symbol_id, pkt->ant_id,
                cycles_to_us(rdtsc() - start_tsc_send, freq_ghz));
        }

        rt_assert(
            completion_queue_.enqueue(tag._tag), "Completion enqueue failed");

        total_tx_packets_rolling++;
        total_tx_packets++;
        if (total_tx_packets_rolling
            == ant_num_this_thread * max_symbol_id * 1000) {
            double end = get_time();
            double byte_len = cfg->packet_length * ant_num_this_thread
                * max_symbol_id * 1000.f;
            double diff = end - begin;
            printf("Thread %zu send %zu frames in %f secs, tput %f Mbps\n",
                (size_t)tid,
                total_tx_packets / (ant_num_this_thread * max_symbol_id),
                diff / 1e6, byte_len * 8 * 1e6 / diff / 1024 / 1024);
            begin = get_time();
            total_tx_packets_rolling = 0;
        }

        if (++cur_radio == radio_hi)
            cur_radio = radio_lo;
    }
}

size_t Sender::get_max_symbol_id() const
{
    size_t max_symbol_id = cfg->downlink_mode
        ? cfg->pilot_symbol_num_perframe
        : cfg->pilot_symbol_num_perframe + cfg->ul_data_symbol_num_perframe;
    return max_symbol_id;
}

void Sender::init_iq_from_file(std::string filename)
{
    const size_t packets_per_frame = cfg->symbol_num_perframe * cfg->BS_ANT_NUM;
    iq_data_short_.calloc(
        packets_per_frame, (cfg->CP_LEN + cfg->OFDM_CA_NUM) * 2, 64);

    Table<float> iq_data_float;
    iq_data_float.calloc(
        packets_per_frame, (cfg->CP_LEN + cfg->OFDM_CA_NUM) * 2, 64);

    FILE* fp = fopen(filename.c_str(), "rb");
    rt_assert(fp != nullptr, "Failed to open IQ data file");

    for (size_t i = 0; i < packets_per_frame; i++) {
        const size_t expected_count = (cfg->CP_LEN + cfg->OFDM_CA_NUM) * 2;
        const size_t actual_count
            = fread(iq_data_float[i], sizeof(float), expected_count, fp);
        if (expected_count != actual_count) {
            fprintf(stderr,
                "Sender: Failed to read IQ data file %s. Packet %zu: expected "
                "%zu I/Q samples but read %zu. Errno %s\n",
                filename.c_str(), i, expected_count, actual_count,
                strerror(errno));
            exit(-1);
        }
        for (size_t j = 0; j < expected_count; j++) {
            iq_data_short_[i][j]
                = static_cast<unsigned short>(iq_data_float[i][j] * 32768);
        }
    }
    fclose(fp);
    iq_data_float.free();
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

void Sender::run_fft(Packet* pkt, complex_float* fft_inout,
    DFTI_DESCRIPTOR_HANDLE mkl_handle) const
{
    // pkt->data has (CP_LEN + OFDM_CA_NUM) unsigned short samples. After FFT,
    // we'll remove the cyclic prefix and have OFDM_CA_NUM short samples left.
    simd_convert_short_to_float(&pkt->data[2 * cfg->CP_LEN],
        reinterpret_cast<float*>(fft_inout), cfg->OFDM_CA_NUM * 2);

    DftiComputeForward(mkl_handle, reinterpret_cast<float*>(fft_inout));

    simd_convert_float32_to_float16(reinterpret_cast<float*>(pkt->data),
        reinterpret_cast<float*>(fft_inout), cfg->OFDM_CA_NUM * 2);
}
