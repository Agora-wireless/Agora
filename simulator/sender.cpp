#include "sender.hpp"
#include "datatype_conversion.h"
#include "udp_client.h"
#include <thread>

bool keep_running = true;

static constexpr size_t kMacAddrBtyes = 17;

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

Sender::Sender(Config* cfg, size_t socket_thread_num, size_t core_offset,
    size_t frame_duration, size_t enable_slow_start,
    std::string server_mac_addr_str, bool create_thread_for_master)
    : cfg(cfg)
    , freq_ghz(measure_rdtsc_freq())
    , ticks_per_usec(freq_ghz * 1e3)
    , socket_thread_num(socket_thread_num)
    , enable_slow_start(enable_slow_start)
    , core_offset(core_offset)
    , frame_duration_(frame_duration)
    , ticks_all(frame_duration_ * ticks_per_usec / cfg->symbol_num_perframe)
    , ticks_wnd_1(
          200000 /* 200 ms */ * ticks_per_usec / cfg->symbol_num_perframe)
    , ticks_wnd_2(
          15 * frame_duration_ * ticks_per_usec / cfg->symbol_num_perframe)
{
    std::printf(
        "Initializing sender, sending to base station server at %s, frame "
        "duration = %.2f ms, slow start = %s\n",
        cfg->bs_server_addr.c_str(), frame_duration / 1000.0,
        enable_slow_start == 1 ? "yes" : "no");

    _unused(server_mac_addr_str);
    for (size_t i = 0; i < kFrameWnd; i++) {
        packet_count_per_symbol[i] = new size_t[cfg->symbol_num_perframe]();
    }

    init_iq_from_file(std::string(TOSTRING(PROJECT_DIRECTORY))
        + "/data/LDPC_rx_data_2048_ant" + std::to_string(cfg->BS_ANT_NUM)
        + ".bin");

    task_ptok = static_cast<moodycamel::ProducerToken**>(
        Agora_memory::padded_aligned_alloc(Agora_memory::Alignment_t::k64Align,
            (socket_thread_num * sizeof(moodycamel::ProducerToken*))));
    for (size_t i = 0; i < socket_thread_num; i++)
        task_ptok[i] = new moodycamel::ProducerToken(send_queue_);

    // Create a master thread when started from simulator
    if (create_thread_for_master)
        create_threads(pthread_fun_wrapper<Sender, &Sender::master_thread>,
            socket_thread_num, socket_thread_num + 1);

#ifdef USE_DPDK
    DpdkTransport::dpdk_init(core_offset, socket_thread_num);
    mbuf_pool = DpdkTransport::create_mempool(cfg->packet_length);

    // Parse IP addresses
    int ret = inet_pton(AF_INET, cfg->bs_rru_addr.c_str(), &bs_rru_addr);
    rt_assert(ret == 1, "Invalid sender IP address");
    ret = inet_pton(AF_INET, cfg->bs_server_addr.c_str(), &bs_server_addr);
    rt_assert(ret == 1, "Invalid server IP address");

    rt_assert(cfg->dpdk_num_ports <= rte_eth_dev_count_avail(),
        "Invalid number of DPDK ports");

    rt_assert(server_mac_addr_str.length()
            == (cfg->dpdk_num_ports * (kMacAddrBtyes + 1) - 1),
        "Invalid length of server MAC address");
    sender_mac_addr.resize(cfg->dpdk_num_ports);
    server_mac_addr.resize(cfg->dpdk_num_ports);

    for (uint16_t port_id = 0; port_id < cfg->dpdk_num_ports; port_id++) {
        if (DpdkTransport::nic_init(
                port_id, mbuf_pool, socket_thread_num, cfg->packet_length)
            != 0)
            rte_exit(EXIT_FAILURE, "Cannot init port %u\n", port_id);
        // Parse MAC addresses
        ether_addr* parsed_mac = ether_aton(
            server_mac_addr_str
                .substr(port_id * (kMacAddrBtyes + 1), kMacAddrBtyes)
                .c_str());
        rt_assert(parsed_mac != NULL, "Invalid server mac address");
        std::memcpy(&server_mac_addr[port_id], parsed_mac, sizeof(ether_addr));

        ret = rte_eth_macaddr_get(port_id, &sender_mac_addr[port_id]);
        rt_assert(ret == 0, "Cannot get MAC address of the port");
        std::printf("Number of DPDK cores: %d\n", rte_lcore_count());
    }

#endif
    num_workers_ready_atomic = 0;
}

Sender::~Sender()
{
    iq_data_short_.free();
    for (size_t i = 0; i < kFrameWnd; i++) {
        std::free(packet_count_per_symbol[i]);
    }
}

void Sender::startTX()
{
    frame_start = new double[kNumStatsFrames]();
    frame_end = new double[kNumStatsFrames]();

    create_threads(pthread_fun_wrapper<Sender, &Sender::worker_thread>, 0,
        socket_thread_num);
    master_thread(0); // Start the master thread
}

void Sender::startTXfromMain(double* in_frame_start, double* in_frame_end)
{
    frame_start = in_frame_start;
    frame_end = in_frame_end;

    create_threads(pthread_fun_wrapper<Sender, &Sender::worker_thread>, 0,
        socket_thread_num);
}

size_t Sender::FindNextSymbol(size_t frame, size_t start_symbol)
{
    size_t next_symbol_id;
    for (next_symbol_id = start_symbol;
         (next_symbol_id < cfg->symbol_num_perframe); next_symbol_id++) {
        SymbolType symbol_type = cfg->get_symbol_type(frame, next_symbol_id);
        if ((symbol_type == SymbolType::kPilot)
            || (symbol_type == SymbolType::kUL)) {
            break;
        }
    }
    return next_symbol_id;
}

void Sender::ScheduleSymbol(size_t frame, size_t symbol_id)
{
    for (size_t i = 0; i < cfg->BS_ANT_NUM; i++) {
        auto req_tag = gen_tag_t::frm_sym_ant(frame, symbol_id, i);
        rt_assert(send_queue_.enqueue(
                      *task_ptok[i % socket_thread_num], req_tag._tag),
            "Send task enqueue failed");
    }
}

void* Sender::master_thread(int)
{
    signal(SIGINT, interrupt_handler);
    pin_to_core_with_offset(ThreadType::kMasterTX, core_offset, 0);

    // Wait for all worker threads to be ready
    while (num_workers_ready_atomic != socket_thread_num) {
        // Wait
    }

    frame_start[0] = get_time();
    double start_time = get_time();
    uint64_t tick_start = rdtsc();

    size_t start_symbol = FindNextSymbol(0, 0);
    //Delay until the start of the first symbol (pilot)
    if (start_symbol > 0) {
        std::printf("Sender: Starting symbol %zu delaying\n", start_symbol);
        delay_ticks(tick_start, get_ticks_for_frame(0) * start_symbol);
        tick_start = rdtsc();
    }
    rt_assert(start_symbol != cfg->symbol_num_perframe,
        "Sender: No valid symbols to transmit");
    ScheduleSymbol(0, start_symbol);

    while (keep_running) {
        gen_tag_t ctag(0); // The completion tag
        int ret = completion_queue_.try_dequeue(ctag._tag);
        if (ret == false) {
            continue;
        }

        const size_t comp_frame_slot = (ctag.frame_id % kFrameWnd);
        packet_count_per_symbol[comp_frame_slot][ctag.symbol_id]++;

        //std::printf("Sender -- checking symbol %d : %zu : %zu\n", ctag.symbol_id, 
		//comp_frame_slot, packet_count_per_symbol[comp_frame_slot][ctag.symbol_id]);
        //Check to see if the current symbol is finished
        if (packet_count_per_symbol[comp_frame_slot][ctag.symbol_id]
            == cfg->BS_ANT_NUM) {
            //Finished with the current symbol
            packet_count_per_symbol[comp_frame_slot][ctag.symbol_id] = 0;

            size_t next_symbol_id
                = FindNextSymbol(ctag.frame_id, (ctag.symbol_id + 1));
            unsigned symbol_delay = next_symbol_id - ctag.symbol_id;
            //std::printf("Sender -- finishing symbol %d : %zu : %zu delayed %d\n", 
			//ctag.symbol_id, cfg->symbol_num_perframe, next_symbol_id, symbol_delay);
            // Add inter-symbol delay
            delay_ticks(
                tick_start, get_ticks_for_frame(ctag.frame_id) * symbol_delay);
            tick_start = rdtsc();

            size_t next_frame_id = ctag.frame_id;
            //Check to see if the current frame is finished
            assert(next_symbol_id <= cfg->symbol_num_perframe);
            if (next_symbol_id == cfg->symbol_num_perframe) {
                if ((kDebugSenderReceiver == true)
                    || (kDebugPrintPerFrameDone == true)) {
                    std::printf("Sender: Transmitted frame %u in %.1f ms\n",
                        ctag.frame_id, (get_time() - start_time) / 1000.0);
                    start_time = get_time();
                }
                next_frame_id++;
                if (next_frame_id == cfg->frames_to_test) {
                    break; /* Finished */
                }
                frame_end[(ctag.frame_id % kNumStatsFrames)] = get_time();
                frame_start[(next_frame_id % kNumStatsFrames)] = get_time();
                tick_start = rdtsc();

                /* Find start symbol of next frame and add proper delay */
                next_symbol_id = FindNextSymbol(next_frame_id, 0);
                delay_ticks(tick_start,
                    get_ticks_for_frame(ctag.frame_id) * next_symbol_id);
                tick_start = rdtsc();
                //std::printf("Sender -- finished frame %d, next frame %zu, start 
				//symbol %zu, delaying\n", ctag.frame_id, next_frame_id, next_symbol_id,);
            }
            ScheduleSymbol(next_frame_id, next_symbol_id);
        }
    }
    write_stats_to_file(cfg->frames_to_test);
    std::exit(0);
    return nullptr;
}

/* Worker expects only valid transmit symbol_ids 'U' 'P' */
void* Sender::worker_thread(int tid)
{
    pin_to_core_with_offset(ThreadType::kWorkerTX, core_offset + 1, tid);

    // Wait for all Sender threads (including master) to start runnung
    num_workers_ready_atomic++;
    while (num_workers_ready_atomic != socket_thread_num) {
        // Wait
    }

    DFTI_DESCRIPTOR_HANDLE mkl_handle;
    DftiCreateDescriptor(
        &mkl_handle, DFTI_SINGLE, DFTI_COMPLEX, 1, cfg->OFDM_CA_NUM);
    DftiCommitDescriptor(mkl_handle);

    const size_t max_symbol_id = cfg->pilot_symbol_num_perframe
        + cfg->ul_data_symbol_num_perframe;
    const size_t radio_lo = tid * cfg->nRadios / socket_thread_num;
    const size_t radio_hi = (tid + 1) * cfg->nRadios / socket_thread_num;
    const size_t ant_num_this_thread = cfg->BS_ANT_NUM / socket_thread_num
        + ((size_t)tid < cfg->BS_ANT_NUM % socket_thread_num ? 1 : 0);
#ifdef USE_DPDK
    const size_t port_id = tid % cfg->dpdk_num_ports;
    const size_t queue_id = tid / cfg->dpdk_num_ports;
    rte_mbuf* tx_mbufs[kDequeueBulkSize];
#endif

    UDPClient udp_client;
    auto fft_inout = static_cast<complex_float*>(
        Agora_memory::padded_aligned_alloc(Agora_memory::Alignment_t::k64Align,
            cfg->OFDM_CA_NUM * sizeof(complex_float)));
    auto* socks_pkt_buf = static_cast<Packet*>(padded_aligned_alloc(
        Agora_memory::Alignment_t::k32Align, cfg->packet_length));

    double begin = get_time();
    size_t total_tx_packets = 0;
    size_t total_tx_packets_rolling = 0;
    size_t cur_radio = radio_lo;

    std::printf("In thread %zu, %zu antennas, BS_ANT_NUM: %zu\n", (size_t)tid,
        ant_num_this_thread, cfg->BS_ANT_NUM);

    // We currently don't support zero-padding OFDM prefix and postfix
    rt_assert(cfg->packet_length
        == Packet::kOffsetOfData
            + (kUse12BitIQ ? 3 : 4) * (cfg->CP_LEN + cfg->OFDM_CA_NUM));
    size_t ant_num_per_cell = cfg->BS_ANT_NUM / cfg->nCells;

    size_t tags[kDequeueBulkSize];
    while (true) {
        size_t num_tags = send_queue_.try_dequeue_bulk_from_producer(
            *(task_ptok[tid]), tags, kDequeueBulkSize);
        if (num_tags == 0) {
            continue;
        }

        for (size_t tag_id = 0; tag_id < num_tags; tag_id++) {
            size_t start_tsc_send = rdtsc();

            auto tag = gen_tag_t(tags[tag_id]);
            assert((cfg->get_symbol_type(tag.frame_id, tag.symbol_id)
                       == SymbolType::kPilot)
                || (cfg->get_symbol_type(tag.frame_id, tag.symbol_id)
                       == SymbolType::kUL));

            // Send a message to the server. We assume that the server is running.
            Packet* pkt = socks_pkt_buf;
#ifdef USE_DPDK
            tx_mbufs[tag_id] = DpdkTransport::alloc_udp(mbuf_pool,
                sender_mac_addr[port_id], server_mac_addr[port_id], bs_rru_addr,
                bs_server_addr, cfg->bs_rru_port + tid,
                cfg->bs_server_port + tid, cfg->packet_length);
            pkt = (Packet*)(rte_pktmbuf_mtod(tx_mbufs[tag_id], uint8_t*)
                + kPayloadOffset);
#endif

            // Update the TX buffer
            //std::printf("Sender : worker processing symbol %d, %d\n", 
			//tag.symbol_id, (int)symbol_type);
            pkt->frame_id = tag.frame_id;
            pkt->symbol_id = tag.symbol_id;
            pkt->cell_id = tag.ant_id / ant_num_per_cell;
            pkt->ant_id = tag.ant_id - ant_num_per_cell * (pkt->cell_id);
            std::memcpy(pkt->data,
                iq_data_short_[(pkt->symbol_id * cfg->BS_ANT_NUM) + tag.ant_id],
                (cfg->CP_LEN + cfg->OFDM_CA_NUM) * (kUse12BitIQ ? 3 : 4));
            if (cfg->fft_in_rru) {
                run_fft(pkt, fft_inout, mkl_handle);
            }

#ifndef USE_DPDK
            udp_client.send(cfg->bs_server_addr,
                cfg->bs_server_port + cur_radio,
                reinterpret_cast<uint8_t*>(socks_pkt_buf), cfg->packet_length);
#endif

            if (kDebugSenderReceiver == true) {
                std::printf(
                    "Thread %d (tag = %s) transmit frame %d, symbol %d, ant "
                    "%d, "
                    "TX time: %.3f us\n",
                    tid, gen_tag_t(tag).to_string().c_str(), pkt->frame_id,
                    pkt->symbol_id, pkt->ant_id,
                    cycles_to_us(rdtsc() - start_tsc_send, freq_ghz));
            }

            total_tx_packets_rolling++;
            total_tx_packets++;
            if (total_tx_packets_rolling
                == ant_num_this_thread * max_symbol_id * 1000) {
                double end = get_time();
                double byte_len = cfg->packet_length * ant_num_this_thread
                    * max_symbol_id * 1000.f;
                double diff = end - begin;
                std::printf(
                    "Thread %zu send %zu frames in %f secs, tput %f Mbps\n",
                    (size_t)tid,
                    total_tx_packets / (ant_num_this_thread * max_symbol_id),
                    diff / 1e6, byte_len * 8 * 1e6 / diff / 1024 / 1024);
                begin = get_time();
                total_tx_packets_rolling = 0;
            }

            if (++cur_radio == radio_hi) {
                cur_radio = radio_lo;
            }
        }

#ifdef USE_DPDK
        size_t nb_tx_new
            = rte_eth_tx_burst(port_id, queue_id, tx_mbufs, num_tags);
        if (unlikely(nb_tx_new != num_tags)) {
            std::printf("Thread %d rte_eth_tx_burst() failed, nb_tx_new: %zu, "
                        "num_tags: %zu\n",
                tid, nb_tx_new, num_tags);
            keep_running = 0;
            break;
        }
#endif
        rt_assert(completion_queue_.enqueue_bulk(tags, num_tags),
            "Completion enqueue failed");
    }
    return nullptr;
}

uint64_t Sender::get_ticks_for_frame(size_t frame_id)
{
    if (enable_slow_start == 0)
        return ticks_all;
    else if (frame_id < kFrameWnd)
        return ticks_wnd_1;
    else if (frame_id < kFrameWnd * 4)
        return ticks_wnd_2;
    else
        return ticks_all;
}

void Sender::init_iq_from_file(std::string filename)
{
    const size_t packets_per_frame = cfg->symbol_num_perframe * cfg->BS_ANT_NUM;
    iq_data_short_.calloc(packets_per_frame,
        (cfg->CP_LEN + cfg->OFDM_CA_NUM) * 2,
        Agora_memory::Alignment_t::k64Align);

    Table<float> iq_data_float;
    iq_data_float.calloc(packets_per_frame,
        (cfg->CP_LEN + cfg->OFDM_CA_NUM) * 2,
        Agora_memory::Alignment_t::k64Align);

    FILE* fp = fopen(filename.c_str(), "rb");
    rt_assert(fp != nullptr, "Failed to open IQ data file");

    for (size_t i = 0; i < packets_per_frame; i++) {
        const size_t expected_count = (cfg->CP_LEN + cfg->OFDM_CA_NUM) * 2;
        const size_t actual_count
            = fread(iq_data_float[i], sizeof(float), expected_count, fp);
        if (expected_count != actual_count) {
            std::fprintf(stderr,
                "Sender: Failed to read IQ data file %s. Packet %zu: expected "
                "%zu I/Q samples but read %zu. Errno %s\n",
                filename.c_str(), i, expected_count, actual_count,
                strerror(errno));
            std::exit(-1);
        }
        if (kUse12BitIQ) {
            // Adapt 32-bit IQ samples to 24-bit to reduce network throughput
            convert_float_to_12bit_iq(iq_data_float[i],
                reinterpret_cast<uint8_t*>(iq_data_short_[i]), expected_count);
        } else {
            for (size_t j = 0; j < expected_count; j++) {
                iq_data_short_[i][j]
                    = static_cast<unsigned short>(iq_data_float[i][j] * 32768);
            }
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
    std::printf(
        "Printing sender results to file \"%s\"...\n", filename.c_str());
    FILE* fp_debug = fopen(filename.c_str(), "w");
    rt_assert(fp_debug != nullptr, "Failed to open stats file");
    for (size_t i = 0; i < tx_frame_count; i++) {
        std::fprintf(fp_debug, "%.5f\n", frame_end[i % kNumStatsFrames]);
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
