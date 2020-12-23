#include "sender.hpp"
#include "datatype_conversion.h"
#include "udp_client.h"
#include <thread>

static std::atomic<bool> keep_running = true;

static constexpr size_t kMacAddrBtyes = 17;

// A spinning barrier to synchronize the start of worker threads
std::atomic<size_t> num_workers_ready_atomic;

void interrupt_handler(int)
{
    std::cout << "Will exit..." << std::endl;
    keep_running.store( false );
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
    , ticks_all(frame_duration_ * ticks_per_usec / cfg->frame().NumTotalSyms())
    , ticks_wnd_1(
          200000 /* 200 ms */ * ticks_per_usec / cfg->frame().NumTotalSyms())
    , ticks_wnd_2(
          15 * frame_duration_ * ticks_per_usec / cfg->frame().NumTotalSyms())
{
    std::printf(
        "Initializing sender, sending to base station server at %s, frame "
        "duration = %.2f ms, slow start = %s\n",
        cfg->bs_server_addr.c_str(), frame_duration / 1000.0,
        enable_slow_start == 1 ? "yes" : "no");

    _unused(server_mac_addr_str);
    for (size_t i = 0; i < kFrameWnd; i++) {
        packet_count_per_symbol[i] = new size_t[get_max_symbol_id()]();
    }
    std::memset(packet_count_per_frame, 0, kFrameWnd * sizeof(size_t));

    init_iq_from_file(std::string(TOSTRING(PROJECT_DIRECTORY))
        + "/data/LDPC_rx_data_2048_ant" + std::to_string(cfg->bs_ant_num())
        + ".bin");

    task_ptok = static_cast<moodycamel::ProducerToken**>(
        Agora_memory::padded_aligned_alloc(Agora_memory::Alignment_t::k64Align,
            (socket_thread_num * sizeof(moodycamel::ProducerToken*))));
    for (size_t i = 0; i < socket_thread_num; i++) {
        task_ptok[i] = new moodycamel::ProducerToken(send_queue_);
    }

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

Sender::~Sender( void )
{
    iq_data_short_.free();
    for (size_t i = 0; i < kFrameWnd; i++) {
        std::free(packet_count_per_symbol[i]);
    }

    for (size_t i = 0; i < socket_thread_num; i++) {
        delete (task_ptok[i]);
    }
    std::free(task_ptok);
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

void* Sender::master_thread(int)
{
    signal(SIGINT, interrupt_handler);
    pin_to_core_with_offset(ThreadType::kMasterTX, core_offset, 0);

    // Wait for all worker threads to be ready
    while (num_workers_ready_atomic != socket_thread_num) {
        // Wait
    }

    const size_t max_symbol_id = get_max_symbol_id();

    // Push tasks of the first symbol into task queue
    for (size_t i = 0; i < cfg->bs_ant_num(); i++) {
        auto req_tag = gen_tag_t::frm_sym_ant(0, 0, i);
        rt_assert(send_queue_.enqueue(
                      *task_ptok[i % socket_thread_num], req_tag._tag),
            "Send task enqueue failed");
    }

    frame_start[0] = get_time();
    double start_time = get_time();
    uint64_t tick_start = rdtsc();
    // Add delay for beacon at the beginning of a frame
    if (cfg->frame().NumBeaconSyms() == 1) {
        delay_ticks(tick_start, get_ticks_for_frame(0));
        tick_start = rdtsc();
    }
    while (keep_running.load() == true) {
        gen_tag_t ctag(0); // The completion tag
        int ret = completion_queue_.try_dequeue(ctag._tag);
        if (!ret)
            continue;

        const size_t comp_frame_slot = ctag.frame_id % kFrameWnd;

        packet_count_per_symbol[comp_frame_slot][ctag.symbol_id]++;
        if (packet_count_per_symbol[comp_frame_slot][ctag.symbol_id]
            == cfg->bs_ant_num()) {

            packet_count_per_symbol[comp_frame_slot][ctag.symbol_id] = 0;
            packet_count_per_frame[comp_frame_slot]++;

            // Add inter-symbol delay
            delay_ticks(tick_start, get_ticks_for_frame(ctag.frame_id));

            tick_start = rdtsc();

            const size_t next_symbol_id = (ctag.symbol_id + 1) % max_symbol_id;
            size_t next_frame_id;
            if (packet_count_per_frame[comp_frame_slot] == max_symbol_id) {
                // Add end-of-frame delay
                if (cfg->frame().NumDLSyms() > 0) {
                    delay_ticks(tick_start,
                        get_ticks_for_frame(ctag.frame_id)
                            * cfg->frame().NumDataSyms());
                }
                if (kDebugSenderReceiver || kDebugPrintPerFrameDone) {
                    std::printf("Sender: Transmitted frame %u in %.1f ms\n",
                        ctag.frame_id, (get_time() - start_time) / 1000.0);
                    start_time = get_time();
                }
                next_frame_id = ctag.frame_id + 1;
                if (next_frame_id == cfg->frames_to_test)
                    break;
                frame_end[ctag.frame_id % kNumStatsFrames] = get_time();
                packet_count_per_frame[comp_frame_slot] = 0;

                frame_start[next_frame_id % kNumStatsFrames] = get_time();

                tick_start = rdtsc();
                // Add delay for beacon at the beginning of a frame
                if (cfg->frame().NumBeaconSyms() == 1) {
                    delay_ticks(tick_start, get_ticks_for_frame(next_frame_id));
                    tick_start = rdtsc();
                }
            } else {
                next_frame_id = ctag.frame_id;
            }

            for (size_t i = 0; i < cfg->bs_ant_num(); i++) {
                auto req_tag
                    = gen_tag_t::frm_sym_ant(next_frame_id, next_symbol_id, i);
                rt_assert(send_queue_.enqueue(
                              *task_ptok[i % socket_thread_num], req_tag._tag),
                    "Send task enqueue failed");
            }
        }
    }
    write_stats_to_file(cfg->frames_to_test);
    std::exit(0);
}

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
        &mkl_handle, DFTI_SINGLE, DFTI_COMPLEX, 1, cfg->ofdm_ca_num());
    DftiCommitDescriptor(mkl_handle);

    const size_t max_symbol_id = get_max_symbol_id();
    const size_t radio_lo = tid * cfg->nRadios / socket_thread_num;
    const size_t radio_hi = (tid + 1) * cfg->nRadios / socket_thread_num;
    const size_t ant_num_this_thread = cfg->bs_ant_num() / socket_thread_num
        + ((size_t)tid < cfg->bs_ant_num() % socket_thread_num ? 1 : 0);
#ifdef USE_DPDK
    const size_t port_id = tid % cfg->dpdk_num_ports;
    const size_t queue_id = tid / cfg->dpdk_num_ports;
    rte_mbuf* tx_mbufs[kDequeueBulkSize];
#endif

    UDPClient udp_client;
    auto fft_inout = static_cast<complex_float*>(
        Agora_memory::padded_aligned_alloc(Agora_memory::Alignment_t::k64Align,
            cfg->ofdm_ca_num() * sizeof(complex_float)));
    auto* socks_pkt_buf = static_cast<Packet*>(padded_aligned_alloc(
        Agora_memory::Alignment_t::k32Align, cfg->packet_length));

    double begin = get_time();
    size_t total_tx_packets = 0;
    size_t total_tx_packets_rolling = 0;
    size_t cur_radio = radio_lo;

    std::printf("In thread %zu, %zu antennas, bs_ant_num(): %zu\n", (size_t)tid,
        ant_num_this_thread, cfg->bs_ant_num());

    // We currently don't support zero-padding OFDM prefix and postfix
    rt_assert(cfg->packet_length
        == Packet::kOffsetOfData
            + (kUse12BitIQ ? 3 : 4) * (cfg->cp_len() + cfg->ofdm_ca_num()));
    size_t ant_num_per_cell = cfg->bs_ant_num() / cfg->nCells;

    size_t tags[kDequeueBulkSize];
    while (true) {
        size_t num_tags = send_queue_.try_dequeue_bulk_from_producer(
            *(task_ptok[tid]), tags, kDequeueBulkSize);
        if (num_tags == 0)
            continue;

        for (size_t tag_id = 0; tag_id < num_tags; tag_id++) {
            size_t start_tsc_send = rdtsc();

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
            auto tag = gen_tag_t(tags[tag_id]);
            pkt->frame_id = tag.frame_id;
            pkt->symbol_id = cfg->GetSymbolId(tag.symbol_id);
            pkt->cell_id = tag.ant_id / ant_num_per_cell;
            pkt->ant_id = tag.ant_id - ant_num_per_cell * (pkt->cell_id);
            std::memcpy(pkt->data,
                iq_data_short_[(pkt->symbol_id * cfg->bs_ant_num()) + tag.ant_id],
                (cfg->cp_len() + cfg->ofdm_ca_num()) * (kUse12BitIQ ? 3 : 4));
            if (cfg->fft_in_rru) {
                run_fft(pkt, fft_inout, mkl_handle);
            }

#ifndef USE_DPDK
            udp_client.send(cfg->bs_server_addr,
                cfg->bs_server_port + cur_radio,
                reinterpret_cast<uint8_t*>(socks_pkt_buf), cfg->packet_length);
#endif

            if (kDebugSenderReceiver) {
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

            if (++cur_radio == radio_hi)
                cur_radio = radio_lo;
        }

#ifdef USE_DPDK
        size_t nb_tx_new
            = rte_eth_tx_burst(port_id, queue_id, tx_mbufs, num_tags);
        if (unlikely(nb_tx_new != num_tags)) {
            std::printf("Thread %d rte_eth_tx_burst() failed, nb_tx_new: %zu, "
                        "num_tags: %zu\n",
                tid, nb_tx_new, num_tags);
            keep_running.store(false);
            break;
        }
#endif
        rt_assert(completion_queue_.enqueue_bulk(tags, num_tags),
            "Completion enqueue failed");
    }

    std::free( static_cast<void *>(socks_pkt_buf) );
    std::free( static_cast<void *>(fft_inout) );
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

size_t Sender::get_max_symbol_id() const
{
    size_t max_symbol_id = (cfg->frame().NumDLSyms() > 0)
        ? cfg->frame().NumPilotSyms()
        : cfg->frame().NumPilotSyms() + cfg->frame().NumULSyms();
    return max_symbol_id;
}

void Sender::init_iq_from_file(std::string filename)
{
    const size_t packets_per_frame = cfg->frame().NumTotalSyms() * cfg->bs_ant_num();
    iq_data_short_.calloc(packets_per_frame,
        (cfg->cp_len() + cfg->ofdm_ca_num()) * 2,
        Agora_memory::Alignment_t::k64Align);

    Table<float> iq_data_float;
    iq_data_float.calloc(packets_per_frame,
        (cfg->cp_len() + cfg->ofdm_ca_num()) * 2,
        Agora_memory::Alignment_t::k64Align);

    FILE* fp = std::fopen(filename.c_str(), "rb");
    rt_assert(fp != nullptr, "Failed to open IQ data file");

    for (size_t i = 0; i < packets_per_frame; i++) {
        const size_t expected_count = (cfg->cp_len() + cfg->ofdm_ca_num()) * 2;
        const size_t actual_count
            = std::fread(iq_data_float[i], sizeof(float), expected_count, fp);
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
    std::fclose(fp);
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
    FILE* fp_debug = std::fopen(filename.c_str(), "w");
    rt_assert(fp_debug != nullptr, "Failed to open stats file");
    for (size_t i = 0; i < tx_frame_count; i++) {
        std::fprintf(fp_debug, "%.5f\n", frame_end[i % kNumStatsFrames]);
    }
}

void Sender::run_fft(Packet* pkt, complex_float* fft_inout,
    DFTI_DESCRIPTOR_HANDLE mkl_handle) const
{
    // pkt->data has (cp_len() + ofdm_ca_num()) unsigned short samples. After FFT,
    // we'll remove the cyclic prefix and have ofdm_ca_num() short samples left.
    simd_convert_short_to_float(&pkt->data[2 * cfg->cp_len()],
        reinterpret_cast<float*>(fft_inout), cfg->ofdm_ca_num() * 2);

    DftiComputeForward(mkl_handle, reinterpret_cast<float*>(fft_inout));

    simd_convert_float32_to_float16(reinterpret_cast<float*>(pkt->data),
        reinterpret_cast<float*>(fft_inout), cfg->ofdm_ca_num() * 2);
}
