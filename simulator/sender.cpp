#include "sender.hpp"
#include "datatype_conversion.h"
#include "logger.h"
#include "udp_client.h"
#include <thread>

bool keep_running = true;

// A spinning barrier to synchronize the start of worker threads
std::atomic<size_t> num_workers_ready_atomic;
std::atomic<size_t> num_masters_ready_atomic;

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

Sender::Sender(Config* cfg, size_t num_master_threads_, size_t num_worker_threads_, 
    size_t core_offset, size_t frame_duration, size_t enable_slow_start,
    std::string server_mac_addr_str, bool create_thread_for_master,
    void* mbuf_pool)
    : cfg(cfg)
    , freq_ghz(measure_rdtsc_freq())
    , ticks_per_usec(freq_ghz * 1e3)
    , num_master_threads_(num_master_threads_)
    , num_worker_threads_(num_worker_threads_)
    , enable_slow_start(enable_slow_start)
    , core_offset(core_offset)
    , frame_duration_(frame_duration)
    , ticks_all(frame_duration_ * ticks_per_usec / cfg->symbol_num_perframe)
    , ticks_wnd_1(
          200000 /* 200 ms */ * ticks_per_usec / cfg->symbol_num_perframe)
    , ticks_wnd_2(
          15 * frame_duration_ * ticks_per_usec / cfg->symbol_num_perframe)
{
    printf("Initializing sender, sending to base station server at %s, frame "
           "duration = %.2f ms, slow start = %s\n",
        cfg->bs_server_addr.c_str(), frame_duration / 1000.0,
        enable_slow_start == 1 ? "yes" : "no");

    running_ = true;

    _unused(server_mac_addr_str);
    for (size_t i = 0; i < SOCKET_BUFFER_FRAME_NUM; i++) {
        packet_count_per_symbol[i] = new size_t[get_max_symbol_id()]();
    }
    memset(packet_count_per_frame, 0, SOCKET_BUFFER_FRAME_NUM * sizeof(size_t));

    init_iq_from_file(std::string(TOSTRING(PROJECT_DIRECTORY))
        + "/data/LDPC_rx_data_2048_ant" + std::to_string(cfg->BS_ANT_NUM)
        + ".bin");

    send_queues_ = (moodycamel::ConcurrentQueue<size_t>**)aligned_alloc(
        64, num_worker_threads_ * sizeof(moodycamel::ConcurrentQueue<size_t>*));
    for (size_t i = 0; i < num_worker_threads_; i ++) {
        send_queues_[i] = new moodycamel::ConcurrentQueue<size_t>(8192);
    }

    // task_ptok = (moodycamel::ProducerToken**)aligned_alloc(
    //     64, num_worker_threads_ * sizeof(moodycamel::ProducerToken*));
    // for (size_t i = 0; i < num_worker_threads_; i++)
    //     task_ptok[i] = new moodycamel::ProducerToken(send_queue_);

    completion_queues_ = (moodycamel::ConcurrentQueue<size_t>**)aligned_alloc(
        64, num_master_threads_ * sizeof(moodycamel::ConcurrentQueue<size_t>*));
    for (size_t i = 0; i < num_master_threads_; i ++) {
        completion_queues_[i] = new moodycamel::ConcurrentQueue<size_t>(8192);
    }

    // comp_ptok = (moodycamel::ProducerToken**)aligned_alloc(
    //     64, num_master_threads_ * sizeof(moodycamel::ProducerToken*));
    // for (size_t i = 0; i < num_master_threads_; i ++) {
    //     comp_ptok[i] = new moodycamel::ProducerToken(completion_queue_);
    // }

    num_workers_ready_atomic = 0;
    num_masters_ready_atomic = 0;

    // Create a master thread when started from simulator
    // if (create_thread_for_master)
    //     create_threads(pthread_fun_wrapper<Sender, &Sender::master_thread>,
    //         num_worker_threads_, num_worker_threads_ + 1);
    // if (create_thread_for_master) {
    //     master_thread_ = std::thread(&Sender::master_thread, this, num_worker_threads_);
    // }

#ifdef USE_DPDK
    printf("Start to init DPDK states\n");
    uint16_t portid = 0; // For now, hard-code to port zero
    if (mbuf_pool == nullptr) {
        DpdkTransport::dpdk_init(core_offset, num_worker_threads_);
        this->mbuf_pool = DpdkTransport::create_mempool();
        
        int res;
        if ((res = DpdkTransport::nic_init(portid, this->mbuf_pool, num_worker_threads_)) != 0) {
            printf("Cannot init port %u: error %d\n", portid, res);
            rte_exit(EXIT_FAILURE, "Cannot init port %u\n", portid);
        }
    } else {
        this->mbuf_pool = reinterpret_cast<rte_mempool*>(mbuf_pool);
    }

    // Parse IP addresses and MAC addresses
    int ret = inet_pton(AF_INET, cfg->bs_rru_addr.c_str(), &bs_rru_addr);
    rt_assert(ret == 1, "Invalid sender IP address");
    ret = inet_pton(AF_INET, cfg->bs_server_addr.c_str(), &bs_server_addr);
    rt_assert(ret == 1, "Invalid server IP address");

    bs_server_addr_list.resize(cfg->bs_server_addr_list.size());
    for (size_t i = 0; i < cfg->bs_server_addr_list.size(); i ++) {
        ret = inet_pton(AF_INET, cfg->bs_server_addr_list[i].c_str(), &bs_server_addr_list[i]);
    }

    server_mac_addr_list.resize(cfg->bs_server_addr_list.size());
    for (size_t i = 0; i < cfg->bs_server_addr_list.size(); i ++) {
        ether_addr* parsed_mac = ether_aton(cfg->bs_server_mac_list[i].c_str());
        rt_assert(parsed_mac != NULL, "Invalid server mac address");
        memcpy(&server_mac_addr_list[i], parsed_mac, sizeof(ether_addr));
    }

    ret = rte_eth_macaddr_get(portid, &sender_mac_addr);
    rt_assert(ret == 0, "Cannot get MAC address of the port");
    printf("Number of DPDK cores: %d\n", rte_lcore_count());
#endif
    printf("Sender init done\n");
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

    // create_threads(pthread_fun_wrapper<Sender, &Sender::worker_thread>, 0,
    //     num_worker_threads_);
    // master_threads_[0] = std::thread(&Sender::master_thread, this, 0);
    for (size_t i = 0; i < num_master_threads_; i ++) {
        master_threads_[i] = std::thread(&Sender::master_thread, this, i);
    }
    for (size_t i = 0; i < num_worker_threads_; i ++) {
        worker_threads_[i] = std::thread(&Sender::worker_thread, this, i);
    }
}

void Sender::startTXfromMainAuto(double* in_frame_start, double* in_frame_end)
{
    frame_start = in_frame_start;
    frame_end = in_frame_end;

    for (size_t i = 0; i < num_worker_threads_; i ++) {
        worker_threads_[i] = std::thread(&Sender::worker_thread_auto, this, i);
    }
}

void Sender::join_thread() {
    // master_threads_[0].join();
    for (size_t i = 0; i < num_master_threads_; i ++) {
        master_threads_[i].join();
    }
    for (size_t i = 0; i < num_worker_threads_; i ++) {
        worker_threads_[i].join();
    }
}

void Sender::join_thread_auto() {
    for (size_t i = 0; i < num_worker_threads_; i ++) {
        worker_threads_[i].join();
    }
}

void* Sender::master_thread(int tid)
{
    size_t* packet_count_per_symbol_loc[SOCKET_BUFFER_FRAME_NUM];
    size_t packet_count_per_frame_loc[SOCKET_BUFFER_FRAME_NUM];
    for (size_t i = 0; i < SOCKET_BUFFER_FRAME_NUM; i ++) {
        packet_count_per_symbol_loc[i] = new size_t[get_max_symbol_id()]();
        memset(packet_count_per_symbol_loc[i], 0, get_max_symbol_id() * sizeof(size_t));
    }
    memset(packet_count_per_frame_loc, 0, SOCKET_BUFFER_FRAME_NUM * sizeof(size_t));
    double* frame_start_loc = new double[kNumStatsFrames]();
    double* frame_end_loc = new double[kNumStatsFrames]();

    signal(SIGINT, interrupt_handler);
    pin_to_core_with_offset(ThreadType::kMasterTX, core_offset, tid);

    // Wait for all master threads to be ready
    num_masters_ready_atomic ++;
    while (num_masters_ready_atomic != num_master_threads_) {
        // Wait
    }

    // Wait for all worker threads to be ready
    while (num_workers_ready_atomic != num_worker_threads_) {
        // Wait
    }

    size_t ant_block_size = cfg->BS_ANT_NUM / num_master_threads_;
    size_t ant_block_off = cfg->BS_ANT_NUM % num_master_threads_;
    size_t antenna_start = tid < ant_block_off ? (ant_block_size + 1) * tid : ant_block_size * tid + ant_block_off;
    size_t antenna_end = tid < ant_block_off ? antenna_start + ant_block_size + 1 : antenna_start + ant_block_size;
    if (tid < ant_block_off) {
        ant_block_size ++;
    }

    const size_t max_symbol_id = get_max_symbol_id();

    // Push tasks of the first symbol into task queue
    for (size_t i = antenna_start; i < antenna_end; i++) {
        auto req_tag = gen_tag_t::frm_sym_ant(0, 0, i);
        // rt_assert(send_queue_.enqueue(
        //               *task_ptok[i % num_worker_threads_], req_tag._tag),
        //     "Send task enqueue failed");
        rt_assert(send_queues_[i % num_worker_threads_]->enqueue(req_tag._tag),
            "Send task enqueue failed");
    }

    frame_start_loc[0] = get_time();
    uint64_t tick_start = rdtsc();
    double start_time = get_time();
    // Add delay for beacon at the beginning of a frame
    delay_ticks(tick_start,
        enable_slow_start == 1 ? get_ticks_for_frame(0) : ticks_all);
    tick_start = rdtsc();
    while (keep_running) {
        gen_tag_t ctag(0); // The completion tag
        // int ret = completion_queue_.try_dequeue_from_producer(*(comp_ptok[tid]), ctag._tag);
        int ret = completion_queues_[tid]->try_dequeue(ctag._tag);
        if (!ret)
            continue;

        // printf("Receive a comp tag %d\n", ctag.ant_id);
        const size_t comp_frame_slot = ctag.frame_id % SOCKET_BUFFER_FRAME_NUM;

        // packet_count_mutex_.lock();
        // packet_count_per_symbol[comp_frame_slot][ctag.symbol_id]++;
        // if (packet_count_per_symbol[comp_frame_slot][ctag.symbol_id]
        //     == cfg->BS_ANT_NUM) {
        packet_count_per_symbol_loc[comp_frame_slot][ctag.symbol_id]++;
        if (packet_count_per_symbol_loc[comp_frame_slot][ctag.symbol_id]
            == ant_block_size) {
            // packet_count_mutex_.unlock();

            packet_count_per_symbol_loc[comp_frame_slot][ctag.symbol_id] = 0;
            packet_count_per_frame_loc[comp_frame_slot]++;

            // Add inter-symbol delay
            delay_ticks(tick_start,
                enable_slow_start == 1 ? get_ticks_for_frame(ctag.frame_id)
                                       : ticks_all);

            tick_start = rdtsc();

            const size_t next_symbol_id = (ctag.symbol_id + 1) % max_symbol_id;
            size_t next_frame_id;
            if (packet_count_per_frame_loc[comp_frame_slot] == max_symbol_id) {
                if (kDebugSenderReceiver || kDebugPrintPerFrameDone) {
                    printf("Sender (master %u): Transmitted frame %u in %.1f ms\n",
                        tid, ctag.frame_id, (get_time() - start_time) / 1000.0);
                    start_time = get_time();
                }
                next_frame_id = ctag.frame_id + 1;
                if (next_frame_id == cfg->frames_to_test)
                    break;
                frame_end_loc[ctag.frame_id % kNumStatsFrames] = get_time();
                packet_count_per_frame_loc[comp_frame_slot] = 0;

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

                frame_start_loc[next_frame_id % kNumStatsFrames] = get_time();

                tick_start = rdtsc();
                // Add delay for beacon at the beginning of a frame
                delay_ticks(tick_start,
                    enable_slow_start == 1 ? get_ticks_for_frame(next_frame_id)
                                           : ticks_all);
                tick_start = rdtsc();
            } else {
                next_frame_id = ctag.frame_id;
            }

            for (size_t i = antenna_start; i < antenna_end; i++) {
                auto req_tag
                    = gen_tag_t::frm_sym_ant(next_frame_id, next_symbol_id, i);
                // rt_assert(
                //     send_queue_.enqueue(
                //         *task_ptok[i % num_worker_threads_], req_tag._tag),
                //     "Send task enqueue failed");
                rt_assert(
                    send_queues_[i % num_worker_threads_]->enqueue(req_tag._tag),
                    "Send task enqueue failed");
            }
        } else {
            // packet_count_mutex_.unlock();
        }
    }
    keep_running = false;
    running_ = false;
    write_stats_to_file(cfg->frames_to_test);
    printf("Master thread ends!\n");
    // exit(0);
    return NULL;
}

void* Sender::worker_thread(int tid)
{
    pin_to_core_with_offset(ThreadType::kWorkerTX, core_offset + num_master_threads_, tid);

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
    auto* socks_pkt_buf
        = reinterpret_cast<Packet*>(memalign(64, cfg->packet_length));
    auto* data_buf
        = reinterpret_cast<Packet*>(memalign(64, cfg->packet_length));

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
        // if (!send_queue_.try_dequeue_from_producer(*(task_ptok[tid]), tag._tag)) {
        if (!send_queues_[tid]->try_dequeue(tag._tag)) {
            if (!keep_running) {
                break;
            }
            continue;
        }

        size_t start_tsc_send = rdtsc();

        // Send a message to the server. We assume that the server is running.
        Packet* pkt = socks_pkt_buf;
#ifdef USE_DPDK
        // rte_mbuf* tx_mbuf = DpdkTransport::alloc_udp(mbuf_pool, sender_mac_addr,
        //     server_mac_addr, bs_rru_addr, bs_server_addr,
        //     cfg->bs_rru_port + tid, cfg->bs_server_port + tid,
        //     cfg->packet_length);
        // pkt = (Packet*)(rte_pktmbuf_mtod(tx_mbuf, uint8_t*) + kPayloadOffset);
        
        rte_mbuf** tx_mbufs = new rte_mbuf*[cfg->bs_server_addr_list.size()];
        for (size_t i = 0; i < cfg->bs_server_addr_list.size(); i ++) {
            tx_mbufs[i] = DpdkTransport::alloc_udp(mbuf_pool, sender_mac_addr,
                server_mac_addr_list[i], bs_rru_addr, bs_server_addr_list[i],
                cfg->bs_rru_port + tid, cfg->bs_server_port + tid,
                cfg->packet_length);
        }
#endif

        // Update the TX buffer
        pkt->pkt_type = Packet::PktType::kIQFromRRU;
        pkt->frame_id = tag.frame_id;
        pkt->symbol_id = cfg->getSymbolId(tag.symbol_id);
        pkt->cell_id = 0;
        pkt->ant_id = tag.ant_id;

        memcpy(data_buf->data,
            iq_data_short_[(tag.symbol_id * cfg->BS_ANT_NUM) + tag.ant_id],
            (cfg->CP_LEN + cfg->OFDM_CA_NUM) * sizeof(unsigned short) * 2);
        run_fft(data_buf->data, fft_inout, mkl_handle);

#ifdef USE_DPDK
        const size_t sc_block_size = cfg->get_num_sc_per_server();
        for (size_t i = 0; i < cfg->bs_server_addr_list.size(); i++) {
            pkt = (Packet*)(rte_pktmbuf_mtod(tx_mbufs[i], uint8_t*) + kPayloadOffset);
            pkt->pkt_type = Packet::PktType::kIQFromRRU;
            pkt->frame_id = tag.frame_id;
            pkt->symbol_id = cfg->getSymbolId(tag.symbol_id);
            pkt->cell_id = 0;
            pkt->ant_id = tag.ant_id;
            memcpy(pkt->data,
                data_buf->data
                    + (i * sc_block_size + cfg->OFDM_DATA_START) * 2,
                sc_block_size * sizeof(unsigned short) * 2);
            MLPD_TRACE("Sender: Sending packet %s (%zu of %zu) to %s:%ld\n",
                pkt->to_string().c_str(), i, cfg->bs_server_addr_list.size(),
                cfg->bs_server_addr_list[i].c_str(),
                cfg->bs_server_port + cur_radio);
        }
        // rt_assert(rte_eth_tx_burst(0, tid, tx_mbufs, cfg->bs_server_addr_list.size()) == cfg->bs_server_addr_list.size(),
        //     "rte_eth_tx_burst() failed");
        size_t pkt_sent = 0;
        size_t loop_count = 0;
        while (pkt_sent < cfg->bs_server_addr_list.size()) {
            size_t pkt_sent_cur = rte_eth_tx_burst(0, tid, tx_mbufs + pkt_sent, cfg->bs_server_addr_list.size() - pkt_sent);
            pkt_sent += pkt_sent_cur;
            rt_assert(loop_count < 10000, "rte_eth_tx_burst() failed");
            loop_count ++;
        }
#else
        const size_t sc_block_size
            = cfg->OFDM_DATA_NUM / cfg->bs_server_addr_list.size();
        for (size_t i = 0; i < cfg->bs_server_addr_list.size(); i++) {
            memcpy(pkt->data,
                data_buf->data
                    + (i * sc_block_size + cfg->OFDM_DATA_START) * 2,
                sc_block_size * sizeof(unsigned short) * 2);
            MLPD_TRACE("Sender: Sending packet %s (%zu of %zu) to %s:%ld\n",
                pkt->to_string().c_str(), i, cfg->bs_server_addr_list.size(),
                cfg->bs_server_addr_list[i].c_str(),
                cfg->bs_server_port + cur_radio);

            udp_client.send(cfg->bs_server_addr_list[i],
                cfg->bs_server_port + cur_radio,
                reinterpret_cast<uint8_t*>(socks_pkt_buf),
                cfg->packet_length);
        }
#endif

        if (kDebugSenderReceiver) {
            printf("Thread %d (tag = %s) transmit frame %d, symbol %d, ant %d, "
                   "TX time: %.3f us\n",
                tid, gen_tag_t(tag).to_string().c_str(), pkt->frame_id,
                pkt->symbol_id, pkt->ant_id,
                cycles_to_us(rdtsc() - start_tsc_send, freq_ghz));
        }

        // rt_assert(
        //     completion_queue_.enqueue(*comp_ptok[tag.ant_id % num_master_threads_], tag._tag), "Completion enqueue failed");
        rt_assert(
            completion_queues_[tag.ant_id % num_master_threads_]->enqueue(tag._tag), "Completion enqueue failed");

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
    printf("Worker thread ends\n");
    return NULL;
}

void* Sender::worker_thread_auto(int tid)
{
    pin_to_core_with_offset(ThreadType::kWorkerTX, core_offset, tid);

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
    const size_t radio_block_size = cfg->nRadios / num_worker_threads_;
    const size_t radio_block_off = cfg->nRadios % num_worker_threads_;
    const size_t radio_lo = tid < radio_block_off ? tid * (radio_block_size + 1) : tid * radio_block_size + radio_block_off;
    const size_t radio_hi = tid < radio_block_off ? radio_lo + radio_block_size + 1 : radio_lo + radio_block_size;
    const size_t ant_num_this_thread = cfg->BS_ANT_NUM / num_worker_threads_
        + ((size_t)tid < cfg->BS_ANT_NUM % num_worker_threads_ ? 1 : 0);

    auto fft_inout = reinterpret_cast<complex_float*>(
        memalign(64, cfg->OFDM_CA_NUM * sizeof(complex_float)));
    auto* socks_pkt_buf
        = reinterpret_cast<Packet*>(memalign(64, cfg->packet_length));
    auto* data_buf
        = reinterpret_cast<Packet*>(memalign(64, cfg->packet_length));

    double begin = get_time();
    size_t cur_radio = radio_lo;
    size_t cur_frame = 0;
    size_t cur_symbol = 0;

    printf("In thread %zu, %zu antennas, BS_ANT_NUM: %zu, num threads %zu:\n",
        (size_t)tid, ant_num_this_thread, cfg->BS_ANT_NUM, num_worker_threads_);

    // We currently don't support zero-padding OFDM prefix and postfix
    rt_assert(cfg->packet_length
        == Packet::kOffsetOfData
            + 2 * sizeof(unsigned short) * (cfg->CP_LEN + cfg->OFDM_CA_NUM));

    double start_time = get_time();

    size_t start_tsc_send = rdtsc();

#ifdef USE_DPDK
    rte_mbuf** tx_mbufs = new rte_mbuf*[cfg->bs_server_addr_list.size()];
    rte_eth_stats tx_stats;
#endif

    while (true) {
        gen_tag_t tag = 0;

#ifdef USE_DPDK
        for (size_t i = 0; i < cfg->bs_server_addr_list.size(); i ++) {
            tx_mbufs[i] = DpdkTransport::alloc_udp(mbuf_pool, sender_mac_addr,
                server_mac_addr_list[i], bs_rru_addr, bs_server_addr_list[i],
                cfg->bs_rru_port + tid, cfg->bs_server_port + tid,
                Packet::kOffsetOfData + cfg->get_num_sc_per_server() * sizeof(unsigned short) * 2);
        }
#endif

        memcpy(data_buf->data,
            iq_data_short_[(cur_symbol * cfg->BS_ANT_NUM) + cur_radio],
            (cfg->CP_LEN + cfg->OFDM_CA_NUM) * sizeof(unsigned short) * 2);
        run_fft(data_buf->data, fft_inout, mkl_handle);

#ifdef USE_DPDK
        const size_t sc_block_size = cfg->get_num_sc_per_server();
        for (size_t i = 0; i < cfg->bs_server_addr_list.size(); i++) {
            auto* pkt = (Packet*)(rte_pktmbuf_mtod(tx_mbufs[i], uint8_t*) + kPayloadOffset);
            pkt->pkt_type = Packet::PktType::kIQFromRRU;
            pkt->frame_id = cur_frame;
            pkt->symbol_id = cfg->getSymbolId(cur_symbol);
            pkt->cell_id = 0;
            pkt->ant_id = cur_radio;
            memcpy(pkt->data,
                data_buf->data
                    + (i * sc_block_size + cfg->OFDM_DATA_START) * 2,
                sc_block_size * sizeof(unsigned short) * 2);
            MLPD_TRACE("Sender: Sending packet %s (%zu of %zu) to %s:%ld\n",
                pkt->to_string().c_str(), i, cfg->bs_server_addr_list.size(),
                cfg->bs_server_addr_list[i].c_str(),
                cfg->bs_server_port + tid);
        }
        // rt_assert(rte_eth_tx_burst(0, tid, tx_mbufs, cfg->bs_server_addr_list.size()) == cfg->bs_server_addr_list.size(),
        //     "rte_eth_tx_burst() failed");
        size_t pkt_sent = 0;
        size_t loop_count = 0;
        while (pkt_sent < cfg->bs_server_addr_list.size()) {
            size_t pkt_sent_cur = rte_eth_tx_burst(0, tid, tx_mbufs + pkt_sent, cfg->bs_server_addr_list.size() - pkt_sent);
            pkt_sent += pkt_sent_cur;
            rt_assert(loop_count < 10000, "rte_eth_tx_burst() failed");
            loop_count ++;
        }
#else
        #error Should not be compiled
#endif

        if (unlikely(++cur_radio == radio_hi)) {
            cur_radio = radio_lo;
            cur_symbol ++;
            if (unlikely(cur_symbol == max_symbol_id)) {
                cur_symbol = 0;
                cur_frame ++;
                if (unlikely(cur_frame == cfg->frames_to_test)) {
                    break;
                }
                delay_ticks(start_tsc_send, cur_frame * max_symbol_id * ticks_all);
                printf("Thread %u send frame %u in %.1f ms\n", tid, cur_frame - 1, (get_time() - start_time) / 1000.0f);
#ifdef USE_DPDK
                if (tid == 0) {
                    rte_eth_stats stats;
                    rte_eth_stats_get(0, &stats);
                    printf("Traffic rate is %lf\n", (double)(stats.obytes - tx_stats.obytes) * 8 / ((get_time() - start_time) * 1000.0f));
                    memcpy(&tx_stats, &stats, sizeof(rte_eth_stats));
                }
#endif
                start_time = get_time();
            }
        }
    }
    printf("Worker thread ends\n");
    return NULL;
}

uint64_t Sender::get_ticks_for_frame(size_t frame_id)
{
    if (frame_id < kFrameWnd)
        return ticks_wnd_1;
    else if (frame_id < kFrameWnd * 4)
        return ticks_wnd_2;
    else
        return ticks_all;
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
    printf("Filename: %s\n", filename.c_str());
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

void Sender::run_fft(short* src, complex_float* fft_inout,
    DFTI_DESCRIPTOR_HANDLE mkl_handle) const
{
    // pkt->data has (CP_LEN + OFDM_CA_NUM) unsigned short samples. After FFT,
    // we'll remove the cyclic prefix and have OFDM_CA_NUM short samples left.
    simd_convert_short_to_float(&src[2 * cfg->CP_LEN],
        reinterpret_cast<float*>(fft_inout), cfg->OFDM_CA_NUM * 2);

    DftiComputeForward(mkl_handle, reinterpret_cast<float*>(fft_inout));

    simd_convert_float32_to_float16(reinterpret_cast<float*>(src),
        reinterpret_cast<float*>(fft_inout), cfg->OFDM_CA_NUM * 2);
}