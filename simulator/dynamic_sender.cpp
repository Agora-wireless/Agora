#include "dynamic_sender.hpp"
#include "datatype_conversion.h"
#include "logger.h"
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
          4 * frame_duration_ * ticks_per_usec / cfg->symbol_num_perframe)
    , ticks_wnd_2(
          2 * frame_duration_ * ticks_per_usec / cfg->symbol_num_perframe)
{
    printf("Initializing sender, sending to base station server at %s, frame "
           "duration = %.2f ms, slow start = %s\n",
        cfg->bs_server_addr.c_str(), frame_duration / 1000.0,
        enable_slow_start == 1 ? "yes" : "no");

    running_ = true;
    fft_tsc_ = 0;
    fft_count_ = 0;

    _unused(server_mac_addr_str);
    for (size_t i = 0; i < SOCKET_BUFFER_FRAME_NUM; i++) {
        packet_count_per_symbol[i] = new size_t[get_max_symbol_id()]();
    }
    memset(packet_count_per_frame, 0, SOCKET_BUFFER_FRAME_NUM * sizeof(size_t));

    init_iq_from_file(std::string(TOSTRING(PROJECT_DIRECTORY))
        + "/data/LDPC_rx_data_" + std::to_string(cfg->OFDM_CA_NUM) + "_ant" + std::to_string(cfg->BS_ANT_NUM)
        + ".bin");
    init_control_from_file(std::string(TOSTRING(PROJECT_DIRECTORY))
        + "/data/control_ue.bin");

    num_workers_ready_atomic = 0;

    printf("Start to init DPDK states\n");
    uint16_t portid = 0; // For now, hard-code to port zero
    if (mbuf_pool == nullptr) {
        DpdkTransport::dpdk_init(core_offset, num_worker_threads_, cfg->pci_addr);
        // this->mbuf_pool = DpdkTransport::create_mempool();
        for (size_t i = 0; i < num_worker_threads_; i ++) {
            mbuf_pools_[i] = DpdkTransport::create_mempool(i);
        }
        
        int res;
        if ((res = DpdkTransport::nic_init(portid, this->mbuf_pools_[0], 1, num_worker_threads_)) != 0) {
            printf("Cannot init port %u: error %d\n", portid, res);
            rte_exit(EXIT_FAILURE, "Cannot init port %u\n", portid);
        }
    } else {
        this->mbuf_pools_[0] = reinterpret_cast<rte_mempool*>(mbuf_pool);
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

    printf("Sender init done\n");
}

Sender::~Sender()
{
    iq_data_short_.free();
    for (size_t i = 0; i < SOCKET_BUFFER_FRAME_NUM; i++) {
        free(packet_count_per_symbol[i]);
    }
}

void Sender::startTXfromMain(double* in_frame_start, double* in_frame_end)
{
    frame_start = in_frame_start;
    frame_end = in_frame_end;

    for (size_t i = 0; i < num_worker_threads_; i ++) {
        worker_threads_[i] = std::thread(&Sender::worker_thread, this, i);
    }
}

void Sender::join_thread() {
    for (size_t i = 0; i < num_worker_threads_; i ++) {
        worker_threads_[i].join();
    }
}

size_t Sender::get_sync_tsc(int tid) {
    start_tsc_mutex_.lock();
    num_invoked_threads_ ++;
    if (num_invoked_threads_ == num_worker_threads_) {
        start_sync_tsc_ = rdtsc();
    }
    start_tsc_mutex_.unlock();
    while (likely(num_invoked_threads_ != num_worker_threads_)) {
        usleep(1);
        // printf("Wait thread %u: %d %d\n", tid, num_invoked_threads_, num_worker_threads_);
    }
    return start_sync_tsc_;
}

void* Sender::worker_thread(int tid)
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
    // auto* data_buf
    //     = reinterpret_cast<Packet*>(memalign(64, cfg->packet_length));
    short *data_buf
        = reinterpret_cast<short*>(memalign(64, 2 * cfg->user_level_list.size() * cfg->num_load_levels * ant_num_this_thread * cfg->OFDM_CA_NUM * sizeof(short) * 2));

    // Prepare the data
    size_t count = 0;
    for (size_t i = 0; i < 2 * cfg->user_level_list.size() * cfg->num_load_levels; i ++) {
        for (size_t j = radio_lo; j < radio_hi; j ++) {
            memcpy(data_buf + (i * ant_num_this_thread + j - radio_lo) * (2 * cfg->OFDM_CA_NUM),
                iq_data_short_[(i * cfg->BS_ANT_NUM) + j],
                (cfg->CP_LEN + cfg->OFDM_CA_NUM) * sizeof(unsigned short) * 2);
            run_fft(data_buf + (i * ant_num_this_thread + j - radio_lo) * (2 * cfg->OFDM_CA_NUM), fft_inout, mkl_handle, false);
        }
    }

    double begin = get_time();
    size_t cur_radio = radio_lo;
    size_t cur_frame = 0;
    size_t cur_symbol = 0;
    size_t cur_slot_idx = control_idx_list_[0];

    printf("In thread %zu, %zu antennas, BS_ANT_NUM: %zu, num threads %zu:\n",
        (size_t)tid, ant_num_this_thread, cfg->BS_ANT_NUM, num_worker_threads_);

    // We currently don't support zero-padding OFDM prefix and postfix
    rt_assert(cfg->packet_length
        == Packet::kOffsetOfData
            + 2 * sizeof(unsigned short) * (cfg->CP_LEN + cfg->OFDM_CA_NUM));

    double start_time = get_time();

    // size_t start_tsc_send = rdtsc();
    size_t start_tsc_send = get_sync_tsc(tid);

    size_t loop_counter[10001] = {};

    rte_mbuf** tx_mbufs = new rte_mbuf*[cfg->bs_server_addr_list.size()];
    rte_eth_stats tx_stats;

    while (true) {
        gen_tag_t tag = 0;

        // for (size_t i = 0; i < cfg->bs_server_addr_list.size(); i ++) {
        //     // tx_mbufs[i] = DpdkTransport::alloc_udp(mbuf_pools_[tid], sender_mac_addr,
        //     //     server_mac_addr_list[i], bs_rru_addr, bs_server_addr_list[i],
        //     //     cfg->bs_rru_port + cur_radio, cfg->bs_server_port + cur_radio,
        //     //     Packet::kOffsetOfData + cfg->get_num_sc_per_server() * sizeof(unsigned short) * 2);
        //     tx_mbufs[i] = DpdkTransport::alloc_udp(mbuf_pools_[tid], sender_mac_addr,
        //         server_mac_addr_list[i], bs_rru_addr, bs_server_addr_list[i],
        //         cfg->bs_rru_port + cur_radio, cfg->bs_server_port + cur_radio,
        //         Packet::kOffsetOfData + cfg->subcarrier_num_list[i] * sizeof(unsigned short) * 2);
        // }

        // // const size_t sc_block_size = cfg->get_num_sc_per_server();
        // for (size_t i = 0; i < cfg->bs_server_addr_list.size(); i++) {
        //     auto* pkt = (Packet*)(rte_pktmbuf_mtod(tx_mbufs[i], uint8_t*) + kPayloadOffset);
        //     pkt->pkt_type = Packet::PktType::kIQFromRRU;
        //     pkt->frame_id = cur_frame;
        //     pkt->symbol_id = cfg->getSymbolId(cur_symbol);
        //     pkt->cell_id = 0;
        //     pkt->ant_id = cur_radio;
        //     size_t buf_offset = cur_slot_idx * 2 + (cur_symbol == 0 ? 0 : 1);
        //     // memcpy(pkt->data,
        //     //     data_buf + (buf_offset * ant_num_this_thread + cur_radio - radio_lo) * (2 * cfg->OFDM_CA_NUM)
        //     //         + (i * sc_block_size + cfg->OFDM_DATA_START) * 2,
        //     //     sc_block_size * sizeof(unsigned short) * 2);
        //     memcpy(pkt->data,
        //         data_buf + (buf_offset * ant_num_this_thread + cur_radio - radio_lo) * (2 * cfg->OFDM_CA_NUM)
        //             + (cfg->subcarrier_num_start[i] + cfg->OFDM_DATA_START) * 2,
        //         cfg->subcarrier_num_list[i] * sizeof(unsigned short) * 2);
        //     MLPD_TRACE("Sender: Sending packet %s (%zu of %zu) to %s:%ld\n",
        //         pkt->to_string().c_str(), i, cfg->bs_server_addr_list.size(),
        //         cfg->bs_server_addr_list[i].c_str(),
        //         cfg->bs_server_port + tid);
        // }
        // rt_assert(rte_eth_tx_burst(0, tid, tx_mbufs, cfg->bs_server_addr_list.size()) == cfg->bs_server_addr_list.size(),
        //     "rte_eth_tx_burst() failed");

        size_t ant_block_size = cfg->nRadios / cfg->bs_server_addr_list.size();
        size_t ant_block_off = cfg->nRadios % cfg->bs_server_addr_list.size();
        size_t server_id = cur_radio < ant_block_off * (ant_block_size + 1) ?
            cur_radio / (ant_block_size + 1) :
            ant_block_off + (cur_radio - ant_block_off * (ant_block_size + 1)) / ant_block_size;
        tx_mbufs[0] = DpdkTransport::alloc_udp(mbuf_pools_[tid], sender_mac_addr,
                server_mac_addr_list[server_id], bs_rru_addr, bs_server_addr_list[server_id],
                cfg->bs_rru_port + cur_radio, cfg->bs_server_port + cur_radio,
                Packet::kOffsetOfData + cfg->OFDM_DATA_NUM * sizeof(unsigned short) * 2);
        auto* pkt = (Packet*)(rte_pktmbuf_mtod(tx_mbufs[0], uint8_t*) + kPayloadOffset);
        pkt->pkt_type = Packet::PktType::kIQFromRRU;
        pkt->frame_id = cur_frame;
        pkt->symbol_id = cfg->getSymbolId(cur_symbol);
        pkt->cell_id = 0;
        pkt->ant_id = cur_radio;
        size_t buf_offset = cur_slot_idx * 2 + (cur_symbol == 0 ? 0 : 1);
        memcpy(pkt->data,
            data_buf + (buf_offset * ant_num_this_thread + cur_radio - radio_lo) * (2 * cfg->OFDM_CA_NUM)
                + cfg->OFDM_DATA_START * 2,
            cfg->OFDM_DATA_NUM * sizeof(unsigned short) * 2);
        rt_assert(rte_eth_tx_burst(0, tid, tx_mbufs, 1) == 1,
            "rte_eth_tx_burst() failed");

        if (unlikely(++cur_radio == radio_hi)) {
            cur_radio = radio_lo;
            cur_symbol ++;
            if (unlikely(cur_symbol == max_symbol_id)) {
                cur_symbol = 0;
                cur_frame ++;
                if (unlikely(cur_frame == cfg->frames_to_test)) {
                    break;
                }
                cur_slot_idx = control_idx_list_[cur_frame];
                // delay_ticks(start_tsc_send, cur_frame * max_symbol_id * ticks_all);
                printf("Thread %u send frame %u in %.1f ms\n", tid, cur_frame - 1, (get_time() - start_time) / 1000.0f);

                if (tid == 0) {
                    rte_eth_stats stats;
                    rte_eth_stats_get(0, &stats);
                    printf("Traffic rate is %lf, packet rate is %lf\n", (double)(stats.obytes - tx_stats.obytes) * 8 / ((get_time() - start_time) * 1000.0f),
                        (double)(stats.opackets - tx_stats.opackets) / ((get_time() - start_time)));
                    memcpy(&tx_stats, &stats, sizeof(rte_eth_stats));
                }

                start_time = get_time();
            }
            if (cur_frame < 80) {
                delay_ticks(start_tsc_send, (cur_frame * max_symbol_id + cur_symbol) * ticks_wnd_1);
            } else if (cur_frame < 200) {
                delay_ticks(start_tsc_send, (80 * max_symbol_id * ticks_wnd_1) + 
                    ((cur_frame - 80) * max_symbol_id + cur_symbol) * ticks_wnd_2);
            } else {
                delay_ticks(start_tsc_send, (80 * max_symbol_id * ticks_wnd_1) +
                    (120 * max_symbol_id * ticks_wnd_2) +
                    ((cur_frame - 200) * max_symbol_id + cur_symbol) * ticks_all);
            }
        }
    }
    // printf("Thread %u: Print loop counters:\n", tid);
    // for (size_t i = 1; i < 10000; i ++) {
    //     if (loop_counter[i] > 0) {
    //         printf("%u:%u ", i, loop_counter[i]);
    //     }
    // }
    // printf("\n");
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
    const size_t num_packet = 2 * cfg->user_level_list.size() * cfg->num_load_levels * cfg->BS_ANT_NUM;
    iq_data_short_.calloc(
        num_packet, (cfg->CP_LEN + cfg->OFDM_CA_NUM) * 2, 64);

    Table<float> iq_data_float;
    iq_data_float.calloc(
        num_packet, (cfg->CP_LEN + cfg->OFDM_CA_NUM) * 2, 64);

    FILE* fp = fopen(filename.c_str(), "rb");
    rt_assert(fp != nullptr, "Failed to open IQ data file");

    for (size_t i = 0; i < num_packet; i++) {
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

void Sender::init_control_from_file(std::string filename)
{
    FILE* fp = fopen(filename.c_str(), "rb");
    rt_assert(fp != nullptr, "Failed to open IQ data file");

    control_idx_list_.resize(cfg->frames_to_test);

    for (size_t i = 0; i < cfg->frames_to_test; i ++) {
        fread(&control_idx_list_[i], sizeof(size_t), 1, fp);
    }

    fclose(fp);
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
    DFTI_DESCRIPTOR_HANDLE mkl_handle, bool debug)
{
    // pkt->data has (CP_LEN + OFDM_CA_NUM) unsigned short samples. After FFT,
    // we'll remove the cyclic prefix and have OFDM_CA_NUM short samples left.
    simd_convert_short_to_float(&src[2 * cfg->CP_LEN],
        reinterpret_cast<float*>(fft_inout), cfg->OFDM_CA_NUM * 2);

    size_t start_tsc = rdtsc();
    DftiComputeForward(mkl_handle, reinterpret_cast<float*>(fft_inout));
    size_t end_tsc = rdtsc();
    if (debug) {
        fft_tsc_ += end_tsc - start_tsc;
        fft_count_ ++;
    }

    simd_convert_float32_to_float16(reinterpret_cast<float*>(src),
        reinterpret_cast<float*>(fft_inout), cfg->OFDM_CA_NUM * 2);
}