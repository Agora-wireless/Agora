#ifndef SENDER
#define SENDER

#include "Symbols.hpp"
#include "concurrentqueue.h"
#include "config.hpp"
#include "gettime.h"
#include "memory_manage.h"
#include "mkl_dfti.h"
#include "net.hpp"
#include "utils.h"
#include <algorithm>
#include <arpa/inet.h>
#include <boost/align/aligned_allocator.hpp>
#include <chrono>
#include <emmintrin.h>
#include <immintrin.h>
#include <iostream>
#include <malloc.h>
#include <numeric>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <thread>
#include <time.h>
#include <unistd.h>
#include <vector>

#ifdef USE_DPDK
#include "dpdk_transport.hpp"
#include <netinet/ether.h>
#endif

class Sender {

public:
    /**
     * @brief Create and optionally start a Sender that sends IQ packets to a
     * server with MAC address [server_mac_addr_str]
     *
     * @param config The Agora config @param num_worker_threads Number of
     * worker threads sending packets 
     *
     * @param core_offset The master thread runs on core [core_offset]. Worker
     * thread #i runs on core [core_offset + i]
     *
     * @param frame_duration The TTI slot duration
     *
     * @param enable_slow_start If 1, the sender initially sends frames in a
     * duration larger than the TTI
     *
     * @param server_mac_addr_str The MAC address of the server's NIC
     */
    Sender(Config* config, size_t num_worker_threads, size_t core_offset = 30,
        size_t frame_duration = 1000, size_t enable_slow_start = 1,
        std::string server_mac_addr_str = "ff:ff:ff:ff:ff:ff",
        bool create_thread_for_master = false);

    ~Sender();

    void startTX();

    // in_frame_start and in_frame_end must have space for at least
    // kNumStatsFrames entries
    void startTXfromMain(double* in_frame_start, double* in_frame_end);

private:
    void* master_thread(int tid);
    void* worker_thread(int tid);

    /**
     * @brief Read time-domain 32-bit floating-point IQ samples from [filename]
     * and populate iq_data_short_ by converting to 16-bit fixed-point samples
     *
     * [filename] must contain data for one frame. For every symbol and antenna,
     * the file must provide (CP_LEN + OFDM_CA_NUM) IQ samples.
     */
    void init_iq_from_file(std::string filename);

    // Get number of CPU ticks for a symbol given a frame index
    uint64_t get_ticks_for_frame(size_t frame_id);
    size_t get_max_symbol_id() const;

    // Launch threads to run worker with thread IDs from tid_start to tid_end
    void create_threads(void* (*worker)(void*), int tid_start, int tid_end);

    void delay_for_symbol(size_t tx_frame_count, uint64_t tick_start);
    void delay_for_frame(size_t tx_frame_count, uint64_t tick_start);

    void write_stats_to_file(size_t tx_frame_count) const;

    // Run FFT on the data field in pkt, output to fft_inout
    // Recombine pkt header data and fft output data into payload
    void run_fft(Packet* pkt, complex_float* fft_inout,
        DFTI_DESCRIPTOR_HANDLE mkl_handle) const;

    Config* cfg;
    const double freq_ghz; // RDTSC frequency in GHz
    const double ticks_per_usec; // RDTSC frequency in GHz
    const size_t num_worker_threads_; // Number of worker threads sending pkts
    const size_t enable_slow_start; // If 1, send frames slowly at first

    // The master thread runs on core core_offset. Worker threads use cores
    // {core_offset + 1, ..., core_offset + thread_num - 1}
    const size_t core_offset;
    const size_t frame_duration_;

    // RDTSC clock ticks between the start of transmission of two symbols in
    // the steady state
    const uint64_t ticks_all;

    // ticks_wnd_1 and ticks_wnd_2 are the RDTSC clock ticks between the start
    // of transmission of two symbols for the first several frames
    const uint64_t ticks_wnd_1;
    const uint64_t ticks_wnd_2;

    moodycamel::ConcurrentQueue<size_t> send_queue_
        = moodycamel::ConcurrentQueue<size_t>(1024);
    moodycamel::ConcurrentQueue<size_t> completion_queue_
        = moodycamel::ConcurrentQueue<size_t>(1024);
    moodycamel::ProducerToken** task_ptok;

    // First dimension: symbol_num_perframe * BS_ANT_NUM
    // Second dimension: (CP_LEN + OFDM_CA_NUM) * 2
    Table<unsigned short> iq_data_short_;

    // Number of packets transmitted for each symbol in a frame
    size_t* packet_count_per_symbol[kFrameWnd];
    size_t packet_count_per_frame[kFrameWnd];

    double* frame_start;
    double* frame_end;

#ifdef USE_DPDK
    struct rte_mempool* mbuf_pool;
    uint32_t bs_rru_addr; // IPv4 address of this data sender
    uint32_t bs_server_addr; // IPv4 address of the remote target Agora server
    rte_ether_addr sender_mac_addr; // MAC address of this data sender

    // MAC address of the remote target Agora server
    rte_ether_addr server_mac_addr;
#endif
};

#endif
