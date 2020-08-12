/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
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
   * @param config The Millipede config
   * @param num_worker_threads Number of worker threads sending packets 
   * @param core_offset
   * @param delay
   * @param enable_slow_start
   * @param server_mac_addr_str
   */
    Sender(Config* config, size_t num_worker_threads, size_t core_offset = 30,
        size_t delay = 0, bool enable_slow_start = true,
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

    /// Read 32-bit floating-point IQ samples from filename and populate
    /// iq_data_short_ by converting to 16-bit fixed-point samples
    void init_iq_from_file(std::string filename);

    size_t get_max_symbol_id() const;
    /* Launch threads to run worker with thread IDs tid_start to tid_end - 1 */
    void create_threads(void* (*worker)(void*), int tid_start, int tid_end);
    void delay_for_symbol(size_t tx_frame_count, uint64_t tick_start);
    void delay_for_frame(size_t tx_frame_count, uint64_t tick_start);

    void write_stats_to_file(size_t tx_frame_count) const;

    // Return the TX buffer index for this frame, symbol index, and antenna ID
    inline size_t get_idx_in_tx_buffers(
        size_t frame_id, size_t symbol_idx, size_t ant_id) const;

    // Run FFT on the data field in pkt, output to fft_inout
    // Recombine pkt header data and fft output data into payload
    void run_fft(const Packet* pkt, complex_float* fft_inout,
        DFTI_DESCRIPTOR_HANDLE mkl_handle, uint8_t* payload) const;

    Config* cfg;
    const double freq_ghz; // RDTSC frequency in GHz
    const double ticks_per_usec; // RDTSC frequency in GHz
    const size_t num_worker_threads_; // Number of worker threads sending pkts
    const bool enable_slow_start; // Send frames slowly at first

    // The master thread runs on core core_offset. Worker threads use cores
    // {core_offset + 1, ..., core_offset + thread_num - 1}
    const size_t core_offset;
    const size_t delay;
    const uint64_t ticks_all;
    const uint64_t ticks_5;
    const uint64_t ticks_100;
    const uint64_t ticks_200;
    const uint64_t ticks_500;

    // First dimension:
    //   SOCKET_BUFFER_FRAME_NUM * symbol_num_perframe * BS_ANT_NUM
    // Second dimension: buffer_length (real and imag)
    Table<char> tx_buffers_;

    moodycamel::ConcurrentQueue<size_t> send_queue_
        = moodycamel::ConcurrentQueue<size_t>(1024);
    moodycamel::ConcurrentQueue<size_t> completion_queue_
        = moodycamel::ConcurrentQueue<size_t>(1024);
    moodycamel::ConcurrentQueue<size_t> data_update_queue_
        = moodycamel::ConcurrentQueue<size_t>(1024);
    moodycamel::ProducerToken** task_ptok;

    // First dimension: symbol_num_perframe * BS_ANT_NUM
    // Second dimension: OFDM_FRAME_LEN * 2 (real and imag)
    Table<unsigned short> iq_data_short_;

    // Number of packets transmitted for each symbol in a frame
    size_t* packet_count_per_symbol[SOCKET_BUFFER_FRAME_NUM];
    size_t packet_count_per_frame[SOCKET_BUFFER_FRAME_NUM];

    double* frame_start;
    double* frame_end;

#ifdef USE_DPDK
    struct rte_mempool* mbuf_pool;
    uint32_t sender_addr; // IPv4 address of this data sender
    uint32_t server_addr; // IPv4 address of the remote target Millipede server
    rte_ether_addr sender_mac_addr; // MAC address of this data sender

    // MAC address of the remote target Millipede server
    rte_ether_addr server_mac_addr;
#endif
};

#endif
