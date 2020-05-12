/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#ifndef SENDER
#define SENDER

#include <arpa/inet.h>
#include <iostream>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <vector>
// #include <ctime>
#include <algorithm>
#include <boost/align/aligned_allocator.hpp>
#include <chrono>
#include <emmintrin.h>
#include <immintrin.h>
#include <numeric>
#include <pthread.h>
#include <signal.h>
#include <thread>
#include <time.h>
#include <unistd.h>

#include "Symbols.hpp"
#include "concurrentqueue.h"
#include "config.hpp"
#include "gettime.h"
#include "memory_manage.h"
#include "net.hpp"
#include "utils.h"

class Sender {
public:
    static constexpr size_t kTXBufOffset = kUseDPDK ? 22 : 0;
    static constexpr size_t kMaxNumSockets = 128; // Max network sockets

public:
    Sender(Config* config, size_t thread_num, size_t core_offset = 30,
        size_t delay = 0);
    ~Sender();

    void startTX();

    // in_frame_start and in_frame_end must have space for at least
    // kNumStatsFrames entries
    void startTXfromMain(double* in_frame_start, double* in_frame_end);

private:
    void* master_thread(int tid);
    void* worker_thread(int tid);
    void init_IQ_from_file();
    size_t get_max_symbol_id() const;
    /* Launch threads to run worker with thread IDs tid_start to tid_end - 1 */
    void create_threads(void* (*worker)(void*), int tid_start, int tid_end);
    void delay_for_symbol(size_t tx_frame_count, uint64_t tick_start);
    void delay_for_frame(size_t tx_frame_count, uint64_t tick_start);
    void update_tx_buffer(gen_tag_t tag);
    void write_stats_to_file(size_t tx_frame_count) const;

    // Return the TX buffer for a tag. The tag must have frame, symbol, and
    // antenna fields set
    inline size_t tag_to_tx_buffers_index(gen_tag_t tag) const;

    Config* cfg;
    const double freq_ghz; // RDTSC frequency in GHz
    const double ticks_per_usec; // RDTSC frequency in GHz
    const size_t thread_num; // Number of worker threads sending packets
    const size_t socket_num; // Total network sockets across worker threads

    // The master thread runs on core core_offset. Worker threads use cores
    // {core_offset + 1, ..., core_offset + thread_num - 1}
    const size_t core_offset;
    const size_t delay;
    const uint64_t ticks_all;
    const uint64_t ticks_5;
    const uint64_t ticks_100;
    const uint64_t ticks_200;
    const uint64_t ticks_500;

    pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
    pthread_cond_t cond = PTHREAD_COND_INITIALIZER;

    sockaddr_in servaddr_ipv4[kMaxNumSockets]; // Server address for IPv4
    sockaddr_in cliaddr_ipv4; // Client address for IPv4
    sockaddr_in6 servaddr_ipv6[kMaxNumSockets]; // Server address for IPv6
    sockaddr_in6 cliaddr_ipv6; // Client address for IPv6
    int socket_[kMaxNumSockets]; // Network sockets

    // First dimension:
    //   SOCKET_BUFFER_FRAME_NUM * symbol_num_perframe * BS_ANT_NUM
    // Second dimension: buffer_length (real and imag)
    Table<char> tx_buffers_;
    pthread_mutex_t lock_;

    moodycamel::ConcurrentQueue<size_t> send_queue_
        = moodycamel::ConcurrentQueue<size_t>(1024);
    moodycamel::ConcurrentQueue<size_t> completion_queue_
        = moodycamel::ConcurrentQueue<size_t>(1024);
    moodycamel::ProducerToken** task_ptok;

    size_t ant_id_;
    size_t frame_id_;
    size_t symbol_id_;

    // First dimension: symbol_num_perframe * BS_ANT_NUM
    // Second dimension: OFDM_FRAME_LEN * 2 (real and imag)
    Table<float> IQ_data;
    Table<ushort> IQ_data_coded;

    // Number of packets transmitted for each symbol in a frame
    size_t* packet_count_per_symbol[SOCKET_BUFFER_FRAME_NUM];
    size_t packet_count_per_frame[SOCKET_BUFFER_FRAME_NUM];

    double* frame_start;
    double* frame_end;
};

#endif
