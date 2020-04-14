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

#define CPU_FREQ 2.3e9

typedef unsigned short ushort;

class Sender {
public:
#ifdef USE_DPDK
    static const size_t kTXBufOffset = 22;
#else
    static const size_t kTXBufOffset = 0;
#endif

public:
    Sender(Config* in_config, size_t in_thread_num, size_t in_core_offset = 30,
        size_t in_delay = 0);
    ~Sender();

    void startTX();
    void startTXfromMain(double* in_frame_start, double* in_frame_end);
    void* loopMain(int tid);
    void* loopSend(int tid);
    int dequeue_send(int tid, int radio_id);
    void init_IQ_from_file();
    size_t get_max_symbol_id();
    /* Launch threads to run worker with thread IDs tid_start to tid_end - 1 */
    void create_threads(void* (*worker)(void*), int tid_start, int tid_end);
    void update_ids(size_t max_ant_id, size_t max_symbol_id);
    void delay_for_symbol(size_t tx_frame_count, uint64_t tick_start);
    void delay_for_frame(size_t tx_frame_count, uint64_t tick_start);
    void preload_tx_buffer();
    void update_tx_buffer(size_t data_ptr);
    void write_stats_to_file(size_t tx_frame_count);

private:
    Config* cfg;

    pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
    pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
#if USE_IPV4
    struct sockaddr_in* servaddr_; /* server address */
    struct sockaddr_in cliaddr_; /* server address */
#else
    struct sockaddr_in6* servaddr_; /* server address */
    struct sockaddr_in6 cliaddr_; /* server address */
#endif
    int* socket_;
    // First dimension: BUFFER_FRAME_NUM * symbol_num_perframe * BS_ANT_NUM
    // Second dimension: buffer_length (real and imag)
    Table<char> tx_buffer_;
    size_t buffer_len_;
    pthread_mutex_t lock_;

    moodycamel::ConcurrentQueue<size_t> task_queue_
        = moodycamel::ConcurrentQueue<size_t>(1024);
    moodycamel::ConcurrentQueue<size_t> message_queue_
        = moodycamel::ConcurrentQueue<size_t>(1024);
    moodycamel::ProducerToken** task_ptok;

    size_t ant_id;
    size_t frame_id;
    size_t symbol_id;

    // First dimension: symbol_num_perframe * BS_ANT_NUM
    // Second dimension: OFDM_FRAME_LEN * 2 (real and imag)
    Table<float> IQ_data;
    Table<ushort> IQ_data_coded;

    size_t thread_num;
    size_t socket_num;

    size_t core_offset;
    size_t delay;

    Table<size_t> packet_count_per_symbol;
    size_t* packet_count_per_frame;

    double* frame_start;
    double* frame_end;

    uint64_t ticks_5 = (uint64_t)500000 * CPU_FREQ / 1e6 / 70;
    uint64_t ticks_100 = (uint64_t)150000 * CPU_FREQ / 1e6 / 70;
    uint64_t ticks_200 = (uint64_t)20000 * CPU_FREQ / 1e6 / 70;
    uint64_t ticks_500 = (uint64_t)10000 * CPU_FREQ / 1e6 / 70;
    uint64_t ticks_all;
};

#endif
