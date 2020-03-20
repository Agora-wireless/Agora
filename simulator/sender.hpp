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
    // static const int ofdm_frame_len = ofdm_ca_num + ofdm_prefix_len;
    // int for: frame_id, subframe_id, cell_id, ant_id
    // unsigned int for: I/Q samples

#ifdef USE_DPDK
    static const size_t tx_buf_offset = 22;
#else
    static const size_t tx_buf_offset = 0;
#endif

    // static const int buffer_length = tx_buf_offset + sizeof(int) * 16 +
    // sizeof(ushort) * ofdm_frame_len * 2; static const int data_offset =
    // sizeof(int) * 16;
    //    static const size_t subframe_num_perframe = 40;
    static const size_t BUFFER_FRAME_NUM = 40;

    // static const size_t max_subframe_id = ENABLE_DOWNLINK ? ue_num :
    // subframe_num_perframe;

public:
    Sender(Config* in_config, size_t in_thread_num, size_t in_core_offset = 30,
        size_t in_delay = 0);
    ~Sender();

    void startTX();
    void startTXfromMain(double* in_frame_start, double* in_frame_end);
    void* loopSend_main(int tid);
    void* loopSend(int tid);

private:
    Config* cfg;

    pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
    pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
#if USE_IPV4
    struct sockaddr_in servaddr_[10]; /* server address */
    struct sockaddr_in cliaddr_; /* server address */
#else
    struct sockaddr_in6 servaddr_[10]; /* server address */
    struct sockaddr_in6 cliaddr_; /* server address */
#endif
    int* socket_;
    // int* socket_tcp_;

    // First dimension: BUFFER_FRAME_NUM * subframe_num_perframe * bs_ant_num
    // Second dimension: buffer_length (real and imag)
    // std::vector<std::vector<char,boost::alignment::aligned_allocator<char,
    // 64>>> trans_buffer_;
    Table<char> trans_buffer_;
    size_t cur_ptr_;
    size_t buffer_len_;
    pthread_mutex_t lock_;

    moodycamel::ConcurrentQueue<size_t> task_queue_
        = moodycamel::ConcurrentQueue<size_t>(1024);
    moodycamel::ConcurrentQueue<size_t> message_queue_
        = moodycamel::ConcurrentQueue<size_t>(1024);
    moodycamel::ProducerToken** task_ptok;

    size_t ant_id;
    size_t frame_id;
    size_t subframe_id;

    // First dimension: subframe_num_perframe * bs_ant_num
    // Second dimension: ofdm_frame_len * 2 (real and imag)
    Table<float> IQ_data;
    Table<ushort> IQ_data_coded;

    size_t thread_num;
    size_t socket_num;

    size_t core_offset;
    size_t delay;

    Table<size_t> packet_count_per_subframe;
    size_t* packet_count_per_frame;

    double* frame_start;
    double* frame_end;
};

#endif
