/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 * 
 */
#ifndef PACKAGESENDER
#define PACKAGESENDER

#include <iostream>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>  /* for fprintf */
#include <string.h> /* for memcpy */
#include <stdlib.h>
#include <vector>
// #include <ctime>
#include <time.h>
#include <algorithm>
#include <numeric>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <pthread.h>
#include <signal.h>
#include <emmintrin.h>
#include <immintrin.h>
#include <boost/align/aligned_allocator.hpp>

#include "gettime.h"
#include "Symbols.hpp"
#include "concurrentqueue.h"
#include "config.hpp"
#include "memory_manage.h"


#define CPU_FREQ 2.3e9

typedef unsigned short ushort;

class PackageSender
{
public:
    // static const int OFDM_FRAME_LEN = OFDM_CA_NUM + OFDM_PREFIX_LEN;
    // int for: frame_id, subframe_id, cell_id, ant_id
    // unsigned int for: I/Q samples
#if USE_DPDK
    static const int tx_buf_offset = 22;
#else
    static const int tx_buf_offset = 0;
#endif
    // static const int buffer_length = tx_buf_offset + sizeof(int) * 16 + sizeof(ushort) * OFDM_FRAME_LEN * 2;
    // static const int data_offset = sizeof(int) * 16;
//    static const int subframe_num_perframe = 40;
    static const int BUFFER_FRAME_NUM = 40;

    // static const int max_subframe_id = ENABLE_DOWNLINK ? UE_NUM : subframe_num_perframe;



    struct PackageSenderContext
    {
        PackageSender *ptr;
        int tid;
    };

public:
    PackageSender(Config *in_config, int in_socket_num, int in_thread_num, int in_core_offset = 30, int in_delay = 0);
    ~PackageSender();

    static void* loopSend(void *context);
    
private:
    Config *config_;
    int BS_ANT_NUM, UE_NUM;
    int OFDM_FRAME_LEN;
    int subframe_num_perframe, data_subframe_num_perframe;
    int package_length;
    int package_header_offset;
    int buffer_length;
    int max_subframe_id;
    int max_length_;
    bool downlink_mode;

    pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
    pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
#if USE_IPV4
    struct sockaddr_in servaddr_[10];    /* server address */
    struct sockaddr_in cliaddr_;    /* server address */
#else
    struct sockaddr_in6 servaddr_[10];    /* server address */
    struct sockaddr_in6 cliaddr_;    /* server address */
#endif
    int* socket_;
    // int* socket_tcp_;

    // First dimension: BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM
    // Second dimension: buffer_length (real and imag)
    // std::vector<std::vector<char,boost::alignment::aligned_allocator<char, 64>>> trans_buffer_;
    char **trans_buffer_;
    int cur_ptr_;
    int buffer_len_;
    pthread_mutex_t lock_;

    // moodycamel::ConcurrentQueue<int> task_queue_ = moodycamel::ConcurrentQueue<int>( BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM);
    // moodycamel::ConcurrentQueue<int> message_queue_ = moodycamel::ConcurrentQueue<int>( BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM);
    moodycamel::ConcurrentQueue<int> task_queue_ = moodycamel::ConcurrentQueue<int>( 1024);
    moodycamel::ConcurrentQueue<int> message_queue_ = moodycamel::ConcurrentQueue<int>( 1024);
    std::unique_ptr<moodycamel::ProducerToken> task_ptok[10]; 
    // int max_length_ = BUFFER_FRAME_NUM * max_subframe_id * BS_ANT_NUM;
    // int max_length_ = max_subframe_id * BS_ANT_NUM;

    int ant_id;
    int frame_id;
    int subframe_id;

    // First dimension: subframe_num_perframe * BS_ANT_NUM
    // Second dimension: OFDM_FRAME_LEN * 2 (real and imag)
    float **IQ_data;
    ushort **IQ_data_coded;

    int thread_num;
    int socket_num;

    int core_offset;
    int delay;
    PackageSenderContext* context;

    int **packet_count_per_subframe;
    int *packet_count_per_frame;
    // int packet_count_per_subframe[BUFFER_FRAME_NUM][max_subframe_id];
    // int packet_count_per_frame[BUFFER_FRAME_NUM];
};


#endif
