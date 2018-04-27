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
#include "Symbols.hpp"
#include <vector>
#include <ctime>
#include <algorithm>
#include <numeric>
#include <unistd.h>
#include <chrono>
#include <pthread.h>
#include "concurrentqueue.h"

typedef unsigned short ushort;

class PackageSender
{
public:
    static const int OFDM_FRAME_LEN = OFDM_CA_NUM + OFDM_PREFIX_LEN;
    // int for: frame_id, subframe_id, cell_id, ant_id
    // unsigned int for: I/Q samples
    static const int buffer_length = sizeof(int) * 4 + sizeof(ushort) * OFDM_FRAME_LEN * 2;
    static const int data_offset = sizeof(int) * 4;
    static const int subframe_num_perframe = 40;
    static const int BUFFER_FRAME_NUM = 40;



    struct PackageSenderContext
    {
        PackageSender *ptr;
        int tid;
    };

public:
    PackageSender(int in_socket_num, int in_thread_num, int in_core_offset = 30);
    ~PackageSender();

    static void* loopSend(void *context);
    
private:
    struct sockaddr_in servaddr_;    /* server address */
    struct sockaddr_in cliaddr_;    /* server address */
    int* socket_;

    std::vector<std::vector<char>> trans_buffer_;
    int cur_ptr_;
    int buffer_len_;
    pthread_mutex_t lock_;

    moodycamel::ConcurrentQueue<int> task_queue_ = moodycamel::ConcurrentQueue<int>( BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM);
    int max_length_ = BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM;

    int ant_id;
    int frame_id;
    int subframe_id;

    float** IQ_data;
    ushort** IQ_data_coded;

    int thread_num;
    int socket_num;

    int core_offset;
    PackageSenderContext* context;

};


#endif