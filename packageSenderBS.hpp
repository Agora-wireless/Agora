#ifndef PACKAGESENDERBS
#define PACKAGESENDERBS

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
#include "buffer.hpp"
#include "concurrentqueue.h"

typedef unsigned short ushort;

class packageSenderBS
{
public:
    static const int OFDM_FRAME_LEN = OFDM_CA_NUM + OFDM_PREFIX_LEN;
    // int for: frame_id, subframe_id, cell_id, ant_id
    // unsigned int for: I/Q samples
    static const int package_length = sizeof(int) * 4 + sizeof(ushort) * OFDM_FRAME_LEN * 2;
    static const int data_offset = sizeof(int) * 4;
    static const int subframe_num_perframe = 40;
    static const int BUFFER_FRAME_NUM = 40;



    struct PackageSenderContext
    {
        packageSenderBS *ptr;
        int tid;
    };

public:
    packageSenderBS(int N_THREAD = 1);

    packageSenderBS(int N_THREAD, moodycamel::ConcurrentQueue<Event_data> * in_queue_message, moodycamel::ConcurrentQueue<Event_data> * in_queue_task);
    ~packageSenderBS();

    static void* loopSend(void *context);

    std::vector<pthread_t> startTX(char* in_buffer, int* in_buffer_status, int in_buffer_frame_num, int in_buffer_length, int in_core_id=0);
    
private:
    struct sockaddr_in servaddr_;    /* server address */
    struct sockaddr_in cliaddr_;    /* server address */
    int* socket_;

    // First dimension: BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM
    // Second dimension: buffer_length (real and imag)
    // std::vector<std::vector<char>> trans_buffer_;
    // int cur_ptr_;
    // int buffer_len_;
    // pthread_mutex_t lock_;

    // moodycamel::ConcurrentQueue<int> task_queue_ = moodycamel::ConcurrentQueue<int>( BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM);
    // int max_length_ = BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM;

    // int ant_id;
    // int frame_id;
    // int subframe_id;


    char* buffer_;
    int* buffer_status_;
    int buffer_length_;
    int buffer_frame_num_;

    int thread_num_;
    // pointer to message_queue_
    moodycamel::ConcurrentQueue<Event_data> *message_queue_;
    moodycamel::ConcurrentQueue<Event_data> *task_queue_;
    int core_id_;


    PackageSenderContext* context;

};


#endif