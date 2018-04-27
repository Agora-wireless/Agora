/**
 * Author: Peiyao Zhao
 * E-Mail: pdszpy19930218@163.com
 * 
 */

#ifndef PACKAGERECEIVER
#define PACKAGERECEIVER

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
#include <pthread.h>
#include <cassert>
#include <unistd.h>
#include <chrono>
#include "buffer.hpp"
#include "concurrentqueue.h"

typedef unsigned short ushort;
class PackageReceiver
{
public:
    
    static const int OFDM_FRAME_LEN = OFDM_CA_NUM + OFDM_PREFIX_LEN;
    // header 4 int for: frame_id, subframe_id, cell_id, ant_id
    // ushort for: I/Q samples
    static const int package_length = sizeof(int) * 4 + sizeof(ushort) * OFDM_FRAME_LEN * 2;
    static const int data_offset = sizeof(int) * 4;
    // use for create pthread 
    struct PackageReceiverContext
    {
        PackageReceiver *ptr;
        int tid;
    };

public:
    PackageReceiver(int N_THREAD = 1);
    /**
     * N_THREAD: socket thread number
     * in_queue: message queue to communicate with main thread
    */ 
    PackageReceiver(int N_THREAD, moodycamel::ConcurrentQueue<Event_data> * in_queue);
    ~PackageReceiver();
    
    /**
     * called in main threads to start the socket threads
     * in_buffer: ring buffer to save packets
     * in_buffer_status: record the status of each memory block (0: empty, 1: full)
     * in_buffer_frame_num: number of packets the ring buffer could hold
     * in_buffer_length: size of ring buffer
     * in_core_id: attach socket threads to {in_core_id, ..., in_core_id + N_THREAD - 1}
    */ 
    std::vector<pthread_t> startRecv(char** in_buffer, int** in_buffer_status, int in_buffer_frame_num, int in_buffer_length, int in_core_id=0);
    /**
     * receive thread
     * context: PackageReceiverContext type
    */
    static void* loopRecv(void *context);
 
private:
    struct sockaddr_in servaddr_;    /* server address */
    int* socket_;

    char** buffer_;
    int** buffer_status_;
    int buffer_length_;
    int buffer_frame_num_;

    int thread_num_;
    // pointer of message_queue_
    moodycamel::ConcurrentQueue<Event_data> *message_queue_;
    int core_id_;

    PackageReceiverContext* context;
};


#endif
