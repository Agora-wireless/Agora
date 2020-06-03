/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */

#ifndef MACPACKETTXRX
#define MACPACKETTXRX

#include "Symbols.hpp"
#include "buffer.hpp"
#include "concurrentqueue.h"
#include "gettime.h"
#include "net.hpp"
#include <algorithm>
#include <arpa/inet.h>
#include <cassert>
#include <chrono>
#include <ctime>
#include <fcntl.h>
#include <iostream>
#include <mutex>
#include <netinet/in.h>
#include <numeric>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
// #include <unistd.h>
#include "config.hpp"
#include <vector>

typedef unsigned short ushort;
class MacPacketTXRX {
public:
    MacPacketTXRX(Config* cfg, size_t core_offset = 1);
    /**
     * queue_message: message queue to communicate with main thread
     */
    MacPacketTXRX(Config* cfg, size_t core_offset,
        moodycamel::ConcurrentQueue<Event_data>* queue_message,
        moodycamel::ConcurrentQueue<Event_data>* in_queue_task,
        moodycamel::ProducerToken** in_rx_ptoks,
        moodycamel::ProducerToken** in_tx_ptoks);
    ~MacPacketTXRX();

    /**
     * called in main threads to start the socket threads
     * buffer: ring buffer to save packets
     * buffer_status: record the status of each memory block (0: empty, 1:
     * full) 
     * packet_num_in_buffer : number of packets the ring buffer could hold
     * core_offset: attach socket threads to {core_offset, ..., core_offset +
     * socket_thread_num - 1}
     */
    bool startTXRX(Table<char>& buffer, Table<int>& buffer_status,
        size_t packet_num_in_buffer, Table<size_t>& frame_start,
        char* tx_buffer);
    /**
     * TXRX thread that runs a while loop to do both tx and rx
     */
    void* loopTXRX(int tid);
    int dequeue_send(int tid);
    struct Packet* recv_enqueue(int tid, int radio_id, int rx_offset);

private:
    Config* cfg;
    const size_t core_offset;
    const size_t mac_thread_num;
    Table<char>* buffer_;
    Table<int>* buffer_status_;
    size_t packet_num_in_buffer_;
    char* tx_buffer_;
    Table<size_t>* frame_start_;
    // pointer of message_queue_
    moodycamel::ConcurrentQueue<Event_data>* message_queue_;
    moodycamel::ConcurrentQueue<Event_data>* task_queue_;
    moodycamel::ProducerToken** rx_ptoks_;
    moodycamel::ProducerToken** tx_ptoks_;

#if USE_IPV4
    struct sockaddr_in* servaddr_; /* server address */
#else
    struct sockaddr_in6* servaddr_; /* server address */
#endif
    int* socket_;

    pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
    pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
};

#endif
