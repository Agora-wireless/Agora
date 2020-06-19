/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */

#ifndef PACKETTXRX
#define PACKETTXRX

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
#include <stdio.h> /* for fprintf */
#include <stdlib.h>
#include <string.h> /* for memcpy */
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
// #include <unistd.h>
#include <vector>

#ifdef USE_ARGOS
#include "radio_lib.hpp"
#else
#include "config.hpp"
#endif

#ifdef USE_DPDK
#include "dpdk_transport.hpp"
#endif

typedef unsigned short ushort;
class PacketTXRX {
public:
    PacketTXRX(Config* cfg, size_t in_core_offset = 1);
    /**
     * queue_message: message queue to communicate with main thread
     */
    PacketTXRX(Config* cfg, size_t core_offset,
        moodycamel::ConcurrentQueue<Event_data>* queue_message,
        moodycamel::ConcurrentQueue<Event_data>* queue_task,
        moodycamel::ProducerToken** rx_ptoks,
        moodycamel::ProducerToken** tx_ptoks);
    ~PacketTXRX();

#ifdef USE_DPDK
    uint16_t dpdk_recv_enqueue(int tid, int& prev_frame_id, size_t& rx_offset);
#endif

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
    const size_t socket_thread_num;
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

#ifdef USE_DPDK
    uint32_t
        client_addr; // IPv4 address of the data sender that sends data to Millipede
    uint32_t server_addr; // IPv4 address of the Millipede server
    struct rte_mempool* mbuf_pool;
#endif

#ifdef USE_ARGOS
    RadioConfig* radioconfig_;
#endif
};

#endif
