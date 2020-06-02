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
     * COMM_THREAD_NUM: socket thread number
     * in_queue: message queue to communicate with main thread
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
     * in_buffer: ring buffer to save packets
     * in_buffer_status: record the status of each memory block (0: empty, 1:
     * full) in_buffer_frame_num: number of packets the ring buffer could hold
     * in_buffer_length: size of ring buffer
     * in_core_id: attach socket threads to {in_core_id, ..., in_core_id +
     * COMM_THREAD_NUM - 1}
     */
    bool startTXRX(Table<char>& buffer, Table<int>& buffer_status,
        size_t packet_num_in_buffer, Table<size_t>& frame_start,
        char* tx_buffer);
    /**
     * receive thread
     */
    void* loopTXRX(int tid);
#if USE_IPV4
    typedef struct sockaddr_in sockaddr_t;
#else
    typedef struct sockaddr_in6 sockaddr_t;
#endif
    int dequeue_send(int tid);
    struct Packet* recv_enqueue(int tid, int radio_id, int rx_offset);

private:
    Config* cfg;
    size_t core_offset;
    size_t socket_thread_num;
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
    uint32_t src_addr;
    uint32_t dst_addr;
    struct rte_mempool* mbuf_pool;
#endif

#ifdef USE_ARGOS
    RadioConfig* radioconfig_;
#endif
};

#endif
