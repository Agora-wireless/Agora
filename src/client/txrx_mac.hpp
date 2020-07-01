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
#include "config.hpp"
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
#include <vector>

#ifdef USE_DPDK
#include "dpdk_transport.hpp"
#endif

typedef unsigned short ushort;
class PacketTXRX {
public:
    PacketTXRX(Config* cfg, int COMM_THREAD_NUM = 1, int in_core_offset = 1);
    /**
     * COMM_THREAD_NUM: socket thread number
     * in_queue: message queue to communicate with main thread
     */
    PacketTXRX(Config* cfg, int COMM_THREAD_NUM, int in_core_offset,
        moodycamel::ConcurrentQueue<Event_data>* in_queue_message,
        moodycamel::ConcurrentQueue<Event_data>* in_queue_task,
        moodycamel::ProducerToken** in_rx_ptoks,
        moodycamel::ProducerToken** in_tx_ptoks);
    ~PacketTXRX();

#ifdef USE_DPDK
    uint16_t dpdk_recv_enqueue(int tid, int& prev_frame_id, int& rx_offset);
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
    bool startTXRX(Table<char>& in_buffer, Table<int>& in_buffer_status,
        int in_buffer_frame_num, long long in_buffer_length, char* in_tx_buffer,
        int* in_tx_buffer_status, int in_tx_buffer_frame_num,
        int in_tx_buffer_length);
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
    struct MacPacket* recv_enqueue(int tid, int radio_id, int rx_offset);

private:
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

    Table<char>* buffer_;
    Table<int>* buffer_status_;
    long long buffer_length_;
    int buffer_frame_num_;

    char* tx_buffer_;
    int* tx_buffer_status_;
    long long tx_buffer_length_;
    int tx_buffer_frame_num_;

    int comm_thread_num_;

    Table<size_t>* frame_start_;
    // pointer of message_queue_
    moodycamel::ConcurrentQueue<Event_data>* message_queue_;
    moodycamel::ConcurrentQueue<Event_data>* task_queue_;
    moodycamel::ProducerToken** rx_ptoks_;
    moodycamel::ProducerToken** tx_ptoks_;
    int core_id_;
    int tx_core_id_;

    Config* config_;
};

#endif
