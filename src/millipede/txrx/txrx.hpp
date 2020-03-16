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
#include "offset.h"
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
#include <unistd.h>
#include <vector>

#ifdef USE_ARGOS
#include "radio_lib.hpp"
#else
#include "config.hpp"
#endif

#ifdef USE_DPDK
#include <inttypes.h>
#include <rte_arp.h>
#include <rte_byteorder.h>
#include <rte_cycles.h>
#include <rte_debug.h>
#include <rte_distributor.h>
#include <rte_eal.h>
#include <rte_ethdev.h>
#include <rte_ether.h>
#include <rte_flow.h>
#include <rte_ip.h>
#include <rte_malloc.h>
#include <rte_pause.h>
#include <rte_prefetch.h>
#include <rte_udp.h>
#endif

#define RX_RING_SIZE 8192 * 4
#define TX_RING_SIZE 8192 * 4

#define NUM_MBUFS ((32 * 1024) - 1)
#define MBUF_SIZE 128 + (sizeof(int) * 16 + sizeof(ushort) * OFDM_FRAME_LEN * 2)
#define MBUF_CACHE_SIZE 128
#define BURST_SIZE 16

#define ETH_HDRLEN 14
#define IP4_HDRLEN 20
#define UDP_HDRLEN 8
#define MAX_JUMBO_FRAME_SIZE 9600 // 9600
#define EMPTY_MASK 0x0
#define FULL_MASK 0xffffffff

typedef unsigned short ushort;
class PacketTXRX {
public:
    //     // use for create pthread
    //     struct PacketTXRXContext
    //     {
    //         PacketTXRX *ptr;
    //         int tid;
    //     };

public:
    PacketTXRX(Config* cfg, int RX_THREAD_NUM = 1, int TX_THREAD_NUM = 1,
        int in_core_offset = 1);
    /**
     * RX_THREAD_NUM: socket thread number
     * in_queue: message queue to communicate with main thread
     */
    PacketTXRX(Config* cfg, int RX_THREAD_NUM, int TX_THREAD_NUM,
        int in_core_offset,
        moodycamel::ConcurrentQueue<event_data_t>* in_queue_message,
        moodycamel::ConcurrentQueue<event_data_t>* in_queue_task,
        moodycamel::ProducerToken** in_rx_ptoks,
        moodycamel::ProducerToken** in_tx_ptoks);
    ~PacketTXRX();

#ifdef USE_DPDK
    int nic_dpdk_init(uint16_t port, struct rte_mempool* mbuf_pool);
    int process_arp(
        struct rte_mbuf* mbuf, struct ether_hdr* eth_h, int len, int tid);
#endif

    /**
     * called in main threads to start the socket threads
     * in_buffer: ring buffer to save packets
     * in_buffer_status: record the status of each memory block (0: empty, 1:
     * full) in_buffer_frame_num: number of packets the ring buffer could hold
     * in_buffer_length: size of ring buffer
     * in_core_id: attach socket threads to {in_core_id, ..., in_core_id +
     * RX_THREAD_NUM - 1}
     */
    std::vector<pthread_t> startRecv(Table<char>& in_buffer,
        Table<int>& in_buffer_status, int in_buffer_frame_num,
        long long in_buffer_length, Table<double>& in_frame_start);
    std::vector<pthread_t> startTX(char* in_buffer, int* in_buffer_status,
        int in_buffer_frame_num, int in_buffer_length);
    /**
     * receive thread
     */
    void* loopRecv(int tid);
    void* loopTXRX(int tid);
    void* loopSend(int tid);
#if USE_IPV4
    typedef struct sockaddr_in sockaddr_t;
#else
    typedef struct sockaddr_in6 sockaddr_t;
#endif
    int dequeue_send(int tid, int socket_local, sockaddr_t* remote_addr);
    struct Packet* recv_enqueue(int tid, int socket_local, int rx_offset);
#ifdef USE_DPDK
    static void* loopRecv_DPDK(void* context);
#endif
#if USE_ARGOS
    void* loopRecv_Argos(int tid);
    void* loopSend_Argos(int tid);
    int dequeue_send_Argos(int tid);
    struct Packet* recv_enqueue_Argos(int tid, int radio_id, int rx_offset);
#endif

private:
#if USE_IPV4
    struct sockaddr_in servaddr_[10]; /* server address */
#else
    struct sockaddr_in6 servaddr_[10]; /* server address */
#endif
    int* socket_;

    pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
    pthread_cond_t cond = PTHREAD_COND_INITIALIZER;

#ifdef USE_DPDK
    struct ether_addr server_eth_addr;
    uint32_t src_addr;
    uint32_t dst_addr;
    int src_port_start = 6000;
    int dst_port_start = 8000;
#endif

    Table<char>* buffer_;
    Table<int>* buffer_status_;
    long long buffer_length_;
    int buffer_frame_num_;

    char* tx_buffer_;
    int* tx_buffer_status_;
    long long tx_buffer_length_;
    int tx_buffer_frame_num_;
    // float *tx_data_buffer_;

    int rx_thread_num_;
    int tx_thread_num_;

    Table<double>* frame_start_;
    // pointer of message_queue_
    moodycamel::ConcurrentQueue<event_data_t>* message_queue_;
    moodycamel::ConcurrentQueue<event_data_t>* task_queue_;
    moodycamel::ProducerToken** rx_ptoks_;
    moodycamel::ProducerToken** tx_ptoks_;
    int core_id_;
    int tx_core_id_;

    // PacketTXRXContext* tx_context;
    // PacketTXRXContext* rx_context;

    Config* config_;
#if USE_ARGOS
    RadioConfig* radioconfig_;
#endif
};

#endif
