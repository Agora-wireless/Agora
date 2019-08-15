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
#include <vector>
#include <ctime>
#include <algorithm>
#include <numeric>
#include <pthread.h>
#include <cassert>
#include <unistd.h>
#include <chrono>
#include <sys/ioctl.h>
#include <fcntl.h>
#include "buffer.hpp"
#include "concurrentqueue.h"
#include "Symbols.hpp"
#include "gettime.h"


#include <inttypes.h>
#if USE_DPDK
#include <rte_arp.h>
#include <rte_byteorder.h>
#include <rte_cycles.h>
#include <rte_debug.h>
#include <rte_distributor.h>
#include <rte_eal.h>
#include <rte_ethdev.h>
#include <rte_ether.h>
#include <rte_ip.h>
#include <rte_malloc.h>
#include <rte_pause.h>
#include <rte_prefetch.h>
#include <rte_udp.h>
#include <rte_flow.h>
#endif


#define RX_RING_SIZE 8192*4
#define TX_RING_SIZE 8192*4

#define NUM_MBUFS ((32 * 1024) - 1)
#define MBUF_SIZE 128 + (sizeof(int) * 16 + sizeof(ushort) * OFDM_FRAME_LEN * 2)
#define MBUF_CACHE_SIZE 128
#define BURST_SIZE 16

#define ETH_HDRLEN 14
#define IP4_HDRLEN 20
#define UDP_HDRLEN 8
#define MAX_JUMBO_FRAME_SIZE 9600//9600
#define EMPTY_MASK 0x0
#define FULL_MASK 0xffffffff




typedef unsigned short ushort;
class PackageReceiver
{
public:
    
    static const int OFDM_FRAME_LEN = OFDM_CA_NUM + OFDM_PREFIX_LEN;
    // header 4 int for: frame_id, subframe_id, cell_id, ant_id
    // ushort for: I/Q samples
    static const int package_length = sizeof(int) * 16 + sizeof(ushort) * OFDM_FRAME_LEN * 2;
    static const int data_offset = sizeof(int) * 16;



    

    // use for create pthread 
    struct PackageReceiverContext
    {
        PackageReceiver *ptr;
        int tid;
    };

public:
    PackageReceiver(int RX_THREAD_NUM = 1, int TX_THREAD_NUM = 1, int in_core_offset = 1);
    /**
     * RX_THREAD_NUM: socket thread number
     * in_queue: message queue to communicate with main thread
    */ 
    PackageReceiver(int RX_THREAD_NUM, int TX_THREAD_NUM,  int in_core_offset, 
            moodycamel::ConcurrentQueue<Event_data> * in_queue_message, moodycamel::ConcurrentQueue<Event_data> * in_queue_task, 
            moodycamel::ProducerToken **in_rx_ptoks, moodycamel::ProducerToken **in_tx_ptoks);
    ~PackageReceiver();
    


    int nic_dpdk_init(uint16_t port, struct rte_mempool *mbuf_pool);

    int process_arp(struct rte_mbuf *mbuf, struct ether_hdr *eth_h, int len, int tid);


    /**
     * called in main threads to start the socket threads
     * in_buffer: ring buffer to save packets
     * in_buffer_status: record the status of each memory block (0: empty, 1: full)
     * in_buffer_frame_num: number of packets the ring buffer could hold
     * in_buffer_length: size of ring buffer
     * in_core_id: attach socket threads to {in_core_id, ..., in_core_id + RX_THREAD_NUM - 1}
    */ 
    std::vector<pthread_t> startRecv(char** in_buffer, int** in_buffer_status, int in_buffer_frame_num, long long in_buffer_length, double **in_frame_start);
    std::vector<pthread_t> startTX(char* in_buffer, int* in_buffer_status, float *in_data_buffer, int in_buffer_frame_num, int in_buffer_length);
    /**
     * receive thread
     * context: PackageReceiverContext type
    */
    static void* loopRecv(void *context);
    static void* loopRecv_DPDK(void *context);
    static void* loopSend(void *context);
    static void* loopTXRX(void *context);


 
private:
#if USE_IPV4
    struct sockaddr_in servaddr_[10];    /* server address */
#else
    struct sockaddr_in6 servaddr_[10];    /* server address */
#endif
    int* socket_;


#if USE_DPDK
    struct ether_addr server_eth_addr;
    uint32_t src_addr;
    uint32_t dst_addr;
    int src_port_start = 6000;
    int dst_port_start = 8000;
#endif

    char** buffer_;
    int** buffer_status_;
    long long buffer_length_;
    int buffer_frame_num_;

    char* tx_buffer_;
    int* tx_buffer_status_;
    long long tx_buffer_length_;
    int tx_buffer_frame_num_;
    float *tx_data_buffer_;

    int rx_thread_num_;
    int tx_thread_num_;

    double **frame_start_;
    // pointer of message_queue_
    moodycamel::ConcurrentQueue<Event_data> *message_queue_;
    moodycamel::ConcurrentQueue<Event_data> *task_queue_;
    moodycamel::ProducerToken **rx_ptoks_;
    moodycamel::ProducerToken **tx_ptoks_;
    int core_id_;
    int tx_core_id_;

    PackageReceiverContext* tx_context;
    PackageReceiverContext* rx_context;
};


#endif
