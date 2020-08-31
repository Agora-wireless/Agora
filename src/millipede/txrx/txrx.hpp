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
#include "radio_lib.hpp"
#include "shared_counters.hpp"
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

/**
 * @brief Implementations of this class provide packet I/O for Millipede.
 *
 * In the vanilla mode, this class provides socket or DPDK-based packet I/O to
 * Millipede (running on the base station server or client) for communicating
 * with simulated peers.
 *
 * In the "Argos" mode, this class provides SoapySDR-based communication for
 * Millipede (running on the base station server or client) for communicating
 * with real wireless hardware peers (antenna hubs for the server, UE devices
 * for the client).
 */
class PacketTXRX {
public:
    PacketTXRX(
        Config* cfg, size_t in_core_offset = 1, RxStatus* rx_status_ = nullptr);
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
    // uint16_t dpdk_recv_enqueue(int tid, int& prev_frame_id, size_t& rx_offset);
    uint16_t dpdk_recv_enqueue(int tid, int& prev_frame_id);
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

private:
    void* loop_tx_rx(int tid); // The TX/RX event loop
    int dequeue_send(int tid);
    int poll_send(int tid);
    struct Packet* recv_enqueue(int tid, int radio_id, int rx_offset);

    void* loop_tx_rx_argos(int tid);
    int dequeue_send_argos(int tid);
    struct Packet* recv_enqueue_argos(int tid, int radio_id, int rx_offset);

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

    std::vector<struct sockaddr_in> servaddr_; /* server address */
    std::vector<int> socket_;

#ifdef USE_DPDK
    uint32_t sender_addr; // IPv4 address of the simulator sender
    uint32_t server_addr; // IPv4 address of the Millipede server
    struct rte_mempool* mbuf_pool;
#endif

    RadioConfig* radioconfig_; // Used only in Argos mode

    RxStatus* rx_status_; // Shared states with workers
};

#endif
