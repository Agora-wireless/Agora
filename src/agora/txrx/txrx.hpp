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

/**
 * @brief Implementations of this class provide packet I/O for Agora.
 *
 * In the vanilla mode, this class provides socket or DPDK-based packet I/O to
 * Agora (running on the base station server or client) for communicating
 * with simulated peers.
 *
 * In the "Argos" mode, this class provides SoapySDR-based communication for
 * Agora (running on the base station server or client) for communicating
 * with real wireless hardware peers (antenna hubs for the server, UE devices
 * for the client).
 */
class PacketTXRX {
public:
    PacketTXRX(Config* cfg, size_t in_core_offset = 1,
        RxStatus* rx_status = nullptr, DemulStatus* demul_status = nullptr,
        DecodeStatus* decode_status = nullptr);

    PacketTXRX(Config* cfg, size_t core_offset,
        moodycamel::ConcurrentQueue<Event_data>* queue_message,
        moodycamel::ConcurrentQueue<Event_data>* queue_task,
        moodycamel::ProducerToken** rx_ptoks,
        moodycamel::ProducerToken** tx_ptoks);
    ~PacketTXRX();

#ifdef USE_DPDK
    // At thread [tid], receive packets from the NIC and enqueue them to the
    // master thread
    uint16_t dpdk_recv(int tid, size_t& prev_frame_id, size_t& rx_offset);
#endif

    /**
     * @brief Start the network I/O threads
     *
     * @param buffer Ring buffer to save packets
     * @param buffer_status Status of each packet buffer (0: empty, 1: full)
     * @packet_num_in_buffer Total number of buffers in an RX ring
     *
     * @return True on successfully starting the network I/O threads, false
     * otherwise
     */
    bool startTXRX(Table<char>& buffer, Table<int>& buffer_status,
        size_t packet_num_in_buffer, Table<size_t>& frame_start,
        char* tx_buffer,
        PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>* demod_buffers_
        = nullptr,
        Table<int8_t>* demod_soft_buffer_to_decode = nullptr);

    // TODO: Add documentation
    void send_beacon(int tid, size_t frame_id);

private:
    void* loop_tx_rx(int tid); // The thread function for thread [tid]
    void* demod_loop_tx_rx(int tid);

    int dequeue_send(int tid);
    int poll_send(int tid);
    struct Packet* recv_enqueue(int tid, int radio_id, int rx_offset);
    // Receive packets and relocate data to the correct address based on
    // the subcarrier range
    struct Packet* recv_relocate(int tid, int radio_id, int rx_offset);

    void recv_demod();

    void* loop_tx_rx_argos(int tid);
    int dequeue_send_argos(int tid);
    struct Packet* recv_enqueue_argos(int tid, int radio_id, int rx_offset);

    Config* cfg;

    // The network I/O threads run on cores
    // {core_offset, ..., core_offset + socket_thread_num - 1}
    const size_t core_offset;

    const size_t socket_thread_num;
    Table<char>* buffer_;
    Table<int>* buffer_status_;
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>* demod_buffers_;
    Table<int8_t>* demod_soft_buffer_to_decode_;
    size_t packet_num_in_buffer_;
    char* tx_buffer_;
    Table<size_t>* frame_start_;
    moodycamel::ConcurrentQueue<Event_data>* message_queue_;
    moodycamel::ConcurrentQueue<Event_data>* task_queue_;
    moodycamel::ProducerToken** rx_ptoks_;
    moodycamel::ProducerToken** tx_ptoks_;

    std::vector<struct sockaddr_in> bs_rru_sockaddr_;
    std::vector<int> socket_;
    std::vector<struct sockaddr_in> millipede_addrs_;
    int demod_tx_socket_;
    int demod_rx_socket_;

    char* send_buffer_;

#ifdef USE_DPDK
    uint32_t bs_rru_addr; // IPv4 address of the simulator sender
    uint32_t bs_server_addr; // IPv4 address of the Agora server
    struct rte_mempool* mbuf_pool;
#endif

    RadioConfig* radioconfig_; // Used only in Argos mode

    RxStatus* rx_status_; // Shared states with workers
    DemulStatus* demul_status_;
    size_t demod_frame_to_send = 0;
    size_t demod_symbol_to_send;
    DecodeStatus* decode_status_;
};

#endif
