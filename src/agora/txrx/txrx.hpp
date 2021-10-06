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
    PacketTXRX(Config* cfg, size_t in_core_offset,
        Table<char>& buffer,
        Table<complex_float>& tx_buffer,
        PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& demod_buffers_to_send,
        Table<int8_t>& demod_buffer_to_decode,
        Table<int8_t>& encoded_buffer_,
        Table<int8_t>& encoded_buffer_to_decode,
        RxStatus* rx_status = nullptr,
        EncodeStatus* encode_status = nullptr,
        PrecodeStatus* precode_status = nullptr);

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
    bool startTXRX();

    // TODO: Add documentation
    void send_beacon(int tid, size_t frame_id);

    // Not used
    size_t get_dl_frame_to_send() { return dl_frame_to_send_; }

    // Current sending frame for each socket thread
    size_t frame_to_send_[kMaxThreads] = {0};

private:
    // The simulation-mode thread function running on thread #tid
    void* loop_tx_rx(int tid);

    // A thread that sends and receives post-demodulation data
    void* demod_tx_thread(int tid);
    void* encode_thread(int tid);

    int dequeue_send(int tid, size_t symbol_to_send, size_t ant_to_send);
    int recv_enqueue(int tid, int radio_id, size_t rx_offset);

    // Receive packets and relocate data to the correct address based on
    // the subcarrier range
    int recv_relocate(int tid);

    void* loop_tx_rx_argos(int tid);
    int dequeue_send_argos(int tid);
    struct Packet* recv_enqueue_argos(int tid, int radio_id, int rx_offset);

    Config* cfg;

    // The network I/O threads run on cores
    // {core_offset, ..., core_offset + socket_thread_num - 1}
    const size_t core_offset;

    const size_t rx_thread_num;
    Table<char>& freq_domain_iq_buffer_;
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& demod_buffer_to_send_;
    Table<int8_t>& demod_buffer_to_decode_;
    size_t packet_num_in_buffer_;
    char* tx_buffer_;
    Table<complex_float>& dl_ifft_buffer_;

    // Downlink buffers
    Table<int8_t>& encoded_buffer_;
    Table<int8_t>& encoded_buffer_to_precode_;

    std::vector<struct sockaddr_in> bs_rru_sockaddr_;
    std::vector<int> socket_;
    std::vector<struct sockaddr_in> bs_server_sockaddrs_;
    int demod_tx_socket_;

    char* send_buffer_;
    uint8_t* recv_buffer_;

#ifdef USE_DPDK
    uint32_t bs_rru_addr_; // IPv4 address of the simulator sender
    struct rte_mempool* mbuf_pool_[kMaxThreads];
    // struct rte_mempool* mbuf_pool_;
    std::vector<uint32_t> bs_server_addrs_;
    std::vector<rte_ether_addr> bs_server_mac_addrs_;
    rte_ether_addr bs_rru_mac_addr_;
    int recv(int tid);
#endif

    RadioConfig* radioconfig_; // Used only in Argos mode

    RxStatus* rx_status_; // Shared states with workers
    size_t demod_frame_to_send_ = 0;
    size_t demod_symbol_ul_to_send_;
    EncodeStatus* encode_status_;
    PrecodeStatus* precode_status_;

    size_t encode_frame_to_send_ = 0;
    size_t encode_symbol_dl_to_send_ = 0;
    size_t encode_ue_to_send_;

    // Not used
    size_t dl_frame_to_send_ = 0;
    size_t dl_symbol_to_send_ = 0;

    size_t last_packet_cycle_[kMaxThreads] = {0};
    size_t max_inter_packet_gap_[kMaxThreads] = {0};
    size_t max_inter_packet_gap_frame_[kMaxThreads] = {0};

    size_t max_packet_record_time_[kMaxThreads] = {0};
    size_t max_packet_record_time_frame_[kMaxThreads] = {0};
};

#endif
