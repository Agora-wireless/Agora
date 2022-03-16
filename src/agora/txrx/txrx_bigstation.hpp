#pragma once

#include "Symbols.hpp"
#include "buffer.hpp"
#include "concurrentqueue.h"
#include "config.hpp"
#include "gettime.h"
#include "net.hpp"
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

class BigStationTXRX {
public:
    BigStationTXRX(Config* cfg, size_t in_core_offset,
        PtrCube<kFrameWnd, kMaxSymbols, kMaxAntennas, uint8_t>& time_iq_buffer,
        BigStationState* bigstation_state);

    ~BigStationTXRX();

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
    bool StartTXRX();

private:
    // The simulation-mode thread function running on thread #tid
    void* loop_tx_rx(int tid);

    // Receive packets and relocate data to the correct address based on
    // the subcarrier range
    int recv_relocate(int tid);

    void* tx_thread(int tid);

    Config* cfg_;

    // The network I/O threads run on cores
    // {core_offset, ..., core_offset + socket_thread_num - 1}
    const size_t core_offset_;
    size_t rx_thread_num_;
    size_t tx_thread_num_;

    BigStationState* bigstation_state_;

    // Current sending frame for each socket thread
    PtrCube<kFrameWnd, kMaxSymbols, kMaxAntennas, uint8_t>& time_iq_buffer_;

    std::vector<uint32_t> bs_rru_addrs_;
    struct rte_mempool* mbuf_pool_[kMaxThreads];
    std::vector<uint32_t> bs_server_addrs_;
    std::vector<rte_ether_addr> bs_server_mac_addrs_;
    std::vector<rte_ether_addr> bs_rru_mac_addrs_;
};