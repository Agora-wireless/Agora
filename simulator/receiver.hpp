#ifndef RECEIVER
#define RECEIVER

#include "Symbols.hpp"
#include "buffer.hpp"
#include "concurrentqueue.h"
#include "config.hpp"
#include "gettime.h"
#include "mkl_dfti.h"
#include "net.hpp"
#include <algorithm>
#include <armadillo>
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
#include <thread>

#ifdef USE_DPDK
#include "dpdk_transport.hpp"
#include <netinet/ether.h>
#endif

static constexpr size_t kMaxRecvNum = 16UL;

typedef unsigned short ushort;
class Receiver {
public:
    Receiver(Config* cfg, size_t rx_thread_num = 1, size_t core_offset = 1, void* mbuf_pool = nullptr);

    /**
     * rx_thread_num: RX thread number
     * in_queue: message queue to communicate with main thread
     */
    Receiver(Config* cfg, size_t rx_thread_num, size_t core_offset,
        moodycamel::ConcurrentQueue<EventData>* in_queue_message,
        moodycamel::ProducerToken** in_rx_ptoks);
    ~Receiver();

    /**
     * Called in main threads to start the socket threads
     * in_buffer: ring buffer to save packets
     * in_buffer_status: record the status of each memory block (0: empty, 1:
     * full) in_buffer_frame_num: number of packets the ring buffer could hold
     * in_buffer_length: size of ring buffer
     * in_core_id: attach socket threads to {in_core_id, ..., in_core_id +
     * RX_THREAD_NUM - 1}
     */
    // std::vector<pthread_t> startRecv(Table<char>& in_buffer,
    //     Table<int>& in_buffer_status, size_t in_buffer_frame_num,
    //     size_t in_buffer_length, Table<double>& in_frame_start);
    void startRecv(Table<char>& in_buffer,
        Table<int>& in_buffer_status, size_t in_buffer_frame_num,
        size_t in_buffer_length, Table<double>& in_frame_start);

    /**
     * receive thread
     * context: ReceiverContext type
     */
    void* loopRecv(int tid);

    void join_thread();

    void run_ifft(short* src, complex_float* ifft_inout,
        DFTI_DESCRIPTOR_HANDLE mkl_handle) const;

    void save_tx_data_to_file(int frame_id);

    std::atomic<size_t> completion_num_;

private:
    pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
    pthread_cond_t cond = PTHREAD_COND_INITIALIZER;

    Table<char>* buffer_;
    Table<int>* buffer_status_;
    long long buffer_length_;
    size_t buffer_frame_num_;

    char* tx_buffer_;
    int* tx_buffer_status_;
    long long tx_buffer_length_;
    int tx_buffer_frame_num_;

    size_t rx_thread_num_;
    size_t tx_thread_num_;

    Table<double>* frame_start_;
    moodycamel::ConcurrentQueue<EventData>* message_queue_;
    moodycamel::ProducerToken** rx_ptoks_;
    size_t core_id_;
    Config* cfg;
    // int radios_per_thread;

    Table<char> socket_buffer_;
    Table<size_t> socket_buffer_status_;
    size_t cur_frame_ = 0;
    std::array<size_t, kFrameWnd> frame_status_;
    std::mutex frame_mutex_;

    // Array to store data after IFFT
    Table<char> dl_ue_data_buffer_;

    // Receiver threads
    std::thread receiver_threads_[kMaxRecvNum];
    
#ifdef USE_DPDK
    struct rte_mempool* mbuf_pool;
    uint32_t bs_rru_addr; // IPv4 address of this data sender
    uint32_t bs_server_addr; // IPv4 address of the remote target Agora server
    std::vector<uint32_t> bs_server_addr_list; // IPv4 address list of all Agora servers
    rte_ether_addr receiver_mac_addr; // MAC address of this data sender

    // MAC address of the remote target Agora server
    rte_ether_addr server_mac_addr;
    std::vector<rte_ether_addr> server_mac_addr_list;
#endif
};

#endif
