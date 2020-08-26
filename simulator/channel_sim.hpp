#ifndef SIMULATOR_HEADER
#define SIMULATOR_HEADER

#include "Symbols.hpp"
#include "buffer.hpp"
//#include "concurrentqueue.h"
#include "concurrent_queue_wrapper.hpp"
#include "config.hpp"
#include "gettime.h"
#include "memory_manage.h"
#include "net.hpp"
#include "signalHandler.hpp"
#include "udp_client.h"
#include "udp_server.h"
#include <algorithm>
#include <armadillo>
#include <arpa/inet.h>
#include <assert.h>
#include <ctime>
#include <math.h>
#include <netinet/in.h>
#include <numeric>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/types.h>

using namespace arma;

class ChannelSim {
public:
    static const int dequeue_bulk_size = 5;

public:
    ChannelSim(Config* bscfg, Config* uecfg, size_t bs_socket_num,
        size_t ue_socket_num, size_t bs_thread_num, size_t user_thread_num,
        size_t worker_thread_num, size_t in_core_offset = 30);
    ~ChannelSim();

    void start();

    // loop thread receiving symbols from client antennas
    void* ue_rx_loop(int tid);
    // loop thread receiving symbols from BS antennas
    void* bs_rx_loop(int tid);

    // transmits symbols to BS antennas after applying channel
    void do_tx_bs(int tid, size_t tag);
    // transmits symbols to client antennas after applying channel
    void do_tx_user(int tid, size_t tag);

    void schedule_task(Event_data do_task,
        moodycamel::ConcurrentQueue<Event_data>* in_queue,
        moodycamel::ProducerToken const& ptok);
    void* taskThread(int tid);

private:
    // bs-facing servers addresses
    std::vector<struct sockaddr_in> servaddr_bs_;
    // bs-facing sockets
    std::vector<int> socket_bs_;
    // ue-facing servers addresses
    std::vector<struct sockaddr_in> servaddr_ue_;
    // ue-facing sockets
    std::vector<int> socket_ue_;

    Config* bscfg;
    Config* uecfg;
    cx_fmat channel;

    // data buffer for symbols to be transmitted to BS antennas (uplink)
    char* tx_buffer_bs;

    // data buffer for symbols to be transmitted to client antennas (downlink)
    char* tx_buffer_ue;

    // data buffer for received symbols from BS antennas (downlink)
    char* rx_buffer_bs;

    // data buffer for received symbols from client antennas (uplink)
    char* rx_buffer_ue;

    // Task Queue for tasks related to incoming BS packets
    moodycamel::ConcurrentQueue<Event_data> task_queue_bs;

    // Task Queue for tasks related to incoming Users' packets
    moodycamel::ConcurrentQueue<Event_data> task_queue_user;

    // Master thread's message queue for event completions;
    moodycamel::ConcurrentQueue<Event_data> message_queue_;
    moodycamel::ProducerToken* task_ptok[kMaxThreads];

    pthread_t* task_threads;

    size_t symbol_perframe;
    size_t ul_data_symbol_perframe;
    size_t pilot_symbol_perframe;
    size_t ul_symbol_perframe;
    size_t dl_data_symbol_perframe;
    size_t dl_symbol_perframe;
    size_t numAntennas;
    size_t samps_persymbol;
    size_t nUEs;
    size_t payload_len;
    size_t payload_samps;

    size_t bs_thread_num;
    size_t user_thread_num;
    size_t bs_socket_num;
    size_t user_socket_num;
    size_t worker_thread_num;
    size_t core_offset;

    size_t* bs_rx_counter_;
    size_t* user_rx_counter_;
    size_t bs_tx_counter_[TASK_BUFFER_FRAME_NUM];
    size_t user_tx_counter_[TASK_BUFFER_FRAME_NUM];
};

#endif
