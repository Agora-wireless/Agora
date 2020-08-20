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

    //struct ChannelSimContext {
    //    ChannelSim* ptr;
    //    int tid;
    //};

public:
    ChannelSim(Config* bscfg, Config* uecfg, size_t bs_socket_num,
        size_t ue_socket_num, size_t bs_thread_num, size_t user_thread_num,
        size_t worker_thread_num, size_t in_core_offset = 30);
    ~ChannelSim();

    void start();

    void* ue_rx_loop(int tid);
    void* bs_rx_loop(int tid);

    void do_tx_bs(int tid, size_t tag);
    void do_tx_user(int tid, size_t tag);

    void schedule_task(Event_data do_task,
        moodycamel::ConcurrentQueue<Event_data>* in_queue,
        moodycamel::ProducerToken const& ptok);
    void* taskThread(int tid);

private:
    UDPClient* udp_client; // UDP endpoint used for sending messages
    std::vector<UDPServer*>
        udp_server_uerx; // UDP endpoint used for receiving messages from client phy
    std::vector<UDPServer*>
        udp_server_bsrx; // UDP endpoint used for receiving messages from base station phy

    Config* bscfg;
    Config* uecfg;
    cx_fmat channel;
    char* tx_buffer_bs;
    char* tx_buffer_ue;
    char* rx_buffer_bs;
    char* rx_buffer_ue;

    moodycamel::ConcurrentQueue<Event_data> task_queue_bs;
    moodycamel::ConcurrentQueue<Event_data> task_queue_user;
    moodycamel::ConcurrentQueue<Event_data> message_queue_;
    moodycamel::ProducerToken* task_ptok[kMaxThreads];

    pthread_t task_threads[kMaxThreads];

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
