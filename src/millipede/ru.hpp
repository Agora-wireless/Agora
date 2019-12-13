
#ifndef RU_HEADER
#define RU_HEADER

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
#include "Symbols.hpp"
#include "buffer.hpp"
#include "concurrentqueue.h"
#include "radio_lib.hpp"
#include <fstream>      // std::ifstream

class RU
{
public:
    
    // static const int OFDM_FRAME_LEN = SYMBOL_LEN; //OFDM_CA_NUM + OFDM_PREFIX_LEN + OFDM_POSTFIX_LEN;
    // header 4 int for: frame_id, subframe_id, cell_id, ant_id
    // ushort for: I/Q samples
    // static const int package_length = sizeof(int) * 4 + sizeof(ushort) * OFDM_FRAME_LEN * 2;
    static const int data_offset = sizeof(int) * 4;
    // use for create pthread 
    struct RUContext
    {
        RU *ptr;
        int tid;
    };

public:
    RU(int n_rx_thread, int n_tx_thread, Config *cfg);
    /**
     * N_THREAD: socket thread number
     * mode: tx=1 or rx=0 operation
     * in_queue: message queue to communicate with main thread
    */ 
    RU(int n_rx_thread, int n_tx_thread, Config *cfg, moodycamel::ConcurrentQueue<Event_data> * in_queue, moodycamel::ConcurrentQueue<Event_data> * in_queue_task);
    ~RU();
    
    void calibrateRadios(std::vector<std::vector<std::complex<float>>>&, std::vector<std::vector<std::complex<float>>>&, int);
    void startRadios();
    /**
     * called in main threads to start the socket threads
     * in_buffer: ring buffer to save packets
     * in_buffer_status: record the status of each memory block (0: empty, 1: full)
     * in_buffer_frame_num: number of packets the ring buffer could hold
     * in_buffer_length: size of ring buffer
     * in_core_id: attach socket threads to {in_core_id, ..., in_core_id + N_THREAD - 1}
    */ 
  std::vector<pthread_t> startProc(Table<char>& in_buffer, Table<int>& in_buffer_status, int in_buffer_frame_num, int in_buffer_length, int in_core_id=0);
    std::vector<pthread_t> startTX(char* in_buffer, char *in_pilot_buffer, int* in_buffer_status, int in_buffer_frame_num, int in_buffer_length, int in_core_id=0);
    /**
     * receive thread
     * context: PackageReceiverContext type
    */
    static void* sendThread_launch(void *context);
    void sendThread(int tid);
    static void* taskThread_launch(void *context);
    void taskThread(int tid);
 
private:
    pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
    pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
    Config *config_;
    struct sockaddr_in servaddr_[10];    /* server address */
    struct sockaddr_in servaddr_tx_[10]; /* server address for tx*/
    struct sockaddr_in cliaddr_[10];    /* client address */
    int* rx_socket_;
    int* tx_socket_;

    RadioConfig *radioconfig_;

    Table<char>* buffer_;
    Table<int>* buffer_status_;
    int buffer_length_;
    int buffer_frame_num_;

    char* tx_buffer_;
    char* pilot_buffer_;
    int* tx_buffer_status_;
    int tx_buffer_frame_num_;
    int tx_buffer_length_;

    int thread_num_;
    int tx_thread_num_;
    // pointer of message_queue_
    moodycamel::ConcurrentQueue<Event_data> *message_queue_;
    moodycamel::ConcurrentQueue<Event_data> *task_queue_;
    std::vector<std::unique_ptr<moodycamel::ProducerToken>> task_ptok;
    //std::vector<std::unique_ptr<moodycamel::ConsumerToken>> task_ctok;
    int core_id_;
    int tx_core_id_;
};
#endif
