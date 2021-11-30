#ifndef SIMULATOR_HEAD
#define SIMULATOR_HEAD

#include "buffer.hpp"
#include "concurrentqueue.h"
#include "config.hpp"
#include "gettime.h"
#include "memory_manage.h"
#include "receiver.hpp"
#include "dynamic_sender.hpp"
#include "signalHandler.hpp"
#include <algorithm>
#include <emmintrin.h>
#include <fcntl.h>
#include <immintrin.h>
#include <iostream>
#include <math.h>
#include <memory>
#include <pthread.h>
#include <queue>
#include <signal.h>
#include <stdint.h>
#include <system_error>
#include <tuple>
#include <unistd.h>
#include <vector>

class Simulator {
public:
    /* optimization parameters for block transpose (see the slides for more
     * details) */
    static constexpr size_t kTransposeBlockSize = 8;
    static constexpr size_t kTransposeBlockNum = 256;
    /* dequeue bulk size, used to reduce the overhead of dequeue in main thread
     */
    static constexpr size_t kDequeueBulkSize = 32;
    static constexpr size_t kDequeueBulkSizeSingle = 8;

    Simulator(Config* cfg, size_t task_thread_num, size_t core_offset,
        size_t sender_delay);

    ~Simulator();

    void start();
    void stop();
    // while loop of task thread
    static void* taskThread(void* context);

    struct EventHandlerContext {
        Simulator* obj_ptr;
        size_t id;
    };

    inline void update_frame_count(int* frame_count);

    void update_rx_counters(size_t frame_id, size_t frame_id_in_buffer,
        size_t symbol_id, size_t ant_id);
    void print_per_frame_done(PrintType print_type, size_t frame_id);

    void initialize_vars_from_cfg(Config* cfg);
    void initialize_queues();
    void initialize_uplink_buffers();
    void free_uplink_buffers();

private:
    size_t BS_ANT_NUM, UE_NUM;
    size_t OFDM_CA_NUM;
    size_t OFDM_DATA_NUM;
    size_t symbol_num_perframe, data_symbol_num_perframe;
    size_t ul_data_symbol_num_perframe, dl_data_symbol_num_perframe;
    size_t dl_data_symbol_start, dl_data_symbol_end;
    size_t packet_length;

    size_t TASK_THREAD_NUM, SOCKET_RX_THREAD_NUM, SOCKET_TX_THREAD_NUM;
    size_t CORE_OFFSET;
    size_t demul_block_size, demul_block_num;

    /* lookup table for 16 QAM, real and imag */
    Table<float> qam16_table_;
    // float *pilots_;
    Config* config_;
    size_t max_equaled_frame = 0;
    float csi_format_offset;
    size_t buffer_frame_num;
    size_t max_packet_num_per_frame;
    // std::unique_ptr<Receiver> receiver_;
    // std::unique_ptr<Sender> sender_;
    Receiver* receiver_;
    Sender* sender_;
    pthread_t* task_threads;
    EventHandlerContext* context;

    // Uplink buffers

    /**
     * Received data
     * 
     * First dimension: SOCKET_THREAD_NUM
     *
     * Second dimension of socket_buffer: SOCKET_BUFFER_FRAME_NUM * BS_ANT_NUM *
     * symbol_num_perframe * packet_length
     *
     * Second dimension of buffer status: SOCKET_BUFFER_FRAME_NUM * BS_ANT_NUM *
     * symbol_num_perframe
     */
    Table<char> socket_buffer_;
    Table<int> socket_buffer_status_;

    size_t socket_buffer_size_;
    size_t socket_buffer_status_size_;

    /* Uplink status checkers used by master thread */
    /* used to check if RX for all antennas and all symbols in a frame is done
     * (max: BS_ANT_NUM * symbol_num_perframe) */
    size_t* rx_counter_packets_;

    /*****************************************************
     * Concurrent queues
     *****************************************************/
    /* main thread message queue for data receiving */
    moodycamel::ConcurrentQueue<EventData> message_queue_;
    /* main thread message queue for task completion*/
    moodycamel::ConcurrentQueue<EventData> complete_task_queue_;

    /* Tokens */
    moodycamel::ProducerToken** rx_ptoks_ptr;
    moodycamel::ProducerToken** tx_ptoks_ptr;
    moodycamel::ProducerToken** task_ptoks_ptr;

    /*****************************************************
     * Timestamps and counters used in worker threads
     *****************************************************/
    Table<double> frame_start;
    double* frame_start_receive;
    double* frame_end_receive;
    double* frame_start_tx;
    double* frame_end_tx;
};

#endif
