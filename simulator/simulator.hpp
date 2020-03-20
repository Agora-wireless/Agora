/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */

#ifndef SIMULATOR_HEAD
#define SIMULATOR_HEAD

#include <fcntl.h>
#include <iostream>
#include <memory>
#include <pthread.h>
#include <queue>
#include <system_error>
#include <unistd.h>
#include <vector>
// #include <complex.h>
#include <math.h>
#include <tuple>
// #include <armadillo>
#include <algorithm>
#include <emmintrin.h>
#include <immintrin.h>
#include <signal.h>
#include <stdint.h>
// #include <aff3ct.hpp>
// #include "mkl_dfti.h"
// #include <hpctoolkit.h>
// #include <cblas.h>
// #include <stdio.h>
#include "buffer.hpp"
#include "concurrentqueue.h"
#include "gettime.h"
#include "receiver.hpp"
#include "sender.hpp"
// #include "compute_common.hpp"
#include "offset.h"
// #include "dofft.hpp"
// #include "dodemul.hpp"
#include "config.hpp"
#include "memory_manage.h"
#include "signalHandler.hpp"

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

    Simulator(Config* cfg, size_t task_thread_num, size_t socket_tx_num,
        size_t core_offset, size_t sender_delay);
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
        size_t subframe_id, size_t ant_id);
    void print_per_frame_done(
        size_t task_type, size_t frame_id, size_t frame_id_in_buffer);

    void initialize_vars_from_cfg(Config* cfg);
    void initialize_queues();
    void initialize_uplink_buffers();
    void free_uplink_buffers();

private:
    size_t bs_ant_num, ue_num;
    size_t ofdm_ca_num;
    size_t ofdm_data_num;
    size_t subframe_num_perframe, data_subframe_num_perframe;
    size_t ul_data_subframe_num_perframe, dl_data_subframe_num_perframe;
    size_t dl_data_subframe_start, dl_data_subframe_end;
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
    std::unique_ptr<Receiver> receiver_;
    std::unique_ptr<Sender> sender_;
    pthread_t* task_threads;
    EventHandlerContext* context;
    /*****************************************************
     * Buffers
     *****************************************************/
    /* Uplink */
    /**
     * received data
     * Frist dimension: SOCKET_THREAD_NUM
     * Second dimension of buffer (type: char): packet_length *
     * subframe_num_perframe * bs_ant_num * SOCKET_BUFFER_FRAME_NUM
     * packet_length = sizeof(int) * 4 + sizeof(ushort) * ofdm_frame_len * 2;
     * Second dimension of buffer_status: subframe_num_perframe * bs_ant_num *
     * SOCKET_BUFFER_FRAME_NUM
     */

    Table<char> socket_buffer_;
    Table<int> socket_buffer_status_;
    size_t socket_buffer_size_;
    size_t socket_buffer_status_size_;

    /* Uplink status checkers used by master thread */
    /* used to check if RX for all antennas and all subframes in a frame is done
     * (max: bs_ant_num * subframe_num_perframe) */
    size_t* rx_counter_packets_;

    /*****************************************************
     * Concurrent queues
     *****************************************************/
    /* main thread message queue for data receiving */
    moodycamel::ConcurrentQueue<Event_data> message_queue_;
    /* main thread message queue for task completion*/
    moodycamel::ConcurrentQueue<Event_data> complete_task_queue_;

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
