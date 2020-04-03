/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */

#ifndef MILLIPEDE_HEAD
#define MILLIPEDE_HEAD

#include <fcntl.h>
#include <iostream>
#include <memory>
#include <pthread.h>
#include <queue>
#include <system_error>
#include <unistd.h>
#include <vector>
// #include <complex.h>
#include <algorithm>
#include <armadillo>
#include <emmintrin.h>
#include <immintrin.h>
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <tuple>
// #include <aff3ct.hpp>
#include "mkl_dfti.h"
// #include <hpctoolkit.h>
// #include <cblas.h>
// #include <stdio.h>
// #include "cpu_attach.hpp"
#include "buffer.hpp"
#include "concurrent_queue_wrapper.hpp"
#include "concurrentqueue.h"
#include "dodemul.hpp"
#include "dofft.hpp"
#include "doprecode.hpp"
#include "dozf.hpp"
#include "gettime.h"
#include "reciprocity.hpp"
#include "txrx.hpp"

#ifdef USE_LDPC
#include "docoding.hpp"
#endif

#include "config.hpp"
#include "memory_manage.h"
#include "signalHandler.hpp"
#include "stats.hpp"
#include "utils.h"

class Millipede {
public:
    /* optimization parameters for block transpose (see the slides for more
     * details) */
    static const int transpose_block_num = 256;
    /* dequeue bulk size, used to reduce the overhead of dequeue in main thread
     */
    static const int kDequeueBulkSizeTXRX = 8;
    static const int kDequeueBulkSizeWorker = 4;

    /**
     * @brief Create a Millipede object and start the worker threads
     */
    Millipede(Config*);
    ~Millipede();

    /**
     * @brief The main Millipede event loop
     */
    void start();
    void stop();

    void* worker_fft(int tid);
    void* worker_zf(int tid);
    void* worker_demul(int tid);
    void* worker(int tid);

    /* Launch threads to run worker with thread IDs tid_start to tid_end - 1 */
    void create_threads(void* (*worker)(void*), int tid_start, int tid_end);

    void handle_event_fft(int tag, ConcurrentQueueWrapper& zf_queue_wrapper,
        ConcurrentQueueWrapper& demul_queue_wrapper,
        ConcurrentQueueWrapper& consumer_rc);

    /* Add tasks into task queue based on event type */
    void schedule_demul_task(int frame_id, int start_sche_id, int end_sche_id,
        ConcurrentQueueWrapper const& demul_queue_wrapper);

    void update_rx_counters(int frame_count, int frame_id, int subframe_id);
    void print_per_frame_done(int task_type, int frame_count, int frame_id);
    void print_per_subframe_done(
        int task_type, int frame_count, int frame_id, int subframe_id);
    void print_per_task_done(
        int task_type, int frame_id, int subframe_id, int ant_or_sc_id);

    void initialize_queues();
    void initialize_uplink_buffers();
    void initialize_downlink_buffers();
    void free_uplink_buffers();
    void free_downlink_buffers();

    void save_demul_data_to_file(int frame_id);
    void save_decode_data_to_file(int frame_id);
    void save_ifft_data_to_file(int frame_id);
    void getDemulData(int** ptr, int* size);
    void getEqualData(float** ptr, int* size);

private:
    /* lookup table for 16 QAM, real and imag */
    float** qam16_table_;
    Config* config_;
    int fft_created_count;
    int max_equaled_frame = 0;
    // int max_packet_num_per_frame;
    std::unique_ptr<PacketTXRX> receiver_;
    Stats* stats_manager_;
    // std::unique_ptr<Stats> stats_manager_;
    // pthread_t task_threads[TASK_THREAD_NUM];
    // EventHandlerContext context[TASK_THREAD_NUM];
    pthread_t* task_threads;
    /*****************************************************
     * Buffers
     *****************************************************/
    /* Uplink */
    /**
     * received data
     * Frist dimension: SOCKET_THREAD_NUM
     * Second dimension of buffer (type: char): packet_length *
     * subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM
     * packet_length = sizeof(int) * 4 + sizeof(ushort) * OFDM_FRAME_LEN * 2;
     * Second dimension of buffer_status: subframe_num_perframe * BS_ANT_NUM *
     * SOCKET_BUFFER_FRAME_NUM
     */
    Table<char> socket_buffer_;
    Table<int> socket_buffer_status_;
    long long socket_buffer_size_;
    int socket_buffer_status_size_;

    /**
     * Estimated CSI data
     * First dimension: OFDM_CA_NUM * TASK_BUFFER_FRAME_NUM
     * Second dimension: BS_ANT_NUM * UE_NUM
     * First dimension: UE_NUM * TASK_BUFFER_FRAME_NUM
     * Second dimension: BS_ANT_NUM * OFDM_CA_NUM
     */
    Table<complex_float> csi_buffer_;

    /**
     * Data symbols after IFFT
     * First dimension: total subframe number in the buffer:
     * data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM second dimension:
     * BS_ANT_NUM * OFDM_CA_NUM second dimension data order: SC1-32 of ants,
     * SC33-64 of ants, ..., SC993-1024 of ants (32 blocks each with 32
     * subcarriers)
     */
    Table<complex_float> data_buffer_;

    /**
     * Calculated precoder
     * First dimension: OFDM_CA_NUM * TASK_BUFFER_FRAME_NUM
     * Second dimension: UE_NUM * BS_ANT_NUM
     */
    Table<complex_float> precoder_buffer_;

    /**
     * Data after equalization
     * First dimension: data_subframe_num_perframe (40-4) *
     * TASK_BUFFER_FRAME_NUM Second dimension: OFDM_CA_NUM * UE_NUM
     */
    Table<complex_float> equal_buffer_;

    /**
     * Data after demodulation
     * First dimension: data_subframe_num_perframe (40-4) *
     * TASK_BUFFER_FRAME_NUM Second dimension: OFDM_CA_NUM * UE_NUM
     */
    Table<uint8_t> demod_hard_buffer_;

    Table<int8_t> demod_soft_buffer_;

    Table<uint8_t> decoded_buffer_;

    RX_stats rx_stats_;
    FFT_stats fft_stats_;
    ZF_stats zf_stats_;
    RC_stats rc_stats_;
    Data_stats demul_stats_;
#ifdef USE_LDPC
    Data_stats decode_stats_;
    Data_stats encode_stats_;
#endif
    Data_stats precode_stats_;
    Data_stats ifft_stats_;
    Data_stats tx_stats_;

    // Per-frame queues of delayed FFT tasks. The queue contains offsets into
    // TX/RX buffers.
    std::array<std::queue<fft_req_tag_t>, TASK_BUFFER_FRAME_NUM> fft_queue_arr;

    /**
     * Raw data
     * First dimension: data_subframe_num_perframe * UE_NUM
     * Second dimension: OFDM_CA_NUM
     */
    Table<long long> dl_IQ_data_long;

    /**
     * Modulated data
     * First dimension: subframe_num_perframe (40) * UE_NUM *
     * TASK_BUFFER_FRAME_NUM Second dimension: OFDM_CA_NUM
     */
    // RawDataBuffer dl_rawdata_buffer_;

    /**
     * Data for IFFT
     * First dimension: FFT_buffer_block_num = BS_ANT_NUM *
     * data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM Second dimension:
     * OFDM_CA_NUM
     */
    Table<complex_float> dl_ifft_buffer_;

    /**
     * Data after IFFT
     * First dimension: data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM
     * second dimension: UE_NUM * OFDM_CA_NUM
     * second dimension data order: SC1-32 of UEs, SC33-64 of UEs, ...,
     * SC993-1024 of UEs (32 blocks each with 32 subcarriers)
     */
    // DataBuffer dl_iffted_data_buffer_;

    Table<complex_float> dl_precoder_buffer_;
    Table<complex_float> recip_buffer_;
    Table<complex_float> calib_buffer_;
    Table<int8_t> dl_encoded_buffer_;

    /**
     * Data for transmission
     * First dimension of buffer (type: char): subframe_num_perframe *
     * SOCKET_BUFFER_FRAME_NUM Second dimension: packet_length * BS_ANT_NUM
     * packet_length = sizeof(int) * 4 + sizeof(ushort) * OFDM_FRAME_LEN * 2;
     * First dimension of buffer_status: subframe_num_perframe * BS_ANT_NUM *
     * SOCKET_BUFFER_FRAME_NUM
     */
    char* dl_socket_buffer_;
    int* dl_socket_buffer_status_;
    long long dl_socket_buffer_size_;
    int dl_socket_buffer_status_size_;

    /*****************************************************
     * Concurrent queues
     *****************************************************/
    /* Uplink*/
    moodycamel::ConcurrentQueue<Event_data> fft_queue_;
    moodycamel::ConcurrentQueue<Event_data> zf_queue_;
    moodycamel::ConcurrentQueue<Event_data> demul_queue_;
#ifdef USE_LDPC
    moodycamel::ConcurrentQueue<Event_data> decode_queue_;
#endif
    /* main thread message queue for data receiving */
    moodycamel::ConcurrentQueue<Event_data> message_queue_;
    /* main thread message queue for task completion*/
    moodycamel::ConcurrentQueue<Event_data> complete_task_queue_;

    /* Downlink*/
    moodycamel::ConcurrentQueue<Event_data> ifft_queue_;
    moodycamel::ConcurrentQueue<Event_data> rc_queue_;
    // moodycamel::ConcurrentQueue<Event_data> modulate_queue_;
#ifdef USE_LDPC
    moodycamel::ConcurrentQueue<Event_data> encode_queue_;
#endif
    moodycamel::ConcurrentQueue<Event_data> precode_queue_;
    moodycamel::ConcurrentQueue<Event_data> tx_queue_;

    /* Tokens */
    moodycamel::ProducerToken** rx_ptoks_ptr;
    moodycamel::ProducerToken** tx_ptoks_ptr;
    moodycamel::ProducerToken** worker_ptoks_ptr;
};

#endif
