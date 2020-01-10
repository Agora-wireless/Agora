/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 * 
 */

#ifndef MILLIPEDE_HEAD
#define MILLIPEDE_HEAD

#include "mufft/fft.h"
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
#include "concurrentqueue.h"
#include "dodemul.hpp"
#include "dofft.hpp"
#include "doprecode.hpp"
#include "dozf.hpp"
#include "gettime.h"
#include "offset.h"
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
    /* optimization parameters for block transpose (see the slides for more details) */
    static const int transpose_block_num = 256;
    /* dequeue bulk size, used to reduce the overhead of dequeue in main thread */
    static const int dequeue_bulk_size = 32;
    static const int dequeue_bulk_size_single = 8;

    Millipede(Config*);
    ~Millipede();

    void start();
    void stop();

    void* worker(int tid);
    void* worker_fft(int tid);
    void* worker_zf(int tid);
    void* worker_demul(int tid);
    void create_threads(thread_type thread, int tid_start, int tid_end);

    // struct EventHandlerContext
    // {
    //     Millipede *obj_ptr;
    //     int id;
    // };

    inline void update_frame_count(int* frame_count);
    /* Add tasks into task queue based on event type */
    void schedule_fft_task(int offset, int frame_count, int frame_id, int subframe_id, int ant_id,
        Consumer const& consumer);
    void schedule_delayed_fft_tasks(int frame_count, int frame_id, int data_subframe_id, Consumer const& consumer);
    void schedule_zf_task(int frame_id, Consumer const& consumer);
    void schedule_demul_task(int frame_id, int start_sche_id, int end_sche_id, Consumer const& consumer);

    void update_rx_counters(int frame_count, int frame_id, int subframe_id);
    void print_per_frame_done(int task_type, int frame_count, int frame_id);
    void print_per_subframe_done(int task_type, int frame_count, int frame_id, int subframe_id);
    void print_per_task_done(int task_type, int frame_id, int subframe_id, int ant_or_sc_id);

    void initialize_vars_from_cfg(Config* cfg);
    void initialize_queues();
    void initialize_uplink_buffers();
    void initialize_downlink_buffers();
    void free_uplink_buffers();
    void free_downlink_buffers();

    void save_demul_data_to_file(int frame_id, int data_subframe_id);
    void getDemulData(int** ptr, int* size);
    void getEqualData(float** ptr, int* size);

private:
    int BS_ANT_NUM, UE_NUM, PILOT_NUM;
    int OFDM_CA_NUM;
    int OFDM_DATA_NUM;
    int subframe_num_perframe;
    int dl_data_subframe_start, dl_data_subframe_end;
    bool downlink_mode;
    int packet_length;

    int TASK_THREAD_NUM, SOCKET_RX_THREAD_NUM, SOCKET_TX_THREAD_NUM;
    int FFT_THREAD_NUM, DEMUL_THREAD_NUM, ZF_THREAD_NUM;
    int CORE_OFFSET;
    int demul_block_num;
    int zf_block_size, zf_block_num;

    LDPCconfig LDPC_config;

    /* lookup table for 16 QAM, real and imag */
    size_t mod_type;
    float** qam16_table_;
    Config* cfg_;
    int max_equaled_frame = 0;
    // int max_packet_num_per_frame;
    std::unique_ptr<PacketTXRX> receiver_;
    Stats* stats_manager_;
    // std::unique_ptr<Stats> stats_manager_;
    // pthread_t task_threads[TASK_THREAD_NUM];
    // EventHandlerContext context[TASK_THREAD_NUM];
    pthread_t* task_threads;
    EventHandlerContext<Millipede>* context;
    /*****************************************************
     * Buffers
     *****************************************************/
    /* Uplink */
    /** 
     * received data 
     * Frist dimension: SOCKET_THREAD_NUM
     * Second dimension of buffer (type: char): packet_length * subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM
     * packet_length = sizeof(int) * 4 + sizeof(ushort) * OFDM_FRAME_LEN * 2;
     * Second dimension of buffer_status: subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM
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
     * First dimension: total subframe number in the buffer: data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM
     * second dimension: BS_ANT_NUM * OFDM_CA_NUM
     * second dimension data order: SC1-32 of ants, SC33-64 of ants, ..., SC993-1024 of ants (32 blocks each with 32 subcarriers)
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
     * First dimension: data_subframe_num_perframe (40-4) * TASK_BUFFER_FRAME_NUM
     * Second dimension: OFDM_CA_NUM * UE_NUM
     */
    Table<complex_float> equal_buffer_;

    /**
     * Data after demodulation
     * First dimension: data_subframe_num_perframe (40-4) * TASK_BUFFER_FRAME_NUM
     * Second dimension: OFDM_CA_NUM * UE_NUM
     */
    Table<uint8_t> demod_hard_buffer_;

    Table<int8_t> demod_soft_buffer_;

    Table<uint8_t> decoded_buffer_;

    RX_stats rx_stats_;
    FFT_stats fft_stats_;
    ZF_stats zf_stats_;
    Data_stats demul_stats_;
    Data_stats decode_stats_;
    Data_stats encode_stats_;
    Data_stats precode_stats_;
    Data_stats ifft_stats_;
    Data_stats tx_stats_;

    Table<int> delay_fft_queue;
    int* delay_fft_queue_cnt;

    /* Downlink */
    /** 
     * Raw data
     * First dimension: data_subframe_num_perframe * UE_NUM
     * Second dimension: OFDM_CA_NUM
     */
    Table<int8_t>* dl_IQ_data;
    Table<long long> dl_IQ_data_long;

    /** 
     * Modulated data
     * First dimension: subframe_num_perframe (40) * UE_NUM * TASK_BUFFER_FRAME_NUM
     * Second dimension: OFDM_CA_NUM
     */
    // RawDataBuffer dl_rawdata_buffer_;

    /**
     * Data for IFFT
     * First dimension: FFT_buffer_block_num = BS_ANT_NUM * data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM
     * Second dimension: OFDM_CA_NUM
     */
    Table<complex_float> dl_ifft_buffer_;

    /**
     * Data after IFFT
     * First dimension: data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM
     * second dimension: UE_NUM * OFDM_CA_NUM
     * second dimension data order: SC1-32 of UEs, SC33-64 of UEs, ..., SC993-1024 of UEs (32 blocks each with 32 subcarriers)
     */
    // DataBuffer dl_iffted_data_buffer_;

    Table<complex_float> dl_precoder_buffer_;
    Table<int8_t> dl_encoded_buffer_;

    /**
     * Data for transmission
     * First dimension of buffer (type: char): subframe_num_perframe * SOCKET_BUFFER_FRAME_NUM
     * Second dimension: packet_length * BS_ANT_NUM
     * packet_length = sizeof(int) * 4 + sizeof(ushort) * OFDM_FRAME_LEN * 2;
     * First dimension of buffer_status: subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM
     */
    char* dl_socket_buffer_;
    int* dl_socket_buffer_status_;
    long long dl_socket_buffer_size_;
    int dl_socket_buffer_status_size_;

    int* prev_frame_counter;
    int prev_frame_counter_max;
    /*****************************************************
     * Concurrent queues 
     *****************************************************/
    /* Uplink*/
    moodycamel::ConcurrentQueue<Event_data> fft_queue_;
    moodycamel::ConcurrentQueue<Event_data> zf_queue_;
    moodycamel::ConcurrentQueue<Event_data> demul_queue_;
    moodycamel::ConcurrentQueue<Event_data> decode_queue_;
    /* main thread message queue for data receiving */
    moodycamel::ConcurrentQueue<Event_data> message_queue_;
    /* main thread message queue for task completion*/
    moodycamel::ConcurrentQueue<Event_data> complete_task_queue_;

    /* Downlink*/
    moodycamel::ConcurrentQueue<Event_data> ifft_queue_;
    // moodycamel::ConcurrentQueue<Event_data> modulate_queue_;
    moodycamel::ConcurrentQueue<Event_data> encode_queue_;
    moodycamel::ConcurrentQueue<Event_data> precode_queue_;
    moodycamel::ConcurrentQueue<Event_data> tx_queue_;

    /* Tokens */
    moodycamel::ProducerToken** task_ptoks_ptr;
    moodycamel::ProducerToken** rx_ptoks_ptr;
    moodycamel::ProducerToken** tx_ptoks_ptr;
};

#endif
