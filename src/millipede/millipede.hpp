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

    void handle_event_fft(size_t tag);
    void update_rx_counters(size_t frame_id, size_t symbol_id);
    void print_per_frame_done(size_t task_type, size_t frame_id);
    void print_per_symbol_done(
        size_t task_type, size_t frame_id, size_t symbol_id);
    void print_per_task_done(size_t task_type, size_t frame_id,
        size_t symbol_id, size_t ant_or_sc_id);

    void schedule_subcarriers(
        EventType task_type, size_t frame_id, size_t symbol_id);
    void schedule_users(EventType task_type, gen_tag_t base_tag);
    void schedule_antennas(
        EventType task_type, size_t frame_id, size_t symbol_id);

    void initialize_queues();
    void initialize_uplink_buffers();
    void initialize_downlink_buffers();
    void free_uplink_buffers();
    void free_downlink_buffers();

    void save_demul_data_to_file(int frame_id);
    void save_decode_data_to_file(int frame_id);
    void save_tx_data_to_file(int frame_id);
    void getDemulData(int** ptr, int* size);
    void getEqualData(float** ptr, int* size);

private:
    /// Fetch the concurrent queue for this event type
    moodycamel::ConcurrentQueue<Event_data>* get_conq(EventType event_type)
    {
        return &sched_info_arr[static_cast<size_t>(event_type)].concurrent_q;
    }

    /// Fetch the producer token for this event type
    moodycamel::ProducerToken* get_ptok(EventType event_type)
    {
        return sched_info_arr[static_cast<size_t>(event_type)].ptok;
    }

    /// Return a string containing the sizes of the FFT queues
    std::string get_fft_queue_sizes_string() const
    {
        std::ostringstream ret;
        ret << "[";
        for (size_t i = 0; i < TASK_BUFFER_FRAME_NUM; i++) {
            ret << std::to_string(fft_queue_arr[i].size()) << " ";
        }
        ret << "]";
        return ret.str();
    }

    /* lookup table for 16 QAM, real and imag */
    float** qam16_table_;
    Config* config_;
    size_t fft_created_count;
    int max_equaled_frame = 0;
    // int max_packet_num_per_frame;
    std::unique_ptr<PacketTXRX> receiver_;
    Stats* stats;
    // std::unique_ptr<Stats> stats_manager_;
    // pthread_t task_threads[TASK_THREAD_NUM];
    // EventHandlerContext context[TASK_THREAD_NUM];
    pthread_t* task_threads;
    double freq_ghz = -1.0; // RDTSC frequency in GHz

    /*****************************************************
     * Buffers
     *****************************************************/

    /* Uplink */
    size_t socket_buffer_size_; // RX buffer size per socket RX thread

    // Max number of packets that can be buffered in one RX thread
    size_t socket_buffer_status_size_;

    // Received data buffers
    // 1st dimension: number of socket RX threads
    // 2nd dimension: socket buffer size
    Table<char> socket_buffer_;

    // Status of received data buffers
    // 1st dimension: number of socket RX threads
    // 2nd dimension: socket buffer status size
    Table<int> socket_buffer_status_;

    // Estimated CSI data
    // 1st dimension: TASK_BUFFER_FRAME_NUM * pilots per frame
    // 2nd dimension: number of antennas * number of OFDM data subcarriers
    Table<complex_float> csi_buffer_;

    // Data symbols after FFT
    // 1st dimension: TASK_BUFFER_FRAME_NUM * uplink data symbols per frame
    // 2nd dimension: number of antennas * number of OFDM data subcarriers
    //
    // 2nd dimension data order: 32 blocks each with 32 subcarriers each:
    // subcarrier 1 -- 32 of antennas, subcarrier 33 -- 64 of antennas, ...,
    // subcarrier 993 -- 1024 of antennas.
    Table<complex_float> data_buffer_;

    // Calculated precoder
    // 1st dimension: TASK_BUFFER_FRAME_NUM * number of OFDM data subcarriers
    // 2nd dimension: number of antennas * number of UEs
    Table<complex_float> ul_precoder_buffer_;

    // Data after equalization
    // 1st dimension: TASK_BUFFER_FRAME_NUM * uplink data symbols per frame
    // 2nd dimension: number of OFDM data subcarriers * number of UEs
    Table<complex_float> equal_buffer_;

    // Data after hard demodulation
    // 1st dimension: TASK_BUFFER_FRAME_NUM * uplink data symbols per frame
    // 2nd dimension: number of OFDM data subcarriers * number of UEs
    Table<uint8_t> demod_hard_buffer_;

    // Data after soft demodulation
    // 1st dimension: TASK_BUFFER_FRAME_NUM * uplink data symbols per frame
    // 2nd dimension: number of OFDM data subcarriers * number of UEs
    Table<int8_t> demod_soft_buffer_;

    // Data after LDPC decoding
    // 1st dimension: TASK_BUFFER_FRAME_NUM * uplink data symbols per frame
    // 2nd dimension: decoded bytes per UE * number of UEs
    Table<uint8_t> decoded_buffer_;

    Table<complex_float> ue_spec_pilot_buffer_;

    RX_stats rx_stats_;
    FFT_stats fft_stats_;
    ZF_stats zf_stats_;
    RC_stats rc_stats_;
    Data_stats demul_stats_;
    Data_stats decode_stats_; // LDPC-only
    Data_stats encode_stats_; // LDPC-only
    Data_stats precode_stats_;
    Data_stats ifft_stats_;
    Data_stats tx_stats_;

    // Per-frame queues of delayed FFT tasks. The queue contains offsets into
    // TX/RX buffers.
    std::array<std::queue<fft_req_tag_t>, TASK_BUFFER_FRAME_NUM> fft_queue_arr;

    // Data for IFFT
    // 1st dimension: TASK_BUFFER_FRAME_NUM * number of antennas * number of
    // data symbols per frame
    // 2nd dimension: number of OFDM carriers (including non-data carriers)
    Table<complex_float> dl_ifft_buffer_;

    // 1st dimension: TASK_BUFFER_FRAME_NUM * number of OFDM data subcarriers
    // 2nd dimension: number of antennas * number of UEs
    Table<complex_float> dl_precoder_buffer_;

    // 1st dimension: TASK_BUFFER_FRAME_NUM
    // 2nd dimension: number of OFDM data subcarriers * number of antennas
    Table<complex_float> recip_buffer_;

    // 1st dimension: TASK_BUFFER_FRAME_NUM
    // 2nd dimension: number of OFDM data subcarriers * number of antennas
    Table<complex_float> calib_buffer_;

    // 1st dimension: TASK_BUFFER_FRAME_NUM * number of data symbols per frame
    // 2nd dimension: number of OFDM data subcarriers * number of UEs
    Table<int8_t> dl_encoded_buffer_;

    /**
     * Data for transmission
     * First dimension of buffer (type: char): symbol_num_perframe *
     * SOCKET_BUFFER_FRAME_NUM Second dimension: packet_length * BS_ANT_NUM
     * packet_length = sizeof(int) * 4 + sizeof(ushort) * OFDM_FRAME_LEN * 2;
     * First dimension of buffer_status: symbol_num_perframe * BS_ANT_NUM *
     * SOCKET_BUFFER_FRAME_NUM
     */
    char* dl_socket_buffer_;
    int* dl_socket_buffer_status_;
    long long dl_socket_buffer_size_;
    int dl_socket_buffer_status_size_;

    struct sched_info_t {
        moodycamel::ConcurrentQueue<Event_data> concurrent_q;
        moodycamel::ProducerToken* ptok;
    };
    sched_info_t sched_info_arr[kNumEventTypes];

    // Master thread's message queue for receiving packets
    moodycamel::ConcurrentQueue<Event_data> message_queue_;

    // Master thread's message queue for event completion from Doers;
    moodycamel::ConcurrentQueue<Event_data> complete_task_queue_;

    moodycamel::ProducerToken* rx_ptoks_ptr[kMaxThreads];
    moodycamel::ProducerToken* tx_ptoks_ptr[kMaxThreads];
    moodycamel::ProducerToken* worker_ptoks_ptr[kMaxThreads];
};

#endif
