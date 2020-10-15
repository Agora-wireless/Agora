#ifndef AGORA_HEAD
#define AGORA_HEAD

#include "buffer.hpp"
#include "concurrent_queue_wrapper.hpp"
#include "concurrentqueue.h"
#include "config.hpp"
#include "docoding.hpp"
#include "dodemul.hpp"
#include "dofft.hpp"
#include "doprecode.hpp"
#include "dozf.hpp"
#include "gettime.h"
#include "mac_thread.hpp"
#include "memory_manage.h"
#include "phy_stats.hpp"
#include "signalHandler.hpp"
#include "stats.hpp"
#include "txrx.hpp"
#include "utils.h"
#include <algorithm>
#include <iostream>
#include <memory>
#include <pthread.h>
#include <queue>
#include <signal.h>
#include <stdint.h>
#include <system_error>
#include <unistd.h>
#include <vector>

class Agora {
public:
    /* optimization parameters for block transpose (see the slides for more
     * details) */
    static const int transpose_block_num = 256;

    // Dequeue batch size, used to reduce the overhead of dequeue in main thread
    static const int kDequeueBulkSizeTXRX = 8;

    static const int kDequeueBulkSizeWorker = 4;

    Agora(Config*); /// Create an Agora object and start the worker threads
    ~Agora();

    void start(); /// The main Agora event loop
    void stop();

    void* worker_fft(int tid);
    void* worker_zf(int tid);
    void* worker_demul(int tid);
    void* worker(int tid);

    /* Launch threads to run worker with thread IDs tid_start to tid_end - 1 */
    void create_threads(void* (*worker)(void*), int tid_start, int tid_end);

    void handle_event_fft(size_t tag);
    void update_rx_counters(size_t frame_id, size_t symbol_id);
    void print_per_frame_done(PrintType print_type, size_t frame_id);
    void print_per_symbol_done(
        PrintType print_type, size_t frame_id, size_t symbol_id);
    void print_per_task_done(PrintType print_type, size_t frame_id,
        size_t symbol_id, size_t ant_or_sc_id);

    /// Update Agora's RAN config parameters
    void update_ran_config(RanConfig rc);

    void schedule_subcarriers(
        EventType task_type, size_t frame_id, size_t symbol_id);
    void schedule_antennas(
        EventType task_type, size_t frame_id, size_t symbol_id);

    /**
     * @brief Schedule LDPC decoding or encoding over code blocks
     * @param task_type Either LDPC decoding or LDPC encoding
     * @param frame_id The monotonically increasing frame ID
     * @param symbol_idx The index of the symbol among uplink symbols for LDPC
     * decoding, and among downlink symbols for LDPC encoding
     */
    void schedule_codeblocks(
        EventType task_type, size_t frame_id, size_t symbol_idx);

    void schedule_users(EventType task_type, size_t frame_id, size_t symbol_id);

    // Send current frame's SNR measurements from PHY to MAC
    void send_snr_report(
        EventType event_type, size_t frame_id, size_t symbol_id);

    void move_events_between_queues(
        EventType event_type1, EventType event_type2);

    void initialize_queues();
    void initialize_uplink_buffers();
    void initialize_downlink_buffers();
    void free_uplink_buffers();
    void free_downlink_buffers();

    void save_decode_data_to_file(int frame_id);
    void save_tx_data_to_file(int frame_id);
    void getEqualData(float** ptr, int* size);

    // Flags that allow developer control over Agora internals
    struct {
        // Before exiting, save LDPC-decoded or demodulated data to a file
        bool enable_save_decode_data_to_file = false;

        // Before exiting, save data sent on downlink to a file
        bool enable_save_tx_data_to_file = false;
    } flags;

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
        for (size_t i = 0; i < kFrameWnd; i++) {
            ret << std::to_string(fft_queue_arr[i].size()) << " ";
        }
        ret << "]";
        return ret.str();
    }

    const double freq_ghz; // RDTSC frequency in GHz

    // Worker thread i runs on core base_worker_core_offset + i
    const size_t base_worker_core_offset;

    Config* config_;
    size_t fft_created_count;
    int max_equaled_frame = 0;
    std::unique_ptr<PacketTXRX> packet_tx_rx_;

    MacThread* mac_thread_; // The thread running MAC layer functions
    std::thread mac_std_thread_; // Handle for the MAC thread

    Stats* stats;
    PhyStats* phy_stats;
    pthread_t* task_threads;

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

    // Preliminary CSI buffers. Each buffer has [number of antennas] rows and
    // [number of OFDM data subcarriers] columns.
    PtrGrid<kFrameWnd, kMaxUEs, complex_float> csi_buffers_;

    // Data symbols after FFT
    // 1st dimension: kFrameWnd * uplink data symbols per frame
    // 2nd dimension: number of antennas * number of OFDM data subcarriers
    //
    // 2nd dimension data order: 32 blocks each with 32 subcarriers each:
    // subcarrier 1 -- 32 of antennas, subcarrier 33 -- 64 of antennas, ...,
    // subcarrier 993 -- 1024 of antennas.
    Table<complex_float> data_buffer_;

    // Calculated uplink zeroforcing detection matrices. Each matrix has
    // [number of antennas] rows and [number of UEs] columns.
    PtrGrid<kFrameWnd, kMaxDataSCs, complex_float> ul_zf_matrices_;

    // Data after equalization
    // 1st dimension: kFrameWnd * uplink data symbols per frame
    // 2nd dimension: number of OFDM data subcarriers * number of UEs
    Table<complex_float> equal_buffer_;

    // Data after demodulation. Each buffer has kMaxModType * number of OFDM
    // data subcarriers
    PtrGrid<kFrameWnd, kMaxUEs, int8_t> demod_buffers_;

    // Data after LDPC decoding. Each buffer has number of code blocks per UE
    // * number of btyes of info bits per block.
    PtrGrid<kFrameWnd, kMaxUEs, uint8_t> decoded_buffer_;

    Table<complex_float> ue_spec_pilot_buffer_;

    RxCounters rx_counters_;
    FFT_stats fft_stats_;
    ZF_stats zf_stats_;
    RC_stats rc_stats_;
    Data_stats demul_stats_;
    Data_stats decode_stats_;
    Encode_stats encode_stats_;
    Data_stats precode_stats_;
    Data_stats ifft_stats_;
    Data_stats tx_stats_;
    Data_stats tomac_stats_;
    Data_stats frommac_stats_;

    // Per-frame queues of delayed FFT tasks. The queue contains offsets into
    // TX/RX buffers.
    std::array<std::queue<fft_req_tag_t>, kFrameWnd> fft_queue_arr;

    // Data for IFFT
    // 1st dimension: kFrameWnd * number of antennas * number of
    // data symbols per frame
    // 2nd dimension: number of OFDM carriers (including non-data carriers)
    Table<complex_float> dl_ifft_buffer_;

    // Calculated uplink zeroforcing detection matrices. Each matrix has
    // [number of UEs] rows and [number of antennas] columns.
    PtrGrid<kFrameWnd, kMaxDataSCs, complex_float> dl_zf_matrices_;

    // 1st dimension: kFrameWnd
    // 2nd dimension: number of OFDM data subcarriers * number of antennas
    Table<complex_float> calib_buffer_;

    // Data after LDPC encoding and bit adaption for modulation input.
    // Each buffer has number of subcarriers btyes (rounded up to 64 btyes)
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t> dl_encoded_buffer_;

    // 1st dimension: kFrameWnd * number of DL data symbols per frame
    // 2nd dimension: number of OFDM data subcarriers * number of UEs
    Table<uint8_t> dl_bits_buffer_;

    // 1st dimension: number of UEs
    // 2nd dimension: number of OFDM data subcarriers * kFrameWnd
    //                * number of DL data symbols per frame
    // Use different dimensions from dl_bits_buffer_ to avoid cache false sharing
    Table<uint8_t> dl_bits_buffer_status_;

    /**
     * Data for transmission
     *
     * Number of downlink socket buffers and status entries:
     * kFrameWnd * symbol_num_perframe * BS_ANT_NUM
     *
     * Size of each downlink socket buffer entry: packet_length bytes
     * Size of each downlink socket buffer status entry: one integer
     */
    char* dl_socket_buffer_;
    int* dl_socket_buffer_status_;

    struct sched_info_t {
        moodycamel::ConcurrentQueue<Event_data> concurrent_q;
        moodycamel::ProducerToken* ptok;
    };
    sched_info_t sched_info_arr[kNumEventTypes];

    // Master thread's message queue for receiving packets
    moodycamel::ConcurrentQueue<Event_data> message_queue_;

    // Master-to-worker queue for MAC
    moodycamel::ConcurrentQueue<Event_data> mac_request_queue_;

    // Worker-to-master queue for MAC
    moodycamel::ConcurrentQueue<Event_data> mac_response_queue_;

    // Master thread's message queue for event completion from Doers;
    moodycamel::ConcurrentQueue<Event_data> complete_task_queue_;

    // Master thread's message queue for event completion from DoDecode;
    moodycamel::ConcurrentQueue<Event_data> complete_decode_task_queue_;

    moodycamel::ProducerToken* rx_ptoks_ptr[kMaxThreads];
    moodycamel::ProducerToken* tx_ptoks_ptr[kMaxThreads];
    moodycamel::ProducerToken* worker_ptoks_ptr[kMaxThreads];
    moodycamel::ProducerToken* decode_ptoks_ptr[kMaxThreads];
};

#endif
