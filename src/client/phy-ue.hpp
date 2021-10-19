#ifndef COMP_HEAD
#define COMP_HEAD
#include "buffer.hpp"
#include "comms-lib.h"
#include "concurrent_queue_wrapper.hpp"
#include "concurrentqueue.h"
#include "config.hpp"
#include "mac_thread.hpp"
#include "mkl_dfti.h"
#include "modulation.hpp"
#include "net.hpp"
#include "signalHandler.hpp"
#include "txrx_client.hpp"
#include <algorithm>
#include <armadillo>
#include <arpa/inet.h>
#include <ctime>
#include <fcntl.h>
#include <immintrin.h>
#include <iostream>
#include <memory>
#include <pthread.h>
#include <queue>
#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <system_error>
#include <tuple>
#include <unistd.h>

// typedef std::vector<complex_float> myVec;
typedef std::vector<complex_float,
    boost::alignment::aligned_allocator<complex_float, 64>>
    myVec;

using namespace arma;

class Phy_UE {
public:
    // dequeue bulk size, used to reduce the overhead of dequeue in main
    // thread
    static const int dequeue_bulk_size = 5;

    Phy_UE(Config* cfg);
    ~Phy_UE();

    void start();
    void stop();

    /*****************************************************
     * Downlink
     *****************************************************/
    void initialize_downlink_buffers();

    /**
     * modulate data from nUEs and does spatial multiplexing by applying
     * beamweights
     */
    void doEncode(int, size_t);
    void doModul(int, size_t);
    void doIFFT(int, size_t);

    /*****************************************************
     * Uplink
     *****************************************************/
    void initialize_uplink_buffers();

    /**
     * Do FFT task for one OFDM symbol
     * @param tid: task thread index, used for selecting muplans and task ptok
     * @param offset: offset of the OFDM symbol in rx_buffer_
     * Buffers: rx_buffer_, fft_buffer_, csi_buffer_, ul_data_buffer_
     *     Input buffer: rx_buffer_
     *     Output buffer: csi_buffer_ if symbol is pilot
     *                    ul_data_buffer_ if symbol is data
     *     Intermediate buffer: fft_buffer_ (FFT_inputs, FFT_outputs)
     * Offsets:
     *     rx_buffer_:
     *         dim1: socket thread index: (offset / # of OFDM symbols per
     * thread) dim2: OFDM symbol index in this socket thread (offset - # of
     * symbols in previous threads) FFT_inputs, FFT_outputs: dim1: frame index
     * * # of OFDM symbols per frame + symbol index * # of atennas + antenna
     * index dim2: subcarrier index csi_buffer_: dim1: frame index * FFT size +
     * subcarrier index in the current frame dim2: user index * # of antennas +
     * antenna index ul_data_buffer_: dim1: frame index * # of data symbols
     * per frame + data symbol index dim2: transpose block index * block size
     * * # of antennas + antenna index * block size Event offset: frame index *
     * # of symbol per frame + symbol index Description:
     *     1. copy received data (one OFDM symbol) from rx_buffer to
     * fft_buffer_.FFT_inputs (remove CP)
     *     2. perform FFT on fft_buffer_.FFT_inputs and store results in
     * fft_buffer_.FFT_outputs
     *     3. if symbol is pilot, do channel estimation from
     * fft_buffer_.FFT_outputs to csi_buffer_ if symbol is data, copy data
     * from fft_buffer_.FFT_outputs to ul_data_buffer_ and do block transpose
     *     4. add an event to the message queue to infrom main thread the
     * completion of this task
     */
    void doFFT(int, size_t);

    /**
     * Do demodulation task for a block of subcarriers (demul_block_size)
     * @param tid: task thread index, used for selecting spm_buffer and task
     * ptok
     * @param offset: offset of the first subcarrier in the block in
     * ul_data_buffer_ Buffers: ul_data_buffer_, spm_buffer_, precoder_buffer_,
     * equal_buffer_, demul_buffer_ Input buffer: ul_data_buffer_,
     * precoder_buffer_ Output buffer: demul_buffer_ Intermediate buffer:
     * spm_buffer, equal_buffer_ Offsets: ul_data_buffer_: dim1: frame index * #
     * of data symbols per frame + data symbol index dim2: transpose block
     * index * block size * # of antennas + antenna index * block size
     *     spm_buffer:
     *         dim1: task thread index
     *         dim2: antenna index
     *     precoder_buffer_:
     *         dim1: frame index * FFT size + subcarrier index in the current
     * frame equal_buffer_, demul_buffer: dim1: frame index * # of data
     * symbols per frame + data symbol index dim2: subcarrier index * # of
     * users Event offset: offset Description:
     *     1. for each subcarrier in the block, block-wisely copy data from
     * ul_data_buffer_ to spm_buffer_
     *     2. perform equalization with data and percoder matrixes
     *     3. perform demodulation on equalized data matrix
     *     4. add an event to the message queue to infrom main thread the
     * completion of this task
     */
    void doDemul(int, size_t);
    void doDecode(int, size_t);

    void getDemulData(long long** ptr, int* size);
    void getEqualPCData(float** ptr, int* size, int);
    void getEqualData(float** ptr, int* size, int);

    struct EventHandlerContext {
        Phy_UE* obj_ptr;
        int id;
    };

    // while loop of task thread
    static void* taskThread_launch(void* context);
    void taskThread(int tid);

    /* Add tasks into task queue based on event type */
    void schedule_task(EventData do_task,
        moodycamel::ConcurrentQueue<EventData>* in_queue,
        moodycamel::ProducerToken const& ptok);

    void initialize_vars_from_cfg(void);

private:
    Config* config_;
    size_t symbol_perframe;
    size_t ul_pilot_symbol_perframe;
    size_t dl_pilot_symbol_perframe;
    size_t ul_data_symbol_perframe;
    size_t dl_data_symbol_perframe;
    size_t ul_symbol_perframe;
    size_t dl_symbol_perframe;
    size_t tx_symbol_perframe;
    size_t symbol_len; // samples in sym without prefix and postfix
    size_t ofdm_syms; // number of OFDM symbols in general symbol (i.e. symbol)
    size_t FFT_LEN;
    size_t CP_LEN;
    size_t nUEs;
    size_t antenna_num;
    size_t hdr_size;
    size_t nCPUs;
    size_t core_offset;
    size_t worker_thread_num;
    size_t rx_thread_num;
    size_t tx_thread_num;
    size_t packet_length;
    size_t tx_packet_length;
    FILE *fp, *fd;

    size_t pilot_sc_len;
    size_t data_sc_len;
    size_t data_sc_start;
    size_t non_null_sc_len;

    size_t RX_BUFFER_FRAME_NUM;
    size_t TX_BUFFER_FRAME_NUM;

    MacThread* mac_thread_; // The thread running MAC layer functions
    std::thread mac_std_thread_; // Handle for the MAC thread

    // The frame ID of the next MAC packet we expect to receive from the MAC
    // thread
    size_t expected_frame_id_from_mac_ = 0;
    size_t current_frame_user_num_ = 0;

    // next_processed_frame_[i] is the next frame index on the uplink
    // to be processed and transmitted by the PHY for UE #i
    size_t next_frame_processed_[kMaxUEs] = {};

    /*****************************************************
     * Uplink
     *****************************************************/

    /**
     * Transmit data
     *
     * Number of transmit buffers (size = packet_length) and buffer status
     * entries: TX_THREAD_NUM * TX_BUFFER_FRAME_NUM * UE_NUM * DL_SYM_PER_FRAME
     */
    char* tx_buffer_;
    int* tx_buffer_status_;

    int tx_buffer_size;
    int tx_buffer_status_size;

    /**
     * Data for IFFT, (prefix added)
     * First dimension: IFFT_buffer_block_num = BS_ANT_NUM *
     * dl_data_symbol_perframe * TASK_BUFFER_FRAME_NUM Second dimension:
     * OFDM_CA_NUM
     */
    Table<complex_float> ifft_buffer_;
    DFTI_DESCRIPTOR_HANDLE mkl_handle;

    /**
     * Data before modulation
     * First dimension: data_symbol_num_perframe (40-4) *
     * TASK_BUFFER_FRAME_NUM Second dimension: OFDM_CA_NUM * UE_NUM
     */
    Table<uint8_t> ul_bits_buffer_;
    Table<uint8_t> ul_bits_buffer_status_;
    int ul_bits_buffer_size_;

    Table<uint8_t> ul_syms_buffer_;
    int ul_syms_buffer_size_;
    /**
     * Data after modulation
     * First dimension: data_symbol_num_perframe (40-4) *
     * TASK_BUFFER_FRAME_NUM Second dimension: OFDM_CA_NUM * UE_NUM
     */
    Table<complex_float> modul_buffer_;

    /*****************************************************
     * Downlink
     *****************************************************/

    std::unique_ptr<RadioTXRX> ru_;

    /**
     * Received data
     *
     * Number of RX buffers (size = packet_length) and buffer status
     * entries: RX_THREAD_NUM * RX_BUFFER_FRAME_NUM * BS_ANT_NUM *
     * symbol_num_perframe
     */
    Table<char> rx_buffer_;
    Table<int> rx_buffer_status_;

    int rx_buffer_size;
    int rx_buffer_status_size;

    /**
     * Data for FFT, after time sync (prefix removed)
     * First dimension: FFT_buffer_block_num = BS_ANT_NUM *
     * symbol_num_perframe * TASK_BUFFER_FRAME_NUM Second dimension:
     * OFDM_CA_NUM
     */
    Table<complex_float> fft_buffer_;

    /**
     * Estimated CSI data
     * First dimension: OFDM_CA_NUM * TASK_BUFFER_FRAME_NUM
     * Second dimension: BS_ANT_NUM * UE_NUM
     */
    std::vector<myVec> csi_buffer_;

    /**
     * Data after equalization
     * First dimension: data_symbol_num_perframe (40-4) *
     * TASK_BUFFER_FRAME_NUM Second dimension: OFDM_CA_NUM * UE_NUM
     */
    std::vector<myVec> equal_buffer_;

    /**
     * Data symbols after IFFT
     * First dimension: total symbol number in the buffer:
     * data_symbol_num_perframe * TASK_BUFFER_FRAME_NUM second dimension:
     * BS_ANT_NUM * OFDM_CA_NUM second dimension data order: SC1-32 of ants,
     * SC33-64 of ants, ..., SC993-1024 of ants (32 blocks each with 32
     * subcarriers)
     */
    Table<int8_t> dl_demod_buffer_;

    /**
     *
     */
    std::vector<std::vector<uint8_t>> dl_decode_buffer_;

    int16_t* resp_var_nodes;
    std::vector<std::complex<float>> pilot_sc_val_;
    std::vector<size_t> non_null_sc_ind_;
    std::vector<std::vector<std::complex<float>>> ue_pilot_vec;
    Table<size_t> decoded_bits_count_;
    Table<size_t> bit_error_count_;
    Table<size_t> decoded_blocks_count_;
    Table<size_t> block_error_count_;

    /* Concurrent queues */
    /* task queue for downlink FFT */
    moodycamel::ConcurrentQueue<EventData> fft_queue_;
    /* task queue for downlink demodulation */
    moodycamel::ConcurrentQueue<EventData> demul_queue_;
    /* task queue for downlink decoding */
    moodycamel::ConcurrentQueue<EventData> decode_queue_;
    /* main thread message queue */
    moodycamel::ConcurrentQueue<EventData> message_queue_;
    moodycamel::ConcurrentQueue<EventData> ifft_queue_;
    moodycamel::ConcurrentQueue<EventData> tx_queue_;
    moodycamel::ConcurrentQueue<EventData> to_mac_queue_;
    moodycamel::ConcurrentQueue<EventData> encode_queue_;
    moodycamel::ConcurrentQueue<EventData> modul_queue_;

    pthread_t task_threads[kMaxThreads];

    moodycamel::ProducerToken* rx_ptoks_ptr[kMaxThreads];
    moodycamel::ProducerToken* tx_ptoks_ptr[kMaxThreads];
    moodycamel::ProducerToken* mac_rx_ptoks_ptr[kMaxThreads];
    moodycamel::ProducerToken* mac_tx_ptoks_ptr[kMaxThreads];
    // moodycamel::ProducerToken* worker_ptoks_ptr[kMaxThreads];
    moodycamel::ProducerToken* task_ptok[kMaxThreads];

    // all checkers
    size_t csi_checker_[TASK_BUFFER_FRAME_NUM];
    size_t data_checker_[TASK_BUFFER_FRAME_NUM];

    // can possibly remove this checker
    size_t* demul_checker_[TASK_BUFFER_FRAME_NUM];
    size_t demul_status_[TASK_BUFFER_FRAME_NUM];

    size_t* demodul_checker_[TASK_BUFFER_FRAME_NUM];
    size_t demodul_status_[TASK_BUFFER_FRAME_NUM];

    size_t* decode_checker_[TASK_BUFFER_FRAME_NUM];
    size_t decode_status_[TASK_BUFFER_FRAME_NUM];

    std::queue<std::tuple<int, int>> taskWaitList;

    // for python
    /**
     * dimension: OFDM*UE_NUM
     */
    int max_equaled_frame = 0;
    // long long* demul_output;
    // float* equal_output;
    size_t record_frame = SIZE_MAX;
};
#endif
