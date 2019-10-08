#ifndef COMP_HEAD
#define COMP_HEAD
#include "ru.hpp"
//#include "l2.hpp"
#include <unistd.h>
#include <memory>
#include <iostream>
#include <sys/epoll.h>
#include <fcntl.h>
#include <system_error>
#include <pthread.h>
#include <queue>
#include <tuple>
#include <immintrin.h>
#include <ctime>
#include <algorithm>
#include "buffer.hpp"
#include "concurrentqueue.h"
#include "modulation.hpp"
#include "signalHandler.hpp"
#include "comms-lib.h"
#include <armadillo>
#include "config.hpp"
#include "offset.h"
//#include "mufft/fft.h"

//typedef std::vector<complex_float> myVec;
typedef std::vector<complex_float, boost::alignment::aligned_allocator<complex_float, 64>> myVec;

class Phy_UE
{
public:
    /* TASK & SOCKET thread number */
    #ifdef SIM
        static const int RX_THREAD_NUM = 1;
        static const int TASK_THREAD_NUM = 8;
    #else
        static const int RX_THREAD_NUM = 2; // Set this RADIO_NUM
        static const int TASK_THREAD_NUM = 12;
    #endif 
    static const int TX_THREAD_NUM = 1;
    static const int L2_THREAD_NUM = 1;
    // defined by protocol usually
    // buffer length of downlink which is synced to uplink 
    #ifdef SIM
    static const int TX_RX_FRAME_OFFSET = 2;
    #else
    static const int TX_RX_FRAME_OFFSET = 12;
    #endif 
    // static const int TX_THREAD_NUM = ENABLE_DOWNLINK ? 7 : 0;
    // buffer length of each socket thread
    // the actual length will be RX_BUFFER_FRAME_NUM
    // * subframe_num_perframe * BS_ANT_NUM
    //static const int RX_BUFFER_FRAME_NUM = 80;
    //static const int TX_BUFFER_FRAME_NUM = 80;
    // buffer length of computation part (for FFT/CSI/ZF/DEMUL buffers)
    //static const int TASK_BUFFER_FRAME_NUM = 60;
    // optimization parameters for block transpose (see the slides for more
    // details)
    static const int transpose_block_size = 64;
    // do demul_block_size sub-carriers in each task
    //static const int demul_block_size = OFDM_CA_NUM*2/transpose_block_size;
    // dequeue bulk size, used to reduce the overhead of dequeue in main
    // thread
    static const int dequeue_bulk_size = 5;

    Phy_UE(Config *cfg);
    ~Phy_UE();

    void start();
    void stop();

    /*****************************************************
     * Downlink 
     *****************************************************/ 
    
    /**
     * modulate data from nUEs and does spatial multiplexing by applying beamweights
     */
    void doTransmit(int tid, int offset, int frame);

    /*****************************************************
     * Uplink 
     *****************************************************/ 
   
    /**
     * Do FFT task for one OFDM symbol 
     * @param tid: task thread index, used for selecting muplans and task ptok
     * @param offset: offset of the OFDM symbol in rx_buffer_
     * Buffers: rx_buffer_, fft_buffer_, csi_buffer_, ul_data_buffer_ 
     *     Input buffer: rx_buffer_
     *     Output buffer: csi_buffer_ if subframe is pilot
     *                    ul_data_buffer_ if subframe is data
     *     Intermediate buffer: fft_buffer_ (FFT_inputs, FFT_outputs)
     * Offsets: 
     *     rx_buffer_: 
     *         dim1: socket thread index: (offset / # of OFDM symbols per thread)
     *         dim2: OFDM symbol index in this socket thread (offset - # of subframes in previous threads)
     *     FFT_inputs, FFT_outputs: 
     *         dim1: frame index * # of OFDM symbols per frame + subframe index * # of atennas + antenna index
     *         dim2: subcarrier index
     *     csi_buffer_: 
     *         dim1: frame index * FFT size + subcarrier index in the current frame
     *         dim2: user index * # of antennas + antenna index
     *     ul_data_buffer_: 
     *         dim1: frame index * # of data subframes per frame + data subframe index
     *         dim2: transpose block index * block size * # of antennas + antenna index * block size
     * Event offset: frame index * # of subframe per frame + subframe index
     * Description: 
     *     1. copy received data (one OFDM symbol) from rx_buffer to fft_buffer_.FFT_inputs (remove CP)
     *     2. perform FFT on fft_buffer_.FFT_inputs and store results in fft_buffer_.FFT_outputs
     *     3. if subframe is pilot, do channel estimation from fft_buffer_.FFT_outputs to csi_buffer_
     *        if subframe is data, copy data from fft_buffer_.FFT_outputs to ul_data_buffer_ and do block transpose     
     *     4. add an event to the message queue to infrom main thread the completion of this task
     */
    void doFFT(int tid, int offset);


    /**
     * Do demodulation task for a block of subcarriers (demul_block_size)
     * @param tid: task thread index, used for selecting spm_buffer and task ptok
     * @param offset: offset of the first subcarrier in the block in ul_data_buffer_
     * Buffers: ul_data_buffer_, spm_buffer_, precoder_buffer_, equal_buffer_, demul_buffer_
     *     Input buffer: ul_data_buffer_, precoder_buffer_
     *     Output buffer: demul_buffer_
     *     Intermediate buffer: spm_buffer, equal_buffer_
     * Offsets: 
     *     ul_data_buffer_: 
     *         dim1: frame index * # of data subframes per frame + data subframe index
     *         dim2: transpose block index * block size * # of antennas + antenna index * block size
     *     spm_buffer: 
     *         dim1: task thread index
     *         dim2: antenna index
     *     precoder_buffer_: 
     *         dim1: frame index * FFT size + subcarrier index in the current frame
     *     equal_buffer_, demul_buffer: 
     *         dim1: frame index * # of data subframes per frame + data subframe index
     *         dim2: subcarrier index * # of users
     * Event offset: offset
     * Description: 
     *     1. for each subcarrier in the block, block-wisely copy data from ul_data_buffer_ to spm_buffer_
     *     2. perform equalization with data and percoder matrixes
     *     3. perform demodulation on equalized data matrix   
     *     4. add an event to the message queue to infrom main thread the completion of this task
     */
    void doDemul(int tid, int offset);


    void getDemulData(long long **ptr, int *size);
    void getEqualPCData(float **ptr, int *size, int);
    void getEqualData(float **ptr, int *size, int);

    struct EventHandlerContext
    {
        Phy_UE *obj_ptr;
        int id;
    };


    // while loop of task thread
    static void* taskThread(void* context);

    /* Add tasks into task queue based on event type */
    void schedule_task(Event_data do_task, moodycamel::ConcurrentQueue<Event_data> * in_queue, moodycamel::ProducerToken const& ptok);

    void initialize_vars_from_cfg(Config *cfg);
private:
    Config *cfg;
    size_t symbol_perframe;
    size_t ul_pilot_symbol_perframe;
    size_t dl_pilot_symbol_perframe;
    // static const int empty_subframe_num_perframe;
    size_t ul_data_symbol_perframe;
    size_t dl_data_symbol_perframe;
    size_t dl_symbol_perframe;
    size_t rx_symbol_perframe;
    size_t tx_symbol_perframe;
    size_t symbol_len; // samples in sym without prefix and postfix
    size_t dl_prefix_len;
    size_t prefix_len;
    size_t postfix_len;
    size_t ofdm_syms; // number of OFDM symbols in general symbol (i.e. subframe)
    size_t FFT_LEN;
    size_t CP_LEN;
    size_t nUEs;
    size_t numAntennas;
    size_t hdr_size;
    size_t nCPUs;
    size_t core_offset;
    size_t rx_thread_num;
    size_t packet_length;
    size_t tx_packet_length;
    size_t packet_header_offset;
    FILE *fp, *fd;
    std::vector<myVec> L2_data_aligned;
    float *pilots_;
    complex_float *ul_pilot;
    char* ul_pilot_aligned;
    int **ul_IQ_data;
    complex_float **ul_IQ_modul;


    int pilot_sc_len;
    int data_sc_len;
    int data_sc_start;
    int non_null_sc_len;

    size_t RX_BUFFER_FRAME_NUM;
    size_t TX_BUFFER_FRAME_NUM;

    /*****************************************************
     * Uplink 
     *****************************************************/ 
    
    //std::unique_ptr<L2> l2_;
    
    /** 
     * transmit data 
     * Frist dimension: TX_THREAD_NUM
     * Second dimension of buffer (type: uchar): packet_length * UE_NUM * DL_SYM_PER_FRAME * TX_BUFFER_FRAME_NUM
     * packet_length = sizeof(int) * 4 + sizeof(uchar) * OFDM_FRAME_LEN;
     * Second dimension of buffer_status: DL_SYM_PER_FRAME * UE_NUM * TX_BUFFER_FRAME_NUM
     */
    char *tx_buffer_;
    int *tx_buffer_status_;
    int tx_buffer_size;
    int tx_buffer_status_size;

    /** 
     * Data for IFFT, (prefix added)
     * First dimension: IFFT_buffer_block_num = BS_ANT_NUM * dl_data_symbol_perframe * TASK_BUFFER_FRAME_NUM
     * Second dimension: OFDM_CA_NUM
     */
    IFFTBuffer ifft_buffer_;

    /**
     * Data before modulation
     * First dimension: data_subframe_num_perframe (40-4) * TASK_BUFFER_FRAME_NUM
     * Second dimension: OFDM_CA_NUM * UE_NUM
     */
    std::vector<complex_float> l2_data_buffer_;
    std::vector<int> l2_buffer_status_;

    /**
     * Data after modulation
     * First dimension: data_subframe_num_perframe (40-4) * TASK_BUFFER_FRAME_NUM
     * Second dimension: OFDM_CA_NUM * UE_NUM
     */
    std::vector<myVec> modul_buffer_;

    mufft_plan_1d *muifftplans_[TASK_THREAD_NUM];


    /*****************************************************
     * Downlink 
     *****************************************************/ 
    
    std::unique_ptr<RU> ru_;
    
    /** 
     * received data 
     * Frist dimension: RX_THREAD_NUM
     * Second dimension of buffer (type: char): packet_length * subframe_num_perframe * BS_ANT_NUM * RX_BUFFER_FRAME_NUM
     * packet_length = sizeof(int) * 4 + sizeof(ushort) * OFDM_FRAME_LEN * 2;
     * Second dimension of buffer_status: subframe_num_perframe * BS_ANT_NUM * RX_BUFFER_FRAME_NUM
     */
    char **rx_buffer_;
    int **rx_buffer_status_;
    int rx_buffer_size;
    int rx_buffer_status_size;

    /** 
     * Data for FFT, after time sync (prefix removed)
     * First dimension: FFT_buffer_block_num = BS_ANT_NUM * subframe_num_perframe * TASK_BUFFER_FRAME_NUM
     * Second dimension: OFDM_CA_NUM
     */
    FFTBuffer fft_buffer_;

    /** 
     * Estimated CSI data 
     * First dimension: OFDM_CA_NUM * TASK_BUFFER_FRAME_NUM
     * Second dimension: BS_ANT_NUM * UE_NUM
     */
    std::vector<myVec> csi_buffer_;

    /** 
     * Data symbols after IFFT
     * First dimension: total subframe number in the buffer: data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM
     * second dimension: BS_ANT_NUM * OFDM_CA_NUM
     * second dimension data order: SC1-32 of ants, SC33-64 of ants, ..., SC993-1024 of ants (32 blocks each with 32 subcarriers)
     */
    std::vector<myVec> dl_data_buffer_;

    /**
     * Data after equalization
     * First dimension: data_subframe_num_perframe (40-4) * TASK_BUFFER_FRAME_NUM
     * Second dimension: OFDM_CA_NUM * UE_NUM
     */
    std::vector<myVec> equal_buffer_;

    /**
     * Data after phase correction
     * First dimension: data_subframe_num_perframe (40-4) * TASK_BUFFER_FRAME_NUM
     * Second dimension: DATA_CA_NUM * UE_NUM
     */
    std::vector<myVec> equal_pc_buffer_;


    std::vector<std::complex<float>> pilot_sc_val_;
    std::vector<int> data_sc_ind_;
    std::vector<int> pilot_sc_ind_;
    std::vector<int> non_null_sc_ind_;

    mufft_plan_1d *mufftplans_[TASK_THREAD_NUM];

    /* Concurrent queues */
    /* task queue for uplink FFT */
    moodycamel::ConcurrentQueue<Event_data> task_queue_;// = moodycamel::ConcurrentQueue<Event_data>(RX_BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM  * 36);
    /* task queue for uplink demodulation */
    moodycamel::ConcurrentQueue<Event_data> demul_queue_;// = moodycamel::ConcurrentQueue<Event_data>(RX_BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM  * 36);
    /* main thread message queue */
    moodycamel::ConcurrentQueue<Event_data> message_queue_;// = moodycamel::ConcurrentQueue<Event_data>(RX_BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM  * 36);
    moodycamel::ConcurrentQueue<Event_data> ifft_queue_;
    moodycamel::ConcurrentQueue<Event_data> tx_queue_;

    pthread_t task_threads[TASK_THREAD_NUM];

    EventHandlerContext context[TASK_THREAD_NUM];

    // all checkers
    // int cropper_checker_[subframe_num_perframe * TASK_BUFFER_FRAME_NUM];
    size_t* cropper_checker_;
    size_t csi_checker_[TASK_BUFFER_FRAME_NUM];
    size_t data_checker_[TASK_BUFFER_FRAME_NUM];

    size_t precoder_checker_[TASK_BUFFER_FRAME_NUM];
    bool precoder_status_[TASK_BUFFER_FRAME_NUM];

    size_t cropper_created_checker_[TASK_BUFFER_FRAME_NUM];

    // can possibly remove this checker
    // int demul_checker_[TASK_BUFFER_FRAME_NUM][(subframe_num_perframe - UE_NUM)];
    size_t* demul_checker_[TASK_BUFFER_FRAME_NUM];
    size_t demul_status_[TASK_BUFFER_FRAME_NUM];

    size_t* demodul_checker_[TASK_BUFFER_FRAME_NUM];
    size_t demodul_status_[TASK_BUFFER_FRAME_NUM];

    std::queue<std::tuple<int, int>> taskWaitList;

    int max_queue_delay = 0;

    int debug_count = 0;

    std::unique_ptr<moodycamel::ProducerToken> task_ptok[TASK_THREAD_NUM];

    /* lookup table for 16 QAM, real and imag */
    float qam16_table[2][16];

    // for python
    /**
     * dimension: OFDM*UE_NUM
     */ 
    int max_equaled_frame=0;
    // long long* demul_output;
    // float* equal_output;
};
#endif
