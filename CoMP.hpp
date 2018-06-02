/**
 * Author: Peiyao Zhao
 * E-Mail: pdszpy19930218@163.com
 * 
 */

#ifndef COMP_HEAD
#define COMP_HEAD

#include "packageReceiver.hpp"
#include "packageSenderBS.hpp"
#include <unistd.h>
#include <memory>
#include <iostream>

#include <fcntl.h>
#include <system_error>
#include <pthread.h>
#include <queue>
#include "mufft/fft.h"
#include <complex.h>
#include <math.h>
#include <tuple>
#include "cpu_attach.hpp"
#include <armadillo>
#include <immintrin.h>
#include "buffer.hpp"
#include "concurrentqueue.h"


class CoMP
{
public:
    // TASK & SOCKET thread number 
    static const int TASK_THREAD_NUM = ENABLE_DOWNLINK ? 1 : 28;
    static const int SOCKET_RX_THREAD_NUM = ENABLE_DOWNLINK ? 4 : 7;
    static const int SOCKET_TX_THREAD_NUM = ENABLE_DOWNLINK ? 1 : 0;
    // buffer length of each socket thread
    // the actual length will be SOCKET_BUFFER_FRAME_NUM
    // * subframe_num_perframe * BS_ANT_NUM
    static const int SOCKET_BUFFER_FRAME_NUM = 120;
    // buffer length of computation part (for FFT/CSI/ZF/DEMUL buffers)
    static const int TASK_BUFFER_FRAME_NUM = 60;
    // do demul_block_size sub-carriers in each task
    static const int demul_block_size = 32;
    // optimization parameters for block transpose (see the slides for more
    // details)
    static const int transpose_block_size = 64;
    // dequeue bulk size, used to reduce the overhead of dequeue in main
    // thread
    static const int dequeue_bulk_size = 5;

    CoMP();
    ~CoMP();

    void start();
    // while loop of task thread
    static void *taskThread(void *context);

    /*****************************************************
     * Uplink 
     *****************************************************/ 
   
    /**
     * Do FFT task for one OFDM symbol 
     * @param tid: task thread index, used for selecting muplans and task ptok
     * @param offset: offset of the OFDM symbol in socket_buffer_
     * Buffers: socket_buffer_, fft_buffer_, csi_buffer_, data_buffer_ 
     *     Input buffer: socket_buffer_
     *     Output buffer: csi_buffer_ if subframe is pilot
     *                    data_buffer_ if subframe is data
     *     Intermediate buffer: fft_buffer_ (FFT_inputs, FFT_outputs)
     * Offsets: 
     *     socket_buffer_: 
     *         dim1: socket thread index: (offset / # of OFDM symbols per thread)
     *         dim2: OFDM symbol index in this socket thread (offset - # of subframes in previous threads)
     *     FFT_inputs, FFT_outputs: 
     *         dim1: frame index * # of OFDM symbols per frame + subframe index * # of atennas + antenna index
     *         dim2: subcarrier index
     *     csi_buffer_: 
     *         dim1: frame index * FFT size + subcarrier index in the current frame
     *         dim2: user index * # of antennas + antenna index
     *     data_buffer_: 
     *         dim1: frame index * # of data subframes per frame + data subframe index
     *         dim2: transpose block index * block size * # of antennas + antenna index * block size
     * Event offset: frame index * # of subframe per frame + subframe index
     * Description: 
     *     1. copy received data (one OFDM symbol) from socket_buffer to fft_buffer_.FFT_inputs (remove CP)
     *     2. perform FFT on fft_buffer_.FFT_inputs and store results in fft_buffer_.FFT_outputs
     *     3. if subframe is pilot, do channel estimation from fft_buffer_.FFT_outputs to csi_buffer_
     *        if subframe is data, copy data from fft_buffer_.FFT_outputs to data_buffer_ and do block transpose     
     *     4. add an event to the message queue to infrom main thread the completion of this task
     */
    void doCrop(int tid, int offset);


    /**
     * Do ZF task for one subcarrier with all pilots in a frame
     * @param tid: task thread index, used for selecting task ptok
     * @param offset: offset of the subcarrier in csi_buffer_
     * Buffers: csi_buffer_, precoder_buffer_
     *     Input buffer: csi_buffer_
     *     Output buffer: precoder_buffer_
     * Offsets:
     *     csi_buffer_, precoder_buffer_: 
     *         dim1: frame index * FFT size + subcarrier index in the current frame
     * Event offset: offset
     * Description:
     *     1. perform pseudo-inverse (pinv) on csi_buffer_ and store results in precoder_buffer_  
     *     2. add an event to the message queue to infrom main thread the completion of this task
     */
    void doZF(int tid, int offset);


    /**
     * Do prediction task for one subcarrier 
     * @param tid: task thread index, used for selecting task ptok
     * @param offset: offset of the subcarrier in csi_buffer_
     * Buffers: csi_buffer_, pred_csi_buffer_, precoder_buffer_
     *     Input buffer: csi_buffer_
     *     Output buffer: precoder_buffer_
     *     Intermediate buffer: pred_csi_buffer_
     * Offsets:
     *     csi_buffer_: 
     *         dim1: frame index * FFT size + subcarrier index in the current frame
     *     pred_csi_buffer:
     *         dim1: subcarrier index in the current frame
     *     precoder_buffer_:
     *         dim1: (frame index + 1) * FFT size + subcarrier index in the current frame
     * Event offset: offset
     * Description:
     *     1. predict CSI (copy CSI from the current frame if prediction is based on stale CSI)
     *     2. perform pseudo-inverse (pinv) on pred_csi_buffer_ and store results in precoder_buffer_  
     *     3. add an event to the message queue to infrom main thread the completion of this task
     */
    void doPred(int tid, int offset);


    /**
     * Do demodulation task for a block of subcarriers (demul_block_size)
     * @param tid: task thread index, used for selecting spm_buffer and task ptok
     * @param offset: offset of the first subcarrier in the block in data_buffer_
     * Buffers: data_buffer_, spm_buffer_, precoder_buffer_, equal_buffer_, demul_buffer_
     *     Input buffer: data_buffer_, precoder_buffer_
     *     Output buffer: demul_buffer_
     *     Intermediate buffer: spm_buffer, equal_buffer_
     * Offsets: 
     *     data_buffer_: 
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
     *     1. for each subcarrier in the block, block-wisely copy data from data_buffer_ to spm_buffer_
     *     2. perform equalization with data and percoder matrixes
     *     3. perform demodulation on equalized data matrix   
     *     4. add an event to the message queue to infrom main thread the completion of this task
     */
    void doDemul(int tid, int offset);
    

    /*****************************************************
     * Downlink 
     *****************************************************/

    /**
     * Do modulation and ifft tasks for one OFDM symbol
     * @param tid: task thread index, used for selecting task ptok
     * @param offset: offset of the OFDM symbol in dl_modulated_buffer_
     * Buffers: dl_IQ_data_long, dl_modulated_buffer_
     *     Input buffer: dl_IQ_data_long
     *     Output buffer: dl_iffted_data_buffer_
     *     Intermediate buffer: dl_ifft_buffer_
     * Offsets: 
     *     dl_IQ_data_long_: 
     *         dim1: data subframe index in the current frame * # of users + user index
     *         dim2: subcarrier index
     *     dl_ifft_buffer_: 
     *         dim1: frame index * # of data subframes per frame * # of users + data subframe index * # of users + user index
     *         dim2: subcarrier index 
     *     dl_iffted_data_buffer_: 
     *         dim1: frame index * # of data subframes per frame + data subframe index
     *         dim2: transpose block index * block size * # of UEs + user index * block size
     * Event offset: offset
     * Description: 
     *     1. for each OFDM symbol, perform modulation and then ifft
     *     2. perform block-wise transpose on IFFT outputs and store results in dl_iffted_data_buffer_
     *     2. add an event to the message queue to infrom main thread the completion of this task
     */
    void do_ifft(int tid, int offset);


    /**
     * Do precoding task for a block of subcarriers (demul_block_size)
     * @param tid: task thread index, used for selecting task ptok and dl_spm_buffer
     * @param offset: offset of the first subcarrier in the block in dl_iffted_data_buffer_
     * Buffers: dl_iffted_data_buffer_, precoder_buffer_, dl_spm_buffer, dl_precoded_data_buffer_
     *     Input buffer: dl_iffted_data_buffer_, precoder_buffer_
     *     Output buffer: dl_precoded_data_buffer_
     *     Intermediate buffer: dl_spm_buffer
     * Offsets: 
     *     dl_iffted_data_buffer_: 
     *         dim1: frame index * # of data subframes per frame + data subframe index
     *         dim2: transpose block index * block size * # of UEs + user index * block size
     *     dl_spm_buffer: 
     *         dim1: task thread index
     *         dim2: user index 
     *     precoder_buffer_:
     *         dim1: frame index * FFT size + subcarrier index in the current frame
     *     dl_precoded_data_buffer_: 
     *         dim1: frame index * # of data subframes per frame + data subframe index
     *         dim2: subcarrier index * # of ants
     * Event offset: offset
     * Description: 
     *     1. for each OFDM symbol, perform modulation and then ifft
     *     2. perform block-wise transpose on IFFT outputs and store results in dl_iffted_data_buffer_
     *     2. add an event to the message queue to infrom main thread the completion of this task
     */
    void do_precode(int tid, int offset); 


    // void do_tx(int tid, int offset);

    
    

    struct EventHandlerContext
    {
        CoMP *obj_ptr;
        int id;
    };

    /* Add tasks into task queue based on event type */
    void schedule_task(Event_data do_task, moodycamel::ConcurrentQueue<Event_data> * in_queue, moodycamel::ProducerToken const& ptok);

    /* combine frame_id & subframe_id into one int */
    inline int getSubframeBufferIndex(int frame_id, int subframe_id);
    inline void splitSubframeBufferIndex(int FFT_buffer_target_id, int *frame_id, int *subframe_id);
    /* combine frame_id & subframe_id & ant_id into one int */
    inline int getFFTBufferIndex(int frame_id, int subframe_id, int ant_id);
    inline void splitFFTBufferIndex(int FFT_buffer_target_id, int *frame_id, int *subframe_id, int *ant_id);
    
    inline bool isPilot(int subframe_id) {return (subframe_id >=0) && (subframe_id < UE_NUM); }
    inline bool isData(int subframe_id) {return (subframe_id < subframe_num_perframe) && (subframe_id >= UE_NUM); }
    // complex divide
    inline complex_float divide(complex_float e1, complex_float e2);

    // inline int demod_16qam(complex_float x);
    inline arma::imat demod_16qam(arma::cx_fmat x);
    inline arma::cx_fmat mod_16qam(arma::imat x);
    inline complex_float mod_16qam_single(int x);

private:
    /*****************************************************
     * Uplink 
     *****************************************************/ 
    
    std::unique_ptr<PackageReceiver> receiver_;
    
    /** 
     * received data 
     * Frist dimension: SOCKET_THREAD_NUM
     * Second dimension of buffer (type: char): package_length * subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM
     * package_length = sizeof(int) * 4 + sizeof(ushort) * OFDM_FRAME_LEN * 2;
     * Second dimension of buffer_status: subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM
     */
    SocketBuffer socket_buffer_[SOCKET_RX_THREAD_NUM];

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
    CSIBuffer csi_buffer_;

    /** 
     * Data symbols after IFFT
     * First dimension: total subframe number in the buffer: data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM
     * second dimension: BS_ANT_NUM * OFDM_CA_NUM
     * second dimension data order: SC1-32 of ants, SC33-64 of ants, ..., SC993-1024 of ants (32 blocks each with 32 subcarriers)
     */
    DataBuffer data_buffer_;

    /**
     * Calculated precoder
     * First dimension: OFDM_CA_NUM * TASK_BUFFER_FRAME_NUM
     * Second dimension: UE_NUM * BS_ANT_NUM
     */
    PrecoderBuffer precoder_buffer_;

    /**
     * Data after equalization
     * First dimension: data_subframe_num_perframe (40-4) * TASK_BUFFER_FRAME_NUM
     * Second dimension: OFDM_CA_NUM * UE_NUM
     */
    EqualBuffer equal_buffer_;

    /**
     * Data after demodulation
     * First dimension: data_subframe_num_perframe (40-4) * TASK_BUFFER_FRAME_NUM
     * Second dimension: OFDM_CA_NUM * UE_NUM
     */
    DemulBuffer demul_buffer_;

    /** 
     * Predicted CSI data 
     * First dimension: OFDM_CA_NUM 
     * Second dimension: BS_ANT_NUM * UE_NUM
     */
    CSIBuffer pred_csi_buffer_;

    /** 
     * First dimension: TASK_THREAD_NUM
     * Second dimension: UE_NUM */
    myVec spm_buffer[TASK_THREAD_NUM];


    std::vector<float> pilots_;

    mufft_plan_1d *muplans_[TASK_THREAD_NUM];


    /* Concurrent queues */
    /* task queue for uplink FFT */
    moodycamel::ConcurrentQueue<Event_data> task_queue_ = moodycamel::ConcurrentQueue<Event_data>(SOCKET_BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM  * 36);
    /* task queue for ZF */
    moodycamel::ConcurrentQueue<Event_data> zf_queue_ = moodycamel::ConcurrentQueue<Event_data>(SOCKET_BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM  * 36);
    /* task queue for uplink demodulation */
    moodycamel::ConcurrentQueue<Event_data> demul_queue_ = moodycamel::ConcurrentQueue<Event_data>(SOCKET_BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM  * 36);
    /* main thread message queue */
    moodycamel::ConcurrentQueue<Event_data> message_queue_ = moodycamel::ConcurrentQueue<Event_data>(SOCKET_BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM  * 36);


    pthread_t task_threads[TASK_THREAD_NUM];

    EventHandlerContext context[TASK_THREAD_NUM];

    // all checkers
    int cropper_checker_[subframe_num_perframe * TASK_BUFFER_FRAME_NUM];
    int csi_checker_[TASK_BUFFER_FRAME_NUM];
    int data_checker_[TASK_BUFFER_FRAME_NUM];

    int precoder_checker_[TASK_BUFFER_FRAME_NUM];
    bool precoder_status_[TASK_BUFFER_FRAME_NUM];

    int cropper_created_checker_[subframe_num_perframe * TASK_BUFFER_FRAME_NUM];

    // can possibly remove this checker
    int demul_checker_[TASK_BUFFER_FRAME_NUM][(subframe_num_perframe - UE_NUM)];
    int demul_status_[TASK_BUFFER_FRAME_NUM];

    std::queue<std::tuple<int, int>> taskWaitList;

    int max_queue_delay = 0;

    int debug_count = 0;

    std::unique_ptr<moodycamel::ProducerToken> task_ptok[TASK_THREAD_NUM];





    /*****************************************************
     * Downlink 
     *****************************************************/  

    std::unique_ptr<packageSenderBS> transmitter_;

    /** 
     * Raw data
     * First dimension: data_subframe_num_perframe * UE_NUM
     * Second dimension: OFDM_FRAME_LEN 
     */
    int **dl_IQ_data;
    long long **dl_IQ_data_long;

    /** 
     * Modulated data
     * First dimension: subframe_num_perframe (40) * UE_NUM * TASK_BUFFER_FRAME_NUM
     * Second dimension: OFDM_CA_NUM
     */
    // RawDataBuffer dl_rawdata_buffer_;


    /** 
     * Modulated data
     * First dimension: subframe_num_perframe (40) * UE_NUM * TASK_BUFFER_FRAME_NUM
     * Second dimension: OFDM_CA_NUM 
     */
    EqualBuffer dl_modulated_buffer_;

    /**
     * Data for IFFT
     * First dimension: FFT_buffer_block_num = UE_NUM * data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM
     * Second dimension: OFDM_CA_NUM
     */
    IFFTBuffer dl_ifft_buffer_;

    /**
     * Data after IFFT
     * First dimension: data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM
     * second dimension: UE_NUM * OFDM_CA_NUM
     * second dimension data order: SC1-32 of UEs, SC33-64 of UEs, ..., SC993-1024 of UEs (32 blocks each with 32 subcarriers)
     */
    DataBuffer dl_iffted_data_buffer_;


    /**
     * Precoded data
     * First dimension: total subframe number in the buffer: data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM
     * second dimension: BS_ANT_NUM * OFDM_CA_NUM
     */
    DataBuffer dl_precoded_data_buffer_;


    /**
     * Data for transmission
     * First dimension of buffer (type: char): subframe_num_perframe * SOCKET_BUFFER_FRAME_NUM
     * Second dimension: package_length * BS_ANT_NUM
     * package_length = sizeof(int) * 4 + sizeof(ushort) * OFDM_FRAME_LEN * 2;
     * First dimension of buffer_status: subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM
     */
    SocketBuffer dl_socket_buffer_;

    /** 
     * First dimension: TASK_THREAD_NUM
     * Second dimension: UE_NUM */
    myVec dl_spm_buffer[TASK_THREAD_NUM];

    /* task queue for downlink IFFT */
    moodycamel::ConcurrentQueue<Event_data> ifft_queue_ = moodycamel::ConcurrentQueue<Event_data>(SOCKET_BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM  * 36);
    /* task queue for downlink modulation */
    moodycamel::ConcurrentQueue<Event_data> modulation_queue_ = moodycamel::ConcurrentQueue<Event_data>(SOCKET_BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM  * 36);
    /* task queue for downlink precoding */
    moodycamel::ConcurrentQueue<Event_data> precode_queue_ = moodycamel::ConcurrentQueue<Event_data>(SOCKET_BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM  * 36);
    /* task queue for downlink data transmission */
    moodycamel::ConcurrentQueue<Event_data> tx_queue_ = moodycamel::ConcurrentQueue<Event_data>(SOCKET_BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM  * 36);

    mufft_plan_1d *muplans_ifft_[TASK_THREAD_NUM];

    int precode_checker_[TASK_BUFFER_FRAME_NUM][(subframe_num_perframe - UE_NUM)];
    int ifft_checker_[TASK_BUFFER_FRAME_NUM][(subframe_num_perframe - UE_NUM)];
    // int precoding_checker_[TASK_BUFFER_FRAME_NUM];

    /* lookup table for 16 QAM, real and imag */
    float qam16_table[2][16];

};

#endif
