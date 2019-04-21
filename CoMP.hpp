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
#include <vector>


#include <fcntl.h>
#include <system_error>
#include <pthread.h>
#include <queue>
#include "mufft/fft.h"
// #include <complex.h>
#include <math.h>
#include <tuple>
#include "cpu_attach.hpp"
#include <armadillo>
#include <immintrin.h>
#include <emmintrin.h>
#include <stdint.h>
#include "buffer.hpp"
#include "concurrentqueue.h"
#include <signal.h>
#include <aff3ct.hpp>
#include "mkl_dfti.h"
// #include <hpctoolkit.h>
// #include <cblas.h>
// #include <stdio.h>


class CoMP
{
public:
    // TASK & SOCKET thread number 
    static const int TASK_THREAD_NUM = ENABLE_DOWNLINK ? 30: 30;
    static const int SOCKET_RX_THREAD_NUM = ENABLE_DOWNLINK ? 2 : 4;
    static const int SOCKET_TX_THREAD_NUM = ENABLE_DOWNLINK ? 2 : 0;
    static const int CORE_OFFSET = 17;

    static const int FFT_THREAD_NUM = 6;
    static const int ZF_THREAD_NUM = 6;//16;
    static const int DEMUL_THREAD_NUM = TASK_THREAD_NUM - FFT_THREAD_NUM - ZF_THREAD_NUM;
    // buffer length of each socket thread
    // the actual length will be SOCKET_BUFFER_FRAME_NUM
    // * subframe_num_perframe * BS_ANT_NUM
    static const int SOCKET_BUFFER_FRAME_NUM = 100;
    // buffer length of computation part (for FFT/CSI/ZF/DEMUL buffers)
    static const int TASK_BUFFER_FRAME_NUM = 60;
    // do demul_block_size sub-carriers in each task
    static const int demul_block_size = 40;
    static const int demul_block_num = OFDM_DATA_NUM/demul_block_size + (OFDM_DATA_NUM % demul_block_size == 0 ? 0 : 1);
    static const int zf_block_size = 40;
    static const int zf_block_num = OFDM_DATA_NUM/zf_block_size + (OFDM_DATA_NUM % zf_block_size == 0 ? 0 : 1);
    // optimization parameters for block transpose (see the slides for more
    // details)
    static const int transpose_block_size = 8;
    static const int transpose_block_num = 256;
    // dequeue bulk size, used to reduce the overhead of dequeue in main
    // thread
    static const int dequeue_bulk_size = 8;

    CoMP();
    ~CoMP();

    void start();
    // while loop of task thread
    static void *taskThread(void *context);
    static void *fftThread(void *context);
    static void *zfThread(void *context);
    static void *demulThread(void *context);

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
    void doFFT(int tid, int offset);


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
     * Buffers: data_buffer_, spm_buffer_, precoder_buffer_, equal_buffer_, demul_hard_buffer_
     *     Input buffer: data_buffer_, precoder_buffer_
     *     Output buffer: demul_hard_buffer_
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

    void doDemulSingleSC(int tid, int offset);

    void doDecode(int tid, int offset);    

    /*****************************************************
     * Downlink 
     *****************************************************/

    void do_modulate(int tid, int offset);
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

    inline int generateOffset2d(int unit_total_num, int frame_id, int unit_id);
    inline int generateOffset3d(int unit_total_num, int frame_id, int current_data_subframe_id, int unit_id);
    inline void interpreteOffset2d(int unit_total_num, int offset, int *frame_id, int *unit_id);
    inline void interpreteOffset3d(int unit_total_num, int offset, int *frame_id, int *total_data_subframe_id, int *current_data_subframe_id, int *unit_id);

    /* combine frame_id & subframe_id & ant_id into one int */
    inline int getFFTBufferIndex(int frame_id, int subframe_id, int ant_id);
    inline void splitFFTBufferIndex(int FFT_buffer_target_id, int *frame_id, int *subframe_id, int *ant_id);
    
    inline bool isPilot(int subframe_id) {return (subframe_id >=0) && (subframe_id < UE_NUM); }
    inline bool isData(int subframe_id) {return (subframe_id < subframe_num_perframe) && (subframe_id >= UE_NUM); }
    inline int getUEId(int subframe_id) {return subframe_id; }
    inline int getULSFIndex(int subframe_id) {return subframe_id - UE_NUM; }
    // complex divide
    inline complex_float divide(complex_float e1, complex_float e2);

    // inline int demod_16qam(complex_float x);
    inline arma::imat demod_16qam(arma::cx_fmat x);
    inline void demod_16qam_loop(float *vec_in, uint8_t *vec_out, int ue_num);
    inline void demod_16qam_soft(float *vec_in, float *vec_out, int length);
    inline arma::cx_fmat mod_16qam(arma::imat x);
    inline complex_float mod_16qam_single(int x);

    void getDemulData(int **ptr, int *size);
    void getEqualData(float **ptr, int *size);

private:
    /*****************************************************
     * Uplink 
     *****************************************************/ 
    
    std::unique_ptr<PackageReceiver> receiver_;
    std::vector<aff3ct::module::Encoder_LDPC_from_QC<>*> Encoders;
    std::vector<aff3ct::module::Modem_generic<>*> Modems;
    std::unique_ptr<aff3ct::module::Decoder_LDPC_BP_horizontal_layered_ONMS_inter<>> Decoders[TASK_THREAD_NUM];
    // std::vector<aff3ct::module::Decoder_LDPC_BP_flooding_inter<>*> Decoders;
    std::vector<unsigned> info_bits_pos[TASK_THREAD_NUM];
    // std::vector<aff3ct::tools::Update_rule_NMS_simd<float,0>> up_rules;
    aff3ct::tools::Sparse_matrix H[TASK_THREAD_NUM];
    const int K = ORIG_CODE_LEN * NUM_BITS;
    const int N = CODED_LEN * NUM_BITS;
    // float ebn0 = 10.0f;
    // const int K = ORIG_CODE_LEN * NUM_BITS;
    // const int N = CODED_LEN * NUM_BITS;
    // const float R = (float)K / (float)N;
    // const float esn0  = aff3ct::tools::ebn0_to_esn0 (ebn0, R);
    // const float sigma = aff3ct::tools::esn0_to_sigma(esn0   );

    // const aff3ct::tools::Update_rule_NMS_simd<> up_rule = aff3ct::tools::Update_rule_NMS_simd <float>(0.75);
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
     * First dimension: UE_NUM * TASK_BUFFER_FRAME_NUM
     * Second dimension: BS_ANT_NUM * OFDM_CA_NUM
     */

    // TODO: need to remove
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
    uint8_t **demul_hard_buffer_;

    float **demul_soft_buffer_;

    /** 
     * Predicted CSI data 
     * First dimension: OFDM_CA_NUM 
     * Second dimension: BS_ANT_NUM * UE_NUM
     */
    CSIBuffer pred_csi_buffer_;


    int **decoded_buffer_;

    

    /** 
     * Intermediate buffer to gather raw data
     * First dimension: TASK_THREAD_NUM
     * Second dimension: BS_ANT_NUM */
    // myVec spm_buffer[TASK_THREAD_NUM];
    complex_float *spm_buffer[TASK_THREAD_NUM];

    /** 
     * Intermediate buffer to gather CSI
     * First dimension: TASK_THREAD_NUM
     * Second dimension: BS_ANT_NUM * UE_NUM */
    complex_float *csi_gather_buffer[TASK_THREAD_NUM];


    /** 
     * Intermediate buffer for calculated precoder 
     * First dimension: TASK_THREAD_NUM
     * Second dimension: BS_ANT_NUM * UE_NUM */
    complex_float *precoder_buffer_temp[TASK_THREAD_NUM];

    /** 
     * Intermediate buffer for equalized data
     * First dimension: TASK_THREAD_NUM
     * Second dimension: UE_NUM * 1 */
    complex_float *equaled_buffer_temp[TASK_THREAD_NUM];
    complex_float *equaled_buffer_T_temp[TASK_THREAD_NUM];


    uint8_t *demul_hard_buffer_temp[TASK_THREAD_NUM];
    float *demul_soft_buffer_temp[TASK_THREAD_NUM];

    int *coded_buffer_temp[TASK_THREAD_NUM];



    float *pilots_;
    // std::vector<float> pilots_;
    // std::vector<complex_float> pilots_complex_;

    mufft_plan_1d *muplans_[TASK_THREAD_NUM];

    DFTI_DESCRIPTOR_HANDLE mkl_handles[TASK_THREAD_NUM];
    MKL_LONG mkl_statuses[TASK_THREAD_NUM];


    /* Concurrent queues */
    /* task queue for uplink FFT */
    // moodycamel::ConcurrentQueue<Event_data> fft_queue_ = moodycamel::ConcurrentQueue<Event_data>(SOCKET_BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM  * 36);
    // /* task queue for ZF */
    // moodycamel::ConcurrentQueue<Event_data> zf_queue_ = moodycamel::ConcurrentQueue<Event_data>(SOCKET_BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM  * 36);
    // /* task queue for uplink demodulation */
    // moodycamel::ConcurrentQueue<Event_data> demul_queue_ = moodycamel::ConcurrentQueue<Event_data>(SOCKET_BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM  * 36);
    // /* task queue for uplink demodulation */
    // moodycamel::ConcurrentQueue<Event_data> decode_queue_ = moodycamel::ConcurrentQueue<Event_data>(SOCKET_BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM  * 36);
    // /* main thread message queue for data receiving */
    // moodycamel::ConcurrentQueue<Event_data> message_queue_ = moodycamel::ConcurrentQueue<Event_data>(SOCKET_BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM  * 36);
    // /* main thread message queue for task completion*/
    // moodycamel::ConcurrentQueue<Event_data> complete_task_queue_ = moodycamel::ConcurrentQueue<Event_data>(SOCKET_BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM  * 36);
     /* task queue for uplink FFT */
    moodycamel::ConcurrentQueue<Event_data> fft_queue_ = moodycamel::ConcurrentQueue<Event_data>(512*data_subframe_num_perframe);
    /* task queue for ZF */
    moodycamel::ConcurrentQueue<Event_data> zf_queue_ = moodycamel::ConcurrentQueue<Event_data>(512*data_subframe_num_perframe);
    /* task queue for uplink demodulation */
    moodycamel::ConcurrentQueue<Event_data> demul_queue_ = moodycamel::ConcurrentQueue<Event_data>(512*data_subframe_num_perframe);
    /* task queue for uplink demodulation */
    moodycamel::ConcurrentQueue<Event_data> decode_queue_ = moodycamel::ConcurrentQueue<Event_data>(512);
    /* main thread message queue for data receiving */
    moodycamel::ConcurrentQueue<Event_data> message_queue_ = moodycamel::ConcurrentQueue<Event_data>(512*subframe_num_perframe);
    /* main thread message queue for task completion*/
    moodycamel::ConcurrentQueue<Event_data> complete_task_queue_ = moodycamel::ConcurrentQueue<Event_data>(512*subframe_num_perframe*4);



    pthread_t task_threads[TASK_THREAD_NUM];

    EventHandlerContext context[TASK_THREAD_NUM];

    // all checkers
    /* used to check if RX for all antennas and all subframes in a frame is done (max: BS_ANT_NUM * subframe_num_perframe) */
    int rx_counter_packets_[TASK_BUFFER_FRAME_NUM];   
    /* used to check if FFT for all antennas in a subframe is done (max: BS_ANT_NUM) */
    int fft_counter_ants_[subframe_num_perframe * TASK_BUFFER_FRAME_NUM];
    /* used to check if FFT for all users/pilots in a frame is done (max: UE_NUM) */
    int csi_counter_users_[TASK_BUFFER_FRAME_NUM];
    /* used to check if FFT for all data subframes in a frame is done (max: data_subframe_num_perframe) */
    int data_counter_subframes_[TASK_BUFFER_FRAME_NUM];
    /* used to check if ZF for all subcarriers in a frame is done (max: OFDM_DATA_NUM) */
    int precoder_counter_scs_[TASK_BUFFER_FRAME_NUM];
    /* used to check if demodulation for all subcarriers in a data subframe is done (max: OFDM_DATA_NUM) */
    int demul_counter_scs_[TASK_BUFFER_FRAME_NUM][(subframe_num_perframe - UE_NUM)];
    /* used to check if demodulation for all data subframes in a frame is done (max: data_subframe_num_perframe) */
    int demul_counter_subframes_[TASK_BUFFER_FRAME_NUM];
    /* used to check if creating FFT for all antennas and all subframes in a frame is done (max: BS_ANT_NUM * subframe_num_perframe) */
    int fft_created_counter_packets_[TASK_BUFFER_FRAME_NUM];

    /* used to check the existance of data after FFT of a subframe in a frame */
    bool data_exist_in_subframe_[TASK_BUFFER_FRAME_NUM][(subframe_num_perframe - UE_NUM)];
    /* used to check the existance of precoder in a frame */
    bool precoder_exist_in_frame_[TASK_BUFFER_FRAME_NUM];
    bool precoder_exist_in_sc_[TASK_BUFFER_FRAME_NUM][OFDM_DATA_NUM];

    int decode_counter_blocks_[TASK_BUFFER_FRAME_NUM][(subframe_num_perframe-UE_NUM)];
    int decode_counter_subframes_[TASK_BUFFER_FRAME_NUM];


    std::queue<std::tuple<int, int>> taskWaitList;

    int max_queue_delay = 0;

    int debug_count = 0;

    std::unique_ptr<moodycamel::ProducerToken> task_ptok[TASK_THREAD_NUM];

    int FFT_task_count[TASK_THREAD_NUM*16];
    int ZF_task_count[TASK_THREAD_NUM*16];
    int Demul_task_count[TASK_THREAD_NUM*16];

    double FFT_task_duration[TASK_THREAD_NUM*8][4];
    double ZF_task_duration[TASK_THREAD_NUM*8][4];
    double Demul_task_duration[TASK_THREAD_NUM*8][4];


    long long socket_buffer_size_;
    int socket_buffer_status_size_;





    /*****************************************************
     * Downlink 
     *****************************************************/  

    std::unique_ptr<packageSenderBS> transmitter_;

    /** 
     * Raw data
     * First dimension: data_subframe_num_perframe * UE_NUM
     * Second dimension: OFDM_CA_NUM
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
     * First dimension: data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM
     * second dimension: UE_NUM * OFDM_CA_NUM
     */
    EqualBuffer dl_modulated_buffer_;

    /**
     * Data for IFFT
     * First dimension: FFT_buffer_block_num = BS_ANT_NUM * data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM
     * Second dimension: OFDM_CA_NUM
     */
    IFFTBuffer dl_ifft_buffer_;

    /**
     * Data after IFFT
     * First dimension: data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM
     * second dimension: UE_NUM * OFDM_CA_NUM
     * second dimension data order: SC1-32 of UEs, SC33-64 of UEs, ..., SC993-1024 of UEs (32 blocks each with 32 subcarriers)
     */
    // DataBuffer dl_iffted_data_buffer_;


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
    complex_float *dl_spm_buffer[TASK_THREAD_NUM];

    /* task queue for downlink IFFT */
    moodycamel::ConcurrentQueue<Event_data> ifft_queue_ = moodycamel::ConcurrentQueue<Event_data>(SOCKET_BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM  * 36);
    /* task queue for downlink modulation */
    moodycamel::ConcurrentQueue<Event_data> modulate_queue_ = moodycamel::ConcurrentQueue<Event_data>(SOCKET_BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM  * 36);
    /* task queue for downlink precoding */
    moodycamel::ConcurrentQueue<Event_data> precode_queue_ = moodycamel::ConcurrentQueue<Event_data>(SOCKET_BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM  * 36);
    /* task queue for downlink data transmission */
    moodycamel::ConcurrentQueue<Event_data> tx_queue_ = moodycamel::ConcurrentQueue<Event_data>(SOCKET_BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM  * 36);

    mufft_plan_1d *muplans_ifft_[TASK_THREAD_NUM];

    int dl_data_counter_scs_[TASK_BUFFER_FRAME_NUM][(subframe_num_perframe - UE_NUM)];
    int dl_data_counter_subframes_[TASK_BUFFER_FRAME_NUM];
    int modulate_checker_[TASK_BUFFER_FRAME_NUM][(subframe_num_perframe - UE_NUM)];
    int ifft_checker_[TASK_BUFFER_FRAME_NUM];
    int tx_counter_ants_[TASK_BUFFER_FRAME_NUM][(subframe_num_perframe - UE_NUM)];
    int tx_counter_subframes_[TASK_BUFFER_FRAME_NUM];
    // int precoding_checker_[TASK_BUFFER_FRAME_NUM];

    /* lookup table for 16 QAM, real and imag */
    float qam16_table[2][16];
    int max_equaled_frame=0;
    float csi_format_offset;

    long long dl_socket_buffer_size_;
    int dl_socket_buffer_status_size_;

    double IFFT_task_duration[TASK_THREAD_NUM][4];
    double Precode_task_duration[TASK_THREAD_NUM][4];

    int IFFT_task_count[TASK_THREAD_NUM];
    int Precode_task_count[TASK_THREAD_NUM];

    double frame_start[SOCKET_RX_THREAD_NUM][10240] __attribute__( ( aligned (4096) ) ) ;



    // const float m_bpsk_lut[256] = {
    // 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 
    // 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 
    // 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 
    // 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 
    // 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 
    // 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 
    // 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 
    // 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 
    // 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    // 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    // 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    // 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    // 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    // 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    // 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 
    // 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 
    // };

    const float m_bpsk_lut[256] = {
    -4, -4, -4, -4, -4, -4, -4, -4, -4, -5, -5, -5, -5, -5, -5, -5, 
    -5, -5, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -7, 
    -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, 
    -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, 
    -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, 
    -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, 
    -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, 
    -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, 
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 
    7, 7, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 
    2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 
    };

    // const float m_qam16_lut2[256] = {
    // 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 
    // 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 
    // 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 
    // 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 6, 5, 5, 5, 4, 4, 
    // 3, 3, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    // 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    // 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    // 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    // 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    // 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    // 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    // 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 
    // 3, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 
    // 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 
    // 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 
    // 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 
    // };

    const float m_qam16_lut2[256] = {
    -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7,
    -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7,
    -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7,
    1, 1, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 6, 
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 
    7, 7, 7, 7, 7, 7, 7, 6, 6, 6, 5, 5, 5, 4, 4, 3,  
    3, 2, 2, 1, 1, 1, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7,  
    -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7,  
    -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7,
    -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7,
    };




};

#endif
