/**
 * Author: Peiyao Zhao
 * E-Mail: pdszpy19930218@163.com
 * 
 */

#ifndef COMP_HEAD
#define COMP_HEAD

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
#include <armadillo>
#include <immintrin.h>
#include <emmintrin.h>
#include <stdint.h>
#include <signal.h>
#include <algorithm>
// #include <aff3ct.hpp>
#include "mkl_dfti.h"
// #include <hpctoolkit.h>
// #include <cblas.h>
// #include <stdio.h>
#include "cpu_attach.hpp"
#include "packageReceiver.hpp"
#include "buffer.hpp"
#include "concurrentqueue.h"
#include "gettime.h"
#include "compute_common.hpp"
#include "offset.h"
#include "dofft.hpp"
#include "dozf.hpp"
#include "dodemul.hpp"
#include "doprecode.hpp"


class CoMP
{
public:
    // TASK & SOCKET thread number 
    static const int TASK_THREAD_NUM = ENABLE_DOWNLINK ? 25: 25;
    static const int SOCKET_RX_THREAD_NUM = ENABLE_DOWNLINK ? 4 : 4;
    static const int SOCKET_TX_THREAD_NUM = ENABLE_DOWNLINK ? 4 : 0;
    static const int CORE_OFFSET = 17;

    static const int FFT_THREAD_NUM = 4;
    // static const int ZF_THREAD_NUM = 8;//16;
    // static const int DEMUL_THREAD_NUM = TASK_THREAD_NUM - FFT_THREAD_NUM - ZF_THREAD_NUM;
    static const int DEMUL_THREAD_NUM = 11;//16;
    static const int ZF_THREAD_NUM = TASK_THREAD_NUM - FFT_THREAD_NUM - DEMUL_THREAD_NUM;
    // buffer length of each socket thread
    // the actual length will be SOCKET_BUFFER_FRAME_NUM
    // * subframe_num_perframe * BS_ANT_NUM
    // static const int SOCKET_BUFFER_FRAME_NUM = 100;
    // buffer length of computation part (for FFT/CSI/ZF/DEMUL buffers)
    // static const int TASK_BUFFER_FRAME_NUM = 60;
    // do demul_block_size sub-carriers in each task
    static const int demul_block_size = 48;
    static const int demul_block_num = OFDM_DATA_NUM/demul_block_size + (OFDM_DATA_NUM % demul_block_size == 0 ? 0 : 1);
    static const int zf_block_size = 1;
    static const int zf_block_num = OFDM_DATA_NUM/zf_block_size + (OFDM_DATA_NUM % zf_block_size == 0 ? 0 : 1);
    // optimization parameters for block transpose (see the slides for more
    // details)
    static const int transpose_block_size = 8;
    static const int transpose_block_num = 256;
    // dequeue bulk size, used to reduce the overhead of dequeue in main
    // thread
    static const int dequeue_bulk_size = 32;
    static const int dequeue_bulk_size_single = 8;

    CoMP();
    ~CoMP();

    void start();
    // while loop of task thread
    static void *taskThread(void *context);
    static void *fftThread(void *context);
    static void *zfThread(void *context);
    static void *demulThread(void *context); 


    struct EventHandlerContext
    {
        CoMP *obj_ptr;
        int id;
    };

    /* Add tasks into task queue based on event type */
    void schedule_task(Event_data do_task, moodycamel::ConcurrentQueue<Event_data> * in_queue, moodycamel::ProducerToken const& ptok);
    
    inline bool isPilot(int subframe_id) {return (subframe_id >=0) && (subframe_id < UE_NUM); }
    inline bool isData(int subframe_id) {return (subframe_id < subframe_num_perframe) && (subframe_id >= UE_NUM); }
    inline int getUEId(int subframe_id) {return subframe_id; }
    inline int getULSFIndex(int subframe_id) {return subframe_id - UE_NUM; }

    void getDemulData(int **ptr, int *size);
    void getEqualData(float **ptr, int *size);

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
     * Intermediate buffer for calculated precoder 
     * First dimension: TASK_THREAD_NUM
     * Second dimension: BS_ANT_NUM * UE_NUM */
    complex_float *precoder_buffer_temp[TASK_THREAD_NUM];
    complex_float *equaled_buffer_T_temp[TASK_THREAD_NUM];


    uint8_t *demul_hard_buffer_temp[TASK_THREAD_NUM];
    float *demul_soft_buffer_temp[TASK_THREAD_NUM];

    int *coded_buffer_temp[TASK_THREAD_NUM];



    float *pilots_;
    // std::vector<float> pilots_;
    // std::vector<complex_float> pilots_complex_;


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
    moodycamel::ConcurrentQueue<Event_data> fft_queue_ = moodycamel::ConcurrentQueue<Event_data>(512*data_subframe_num_perframe*4);
    /* task queue for ZF */
    moodycamel::ConcurrentQueue<Event_data> zf_queue_ = moodycamel::ConcurrentQueue<Event_data>(512*data_subframe_num_perframe*4);
    /* task queue for uplink demodulation */
    moodycamel::ConcurrentQueue<Event_data> demul_queue_ = moodycamel::ConcurrentQueue<Event_data>(512*data_subframe_num_perframe*4);
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
    /* used to check if RX for all antennas and all subframes in a frame is done (max: BS_ANT_NUM * subframe_num_perframe) */
    int rx_counter_packets_pilots_[TASK_BUFFER_FRAME_NUM];   
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

    std::unique_ptr<moodycamel::ProducerToken> rx_ptok[SOCKET_RX_THREAD_NUM]; 

    std::unique_ptr<moodycamel::ProducerToken> tx_ptok[SOCKET_RX_THREAD_NUM]; 



    int CSI_task_count[TASK_THREAD_NUM*16];
    int FFT_task_count[TASK_THREAD_NUM*16];
    int ZF_task_count[TASK_THREAD_NUM*16];
    int Demul_task_count[TASK_THREAD_NUM*16];

    double CSI_task_duration[TASK_THREAD_NUM*8][4];
    double FFT_task_duration[TASK_THREAD_NUM*8][4];
    double ZF_task_duration[TASK_THREAD_NUM*8][4];
    double Demul_task_duration[TASK_THREAD_NUM*8][4];


    long long socket_buffer_size_;
    int socket_buffer_status_size_;





    /*****************************************************
     * Downlink 
     *****************************************************/  

    // std::unique_ptr<packageSenderBS> transmitter_;

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

    // /* task queue for downlink IFFT */
    // moodycamel::ConcurrentQueue<Event_data> ifft_queue_ = moodycamel::ConcurrentQueue<Event_data>(SOCKET_BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM  * 36);
    // /* task queue for downlink modulation */
    // moodycamel::ConcurrentQueue<Event_data> modulate_queue_ = moodycamel::ConcurrentQueue<Event_data>(SOCKET_BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM  * 36);
    // /* task queue for downlink precoding */
    // moodycamel::ConcurrentQueue<Event_data> precode_queue_ = moodycamel::ConcurrentQueue<Event_data>(SOCKET_BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM  * 36);
    // /* task queue for downlink data transmission */
    // moodycamel::ConcurrentQueue<Event_data> tx_queue_ = moodycamel::ConcurrentQueue<Event_data>(SOCKET_BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM  * 36);
        /* task queue for downlink IFFT */
    moodycamel::ConcurrentQueue<Event_data> ifft_queue_ = moodycamel::ConcurrentQueue<Event_data>(512*data_subframe_num_perframe*4);
    /* task queue for downlink modulation */
    moodycamel::ConcurrentQueue<Event_data> modulate_queue_ = moodycamel::ConcurrentQueue<Event_data>(512*data_subframe_num_perframe*4);
    /* task queue for downlink precoding */
    moodycamel::ConcurrentQueue<Event_data> precode_queue_ = moodycamel::ConcurrentQueue<Event_data>(512*data_subframe_num_perframe*4);
    /* task queue for downlink data transmission */
    moodycamel::ConcurrentQueue<Event_data> tx_queue_ = moodycamel::ConcurrentQueue<Event_data>(512*data_subframe_num_perframe*4);


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

    double IFFT_task_duration[TASK_THREAD_NUM*8][4];
    double Precode_task_duration[TASK_THREAD_NUM*8][4];

    int IFFT_task_count[TASK_THREAD_NUM*16];
    int Precode_task_count[TASK_THREAD_NUM*16];

    double frame_start[SOCKET_RX_THREAD_NUM][10240] __attribute__( ( aligned (4096) ) ) ;


};

#endif
