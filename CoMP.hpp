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
    static const int TASK_THREAD_NUM = ENABLE_DOWNLINK ? 27 : 28;
    static const int SOCKET_RX_THREAD_NUM = ENABLE_DOWNLINK ? 4 : 7;
    static const int SOCKET_TX_THREAD_NUM = ENABLE_DOWNLINK ? 4 : 0;
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
    static void* taskThread(void* context);
    // do different tasks
    // Uplink
    void doCrop(int tid, int offset);
    void doZF(int tid, int offset);
    void doDemul(int tid, int offset);
    void doPred(int tid, int offset);

    // Downlink
    void do_ifft(int tid, int offset);
    void do_precode(int tid, int offset); 
    void do_modulation(int tid, int offset);

    struct EventHandlerContext
    {
        CoMP* obj_ptr;
        int id;
    };
    // combine frame_id & subframe_id into one int
    inline int getSubframeBufferIndex(int frame_id, int subframe_id);
    inline void splitSubframeBufferIndex(int FFT_buffer_target_id, int *frame_id, int *subframe_id);
    // combine frame_id & subframe_id & ant_id into one int
    inline int getFFTBufferIndex(int frame_id, int subframe_id, int ant_id);
    inline void splitFFTBufferIndex(int FFT_buffer_target_id, int *frame_id, int *subframe_id, int *ant_id);
    
    inline bool isPilot(int subframe_id) {return (subframe_id >=0) && (subframe_id < UE_NUM); }
    inline bool isData(int subframe_id) {return (subframe_id < subframe_num_perframe) && (subframe_id >= UE_NUM); }
    // complex divide
    inline complex_float divide(complex_float e1, complex_float e2);

    // inline int demod_16qam(complex_float x);
    inline arma::imat demod_16qam(arma::cx_fmat x);
    inline arma::cx_fmat mod_16qam(arma::imat x);

private:
    /**********************/
    /* Uplink */
    /**********************/
    
    std::unique_ptr<PackageReceiver> receiver_;
    
    // received data 
    // Frist dimension: SOCKET_THREAD_NUM
    // Second dimension of buffer (type: char): package_length * subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM
    // package_length = sizeof(int) * 4 + sizeof(ushort) * OFDM_FRAME_LEN * 2;
    // Second dimension of buffer_status: subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM
    SocketBuffer socket_buffer_[SOCKET_RX_THREAD_NUM];

    // Data for FFT, after time sync (prefix removed)
    // First dimension: FFT_buffer_block_num = BS_ANT_NUM * subframe_num_perframe * TASK_BUFFER_FRAME_NUM
    // Second dimension: OFDM_CA_NUM
    FFTBuffer fft_buffer_;

    // Estimated CSI data 
    // First dimension: OFDM_CA_NUM * TASK_BUFFER_FRAME_NUM
    // Second dimension: BS_ANT_NUM * UE_NUM
    CSIBuffer csi_buffer_;

    // Data symbols after IFFT
    // First dimension: total subframe number in the buffer: data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM
    // second dimension: BS_ANT_NUM * OFDM_CA_NUM
    // second dimension data order: SC1-32 of ants, SC33-64 of ants, ..., SC993-1024 of ants (32 blocks each with 32 subcarriers)
    DataBuffer data_buffer_;

    // Calculated precoder
    // First dimension: OFDM_CA_NUM * TASK_BUFFER_FRAME_NUM
    // Second dimension: UE_NUM * BS_ANT_NUM
    PrecoderBuffer precoder_buffer_;

    // Data after equalization
    // First dimension: data_subframe_num_perframe (40-4) * TASK_BUFFER_FRAME_NUM
    // Second dimension: OFDM_CA_NUM * UE_NUM
    EqualBuffer equal_buffer_;

    // Data after demudulation
    // First dimension: data_subframe_num_perframe (40-4) * TASK_BUFFER_FRAME_NUM
    // Second dimension: OFDM_CA_NUM * UE_NUM
    DemulBuffer demul_buffer_;

    // Predicted CSI data 
    // First dimension: OFDM_CA_NUM 
    // Second dimension: BS_ANT_NUM * UE_NUM
    CSIBuffer pred_csi_buffer_;


    std::vector<float> pilots_;

    mufft_plan_1d* muplans_[TASK_THREAD_NUM];


    // Concurrent queues
    // task queue for uplink FFT
    moodycamel::ConcurrentQueue<Event_data> task_queue_ = moodycamel::ConcurrentQueue<Event_data>(SOCKET_BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM  * 36);
    // task queue for ZF
    moodycamel::ConcurrentQueue<Event_data> zf_queue_ = moodycamel::ConcurrentQueue<Event_data>(SOCKET_BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM  * 36);
    // task queue for uplink demodulation
    moodycamel::ConcurrentQueue<Event_data> demul_queue_ = moodycamel::ConcurrentQueue<Event_data>(SOCKET_BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM  * 36);
    // main thread message queue
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

    // First dimension: TASK_THREAD_NUM
    // Second dimension: BS_ANT_NUM
    myVec spm_buffer[TASK_THREAD_NUM];



    /**********************/
    /* Downlink */
    /**********************/  

    std::unique_ptr<packageSenderBS> transmitter_;

    // Raw data
    // First dimension: subframe_num_perframe * UE_NUM
    // Second dimension: OFDM_FRAME_LEN 
    int** dl_IQ_data;
    long long** dl_IQ_data_long;

    // Modulated data
    // First dimension: subframe_num_perframe (40) * TASK_BUFFER_FRAME_NUM
    // Second dimension: OFDM_CA_NUM * UE_NUM
    EqualBuffer dl_modulated_buffer_;


    // Precoded data
    // First dimension: total subframe number in the buffer: data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM
    // second dimension: BS_ANT_NUM * OFDM_CA_NUM
    // second dimension data order: SC1-32 of ants, SC33-64 of ants, ..., SC993-1024 of ants (32 blocks each with 32 subcarriers)
    DataBuffer dl_precoded_data_buffer_;


    // Data after IFFT
    // First dimension: FFT_buffer_block_num = UE_NUM * subframe_num_perframe * TASK_BUFFER_FRAME_NUM
    // Second dimension: OFDM_CA_NUM
    FFTBuffer dl_ifft_buffer_;

    // Data for transmission
    // First dimension of buffer (type: char): package_length * subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM
    // package_length = sizeof(int) * 4 + sizeof(ushort) * OFDM_FRAME_LEN * 2;
    // First dimension of buffer_status: subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM
    SocketBuffer dl_socket_buffer_;

    // First dimension: TASK_THREAD_NUM
    // Second dimension: UE_NUM
    myVec dl_spm_buffer[TASK_THREAD_NUM];

    // task queue for downlink IFFT
    moodycamel::ConcurrentQueue<Event_data> ifft_queue_ = moodycamel::ConcurrentQueue<Event_data>(SOCKET_BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM  * 36);
    // task queue for downlink modulation
    moodycamel::ConcurrentQueue<Event_data> modulation_queue_ = moodycamel::ConcurrentQueue<Event_data>(SOCKET_BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM  * 36);
    // task queue for downlink precoding
    moodycamel::ConcurrentQueue<Event_data> precode_queue_ = moodycamel::ConcurrentQueue<Event_data>(SOCKET_BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM  * 36);
    // task queue for downlink data transmission 
    moodycamel::ConcurrentQueue<Event_data> tx_queue_ = moodycamel::ConcurrentQueue<Event_data>(SOCKET_BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM  * 36);

    mufft_plan_1d* muplans_ifft_[TASK_THREAD_NUM];

    int modul_checker_[TASK_BUFFER_FRAME_NUM][(subframe_num_perframe - UE_NUM)];
    int ifft_checker_[TASK_BUFFER_FRAME_NUM];
    // int precoding_checker_[TASK_BUFFER_FRAME_NUM];

};

#endif
