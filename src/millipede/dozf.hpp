/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 * 
 */
#ifndef DOZF
#define DOZF

#include <iostream>
#include <stdio.h>  /* for fprintf */
#include <string.h> /* for memcpy */
#include <vector>
#include <armadillo>
#include "buffer.hpp"
#include "concurrentqueue.h"
#include "Symbols.hpp"
#include "gettime.h"
#include "offset.h"
#include "config.hpp"
#include "stats.hpp"
// #include "mkl_dfti.h"


class DoZF
{
public:
    DoZF(Config *cfg, int in_tid, int in_zf_block_size, int in_transpose_block_size,
        moodycamel::ConcurrentQueue<Event_data> *in_complete_task_queue, moodycamel::ProducerToken *in_task_ptok,
        complex_float **in_csi_buffer, complex_float **in_precoder_buffer, complex_float **in_dl_precoder_buffer, complex_float **in_recip_buffer, complex_float **in_pred_csi_buffer, 
        Stats *in_stats_manager);
    ~DoZF();

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
    void ZF(int offset);

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
    void Predict(int offset);
    
 
private:
    Config *config_;
    int BS_ANT_NUM, UE_NUM;
    int OFDM_CA_NUM, OFDM_DATA_NUM;

    int tid;
    int transpose_block_size;
    int zf_block_size;
    moodycamel::ConcurrentQueue<Event_data> *complete_task_queue_;
    moodycamel::ProducerToken *task_ptok;

    complex_float **csi_buffer_;
    complex_float **precoder_buffer_;
    complex_float **dl_precoder_buffer_;
    complex_float **recip_buffer_;
    complex_float **pred_csi_buffer_;

    double **ZF_task_duration;
    int *ZF_task_count;

    /** 
     * Intermediate buffer to gather CSI
     * First dimension: TASK_THREAD_NUM
     * Second dimension: BS_ANT_NUM * UE_NUM */
    complex_float *csi_gather_buffer;


};


#endif
