/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 * 
 */
#ifndef DORECIP
#define DORECIP

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
// #include "mkl_dfti.h"


class Reciprocity
{
public:
    Reciprocity(Config *cfg, int in_tid,
        moodycamel::ConcurrentQueue<Event_data> *in_complete_task_queue, moodycamel::ProducerToken *in_task_ptok,
        complex_float **in_calib_buffer, complex_float **in_recip_buffer, 
        double **in_recip_task_duration, int *in_recip_task_count);
    ~Reciprocity();

    void computeReciprocityCalib(int offset);

private:
    Config *config_;
    int BS_ANT_NUM, UE_NUM;
    int OFDM_CA_NUM, OFDM_DATA_NUM;
    int ref_ant;

    int tid;
    moodycamel::ConcurrentQueue<Event_data> *complete_task_queue_;
    moodycamel::ProducerToken *task_ptok;

    complex_float **calib_buffer_;
    complex_float **recip_buffer_;

    double **RC_task_duration;
    int *RC_task_count;

    /** 
     * Intermediate buffer to gather CSI
     * First dimension: TASK_THREAD_NUM
     * Second dimension: BS_ANT_NUM * UE_NUM */
    complex_float *calib_gather_buffer;


};


#endif
