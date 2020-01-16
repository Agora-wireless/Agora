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
#include "stats.hpp"
// #include "mkl_dfti.h"
class Consumer;

class Reciprocity
{
public:
    Reciprocity(Config *cfg, int in_tid, Consumer& in_consumer,
        Table<complex_float> &in_calib_buffer, Table<complex_float> &in_recip_buffer, 
        Stats *in_stats_manager);
    ~Reciprocity();

    void computeReciprocityCalib(int offset);

private:
    Config *config_;
    int BS_ANT_NUM, UE_NUM;
    int OFDM_CA_NUM, OFDM_DATA_NUM;
    int ref_ant;

    int tid;
    Consumer& consumer_;

    Table<complex_float> calib_buffer_;
    Table<complex_float> recip_buffer_;

    Table<double>* RC_task_duration;
    int *RC_task_count;

    /** 
     * Intermediate buffer to gather CSI
     * First dimension: TASK_THREAD_NUM
     * Second dimension: BS_ANT_NUM * UE_NUM */
    complex_float *calib_gather_buffer;


};


#endif
