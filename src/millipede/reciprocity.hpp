/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#ifndef DORECIP
#define DORECIP

#include "Symbols.hpp"
#include "buffer.hpp"
#include "concurrentqueue.h"
#include "config.hpp"
#include "doer.hpp"
#include "gettime.h"
#include "stats.hpp"
#include "utils.h"
#include <armadillo>
#include <iostream>
#include <stdio.h> /* for fprintf */
#include <string.h> /* for memcpy */
#include <vector>
// #include "mkl_dfti.h"

class Reciprocity : public Doer {
public:
    Reciprocity(Config* cfg, int in_tid,
        moodycamel::ConcurrentQueue<Event_data>& in_task_queue,
        Consumer& in_consumer, Table<complex_short>& in_calib_buffer,
        Table<complex_float>& in_recip_buffer, Stats* in_stats_manager);
    ~Reciprocity();
    Event_data launch(int offset);

private:
    int BS_ANT_NUM;
    int OFDM_DATA_NUM;

    Table<complex_short> calib_buffer_;
    complex_float* calib_buffer_float;
    Table<complex_float> recip_buffer_;

    Table<double>* RC_task_duration;
    int* RC_task_count;

    /**
     * Intermediate buffer to gather CSI
     * First dimension: TASK_THREAD_NUM
     * Second dimension: BS_ANT_NUM * UE_NUM */
    complex_float* calib_gather_buffer;
};

#endif
