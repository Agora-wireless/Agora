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
        moodycamel::ConcurrentQueue<Event_data>& complete_task_queue,
        moodycamel::ProducerToken* worker_producer_token,
        Table<complex_float>& in_calib_buffer,
        Table<complex_float>& in_recip_buffer, Stats* in_stats_manager);
    ~Reciprocity();
    Event_data launch(int offset);

private:
    int BS_ANT_NUM;
    int OFDM_DATA_NUM;

    Table<complex_float> calib_buffer_;
    Table<complex_float> recip_buffer_;
    DurationStat* duration_stat;

    /**
     * Intermediate buffer to gather CSI
     * First dimension: TASK_THREAD_NUM
     * Second dimension: BS_ANT_NUM * UE_NUM */
    complex_float* calib_gather_buffer;
};

#endif
