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
#include "offset.h"
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
        Consumer& in_consumer, Table<complex_float>& in_calib_buffer,
        Table<complex_float>& in_recip_buffer, Stats* in_stats_manager);
    ~Reciprocity();
    void launch(int offset);

private:
    // XXX: Can we infer these from Doer::cfg?
    int bs_ant_num;
    int ofdm_data_num;

    Table<complex_float> calib_buffer_;
    Table<complex_float> recip_buffer_;

    Table<double>* RC_task_duration;
    int* RC_task_count;

    /**
     * Intermediate buffer to gather CSI
     * First dimension: TASK_THREAD_NUM
     * Second dimension: bs_ant_num * ue_num */
    complex_float* calib_gather_buffer;
};

#endif
