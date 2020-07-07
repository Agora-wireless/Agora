/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#ifndef DOCODING
#define DOCODING

#include "Symbols.hpp"
#include "buffer.hpp"
#include "concurrentqueue.h"
#include "config.hpp"
#include "doer.hpp"
#include "gettime.h"
#include "memory_manage.h"
#include "modulation.hpp"
#include "stats.hpp"
#include <armadillo>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <vector>

#include "encoder.hpp"
#include "iobuffer.hpp"
#include "utils_ldpc.hpp"

class DoEncode : public Doer {
public:
    DoEncode(Config* in_config, int in_tid, double freq_ghz,
        moodycamel::ConcurrentQueue<Event_data>& in_task_queue,
        moodycamel::ConcurrentQueue<Event_data>& complete_task_queue,
        moodycamel::ProducerToken* worker_producer_token,
        Table<int8_t>& in_raw_data_buffer, Table<int8_t>& in_encoded_buffer,
        Stats* in_stats_manager);
    ~DoEncode();

    Event_data launch(size_t tag);

private:
    Table<int8_t>& raw_data_buffer_;
    int8_t* parity_buffer; // Intermediate buffer to hold LDPC encoding parity

    // Intermediate buffer to hold LDPC encoding output
    int8_t* encoded_buffer_temp;
    Table<int8_t>& encoded_buffer_;
    DurationStat* duration_stat;
};

class DoDecode : public Doer {
public:
    DoDecode(Config* in_config, int in_tid, double freq_ghz,
        moodycamel::ConcurrentQueue<Event_data>& in_task_queue,
        moodycamel::ConcurrentQueue<Event_data>& complete_task_queue,
        moodycamel::ProducerToken* worker_producer_token,
        Table<int8_t>& in_demod_buffer, Table<uint8_t>& in_decoded_buffer,
        Table<int>& in_decoded_bits_count, Table<int>& in_error_bits_count,
        Stats* in_stats_manager);
    ~DoDecode();

    Event_data launch(size_t tag);

    friend void decode_cont_func(void *_context, void *_tag);

private:
    int16_t* resp_var_nodes;
    Table<int8_t>& llr_buffer_;
    Table<uint8_t>& decoded_buffer_;
    Table<int> decoded_bits_count_;
    Table<int> error_bits_count_;
    DurationStat* duration_stat;
};

class DecodeTag {
public:
    size_t symbol_offset;
    size_t output_offset;
    size_t tag;
    int tid;
};

#endif
