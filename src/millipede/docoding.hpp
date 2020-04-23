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
#include <stdio.h> /* for fprintf */
#include <string.h> /* for memcpy */
#include <vector>

#include "encoder.hpp"
#include "iobuffer.hpp"
#include "phy_ldpc_decoder_5gnr.h"

// #include "mkl_dfti.h"

class DoEncode : public Doer {
public:
    DoEncode(Config* in_config, int in_tid, double freq_ghz,
        moodycamel::ConcurrentQueue<Event_data>& in_task_queue,
        moodycamel::ConcurrentQueue<Event_data>& complete_task_queue,
        moodycamel::ProducerToken* worker_producer_token,
        Table<int8_t>& in_raw_data_buffer, Table<int8_t>& in_encoded_buffer,
        Stats* in_stats_manager);
    ~DoEncode();

    /**
     * Do Encode task for one code block
     */
    Event_data launch(size_t offset);

private:
    Table<int8_t>& raw_data_buffer_;
    int8_t* encoded_buffer_temp;
    Table<int8_t>& encoded_buffer_;
    struct bblib_ldpc_decoder_5gnr_response ldpc_decoder_5gnr_response {
    };
    DurationStat* duration_stat;
    const int16_t* pShiftMatrix;
    const int16_t* pMatrixNumPerCol;
    const int16_t* pAddr;
    uint8_t i_LS; // i_Ls decides the base matrix entries
    LDPC_ADAPTER_P ldpc_adapter_func;
    LDPC_ENCODER ldpc_encoder_func;

    // buffers for encoders
    __attribute__((aligned(64)))
    int8_t internalBuffer0[BG1_ROW_TOTAL * PROC_BYTES]
        = { 0 };
    __attribute__((aligned(64)))
    int8_t internalBuffer1[BG1_ROW_TOTAL * PROC_BYTES]
        = { 0 };
    __attribute__((aligned(64)))
    int8_t internalBuffer2[BG1_COL_TOTAL * PROC_BYTES]
        = { 0 };
};

class DoDecode : public Doer {
public:
    DoDecode(Config* in_config, int in_tid, double freq_ghz,
        moodycamel::ConcurrentQueue<Event_data>& in_task_queue,
        moodycamel::ConcurrentQueue<Event_data>& complete_task_queue,
        moodycamel::ProducerToken* worker_producer_token,
        Table<int8_t>& in_demod_buffer, Table<uint8_t>& in_decoded_buffer,
        Stats* in_stats_manager);
    ~DoDecode();

    /**
     * Do Decode task for one code block
     */
    Event_data launch(size_t offset);

private:
    Table<int8_t>& llr_buffer_;
    Table<uint8_t>& decoded_buffer_;
    DurationStat* duration_stat;
    struct bblib_ldpc_decoder_5gnr_request ldpc_decoder_5gnr_request {
    };
    struct bblib_ldpc_decoder_5gnr_response ldpc_decoder_5gnr_response {
    };
};

#endif
