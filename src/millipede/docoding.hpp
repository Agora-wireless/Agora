/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 * 
 */
#ifndef DOCODING
#define DOCODING

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
#include "memory_manage.h"
#include "modulation.hpp"
#include "stats.hpp"

#include "encoder.hpp"
#include "iobuffer.hpp"
#include "phy_ldpc_decoder_5gnr.h"
class Consumer;


// #include "mkl_dfti.h"


class DoCoding
{
public:
  DoCoding(Config *cfg, int in_tid, Consumer &in_consumer,
        Table<int8_t> &in_raw_data_buffer, Table<int8_t> &in_encoded_buffer, Table<int8_t> &in_demod_buffer, Table<uint8_t> &in_decoded_buffer, 
        Stats *in_stats_manager);
    ~DoCoding();

    /**
     * Do Encode task for one code block 
     */
    void Encode(int offset);

    /**
     * Do Decode task for one code block 
     */
    void Decode(int offset);
    
 
private:
    Config *config_;
    int BS_ANT_NUM, UE_NUM;
    int OFDM_CA_NUM, OFDM_DATA_NUM;

    int tid;
    Consumer &consumer_;

    Table<int8_t> &raw_data_buffer_;
    int8_t *encoded_buffer_temp;
    Table<int8_t> &encoded_buffer_;
    Table<int8_t> &llr_buffer_;
    Table<uint8_t> decoded_buffer_;

    Table<double> &Encode_task_duration;
    int *Encode_task_count;

    Table<double> &Decode_task_duration;
    int *Decode_task_count;

    LDPCconfig LDPC_config;

    struct bblib_ldpc_decoder_5gnr_request ldpc_decoder_5gnr_request{};
    struct bblib_ldpc_decoder_5gnr_response ldpc_decoder_5gnr_response{};
    const int16_t *pShiftMatrix;
    const int16_t *pMatrixNumPerCol;
    const int16_t *pAddr;
    uint8_t i_LS;       // i_Ls decides the base matrix entries
    LDPC_ADAPTER_P ldpc_adapter_func;
    LDPC_ENCODER ldpc_encoder_func;

    // buffers for encoders
    __attribute__ ((aligned (64))) int8_t internalBuffer0[BG1_ROW_TOTAL * PROC_BYTES] = {0};
    __attribute__ ((aligned (64))) int8_t internalBuffer1[BG1_ROW_TOTAL * PROC_BYTES] = {0};
    __attribute__ ((aligned (64))) int8_t internalBuffer2[BG1_COL_TOTAL * PROC_BYTES] = {0};

};


#endif
