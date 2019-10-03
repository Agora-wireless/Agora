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

#include "encoder.hpp"
#include "iobuffer.hpp"
#include "phy_ldpc_decoder_5gnr.h"


// #include "mkl_dfti.h"


class DoCoding
{
public:
    DoCoding(Config *cfg, int in_tid, 
        moodycamel::ConcurrentQueue<Event_data> *in_complete_task_queue, moodycamel::ProducerToken *in_task_ptok,
        int8_t **in_raw_data_buffer, int8_t **in_encoded_buffer, int8_t **in_demod_buffer, uint8_t **in_decoded_buffer, 
        double **in_Encode_task_duration, int *in_Encode_task_count, double **in_Decode_task_duration, int *in_Decode_task_count);
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
    moodycamel::ConcurrentQueue<Event_data> *complete_task_queue_;
    moodycamel::ProducerToken *task_ptok;

    int8_t **raw_data_buffer;
    int8_t **encoded_buffer;
    int8_t **llr_buffer;
    uint8_t **decoded_buffer;

    double **Encode_task_duration;
    int *Encode_task_count;

    double **Decode_task_duration;
    int *Decode_task_count;

    LDPCconfig LDPC_config;

    struct bblib_ldpc_decoder_5gnr_request ldpc_decoder_5gnr_request{};
    struct bblib_ldpc_decoder_5gnr_response ldpc_decoder_5gnr_response{};


};


#endif
