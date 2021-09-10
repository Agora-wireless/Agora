#include "Symbols.hpp"
#include "encoder.hpp"
#include "gettime.h"
#include "phy_ldpc_decoder_5gnr.h"
#include "utils_ldpc.hpp"
#include <algorithm>
#include <assert.h>
#include <bitset>
#include <fstream>
#include <malloc.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <vector>

int8_t src[10000];
uint8_t dst[10000];

int main() {
    struct bblib_ldpc_decoder_5gnr_request ldpc_decoder_5gnr_request {
    };
    struct bblib_ldpc_decoder_5gnr_response ldpc_decoder_5gnr_response {
    };

    int16_t* resp_var_nodes = (int16_t*)memalign(64, 1024 * 1024 * sizeof(int16_t)); 

    // Decoder setup
    int16_t numFillerBits = 0;
    size_t nRows = 46;
    uint32_t cbCodewLen = ldpc_num_encoded_bits(1, 104, nRows);
    uint32_t cbLen = ldpc_num_input_bits(1, 104);
    // int16_t numChannelLlrs = LDPC_config.cbCodewLen;
    int16_t numChannelLlrs = cbCodewLen;

    ldpc_decoder_5gnr_request.numChannelLlrs = numChannelLlrs;
    ldpc_decoder_5gnr_request.numFillerBits = numFillerBits;
    ldpc_decoder_5gnr_request.maxIterations = 5;
    ldpc_decoder_5gnr_request.enableEarlyTermination
        = 1;
    // ldpc_decoder_5gnr_request.Zc = LDPC_config.Zc;
    ldpc_decoder_5gnr_request.Zc = 104;
    // ldpc_decoder_5gnr_request.baseGraph = LDPC_config.Bg;
    ldpc_decoder_5gnr_request.baseGraph = 1;
    // ldpc_decoder_5gnr_request.nRows = LDPC_config.nRows;
    ldpc_decoder_5gnr_request.nRows = nRows;

    // int numMsgBits = LDPC_config.cbLen - numFillerBits;
    int numMsgBits = cbLen - numFillerBits;
    ldpc_decoder_5gnr_response.numMsgBits = numMsgBits;
    ldpc_decoder_5gnr_response.varNodes = resp_var_nodes;

    ldpc_decoder_5gnr_request.varNodes = src;
    ldpc_decoder_5gnr_response.compactedMessageBytes = dst;

    FILE* file = fopen("data/tmp_decode.bin", "rb");
    fread(src, 1, cbCodewLen, file);
    fclose(file);

    size_t start_tsc = rdtsc();
    for (size_t i = 0; i < 10000; i ++)
    bblib_ldpc_decoder_5gnr(
        &ldpc_decoder_5gnr_request, &ldpc_decoder_5gnr_response);
    size_t end_tsc = rdtsc();

    printf("Use: %f us\n", cycles_to_us(end_tsc-start_tsc, measure_rdtsc_freq()));

    return 0;
}