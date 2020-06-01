/*
    accuracy and performance test for ldpc encoder implemented with AVX256 and
   Intel's decoder
 */

#include "common/gettime.h"
#include "encoder/encoder.hpp"
#include "encoder/iobuffer.hpp"
#include "phy_ldpc_decoder_5gnr.h"
#include <bitset>
#include <fstream>
#include <immintrin.h>
#include <iostream>
#include <malloc.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <vector>

static constexpr size_t kNumCodeBlocks = 10000;
static constexpr size_t kBaseGraph = 1;
static constexpr bool kEnableEarlyTermination = true;
static constexpr size_t kNumFillerBits = 0;
static constexpr size_t kMaxDecoderIters = 20;

char* read_binfile(std::string filename, size_t buffer_size)
{
    std::ifstream infile;
    infile.open(filename, std::ios::binary | std::ios::in);

    char* x = (char*)malloc(buffer_size * sizeof(char));
    infile.read((char*)x, buffer_size * sizeof(char));
    infile.close();
    return x;
}

int main()
{
    int8_t* input[kNumCodeBlocks];
    int8_t* encoded[kNumCodeBlocks];
    uint8_t* decoded[kNumCodeBlocks];

    // Buffers for encoders
    int8_t* internalBuffer0 = reinterpret_cast<int8_t*>(
        memalign(PROC_BYTES, BG1_ROW_TOTAL * PROC_BYTES * sizeof(int8_t)));
    int8_t* internalBuffer1 = reinterpret_cast<int8_t*>(
        memalign(PROC_BYTES, BG1_ROW_TOTAL * PROC_BYTES * sizeof(int8_t)));
    int8_t* internalBuffer2 = reinterpret_cast<int8_t*>(
        memalign(PROC_BYTES, BG1_COL_TOTAL * PROC_BYTES * sizeof(int8_t)));

    double encode_total_us = 0; // Total microseconds spent in encode
    double decode_total_us = 0; // Total microseconds spent in decode

    std::vector<size_t> zc_vec
        = { 2, 8, 10, 12, 14, 16, 20, 32, 64, 96, 144, 192 };
    for (const size_t& zc : zc_vec) {
        const size_t nRows = (kBaseGraph == 1) ? 46 : 42;
        const size_t cbEncLen = nRows * zc;
        const size_t cbLen = (kBaseGraph == 1) ? zc * 22 : zc * 10;
        const size_t cbCodewLen = (kBaseGraph == 1) ? zc * 66 : zc * 50;

        for (size_t i = 0; i < kNumCodeBlocks; i++) {
            input[i] = (int8_t*)malloc(((cbLen + 7) >> 3) * sizeof(int8_t));
            encoded[i]
                = (int8_t*)malloc(BG1_COL_TOTAL * PROC_BYTES * sizeof(int8_t));
            decoded[i] = (uint8_t*)malloc(((cbLen + 7) >> 3) * sizeof(uint8_t));
        }

        printf("Zc: %zu, code block len: %zu, encoded block len: %zu, encoder "
               "buf: %d Bytes, decoder buf: %zu Bytes, max decoder iters %zu\n",
            zc, cbLen, cbCodewLen, BG1_COL_TOTAL * PROC_BYTES, (cbLen + 7) >> 3,
            kMaxDecoderIters);

        memset(internalBuffer0, 0, BG1_ROW_TOTAL * PROC_BYTES * sizeof(int8_t));
        memset(internalBuffer1, 0, BG1_ROW_TOTAL * PROC_BYTES * sizeof(int8_t));
        memset(internalBuffer2, 0, BG1_COL_TOTAL * PROC_BYTES * sizeof(int8_t));

        // Randomly generate input
        srand(time(NULL));
        for (size_t n = 0; n < kNumCodeBlocks; n++) {
            int8_t* p_input = input[n];
            for (size_t i = 0; i < ((cbLen + 7) >> 3); i++)
                p_input[i] = (int8_t)rand();
        }

        // Encoder setup
        int16_t numChannelLlrs = cbCodewLen;
        const int16_t* pShiftMatrix;
        const int16_t* pMatrixNumPerCol;
        const int16_t* pAddr;

        uint8_t i_LS; // i_Ls decides the base matrix entries
        if ((zc % 15) == 0)
            i_LS = 7;
        else if ((zc % 13) == 0)
            i_LS = 6;
        else if ((zc % 11) == 0)
            i_LS = 5;
        else if ((zc % 9) == 0)
            i_LS = 4;
        else if ((zc % 7) == 0)
            i_LS = 3;
        else if ((zc % 5) == 0)
            i_LS = 2;
        else if ((zc % 3) == 0)
            i_LS = 1;
        else
            i_LS = 0;

        if (kBaseGraph == 1) {
            pShiftMatrix = Bg1HShiftMatrix + i_LS * BG1_NONZERO_NUM;
            pMatrixNumPerCol = Bg1MatrixNumPerCol;
            pAddr = Bg1Address;
        } else {
            pShiftMatrix = Bg2HShiftMatrix + i_LS * BG2_NONZERO_NUM;
            pMatrixNumPerCol = Bg2MatrixNumPerCol;
            pAddr = Bg2Address;
        }

        printf("Encoding\n");
        LDPC_ADAPTER_P ldpc_adapter_func = ldpc_select_adapter_func(zc);
        LDPC_ENCODER ldpc_encoder_func = ldpc_select_encoder_func(kBaseGraph);

        double start_time_us = get_time();
        for (size_t n = 0; n < kNumCodeBlocks; n++) {
            // Read input into z-bit segments, then encode
            ldpc_adapter_func(input[n], internalBuffer0, zc, cbLen, 1);
            ldpc_encoder_func(internalBuffer0, internalBuffer1,
                pMatrixNumPerCol, pAddr, pShiftMatrix, (int16_t)zc, i_LS);

            // Combine input sequence and parity bits into codeword outputs
            memcpy(internalBuffer2, internalBuffer0 + 2 * PROC_BYTES,
                (cbLen / zc - 2) * PROC_BYTES);
            memcpy(internalBuffer2 + (cbLen / zc - 2) * PROC_BYTES,
                internalBuffer1, cbEncLen / zc * PROC_BYTES);
            ldpc_adapter_func(encoded[n], internalBuffer2, zc, cbCodewLen, 0);
        }
        double end_time_us = get_time();
        printf("Encoding time: %.3f us per code block\n",
            (end_time_us - start_time_us) / kNumCodeBlocks);
        encode_total_us += (end_time_us - start_time_us);

        // Generate llrs (replace this with channel output)
        int8_t* llrs[kNumCodeBlocks];
        for (size_t n = 0; n < kNumCodeBlocks; n++) {
            llrs[n] = reinterpret_cast<int8_t*>(memalign(32, cbCodewLen));
        }

        for (size_t n = 0; n < kNumCodeBlocks; n++) {
            for (size_t i = 0; i < cbCodewLen; i++) {
                uint8_t msgbyte = encoded[n][i / 8];
                if ((msgbyte >> (i % 8)) & 1)
                    llrs[n][i] = -127;
                else
                    llrs[n][i] = 127;
            }
        }

        // Decoder setup
        struct bblib_ldpc_decoder_5gnr_request ldpc_decoder_5gnr_request = {};
        struct bblib_ldpc_decoder_5gnr_response ldpc_decoder_5gnr_response = {};
        ldpc_decoder_5gnr_request.numChannelLlrs = numChannelLlrs;
        ldpc_decoder_5gnr_request.numFillerBits = kNumFillerBits;
        ldpc_decoder_5gnr_request.maxIterations = kMaxDecoderIters;
        ldpc_decoder_5gnr_request.enableEarlyTermination
            = kEnableEarlyTermination;
        ldpc_decoder_5gnr_request.Zc = zc;
        ldpc_decoder_5gnr_request.baseGraph = kBaseGraph;
        ldpc_decoder_5gnr_request.nRows = nRows;

        const size_t buffer_len = 1024 * 1024;
        size_t numMsgBits = cbLen - kNumFillerBits;
        size_t numMsgBytes = (numMsgBits + 7) / 8;
        ldpc_decoder_5gnr_response.numMsgBits = numMsgBits;
        ldpc_decoder_5gnr_response.varNodes = reinterpret_cast<int16_t*>(
            memalign(32, buffer_len * sizeof(int16_t)));

        // Decoding
        start_time_us = get_time_us();
        for (size_t n = 0; n < kNumCodeBlocks; n++) {
            ldpc_decoder_5gnr_request.varNodes = llrs[n];
            ldpc_decoder_5gnr_response.compactedMessageBytes = decoded[n];
            bblib_ldpc_decoder_5gnr(
                &ldpc_decoder_5gnr_request, &ldpc_decoder_5gnr_response);
        }
        end_time_us = get_time_us();
        printf("Decoding time: %.3f us per code block\n",
            (end_time_us - start_time_us) / kNumCodeBlocks);
        decode_total_us += (end_time_us - start_time_us);

        printf("Results for %zu code blocks: \n", kNumCodeBlocks);
        size_t err_cnt = 0;
        for (size_t n = 0; n < kNumCodeBlocks; n++) {
            uint8_t* input_buffer = (uint8_t*)input[n];
            uint8_t* output_buffer = decoded[n];
            for (size_t i = 0; i < (cbLen >> 3); i++) {
                // printf("input: %i, output: %i\n", input_buffer[i],
                // output_buffer[i]);
                uint8_t error = input_buffer[i] ^ output_buffer[i];
                for (size_t j = 0; j < 8; j++) {
                    err_cnt += error & 1;
                    error >>= 1;
                }
            }
        }

        double ber = (double)err_cnt / cbLen / kNumCodeBlocks;
        printf("BER = %.3f\n", ber);
        printf("Encoder: %.2f MB/sec\n",
            (cbLen / 8.0) * kNumCodeBlocks / encode_total_us);
        printf("Decoder: %.2f MB/sec\n",
            (cbLen / 8.0) * kNumCodeBlocks / decode_total_us);

        for (size_t n = 0; n < kNumCodeBlocks; n++) {
            free(input[n]);
            free(encoded[n]);
            free(llrs[n]);
            free(decoded[n]);
        }
    }

    return 0;
}