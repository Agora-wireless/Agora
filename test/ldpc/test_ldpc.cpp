/*
    accuracy and performance test for ldpc encoder implemented with AVX256 and
   Intel's decoder
 */

#include "encoder.hpp"
#include "iobuffer.hpp"
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
#define USE_RDTSC 1

// these values depend on the application
// uint16_t Zc = 14;
int numberCodeblocks = 10000;
uint16_t Bg = 1;
bool earlyTermination = 1;
int16_t numFillerBits = 0;
int16_t decoderIter = 20;

char* read_binfile(std::string filename, int buffer_size)
{
    std::ifstream infile;
    infile.open(filename, std::ios::binary | std::ios::in);

    char* x = (char*)malloc(buffer_size * sizeof(char));

    infile.read((char*)x, buffer_size * sizeof(char));
    infile.close();

    return x;
}

template <typename T>
T* aligned_malloc(const int size, const unsigned alignment)
{
#ifdef _BBLIB_DPDK_
    return (T*)rte_malloc(NULL, sizeof(T) * size, alignment);
#else
#ifndef _WIN64
    return (T*)memalign(alignment, sizeof(T) * size);
#else
    return (T*)_aligned_malloc(sizeof(T) * size, alignment);
#endif
#endif
}

static inline uint64_t RDTSC()
{
    unsigned int hi, lo;
    __asm__ volatile("rdtsc" : "=a"(lo), "=d"(hi));
    return ((uint64_t)hi << 32) | lo;
};

static inline double get_time(void)
{
#if USE_RDTSC
    return double(RDTSC()) / 2.3e3;
#else
    struct timespec tv;
    clock_gettime(CLOCK_MONOTONIC, &tv);
    return tv.tv_sec * 1000000 + tv.tv_nsec / 1000.0;
#endif
};

int main()
{

    printf("--------5gnr ldpc test--------\n");
    clock_t t_enc_start, t_enc_end, t_dec_start, t_dec_end;
    double t_encoder[numberCodeblocks];
    double t_enc_total = 0;
    double t_decoder[numberCodeblocks];
    double t_dec_total = 0;

    int8_t* input[numberCodeblocks];
    int8_t* encoded[numberCodeblocks];
    uint8_t* decoded[numberCodeblocks];
    uint16_t Zc_array[12] = { 2, 8, 10, 12, 14, 16, 20, 32, 64, 96, 144, 192 };
    for (int zc_itr = 0; zc_itr < 12; zc_itr++) {
        uint16_t Zc = Zc_array[zc_itr];
        int nRows = (Bg == 1) ? 46 : 42;
        uint32_t cbEncLen = nRows * Zc;
        uint32_t cbLen = (Bg == 1) ? Zc * 22 : Zc * 10;
        uint32_t cbCodewLen = (Bg == 1) ? Zc * 66 : Zc * 50;

        for (int i = 0; i < numberCodeblocks; i++) {
            input[i] = (int8_t*)malloc(((cbLen + 7) >> 3) * sizeof(int8_t));
            encoded[i]
                = (int8_t*)malloc(BG1_COL_TOTAL * PROC_BYTES * sizeof(int8_t));
            decoded[i] = (uint8_t*)malloc(((cbLen + 7) >> 3) * sizeof(uint8_t));
        }

        printf("Zc: %d, code block len: %d, encoded block len: %d, encoder "
               "buf: %d Bytes, decoder buf: %d Bytes, decoder iterations: %d\n",
            Zc, cbLen, cbCodewLen, BG1_COL_TOTAL * PROC_BYTES, (cbLen + 7) >> 3,
            decoderIter);

        // Buffers for encoders
        __declspec(align(PROC_BYTES))
            int8_t internalBuffer0[BG1_ROW_TOTAL * PROC_BYTES]
            = { 0 };
        __declspec(align(PROC_BYTES))
            int8_t internalBuffer1[BG1_ROW_TOTAL * PROC_BYTES]
            = { 0 };
        __declspec(align(PROC_BYTES))
            int8_t internalBuffer2[BG1_COL_TOTAL * PROC_BYTES]
            = { 0 };

        // Randomly generate input
        srand(time(NULL));
        for (int n = 0; n < numberCodeblocks; n++) {
            int8_t* p_input = input[n];
            for (int i = 0; i < ((cbLen + 7) >> 3); i++)
                p_input[i] = (int8_t)rand();
        }

        // Encoder setup
        int16_t numChannelLlrs = cbCodewLen;
        const int16_t* pShiftMatrix;
        const int16_t* pMatrixNumPerCol;
        const int16_t* pAddr;

        uint8_t i_LS; // i_Ls decides the base matrix entries
        if ((Zc % 15) == 0)
            i_LS = 7;
        else if ((Zc % 13) == 0)
            i_LS = 6;
        else if ((Zc % 11) == 0)
            i_LS = 5;
        else if ((Zc % 9) == 0)
            i_LS = 4;
        else if ((Zc % 7) == 0)
            i_LS = 3;
        else if ((Zc % 5) == 0)
            i_LS = 2;
        else if ((Zc % 3) == 0)
            i_LS = 1;
        else
            i_LS = 0;

        if (Bg == 1) {
            pShiftMatrix = Bg1HShiftMatrix + i_LS * BG1_NONZERO_NUM;
            pMatrixNumPerCol = Bg1MatrixNumPerCol;
            pAddr = Bg1Address;
        } else {
            pShiftMatrix = Bg2HShiftMatrix + i_LS * BG2_NONZERO_NUM;
            pMatrixNumPerCol = Bg2MatrixNumPerCol;
            pAddr = Bg2Address;
        }

        // encoding
        // --------------------------------------------------------------------
        printf("encoding----------------------\n");
        LDPC_ADAPTER_P ldpc_adapter_func = ldpc_select_adapter_func(Zc);
        LDPC_ENCODER ldpc_encoder_func = ldpc_select_encoder_func(Bg);

        double start_time = get_time();
        for (int n = 0; n < numberCodeblocks; n++) {
            // t_enc_start = clock();
            // read input into z-bit segments
            ldpc_adapter_func(input[n], internalBuffer0, Zc, cbLen, 1);
            // encode
            ldpc_encoder_func(internalBuffer0, internalBuffer1,
                pMatrixNumPerCol, pAddr, pShiftMatrix, (int16_t)Zc, i_LS);
            // scatter the output back to compacted
            // t_enc_end = clock();
            // combine the input sequence and the parity bits into codeword
            // outputs
            memcpy(internalBuffer2, internalBuffer0 + 2 * PROC_BYTES,
                (cbLen / Zc - 2) * PROC_BYTES);
            memcpy(internalBuffer2 + (cbLen / Zc - 2) * PROC_BYTES,
                internalBuffer1, cbEncLen / Zc * PROC_BYTES);

            ldpc_adapter_func(encoded[n], internalBuffer2, Zc, cbCodewLen, 0);

            // t_encoder[n] = (t_enc_start-t_enc_end)/CLOCKS_PER_SEC*(1e9);
            // t_enc_total = t_enc_total + t_encoder[n];

            // printf("the encoding for the %dth code block took %f nano
            // seconds\n", n, t_encoder[n]);
        }
        double end_time = get_time();
        printf("encoding time: %.3f\n",
            (end_time - start_time) / numberCodeblocks);
        // generate llrs
        // replace this with channel output
        int8_t* llrs[numberCodeblocks];
        for (int n = 0; n < numberCodeblocks; n++) {
            llrs[n] = aligned_malloc<int8_t>(cbCodewLen * sizeof(int8_t), 32);
        }

        for (int n = 0; n < numberCodeblocks; n++) {
            for (int i = 0; i < cbCodewLen; i++) {
                uint8_t msgbyte = encoded[n][i / 8];
                if ((msgbyte >> (i % 8)) & 1)
                    llrs[n][i] = -127;
                else
                    llrs[n][i] = 127;
            }
        }

        // Decoder setup
        struct bblib_ldpc_decoder_5gnr_request ldpc_decoder_5gnr_request {
        };
        struct bblib_ldpc_decoder_5gnr_response ldpc_decoder_5gnr_response {
        };

        ldpc_decoder_5gnr_request.numChannelLlrs = numChannelLlrs;
        ldpc_decoder_5gnr_request.numFillerBits = numFillerBits;
        ldpc_decoder_5gnr_request.maxIterations = decoderIter;
        ldpc_decoder_5gnr_request.enableEarlyTermination = earlyTermination;
        const long int buffer_len = 1024 * 1024;
        ldpc_decoder_5gnr_request.Zc = Zc;
        ldpc_decoder_5gnr_request.baseGraph = Bg;
        ldpc_decoder_5gnr_request.nRows = nRows;

        int numMsgBits = cbLen - numFillerBits;
        int numMsgBytes = (numMsgBits + 7) / 8;
        ldpc_decoder_5gnr_response.numMsgBits = numMsgBits;
        ldpc_decoder_5gnr_response.varNodes
            = aligned_malloc<int16_t>(buffer_len, 32);

        // Decoding
        start_time = get_time();
        for (int n = 0; n < numberCodeblocks; n++) {
            ldpc_decoder_5gnr_request.varNodes = llrs[n];
            ldpc_decoder_5gnr_response.compactedMessageBytes = decoded[n];
            bblib_ldpc_decoder_5gnr(
                &ldpc_decoder_5gnr_request, &ldpc_decoder_5gnr_response);
        }
        end_time = get_time();
        printf("decoding time: %.3f\n",
            (end_time - start_time) / numberCodeblocks);

        printf("results---------------------\n");
        printf("there are %d code blocks in total\n", numberCodeblocks);

        int err_cnt = 0;
        for (int n = 0; n < numberCodeblocks; n++) {
            uint8_t* input_buffer = (uint8_t*)input[n];
            uint8_t* output_buffer = decoded[n];
            for (int i = 0; i < (cbLen >> 3); i++) {
                // printf("input: %i, output: %i\n", input_buffer[i],
                // output_buffer[i]);
                uint8_t error = input_buffer[i] ^ output_buffer[i];
                for (int j = 0; j < 8; j++) {
                    err_cnt += error & 1;
                    error >>= 1;
                }
            }
        }

        double ber = (double)err_cnt / cbLen / numberCodeblocks;
        printf("the bit error rate is %f\n", ber);

        double enc_thruput
            = (double)cbLen * numberCodeblocks / t_enc_total * 125;
        printf("the encoder's speed is %f MB/sec\n", enc_thruput);
        double dec_thruput
            = (double)cbLen * numberCodeblocks / t_dec_total * 125;
        printf("the decoder's speed is %f MB/sec\n", dec_thruput);

        for (int n = 0; n < numberCodeblocks; n++) {
            free(input[n]);
            free(encoded[n]);
            free(llrs[n]);
            free(decoded[n]);
        }
    }

    return 0;
}