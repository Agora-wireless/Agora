/**
 * @file test_ldpc.cpp
 *
 * @brief Accuracy and performance test for LDPC. The encoder is Millipede's
 * avx2enc - unlike FlexRAN's encoder, avx2enc works with AVX2 (i.e., unlike
 * FlexRAN's encoder, avx2enc does not require AVX-512). The decoder is
 * FlexRAN's decoder, which supports AVX2.
 */

#include "common/gettime.h"
#include "common/utils_ldpc.hpp"
#include "encoder.hpp"
#include "phy_ldpc_decoder_5gnr.h"
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

static constexpr size_t kNumCodeBlocks = 2;
static constexpr size_t kBaseGraph = 1;
static constexpr bool kEnableEarlyTermination = true;
static constexpr size_t kNumFillerBits = 0;
static constexpr size_t kMaxDecoderIters = 20;
static constexpr size_t k5GNRNumPunctured = 2;

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
    double freq_ghz = measure_rdtsc_freq();
    int8_t* input[kNumCodeBlocks];
    int8_t* parity[kNumCodeBlocks];
    int8_t* encoded[kNumCodeBlocks];
    uint8_t* decoded[kNumCodeBlocks];

    std::vector<size_t> zc_vec = { 8, 12, 16, 20, 32, 64, 96, 144, 192 };
    for (const size_t& zc : zc_vec) {
        // This ensures that the input bits after puncturing are byte-aligned.
        // Else we'd have to paste the parity bits at a byte-misaligned start
        // address, which isn't implemented yet.
        assert(zc % 4 == 0);

        const size_t num_input_bits = zc
            * (kBaseGraph == 1 ? avx2enc::BG1_COL_INF_NUM
                               : avx2enc::BG2_COL_INF_NUM);

        // Number of rows of the (non-expanded) base graph used
        const size_t num_rows_bg = (kBaseGraph == 1 ? avx2enc::BG1_ROW_TOTAL
                                                    : avx2enc::BG2_ROW_TOTAL);
        const size_t num_parity_bits = zc * num_rows_bg;

        const size_t num_encoded_bits = zc
            * (kBaseGraph == 1 ? (avx2enc::BG1_COL_TOTAL - k5GNRNumPunctured)
                               : (avx2enc::BG2_COL_TOTAL - k5GNRNumPunctured));

        for (size_t i = 0; i < kNumCodeBlocks; i++) {
            input[i] = new int8_t[bits_to_bytes(num_input_bits)
                + avx2enc::PROC_BYTES]();
            parity[i] = new int8_t[bits_to_bytes(num_parity_bits)
                + avx2enc::PROC_BYTES]();
            encoded[i] = new int8_t[bits_to_bytes(num_encoded_bits)
                + avx2enc::PROC_BYTES]();
            decoded[i] = new uint8_t[bits_to_bytes(num_encoded_bits)
                + avx2enc::PROC_BYTES]();
        }

        // Randomly generate input
        srand(time(NULL));
        for (size_t n = 0; n < kNumCodeBlocks; n++) {
            for (size_t i = 0; i < bits_to_bytes(num_input_bits); i++)
                input[n][i] = (int8_t)rand();
        }

        avx2enc::bblib_ldpc_encoder_5gnr_request req;
        avx2enc::bblib_ldpc_encoder_5gnr_response resp;
        req.baseGraph = kBaseGraph;
        req.Zc = zc;
        req.numberCodeblocks = kNumCodeBlocks;
        for (size_t n = 0; n < kNumCodeBlocks; n++) {
            req.input[n] = input[n];
            resp.output[n] = parity[n];
        }

        size_t encoding_start_tsc = rdtsc();
        avx2enc::ldpc_encoder_avx2(&req, &resp);
        double encoding_us
            = cycles_to_us(rdtsc() - encoding_start_tsc, freq_ghz);
        printf("Encoding: %.3f Mbps wrt input bits (%.3f us per code block)\n",
            num_input_bits * kNumCodeBlocks / encoding_us,
            encoding_us / kNumCodeBlocks);

        for (size_t n = 0; n < kNumCodeBlocks; n++) {
            generate_encoded_buffer(encoded[n], &req, &resp, n);
        }

        // Generate log-likelihood ratios, one byte per input bit
        int8_t* llrs[kNumCodeBlocks];
        for (size_t n = 0; n < kNumCodeBlocks; n++) {
            llrs[n] = reinterpret_cast<int8_t*>(memalign(32, num_encoded_bits));
            for (size_t i = 0; i < num_encoded_bits; i++) {
                uint8_t bit_i = (encoded[n][i / 8] >> (i % 8)) & 1;
                llrs[n][i] = (bit_i == 1 ? -127 : 127);
            }
        }

        // Decoder setup
        struct bblib_ldpc_decoder_5gnr_request ldpc_decoder_5gnr_request = {};
        struct bblib_ldpc_decoder_5gnr_response ldpc_decoder_5gnr_response = {};
        ldpc_decoder_5gnr_request.numChannelLlrs = num_encoded_bits;
        ldpc_decoder_5gnr_request.numFillerBits = kNumFillerBits;
        ldpc_decoder_5gnr_request.maxIterations = kMaxDecoderIters;
        ldpc_decoder_5gnr_request.enableEarlyTermination
            = kEnableEarlyTermination;
        ldpc_decoder_5gnr_request.Zc = zc;
        ldpc_decoder_5gnr_request.baseGraph = kBaseGraph;
        ldpc_decoder_5gnr_request.nRows = num_rows_bg;

        const size_t buffer_len = 1024 * 1024;
        size_t numMsgBits = num_input_bits - kNumFillerBits;
        ldpc_decoder_5gnr_response.numMsgBits = numMsgBits;
        ldpc_decoder_5gnr_response.varNodes = reinterpret_cast<int16_t*>(
            memalign(32, buffer_len * sizeof(int16_t)));

        // Decoding
        size_t decoding_start_tsc = rdtsc();
        for (size_t n = 0; n < kNumCodeBlocks; n++) {
            ldpc_decoder_5gnr_request.varNodes = llrs[n];
            ldpc_decoder_5gnr_response.compactedMessageBytes = decoded[n];
            bblib_ldpc_decoder_5gnr(
                &ldpc_decoder_5gnr_request, &ldpc_decoder_5gnr_response);
        }

        double decoding_us
            = cycles_to_us(rdtsc() - decoding_start_tsc, freq_ghz);
        printf("Decoding: %.3f Mbps wrt output bits (%.3f us per code block)\n",
            num_input_bits * kNumCodeBlocks / decoding_us,
            decoding_us / kNumCodeBlocks);

        // Check for errors
        size_t err_cnt = 0;
        for (size_t n = 0; n < kNumCodeBlocks; n++) {
            uint8_t* input_buffer = (uint8_t*)input[n];
            uint8_t* output_buffer = decoded[n];
            for (size_t i = 0; i < bits_to_bytes(num_input_bits); i++) {
                // printf("input: %i, output: %i\n", input_buffer[i],
                // output_buffer[i]);
                uint8_t error = input_buffer[i] ^ output_buffer[i];
                for (size_t j = 0; j < 8; j++) {
                    err_cnt += error & 1;
                    error >>= 1;
                }
            }
        }

        printf("Number of bit errors = %zu, BER = %.3f\n", err_cnt,
            err_cnt * 1.0 / (kNumCodeBlocks * num_input_bits));

        for (size_t i = 0; i < kNumCodeBlocks; i++) {
            delete[] input[i];
            delete[] parity[i];
            delete[] encoded[i];
            delete[] decoded[i];
            free(llrs[i]);
        }
    }

    return 0;
}