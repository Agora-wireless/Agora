/**
 * @file test_ldpc_flexran.cpp
 *
 * @brief Accuracy and performance test for LDPC. 
 * The encoder is FlexRAN's encoder (requires AVX512 support). 
 * The decoder is FlexRAN's decoder.
 */

#include "common/Symbols.hpp"
#include "common/gettime.h"
#include "common/utils_ldpc.hpp"
#include "encoder.hpp"
#include "phy_ldpc_decoder_5gnr.h"
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

static constexpr size_t kNumCodeBlocks = 1;
static constexpr size_t kBaseGraph = 1;
static constexpr bool kEnableEarlyTermination = true;
static constexpr size_t kNumFillerBits = 0;
static constexpr size_t kMaxDecoderIters = 20;
static constexpr size_t k5GNRNumPunctured = 2;
static constexpr size_t kNumRows = 46;

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
    std::vector<size_t> zc_vec = { 2, 4, 8, 16, 32, 64, 128, 256, 3, 6, 12, 24,
        48, 96, 192, 384, 5, 10, 20, 40, 80, 160, 320, 7, 14, 28, 56, 112, 224,
        9, 18, 36, 72, 144, 288, 11, 22, 44, 88, 176, 352, 13, 26, 52, 104, 208,
        15, 30, 60, 120, 240 };

    std::sort(zc_vec.begin(), zc_vec.end());
    for (const size_t& zc : zc_vec) {
        if (zc == 2 || zc == 3 || zc == 5) {
            fprintf(stderr,
                "Zc value %zu not supported by FlexRAN encoder. Skipping.\n",
                zc);
            continue;
        }
        const size_t num_input_bits = ldpc_num_input_bits(kBaseGraph, zc);
        const size_t num_parity_bits = ldpc_num_parity_bits(kBaseGraph, zc);
        const size_t num_encoded_bits = ldpc_num_encoded_bits(kBaseGraph, zc);

        for (size_t i = 0; i < kNumCodeBlocks; i++) {
            input[i] = (int8_t*)memalign(
                64, ldpc_encoding_input_buf_size(kBaseGraph, zc));
            parity[i] = (int8_t*)memalign(
                64, ldpc_encoding_parity_buf_size(kBaseGraph, zc));
            encoded[i] = (int8_t*)memalign(
                64, ldpc_encoding_encoded_buf_size(kBaseGraph, zc));
            decoded[i] = (uint8_t*)memalign(
                64, ldpc_encoding_encoded_buf_size(kBaseGraph, zc));
        }

        // Randomly generate input
        srand(time(NULL));
        for (size_t n = 0; n < kNumCodeBlocks; n++) {
            for (size_t i = 0; i < bits_to_bytes(num_input_bits); i++)
                input[n][i] = (int8_t)rand();
        }

        const size_t encoding_start_tsc = rdtsc();
        for (size_t n = 0; n < kNumCodeBlocks; n++) {
            ldpc_encode_helper_avx512(
                kBaseGraph, zc, kNumRows, encoded[n], parity[n], input[n]);
        }

        const double encoding_us
            = cycles_to_us(rdtsc() - encoding_start_tsc, freq_ghz);

        // For decoding, generate log-likelihood ratios, one byte per input bit
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
        ldpc_decoder_5gnr_request.nRows = num_parity_bits / zc;

        const size_t buffer_len = 1024 * 1024;
        const size_t numMsgBits = num_input_bits - kNumFillerBits;
        ldpc_decoder_5gnr_response.numMsgBits = numMsgBits;
        ldpc_decoder_5gnr_response.varNodes = reinterpret_cast<int16_t*>(
            memalign(64, buffer_len * sizeof(int16_t)));

        // Decoding
        const size_t decoding_start_tsc = rdtsc();
        for (size_t n = 0; n < kNumCodeBlocks; n++) {
            ldpc_decoder_5gnr_request.varNodes = llrs[n];
            ldpc_decoder_5gnr_response.compactedMessageBytes = decoded[n];
            bblib_ldpc_decoder_5gnr(
                &ldpc_decoder_5gnr_request, &ldpc_decoder_5gnr_response);
        }

        const double decoding_us
            = cycles_to_us(rdtsc() - decoding_start_tsc, freq_ghz);

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
                    if (i * 8 + j >= num_input_bits) {
                        continue; // Don't compare beyond end of input bits
                    }
                    err_cnt += error & 1;
                    error >>= 1;
                }
            }
        }

        printf("Zc = %zu, {encoding, decoding}: {%.2f, %.2f} Mbps, {%.2f, "
               "%.2f} us per code block. Bit errors = %zu, BER = %.3f\n",
            zc, num_input_bits * kNumCodeBlocks / encoding_us,
            num_input_bits * kNumCodeBlocks / decoding_us,
            encoding_us / kNumCodeBlocks, decoding_us / kNumCodeBlocks, err_cnt,
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