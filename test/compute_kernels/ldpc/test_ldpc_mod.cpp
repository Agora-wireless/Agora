/**
 * @file test_ldpc_baseband.cpp
 * @brief Test LDPC performance after encoding, modulation, demodulation, 
 * and decoding when different levels of 
 * Gaussian noise is added to CSI
 */

#include "comms-lib.h"
#include "config.hpp"
#include "data_generator.h"
#include "gettime.h"
#include "memory_manage.h"
#include "modulation.hpp"
#include "phy_ldpc_decoder_5gnr.h"
#include "utils_ldpc.hpp"
#include <armadillo>
#include <bitset>
#include <chrono>
#include <fstream>
#include <gflags/gflags.h>
#include <immintrin.h>
#include <iostream>
#include <random>

static constexpr bool kVerbose = false;
static constexpr bool kPrintUplinkInformationBytes = false;
static constexpr float noise_levels[15]
    = { 1.7783, 1.3335, 1.0000, 0.7499, 0.5623, 0.4217, 0.3162, 0.2371, 0.1778,
          0.1334, 0.1000, 0.0750, 0.0562, 0.0422, 0.0316 };
static constexpr float snr_levels[15]
    = { -5, -2.5, 0, 2.5, 5, 7.5, 10, 12.5, 15, 17.5, 20, 22.5, 25, 27.5, 30 };
DEFINE_string(profile, "random",
    "The profile of the input user bytes (e.g., 'random', '123')");
DEFINE_string(conf_file,
    TOSTRING(PROJECT_DIRECTORY) "/data/tddconfig-sim-ul.json",
    "Agora config filename");

float rand_float(float min, float max)
{
    return ((float(rand()) / float(RAND_MAX)) * (max - min)) + min;
}

float rand_float_from_short(float min, float max)
{
    float rand_val = ((float(rand()) / float(RAND_MAX)) * (max - min)) + min;
    short rand_val_ushort = (short)(rand_val * 32768);
    rand_val = (float)rand_val_ushort / 32768;
    return rand_val;
}

int main(int argc, char* argv[])
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::normal_distribution<double> distribution(0.0, 1.0);

    const std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    auto* cfg = new Config(FLAGS_conf_file.c_str());

    const DataGenerator::Profile profile = FLAGS_profile == "123"
        ? DataGenerator::Profile::k123
        : DataGenerator::Profile::kRandom;
    DataGenerator data_generator(cfg, 0 /* RNG seed */, profile);

    std::printf("DataGenerator: Config file: %s, data profile = %s\n",
        FLAGS_conf_file.c_str(),
        profile == DataGenerator::Profile::k123 ? "123" : "random");

    std::printf("DataGenerator: Using %s-orthogonal pilots\n",
        cfg->freq_orthogonal_pilot ? "frequency" : "time");

    std::printf("DataGenerator: Generating encoded and modulated data\n");
    srand(time(nullptr));

    // Step 1: Generate the information buffers and LDPC-encoded buffers for
    // uplink
    size_t num_symbols_per_cb = 1;
    size_t bits_per_symbol = cfg->OFDM_DATA_NUM * cfg->mod_order_bits;
    if (cfg->LDPC_config.cbCodewLen > bits_per_symbol)
        num_symbols_per_cb = (cfg->LDPC_config.cbCodewLen + bits_per_symbol - 1)
            / bits_per_symbol;
    size_t num_cbs_per_ue = cfg->data_symbol_num_perframe / num_symbols_per_cb;
    std::printf("Number of symbols per block: %zu, blocks per frame: %zu\n",
        num_symbols_per_cb, num_cbs_per_ue);

    const size_t num_codeblocks = num_cbs_per_ue * cfg->UE_ANT_NUM;
    std::printf("Total number of blocks: %zu\n", num_codeblocks);
    for (size_t noise_id = 0; noise_id < 15; noise_id++) {

        std::vector<std::vector<int8_t>> information(num_codeblocks);
        std::vector<std::vector<int8_t>> encoded_codewords(num_codeblocks);
        for (size_t i = 0; i < num_codeblocks; i++) {
            data_generator.gen_codeblock(information[i],
                encoded_codewords[i], i % cfg->UE_NUM /* UE ID */);
        }

        // Save uplink information bytes to file
        const size_t input_bytes_per_cb = bits_to_bytes(
            ldpc_num_input_bits(cfg->LDPC_config.Bg, cfg->LDPC_config.Zc));
        if (kPrintUplinkInformationBytes) {
            std::printf("Uplink information bytes\n");
            for (size_t n = 0; n < num_codeblocks; n++) {
                std::printf("Symbol %zu, UE %zu\n", n / cfg->UE_ANT_NUM,
                    n % cfg->UE_ANT_NUM);
                for (size_t i = 0; i < input_bytes_per_cb; i++) {
                    std::printf("%u ", (uint8_t)information[n][i]);
                }
                std::printf("\n");
            }
        }

        Table<complex_float> modulated_codewords;
        modulated_codewords.calloc(num_codeblocks, cfg->OFDM_DATA_NUM,
            Agora_memory::Alignment_t::k64Align);
        Table<int8_t> demod_data_all_symbols;
        demod_data_all_symbols.calloc(num_codeblocks, cfg->OFDM_DATA_NUM * 8,
            Agora_memory::Alignment_t::k64Align);
        std::vector<uint8_t> mod_input(cfg->OFDM_DATA_NUM);

        // Modulate, add noise, and demodulate the encoded codewords
        for (size_t i = 0; i < num_codeblocks; i++) {

            adapt_bits_for_mod(
                reinterpret_cast<const uint8_t*>(&encoded_codewords[i][0]),
                &mod_input[0], cfg->LDPC_config.num_encoded_bytes(),
                cfg->mod_order_bits);

            for (size_t j = 0; j < cfg->OFDM_DATA_NUM; j++) {
                modulated_codewords[i][j]
                    = mod_single_uint8(mod_input[j], cfg->mod_table);
            }

            for (size_t j = 0; j < cfg->OFDM_DATA_NUM; j++) {
                complex_float noise
                    = { static_cast<float>(distribution(generator))
                              * noise_levels[noise_id],
                          static_cast<float>(distribution(generator))
                              * noise_levels[noise_id] };
                modulated_codewords[i][j].re
                    = modulated_codewords[i][j].re + noise.re;
                modulated_codewords[i][j].im
                    = modulated_codewords[i][j].im + noise.im;
            }

            switch (cfg->mod_order_bits) {
            case (4):
                demod_16qam_soft_avx2((float*)modulated_codewords[i],
                    demod_data_all_symbols[i], cfg->OFDM_DATA_NUM);
                break;
            case (6):
                demod_64qam_soft_avx2((float*)modulated_codewords[i],
                    demod_data_all_symbols[i], cfg->OFDM_DATA_NUM);
                break;
            default:
                std::printf("Demodulation: modulation type %s not supported!\n",
                    cfg->modulation.c_str());
            }
        }

        LDPCconfig LDPC_config = cfg->LDPC_config;

        struct bblib_ldpc_decoder_5gnr_request ldpc_decoder_5gnr_request {
        };
        struct bblib_ldpc_decoder_5gnr_response ldpc_decoder_5gnr_response {
        };

        // Decoder setup
        ldpc_decoder_5gnr_request.numChannelLlrs = LDPC_config.cbCodewLen;
        ldpc_decoder_5gnr_request.numFillerBits = 0;
        ldpc_decoder_5gnr_request.maxIterations = LDPC_config.decoderIter;
        ldpc_decoder_5gnr_request.enableEarlyTermination
            = LDPC_config.earlyTermination;
        ldpc_decoder_5gnr_request.Zc = LDPC_config.Zc;
        ldpc_decoder_5gnr_request.baseGraph = LDPC_config.Bg;
        ldpc_decoder_5gnr_request.nRows = LDPC_config.nRows;
        ldpc_decoder_5gnr_response.numMsgBits = LDPC_config.cbLen;
        auto* resp_var_nodes
            = static_cast<int16_t*>(Agora_memory::padded_aligned_alloc(
                Agora_memory::Alignment_t::k64Align,
                1024 * 1024 * sizeof(int16_t)));
        ldpc_decoder_5gnr_response.varNodes = resp_var_nodes;

        Table<uint8_t> decoded_codewords;
        decoded_codewords.calloc(num_codeblocks, cfg->OFDM_DATA_NUM,
            Agora_memory::Alignment_t::k64Align);

        double freq_ghz = measure_rdtsc_freq();
        size_t start_tsc = worker_rdtsc();
        for (size_t i = 0; i < num_codeblocks; i++) {
            ldpc_decoder_5gnr_request.varNodes = demod_data_all_symbols[i];
            ldpc_decoder_5gnr_response.compactedMessageBytes
                = decoded_codewords[i];
            bblib_ldpc_decoder_5gnr(
                &ldpc_decoder_5gnr_request, &ldpc_decoder_5gnr_response);
        }
        size_t duration = worker_rdtsc() - start_tsc;
        std::printf("Decoding of %zu blocks takes %.2f us per block\n",
            num_codeblocks, cycles_to_us(duration, freq_ghz) / num_codeblocks);

        // Correctness check
        size_t error_num = 0;
        size_t total = num_codeblocks * LDPC_config.cbLen;
        size_t block_error_num = 0;

        for (size_t i = 0; i < num_codeblocks; i++) {
            size_t error_in_block = 0;
            for (size_t j = 0; j < LDPC_config.cbLen / 8; j++) {
                uint8_t input = (uint8_t)information[i][j];
                uint8_t output = decoded_codewords[i][j];
                if (input != output) {
                    for (size_t i = 0; i < 8; i++) {
                        uint8_t mask = 1 << i;
                        if ((input & mask) != (output & mask)) {
                            error_num++;
                            error_in_block++;
                        }
                    }
                    // std::printf("block %zu j: %zu: (%u, %u)\n", i, j,
                    //     (uint8_t)information[i][j], decoded_codewords[i][j]);
                }
            }
            if (error_in_block > 0) {
                block_error_num++;
                // std::printf("errors in block %zu: %zu\n", i, error_in_block);
            }
        }

        std::printf(
            "Noise: %.3f, snr: %.1f dB, error rate: %zu/%zu = %.6f, block "
            "error: "
            "%zu/%zu = %.6f\n",
            noise_levels[noise_id], snr_levels[noise_id], error_num, total,
            1.f * error_num / total, block_error_num, num_codeblocks,
            1.f * block_error_num / num_codeblocks);
    }

    return 0;
}
