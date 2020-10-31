/**
 * @file test_ldpc_baseband.cpp
 * @brief Test LDPC performance in baseband procesing when different levels of 
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
#include <malloc.h>
#include <math.h>
#include <random>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

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

    printf("DataGenerator: Config file: %s, data profile = %s\n",
        FLAGS_conf_file.c_str(),
        profile == DataGenerator::Profile::k123 ? "123" : "random");

    printf("DataGenerator: Using %s-orthogonal pilots\n",
        cfg->freq_orthogonal_pilot ? "frequency" : "time");

    printf("DataGenerator: Generating encoded and modulated data\n");
    srand(time(nullptr));

    // Step 1: Generate the information buffers and LDPC-encoded buffers for
    // uplink
    size_t num_symbols_per_cb = 1;
    size_t bits_per_symbol = cfg->OFDM_DATA_NUM * cfg->mod_order_bits;
    if (cfg->LDPC_config.cbCodewLen > bits_per_symbol)
        num_symbols_per_cb = (cfg->LDPC_config.cbCodewLen + bits_per_symbol - 1)
            / bits_per_symbol;
    size_t num_cbs_per_ue = cfg->data_symbol_num_perframe / num_symbols_per_cb;
    printf("Number of symbols per block: %zu, blocks per frame: %zu\n",
        num_symbols_per_cb, num_cbs_per_ue);

    const size_t num_codeblocks = num_cbs_per_ue * cfg->UE_ANT_NUM;
    printf("Total number of blocks: %zu\n", num_codeblocks);
    for (size_t noise_id = 0; noise_id < 15; noise_id++) {

        std::vector<std::vector<int8_t>> information(num_codeblocks);
        std::vector<std::vector<int8_t>> encoded_codewords(num_codeblocks);
        for (size_t i = 0; i < num_codeblocks; i++) {
            data_generator.gen_codeblock_ul(information[i],
                encoded_codewords[i], i % cfg->UE_NUM /* UE ID */);
        }

        // Save uplink information bytes to file
        const size_t input_bytes_per_cb = bits_to_bytes(
            ldpc_num_input_bits(cfg->LDPC_config.Bg, cfg->LDPC_config.Zc));
        if (kPrintUplinkInformationBytes) {
            printf("Uplink information bytes\n");
            for (size_t n = 0; n < num_codeblocks; n++) {
                printf("Symbol %zu, UE %zu\n", n / cfg->UE_ANT_NUM,
                    n % cfg->UE_ANT_NUM);
                for (size_t i = 0; i < input_bytes_per_cb; i++) {
                    printf("%u ", (uint8_t)information[n][i]);
                }
                printf("\n");
            }
        }

        // Modulate the encoded codewords
        std::vector<std::vector<complex_float>> modulated_codewords(
            cfg->UE_ANT_NUM * cfg->data_symbol_num_perframe);
        size_t num_used_symbol = num_cbs_per_ue * num_symbols_per_cb;
        size_t num_unused_symbol
            = cfg->data_symbol_num_perframe - num_used_symbol;
        for (size_t ue_id = 0; ue_id < cfg->UE_ANT_NUM; ue_id++) {
            for (size_t i = 0; i < num_cbs_per_ue; i++) {
                size_t remaining_bits = cfg->LDPC_config.cbCodewLen;
                size_t offset = 0;
                for (size_t j = 0; j < num_symbols_per_cb; j++) {

                    size_t num_bits = ((j + 1) < num_symbols_per_cb)
                        ? bits_per_symbol
                        : remaining_bits;
                    modulated_codewords[ue_id * cfg->data_symbol_num_perframe
                        + i * num_symbols_per_cb + j]
                        = data_generator.get_modulation(
                            &encoded_codewords[ue_id * num_cbs_per_ue + i]
                                              [offset],
                            num_bits);
                    remaining_bits -= bits_per_symbol;
                    offset += bits_to_bytes(bits_per_symbol);
                }
            }
            for (size_t i = 0; i < num_unused_symbol; i++) {
                modulated_codewords[ue_id * cfg->data_symbol_num_perframe
                    + num_used_symbol + i]
                    .resize(cfg->OFDM_DATA_NUM);
            }
        }

        // Place modulated uplink data codewords into central IFFT bins
        std::vector<std::vector<complex_float>> pre_ifft_data_syms(
            cfg->UE_ANT_NUM * cfg->data_symbol_num_perframe);
        for (size_t i = 0; i < pre_ifft_data_syms.size(); i++) {
            pre_ifft_data_syms[i]
                = data_generator.bin_for_ifft(modulated_codewords[i]);
        }

        std::vector<complex_float> pilot_td
            = data_generator.get_common_pilot_time_domain();

        // Put pilot and data symbols together
        Table<complex_float> tx_data_all_symbols;
        tx_data_all_symbols.calloc(
            cfg->symbol_num_perframe, cfg->UE_ANT_NUM * cfg->OFDM_CA_NUM, 64);

        if (cfg->freq_orthogonal_pilot) {
            for (size_t i = 0; i < cfg->UE_ANT_NUM; i++) {
                std::vector<complex_float> pilots_t_ue(
                    cfg->OFDM_CA_NUM); // Zeroed
                for (size_t j = cfg->OFDM_DATA_START;
                     j < cfg->OFDM_DATA_START + cfg->OFDM_DATA_NUM;
                     j += cfg->UE_ANT_NUM) {
                    pilots_t_ue[i + j] = pilot_td[i + j];
                }
                // Load pilot to the second symbol
                // The first symbol is reserved for beacon
                memcpy(tx_data_all_symbols[cfg->beacon_symbol_num_perframe]
                        + i * cfg->OFDM_CA_NUM,
                    &pilots_t_ue[0], cfg->OFDM_CA_NUM * sizeof(complex_float));
            }
        } else {
            for (size_t i = 0; i < cfg->UE_ANT_NUM; i++)
                memcpy(tx_data_all_symbols[i + cfg->beacon_symbol_num_perframe]
                        + i * cfg->OFDM_CA_NUM,
                    &pilot_td[0], cfg->OFDM_CA_NUM * sizeof(complex_float));
        }

        size_t data_sym_start
            = cfg->pilot_symbol_num_perframe + cfg->beacon_symbol_num_perframe;
        for (size_t i = data_sym_start; i < cfg->symbol_num_perframe; i++) {
            const size_t data_sym_id = (i - data_sym_start);
            for (size_t j = 0; j < cfg->UE_ANT_NUM; j++) {
                memcpy(tx_data_all_symbols[i] + j * cfg->OFDM_CA_NUM,
                    &pre_ifft_data_syms[j * cfg->data_symbol_num_perframe
                        + data_sym_id][0],
                    cfg->OFDM_CA_NUM * sizeof(complex_float));
            }
        }

        // Generate CSI matrix
        Table<complex_float> csi_matrices_no_noise;
        csi_matrices_no_noise.calloc(
            cfg->OFDM_CA_NUM, cfg->UE_ANT_NUM * cfg->BS_ANT_NUM, 32);
        for (size_t i = 0; i < cfg->UE_ANT_NUM * cfg->BS_ANT_NUM; i++) {
            complex_float csi = { static_cast<float>(distribution(generator)),
                static_cast<float>(distribution(generator)) };
            // printf("noise of ant %d, ue %d\n", i % cfg->BS_ANT_NUM, i / cfg->BS_ANT_NUM );
            for (size_t j = 0; j < cfg->OFDM_CA_NUM; j++) {
                // printf("%.4f+%.4fi ", noise.re, noise.im);
                csi_matrices_no_noise[j][i].re = csi.re;
                csi_matrices_no_noise[j][i].im = csi.im;
            }
            // printf("\n");
        }

        Table<complex_float> csi_matrices_pilot;
        csi_matrices_pilot.calloc(
            cfg->OFDM_CA_NUM, cfg->UE_ANT_NUM * cfg->BS_ANT_NUM, 32);
        for (size_t i = 0; i < cfg->UE_ANT_NUM * cfg->BS_ANT_NUM; i++) {
            for (size_t j = 0; j < cfg->OFDM_CA_NUM; j++) {
                complex_float noise
                    = { static_cast<float>(distribution(generator))
                              * noise_levels[noise_id],
                          static_cast<float>(distribution(generator))
                              * noise_levels[noise_id] };
                // printf("%.4f+%.4fi ", noise.re, noise.im);
                csi_matrices_pilot[j][i].re
                    = csi_matrices_no_noise[j][i].re + noise.re;
                csi_matrices_pilot[j][i].im
                    = csi_matrices_no_noise[j][i].im + noise.im;
            }
            // printf("\n");
        }

        Table<complex_float> csi_matrices_data;
        csi_matrices_data.calloc(
            cfg->OFDM_CA_NUM, cfg->UE_ANT_NUM * cfg->BS_ANT_NUM, 32);
        for (size_t i = 0; i < cfg->UE_ANT_NUM * cfg->BS_ANT_NUM; i++) {
            for (size_t j = 0; j < cfg->OFDM_CA_NUM; j++) {
                complex_float noise
                    = { static_cast<float>(distribution(generator))
                              * noise_levels[noise_id],
                          static_cast<float>(distribution(generator))
                              * noise_levels[noise_id] };
                // printf("%.4f+%.4fi ", noise.re, noise.im);
                csi_matrices_data[j][i].re
                    = csi_matrices_no_noise[j][i].re + noise.re;
                csi_matrices_data[j][i].im
                    = csi_matrices_no_noise[j][i].im + noise.im;
            }
            // printf("\n");
        }

        // Generate RX data received by base station after going through channels
        Table<complex_float> rx_data_all_symbols;
        rx_data_all_symbols.calloc(
            cfg->symbol_num_perframe, cfg->OFDM_CA_NUM * cfg->BS_ANT_NUM, 64);
        for (size_t i = 0; i < cfg->symbol_num_perframe; i++) {
            arma::cx_fmat mat_input_data(
                reinterpret_cast<arma::cx_float*>(tx_data_all_symbols[i]),
                cfg->OFDM_CA_NUM, cfg->UE_ANT_NUM, false);
            arma::cx_fmat mat_output(
                reinterpret_cast<arma::cx_float*>(rx_data_all_symbols[i]),
                cfg->OFDM_CA_NUM, cfg->BS_ANT_NUM, false);

            for (size_t j = 0; j < cfg->OFDM_CA_NUM; j++) {
                arma::cx_fmat mat_csi(
                    reinterpret_cast<arma::cx_float*>(csi_matrices_data[j]),
                    cfg->BS_ANT_NUM, cfg->UE_ANT_NUM);
                mat_output.row(j) = mat_input_data.row(j) * mat_csi.st();
            }
        }

        // Compute precoder
        Table<complex_float> precoder;
        precoder.calloc(
            cfg->OFDM_CA_NUM, cfg->UE_ANT_NUM * cfg->BS_ANT_NUM, 32);
        for (size_t i = 0; i < cfg->OFDM_CA_NUM; i++) {
            arma::cx_fmat mat_input(
                reinterpret_cast<arma::cx_float*>(csi_matrices_pilot[i]),
                cfg->BS_ANT_NUM, cfg->UE_ANT_NUM, false);
            arma::cx_fmat mat_output(
                reinterpret_cast<arma::cx_float*>(precoder[i]), cfg->UE_ANT_NUM,
                cfg->BS_ANT_NUM, false);
            pinv(mat_output, mat_input, 1e-2, "dc");
        }

        Table<complex_float> equalized_data_all_symbols;
        equalized_data_all_symbols.calloc(
            cfg->symbol_num_perframe, cfg->OFDM_DATA_NUM * cfg->UE_ANT_NUM, 64);
        Table<int8_t> demod_data_all_symbols;
        demod_data_all_symbols.calloc(cfg->UE_ANT_NUM,
            cfg->OFDM_DATA_NUM * cfg->data_symbol_num_perframe * 8, 64);
        for (size_t i = data_sym_start; i < cfg->symbol_num_perframe; i++) {
            arma::cx_fmat mat_rx_data(
                reinterpret_cast<arma::cx_float*>(rx_data_all_symbols[i]),
                cfg->OFDM_CA_NUM, cfg->BS_ANT_NUM, false);
            arma::cx_fmat mat_equalized_data(
                reinterpret_cast<arma::cx_float*>(
                    equalized_data_all_symbols[i - data_sym_start]),
                cfg->OFDM_DATA_NUM, cfg->UE_ANT_NUM, false);
            for (size_t j = 0; j < cfg->OFDM_DATA_NUM; j++) {
                arma::cx_fmat mat_precoder(
                    reinterpret_cast<arma::cx_float*>(
                        precoder[cfg->freq_orthogonal_pilot
                                ? (j % cfg->UE_ANT_NUM)
                                : j]),
                    cfg->UE_ANT_NUM, cfg->BS_ANT_NUM, false);
                mat_equalized_data.row(j) = (mat_precoder
                    * mat_rx_data.row(j + cfg->OFDM_DATA_START).st())
                                                .st();
            }

            mat_equalized_data = mat_equalized_data.st();

            for (size_t j = 0; j < cfg->UE_ANT_NUM; j++) {
                size_t cb_id = (i - data_sym_start) / num_symbols_per_cb;
                size_t symbol_id_in_cb
                    = (i - data_sym_start) % num_symbols_per_cb;
                auto* demod_ptr = demod_data_all_symbols[j]
                    + (cb_id * num_symbols_per_cb * 8
                          + symbol_id_in_cb * cfg->mod_order_bits)
                        * cfg->OFDM_DATA_NUM;
                auto* equal_T_ptr
                    = (float*)(equalized_data_all_symbols[i - data_sym_start]
                        + j * cfg->OFDM_DATA_NUM);
                switch (cfg->mod_order_bits) {
                case (4):
                    demod_16qam_soft_avx2(
                        equal_T_ptr, demod_ptr, cfg->OFDM_DATA_NUM);
                    break;
                case (6):
                    demod_64qam_soft_avx2(
                        equal_T_ptr, demod_ptr, cfg->OFDM_DATA_NUM);
                    break;
                default:
                    printf("Demodulation: modulation type %s not supported!\n",
                        cfg->modulation.c_str());
                }
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
            = (int16_t*)memalign(64, 1024 * 1024 * sizeof(int16_t));
        ldpc_decoder_5gnr_response.varNodes = resp_var_nodes;

        Table<uint8_t> decoded_codewords;
        decoded_codewords.calloc(num_codeblocks, cfg->OFDM_DATA_NUM, 64);
        double freq_ghz = measure_rdtsc_freq();
        size_t start_tsc = worker_rdtsc();
        for (size_t i = 0; i < cfg->UE_ANT_NUM; i++) {
            for (size_t j = 0; j < num_cbs_per_ue; j++) {
                ldpc_decoder_5gnr_request.varNodes = demod_data_all_symbols[i]
                    + j * cfg->OFDM_DATA_NUM * 8 * num_symbols_per_cb;
                ldpc_decoder_5gnr_response.compactedMessageBytes
                    = decoded_codewords[i * num_cbs_per_ue + j];
                bblib_ldpc_decoder_5gnr(
                    &ldpc_decoder_5gnr_request, &ldpc_decoder_5gnr_response);
            }
        }
        size_t duration = worker_rdtsc() - start_tsc;
        printf("Decoding of %zu blocks takes %.2f us per block\n",
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
                    // printf("block %zu j: %zu: (%u, %u)\n", i, j,
                    //     (uint8_t)information[i][j], decoded_codewords[i][j]);
                }
            }
            if (error_in_block > 0) {
                block_error_num++;
                // printf("errors in block %zu: %zu\n", i, error_in_block);
            }
        }

        printf("Noise: %.3f, snr: %.1f dB, error rate: %zu/%zu = %.6f, block "
               "error: "
               "%zu/%zu = %.6f\n",
            noise_levels[noise_id], snr_levels[noise_id], error_num, total,
            1.f * error_num / total, block_error_num, num_codeblocks,
            1.f * block_error_num / num_codeblocks);
    }

    return 0;
}
