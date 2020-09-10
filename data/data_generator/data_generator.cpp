/**
 * @file data_generator.cpp
 * @brief Data generator to generate binary files as inputs to Millipede, sender
 * and correctness tests
 */

#include "data_generator.h"
#include "comms-lib.h"
#include "config.hpp"
#include "memory_manage.h"
#include "modulation.hpp"
#include "utils_ldpc.hpp"
#include <armadillo>
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

using namespace arma;

static constexpr float kNoiseLevel = 1.0 / 200;
static constexpr bool kVerbose = false;
static constexpr bool kPrintUplinkInformationBytes = false;

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
    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string confFile = cur_directory + "/data/tddconfig-sim-ul.json";
    if (argc == 2)
        confFile = std::string(argv[1]);
    auto* cfg = new Config(confFile.c_str());
    DataGenerator data_generator(cfg, 0, DataGenerator::Profile::kTest);

    printf("Config file: %s\n", confFile.c_str());
    printf("Using %s-orthogonal pilots\n",
        cfg->freq_orthogonal_pilot ? "frequency" : "time");
    printf("Generating encoded and modulated data\n");
    srand(time(NULL));

    // Step 1: Generate the information buffers and LDPC-encoded buffers for
    // uplink
    const size_t num_codeblocks = cfg->data_symbol_num_perframe
        * cfg->LDPC_config.nblocksInSymbol * cfg->UE_ANT_NUM;
    printf("Total number of blocks: %zu\n", num_codeblocks);

    std::vector<std::vector<int8_t>> information(num_codeblocks);
    std::vector<std::vector<int8_t>> encoded_codewords(num_codeblocks);
    for (size_t i = 0; i < num_codeblocks; i++) {
        data_generator.gen_codeblock_ul(information[i], encoded_codewords[i], i);
    }

    {
        // Save uplink information bytes to file
        const size_t input_bytes_per_cb = bits_to_bytes(
            ldpc_num_input_bits(cfg->LDPC_config.Bg, cfg->LDPC_config.Zc));

        const std::string filename_input = cur_directory
            + "/data/LDPC_orig_data_" + std::to_string(cfg->OFDM_CA_NUM)
            + "_ant" + std::to_string(cfg->UE_ANT_NUM) + ".bin";
        printf("Saving raw data (using LDPC) to %s\n", filename_input.c_str());
        FILE* fp_input = fopen(filename_input.c_str(), "wb");
        for (size_t i = 0; i < num_codeblocks; i++) {
            fwrite(reinterpret_cast<uint8_t*>(&information[i][0]),
                input_bytes_per_cb, sizeof(uint8_t), fp_input);
        }
        fclose(fp_input);

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
    }

    // Modulate the encoded codewords
    std::vector<std::vector<complex_float>> modulated_codewords(num_codeblocks);
    for (size_t i = 0; i < num_codeblocks; i++) {
        modulated_codewords[i]
            = data_generator.get_modulation(encoded_codewords[i]);
    }

    // Place modulated uplink data codewords into central IFFT bins
    rt_assert(cfg->LDPC_config.nblocksInSymbol == 1); // TODO: Assumption
    std::vector<std::vector<complex_float>> pre_ifft_data_syms(
        cfg->UE_ANT_NUM * cfg->data_symbol_num_perframe);
    for (size_t i = 0; i < pre_ifft_data_syms.size(); i++) {
        pre_ifft_data_syms[i]
            = data_generator.bin_for_ifft(modulated_codewords[i]);
    }

    std::vector<complex_float> pilot_td
        = data_generator.get_common_pilot_time_domain();

    // Generate UE-specific pilots
    Table<complex_float> ue_specific_pilot;
    const std::vector<std::complex<float>> zc_seq = Utils::double_to_cfloat(
        CommsLib::getSequence(cfg->OFDM_DATA_NUM, CommsLib::LTE_ZADOFF_CHU));
    const std::vector<std::complex<float>> zc_common_pilot
        = CommsLib::seqCyclicShift(zc_seq, M_PI / 4.0); // Used in LTE SRS
    ue_specific_pilot.malloc(cfg->UE_ANT_NUM, cfg->OFDM_DATA_NUM, 64);
    for (size_t i = 0; i < cfg->UE_ANT_NUM; i++) {
        auto zc_ue_pilot_i
            = CommsLib::seqCyclicShift(zc_seq, i * M_PI / 6.0); // LTE DMRS
        for (size_t j = 0; j < cfg->OFDM_DATA_NUM; j++) {
            ue_specific_pilot[i][j]
                = { zc_ue_pilot_i[j].real(), zc_ue_pilot_i[j].imag() };
        }
    }

    // Put pilot and data symbols together
    Table<complex_float> tx_data_all_symbols;
    tx_data_all_symbols.calloc(
        cfg->symbol_num_perframe, cfg->UE_ANT_NUM * cfg->OFDM_CA_NUM, 64);

    if (cfg->freq_orthogonal_pilot) {
        for (size_t i = 0; i < cfg->UE_ANT_NUM; i++) {
            std::vector<complex_float> pilots_t_ue(cfg->OFDM_CA_NUM); // Zeroed
            for (size_t j = cfg->OFDM_DATA_START;
                 j < cfg->OFDM_DATA_START + cfg->OFDM_DATA_NUM;
                 j += cfg->UE_ANT_NUM) {
                pilots_t_ue[i + j] = pilot_td[i + j];
            }
            memcpy(tx_data_all_symbols[0] + i * cfg->OFDM_CA_NUM,
                &pilots_t_ue[0], cfg->OFDM_CA_NUM * sizeof(complex_float));
        }
    } else {
        for (size_t i = 0; i < cfg->UE_ANT_NUM; i++)
            memcpy(tx_data_all_symbols[i] + i * cfg->OFDM_CA_NUM, &pilot_td[0],
                cfg->OFDM_CA_NUM * sizeof(complex_float));
    }

    for (size_t i = cfg->pilot_symbol_num_perframe;
         i < cfg->symbol_num_perframe; i++) {
        const size_t data_sym_id = (i - cfg->pilot_symbol_num_perframe);
        for (size_t j = 0; j < cfg->UE_ANT_NUM; j++) {
            if (data_sym_id < cfg->UL_PILOT_SYMS) {
                memcpy(tx_data_all_symbols[i] + j * cfg->OFDM_CA_NUM
                        + cfg->OFDM_DATA_START,
                    ue_specific_pilot[j],
                    cfg->OFDM_DATA_NUM * sizeof(complex_float));
            } else {
                memcpy(tx_data_all_symbols[i] + j * cfg->OFDM_CA_NUM,
                    &pre_ifft_data_syms[data_sym_id * cfg->UE_ANT_NUM + j][0],
                    cfg->OFDM_CA_NUM * sizeof(complex_float));
            }
        }
    }

    // Generate CSI matrix
    Table<complex_float> CSI_matrix;
    CSI_matrix.calloc(cfg->OFDM_CA_NUM, cfg->UE_ANT_NUM * cfg->BS_ANT_NUM, 32);
    for (size_t i = 0; i < cfg->UE_ANT_NUM * cfg->BS_ANT_NUM; i++) {
        complex_float csi
            = { rand_float_from_short(-1, 1), rand_float_from_short(-1, 1) };
        // printf("noise of ant %d, ue %d\n", i % cfg->BS_ANT_NUM, i / cfg->BS_ANT_NUM );
        for (size_t j = 0; j < cfg->OFDM_CA_NUM; j++) {
            complex_float noise = { rand_float_from_short(-1, 1) * kNoiseLevel,
                rand_float_from_short(-1, 1) * kNoiseLevel };
            // printf("%.4f+%.4fi ", noise.re, noise.im);
            CSI_matrix[j][i].re = csi.re + noise.re;
            CSI_matrix[j][i].im = csi.im + noise.im;
        }
        // printf("\n");
    }

    // Generate RX data received by base station after going through channels
    Table<complex_float> rx_data_all_symbols;
    rx_data_all_symbols.calloc(
        cfg->symbol_num_perframe, cfg->OFDM_CA_NUM * cfg->BS_ANT_NUM, 64);
    for (size_t i = 0; i < cfg->symbol_num_perframe; i++) {
        auto* ptr_in_data = (cx_float*)tx_data_all_symbols[i];
        cx_fmat mat_input_data(
            ptr_in_data, cfg->OFDM_CA_NUM, cfg->UE_ANT_NUM, false);
        auto* ptr_out = (cx_float*)rx_data_all_symbols[i];
        cx_fmat mat_output(ptr_out, cfg->OFDM_CA_NUM, cfg->BS_ANT_NUM, false);
        for (size_t j = 0; j < cfg->OFDM_CA_NUM; j++) {
            auto* ptr_in_csi = (cx_float*)CSI_matrix[j];
            cx_fmat mat_csi(ptr_in_csi, cfg->BS_ANT_NUM, cfg->UE_ANT_NUM);
            mat_output.row(j) = mat_input_data.row(j) * mat_csi.st();
        }
        for (size_t j = 0; j < cfg->BS_ANT_NUM; j++) {
            CommsLib::IFFT(rx_data_all_symbols[i] + j * cfg->OFDM_CA_NUM,
                cfg->OFDM_CA_NUM, false);
        }
    }

    std::string filename_rx = cur_directory + "/data/LDPC_rx_data_"
        + std::to_string(cfg->OFDM_CA_NUM) + "_ant"
        + std::to_string(cfg->BS_ANT_NUM) + ".bin";
    printf("Saving rx data to %s\n", filename_rx.c_str());
    FILE* fp_rx = fopen(filename_rx.c_str(), "wb");
    for (size_t i = 0; i < cfg->symbol_num_perframe; i++) {
        auto* ptr = (float*)rx_data_all_symbols[i];
        fwrite(
            ptr, cfg->OFDM_CA_NUM * cfg->BS_ANT_NUM * 2, sizeof(float), fp_rx);
    }
    fclose(fp_rx);

    // printf("rx data\n");
    // for (int i = 0; i < 10; i++) {
    //     for (int j = 0; j < cfg->OFDM_CA_NUM * cfg->BS_ANT_NUM; j++) {
    //         if (j % cfg->OFDM_CA_NUM == 0) {
    //             printf("\nsymbol %d ant %d\n", i, j / cfg->OFDM_CA_NUM);
    //         }
    //         printf("%.4f+%.4fi ", rx_data_all_symbols[i][j].re,
    //             rx_data_all_symbols[i][j].im);
    //     }
    //     printf("\n");
    // }

    /* ------------------------------------------------
     * Generate data for downlink test
     * ------------------------------------------------ */

    // Compute precoder
    Table<complex_float> precoder;
    precoder.calloc(cfg->OFDM_CA_NUM, cfg->UE_ANT_NUM * cfg->BS_ANT_NUM, 32);
    for (size_t i = 0; i < cfg->OFDM_CA_NUM; i++) {
        auto* ptr_in = (cx_float*)CSI_matrix[i];
        cx_fmat mat_input(ptr_in, cfg->BS_ANT_NUM, cfg->UE_ANT_NUM, false);
        auto* ptr_out = (cx_float*)precoder[i];
        cx_fmat mat_output(ptr_out, cfg->UE_ANT_NUM, cfg->BS_ANT_NUM, false);
        pinv(mat_output, mat_input, 1e-2, "dc");
    }

    // printf("CSI \n");
    // // for (int i = 0; i < cfg->OFDM_CA_NUM; i++)
    // for (int j = 0; j < cfg->UE_ANT_NUM * cfg->BS_ANT_NUM; j++)
    //     printf("%.3f+%.3fi ",
    //         CSI_matrix[cfg->OFDM_DATA_START][j].re,
    //         CSI_matrix[cfg->OFDM_DATA_START][j].im);
    // printf("\n");
    // printf("precoder \n");
    // // for (int i = 0; i < cfg->OFDM_CA_NUM; i++)
    // for (int j = 0; j < cfg->UE_ANT_NUM * cfg->BS_ANT_NUM; j++)
    //     printf("%.3f+%.3fi ",
    //         precoder[cfg->OFDM_DATA_START][j].re,
    //         precoder[cfg->OFDM_DATA_START][j].im);
    // printf("\n");

    // Prepare downlink data from mod_output
    Table<complex_float> dl_mod_data;
    dl_mod_data.calloc(cfg->dl_data_symbol_num_perframe,
        cfg->OFDM_CA_NUM * cfg->UE_ANT_NUM, 64);
    for (size_t i = 0; i < cfg->dl_data_symbol_num_perframe; i++) {
        for (size_t j = 0; j < cfg->UE_ANT_NUM; j++) {
            if (cfg->DL_PILOT_SYMS > 0 and i <= cfg->DL_PILOT_SYMS - 1) {
                for (size_t sc_id = 0; sc_id < cfg->OFDM_DATA_NUM; sc_id++)
                    dl_mod_data[i][j * cfg->OFDM_CA_NUM + sc_id
                        + cfg->OFDM_DATA_START]
                        = ue_specific_pilot[j][sc_id];
            } else {
                for (size_t sc_id = 0; sc_id < cfg->OFDM_DATA_NUM; sc_id++)
                    dl_mod_data[i][j * cfg->OFDM_CA_NUM + sc_id
                        + cfg->OFDM_DATA_START]
                        = (sc_id % cfg->OFDM_PILOT_SPACING == 0)
                        ? ue_specific_pilot[j][sc_id]
                        : modulated_codewords[i * cfg->UE_ANT_NUM + j][sc_id];
            }
        }
    }

    // printf("dl mod data \n");
    // for (int i = 0; i < dl_data_symbol_num_perframe; i++) {
    //     for (int k = cfg->OFDM_DATA_START; k < cfg->OFDM_DATA_START + cfg->OFDM_DATA_NUM;
    //          k++) {
    //         printf("symbol %d, subcarrier %d\n", i, k);
    //         for (int j = 0; j < cfg->UE_ANT_NUM; j++) {

    //             // for (int k = cfg->OFDM_DATA_START; k < cfg->OFDM_DATA_START + cfg->OFDM_DATA_NUM;
    //             //      k++) {
    //             printf("%.3f+%.3fi ", dl_mod_data[i][j * cfg->OFDM_CA_NUM + k].re,
    //                 dl_mod_data[i][j * cfg->OFDM_CA_NUM + k].im);
    //         }
    //         printf("\n");
    //     }
    // }

    // Perform precoding and IFFT
    Table<complex_float> dl_ifft_data;
    dl_ifft_data.calloc(cfg->dl_data_symbol_num_perframe,
        cfg->OFDM_CA_NUM * cfg->BS_ANT_NUM, 64);
    Table<short> dl_tx_data;
    dl_tx_data.calloc(cfg->dl_data_symbol_num_perframe,
        2 * cfg->sampsPerSymbol * cfg->BS_ANT_NUM, 64);
    for (size_t i = 0; i < cfg->dl_data_symbol_num_perframe; i++) {
        auto* ptr_in_data = (cx_float*)dl_mod_data[i];
        cx_fmat mat_input_data(
            ptr_in_data, cfg->OFDM_CA_NUM, cfg->UE_ANT_NUM, false);
        auto* ptr_out = (cx_float*)dl_ifft_data[i];
        cx_fmat mat_output(ptr_out, cfg->OFDM_CA_NUM, cfg->BS_ANT_NUM, false);
        for (size_t j = cfg->OFDM_DATA_START;
             j < cfg->OFDM_DATA_NUM + cfg->OFDM_DATA_START; j++) {
            auto* ptr_in_precoder = (cx_float*)precoder[j];
            cx_fmat mat_precoder(
                ptr_in_precoder, cfg->UE_ANT_NUM, cfg->BS_ANT_NUM, false);
            mat_output.row(j) = mat_input_data.row(j) * mat_precoder;

            // printf("symbol %d, sc: %d\n", i, j - cfg->OFDM_DATA_START);
            // cout << "Precoder: \n" << mat_precoder << endl;
            // cout << "Data: \n" << mat_input_data.row(j) << endl;
            // cout << "Precoded data: \n" << mat_output.row(j) << endl;
        }
        for (size_t j = 0; j < cfg->BS_ANT_NUM; j++) {
            complex_float* ptr_ifft = dl_ifft_data[i] + j * cfg->OFDM_CA_NUM;
            CommsLib::IFFT(ptr_ifft, cfg->OFDM_CA_NUM, false);

            cx_fmat mat_data((cx_float*)ptr_ifft, 1, cfg->OFDM_CA_NUM, false);
            float scale = abs(mat_data).max();
            mat_data /= scale;

            short* txSymbol = dl_tx_data[i] + j * cfg->sampsPerSymbol * 2;
            memset(txSymbol, 0, sizeof(short) * 2 * cfg->ofdm_tx_zero_prefix_);
            for (size_t k = 0; k < cfg->OFDM_CA_NUM; k++) {
                txSymbol[2 * (k + cfg->CP_LEN + cfg->ofdm_tx_zero_prefix_)]
                    = (short)(32768 * ptr_ifft[k].re);
                txSymbol[2 * (k + cfg->CP_LEN + cfg->ofdm_tx_zero_prefix_) + 1]
                    = (short)(32768 * ptr_ifft[k].im);
            }
            for (size_t k = 0; k < 2 * cfg->CP_LEN; k++) {
                txSymbol[2 * cfg->ofdm_tx_zero_prefix_ + k] = txSymbol[2
                    * (cfg->ofdm_tx_zero_prefix_ + cfg->OFDM_CA_NUM)];
            }

            const size_t tx_zero_postfix_offset = 2
                * (cfg->ofdm_tx_zero_prefix_ + cfg->CP_LEN + cfg->OFDM_CA_NUM);
            memset(txSymbol + tx_zero_postfix_offset, 0,
                sizeof(short) * 2 * cfg->ofdm_tx_zero_postfix_);
        }
    }

    std::string filename_dl_tx = cur_directory + "/data/LDPC_dl_tx_data_"
        + std::to_string(cfg->OFDM_CA_NUM) + "_ant"
        + std::to_string(cfg->BS_ANT_NUM) + ".bin";
    printf("Saving dl tx data to %s\n", filename_dl_tx.c_str());
    FILE* fp_dl_tx = fopen(filename_dl_tx.c_str(), "wb");
    for (size_t i = 0; i < cfg->dl_data_symbol_num_perframe; i++) {
        short* ptr = (short*)dl_tx_data[i];
        fwrite(ptr, cfg->sampsPerSymbol * cfg->BS_ANT_NUM * 2, sizeof(short),
            fp_dl_tx);
    }
    fclose(fp_dl_tx);

    // printf("rx data\n");
    // for (int i = 0; i < 10; i++) {

    //     for (int j = 0; j < cfg->OFDM_CA_NUM * cfg->BS_ANT_NUM; j++) {
    //         if (j % cfg->OFDM_CA_NUM == 0) {
    //             printf("symbol %d ant %d\n", i, j / cfg->OFDM_CA_NUM);
    //         }
    //         printf("%.3f+%.3fi ", dl_tx_data[i][j].re,
    //             dl_tx_data[i][j].im);
    //     }
    // }
    // printf("\n");

    CSI_matrix.free();
    tx_data_all_symbols.free();
    rx_data_all_symbols.free();
    ue_specific_pilot.free();

    return 0;
}
