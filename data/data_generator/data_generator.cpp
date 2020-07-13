/*
    Data generator to generate binary files as inputs to Millipede, sender and correctness tests
 */
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

static const float NOISE_LEVEL = 1.0 / 200;
static constexpr bool kVerbose = false;

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

    printf("Config file: %s\n", confFile.c_str());
    auto* config_ = new Config(confFile.c_str());

    printf("LDPC %s\n", kUseLDPC ? "enabled" : "disabled");
    printf("Using %s-orthogonal pilots\n",
        config_->freq_orthogonal_pilot ? "frequency" : "time");

    printf("Generating encoded and modulated data\n");
    int mod_type = config_->mod_type;
    int UE_ANT_NUM = config_->UE_ANT_NUM;
    int BS_ANT_NUM = config_->BS_ANT_NUM;
    int OFDM_CA_NUM = config_->OFDM_CA_NUM;
    int OFDM_DATA_NUM = config_->OFDM_DATA_NUM;
    int OFDM_DATA_START = config_->OFDM_DATA_START;
    int OFDM_PILOT_SPACING = config_->OFDM_PILOT_SPACING;
    int symbol_num_perframe = config_->symbol_num_perframe;
    int data_symbol_num_perframe = config_->data_symbol_num_perframe;
    int pilot_symbol_num_perframe = config_->pilot_symbol_num_perframe;
    int dl_data_symbol_num_perframe = config_->dl_data_symbol_num_perframe;
    int sampsPerSymbol = config_->sampsPerSymbol;
    int CP_LEN = config_->CP_LEN;
    int prefix = config_->prefix;
    int postfix = config_->postfix;
    int DL_PILOT_SYMS = config_->DL_PILOT_SYMS;
    int UL_PILOT_SYMS = config_->UL_PILOT_SYMS;
    auto LDPC_config = config_->LDPC_config;
    // randomly generate input
    srand(time(NULL));
    // srand(0);

    // Randomly generate input
    size_t num_mod = kUseLDPC
        ? ldpc_num_encoded_bits(LDPC_config.Bg, LDPC_config.Zc) / mod_type
        : OFDM_DATA_NUM;
    size_t numberCodeblocks = kUseLDPC
        ? data_symbol_num_perframe * LDPC_config.nblocksInSymbol * UE_ANT_NUM
        : data_symbol_num_perframe * UE_ANT_NUM;
    printf("Total number of blocks: %zu\n", numberCodeblocks);

    // Initialize buffers
    int8_t* input[numberCodeblocks];
    int8_t* parity[numberCodeblocks];
    int8_t* encoded[numberCodeblocks];

    if (kUseLDPC) {
        const size_t base_graph = LDPC_config.Bg;
        const size_t zc = LDPC_config.Zc;

        for (size_t i = 0; i < numberCodeblocks; i++) {
            input[i] = new int8_t[ldpc_encoding_input_buf_size(base_graph, zc)];
            parity[i]
                = new int8_t[ldpc_encoding_parity_buf_size(base_graph, zc)];
            encoded[i]
                = new int8_t[ldpc_encoding_encoded_buf_size(base_graph, zc)];
        }

        const size_t num_input_bytes
            = bits_to_bytes(ldpc_num_input_bits(base_graph, zc));
        for (size_t n = 0; n < numberCodeblocks; n++) {
            for (size_t i = 0; i < num_input_bytes; i++) {
                input[n][i] = (int8_t)rand();
            }
        }

        if (kVerbose) {
            printf("Raw input\n");
            for (size_t n = 0; n < numberCodeblocks; n++) {
                if (n % UE_ANT_NUM == 0) {
                    printf("Symbol %zu\n", n / UE_ANT_NUM);
                }
                printf("UE %zu\n", n % UE_ANT_NUM);

                for (size_t i = 0; i < num_input_bytes; i++) {
                    // std::cout << std::bitset<8>(input[n][i]) << " ";
                    printf("%u ", (uint8_t)input[n][i]);
                    printf("\n");
                }
                printf("\n");
            }
        }

        for (size_t n = 0; n < numberCodeblocks; n++) {
            ldpc_encode_helper(base_graph, zc, encoded[n], parity[n], input[n]);
        }
    }

    Table<uint8_t> mod_input;
    Table<complex_float> mod_output;
    mod_input.calloc(numberCodeblocks, OFDM_DATA_NUM, 32);
    mod_output.calloc(numberCodeblocks, OFDM_DATA_NUM, 32);
    Table<float> mod_table;
    init_modulation_table(mod_table, mod_type);

    for (size_t n = 0; n < numberCodeblocks; n++) {
        if (kUseLDPC) {
            adapt_bits_for_mod(encoded[n], mod_input[n],
                bits_to_bytes(
                    ldpc_num_encoded_bits(LDPC_config.Bg, LDPC_config.Zc)),
                mod_type);
        } else {
            for (size_t i = 0; i < num_mod; i++)
                mod_input[n][i] = (uint8_t)(rand() % config_->mod_order);
            // printf("symbol %d ue %d\n", n / UE_ANT_NUM, n % UE_ANT_NUM );
            // for (int i = 0; i < num_mod; i++)
            //     printf("%u ", mod_input[n][i]);
            // printf("\n");
        }
        for (int i = 0; i < OFDM_DATA_NUM; i++)
            mod_output[n][i]
                = mod_single_uint8((uint8_t)mod_input[n][i], mod_table);
    }


    if (kUseLDPC) {
        std::string filename_input = cur_directory + "/data/LDPC_orig_data_"
            + std::to_string(OFDM_CA_NUM) + "_ant" + std::to_string(UE_ANT_NUM)
            + ".bin";
        printf("Saving raw data (using LDPC) to %s\n", filename_input.c_str());
        FILE* fp_input = fopen(filename_input.c_str(), "wb");
        for (size_t i = 0; i < numberCodeblocks; i++) {
            uint8_t* ptr = (uint8_t*)input[i];
            const size_t num_input_bytes = bits_to_bytes(
                ldpc_num_input_bits(LDPC_config.Bg, LDPC_config.Zc));
            fwrite(ptr, num_input_bytes, sizeof(uint8_t), fp_input);
        }
        fclose(fp_input);
    } else {
        std::string filename_input = cur_directory + "/data/orig_data_"
            + std::to_string(OFDM_CA_NUM) + "_ant" + std::to_string(UE_ANT_NUM)
            + ".bin";
        printf("Saving raw data to %s\n", filename_input.c_str());
        FILE* fp_input = fopen(filename_input.c_str(), "wb");
        for (size_t i = 0; i < numberCodeblocks; i++) {
            uint8_t* ptr = (uint8_t*)mod_input[i];
            fwrite(ptr, OFDM_DATA_NUM, sizeof(uint8_t), fp_input);
        }
        fclose(fp_input);
    }

    /* Convert data into time domain */
    Table<complex_float> IFFT_data;
    IFFT_data.calloc(UE_ANT_NUM * data_symbol_num_perframe, OFDM_CA_NUM, 64);
    for (int i = 0; i < UE_ANT_NUM * data_symbol_num_perframe; i++) {
        memcpy(IFFT_data[i] + OFDM_DATA_START, mod_output[i],
            OFDM_DATA_NUM * sizeof(complex_float));
        // CommsLib::IFFT(IFFT_data[i], OFDM_CA_NUM);
    }

    /* generate pilot data and convert to time domain */
    auto zc_common_pilot_double
        = CommsLib::getSequence(OFDM_DATA_NUM, CommsLib::LTE_ZADOFF_CHU);
    auto zc_common_pilot_seq = Utils::double_to_cfloat(zc_common_pilot_double);
    auto zc_common_pilot = CommsLib::seqCyclicShift(
        zc_common_pilot_seq, M_PI / 4); // Used in LTE SRS

    complex_float* pilots_f = (complex_float*)aligned_alloc(
        64, OFDM_DATA_NUM * sizeof(complex_float));
    for (int i = 0; i < OFDM_DATA_NUM; i++) {
        pilots_f[i] = { zc_common_pilot[i].real(), zc_common_pilot[i].imag() };
    }

    complex_float* pilots_t;
    alloc_buffer_1d(&pilots_t, OFDM_CA_NUM, 64, 1);
    for (int i = 0; i < OFDM_DATA_NUM; i++)
        pilots_t[i + OFDM_DATA_START] = pilots_f[i];
    // CommsLib::IFFT(pilots_t, OFDM_CA_NUM);

    /* generate ue-specific pilot data */
    Table<complex_float> ue_specific_pilot;
    ue_specific_pilot.malloc(UE_ANT_NUM, OFDM_DATA_NUM, 64);
    auto zc_ue_pilot_double
        = CommsLib::getSequence(OFDM_DATA_NUM, CommsLib::LTE_ZADOFF_CHU);
    auto zc_ue_pilot = Utils::double_to_cfloat(zc_ue_pilot_double);
    for (int i = 0; i < UE_ANT_NUM; i++) {
        auto zc_ue_pilot_i = CommsLib::seqCyclicShift(
            zc_ue_pilot, i * (float)M_PI / 6); // LTE DMRS
        for (int j = 0; j < OFDM_DATA_NUM; j++) {
            ue_specific_pilot[i][j]
                = { zc_ue_pilot_i[j].real(), zc_ue_pilot_i[j].imag() };
        }
    }

    /* put pilot and data symbols together */
    Table<complex_float> tx_data_all_symbols;
    tx_data_all_symbols.calloc(symbol_num_perframe, UE_ANT_NUM * OFDM_CA_NUM, 64);

    if (config_->freq_orthogonal_pilot) {
        complex_float* pilots_t_ue;
        alloc_buffer_1d(&pilots_t_ue, OFDM_CA_NUM, 64, 1);
        for (int i = 0; i < UE_ANT_NUM; i++) {
            /* TODO: fix user pilots distribution in pilot symbols */
            /* Right now we assume one pilot symbol hold all user pilots
             * in freqency orthogonal pilot */
            memset(pilots_t_ue, 0, OFDM_CA_NUM * sizeof(complex_float));
            for (int j = OFDM_DATA_START; j < OFDM_DATA_START + OFDM_DATA_NUM;
                 j += UE_ANT_NUM) {
                pilots_t_ue[i + j] = pilots_t[i + j];
            }
            // CommsLib::IFFT(pilots_t_ue, OFDM_CA_NUM);
            memcpy(tx_data_all_symbols[0] + i * OFDM_CA_NUM, pilots_t_ue,
                OFDM_CA_NUM * sizeof(complex_float));
        }
    } else {
        for (int i = 0; i < UE_ANT_NUM; i++)
            memcpy(tx_data_all_symbols[i] + i * OFDM_CA_NUM, pilots_t,
                OFDM_CA_NUM * sizeof(complex_float));
    }

    for (int i = pilot_symbol_num_perframe; i < symbol_num_perframe; i++) {
        int data_symbol_id = (i - pilot_symbol_num_perframe);
        for (int j = 0; j < UE_ANT_NUM; j++) {
            if (data_symbol_id < UL_PILOT_SYMS)
                memcpy(
                    tx_data_all_symbols[i] + j * OFDM_CA_NUM + OFDM_DATA_START,
                    ue_specific_pilot[j],
                    OFDM_DATA_NUM * sizeof(complex_float));
            else
                memcpy(tx_data_all_symbols[i] + j * OFDM_CA_NUM,
                    IFFT_data[data_symbol_id * UE_ANT_NUM + j],
                    OFDM_CA_NUM * sizeof(complex_float));
        }
    }

    /* generate CSI matrix */
    Table<complex_float> CSI_matrix;
    CSI_matrix.calloc(OFDM_CA_NUM, UE_ANT_NUM * BS_ANT_NUM, 32);
    for (int i = 0; i < UE_ANT_NUM * BS_ANT_NUM; i++) {
        complex_float csi
            = { rand_float_from_short(-1, 1), rand_float_from_short(-1, 1) };
        // printf("noise of ant %d, ue %d\n", i % BS_ANT_NUM, i / BS_ANT_NUM );
        for (int j = 0; j < OFDM_CA_NUM; j++) {
            complex_float noise = { rand_float_from_short(-1, 1) * NOISE_LEVEL,
                rand_float_from_short(-1, 1) * NOISE_LEVEL };
            // printf("%.4f+%.4fi ", noise.re, noise.im);
            CSI_matrix[j][i].re = csi.re + noise.re;
            CSI_matrix[j][i].im = csi.im + noise.im;
        }
        // printf("\n");
    }

    /* generate rx data received by BS after going through channels */
    Table<complex_float> rx_data_all_symbols;
    rx_data_all_symbols.calloc(
        symbol_num_perframe, OFDM_CA_NUM * BS_ANT_NUM, 64);
    for (int i = 0; i < symbol_num_perframe; i++) {
        cx_float* ptr_in_data = (cx_float*)tx_data_all_symbols[i];
        cx_fmat mat_input_data(ptr_in_data, OFDM_CA_NUM, UE_ANT_NUM, false);
        cx_float* ptr_out = (cx_float*)rx_data_all_symbols[i];
        cx_fmat mat_output(ptr_out, OFDM_CA_NUM, BS_ANT_NUM, false);
        for (int j = 0; j < OFDM_CA_NUM; j++) {
            cx_float* ptr_in_csi = (cx_float*)CSI_matrix[j];
            cx_fmat mat_csi(ptr_in_csi, BS_ANT_NUM, UE_ANT_NUM);
            mat_output.row(j) = mat_input_data.row(j) * mat_csi.st();
        }
        for (int j = 0; j < BS_ANT_NUM; j++) {
            CommsLib::IFFT(
                rx_data_all_symbols[i] + j * OFDM_CA_NUM, OFDM_CA_NUM, false);
        }
    }

    std::string filename_rx = cur_directory
        + (kUseLDPC ? "/data/LDPC_rx_data_" : "/data/rx_data_")
        + std::to_string(OFDM_CA_NUM) + "_ant" + std::to_string(BS_ANT_NUM)
        + ".bin";
    printf("Saving rx data to %s\n", filename_rx.c_str());
    FILE* fp_rx = fopen(filename_rx.c_str(), "wb");
    for (int i = 0; i < symbol_num_perframe; i++) {
        float* ptr = (float*)rx_data_all_symbols[i];
        fwrite(ptr, OFDM_CA_NUM * BS_ANT_NUM * 2, sizeof(float), fp_rx);
    }
    fclose(fp_rx);

    // printf("rx data\n");
    // for (int i = 0; i < 10; i++) {
    //     for (int j = 0; j < OFDM_CA_NUM * BS_ANT_NUM; j++) {
    //         if (j % OFDM_CA_NUM == 0) {
    //             printf("\nsymbol %d ant %d\n", i, j / OFDM_CA_NUM);
    //         }
    //         printf("%.4f+%.4fi ", rx_data_all_symbols[i][j].re,
    //             rx_data_all_symbols[i][j].im);
    //     }
    //     printf("\n");
    // }

    /* ------------------------------------------------
     * generate data for downlink test
     * ------------------------------------------------ */

    /* compute precoder */
    Table<complex_float> precoder;
    precoder.calloc(OFDM_CA_NUM, UE_ANT_NUM * BS_ANT_NUM, 32);
    for (int i = 0; i < OFDM_CA_NUM; i++) {
        cx_float* ptr_in = (cx_float*)CSI_matrix[i];
        cx_fmat mat_input(ptr_in, BS_ANT_NUM, UE_ANT_NUM, false);
        cx_float* ptr_out = (cx_float*)precoder[i];
        cx_fmat mat_output(ptr_out, UE_ANT_NUM, BS_ANT_NUM, false);
        pinv(mat_output, mat_input, 1e-2, "dc");
    }

    // printf("CSI \n");
    // // for (int i = 0; i < OFDM_CA_NUM; i++)
    // for (int j = 0; j < UE_ANT_NUM * BS_ANT_NUM; j++)
    //     printf("%.3f+%.3fi ",
    //         CSI_matrix[OFDM_DATA_START][j].re,
    //         CSI_matrix[OFDM_DATA_START][j].im);
    // printf("\n");
    // printf("precoder \n");
    // // for (int i = 0; i < OFDM_CA_NUM; i++)
    // for (int j = 0; j < UE_ANT_NUM * BS_ANT_NUM; j++)
    //     printf("%.3f+%.3fi ",
    //         precoder[OFDM_DATA_START][j].re,
    //         precoder[OFDM_DATA_START][j].im);
    // printf("\n");

    /* prepare downlink data from mod_output */
    Table<complex_float> dl_mod_data;
    dl_mod_data.calloc(dl_data_symbol_num_perframe, OFDM_CA_NUM * UE_ANT_NUM, 64);
    for (int i = 0; i < dl_data_symbol_num_perframe; i++) {
        for (int j = 0; j < UE_ANT_NUM; j++) {
            if (i <= DL_PILOT_SYMS - 1) {
                for (int sc_id = 0; sc_id < OFDM_DATA_NUM; sc_id++)
                    dl_mod_data[i][j * OFDM_CA_NUM + sc_id + OFDM_DATA_START]
                        = ue_specific_pilot[j][sc_id];
            } else {
                for (int sc_id = 0; sc_id < OFDM_DATA_NUM; sc_id++)
                    dl_mod_data[i][j * OFDM_CA_NUM + sc_id + OFDM_DATA_START]
                        = (sc_id % OFDM_PILOT_SPACING == 0)
                        ? ue_specific_pilot[j][sc_id]
                        : mod_output[i * UE_ANT_NUM + j][sc_id];
            }
        }
    }

    // printf("dl mod data \n");
    // for (int i = 0; i < dl_data_symbol_num_perframe; i++) {
    //     for (int k = OFDM_DATA_START; k < OFDM_DATA_START + OFDM_DATA_NUM;
    //          k++) {
    //         printf("symbol %d, subcarrier %d\n", i, k);
    //         for (int j = 0; j < UE_ANT_NUM; j++) {

    //             // for (int k = OFDM_DATA_START; k < OFDM_DATA_START + OFDM_DATA_NUM;
    //             //      k++) {
    //             printf("%.3f+%.3fi ", dl_mod_data[i][j * OFDM_CA_NUM + k].re,
    //                 dl_mod_data[i][j * OFDM_CA_NUM + k].im);
    //         }
    //         printf("\n");
    //     }
    // }

    /* perform precoding and ifft */
    Table<complex_float> dl_ifft_data;
    dl_ifft_data.calloc(
        dl_data_symbol_num_perframe, OFDM_CA_NUM * BS_ANT_NUM, 64);
    Table<short> dl_tx_data;
    dl_tx_data.calloc(
        dl_data_symbol_num_perframe, 2 * sampsPerSymbol * BS_ANT_NUM, 64);
    for (int i = 0; i < dl_data_symbol_num_perframe; i++) {
        cx_float* ptr_in_data = (cx_float*)dl_mod_data[i];
        cx_fmat mat_input_data(ptr_in_data, OFDM_CA_NUM, UE_ANT_NUM, false);
        cx_float* ptr_out = (cx_float*)dl_ifft_data[i];
        cx_fmat mat_output(ptr_out, OFDM_CA_NUM, BS_ANT_NUM, false);
        for (int j = OFDM_DATA_START; j < OFDM_DATA_NUM + OFDM_DATA_START;
             j++) {
            cx_float* ptr_in_precoder = (cx_float*)precoder[j];
            cx_fmat mat_precoder(ptr_in_precoder, UE_ANT_NUM, BS_ANT_NUM, false);
            mat_output.row(j) = mat_input_data.row(j) * mat_precoder;

            // printf("symbol %d, sc: %d\n", i, j - OFDM_DATA_START);
            // cout << "Precoder: \n" << mat_precoder << endl;
            // cout << "Data: \n" << mat_input_data.row(j) << endl;
            // cout << "Precoded data: \n" << mat_output.row(j) << endl;
        }
        for (int j = 0; j < BS_ANT_NUM; j++) {
            complex_float* ptr_ifft = dl_ifft_data[i] + j * OFDM_CA_NUM;

            CommsLib::IFFT(ptr_ifft, OFDM_CA_NUM, false);

            cx_fmat mat_data((cx_float*)ptr_ifft, 1, OFDM_CA_NUM, false);
            float scale = abs(mat_data).max();
            mat_data /= scale;

            short* txSymbol = dl_tx_data[i] + j * sampsPerSymbol * 2;
            memset(txSymbol, 0, sizeof(short) * 2 * prefix);
            for (int k = 0; k < OFDM_CA_NUM; k++) {
                txSymbol[2 * (k + CP_LEN + prefix)]
                    = (short)(32768 * ptr_ifft[k].re);
                txSymbol[2 * (k + CP_LEN + prefix) + 1]
                    = (short)(32768 * ptr_ifft[k].im);
            }
            for (int k = 0; k < 2 * CP_LEN; k++) {
                txSymbol[2 * prefix + k] = txSymbol[2 * (prefix + OFDM_CA_NUM)];
            }
            memset(txSymbol + 2 * (prefix + CP_LEN + OFDM_CA_NUM), 0,
                sizeof(short) * 2 * postfix);
        }
    }

    std::string filename_dl_tx = cur_directory
        + (kUseLDPC ? "/data/LDPC_dl_tx_data_" : "/data/dl_tx_data_")
        + std::to_string(OFDM_CA_NUM) + "_ant" + std::to_string(BS_ANT_NUM)
        + ".bin";
    printf("Saving dl tx data to %s\n", filename_dl_tx.c_str());
    FILE* fp_dl_tx = fopen(filename_dl_tx.c_str(), "wb");
    for (int i = 0; i < dl_data_symbol_num_perframe; i++) {
        short* ptr = (short*)dl_tx_data[i];
        fwrite(ptr, sampsPerSymbol * BS_ANT_NUM * 2, sizeof(short), fp_dl_tx);
    }
    fclose(fp_dl_tx);

    // printf("rx data\n");
    // for (int i = 0; i < 10; i++) {

    //     for (int j = 0; j < OFDM_CA_NUM * BS_ANT_NUM; j++) {
    //         if (j % OFDM_CA_NUM == 0) {
    //             printf("symbol %d ant %d\n", i, j / OFDM_CA_NUM);
    //         }
    //         printf("%.3f+%.3fi ", dl_tx_data[i][j].re,
    //             dl_tx_data[i][j].im);
    //     }
    // }
    // printf("\n");

    if (kUseLDPC) {
        for (size_t n = 0; n < numberCodeblocks; n++) {
            delete[] input[n];
            delete[] parity[n];
            delete[] encoded[n];
        }
    }

    mod_input.free();
    mod_output.free();
    IFFT_data.free();
    CSI_matrix.free();
    free_buffer_1d(&pilots_f);
    free_buffer_1d(&pilots_t);
    tx_data_all_symbols.free();
    rx_data_all_symbols.free();
    ue_specific_pilot.free();

    return 0;
}
