/*
    accuracy and performance test for ldpc encoder implemented with AVX256 and
   Intel's decoder
 */
#ifdef USE_LDPC
#include "encoder.hpp"
#include "iobuffer.hpp"
#include "phy_ldpc_decoder_5gnr.h"
#endif
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

#include "comms-lib.h"
#include "config.hpp"
#include "memory_manage.h"
#include "modulation.hpp"

#include <time.h>

using namespace arma;

static const float NOISE_LEVEL = 1.0 / 200;

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

#ifdef USE_LDPC
#ifndef __has_builtin
#define __has_builtin(x) 0
#endif

static inline uint8_t bitreverse8(uint8_t x)
{
#if __has_builtin(__builtin_bireverse8)
    return (__builtin_bitreverse8(x));
#else
    x = (x << 4) | (x >> 4);
    x = ((x & 0x33) << 2) | ((x >> 2) & 0x33);
    x = ((x & 0x55) << 1) | ((x >> 1) & 0x55);
    return (x);
#endif
}

/*
 * Copy packed, bit-reversed m-bit fields (m == mod_type) stored in
 * vec_in[0..len-1] into unpacked vec_out.  Storage at vec_out must be
 * at least 8*len/m bytes.
 */
static void adapt_bits_for_mod(
    int8_t* vec_in, uint8_t* vec_out, int len, int mod_type)
{
    int bits_avail = 0;
    uint16_t bits = 0;
    for (int i = 0; i < len; i++) {
        bits |= bitreverse8(vec_in[i]) << 8 - bits_avail;
        bits_avail += 8;
        while (bits_avail >= mod_type) {
            *vec_out++ = bits >> (16 - mod_type);
            bits <<= mod_type;
            bits_avail -= mod_type;
        }
    }
}

uint8_t select_base_matrix_entry(uint16_t Zc)
{
    uint8_t i_LS;
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
    return i_LS;
}
#endif

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

    std::string confFile;
    if (argc == 2)
        confFile = std::string("/") + std::string(argv[1]);
    else
        confFile = "/data/tddconfig-sim-ul.json";
    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string filename = cur_directory + confFile;

    printf("default config file: %s\n", filename.c_str());
    auto* config_ = new Config(filename.c_str());

#ifdef USE_LDPC
    printf("LDPC enabled\n");
#else
    printf("LDPC not enabled\n");
#endif
    if (config_->freq_orthogonal_pilot)
        printf("use frequency-orthogonal pilots\n");
    else
        printf("use time-orthogonal pilots\n");
    printf("generating encoded and modulated data........\n");
    int mod_type = config_->mod_type;
    int UE_NUM = config_->UE_NUM;
    int BS_ANT_NUM = config_->BS_ANT_NUM;
    int OFDM_CA_NUM = config_->OFDM_CA_NUM;
    int OFDM_DATA_NUM = config_->OFDM_DATA_NUM;
    int OFDM_DATA_START = config_->OFDM_DATA_START;
    int symbol_num_perframe = config_->symbol_num_perframe;
    int data_symbol_num_perframe = config_->data_symbol_num_perframe;
    int pilot_symbol_num_perframe = config_->pilot_symbol_num_perframe;
    int dl_data_symbol_num_perframe = config_->dl_data_symbol_num_perframe;
    int sampsPerSymbol = config_->sampsPerSymbol;
    int CP_LEN = config_->CP_LEN;
    int prefix = config_->prefix;
    int postfix = config_->postfix;
    // randomly generate input
    srand(time(NULL));
    // srand(0);

#ifdef USE_LDPC
    auto LDPC_config = config_->LDPC_config;
    int numberCodeblocks
        = data_symbol_num_perframe * LDPC_config.nblocksInSymbol * UE_NUM;
    uint16_t Zc = LDPC_config.Zc;
    uint16_t Bg = LDPC_config.Bg;
    int16_t decoderIter = LDPC_config.decoderIter;
    int nRows = LDPC_config.nRows;
    uint32_t cbEncLen = LDPC_config.cbEncLen;
    uint32_t cbLen = LDPC_config.cbLen;
    uint32_t cbCodewLen = LDPC_config.cbCodewLen;

    printf("total number of blocks: %d\n", numberCodeblocks);
    /* initialize buffers */
    int8_t* input[numberCodeblocks];
    int8_t* encoded[numberCodeblocks];

    int input_lenth = ((cbLen + 7) >> 3);
    for (int i = 0; i < numberCodeblocks; i++) {
        input[i] = (int8_t*)malloc(input_lenth * sizeof(int8_t));
        encoded[i]
            = (int8_t*)malloc(BG1_COL_TOTAL * PROC_BYTES * sizeof(int8_t));
    }

    // buffers for encoders
    __declspec(align(PROC_BYTES))
        int8_t internalBuffer0[BG1_ROW_TOTAL * PROC_BYTES]
        = { 0 };
    __declspec(align(PROC_BYTES))
        int8_t internalBuffer1[BG1_ROW_TOTAL * PROC_BYTES]
        = { 0 };
    __declspec(align(PROC_BYTES))
        int8_t internalBuffer2[BG1_COL_TOTAL * PROC_BYTES]
        = { 0 };

    for (int n = 0; n < numberCodeblocks; n++) {
        for (int i = 0; i < input_lenth; i++)
            input[n][i] = (int8_t)rand();
    }

    // printf("Raw input\n");
    // for (int n = 0; n < numberCodeblocks; n++) {
    //     if (n % UE_NUM == 0) {
    //         printf("symbol %d\n", n / UE_NUM);
    //     }
    //     printf("ue %d\n", n % UE_NUM);
    //     for (int i = 0; i < input_lenth; i++)
    //         // std::cout << std::bitset<8>(input[n][i]) << " ";
    //         printf("%u ", (uint8_t)input[n][i]);
    //     printf("\n");
    // }
    // printf("\n");

    /* encoder setup
     * ----------------------------------------------------------- */

    int16_t numChannelLlrs = cbCodewLen;
    const int16_t* pShiftMatrix;
    const int16_t* pMatrixNumPerCol;
    const int16_t* pAddr;

    /* i_Ls decides the base matrix entries */
    uint8_t i_LS = select_base_matrix_entry(Zc);

    if (Bg == 1) {
        pShiftMatrix = Bg1HShiftMatrix + i_LS * BG1_NONZERO_NUM;
        pMatrixNumPerCol = Bg1MatrixNumPerCol;
        pAddr = Bg1Address;
    } else {
        pShiftMatrix = Bg2HShiftMatrix + i_LS * BG2_NONZERO_NUM;
        pMatrixNumPerCol = Bg2MatrixNumPerCol;
        pAddr = Bg2Address;
    }

    /* encoding
     * --------------------------------------------------------------- */
    printf("encoding----------------------\n");
    LDPC_ADAPTER_P ldpc_adapter_func = ldpc_select_adapter_func(Zc);
    LDPC_ENCODER ldpc_encoder_func = ldpc_select_encoder_func(Bg);

    double start_time = get_time();
    for (int n = 0; n < numberCodeblocks; n++) {
        // read input into z-bit segments
        ldpc_adapter_func(input[n], internalBuffer0, Zc, cbLen, 1);
        // encode
        ldpc_encoder_func(internalBuffer0, internalBuffer1, pMatrixNumPerCol,
            pAddr, pShiftMatrix, (int16_t)Zc, i_LS);
        // scatter the output back to compacted
        // combine the input sequence and the parity bits into codeword
        // outputs
        memcpy(internalBuffer2, internalBuffer0 + 2 * PROC_BYTES,
            (cbLen / Zc - 2) * PROC_BYTES);
        memcpy(internalBuffer2 + (cbLen / Zc - 2) * PROC_BYTES, internalBuffer1,
            cbEncLen / Zc * PROC_BYTES);

        ldpc_adapter_func(encoded[n], internalBuffer2, Zc, cbCodewLen, 0);
    }
    double end_time = get_time();
    double encoding_time = end_time - start_time;
    printf("encoding time: %.3f\n", encoding_time / numberCodeblocks);
    double enc_thruput = (double)cbLen * numberCodeblocks / encoding_time;
    printf("the encoder's speed is %f Mbps\n", enc_thruput);
    int num_mod = cbCodewLen / mod_type;
#else
    int numberCodeblocks = data_symbol_num_perframe * UE_NUM;
    int num_mod = OFDM_DATA_NUM;
#endif
    Table<uint8_t> mod_input;
    Table<complex_float> mod_output;
    mod_input.calloc(numberCodeblocks, OFDM_DATA_NUM, 32);
    mod_output.calloc(numberCodeblocks, OFDM_DATA_NUM, 32);
    Table<float> mod_table;
    init_modulation_table(mod_table, mod_type);

    for (int n = 0; n < numberCodeblocks; n++) {
#ifdef USE_LDPC
        adapt_bits_for_mod(
            encoded[n], mod_input[n], (cbCodewLen + 7) >> 3, mod_type);
#else
        for (int i = 0; i < num_mod; i++)
            mod_input[n][i] = (uint8_t)(rand() % config_->mod_order);
            // printf("symbol %d ue %d\n", n / UE_NUM, n % UE_NUM );
            // for (int i = 0; i < num_mod; i++)
            //     printf("%u ", mod_input[n][i]);
            // printf("\n");
#endif
        for (int i = 0; i < OFDM_DATA_NUM; i++)
            mod_output[n][i]
                = mod_single_uint8((uint8_t)mod_input[n][i], mod_table);
    }

    printf("saving raw data...\n");
#ifdef USE_LDPC
    std::string filename_input = cur_directory + "/data/LDPC_orig_data_"
        + std::to_string(OFDM_CA_NUM) + "_ant" + std::to_string(BS_ANT_NUM)
        + ".bin";
    FILE* fp_input = fopen(filename_input.c_str(), "wb");
    for (int i = 0; i < numberCodeblocks; i++) {
        uint8_t* ptr = (uint8_t*)input[i];
        fwrite(ptr, input_lenth, sizeof(uint8_t), fp_input);
    }
    fclose(fp_input);
#else
    std::string filename_input = cur_directory + "/data/orig_data_"
        + std::to_string(OFDM_CA_NUM) + "_ant" + std::to_string(BS_ANT_NUM)
        + ".bin";
    FILE* fp_input = fopen(filename_input.c_str(), "wb");
    for (int i = 0; i < numberCodeblocks; i++) {
        uint8_t* ptr = (uint8_t*)mod_input[i];
        fwrite(ptr, OFDM_DATA_NUM, sizeof(uint8_t), fp_input);
    }
    fclose(fp_input);
#endif

    /* convert data into time domain */
    Table<complex_float> IFFT_data;
    IFFT_data.calloc(UE_NUM * data_symbol_num_perframe, OFDM_CA_NUM, 64);
    for (int i = 0; i < UE_NUM * data_symbol_num_perframe; i++) {
        memcpy(IFFT_data[i] + OFDM_DATA_START, mod_output[i],
            OFDM_DATA_NUM * sizeof(complex_float));
        // CommsLib::IFFT(IFFT_data[i], OFDM_CA_NUM);
    }

    /* generate pilot data and convert to time domain */
    float* pilots_f = (float*)aligned_alloc(64, OFDM_CA_NUM * sizeof(float));
    for (int i = 0; i < OFDM_CA_NUM; i++) {
        if (i < OFDM_DATA_START || i >= OFDM_DATA_START + OFDM_DATA_NUM)
            pilots_f[i] = 0;
        else
            pilots_f[i] = 1 - 2 * (rand() % 2);
    }
    std::string filename_pilot_f = cur_directory + "/data/pilot_f_"
        + std::to_string(OFDM_CA_NUM) + ".bin";
    FILE* fp_pilot_f = fopen(filename_pilot_f.c_str(), "wb");
    fwrite(pilots_f, OFDM_CA_NUM, sizeof(float), fp_pilot_f);
    fclose(fp_pilot_f);

    complex_float* pilots_t;
    alloc_buffer_1d(&pilots_t, OFDM_CA_NUM, 64, 1);
    for (int i = 0; i < OFDM_CA_NUM; i++)
        pilots_t[i].re = pilots_f[i];
    // CommsLib::IFFT(pilots_t, OFDM_CA_NUM);

    /* put pilot and data symbols together */
    Table<complex_float> tx_data_all_symbols;
    tx_data_all_symbols.calloc(symbol_num_perframe, UE_NUM * OFDM_CA_NUM, 64);

    if (config_->freq_orthogonal_pilot) {
        complex_float* pilots_t_ue;
        alloc_buffer_1d(&pilots_t_ue, OFDM_CA_NUM, 64, 1);
        for (int i = 0; i < UE_NUM; i++) {
            /* TODO: fix user pilots distribution in pilot symbols */
            /* Right now we assume one pilot symbol hold all user pilots
             * in freqency orthogonal pilot */
            memset(pilots_t_ue, 0, OFDM_CA_NUM * sizeof(complex_float));
            for (int j = OFDM_DATA_START; j < OFDM_DATA_START + OFDM_DATA_NUM;
                 j += UE_NUM) {
                pilots_t_ue[i + j].re = pilots_f[i + j];
            }
            // CommsLib::IFFT(pilots_t_ue, OFDM_CA_NUM);
            memcpy(tx_data_all_symbols[0] + i * OFDM_CA_NUM, pilots_t_ue,
                OFDM_CA_NUM * sizeof(complex_float));
        }
    } else {
        for (int i = 0; i < UE_NUM; i++)
            memcpy(tx_data_all_symbols[i] + i * OFDM_CA_NUM, pilots_t,
                OFDM_CA_NUM * sizeof(complex_float));
    }

    for (int i = pilot_symbol_num_perframe; i < symbol_num_perframe; i++) {
        for (int j = 0; j < UE_NUM; j++) {
            memcpy(tx_data_all_symbols[i] + j * OFDM_CA_NUM,
                IFFT_data[(i - pilot_symbol_num_perframe) * UE_NUM + j],
                OFDM_CA_NUM * sizeof(complex_float));
        }
    }

    /* generate CSI matrix */
    Table<complex_float> CSI_matrix;
    CSI_matrix.calloc(OFDM_CA_NUM, UE_NUM * BS_ANT_NUM, 32);
    for (int i = 0; i < UE_NUM * BS_ANT_NUM; i++) {
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
        cx_fmat mat_input_data(ptr_in_data, OFDM_CA_NUM, UE_NUM, false);
        cx_float* ptr_out = (cx_float*)rx_data_all_symbols[i];
        cx_fmat mat_output(ptr_out, OFDM_CA_NUM, BS_ANT_NUM, false);
        for (int j = 0; j < OFDM_CA_NUM; j++) {
            cx_float* ptr_in_csi = (cx_float*)CSI_matrix[j];
            cx_fmat mat_csi(ptr_in_csi, BS_ANT_NUM, UE_NUM);
            mat_output.row(j) = mat_input_data.row(j) * mat_csi.st();
        }
        for (int j = 0; j < BS_ANT_NUM; j++) {
            CommsLib::IFFT(
                rx_data_all_symbols[i] + j * OFDM_CA_NUM, OFDM_CA_NUM);
        }
    }

    printf("saving rx data...\n");
#ifdef USE_LDPC
    std::string filename_rx = cur_directory + "/data/LDPC_rx_data_"
        + std::to_string(OFDM_CA_NUM) + "_ant" + std::to_string(BS_ANT_NUM)
        + ".bin";
#else
    std::string filename_rx = cur_directory + "/data/rx_data_"
        + std::to_string(OFDM_CA_NUM) + "_ant" + std::to_string(BS_ANT_NUM)
        + ".bin";
#endif
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
    precoder.calloc(OFDM_CA_NUM, UE_NUM * BS_ANT_NUM, 32);
    for (int i = 0; i < OFDM_CA_NUM; i++) {
        cx_float* ptr_in = (cx_float*)CSI_matrix[i];
        cx_fmat mat_input(ptr_in, BS_ANT_NUM, UE_NUM, false);
        cx_float* ptr_out = (cx_float*)precoder[i];
        cx_fmat mat_output(ptr_out, UE_NUM, BS_ANT_NUM, false);
        pinv(mat_output, mat_input, 1e-2, "dc");
    }

    // printf("CSI \n");
    // // for (int i = 0; i < OFDM_CA_NUM; i++)
    // for (int j = 0; j < UE_NUM * BS_ANT_NUM; j++)
    //     printf("%.3f+%.3fi ",
    //         CSI_matrix[OFDM_DATA_START][j].re,
    //         CSI_matrix[OFDM_DATA_START][j].im);
    // printf("\n");
    // printf("precoder \n");
    // // for (int i = 0; i < OFDM_CA_NUM; i++)
    // for (int j = 0; j < UE_NUM * BS_ANT_NUM; j++)
    //     printf("%.3f+%.3fi ",
    //         precoder[OFDM_DATA_START][j].re,
    //         precoder[OFDM_DATA_START][j].im);
    // printf("\n");

    /* prepare downlink data from mod_output */
    Table<complex_float> dl_mod_data;
    dl_mod_data.calloc(dl_data_symbol_num_perframe, OFDM_CA_NUM * UE_NUM, 64);
    for (int i = 0; i < dl_data_symbol_num_perframe; i++) {
        for (int j = 0; j < UE_NUM; j++) {
            if (i <= DL_PILOT_SYMS - 1) {
                for (int sc_id = 0; sc_id < OFDM_CA_NUM; sc_id++)
                    dl_mod_data[i][j * OFDM_CA_NUM + sc_id] = pilots_t[sc_id];
            } else {
                memcpy(dl_mod_data[i] + j * OFDM_CA_NUM + OFDM_DATA_START,
                    mod_output[i * UE_NUM + j],
                    OFDM_DATA_NUM * sizeof(complex_float));
            }
        }
    }

    // printf("dl mod data \n");
    // for (int i = 0; i < dl_data_symbol_num_perframe; i++) {
    //     for (int k = OFDM_DATA_START; k < OFDM_DATA_START + OFDM_DATA_NUM;
    //          k++) {
    //         printf("symbol %d, subcarrier %d\n", i, k);
    //         for (int j = 0; j < UE_NUM; j++) {

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
        cx_fmat mat_input_data(ptr_in_data, OFDM_CA_NUM, UE_NUM, false);
        cx_float* ptr_out = (cx_float*)dl_ifft_data[i];
        cx_fmat mat_output(ptr_out, OFDM_CA_NUM, BS_ANT_NUM, false);
        for (int j = OFDM_DATA_START; j < OFDM_DATA_NUM + OFDM_DATA_START;
             j++) {
            cx_float* ptr_in_precoder = (cx_float*)precoder[j];
            cx_fmat mat_precoder(ptr_in_precoder, UE_NUM, BS_ANT_NUM, false);
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

    printf("saving dl tx data...\n");
#ifdef USE_LDPC
    std::string filename_dl_tx = cur_directory + "/data/LDPC_dl_tx_data_"
        + std::to_string(OFDM_CA_NUM) + "_ant" + std::to_string(BS_ANT_NUM)
        + ".bin";
#else
    std::string filename_dl_tx = cur_directory + "/data/dl_tx_data_"
        + std::to_string(OFDM_CA_NUM) + "_ant" + std::to_string(BS_ANT_NUM)
        + ".bin";
#endif

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

#ifdef USE_LDPC
    for (int n = 0; n < numberCodeblocks; n++) {
        free(input[n]);
        free(encoded[n]);
    }
#endif

    mod_input.free();
    mod_output.free();
    IFFT_data.free();
    CSI_matrix.free();
    free_buffer_1d(&pilots_t);
    tx_data_all_symbols.free();
    rx_data_all_symbols.free();

    return 0;
}
