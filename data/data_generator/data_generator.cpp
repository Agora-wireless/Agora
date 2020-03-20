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
    int ue_num = config_->ue_num;
    int bs_ant_num = config_->bs_ant_num;
    int ofdm_ca_num = config_->ofdm_ca_num;
    int ofdm_data_num = config_->ofdm_data_num;
    int ofdm_data_start = config_->ofdm_data_start;
    int symbol_num_perframe = config_->symbol_num_perframe;
    int data_symbol_num_perframe = config_->data_symbol_num_perframe;
    int pilot_symbol_num_perframe = config_->pilot_symbol_num_perframe;
    // randomly generate input
    srand(time(NULL));
    // srand(0);

#ifdef USE_LDPC
    auto LDPC_config = config_->LDPC_config;
    int numberCodeblocks
        = data_symbol_num_perframe * LDPC_config.nblocksInSymbol * ue_num;
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
    //     if (n % ue_num == 0) {
    //         printf("symbol %d\n", n / ue_num);
    //     }
    //     printf("ue %d\n", n % ue_num);
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
    int numberCodeblocks = data_symbol_num_perframe * ue_num;
    int num_mod = ofdm_data_num;
#endif
    Table<uint8_t> mod_input;
    Table<complex_float> mod_output;
    mod_input.calloc(numberCodeblocks, ofdm_data_num, 32);
    mod_output.calloc(numberCodeblocks, ofdm_data_num, 32);
    Table<float> mod_table;
    init_modulation_table(mod_table, mod_type);

    for (int n = 0; n < numberCodeblocks; n++) {
#ifdef USE_LDPC
        adapt_bits_for_mod(
            encoded[n], mod_input[n], (cbCodewLen + 7) >> 3, mod_type);
#else
        for (int i = 0; i < num_mod; i++)
            mod_input[n][i] = (uint8_t)(rand() % config_->mod_order);
            // printf("symbol %d ue %d\n", n / ue_num, n % ue_num );
            // for (int i = 0; i < num_mod; i++)
            //     printf("%u ", mod_input[n][i]);
            // printf("\n");
#endif
        for (int i = 0; i < num_mod; i++)
            mod_output[n][i]
                = mod_single_uint8((uint8_t)mod_input[n][i], mod_table);
    }

    printf("saving raw data...\n");
#ifdef USE_LDPC
    std::string filename_input = cur_directory + "/data/LDPC_orig_data_2048_ant"
        + std::to_string(bs_ant_num) + ".bin";
    FILE* fp_input = fopen(filename_input.c_str(), "wb");
    for (int i = 0; i < numberCodeblocks; i++) {
        uint8_t* ptr = (uint8_t*)input[i];
        fwrite(ptr, input_lenth, sizeof(uint8_t), fp_input);
    }
    fclose(fp_input);
#else
    std::string filename_input = cur_directory + "/data/orig_data_2048_ant"
        + std::to_string(bs_ant_num) + ".bin";
    FILE* fp_input = fopen(filename_input.c_str(), "wb");
    for (int i = 0; i < numberCodeblocks; i++) {
        uint8_t* ptr = (uint8_t*)mod_input[i];
        fwrite(ptr, ofdm_data_num, sizeof(uint8_t), fp_input);
    }
    fclose(fp_input);
#endif

    /* convert data into time domain */
    Table<complex_float> IFFT_data;
    IFFT_data.calloc(ue_num * data_symbol_num_perframe, ofdm_ca_num, 64);
    for (int i = 0; i < ue_num * data_symbol_num_perframe; i++) {
        memcpy(IFFT_data[i] + ofdm_data_start, mod_output[i],
            ofdm_data_num * sizeof(complex_float));
        // CommsLib::IFFT(IFFT_data[i], ofdm_ca_num);
    }

    /* get pilot data from file and convert to time domain */
    float* pilots_f = config_->pilots_;
    complex_float* pilots_t;
    alloc_buffer_1d(&pilots_t, ofdm_ca_num, 64, 1);
    for (int i = 0; i < ofdm_ca_num; i++)
        pilots_t[i].re = pilots_f[i];
    // CommsLib::IFFT(pilots_t, ofdm_ca_num);

    /* put pilot and data symbols together */
    Table<complex_float> tx_data_all_symbols;
    tx_data_all_symbols.calloc(symbol_num_perframe, ue_num * ofdm_ca_num, 64);

    if (config_->freq_orthogonal_pilot) {
        complex_float* pilots_t_ue;
        alloc_buffer_1d(&pilots_t_ue, ofdm_ca_num, 64, 1);
        for (int i = 0; i < ue_num; i++) {
            /* TODO: fix user pilots distribution in pilot symbols */
            /* Right now we assume one pilot symbol hold all user pilots
             * in freqency orthogonal pilot */
            memset(pilots_t_ue, 0, ofdm_ca_num * sizeof(complex_float));
            for (int j = ofdm_data_start; j < ofdm_data_start + ofdm_data_num;
                 j += ue_num) {
                pilots_t_ue[i + j].re = pilots_f[i + j];
            }
            // CommsLib::IFFT(pilots_t_ue, ofdm_ca_num);
            memcpy(tx_data_all_symbols[0] + i * ofdm_ca_num, pilots_t_ue,
                ofdm_ca_num * sizeof(complex_float));
        }
    } else {
        for (int i = 0; i < ue_num; i++)
            memcpy(tx_data_all_symbols[i] + i * ofdm_ca_num, pilots_t,
                ofdm_ca_num * sizeof(complex_float));
    }

    for (int i = pilot_symbol_num_perframe; i < symbol_num_perframe; i++) {
        for (int j = 0; j < ue_num; j++) {
            memcpy(tx_data_all_symbols[i] + j * ofdm_ca_num,
                IFFT_data[(i - pilot_symbol_num_perframe) * ue_num + j],
                ofdm_ca_num * sizeof(complex_float));
        }
    }

    /* generate CSI matrix */
    Table<complex_float> CSI_matrix;
    CSI_matrix.calloc(ofdm_ca_num, ue_num * bs_ant_num, 32);
    for (int i = 0; i < ue_num * bs_ant_num; i++) {
        complex_float csi
            = { rand_float_from_short(-1, 1), rand_float_from_short(-1, 1) };
        // printf("noise of ant %d, ue %d\n", i % bs_ant_num, i / bs_ant_num );
        for (int j = 0; j < ofdm_ca_num; j++) {
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
        symbol_num_perframe, ofdm_ca_num * bs_ant_num, 64);
    for (int i = 0; i < symbol_num_perframe; i++) {
        cx_float* ptr_in_data = (cx_float*)tx_data_all_symbols[i];
        cx_fmat mat_input_data(ptr_in_data, ofdm_ca_num, ue_num, false);
        cx_float* ptr_out = (cx_float*)rx_data_all_symbols[i];
        cx_fmat mat_output(ptr_out, ofdm_ca_num, bs_ant_num, false);
        for (int j = 0; j < ofdm_ca_num; j++) {
            cx_float* ptr_in_csi = (cx_float*)CSI_matrix[j];
            cx_fmat mat_csi(ptr_in_csi, bs_ant_num, ue_num);
            mat_output.row(j) = mat_input_data.row(j) * mat_csi.st();
        }
        for (int j = 0; j < bs_ant_num; j++) {
            CommsLib::IFFT(
                rx_data_all_symbols[i] + j * ofdm_ca_num, ofdm_ca_num);
        }
    }

    printf("saving rx data...\n");
#ifdef USE_LDPC
    std::string filename_rx = cur_directory + "/data/LDPC_rx_data_2048_ant"
        + std::to_string(bs_ant_num) + ".bin";
#else
    std::string filename_rx = cur_directory + "/data/rx_data_2048_ant"
        + std::to_string(bs_ant_num) + ".bin";
#endif
    FILE* fp_rx = fopen(filename_rx.c_str(), "wb");
    for (int i = 0; i < symbol_num_perframe; i++) {
        float* ptr = (float*)rx_data_all_symbols[i];
        fwrite(ptr, ofdm_ca_num * bs_ant_num * 2, sizeof(float), fp_rx);
    }
    fclose(fp_rx);

    // printf("rx data\n");
    // for (int i = 0; i < 10; i++) {
    //     for (int j = 0; j < ofdm_ca_num * bs_ant_num; j++) {
    //         if (j % ofdm_ca_num == 0) {
    //             printf("\nsymbol %d ant %d\n", i, j / ofdm_ca_num);
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
    precoder.calloc(ofdm_ca_num, ue_num * bs_ant_num, 32);
    for (int i = 0; i < ofdm_ca_num; i++) {
        cx_float* ptr_in = (cx_float*)CSI_matrix[i];
        cx_fmat mat_input(ptr_in, bs_ant_num, ue_num, false);
        cx_float* ptr_out = (cx_float*)precoder[i];
        cx_fmat mat_output(ptr_out, ue_num, bs_ant_num, false);
        pinv(mat_output, mat_input, 1e-1, "dc");
    }

    // printf("CSI \n");
    // // for (int i = 0; i < ofdm_ca_num; i++)
    // for (int j = 0; j < ue_num * bs_ant_num; j++)
    //     printf("%.3f+%.3fi ",
    //         CSI_matrix[ofdm_data_start][j].re,
    //         CSI_matrix[ofdm_data_start][j].im);
    // printf("\n");
    // printf("precoder \n");
    // // for (int i = 0; i < ofdm_ca_num; i++)
    // for (int j = 0; j < ue_num * bs_ant_num; j++)
    //     printf("%.3f+%.3fi ",
    //         precoder[ofdm_data_start][j].re,
    //         precoder[ofdm_data_start][j].im);
    // printf("\n");

    /* prepare downlink data from mod_output */
    Table<complex_float> dl_mod_data;
    dl_mod_data.calloc(data_symbol_num_perframe, ofdm_ca_num * ue_num, 64);
    for (int i = 0; i < data_symbol_num_perframe; i++) {
        for (int j = 0; j < ue_num; j++)
            memcpy(dl_mod_data[i] + j * ofdm_ca_num + ofdm_data_start,
                mod_output[i * ue_num + j],
                ofdm_data_num * sizeof(complex_float));
    }

    // printf("dl mod data \n");
    // for (int i = 0; i < 10; i++) {
    //     for (int j = 0; j < ue_num; j++) {
    //         printf("symbol %d, ue %d\n", i, j);
    //         for (int k = ofdm_data_start;
    //             k < ofdm_data_start + ofdm_data_num; k++) {
    //             printf("%.3f+%.3fi ",
    //                 dl_mod_data[i][j * ofdm_ca_num + k].re,
    //                 dl_mod_data[i][j * ofdm_ca_num + k].im);
    //         }
    //         printf("\n");
    //     }
    // }

    /* perform precoding and ifft */
    Table<complex_float> dl_tx_data;
    dl_tx_data.calloc(data_symbol_num_perframe, ofdm_ca_num * bs_ant_num, 64);
    for (int i = 0; i < data_symbol_num_perframe; i++) {
        cx_float* ptr_in_data = (cx_float*)dl_mod_data[i];
        cx_fmat mat_input_data(ptr_in_data, ofdm_ca_num, ue_num, false);
        cx_float* ptr_out = (cx_float*)dl_tx_data[i];
        cx_fmat mat_output(ptr_out, ofdm_ca_num, bs_ant_num, false);
        for (int j = ofdm_data_start; j < ofdm_data_num + ofdm_data_start;
             j++) {
            cx_float* ptr_in_precoder = (cx_float*)precoder[j];
            cx_fmat mat_precoder(ptr_in_precoder, ue_num, bs_ant_num, false);
            mat_output.row(j) = mat_input_data.row(j) * mat_precoder;
            // if (i < 10) {
            //     printf("symbol %d, sc: %d\n", i, j - ofdm_data_start);
            //     cout<<"Precoder: \n"<<mat_precoder<<endl;
            //     cout<<"Data: \n"<<mat_input_data.row(j)<<endl;
            //     cout <<"Precoded data: \n" << mat_output.row(j)<<endl;
            // }
        }
        for (int j = 0; j < bs_ant_num; j++) {
            complex_float* ptr_ifft = dl_tx_data[i] + j * ofdm_ca_num;
            CommsLib::IFFT(ptr_ifft, ofdm_ca_num, false);
        }
    }

    printf("saving dl tx data...\n");
#ifdef USE_LDPC
    std::string filename_dl_tx = cur_directory
        + "/data/LDPC_dl_ifft_data_2048_ant" + std::to_string(bs_ant_num)
        + ".bin";
#else
    std::string filename_dl_tx = cur_directory + "/data/dl_ifft_data_2048_ant"
        + std::to_string(bs_ant_num) + ".bin";
#endif

    FILE* fp_dl_tx = fopen(filename_dl_tx.c_str(), "wb");
    for (int i = 0; i < data_symbol_num_perframe; i++) {
        float* ptr = (float*)dl_tx_data[i];
        fwrite(ptr, ofdm_ca_num * bs_ant_num * 2, sizeof(float), fp_dl_tx);
    }
    fclose(fp_dl_tx);

    // printf("rx data\n");
    // for (int i = 0; i < 10; i++) {

    //     for (int j = 0; j < ofdm_ca_num * bs_ant_num; j++) {
    //         if (j % ofdm_ca_num == 0) {
    //             printf("symbol %d ant %d\n", i, j / ofdm_ca_num);
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