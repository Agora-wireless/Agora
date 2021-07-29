/**
 * @file data_generator.cpp
 * @brief Data generator to generate binary files as inputs to Agora, sender
 * and correctness tests
 */

#include "dynamic_generator.h"
#include "comms-lib.h"
#include "config.hpp"
#include "control.hpp"
#include "memory_manage.h"
#include "modulation.hpp"
#include "utils_ldpc.hpp"
#include <armadillo>
#include <bitset>
#include <fstream>
#include <gflags/gflags.h>
#include <immintrin.h>
#include <iostream>
#include <malloc.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static constexpr float kNoiseLevel = 1.0 / 200;
static constexpr bool kVerbose = false;
static constexpr bool kPrintUplinkInformationBytes = false;

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

static std::vector<std::vector<ControlInfo>> control_info_table;

int main(int argc, char* argv[])
{
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

    std::vector<size_t> zc_list = {4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
        15, 16, 18, 20, 22, 24, 26, 28, 30, 32, 36, 40, 44, 48, 52, 56, 
        60, 64, 72, 80, 88, 96, 104};
    
    {
        const std::string filename_input = cur_directory
            + "/data/control_ue_template.bin";
        FILE* fp_input = fopen(filename_input.c_str(), "rb");
        // for (size_t i = 0; i < kNumSlot; i ++) {
        for (size_t i = 0; i < cfg->user_level_list.size() * cfg->num_load_levels; i ++) {
            // size_t num_ue = (i / kNumLoadSetting + 1) * (cfg->UE_NUM / kNumUESetting);
            size_t num_ue = cfg->user_level_list[i / cfg->num_load_levels];
            std::vector<ControlInfo> info_list;
            ControlInfo tmp;
            for (size_t j = 0; j < num_ue; j ++) {
                fread(&tmp, sizeof(ControlInfo), 1, fp_input);
                info_list.push_back(tmp);
            }
            control_info_table.push_back(info_list);
        }
        fclose(fp_input);
    }

    // Step 1: Generate the information buffers and LDPC-encoded buffers for
    // uplink
    // const size_t num_codeblocks = cfg->UE_NUM * kNumSlot;
    const size_t num_codeblocks = cfg->UE_NUM * cfg->user_level_list.size() * cfg->num_load_levels;
    printf("Total number of blocks: %zu\n", num_codeblocks);

    std::vector<std::vector<int8_t>> information(num_codeblocks);
    std::vector<std::vector<int8_t>> encoded_codewords(num_codeblocks);
    // for (size_t i = 0; i < kNumSlot; i++) {
    for (size_t i = 0; i < cfg->user_level_list.size() * cfg->num_load_levels; i ++) {
        // size_t num_ue = (i / kNumLoadSetting + 1) * (cfg->UE_NUM / kNumUESetting);
        size_t num_ue = cfg->user_level_list[i / cfg->num_load_levels];
        for (size_t j = 0; j < num_ue; j ++) {
            size_t Bg = control_info_table[i][j].Bg;
            size_t Zc = control_info_table[i][j].Zc;
            data_generator.gen_codeblock_ul(
                information[i * cfg->UE_NUM + j], 
                encoded_codewords[i * cfg->UE_NUM + j], Bg, Zc);
            // information[i * cfg->UE_NUM + j].resize(bits_to_bytes(ldpc_num_input_bits(Bg, Zc)));
            // encoded_codewords[i * cfg->UE_NUM + j].resize(bits_to_bytes(ldpc_num_encoded_bits(Bg, Zc, Bg == 1 ? 46 : 42)));
        }
    }

    {
        // Save uplink information bytes to file
        const std::string filename_input = cur_directory
            + "/data/LDPC_orig_data_" + std::to_string(cfg->OFDM_CA_NUM)
            + "_ant" + std::to_string(cfg->UE_ANT_NUM) + "_dynamic.bin";
        printf("Saving raw data (using LDPC) to %s\n", filename_input.c_str());
        FILE* fp_input = fopen(filename_input.c_str(), "wb");

        // for (size_t i = 0; i < kNumSlot; i++) {
        for (size_t i = 0; i < cfg->user_level_list.size() * cfg->num_load_levels; i ++) {
            // size_t num_ue = (i / kNumLoadSetting + 1) * (cfg->UE_NUM / kNumUESetting);
            size_t num_ue = cfg->user_level_list[i / cfg->num_load_levels];
            for (size_t j = 0; j < num_ue; j ++) {
                size_t Bg = control_info_table[i][j].Bg;
                size_t Zc = control_info_table[i][j].Zc;
                size_t input_bytes_per_cb = bits_to_bytes(
                    ldpc_num_input_bits(Bg, Zc));
                fwrite(reinterpret_cast<uint8_t*>(&information[i * cfg->UE_NUM + j][0]),
                        input_bytes_per_cb, sizeof(uint8_t), fp_input);
            }
        }
        fclose(fp_input);
    }

    // Modulate the encoded codewords
    std::vector<std::vector<complex_float>> modulated_codewords(num_codeblocks);
    // for (size_t i = 0; i < kNumSlot; i++) {
    for (size_t i = 0; i < cfg->user_level_list.size() * cfg->num_load_levels; i ++) {
        // size_t num_ue = (i / kNumLoadSetting + 1) * (cfg->UE_NUM / kNumUESetting);
        size_t num_ue = cfg->user_level_list[i / cfg->num_load_levels];
        for (size_t j = 0; j < num_ue; j ++) {
            size_t Bg = control_info_table[i][j].Bg;
            size_t Zc = control_info_table[i][j].Zc;
            modulated_codewords[i * cfg->UE_NUM + j]
                = data_generator.get_modulation(encoded_codewords[i * cfg->UE_NUM + j],
                    cfg->mod_order_bits, Bg, Zc);
        } 
    }

    // Place modulated uplink data codewords into central IFFT bins
    rt_assert(cfg->LDPC_config.nblocksInSymbol == 1); // TODO: Assumption
    // std::vector<std::vector<complex_float>> pre_ifft_data_syms(
    //     cfg->UE_ANT_NUM * kNumSlot);
    std::vector<std::vector<complex_float>> pre_ifft_data_syms(
        cfg->UE_ANT_NUM * cfg->user_level_list.size() * cfg->num_load_levels);
    // for (size_t i = 0; i < kNumSlot; i++) {
    for (size_t i = 0; i < cfg->user_level_list.size() * cfg->num_load_levels; i ++) {
        // size_t num_ue = (i / kNumLoadSetting + 1) * (cfg->UE_NUM / kNumUESetting);
        size_t num_ue = cfg->user_level_list[i / cfg->num_load_levels];
        for (size_t j = 0; j < num_ue; j ++) {
            size_t sc_start = control_info_table[i][j].sc_start;
            size_t sc_end = control_info_table[i][j].sc_end;
            pre_ifft_data_syms[i * cfg->UE_NUM + j]
                = data_generator.bin_for_ifft(modulated_codewords[i * cfg->UE_NUM + j],
                    sc_start, sc_end);
        }
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
    // tx_data_all_symbols.calloc(
    //     2 * kNumSlot, cfg->UE_ANT_NUM * cfg->OFDM_CA_NUM, 64);
    tx_data_all_symbols.calloc(
        2 * cfg->user_level_list.size() * cfg->num_load_levels, cfg->UE_ANT_NUM * cfg->OFDM_CA_NUM, 64);

    if (cfg->freq_orthogonal_pilot) {
        // for (size_t i = 0; i < kNumSlot; i++) {
        for (size_t i = 0; i < cfg->user_level_list.size() * cfg->num_load_levels; i ++) {
            // size_t num_ue = (i / kNumLoadSetting + 1) * (cfg->UE_NUM / kNumUESetting);
            size_t num_ue = cfg->user_level_list[i / cfg->num_load_levels];
            for (size_t j = 0; j < num_ue; j ++) {
                std::vector<complex_float> pilots_t_ue(cfg->OFDM_CA_NUM); // Zeroed
                size_t sc_start = control_info_table[i][j].sc_start;
                size_t sc_end = control_info_table[i][j].sc_end;
                for (size_t k = cfg->OFDM_DATA_START + sc_start; 
                    k < cfg->OFDM_DATA_START + sc_end; k += cfg->UE_NUM) {
                    pilots_t_ue[j + k] = pilot_td[j + k];
                }
                // Load pilot to the second symbol
                // The first symbol is reserved for beacon
                memcpy(tx_data_all_symbols[2 * i]
                        + j * cfg->OFDM_CA_NUM,
                    &pilots_t_ue[0], cfg->OFDM_CA_NUM * sizeof(complex_float));
            }
        }
    } else {
        for (size_t i = 0; i < cfg->UE_ANT_NUM; i++)
            memcpy(tx_data_all_symbols[i + cfg->beacon_symbol_num_perframe]
                    + i * cfg->OFDM_CA_NUM,
                &pilot_td[0], cfg->OFDM_CA_NUM * sizeof(complex_float));
    }

    // for (size_t i = 0; i < kNumSlot; i++) {
    for (size_t i = 0; i < cfg->user_level_list.size() * cfg->num_load_levels; i ++) {
        // size_t num_ue = (i / kNumLoadSetting + 1) * (cfg->UE_NUM / kNumUESetting);
        size_t num_ue = cfg->user_level_list[i / cfg->num_load_levels];
        for (size_t j = 0; j < num_ue; j++) {
            memcpy(tx_data_all_symbols[2 * i + 1] + j * cfg->OFDM_CA_NUM,
                &pre_ifft_data_syms[i * cfg->UE_ANT_NUM + j][0],
                cfg->OFDM_CA_NUM * sizeof(complex_float));
        }
    }

    // Generate CSI matrix
    Table<complex_float> csi_matrices;
    csi_matrices.calloc(
        cfg->OFDM_CA_NUM, cfg->UE_ANT_NUM * cfg->BS_ANT_NUM, 32);
    for (size_t i = 0; i < cfg->UE_ANT_NUM * cfg->BS_ANT_NUM; i++) {
        complex_float csi
            = { rand_float_from_short(-1, 1), rand_float_from_short(-1, 1) };
        // printf("noise of ant %d, ue %d\n", i % cfg->BS_ANT_NUM, i / cfg->BS_ANT_NUM );
        for (size_t j = 0; j < cfg->OFDM_CA_NUM; j++) {
            complex_float noise = { rand_float_from_short(-1, 1) * kNoiseLevel,
                rand_float_from_short(-1, 1) * kNoiseLevel };
            // printf("%.4f+%.4fi ", noise.re, noise.im);
            csi_matrices[j][i].re = csi.re + noise.re;
            csi_matrices[j][i].im = csi.im + noise.im;
        }
        // printf("\n");
    }

    // Generate RX data received by base station after going through channels
    Table<complex_float> rx_data_all_symbols;
    // rx_data_all_symbols.calloc(
    //     2 * kNumSlot, cfg->OFDM_CA_NUM * cfg->BS_ANT_NUM, 64);
    rx_data_all_symbols.calloc(
        2 * cfg->user_level_list.size() * cfg->num_load_levels, cfg->OFDM_CA_NUM * cfg->BS_ANT_NUM, 64);
    for (size_t i = 0; i < 2 * cfg->user_level_list.size() * cfg->num_load_levels; i++) {
        arma::cx_fmat mat_input_data(
            reinterpret_cast<arma::cx_float*>(tx_data_all_symbols[i]),
            cfg->OFDM_CA_NUM, cfg->UE_ANT_NUM, false);
        arma::cx_fmat mat_output(
            reinterpret_cast<arma::cx_float*>(rx_data_all_symbols[i]),
            cfg->OFDM_CA_NUM, cfg->BS_ANT_NUM, false);

        for (size_t j = 0; j < cfg->OFDM_CA_NUM; j++) {
            arma::cx_fmat mat_csi(
                reinterpret_cast<arma::cx_float*>(csi_matrices[j]),
                cfg->BS_ANT_NUM, cfg->UE_ANT_NUM);
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
    for (size_t i = 0; i < 2 * cfg->user_level_list.size() * cfg->num_load_levels; i++) {
        auto* ptr = (float*)rx_data_all_symbols[i];
        fwrite(
            ptr, cfg->OFDM_CA_NUM * cfg->BS_ANT_NUM * 2, sizeof(float), fp_rx);
    }
    fclose(fp_rx);

    csi_matrices.free();
    tx_data_all_symbols.free();
    rx_data_all_symbols.free();
    ue_specific_pilot.free();

    return 0;
}
