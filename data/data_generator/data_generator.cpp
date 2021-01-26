/**
 * @file data_generator.cpp
 * @brief Data generator to generate binary files as inputs to Agora, sender
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
#include <gflags/gflags.h>
#include <immintrin.h>
#include <iostream>

static constexpr bool kVerbose = false;
static constexpr bool kPrintUplinkInformationBytes = false;
static constexpr bool kPrintDownlinkInformationBytes = false;

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
    const size_t num_ul_codeblocks = cfg->ul_data_symbol_num_perframe
        * (cfg->LDPC_config.nblocksInSymbol * cfg->UE_ANT_NUM);
    std::printf("Total number of ul blocks: %zu\n", num_ul_codeblocks);

    std::vector<std::vector<int8_t>> ul_information(num_ul_codeblocks);
    std::vector<std::vector<int8_t>> ul_encoded_codewords(num_ul_codeblocks);
    for (size_t i = 0; i < num_ul_codeblocks; i++) {
        data_generator.gen_codeblock(ul_information.at(i),
            ul_encoded_codewords.at(i), (i % cfg->UE_NUM) /* UE ID */);
    }

    {
        // Save uplink information bytes to file
        const size_t input_bytes_per_cb = bits_to_bytes(
            ldpc_num_input_bits(cfg->LDPC_config.Bg, cfg->LDPC_config.Zc));

        const std::string filename_input = cur_directory
            + "/data/LDPC_orig_ul_data_" + std::to_string(cfg->OFDM_CA_NUM)
            + "_ant" + std::to_string(cfg->UE_ANT_NUM) + ".bin";
        std::printf("Saving raw uplink data (using LDPC) to %s\n",
            filename_input.c_str());
        FILE* fp_input = std::fopen(filename_input.c_str(), "wb");
        for (size_t i = 0; i < num_ul_codeblocks; i++) {
            std::fwrite(reinterpret_cast<uint8_t*>(&ul_information.at(i).at(0)),
                input_bytes_per_cb, sizeof(uint8_t), fp_input);
        }
        std::fclose(fp_input);

        if (kPrintUplinkInformationBytes) {
            std::printf("Uplink information bytes\n");
            for (size_t n = 0; n < num_ul_codeblocks; n++) {
                std::printf("Symbol %zu, UE %zu\n", n / cfg->UE_ANT_NUM,
                    n % cfg->UE_ANT_NUM);
                for (size_t i = 0; i < input_bytes_per_cb; i++) {
                    std::printf("%u ", (uint8_t)ul_information.at(n).at(i));
                }
                std::printf("\n");
            }
        }
    }

    // Modulate the encoded codewords
    std::vector<std::vector<complex_float>> ul_modulated_codewords(
        num_ul_codeblocks);
    for (size_t i = 0; i < num_ul_codeblocks; i++) {
        ul_modulated_codewords.at(i)
            = data_generator.get_modulation(ul_encoded_codewords[i]);
    }

    // Place modulated uplink data codewords into central IFFT bins
    rt_assert(cfg->LDPC_config.nblocksInSymbol == 1); // TODO: Assumption
    std::vector<std::vector<complex_float>> pre_ifft_data_syms(
        cfg->UE_ANT_NUM * cfg->ul_data_symbol_num_perframe);
    for (size_t i = 0; i < pre_ifft_data_syms.size(); i++) {
        pre_ifft_data_syms.at(i)
            = data_generator.bin_for_ifft(ul_modulated_codewords.at(i));
    }

    std::vector<complex_float> pilot_td
        = data_generator.get_common_pilot_time_domain();

    // Generate UE-specific pilots
    Table<complex_float> ue_specific_pilot;
    const std::vector<std::complex<float>> zc_seq = Utils::double_to_cfloat(
        CommsLib::getSequence(cfg->OFDM_DATA_NUM, CommsLib::LTE_ZADOFF_CHU));
    const std::vector<std::complex<float>> zc_common_pilot
        = CommsLib::seqCyclicShift(zc_seq, M_PI / 4.0); // Used in LTE SRS
    ue_specific_pilot.malloc(cfg->UE_ANT_NUM, cfg->OFDM_DATA_NUM,
        Agora_memory::Alignment_t::k64Align);
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
    tx_data_all_symbols.calloc(cfg->symbol_num_perframe,
        cfg->UE_ANT_NUM * cfg->OFDM_CA_NUM,
        Agora_memory::Alignment_t::k64Align);

    if (cfg->freq_orthogonal_pilot) {
        for (size_t i = 0; i < cfg->UE_ANT_NUM; i++) {
            std::vector<complex_float> pilots_t_ue(cfg->OFDM_CA_NUM); // Zeroed
            for (size_t j = cfg->OFDM_DATA_START;
                 j < cfg->OFDM_DATA_START + cfg->OFDM_DATA_NUM;
                 j += cfg->UE_ANT_NUM) {
                pilots_t_ue.at(i + j) = pilot_td.at(i + j);
            }
            // Load pilot to the second symbol
            // The first symbol is reserved for beacon
            std::memcpy(tx_data_all_symbols[cfg->beacon_symbol_num_perframe]
                    + (i * cfg->OFDM_CA_NUM),
                &pilots_t_ue.at(0), cfg->OFDM_CA_NUM * sizeof(complex_float));
        }
    } else {
        for (size_t i = 0; i < cfg->UE_ANT_NUM; i++)
            std::memcpy(tx_data_all_symbols[i + cfg->beacon_symbol_num_perframe]
                    + i * cfg->OFDM_CA_NUM,
                &pilot_td.at(0), cfg->OFDM_CA_NUM * sizeof(complex_float));
    }

    // Populate the UL symbols
    for (size_t i = 0; i < cfg->ul_data_symbol_num_perframe; i++) {
        const size_t data_sym_id = cfg->ULSymbols.at(0).at(i);
        for (size_t j = 0; j < cfg->UE_ANT_NUM; j++) {
            if (i < cfg->UL_PILOT_SYMS) {
                std::memcpy(tx_data_all_symbols[data_sym_id]
                        + (j * cfg->OFDM_CA_NUM) + cfg->OFDM_DATA_START,
                    ue_specific_pilot[j],
                    cfg->OFDM_DATA_NUM * sizeof(complex_float));
            } else {
                std::memcpy(
                    tx_data_all_symbols[data_sym_id] + (j * cfg->OFDM_CA_NUM),
                    &pre_ifft_data_syms.at(i * cfg->UE_ANT_NUM + j).at(0),
                    cfg->OFDM_CA_NUM * sizeof(complex_float));
            }
        }
    }
    // Generate CSI matrix
    Table<complex_float> csi_matrices;
    csi_matrices.calloc(cfg->OFDM_CA_NUM, cfg->UE_ANT_NUM * cfg->BS_ANT_NUM,
        Agora_memory::Alignment_t::k32Align);
    for (size_t i = 0; i < cfg->UE_ANT_NUM * cfg->BS_ANT_NUM; i++) {
        complex_float csi
            = { rand_float_from_short(-1, 1), rand_float_from_short(-1, 1) };
        // std::printf("noise of ant %d, ue %d\n", i % cfg->BS_ANT_NUM, i / cfg->BS_ANT_NUM );
        for (size_t j = 0; j < cfg->OFDM_CA_NUM; j++) {
            complex_float noise
                = { rand_float_from_short(-1, 1) * cfg->noise_level,
                      rand_float_from_short(-1, 1) * cfg->noise_level };
            // std::printf("%.4f+%.4fi ", noise.re, noise.im);
            csi_matrices[j][i].re = csi.re + noise.re;
            csi_matrices[j][i].im = csi.im + noise.im;
        }
        // std::printf("\n");
    }

    // Generate RX data received by base station after going through channels
    Table<complex_float> rx_data_all_symbols;
    rx_data_all_symbols.calloc(cfg->symbol_num_perframe,
        cfg->OFDM_CA_NUM * cfg->BS_ANT_NUM,
        Agora_memory::Alignment_t::k64Align);
    for (size_t i = 0; i < cfg->symbol_num_perframe; i++) {
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
    std::printf("Saving rx data to %s\n", filename_rx.c_str());
    FILE* fp_rx = std::fopen(filename_rx.c_str(), "wb");
    for (size_t i = 0; i < cfg->symbol_num_perframe; i++) {
        auto* ptr = (float*)rx_data_all_symbols[i];
        std::fwrite(
            ptr, cfg->OFDM_CA_NUM * cfg->BS_ANT_NUM * 2, sizeof(float), fp_rx);
    }
    std::fclose(fp_rx);

    // std::printf("rx data\n");
    // for (int i = 0; i < 10; i++) {
    //     for (int j = 0; j < cfg->OFDM_CA_NUM * cfg->BS_ANT_NUM; j++) {
    //         if (j % cfg->OFDM_CA_NUM == 0) {
    //             std::printf("\nsymbol %d ant %d\n", i, j / cfg->OFDM_CA_NUM);
    //         }
    //         std::printf("%.4f+%.4fi ", rx_data_all_symbols[i][j].re,
    //             rx_data_all_symbols[i][j].im);
    //     }
    //     std::printf("\n");
    // }

    /* ------------------------------------------------
     * Generate data for downlink test
     * ------------------------------------------------ */
    const size_t num_dl_codeblocks = cfg->dl_data_symbol_num_perframe
        * cfg->LDPC_config.nblocksInSymbol * cfg->UE_ANT_NUM;
    std::printf("Total number of dl blocks: %zu\n", num_dl_codeblocks);

    std::vector<std::vector<int8_t>> dl_information(num_dl_codeblocks);
    std::vector<std::vector<int8_t>> dl_encoded_codewords(num_dl_codeblocks);
    for (size_t i = 0; i < num_dl_codeblocks; i++) {
        data_generator.gen_codeblock(dl_information.at(i),
            dl_encoded_codewords.at(i), (i % cfg->UE_NUM) /* UE ID */);
    }

    // Modulate the encoded codewords
    std::vector<std::vector<complex_float>> dl_modulated_codewords(
        num_dl_codeblocks);
    for (size_t i = 0; i < num_dl_codeblocks; i++) {
        dl_modulated_codewords.at(i)
            = data_generator.get_modulation(dl_encoded_codewords[i]);
    }

    {
        // Save downlink information bytes to file
        const size_t input_bytes_per_cb = bits_to_bytes(
            ldpc_num_input_bits(cfg->LDPC_config.Bg, cfg->LDPC_config.Zc));

        const std::string filename_input = cur_directory
            + "/data/LDPC_orig_dl_data_" + std::to_string(cfg->OFDM_CA_NUM)
            + "_ant" + std::to_string(cfg->UE_ANT_NUM) + ".bin";
        std::printf(
            "Saving raw dl data (using LDPC) to %s\n", filename_input.c_str());
        FILE* fp_input = std::fopen(filename_input.c_str(), "wb");
        for (size_t i = 0; i < num_dl_codeblocks; i++) {
            std::fwrite(reinterpret_cast<uint8_t*>(&dl_information.at(i).at(0)),
                input_bytes_per_cb, sizeof(uint8_t), fp_input);
        }
        std::fclose(fp_input);

        if (kPrintDownlinkInformationBytes == true) {
            std::printf("Downlink information bytes\n");
            for (size_t n = 0; n < num_dl_codeblocks; n++) {
                std::printf("Symbol %zu, UE %zu\n", n / cfg->UE_ANT_NUM,
                    n % cfg->UE_ANT_NUM);
                for (size_t i = 0; i < input_bytes_per_cb; i++) {
                    std::printf("%u ", (uint8_t)dl_information.at(n).at(i));
                }
                std::printf("\n");
            }
        }
    }

    // Compute precoder
    Table<complex_float> precoder;
    precoder.calloc(cfg->OFDM_CA_NUM, cfg->UE_ANT_NUM * cfg->BS_ANT_NUM,
        Agora_memory::Alignment_t::k32Align);
    for (size_t i = 0; i < cfg->OFDM_CA_NUM; i++) {
        arma::cx_fmat mat_input(
            reinterpret_cast<arma::cx_float*>(csi_matrices[i]), cfg->BS_ANT_NUM,
            cfg->UE_ANT_NUM, false);
        arma::cx_fmat mat_output(reinterpret_cast<arma::cx_float*>(precoder[i]),
            cfg->UE_ANT_NUM, cfg->BS_ANT_NUM, false);
        pinv(mat_output, mat_input, 1e-2, "dc");
    }

    // std::printf("CSI \n");
    // // for (int i = 0; i < cfg->OFDM_CA_NUM; i++)
    // for (int j = 0; j < cfg->UE_ANT_NUM * cfg->BS_ANT_NUM; j++)
    //     std::printf("%.3f+%.3fi ",
    //         csi_matrices[cfg->OFDM_DATA_START][j].re,
    //         csi_matrices[cfg->OFDM_DATA_START][j].im);
    // std::printf("\n");
    // std::printf("precoder \n");
    // // for (int i = 0; i < cfg->OFDM_CA_NUM; i++)
    // for (int j = 0; j < cfg->UE_ANT_NUM * cfg->BS_ANT_NUM; j++)
    //     std::printf("%.3f+%.3fi ",
    //         precoder[cfg->OFDM_DATA_START][j].re,
    //         precoder[cfg->OFDM_DATA_START][j].im);
    // std::printf("\n");

    // Prepare downlink data from mod_output
    Table<complex_float> dl_mod_data;
    dl_mod_data.calloc(cfg->dl_data_symbol_num_perframe,
        cfg->OFDM_CA_NUM * cfg->UE_ANT_NUM,
        Agora_memory::Alignment_t::k64Align);
    for (size_t i = 0; i < cfg->dl_data_symbol_num_perframe; i++) {
        for (size_t j = 0; j < cfg->UE_ANT_NUM; j++) {
            if (i >= cfg->DL_PILOT_SYMS) {
                for (size_t sc_id = 0; sc_id < cfg->OFDM_DATA_NUM; sc_id++)
                    dl_mod_data[i][j * cfg->OFDM_CA_NUM + sc_id
                        + cfg->OFDM_DATA_START]
                        = (sc_id % cfg->OFDM_PILOT_SPACING == 0)
                        ? ue_specific_pilot[0][sc_id]
                        : dl_modulated_codewords.at(i * cfg->UE_ANT_NUM + j)
                              .at(sc_id);
            } else {
                for (size_t sc_id = 0; sc_id < cfg->OFDM_DATA_NUM; sc_id++)
                    dl_mod_data[i][j * cfg->OFDM_CA_NUM + sc_id
                        + cfg->OFDM_DATA_START]
                        = ue_specific_pilot[0][sc_id];
            }
        }
    }

    // std::printf("dl mod data \n");
    // for (int i = 0; i < dl_data_symbol_num_perframe; i++) {
    //     for (int k = cfg->OFDM_DATA_START; k < cfg->OFDM_DATA_START + cfg->OFDM_DATA_NUM;
    //          k++) {
    //         std::printf("symbol %d, subcarrier %d\n", i, k);
    //         for (int j = 0; j < cfg->UE_ANT_NUM; j++) {

    //             // for (int k = cfg->OFDM_DATA_START; k < cfg->OFDM_DATA_START + cfg->OFDM_DATA_NUM;
    //             //      k++) {
    //             std::printf("%.3f+%.3fi ", dl_mod_data[i][j * cfg->OFDM_CA_NUM + k].re,
    //                 dl_mod_data[i][j * cfg->OFDM_CA_NUM + k].im);
    //         }
    //         std::printf("\n");
    //     }
    // }

    // Perform precoding and IFFT
    Table<complex_float> dl_ifft_data;
    dl_ifft_data.calloc(cfg->dl_data_symbol_num_perframe,
        cfg->OFDM_CA_NUM * cfg->BS_ANT_NUM,
        Agora_memory::Alignment_t::k64Align);
    Table<short> dl_tx_data;
    dl_tx_data.calloc(cfg->dl_data_symbol_num_perframe,
        2 * cfg->sampsPerSymbol * cfg->BS_ANT_NUM,
        Agora_memory::Alignment_t::k64Align);
    for (size_t i = 0; i < cfg->dl_data_symbol_num_perframe; i++) {
        arma::cx_fmat mat_input_data(
            reinterpret_cast<arma::cx_float*>(dl_mod_data[i]), cfg->OFDM_CA_NUM,
            cfg->UE_ANT_NUM, false);

        arma::cx_fmat mat_output(
            reinterpret_cast<arma::cx_float*>(dl_ifft_data[i]),
            cfg->OFDM_CA_NUM, cfg->BS_ANT_NUM, false);

        for (size_t j = cfg->OFDM_DATA_START;
             j < cfg->OFDM_DATA_NUM + cfg->OFDM_DATA_START; j++) {
            arma::cx_fmat mat_precoder(
                reinterpret_cast<arma::cx_float*>(precoder[j]), cfg->UE_ANT_NUM,
                cfg->BS_ANT_NUM, false);
            mat_precoder /= abs(mat_precoder).max();
            mat_output.row(j) = mat_input_data.row(j) * mat_precoder;

            // std::printf("symbol %d, sc: %d\n", i, j - cfg->OFDM_DATA_START);
            // cout << "Precoder: \n" << mat_precoder << endl;
            // cout << "Data: \n" << mat_input_data.row(j) << endl;
            // cout << "Precoded data: \n" << mat_output.row(j) << endl;
        }
        for (size_t j = 0; j < cfg->BS_ANT_NUM; j++) {
            complex_float* ptr_ifft = dl_ifft_data[i] + j * cfg->OFDM_CA_NUM;
            CommsLib::IFFT(ptr_ifft, cfg->OFDM_CA_NUM, false);

            short* txSymbol = dl_tx_data[i] + j * cfg->sampsPerSymbol * 2;
            std::memset(
                txSymbol, 0, sizeof(short) * 2 * cfg->ofdm_tx_zero_prefix_);
            for (size_t k = 0; k < cfg->OFDM_CA_NUM; k++) {
                txSymbol[2 * (k + cfg->CP_LEN + cfg->ofdm_tx_zero_prefix_)]
                    = (short)(32768 * ptr_ifft[k].re
                        * std::sqrt(cfg->BS_ANT_NUM * 1.f));
                txSymbol[2 * (k + cfg->CP_LEN + cfg->ofdm_tx_zero_prefix_) + 1]
                    = (short)(32768 * ptr_ifft[k].im
                        * std::sqrt(cfg->BS_ANT_NUM * 1.f));
            }
            for (size_t k = 0; k < 2 * cfg->CP_LEN; k++) {
                txSymbol[2 * cfg->ofdm_tx_zero_prefix_ + k] = txSymbol[2
                    * (cfg->ofdm_tx_zero_prefix_ + cfg->OFDM_CA_NUM)];
            }

            const size_t tx_zero_postfix_offset = 2
                * (cfg->ofdm_tx_zero_prefix_ + cfg->CP_LEN + cfg->OFDM_CA_NUM);
            std::memset(txSymbol + tx_zero_postfix_offset, 0,
                sizeof(short) * 2 * cfg->ofdm_tx_zero_postfix_);
        }
    }

    std::string filename_dl_tx = cur_directory + "/data/LDPC_dl_tx_data_"
        + std::to_string(cfg->OFDM_CA_NUM) + "_ant"
        + std::to_string(cfg->BS_ANT_NUM) + ".bin";
    std::printf("Saving dl tx data to %s\n", filename_dl_tx.c_str());
    FILE* fp_dl_tx = std::fopen(filename_dl_tx.c_str(), "wb");
    for (size_t i = 0; i < cfg->dl_data_symbol_num_perframe; i++) {
        short* ptr = (short*)dl_tx_data[i];
        std::fwrite(ptr, cfg->sampsPerSymbol * cfg->BS_ANT_NUM * 2,
            sizeof(short), fp_dl_tx);
    }
    std::fclose(fp_dl_tx);

    // std::printf("rx data\n");
    // for (int i = 0; i < 10; i++) {

    //     for (int j = 0; j < cfg->OFDM_CA_NUM * cfg->BS_ANT_NUM; j++) {
    //         if (j % cfg->OFDM_CA_NUM == 0) {
    //             std::printf("symbol %d ant %d\n", i, j / cfg->OFDM_CA_NUM);
    //         }
    //         std::printf("%.3f+%.3fi ", dl_tx_data[i][j].re,
    //             dl_tx_data[i][j].im);
    //     }
    // }
    // std::printf("\n");

    /* Clean Up memory */
    dl_ifft_data.free();
    dl_tx_data.free();
    dl_mod_data.free();
    precoder.free();

    csi_matrices.free();
    tx_data_all_symbols.free();
    rx_data_all_symbols.free();
    ue_specific_pilot.free();
    delete cfg;

    return 0;
}
