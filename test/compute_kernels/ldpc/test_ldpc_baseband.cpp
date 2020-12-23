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
        cfg->freq_orthogonal_pilot() ? "frequency" : "time");

    std::printf("DataGenerator: Generating encoded and modulated data\n");
    srand(time(nullptr));

    // Step 1: Generate the information buffers and LDPC-encoded buffers for
    // uplink
    size_t num_symbols_per_cb = 1;
    size_t bits_per_symbol = cfg->ofdm_data_num() * cfg->mod_order_bits();
    if (cfg->ldpc_config().num_cb_codew_len() > bits_per_symbol)
        num_symbols_per_cb = (cfg->ldpc_config().num_cb_codew_len() + bits_per_symbol - 1)
            / bits_per_symbol;
    size_t num_cbs_per_ue = cfg->frame().NumDataSyms() / num_symbols_per_cb;
    std::printf("Number of symbols per block: %zu, blocks per frame: %zu\n",
        num_symbols_per_cb, num_cbs_per_ue);

    const size_t num_codeblocks = num_cbs_per_ue * cfg->ue_ant_num();
    std::printf("Total number of blocks: %zu\n", num_codeblocks);
    for (size_t noise_id = 0; noise_id < 15; noise_id++) {

        std::vector<std::vector<int8_t>> information(num_codeblocks);
        std::vector<std::vector<int8_t>> encoded_codewords(num_codeblocks);
        for (size_t i = 0; i < num_codeblocks; i++) {
            data_generator.gen_codeblock_ul(information[i],
                encoded_codewords[i], i % cfg->ue_num() /* UE ID */);
        }

        // Save uplink information bytes to file
        const size_t input_bytes_per_cb = bits_to_bytes(
            ldpc_num_input_bits(cfg->ldpc_config().base_graph(), cfg->ldpc_config().expansion_factor()));
        if (kPrintUplinkInformationBytes) {
            std::printf("Uplink information bytes\n");
            for (size_t n = 0; n < num_codeblocks; n++) {
                std::printf("Symbol %zu, UE %zu\n", n / cfg->ue_ant_num(),
                    n % cfg->ue_ant_num());
                for (size_t i = 0; i < input_bytes_per_cb; i++) {
                    std::printf("%u ", (uint8_t)information[n][i]);
                }
                std::printf("\n");
            }
        }

        // Modulate the encoded codewords
        std::vector<std::vector<complex_float>> modulated_codewords(
            cfg->ue_ant_num() * cfg->frame().NumDataSyms());
        size_t num_used_symbol = num_cbs_per_ue * num_symbols_per_cb;
        size_t num_unused_symbol
            = cfg->frame().NumDataSyms() - num_used_symbol;
        for (size_t ue_id = 0; ue_id < cfg->ue_ant_num(); ue_id++) {
            for (size_t i = 0; i < num_cbs_per_ue; i++) {
                size_t remaining_bits = cfg->ldpc_config().num_cb_codew_len();
                size_t offset = 0;
                for (size_t j = 0; j < num_symbols_per_cb; j++) {

                    size_t num_bits = ((j + 1) < num_symbols_per_cb)
                        ? bits_per_symbol
                        : remaining_bits;
                    modulated_codewords[ue_id * cfg->frame().NumDataSyms()
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
                modulated_codewords[ue_id * cfg->frame().NumDataSyms()
                    + num_used_symbol + i]
                    .resize(cfg->ofdm_data_num());
            }
        }

        // Place modulated uplink data codewords into central IFFT bins
        std::vector<std::vector<complex_float>> pre_ifft_data_syms(
            cfg->ue_ant_num() * cfg->frame().NumDataSyms());
        for (size_t i = 0; i < pre_ifft_data_syms.size(); i++) {
            pre_ifft_data_syms[i]
                = data_generator.bin_for_ifft(modulated_codewords[i]);
        }

        std::vector<complex_float> pilot_td
            = data_generator.get_common_pilot_time_domain();

        // Put pilot and data symbols together
        Table<complex_float> tx_data_all_symbols;
        tx_data_all_symbols.calloc(
            cfg->frame().NumTotalSyms(), cfg->ue_ant_num() * cfg->ofdm_ca_num(), Agora_memory::Alignment_t::k64Align);

        if (cfg->freq_orthogonal_pilot() == true) {
            for (size_t i = 0; i < cfg->ue_ant_num(); i++) {
                std::vector<complex_float> pilots_t_ue(
                    cfg->ofdm_ca_num()); // Zeroed
                for (size_t j = cfg->ofdm_data_start();
                     j < cfg->ofdm_data_start() + cfg->ofdm_data_num();
                     j += cfg->ue_ant_num()) {
                    pilots_t_ue[i + j] = pilot_td[i + j];
                }
                // Load pilot to the second symbol
                // The first symbol is reserved for beacon
                std::memcpy(tx_data_all_symbols[cfg->frame().NumBeaconSyms()]
                        + i * cfg->ofdm_ca_num(),
                    &pilots_t_ue[0], cfg->ofdm_ca_num() * sizeof(complex_float));
            }
        } else {
            for (size_t i = 0; i < cfg->ue_ant_num(); i++)
                std::memcpy(tx_data_all_symbols[i + cfg->frame().NumBeaconSyms()]
                        + i * cfg->ofdm_ca_num(),
                    &pilot_td[0], cfg->ofdm_ca_num() * sizeof(complex_float));
        }

        size_t data_sym_start
            = cfg->frame().NumPilotSyms() + cfg->frame().NumBeaconSyms();
        for (size_t i = data_sym_start; i < cfg->frame().NumTotalSyms(); i++) {
            const size_t data_sym_id = (i - data_sym_start);
            for (size_t j = 0; j < cfg->ue_ant_num(); j++) {
                std::memcpy(tx_data_all_symbols[i] + j * cfg->ofdm_ca_num(),
                    &pre_ifft_data_syms[j * cfg->frame().NumDataSyms()
                        + data_sym_id][0],
                    cfg->ofdm_ca_num() * sizeof(complex_float));
            }
        }

        // Generate CSI matrix without noise
        Table<complex_float> csi_matrices_no_noise;
        csi_matrices_no_noise.calloc(
            cfg->ofdm_ca_num(), cfg->ue_ant_num() * cfg->bs_ant_num(), Agora_memory::Alignment_t::k32Align);
        for (size_t i = 0; i < cfg->ue_ant_num() * cfg->bs_ant_num(); i++) {
            complex_float csi = { static_cast<float>(distribution(generator)),
                static_cast<float>(distribution(generator)) };
            for (size_t j = 0; j < cfg->ofdm_ca_num(); j++) {
                csi_matrices_no_noise[j][i].re = csi.re;
                csi_matrices_no_noise[j][i].im = csi.im;
            }
        }

        // Generate CSI matrix with noise for pilot symbols
        Table<complex_float> csi_matrices_pilot;
        csi_matrices_pilot.calloc(
            cfg->ofdm_ca_num(), cfg->ue_ant_num() * cfg->bs_ant_num(), Agora_memory::Alignment_t::k32Align);
        for (size_t i = 0; i < cfg->ue_ant_num() * cfg->bs_ant_num(); i++) {
            for (size_t j = 0; j < cfg->ofdm_ca_num(); j++) {
                complex_float noise
                    = { static_cast<float>(distribution(generator))
                              * noise_levels[noise_id],
                          static_cast<float>(distribution(generator))
                              * noise_levels[noise_id] };
                csi_matrices_pilot[j][i].re
                    = csi_matrices_no_noise[j][i].re + noise.re;
                csi_matrices_pilot[j][i].im
                    = csi_matrices_no_noise[j][i].im + noise.im;
            }
        }

        // Generate CSI matrix with noise for data symbols
        Table<complex_float> csi_matrices_data;
        csi_matrices_data.calloc(
            cfg->ofdm_ca_num(), cfg->ue_ant_num() * cfg->bs_ant_num(), Agora_memory::Alignment_t::k32Align);
        for (size_t i = 0; i < cfg->ue_ant_num() * cfg->bs_ant_num(); i++) {
            for (size_t j = 0; j < cfg->ofdm_ca_num(); j++) {
                complex_float noise
                    = { static_cast<float>(distribution(generator))
                              * noise_levels[noise_id],
                          static_cast<float>(distribution(generator))
                              * noise_levels[noise_id] };
                csi_matrices_data[j][i].re
                    = csi_matrices_no_noise[j][i].re + noise.re;
                csi_matrices_data[j][i].im
                    = csi_matrices_no_noise[j][i].im + noise.im;
            }
        }

        // Generate RX data received by base station after going through channels
        Table<complex_float> rx_data_all_symbols;
        rx_data_all_symbols.calloc(
            cfg->frame().NumTotalSyms(), cfg->ofdm_ca_num() * cfg->bs_ant_num(), Agora_memory::Alignment_t::k64Align);
        for (size_t i = 0; i < cfg->frame().NumTotalSyms(); i++) {
            arma::cx_fmat mat_input_data(
                reinterpret_cast<arma::cx_float*>(tx_data_all_symbols[i]),
                cfg->ofdm_ca_num(), cfg->ue_ant_num(), false);
            arma::cx_fmat mat_output(
                reinterpret_cast<arma::cx_float*>(rx_data_all_symbols[i]),
                cfg->ofdm_ca_num(), cfg->bs_ant_num(), false);

            for (size_t j = 0; j < cfg->ofdm_ca_num(); j++) {
                arma::cx_fmat mat_csi(
                    reinterpret_cast<arma::cx_float*>(csi_matrices_data[j]),
                    cfg->bs_ant_num(), cfg->ue_ant_num());
                mat_output.row(j) = mat_input_data.row(j) * mat_csi.st();
            }
        }

        // Compute precoder
        Table<complex_float> precoder;
        precoder.calloc(
            cfg->ofdm_ca_num(), cfg->ue_ant_num() * cfg->bs_ant_num(), Agora_memory::Alignment_t::k32Align);
        for (size_t i = 0; i < cfg->ofdm_ca_num(); i++) {
            arma::cx_fmat mat_input(
                reinterpret_cast<arma::cx_float*>(csi_matrices_pilot[i]),
                cfg->bs_ant_num(), cfg->ue_ant_num(), false);
            arma::cx_fmat mat_output(
                reinterpret_cast<arma::cx_float*>(precoder[i]), cfg->ue_ant_num(),
                cfg->bs_ant_num(), false);
            pinv(mat_output, mat_input, 1e-2, "dc");
        }

        Table<complex_float> equalized_data_all_symbols;
        equalized_data_all_symbols.calloc(
            cfg->frame().NumTotalSyms(), cfg->ofdm_data_num() * cfg->ue_ant_num(), Agora_memory::Alignment_t::k64Align);
        Table<int8_t> demod_data_all_symbols;
        demod_data_all_symbols.calloc(cfg->ue_ant_num(),
            cfg->ofdm_data_num() * cfg->frame().NumDataSyms() * 8, Agora_memory::Alignment_t::k64Align);
        for (size_t i = data_sym_start; i < cfg->frame().NumTotalSyms(); i++) {
            arma::cx_fmat mat_rx_data(
                reinterpret_cast<arma::cx_float*>(rx_data_all_symbols[i]),
                cfg->ofdm_ca_num(), cfg->bs_ant_num(), false);
            arma::cx_fmat mat_equalized_data(
                reinterpret_cast<arma::cx_float*>(
                    equalized_data_all_symbols[i - data_sym_start]),
                cfg->ofdm_data_num(), cfg->ue_ant_num(), false);
            for (size_t j = 0; j < cfg->ofdm_data_num(); j++) {
                arma::cx_fmat mat_precoder(
                    reinterpret_cast<arma::cx_float*>(
                        precoder[cfg->freq_orthogonal_pilot()
                                ? (j % cfg->ue_ant_num())
                                : j]),
                    cfg->ue_ant_num(), cfg->bs_ant_num(), false);
                mat_equalized_data.row(j) = (mat_precoder
                    * mat_rx_data.row(j + cfg->ofdm_data_start()).st())
                                                .st();
            }

            mat_equalized_data = mat_equalized_data.st();

            for (size_t j = 0; j < cfg->ue_ant_num(); j++) {
                size_t cb_id = (i - data_sym_start) / num_symbols_per_cb;
                size_t symbol_id_in_cb
                    = (i - data_sym_start) % num_symbols_per_cb;
                auto* demod_ptr = demod_data_all_symbols[j]
                    + (cb_id * num_symbols_per_cb * 8
                          + symbol_id_in_cb * cfg->mod_order_bits())
                        * cfg->ofdm_data_num();
                auto* equal_T_ptr
                    = (float*)(equalized_data_all_symbols[i - data_sym_start]
                        + j * cfg->ofdm_data_num());
                switch (cfg->mod_order_bits() == true) {
                case (4):
                    demod_16qam_soft_avx2(
                        equal_T_ptr, demod_ptr, cfg->ofdm_data_num());
                    break;
                case (6):
                    demod_64qam_soft_avx2(
                        equal_T_ptr, demod_ptr, cfg->ofdm_data_num());
                    break;
                default:
                    std::printf(
                        "Demodulation: modulation type %s not supported!\n",
                        cfg->modulation().c_str());
                }
            }
        }

        const LDPCconfig& ldpc_config = cfg->ldpc_config();

        struct bblib_ldpc_decoder_5gnr_request ldpc_decoder_5gnr_request {
        };
        struct bblib_ldpc_decoder_5gnr_response ldpc_decoder_5gnr_response {
        };

        // Decoder setup
        ldpc_decoder_5gnr_request.numChannelLlrs = ldpc_config.num_cb_codew_len();
        ldpc_decoder_5gnr_request.numFillerBits = 0;
        ldpc_decoder_5gnr_request.maxIterations = ldpc_config.max_decoder_iter();
        ldpc_decoder_5gnr_request.enableEarlyTermination
            = ldpc_config.early_termination();
        ldpc_decoder_5gnr_request.Zc = ldpc_config.expansion_factor();
        ldpc_decoder_5gnr_request.baseGraph = ldpc_config.base_graph();
        ldpc_decoder_5gnr_request.nRows = ldpc_config.num_rows();
        ldpc_decoder_5gnr_response.numMsgBits = ldpc_config.num_cb_len();
        auto* resp_var_nodes
            = static_cast<int16_t*>(Agora_memory::padded_aligned_alloc(Agora_memory::Alignment_t::k64Align, 1024 * 1024 * sizeof(int16_t)));
        ldpc_decoder_5gnr_response.varNodes = resp_var_nodes;

        Table<uint8_t> decoded_codewords;
        decoded_codewords.calloc(num_codeblocks, cfg->ofdm_data_num(), Agora_memory::Alignment_t::k64Align);
        double freq_ghz = measure_rdtsc_freq();
        size_t start_tsc = worker_rdtsc();
        for (size_t i = 0; i < cfg->ue_ant_num(); i++) {
            for (size_t j = 0; j < num_cbs_per_ue; j++) {
                ldpc_decoder_5gnr_request.varNodes = demod_data_all_symbols[i]
                    + j * cfg->ofdm_data_num() * 8 * num_symbols_per_cb;
                ldpc_decoder_5gnr_response.compactedMessageBytes
                    = decoded_codewords[i * num_cbs_per_ue + j];
                bblib_ldpc_decoder_5gnr(
                    &ldpc_decoder_5gnr_request, &ldpc_decoder_5gnr_response);
            }
        }
        size_t duration = worker_rdtsc() - start_tsc;
        std::printf("Decoding of %zu blocks takes %.2f us per block\n",
            num_codeblocks, cycles_to_us(duration, freq_ghz) / num_codeblocks);

        // Correctness check
        size_t error_num = 0;
        size_t total = num_codeblocks * ldpc_config.num_cb_len();
        size_t block_error_num = 0;

        for (size_t i = 0; i < num_codeblocks; i++) {
            size_t error_in_block = 0;
            for (size_t j = 0; j < ldpc_config.num_cb_len() / 8; j++) {
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
