/**
 * @file test_ldpc_baseband.cpp
 * @brief Test LDPC performance after encoding, modulation, demodulation, 
 * and decoding when different levels of 
 * Gaussian noise is added to CSI
 */

#include "comms-lib.h"
#include "config.h"
#include "data_generator.h"
#include "gettime.h"
#include "memory_manage.h"
#include "modulation.h"
#include "phy_ldpc_decoder_5gnr.h"
#include "utils_ldpc.h"
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
static constexpr float kNoiseLevels[15]
    = { 1.7783, 1.3335, 1.0000, 0.7499, 0.5623, 0.4217, 0.3162, 0.2371, 0.1778,
          0.1334, 0.1000, 0.0750, 0.0562, 0.0422, 0.0316 };
static constexpr float kSnrLevels[15]
    = { -5, -2.5, 0, 2.5, 5, 7.5, 10, 12.5, 15, 17.5, 20, 22.5, 25, 27.5, 30 };
DEFINE_string(profile, "random",
    "The profile of the input user bytes (e.g., 'random', '123')");
DEFINE_string(conf_file,
    TOSTRING(PROJECT_DIRECTORY) "/data/tddconfig-sim-ul.json",
    "Agora config filename");

float RandFloat(float min, float max)
{
    return ((float(rand()) / float(RAND_MAX)) * (max - min)) + min;
}

float RandFloatFromShort(float min, float max)
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
        ? DataGenerator::Profile::kK123
        : DataGenerator::Profile::kRandom;
    DataGenerator data_generator(cfg, 0 /* RNG seed */, profile);

    std::printf("DataGenerator: Config file: %s, data profile = %s\n",
        FLAGS_conf_file.c_str(),
        profile == DataGenerator::Profile::kK123 ? "123" : "random");

    std::printf("DataGenerator: Using %s-orthogonal pilots\n",
        cfg->freq_orthogonal_pilot_ ? "frequency" : "time");

    std::printf("DataGenerator: Generating encoded and modulated data\n");
    srand(time(nullptr));

    // Step 1: Generate the information buffers and LDPC-encoded buffers for
    // uplink
    size_t num_symbols_per_cb = 1;
    size_t bits_per_symbol = cfg->ofdm_data_num_ * cfg->mod_order_bits_;
    if (cfg->ldpc_config_.cb_codew_len_ > bits_per_symbol) {
        num_symbols_per_cb
            = (cfg->ldpc_config_.cb_codew_len_ + bits_per_symbol - 1)
            / bits_per_symbol;
    }
    size_t num_cbs_per_ue = cfg->data_symbol_num_perframe_ / num_symbols_per_cb;
    std::printf("Number of symbols per block: %zu, blocks per frame: %zu\n",
        num_symbols_per_cb, num_cbs_per_ue);

    const size_t num_codeblocks = num_cbs_per_ue * cfg->ue_ant_num_;
    std::printf("Total number of blocks: %zu\n", num_codeblocks);
    for (size_t noise_id = 0; noise_id < 15; noise_id++) {

        std::vector<std::vector<int8_t>> information(num_codeblocks);
        std::vector<std::vector<int8_t>> encoded_codewords(num_codeblocks);
        for (size_t i = 0; i < num_codeblocks; i++) {
            data_generator.GenCodeblock(information[i], encoded_codewords[i],
                i % cfg->ue_num_ /* UE ID */);
        }

        // Save uplink information bytes to file
        const size_t input_bytes_per_cb = BitsToBytes(
            LdpcNumInputBits(cfg->ldpc_config_.bg_, cfg->ldpc_config_.zc_));
        if (kPrintUplinkInformationBytes) {
            std::printf("Uplink information bytes\n");
            for (size_t n = 0; n < num_codeblocks; n++) {
                std::printf("Symbol %zu, UE %zu\n", n / cfg->ue_ant_num_,
                    n % cfg->ue_ant_num_);
                for (size_t i = 0; i < input_bytes_per_cb; i++) {
                    std::printf("%u ", (uint8_t)information[n][i]);
                }
                std::printf("\n");
            }
        }

        Table<complex_float> modulated_codewords;
        modulated_codewords.Calloc(num_codeblocks, cfg->ofdm_data_num_,
            Agora_memory::Alignment_t::kK64Align);
        Table<int8_t> demod_data_all_symbols;
        demod_data_all_symbols.Calloc(num_codeblocks, cfg->ofdm_data_num_ * 8,
            Agora_memory::Alignment_t::kK64Align);
        std::vector<uint8_t> mod_input(cfg->ofdm_data_num_);

        // Modulate, add noise, and demodulate the encoded codewords
        for (size_t i = 0; i < num_codeblocks; i++) {

            AdaptBitsForMod(
                reinterpret_cast<const uint8_t*>(&encoded_codewords[i][0]),
                &mod_input[0], cfg->ldpc_config_.NumEncodedBytes(),
                cfg->mod_order_bits_);

            for (size_t j = 0; j < cfg->ofdm_data_num_; j++) {
                modulated_codewords[i][j]
                    = ModSingleUint8(mod_input[j], cfg->mod_table_);
            }

            for (size_t j = 0; j < cfg->ofdm_data_num_; j++) {
                complex_float noise
                    = { static_cast<float>(distribution(generator))
                              * kNoiseLevels[noise_id],
                          static_cast<float>(distribution(generator))
                              * kNoiseLevels[noise_id] };
                modulated_codewords[i][j].re
                    = modulated_codewords[i][j].re + noise.re;
                modulated_codewords[i][j].im
                    = modulated_codewords[i][j].im + noise.im;
            }

            switch (cfg->mod_order_bits_) {
            case (4):
                Demod16qamSoftAvx2((float*)modulated_codewords[i],
                    demod_data_all_symbols[i], cfg->ofdm_data_num_);
                break;
            case (6):
                Demod64qamSoftAvx2((float*)modulated_codewords[i],
                    demod_data_all_symbols[i], cfg->ofdm_data_num_);
                break;
            default:
                std::printf("Demodulation: modulation type %s not supported!\n",
                    cfg->modulation_.c_str());
            }
        }

        LDPCconfig ldpc_config = cfg->ldpc_config_;

        struct bblib_ldpc_decoder_5gnr_request ldpc_decoder_5gnr_request {
        };
        struct bblib_ldpc_decoder_5gnr_response ldpc_decoder_5gnr_response {
        };

        // Decoder setup
        ldpc_decoder_5gnr_request.numChannelLlrs = ldpc_config.cb_codew_len_;
        ldpc_decoder_5gnr_request.numFillerBits = 0;
        ldpc_decoder_5gnr_request.maxIterations = ldpc_config.decoder_iter_;
        ldpc_decoder_5gnr_request.enableEarlyTermination
            = ldpc_config.early_termination_;
        ldpc_decoder_5gnr_request.Zc = ldpc_config.zc_;
        ldpc_decoder_5gnr_request.baseGraph = ldpc_config.bg_;
        ldpc_decoder_5gnr_request.nRows = ldpc_config.n_rows_;
        ldpc_decoder_5gnr_response.numMsgBits = ldpc_config.cb_len_;
        auto* resp_var_nodes
            = static_cast<int16_t*>(Agora_memory::PaddedAlignedAlloc(
                Agora_memory::Alignment_t::kK64Align,
                1024 * 1024 * sizeof(int16_t)));
        ldpc_decoder_5gnr_response.varNodes = resp_var_nodes;

        Table<uint8_t> decoded_codewords;
        decoded_codewords.Calloc(num_codeblocks, cfg->ofdm_data_num_,
            Agora_memory::Alignment_t::kK64Align);

        double freq_ghz = MeasureRdtscFreq();
        size_t start_tsc = WorkerRdtsc();
        for (size_t i = 0; i < num_codeblocks; i++) {
            ldpc_decoder_5gnr_request.varNodes = demod_data_all_symbols[i];
            ldpc_decoder_5gnr_response.compactedMessageBytes
                = decoded_codewords[i];
            bblib_ldpc_decoder_5gnr(
                &ldpc_decoder_5gnr_request, &ldpc_decoder_5gnr_response);
        }
        size_t duration = WorkerRdtsc() - start_tsc;
        std::printf("Decoding of %zu blocks takes %.2f us per block\n",
            num_codeblocks, CyclesToUs(duration, freq_ghz) / num_codeblocks);

        // Correctness check
        size_t error_num = 0;
        size_t total = num_codeblocks * ldpc_config.cb_len_;
        size_t block_error_num = 0;

        for (size_t i = 0; i < num_codeblocks; i++) {
            size_t error_in_block = 0;
            for (size_t j = 0; j < ldpc_config.cb_len_ / 8; j++) {
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
            kNoiseLevels[noise_id], kSnrLevels[noise_id], error_num, total,
            1.f * error_num / total, block_error_num, num_codeblocks,
            1.f * block_error_num / num_codeblocks);

        modulated_codewords.Free();
        demod_data_all_symbols.Free();
        decoded_codewords.Free();
        std::free(resp_var_nodes);
    }

    delete cfg;

    return 0;
}
