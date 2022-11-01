/**
 * @file test_ldpc_baseband.cc
 * @brief Test LDPC performance after encoding, modulation, demodulation,
 * and decoding when different levels of
 * Gaussian noise is added to CSI
 */

#include <gflags/gflags.h>
#include <immintrin.h>

#include <bitset>
#include <chrono>
#include <fstream>
#include <iostream>
#include <random>

#include "armadillo"
#include "comms-lib.h"
#include "config.h"
#include "data_generator.h"
#include "datatype_conversion.h"
#include "gettime.h"
#include "memory_manage.h"
#include "modulation.h"
#include "phy_ldpc_decoder_5gnr.h"
#include "utils_ldpc.h"

static constexpr bool kVerbose = false;
static constexpr bool kPrintUplinkInformationBytes = false;
static constexpr float kNoiseLevels[15] = {
    1.7783, 1.3335, 1.0000, 0.7499, 0.5623, 0.4217, 0.3162, 0.2371,
    0.1778, 0.1334, 0.1000, 0.0750, 0.0562, 0.0422, 0.0316};
static constexpr float kSnrLevels[15] = {-5, -2.5, 0,  2.5,  5,  7.5,  10, 12.5,
                                         15, 17.5, 20, 22.5, 25, 27.5, 30};
DEFINE_string(profile, "random",
              "The profile of the input user bytes (e.g., 'random', '123')");
DEFINE_string(
    conf_file,
    TOSTRING(PROJECT_DIRECTORY) "/files/config/ci/tddconfig-sim-ul.json",
    "Agora config filename");

float RandFloat(float min, float max) {
  return ((float(rand()) / float(RAND_MAX)) * (max - min)) + min;
}

float RandFloatFromShort(float min, float max) {
  float rand_val =
      ((static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) *
       (max - min)) +
      min;
  auto rand_val_ushort = static_cast<short>(rand_val * kShrtFltConvFactor);
  rand_val = (float)rand_val_ushort / kShrtFltConvFactor;
  return rand_val;
}

int main(int argc, char* argv[]) {
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::normal_distribution<double> distribution(0.0, 1.0);

  const std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  auto cfg = std::make_unique<Config>(FLAGS_conf_file.c_str());
  Direction dir =
      cfg->Frame().NumULSyms() > 0 ? Direction::kUplink : Direction::kDownlink;

  const DataGenerator::Profile profile =
      FLAGS_profile == "123" ? DataGenerator::Profile::kProfile123
                             : DataGenerator::Profile::kRandom;
  DataGenerator data_generator(cfg.get(), 0 /* RNG seed */, profile);

  std::printf(
      "DataGenerator: Config file: %s, data profile = %s\n",
      FLAGS_conf_file.c_str(),
      profile == DataGenerator::Profile::kProfile123 ? "123" : "random");

  std::printf("DataGenerator: Using %s-orthogonal pilots\n",
              cfg->FreqOrthogonalPilot() ? "frequency" : "time");

  std::printf("DataGenerator: Generating encoded and modulated data\n");
  srand(time(nullptr));

  // Step 1: Generate the information buffers and LDPC-encoded buffers for
  // uplink
  size_t num_symbols_per_cb = 1;
  size_t bits_per_symbol = cfg->OfdmDataNum() * cfg->ModOrderBits(dir);
  if (cfg->LdpcConfig(dir).NumCbCodewLen() > bits_per_symbol) {
    num_symbols_per_cb =
        (cfg->LdpcConfig(dir).NumCbCodewLen() + bits_per_symbol - 1) /
        bits_per_symbol;
  }
  size_t num_cbs_per_ue = cfg->Frame().NumDataSyms() / num_symbols_per_cb;
  std::printf("Number of symbols per block: %zu, blocks per frame: %zu\n",
              num_symbols_per_cb, num_cbs_per_ue);

  const size_t num_codeblocks = num_cbs_per_ue * cfg->UeAntNum();
  std::printf("Total number of blocks: %zu\n", num_codeblocks);
  size_t input_size = Roundup<64>(
      LdpcEncodingInputBufSize(cfg->LdpcConfig(dir).BaseGraph(),
                               cfg->LdpcConfig(dir).ExpansionFactor()));
  auto* input_ptr = new int8_t[input_size];
  for (size_t noise_id = 0; noise_id < 15; noise_id++) {
    std::vector<std::vector<int8_t>> information(num_codeblocks);
    std::vector<std::vector<int8_t>> encoded_codewords(num_codeblocks);
    for (size_t i = 0; i < num_codeblocks; i++) {
      data_generator.GenRawData(dir, information.at(i),
                                i % cfg->UeAntNum() /* UE ID */);
      std::memcpy(input_ptr, information.at(i).data(), input_size);
      data_generator.GenCodeblock(dir, input_ptr, encoded_codewords.at(i));
    }

    // Save uplink information bytes to file
    const size_t input_bytes_per_cb =
        BitsToBytes(LdpcNumInputBits(cfg->LdpcConfig(dir).BaseGraph(),
                                     cfg->LdpcConfig(dir).ExpansionFactor()));
    if (kPrintUplinkInformationBytes) {
      std::printf("Uplink information bytes\n");
      for (size_t n = 0; n < num_codeblocks; n++) {
        std::printf("Symbol %zu, UE %zu\n", n / cfg->UeAntNum(),
                    n % cfg->UeAntNum());
        for (size_t i = 0; i < input_bytes_per_cb; i++) {
          std::printf("%u ", (uint8_t)information[n][i]);
        }
        std::printf("\n");
      }
    }

    Table<complex_float> modulated_codewords;
    modulated_codewords.Calloc(num_codeblocks, cfg->OfdmDataNum(),
                               Agora_memory::Alignment_t::kAlign64);
    Table<int8_t> demod_data_all_symbols;
    demod_data_all_symbols.Calloc(num_codeblocks, cfg->OfdmDataNum() * 8,
                                  Agora_memory::Alignment_t::kAlign64);
    std::vector<uint8_t> mod_input(cfg->OfdmDataNum());

    // Modulate, add noise, and demodulate the encoded codewords
    for (size_t i = 0; i < num_codeblocks; i++) {
      AdaptBitsForMod(
          reinterpret_cast<const uint8_t*>(&encoded_codewords[i][0]),
          &mod_input[0], cfg->LdpcConfig(dir).NumEncodedBytes(),
          cfg->ModOrderBits(dir));

      for (size_t j = 0; j < cfg->OfdmDataNum(); j++) {
        modulated_codewords[i][j] =
            ModSingleUint8(mod_input[j], cfg->ModTable(dir));
      }

      for (size_t j = 0; j < cfg->OfdmDataNum(); j++) {
        complex_float noise = {static_cast<float>(distribution(generator)) *
                                   kNoiseLevels[noise_id],
                               static_cast<float>(distribution(generator)) *
                                   kNoiseLevels[noise_id]};
        modulated_codewords[i][j].re = modulated_codewords[i][j].re + noise.re;
        modulated_codewords[i][j].im = modulated_codewords[i][j].im + noise.im;
      }

      switch (cfg->ModOrderBits(dir)) {
        case (4):
          Demod16qamSoftAvx2((float*)modulated_codewords[i],
                             demod_data_all_symbols[i], cfg->OfdmDataNum());
          break;
        case (6):
          Demod64qamSoftAvx2((float*)modulated_codewords[i],
                             demod_data_all_symbols[i], cfg->OfdmDataNum());
          break;
        default:
          std::printf("Demodulation: modulation type %s not supported!\n",
                      cfg->Modulation(dir).c_str());
      }
    }

    const LDPCconfig& ldpc_config = cfg->LdpcConfig(dir);

    struct bblib_ldpc_decoder_5gnr_request ldpc_decoder_5gnr_request {};
    struct bblib_ldpc_decoder_5gnr_response ldpc_decoder_5gnr_response {};

    // Decoder setup
    ldpc_decoder_5gnr_request.numChannelLlrs = ldpc_config.NumCbCodewLen();
    ldpc_decoder_5gnr_request.numFillerBits = 0;
    ldpc_decoder_5gnr_request.maxIterations = ldpc_config.MaxDecoderIter();
    ldpc_decoder_5gnr_request.enableEarlyTermination =
        ldpc_config.EarlyTermination();
    ldpc_decoder_5gnr_request.Zc = ldpc_config.ExpansionFactor();
    ldpc_decoder_5gnr_request.baseGraph = ldpc_config.BaseGraph();
    ldpc_decoder_5gnr_request.nRows = ldpc_config.NumRows();
    ldpc_decoder_5gnr_response.numMsgBits = ldpc_config.NumCbLen();
    auto* resp_var_nodes = static_cast<int16_t*>(
        Agora_memory::PaddedAlignedAlloc(Agora_memory::Alignment_t::kAlign64,
                                         1024 * 1024 * sizeof(int16_t)));
    ldpc_decoder_5gnr_response.varNodes = resp_var_nodes;

    Table<uint8_t> decoded_codewords;
    decoded_codewords.Calloc(num_codeblocks, cfg->OfdmDataNum(),
                             Agora_memory::Alignment_t::kAlign64);

    double freq_ghz = GetTime::MeasureRdtscFreq();
    size_t start_tsc = GetTime::WorkerRdtsc();
    for (size_t i = 0; i < num_codeblocks; i++) {
      ldpc_decoder_5gnr_request.varNodes = demod_data_all_symbols[i];
      ldpc_decoder_5gnr_response.compactedMessageBytes = decoded_codewords[i];
      bblib_ldpc_decoder_5gnr(&ldpc_decoder_5gnr_request,
                              &ldpc_decoder_5gnr_response);
    }

    size_t duration = GetTime::WorkerRdtsc() - start_tsc;
    std::printf("Decoding of %zu blocks takes %.2f us per block\n",
                num_codeblocks,
                GetTime::CyclesToUs(duration, freq_ghz) / num_codeblocks);

    // Correctness check
    size_t error_num = 0;
    size_t total = num_codeblocks * ldpc_config.NumCbLen();
    size_t block_error_num = 0;

    for (size_t i = 0; i < num_codeblocks; i++) {
      size_t error_in_block = 0;
      for (size_t j = 0; j < ldpc_config.NumCbLen() / 8; j++) {
        auto input = static_cast<uint8_t>(information.at(i).at(j));
        uint8_t output = decoded_codewords[i][j];
        if (input != output) {
          for (size_t k = 0; k < 8; k++) {
            uint8_t mask = 1 << k;
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

    std::free(resp_var_nodes);
    modulated_codewords.Free();
    demod_data_all_symbols.Free();
    decoded_codewords.Free();
  }
  delete[] input_ptr;
  return 0;
}
