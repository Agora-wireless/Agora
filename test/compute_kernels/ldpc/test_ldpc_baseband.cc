/**
 * @file test_ldpc_baseband.cc
 * @brief Test LDPC performance in baseband procesing when different levels of
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

    // Modulate the encoded codewords
    std::vector<std::vector<complex_float>> modulated_codewords(
        cfg->UeAntNum() * cfg->Frame().NumDataSyms());
    size_t num_used_symbol = num_cbs_per_ue * num_symbols_per_cb;
    size_t num_unused_symbol = cfg->Frame().NumDataSyms() - num_used_symbol;
    for (size_t ue_id = 0; ue_id < cfg->UeAntNum(); ue_id++) {
      for (size_t i = 0; i < num_cbs_per_ue; i++) {
        size_t remaining_bits = cfg->LdpcConfig(dir).NumCbCodewLen();
        size_t offset = 0;
        for (size_t j = 0; j < num_symbols_per_cb; j++) {
          size_t num_bits =
              ((j + 1) < num_symbols_per_cb) ? bits_per_symbol : remaining_bits;
          modulated_codewords[ue_id * cfg->Frame().NumDataSyms() +
                              i * num_symbols_per_cb + j] =
              data_generator.GetModulation(
                  &encoded_codewords[ue_id * num_cbs_per_ue + i][offset],
                  num_bits);
          remaining_bits -= bits_per_symbol;
          offset += BitsToBytes(bits_per_symbol);
        }
      }
      for (size_t i = 0; i < num_unused_symbol; i++) {
        modulated_codewords[ue_id * cfg->Frame().NumDataSyms() +
                            num_used_symbol + i]
            .resize(cfg->OfdmDataNum());
      }
    }

    // Place modulated uplink data codewords into central IFFT bins
    std::vector<std::vector<complex_float>> pre_ifft_data_syms(
        cfg->UeAntNum() * cfg->Frame().NumDataSyms());
    for (size_t i = 0; i < pre_ifft_data_syms.size(); i++) {
      pre_ifft_data_syms[i] = data_generator.BinForIfft(modulated_codewords[i]);
    }

    std::vector<complex_float> pilot_td =
        data_generator.GetCommonPilotTimeDomain();

    // Put pilot and data symbols together
    Table<complex_float> tx_data_all_symbols;
    tx_data_all_symbols.Calloc(cfg->Frame().NumTotalSyms(),
                               cfg->UeAntNum() * cfg->OfdmCaNum(),
                               Agora_memory::Alignment_t::kAlign64);

    if (cfg->FreqOrthogonalPilot() == true) {
      for (size_t i = 0; i < cfg->UeAntNum(); i++) {
        std::vector<complex_float> pilots_t_ue(cfg->OfdmCaNum());  // Zeroed
        for (size_t j = cfg->OfdmDataStart();
             j < cfg->OfdmDataStart() + cfg->OfdmDataNum();
             j += cfg->UeAntNum()) {
          pilots_t_ue[i + j] = pilot_td[i + j];
        }
        // Load pilot
        std::memcpy(tx_data_all_symbols[cfg->Frame().NumBeaconSyms()] +
                        i * cfg->OfdmCaNum(),
                    &pilots_t_ue[0], cfg->OfdmCaNum() * sizeof(complex_float));
      }
    } else {
      for (size_t i = 0; i < cfg->UeAntNum(); i++) {
        std::memcpy(tx_data_all_symbols[i + cfg->Frame().NumBeaconSyms()] +
                        i * cfg->OfdmCaNum(),
                    &pilot_td[0], cfg->OfdmCaNum() * sizeof(complex_float));
      }
    }

    size_t data_sym_start =
        cfg->Frame().NumPilotSyms() + cfg->Frame().NumBeaconSyms();
    for (size_t i = data_sym_start; i < cfg->Frame().NumTotalSyms(); i++) {
      const size_t data_sym_id = (i - data_sym_start);
      for (size_t j = 0; j < cfg->UeAntNum(); j++) {
        std::memcpy(tx_data_all_symbols[i] + j * cfg->OfdmCaNum(),
                    &pre_ifft_data_syms[j * cfg->Frame().NumDataSyms() +
                                        data_sym_id][0],
                    cfg->OfdmCaNum() * sizeof(complex_float));
      }
    }

    // Generate CSI matrix without noise
    Table<complex_float> csi_matrices_no_noise;
    csi_matrices_no_noise.Calloc(cfg->OfdmCaNum(),
                                 cfg->UeAntNum() * cfg->BsAntNum(),
                                 Agora_memory::Alignment_t::kAlign32);
    for (size_t i = 0; i < cfg->UeAntNum() * cfg->BsAntNum(); i++) {
      complex_float csi = {static_cast<float>(distribution(generator)),
                           static_cast<float>(distribution(generator))};
      for (size_t j = 0; j < cfg->OfdmCaNum(); j++) {
        csi_matrices_no_noise[j][i].re = csi.re;
        csi_matrices_no_noise[j][i].im = csi.im;
      }
    }

    // Generate CSI matrix with noise for pilot symbols
    Table<complex_float> csi_matrices_pilot;
    csi_matrices_pilot.Calloc(cfg->OfdmCaNum(),
                              cfg->UeAntNum() * cfg->BsAntNum(),
                              Agora_memory::Alignment_t::kAlign32);
    for (size_t i = 0; i < cfg->UeAntNum() * cfg->BsAntNum(); i++) {
      for (size_t j = 0; j < cfg->OfdmCaNum(); j++) {
        complex_float noise = {static_cast<float>(distribution(generator)) *
                                   kNoiseLevels[noise_id],
                               static_cast<float>(distribution(generator)) *
                                   kNoiseLevels[noise_id]};
        csi_matrices_pilot[j][i].re = csi_matrices_no_noise[j][i].re + noise.re;
        csi_matrices_pilot[j][i].im = csi_matrices_no_noise[j][i].im + noise.im;
      }
    }

    // Generate CSI matrix with noise for data symbols
    Table<complex_float> csi_matrices_data;
    csi_matrices_data.Calloc(cfg->OfdmCaNum(),
                             cfg->UeAntNum() * cfg->BsAntNum(),
                             Agora_memory::Alignment_t::kAlign32);
    for (size_t i = 0; i < cfg->UeAntNum() * cfg->BsAntNum(); i++) {
      for (size_t j = 0; j < cfg->OfdmCaNum(); j++) {
        complex_float noise = {static_cast<float>(distribution(generator)) *
                                   kNoiseLevels[noise_id],
                               static_cast<float>(distribution(generator)) *
                                   kNoiseLevels[noise_id]};
        csi_matrices_data[j][i].re = csi_matrices_no_noise[j][i].re + noise.re;
        csi_matrices_data[j][i].im = csi_matrices_no_noise[j][i].im + noise.im;
      }
    }

    // Generate RX data received by base station after going through channels
    Table<complex_float> rx_data_all_symbols;
    rx_data_all_symbols.Calloc(cfg->Frame().NumTotalSyms(),
                               cfg->OfdmCaNum() * cfg->BsAntNum(),
                               Agora_memory::Alignment_t::kAlign64);
    for (size_t i = 0; i < cfg->Frame().NumTotalSyms(); i++) {
      arma::cx_fmat mat_input_data(
          reinterpret_cast<arma::cx_float*>(tx_data_all_symbols[i]),
          cfg->OfdmCaNum(), cfg->UeAntNum(), false);
      arma::cx_fmat mat_output(
          reinterpret_cast<arma::cx_float*>(rx_data_all_symbols[i]),
          cfg->OfdmCaNum(), cfg->BsAntNum(), false);

      for (size_t j = 0; j < cfg->OfdmCaNum(); j++) {
        arma::cx_fmat mat_csi(
            reinterpret_cast<arma::cx_float*>(csi_matrices_data[j]),
            cfg->BsAntNum(), cfg->UeAntNum(), false);
        mat_output.row(j) = mat_input_data.row(j) * mat_csi.st();
      }
    }

    // Compute precoder
    Table<complex_float> precoder;
    precoder.Calloc(cfg->OfdmCaNum(), cfg->UeAntNum() * cfg->BsAntNum(),
                    Agora_memory::Alignment_t::kAlign32);
    for (size_t i = 0; i < cfg->OfdmCaNum(); i++) {
      arma::cx_fmat mat_input(
          reinterpret_cast<arma::cx_float*>(csi_matrices_pilot[i]),
          cfg->BsAntNum(), cfg->UeAntNum(), false);
      arma::cx_fmat mat_output(reinterpret_cast<arma::cx_float*>(precoder[i]),
                               cfg->UeAntNum(), cfg->BsAntNum(), false);
      pinv(mat_output, mat_input, 1e-2, "dc");
    }

    Table<complex_float> equalized_data_all_symbols;
    equalized_data_all_symbols.Calloc(cfg->Frame().NumTotalSyms(),
                                      cfg->OfdmDataNum() * cfg->UeAntNum(),
                                      Agora_memory::Alignment_t::kAlign64);
    Table<int8_t> demod_data_all_symbols;
    demod_data_all_symbols.Calloc(
        cfg->UeAntNum(), cfg->OfdmDataNum() * cfg->Frame().NumDataSyms() * 8,
        Agora_memory::Alignment_t::kAlign64);
    for (size_t i = data_sym_start; i < cfg->Frame().NumTotalSyms(); i++) {
      arma::cx_fmat mat_rx_data(
          reinterpret_cast<arma::cx_float*>(rx_data_all_symbols[i]),
          cfg->OfdmCaNum(), cfg->BsAntNum(), false);
      arma::cx_fmat mat_equalized_data(
          reinterpret_cast<arma::cx_float*>(
              equalized_data_all_symbols[i - data_sym_start]),
          cfg->OfdmDataNum(), cfg->UeAntNum(), false);
      for (size_t j = 0; j < cfg->OfdmDataNum(); j++) {
        arma::cx_fmat mat_precoder(
            reinterpret_cast<arma::cx_float*>(
                precoder[cfg->FreqOrthogonalPilot() ? (j % cfg->UeAntNum())
                                                    : j]),
            cfg->UeAntNum(), cfg->BsAntNum(), false);
        mat_equalized_data.row(j) =
            (mat_precoder * mat_rx_data.row(j + cfg->OfdmDataStart()).st())
                .st();
      }

      mat_equalized_data = mat_equalized_data.st();

      for (size_t j = 0; j < cfg->UeAntNum(); j++) {
        size_t cb_id = (i - data_sym_start) / num_symbols_per_cb;
        size_t symbol_id_in_cb = (i - data_sym_start) % num_symbols_per_cb;
        auto* demod_ptr = demod_data_all_symbols[j] +
                          (cb_id * num_symbols_per_cb * 8 +
                           symbol_id_in_cb * cfg->ModOrderBits(dir)) *
                              cfg->OfdmDataNum();
        auto* equal_t_ptr =
            (float*)(equalized_data_all_symbols[i - data_sym_start] +
                     j * cfg->OfdmDataNum());
        switch (cfg->ModOrderBits(dir)) {
          case (4):
            Demod16qamSoftAvx2(equal_t_ptr, demod_ptr, cfg->OfdmDataNum());
            break;
          case (6):
            Demod64qamSoftAvx2(equal_t_ptr, demod_ptr, cfg->OfdmDataNum());
            break;
          default:
            std::printf("Demodulation: modulation type %s not supported!\n",
                        cfg->Modulation(dir).c_str());
        }
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
    for (size_t i = 0; i < cfg->UeAntNum(); i++) {
      for (size_t j = 0; j < num_cbs_per_ue; j++) {
        ldpc_decoder_5gnr_request.varNodes =
            demod_data_all_symbols[i] +
            j * cfg->OfdmDataNum() * 8 * num_symbols_per_cb;
        ldpc_decoder_5gnr_response.compactedMessageBytes =
            decoded_codewords[i * num_cbs_per_ue + j];
        bblib_ldpc_decoder_5gnr(&ldpc_decoder_5gnr_request,
                                &ldpc_decoder_5gnr_response);
      }
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
    demod_data_all_symbols.Free();
    equalized_data_all_symbols.Free();
    precoder.Free();
    tx_data_all_symbols.Free();
    rx_data_all_symbols.Free();
    csi_matrices_no_noise.Free();
    csi_matrices_pilot.Free();
    csi_matrices_data.Free();
    decoded_codewords.Free();
  }
  delete[] input_ptr;
  return 0;
}
