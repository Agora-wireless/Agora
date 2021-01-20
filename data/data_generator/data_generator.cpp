/**
 * @file data_generator.cpp
 * @brief Data generator to generate binary files as inputs to Agora, sender
 * and correctness tests
 */

#include "data_generator.h"

#include <gflags/gflags.h>
#include <immintrin.h>

#include <armadillo>
#include <bitset>
#include <fstream>
#include <iostream>

#include "comms-lib.h"
#include "config.hpp"
#include "memory_manage.h"
#include "modulation.hpp"
#include "utils_ldpc.hpp"

static constexpr bool kVerbose = false;
static constexpr bool kPrintUplinkInformationBytes = false;
static constexpr bool kPrintDownlinkInformationBytes = false;

DEFINE_string(profile, "random",
              "The profile of the input user bytes (e.g., 'random', '123')");
DEFINE_string(conf_file,
              TOSTRING(PROJECT_DIRECTORY) "/data/tddconfig-sim-ul.json",
              "Agora config filename");

float rand_float(float min, float max) {
  return ((float(rand()) / float(RAND_MAX)) * (max - min)) + min;
}

float rand_float_from_short(float min, float max) {
  float rand_val = ((float(rand()) / float(RAND_MAX)) * (max - min)) + min;
  short rand_val_ushort = (short)(rand_val * 32768);
  rand_val = (float)rand_val_ushort / 32768;
  return rand_val;
}

int main(int argc, char* argv[]) {
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
  const size_t ul_codeblocks = cfg->frame().NumULSyms() *
                               cfg->ldpc_config().num_blocks_in_symbol() *
                               cfg->ue_ant_num();
  std::printf("Total number of ul blocks: %zu\n", ul_codeblocks);

  std::vector<std::vector<int8_t>> ul_information(ul_codeblocks);
  std::vector<std::vector<int8_t>> ul_encoded_codewords(ul_codeblocks);
  for (size_t i = 0; i < ul_codeblocks; i++) {
    data_generator.gen_codeblock(ul_information.at(i),
                                 ul_encoded_codewords.at(i),
                                 (i % cfg->ue_num()) /* UE ID */);
  }

  {
    // Save uplink information bytes to file
    const size_t input_bytes_per_cb = bits_to_bytes(
        ldpc_num_input_bits(cfg->ldpc_config().base_graph(),
                            cfg->ldpc_config().expansion_factor()));

    const std::string filename_input =
        cur_directory + "/data/LDPC_orig_data_" +
        std::to_string(cfg->ofdm_ca_num()) + "_ant" +
        std::to_string(cfg->ue_ant_num()) + ".bin";
    std::printf("Saving raw uplink data (using LDPC) to %s\n",
                filename_input.c_str());
    FILE* fp_input = std::fopen(filename_input.c_str(), "wb");
    for (size_t i = 0; i < ul_codeblocks; i++) {
      std::fwrite(reinterpret_cast<uint8_t*>(&ul_information.at(i).at(0)),
                  input_bytes_per_cb, sizeof(uint8_t), fp_input);
    }
    // std::printf("LDPC file location %zu write size %zu\n",
    // std::ftell(fp_input), input_bytes_per_cb * sizeof(uint8_t));
    std::fclose(fp_input);

    if (kPrintUplinkInformationBytes) {
      std::printf("Uplink information bytes\n");
      for (size_t n = 0; n < ul_codeblocks; n++) {
        std::printf("Symbol %zu, UE %zu\n", n / cfg->ue_ant_num(),
                    n % cfg->ue_ant_num());
        for (size_t i = 0; i < input_bytes_per_cb; i++) {
          std::printf("%u ", static_cast<uint8_t>(ul_information.at(n).at(i)));
        }
        std::printf("\n");
      }
    }
  }

  // Modulate the encoded codewords
  std::vector<std::vector<complex_float>> ul_modulated_codewords(ul_codeblocks);
  for (size_t i = 0; i < ul_codeblocks; i++) {
    ul_modulated_codewords.at(i) =
        data_generator.get_modulation(ul_encoded_codewords.at(i));
  }

  // Place modulated uplink data codewords into central IFFT bins
  rt_assert(cfg->ldpc_config().num_blocks_in_symbol() ==
            1);  // TODO: Assumption
  std::vector<std::vector<complex_float>> pre_ifft_data_syms(
      cfg->ue_ant_num() * cfg->frame().NumULSyms());
  for (size_t i = 0; i < pre_ifft_data_syms.size(); i++) {
    pre_ifft_data_syms.at(i) =
        data_generator.bin_for_ifft(ul_modulated_codewords.at(i));
  }

  std::vector<complex_float> pilot_td =
      data_generator.get_common_pilot_time_domain();

  // Generate UE-specific pilots
  Table<complex_float> ue_specific_pilot;
  const std::vector<std::complex<float>> zc_seq = Utils::double_to_cfloat(
      CommsLib::getSequence(cfg->ofdm_data_num(), CommsLib::LTE_ZADOFF_CHU));
  const std::vector<std::complex<float>> zc_common_pilot =
      CommsLib::seqCyclicShift(zc_seq, M_PI / 4.0);  // Used in LTE SRS
  ue_specific_pilot.malloc(cfg->ue_ant_num(), cfg->ofdm_data_num(),
                           Agora_memory::Alignment_t::k64Align);
  for (size_t i = 0; i < cfg->ue_ant_num(); i++) {
    auto zc_ue_pilot_i =
        CommsLib::seqCyclicShift(zc_seq, i * M_PI / 6.0);  // LTE DMRS
    for (size_t j = 0; j < cfg->ofdm_data_num(); j++) {
      ue_specific_pilot[i][j] = {zc_ue_pilot_i[j].real(),
                                 zc_ue_pilot_i[j].imag()};
    }
  }

  // Put pilot and data symbols together
  Table<complex_float> tx_data_all_symbols;
  tx_data_all_symbols.calloc(cfg->frame().NumTotalSyms(),
                             cfg->ue_ant_num() * cfg->ofdm_ca_num(),
                             Agora_memory::Alignment_t::k64Align);

  if (cfg->freq_orthogonal_pilot() == true) {
    for (size_t i = 0; i < cfg->ue_ant_num(); i++) {
      std::vector<complex_float> pilots_t_ue(cfg->ofdm_ca_num());  // Zeroed
      for (size_t j = cfg->ofdm_data_start();
           j < cfg->ofdm_data_start() + cfg->ofdm_data_num();
           j += cfg->ue_ant_num()) {
        pilots_t_ue.at(i + j) = pilot_td.at(i + j);
      }
      // Load pilot to the second symbol
      // The first symbol is reserved for beacon
      std::memcpy(tx_data_all_symbols[cfg->frame().NumBeaconSyms()] +
                      (i * cfg->ofdm_ca_num()),
                  &pilots_t_ue.at(0),
                  (cfg->ofdm_ca_num() * sizeof(complex_float)));
    }
  } else {
    for (size_t i = 0; i < cfg->ue_ant_num(); i++)
      std::memcpy(tx_data_all_symbols[i + cfg->frame().NumBeaconSyms()] +
                      i * cfg->ofdm_ca_num(),
                  &pilot_td.at(0),
                  (cfg->ofdm_ca_num() * sizeof(complex_float)));
  }

  // Populate the UL symbols
  for (size_t i = 0; i < cfg->frame().NumULSyms(); i++) {
    const size_t data_sym_id = cfg->frame().GetULSymbol(i);
    for (size_t j = 0; j < cfg->ue_ant_num(); j++) {
      if (i < cfg->frame().client_ul_pilot_symbols()) {
        std::memcpy(tx_data_all_symbols[data_sym_id] +
                        (j * cfg->ofdm_ca_num()) + cfg->ofdm_data_start(),
                    ue_specific_pilot[j],
                    cfg->ofdm_data_num() * sizeof(complex_float));
      } else {
        std::memcpy(tx_data_all_symbols[data_sym_id] + (j * cfg->ofdm_ca_num()),
                    &pre_ifft_data_syms.at(i * cfg->ue_ant_num() + j).at(0),
                    cfg->ofdm_ca_num() * sizeof(complex_float));
      }
    }
  }

  // Generate CSI matrix
  Table<complex_float> csi_matrices;
  csi_matrices.calloc(cfg->ofdm_ca_num(), cfg->ue_ant_num() * cfg->bs_ant_num(),
                      Agora_memory::Alignment_t::k32Align);
  for (size_t i = 0; i < (cfg->ue_ant_num() * cfg->bs_ant_num()); i++) {
    complex_float csi = {rand_float_from_short(-1, 1),
                         rand_float_from_short(-1, 1)};
    // std::printf("noise of ant %d, ue %d\n", i % cfg->bs_ant_num(), i /
    // cfg->bs_ant_num() );
    for (size_t j = 0; j < cfg->ofdm_ca_num(); j++) {
      complex_float noise = {rand_float_from_short(-1, 1) * cfg->noise_level(),
                             rand_float_from_short(-1, 1) * cfg->noise_level()};
      // std::printf("%.4f+%.4fi ", noise.re, noise.im);
      csi_matrices[j][i].re = csi.re + noise.re;
      csi_matrices[j][i].im = csi.im + noise.im;
    }
    // std::printf("\n");
  }

  // Generate RX data received by base station after going through channels
  Table<complex_float> rx_data_all_symbols;
  rx_data_all_symbols.calloc(cfg->frame().NumTotalSyms(),
                             cfg->ofdm_ca_num() * cfg->bs_ant_num(),
                             Agora_memory::Alignment_t::k64Align);
  for (size_t i = 0; i < cfg->frame().NumTotalSyms(); i++) {
    arma::cx_fmat mat_input_data(
        reinterpret_cast<arma::cx_float*>(tx_data_all_symbols[i]),
        cfg->ofdm_ca_num(), cfg->ue_ant_num(), false);
    arma::cx_fmat mat_output(
        reinterpret_cast<arma::cx_float*>(rx_data_all_symbols[i]),
        cfg->ofdm_ca_num(), cfg->bs_ant_num(), false);

    for (size_t j = 0; j < cfg->ofdm_ca_num(); j++) {
      arma::cx_fmat mat_csi(reinterpret_cast<arma::cx_float*>(csi_matrices[j]),
                            cfg->bs_ant_num(), cfg->ue_ant_num());
      mat_output.row(j) = mat_input_data.row(j) * mat_csi.st();
    }
    for (size_t j = 0; j < cfg->bs_ant_num(); j++) {
      CommsLib::IFFT(rx_data_all_symbols[i] + j * cfg->ofdm_ca_num(),
                     cfg->ofdm_ca_num(), false);
    }
  }

  std::string filename_rx = cur_directory + "/data/LDPC_rx_data_" +
                            std::to_string(cfg->ofdm_ca_num()) + "_ant" +
                            std::to_string(cfg->bs_ant_num()) + ".bin";
  std::printf("Saving rx data to %s\n", filename_rx.c_str());
  FILE* fp_rx = std::fopen(filename_rx.c_str(), "wb");
  for (size_t i = 0; i < cfg->frame().NumTotalSyms(); i++) {
    auto* ptr = reinterpret_cast<float*>(rx_data_all_symbols[i]);
    std::fwrite(ptr, cfg->ofdm_ca_num() * cfg->bs_ant_num() * 2, sizeof(float),
                fp_rx);
  }
  std::fclose(fp_rx);

  // std::printf("rx data\n");
  // for (int i = 0; i < 10; i++) {
  //     for (int j = 0; j < cfg->ofdm_ca_num() * cfg->bs_ant_num(); j++) {
  //         if (j % cfg->ofdm_ca_num() == 0) {
  //             std::printf("\nsymbol %d ant %d\n", i, j / cfg->ofdm_ca_num());
  //         }
  //         std::printf("%.4f+%.4fi ", rx_data_all_symbols[i][j].re,
  //             rx_data_all_symbols[i][j].im);
  //     }
  //     std::printf("\n");
  // }

  /* ------------------------------------------------
   * Generate data for downlink test
   * ------------------------------------------------ */
  const size_t dl_codeblocks = cfg->frame().NumDLSyms() *
                               cfg->ldpc_config().num_blocks_in_symbol() *
                               cfg->ue_ant_num();
  std::printf("Total number of dl blocks: %zu\n", dl_codeblocks);

  std::vector<std::vector<int8_t>> dl_information(dl_codeblocks);
  std::vector<std::vector<int8_t>> dl_encoded_codewords(dl_codeblocks);
  for (size_t i = 0; i < dl_codeblocks; i++) {
    data_generator.gen_codeblock(dl_information.at(i),
                                 dl_encoded_codewords.at(i),
                                 (i % cfg->ue_num()) /* UE ID */);
  }

  // Modulate the encoded codewords
  std::vector<std::vector<complex_float>> dl_modulated_codewords(dl_codeblocks);
  for (size_t i = 0; i < dl_codeblocks; i++) {
    dl_modulated_codewords.at(i) =
        data_generator.get_modulation(dl_encoded_codewords[i]);
  }

  {
    // Save downlink information bytes to file
    const size_t input_bytes_per_cb = bits_to_bytes(
        ldpc_num_input_bits(cfg->ldpc_config().base_graph(),
                            cfg->ldpc_config().expansion_factor()));

    const std::string filename_input =
        cur_directory + "/data/LDPC_orig_data_" +
        std::to_string(cfg->ofdm_ca_num()) + "_ant" +
        std::to_string(cfg->ue_ant_num()) + ".bin";
    std::printf("Saving raw dl data (using LDPC) to %s\n",
                filename_input.c_str());
    FILE* fp_input = std::fopen(filename_input.c_str(), "ab");
    for (size_t i = 0; i < dl_codeblocks; i++) {
      std::fwrite(reinterpret_cast<uint8_t*>(&dl_information.at(i).at(0)),
                  input_bytes_per_cb, sizeof(uint8_t), fp_input);
    }
    std::fclose(fp_input);

    if (kPrintDownlinkInformationBytes == true) {
      std::printf("Downlink information bytes\n");
      for (size_t n = 0; n < dl_codeblocks; n++) {
        std::printf("Symbol %zu, UE %zu\n", n / cfg->ue_ant_num(),
                    n % cfg->ue_ant_num());
        for (size_t i = 0; i < input_bytes_per_cb; i++) {
          std::printf("%u ", static_cast<unsigned>(dl_information.at(n).at(i)));
        }
        std::printf("\n");
      }
    }
  }

  // Compute precoder
  Table<complex_float> precoder;
  precoder.calloc(cfg->ofdm_ca_num(), cfg->ue_ant_num() * cfg->bs_ant_num(),
                  Agora_memory::Alignment_t::k32Align);
  for (size_t i = 0; i < cfg->ofdm_ca_num(); i++) {
    arma::cx_fmat mat_input(reinterpret_cast<arma::cx_float*>(csi_matrices[i]),
                            cfg->bs_ant_num(), cfg->ue_ant_num(), false);
    arma::cx_fmat mat_output(reinterpret_cast<arma::cx_float*>(precoder[i]),
                             cfg->ue_ant_num(), cfg->bs_ant_num(), false);
    pinv(mat_output, mat_input, 1e-2, "dc");
  }

  // std::printf("CSI \n");
  // // for (int i = 0; i < cfg->ofdm_ca_num(); i++)
  // for (int j = 0; j < cfg->ue_ant_num() * cfg->bs_ant_num(); j++)
  //     std::printf("%.3f+%.3fi ",
  //         csi_matrices[cfg->ofdm_data_start()][j].re,
  //         csi_matrices[cfg->ofdm_data_start()][j].im);
  // std::printf("\n");
  // std::printf("precoder \n");
  // // for (int i = 0; i < cfg->ofdm_ca_num(); i++)
  // for (int j = 0; j < cfg->ue_ant_num() * cfg->bs_ant_num(); j++)
  //     std::printf("%.3f+%.3fi ",
  //         precoder[cfg->ofdm_data_start()][j].re,
  //         precoder[cfg->ofdm_data_start()][j].im);
  // std::printf("\n");

  // Prepare downlink data from mod_output
  Table<complex_float> dl_mod_data;
  dl_mod_data.calloc(cfg->frame().NumDLSyms(),
                     cfg->ofdm_ca_num() * cfg->ue_ant_num(),
                     Agora_memory::Alignment_t::k64Align);
  for (size_t i = 0; i < cfg->frame().NumDLSyms(); i++) {
    for (size_t j = 0; j < cfg->ue_ant_num(); j++) {
      if ((i >= cfg->frame().client_dl_pilot_symbols())) {
        for (size_t sc_id = 0; sc_id < cfg->ofdm_data_num(); sc_id++)
          dl_mod_data[i][j * cfg->ofdm_ca_num() + sc_id +
                         cfg->ofdm_data_start()] =
              (sc_id % cfg->ofdm_pilot_spacing() == 0)
                  ? ue_specific_pilot[j][sc_id]
                  : dl_modulated_codewords[i * cfg->ue_ant_num() + j][sc_id];
      } else {
        for (size_t sc_id = 0; sc_id < cfg->ofdm_data_num(); sc_id++)
          dl_mod_data[i][j * cfg->ofdm_ca_num() + sc_id +
                         cfg->ofdm_data_start()] = ue_specific_pilot[j][sc_id];
      }
    }
  }

  // std::printf("dl mod data \n");
  // for (int i = 0; i < dl_data_symbol_num_perframe(); i++) {
  //     for (int k = cfg->ofdm_data_start(); k < cfg->ofdm_data_start() +
  //     cfg->ofdm_data_num();
  //          k++) {
  //         std::printf("symbol %d, subcarrier %d\n", i, k);
  //         for (int j = 0; j < cfg->ue_ant_num(); j++) {

  //             // for (int k = cfg->ofdm_data_start(); k <
  //             cfg->ofdm_data_start() + cfg->ofdm_data_num();
  //             //      k++) {
  //             std::printf("%.3f+%.3fi ", dl_mod_data[i][j *
  //             cfg->ofdm_ca_num() + k].re,
  //                 dl_mod_data[i][j * cfg->ofdm_ca_num() + k].im);
  //         }
  //         std::printf("\n");
  //     }
  // }

  // Perform precoding and IFFT
  Table<complex_float> dl_ifft_data;
  dl_ifft_data.calloc(cfg->frame().NumDLSyms(),
                      cfg->ofdm_ca_num() * cfg->bs_ant_num(),
                      Agora_memory::Alignment_t::k64Align);
  Table<short> dl_tx_data;
  dl_tx_data.calloc(cfg->frame().NumDLSyms(),
                    2 * cfg->samps_per_symbol() * cfg->bs_ant_num(),
                    Agora_memory::Alignment_t::k64Align);
  for (size_t i = 0; i < cfg->frame().NumDLSyms(); i++) {
    arma::cx_fmat mat_input_data(
        reinterpret_cast<arma::cx_float*>(dl_mod_data[i]), cfg->ofdm_ca_num(),
        cfg->ue_ant_num(), false);

    arma::cx_fmat mat_output(reinterpret_cast<arma::cx_float*>(dl_ifft_data[i]),
                             cfg->ofdm_ca_num(), cfg->bs_ant_num(), false);

    for (size_t j = cfg->ofdm_data_start();
         j < cfg->ofdm_data_num() + cfg->ofdm_data_start(); j++) {
      arma::cx_fmat mat_precoder(reinterpret_cast<arma::cx_float*>(precoder[j]),
                                 cfg->ue_ant_num(), cfg->bs_ant_num(), false);
      mat_precoder /= abs(mat_precoder).max();
      mat_output.row(j) = mat_input_data.row(j) * mat_precoder;

      // std::printf("symbol %d, sc: %d\n", i, j - cfg->ofdm_data_start());
      // cout << "Precoder: \n" << mat_precoder << endl;
      // cout << "Data: \n" << mat_input_data.row(j) << endl;
      // cout << "Precoded data: \n" << mat_output.row(j) << endl;
    }
    for (size_t j = 0; j < cfg->bs_ant_num(); j++) {
      complex_float* ptr_ifft = dl_ifft_data[i] + j * cfg->ofdm_ca_num();
      CommsLib::IFFT(ptr_ifft, cfg->ofdm_ca_num(), false);

      short* txSymbol = dl_tx_data[i] + j * cfg->samps_per_symbol() * 2;
      std::memset(txSymbol, 0, sizeof(short) * 2 * cfg->ofdm_tx_zero_prefix());
      for (size_t k = 0; k < cfg->ofdm_ca_num(); k++) {
        txSymbol[2 * (k + cfg->cp_len() + cfg->ofdm_tx_zero_prefix())] =
            (short)(32768 * ptr_ifft[k].re);
        txSymbol[2 * (k + cfg->cp_len() + cfg->ofdm_tx_zero_prefix()) + 1] =
            (short)(32768 * ptr_ifft[k].im);
      }
      for (size_t k = 0; k < (2 * cfg->cp_len()); k++) {
        txSymbol[2 * cfg->ofdm_tx_zero_prefix() + k] =
            txSymbol[2 * (cfg->ofdm_tx_zero_prefix() + cfg->ofdm_ca_num())];
      }

      const size_t tx_zero_postfix_offset =
          2 * (cfg->ofdm_tx_zero_prefix() + cfg->cp_len() + cfg->ofdm_ca_num());
      std::memset(txSymbol + tx_zero_postfix_offset, 0,
                  sizeof(short) * 2 * cfg->ofdm_tx_zero_postfix());
    }
  }

  std::string filename_dl_tx = cur_directory + "/data/LDPC_dl_tx_data_" +
                               std::to_string(cfg->ofdm_ca_num()) + "_ant" +
                               std::to_string(cfg->bs_ant_num()) + ".bin";
  std::printf("Saving dl tx data to %s\n", filename_dl_tx.c_str());
  FILE* fp_dl_tx = std::fopen(filename_dl_tx.c_str(), "wb");
  for (size_t i = 0; i < cfg->frame().NumDLSyms(); i++) {
    short* ptr = dl_tx_data[i];
    std::fwrite(ptr, cfg->samps_per_symbol() * cfg->bs_ant_num() * 2,
                sizeof(short), fp_dl_tx);
  }
  std::fclose(fp_dl_tx);

  // std::printf("rx data\n");
  // for (int i = 0; i < 10; i++) {

  //     for (int j = 0; j < cfg->ofdm_ca_num() * cfg->bs_ant_num(); j++) {
  //         if (j % cfg->ofdm_ca_num() == 0) {
  //             std::printf("symbol %d ant %d\n", i, j / cfg->ofdm_ca_num());
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
