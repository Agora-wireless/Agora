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
#include "config.h"
#include "memory_manage.h"
#include "modulation.h"
#include "utils_ldpc.h"

static constexpr bool kVerbose = false;
static constexpr bool kPrintUplinkInformationBytes = false;
static constexpr bool kPrintDownlinkInformationBytes = false;

DEFINE_string(profile, "random",
              "The profile of the input user bytes (e.g., 'random', '123')");
DEFINE_string(conf_file,
              TOSTRING(PROJECT_DIRECTORY) "/data/tddconfig-sim-ul.json",
              "Agora config filename");

float RandFloat(float min, float max) {
  return ((float(rand()) / float(RAND_MAX)) * (max - min)) + min;
}

float RandFloatFromShort(float min, float max) {
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
  const size_t num_ul_codeblocks =
      cfg->ul_data_symbol_num_perframe_ *
      (cfg->ldpc_config_.nblocks_in_symbol_ * cfg->ue_ant_num_);
  std::printf("Total number of ul blocks: %zu\n", num_ul_codeblocks);

  std::vector<std::vector<int8_t>> ul_information(num_ul_codeblocks);
  std::vector<std::vector<int8_t>> ul_encoded_codewords(num_ul_codeblocks);
  for (size_t i = 0; i < num_ul_codeblocks; i++) {
    data_generator.GenCodeblock(ul_information.at(i),
                                ul_encoded_codewords.at(i),
                                (i % cfg->ue_num_) /* UE ID */);
  }

  {
    // Save uplink information bytes to file
    const size_t input_bytes_per_cb = BitsToBytes(
        LdpcNumInputBits(cfg->ldpc_config_.bg_, cfg->ldpc_config_.zc_));

    const std::string filename_input =
        cur_directory + "/data/LDPC_orig_ul_data_" +
        std::to_string(cfg->ofdm_ca_num_) + "_ant" +
        std::to_string(cfg->ue_ant_num_) + ".bin";
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
        std::printf("Symbol %zu, UE %zu\n", n / cfg->ue_ant_num_,
                    n % cfg->ue_ant_num_);
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
    ul_modulated_codewords.at(i) =
        data_generator.GetModulation(ul_encoded_codewords[i]);
  }

  // Place modulated uplink data codewords into central IFFT bins
  RtAssert(cfg->ldpc_config_.nblocks_in_symbol_ == 1);  // TODO: Assumption
  std::vector<std::vector<complex_float>> pre_ifft_data_syms(
      cfg->ue_ant_num_ * cfg->ul_data_symbol_num_perframe_);
  for (size_t i = 0; i < pre_ifft_data_syms.size(); i++) {
    pre_ifft_data_syms.at(i) =
        data_generator.BinForIfft(ul_modulated_codewords.at(i));
  }

  std::vector<complex_float> pilot_td =
      data_generator.GetCommonPilotTimeDomain();

  // Generate UE-specific pilots
  Table<complex_float> ue_specific_pilot;
  const std::vector<std::complex<float>> zc_seq = Utils::DoubleToCfloat(
      CommsLib::GetSequence(cfg->ofdm_data_num_, CommsLib::kLteZadoffChu));
  const std::vector<std::complex<float>> zc_common_pilot =
      CommsLib::SeqCyclicShift(zc_seq, M_PI / 4.0);  // Used in LTE SRS
  ue_specific_pilot.Malloc(cfg->ue_ant_num_, cfg->ofdm_data_num_,
                           Agora_memory::Alignment_t::kK64Align);
  for (size_t i = 0; i < cfg->ue_ant_num_; i++) {
    auto zc_ue_pilot_i =
        CommsLib::SeqCyclicShift(zc_seq, i * M_PI / 6.0);  // LTE DMRS
    for (size_t j = 0; j < cfg->ofdm_data_num_; j++) {
      ue_specific_pilot[i][j] = {zc_ue_pilot_i[j].real(),
                                 zc_ue_pilot_i[j].imag()};
    }
  }

  // Put pilot and data symbols together
  Table<complex_float> tx_data_all_symbols;
  tx_data_all_symbols.Calloc(cfg->symbol_num_perframe_,
                             cfg->ue_ant_num_ * cfg->ofdm_ca_num_,
                             Agora_memory::Alignment_t::kK64Align);

  if (cfg->freq_orthogonal_pilot_) {
    for (size_t i = 0; i < cfg->ue_ant_num_; i++) {
      std::vector<complex_float> pilots_t_ue(cfg->ofdm_ca_num_);  // Zeroed
      for (size_t j = cfg->ofdm_data_start_;
           j < cfg->ofdm_data_start_ + cfg->ofdm_data_num_;
           j += cfg->ue_ant_num_) {
        pilots_t_ue.at(i + j) = pilot_td.at(i + j);
      }
      // Load pilot to the second symbol
      // The first symbol is reserved for beacon
      std::memcpy(tx_data_all_symbols[cfg->beacon_symbol_num_perframe_] +
                      (i * cfg->ofdm_ca_num_),
                  &pilots_t_ue.at(0),
                  cfg->ofdm_ca_num_ * sizeof(complex_float));
    }
  } else {
    for (size_t i = 0; i < cfg->ue_ant_num_; i++) {
      std::memcpy(tx_data_all_symbols[i + cfg->beacon_symbol_num_perframe_] +
                      i * cfg->ofdm_ca_num_,
                  &pilot_td.at(0), cfg->ofdm_ca_num_ * sizeof(complex_float));
    }
  }

  // Populate the UL symbols
  for (size_t i = 0; i < cfg->ul_data_symbol_num_perframe_; i++) {
    const size_t data_sym_id = cfg->ul_symbols_.at(0).at(i);
    for (size_t j = 0; j < cfg->ue_ant_num_; j++) {
      if (i < cfg->ul_pilot_syms_) {
        std::memcpy(tx_data_all_symbols[data_sym_id] + (j * cfg->ofdm_ca_num_) +
                        cfg->ofdm_data_start_,
                    ue_specific_pilot[j],
                    cfg->ofdm_data_num_ * sizeof(complex_float));
      } else {
        std::memcpy(tx_data_all_symbols[data_sym_id] + (j * cfg->ofdm_ca_num_),
                    &pre_ifft_data_syms.at(i * cfg->ue_ant_num_ + j).at(0),
                    cfg->ofdm_ca_num_ * sizeof(complex_float));
      }
    }
  }
  // Generate CSI matrix
  Table<complex_float> csi_matrices;
  csi_matrices.Calloc(cfg->ofdm_ca_num_, cfg->ue_ant_num_ * cfg->bs_ant_num_,
                      Agora_memory::Alignment_t::kK32Align);
  for (size_t i = 0; i < cfg->ue_ant_num_ * cfg->bs_ant_num_; i++) {
    complex_float csi = {RandFloatFromShort(-1, 1), RandFloatFromShort(-1, 1)};
    // std::printf("noise of ant %d, ue %d\n", i % cfg->BS_ANT_NUM, i /
    // cfg->BS_ANT_NUM );
    for (size_t j = 0; j < cfg->ofdm_ca_num_; j++) {
      complex_float noise = {RandFloatFromShort(-1, 1) * cfg->noise_level_,
                             RandFloatFromShort(-1, 1) * cfg->noise_level_};
      // std::printf("%.4f+%.4fi ", noise.re, noise.im);
      csi_matrices[j][i].re = csi.re + noise.re;
      csi_matrices[j][i].im = csi.im + noise.im;
    }
    // std::printf("\n");
  }

  // Generate RX data received by base station after going through channels
  Table<complex_float> rx_data_all_symbols;
  rx_data_all_symbols.Calloc(cfg->symbol_num_perframe_,
                             cfg->ofdm_ca_num_ * cfg->bs_ant_num_,
                             Agora_memory::Alignment_t::kK64Align);
  for (size_t i = 0; i < cfg->symbol_num_perframe_; i++) {
    arma::cx_fmat mat_input_data(
        reinterpret_cast<arma::cx_float*>(tx_data_all_symbols[i]),
        cfg->ofdm_ca_num_, cfg->ue_ant_num_, false);
    arma::cx_fmat mat_output(
        reinterpret_cast<arma::cx_float*>(rx_data_all_symbols[i]),
        cfg->ofdm_ca_num_, cfg->bs_ant_num_, false);

    for (size_t j = 0; j < cfg->ofdm_ca_num_; j++) {
      arma::cx_fmat mat_csi(reinterpret_cast<arma::cx_float*>(csi_matrices[j]),
                            cfg->bs_ant_num_, cfg->ue_ant_num_);
      mat_output.row(j) = mat_input_data.row(j) * mat_csi.st();
    }
    for (size_t j = 0; j < cfg->bs_ant_num_; j++) {
      CommsLib::IFFT(rx_data_all_symbols[i] + j * cfg->ofdm_ca_num_,
                     cfg->ofdm_ca_num_, false);
    }
  }

  std::string filename_rx = cur_directory + "/data/LDPC_rx_data_" +
                            std::to_string(cfg->ofdm_ca_num_) + "_ant" +
                            std::to_string(cfg->bs_ant_num_) + ".bin";
  std::printf("Saving rx data to %s\n", filename_rx.c_str());
  FILE* fp_rx = std::fopen(filename_rx.c_str(), "wb");
  for (size_t i = 0; i < cfg->symbol_num_perframe_; i++) {
    auto* ptr = (float*)rx_data_all_symbols[i];
    std::fwrite(ptr, cfg->ofdm_ca_num_ * cfg->bs_ant_num_ * 2, sizeof(float),
                fp_rx);
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
  const size_t num_dl_codeblocks = cfg->dl_data_symbol_num_perframe_ *
                                   cfg->ldpc_config_.nblocks_in_symbol_ *
                                   cfg->ue_ant_num_;
  std::printf("Total number of dl blocks: %zu\n", num_dl_codeblocks);

  std::vector<std::vector<int8_t>> dl_information(num_dl_codeblocks);
  std::vector<std::vector<int8_t>> dl_encoded_codewords(num_dl_codeblocks);
  for (size_t i = 0; i < num_dl_codeblocks; i++) {
    data_generator.GenCodeblock(dl_information.at(i),
                                dl_encoded_codewords.at(i),
                                (i % cfg->ue_num_) /* UE ID */);
  }

  // Modulate the encoded codewords
  std::vector<std::vector<complex_float>> dl_modulated_codewords(
      num_dl_codeblocks);
  for (size_t i = 0; i < num_dl_codeblocks; i++) {
    dl_modulated_codewords.at(i) =
        data_generator.GetModulation(dl_encoded_codewords[i]);
  }

  {
    // Save downlink information bytes to file
    const size_t input_bytes_per_cb = BitsToBytes(
        LdpcNumInputBits(cfg->ldpc_config_.bg_, cfg->ldpc_config_.zc_));

    const std::string filename_input =
        cur_directory + "/data/LDPC_orig_dl_data_" +
        std::to_string(cfg->ofdm_ca_num_) + "_ant" +
        std::to_string(cfg->ue_ant_num_) + ".bin";
    std::printf("Saving raw dl data (using LDPC) to %s\n",
                filename_input.c_str());
    FILE* fp_input = std::fopen(filename_input.c_str(), "wb");
    for (size_t i = 0; i < num_dl_codeblocks; i++) {
      std::fwrite(reinterpret_cast<uint8_t*>(&dl_information.at(i).at(0)),
                  input_bytes_per_cb, sizeof(uint8_t), fp_input);
    }
    std::fclose(fp_input);

    if (kPrintDownlinkInformationBytes == true) {
      std::printf("Downlink information bytes\n");
      for (size_t n = 0; n < num_dl_codeblocks; n++) {
        std::printf("Symbol %zu, UE %zu\n", n / cfg->ue_ant_num_,
                    n % cfg->ue_ant_num_);
        for (size_t i = 0; i < input_bytes_per_cb; i++) {
          std::printf("%u ", (uint8_t)dl_information.at(n).at(i));
        }
        std::printf("\n");
      }
    }
  }

  // Compute precoder
  Table<complex_float> precoder;
  precoder.Calloc(cfg->ofdm_ca_num_, cfg->ue_ant_num_ * cfg->bs_ant_num_,
                  Agora_memory::Alignment_t::kK32Align);
  for (size_t i = 0; i < cfg->ofdm_ca_num_; i++) {
    arma::cx_fmat mat_input(reinterpret_cast<arma::cx_float*>(csi_matrices[i]),
                            cfg->bs_ant_num_, cfg->ue_ant_num_, false);
    arma::cx_fmat mat_output(reinterpret_cast<arma::cx_float*>(precoder[i]),
                             cfg->ue_ant_num_, cfg->bs_ant_num_, false);
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
  dl_mod_data.Calloc(cfg->dl_data_symbol_num_perframe_,
                     cfg->ofdm_ca_num_ * cfg->ue_ant_num_,
                     Agora_memory::Alignment_t::kK64Align);
  for (size_t i = 0; i < cfg->dl_data_symbol_num_perframe_; i++) {
    for (size_t j = 0; j < cfg->ue_ant_num_; j++) {
      if (i >= cfg->dl_pilot_syms_) {
        for (size_t sc_id = 0; sc_id < cfg->ofdm_data_num_; sc_id++) {
          dl_mod_data[i][j * cfg->ofdm_ca_num_ + sc_id +
                         cfg->ofdm_data_start_] =
              (sc_id % cfg->ofdm_pilot_spacing_ == 0)
                  ? ue_specific_pilot[0][sc_id]
                  : dl_modulated_codewords.at(i * cfg->ue_ant_num_ + j)
                        .at(sc_id);
        }
      } else {
        for (size_t sc_id = 0; sc_id < cfg->ofdm_data_num_; sc_id++) {
          dl_mod_data[i][j * cfg->ofdm_ca_num_ + sc_id +
                         cfg->ofdm_data_start_] = ue_specific_pilot[0][sc_id];
        }
      }
    }
  }

  // std::printf("dl mod data \n");
  // for (int i = 0; i < dl_data_symbol_num_perframe; i++) {
  //     for (int k = cfg->OFDM_DATA_START; k < cfg->OFDM_DATA_START +
  //     cfg->OFDM_DATA_NUM;
  //          k++) {
  //         std::printf("symbol %d, subcarrier %d\n", i, k);
  //         for (int j = 0; j < cfg->UE_ANT_NUM; j++) {

  //             // for (int k = cfg->OFDM_DATA_START; k < cfg->OFDM_DATA_START
  //             + cfg->OFDM_DATA_NUM;
  //             //      k++) {
  //             std::printf("%.3f+%.3fi ", dl_mod_data[i][j * cfg->OFDM_CA_NUM
  //             + k].re,
  //                 dl_mod_data[i][j * cfg->OFDM_CA_NUM + k].im);
  //         }
  //         std::printf("\n");
  //     }
  // }

  // Perform precoding and IFFT
  Table<complex_float> dl_ifft_data;
  dl_ifft_data.Calloc(cfg->dl_data_symbol_num_perframe_,
                      cfg->ofdm_ca_num_ * cfg->bs_ant_num_,
                      Agora_memory::Alignment_t::kK64Align);
  Table<short> dl_tx_data;
  dl_tx_data.Calloc(cfg->dl_data_symbol_num_perframe_,
                    2 * cfg->samps_per_symbol_ * cfg->bs_ant_num_,
                    Agora_memory::Alignment_t::kK64Align);
  for (size_t i = 0; i < cfg->dl_data_symbol_num_perframe_; i++) {
    arma::cx_fmat mat_input_data(
        reinterpret_cast<arma::cx_float*>(dl_mod_data[i]), cfg->ofdm_ca_num_,
        cfg->ue_ant_num_, false);

    arma::cx_fmat mat_output(reinterpret_cast<arma::cx_float*>(dl_ifft_data[i]),
                             cfg->ofdm_ca_num_, cfg->bs_ant_num_, false);

    for (size_t j = cfg->ofdm_data_start_;
         j < cfg->ofdm_data_num_ + cfg->ofdm_data_start_; j++) {
      arma::cx_fmat mat_precoder(reinterpret_cast<arma::cx_float*>(precoder[j]),
                                 cfg->ue_ant_num_, cfg->bs_ant_num_, false);
      mat_precoder /= abs(mat_precoder).max();
      mat_output.row(j) = mat_input_data.row(j) * mat_precoder;

      // std::printf("symbol %d, sc: %d\n", i, j - cfg->OFDM_DATA_START);
      // cout << "Precoder: \n" << mat_precoder << endl;
      // cout << "Data: \n" << mat_input_data.row(j) << endl;
      // cout << "Precoded data: \n" << mat_output.row(j) << endl;
    }
    for (size_t j = 0; j < cfg->bs_ant_num_; j++) {
      complex_float* ptr_ifft = dl_ifft_data[i] + j * cfg->ofdm_ca_num_;
      CommsLib::IFFT(ptr_ifft, cfg->ofdm_ca_num_, false);

      short* tx_symbol = dl_tx_data[i] + j * cfg->samps_per_symbol_ * 2;
      std::memset(tx_symbol, 0, sizeof(short) * 2 * cfg->ofdm_tx_zero_prefix_);
      for (size_t k = 0; k < cfg->ofdm_ca_num_; k++) {
        tx_symbol[2 * (k + cfg->cp_len_ + cfg->ofdm_tx_zero_prefix_)] =
            static_cast<short>(32768 * ptr_ifft[k].re *
                               std::sqrt(cfg->bs_ant_num_ * 1.f));
        tx_symbol[2 * (k + cfg->cp_len_ + cfg->ofdm_tx_zero_prefix_) + 1] =
            static_cast<short>(32768 * ptr_ifft[k].im *
                               std::sqrt(cfg->bs_ant_num_ * 1.f));
      }
      for (size_t k = 0; k < 2 * cfg->cp_len_; k++) {
        tx_symbol[2 * cfg->ofdm_tx_zero_prefix_ + k] =
            tx_symbol[2 * (cfg->ofdm_tx_zero_prefix_ + cfg->ofdm_ca_num_)];
      }

      const size_t tx_zero_postfix_offset =
          2 * (cfg->ofdm_tx_zero_prefix_ + cfg->cp_len_ + cfg->ofdm_ca_num_);
      std::memset(tx_symbol + tx_zero_postfix_offset, 0,
                  sizeof(short) * 2 * cfg->ofdm_tx_zero_postfix_);
    }
  }

  std::string filename_dl_tx = cur_directory + "/data/LDPC_dl_tx_data_" +
                               std::to_string(cfg->ofdm_ca_num_) + "_ant" +
                               std::to_string(cfg->bs_ant_num_) + ".bin";
  std::printf("Saving dl tx data to %s\n", filename_dl_tx.c_str());
  FILE* fp_dl_tx = std::fopen(filename_dl_tx.c_str(), "wb");
  for (size_t i = 0; i < cfg->dl_data_symbol_num_perframe_; i++) {
    short* ptr = (short*)dl_tx_data[i];
    std::fwrite(ptr, cfg->samps_per_symbol_ * cfg->bs_ant_num_ * 2,
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
  dl_ifft_data.Free();
  dl_tx_data.Free();
  dl_mod_data.Free();
  precoder.Free();

  csi_matrices.Free();
  tx_data_all_symbols.Free();
  rx_data_all_symbols.Free();
  ue_specific_pilot.Free();
  delete cfg;

  return 0;
}
