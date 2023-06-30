/**
 * @file data_generator_main.cc
 * @brief Data generator to generate binary files as inputs to Agora, sender
 * and correctness tests
 */
#include <gflags/gflags.h>

#include <cstddef>
#include <memory>
#include <string>

#include "comms-lib.h"
#include "config.h"
#include "crc.h"
#include "data_generator.h"
#include "datatype_conversion.h"
#include "logger.h"
#include "version_config.h"

static constexpr bool kVerbose = false;
static constexpr bool kPrintDebugCSI = false;
static constexpr bool kDebugPrintRxData = false;
static constexpr bool kPrintDlTxData = false;
static constexpr bool kPrintDlModData = false;
static constexpr bool kPrintUplinkInformationBytes = false;
static constexpr bool kPrintDownlinkInformationBytes = false;

///Output files
static const std::string kUlDataPrefix = "orig_ul_data_";
static const std::string kUlLdpcDataPrefix = "LDPC_orig_ul_data_";
static const std::string kUlModDataPrefix = "mod_ul_data_";
static const std::string kUlTxPrefix = "ul_ifft_data_";
static const std::string kDlDataPrefix = "orig_dl_data_";
static const std::string kDlLdpcDataPrefix = "LDPC_orig_dl_data_";
static const std::string kRxLdpcPrefix = "LDPC_rx_data_";
static const std::string kDlTxPrefix = "LDPC_dl_tx_data_";

DEFINE_string(profile, "random",
              "The profile of the input user bytes (e.g., 'random', '123')");
DEFINE_string(
    conf_file,
    TOSTRING(PROJECT_DIRECTORY) "/files/examples/ci/tddconfig-sim-both.json",
    "Agora config filename");

static float RandFloatFromShort(float min, float max) {
  float rand_val = ((float(rand()) / float(RAND_MAX)) * (max - min)) + min;
  const auto rand_val_short = static_cast<short>(rand_val * kShrtFltConvFactor);
  rand_val = static_cast<float>(rand_val_short) / kShrtFltConvFactor;
  return rand_val;
}

int main(int argc, char* argv[]) {
  const std::string directory =
      TOSTRING(PROJECT_DIRECTORY) "/files/experiment/";
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  gflags::SetVersionString(GetAgoraProjectVersion());
  AGORA_LOG_INIT();
  auto cfg_ = std::make_unique<Config>(FLAGS_conf_file.c_str());

  const DataGenerator::Profile profile =
      FLAGS_profile == "123" ? DataGenerator::Profile::kProfile123
                             : DataGenerator::Profile::kRandom;
  std::unique_ptr<DataGenerator> data_generator =
      std::make_unique<DataGenerator>(cfg_.get(), 0 /* RNG seed */, profile);

  AGORA_LOG_INFO("DataGenerator: Using %s-orthogonal pilots\n",
                 cfg_->FreqOrthogonalPilot() ? "frequency" : "time");

  AGORA_LOG_INFO("DataGenerator: Generating encoded and modulated data\n");

  //Make sure the directory exists
  if (std::filesystem::is_directory(directory) == false) {
    std::filesystem::create_directory(directory);
  }
  srand(time(nullptr));
  std::unique_ptr<DoCRC> crc_obj = std::make_unique<DoCRC>();
  const size_t ul_cb_bytes = cfg_->NumBytesPerCb(Direction::kUplink);
  LDPCconfig ul_ldpc_config = cfg_->LdpcConfig(Direction::kUplink);

  // Step 1: Generate the information buffers (MAC Packets) and LDPC-encoded
  // buffers for uplink
  std::vector<std::vector<complex_float>> pre_ifft_data_syms;
  const size_t num_ul_mac_bytes = cfg_->MacBytesNumPerframe(Direction::kUplink);
  if (num_ul_mac_bytes > 0) {
    std::vector<std::vector<int8_t>> ul_mac_info(cfg_->UeAntNum());
    AGORA_LOG_INFO("Total number of uplink MAC bytes: %zu\n", num_ul_mac_bytes);
    for (size_t ue_id = 0; ue_id < cfg_->UeAntNum(); ue_id++) {
      ul_mac_info.at(ue_id).resize(num_ul_mac_bytes);
      for (size_t pkt_id = 0;
           pkt_id < cfg_->MacPacketsPerframe(Direction::kUplink); pkt_id++) {
        size_t pkt_offset = pkt_id * cfg_->MacPacketLength(Direction::kUplink);
        auto* pkt = reinterpret_cast<MacPacketPacked*>(
            &ul_mac_info.at(ue_id).at(pkt_offset));

        pkt->Set(0, pkt_id, ue_id,
                 cfg_->MacPayloadMaxLength(Direction::kUplink));
        data_generator->GenMacData(pkt, ue_id);
        pkt->Crc((uint16_t)(crc_obj->CalculateCrc24(
                                pkt->Data(),
                                cfg_->MacPayloadMaxLength(Direction::kUplink)) &
                            0xFFFF));
      }
    }

    {
      const std::string filename_input =
          directory + kUlDataPrefix + std::to_string(cfg_->OfdmCaNum()) +
          "_ant" + std::to_string(cfg_->UeAntNum()) + ".bin";
      AGORA_LOG_INFO("Saving uplink MAC data to %s\n", filename_input.c_str());
      for (size_t i = 0; i < cfg_->UeAntNum(); i++) {
        Utils::WriteBinaryFile(filename_input, sizeof(uint8_t),
                               num_ul_mac_bytes, ul_mac_info.at(i).data(),
                               i != 0);  //Do not append in the first write
      }

      if (kPrintUplinkInformationBytes) {
        std::printf("Uplink information bytes\n");
        for (size_t n = 0; n < cfg_->UeAntNum(); n++) {
          std::printf("UE %zu\n", n % cfg_->UeAntNum());
          for (size_t i = 0; i < num_ul_mac_bytes; i++) {
            std::printf("%u ", static_cast<uint8_t>(ul_mac_info.at(n).at(i)));
          }
          std::printf("\n");
        }
      }
    }

    const size_t symbol_blocks =
        ul_ldpc_config.NumBlocksInSymbol() * cfg_->UeAntNum();
    const size_t num_ul_codeblocks =
        cfg_->Frame().NumUlDataSyms() * symbol_blocks;
    AGORA_LOG_SYMBOL("Total number of ul blocks: %zu\n", num_ul_codeblocks);

    std::vector<std::vector<int8_t>> ul_information(num_ul_codeblocks);
    std::vector<std::vector<int8_t>> ul_encoded_codewords(num_ul_codeblocks);
    for (size_t cb = 0; cb < num_ul_codeblocks; cb++) {
      // i : symbol -> ue -> cb (repeat)
      size_t sym_id = cb / (symbol_blocks);
      // ue antenna for code block
      size_t sym_offset = cb % (symbol_blocks);
      size_t ue_id = sym_offset / ul_ldpc_config.NumBlocksInSymbol();
      size_t ue_cb_id = sym_offset % ul_ldpc_config.NumBlocksInSymbol();
      size_t ue_cb_cnt =
          (sym_id * ul_ldpc_config.NumBlocksInSymbol()) + ue_cb_id;

      AGORA_LOG_TRACE(
          "cb %zu -- user %zu -- user block %zu -- user cb id %zu -- input "
          "size %zu, index %zu, total size %zu\n",
          cb, ue_id, ue_cb_id, ue_cb_cnt, ul_cb_bytes, ue_cb_cnt * ul_cb_bytes,
          ul_mac_info.at(ue_id).size());
      int8_t* cb_start = &ul_mac_info.at(ue_id).at(ue_cb_cnt * ul_cb_bytes);
      ul_information.at(cb) =
          std::vector<int8_t>(cb_start, cb_start + ul_cb_bytes);
      ul_encoded_codewords.at(cb) = DataGenerator::GenCodeblock(
          ul_ldpc_config, &ul_information.at(cb).at(0), ul_cb_bytes,
          cfg_->ScrambleEnabled());
    }

    {
      const std::string filename_input =
          directory + kUlLdpcDataPrefix + std::to_string(cfg_->OfdmCaNum()) +
          "_ant" + std::to_string(cfg_->UeAntNum()) + ".bin";
      AGORA_LOG_INFO("Saving raw uplink data (using LDPC) to %s\n",
                     filename_input.c_str());
      for (size_t i = 0; i < num_ul_codeblocks; i++) {
        Utils::WriteBinaryFile(filename_input, sizeof(uint8_t), ul_cb_bytes,
                               ul_information.at(i).data(),
                               i != 0);  //Do not append in the first write
      }

      if (kPrintUplinkInformationBytes) {
        std::printf("Uplink information bytes\n");
        for (size_t n = 0; n < num_ul_codeblocks; n++) {
          std::printf("Symbol %zu, UE %zu\n", n / cfg_->UeAntNum(),
                      n % cfg_->UeAntNum());
          for (size_t i = 0; i < ul_cb_bytes; i++) {
            std::printf("%u ",
                        static_cast<uint8_t>(ul_information.at(n).at(i)));
          }
          std::printf("\n");
        }
      }
    }

    // Modulate the encoded codewords
    std::vector<std::vector<uint8_t>> ul_modulated_codewords(num_ul_codeblocks);
    std::vector<std::vector<complex_float>> ul_modulated_symbols(
        num_ul_codeblocks);
    for (size_t i = 0; i < num_ul_codeblocks; i++) {
      ul_modulated_codewords.at(i).resize(cfg_->OfdmDataNum());
      auto ofdm_symbol = DataGenerator::GetModulation(
          &ul_encoded_codewords.at(i)[0], &ul_modulated_codewords.at(i).at(0),
          cfg_->ModTable(Direction::kUplink),
          cfg_->LdpcConfig(Direction::kUplink).NumCbCodewLen(),
          cfg_->OfdmDataNum(), cfg_->ModOrderBits(Direction::kUplink));
      ul_modulated_symbols.at(i) = DataGenerator::MapOFDMSymbol(
          cfg_.get(), ofdm_symbol, nullptr, SymbolType::kUL);
    }

    {
      const std::string filename_input =
          directory + kUlModDataPrefix + std::to_string(cfg_->OfdmCaNum()) +
          "_ant" + std::to_string(cfg_->UeAntNum()) + ".bin";
      AGORA_LOG_INFO("Saving modulated uplink data to %s\n",
                     filename_input.c_str());
      for (size_t i = 0; i < num_ul_codeblocks; i++) {
        Utils::WriteBinaryFile(filename_input, sizeof(uint8_t),
                               cfg_->OfdmDataNum(),
                               ul_modulated_codewords.at(i).data(),
                               i != 0);  //Do not append in the first write
      }
    }

    // Place modulated uplink data codewords into central IFFT bins
    RtAssert(ul_ldpc_config.NumBlocksInSymbol() == 1);  // TODO: Assumption
    pre_ifft_data_syms.resize(cfg_->UeAntNum() * cfg_->Frame().NumUlDataSyms());
    for (size_t i = 0; i < pre_ifft_data_syms.size(); i++) {
      pre_ifft_data_syms.at(i) =
          DataGenerator::BinForIfft(cfg_.get(), ul_modulated_symbols.at(i));
    }
  }

  // Generate common sounding pilots
  std::vector<complex_float> pilot_fd =
      data_generator->GetCommonPilotFreqDomain();

  // Generate UE-specific pilots (phase tracking & downlink channel estimation)
  Table<complex_float> ue_specific_pilot =
      data_generator->GetUeSpecificPilotFreqDomain();

  // Put pilot and data symbols together
  Table<complex_float> tx_data_all_symbols;
  tx_data_all_symbols.Calloc(cfg_->Frame().NumTotalSyms(),
                             cfg_->UeAntNum() * cfg_->OfdmCaNum(),
                             Agora_memory::Alignment_t::kAlign64);

  if (cfg_->FreqOrthogonalPilot()) {
    const size_t pilot_sym_idx = cfg_->Frame().GetPilotSymbol(0);
    RtAssert(cfg_->Frame().NumPilotSyms() == 1,
             "Number of pilot symbols must be 1");
    for (size_t i = 0; i < cfg_->UeAntNum(); i++) {
      std::vector<complex_float> pilots_f_ue(cfg_->OfdmCaNum());  // Zeroed
      for (size_t j = cfg_->OfdmDataStart(); j < cfg_->OfdmDataStop();
           j += cfg_->PilotScGroupSize()) {
        pilots_f_ue.at(i + j) = pilot_fd.at(i + j);
      }
      // Load pilots
      std::memcpy(tx_data_all_symbols[pilot_sym_idx] + (i * cfg_->OfdmCaNum()),
                  &pilots_f_ue.at(0),
                  (cfg_->OfdmCaNum() * sizeof(complex_float)));
    }
  } else {
    for (size_t i = 0; i < cfg_->UeAntNum(); i++) {
      const size_t pilot_sym_idx = cfg_->Frame().GetPilotSymbol(i);
      std::memcpy(tx_data_all_symbols[pilot_sym_idx] + i * cfg_->OfdmCaNum(),
                  &pilot_fd.at(0), (cfg_->OfdmCaNum() * sizeof(complex_float)));
    }
  }

  // Populate the UL symbols
  for (size_t i = 0; i < cfg_->Frame().NumULSyms(); i++) {
    const size_t data_sym_id = cfg_->Frame().GetULSymbol(i);
    for (size_t j = 0; j < cfg_->UeAntNum(); j++) {
      if (i < cfg_->Frame().ClientUlPilotSymbols()) {
        std::memcpy(tx_data_all_symbols[data_sym_id] + (j * cfg_->OfdmCaNum()) +
                        cfg_->OfdmDataStart(),
                    ue_specific_pilot[j],
                    cfg_->OfdmDataNum() * sizeof(complex_float));
      } else {
        const size_t k = i - cfg_->Frame().ClientUlPilotSymbols();
        std::memcpy(tx_data_all_symbols[data_sym_id] + (j * cfg_->OfdmCaNum()),
                    &pre_ifft_data_syms.at(k * cfg_->UeAntNum() + j).at(0),
                    cfg_->OfdmCaNum() * sizeof(complex_float));
      }
    }
  }

  {
    const std::string filename_tx = directory + kUlTxPrefix +
                                    std::to_string(cfg_->OfdmCaNum()) + "_ant" +
                                    std::to_string(cfg_->UeAntNum()) + ".bin";
    AGORA_LOG_INFO("Saving UL tx data to %s\n", filename_tx.c_str());
    for (size_t i = 0; i < cfg_->Frame().NumULSyms(); i++) {
      const size_t sym_id = cfg_->Frame().GetULSymbol(i);
      Utils::WriteBinaryFile(filename_tx, sizeof(complex_float),
                             cfg_->OfdmCaNum() * cfg_->UeAntNum(),
                             tx_data_all_symbols[sym_id],
                             i != 0);  //Do not append in the first write
    }
  }

  // Generate CSI matrix
  Table<complex_float> csi_matrices;
  float sqrt2_norm = 1 / std::sqrt(2);
  csi_matrices.Calloc(cfg_->OfdmCaNum(), cfg_->UeAntNum() * cfg_->BsAntNum(),
                      Agora_memory::Alignment_t::kAlign32);
  for (size_t i = 0; i < (cfg_->UeAntNum() * cfg_->BsAntNum()); i++) {
    complex_float csi = {RandFloatFromShort(-1, 1), RandFloatFromShort(-1, 1)};
    for (size_t j = 0; j < cfg_->OfdmCaNum(); j++) {
      csi_matrices[j][i].re = csi.re * sqrt2_norm;
      csi_matrices[j][i].im = csi.im * sqrt2_norm;
    }
  }
  arma::arma_rng::set_seed_random();

  // Generate RX data received by base station after going through channels
  Table<complex_float> rx_data_all_symbols;
  rx_data_all_symbols.Calloc(cfg_->Frame().NumTotalSyms(),
                             cfg_->OfdmCaNum() * cfg_->BsAntNum(),
                             Agora_memory::Alignment_t::kAlign64);
  for (size_t i = 0; i < cfg_->Frame().NumTotalSyms(); i++) {
    arma::cx_fmat mat_input_data(
        reinterpret_cast<arma::cx_float*>(tx_data_all_symbols[i]),
        cfg_->OfdmCaNum(), cfg_->UeAntNum(), false);
    arma::cx_fmat mat_output(
        reinterpret_cast<arma::cx_float*>(rx_data_all_symbols[i]),
        cfg_->OfdmCaNum(), cfg_->BsAntNum(), false);

    for (size_t j = 0; j < cfg_->OfdmCaNum(); j++) {
      arma::cx_fmat mat_csi(reinterpret_cast<arma::cx_float*>(csi_matrices[j]),
                            cfg_->BsAntNum(), cfg_->UeAntNum(), false);
      mat_output.row(j) = mat_input_data.row(j) * mat_csi.st();
    }
    arma::cx_fmat noise_mat(size(mat_output));
    noise_mat.set_real(arma::randn<arma::fmat>(size(real(mat_output))));
    noise_mat.set_imag(arma::randn<arma::fmat>(size(real(mat_output))));
    mat_output += (noise_mat * cfg_->NoiseLevel() * sqrt2_norm);
    for (size_t j = 0; j < cfg_->BsAntNum(); j++) {
      auto* this_ofdm_symbol = rx_data_all_symbols[i] + j * cfg_->OfdmCaNum();
      CommsLib::FFTShift(this_ofdm_symbol, cfg_->OfdmCaNum());
      CommsLib::IFFT(this_ofdm_symbol, cfg_->OfdmCaNum(), false);
    }
  }

  const std::string filename_rx = directory + kRxLdpcPrefix +
                                  std::to_string(cfg_->OfdmCaNum()) + "_ant" +
                                  std::to_string(cfg_->BsAntNum()) + ".bin";
  AGORA_LOG_INFO("Saving rx data to %s\n", filename_rx.c_str());
  for (size_t i = 0; i < cfg_->Frame().NumTotalSyms(); i++) {
    Utils::WriteBinaryFile(filename_rx, sizeof(float),
                           cfg_->OfdmCaNum() * cfg_->BsAntNum() * 2,
                           reinterpret_cast<void*>(rx_data_all_symbols[i]),
                           i != 0);  //Do not append in the first write
  }

  if (kDebugPrintRxData) {
    std::printf("rx data\n");
    for (size_t i = 0; i < 10; i++) {
      for (size_t j = 0; j < cfg_->OfdmCaNum() * cfg_->BsAntNum(); j++) {
        if (j % cfg_->OfdmCaNum() == 0) {
          std::printf("\nsymbol %zu ant %zu\n", i, j / cfg_->OfdmCaNum());
        }
        std::printf("%.4f+%.4fi ", rx_data_all_symbols[i][j].re,
                    rx_data_all_symbols[i][j].im);
      }
      std::printf("\n");
    }
  }

  /* ------------------------------------------------
   * Generate data for downlink test
   * ------------------------------------------------ */
  const LDPCconfig dl_ldpc_config = cfg_->LdpcConfig(Direction::kDownlink);
  const size_t dl_cb_bytes = cfg_->NumBytesPerCb(Direction::kDownlink);

  if (cfg_->Frame().NumDLSyms() > 0) {
    const size_t num_dl_mac_bytes =
        cfg_->MacBytesNumPerframe(Direction::kDownlink);
    std::vector<std::vector<int8_t>> dl_mac_info(cfg_->UeAntNum());
    AGORA_LOG_SYMBOL("Total number of downlink MAC bytes: %zu\n",
                     num_dl_mac_bytes);
    for (size_t ue_id = 0; ue_id < cfg_->UeAntNum(); ue_id++) {
      dl_mac_info[ue_id].resize(num_dl_mac_bytes);
      for (size_t pkt_id = 0;
           pkt_id < cfg_->MacPacketsPerframe(Direction::kDownlink); pkt_id++) {
        size_t pkt_offset =
            pkt_id * cfg_->MacPacketLength(Direction::kDownlink);
        auto* pkt = reinterpret_cast<MacPacketPacked*>(
            &dl_mac_info.at(ue_id).at(pkt_offset));

        pkt->Set(0, pkt_id, ue_id,
                 cfg_->MacPayloadMaxLength(Direction::kDownlink));
        data_generator->GenMacData(pkt, ue_id);
        pkt->Crc((uint16_t)(crc_obj->CalculateCrc24(pkt->Data(),
                                                    cfg_->MacPayloadMaxLength(
                                                        Direction::kDownlink)) &
                            0xFFFF));
      }
    }

    {
      const std::string filename_input =
          directory + kDlDataPrefix + std::to_string(cfg_->OfdmCaNum()) +
          "_ant" + std::to_string(cfg_->UeAntNum()) + ".bin";
      AGORA_LOG_INFO("Saving downlink MAC data to %s\n",
                     filename_input.c_str());
      for (size_t i = 0; i < cfg_->UeAntNum(); i++) {
        Utils::WriteBinaryFile(filename_input, sizeof(uint8_t),
                               num_dl_mac_bytes, dl_mac_info.at(i).data(),
                               i != 0);  //Do not append in the first write
      }

      if (kPrintDownlinkInformationBytes) {
        std::printf("Downlink information bytes\n");
        for (size_t n = 0; n < cfg_->UeAntNum(); n++) {
          std::printf("UE %zu\n", n % cfg_->UeAntNum());
          for (size_t i = 0; i < num_dl_mac_bytes; i++) {
            std::printf("%u ", static_cast<uint8_t>(dl_mac_info.at(n).at(i)));
          }
          std::printf("\n");
        }
      }
    }

    const size_t symbol_blocks =
        dl_ldpc_config.NumBlocksInSymbol() * cfg_->UeAntNum();
    const size_t num_dl_codeblocks =
        cfg_->Frame().NumDlDataSyms() * symbol_blocks;
    AGORA_LOG_SYMBOL("Total number of dl data blocks: %zu\n",
                     num_dl_codeblocks);

    std::vector<std::vector<int8_t>> dl_information(num_dl_codeblocks);
    std::vector<std::vector<int8_t>> dl_encoded_codewords(num_dl_codeblocks);
    for (size_t cb = 0; cb < num_dl_codeblocks; cb++) {
      // i : symbol -> ue -> cb (repeat)
      const size_t sym_id = cb / (symbol_blocks);
      // ue antenna for code block
      const size_t sym_offset = cb % (symbol_blocks);
      const size_t ue_id = sym_offset / dl_ldpc_config.NumBlocksInSymbol();
      const size_t ue_cb_id = sym_offset % dl_ldpc_config.NumBlocksInSymbol();
      const size_t ue_cb_cnt =
          (sym_id * dl_ldpc_config.NumBlocksInSymbol()) + ue_cb_id;
      int8_t* cb_start = &dl_mac_info.at(ue_id).at(ue_cb_cnt * dl_cb_bytes);
      dl_information.at(cb) =
          std::vector<int8_t>(cb_start, cb_start + dl_cb_bytes);
      dl_encoded_codewords.at(cb) = DataGenerator::GenCodeblock(
          dl_ldpc_config, &dl_information.at(cb).at(0), dl_cb_bytes,
          cfg_->ScrambleEnabled());
    }

    // Modulate the encoded codewords
    std::vector<std::vector<complex_float>> dl_modulated_codewords(
        num_dl_codeblocks);
    for (size_t i = 0; i < num_dl_codeblocks; i++) {
      const size_t sym_offset = i % (symbol_blocks);
      const size_t ue_id = sym_offset / dl_ldpc_config.NumBlocksInSymbol();
      auto ofdm_symbol = DataGenerator::GetModulation(
          &dl_encoded_codewords.at(i)[0], cfg_->ModTable(Direction::kDownlink),
          cfg_->LdpcConfig(Direction::kDownlink).NumCbCodewLen(),
          cfg_->OfdmDataNum(), cfg_->ModOrderBits(Direction::kDownlink));
      dl_modulated_codewords.at(i) = DataGenerator::MapOFDMSymbol(
          cfg_.get(), ofdm_symbol, ue_specific_pilot[ue_id], SymbolType::kDL);
    }

    {
      // Save downlink information bytes to file
      const std::string filename_input =
          directory + kDlLdpcDataPrefix + std::to_string(cfg_->OfdmCaNum()) +
          "_ant" + std::to_string(cfg_->UeAntNum()) + ".bin";
      AGORA_LOG_INFO("Saving raw dl data (using LDPC) to %s\n",
                     filename_input.c_str());
      for (size_t i = 0; i < num_dl_codeblocks; i++) {
        Utils::WriteBinaryFile(filename_input, sizeof(uint8_t), dl_cb_bytes,
                               dl_information.at(i).data(),
                               i != 0);  //Do not append in the first write
      }

      if (kPrintDownlinkInformationBytes == true) {
        std::printf("Downlink information bytes\n");
        for (size_t n = 0; n < num_dl_codeblocks; n++) {
          std::printf("Symbol %zu, UE %zu\n", n / cfg_->UeAntNum(),
                      n % cfg_->UeAntNum());
          for (size_t i = 0; i < dl_cb_bytes; i++) {
            std::printf("%u ",
                        static_cast<unsigned>(dl_information.at(n).at(i)));
          }
          std::printf("\n");
        }
      }
    }

    // Compute precoder
    Table<complex_float> precoder;
    precoder.Calloc(cfg_->OfdmCaNum(), cfg_->UeAntNum() * cfg_->BsAntNum(),
                    Agora_memory::Alignment_t::kAlign32);
    for (size_t i = 0; i < cfg_->OfdmCaNum(); i++) {
      arma::cx_fmat mat_input(
          reinterpret_cast<arma::cx_float*>(csi_matrices[i]), cfg_->BsAntNum(),
          cfg_->UeAntNum(), false);
      arma::cx_fmat mat_output(reinterpret_cast<arma::cx_float*>(precoder[i]),
                               cfg_->UeAntNum(), cfg_->BsAntNum(), false);
      pinv(mat_output, mat_input, 1e-2, "dc");
    }

    if (kPrintDebugCSI) {
      std::printf("CSI \n");
      for (size_t j = 0; j < cfg_->UeAntNum() * cfg_->BsAntNum(); j++) {
        std::printf("%.3f+%.3fi ", csi_matrices[cfg_->OfdmDataStart()][j].re,
                    csi_matrices[cfg_->OfdmDataStart()][j].im);
      }
      std::printf("\nprecoder \n");
      for (size_t j = 0; j < cfg_->UeAntNum() * cfg_->BsAntNum(); j++) {
        std::printf("%.3f+%.3fi ", precoder[cfg_->OfdmDataStart()][j].re,
                    precoder[cfg_->OfdmDataStart()][j].im);
      }
      std::printf("\n");
    }

    // Prepare downlink data from mod_output
    Table<complex_float> dl_mod_data;
    dl_mod_data.Calloc(cfg_->Frame().NumDLSyms(),
                       cfg_->OfdmCaNum() * cfg_->UeAntNum(),
                       Agora_memory::Alignment_t::kAlign64);
    for (size_t i = 0; i < cfg_->Frame().NumDLSyms(); i++) {
      for (size_t j = 0; j < cfg_->UeAntNum(); j++) {
        for (size_t sc_id = 0; sc_id < cfg_->OfdmDataNum(); sc_id++) {
          complex_float sc_data;
          if ((i < cfg_->Frame().ClientDlPilotSymbols()) ||
              (sc_id % cfg_->OfdmPilotSpacing() == 0)) {
            sc_data = ue_specific_pilot[j][sc_id];
          } else {
            sc_data = dl_modulated_codewords
                          .at(((i - cfg_->Frame().ClientDlPilotSymbols()) *
                               cfg_->UeAntNum()) +
                              j)
                          .at(sc_id);
          }
          dl_mod_data[i][j * cfg_->OfdmCaNum() + sc_id +
                         cfg_->OfdmDataStart()] = sc_data;
        }
      }
    }

    if (kPrintDlModData) {
      std::printf("dl mod data \n");
      for (size_t i = 0; i < cfg_->Frame().NumDLSyms(); i++) {
        for (size_t k = cfg_->OfdmDataStart();
             k < cfg_->OfdmDataStart() + cfg_->OfdmDataNum(); k++) {
          std::printf("symbol %zu, subcarrier %zu\n", i, k);
          for (size_t j = 0; j < cfg_->UeAntNum(); j++) {
            std::printf("%.3f+%.3fi ",
                        dl_mod_data[i][j * cfg_->OfdmCaNum() + k].re,
                        dl_mod_data[i][j * cfg_->OfdmCaNum() + k].im);
          }
          std::printf("\n");
        }
      }
    }

    // Perform precoding and IFFT
    Table<complex_float> dl_ifft_data;
    dl_ifft_data.Calloc(cfg_->Frame().NumDLSyms(),
                        cfg_->OfdmCaNum() * cfg_->BsAntNum(),
                        Agora_memory::Alignment_t::kAlign64);
    Table<short> dl_tx_data;
    dl_tx_data.Calloc(cfg_->Frame().NumDLSyms(),
                      2 * cfg_->SampsPerSymbol() * cfg_->BsAntNum(),
                      Agora_memory::Alignment_t::kAlign64);

    for (size_t i = 0; i < cfg_->Frame().NumDLSyms(); i++) {
      arma::cx_fmat mat_input_data(
          reinterpret_cast<arma::cx_float*>(dl_mod_data[i]), cfg_->OfdmCaNum(),
          cfg_->UeAntNum(), false);

      arma::cx_fmat mat_output(
          reinterpret_cast<arma::cx_float*>(dl_ifft_data[i]), cfg_->OfdmCaNum(),
          cfg_->BsAntNum(), false);

      for (size_t j = cfg_->OfdmDataStart();
           j < cfg_->OfdmDataNum() + cfg_->OfdmDataStart(); j++) {
        arma::cx_fmat mat_precoder(
            reinterpret_cast<arma::cx_float*>(precoder[j]), cfg_->UeAntNum(),
            cfg_->BsAntNum(), false);
        mat_precoder /= abs(mat_precoder).max();
        mat_output.row(j) = mat_input_data.row(j) * mat_precoder;

        // std::printf("symbol %d, sc: %d\n", i, j -
        // cfg_->ofdm_data_start()); cout << "Precoder: \n" <<
        // mat_precoder
        // << endl; cout << "Data: \n" << mat_input_data.row(j) << endl; cout <<
        // "Precoded data: \n" << mat_output.row(j) << endl;
      }
      for (size_t j = 0; j < cfg_->BsAntNum(); j++) {
        complex_float* ptr_ifft = dl_ifft_data[i] + j * cfg_->OfdmCaNum();
        CommsLib::FFTShift(ptr_ifft, cfg_->OfdmCaNum());
        CommsLib::IFFT(ptr_ifft, cfg_->OfdmCaNum(), false);

        short* tx_symbol = dl_tx_data[i] + j * cfg_->SampsPerSymbol() * 2;
        std::memset(tx_symbol, 0, sizeof(short) * 2 * cfg_->OfdmTxZeroPrefix());
        for (size_t k = 0; k < cfg_->OfdmCaNum(); k++) {
          tx_symbol[2 * (k + cfg_->CpLen() + cfg_->OfdmTxZeroPrefix())] =
              static_cast<short>(kShrtFltConvFactor * ptr_ifft[k].re);
          tx_symbol[2 * (k + cfg_->CpLen() + cfg_->OfdmTxZeroPrefix()) + 1] =
              static_cast<short>(kShrtFltConvFactor * ptr_ifft[k].im);
        }
        for (size_t k = 0; k < (2 * cfg_->CpLen()); k++) {
          tx_symbol[2 * cfg_->OfdmTxZeroPrefix() + k] =
              tx_symbol[2 * (cfg_->OfdmTxZeroPrefix() + cfg_->OfdmCaNum()) + k];
        }

        const size_t tx_zero_postfix_offset =
            2 * (cfg_->OfdmTxZeroPrefix() + cfg_->CpLen() + cfg_->OfdmCaNum());
        std::memset(tx_symbol + tx_zero_postfix_offset, 0,
                    sizeof(short) * 2 * cfg_->OfdmTxZeroPostfix());
      }
    }

    {
      std::string filename_dl_tx = directory + kDlTxPrefix +
                                   std::to_string(cfg_->OfdmCaNum()) + "_ant" +
                                   std::to_string(cfg_->BsAntNum()) + ".bin";
      AGORA_LOG_INFO("Saving dl tx data to %s\n", filename_dl_tx.c_str());
      for (size_t i = 0; i < cfg_->Frame().NumDLSyms(); i++) {
        Utils::WriteBinaryFile(filename_dl_tx, sizeof(short),
                               cfg_->SampsPerSymbol() * cfg_->BsAntNum() * 2,
                               reinterpret_cast<void*>(dl_tx_data[i]),
                               i != 0);  //Do not append in the first write
      }

      if (kPrintDlTxData) {
        std::printf("rx data\n");
        for (size_t i = 0; i < 10; i++) {
          for (size_t j = 0; j < cfg_->OfdmCaNum() * cfg_->BsAntNum(); j++) {
            if (j % cfg_->OfdmCaNum() == 0) {
              std::printf("symbol %zu ant %zu\n", i, j / cfg_->OfdmCaNum());
            }
            // TODO keep and fix or remove
            // std::printf("%d+%di ", dl_tx_data[i][j], dl_tx_data[i][j]);
          }
        }
        std::printf("\n");
      }
    }
    /* Clean Up memory */
    dl_ifft_data.Free();
    dl_tx_data.Free();
    dl_mod_data.Free();
    precoder.Free();
  }

  csi_matrices.Free();
  tx_data_all_symbols.Free();
  rx_data_all_symbols.Free();
  ue_specific_pilot.Free();
  AGORA_LOG_SHUTDOWN();
  return 0;
}
