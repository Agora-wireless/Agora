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
#include "mac_utils.h"
#include "utils.h"
#include "version_config.h"

static constexpr bool kVerbose = false;
static constexpr bool kPrintDebugCSI = false;
static constexpr bool kDebugPrintRxData = false;
static constexpr bool kPrintDlTxData = false;
static constexpr bool kPrintDlModData = false;
static constexpr bool kPrintUplinkInformationBytes = false;
static constexpr bool kPrintDownlinkInformationBytes = false;
static constexpr bool kPrintFreqDomainSamples = false;

static constexpr bool kPrintUlRxData = false;
static constexpr bool kPrintUplinkMacBytes = false;
static constexpr bool kPrintDownlinkMacBytes = false;
static constexpr bool kPrintUplinkEncodedBytes = false;
static constexpr bool kPrintDownlinkEncodedBytes = false;

static constexpr bool kPrintUeSchedule = false;

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

static void GenerateTestVectors(Config* cfg, const std::string& profile_flag) {
  const std::string directory =
      TOSTRING(PROJECT_DIRECTORY) "/files/experiment/";
  AGORA_LOG_INIT();

  const DataGenerator::Profile profile =
      profile_flag == "123" ? DataGenerator::Profile::kProfile123
                            : DataGenerator::Profile::kRandom;
  std::unique_ptr<DataGenerator> data_generator =
      std::make_unique<DataGenerator>(cfg, 0 /* RNG seed */, profile);

  //Make sure the directory exists
  if (std::filesystem::is_directory(directory) == false) {
    std::filesystem::create_directory(directory);
  }
  srand(time(nullptr));

  // Generating an array of cfg->FramesToTest() * cfg->UeAntNum() elements
  // containing a bitmap of scheduled UEs across frames
  size_t n_sched = static_cast<size_t>(std::pow(2, cfg->UeAntNum())) - 1;
  // Set seed for the random number generator
  std::random_device rd;
  std::mt19937 gen(rd());

  // Define the binary distribution for bitmap of scheduled UEs
  std::uniform_int_distribution<> distribution(0, 1);
  std::vector<uint8_t> sched_ue_map;
  std::vector<size_t> sched_ue_set;  // a condensed and sorted set of schedules
  std::uniform_int_distribution<> mcs_distribution(10, 25);
  std::vector<uint8_t> sched_ul_mcs;
  std::vector<uint8_t> sched_dl_mcs;
  if (cfg->AdaptUes() == true) {
    sched_ue_map.resize(cfg->FramesToTest() * cfg->UeAntNum(), 1);
    for (size_t i = 0; i < cfg->FramesToTest(); ++i) {
      size_t max_ue_num = 0;
      size_t ue_sched_id = 0;
      for (size_t u = 0; u < cfg->UeAntNum(); ++u) {
        uint8_t val = distribution(gen);
        sched_ue_map[i * cfg->UeAntNum() + u] = val;
        max_ue_num += val;  // count the schedule UE
        ue_sched_id += static_cast<size_t>(val * std::pow(2, u));
      }
      sched_ul_mcs.push_back(mcs_distribution(gen));
      sched_dl_mcs.push_back(mcs_distribution(gen));
      // if no UE was scheduled in this frame, schedule UE 0
      if (max_ue_num == 0) {
        sched_ue_map[i * cfg->UeAntNum()] = 1;
        ue_sched_id = 1;  // schedule UE 0
      }
      if (sched_ue_set.empty()) {
        sched_ue_set.push_back(ue_sched_id);
      } else {
        // search for an existing schedule before inserting into a sorted list
        std::vector<size_t>::iterator it;
        for (it = sched_ue_set.begin(); it < sched_ue_set.end(); it++) {
          if (ue_sched_id == *it) {  // dont's push this to keep vector unique
            break;
          } else if (ue_sched_id > *it && (it + 1) == sched_ue_set.end()) {
            sched_ue_set.push_back(ue_sched_id);
            break;
          } else if (ue_sched_id < *it && it == sched_ue_set.begin()) {
            sched_ue_set.insert(it, ue_sched_id);
            break;
          } else if (ue_sched_id > *it && ue_sched_id < *(it + 1)) {
            sched_ue_set.insert(it + 1, ue_sched_id);
            break;
          }
        }
      }
    }
    const std::string filename_sched = directory + kUeSchedulePrefix +
                                       std::to_string(cfg->UeAntNum()) +
                                       "ue.bin";
    AGORA_LOG_INFO("Saving scheduled number of UEs across frames to %s\n",
                   filename_sched.c_str());
    Utils::WriteBinaryFile(filename_sched, sizeof(uint8_t), sched_ue_map.size(),
                           sched_ue_map.data(), false);
    if (kPrintUeSchedule) {
      for (size_t i = 0; i < cfg->FramesToTest(); i++) {
        std::printf("Scheduled UEs at frame %zu:\n", i);
        for (size_t u = 0; u < cfg->UeAntNum(); u++) {
          std::printf("%u ", sched_ue_map.at(i * cfg->UeAntNum() + u));
        }
        std::printf("\n");
      }
    }
    const std::string filename_ul_mcs = directory + kUeSchedulePrefix +
                                        std::to_string(cfg->UeAntNum()) +
                                        "ue_ul_mcs.bin";
    Utils::WriteBinaryFile(filename_ul_mcs, sizeof(uint8_t),
                           sched_ul_mcs.size(), sched_ul_mcs.data(), false);
    const std::string filename_dl_mcs = directory + kUeSchedulePrefix +
                                        std::to_string(cfg->UeAntNum()) +
                                        "ue_dl_mcs.bin";
    Utils::WriteBinaryFile(filename_dl_mcs, sizeof(uint8_t),
                           sched_dl_mcs.size(), sched_dl_mcs.data(), false);
  } else {
    sched_ue_set.push_back(n_sched);
    sched_ul_mcs.push_back(cfg->MacParams().McsIndex(Direction::kUplink));
    sched_dl_mcs.push_back(cfg->MacParams().McsIndex(Direction::kDownlink));
  }

  AGORA_LOG_INFO("Generating data for %zu frame schedules\n",
                 sched_ue_set.size());

  // Step 1: Generate and Populate Uplink Pilots
  // Generate common sounding pilots
  std::vector<complex_float> pilot_fd =
      data_generator->GetCommonPilotFreqDomain();

  // Generate UE-specific pilots (phase tracking & downlink channel estimation)
  Table<complex_float> ue_specific_pilot =
      data_generator->GetUeSpecificPilotFreqDomain();

  Table<complex_float> tx_data_all_symbols;
  tx_data_all_symbols.Calloc(cfg->Frame().NumTotalSyms(),
                             cfg->UeAntNum() * cfg->OfdmCaNum(),
                             Agora_memory::Alignment_t::kAlign64);
  Table<complex_float> rx_data_all_symbols;
  rx_data_all_symbols.Calloc(cfg->Frame().NumTotalSyms(),
                             cfg->OfdmCaNum() * cfg->BsAntNum(),
                             Agora_memory::Alignment_t::kAlign64);
  auto* rx_data_temp =
      static_cast<std::complex<short>*>(Agora_memory::PaddedAlignedAlloc(
          Agora_memory::Alignment_t::kAlign64,
          cfg->OfdmCaNum() * cfg->BsAntNum() * sizeof(short) * 2));

  Table<complex_float> precoder;
  precoder.Calloc(cfg->OfdmCaNum(), cfg->UeAntNum() * cfg->BsAntNum(),
                  Agora_memory::Alignment_t::kAlign32);

  Table<complex_float> dl_mod_data;
  Table<complex_float> dl_ifft_data;
  Table<short> dl_tx_data;
  if (cfg->Frame().NumDLSyms() != 0u) {
    dl_mod_data.Calloc(cfg->Frame().NumDLSyms(),
                       cfg->OfdmCaNum() * cfg->UeAntNum(),
                       Agora_memory::Alignment_t::kAlign64);
    dl_ifft_data.Calloc(cfg->Frame().NumDLSyms(),
                        cfg->OfdmCaNum() * cfg->BsAntNum(),
                        Agora_memory::Alignment_t::kAlign64);
    dl_tx_data.Calloc(cfg->Frame().NumDLSyms(),
                      2 * cfg->SampsPerSymbol() * cfg->BsAntNum(),
                      Agora_memory::Alignment_t::kAlign64);
  }

  // Populate pilot symbols
  AGORA_LOG_INFO("DataGenerator: Using %s-orthogonal pilots\n",
                 cfg->FreqOrthogonalPilot() ? "frequency" : "time");
  if (cfg->FreqOrthogonalPilot()) {
    const size_t pilot_sym_idx = cfg->Frame().GetPilotSymbol(0);
    RtAssert(cfg->Frame().NumPilotSyms() == 1,
             "Number of pilot symbols must be 1");
    for (size_t i = 0; i < cfg->UeAntNum(); i++) {
      std::vector<complex_float> pilots_f_ue(cfg->OfdmCaNum());  // Zeroed
      for (size_t j = cfg->OfdmDataStart(); j < cfg->OfdmDataStop();
           j += cfg->PilotScGroupSize()) {
        pilots_f_ue.at(i + j) = pilot_fd.at(i + j);
      }
      // Load pilots
      std::memcpy(tx_data_all_symbols[pilot_sym_idx] + (i * cfg->OfdmCaNum()),
                  &pilots_f_ue.at(0),
                  (cfg->OfdmCaNum() * sizeof(complex_float)));
    }
  } else {
    for (size_t i = 0; i < cfg->UeAntNum(); i++) {
      const size_t pilot_sym_idx = cfg->Frame().GetPilotSymbol(i);
      std::memcpy(tx_data_all_symbols[pilot_sym_idx] + i * cfg->OfdmCaNum(),
                  &pilot_fd.at(0), (cfg->OfdmCaNum() * sizeof(complex_float)));
    }
  }

  // Generate CSI matrix to be used for both uplink and downlink
  Table<complex_float> csi_matrices;
  float sqrt2_norm = 1 / std::sqrt(2);
  csi_matrices.Calloc(cfg->OfdmCaNum(), cfg->UeAntNum() * cfg->BsAntNum(),
                      Agora_memory::Alignment_t::kAlign32);
  for (size_t i = 0; i < (cfg->UeAntNum() * cfg->BsAntNum()); i++) {
    complex_float csi = {RandFloatFromShort(-1, 1), RandFloatFromShort(-1, 1)};
    for (size_t j = 0; j < cfg->OfdmCaNum(); j++) {
      csi_matrices[j][i].re = csi.re * sqrt2_norm;
      csi_matrices[j][i].im = csi.im * sqrt2_norm;
    }
  }
  arma::arma_rng::set_seed_random();
  std::unique_ptr<DoCRC> crc_obj = std::make_unique<DoCRC>();
  MacUtils mac_params(cfg->Frame(), cfg->GetFrameDurationSec(),
                      cfg->OfdmDataNum(), cfg->GetOFDMDataNum(),
                      cfg->GetOFDMCtrlNum());
  mac_params.UpdateUlMcsParams(cfg->MacParams().McsIndex(Direction::kUplink));
  mac_params.UpdateDlMcsParams(cfg->MacParams().McsIndex(Direction::kDownlink));

  /* ------------------------------------------------
   * Generate data for uplink test
   * ------------------------------------------------ */
  AGORA_LOG_INFO("DataGenerator: Generating encoded and modulated data\n");
  const size_t num_ul_pilots = cfg->Frame().ClientUlPilotSymbols();
  const size_t ul_pkt_per_frame =
      mac_params.MacPacketsPerframe(Direction::kUplink);
  const size_t num_ul_max_bytes =
      mac_params.MaxPacketBytes(Direction::kUplink) * ul_pkt_per_frame;
  const size_t dl_pkt_per_frame =
      mac_params.MacPacketsPerframe(Direction::kDownlink);
  const size_t num_dl_max_bytes =
      mac_params.MaxPacketBytes(Direction::kDownlink) * dl_pkt_per_frame;
  for (size_t sched = 0; sched < sched_ue_set.size(); sched++) {
    auto sched_id = sched_ue_set.at(sched);
    auto ue_map = Utils::Int2Bits(sched_id, cfg->UeAntNum());
    AGORA_LOG_INFO(
        "Frame Schedule %zu: Generating data for %zu UEs with UL MCS %zu & DL "
        "MCS %zu\n",
        sched, ue_map.n_elem, sched_ul_mcs.at(sched), sched_dl_mcs.at(sched));
    if (kPrintUeSchedule) {
      std::cout << ue_map << std::endl;
    }
    // Generate the information buffers (MAC Packets) and LDPC-encoded buffers
    if (cfg->Frame().NumULSyms() != 0) {
      if (cfg->AdaptUes()) {
        mac_params.UpdateUlMcsParams(sched_ul_mcs.at(sched));
      }
      const size_t ul_cb_bytes = mac_params.NumBytesPerCb(Direction::kUplink);
      LDPCconfig ul_ldpc_config = mac_params.LdpcConfig(Direction::kUplink);
      const size_t num_ul_mac_bytes =
          mac_params.MacBytesNumPerframe(Direction::kUplink);
      std::vector<std::vector<int8_t>> ul_mac_info(cfg->UeAntNum());
      AGORA_LOG_FRAME("Total number of uplink MAC bytes: %zu\n",
                      num_ul_mac_bytes);
      for (size_t ue_id = 0; ue_id < cfg->UeAntNum(); ue_id++) {
        ul_mac_info.at(ue_id).resize(num_ul_mac_bytes);
        for (size_t pkt_id = 0; pkt_id < ul_pkt_per_frame; pkt_id++) {
          size_t pkt_offset =
              pkt_id * mac_params.MacPacketLength(Direction::kUplink);
          auto* pkt = reinterpret_cast<MacPacketPacked*>(
              &ul_mac_info.at(ue_id).at(pkt_offset));

          pkt->Set(0, cfg->Frame().GetULSymbol(pkt_id + num_ul_pilots), ue_id,
                   mac_params.MacPayloadMaxLength(Direction::kUplink));
          data_generator->GenMacData(pkt, ue_id);
          pkt->Crc((uint16_t)(crc_obj->CalculateCrc24(
                                  pkt->Data(), mac_params.MacPayloadMaxLength(
                                                   Direction::kUplink)) &
                              0xFFFF));
        }
      }

      if (kPrintUplinkInformationBytes) {
        std::printf("Uplink information bytes\n");
        for (size_t n = 0; n < cfg->UeAntNum(); n++) {
          std::printf("UE %zu\n", n % cfg->UeAntNum());
          for (size_t i = 0; i < num_ul_mac_bytes; i++) {
            std::printf("%u ", static_cast<uint8_t>(ul_mac_info.at(n).at(i)));
          }
          std::printf("\n");
        }
      }

      const size_t symbol_blocks =
          ul_ldpc_config.NumBlocksInSymbol() * cfg->UeAntNum();
      const size_t num_ul_codeblocks =
          cfg->Frame().NumUlDataSyms() * symbol_blocks;
      AGORA_LOG_FRAME("Total number of ul blocks: %zu\n", num_ul_codeblocks);

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
            cb, ue_id, ue_cb_id, ue_cb_cnt, ul_cb_bytes,
            ue_cb_cnt * ul_cb_bytes, ul_mac_info.at(ue_id).size());
        int8_t* cb_start = &ul_mac_info.at(ue_id).at(ue_cb_cnt * ul_cb_bytes);
        ul_information.at(cb) =
            std::vector<int8_t>(cb_start, cb_start + ul_cb_bytes);
        ul_encoded_codewords.at(cb) = DataGenerator::GenCodeblock(
            ul_ldpc_config, &ul_information.at(cb).at(0), ul_cb_bytes,
            cfg->ScrambleEnabled());
      }

      if (kPrintUplinkInformationBytes) {
        std::printf("Uplink Information Bytes\n");
        for (size_t n = 0; n < num_ul_codeblocks; n++) {
          std::printf("Symbol %zu, UE %zu\n", n / cfg->UeAntNum(),
                      n % cfg->UeAntNum());
          for (size_t i = 0; i < ul_cb_bytes; i++) {
            std::printf("%u ",
                        static_cast<uint8_t>(ul_information.at(n).at(i)));
          }
          std::printf("\n");
        }
      }

      if (kOutputUlScData) {
        DataGenerator::WriteUlScDataToFile(cfg, directory,
                                           ul_encoded_codewords);
      }

      // Modulate the encoded codewords
      std::vector<std::vector<uint8_t>> ul_modulated_codewords(
          num_ul_codeblocks);
      std::vector<std::vector<complex_float>> ul_modulated_symbols(
          num_ul_codeblocks);
      for (size_t i = 0; i < num_ul_codeblocks; i++) {
        ul_modulated_codewords.at(i).resize(cfg->OfdmDataNum(), 0);
        auto ofdm_symbol = DataGenerator::GetModulation(
            &ul_encoded_codewords.at(i).at(0),
            &ul_modulated_codewords.at(i).at(0),
            mac_params.ModTable(Direction::kUplink),
            mac_params.LdpcConfig(Direction::kUplink).NumCbCodewLen(),
            cfg->OfdmDataNum(), mac_params.ModOrderBits(Direction::kUplink));
        ul_modulated_symbols.at(i) = DataGenerator::MapOFDMSymbol(
            cfg, ofdm_symbol, nullptr, SymbolType::kUL);
      }

      // Place modulated uplink data codewords into central IFFT bins
      RtAssert(ul_ldpc_config.NumBlocksInSymbol() == 1);  // TODO: Assumption
      std::vector<std::vector<complex_float>> pre_ifft_data_syms;
      pre_ifft_data_syms.resize(cfg->UeAntNum() * cfg->Frame().NumUlDataSyms());
      for (size_t i = 0; i < pre_ifft_data_syms.size(); i++) {
        pre_ifft_data_syms.at(i) =
            DataGenerator::BinForIfft(cfg, ul_modulated_symbols.at(i));
      }

      {
        if (kPrintFreqDomainSamples) {
          std::printf("Uplink Frequency-Domain Samples\n");
          for (size_t n = 0; n < num_ul_codeblocks; n++) {
            std::printf("Symbol %zu, UE %zu\n", n / cfg->UeAntNum(),
                        n % cfg->UeAntNum());
            for (size_t i = 0; i < cfg->OfdmCaNum(); i++) {
              complex_float iq_s = pre_ifft_data_syms.at(n).at(i);
              std::printf("%.4f+%.4fi, ", iq_s.re, iq_s.im);
            }
            std::printf("\n");
          }
        }
      }

      {
        // Save uplink information bytes to file
        const std::string filename_ldpc =
            directory + kUlLdpcDataPrefix + std::to_string(cfg->OfdmCaNum()) +
            "_ue" + std::to_string(cfg->UeAntNum()) + ".bin";
        AGORA_LOG_INFO("Saving uplink data bits (encoder input) to %s\n",
                       filename_ldpc.c_str());
        std::vector<uint8_t> ul_padding(num_ul_max_bytes - num_ul_mac_bytes);
        for (size_t i = 0; i < cfg->UeAntNum(); i++) {
          Utils::WriteBinaryFile(
              filename_ldpc, sizeof(uint8_t), num_ul_mac_bytes,
              ul_mac_info.at(i).data(),
              i != 0 || sched != 0);  //Do not append in the first write
          // Padd each write with zeros up to max possible bytes (higher MCS)
          Utils::WriteBinaryFile(filename_ldpc, sizeof(uint8_t),
                                 ul_padding.size(), ul_padding.data(), true);
        }

        const std::string filename_modul =
            directory + kUlModDataPrefix + std::to_string(cfg->OfdmCaNum()) +
            "_ue" + std::to_string(cfg->UeAntNum()) + ".bin";
        AGORA_LOG_INFO("Saving uplink encoded data bits to %s\n",
                       filename_modul.c_str());
        for (size_t i = 0; i < num_ul_codeblocks; i++) {
          Utils::WriteBinaryFile(
              filename_modul, sizeof(uint8_t), cfg->OfdmDataNum(),
              ul_modulated_codewords.at(i).data(),
              i != 0 || sched != 0);  //Do not append in the first write
        }

        const std::string filename_tx =
            directory + kUlIfftPrefix + std::to_string(cfg->OfdmCaNum()) +
            "_ue" + std::to_string(cfg->UeAntNum()) + ".bin";
        AGORA_LOG_INFO("Saving uplink mapped ofdm data to %s\n",
                       filename_tx.c_str());
        for (size_t i = 0; i < cfg->UeAntNum() * cfg->Frame().NumUlDataSyms();
             i++) {
          Utils::WriteBinaryFile(
              filename_tx, sizeof(complex_float), cfg->OfdmCaNum(),
              pre_ifft_data_syms.at(i).data(),
              i != 0 || sched != 0);  //Do not append in the first write
        }
      }
      // Populate the UL symbols
      for (size_t i = 0; i < cfg->Frame().NumULSyms(); i++) {
        const size_t sym_id = cfg->Frame().GetULSymbol(i);
        for (size_t j = 0; j < cfg->UeAntNum(); j++) {
          if (i < cfg->Frame().ClientUlPilotSymbols()) {
            std::memcpy(tx_data_all_symbols[sym_id] + (j * cfg->OfdmCaNum()) +
                            cfg->OfdmDataStart(),
                        ue_specific_pilot[j],
                        cfg->OfdmDataNum() * sizeof(complex_float));
          } else {
            const size_t k = i - cfg->Frame().ClientUlPilotSymbols();
            std::memcpy(tx_data_all_symbols[sym_id] + (j * cfg->OfdmCaNum()),
                        &pre_ifft_data_syms.at(k * cfg->UeAntNum() + j).at(0),
                        cfg->OfdmCaNum() * sizeof(complex_float));
          }
        }
      }
    }

    // Generate RX data received by base station after going through channels
    const std::string filename_rx =
        directory + kUlRxPrefix + std::to_string(cfg->OfdmCaNum()) + "_bsant" +
        std::to_string(cfg->BsAntNum()) + "_ueant" +
        std::to_string(cfg->UeAntNum()) + ".bin";
    AGORA_LOG_INFO("Saving uplink rx samples to %s\n", filename_rx.c_str());
    auto ue_map_mat = arma::repmat(ue_map, cfg->BsAntNum(), 1);
    for (size_t i = 0; i < cfg->Frame().NumTotalSyms(); i++) {
      arma::cx_fmat mat_input_data(
          reinterpret_cast<arma::cx_float*>(tx_data_all_symbols[i]),
          cfg->OfdmCaNum(), cfg->UeAntNum(), false);
      arma::cx_fmat mat_output(
          reinterpret_cast<arma::cx_float*>(rx_data_all_symbols[i]),
          cfg->OfdmCaNum(), cfg->BsAntNum(), false);

      for (size_t j = 0; j < cfg->OfdmCaNum(); j++) {
        arma::cx_fmat mat_csi(
            reinterpret_cast<arma::cx_float*>(csi_matrices[j]), cfg->BsAntNum(),
            cfg->UeAntNum(), false);
        mat_output.row(j) =
            (mat_input_data.row(j) % ue_map) * (mat_csi % ue_map_mat).st();
      }
      arma::cx_fmat noise_mat(size(mat_output));
      noise_mat.set_real(arma::randn<arma::fmat>(size(real(mat_output))));
      noise_mat.set_imag(arma::randn<arma::fmat>(size(real(mat_output))));
      mat_output += (noise_mat * cfg->NoiseLevel() * sqrt2_norm);
      for (size_t j = 0; j < cfg->BsAntNum(); j++) {
        auto* this_ofdm_symbol = rx_data_all_symbols[i] + j * cfg->OfdmCaNum();
        CommsLib::FFTShift(this_ofdm_symbol, cfg->OfdmCaNum());
        CommsLib::IFFT(this_ofdm_symbol, cfg->OfdmCaNum(), false);
      }
      SimdConvertFloatToShort(reinterpret_cast<float*>(rx_data_all_symbols[i]),
                              reinterpret_cast<short*>(rx_data_temp),
                              2 * cfg->OfdmCaNum() * cfg->BsAntNum());
      Utils::WriteBinaryFile(
          filename_rx, sizeof(short), cfg->OfdmCaNum() * cfg->BsAntNum() * 2,
          rx_data_temp,
          i != 0 || sched != 0);  //Do not append in the first write
    }

    if (kDebugPrintRxData) {
      std::printf("For %zu ue(s), rx data\n", sched_id);
      for (size_t i = 0; i < 10; i++) {
        for (size_t j = 0; j < cfg->OfdmCaNum() * cfg->BsAntNum(); j++) {
          if (j % cfg->OfdmCaNum() == 0) {
            std::printf("\nsymbol %zu ant %zu\n", i, j / cfg->OfdmCaNum());
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
    if (cfg->Frame().NumDLSyms() != 0) {
      if (cfg->AdaptUes()) {
        mac_params.UpdateDlMcsParams(sched_dl_mcs.at(sched));
      }
      const LDPCconfig dl_ldpc_config =
          mac_params.LdpcConfig(Direction::kDownlink);
      const size_t dl_cb_bytes = mac_params.NumBytesPerCb(Direction::kDownlink);
      const size_t num_dl_mac_bytes =
          mac_params.MacBytesNumPerframe(Direction::kDownlink);
      const size_t num_dl_pilots = cfg->Frame().ClientDlPilotSymbols();
      std::vector<std::vector<int8_t>> dl_mac_info(cfg->UeAntNum());
      AGORA_LOG_FRAME("Total number of downlink MAC bytes: %zu\n",
                      num_dl_mac_bytes);
      for (size_t ue_id = 0; ue_id < cfg->UeAntNum(); ue_id++) {
        dl_mac_info[ue_id].resize(num_dl_mac_bytes);
        for (size_t pkt_id = 0;
             pkt_id < mac_params.MacPacketsPerframe(Direction::kDownlink);
             pkt_id++) {
          size_t pkt_offset =
              pkt_id * mac_params.MacPacketLength(Direction::kDownlink);
          auto* pkt = reinterpret_cast<MacPacketPacked*>(
              &dl_mac_info.at(ue_id).at(pkt_offset));

          pkt->Set(0, cfg->Frame().GetDLSymbol(pkt_id + num_dl_pilots), ue_id,
                   mac_params.MacPayloadMaxLength(Direction::kDownlink));
          data_generator->GenMacData(pkt, ue_id);
          pkt->Crc((uint16_t)(crc_obj->CalculateCrc24(
                                  pkt->Data(), mac_params.MacPayloadMaxLength(
                                                   Direction::kDownlink)) &
                              0xFFFF));
        }
      }

      {
        if (kPrintDownlinkInformationBytes) {
          std::printf("Downlink information bytes\n");
          for (size_t n = 0; n < cfg->UeAntNum(); n++) {
            std::printf("UE %zu\n", n % cfg->UeAntNum());
            for (size_t i = 0; i < num_dl_mac_bytes; i++) {
              std::printf("%u ", static_cast<uint8_t>(dl_mac_info.at(n).at(i)));
            }
            std::printf("\n");
          }
        }
      }

      const size_t symbol_blocks =
          dl_ldpc_config.NumBlocksInSymbol() * cfg->UeAntNum();
      const size_t num_dl_codeblocks =
          cfg->Frame().NumDlDataSyms() * symbol_blocks;
      AGORA_LOG_FRAME("Total number of dl data blocks: %zu\n",
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
            cfg->ScrambleEnabled());
      }

      if (kPrintDownlinkInformationBytes == true) {
        std::printf("Downlink information bytes\n");
        for (size_t n = 0; n < num_dl_codeblocks; n++) {
          std::printf("Symbol %zu, UE %zu\n", n / cfg->UeAntNum(),
                      n % cfg->UeAntNum());
          for (size_t i = 0; i < dl_cb_bytes; i++) {
            std::printf("%u ",
                        static_cast<unsigned>(dl_information.at(n).at(i)));
          }
          std::printf("\n");
        }
      }

      // Modulate the encoded codewords
      std::vector<std::vector<uint8_t>> dl_modulated_codewords(
          num_dl_codeblocks);
      std::vector<std::vector<complex_float>> dl_modulated_symbols(
          num_dl_codeblocks);
      for (size_t i = 0; i < num_dl_codeblocks; i++) {
        const size_t sym_offset = i % (symbol_blocks);
        const size_t ue_id = sym_offset / dl_ldpc_config.NumBlocksInSymbol();
        dl_modulated_codewords.at(i).resize(cfg->GetOFDMDataNum());
        auto ofdm_symbol = DataGenerator::GetModulation(
            &dl_encoded_codewords.at(i)[0], &dl_modulated_codewords.at(i)[0],
            mac_params.ModTable(Direction::kDownlink),
            mac_params.LdpcConfig(Direction::kDownlink).NumCbCodewLen(),
            cfg->GetOFDMDataNum(),
            mac_params.ModOrderBits(Direction::kDownlink));
        dl_modulated_symbols.at(i) = DataGenerator::MapOFDMSymbol(
            cfg, ofdm_symbol, ue_specific_pilot[ue_id], SymbolType::kDL);
      }

      // Non-beamformed version of downlink data
      std::vector<std::vector<complex_float>> pre_ifft_dl_data_syms(
          cfg->UeAntNum() * cfg->Frame().NumDlDataSyms());
      for (size_t i = 0; i < pre_ifft_dl_data_syms.size(); i++) {
        pre_ifft_dl_data_syms.at(i) =
            DataGenerator::BinForIfft(cfg, dl_modulated_symbols.at(i));
      }

      {
        // Save downlink information bytes to file
        const std::string filename_ldpc =
            directory + kDlLdpcDataPrefix + std::to_string(cfg->OfdmCaNum()) +
            "_ue" + std::to_string(cfg->UeAntNum()) + ".bin";
        AGORA_LOG_INFO("Saving downlink data bits (encoder input) to %s\n",
                       filename_ldpc.c_str());
        std::vector<uint8_t> dl_padding(num_dl_max_bytes - num_dl_mac_bytes);
        for (size_t i = 0; i < cfg->UeAntNum(); i++) {
          Utils::WriteBinaryFile(
              filename_ldpc, sizeof(uint8_t), num_dl_mac_bytes,
              dl_mac_info.at(i).data(),
              i != 0 || sched != 0);  //Do not append in the first write
          Utils::WriteBinaryFile(filename_ldpc, sizeof(uint8_t),
                                 dl_padding.size(), dl_padding.data(), true);
        }

        const std::string filename_modul =
            directory + kDlModDataPrefix + std::to_string(cfg->OfdmCaNum()) +
            "_ue" + std::to_string(cfg->UeAntNum()) + ".bin";
        AGORA_LOG_INFO("Saving downlink encoded data bits to %s\n",
                       filename_modul.c_str());
        for (size_t i = 0; i < num_dl_codeblocks; i++) {
          Utils::WriteBinaryFile(
              filename_modul, sizeof(uint8_t), cfg->GetOFDMDataNum(),
              dl_modulated_codewords.at(i).data(),
              i != 0 || sched != 0);  //Do not append in the first write
        }

        const std::string filename_tx =
            directory + kDlIfftPrefix + std::to_string(cfg->OfdmCaNum()) +
            "_ue" + std::to_string(cfg->UeAntNum()) + ".bin";
        AGORA_LOG_INFO("Saving downlink mapped ofdm data to %s\n",
                       filename_tx.c_str());
        for (size_t i = 0; i < cfg->UeAntNum() * cfg->Frame().NumDlDataSyms();
             i++) {
          Utils::WriteBinaryFile(
              filename_tx, sizeof(complex_float), cfg->OfdmCaNum(),
              pre_ifft_dl_data_syms[i].data(),
              i != 0 || sched != 0);  //Do not append in the first write
        }
      }

      // Prepare downlink data from mod_output
      for (size_t i = 0; i < cfg->Frame().NumDLSyms(); i++) {
        for (size_t j = 0; j < cfg->UeAntNum(); j++) {
          void* dst_ptr =
              &dl_mod_data[i][j * cfg->OfdmCaNum() + cfg->OfdmDataStart()];
          if (i < cfg->Frame().ClientDlPilotSymbols()) {
            std::memcpy(dst_ptr, ue_specific_pilot[j],
                        cfg->OfdmDataNum() * sizeof(complex_float));
          } else {
            size_t data_sym_id = i - cfg->Frame().ClientDlPilotSymbols();
            std::memcpy(
                dst_ptr,
                dl_modulated_symbols.at(data_sym_id * cfg->UeAntNum() + j)
                    .data(),
                cfg->OfdmDataNum() * sizeof(complex_float));
          }
        }
      }

      if (kPrintDlModData) {
        std::printf("dl mod data \n");
        for (size_t i = 0; i < cfg->Frame().NumDLSyms(); i++) {
          for (size_t k = cfg->OfdmDataStart();
               k < cfg->OfdmDataStart() + cfg->OfdmDataNum(); k++) {
            std::printf("symbol %zu, subcarrier %zu\n", i, k);
            for (size_t j = 0; j < cfg->UeAntNum(); j++) {
              std::printf("%.3f+%.3fi ",
                          dl_mod_data[i][j * cfg->OfdmCaNum() + k].re,
                          dl_mod_data[i][j * cfg->OfdmCaNum() + k].im);
            }
            std::printf("\n");
          }
        }
      }

      // Perform precoding and IFFT
      std::string filename_dl_tx = directory + kDlTxPrefix +
                                   std::to_string(cfg->OfdmCaNum()) + "_bsant" +
                                   std::to_string(cfg->BsAntNum()) + "_ueant" +
                                   std::to_string(cfg->UeAntNum()) + ".bin";
      AGORA_LOG_INFO("Saving downlink tx data to %s\n", filename_dl_tx.c_str());
      auto sched_ues = Utils::BitOneIndices(sched_id, cfg->UeAntNum());
      for (size_t i = 0; i < cfg->Frame().NumDLSyms(); i++) {
        arma::cx_fmat mat_input_data(cfg->OfdmCaNum(), sched_ues.n_elem,
                                     arma::fill::zeros);
        for (size_t u = 0; u < sched_ues.n_elem; u++) {
          arma::cx_fvec ue_data(
              reinterpret_cast<arma::cx_float*>(
                  dl_mod_data[i] + cfg->OfdmCaNum() * sched_ues(u)),
              cfg->OfdmCaNum(), false);
          mat_input_data.col(u) = ue_data;
        }

        arma::cx_fmat mat_output(
            reinterpret_cast<arma::cx_float*>(dl_ifft_data[i]),
            cfg->OfdmCaNum(), cfg->BsAntNum(), false);

        for (size_t j = cfg->OfdmDataStart();
             j < cfg->OfdmDataNum() + cfg->OfdmDataStart(); j++) {
          arma::cx_fmat mat_csi(
              reinterpret_cast<arma::cx_float*>(csi_matrices[j]),
              cfg->BsAntNum(), cfg->UeAntNum(), false);
          // Compute precoder
          arma::cx_fmat mat_precoder(
              reinterpret_cast<arma::cx_float*>(precoder[j]), sched_ues.n_elem,
              cfg->BsAntNum(), false);
          pinv(mat_precoder, mat_csi.cols(sched_ues), 1e-2, "dc");
          mat_precoder /= abs(mat_precoder).max();
          mat_output.row(j) = mat_input_data.row(j) * mat_precoder;
        }
        if (kPrintDebugCSI) {
          std::printf("CSI \n");
          for (size_t j = 0; j < cfg->UeAntNum() * cfg->BsAntNum(); j++) {
            std::printf("%.3f+%.3fi ", csi_matrices[cfg->OfdmDataStart()][j].re,
                        csi_matrices[cfg->OfdmDataStart()][j].im);
          }
          std::printf("\nprecoder \n");
          for (size_t j = 0; j < cfg->UeAntNum() * cfg->BsAntNum(); j++) {
            std::printf("%.3f+%.3fi ", precoder[cfg->OfdmDataStart()][j].re,
                        precoder[cfg->OfdmDataStart()][j].im);
          }
          std::printf("\n");
        }
        for (size_t j = 0; j < cfg->BsAntNum(); j++) {
          complex_float* ptr_ifft = dl_ifft_data[i] + j * cfg->OfdmCaNum();
          CommsLib::FFTShift(ptr_ifft, cfg->OfdmCaNum());
          CommsLib::IFFT(ptr_ifft, cfg->OfdmCaNum(), false);

          short* tx_symbol = dl_tx_data[i] + j * cfg->SampsPerSymbol() * 2;
          std::memset(tx_symbol, 0,
                      sizeof(short) * 2 * cfg->OfdmTxZeroPrefix());
          for (size_t k = 0; k < cfg->OfdmCaNum(); k++) {
            tx_symbol[2 * (k + cfg->CpLen() + cfg->OfdmTxZeroPrefix())] =
                static_cast<short>(kShrtFltConvFactor * ptr_ifft[k].re);
            tx_symbol[2 * (k + cfg->CpLen() + cfg->OfdmTxZeroPrefix()) + 1] =
                static_cast<short>(kShrtFltConvFactor * ptr_ifft[k].im);
          }
          for (size_t k = 0; k < (2 * cfg->CpLen()); k++) {
            tx_symbol[2 * cfg->OfdmTxZeroPrefix() + k] =
                tx_symbol[2 * (cfg->OfdmTxZeroPrefix() + cfg->OfdmCaNum()) + k];
          }

          const size_t tx_zero_postfix_offset =
              2 * (cfg->OfdmTxZeroPrefix() + cfg->CpLen() + cfg->OfdmCaNum());
          std::memset(tx_symbol + tx_zero_postfix_offset, 0,
                      sizeof(short) * 2 * cfg->OfdmTxZeroPostfix());
        }
        Utils::WriteBinaryFile(
            filename_dl_tx, sizeof(short),
            cfg->SampsPerSymbol() * cfg->BsAntNum() * 2,
            reinterpret_cast<void*>(dl_tx_data[i]),
            i != 0 || sched != 0);  //Do not append in the first write
      }
      if (kPrintDlTxData) {
        std::printf("dl tx data\n");
        for (size_t i = 0; i < 1; i++) {
          for (size_t j = 0; j < cfg->OfdmCaNum() * cfg->BsAntNum(); j++) {
            if (j % cfg->OfdmCaNum() == 0) {
              std::printf("symbol %zu ant %zu\n", i, j / cfg->OfdmCaNum());
            }
            // TODO keep and fix or remove
            std::printf("%d+%di ", dl_tx_data[i][j], dl_tx_data[i][j]);
          }
        }
        std::printf("\n");
      }
    }
  }

  /* Clean Up memory */
  if (cfg->Frame().NumDLSyms() != 0u) {
    dl_mod_data.Free();
    precoder.Free();
    dl_ifft_data.Free();
    dl_tx_data.Free();
  }
  csi_matrices.Free();
  ue_specific_pilot.Free();
  tx_data_all_symbols.Free();
  rx_data_all_symbols.Free();
  std::free(rx_data_temp);
  AGORA_LOG_SHUTDOWN();
}

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  gflags::SetVersionString(GetAgoraProjectVersion());
  auto cfg = std::make_unique<Config>(FLAGS_conf_file.c_str());
  GenerateTestVectors(cfg.get(), FLAGS_profile);
  return 0;
}
