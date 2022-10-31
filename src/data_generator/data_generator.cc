/**
 * @file data_generator.cc
 * @brief Data generator to generate binary files as inputs to Agora, sender
 * and correctness tests
 */

#include "data_generator.h"

#include <immintrin.h>

#include <bitset>
#include <fstream>
#include <iostream>
#include <memory>

#include "armadillo"
#include "comms-lib.h"
#include "config.h"
#include "crc.h"
#include "datatype_conversion.h"
#include "logger.h"
#include "memory_manage.h"
#include "modulation.h"
#include "scrambler.h"
#include "simd_types.h"
#include "utils_ldpc.h"

static constexpr bool kPrintDebugCSI = false;
static constexpr bool kDebugPrintRxData = false;
static constexpr bool kPrintDlTxData = false;
static constexpr bool kPrintDlModData = false;
static constexpr bool kPrintUplinkInformationBytes = false;
static constexpr bool kPrintDownlinkInformationBytes = false;

///Output files
static const std::string kUlDataPrefix = "orig_ul_data_";
static const std::string kUlLdpcDataPrefix = "LDPC_orig_ul_data_";
static const std::string kDlDataPrefix = "orig_dl_data_";
static const std::string kDlLdpcDataPrefix = "LDPC_orig_dl_data_";
static const std::string kRxLdpcPrefix = "LDPC_rx_data_";
static const std::string kDlTxPrefix = "LDPC_dl_tx_data_";
static const std::string kUlScBitsPrefix = "ul_data_b_";

static float RandFloatFromShort(float min, float max) {
  float rand_val = ((float(rand()) / float(RAND_MAX)) * (max - min)) + min;
  const auto rand_val_short = static_cast<short>(rand_val * kShrtFltConvFactor);
  rand_val = static_cast<float>(rand_val_short) / kShrtFltConvFactor;
  return rand_val;
}

void DataGenerator::DoDataGeneration(const std::string& directory) {
  //Make sure the directory exists
  if (std::filesystem::is_directory(directory) == false) {
    std::filesystem::create_directory(directory);
  }
  srand(time(nullptr));
  auto scrambler = std::make_unique<AgoraScrambler::Scrambler>();
  std::unique_ptr<DoCRC> crc_obj = std::make_unique<DoCRC>();
  const size_t ul_cb_bytes = cfg_->NumBytesPerCb(Direction::kUplink);
  const size_t ul_cb_padding = cfg_->NumPaddingBytesPerCb(Direction::kUplink);
  LDPCconfig ul_ldpc_config = this->cfg_->LdpcConfig(Direction::kUplink);

  SimdAlignByteVector ul_scrambler_buffer(ul_cb_bytes + ul_cb_padding,
                                          std::byte(0));

  // Step 1: Generate the information buffers (MAC Packets) and LDPC-encoded
  // buffers for uplink
  std::vector<std::vector<complex_float>> pre_ifft_data_syms;
  const size_t num_ul_mac_bytes =
      this->cfg_->MacBytesNumPerframe(Direction::kUplink);
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
        this->GenMacData(pkt, ue_id);
        pkt->Crc((uint16_t)(
            crc_obj->CalculateCrc24(
                pkt->Data(), cfg_->MacPayloadMaxLength(Direction::kUplink)) &
            0xFFFF));
      }
    }

    {
      const std::string filename_input =
          directory + kUlDataPrefix + std::to_string(this->cfg_->OfdmCaNum()) +
          "_ant" + std::to_string(this->cfg_->UeAntNum()) + ".bin";
      AGORA_LOG_INFO("Saving uplink MAC data to %s\n", filename_input.c_str());
      auto* fp_input = std::fopen(filename_input.c_str(), "wb");
      if (fp_input == nullptr) {
        AGORA_LOG_ERROR("Failed to create file %s\n", filename_input.c_str());
        throw std::runtime_error("Failed to create file" + filename_input);
      } else {
        for (size_t i = 0; i < cfg_->UeAntNum(); i++) {
          const auto write_status =
              std::fwrite(reinterpret_cast<uint8_t*>(ul_mac_info.at(i).data()),
                          sizeof(uint8_t), num_ul_mac_bytes, fp_input);
          if (write_status != num_ul_mac_bytes) {
            AGORA_LOG_ERROR("Wrote %zu out of %zu to file %s\n", write_status,
                            num_ul_mac_bytes, filename_input.c_str());
            throw std::runtime_error("Failed to write to file" +
                                     filename_input);
          }
        }
        const auto close_status = std::fclose(fp_input);
        if (close_status != 0) {
          throw std::runtime_error("Failed to close file" + filename_input);
        }
      }

      if (kPrintUplinkInformationBytes) {
        std::printf("Uplink information bytes\n");
        for (size_t n = 0; n < cfg_->UeAntNum(); n++) {
          std::printf("UE %zu\n", n % this->cfg_->UeAntNum());
          for (size_t i = 0; i < num_ul_mac_bytes; i++) {
            std::printf("%u ", static_cast<uint8_t>(ul_mac_info.at(n).at(i)));
          }
          std::printf("\n");
        }
      }
    }

    const size_t symbol_blocks =
        ul_ldpc_config.NumBlocksInSymbol() * this->cfg_->UeAntNum();
    const size_t num_ul_codeblocks =
        this->cfg_->Frame().NumUlDataSyms() * symbol_blocks;
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

      std::memcpy(ul_scrambler_buffer.data(), ul_information.at(cb).data(),
                  ul_cb_bytes);

      if (this->cfg_->ScrambleEnabled()) {
        scrambler->Scramble(ul_scrambler_buffer.data(), ul_cb_bytes);
      }

      if (ul_cb_padding > 0) {
        std::memset(&ul_scrambler_buffer.at(ul_cb_bytes), 0u, ul_cb_padding);
      }
      this->GenCodeblock(Direction::kUplink,
                         reinterpret_cast<int8_t*>(ul_scrambler_buffer.data()),
                         ul_encoded_codewords.at(cb));
    }

    {
      const std::string filename_input =
          directory + kUlLdpcDataPrefix +
          std::to_string(this->cfg_->OfdmCaNum()) + "_ant" +
          std::to_string(this->cfg_->UeAntNum()) + ".bin";
      AGORA_LOG_INFO("Saving raw uplink data (using LDPC) to %s\n",
                     filename_input.c_str());
      auto* fp_input = std::fopen(filename_input.c_str(), "wb");
      if (fp_input == nullptr) {
        AGORA_LOG_ERROR("Failed to create file %s\n", filename_input.c_str());
        throw std::runtime_error("Failed to create file" + filename_input);
      } else {
        for (size_t i = 0; i < num_ul_codeblocks; i++) {
          const auto write_status = std::fwrite(
              reinterpret_cast<uint8_t*>(&ul_information.at(i).at(0)),
              sizeof(uint8_t), ul_cb_bytes, fp_input);
          if (write_status != ul_cb_bytes) {
            AGORA_LOG_ERROR("Wrote %zu out of %zu to file %s\n", write_status,
                            ul_cb_bytes, filename_input.c_str());
            throw std::runtime_error("Failed to write to file" +
                                     filename_input);
          }
        }
        const auto close_status = std::fclose(fp_input);
        if (close_status != 0) {
          throw std::runtime_error("Failed to close file" + filename_input);
        }
      }

      if (kPrintUplinkInformationBytes) {
        std::printf("Uplink information bytes\n");
        for (size_t n = 0; n < num_ul_codeblocks; n++) {
          std::printf("Symbol %zu, UE %zu\n", n / this->cfg_->UeAntNum(),
                      n % this->cfg_->UeAntNum());
          for (size_t i = 0; i < ul_cb_bytes; i++) {
            std::printf("%u ",
                        static_cast<uint8_t>(ul_information.at(n).at(i)));
          }
          std::printf("\n");
        }
      }
    }

    if (kOutputUlScData) {
      std::vector<std::vector<std::vector<std::vector<std::vector<uint8_t>>>>>
          ul_ofdm_data(
              this->cfg_->UeNum(),
              std::vector<std::vector<std::vector<std::vector<uint8_t>>>>(
                  kOutputFrameNum,
                  std::vector<std::vector<std::vector<uint8_t>>>(
                      this->cfg_->Frame().NumULSyms(),
                      std::vector<std::vector<uint8_t>>(
                          this->cfg_->NumUeChannels(),
                          std::vector<uint8_t>(this->cfg_->OfdmDataNum())))));
      for (size_t n = 0; n < num_ul_codeblocks; n++) {
        const size_t cl_sdr = (n % this->cfg_->UeNum());
        const size_t ul_slot = (n / this->cfg_->UeAntNum()) +
                               this->cfg_->Frame().ClientUlPilotSymbols();
        const size_t cl_sdr_ch =
            (n % this->cfg_->UeAntNum()) % this->cfg_->NumUeChannels();
        std::vector<uint8_t> odfm_symbol(this->cfg_->OfdmDataNum());
        AdaptBitsForMod(
            reinterpret_cast<const uint8_t*>(ul_encoded_codewords.at(n).data()),
            odfm_symbol.data(),
            this->cfg_->LdpcConfig(Direction::kUplink).NumEncodedBytes(),
            this->cfg_->ModOrderBits(Direction::kUplink));
        for (size_t f = 0; f < kOutputFrameNum; f++) {
          ul_ofdm_data.at(cl_sdr).at(f).at(ul_slot).at(cl_sdr_ch) = odfm_symbol;
        }
      }
      for (size_t i = 0; i < this->cfg_->UeNum(); i++) {
        const std::string filename_input =
            directory + kUlScBitsPrefix +
            this->cfg_->Modulation(Direction::kUplink) + "_" +
            std::to_string(this->cfg_->OfdmDataNum()) + "_" +
            std::to_string(this->cfg_->OfdmCaNum()) + "_" +
            std::to_string(kOfdmSymbolPerSlot) + "_" +
            std::to_string(this->cfg_->Frame().NumULSyms()) + "_" +
            std::to_string(kOutputFrameNum) + "_" + this->cfg_->UeChannel() +
            "_" + std::to_string(i) + ".bin";
        AGORA_LOG_INFO("Saving uplink sc bits to %s\n", filename_input.c_str());
        auto* fp_tx_b = std::fopen(filename_input.c_str(), "wb");
        if (fp_tx_b == nullptr) {
          throw std::runtime_error(
              "DataGenerator: Failed to create ul sc bits file");
        }
        for (size_t f = 0; f < kOutputFrameNum; f++) {
          for (size_t u = 0; u < this->cfg_->Frame().NumULSyms(); u++) {
            for (size_t h = 0; h < this->cfg_->NumUeChannels(); h++) {
              const auto write_status = std::fwrite(
                  ul_ofdm_data.at(i).at(f).at(u).at(h).data(), sizeof(uint8_t),
                  this->cfg_->OfdmDataNum(), fp_tx_b);
              if (write_status != this->cfg_->OfdmDataNum()) {
                throw std::runtime_error(
                    "DataGenerator: Failed to write ul sc bits file");
              }
            }
          }
        }
        const auto close_status = std::fclose(fp_tx_b);
        if (close_status != 0) {
          throw std::runtime_error(
              "DataGenerator: Failed to close ul sc bits file");
        }
      }
    }

    // Modulate the encoded codewords
    std::vector<std::vector<complex_float>> ul_modulated_codewords(
        num_ul_codeblocks);
    for (size_t i = 0; i < num_ul_codeblocks; i++) {
      ul_modulated_codewords.at(i) =
          this->GetModulation(ul_encoded_codewords.at(i));
    }

    // Place modulated uplink data codewords into central IFFT bins
    RtAssert(ul_ldpc_config.NumBlocksInSymbol() == 1);  // TODO: Assumption
    pre_ifft_data_syms.resize(this->cfg_->UeAntNum() *
                              this->cfg_->Frame().NumUlDataSyms());
    for (size_t i = 0; i < pre_ifft_data_syms.size(); i++) {
      pre_ifft_data_syms.at(i) = this->BinForIfft(ul_modulated_codewords.at(i));
    }
  }

  // Generate UE-specific pilots (phase tracking & downlink channel estimation)
  Table<complex_float> ue_specific_pilot;
  const std::vector<std::complex<float>> zc_seq =
      Utils::DoubleToCfloat(CommsLib::GetSequence(this->cfg_->OfdmDataNum(),
                                                  CommsLib::kLteZadoffChu));
  const std::vector<std::complex<float>> zc_common_pilot =
      CommsLib::SeqCyclicShift(zc_seq, M_PI / 4.0);  // Used in LTE SRS
  ue_specific_pilot.Malloc(this->cfg_->UeAntNum(), this->cfg_->OfdmDataNum(),
                           Agora_memory::Alignment_t::kAlign64);
  for (size_t i = 0; i < this->cfg_->UeAntNum(); i++) {
    auto zc_ue_pilot_i =
        CommsLib::SeqCyclicShift(zc_seq, i * M_PI / 6.0);  // LTE DMRS
    for (size_t j = 0; j < this->cfg_->OfdmDataNum(); j++) {
      ue_specific_pilot[i][j] = {zc_ue_pilot_i[j].real(),
                                 zc_ue_pilot_i[j].imag()};
    }
  }

  // Generate common sounding pilots
  std::vector<complex_float> pilot_td = this->GetCommonPilotTimeDomain();

  // Put pilot and data symbols together
  Table<complex_float> tx_data_all_symbols;
  tx_data_all_symbols.Calloc(this->cfg_->Frame().NumTotalSyms(),
                             this->cfg_->UeAntNum() * this->cfg_->OfdmCaNum(),
                             Agora_memory::Alignment_t::kAlign64);

  if (this->cfg_->FreqOrthogonalPilot() == true) {
    for (size_t i = 0; i < this->cfg_->UeAntNum(); i++) {
      std::vector<complex_float> pilots_t_ue(
          this->cfg_->OfdmCaNum());  // Zeroed
      for (size_t j = this->cfg_->OfdmDataStart();
           j < this->cfg_->OfdmDataStop(); j += this->cfg_->UeAntNum()) {
        pilots_t_ue.at(i + j) = pilot_td.at(i + j);
      }
      // Load pilots
      std::memcpy(tx_data_all_symbols[this->cfg_->Frame().NumBeaconSyms()] +
                      (i * this->cfg_->OfdmCaNum()),
                  &pilots_t_ue.at(0),
                  (this->cfg_->OfdmCaNum() * sizeof(complex_float)));
    }
  } else {
    for (size_t i = 0; i < this->cfg_->UeAntNum(); i++) {
      std::memcpy(tx_data_all_symbols[i + this->cfg_->Frame().NumBeaconSyms()] +
                      i * this->cfg_->OfdmCaNum(),
                  &pilot_td.at(0),
                  (this->cfg_->OfdmCaNum() * sizeof(complex_float)));
    }
  }

  // Populate the UL symbols
  for (size_t i = 0; i < this->cfg_->Frame().NumULSyms(); i++) {
    const size_t data_sym_id = this->cfg_->Frame().GetULSymbol(i);
    for (size_t j = 0; j < this->cfg_->UeAntNum(); j++) {
      if (i < this->cfg_->Frame().ClientUlPilotSymbols()) {
        std::memcpy(tx_data_all_symbols[data_sym_id] +
                        (j * this->cfg_->OfdmCaNum()) +
                        this->cfg_->OfdmDataStart(),
                    ue_specific_pilot[j],
                    this->cfg_->OfdmDataNum() * sizeof(complex_float));
      } else {
        size_t k = i - this->cfg_->Frame().ClientUlPilotSymbols();
        std::memcpy(
            tx_data_all_symbols[data_sym_id] + (j * this->cfg_->OfdmCaNum()),
            &pre_ifft_data_syms.at(k * this->cfg_->UeAntNum() + j).at(0),
            this->cfg_->OfdmCaNum() * sizeof(complex_float));
      }
    }
  }

  // Generate CSI matrix
  Table<complex_float> csi_matrices;
  float sqrt2_norm = 1 / std::sqrt(2);
  csi_matrices.Calloc(this->cfg_->OfdmCaNum(),
                      this->cfg_->UeAntNum() * this->cfg_->BsAntNum(),
                      Agora_memory::Alignment_t::kAlign32);
  for (size_t i = 0; i < (this->cfg_->UeAntNum() * this->cfg_->BsAntNum());
       i++) {
    complex_float csi = {RandFloatFromShort(-1, 1), RandFloatFromShort(-1, 1)};
    for (size_t j = 0; j < this->cfg_->OfdmCaNum(); j++) {
      csi_matrices[j][i].re = csi.re * sqrt2_norm;
      csi_matrices[j][i].im = csi.im * sqrt2_norm;
    }
  }
  arma::arma_rng::set_seed_random();

  // Generate RX data received by base station after going through channels
  Table<complex_float> rx_data_all_symbols;
  rx_data_all_symbols.Calloc(
      this->cfg_->Frame().NumTotalSyms(),
      this->cfg_->SampsPerSymbol() * this->cfg_->BsAntNum(),
      Agora_memory::Alignment_t::kAlign64);
  size_t data_start = this->cfg_->CpLen() + this->cfg_->OfdmTxZeroPrefix();
  auto* ifft_shift_tmp =
      static_cast<complex_float*>(Agora_memory::PaddedAlignedAlloc(
          Agora_memory::Alignment_t::kAlign64,
          cfg_->OfdmCaNum() * sizeof(complex_float)));
  for (size_t i = 0; i < this->cfg_->Frame().NumTotalSyms(); i++) {
    arma::cx_fmat mat_input_data(
        reinterpret_cast<arma::cx_float*>(tx_data_all_symbols[i]),
        this->cfg_->OfdmCaNum(), this->cfg_->UeAntNum(), false);
    arma::cx_fmat mat_output(
        reinterpret_cast<arma::cx_float*>(rx_data_all_symbols[i]),
        this->cfg_->SampsPerSymbol(), this->cfg_->BsAntNum(), false);

    for (size_t j = 0; j < this->cfg_->OfdmCaNum(); j++) {
      arma::cx_fmat mat_csi(reinterpret_cast<arma::cx_float*>(csi_matrices[j]),
                            this->cfg_->BsAntNum(), this->cfg_->UeAntNum(),
                            false);
      mat_output.row(j + data_start) = mat_input_data.row(j) * mat_csi.st();
    }
    arma::cx_fmat noise_mat(size(mat_output));
    noise_mat.set_real(arma::randn<arma::fmat>(size(real(mat_output))));
    noise_mat.set_imag(arma::randn<arma::fmat>(size(real(mat_output))));
    mat_output += (noise_mat * this->cfg_->NoiseLevel() * sqrt2_norm);
    for (size_t j = 0; j < this->cfg_->BsAntNum(); j++) {
      auto* this_ofdm_symbol =
          rx_data_all_symbols[i] + j * this->cfg_->SampsPerSymbol() +
          this->cfg_->CpLen() + this->cfg_->OfdmTxZeroPrefix();
      CommsLib::FFTShift(this_ofdm_symbol, ifft_shift_tmp,
                         this->cfg_->OfdmCaNum());
      CommsLib::IFFT(this_ofdm_symbol, this->cfg_->OfdmCaNum(), false);
    }
  }

  const std::string filename_rx =
      directory + kRxLdpcPrefix + std::to_string(this->cfg_->OfdmCaNum()) +
      "_ant" + std::to_string(this->cfg_->BsAntNum()) + ".bin";
  AGORA_LOG_INFO("Saving rx data to %s\n", filename_rx.c_str());
  auto* fp_rx = std::fopen(filename_rx.c_str(), "wb");
  if (fp_rx == nullptr) {
    AGORA_LOG_ERROR("Failed to create file %s\n", filename_rx.c_str());
    throw std::runtime_error("Failed to create file" + filename_rx);
  } else {
    for (size_t i = 0; i < this->cfg_->Frame().NumTotalSyms(); i++) {
      const auto* ptr = reinterpret_cast<float*>(rx_data_all_symbols[i]);
      const auto write_status = std::fwrite(
          ptr, sizeof(float),
          this->cfg_->SampsPerSymbol() * this->cfg_->BsAntNum() * 2, fp_rx);
      if (write_status !=
          this->cfg_->SampsPerSymbol() * this->cfg_->BsAntNum() * 2) {
        AGORA_LOG_ERROR("Write %zu out of %zu to file %s\n", write_status,
                        num_ul_mac_bytes, filename_rx.c_str());
        throw std::runtime_error("Failed to write to file" + filename_rx);
      }
    }
    const auto close_status = std::fclose(fp_rx);
    if (close_status != 0) {
      throw std::runtime_error("Failed to close file" + filename_rx);
    }
  }

  if (kDebugPrintRxData) {
    std::printf("rx data\n");
    for (size_t i = 0; i < 10; i++) {
      for (size_t j = 0; j < this->cfg_->OfdmCaNum() * this->cfg_->BsAntNum();
           j++) {
        if (j % this->cfg_->OfdmCaNum() == 0) {
          std::printf("\nsymbol %zu ant %zu\n", i, j / this->cfg_->OfdmCaNum());
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
  const LDPCconfig dl_ldpc_config =
      this->cfg_->LdpcConfig(Direction::kDownlink);
  const size_t dl_cb_bytes = cfg_->NumBytesPerCb(Direction::kDownlink);
  const size_t dl_cb_padding = cfg_->NumPaddingBytesPerCb(Direction::kDownlink);

  SimdAlignByteVector dl_scrambler_buffer(dl_cb_bytes + dl_cb_padding,
                                          std::byte(0));

  if (this->cfg_->Frame().NumDLSyms() > 0) {
    const size_t num_dl_mac_bytes =
        this->cfg_->MacBytesNumPerframe(Direction::kDownlink);
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
        this->GenMacData(pkt, ue_id);
        pkt->Crc((uint16_t)(
            crc_obj->CalculateCrc24(
                pkt->Data(), cfg_->MacPayloadMaxLength(Direction::kDownlink)) &
            0xFFFF));
      }
    }

    {
      const std::string filename_input =
          directory + kDlDataPrefix + std::to_string(this->cfg_->OfdmCaNum()) +
          "_ant" + std::to_string(this->cfg_->UeAntNum()) + ".bin";
      AGORA_LOG_INFO("Saving downlink MAC data to %s\n",
                     filename_input.c_str());
      FILE* fp_input = std::fopen(filename_input.c_str(), "wb");
      if (fp_input == nullptr) {
        AGORA_LOG_ERROR("Failed to create file %s\n", filename_input.c_str());
        throw std::runtime_error("Failed to create file" + filename_input);
      } else {
        for (size_t i = 0; i < cfg_->UeAntNum(); i++) {
          const auto write_status =
              std::fwrite(reinterpret_cast<uint8_t*>(dl_mac_info.at(i).data()),
                          sizeof(uint8_t), num_dl_mac_bytes, fp_input);
          if (write_status != num_dl_mac_bytes) {
            AGORA_LOG_ERROR("Wrote %zu out of %zu to file %s\n", write_status,
                            num_dl_mac_bytes, filename_input.c_str());
            throw std::runtime_error("Failed to write to file" +
                                     filename_input);
          }
        }
        const auto close_status = std::fclose(fp_input);
        if (close_status != 0) {
          throw std::runtime_error("Failed to close file" + filename_input);
        }
      }

      if (kPrintDownlinkInformationBytes) {
        std::printf("Downlink information bytes\n");
        for (size_t n = 0; n < cfg_->UeAntNum(); n++) {
          std::printf("UE %zu\n", n % this->cfg_->UeAntNum());
          for (size_t i = 0; i < num_dl_mac_bytes; i++) {
            std::printf("%u ", static_cast<uint8_t>(dl_mac_info.at(n).at(i)));
          }
          std::printf("\n");
        }
      }
    }

    const size_t symbol_blocks =
        dl_ldpc_config.NumBlocksInSymbol() * this->cfg_->UeAntNum();
    const size_t num_dl_codeblocks =
        this->cfg_->Frame().NumDlDataSyms() * symbol_blocks;
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

      std::memcpy(dl_scrambler_buffer.data(), dl_information.at(cb).data(),
                  dl_cb_bytes);

      if (this->cfg_->ScrambleEnabled()) {
        scrambler->Scramble(dl_scrambler_buffer.data(), dl_cb_bytes);
      }

      if (dl_cb_padding > 0u) {
        std::memset(&dl_scrambler_buffer.at(dl_cb_bytes), 0u, dl_cb_padding);
      }
      this->GenCodeblock(Direction::kDownlink,
                         reinterpret_cast<int8_t*>(dl_scrambler_buffer.data()),
                         dl_encoded_codewords.at(cb));
    }

    // Modulate the encoded codewords
    std::vector<std::vector<complex_float>> dl_modulated_codewords(
        num_dl_codeblocks);
    for (size_t i = 0; i < num_dl_codeblocks; i++) {
      const size_t sym_offset = i % (symbol_blocks);
      const size_t ue_id = sym_offset / dl_ldpc_config.NumBlocksInSymbol();
      dl_modulated_codewords.at(i) = this->GetDLModulation(
          dl_encoded_codewords.at(i), ue_specific_pilot[ue_id]);
    }

    {
      // Save downlink information bytes to file
      const std::string filename_input =
          directory + kDlLdpcDataPrefix +
          std::to_string(this->cfg_->OfdmCaNum()) + "_ant" +
          std::to_string(this->cfg_->UeAntNum()) + ".bin";
      AGORA_LOG_INFO("Saving raw dl data (using LDPC) to %s\n",
                     filename_input.c_str());
      auto* fp_input = std::fopen(filename_input.c_str(), "wb");
      if (fp_input == nullptr) {
        AGORA_LOG_ERROR("Failed to create file %s\n", filename_input.c_str());
        throw std::runtime_error("Failed to create file" + filename_input);
      } else {
        for (size_t i = 0; i < num_dl_codeblocks; i++) {
          const auto write_status = std::fwrite(
              reinterpret_cast<uint8_t*>(&dl_information.at(i).at(0)),
              sizeof(uint8_t), dl_cb_bytes, fp_input);
          if (write_status != dl_cb_bytes) {
            AGORA_LOG_ERROR("Wrote %zu out of %zu to file %s\n", write_status,
                            dl_cb_bytes, filename_input.c_str());
            throw std::runtime_error("Failed to write to file" +
                                     filename_input);
          }
        }
        const auto close_status = std::fclose(fp_input);
        if (close_status != 0) {
          throw std::runtime_error("Failed to close file" + filename_input);
        }
      }

      if (kPrintDownlinkInformationBytes == true) {
        std::printf("Downlink information bytes\n");
        for (size_t n = 0; n < num_dl_codeblocks; n++) {
          std::printf("Symbol %zu, UE %zu\n", n / this->cfg_->UeAntNum(),
                      n % this->cfg_->UeAntNum());
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
    precoder.Calloc(this->cfg_->OfdmCaNum(),
                    this->cfg_->UeAntNum() * this->cfg_->BsAntNum(),
                    Agora_memory::Alignment_t::kAlign32);
    for (size_t i = 0; i < this->cfg_->OfdmCaNum(); i++) {
      arma::cx_fmat mat_input(
          reinterpret_cast<arma::cx_float*>(csi_matrices[i]),
          this->cfg_->BsAntNum(), this->cfg_->UeAntNum(), false);
      arma::cx_fmat mat_output(reinterpret_cast<arma::cx_float*>(precoder[i]),
                               this->cfg_->UeAntNum(), this->cfg_->BsAntNum(),
                               false);
      pinv(mat_output, mat_input, 1e-2, "dc");
    }

    if (kPrintDebugCSI) {
      std::printf("CSI \n");
      // for (size_t i = 0; i < this->cfg_->ofdm_ca_num(); i++)
      for (size_t j = 0; j < this->cfg_->UeAntNum() * this->cfg_->BsAntNum();
           j++) {
        std::printf("%.3f+%.3fi ",
                    csi_matrices[this->cfg_->OfdmDataStart()][j].re,
                    csi_matrices[this->cfg_->OfdmDataStart()][j].im);
      }
      std::printf("\nprecoder \n");
      // for (size_t i = 0; i < this->cfg_->ofdm_ca_num(); i++)
      for (size_t j = 0; j < this->cfg_->UeAntNum() * this->cfg_->BsAntNum();
           j++) {
        std::printf("%.3f+%.3fi ", precoder[this->cfg_->OfdmDataStart()][j].re,
                    precoder[this->cfg_->OfdmDataStart()][j].im);
      }
      std::printf("\n");
    }

    // Prepare downlink data from mod_output
    Table<complex_float> dl_mod_data;
    dl_mod_data.Calloc(this->cfg_->Frame().NumDLSyms(),
                       this->cfg_->OfdmCaNum() * this->cfg_->UeAntNum(),
                       Agora_memory::Alignment_t::kAlign64);
    for (size_t i = 0; i < this->cfg_->Frame().NumDLSyms(); i++) {
      for (size_t j = 0; j < this->cfg_->UeAntNum(); j++) {
        for (size_t sc_id = 0; sc_id < this->cfg_->OfdmDataNum(); sc_id++) {
          complex_float sc_data;
          if ((i < this->cfg_->Frame().ClientDlPilotSymbols()) ||
              (sc_id % this->cfg_->OfdmPilotSpacing() == 0)) {
            sc_data = ue_specific_pilot[j][sc_id];
          } else {
            sc_data =
                dl_modulated_codewords
                    .at(((i - this->cfg_->Frame().ClientDlPilotSymbols()) *
                         this->cfg_->UeAntNum()) +
                        j)
                    .at(sc_id);
          }
          dl_mod_data[i][j * this->cfg_->OfdmCaNum() + sc_id +
                         this->cfg_->OfdmDataStart()] = sc_data;
        }
      }
    }

    if (kPrintDlModData) {
      std::printf("dl mod data \n");
      for (size_t i = 0; i < this->cfg_->Frame().NumDLSyms(); i++) {
        for (size_t k = this->cfg_->OfdmDataStart();
             k < this->cfg_->OfdmDataStart() + this->cfg_->OfdmDataNum(); k++) {
          std::printf("symbol %zu, subcarrier %zu\n", i, k);
          for (size_t j = 0; j < this->cfg_->UeAntNum(); j++) {
            // for (int k = this->cfg_->OfdmDataStart(); k <
            // this->cfg_->OfdmDataStart() + this->cfg_->OfdmDataNum();
            //      k++) {
            std::printf("%.3f+%.3fi ",
                        dl_mod_data[i][j * this->cfg_->OfdmCaNum() + k].re,
                        dl_mod_data[i][j * this->cfg_->OfdmCaNum() + k].im);
          }
          std::printf("\n");
        }
      }
    }

    // Perform precoding and IFFT
    Table<complex_float> dl_ifft_data;
    dl_ifft_data.Calloc(this->cfg_->Frame().NumDLSyms(),
                        this->cfg_->OfdmCaNum() * this->cfg_->BsAntNum(),
                        Agora_memory::Alignment_t::kAlign64);
    Table<short> dl_tx_data;
    dl_tx_data.Calloc(this->cfg_->Frame().NumDLSyms(),
                      2 * this->cfg_->SampsPerSymbol() * this->cfg_->BsAntNum(),
                      Agora_memory::Alignment_t::kAlign64);

    for (size_t i = 0; i < this->cfg_->Frame().NumDLSyms(); i++) {
      arma::cx_fmat mat_input_data(
          reinterpret_cast<arma::cx_float*>(dl_mod_data[i]),
          this->cfg_->OfdmCaNum(), this->cfg_->UeAntNum(), false);

      arma::cx_fmat mat_output(
          reinterpret_cast<arma::cx_float*>(dl_ifft_data[i]),
          this->cfg_->OfdmCaNum(), this->cfg_->BsAntNum(), false);

      for (size_t j = this->cfg_->OfdmDataStart();
           j < this->cfg_->OfdmDataNum() + this->cfg_->OfdmDataStart(); j++) {
        arma::cx_fmat mat_precoder(
            reinterpret_cast<arma::cx_float*>(precoder[j]),
            this->cfg_->UeAntNum(), this->cfg_->BsAntNum(), false);
        mat_precoder /= abs(mat_precoder).max();
        mat_output.row(j) = mat_input_data.row(j) * mat_precoder;

        // std::printf("symbol %d, sc: %d\n", i, j -
        // this->cfg_->ofdm_data_start()); cout << "Precoder: \n" <<
        // mat_precoder
        // << endl; cout << "Data: \n" << mat_input_data.row(j) << endl; cout <<
        // "Precoded data: \n" << mat_output.row(j) << endl;
      }
      for (size_t j = 0; j < this->cfg_->BsAntNum(); j++) {
        complex_float* ptr_ifft = dl_ifft_data[i] + j * this->cfg_->OfdmCaNum();
        CommsLib::FFTShift(ptr_ifft, ifft_shift_tmp, this->cfg_->OfdmCaNum());
        CommsLib::IFFT(ptr_ifft, this->cfg_->OfdmCaNum(), false);

        short* tx_symbol = dl_tx_data[i] + j * this->cfg_->SampsPerSymbol() * 2;
        std::memset(tx_symbol, 0,
                    sizeof(short) * 2 * this->cfg_->OfdmTxZeroPrefix());
        for (size_t k = 0; k < this->cfg_->OfdmCaNum(); k++) {
          tx_symbol[2 * (k + this->cfg_->CpLen() +
                         this->cfg_->OfdmTxZeroPrefix())] =
              static_cast<short>(kShrtFltConvFactor * ptr_ifft[k].re);
          tx_symbol[2 * (k + this->cfg_->CpLen() +
                         this->cfg_->OfdmTxZeroPrefix()) +
                    1] =
              static_cast<short>(kShrtFltConvFactor * ptr_ifft[k].im);
        }
        for (size_t k = 0; k < (2 * this->cfg_->CpLen()); k++) {
          tx_symbol[2 * this->cfg_->OfdmTxZeroPrefix() + k] =
              tx_symbol[2 * (this->cfg_->OfdmTxZeroPrefix() +
                             this->cfg_->OfdmCaNum()) +
                        k];
        }

        const size_t tx_zero_postfix_offset =
            2 * (this->cfg_->OfdmTxZeroPrefix() + this->cfg_->CpLen() +
                 this->cfg_->OfdmCaNum());
        std::memset(tx_symbol + tx_zero_postfix_offset, 0,
                    sizeof(short) * 2 * this->cfg_->OfdmTxZeroPostfix());
      }
    }

    std::string filename_dl_tx =
        directory + kDlTxPrefix + std::to_string(this->cfg_->OfdmCaNum()) +
        "_ant" + std::to_string(this->cfg_->BsAntNum()) + ".bin";
    AGORA_LOG_INFO("Saving dl tx data to %s\n", filename_dl_tx.c_str());
    auto* fp_dl_tx = std::fopen(filename_dl_tx.c_str(), "wb");
    if (fp_dl_tx == nullptr) {
      AGORA_LOG_ERROR("Failed to create file %s\n", filename_dl_tx.c_str());
      throw std::runtime_error("Failed to create file" + filename_dl_tx);
    } else {
      for (size_t i = 0; i < this->cfg_->Frame().NumDLSyms(); i++) {
        const short* ptr = dl_tx_data[i];
        const auto write_status = std::fwrite(
            ptr, sizeof(short),
            this->cfg_->SampsPerSymbol() * this->cfg_->BsAntNum() * 2,
            fp_dl_tx);
        if (write_status !=
            this->cfg_->SampsPerSymbol() * this->cfg_->BsAntNum() * 2) {
          AGORA_LOG_ERROR(
              "Wrote %zu out of %zu to file %s\n", write_status,
              this->cfg_->SampsPerSymbol() * this->cfg_->BsAntNum() * 2,
              filename_dl_tx.c_str());
          throw std::runtime_error("Failed to write to file" + filename_dl_tx);
        }
      }
      const auto close_status = std::fclose(fp_dl_tx);
      if (close_status != 0) {
        throw std::runtime_error("Failed to close file" + filename_dl_tx);
      }
    }

    if (kPrintDlTxData) {
      std::printf("rx data\n");
      for (size_t i = 0; i < 10; i++) {
        for (size_t j = 0; j < this->cfg_->OfdmCaNum() * this->cfg_->BsAntNum();
             j++) {
          if (j % this->cfg_->OfdmCaNum() == 0) {
            std::printf("symbol %zu ant %zu\n", i, j / this->cfg_->OfdmCaNum());
          }
          // TODO keep and fix or remove
          // std::printf("%d+%di ", dl_tx_data[i][j], dl_tx_data[i][j]);
        }
      }
      std::printf("\n");
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
  std::free(ifft_shift_tmp);
}