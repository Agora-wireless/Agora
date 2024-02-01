/**
 * @file data_generator.cc
 * @brief Data generator to generate binary files as inputs to Agora, sender
 * and correctness tests
 */

#include "data_generator.h"

#include <cstdio>
#include <memory>

#include "comms-lib.h"
#include "crc.h"
#include "datatype_conversion.h"
#include "logger.h"
#include "modulation.h"
#include "phy_ldpc_decoder_5gnr.h"
#include "scrambler.h"

DataGenerator::DataGenerator(Config* cfg, uint64_t seed, Profile profile)
    : cfg_(cfg), seed_(seed), profile_(profile) {
  if (seed != 0) {
    fast_rand_.seed_ = seed;
  }
}

/**
   * @brief                        Generate random Mac payload bit
   * sequence
   *
   * @param  information           The generated input bit sequence
   * @param  ue_id                 ID of the UE that this codeblock belongs to
   */
void DataGenerator::GenMacData(MacPacketPacked* mac, size_t ue_id) {
  for (size_t i = 0; i < mac->PayloadLength(); i++) {
    if (profile_ == Profile::kRandom) {
      mac->DataPtr()[i] = static_cast<int8_t>(fast_rand_.NextU32());
    } else if (profile_ == Profile::kProfile123) {
      mac->DataPtr()[i] = 1 + (ue_id * 3) + (i % 3);
    }
  }
}

void DataGenerator::GenMacRandomBits(MacPacketPacked* mac) {
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  rand_byte_generator byte_gen(seed);
  for (size_t i = 0; i < mac->PayloadLength(); i++) {
    mac->DataPtr()[i] = byte_gen();
  }
}

/**
   * @brief                        Generate one raw information bit sequence
   *
   * @param  information           The generated input bit sequence
   * @param  ue_id                 ID of the UE that this codeblock belongs to
   */
void DataGenerator::GenRawData(const LDPCconfig& lc,
                               std::vector<int8_t>& information, size_t ue_id) {
  // const LDPCconfig& lc = cfg_->LdpcConfig(dir);
  information.resize(
      LdpcEncodingInputBufSize(lc.BaseGraph(), lc.ExpansionFactor()));

  for (size_t i = 0; i < lc.NumInputBytes(); i++) {
    if (profile_ == Profile::kRandom) {
      information.at(i) = static_cast<int8_t>(fast_rand_.NextU32());
    } else if (profile_ == Profile::kProfile123) {
      information.at(i) = 1 + (ue_id * 3) + (i % 3);
    }
  }
}

/// Return the frequency-domain pilot symbol with OfdmCaNum complex floats
std::vector<complex_float> DataGenerator::GetCommonPilotFreqDomain() const {
  const std::vector<std::complex<float>> zc_seq = Utils::DoubleToCfloat(
      CommsLib::GetSequence(cfg_->OfdmDataNum(), CommsLib::kLteZadoffChu));

  const std::vector<std::complex<float>> zc_common_pilot =
      CommsLib::SeqCyclicShift(zc_seq, M_PI / 4.0);  // Used in LTE SRS

  //auto ret = DataGenerator::BinForIfft(this->cfg_, zc_common_pilot, false);
  std::vector<complex_float> ret(cfg_->OfdmCaNum());  // Zeroed
  for (size_t i = 0; i < cfg_->OfdmDataNum(); i++) {
    ret[i + cfg_->OfdmDataStart()] = {zc_common_pilot[i].real(),
                                      zc_common_pilot[i].imag()};
  }

  return ret;
}

/// Return the user-spepcific frequency-domain pilot symbol with OfdmCaNum complex floats
Table<complex_float> DataGenerator::GetUeSpecificPilotFreqDomain() const {
  Table<complex_float> ue_specific_pilot;
  const std::vector<std::complex<float>> zc_seq = Utils::DoubleToCfloat(
      CommsLib::GetSequence(cfg_->OfdmDataNum(), CommsLib::kLteZadoffChu));
  const std::vector<std::complex<float>> zc_common_pilot =
      CommsLib::SeqCyclicShift(zc_seq, M_PI / 4.0);  // Used in LTE SRS
  ue_specific_pilot.Malloc(cfg_->UeAntNum(), cfg_->OfdmDataNum(),
                           Agora_memory::Alignment_t::kAlign64);
  for (size_t i = 0; i < cfg_->UeAntNum(); i++) {
    auto zc_ue_pilot_i =
        CommsLib::SeqCyclicShift(zc_seq, i * M_PI / 6.0);  // LTE DMRS
    for (size_t j = 0; j < cfg_->OfdmDataNum(); j++) {
      ue_specific_pilot[i][j] = {zc_ue_pilot_i[j].real(),
                                 zc_ue_pilot_i[j].imag()};
    }
  }
  return ue_specific_pilot;
}

void DataGenerator::GetNoisySymbol(
    const std::vector<complex_float>& modulated_symbol,
    std::vector<complex_float>& noisy_symbol, float noise_level) const {
  std::default_random_engine generator(seed_);
  std::normal_distribution<double> distribution(0.0, 1.0);
  for (size_t j = 0; j < modulated_symbol.size(); j++) {
    complex_float noise = {
        static_cast<float>(distribution(generator)) * noise_level,
        static_cast<float>(distribution(generator)) * noise_level};
    noisy_symbol.at(j).re = modulated_symbol.at(j).re + noise.re;
    noisy_symbol.at(j).im = modulated_symbol.at(j).im + noise.im;
  }
}

void DataGenerator::GetNoisySymbol(const complex_float* modulated_symbol,
                                   complex_float* noisy_symbol, size_t length,
                                   float noise_level) const {
  std::default_random_engine generator(seed_);
  std::normal_distribution<double> distribution(0.0, 1.0);
  for (size_t j = 0; j < length; j++) {
    complex_float noise = {
        static_cast<float>(distribution(generator)) * noise_level,
        static_cast<float>(distribution(generator)) * noise_level};
    noisy_symbol[j].re = modulated_symbol[j].re + noise.re;
    noisy_symbol[j].im = modulated_symbol[j].im + noise.im;
  }
}

/**
   * @brief                        Generate the encoded bit sequence for one
   * code block for the active LDPC configuration from the input bit sequence
   *
   * @param  input_ptr             The input bit sequence to be encoded
   * @param  encoded_codeword      The generated encoded codeword bit sequence
   */
std::vector<int8_t> DataGenerator::GenCodeblock(const LDPCconfig& lc,
                                                const int8_t* input_ptr,
                                                size_t input_size,
                                                bool scramble_enabled) {
  std::vector<int8_t> scramble_buffer(input_ptr, input_ptr + input_size);
  if (scramble_enabled) {
    auto scrambler = std::make_unique<AgoraScrambler::Scrambler>();
    scrambler->Scramble(scramble_buffer.data(), input_size);
  }
  scramble_buffer.resize(Roundup<64>(input_size), 0);

  std::vector<int8_t> parity;
  parity.resize(
      LdpcEncodingParityBufSize(lc.BaseGraph(), lc.ExpansionFactor()));

  const size_t encoded_bytes = BitsToBytes(lc.NumCbCodewLen());
  std::vector<int8_t> encoded_codeword(encoded_bytes, 0);

  LdpcEncodeHelper(lc.BaseGraph(), lc.ExpansionFactor(), lc.NumRows(),
                   &encoded_codeword.at(0), &parity.at(0),
                   reinterpret_cast<int8_t*>(scramble_buffer.data()));
  return encoded_codeword;
}

/**
   * @brief Return the output of modulating the encoded codeword
   * @param encoded_codeword The encoded LDPC codeword bit sequence
   * @return An array of complex floats with OfdmDataNum() elements
   */
std::vector<complex_float> DataGenerator::GetModulation(
    const int8_t* encoded_codeword, Table<complex_float> mod_table,
    const size_t num_bits, const size_t num_subcarriers,
    const size_t mod_order_bits) {
  std::vector<complex_float> modulated_codeword(num_subcarriers);
  std::vector<uint8_t> mod_input(num_subcarriers, 0);

  AdaptBitsForMod(reinterpret_cast<const uint8_t*>(&encoded_codeword[0]),
                  &mod_input[0], BitsToBytes(num_bits), mod_order_bits);

  for (size_t i = 0; i < num_subcarriers; i++) {
    modulated_codeword[i] = ModSingleUint8(mod_input[i], mod_table);
  }
  return modulated_codeword;
}

std::vector<complex_float> DataGenerator::GetModulation(
    const int8_t* encoded_codeword, uint8_t* modulation_data,
    Table<complex_float> mod_table, const size_t num_bits,
    const size_t num_subcarriers, const size_t mod_order_bits) {
  std::vector<complex_float> modulated_codeword(num_subcarriers,
                                                {0, 0});  // Zeroed

  AdaptBitsForMod(reinterpret_cast<const uint8_t*>(&encoded_codeword[0]),
                  modulation_data, BitsToBytes(num_bits), mod_order_bits);

  for (size_t i = 0; i < num_subcarriers; i++) {
    modulated_codeword[i] = ModSingleUint8(modulation_data[i], mod_table);
  }
  return modulated_codeword;
}

std::vector<complex_float> DataGenerator::MapOFDMSymbol(
    Config* cfg, const std::vector<complex_float>& modulated_codeword,
    const complex_float* pilot_seq, SymbolType symbol_type) {
  std::vector<complex_float> ofdm_symbol;
  for (size_t i = 0; i < cfg->OfdmDataNum(); i++) {
    if (symbol_type == SymbolType::kUL) {
      if (i < modulated_codeword.size()) {
        ofdm_symbol.push_back(modulated_codeword.at(i));
      }
    } else if (symbol_type == SymbolType::kDL) {
      if (cfg->IsDataSubcarrier(i) == true) {
        size_t data_idx = cfg->GetOFDMDataIndex(i);
        if (data_idx < modulated_codeword.size()) {
          ofdm_symbol.push_back(modulated_codeword.at(data_idx));
        }
      } else {
        ofdm_symbol.push_back(pilot_seq[i]);
      }
    } else if (symbol_type == SymbolType::kControl) {
      if (cfg->IsControlSubcarrier(i) == true) {
        size_t ctrl_idx = cfg->GetOFDMCtrlIndex(i);
        if (ctrl_idx < modulated_codeword.size()) {
          ofdm_symbol.push_back(modulated_codeword.at(ctrl_idx));
        }
      } else {
        ofdm_symbol.push_back(pilot_seq[i]);
      }
    }
  }
  return ofdm_symbol;
}

/**
   * @param modulated_codeword The modulated codeword with OfdmDataNum()
   * elements
   * @brief An array with OfdmDataNum() elements with the OfdmDataNum()
   * modulated elements binned at the center
   */
std::vector<complex_float> DataGenerator::BinForIfft(
    Config* cfg, const std::vector<complex_float>& modulated_codeword,
    bool is_fftshifted) {
  std::vector<complex_float> pre_ifft_symbol(cfg->OfdmCaNum());  // Zeroed
  std::memcpy(&pre_ifft_symbol[cfg->OfdmDataStart()], &modulated_codeword[0],
              cfg->OfdmDataNum() * sizeof(complex_float));

  return is_fftshifted ? CommsLib::FFTShift(pre_ifft_symbol) : pre_ifft_symbol;
}

void DataGenerator::GetNoisySymbol(complex_float* modulated_symbol,
                                   size_t length, float noise_level,
                                   unsigned seed) {
  std::default_random_engine generator(seed);
  std::normal_distribution<double> distribution(0.0, 1.0);
  for (size_t j = 0; j < length; j++) {
    complex_float noise = {
        static_cast<float>(distribution(generator)) * noise_level,
        static_cast<float>(distribution(generator)) * noise_level};
    modulated_symbol[j].re += noise.re;
    modulated_symbol[j].im += noise.im;
  }
}

void DataGenerator::GetDecodedData(int8_t* demoded_data,
                                   uint8_t* decoded_codewords,
                                   const LDPCconfig& ldpc_config,
                                   size_t num_decoded_bytes,
                                   bool scramble_enabled) {
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
  auto* resp_var_nodes = static_cast<int16_t*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, 1024 * 1024 * sizeof(int16_t)));
  ldpc_decoder_5gnr_response.varNodes = resp_var_nodes;

  ldpc_decoder_5gnr_request.varNodes = demoded_data;
  ldpc_decoder_5gnr_response.compactedMessageBytes = decoded_codewords;
  bblib_ldpc_decoder_5gnr(&ldpc_decoder_5gnr_request,
                          &ldpc_decoder_5gnr_response);
  if (scramble_enabled) {
    auto scrambler = std::make_unique<AgoraScrambler::Scrambler>();
    scrambler->Descramble(decoded_codewords, num_decoded_bytes);
  }
  std::free(resp_var_nodes);
}

void DataGenerator::GetDecodedDataBatch(Table<int8_t>& demoded_data,
                                        Table<uint8_t>& decoded_codewords,
                                        const LDPCconfig& ldpc_config,
                                        size_t num_codeblocks,
                                        size_t num_decoded_bytes,
                                        bool scramble_enabled) {
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
  auto* resp_var_nodes = static_cast<int16_t*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, 1024 * 1024 * sizeof(int16_t)));
  ldpc_decoder_5gnr_response.varNodes = resp_var_nodes;

  for (size_t i = 0; i < num_codeblocks; i++) {
    ldpc_decoder_5gnr_request.varNodes = demoded_data[i];
    ldpc_decoder_5gnr_response.compactedMessageBytes = decoded_codewords[i];
    bblib_ldpc_decoder_5gnr(&ldpc_decoder_5gnr_request,
                            &ldpc_decoder_5gnr_response);
    if (scramble_enabled) {
      auto scrambler = std::make_unique<AgoraScrambler::Scrambler>();
      scrambler->Descramble(decoded_codewords[i], num_decoded_bytes);
    }
  }
  std::free(resp_var_nodes);
}

size_t DataGenerator::DecodeBroadcastSlots(
    Config* cfg, const int16_t* const bcast_iq_samps) {
  size_t start_tsc = GetTime::WorkerRdtsc();
  size_t delay_offset = (cfg->OfdmRxZeroPrefixClient() + cfg->CpLen()) * 2;
  complex_float* bcast_fft_buff = static_cast<complex_float*>(
      Agora_memory::PaddedAlignedAlloc(Agora_memory::Alignment_t::kAlign64,
                                       cfg->OfdmCaNum() * sizeof(float) * 2));
  SimdConvertShortToFloat(&bcast_iq_samps[delay_offset],
                          reinterpret_cast<float*>(bcast_fft_buff),
                          cfg->OfdmCaNum() * 2);
  CommsLib::FFT(bcast_fft_buff, cfg->OfdmCaNum());
  CommsLib::FFTShift(bcast_fft_buff, cfg->OfdmCaNum());
  auto* bcast_buff_complex = reinterpret_cast<arma::cx_float*>(bcast_fft_buff);

  const size_t sc_num = cfg->GetOFDMCtrlNum();
  const size_t ctrl_sc_num =
      cfg->BcLdpcConfig().NumCbCodewLen() / cfg->BcModOrderBits();
  std::vector<arma::cx_float> csi_buff(cfg->OfdmDataNum());
  arma::cx_float* eq_buff =
      static_cast<arma::cx_float*>(Agora_memory::PaddedAlignedAlloc(
          Agora_memory::Alignment_t::kAlign64, sc_num * sizeof(float) * 2));

  // estimate channel from pilot subcarriers
  float phase_shift = 0;
  for (size_t j = 0; j < cfg->OfdmDataNum(); j++) {
    size_t sc_id = j + cfg->OfdmDataStart();
    complex_float p = cfg->Pilots()[j];
    if (j % cfg->OfdmPilotSpacing() == 0) {
      csi_buff.at(j) = (bcast_buff_complex[sc_id] / arma::cx_float(p.re, p.im));
    } else {
      ///\todo not correct when 0th subcarrier is not pilot
      csi_buff.at(j) = csi_buff.at(j - 1);
      if (j % cfg->OfdmPilotSpacing() == 1) {
        phase_shift += arg((bcast_buff_complex[sc_id] / csi_buff.at(j)) *
                           arma::cx_float(p.re, -p.im));
      }
    }
  }
  phase_shift /= cfg->GetOFDMPilotNum();
  for (size_t j = 0; j < cfg->OfdmDataNum(); j++) {
    size_t sc_id = j + cfg->OfdmDataStart();
    if (cfg->IsControlSubcarrier(j) == true) {
      eq_buff[cfg->GetOFDMCtrlIndex(j)] =
          (bcast_buff_complex[sc_id] / csi_buff.at(j)) *
          exp(arma::cx_float(0, -phase_shift));
    }
  }
  int8_t* demod_buff_ptr = static_cast<int8_t*>(
      Agora_memory::PaddedAlignedAlloc(Agora_memory::Alignment_t::kAlign64,
                                       cfg->BcModOrderBits() * ctrl_sc_num));
  Demodulate(reinterpret_cast<float*>(&eq_buff[0]), demod_buff_ptr,
             2 * ctrl_sc_num, cfg->BcModOrderBits(), false);

  const int num_bcast_bytes = BitsToBytes(cfg->BcLdpcConfig().NumCbLen());
  std::vector<uint8_t> decode_buff(num_bcast_bytes, 0u);

  DataGenerator::GetDecodedData(demod_buff_ptr, &decode_buff.at(0),
                                cfg->BcLdpcConfig(), num_bcast_bytes,
                                cfg->ScrambleEnabled());
  FreeBuffer1d(&bcast_fft_buff);
  FreeBuffer1d(&eq_buff);
  FreeBuffer1d(&demod_buff_ptr);
  const double duration =
      GetTime::CyclesToUs(GetTime::WorkerRdtsc() - start_tsc, cfg->FreqGhz());
  if (kDebugPrintInTask) {
    std::printf("DecodeBroadcast completed in %2.2f us\n", duration);
  }
  return (reinterpret_cast<size_t*>(decode_buff.data()))[0];
}

void DataGenerator::GenBroadcastSlots(
    Config* cfg, std::vector<std::complex<int16_t>*>& bcast_iq_samps,
    std::vector<size_t> ctrl_msg) {
  ///\todo enable a vector of bytes to TX'ed in each symbol
  assert(bcast_iq_samps.size() == cfg->Frame().NumDlControlSyms());
  const size_t start_tsc = GetTime::WorkerRdtsc();

  int num_bcast_bytes = BitsToBytes(cfg->BcLdpcConfig().NumCbLen());
  std::vector<int8_t> bcast_bits_buffer(num_bcast_bytes, 0);

  Table<complex_float> dl_bcast_mod_table;
  InitModulationTable(dl_bcast_mod_table, cfg->BcModOrderBits());

  for (size_t i = 0; i < cfg->Frame().NumDlControlSyms(); i++) {
    std::memcpy(bcast_bits_buffer.data(), ctrl_msg.data(), sizeof(size_t));

    const auto coded_bits_ptr = DataGenerator::GenCodeblock(
        cfg->BcLdpcConfig(), &bcast_bits_buffer.at(0), num_bcast_bytes,
        cfg->ScrambleEnabled());

    auto modulated_vector =
        DataGenerator::GetModulation(&coded_bits_ptr[0], dl_bcast_mod_table,
                                     cfg->BcLdpcConfig().NumCbCodewLen(),
                                     cfg->OfdmDataNum(), cfg->BcModOrderBits());
    auto mapped_symbol = DataGenerator::MapOFDMSymbol(
        cfg, modulated_vector, cfg->Pilots(), SymbolType::kControl);
    auto ofdm_symbol = DataGenerator::BinForIfft(cfg, mapped_symbol, true);
    CommsLib::IFFT(&ofdm_symbol[0], cfg->OfdmCaNum(), false);
    // additional 2^2 (6dB) power backoff
    float dl_bcast_scale =
        2 * CommsLib::FindMaxAbs(&ofdm_symbol[0], ofdm_symbol.size());
    CommsLib::Ifft2tx(&ofdm_symbol[0], bcast_iq_samps[i], cfg->OfdmCaNum(),
                      cfg->OfdmTxZeroPrefix(), cfg->CpLen(), dl_bcast_scale);
  }
  dl_bcast_mod_table.Free();
  const double duration =
      GetTime::CyclesToUs(GetTime::WorkerRdtsc() - start_tsc, cfg->FreqGhz());
  if (kDebugPrintInTask) {
    std::printf("GenBroadcast completed in %2.2f us\n", duration);
  }
}

void DataGenerator::GenerateUlTxTestVectors(Config* const cfg) {
  //Make sure the directory exists
  if (std::filesystem::is_directory(kExperimentFilepath) == false) {
    std::filesystem::create_directory(kExperimentFilepath);
  }
  std::unique_ptr<DoCRC> crc_obj = std::make_unique<DoCRC>();
  const size_t ul_cb_bytes = cfg->NumBytesPerCb(Direction::kUplink);
  LDPCconfig ul_ldpc_config = cfg->LdpcConfig(Direction::kUplink);
  const size_t num_ul_mac_bytes = cfg->MacBytesNumPerframe(Direction::kUplink);
  if (num_ul_mac_bytes > 0) {
    std::vector<std::vector<int8_t>> ul_mac_info(cfg->UeAntNum());
    AGORA_LOG_INFO("Total number of uplink MAC bytes: %zu\n", num_ul_mac_bytes);
    for (size_t ue_id = 0; ue_id < cfg->UeAntNum(); ue_id++) {
      ul_mac_info.at(ue_id).resize(num_ul_mac_bytes);
      for (size_t pkt_id = 0;
           pkt_id < cfg->MacPacketsPerframe(Direction::kUplink); pkt_id++) {
        size_t pkt_offset = pkt_id * cfg->MacPacketLength(Direction::kUplink);
        auto* pkt = reinterpret_cast<MacPacketPacked*>(
            &ul_mac_info.at(ue_id).at(pkt_offset));

        pkt->Set(0, pkt_id, ue_id,
                 cfg->MacPayloadMaxLength(Direction::kUplink));
        DataGenerator::GenMacRandomBits(pkt);
        pkt->Crc((uint16_t)(
            crc_obj->CalculateCrc24(
                pkt->Data(), cfg->MacPayloadMaxLength(Direction::kUplink)) &
            0xFFFF));
      }
    }

    const size_t symbol_blocks =
        ul_ldpc_config.NumBlocksInSymbol() * cfg->UeAntNum();
    const size_t num_ul_codeblocks =
        cfg->Frame().NumUlDataSyms() * symbol_blocks;

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

      int8_t* cb_start = &ul_mac_info.at(ue_id).at(ue_cb_cnt * ul_cb_bytes);
      ul_information.at(cb) =
          std::vector<int8_t>(cb_start, cb_start + ul_cb_bytes);
      ul_encoded_codewords.at(cb) = DataGenerator::GenCodeblock(
          ul_ldpc_config, &ul_information.at(cb).at(0), ul_cb_bytes,
          cfg->ScrambleEnabled());
    }

    {
      const std::string filename_input =
          kExperimentFilepath + kUlLdpcDataPrefix +
          std::to_string(cfg->OfdmCaNum()) + "_ue" +
          std::to_string(cfg->UeAntNum()) + ".bin";
      AGORA_LOG_INFO("Saving raw uplink data (using LDPC) to %s\n",
                     filename_input.c_str());
      for (size_t i = 0; i < num_ul_codeblocks; i++) {
        Utils::WriteBinaryFile(filename_input, sizeof(uint8_t), ul_cb_bytes,
                               ul_information.at(i).data(),
                               i != 0);  //Do not append in the first write
      }
    }

    // Modulate the encoded codewords
    std::vector<std::vector<uint8_t>> ul_modulated_codewords(num_ul_codeblocks);
    std::vector<std::vector<complex_float>> ul_modulated_symbols(
        num_ul_codeblocks);
    for (size_t i = 0; i < num_ul_codeblocks; i++) {
      ul_modulated_codewords.at(i).resize(cfg->OfdmDataNum());
      auto ofdm_symbol = DataGenerator::GetModulation(
          &ul_encoded_codewords.at(i)[0], &ul_modulated_codewords.at(i).at(0),
          cfg->ModTable(Direction::kUplink),
          cfg->LdpcConfig(Direction::kUplink).NumCbCodewLen(),
          cfg->OfdmDataNum(), cfg->ModOrderBits(Direction::kUplink));
      ul_modulated_symbols.at(i) = DataGenerator::MapOFDMSymbol(
          cfg, ofdm_symbol, nullptr, SymbolType::kUL);
    }

    {
      const std::string filename_input =
          kExperimentFilepath + kUlModDataPrefix +
          std::to_string(cfg->OfdmCaNum()) + "_ue" +
          std::to_string(cfg->UeAntNum()) + ".bin";
      AGORA_LOG_INFO("Saving modulated uplink data to %s\n",
                     filename_input.c_str());
      for (size_t i = 0; i < num_ul_codeblocks; i++) {
        Utils::WriteBinaryFile(filename_input, sizeof(uint8_t),
                               cfg->OfdmDataNum(),
                               ul_modulated_codewords.at(i).data(),
                               i != 0);  //Do not append in the first write
      }
    }
    std::vector<std::vector<complex_float>> pre_ifft_data_syms(
        cfg->UeAntNum() * cfg->Frame().NumUlDataSyms());
    for (size_t i = 0; i < pre_ifft_data_syms.size(); i++) {
      pre_ifft_data_syms.at(i) =
          DataGenerator::BinForIfft(cfg, ul_modulated_symbols.at(i));
    }

    std::vector<std::vector<complex_float>> tx_data_symbols(
        cfg->Frame().NumUlDataSyms());
    // Populate the UL symbols
    for (size_t i = 0; i < cfg->Frame().NumUlDataSyms(); i++) {
      tx_data_symbols.at(i).resize(cfg->UeAntNum() * cfg->OfdmCaNum());
      for (size_t j = 0; j < cfg->UeAntNum(); j++) {
        const size_t k = i - cfg->Frame().ClientUlPilotSymbols();
        std::memcpy(&tx_data_symbols.at(i).at(0) + (j * cfg->OfdmCaNum()),
                    &pre_ifft_data_syms.at(k * cfg->UeAntNum() + j).at(0),
                    cfg->OfdmCaNum() * sizeof(complex_float));
      }
    }

    {
      const std::string filename_tx = kExperimentFilepath + kUlIfftPrefix +
                                      std::to_string(cfg->OfdmCaNum()) + "_ue" +
                                      std::to_string(cfg->UeAntNum()) + ".bin";
      AGORA_LOG_INFO("Saving UL tx data to %s\n", filename_tx.c_str());
      for (size_t i = 0; i < cfg->Frame().NumUlDataSyms(); i++) {
        Utils::WriteBinaryFile(filename_tx, sizeof(complex_float),
                               cfg->OfdmCaNum() * cfg->UeAntNum(),
                               tx_data_symbols.at(i).data(),
                               i != 0);  //Do not append in the first write
      }
    }
  }
}

void DataGenerator::GenerateDlTxTestVectors(Config* const cfg,
                                            Table<complex_float>& dmrs) {
  //Make sure the directory exists
  if (std::filesystem::is_directory(kExperimentFilepath) == false) {
    std::filesystem::create_directory(kExperimentFilepath);
  }
  assert(dmrs.Dim1() == cfg->UeAntNum());
  std::unique_ptr<DoCRC> crc_obj = std::make_unique<DoCRC>();
  const LDPCconfig dl_ldpc_config = cfg->LdpcConfig(Direction::kDownlink);
  const size_t dl_cb_bytes = cfg->NumBytesPerCb(Direction::kDownlink);
  const size_t num_dl_mac_bytes =
      cfg->MacBytesNumPerframe(Direction::kDownlink);
  std::vector<std::vector<int8_t>> dl_mac_info(cfg->UeAntNum());
  if (num_dl_mac_bytes > 0) {
    for (size_t ue_id = 0; ue_id < cfg->UeAntNum(); ue_id++) {
      dl_mac_info[ue_id].resize(num_dl_mac_bytes);
      for (size_t pkt_id = 0;
           pkt_id < cfg->MacPacketsPerframe(Direction::kDownlink); pkt_id++) {
        size_t pkt_offset = pkt_id * cfg->MacPacketLength(Direction::kDownlink);
        auto* pkt = reinterpret_cast<MacPacketPacked*>(
            &dl_mac_info.at(ue_id).at(pkt_offset));

        pkt->Set(0, pkt_id, ue_id,
                 cfg->MacPayloadMaxLength(Direction::kDownlink));
        DataGenerator::GenMacRandomBits(pkt);
        pkt->Crc((uint16_t)(
            crc_obj->CalculateCrc24(
                pkt->Data(), cfg->MacPayloadMaxLength(Direction::kDownlink)) &
            0xFFFF));
      }
    }

    const size_t symbol_blocks =
        dl_ldpc_config.NumBlocksInSymbol() * cfg->UeAntNum();
    const size_t num_dl_codeblocks =
        cfg->Frame().NumDlDataSyms() * symbol_blocks;

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

    {
      // Save downlink information bytes to file
      const std::string filename_input =
          kExperimentFilepath + kDlLdpcDataPrefix +
          std::to_string(cfg->OfdmCaNum()) + "_ue" +
          std::to_string(cfg->UeAntNum()) + ".bin";
      AGORA_LOG_INFO("Saving raw dl data (using LDPC) to %s\n",
                     filename_input.c_str());
      for (size_t i = 0; i < num_dl_codeblocks; i++) {
        Utils::WriteBinaryFile(filename_input, sizeof(uint8_t), dl_cb_bytes,
                               dl_information.at(i).data(),
                               i != 0);  //Do not append in the first write
      }
    }

    // Modulate the encoded codewords
    std::vector<std::vector<uint8_t>> dl_modulated_codewords(num_dl_codeblocks);
    std::vector<std::vector<complex_float>> dl_modulated_symbols(
        num_dl_codeblocks);
    for (size_t i = 0; i < num_dl_codeblocks; i++) {
      const size_t sym_offset = i % (symbol_blocks);
      const size_t ue_id = sym_offset / dl_ldpc_config.NumBlocksInSymbol();
      dl_modulated_codewords.at(i).resize(cfg->OfdmDataNum());
      auto ofdm_symbol = DataGenerator::GetModulation(
          &dl_encoded_codewords.at(i)[0], &dl_modulated_codewords.at(i)[0],
          cfg->ModTable(Direction::kDownlink),
          cfg->LdpcConfig(Direction::kDownlink).NumCbCodewLen(),
          cfg->OfdmDataNum(), cfg->ModOrderBits(Direction::kDownlink));
      dl_modulated_symbols.at(i) = DataGenerator::MapOFDMSymbol(
          cfg, ofdm_symbol, dmrs[ue_id], SymbolType::kDL);
    }
    {
      const std::string filename_input =
          kExperimentFilepath + kDlModDataPrefix +
          std::to_string(cfg->OfdmCaNum()) + "_ue" +
          std::to_string(cfg->UeAntNum()) + ".bin";
      AGORA_LOG_INFO("Saving modulated downlink data to %s\n",
                     filename_input.c_str());
      for (size_t i = 0; i < num_dl_codeblocks; i++) {
        Utils::WriteBinaryFile(filename_input, sizeof(uint8_t),
                               cfg->OfdmDataNum(),
                               dl_modulated_codewords.at(i).data(),
                               i != 0);  //Do not append in the first write
      }
    }
    std::vector<std::vector<complex_float>> pre_ifft_data_syms(
        cfg->UeAntNum() * cfg->Frame().NumDlDataSyms());
    for (size_t i = 0; i < pre_ifft_data_syms.size(); i++) {
      pre_ifft_data_syms.at(i) =
          DataGenerator::BinForIfft(cfg, dl_modulated_symbols.at(i));
    }

    std::vector<std::vector<complex_float>> tx_data_symbols(
        cfg->Frame().NumDlDataSyms());
    // Populate the UL symbols
    for (size_t i = 0; i < cfg->Frame().NumDlDataSyms(); i++) {
      tx_data_symbols.at(i).resize(cfg->UeAntNum() * cfg->OfdmCaNum());
      for (size_t j = 0; j < cfg->UeAntNum(); j++) {
        const size_t k = i - cfg->Frame().ClientDlPilotSymbols();
        std::memcpy(&tx_data_symbols.at(i).at(0) + (j * cfg->OfdmCaNum()),
                    &pre_ifft_data_syms.at(k * cfg->UeAntNum() + j).at(0),
                    cfg->OfdmCaNum() * sizeof(complex_float));
      }
    }

    {
      const std::string filename_tx = kExperimentFilepath + kDlIfftPrefix +
                                      std::to_string(cfg->OfdmCaNum()) + "_ue" +
                                      std::to_string(cfg->UeAntNum()) + ".bin";
      AGORA_LOG_INFO("Saving UL tx data to %s\n", filename_tx.c_str());
      for (size_t i = 0; i < cfg->Frame().NumDlDataSyms(); i++) {
        Utils::WriteBinaryFile(filename_tx, sizeof(complex_float),
                               cfg->OfdmCaNum() * cfg->UeAntNum(),
                               tx_data_symbols.at(i).data(),
                               i != 0);  //Do not append in the first write
      }
    }
  }
}
