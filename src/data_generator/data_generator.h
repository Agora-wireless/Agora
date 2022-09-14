/**
 * @file data_generator.h
 * @brief Implementation file for the Data generator class to generate binary
 * files as inputs to Agora, sender and correctness tests
 */
#ifndef DATA_GENERATOR_H_
#define DATA_GENERATOR_H_

#include <string>

#include "common_typedef_sdk.h"
#include "comms-lib.h"
#include "config.h"
#include "modulation.h"
#include "utils_ldpc.h"

/**
 * @brief Building blocks for generating end-to-end or unit test workloads for
 * Agora
 */
class DataGenerator {
 public:
  // The profile of the input information bits
  enum class Profile {
    kRandom,  // The input information bytes are chosen at random

    // The input informatioon bytes are {1, 2, 3, 1, 2, 3, ...} for UE 0,
    // {4, 5, 6, 4, 5, 6, ...} for UE 1, and so on
    kProfile123
  };

  explicit DataGenerator(Config* cfg, uint64_t seed = 0,
                         Profile profile = Profile::kRandom)
      : cfg_(cfg), profile_(profile) {
    if (seed != 0) {
      fast_rand_.seed_ = seed;
    }
  }

  void DoDataGeneration(const std::string& directory);

  /**
   * @brief                        Generate random Mac payload bit
   * sequence
   *
   * @param  information           The generated input bit sequence
   * @param  ue_id                 ID of the UE that this codeblock belongs to
   */
  void GenMacData(MacPacketPacked* mac, size_t ue_id) {
    for (size_t i = 0; i < mac->PayloadLength(); i++) {
      if (profile_ == Profile::kRandom) {
        mac->DataPtr()[i] = static_cast<int8_t>(fast_rand_.NextU32());
      } else if (profile_ == Profile::kProfile123) {
        mac->DataPtr()[i] = 1 + (ue_id * 3) + (i % 3);
      }
    }
  }

  /**
   * @brief                        Generate one raw information bit sequence
   *
   * @param  information           The generated input bit sequence
   * @param  ue_id                 ID of the UE that this codeblock belongs to
   */
  void GenRawData(Direction dir, std::vector<int8_t>& information,
                  size_t ue_id) {
    const LDPCconfig& lc = cfg_->LdpcConfig(dir);
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

  /**
   * @brief                        Generate the encoded bit sequence for one
   * code block for the active LDPC configuration from the input bit sequence
   *
   * @param  input_ptr             The input bit sequence to be encoded
   * @param  encoded_codeword      The generated encoded codeword bit sequence
   */
  void GenCodeblock(Direction dir, const int8_t* input_ptr,
                    std::vector<int8_t>& encoded_codeword) {
    const LDPCconfig& lc = cfg_->LdpcConfig(dir);
    std::vector<int8_t> parity;
    parity.resize(
        LdpcEncodingParityBufSize(lc.BaseGraph(), lc.ExpansionFactor()));

    encoded_codeword.resize(
        LdpcEncodingEncodedBufSize(lc.BaseGraph(), lc.ExpansionFactor()));

    LdpcEncodeHelper(lc.BaseGraph(), lc.ExpansionFactor(), lc.NumRows(),
                     &encoded_codeword.at(0), &parity.at(0), input_ptr);

    encoded_codeword.resize(lc.NumEncodedBytes());
  }

  /**
   * @brief Return the output of modulating the encoded codeword
   * @param encoded_codeword The encoded LDPC codeword bit sequence
   * @return An array of complex floats with OfdmDataNum() elements
   */
  std::vector<complex_float> GetModulation(
      const std::vector<int8_t>& encoded_codeword) {
    std::vector<complex_float> modulated_codeword(cfg_->OfdmDataNum());
    std::vector<uint8_t> mod_input(cfg_->OfdmDataNum());

    AdaptBitsForMod(reinterpret_cast<const uint8_t*>(&encoded_codeword[0]),
                    &mod_input[0],
                    cfg_->LdpcConfig(Direction::kUplink).NumEncodedBytes(),
                    cfg_->ModOrderBits(Direction::kUplink));

    for (size_t i = 0; i < cfg_->OfdmDataNum(); i++) {
      modulated_codeword[i] =
          ModSingleUint8(mod_input[i], cfg_->ModTable(Direction::kUplink));
    }
    return modulated_codeword;
  }

  std::vector<complex_float> GetDLModulation(
      const std::vector<int8_t>& encoded_codeword, complex_float* pilot_seq) {
    std::vector<complex_float> modulated_codeword(cfg_->OfdmDataNum());
    std::vector<uint8_t> mod_input(cfg_->GetOFDMDataNum());

    AdaptBitsForMod(reinterpret_cast<const uint8_t*>(&encoded_codeword[0]),
                    &mod_input[0],
                    cfg_->LdpcConfig(Direction::kDownlink).NumEncodedBytes(),
                    cfg_->ModOrderBits(Direction::kDownlink));

    for (size_t i = 0; i < cfg_->OfdmDataNum(); i++) {
      if (cfg_->IsDataSubcarrier(i) == true) {
        modulated_codeword[i] =
            ModSingleUint8(mod_input[cfg_->GetOFDMDataIndex(i)],
                           cfg_->ModTable(Direction::kDownlink));
      } else {
        modulated_codeword[i] = pilot_seq[i];
      }
    }
    return modulated_codeword;
  }

  std::vector<complex_float> GetModulation(const int8_t* encoded_codeword,
                                           size_t num_bits) {
    std::vector<complex_float> modulated_codeword(cfg_->OfdmDataNum());
    std::vector<uint8_t> mod_input(cfg_->OfdmDataNum());

    AdaptBitsForMod(reinterpret_cast<const uint8_t*>(&encoded_codeword[0]),
                    &mod_input[0], BitsToBytes(num_bits),
                    cfg_->ModOrderBits(Direction::kUplink));

    for (size_t i = 0; i < cfg_->OfdmDataNum(); i++) {
      modulated_codeword[i] =
          ModSingleUint8(mod_input[i], cfg_->ModTable(Direction::kUplink));
    }
    return modulated_codeword;
  }

  /**
   * @param modulated_codeword The modulated codeword with OfdmDataNum()
   * elements
   * @brief An array with OfdmDataNum() elements with the OfdmDataNum()
   * modulated elements binned at the center
   */
  std::vector<complex_float> BinForIfft(
      const std::vector<complex_float>& modulated_codeword) const {
    std::vector<complex_float> pre_ifft_symbol(cfg_->OfdmCaNum());  // Zeroed
    std::memcpy(&pre_ifft_symbol[cfg_->OfdmDataStart()], &modulated_codeword[0],
                cfg_->OfdmDataNum() * sizeof(complex_float));

    return pre_ifft_symbol;
  }

  /// Return the time-domain pilot symbol with OfdmCaNum complex floats
  std::vector<complex_float> GetCommonPilotTimeDomain() const {
    const std::vector<std::complex<float>> zc_seq = Utils::DoubleToCfloat(
        CommsLib::GetSequence(cfg_->OfdmDataNum(), CommsLib::kLteZadoffChu));

    const std::vector<std::complex<float>> zc_common_pilot =
        CommsLib::SeqCyclicShift(zc_seq, M_PI / 4.0);  // Used in LTE SRS

    std::vector<complex_float> ret(cfg_->OfdmCaNum());  // Zeroed
    for (size_t i = 0; i < cfg_->OfdmDataNum(); i++) {
      ret[i + cfg_->OfdmDataStart()] = {zc_common_pilot[i].real(),
                                        zc_common_pilot[i].imag()};
    }

    return ret;
  }

 private:
  FastRand fast_rand_;     // A fast random number generator
  Config* cfg_;            // The global Agora config
  const Profile profile_;  // The pattern of the input byte sequence
};

#endif  // DATA_GENERATOR_H_
