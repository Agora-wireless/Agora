#pragma once

#include <string>

#include "config.hpp"
#include "utils_ldpc.hpp"

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
    k123
  };

  DataGenerator(Config* cfg, uint64_t seed = 0,
                Profile profile = Profile::kRandom)
      : cfg_(cfg), kProfile(profile) {
    if (seed != 0) {
      fast_rand_.seed_ = seed;
    }
  }

  /**
   * @brief Generate one information bit sequence and the corresponding
   * encoded bit sequence for one code block for the active LDPC configuration
   *
   * @param information The generated input bit sequence
   * @param encoded_codeword The generated encoded codeword bit sequence
   * @param ue_id ID of the UE that this codeblock belongs to
   */
  void gen_codeblock(std::vector<int8_t>& information,
                     std::vector<int8_t>& encoded_codeword, size_t ue_id) {
    const LDPCconfig& lc = cfg_->ldpc_config();
    std::vector<int8_t> parity;
    parity.resize(
        ldpc_encoding_parity_buf_size(lc.base_graph(), lc.expansion_factor()));

    information.resize(
        ldpc_encoding_input_buf_size(lc.base_graph(), lc.expansion_factor()));
    encoded_codeword.resize(
        ldpc_encoding_encoded_buf_size(lc.base_graph(), lc.expansion_factor()));

    for (size_t i = 0; i < lc.numInputBytes(); i++) {
      if (kProfile == Profile::kRandom) {
        information.at(i) = static_cast<int8_t>(fast_rand_.next_u32());
      } else if (kProfile == Profile::k123) {
        information.at(i) = 1 + (ue_id * 3) + (i % 3);
      }
    }

    ldpc_encode_helper(cfg_->ldpc_config().base_graph(),
                       cfg_->ldpc_config().expansion_factor(),
                       cfg_->ldpc_config().num_rows(), &encoded_codeword.at(0),
                       &parity.at(0), &information.at(0));

    information.resize(lc.numInputBytes());
    encoded_codeword.resize(lc.numEncodedBytes());
  }

  /**
   * @brief Return the output of modulating the encoded codeword
   * @param encoded_codeword The encoded LDPC codeword bit sequence
   * @return An array of complex floats with ofdm_data_num() elements
   */
  std::vector<complex_float> get_modulation(
      const std::vector<int8_t>& encoded_codeword) {
    std::vector<complex_float> modulated_codeword(cfg_->ofdm_data_num());
    std::vector<uint8_t> mod_input(cfg_->ofdm_data_num());

    adapt_bits_for_mod(reinterpret_cast<const uint8_t*>(&encoded_codeword[0]),
                       &mod_input[0], cfg_->ldpc_config().numEncodedBytes(),
                       cfg_->mod_order_bits());

    for (size_t i = 0; i < cfg_->ofdm_data_num(); i++) {
      modulated_codeword[i] = mod_single_uint8(mod_input[i], cfg_->mod_table());
    }
    return modulated_codeword;
  }

  std::vector<complex_float> get_modulation(const int8_t* encoded_codeword,
                                            size_t num_bits) {
    std::vector<complex_float> modulated_codeword(cfg_->ofdm_data_num());
    std::vector<uint8_t> mod_input(cfg_->ofdm_data_num());

    adapt_bits_for_mod(reinterpret_cast<const uint8_t*>(&encoded_codeword[0]),
                       &mod_input[0], bits_to_bytes(num_bits),
                       cfg_->mod_order_bits());

    for (size_t i = 0; i < cfg_->ofdm_data_num(); i++) {
      modulated_codeword[i] = mod_single_uint8(mod_input[i], cfg_->mod_table());
    }
    return modulated_codeword;
  }

  /**
   * @param modulated_codeword The modulated codeword with ofdm_data_num()
   * elements
   * @brief An array with ofdm_ca_num() elements with the ofdm_data_num()
   * modulated elements binned at the center
   */
  std::vector<complex_float> bin_for_ifft(
      const std::vector<complex_float> modulated_codeword) const {
    std::vector<complex_float> pre_ifft_symbol(cfg_->ofdm_ca_num());  // Zeroed
    std::memcpy(&pre_ifft_symbol[cfg_->ofdm_data_start()],
                &modulated_codeword[0],
                cfg_->ofdm_data_num() * sizeof(complex_float));

    return pre_ifft_symbol;
  }

  /// Return the time-domain pilot symbol with ofdm_ca_num() complex floats
  std::vector<complex_float> get_common_pilot_time_domain() const {
    const std::vector<std::complex<float>> zc_seq = Utils::double_to_cfloat(
        CommsLib::getSequence(cfg_->ofdm_data_num(), CommsLib::LTE_ZADOFF_CHU));

    const std::vector<std::complex<float>> zc_common_pilot =
        CommsLib::seqCyclicShift(zc_seq, M_PI / 4.0);  // Used in LTE SRS

    std::vector<complex_float> ret(cfg_->ofdm_ca_num());  // Zeroed
    for (size_t i = 0; i < cfg_->ofdm_data_num(); i++) {
      ret[i + cfg_->ofdm_data_start()] = {zc_common_pilot[i].real(),
                                         zc_common_pilot[i].imag()};
    }

    return ret;
  }

 private:
  FastRand fast_rand_;     // A fast random number generator
  Config* cfg_;            // The global Agora config
  const Profile kProfile;  // The pattern of the input byte sequence
};
