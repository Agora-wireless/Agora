/**
 * @file data_generator.h
 * @brief Implementation file for the Data generator class to generate binary
 * files as inputs to Agora, sender and correctness tests
 */
#ifndef DATA_GENERATOR_H_
#define DATA_GENERATOR_H_

#include <cstdint>
#include <string>
#include <vector>

#include "config.h"
#include "ldpc_config.h"
#include "memory_manage.h"
#include "message.h"

/**
 * @brief Building blocks for generating end-to-end or unit test workloads for
 * Agora
 */
class DataGenerator {
 public:
  // The profile of the input information bits
  enum class Profile {
    kRandom,  // The input information bytes are chosen at random

    // The input information bytes are {1, 2, 3, 1, 2, 3, ...} for UE 0,
    // {4, 5, 6, 4, 5, 6, ...} for UE 1, and so on
    kProfile123
  };

  explicit DataGenerator(Config* cfg, uint64_t seed = 0,
                         Profile profile = Profile::kRandom);

  /**
   * @brief                        Generate random Mac payload bit
   * sequence
   *
   * @param  information           The generated input bit sequence
   * @param  ue_id                 ID of the UE that this codeblock belongs to
   */
  void GenMacData(MacPacketPacked* mac, size_t ue_id);

  /**
   * @brief                        Generate one raw information bit sequence
   *
   * @param  information           The generated input bit sequence
   * @param  ue_id                 ID of the UE that this codeblock belongs to
   */
  void GenRawData(const LDPCconfig& lc, std::vector<int8_t>& information,
                  size_t ue_id);

  /**
   * @brief                        Generate the encoded bit sequence for one
   * code block for the active LDPC configuration from the input bit sequence
   *
   * @param  input_ptr             The input bit sequence to be encoded
   * @param  encoded_codeword      The generated encoded codeword bit sequence
   */
  static std::vector<int8_t> GenCodeblock(const LDPCconfig& lc,
                                          const int8_t* input_ptr,
                                          size_t input_size,
                                          bool scramble_enabled = false);

  /**
   * @brief Return the output of modulating the encoded codeword
   * @param encoded_codeword The encoded LDPC codeword bit sequence
   * @return An array of complex floats with OfdmDataNum() elements
   */
  static std::vector<complex_float> GetModulation(
      const int8_t* encoded_codeword, Table<complex_float> mod_table,
      const size_t num_bits, const size_t num_subcarriers,
      const size_t mod_order_bits);

  static std::vector<complex_float> MapOFDMSymbol(
      Config* cfg, const std::vector<complex_float>& modulated_codeword,
      complex_float* pilot_seq, SymbolType symbol_type);

  /**
   * @param modulated_codeword The modulated codeword with OfdmDataNum()
   * elements
   * @brief An array with OfdmDataNum() elements with the OfdmDataNum()
   * modulated elements binned at the center
   */
  static std::vector<complex_float> BinForIfft(
      Config* cfg, const std::vector<complex_float>& modulated_codeword,
      bool is_fftshifted = false);

  /// Return the frequency-domain pilot symbol with OfdmCaNum complex floats
  std::vector<complex_float> GetCommonPilotFreqDomain() const;

  /// Return the user-spepcific frequency-domain pilot symbol with OfdmCaNum complex floats
  Table<complex_float> GetUeSpecificPilotFreqDomain() const;

  void GetNoisySymbol(const std::vector<complex_float>& modulated_symbol,
                      std::vector<complex_float>& noisy_symbol,
                      float noise_level);
  void GetNoisySymbol(const complex_float* modulated_symbol,
                      complex_float* noisy_symbol, size_t length,
                      float noise_level);

  static void GetNoisySymbol(complex_float* modulated_symbol, size_t length,
                             float noise_level, unsigned seed = 0);

  static void GetDecodedData(int8_t* demoded_data, uint8_t* decoded_codewords,
                             const LDPCconfig& ldpc_config,
                             size_t num_decoded_bytes,
                             bool scramble_enabled = false);

  static void GetDecodedDataBatch(Table<int8_t>& demoded_data,
                                  Table<uint8_t>& decoded_codewords,
                                  const LDPCconfig& ldpc_config,
                                  size_t num_codeblocks,
                                  size_t num_decoded_bytes,
                                  bool scramble_enabled = false);

 private:
  FastRand fast_rand_;  // A fast random number generator
  Config* cfg_;         // The global Agora config
  uint64_t seed_;
  const Profile profile_;  // The pattern of the input byte sequence
};

#endif  // DATA_GENERATOR_H_
