#ifndef LDPC_CONFIG_HPP_
#define LDPC_CONFIG_HPP_

#include <cstdint>
#include <cstdlib>

#include "utils_ldpc.hpp"

class LDPCconfig {
 public:
  LDPCconfig(uint16_t bg, uint16_t zc, int16_t max_dec_itr, bool early_term,
             uint32_t num_cb_len, uint32_t num_cb_codew_len, size_t num_rows,
             size_t num_blocks_in_symbol)
      : base_graph_(bg),
        expansion_factor_(zc),
        max_decoder_iter_(max_dec_itr),
        early_termination_(early_term),
        num_rows_(num_rows),
        num_cb_len_(num_cb_len),
        num_cb_codew_len_(num_cb_codew_len),
        num_blocks_in_symbol_(num_blocks_in_symbol) {}

  // Return the number of bytes in the information bit sequence for LDPC
  // encoding of one code block
  size_t NumInputBytes(void) const {
    return BitsToBytes(
        LdpcNumInputBits(this->base_graph_, this->expansion_factor_));
  }

  // Return the number of bytes in the encoded LDPC code word
  size_t NumEncodedBytes(void) const {
    return BitsToBytes(LdpcNumEncodedBits(
        this->base_graph_, this->expansion_factor_, this->num_rows_));
  }

  inline void NumBlocksInSymbol(size_t num_blocks) {
    this->num_blocks_in_symbol_ = num_blocks;
  }

  /* Accessors */
  inline uint16_t BaseGraph(void) const { return this->base_graph_; }
  inline uint16_t ExpansionFactor(void) const {
    return this->expansion_factor_;
  }
  inline int16_t MaxDecoderIter(void) const {
    return this->max_decoder_iter_;
  }
  inline bool EarlyTermination(void) const { return this->early_termination_; }
  inline uint32_t NumCbLen(void) const { return this->num_cb_len_; }
  inline uint32_t NumCbCodewLen(void) const {
    return this->num_cb_codew_len_;
  }
  inline size_t NumRows(void) const { return this->num_rows_; }
  inline size_t NumBlocksInSymbol(void) const {
    return this->num_blocks_in_symbol_;
  }

 private:
  LDPCconfig(void) {}

  uint16_t base_graph_;        /// The 5G NR LDPC base graph (one or two)
  uint16_t expansion_factor_;  /// The 5G NR LDPC expansion factor
  int16_t
      max_decoder_iter_;  /// Maximum number of decoder iterations per codeblock

  /// Allow the LDPC decoder to terminate without completing all iterations
  /// if it decodes the codeblock eariler
  bool early_termination_;

  size_t num_rows_;      /// Number of rows in the LDPC base graph to use
  uint32_t num_cb_len_;  /// Number of information bits input to LDPC encoding
  uint32_t
      num_cb_codew_len_;  /// Number of codeword bits output from LDPC encodings
  size_t num_blocks_in_symbol_;
};

#endif /* LDPC_CONFIG_HPP_ */