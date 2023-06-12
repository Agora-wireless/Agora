/**
 * @file ldpc_config.inc
 * @brief Self defined file for the LDPCconfig class.
 */

#ifndef LDPC_CONFIG_INC_
#define LDPC_CONFIG_INC_

#include <cstdint>
#include <cstdlib>

#include "utils_ldpc.h"

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
  size_t NumInputBytes() const {
    return BitsToBytes(
        LdpcNumInputBits(this->base_graph_, this->expansion_factor_));
  }

  // Return the number of bytes in the encoded LDPC code word
  size_t NumEncodedBytes() const {
    return BitsToBytes(LdpcNumEncodedBits(
        this->base_graph_, this->expansion_factor_, this->num_rows_));
  }

  inline void NumBlocksInSymbol(size_t num_blocks) {
    this->num_blocks_in_symbol_ = num_blocks;
  }

  /* Accessors */
  inline uint16_t BaseGraph() const { return this->base_graph_; }
  inline uint16_t ExpansionFactor() const { return this->expansion_factor_; }
  inline int16_t MaxDecoderIter() const { return this->max_decoder_iter_; }
  inline bool EarlyTermination() const { return this->early_termination_; }
  inline uint32_t NumCbLen() const { return this->num_cb_len_; }
  inline uint32_t NumCbCodewLen() const { return this->num_cb_codew_len_; }
  inline size_t NumRows() const { return this->num_rows_; }
  inline size_t NumBlocksInSymbol() const {
    return this->num_blocks_in_symbol_;
  }

 private:
  LDPCconfig() = default;

  /// The 5G NR LDPC base graph (one or two)
  uint16_t base_graph_;
  /// The 5G NR LDPC expansion factor
  uint16_t expansion_factor_;
  /// Maximum number of decoder iterations per codeblock
  int16_t max_decoder_iter_;

  /// Allow the LDPC decoder to terminate without completing all iterations
  /// if it decodes the codeblock eariler
  bool early_termination_;

  /// Number of rows in the LDPC base graph to use
  size_t num_rows_;
  /// Number of information bits input to LDPC encoding
  uint32_t num_cb_len_;
  /// Number of codeword bits output from LDPC encodings
  uint32_t num_cb_codew_len_;
  size_t num_blocks_in_symbol_;
};

#endif  // LDPC_CONFIG_INC_