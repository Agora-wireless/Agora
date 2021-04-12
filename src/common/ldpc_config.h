/**
 * @file ldpc_config.h
 * @brief Self defined file for the LDPCconfig class.
 */

#ifndef LDPC_CONFIG_H_
#define LDPC_CONFIG_H_

#include <cstdint>
#include <cstdlib>

#include "utils_ldpc.h"

class LDPCconfig {
 public:
  LDPCconfig(uint16_t bg, uint16_t zc, int16_t max_dec_itr, bool early_term,
             uint32_t cb_len, uint32_t num_cb_codew_len, size_t num_rows,
             size_t num_blocks_in_symbol, float code_rate, size_t num_cbs)
      : base_graph_(bg),
        expansion_factor_(zc),
        max_decoder_iter_(max_dec_itr),
        early_termination_(early_term),
        num_rows_(num_rows),
        cb_len_(cb_len),
        num_cb_codew_len_(num_cb_codew_len),
        num_blocks_in_symbol_(num_blocks_in_symbol),
        code_rate_(code_rate),
        num_cbs_(num_cbs) {}

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

  inline void SetNumBlocksInSymbol(size_t num_blocks) {
    this->num_blocks_in_symbol_ = num_blocks;
  }
  inline void SetBaseGraph(uint16_t bg) { this->base_graph_ = bg; }
  inline void SetCodeRate(float code_rate) { this->code_rate_ = code_rate; }
  inline void SetNumRows(size_t num_rows) { this->num_rows_ = num_rows; }
  inline void SetNumCbs(size_t num_cbs) { this->num_cbs_ = num_cbs; }
  inline void SetCbLen(uint32_t cb_len) { this->cb_len_ = cb_len; }
  inline void SetZc(uint16_t zc) { this->expansion_factor_ = zc; }

  /* Accessors */
  inline uint16_t BaseGraph() const { return this->base_graph_; }
  inline uint16_t ExpansionFactor() const { return this->expansion_factor_; }
  inline int16_t MaxDecoderIter() const { return this->max_decoder_iter_; }
  inline bool EarlyTermination() const { return this->early_termination_; }
  inline uint32_t CbLen() const { return this->cb_len_; }
  inline uint32_t NumCbCodewLen() const { return this->num_cb_codew_len_; }
  inline size_t NumRows() const { return this->num_rows_; }
  inline size_t NumBlocksInSymbol() const {
    return this->num_blocks_in_symbol_;
  }
  inline size_t CodeRate() const { return this->code_rate_; }
  inline size_t NumCbs() const { return this->num_cbs_; }

  // Generate lookup tables for the mappling between symbols and code blocks
  void MapSymbolsToCbs(size_t symbol_num_perframe, size_t ofdm_data_num,
                       size_t mod_order_bits) {
    lut_symbol_to_cb_.resize(symbol_num_perframe);
    lut_cb_to_symbol_.resize(num_cbs_);
    lut_cb_chunks_bytes_.resize(symbol_num_perframe);
    lut_cb_chunks_scs_.resize(symbol_num_perframe);
    size_t num_encoded_bytes_per_symbol =
        BitsToBytes(ofdm_data_num * mod_order_bits);
    if (symbol_num_perframe % num_cbs_ == 0) {
      size_t symbol_per_cb = symbol_num_perframe / num_cbs_;
      for (size_t i = 0; i < num_cbs_; i++) {
        for (size_t j = 0; j < symbol_per_cb; j++) {
          lut_cb_to_symbol_[i].push_back(i * symbol_per_cb + j);
          // Use all subcarriers in symbols except the last symbol
          if (j == symbol_per_cb - 1) {
            lut_cb_chunks_bytes_[i].push_back(NumEncodedBytes() -
                                              j * num_encoded_bytes_per_symbol);
            lut_cb_chunks_scs_[i].push_back(num_cb_codew_len_ / mod_order_bits -
                                            j * ofdm_data_num);
          } else {
            lut_cb_chunks_bytes_[i].push_back(num_encoded_bytes_per_symbol);
            lut_cb_chunks_scs_[i].push_back(ofdm_data_num);
          }
        }
      }
      for (size_t i = 0; i < symbol_num_perframe; i++) {
        lut_symbol_to_cb_[i].push_back(i / symbol_per_cb);
      }
    } else {
      // Number of subcarriers required for a code block
      // Pad to 64 bytes to avoid cache false sharing
      size_t num_encoded_scs_pad =
          Roundup<64>(num_cb_codew_len_ / mod_order_bits);
      size_t unmapped_scs_in_symbol = ofdm_data_num;
      size_t symbol_id = 0;
      for (size_t i = 0; i < num_cbs_; i++) {
        size_t unmapped_scs_in_cb = num_encoded_scs_pad;
        while (unmapped_scs_in_cb > 0) {
          lut_cb_to_symbol_[i].push_back(symbol_id);
          lut_symbol_to_cb_[symbol_id].push_back(i);
          if (unmapped_scs_in_cb >= unmapped_scs_in_symbol) {
            lut_cb_chunks_bytes_[i].push_back(
                BitsToBytes(unmapped_scs_in_symbol * mod_order_bits));
            lut_cb_chunks_scs_[i].push_back(unmapped_scs_in_symbol);
            unmapped_scs_in_cb -= unmapped_scs_in_symbol;
            symbol_id++;
            unmapped_scs_in_symbol = ofdm_data_num;
          } else {
            lut_cb_chunks_bytes_[i].push_back(
                BitsToBytes(unmapped_scs_in_cb * mod_order_bits));
            lut_cb_chunks_scs_[i].push_back(unmapped_scs_in_cb);
            unmapped_scs_in_cb = 0;
            unmapped_scs_in_symbol -= unmapped_scs_in_cb;
          }
        }
      }
    }
  }

  // Return the offset of subcarrier for the start of a code block's
  // current chunk within a symbol
  size_t get_chunk_start_sc(size_t cb_id, size_t chunk_id) const {
    size_t symbol_id = lut_cb_to_symbol_[cb_id][chunk_id];
    const auto it = std::find(lut_symbol_to_cb_[symbol_id].begin(),
                              lut_symbol_to_cb_[symbol_id].end(), cb_id);
    RtAssert(it != lut_symbol_to_cb_[symbol_id].end(),
             "Code block does not exist in the symbol");
    size_t cb_id_in_symbol = it - lut_symbol_to_cb_[symbol_id].begin();
    size_t chunk_start_sc = 0;
    for (size_t i = 0; i < cb_id_in_symbol; i++)
      chunk_start_sc +=
          lut_cb_chunks_scs_[lut_symbol_to_cb_[symbol_id][i]].back();
    return chunk_start_sc;
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
  uint32_t cb_len_;
  /// Number of codeword bits output from LDPC encodings
  uint32_t num_cb_codew_len_;
  size_t num_blocks_in_symbol_;
  float code_rate_;
  /// Number of code blocks in a frame
  size_t num_cbs_;
  /// Lookup table that maps code blocks to symbols
  std::vector<std::vector<size_t>> lut_cb_to_symbol_;
  /// Lookup table that maps symbols to code blocks
  std::vector<std::vector<size_t>> lut_symbol_to_cb_;
  /// Lookup table for number of bytes within chunks of code blocks
  std::vector<std::vector<size_t>> lut_cb_chunks_bytes_;
  /// Lookup table for number of subcarriers occupied by chunks of code blocks
  std::vector<std::vector<size_t>> lut_cb_chunks_scs_;
};

#endif  // LDPC_CONFIG_H_