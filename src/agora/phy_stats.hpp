#ifndef PHY_STATS
#define PHY_STATS

#include <armadillo>

#include "Symbols.hpp"
#include "config.hpp"
#include "memory_manage.h"

using namespace arma;
class PhyStats {
 public:
  PhyStats(Config* const cfg);
  ~PhyStats();
  void PrintPhyStats();
  void UpdateBitErrors(size_t, size_t, uint8_t, uint8_t);
  void UpdateDecodedBits(size_t, size_t, size_t);
  void UpdateBlockErrors(size_t, size_t, size_t);
  void IncrementDecodedBlocks(size_t, size_t);
  void UpdateUncodedBitErrors(size_t, size_t, size_t, uint8_t, uint8_t);
  void UpdateUncodedBits(size_t, size_t, size_t);
  void UpdateEvmStats(size_t, size_t, const cx_fmat&);
  void PrintEvmStats(size_t);
  void UpdatePilotSnr(size_t, size_t, complex_float*);
  float GetEvmSnr(size_t frame_id, size_t ue_id);
  void PrintSnrStats(size_t);

 private:
  Config const* const kConfig;
  Table<size_t> decoded_bits_count_;
  Table<size_t> bit_error_count_;
  Table<size_t> decoded_blocks_count_;
  Table<size_t> block_error_count_;
  Table<size_t> uncoded_bits_count_;
  Table<size_t> uncoded_bit_error_count_;
  Table<float> evm_buffer_;
  Table<float> pilot_snr_;

  cx_fmat ul_gt_mat_;
};

#endif
