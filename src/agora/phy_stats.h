/**
 * @file phy_stats.h
 * @brief Declaration file for the PhyStats class.
 */
#ifndef PHY_STATS_H_
#define PHY_STATS_H_

#include <armadillo>

#include "config.h"
#include "csv_logger.h"
#include "memory_manage.h"
#include "symbols.h"

class PhyStats {
 public:
  explicit PhyStats(Config* const cfg, Direction dir);
  ~PhyStats();
  void PrintPhyStats();
  void UpdateBitErrors(size_t ue_id, size_t offset, uint8_t tx_byte,
                       uint8_t rx_byte);
  void UpdateDecodedBits(size_t ue_id, size_t offset, size_t new_bits_num);
  void UpdateBlockErrors(size_t ue_id, size_t offset, size_t block_error_count);
  float GetBitErrorRate(size_t ue_id, size_t offset);
  void IncrementDecodedBlocks(size_t ue_id, size_t offset);
  void UpdateUncodedBitErrors(size_t ue_id, size_t offset, size_t mod_bit_size,
                              uint8_t tx_byte, uint8_t rx_byte);
  void UpdateUncodedBits(size_t ue_id, size_t offset, size_t new_bits_num);
  void UpdateEvmStats(size_t frame_id, size_t sc_id, const arma::cx_fmat& eq);
  void PrintEvmStats(size_t frame_id);
  float GetEvmSnr(size_t frame_id, size_t ue_id);
  void UpdatePilotSnr(size_t frame_id, size_t ue_id, size_t ant_id,
                      complex_float* fft_data);
  void UpdateDlPilotSnr(size_t frame_id, size_t symbol_id, size_t ant_id,
                        complex_float* fft_data);
  void PrintSnrStats(size_t frame_id);
  void PrintDlSnrStats(size_t frame_id, size_t ant_id);
  void RecordDlPilotSnr(CsvLog::CsvLogger* logger, size_t frame_id,
                        size_t ant_id);
  void UpdateCalibPilotSnr(size_t frame_id, size_t calib_sym_id, size_t ant_id,
                           complex_float* fft_data);
  void PrintCalibSnrStats(size_t frame_id);
  void UpdateCsiCond(size_t frame_id, size_t sc_id, float cond);
  void PrintZfStats(size_t frame_id);

 private:
  Config const* const config_;
  Direction dir_;
  Table<size_t> decoded_bits_count_;
  Table<size_t> bit_error_count_;
  Table<size_t> decoded_blocks_count_;
  Table<size_t> block_error_count_;
  Table<size_t> uncoded_bits_count_;
  Table<size_t> uncoded_bit_error_count_;
  Table<float> evm_buffer_;
  Table<float> pilot_snr_;
  Table<float> dl_pilot_snr_;
  Table<float> calib_pilot_snr_;
  Table<float> csi_cond_;

  arma::cx_fmat gt_mat_;
  size_t num_rx_symbols_;
};

#endif  // PHY_STATS_H_
