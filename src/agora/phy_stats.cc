/**
 * @file phy_stats.cc
 * @brief Declaration file for the PhyStats class.
 */
#include "phy_stats.h"

#include <cmath>

using namespace arma;

PhyStats::PhyStats(Config* const cfg) : config_(cfg) {
  const size_t task_buffer_symbol_num_ul = cfg->Frame().NumULSyms() * kFrameWnd;
  decoded_bits_count_.Calloc(cfg->UeNum(), task_buffer_symbol_num_ul,
                             Agora_memory::Alignment_t::kAlign64);
  bit_error_count_.Calloc(cfg->UeNum(), task_buffer_symbol_num_ul,
                          Agora_memory::Alignment_t::kAlign64);

  decoded_blocks_count_.Calloc(cfg->UeNum(), task_buffer_symbol_num_ul,
                               Agora_memory::Alignment_t::kAlign64);
  block_error_count_.Calloc(cfg->UeNum(), task_buffer_symbol_num_ul,
                            Agora_memory::Alignment_t::kAlign64);

  uncoded_bits_count_.Calloc(cfg->UeNum(), task_buffer_symbol_num_ul,
                             Agora_memory::Alignment_t::kAlign64);
  uncoded_bit_error_count_.Calloc(cfg->UeNum(), task_buffer_symbol_num_ul,
                                  Agora_memory::Alignment_t::kAlign64);

  evm_buffer_.Calloc(kFrameWnd, cfg->UeAntNum(),
                     Agora_memory::Alignment_t::kAlign64);

  if (cfg->Frame().NumULSyms() > 0) {
    auto* ul_iq_f_ptr = reinterpret_cast<cx_float*>(
        cfg->UlIqF()[cfg->Frame().ClientUlPilotSymbols()]);
    cx_fmat ul_iq_f_mat(ul_iq_f_ptr, cfg->OfdmCaNum(), cfg->UeAntNum(), false);
    ul_gt_mat_ = ul_iq_f_mat.st();
    ul_gt_mat_ =
        ul_gt_mat_.cols(cfg->OfdmDataStart(), (cfg->OfdmDataStop() - 1));
  }
  pilot_snr_.Calloc(kFrameWnd, cfg->UeAntNum(),
                    Agora_memory::Alignment_t::kAlign64);
}

PhyStats::~PhyStats() {
  decoded_bits_count_.Free();
  bit_error_count_.Free();

  decoded_blocks_count_.Free();
  block_error_count_.Free();

  uncoded_bits_count_.Free();
  uncoded_bit_error_count_.Free();

  evm_buffer_.Free();
  pilot_snr_.Free();
}

void PhyStats::PrintPhyStats() {
  const size_t task_buffer_symbol_num_ul =
      this->config_->Frame().NumULSyms() * kFrameWnd;

  if (this->config_->Frame().NumULSyms() > 0) {
    for (size_t ue_id = 0; ue_id < this->config_->UeNum(); ue_id++) {
      size_t total_decoded_bits(0);
      size_t total_bit_errors(0);
      size_t total_decoded_blocks(0);
      size_t total_block_errors(0);

      for (size_t i = 0u; i < task_buffer_symbol_num_ul; i++) {
        total_decoded_bits += decoded_bits_count_[ue_id][i];
        total_bit_errors += bit_error_count_[ue_id][i];
        total_decoded_blocks += decoded_blocks_count_[ue_id][i];
        total_block_errors += block_error_count_[ue_id][i];
      }
      std::cout << "UE " << ue_id << ": bit errors (BER) " << total_bit_errors
                << "/" << total_decoded_bits << "("
                << 1.0 * total_bit_errors / total_decoded_bits
                << "), block errors (BLER) " << total_block_errors << "/"
                << total_decoded_blocks << " ("
                << 1.0 * total_block_errors / total_decoded_blocks << ")"
                << std::endl;
    }
  }
}

void PhyStats::PrintEvmStats(size_t frame_id) {
  fmat evm_mat(evm_buffer_[frame_id % kFrameWnd], config_->UeNum(), 1, false);
  evm_mat = sqrt(evm_mat) / config_->OfdmDataNum();
  std::stringstream ss;
  ss << "Frame " << frame_id << " Constellation:\n"
     << "  EVM " << 100 * evm_mat.st() << ", SNR " << -10 * log10(evm_mat.st());
  std::cout << ss.str();
}

float PhyStats::GetEvmSnr(size_t frame_id, size_t ue_id) {
  float evm = evm_buffer_[frame_id % kFrameWnd][ue_id];
  evm = std::sqrt(evm) / config_->OfdmDataNum();
  return -10 * std::log10(evm);
}

void PhyStats::PrintSnrStats(size_t frame_id) {
  std::stringstream ss;
  ss << "Frame " << frame_id << " Pilot Signal SNR: ";
  for (size_t i = 0; i < config_->UeNum(); i++) {
    ss << pilot_snr_[frame_id % kFrameWnd][i] << " ";
  }
  ss << std::endl;
  std::cout << ss.str();
}

void PhyStats::UpdatePilotSnr(size_t frame_id, size_t ue_id,
                              complex_float* fft_data) {
  cx_fmat fft_mat((cx_float*)fft_data, config_->OfdmCaNum(), 1, false);
  fmat fft_abs_mat = abs(fft_mat);
  fmat fft_abs_mag = fft_abs_mat % fft_abs_mat;
  float rssi = as_scalar(sum(fft_abs_mag));
  float noise_per_sc1 =
      as_scalar(mean(fft_abs_mag.rows(0, config_->OfdmDataStart() - 1)));
  float noise_per_sc2 = as_scalar(mean(
      fft_abs_mag.rows(config_->OfdmDataStop(), config_->OfdmCaNum() - 1)));
  float noise = config_->OfdmCaNum() * (noise_per_sc1 + noise_per_sc2) / 2;
  float snr = (rssi - noise) / noise;
  pilot_snr_[frame_id % kFrameWnd][ue_id] = 10 * std::log10(snr);
}

void PhyStats::UpdateEvmStats(size_t frame_id, size_t sc_id,
                              const cx_fmat& eq) {
  if (this->config_->Frame().NumULSyms() > 0) {
    fmat evm = abs(eq - ul_gt_mat_.col(sc_id));
    fmat cur_evm_mat(evm_buffer_[frame_id % kFrameWnd], config_->UeNum(), 1,
                     false);
    cur_evm_mat += evm % evm;
  }
}

void PhyStats::UpdateBitErrors(size_t ue_id, size_t offset, uint8_t tx_byte,
                               uint8_t rx_byte) {
  static constexpr size_t kBitsInByte = 8;
  // std::printf("Updating bit errors: %zu %zu %d %d\n", ue_id, offset, tx_byte,
  // rx_byte);
  uint8_t xor_byte(tx_byte ^ rx_byte);
  size_t bit_errors = 0;
  for (size_t j = 0; j < kBitsInByte; j++) {
    bit_errors += (xor_byte & 1);
    xor_byte >>= 1;
  }
  bit_error_count_[ue_id][offset] += bit_errors;
}

void PhyStats::UpdateDecodedBits(size_t ue_id, size_t offset,
                                 size_t new_bits_num) {
  decoded_bits_count_[ue_id][offset] += new_bits_num;
}

void PhyStats::UpdateBlockErrors(size_t ue_id, size_t offset,
                                 size_t block_error_count) {
  block_error_count_[ue_id][offset] +=
      static_cast<unsigned long>(block_error_count > 0);
}

void PhyStats::IncrementDecodedBlocks(size_t ue_id, size_t offset) {
  decoded_blocks_count_[ue_id][offset]++;
}

void PhyStats::UpdateUncodedBitErrors(size_t ue_id, size_t offset,
                                      size_t mod_bit_size, uint8_t tx_byte,
                                      uint8_t rx_byte) {
  uint8_t xor_byte(tx_byte ^ rx_byte);
  size_t bit_errors = 0;
  for (size_t j = 0; j < mod_bit_size; j++) {
    bit_errors += xor_byte & 1;
    xor_byte >>= 1;
  }
  uncoded_bit_error_count_[ue_id][offset] += bit_errors;
}

void PhyStats::UpdateUncodedBits(size_t ue_id, size_t offset,
                                 size_t new_bits_num) {
  uncoded_bits_count_[ue_id][offset] += new_bits_num;
}
