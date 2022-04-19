/**
 * @file phy_stats.cc
 * @brief Declaration file for the PhyStats class.
 */
#include "phy_stats.h"

#include <cfloat>
#include <cmath>

#include "logger.h"

PhyStats::PhyStats(Config* const cfg, Direction dir) : config_(cfg), dir_(dir) {
  if (dir_ == Direction::kDownlink) {
    num_rx_symbols_ = cfg->Frame().NumDLSyms();
  } else {
    num_rx_symbols_ = cfg->Frame().NumULSyms();
  }
  const size_t task_buffer_symbol_num = num_rx_symbols_ * kFrameWnd;

  decoded_bits_count_.Calloc(cfg->UeAntNum(), task_buffer_symbol_num,
                             Agora_memory::Alignment_t::kAlign64);
  bit_error_count_.Calloc(cfg->UeAntNum(), task_buffer_symbol_num,
                          Agora_memory::Alignment_t::kAlign64);

  decoded_blocks_count_.Calloc(cfg->UeAntNum(), task_buffer_symbol_num,
                               Agora_memory::Alignment_t::kAlign64);
  block_error_count_.Calloc(cfg->UeAntNum(), task_buffer_symbol_num,
                            Agora_memory::Alignment_t::kAlign64);

  uncoded_bits_count_.Calloc(cfg->UeAntNum(), task_buffer_symbol_num,
                             Agora_memory::Alignment_t::kAlign64);
  uncoded_bit_error_count_.Calloc(cfg->UeAntNum(), task_buffer_symbol_num,
                                  Agora_memory::Alignment_t::kAlign64);

  evm_buffer_.Calloc(kFrameWnd, cfg->UeAntNum(),
                     Agora_memory::Alignment_t::kAlign64);

  if (num_rx_symbols_ > 0) {
    if (dir_ == Direction::kDownlink) {
      auto* dl_iq_f_ptr = reinterpret_cast<arma::cx_float*>(
          cfg->DlIqF()[cfg->Frame().ClientDlPilotSymbols()]);
      arma::cx_fmat dl_iq_f_mat(dl_iq_f_ptr, cfg->OfdmCaNum(), cfg->UeAntNum(),
                                false);
      gt_mat_ = dl_iq_f_mat.st();
    } else {
      auto* ul_iq_f_ptr = reinterpret_cast<arma::cx_float*>(
          cfg->UlIqF()[cfg->Frame().ClientUlPilotSymbols()]);
      arma::cx_fmat ul_iq_f_mat(ul_iq_f_ptr, cfg->OfdmCaNum(), cfg->UeAntNum(),
                                false);
      gt_mat_ = ul_iq_f_mat.st();
    }
    gt_mat_ = gt_mat_.cols(cfg->OfdmDataStart(), (cfg->OfdmDataStop() - 1));
  }
  dl_pilot_snr_.Calloc(kFrameWnd,
                       cfg->UeAntNum() * cfg->Frame().ClientDlPilotSymbols(),
                       Agora_memory::Alignment_t::kAlign64);
  pilot_snr_.Calloc(kFrameWnd, cfg->UeAntNum() * cfg->BsAntNum(),
                    Agora_memory::Alignment_t::kAlign64);
  calib_pilot_snr_.Calloc(kFrameWnd, 2 * cfg->BsAntNum(),
                          Agora_memory::Alignment_t::kAlign64);
  csi_cond_.Calloc(kFrameWnd, cfg->OfdmDataNum(),
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
  csi_cond_.Free();
  calib_pilot_snr_.Free();
  dl_pilot_snr_.Free();
}

void PhyStats::PrintPhyStats() {
  const size_t task_buffer_symbol_num = num_rx_symbols_ * kFrameWnd;
  std::string tx_type;
  if (dir_ == Direction::kDownlink) {
    tx_type = "Downlink";
  } else {
    tx_type = "Uplink";
  }

  if (num_rx_symbols_ > 0) {
    for (size_t ue_id = 0; ue_id < this->config_->UeAntNum(); ue_id++) {
      size_t total_decoded_bits(0);
      size_t total_bit_errors(0);
      size_t total_decoded_blocks(0);
      size_t total_block_errors(0);

      for (size_t i = 0u; i < task_buffer_symbol_num; i++) {
        total_decoded_bits += decoded_bits_count_[ue_id][i];
        total_bit_errors += bit_error_count_[ue_id][i];
        total_decoded_blocks += decoded_blocks_count_[ue_id][i];
        total_block_errors += block_error_count_[ue_id][i];
      }

      AGORA_LOG_INFO(
          "UE %zu: %s bit errors (BER) %zu/%zu (%f), block errors (BLER) "
          "%zu/%zu (%f)\n",
          ue_id, tx_type.c_str(), total_bit_errors, total_decoded_bits,
          static_cast<float>(total_bit_errors) /
              static_cast<float>(total_decoded_bits),
          total_block_errors, total_decoded_blocks,
          static_cast<float>(total_block_errors) /
              static_cast<float>(total_decoded_blocks));
    }
  }
}

void PhyStats::PrintEvmStats(size_t frame_id) {
  arma::fmat evm_mat(evm_buffer_[frame_id % kFrameWnd], config_->UeAntNum(), 1,
                     false);
  evm_mat = evm_mat / config_->OfdmDataNum();

  std::stringstream ss;
  ss << "Frame " << frame_id << " Constellation:\n"
     << "  EVM " << (100.0f * evm_mat.st()) << ", SNR "
     << (-10.0f * log10(evm_mat.st()));
  AGORA_LOG_INFO("%s\n", ss.str().c_str());
}

float PhyStats::GetEvmSnr(size_t frame_id, size_t ue_id) {
  float evm = evm_buffer_[frame_id % kFrameWnd][ue_id];
  evm = evm / config_->OfdmDataNum();
  return (-10.0f * std::log10(evm));
}

void PhyStats::PrintDlSnrStats(size_t frame_id, size_t ant_id) {
  std::stringstream ss;
  ss << "Frame " << frame_id << " Pilot SNR (dB) at UE Antenna " << ant_id
     << ": [" << std::fixed << std::setw(5) << std::setprecision(1);
  size_t dl_pilots_num = config_->Frame().ClientDlPilotSymbols();
  for (size_t i = 0; i < dl_pilots_num; i++) {
    float frame_snr =
        dl_pilot_snr_[frame_id % kFrameWnd][ant_id * dl_pilots_num + i];
    ss << frame_snr << " ";
  }
  ss << "]" << std::endl;
  AGORA_LOG_INFO("%s", ss.str().c_str());
}

void PhyStats::RecordDlPilotSnr(CsvLog::CsvLogger* logger, size_t frame_id,
                                size_t ant_id) {
  if (kEnableCsvLog) {
    const size_t dl_pilots_num = config_->Frame().ClientDlPilotSymbols();
    if ((logger != nullptr) && (dl_pilots_num > 0)) {
      std::stringstream ss;
      ss << frame_id << "," << ant_id;
      for (size_t i = 0; i < dl_pilots_num; i++) {
        ss << ","
           << dl_pilot_snr_[frame_id % kFrameWnd][ant_id * dl_pilots_num + i];
      }
      logger->Write(ss.str());
    }
  }
}

void PhyStats::PrintSnrStats(size_t frame_id) {
  std::stringstream ss;
  ss << "Frame " << frame_id
     << " Pilot Signal SNR (dB) Range at BS Antennas: " << std::fixed
     << std::setw(5) << std::setprecision(1);
  for (size_t i = 0; i < config_->UeAntNum(); i++) {
    float max_snr = FLT_MIN;
    float min_snr = FLT_MAX;
    const float* frame_snr =
        &pilot_snr_[frame_id % kFrameWnd][i * config_->BsAntNum()];
    for (size_t j = 0; j < config_->BsAntNum(); j++) {
      const size_t radio_id = j / config_->NumChannels();
      const size_t cell_id = config_->CellId().at(radio_id);
      if (config_->ExternalRefNode(cell_id) == true &&
          radio_id == config_->RefRadio(cell_id)) {
        continue;
      }
      if (frame_snr[j] < min_snr) {
        min_snr = frame_snr[j];
      }
      if (frame_snr[j] > max_snr) {
        max_snr = frame_snr[j];
      }
    }
    if (min_snr == FLT_MAX) {
      min_snr = -100;
    }
    if (max_snr == FLT_MIN) {
      max_snr = -100;
    }
    ss << "User " << i << ": [" << min_snr << "," << max_snr << "]"
       << " ";
  }
  ss << std::endl;
  AGORA_LOG_INFO("%s", ss.str().c_str());
}

void PhyStats::PrintCalibSnrStats(size_t frame_id) {
  std::stringstream ss;
  ss << "Cal Index " << frame_id
     << " Calibration Pilot Signal SNR (dB) Range at BS Antennas: "
     << std::fixed << std::setw(5) << std::setprecision(1);
  for (size_t i = 0; i < 2; i++) {
    float max_snr = FLT_MIN;
    float min_snr = FLT_MAX;
    const float* frame_snr =
        &calib_pilot_snr_[frame_id % kFrameWnd][i * config_->BsAntNum()];
    for (size_t j = 0; j < config_->BsAntNum(); j++) {
      const size_t radio_id = j / config_->NumChannels();
      const size_t cell_id = config_->CellId().at(radio_id);
      if (config_->ExternalRefNode(cell_id) == true &&
          radio_id == config_->RefRadio(cell_id)) {
        continue;
      }
      if (frame_snr[j] < min_snr) {
        min_snr = frame_snr[j];
      }
      if (frame_snr[j] > max_snr) {
        max_snr = frame_snr[j];
      }
    }
    if (min_snr == FLT_MAX) {
      min_snr = -100;
    }
    if (max_snr == FLT_MIN) {
      max_snr = -100;
    }
    if (i == 0) {
      ss << "Downlink ";
    } else {
      ss << "Uplink ";
    }
    ss << ": [" << min_snr << "," << max_snr << "] ";
  }
  ss << std::endl;
  AGORA_LOG_INFO("%s", ss.str().c_str());
}

void PhyStats::UpdateCalibPilotSnr(size_t frame_id, size_t calib_sym_id,
                                   size_t ant_id, complex_float* fft_data) {
  const arma::cx_fmat fft_mat(reinterpret_cast<arma::cx_float*>(fft_data),
                              config_->OfdmCaNum(), 1, false);
  arma::fmat fft_abs_mat = arma::abs(fft_mat);
  arma::fmat fft_abs_mag = fft_abs_mat % fft_abs_mat;
  const float rssi = arma::as_scalar(arma::sum(fft_abs_mag));
  const float noise_per_sc1 = arma::as_scalar(
      arma::mean(fft_abs_mag.rows(0, config_->OfdmDataStart() - 1)));
  const float noise_per_sc2 = arma::as_scalar(arma::mean(
      fft_abs_mag.rows(config_->OfdmDataStop(), config_->OfdmCaNum() - 1)));
  const float noise =
      config_->OfdmCaNum() * (noise_per_sc1 + noise_per_sc2) / 2;
  const float snr = (rssi - noise) / noise;
  calib_pilot_snr_[frame_id % kFrameWnd][calib_sym_id * config_->BsAntNum() +
                                         ant_id] = (10.0f * std::log10(snr));
}

void PhyStats::UpdatePilotSnr(size_t frame_id, size_t ue_id, size_t ant_id,
                              complex_float* fft_data) {
  arma::cx_fmat fft_mat((arma::cx_float*)fft_data, config_->OfdmCaNum(), 1,
                        false);
  arma::fmat fft_abs_mat = abs(fft_mat);
  arma::fmat fft_abs_mag = fft_abs_mat % fft_abs_mat;
  const float rssi = arma::as_scalar(arma::sum(fft_abs_mag));
  const float noise_per_sc1 = arma::as_scalar(
      arma::mean(fft_abs_mag.rows(0, config_->OfdmDataStart() - 1)));
  const float noise_per_sc2 = arma::as_scalar(arma::mean(
      fft_abs_mag.rows(config_->OfdmDataStop(), config_->OfdmCaNum() - 1)));
  const float noise =
      config_->OfdmCaNum() * (noise_per_sc1 + noise_per_sc2) / 2;
  const float snr = (rssi - noise) / noise;
  pilot_snr_[frame_id % kFrameWnd][ue_id * config_->BsAntNum() + ant_id] =
      (10.0f * std::log10(snr));
}

void PhyStats::UpdateDlPilotSnr(size_t frame_id, size_t symbol_id,
                                size_t ant_id, complex_float* fft_data) {
  arma::cx_fmat fft_mat((arma::cx_float*)fft_data, config_->OfdmCaNum(), 1,
                        false);
  arma::fmat fft_abs_mat = arma::abs(fft_mat);
  arma::fmat fft_abs_mag = fft_abs_mat % fft_abs_mat;
  float rssi = arma::as_scalar(sum(fft_abs_mag));
  float noise_per_sc1 = arma::as_scalar(
      arma::mean(fft_abs_mag.rows(0, config_->OfdmDataStart() - 1)));
  float noise_per_sc2 = arma::as_scalar(arma::mean(
      fft_abs_mag.rows(config_->OfdmDataStop(), config_->OfdmCaNum() - 1)));
  float noise = config_->OfdmCaNum() * (noise_per_sc1 + noise_per_sc2) / 2;
  float snr = (rssi - noise) / noise;
  size_t dl_pilots_num = config_->Frame().ClientDlPilotSymbols();
  dl_pilot_snr_[frame_id % kFrameWnd][ant_id * dl_pilots_num + symbol_id] =
      10.0f * std::log10(snr);
}

void PhyStats::PrintZfStats(size_t frame_id) {
  const size_t frame_slot = frame_id % kFrameWnd;
  std::stringstream ss;
  ss << "Frame " << frame_id
     << " ZF matrix inverse condition number range: " << std::fixed
     << std::setw(5) << std::setprecision(2);
  arma::fvec cond_vec(csi_cond_[frame_slot], config_->OfdmDataNum(), false);
  float max_cond = 0;
  float min_cond = 1;
  for (size_t j = 0; j < config_->OfdmDataNum(); j++) {
    if (cond_vec.at(j) < min_cond) {
      min_cond = cond_vec.at(j);
    }
    if (cond_vec.at(j) > max_cond) {
      max_cond = cond_vec.at(j);
    }
  }
  ss << "[" << min_cond << "," << max_cond
     << "], Mean: " << arma::mean(cond_vec);
  ss << std::endl;
  AGORA_LOG_INFO("%s", ss.str().c_str());
}

void PhyStats::UpdateCsiCond(size_t frame_id, size_t sc_id, float cond) {
  csi_cond_[frame_id % kFrameWnd][sc_id] = cond;
}

void PhyStats::UpdateEvmStats(size_t frame_id, size_t sc_id,
                              const arma::cx_fmat& eq) {
  if (num_rx_symbols_ > 0) {
    arma::fmat evm = abs(eq - gt_mat_.col(sc_id));
    arma::fmat cur_evm_mat(evm_buffer_[frame_id % kFrameWnd],
                           config_->UeAntNum(), 1, false);
    cur_evm_mat += evm % evm;
  }
}

void PhyStats::UpdateBitErrors(size_t ue_id, size_t offset, uint8_t tx_byte,
                               uint8_t rx_byte) {
  static constexpr size_t kBitsInByte = 8;
  AGORA_LOG_TRACE("Updating bit errors: User %zu Offset  %zu Tx %d Rx %d\n",
                  ue_id, offset, tx_byte, rx_byte);
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

float PhyStats::GetBitErrorRate(size_t ue_id, size_t offset) {
  return (static_cast<float>(bit_error_count_[ue_id][offset]) /
          static_cast<float>(decoded_bits_count_[ue_id][offset]));
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