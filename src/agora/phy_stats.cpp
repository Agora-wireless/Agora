#include "phy_stats.hpp"

PhyStats::PhyStats(Config* cfg)
    : config_(cfg)
{
    const size_t task_buffer_symbol_num_ul
        = cfg->ul_data_symbol_num_perframe() * kFrameWnd;
    decoded_bits_count_.calloc(cfg->ue_num(), task_buffer_symbol_num_ul,
        Agora_memory::Alignment_t::k64Align);
    bit_error_count_.calloc(cfg->ue_num(), task_buffer_symbol_num_ul,
        Agora_memory::Alignment_t::k64Align);

    decoded_blocks_count_.calloc(cfg->ue_num(), task_buffer_symbol_num_ul,
        Agora_memory::Alignment_t::k64Align);
    block_error_count_.calloc(cfg->ue_num(), task_buffer_symbol_num_ul,
        Agora_memory::Alignment_t::k64Align);

    uncoded_bits_count_.calloc(cfg->ue_num(), task_buffer_symbol_num_ul,
        Agora_memory::Alignment_t::k64Align);
    uncoded_bit_error_count_.calloc(cfg->ue_num(), task_buffer_symbol_num_ul,
        Agora_memory::Alignment_t::k64Align);

    evm_buffer_.calloc(
        kFrameWnd, cfg->ue_ant_num(), Agora_memory::Alignment_t::k64Align);

    if (cfg->ul_data_symbol_num_perframe() > 0) {
        auto ul_iq_f_ptr
            = reinterpret_cast<cx_float*>(cfg->ul_iq_f[cfg->ul_pilot_syms()]);
        cx_fmat ul_iq_f_mat(
            ul_iq_f_ptr, cfg->ofdm_ca_num(), cfg->ue_ant_num(), false);
        ul_gt_mat_ = ul_iq_f_mat.st(); /* Out of bounds read.... */
        ul_gt_mat_
            = ul_gt_mat_.cols(cfg->ofdm_data_start(), (cfg->ofdm_data_stop() - 1));
    }
    pilot_snr_.calloc(
        kFrameWnd, cfg->ue_ant_num(), Agora_memory::Alignment_t::k64Align);
}

PhyStats::~PhyStats(void)
{
    decoded_bits_count_.free();
    bit_error_count_.free();

    decoded_blocks_count_.free();
    block_error_count_.free();

    uncoded_bits_count_.free();
    uncoded_bit_error_count_.free();

    evm_buffer_.free();
    pilot_snr_.free();
}

void PhyStats::print_phy_stats()
{
    auto& cfg = config_;
    const size_t task_buffer_symbol_num_ul
        = cfg->ul_data_symbol_num_perframe() * kFrameWnd;
    for (size_t ue_id = 0; ue_id < cfg->ue_num(); ue_id++) {
        size_t total_decoded_bits(0);
        size_t total_bit_errors(0);
        size_t total_decoded_blocks(0);
        size_t total_block_errors(0);
        for (size_t i = 0; i < task_buffer_symbol_num_ul; i++) {
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

void PhyStats::print_evm_stats(size_t frame_id)
{
    fmat evm_mat(evm_buffer_[frame_id % kFrameWnd], config_->ue_num(), 1, false);
    evm_mat = sqrt(evm_mat) / config_->ofdm_data_num();
    std::stringstream ss;
    ss << "Frame " << frame_id << " Constellation:\n"
       << "  EVM " << 100 * evm_mat.st() << ", SNR "
       << -10 * log10(evm_mat.st());
    std::cout << ss.str();
}

float PhyStats::get_evm_snr(size_t frame_id, size_t ue_id)
{
    float evm = evm_buffer_[frame_id % kFrameWnd][ue_id];
    evm = sqrt(evm) / config_->ofdm_data_num();
    return -10 * std::log10(evm);
}

void PhyStats::print_snr_stats(size_t frame_id)
{
    std::stringstream ss;
    ss << "Frame " << frame_id << " Pilot Signal SNR: ";
    for (size_t i = 0; i < config_->ue_num(); i++)
        ss << pilot_snr_[frame_id % kFrameWnd][i] << " ";
    ss << std::endl;
    std::cout << ss.str();
}

void PhyStats::update_pilot_snr(
    size_t frame_id, size_t ue_id, complex_float* fft_data)
{
    cx_fmat fft_mat((cx_float*)fft_data, config_->ofdm_ca_num(), 1, false);
    fmat fft_abs_mat = abs(fft_mat);
    fmat fft_abs_mag = fft_abs_mat % fft_abs_mat;
    float rssi = as_scalar(sum(fft_abs_mag));
    float noise_per_sc1
        = as_scalar(mean(fft_abs_mag.rows(0, config_->ofdm_data_start() - 1)));
    float noise_per_sc2 = as_scalar(mean(
        fft_abs_mag.rows(config_->ofdm_data_stop(), config_->ofdm_ca_num() - 1)));
    float noise = config_->ofdm_ca_num() * (noise_per_sc1 + noise_per_sc2) / 2;
    float snr = (rssi - noise) / noise;
    pilot_snr_[frame_id % kFrameWnd][ue_id] = 10 * std::log10(snr);
}

void PhyStats::update_evm_stats(size_t frame_id, size_t sc_id, cx_fmat eq)
{
    if (this->config_->ul_data_symbol_num_perframe() > 0) {
        fmat evm = abs(eq - ul_gt_mat_.col(sc_id));
        fmat cur_evm_mat(
            evm_buffer_[frame_id % kFrameWnd], config_->ue_num(), 1, false);
        cur_evm_mat += evm % evm;
    }
}

void PhyStats::update_bit_errors(
    size_t ue_id, size_t offset, uint8_t tx_byte, uint8_t rx_byte)
{
    uint8_t xor_byte(tx_byte ^ rx_byte);
    size_t bit_errors = 0;
    for (size_t j = 0; j < 8; j++) {
        bit_errors += xor_byte & 1;
        xor_byte >>= 1;
    }
    bit_error_count_[ue_id][offset] += bit_errors;
}

void PhyStats::update_decoded_bits(
    size_t ue_id, size_t offset, size_t new_bits_num)
{
    decoded_bits_count_[ue_id][offset] += new_bits_num;
}

void PhyStats::update_block_errors(
    size_t ue_id, size_t offset, size_t block_error_count)
{
    block_error_count_[ue_id][offset] += (block_error_count > 0);
}

void PhyStats::increment_decoded_blocks(size_t ue_id, size_t offset)
{
    decoded_blocks_count_[ue_id][offset]++;
}

void PhyStats::update_uncoded_bit_errors(size_t ue_id, size_t offset,
    size_t mod_bit_size, uint8_t tx_byte, uint8_t rx_byte)
{
    uint8_t xor_byte(tx_byte ^ rx_byte);
    size_t bit_errors = 0;
    for (size_t j = 0; j < mod_bit_size; j++) {
        bit_errors += xor_byte & 1;
        xor_byte >>= 1;
    }
    uncoded_bit_error_count_[ue_id][offset] += bit_errors;
}

void PhyStats::update_uncoded_bits(
    size_t ue_id, size_t offset, size_t new_bits_num)
{
    uncoded_bits_count_[ue_id][offset] += new_bits_num;
}
