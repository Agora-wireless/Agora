#ifndef PHY_STATS
#define PHY_STATS

#include "Symbols.hpp"
#include "config.hpp"
#include "memory_manage.h"
#include <armadillo>

using namespace arma;
class PhyStats {
public:
    PhyStats(Config* cfg);
    ~PhyStats();
    void print_phy_stats();
    void update_bit_errors(size_t, size_t, uint8_t, uint8_t);
    void update_decoded_bits(size_t, size_t, size_t);
    void update_block_errors(size_t, size_t, size_t);
    void increment_decoded_blocks(size_t, size_t);
    void update_uncoded_bit_errors(size_t, size_t, size_t, uint8_t, uint8_t);
    void update_uncoded_bits(size_t, size_t, size_t);
    void update_evm_stats(size_t, size_t, cx_fmat);
    void print_evm_stats(size_t);
    void update_pilot_snr(size_t, size_t, complex_float*);
    float get_evm_snr(size_t frame_id, size_t ue_id);
    void print_snr_stats(size_t);

private:
    Config const* const config_;
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
