#pragma once

#include "config.hpp"
#include "utils_ldpc.hpp"
#include <string>

class DataGenerator {
public:
    DataGenerator(Config* cfg, uint64_t seed = 0)
        : cfg(cfg)
    {
        init_modulation_table(mod_table, cfg->mod_type);
        if (seed != 0) {
            fast_rand.seed = seed;
        }
    }

    void gen_codeblock_ul(
        std::vector<int8_t>& information, std::vector<int8_t>& encoded_codeword)
    {
        std::vector<int8_t> parity;
        information.resize(ldpc_encoding_input_buf_size(
            cfg->LDPC_config.Bg, cfg->LDPC_config.Zc));
        parity.resize(ldpc_encoding_parity_buf_size(
            cfg->LDPC_config.Bg, cfg->LDPC_config.Zc));
        encoded_codeword.resize(ldpc_encoding_encoded_buf_size(
            cfg->LDPC_config.Bg, cfg->LDPC_config.Zc));

        const size_t input_bytes_per_cb = bits_to_bytes(
            ldpc_num_input_bits(cfg->LDPC_config.Bg, cfg->LDPC_config.Zc));
        for (size_t i = 0; i < input_bytes_per_cb; i++) {
            information[i] = static_cast<int8_t>(fast_rand.next_u32());
        }

        ldpc_encode_helper(cfg->LDPC_config.Bg, cfg->LDPC_config.Zc,
            cfg->LDPC_config.nRows, &encoded_codeword[0], &parity[0],
            &information[0]);
    }

    std::vector<complex_float> get_modulation(
        const std::vector<int8_t>& encoded_codeword)
    {
        std::vector<complex_float> modulated_codeword(cfg->OFDM_DATA_NUM);

        const size_t encoded_bytes_per_cb = bits_to_bytes(ldpc_num_encoded_bits(
            cfg->LDPC_config.Bg, cfg->LDPC_config.Zc, cfg->LDPC_config.nRows));
        std::vector<uint8_t> mod_input(cfg->OFDM_DATA_NUM);
        adapt_bits_for_mod(
            reinterpret_cast<const uint8_t*>(&encoded_codeword[0]),
            &mod_input[0], encoded_bytes_per_cb, cfg->mod_type);

        for (size_t i = 0; i < cfg->OFDM_DATA_NUM; i++) {
            modulated_codeword[i] = mod_single_uint8(mod_input[i], mod_table);
        }

        return modulated_codeword;
    }

    std::vector<complex_float> get_pre_ifft_symbol(
        const std::vector<complex_float> modulated_codeword)
    {
        std::vector<complex_float> pre_ifft_symbol(cfg->OFDM_CA_NUM);
        memset(
            &pre_ifft_symbol[0], 0, cfg->OFDM_CA_NUM * sizeof(complex_float));
        memcpy(&pre_ifft_symbol[cfg->OFDM_DATA_START], &modulated_codeword[0],
            cfg->OFDM_DATA_NUM * sizeof(complex_float));

        return pre_ifft_symbol;
    }

private:
    FastRand fast_rand;
    Config* cfg;
    Table<float> mod_table;
};
