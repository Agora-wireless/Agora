#pragma once

#include "config.hpp"
#include "utils_ldpc.hpp"
#include <string>

class DataGenerator {
public:
    DataGenerator(Config* cfg)
        : cfg(cfg)
    {
        init_modulation_table(mod_table, cfg->mod_type);
    }

    void gen_rand_information_codeblocks_ul(std::vector<int8_t*>& information,
        std::vector<int8_t*>& encoded, size_t num_codeblocks) const
    {
        information.resize(num_codeblocks);
        encoded.resize(num_codeblocks);

        std::vector<int8_t*> parity(num_codeblocks);

        for (size_t i = 0; i < num_codeblocks; i++) {
            information[i] = new int8_t[ldpc_encoding_input_buf_size(
                cfg->LDPC_config.Bg, cfg->LDPC_config.Zc)];
            parity[i] = new int8_t[ldpc_encoding_parity_buf_size(
                cfg->LDPC_config.Bg, cfg->LDPC_config.Zc)];
            encoded[i] = new int8_t[ldpc_encoding_encoded_buf_size(
                cfg->LDPC_config.Bg, cfg->LDPC_config.Zc)];
        }

        const size_t input_bytes_per_cb = bits_to_bytes(
            ldpc_num_input_bits(cfg->LDPC_config.Bg, cfg->LDPC_config.Zc));
        for (size_t n = 0; n < num_codeblocks; n++) {
            for (size_t i = 0; i < input_bytes_per_cb; i++) {
                information[n][i] = (int8_t)rand();
            }
        }

        for (size_t n = 0; n < num_codeblocks; n++) {
            ldpc_encode_helper(cfg->LDPC_config.Bg, cfg->LDPC_config.Zc,
                cfg->LDPC_config.nRows, encoded[n], parity[n], information[n]);
        }
    }

private:
    Config* cfg;
    Table<float> mod_table;
};
