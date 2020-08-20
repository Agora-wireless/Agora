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

    void gen_codeblocks_ul(std::vector<std::vector<int8_t>>& information,
        std::vector<std::vector<int8_t>>& encoded, size_t num_codeblocks)
    {
        information.resize(num_codeblocks);
        encoded.resize(num_codeblocks);
        std::vector<std::vector<int8_t>> parity(num_codeblocks);

        for (size_t i = 0; i < num_codeblocks; i++) {
            information[i].resize(ldpc_encoding_input_buf_size(
                cfg->LDPC_config.Bg, cfg->LDPC_config.Zc));
            parity[i].resize(ldpc_encoding_parity_buf_size(
                cfg->LDPC_config.Bg, cfg->LDPC_config.Zc));
            encoded[i].resize(ldpc_encoding_encoded_buf_size(
                cfg->LDPC_config.Bg, cfg->LDPC_config.Zc));
        }

        const size_t input_bytes_per_cb = bits_to_bytes(
            ldpc_num_input_bits(cfg->LDPC_config.Bg, cfg->LDPC_config.Zc));
        for (size_t n = 0; n < num_codeblocks; n++) {
            for (size_t i = 0; i < input_bytes_per_cb; i++) {
                information[n][i] = static_cast<int8_t>(fast_rand.next_u32());
            }
        }

        for (size_t n = 0; n < num_codeblocks; n++) {
            ldpc_encode_helper(cfg->LDPC_config.Bg, cfg->LDPC_config.Zc,
                cfg->LDPC_config.nRows, &encoded[n][0], &parity[n][0],
                &information[n][0]);
        }
    }

    void gen_mod_output(std::vector<int8_t*>& encoded, std::vector<uint8_t*>) {}

private:
    FastRand fast_rand;
    Config* cfg;
    Table<float> mod_table;
};
