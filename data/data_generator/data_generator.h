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

    /**
     * @brief Generate one information bit sequence and the corresponding
     * encoded bit sequence for one code block for the active LDPC configuration
     *
     * @param information The generated input bit sequence
     * @param encoded_codeword The generated encoded codeword bit sequence
     */
    void gen_codeblock_ul(
        std::vector<int8_t>& information, std::vector<int8_t>& encoded_codeword)
    {
        const LDPCconfig& lc = cfg->LDPC_config;
        std::vector<int8_t> parity;
        parity.resize(ldpc_encoding_parity_buf_size(lc.Bg, lc.Zc));

        information.resize(ldpc_encoding_input_buf_size(lc.Bg, lc.Zc));
        encoded_codeword.resize(ldpc_encoding_encoded_buf_size(lc.Bg, lc.Zc));

        for (size_t i = 0; i < lc.num_input_bytes(); i++) {
            information[i] = static_cast<int8_t>(fast_rand.next_u32());
        }

        ldpc_encode_helper(cfg->LDPC_config.Bg, cfg->LDPC_config.Zc,
            cfg->LDPC_config.nRows, &encoded_codeword[0], &parity[0],
            &information[0]);

        information.resize(lc.num_input_bytes());
        encoded_codeword.resize(lc.num_encoded_bytes());
    }

    std::vector<complex_float> get_modulation(
        const std::vector<int8_t>& encoded_codeword)
    {
        std::vector<complex_float> modulated_codeword(cfg->OFDM_DATA_NUM);
        std::vector<uint8_t> mod_input(cfg->OFDM_DATA_NUM);

        adapt_bits_for_mod(
            reinterpret_cast<const uint8_t*>(&encoded_codeword[0]),
            &mod_input[0], cfg->LDPC_config.num_encoded_bytes(), cfg->mod_type);

        for (size_t i = 0; i < cfg->OFDM_DATA_NUM; i++) {
            modulated_codeword[i] = mod_single_uint8(mod_input[i], mod_table);
        }
        return modulated_codeword;
    }

    std::vector<complex_float> get_pre_ifft_symbol(
        const std::vector<complex_float> modulated_codeword) const
    {
        std::vector<complex_float> pre_ifft_symbol(cfg->OFDM_CA_NUM);
        memset(
            &pre_ifft_symbol[0], 0, cfg->OFDM_CA_NUM * sizeof(complex_float));
        memcpy(&pre_ifft_symbol[cfg->OFDM_DATA_START], &modulated_codeword[0],
            cfg->OFDM_DATA_NUM * sizeof(complex_float));

        return pre_ifft_symbol;
    }

    std::vector<complex_float> get_common_pilot_time_domain() const
    {
        const std::vector<std::complex<float>> zc_seq
            = Utils::double_to_cfloat(CommsLib::getSequence(
                cfg->OFDM_DATA_NUM, CommsLib::LTE_ZADOFF_CHU));

        const std::vector<std::complex<float>> zc_common_pilot
            = CommsLib::seqCyclicShift(zc_seq, M_PI / 4.0); // Used in LTE SRS

        std::vector<complex_float> ret(cfg->OFDM_CA_NUM);
        memset(&ret[0], 0, cfg->OFDM_CA_NUM * sizeof(complex_float));

        for (size_t i = 0; i < cfg->OFDM_DATA_NUM; i++) {
            ret[i + cfg->OFDM_DATA_START]
                = { zc_common_pilot[i].real(), zc_common_pilot[i].imag() };
        }

        return ret;
    }

private:
    FastRand fast_rand;
    Config* cfg;
    Table<float> mod_table;
};
