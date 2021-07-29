#pragma once

#include "config.hpp"
#include "utils_ldpc.hpp"
#include <string>

/**
 * @brief Building blocks for generating end-to-end or unit test workloads for
 * Agora
 */
class DataGenerator {
public:
    // The profile of the input information bits
    enum class Profile {
        kRandom, // The input information bytes are chosen at random

        // The input informatioon bytes are {1, 2, 3, 1, 2, 3, ...} for UE 0,
        // {4, 5, 6, 4, 5, 6, ...} for UE 1, and so on
        k123
    };

    DataGenerator(
        Config* cfg, uint64_t seed = 0, Profile profile = Profile::kRandom)
        : cfg(cfg)
        , profile(profile)
    {
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
     * @param ue_id ID of the UE that this codeblock belongs to
     */
    void gen_codeblock_ul(std::vector<int8_t>& information,
        std::vector<int8_t>& encoded_codeword, size_t Bg, size_t Zc)
    {
        const LDPCconfig& lc = cfg->LDPC_config;
        size_t nRows = Bg == 1 ? 46 : 42;
        std::vector<int8_t> parity;
        // printf("parity size=%u, Bg=%u Zc=%u\n", ldpc_encoding_parity_buf_size(Bg, Zc), Bg, Zc);
        parity.resize(ldpc_encoding_parity_buf_size(Bg, Zc));

        // printf("information size=%u, Bg=%u Zc=%u\n", ldpc_encoding_input_buf_size(Bg, Zc), Bg, Zc);
        information.resize(ldpc_encoding_input_buf_size(Bg, Zc));
        // printf("encoded size=%u, Bg=%u Zc=%u\n", ldpc_encoding_encoded_buf_size(Bg, Zc), Bg, Zc);
        encoded_codeword.resize(ldpc_encoding_encoded_buf_size(Bg, Zc));

        size_t num_input_bytes = bits_to_bytes(ldpc_num_input_bits(Bg, Zc));

        for (size_t i = 0; i < num_input_bytes; i++) {
            if (profile == Profile::kRandom) {
                information[i] = static_cast<int8_t>(fast_rand.next_u32());
            } else if (profile == Profile::k123) {
                information[i] = 1 + (i % 3);
            }
        }

        ldpc_encode_helper(Bg, Zc, nRows, &encoded_codeword[0], 
            &parity[0], &information[0]);

        // printf("information size=%u, Bg=%u Zc=%u\n", ldpc_num_input_bits(Bg, Zc), Bg, Zc);
        information.resize(bits_to_bytes(ldpc_num_input_bits(Bg, Zc)));
        // printf("encoded size=%u, Bg=%u Zc=%u nRows=%u\n", ldpc_num_encoded_bits(Bg, Zc, nRows), Bg, Zc, nRows);
        encoded_codeword.resize(bits_to_bytes(ldpc_num_encoded_bits(Bg, Zc, nRows)));
    }

    /**
     * @brief Return the output of modulating the encoded codeword
     * @param encoded_codeword The encoded LDPC codeword bit sequence
     * @return An array of complex floats with OFDM_DATA_NUM elements
     */
    std::vector<complex_float> get_modulation(
        const std::vector<int8_t>& encoded_codeword, size_t mod_order_bits,
        size_t Bg, size_t Zc)
    {
        std::vector<complex_float> modulated_codeword(cfg->OFDM_DATA_NUM);
        std::vector<uint8_t> mod_input(cfg->OFDM_DATA_NUM);

        size_t nRows = Bg == 1 ? 46 : 42;
        size_t num_encoded_bits = bits_to_bytes(ldpc_num_encoded_bits(Bg, Zc, nRows));

        adapt_bits_for_mod(
            reinterpret_cast<const uint8_t*>(&encoded_codeword[0]),
            &mod_input[0], num_encoded_bits, mod_order_bits);

        for (size_t i = 0; i < cfg->OFDM_DATA_NUM; i++) {
            modulated_codeword[i]
                = mod_single_uint8(mod_input[i], cfg->mod_table);
        }
        return modulated_codeword;
    }

    /**
     * @param modulated_codeword The modulated codeword with OFDM_DATA_NUM
     * elements
     * @brief An array with OFDM_CA_NUM elements with the OFDM_DATA_NUM
     * modulated elements binned at the center
     */
    std::vector<complex_float> bin_for_ifft(
        const std::vector<complex_float> modulated_codeword, size_t sc_start,
        size_t sc_end) const
    {
        std::vector<complex_float> pre_ifft_symbol(cfg->OFDM_CA_NUM); // Zeroed
        memcpy(&pre_ifft_symbol[cfg->OFDM_DATA_START + sc_start], &modulated_codeword[0],
            (sc_end - sc_start) * sizeof(complex_float));

        return pre_ifft_symbol;
    }

    /// Return the time-domain pilot symbol with OFDM_CA_NUM complex floats
    std::vector<complex_float> get_common_pilot_time_domain() const
    {
        const std::vector<std::complex<float>> zc_seq
            = Utils::double_to_cfloat(CommsLib::getSequence(
                cfg->OFDM_DATA_NUM, CommsLib::LTE_ZADOFF_CHU));

        const std::vector<std::complex<float>> zc_common_pilot
            = CommsLib::seqCyclicShift(zc_seq, M_PI / 4.0); // Used in LTE SRS

        std::vector<complex_float> ret(cfg->OFDM_CA_NUM); // Zeroed
        for (size_t i = 0; i < cfg->OFDM_DATA_NUM; i++) {
            ret[i + cfg->OFDM_DATA_START]
                = { zc_common_pilot[i].real(), zc_common_pilot[i].imag() };
        }

        return ret;
    }

private:
    FastRand fast_rand; // A fast random number generator
    Config* cfg; // The global Agora config
    const Profile profile; // The pattern of the input byte sequence
};
