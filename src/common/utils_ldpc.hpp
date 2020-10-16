#ifndef UTILS_LDPC
#define UTILS_LDPC

#include "Symbols.hpp"
#include "encoder.hpp"
#include "iobuffer.hpp"
#include "phy_ldpc_encoder_5gnr.h"
#include "utils.h"
#include <assert.h>
#include <malloc.h>

#ifndef __has_builtin
#define __has_builtin(x) 0
#endif

static inline uint8_t bitreverse8(uint8_t x)
{
#if __has_builtin(__builtin_bireverse8)
    return (__builtin_bitreverse8(x));
#else
    x = (x << 4) | (x >> 4);
    x = ((x & 0x33) << 2) | ((x >> 2) & 0x33);
    x = ((x & 0x55) << 1) | ((x >> 1) & 0x55);
    return (x);
#endif
}

/**
 * \brief Fill-in the bytes of \p bytes_out with \p mod_type bits per byte,
 * taken from the bit sequence \p bit_seq_in
 *
 * \param bit_seq_in The input bit sequence
 *
 * \param bytes_out The output byte array with \p mod_type bits per byte. It
 * must have space for ceil(len * 8.0 / mod_type) bytes.
 *
 * \param len The number of bytes in \p bit_seq_in
 *
 * \param mod_type The number of bits in one modulated symbol (e.g., mod_type =
 * 6 for 64-QAM modulation)
 */
static inline void adapt_bits_for_mod(
    const uint8_t* bit_seq_in, uint8_t* bytes_out, size_t len, size_t mod_type)
{
    uint16_t bits = 0; // Bits collected from the input
    size_t bits_avail = 0; // Number of valid bits filled into [bits]
    for (size_t i = 0; i < len; i++) {
        bits |= static_cast<uint32_t>(bitreverse8(bit_seq_in[i]))
            << (8 - bits_avail);
        bits_avail += 8;
        while (bits_avail >= mod_type) {
            *bytes_out++ = bits >> (16 - mod_type);
            bits <<= mod_type;
            bits_avail -= mod_type;
        }
    }

    if (bits_avail > 0) {
        *bytes_out++ = bits >> (16 - mod_type);
    }
}

/*
 * Copy packed, bit-reversed 8-bit fields stored in
 * vec_in[0..len-1] into unpacked m-bit vec_out (m == mod_type). 
 * Storage at vec_out must be at least (m*len+7)/8 bytes.
 */
static inline void adapt_bits_from_mod(
    const uint8_t* vec_in, uint8_t* vec_out, int len, int mod_type)
{
    int bits_avail = 0;
    uint16_t bits = 0;
    for (int i = 0; i < len; i++) {
        bits |= (bitreverse8(vec_in[i]) >> (8 - mod_type)) << bits_avail;
        bits_avail += mod_type;
        while (bits_avail >= 8) {
            *vec_out++ = bits & 0xff;
            bits >>= 8;
            bits_avail -= 8;
        }
    }
}

static inline uint8_t select_base_matrix_entry(uint16_t Zc)
{
    uint8_t i_LS;
    if ((Zc % 15) == 0)
        i_LS = 7;
    else if ((Zc % 13) == 0)
        i_LS = 6;
    else if ((Zc % 11) == 0)
        i_LS = 5;
    else if ((Zc % 9) == 0)
        i_LS = 4;
    else if ((Zc % 7) == 0)
        i_LS = 3;
    else if ((Zc % 5) == 0)
        i_LS = 2;
    else if ((Zc % 3) == 0)
        i_LS = 1;
    else
        i_LS = 0;
    return i_LS;
}

// Return the number of bytes needed to store n_bits bits
static inline size_t bits_to_bytes(size_t n_bits) { return (n_bits + 7) / 8; }

// Return the number of non-expanded base graph columns used for information
// bits for this base graph
static inline size_t ldpc_num_input_cols(size_t base_graph)
{
    return (base_graph == 1 ? BG1_COL_INF_NUM : BG2_COL_INF_NUM);
}

// Return the maximum number of rows in this non-expanded base graph
static inline size_t ldpc_max_num_rows(size_t base_graph)
{
    return (base_graph == 1 ? BG1_ROW_TOTAL : BG2_ROW_TOTAL);
}

// Return the number of input information bits per codeword with this base
// graph and expansion factor
static inline size_t ldpc_num_input_bits(size_t base_graph, size_t zc)
{
    return zc * ldpc_num_input_cols(base_graph);
}

// Return the number of parity bits per codeword with this base graph and
// expansion factor
static inline size_t ldpc_max_num_parity_bits(size_t base_graph, size_t zc)
{
    return zc * ldpc_max_num_rows(base_graph);
}

// Return the maximum number of total bits per code block with this base graph
// and expansion factor
static inline size_t ldpc_max_num_encoded_bits(size_t base_graph, size_t zc)
{
    static size_t num_punctured_cols = 2;
    return zc
        * (base_graph == 1 ? (BG1_COL_TOTAL - num_punctured_cols)
                           : (BG2_COL_TOTAL - num_punctured_cols));
}

// Return the number of total bits per codeword (i.e., including both input
// bits and parity bits) with this base graph and expansion factor
static inline size_t ldpc_num_encoded_bits(
    size_t base_graph, size_t zc, size_t nRows)
{
    static size_t num_punctured_cols = 2;
    return zc * (ldpc_num_input_cols(base_graph) + nRows - num_punctured_cols);
}

// Return the number of bytes required in the input buffer used for LDPC
// encoding
static inline size_t ldpc_encoding_input_buf_size(size_t base_graph, size_t zc)
{
    // We add kMaxProcBytes as padding for the encoder's scatter function
    return bits_to_bytes(ldpc_num_input_bits(base_graph, zc)) + kMaxProcBytes;
}

// Return the number of bytes required in the parity buffer used for LDPC
// encoding
static inline size_t ldpc_encoding_parity_buf_size(size_t base_graph, size_t zc)
{
    // We add kMaxProcBytes as padding for the encoder's gather function
    return bits_to_bytes(ldpc_max_num_parity_bits(base_graph, zc))
        + kMaxProcBytes;
}

// Return the number of bytes required in the output encoded codeword buffer
// used for LDPC encoding
static inline size_t ldpc_encoding_encoded_buf_size(
    size_t base_graph, size_t zc)
{
    // We add kMaxProcBytes as padding for the encoder's gather function
    return bits_to_bytes(ldpc_max_num_encoded_bits(base_graph, zc))
        + kMaxProcBytes;
}

// Return the minimum LDPC expansion factor supported
static inline size_t ldpc_get_min_zc() { return kUseAVX2Encoder ? 2 : 6; }

// Return the maximum LDPC expansion factor supported
static inline size_t ldpc_get_max_zc()
{
    return kUseAVX2Encoder ? avx2enc::kZcMax : ZC_MAX;
}

// Generate the codeword output and parity buffer for this input buffer
static inline void ldpc_encode_helper(size_t base_graph, size_t zc,
    size_t nRows, int8_t* encoded_buffer, int8_t* parity_buffer,
    const int8_t* input_buffer)
{
    const size_t num_input_bits = ldpc_num_input_bits(base_graph, zc);
    const size_t num_parity_bits = nRows * zc;

    bblib_ldpc_encoder_5gnr_request req;
    bblib_ldpc_encoder_5gnr_response resp;
    req.baseGraph = base_graph;
    req.nRows = kUseAVX2Encoder ? ldpc_max_num_rows(base_graph) : nRows;
    req.Zc = zc;
    req.nRows = nRows;
    req.numberCodeblocks = 1;
    req.input[0] = const_cast<int8_t*>(input_buffer);
    resp.output[0] = parity_buffer;

    kUseAVX2Encoder ? avx2enc::bblib_ldpc_encoder_5gnr(&req, &resp)
                    : bblib_ldpc_encoder_5gnr(&req, &resp);

    // Copy punctured input bits from the encoding request, and parity bits from
    // the encoding response into encoded_buffer
    static size_t kNumPuncturedCols = 2;
    if (zc % 4 == 0) {
        // In this case, the start and end of punctured input bits is
        // byte-aligned, so we can memcpy
        const size_t num_punctured_bytes
            = bits_to_bytes(zc * kNumPuncturedCols);
        const size_t num_input_bytes_to_copy
            = bits_to_bytes(num_input_bits) - num_punctured_bytes;

        memcpy(encoded_buffer, input_buffer + num_punctured_bytes,
            num_input_bytes_to_copy);
        memcpy(encoded_buffer + num_input_bytes_to_copy, parity_buffer,
            bits_to_bytes(num_parity_bits));
    } else {
        // Otherwise, we need to memcpy from/to byte-unaligned locations. A
        // simple but perhaps inefficient way to do this is to use the encoder's
        // internal scatter/gather functions. We don't have access to these
        // functions for FlexRAN's internal AVX-512 encoder.
        if (zc >= avx2enc::kProcBytes * 8) {
            fprintf(stderr,
                "Zc values >= %zu that are not multiples of four are not "
                "supported yet.\n",
                zc);
            exit(-1);
        }

        __attribute__((aligned(avx2enc::kProcBytes)))
        int8_t internal_buffer0[BG1_COL_INF_NUM * avx2enc::kProcBytes]
            = { 0 };
        __attribute__((aligned(avx2enc::kProcBytes)))
        int8_t internal_buffer1[BG1_ROW_TOTAL * avx2enc::kProcBytes]
            = { 0 };
        __attribute__((aligned(avx2enc::kProcBytes)))
        int8_t internal_buffer2[BG1_COL_TOTAL * avx2enc::kProcBytes]
            = { 0 };

        auto adapter_func = avx2enc::ldpc_select_adapter_func(zc);

        // Scatter input and parity into zc-bit chunks
        adapter_func(
            (int8_t*)input_buffer, internal_buffer0, zc, num_input_bits, 1);
        adapter_func(parity_buffer, internal_buffer1, zc, num_parity_bits, 1);

        // Concactenate the chunks for input and parity
        memcpy(internal_buffer2,
            internal_buffer0 + kNumPuncturedCols * avx2enc::kProcBytes,
            (ldpc_num_input_cols(base_graph) - kNumPuncturedCols)
                * avx2enc::kProcBytes);
        memcpy(internal_buffer2
                + (ldpc_num_input_cols(base_graph) - kNumPuncturedCols)
                    * avx2enc::kProcBytes,
            internal_buffer1,
            ldpc_max_num_rows(base_graph) * avx2enc::kProcBytes);

        // Gather the concatenated chunks to create the encoded buffer
        adapter_func(encoded_buffer, internal_buffer2, zc,
            ldpc_num_encoded_bits(base_graph, zc, nRows), 0);
    }
}

// Lookup table for determining modulation order based on modulation and coding
// scheme (MCS)
// 3GPP TS38.214-Table 5.1.3.1-2
static constexpr size_t McsToModOrderBits[28] = { 2, 2, 2, 2, 2, 4, 4, 4, 4, 4,
    4, 6, 6, 6, 6, 6, 6, 6, 6, 6, 8, 8, 8, 8, 8, 8, 8, 8 };

// Lookup table for determining code rate based on MCS
// 3GPP TS38.214-Table 5.1.3.1-2
static constexpr float McsToCodeRate[28] = { 0.1172, 0.1885, 0.3008, 0.4385,
    0.5879, 0.3691, 0.4238, 0.4785, 0.5400, 0.6016, 0.6426, 0.4551, 0.5049,
    0.5537, 0.6016, 0.6504, 0.7021, 0.7539, 0.8027, 0.8525, 0.6665, 0.6943,
    0.7363, 0.7783, 0.8213, 0.8643, 0.8950, 0.9258 };

// Lookup table for transport block size from 3GPP TS38.214-Table 5.1.3.2-2
static std::vector<size_t> LutNInfo = { 24, 32, 40, 48, 56, 64, 72, 80, 88, 96,
    104, 112, 120, 128, 136, 144, 152, 160, 168, 176, 184, 192, 208, 224, 240,
    256, 272, 288, 304, 320, 336, 352, 368, 384, 408, 432, 456, 480, 504, 528,
    552, 576, 608, 640, 672, 704, 736, 768, 808, 848, 928, 984, 1032, 1064,
    1128, 1160, 1192, 1224, 1256, 1288, 1320, 1352, 1416, 1480, 1544, 1608,
    1672, 1736, 1800, 1864, 1928, 2024, 2088, 2152, 2216, 2280, 2408, 2472,
    2536, 2600, 2664, 2728, 2792, 2856, 2976, 3104, 3240, 3368, 3496, 3624,
    3752, 3824 };

// Set of LDPC lifting size Zc from 3GPP TS38.212-Table 5.3.2-1
static std::vector<size_t> LutZc = { 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
    15, 16, 18, 20, 22, 24, 26, 28, 30, 32, 36, 40, 44, 48, 52, 56, 60, 64, 72,
    80, 88, 96, 104, 112, 120, 128, 144, 160, 176, 192, 208, 224, 240, 256, 288,
    320, 352, 384 };

// Select base graph based on transport block size and code rate.
// 3GPP TS38.212-7.2.2
static inline uint16_t select_base_graph(size_t tb_size, float code_rate)
{
    if (tb_size < 292 or (tb_size <= 3824 and code_rate <= 0.67)
        or code_rate <= 0.25)
        return 2;
    return 1;
}

// Return the closest value in a vector that is not less than the input value.
static inline size_t closest(std::vector<size_t> const& vec, size_t value)
{
    auto const it = std::lower_bound(vec.begin(), vec.end(), value);
    if (it == vec.end())
        return -1;
    return *it;
}

// Return lifting size Zc based on 3GPP TS38.212-5.2.2
static inline size_t select_zc(
    size_t tb_size, size_t n_info_per_cb, uint16_t base_graph)
{
    size_t kb;
    if (base_graph == 1) {
        kb = 22;
    } else {
        if (tb_size > 640)
            kb = 10;
        else if (tb_size > 560)
            kb = 9;
        else if (tb_size > 192)
            kb = 8;
        else
            kb = 6;
    }

    size_t zc = closest(LutZc, std::ceil(1.f * n_info_per_cb / kb));
    return zc;
}

// Code block segmentation based on 3GPP TS38.212-5.2.2
static inline void code_block_segmentation(size_t tb_size, uint16_t base_graph,
    size_t& n_cb, uint32_t& n_info_per_cb, uint16_t& zc)
{
    size_t max_k_cb = base_graph == 1 ? 8448 : 3840;
    size_t tb_size_prime = tb_size;
    // Determine number of code blocks
    if (tb_size < max_k_cb) {
        n_cb = 1;
        n_info_per_cb = tb_size;
    } else {
        size_t n_crc_bits = 24;
        n_cb = std::ceil(1.f * tb_size / (max_k_cb - n_crc_bits));
        tb_size_prime = tb_size + n_cb * n_crc_bits;
    }

    // Determine number of info bits in each code block
    n_info_per_cb = tb_size_prime / n_cb;

    // Determine LDPC lifting size
    zc = select_zc(tb_size_prime, n_info_per_cb, base_graph);

    // Update number of info bits in each code block
    n_info_per_cb = ldpc_num_input_bits(base_graph, zc);
}

static inline size_t compute_n_info(
    size_t n_symbol, size_t n_sc, size_t mod_order_bits, float code_rate)
{
    return std::floor(n_symbol * n_sc * mod_order_bits * code_rate);
}

// Return transport block size based on number of info bits avaible in a slot
// and LDPC code rate
// 3GPP TS38.214-5.1.3.2
static inline size_t compute_tb_size(size_t n_info, float code_rate)
{
    size_t transport_block_size;
    if (n_info <= 3824) {
        size_t n = std::max(3, (int)std::floor(std::log2(n_info)) - 6);
        size_t n_info_prime = std::max(
            24, (int)(std::pow(2, n) * std::floor(n_info / std::pow(2, n))));
        transport_block_size = closest(LutNInfo, n_info_prime);
    } else {
        size_t n = std::floor(std::log2(n_info - 24)) - 5;
        size_t n_info_prime
            = std::pow(2, n) * std::round(1.f * (n_info - 24) / std::pow(2, n));
        size_t c = 1;
        if (code_rate < 0.25)
            c = std::ceil(1.f * (n_info_prime + 24) / 3816);
        else if (n_info_prime >= 8424)
            c = std::ceil(1.f * (n_info_prime + 24) / 8424);
        transport_block_size
            = 8 * c * std::ceil(1.f * (n_info_prime + 24) / (8 * c)) - 24;
    }
    return transport_block_size;
}

static inline size_t compute_n_rows(float target_code_rate, uint16_t base_graph)
{
    size_t n_info_cols = ldpc_num_input_cols(base_graph);
    size_t n_rows = std::max((size_t)4,
        std::min(ldpc_max_num_rows(base_graph),
            (int)std::round(1.f * n_info_cols / target_code_rate)
                - n_info_cols));
    return n_rows;
}

static inline float compute_code_rate(size_t n_rows, uint16_t base_graph)
{
    static size_t num_punctured_cols = 2;
    return (1.f * ldpc_num_input_cols(base_graph)
        / (ldpc_num_input_cols(base_graph) + n_rows - num_punctured_cols));
}

class LDPCconfig {
public:
    uint16_t Bg; /// The 5G NR LDPC base graph (one or two)
    uint16_t Zc; /// The 5G NR LDPC expansion factor
    int16_t decoderIter; /// Maximum number of decoder iterations per codeblock

    /// Allow the LDPC decoder to terminate without completing all iterations
    /// if it decodes the codeblock eariler
    bool earlyTermination;

    size_t nRows; /// Number of rows in the LDPC base graph to use
    uint32_t cbLen; /// Number of information bits input to LDPC encoding
    uint32_t cbCodewLen; /// Number of codeword bits output from LDPC encoding
    size_t nCb; /// Number of code blocks in a frame
    float code_rate;
    /// Lookup table that maps code blocks to symbols
    std::vector<std::vector<size_t>> lut_cb_to_symbol;
    /// Lookup table that maps symbols to code blocks
    std::vector<std::vector<size_t>> lut_symbol_to_cb;
    /// Lookup table for number of bytes within chunks of code blocks
    std::vector<std::vector<size_t>> lut_cb_chunks_bytes;
    /// Lookup table for number of subcarriers occupied by chunks of code blocks
    std::vector<std::vector<size_t>> lut_cb_chunks_scs;

    // Return the number of bytes in the information bit sequence for LDPC
    // encoding of one code block
    size_t num_input_bytes() const
    {
        return bits_to_bytes(ldpc_num_input_bits(Bg, Zc));
    }

    // Return the number of bytes in the encoded LDPC code word
    size_t num_encoded_bytes() const
    {
        return bits_to_bytes(ldpc_num_encoded_bits(Bg, Zc, nRows));
    }

    // Return the number of bytes in the encoded LDPC code word
    // rounded up to 64 btyes
    size_t num_encoded_bytes_pad() const
    {
        return roundup<64>(num_encoded_bytes());
    }

    // Generate lookup tables for the mappling between symbols and code blocks
    void map_symbols_to_cbs(
        size_t symbol_num_perframe, size_t ofdm_data_num, size_t mod_order_bits)
    {
        // rt_assert(LDPC_config.nCb <= symbol_num_perframe,
        //     "LDPC lifting factor (Zc) is too small (requires number of "
        //     "symbols per frame > number of code blocks per frame).");
        lut_symbol_to_cb.resize(symbol_num_perframe);
        lut_cb_to_symbol.resize(nCb);
        lut_cb_chunks_bytes.resize(symbol_num_perframe);
        lut_cb_chunks_scs.resize(symbol_num_perframe);
        size_t num_encoded_bytes_per_symbol
            = bits_to_bytes(ofdm_data_num * mod_order_bits);
        if (symbol_num_perframe % nCb == 0) {
            size_t symbol_per_cb = symbol_num_perframe / nCb;
            for (size_t i = 0; i < nCb; i++) {
                for (size_t j = 0; j < symbol_per_cb; j++) {
                    lut_cb_to_symbol[i].push_back(i * symbol_per_cb + j);
                    // Use all subcarriers in symbols except the last symbol
                    if (j == symbol_per_cb - 1) {
                        lut_cb_chunks_bytes[i].push_back(num_encoded_bytes()
                            - j * num_encoded_bytes_per_symbol);
                        lut_cb_chunks_scs[i].push_back(
                            cbCodewLen / mod_order_bits - j * ofdm_data_num);
                    } else {
                        lut_cb_chunks_bytes[i].push_back(
                            num_encoded_bytes_per_symbol);
                        lut_cb_chunks_scs[i].push_back(ofdm_data_num);
                    }
                }
            }
            for (size_t i = 0; i < symbol_num_perframe; i++) {
                lut_symbol_to_cb[i].push_back(
                    symbol_num_perframe / symbol_per_cb);
            }
        } else {
            // Number of subcarriers required for a code block
            // Pad to 64 bytes to avoid cache false sharing
            size_t num_encoded_scs_pad
                = roundup<64>(cbCodewLen / mod_order_bits);
            size_t unmapped_scs_in_symbol = ofdm_data_num;
            size_t symbol_id = 0;
            for (size_t i = 0; i < nCb; i++) {
                size_t unmapped_scs_in_cb = num_encoded_scs_pad;
                while (unmapped_scs_in_cb > 0) {
                    lut_cb_to_symbol[i].push_back(symbol_id);
                    lut_symbol_to_cb[symbol_id].push_back(i);
                    if (unmapped_scs_in_cb >= unmapped_scs_in_symbol) {
                        lut_cb_chunks_bytes[i].push_back(bits_to_bytes(
                            unmapped_scs_in_symbol * mod_order_bits));
                        lut_cb_chunks_scs[i].push_back(unmapped_scs_in_symbol);
                        unmapped_scs_in_cb -= unmapped_scs_in_symbol;
                        symbol_id++;
                        unmapped_scs_in_symbol = ofdm_data_num;
                    } else {
                        lut_cb_chunks_bytes[i].push_back(
                            bits_to_bytes(unmapped_scs_in_cb * mod_order_bits));
                        lut_cb_chunks_scs[i].push_back(unmapped_scs_in_cb);
                        unmapped_scs_in_cb = 0;
                        unmapped_scs_in_symbol -= unmapped_scs_in_cb;
                    }
                }
            }
        }
    }

    // Return the offset of subcarrier for the start of a code block's
    // current chunk within a symbol
    size_t get_chunk_start_sc(size_t cb_id, size_t chunk_id) const
    {
        size_t symbol_id = lut_cb_to_symbol[cb_id][chunk_id];
        const auto it = std::find((lut_symbol_to_cb[symbol_id]).begin(),
            (lut_symbol_to_cb[symbol_id]).end(), cb_id);
        rt_assert(it != lut_symbol_to_cb[symbol_id].end(),
            "Code block does not exist in the symbol");
        size_t cb_id_in_symbol = it - lut_symbol_to_cb[symbol_id].begin();
        size_t chunk_start_sc = 0;
        for (size_t i = 0; i < cb_id_in_symbol; i++)
            chunk_start_sc
                += lut_cb_chunks_scs[lut_symbol_to_cb[symbol_id][i]].back();
        return chunk_start_sc;
    }
};

#endif