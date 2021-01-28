#ifndef UTILS_LDPC
#define UTILS_LDPC

#include "Symbols.hpp"
#include "encoder.hpp"
#include "iobuffer.hpp"
#include "phy_ldpc_encoder_5gnr.h"
#include "utils.h"

#include <cstdlib> /* for std::aligned_alloc */

LDPC_ADAPTER_P ldpc_select_adapter_func(uint16_t zcSize, uint8_t num_ways);

template <typename T>
T* aligned_malloc(const int size, const unsigned alignment)
{
#ifdef _BBLIB_DPDK_
    return reinterpret_cast<T*>(rte_malloc(NULL, sizeof(T) * size, alignment));
#else
#ifndef _WIN64
    return reinterpret_cast<T*>(
        std::aligned_alloc(alignment, sizeof(T) * size));
#else
    return reinterpret_cast<T*>(_aligned_malloc(sizeof(T) * size, alignment));
#endif
#endif
}

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
        // byte-aligned, so we can std::memcpy
        const size_t num_punctured_bytes
            = bits_to_bytes(zc * kNumPuncturedCols);
        const size_t num_input_bytes_to_copy
            = bits_to_bytes(num_input_bits) - num_punctured_bytes;

        std::memcpy(encoded_buffer, input_buffer + num_punctured_bytes,
            num_input_bytes_to_copy);
        std::memcpy(encoded_buffer + num_input_bytes_to_copy, parity_buffer,
            bits_to_bytes(num_parity_bits));
    } else {
        // Otherwise, we need to std::memcpy from/to byte-unaligned locations. A
        // simple but perhaps inefficient way to do this is to use the encoder's
        // internal scatter/gather functions. We don't have access to these
        // functions for FlexRAN's internal AVX-512 encoder.
        if (zc >= avx2enc::kProcBytes * 8) {
            std::fprintf(stderr,
                "Zc values >= %zu that are not multiples of four are not "
                "supported yet.\n",
                zc);
            std::exit(-1);
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
        std::memcpy(internal_buffer2,
            internal_buffer0 + kNumPuncturedCols * avx2enc::kProcBytes,
            (ldpc_num_input_cols(base_graph) - kNumPuncturedCols)
                * avx2enc::kProcBytes);
        std::memcpy(internal_buffer2
                + (ldpc_num_input_cols(base_graph) - kNumPuncturedCols)
                    * avx2enc::kProcBytes,
            internal_buffer1,
            ldpc_max_num_rows(base_graph) * avx2enc::kProcBytes);

        // Gather the concatenated chunks to create the encoded buffer
        adapter_func(encoded_buffer, internal_buffer2, zc,
            ldpc_num_encoded_bits(base_graph, zc, nRows), 0);
    }
}

#endif
