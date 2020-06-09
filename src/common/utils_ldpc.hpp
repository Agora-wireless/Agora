#ifndef UTILS_LDPC
#define UTILS_LDPC

#ifdef USE_LDPC
#include "encoder.hpp"
#include "iobuffer.hpp"
#endif

#include "Symbols.hpp"
#include <assert.h>
#include <malloc.h>

template <typename T>
T* aligned_malloc(const int size, const unsigned alignment)
{
#ifdef _BBLIB_DPDK_
    return (T*)rte_malloc(NULL, sizeof(T) * size, alignment);
#else
#ifndef _WIN64
    return (T*)memalign(alignment, sizeof(T) * size);
#else
    return (T*)_aligned_malloc(sizeof(T) * size, alignment);
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

/*
 * Copy packed, bit-reversed m-bit fields (m == mod_type) stored in
 * vec_in[0..len-1] into unpacked vec_out.  Storage at vec_out must be
 * at least 8*len/m bytes.
 */
static void adapt_bits_for_mod(
    int8_t* vec_in, uint8_t* vec_out, int len, int mod_type)
{
    int bits_avail = 0;
    uint16_t bits = 0;
    for (int i = 0; i < len; i++) {
        bits |= bitreverse8(vec_in[i]) << 8 - bits_avail;
        bits_avail += 8;
        while (bits_avail >= mod_type) {
            *vec_out++ = bits >> (16 - mod_type);
            bits <<= mod_type;
            bits_avail -= mod_type;
        }
    }
}

static void adapt_bits_for_mod(
    int8_t* vec_in, int8_t* vec_out, int len, int mod_type)
{
    int bits_avail = 0;
    uint16_t bits = 0;
    for (int i = 0; i < len; i++) {
        bits |= bitreverse8(vec_in[i]) << 8 - bits_avail;
        bits_avail += 8;
        while (bits_avail >= mod_type) {
            *vec_out++ = bits >> (16 - mod_type);
            bits <<= mod_type;
            bits_avail -= mod_type;
        }
    }
}

static uint8_t select_base_matrix_entry(uint16_t Zc)
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

// Return the number of input information bits per code block with this base
// graph and expansion factor
static size_t ldpc_num_input_bits(size_t base_graph, size_t zc)
{
    return zc
        * (base_graph == 1 ? avx2enc::BG1_COL_INF_NUM
                           : avx2enc::BG2_COL_INF_NUM);
}

// Return the number of parity bits per code block with this base graph and
// expansion factor
static size_t ldpc_num_parity_bits(size_t base_graph, size_t zc)
{
    // Number of rows of the (non-expanded) base graph used
    const size_t num_rows_bg
        = (base_graph == 1 ? avx2enc::BG1_ROW_TOTAL : avx2enc::BG2_ROW_TOTAL);
    return zc * num_rows_bg;
}

// Return the number of total bits per code block with this base graph and
// expansion factor
static size_t ldpc_num_encoded_bits(size_t base_graph, size_t zc)
{
    static size_t num_punctured_cols = 2;
    return zc
        * (base_graph == 1 ? (avx2enc::BG1_COL_TOTAL - num_punctured_cols)
                           : (avx2enc::BG2_COL_TOTAL - num_punctured_cols));
}

// Return the number of bytes required in the input buffer used for LDPC
// encoding
static size_t ldpc_encoding_input_buf_size(size_t base_graph, size_t zc)
{
    // We add avx2enc::PROC_BYTES as padding for the encoder's scatter function
    return bits_to_bytes(ldpc_num_input_bits(base_graph, zc))
        + avx2enc::PROC_BYTES;
}

// Return the number of bytes required in the parity buffer used for LDPC
// encoding
static size_t ldpc_encoding_parity_buf_size(size_t base_graph, size_t zc)
{
    // We add avx2enc::PROC_BYTES as padding for the encoder's gather function
    return bits_to_bytes(ldpc_num_parity_bits(base_graph, zc))
        + avx2enc::PROC_BYTES;
}

// Return the number of bytes required in the output encoded codeword buffer
// used for LDPC encoding
static size_t ldpc_encoding_encoded_buf_size(size_t base_graph, size_t zc)
{
    // We add avx2enc::PROC_BYTES as padding for the encoder's gather function
    return bits_to_bytes(ldpc_num_encoded_bits(base_graph, zc))
        + avx2enc::PROC_BYTES;
}

// Generate the codeword output and parity buffer for this input buffer
static void ldpc_encode_helper(size_t base_graph, size_t zc,
    int8_t* encoded_buffer, int8_t* parity_buffer, const int8_t* input_buffer)
{
    const size_t num_input_bits = ldpc_num_input_bits(base_graph, zc);
    const size_t num_parity_bits = ldpc_num_parity_bits(base_graph, zc);
    const size_t num_encoded_bits = ldpc_num_encoded_bits(base_graph, zc);

    avx2enc::bblib_ldpc_encoder_5gnr_request req;
    avx2enc::bblib_ldpc_encoder_5gnr_response resp;
    req.baseGraph = base_graph;
    req.Zc = zc;
    req.numberCodeblocks = 1;
    req.input[0] = const_cast<int8_t*>(input_buffer);
    resp.output[0] = parity_buffer;

    avx2enc::ldpc_encoder_avx2(&req, &resp);

    // Copy punctured input bits from the encoding request, and parity bits from
    // the encoding response into encoded_buffer

    // This ensures that the input bits after puncturing are byte-aligned.
    // Else we'd have to paste the parity bits at a byte-misaligned start
    // address, which isn't implemented yet.
    assert(req.Zc % 4 == 0);

    static size_t num_punctured_cols = 2;
    const size_t num_punctured_bytes = bits_to_bytes(zc * num_punctured_cols);
    const size_t num_input_bytes_to_copy
        = bits_to_bytes(num_input_bits) - num_punctured_bytes;

    memcpy(encoded_buffer, input_buffer + num_punctured_bytes,
        num_input_bytes_to_copy);
    memcpy(encoded_buffer + num_input_bytes_to_copy, parity_buffer,
        bits_to_bytes(num_parity_bits));
}

#endif