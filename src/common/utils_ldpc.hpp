#ifndef UTILS_LDPC
#define UTILS_LDPC

#ifdef USE_LDPC
#include "encoder.hpp"
#include "iobuffer.hpp"
#include "phy_ldpc_decoder_5gnr.h"
#endif

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

// Copy punctured input bits from the encoding request, and parity bits from
// the encoding response into encoded_buffer
static void generate_encoded_buffer(int8_t* encoded_buffer,
    avx2enc::bblib_ldpc_encoder_5gnr_request* req,
    avx2enc::bblib_ldpc_encoder_5gnr_response* resp, size_t cb_index = 0)
{
    // This ensures that the input bits after puncturing are byte-aligned.
    // Else we'd have to paste the parity bits at a byte-misaligned start
    // address, which isn't implemented yet.
    assert(req->Zc % 4 == 0);

    const size_t num_input_bits = req->Zc
        * (req->baseGraph == 1 ? avx2enc::BG1_COL_INF_NUM
                               : avx2enc::BG2_COL_INF_NUM);

    // Number of rows of the (non-expanded) base graph used
    const size_t num_rows_bg = (req->baseGraph == 1 ? avx2enc::BG1_ROW_TOTAL
                                                    : avx2enc::BG2_ROW_TOTAL);
    const size_t num_parity_bits = req->Zc * num_rows_bg;
    const size_t num_punctured_bytes
        = bits_to_bytes(req->Zc * 2 /* number of punctured columns */);
    const size_t num_input_bytes_copied
        = bits_to_bytes(num_input_bits) - num_punctured_bytes;

    memcpy(encoded_buffer, req->input[cb_index] + num_punctured_bytes,
        num_input_bytes_copied);
    memcpy(encoded_buffer + num_input_bytes_copied, resp->output[cb_index],
        bits_to_bytes(num_parity_bits));
}

#endif