#ifndef UTILS_LDPC
#define UTILS_LDPC

#ifdef USE_LDPC
#include "encoder.hpp"
#include "iobuffer.hpp"
#include "phy_ldpc_decoder_5gnr.h"
#endif

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

#endif