/**********************************************************************
*
* INTEL CONFIDENTIAL
* Copyright 2009-2019 Intel Corporation All Rights Reserved.
* 
* The source code contained or described herein and all documents related to the
* source code ("Material") are owned by Intel Corporation or its suppliers or
* licensors. Title to the Material remains with Intel Corporation or its
* suppliers and licensors. The Material may contain trade secrets and proprietary
* and confidential information of Intel Corporation and its suppliers and
* licensors, and is protected by worldwide copyright and trade secret laws and
* treaty provisions. No part of the Material may be used, copied, reproduced,
* modified, published, uploaded, posted, transmitted, distributed, or disclosed
* in any way without Intel's prior express written permission.
* 
* No license under any patent, copyright, trade secret or other intellectual
* property right is granted to or conferred upon you by disclosure or delivery
* of the Materials, either expressly, by implication, inducement, estoppel or
* otherwise. Any license under such intellectual property rights must be
* express and approved by Intel in writing.
* 
* Unless otherwise agreed by Intel in writing, you may not remove or alter this
* notice or any other notice embedded in Materials by Intel or Intel's suppliers
* or licensors in any way.
* 
*  version: SDK-jenkins-FlexRAN-SDK-REL-448-g3be238
*
**********************************************************************/

/**
 * @file   phy_crc_avx512.cpp
 * @brief  Implementation of CRC algorithms CRC24A, CRC24B, CRC24C, CRC16, CRC11 & CRC6
 * all initialised with zeros and CRC24C initialised with ones (appending 1's to data),
 * in accordance with 3GPP TS 38.212 v15.1.1 spec.
 * This implementation is based on the efficient data folding & barrett reduction technique,
 * using optimised intrinsics.
 */

/**
 * Include public/global header files
 */
#include <immintrin.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <functional>

#include "phy_crc.h"
#include "phy_crc_internal.h"


// Internal functions
static __m128i fold_4stage(__m128i* data_in, uint32_t len);
static __m128i barrett_reduction(__m128i fold_data);
static void crc_generate(bblib_crc_request *request, bblib_crc_response *response);
static void crc_check(bblib_crc_request *request, bblib_crc_response *response);
static __m128i bit_shift_right_m128i(__m128i data, __m128i count);
static __m128i bit_shift_left_m128i(__m128i data, int count);
static __m128i shift_right_256(__m128i data, __m128i next_data, __m128i count);



static const __m128i k_endian_shuf_mask128 = _mm_set_epi8(0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                                          0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F);
static const __m128i k_endian_shuf_mask32 = _mm_set_epi8(0x00, 0x01, 0x02, 0x03, 0x00, 0x01, 0x02, 0x03,
                                                         0x00, 0x01, 0x02, 0x03, 0x00, 0x01, 0x02, 0x03);
static const __m128i k_shuf_mask = _mm_set_epi8(0x03, 0x02, 0x01, 0x00, 0x03, 0x02, 0x01, 0x00,
                                                0x03, 0x02, 0x01, 0x00, 0x03, 0x02, 0x01, 0x00);
static const __m128i k_shift_mask = _mm_set_epi8(0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                                                 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff);
static const __m128i k_64 = _mm_set_epi8(0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40);


#if defined (_BBLIB_AVX512_)
struct init_crc_avx512
{
    init_crc_avx512()
    {
        bblib_print_crc_version();
    }
};

init_crc_avx512 do_constructor_crc_avx512;

using check_func = std::function<void(bblib_crc_request *request, bblib_crc_response *response)>;

// CRC Structures
// One for each CRC type supported (CRC24A, 24B, 24C & CRC16, CRC11 & CRC6 & 24C init with 1's)
struct crc24a_precompute
{
    static constexpr uint64_t k_CRCPOLY = 0x1864CFB;      //CRC24A polynomial
    //Shift CRCPOLY by 32 minus 24bits since using a 24bit polynomial
    static constexpr uint64_t k_crc_shifted_poly = k_CRCPOLY << 8;
    static constexpr uint32_t k_t192 = 0x2c8c9d00;        // t=128+64, x^192 mod crc_shifted_poly
    static constexpr uint32_t k_t128 = 0x64e4d700;        // t=128, x^128 mod crc_shifted_poly
    static constexpr uint32_t k_t96 = 0xfd7e0c00;         // t=96, x^96 mod crc_shifted_poly
    static constexpr uint32_t k_t64 = 0xd9fe8c00;         // t=64, x^64 mod crc_shifted_poly
    static constexpr uint64_t k_u = 0x1f845fe24;          // u for crc24C * 256, floor(x^64 / crc_shifted_poly)
    static constexpr uint16_t k_crc_bits = 24;            // crc_size (in bits)
    static constexpr uint16_t k_crc_bytes = (k_crc_bits-1)/8+1;  // crc size (bytes)
    static constexpr uint32_t k_init_value = 0x00000000;  // shift reg initialisation value
    static const check_func crc_generate;                 // crc generate function type
};
const check_func crc24a_precompute::crc_generate = bblib_lte_crc24a_gen_avx512;

struct crc24b_precompute
{
    static constexpr uint64_t k_CRCPOLY = 0x1800063;      //CRC24B polynomial
    //Shift CRCPOLY by 32 minus 24bits since using a 24bit polynomial
    static constexpr uint64_t k_crc_shifted_poly = k_CRCPOLY << 8;
    static constexpr uint32_t k_t192 = 0x42000100;        // t=128+64, x^192 mod crc_shifted_poly
    static constexpr uint32_t k_t128 = 0x80140500;        // t=128, x^128 mod crc_shifted_poly
    static constexpr uint32_t k_t96 = 0x09000200;         // t=96, x^96 mod crc_shifted_poly
    static constexpr uint32_t k_t64 = 0x90042100;         // t=64, x^64 mod crc_shifted_poly
    static constexpr uint64_t k_u = 0x1ffff83ff;          // u for crc24C * 256, floor(x^64 / crc_shifted_poly)
    static constexpr uint16_t k_crc_bits = 24;            // crc_size (in bits)
    static constexpr uint16_t k_crc_bytes = (k_crc_bits-1)/8+1;  // crc size (bytes)
    static constexpr uint32_t k_init_value = 0x00000000;  // shift reg initialisation value
    static const check_func crc_generate;                 // crc generate function type
};
const check_func crc24b_precompute::crc_generate = bblib_lte_crc24b_gen_avx512;

struct crc24c_precompute
{
    static constexpr uint64_t k_CRCPOLY = 0x1B2B117;      //CRC24C polynomial
    //Shift CRCPOLY by 32 minus 24bits since using a 24bit polynomial
    static constexpr uint64_t k_crc_shifted_poly = k_CRCPOLY << 8;
    static constexpr uint32_t k_t192 = 0x8cfa5500;        // t=128+64, x^192 mod crc_shifted_poly
    static constexpr uint32_t k_t128 = 0x6ccc8e00;        // t=128, x^128 mod crc_shifted_poly
    static constexpr uint32_t k_t96 = 0x13979900;         // t=96, x^96 mod crc_shifted_poly
    static constexpr uint32_t k_t64 = 0x74809300;         // t=64, x^64 mod crc_shifted_poly
    static constexpr uint64_t k_u = 0x1c52cdcad;          // u for crc24C * 256, floor(x^64 / crc_shifted_poly)
    static constexpr uint16_t k_crc_bits = 24;            // crc_size (in bits)
    static constexpr uint16_t k_crc_bytes = (k_crc_bits-1)/8+1;  // crc size (bytes)
    static constexpr uint32_t k_init_value = 0x00000000;  // shift reg initialisation value
    static const check_func crc_generate;                 // crc generate function type
};
const check_func crc24c_precompute::crc_generate = bblib_lte_crc24c_gen_avx512;

struct crc24c_1_precompute
{
    // crc24c_1 is the crc24c algorithm but with the payload data appended by crc length
    // number of 1's (ie. 24 1's) as specified in 3GPP TS 38.212 v15.1.1 section 7.3.2
    // Since it is the same algorithm except the initialisation, then it uses the same set of
    // constants for the crc24c, except the initialisation value k_init_value.
    static constexpr uint64_t k_CRCPOLY = crc24c_precompute::k_CRCPOLY;
    //Shift CRCPOLY by 32 minus 24bits since using a 24bit polynomial
    static constexpr uint64_t k_crc_shifted_poly = k_CRCPOLY << 8;
    static constexpr uint32_t k_t192 = crc24c_precompute::k_t192;
    static constexpr uint32_t k_t128 = crc24c_precompute::k_t128;
    static constexpr uint32_t k_t96 = crc24c_precompute::k_t96;
    static constexpr uint32_t k_t64 = crc24c_precompute::k_t64;
    static constexpr uint64_t k_u = crc24c_precompute::k_u;
    static constexpr uint16_t k_crc_bits = crc24c_precompute::k_crc_bits;
    static constexpr uint16_t k_crc_bytes = crc24c_precompute::k_crc_bytes;
    static constexpr uint32_t k_init_value = 0xffffff00;  // initialisation value (24 1's)
    static const check_func crc_generate;                 // crc generate function type
};
const check_func crc24c_1_precompute::crc_generate = bblib_lte_crc24c_1_gen_avx512;

struct crc16_precompute
{
    static constexpr uint64_t k_CRCPOLY = 0x11021;        //CRC16 polynomial
    //Shift CRCPOLY by 32 minus 16bits since using a 16bit polynomial
    static constexpr uint64_t k_crc_shifted_poly = k_CRCPOLY << 16;
    static constexpr uint32_t k_t192 = 0xd5f60000;        // t=128+64, x^192 mod crc_shifted_poly
    static constexpr uint32_t k_t128 = 0x45630000;        // t=128, x^128 mod crc_shifted_poly
    static constexpr uint32_t k_t96 = 0xeb230000;         // t=96, x^96 mod crc_shifted_poly
    static constexpr uint32_t k_t64 = 0xaa510000;         // t=64, x^64 mod crc_shifted_poly
    static constexpr uint64_t k_u = 0x111303471;          // u for crc16 * 256, floor(x^64 / crc_shifted_poly)
    static constexpr uint16_t k_crc_bits = 16;            // crc_size (in bits)
    static constexpr uint16_t k_crc_bytes = (k_crc_bits-1)/8+1;  // crc size (bytes)
    static constexpr uint32_t k_init_value = 0x00000000;  // shift reg initialisation value
    static const check_func crc_generate;                 // crc generate function type
};
const check_func crc16_precompute::crc_generate = bblib_lte_crc16_gen_avx512;

struct crc11_precompute
{
    static constexpr uint64_t k_CRCPOLY = 0xe21;        //CRC11 polynomial
    //Shift CRCPOLY by 32 minus 11bits since using an 11bit polynomial
    static constexpr uint64_t k_crc_shifted_poly = k_CRCPOLY << 21;
    static constexpr uint32_t k_t192 = 0x8ea00000;        // t=128+64, x^192 mod crc_shifted_poly
    static constexpr uint32_t k_t128 = 0x47600000;        // t=128, x^128 mod crc_shifted_poly
    static constexpr uint32_t k_t96 = 0x5e600000;         // t=96, x^96 mod crc_shifted_poly
    static constexpr uint32_t k_t64 = 0xc9000000;         // t=64, x^64 mod crc_shifted_poly
    static constexpr uint64_t k_u = 0x1b3fa1f48;          // u for crc11 * 256, floor(x^64 / crc_shifted_poly)
    static constexpr uint16_t k_crc_bits = 11;            // crc_size (in bits)
    static constexpr uint16_t k_crc_bytes = (k_crc_bits-1)/8+1;   // crc size (bytes)
    static constexpr uint32_t k_init_value = 0x00000000;  // shift reg initialisation value
    static const check_func crc_generate;                 // crc generate function type
};
const check_func crc11_precompute::crc_generate = bblib_lte_crc11_gen_avx512;

struct crc6_precompute
{
    static constexpr uint64_t k_CRCPOLY = 0x61;        //CRC6 polynomial
    //Shift CRCPOLY by 32 minus 6bits since using a 6bit polynomial
    static constexpr uint64_t k_crc_shifted_poly = k_CRCPOLY << 26;
    static constexpr uint32_t k_t192 = 0x38000000;        // t=128+64, x^192 mod crc_shifted_poly
    static constexpr uint32_t k_t128 = 0x1c000000;        // t=128, x^128 mod crc_shifted_poly
    static constexpr uint32_t k_t96 = 0x8c000000;         // t=96, x^96 mod crc_shifted_poly
    static constexpr uint32_t k_t64 = 0xcc000000;         // t=64, x^64 mod crc_shifted_poly
    static constexpr uint64_t k_u = 0x1fab37693;          // u for crc6 * 256, floor(x^64 / crc_shifted_poly)
    static constexpr uint16_t k_crc_bits = 6;             // crc_size (in bits)
    static constexpr uint16_t k_crc_bytes = (k_crc_bits-1)/8+1;  // crc size (bytes)
    static constexpr uint32_t k_init_value = 0x00000000;  // shift reg initialisation value
    static const check_func crc_generate;                 // crc generate function type
};
const check_func crc6_precompute::crc_generate = bblib_lte_crc6_gen_avx512;


// Internal Functions
// Function to bit shift right in m128i register, taking care of 64bit boundary
// shifts are limited to < 64bits
// Params: data  - data in m128i register
//         count - number of bits to shift right (< 64bits)
// Return: shifted data
__m128i bit_shift_right_m128i(__m128i data, __m128i count)
{
    // Rotate top 64bits to isolate required bits to move across 64bit boundary (src0)
    // Perform required shift on rest of data (src1), then use ternarylogic to blend
    // src0 & src1 together to get overall shifted result
    const auto src0 = _mm_rorv_epi64(data, count);
    const auto src1 = _mm_srli_si128(src0, 8);

    // Blend src0 & src1 together based on mask. imm 0xd8 is pre-determined for blend logic table
    const auto mask_r = _mm_sll_epi64(k_shift_mask, (_mm_sub_epi64(k_64, count)));
    return (_mm_ternarylogic_epi64(src0, src1, mask_r, 0xd8));
}


// Function to shift data right, across two __m128i registers
// Params: data      - data in first m128i register
//         next_data - data in second m128i register
//         count     - amount of shift required in bits
// Return: shifted result
__m128i shift_right_256(__m128i data, __m128i next_data, __m128i count)
{
    // Isolate bits to transfer across m128i boundary (src1)
    const auto rot_data = _mm_rorv_epi64(data, count);
    const auto src1 = _mm_slli_si128(rot_data, 8);

    // Shift right next data in m128 register by required amount (src0)
    const auto src0 = bit_shift_right_m128i(next_data, count);

    // Determine mask
    const auto mask_shift_r = _mm_sub_epi64(k_64, count);
    const auto mask_r = _mm_maskz_sll_epi64(0x02, k_shift_mask, mask_shift_r);

    // Blend src0 & src1 together based on mask. imm 0xd8 is pre-determined for blend logic table
    return (_mm_ternarylogic_epi64(src0, src1, mask_r, 0xd8));
}


// CRC Fold Data Function
// Templates: PARAMS     - set of constant values for each CRC type
//            IS_ALIGNED - Indicates if data is byte aligned (ie. multiple of 8 bits)
// Params:    data_in    - pointer to array of m128i types containing message data
//            len        - length of data in bits
// Return: folded data result
template<typename PARAMS, bool IS_ALIGNED>
__m128i fold_4stage(__m128i* data_in, uint32_t len)
{
    const auto iota = _mm_setr_epi8(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15);

    // 1) fold by 128bit. remaining length <=2*128bits
    // set constants and load 1st 16 bytes (128bits) of data, do endian swap
    const auto k_set1 = _mm_set_epi64x(PARAMS::k_t192, PARAMS::k_t128);

    // Determine bit shift size for appending init data
    auto k_shift_size = _mm_maskz_set1_epi32(0x05, PARAMS::k_crc_bits);

    // Load 1st 128bits of data and endian swap. Preserve for later use
    auto foldData = _mm_shuffle_epi8(data_in[0], k_endian_shuf_mask128);
    auto previous_original = foldData;


    // Handle initialising data with 1's at start of data sequence
    // Append CRC length number of 1's to the start of the data sequence for CRCs required
    // to support this, such as CRC24C_1.
    if (PARAMS::k_init_value)
    {
        // shift data to right by CRC size and append initialised value (usually 1's)
        foldData = bit_shift_right_m128i(foldData, k_shift_size);
        auto k_init_mask = _mm_set_epi32(PARAMS::k_init_value,0,0,0);
        foldData = _mm_maskz_or_epi32 (0xff, foldData, k_init_mask);

        // Update the length to include appended value
        len = len + PARAMS::k_crc_bits;
    }
    // preserve state of this data
    auto previous_data = foldData;
    auto previous_init = foldData;


    // Determine length of data (whole bytes)
    const int len_bytes = ((len-1)/8)+1;

    // Determine pad size and if non-aligned, pad data to re-align
    int pad_size = 0;
    auto pad_size_r = _mm_maskz_set1_epi32(0x05, pad_size);
    if (!IS_ALIGNED)
    {
        pad_size = len_bytes*8 - len;
        pad_size_r = _mm_maskz_set1_epi32(0x05, pad_size);
        foldData = bit_shift_right_m128i(foldData, pad_size_r);
    }


    // If len is 32bytes (256bits) or more, process & fold 16 byte sections at a time
    for (int i=1; i < (int)(len_bytes/16); i++)
    {
        previous_data = previous_original;

        // Load next 16 bytes of data & endian swap
        auto nextData_stage1 = _mm_shuffle_epi8(data_in[i], k_endian_shuf_mask128);
        previous_original = nextData_stage1;

        // Handle initialising data with 1's at start of data sequence
        if (PARAMS::k_init_value)
        {
            nextData_stage1 = shift_right_256(previous_data, nextData_stage1, k_shift_size);
        }
        previous_data = nextData_stage1;

        auto previous_data_reserved = nextData_stage1;

        // For none byte aligned data, need to preserve bits that are shifted off the end of
        // the m128i register when padding applied. Isolate these bits and append to the
        // start of the next byte of data.
        if (!IS_ALIGNED)
        {
            nextData_stage1 = shift_right_256(previous_init, nextData_stage1, pad_size_r);
        }

        // Fold data using clmul & xor with next data
        const auto fold128_clmul192_value = _mm_clmulepi64_si128(foldData, k_set1, 0x00);
        const auto fold128_clmul128_value = _mm_clmulepi64_si128(foldData, k_set1, 0x11);
        // use ternary logic to perform 2 xor operations (spec by 0x96) on the above clmul results
        foldData = _mm_ternarylogic_epi32(fold128_clmul192_value, fold128_clmul128_value, nextData_stage1, 0x96);
        previous_init = previous_data_reserved;
    }


    // 2) If remaining length > 128 bits, then pad zero to the most significant bit to align with
    // 256bits, then fold once to 128 bits.
    auto fold128Result = foldData;

    // When greater than 16 bytes, load remaining len%16 bytes and maybe some garbage bytes.
    if (len_bytes > 16)
    {
        // load next 16 bytes of data & endian swap
        auto nextData_stage2 = _mm_shuffle_epi8(data_in[len_bytes/16], k_endian_shuf_mask128);

        // Handle initialising data with 1's at start of data sequence
        if (PARAMS::k_init_value)
        {
            nextData_stage2 = shift_right_256(previous_original, nextData_stage2, k_shift_size);
        }

        // For none byte aligned data, need to preserve bits that are shifted off the end of
        // the m128i register when padding applied. Isolate these bits and append to the
        // start of the next byte of data.
        if (!IS_ALIGNED)
        {
            auto previous_loaded_data = _mm_shuffle_epi8(data_in[(len_bytes/16)-1], k_endian_shuf_mask128);

            // Handle initialising data with 1's at start of data sequence
            if (PARAMS::k_init_value)
            {
                previous_loaded_data = previous_init;
            }

            nextData_stage2 = shift_right_256(previous_loaded_data, nextData_stage2, pad_size_r);
        }

        // Determine byte shifts required for padding operations
        const auto byte_shift = (char)(16 - len_bytes % 16);

        // Setup shift value, rotate foldData, so top block in lsb position & bottom black in msb position
        // Note: bit position corresponds to the destination bit position
        //       and value corresponds to the source bit position
        const auto shift_value = _mm_add_epi8(iota, _mm_set1_epi8(byte_shift));
        const auto shifted_foldData = _mm_shuffle_epi8(foldData, shift_value);

        // Rotate nextData
        const auto shifted_nextData = _mm_shuffle_epi8(nextData_stage2, shift_value);

        // Move top block from foldData into nextData
        const auto next_mask = (__mmask16)(0xFFFF << (16-byte_shift));
        const auto new_nextData_stage2 = _mm_mask_shuffle_epi8(shifted_nextData, next_mask, shifted_foldData, iota);

        // Clear remaining bits in foldData
        const auto fold_mask = (__mmask16)~(next_mask);
        foldData = _mm_maskz_mov_epi8(fold_mask, shifted_foldData);

        // Fold the padded data
        const auto pad128_clmul192_value = _mm_clmulepi64_si128(foldData, k_set1, 0x00);
        const auto pad128_clmul128_value = _mm_clmulepi64_si128(foldData, k_set1, 0x11);
        // Use ternary logic to perform 2 xor operations (spec by 0x96) on the above clmul results
        fold128Result = _mm_ternarylogic_epi32(pad128_clmul192_value, pad128_clmul128_value, new_nextData_stage2, 0x96);
    }
    else
    {
        // Less than 16 bytes, so pad out with zeros
        const auto num_bytes = (char)(16 - len_bytes);
        const auto shift_value = _mm_add_epi8(iota, _mm_set1_epi8(num_bytes));
        const auto mask = (__mmask16)(0xFFFF >> num_bytes);
        fold128Result = _mm_maskz_shuffle_epi8(mask, foldData, shift_value);
    }


    // 3) apply 64 bits fold to 64 bits + 32 bits crc(32 bits zero)
    const auto k_set2 = _mm_set_epi64x(PARAMS::k_t64, PARAMS::k_t96);
    const auto fold64_clmul96_value = _mm_clmulepi64_si128(fold128Result, k_set2, 0x01);

    // realign data to center, packed with 0's
    auto fold64_realign = _mm_slli_si128(fold128Result, 8);
    fold64_realign = _mm_srli_si128(fold64_realign, 4);
    const auto fold64Result = _mm_xor_si128(fold64_clmul96_value, fold64_realign);


    // 4) Apply 32 bits fold to 32 bits + 32 bits crc(32 bits zero)
    const auto fold32_clmul64_value = _mm_clmulepi64_si128(fold64Result, k_set2, 0x11);
    auto fold32_realign = _mm_slli_si128(fold64Result, 8);
    fold32_realign = _mm_srli_si128(fold32_realign, 8);
    return (_mm_xor_si128(fold32_clmul64_value, fold32_realign) );
}


// CRC Barrett Reduction Calculation
// Templates: PARAMS     - set of constant values for each CRC type
//            IS_ALIGNED - Indicates if data is byte aligned (ie. multipleof 8 bits)
// Params:    fold_data  - Output of fold data processes
// Return: CRC value
template<typename PARAMS>
__m128i barrett_reduction(__m128i fold_data)
{
    // 5) Use Barrett Reduction Algorithm to calculate the 32 bit based CRC
    // Output: C(x)  = R(x) mod P(x)
    // Step 1: T1(x) = floor(R(x)/x^32)) * u
    // Step 2: T2(x) = floor(T1(x)/x^32)) * P(x)
    // Step 3: C(x)  = R(x) xor T2(x) mod x^32
    const auto k_br_constants = _mm_set_epi32(1, (PARAMS::k_crc_shifted_poly & 0xFFFFFFFF), 1, (PARAMS::k_u & 0xFFFFFFFF));
    const auto br_realign = _mm_srli_si128(fold_data, 4);
    const auto clmul_u_value = _mm_clmulepi64_si128(k_br_constants, br_realign, 0x00);
    const auto br_clmul_realign = _mm_srli_si128(clmul_u_value, 4);
    const auto br_clmul = _mm_clmulepi64_si128(k_br_constants, br_clmul_realign, 0x01);
    return (_mm_maskz_xor_epi32(0x01,fold_data,br_clmul));
}


// Main CRC Generate Function
// Calculates CRC based on CRC type and message data
// Templates: PARAMS     - set of constant values for each CRC type
//            IS_ALIGNED - Indicates if data is byte aligned (ie. multipleof 8 bits)
// Params:    request    - pointer to request structure
//            response   - pointer to response structure
template<typename PARAMS,bool IS_ALIGNED>
void crc_generate(bblib_crc_request *request, bblib_crc_response *response)
{
    __m128i* dataIn = (__m128i*)request->data;
    uint8_t *dataOut = response->data;

    // Determine data length in whole bytes, based on passed in length in bits.
    const int data_byte_len = ((request->len-1)/8)+1;

    // 1) Calculate CRC using data folding & barrett reduction technique
    const auto fold_result = fold_4stage<PARAMS,IS_ALIGNED>(dataIn, request->len);
    auto br_result = barrett_reduction<PARAMS>(fold_result);


    // 2) Write CRC value (br_result) into response structure
    // Align CRC to right of 32bit segment, then further align CRC value based on CRC size
    constexpr uint32_t crc_shift_value = (32-PARAMS::k_crc_bytes*8)+(PARAMS::k_crc_bytes*8-PARAMS::k_crc_bits);
    const auto align_br_result = _mm_srli_epi32(br_result, crc_shift_value  );
    _mm_mask_storeu_epi8 (&response->crc_value, 0x000f, align_br_result);


    // 3) Append CRC to end of Data steam
    // Identify index to last byte (part or whole)
    const int end_data_idx = data_byte_len - 1;

    // Determine padding size
    const int pad_size = (data_byte_len*8 - request->len) + 24;
    const auto pad_size_r = _mm_maskz_set1_epi32(0x01, pad_size);

    // Load last byte of data (endian swapped), using mask to ensure rest of data is zeroed
    // then pad data to RHS of 32bit word ready to be merged with CRC
    const auto end_data_es = _mm_maskz_shuffle_epi8(0x8000, *(__m128i*)(request->data+end_data_idx), k_endian_shuf_mask128);
    const auto pad_end_data = _mm_srl_epi32(end_data_es, pad_size_r);

    // Merge data with CRC value (br_result) and shift back
    const auto merged_crc = _mm_mask_shuffle_epi8 (pad_end_data, 0x0f00, br_result, k_shuf_mask);
    const auto appended_data = _mm_sll_epi64(merged_crc, pad_size_r);

    // Write CRC appended data to memory (needs an endian swap)
    const auto appended_data_es = _mm_shuffle_epi8(appended_data, k_endian_shuf_mask128);
    _mm_mask_storeu_epi8 (dataOut+end_data_idx, 0xffff, appended_data_es);

    // Update length of CRC appended data
    response->len = request->len + PARAMS::k_crc_bits;
}


// Main CRC Check Function
// Extracts CRC from data and compares with calculated version to validate CRC
// Templates: PARAMS     - set of constant values for each CRC type
//            IS_ALIGNED - Indicates if data is byte aligned (ie. multipleof 8 bits)
// Params:    request    - pointer to request structure
//            response   - pointer to response structure
template<typename PARAMS,bool IS_ALIGNED>
void crc_check(bblib_crc_request *request, bblib_crc_response *response)
{
    // Identify load point in data, which includes start of CRC bits
    const int load_idx = request->len/8;

    // determine alignment shift for left alignment of crc
    const int align_shift = request->len%8;
    const auto align_shift_r = _mm_setr_epi32(align_shift,0,0,0);

    // Extract CRC which may include some bits from the end of data
    // Load last bytes of data + CRC (endian swapped), masking out rest of data.
    const auto crc_end_data = _mm_maskz_shuffle_epi8 (0x000f, *(__m128i*)(request->data+load_idx), k_endian_shuf_mask32);

    // Bit left align CRC to remove remaining bits of data
    const auto crc_no_data = _mm_sll_epi32(crc_end_data, align_shift_r);

    // Byte align to right of 32bit word, taking into account size of CRC (6, 11 etc)
    static constexpr uint16_t k_crc_alignment_shift = (4-PARAMS::k_crc_bytes)*8 + (PARAMS::k_crc_bytes*8 - PARAMS::k_crc_bits);
    const auto crc = _mm_srli_epi32(crc_no_data, k_crc_alignment_shift);
    uint32_t *p_crc = (uint32_t*)&crc;

    // Generate CRC & compare results with crc from data (orig_crc)
    crc_generate<PARAMS, IS_ALIGNED>(request, response);
    response->check_passed = (response->crc_value == *p_crc);
}


// External Functions
void bblib_lte_crc24a_gen_avx512(bblib_crc_request *request, bblib_crc_response *response)
{
    if (request->len%8)
    {
        crc_generate<crc24a_precompute,false>(request, response);
    }
    else
    {
        crc_generate<crc24a_precompute,true>(request, response);
    }
}

void bblib_lte_crc24b_gen_avx512(bblib_crc_request *request, bblib_crc_response *response)
{
    if (request->len%8)
    {
        crc_generate<crc24b_precompute,false>(request, response);
    }
    else
    {
        crc_generate<crc24b_precompute,true>(request, response);
    }
}

void bblib_lte_crc24c_gen_avx512(bblib_crc_request *request, bblib_crc_response *response)
{
    if (request->len%8)
    {
        crc_generate<crc24c_precompute,false>(request, response);
    }
    else
    {
        crc_generate<crc24c_precompute,true>(request, response);
    }
}

void bblib_lte_crc24c_1_gen_avx512(bblib_crc_request *request, bblib_crc_response *response)
{
    if (request->len%8)
    {
        crc_generate<crc24c_1_precompute,false>(request, response);
    }
    else
    {
        crc_generate<crc24c_1_precompute,true>(request, response);
    }
}

void bblib_lte_crc16_gen_avx512(bblib_crc_request *request, bblib_crc_response *response)
{
    if (request->len%8)
    {
        crc_generate<crc16_precompute,false>(request, response);
    }
    else
    {
        crc_generate<crc16_precompute,true>(request, response);
    }
}

void bblib_lte_crc11_gen_avx512(bblib_crc_request *request, bblib_crc_response *response)
{
    if (request->len%8)
    {
        crc_generate<crc11_precompute,false>(request, response);
    }
    else
    {
        crc_generate<crc11_precompute,true>(request, response);
    }
}

void bblib_lte_crc6_gen_avx512(bblib_crc_request *request, bblib_crc_response *response)
{
    if (request->len%8)
    {
        crc_generate<crc6_precompute,false>(request, response);
    }
    else
    {
        crc_generate<crc6_precompute,true>(request, response);
    }
}

void bblib_lte_crc24a_check_avx512(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    if (request->len%8)
    {
        crc_check<crc24a_precompute,false>(request, response);
    }
    else
    {
        crc_check<crc24a_precompute,true>(request, response);
    }
}

void bblib_lte_crc24b_check_avx512(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    if (request->len%8)
    {
        crc_check<crc24b_precompute,false>(request, response);
    }
    else
    {
        crc_check<crc24b_precompute,true>(request, response);
    }
}

void bblib_lte_crc24c_check_avx512(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    if (request->len%8)
    {
        crc_check<crc24c_precompute,false>(request, response);
    }
    else
    {
        crc_check<crc24c_precompute,true>(request, response);
    }
}

void bblib_lte_crc24c_1_check_avx512(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    if (request->len%8)
    {
        crc_check<crc24c_1_precompute,false>(request, response);
    }
    else
    {
        crc_check<crc24c_1_precompute,true>(request, response);
    }
}

void bblib_lte_crc16_check_avx512(bblib_crc_request *request, bblib_crc_response *response)
{
    if (request->len%8)
    {
        crc_check<crc16_precompute,false>(request, response);
    }
    else
    {
        crc_check<crc16_precompute,true>(request, response);
    }
}

void bblib_lte_crc11_check_avx512(bblib_crc_request *request, bblib_crc_response *response)
{
    if (request->len%8)
    {
        crc_check<crc11_precompute,false>(request, response);
    }
    else
    {
        crc_check<crc11_precompute,true>(request, response);
    }
}

void bblib_lte_crc6_check_avx512(bblib_crc_request *request, bblib_crc_response *response)
{
    if (request->len%8)
    {
        crc_check<crc6_precompute,false>(request, response);
    }
    else
    {
        crc_check<crc6_precompute,true>(request, response);
    }
}


#else
void bblib_lte_crc24a_gen_avx512(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    printf("bblib_crc requires AVX512 ISA support to run\n");
    exit(-1);
}
void bblib_lte_crc24b_gen_avx512(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    printf("bblib_crc requires AVX512 ISA support to run\n");
    exit(-1);
}
void bblib_lte_crc24c_gen_avx512(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    printf("bblib_crc requires AVX512 ISA support to run\n");
    exit(-1);
}
void bblib_lte_crc24c_1_gen_avx512(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    printf("bblib_crc requires AVX512 ISA support to run\n");
    exit(-1);
}
void bblib_lte_crc16_gen_avx512(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    printf("bblib_crc requires AVX512 ISA support to run\n");
    exit(-1);
}
void bblib_lte_crc6_gen_avx512(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    printf("bblib_crc requires AVX512 ISA support to run\n");
    exit(-1);
}
void bblib_lte_crc11_gen_avx512(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    printf("bblib_crc requires AVX512 ISA support to run\n");
    exit(-1);
}

void bblib_lte_crc24a_check_avx512(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    printf("bblib_crc requires AVX512 ISA support to run\n");
    exit(-1);
}
void bblib_lte_crc24b_check_avx512(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    printf("bblib_crc requires AVX512 ISA support to run\n");
    exit(-1);
}
void bblib_lte_crc24c_check_avx512(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    printf("bblib_crc requires AVX512 ISA support to run\n");
    exit(-1);
}
void bblib_lte_crc24c_1_check_avx512(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    printf("bblib_crc requires AVX512 ISA support to run\n");
    exit(-1);
}
void bblib_lte_crc16_check_avx512(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    printf("bblib_crc requires AVX512 ISA support to run\n");
    exit(-1);
}
void bblib_lte_crc6_check_avx512(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    printf("bblib_crc requires AVX512 ISA support to run\n");
    exit(-1);
}
void bblib_lte_crc11_check_avx512(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    printf("bblib_crc requires AVX512 ISA support to run\n");
    exit(-1);
}

#endif
