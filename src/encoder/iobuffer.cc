/** 
 * @file iobuffer.cpp
 * @brief Read the input files into z-bit segments; regroup the output from 
 * z-bit segments to bytes
 */

#include "iobuffer.h"
#include "common_typedef_sdk.h"
#include "encoder.h"

namespace avx2enc {
void ScatterSlow(
    uint8_t* dst, const uint8_t* src, unsigned num_bits, uint8_t src_offbits)
{
    // Process byte by byte
    while (num_bits != 0) {
        unsigned num_bits_in_b = MIN(8, num_bits);
        uint8_t new_b;
        if (src_offbits == 0) {
            new_b = src[0];
        } else {
            new_b = ((src[0] & 0xFF) >> src_offbits)
                | ((src[1] & 0xFF) << (8 - src_offbits));
        }
        dst[0] = new_b & BITMASKU8(num_bits_in_b);
        num_bits -= num_bits_in_b;

        dst++;
        src++;
    }
}

void GatherSlow(
    uint8_t* dst, const uint8_t* src, int16_t num_bits, uint8_t dst_offbits)
{
    // Process byte by byte
    bool first_byte = true;
    while (num_bits > 0) {
        unsigned num_bits_in_b = MIN(8, num_bits);
        uint8_t new_b;
        if (dst_offbits == 0) {
            // simple copy
            new_b = src[0] & BITMASKU8(num_bits_in_b);
            src++;
        } else {
            if (first_byte) {
                new_b = (dst[0] & BITMASKU8(dst_offbits))
                    | (src[0] & 0xFF) << dst_offbits;
                num_bits_in_b = 8 - dst_offbits;
                first_byte = false;
            } else {
                new_b = ((src[0] & 0xFF) >> (8 - dst_offbits)
                            | (src[1] & 0xFF) << dst_offbits)
                    & BITMASKU8(num_bits_in_b);
                src++;
            }
        }
        dst[0] = new_b;
        num_bits -= num_bits_in_b;
        dst++;
    }
}

void Adapter2to64(int8_t* pBuff0, int8_t* pBuff1, uint16_t zcSize,
    uint32_t cbLen, int8_t direct)
{
    int8_t* p_buff_0;
    int8_t* p_buff_1;
    uint8_t dst_offbits = 0;
    uint8_t src_offbits = 0;
    p_buff_0 = pBuff0;
    p_buff_1 = pBuff1;

    if (1 == direct) {
        /* parsing the input
        p_buff_0 is the input, p_buff_1 is the buffer for barrel shifter */
        for (size_t i = 0; i < cbLen / zcSize; i++) {

            ScatterSlow(
                (uint8_t*)p_buff_1, (uint8_t*)p_buff_0, zcSize, src_offbits);
            uint8_t byte_offset = (src_offbits + zcSize) >> 3;
            src_offbits = (src_offbits + zcSize) - (byte_offset << 3);
            p_buff_0 = p_buff_0 + byte_offset;
            p_buff_1 = p_buff_1 + kProcBytes;
        }
    } else {
        /* storing encoded bits into output buffer
        p_buff_0 is the output, p_buff_1 is the buffer for processing data*/
        for (size_t i = 0; i < cbLen / zcSize; i++) {
            GatherSlow((uint8_t*)p_buff_0, (uint8_t*)p_buff_1, (int16_t)zcSize,
                dst_offbits);
            uint8_t byte_offset = (dst_offbits + zcSize) >> 3;
            dst_offbits = (dst_offbits + zcSize) - (byte_offset << 3);
            p_buff_0 = p_buff_0 + byte_offset;
            p_buff_1 = p_buff_1 + kProcBytes;
        }
    }
}

// void print256_epi8(__m256i var)
// {
//     int8_t *val = (int8_t*) &var;
//     std::printf("Numerical int8_t: %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i
//     %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i \n",
//            val[0], val[1], val[2], val[3], val[4], val[5], val[6], val[7],
//            val[8], val[9], val[10], val[11], val[12], val[13], val[14],
//            val[15], val[16], val[17], val[18], val[19], val[20], val[21],
//            val[22], val[23], val[24], val[25], val[26], val[27], val[28],
//            val[29], val[30], val[31]
//            );
// }

void Adapter64to256(int8_t* pBuff0, int8_t* pBuff1, uint16_t zcSize,
    uint32_t cbLen, int8_t direct)
{
    /* after 64, z is always a multiple of 8 so no need for shifting bytes*/

    int8_t* p_buff_0;
    int8_t* p_buff_1;
    __m256i bit_mask;
    __m256i x0;
    __m256i x1;
    int64_t e0;
    int64_t e1;
    int64_t e2;
    int16_t byte_num = zcSize >> 3;

    p_buff_0 = pBuff0;
    p_buff_1 = pBuff1;

    if (zcSize >= 256) {
        e0 = 0xffffffffffffffff;
        e1 = 0xffffffffffffffff;
        e2 = 0xffffffffffffffff;
    } else if (zcSize >= 192) {
        e0 = 0xffffffffffffffff;
        e1 = 0xffffffffffffffff;
        e2 = (1UL << (zcSize - 192)) - 1;
    } else if (zcSize >= 128) {
        e0 = 0xffffffffffffffff;
        e1 = (1UL << (zcSize - 128)) - 1;
        e2 = 0;
    } else if (zcSize >= 64) {
        e0 = (1UL << (zcSize - 64)) - 1;
        e1 = 0;
        e2 = 0;
    }
    bit_mask = _mm256_set_epi64x(e2, e1, e0, 0xffffffffffffffff);

    // scatter
    if (1 == direct) {
        for (size_t i = 0; i < cbLen / zcSize; i++) {
            x0 = _mm256_loadu_si256((__m256i*)(p_buff_0 + i * byte_num));
            x1 = _mm256_and_si256(x0, bit_mask);
            _mm256_storeu_si256((__m256i*)p_buff_1, x1);
            // std::printf("before: ");
            // print256_epi8(x0);
            // std::printf("after: ");
            // print256_epi8(x1);
            p_buff_1 = p_buff_1 + kProcBytes;
        }
    }
    // gather
    else {
        for (size_t i = 0; i < cbLen / zcSize; i++) {
            x0 = _mm256_loadu_si256((__m256i*)p_buff_1);
            x1 = _mm256_and_si256(x0, bit_mask);
            _mm256_storeu_si256((__m256i*)p_buff_0, x1);
            // std::printf("before: ");
            // print256_epi8(x0);
            // std::printf("after: ");
            // print256_epi8(x1);
            p_buff_1 = p_buff_1 + kProcBytes;
            p_buff_0 = p_buff_0 + byte_num;
        }
    }
}

void Adapter288to384(int8_t* pBuff0, int8_t* pBuff1, uint16_t zcSize,
    uint32_t cbLen, int8_t direct)
{
    /* use two __m256i to store one segment of length zc */
    int8_t* p_buff_in;
    int8_t* p_buff_out;
    __m256i x0;
    __m256i bit_mask;

    p_buff_in = pBuff0;
    p_buff_out = pBuff1;
    int xtra_byte_num = (zcSize - 256) >> 3;

    bit_mask = _mm256_set_epi32(0, 0, 0, 0, -(xtra_byte_num - 15),
        -(xtra_byte_num - 11), -(xtra_byte_num - 7), -1);

    if (1 == direct) {
        for (size_t i = 0; i < cbLen / zcSize; i++) {
            // read the first 256 bits
            x0 = _mm256_loadu_si256((__m256i*)p_buff_in);
            _mm256_storeu_si256((__m256i*)p_buff_out, x0);
            p_buff_out = p_buff_out + kProcBytes;
            p_buff_in = p_buff_in + kProcBytes;
            // read the remaining bits
            x0 = _mm256_loadu_si256((__m256i*)p_buff_in);
            _mm256_maskstore_epi32((int*)p_buff_out, bit_mask, x0);
            p_buff_out = p_buff_out + kProcBytes;
            p_buff_in = p_buff_in + xtra_byte_num;
        }
    }
}

LDPC_ADAPTER_P LdpcSelectAdapterFunc(uint16_t zcSize)
{
    if (zcSize < 64) {
        return Adapter2to64;
    } else if (zcSize <= 256) {
        return Adapter64to256;
    } else {
        return Adapter288to384;
    }
}
} // namespace avx2en
