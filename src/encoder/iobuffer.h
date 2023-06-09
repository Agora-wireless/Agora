/**
 * @file iobuffer.h
 * @brief Declaration for for the avx2enc iobuffer functions
 */
#ifndef IOBUFFER_H_
#define IOBUFFER_H_

#include <immintrin.h>

#include "encoder.h"

namespace avx2enc {
void Adapter2to64(int8_t* ptr_buff_0, __int8_t* ptr_buff_1, uint16_t zc_size,
                  uint32_t cb_len_bits, int8_t direct);

void Adapter64to256(int8_t* ptr_buff_0, int8_t* ptr_buff_1, uint16_t zc_size,
                    uint32_t cb_len_bits, int8_t direct);

void Adapter288to384(int8_t* ptr_buff_0, int8_t* ptr_buff_1, uint16_t zc_size,
                     uint32_t cb_len_bits, int8_t direct);

using LDPC_ADAPTER_P = void (*)(int8_t*, int8_t*, uint16_t, uint32_t, int8_t);
LDPC_ADAPTER_P LdpcSelectAdapterFunc(uint16_t zc_size);
}  // namespace avx2enc

#endif  // IOBUFFER_H_
