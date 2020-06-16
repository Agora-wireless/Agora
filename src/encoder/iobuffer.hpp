#ifndef _IOBUFFER_H_
#define _IOBUFFER_H_

#include <immintrin.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace avx2enc {
void adapter_2to64(int8_t* pBuff0, __int8_t* pBuff1, uint16_t zcSize,
    uint32_t cbLen, int8_t direct);

void adapter_64to256(int8_t* pBuff0, int8_t* pBuff1, uint16_t zcSize,
    uint32_t cbLen, int8_t direct);

void adapter_288to384(int8_t* pBuff0, int8_t* pBuff1, uint16_t zcSize,
    uint32_t cbLen, int8_t direct);

using LDPC_ADAPTER_P = void (*)(int8_t*, int8_t*, uint16_t, uint32_t, int8_t);
LDPC_ADAPTER_P ldpc_select_adapter_func(uint16_t zcSize);
} // namespace avx2enc

#endif
