#include "encoder.hpp"
#include "common_typedef_sdk.h"
#include "cyclic_shift.hpp"
#include "iobuffer.hpp"

namespace avx2enc {
void ldpc_encoder_bg1(int8_t* pDataIn, int8_t* pDataOut,
    const int16_t* pMatrixNumPerCol, const int16_t* pAddr,
    const int16_t* pShiftMatrix, int16_t zcSize, uint8_t i_LS)
{
    const int16_t *pTempAddr, *pTempMatrix;
    int8_t *pTempIn, *pTempOut;
    int16_t addrOffset = 0;
    size_t i = 0;
    __m256i x1, x2, x3, x4, x5, x6, x7, x8, x9;
    CYCLIC_BIT_SHIFT cycle_bit_shift_p = ldpc_select_shift_func(zcSize);

    for (size_t j = 0; j < BG1_ROW_TOTAL; j++) {
        _mm256_storeu_si256(
            (__m256i*)(pDataOut + j * kProcBytes), _mm256_set1_epi8(0));
    }

    pTempAddr = pAddr;
    pTempMatrix = pShiftMatrix;
    pTempIn = pDataIn;
    pTempOut = pDataOut;

    x1 = _mm256_loadu_si256((__m256i*)(pTempIn + i * kProcBytes));

    // getting lambdas
    for (int32_t j = 0; j < *(pMatrixNumPerCol + i); j++) {
        x2 = cycle_bit_shift_p(x1, *pTempMatrix++, zcSize);
        addrOffset = (*pTempAddr++) >> 1;
        _mm256_storeu_si256((__m256i*)(pTempOut + addrOffset), x2);
    }

    i = 1;
    for (; i < BG1_COL_INF_NUM; i++) {
        x1 = _mm256_loadu_si256((__m256i*)(pTempIn + i * kProcBytes));
        for (int32_t j = 0; j < *(pMatrixNumPerCol + i); j++) {
            addrOffset = (*pTempAddr++) >> 1;
            x2 = cycle_bit_shift_p(x1, *pTempMatrix++, zcSize);
            x3 = _mm256_loadu_si256((__m256i*)(pTempOut + addrOffset));
            x4 = _mm256_xor_si256(x2, x3);
            _mm256_storeu_si256((__m256i*)(pTempOut + addrOffset), x4);
        }
    }

    // std::printf("pTempOut: \n");
    // for (int i = 0; i < BG1_ROW_TOTAL * kProcBytes; i++) {
    //     std::printf("%i ", pTempOut[i]);
    // }
    // std::printf("\n");

    // Row Transform to resolve the small 4x4 parity matrix
    // lambdas
    x1 = _mm256_loadu_si256((__m256i*)pTempOut);
    x2 = _mm256_loadu_si256((__m256i*)(pTempOut + kProcBytes));
    x3 = _mm256_loadu_si256((__m256i*)(pTempOut + 2 * kProcBytes));
    x4 = _mm256_loadu_si256((__m256i*)(pTempOut + 3 * kProcBytes));

    // first 384
    // x5 is p_a1
    x5 = _mm256_xor_si256(x1, x2);
    x5 = _mm256_xor_si256(x5, x3);
    x5 = _mm256_xor_si256(x5, x4);

    // Special case for the circulant
    if (i_LS == 6) {
        x5 = cycle_bit_shift_p(x5, 103, zcSize);
        _mm256_storeu_si256((__m256i*)pDataOut, x5);
    } else
        _mm256_storeu_si256((__m256i*)pDataOut, x5);

    // second 384
    // x7 is p_a2
    if (i_LS == 6)
        x6 = x5;
    else
        x6 = cycle_bit_shift_p(x5, 1, zcSize);

    x7 = _mm256_xor_si256(x1, x6);
    _mm256_storeu_si256((__m256i*)(pDataOut + kProcBytes), x7);

    // third 384 - c2(x2)+w2(x7)=w3(x8)
    // p_a3
    x8 = _mm256_xor_si256(x4, x6);
    _mm256_storeu_si256((__m256i*)(pDataOut + 3 * kProcBytes), x8);

    // fourth 384 - c4(x4)+w1_0(x6)=w4(x9)
    // pa_4
    x9 = _mm256_xor_si256(x3, x8);
    _mm256_storeu_si256((__m256i*)(pDataOut + 2 * kProcBytes), x9);

    // Rest of parity based on identity matrix
    // p_c's
    for (; i < 4 + BG1_COL_INF_NUM; i++) {
        x1 = _mm256_loadu_si256(
            (__m256i*)(pDataOut + (i - BG1_COL_INF_NUM) * kProcBytes));
        for (int32_t j = 0; j < *(pMatrixNumPerCol + i); j++) {
            addrOffset = (*pTempAddr++) >> 1;
            // addrOffset = *pTempAddr++;
            x2 = cycle_bit_shift_p(x1, *pTempMatrix++, zcSize);
            x3 = _mm256_loadu_si256((__m256i*)(pTempOut + addrOffset));
            x4 = _mm256_xor_si256(x2, x3);
            _mm256_storeu_si256((__m256i*)(pTempOut + addrOffset), x4);
        }
    }
}

void ldpc_encoder_bg2(int8_t* pDataIn, int8_t* pDataOut,
    const int16_t* pMatrixNumPerCol, const int16_t* pAddr,
    const int16_t* pShiftMatrix, int16_t zcSize, uint8_t i_LS)
{
    const int16_t *pTempAddr, *pTempMatrix;
    int8_t *pTempIn, *pTempOut;
    int16_t addrOffset = 0;
    size_t i = 0;
    __m256i x1, x2, x3, x4, x5, x6, x7, x8, x9;
    CYCLIC_BIT_SHIFT cycle_bit_shift_p = ldpc_select_shift_func(zcSize);

    for (size_t j = 0; j < BG2_ROW_TOTAL; j++) {
        _mm256_storeu_si256(
            (__m256i*)(pDataOut + j * kProcBytes), _mm256_set1_epi8(0));
    }

    pTempAddr = pAddr;
    pTempMatrix = pShiftMatrix;
    pTempIn = pDataIn;
    pTempOut = pDataOut;

    x1 = _mm256_loadu_si256((__m256i*)(pTempIn + i * kProcBytes));

    // getting lambdas
    for (int32_t j = 0; j < *(pMatrixNumPerCol + i); j++) {
        x2 = cycle_bit_shift_p(x1, *pTempMatrix++, zcSize);
        addrOffset = (*pTempAddr++) >> 1;
        _mm256_storeu_si256((__m256i*)(pTempOut + addrOffset), x2);
    }
    i = 1;
    for (; i < BG2_COL_INF_NUM; i++) {
        x1 = _mm256_loadu_si256((__m256i*)(pTempIn + i * kProcBytes));
        // can add threading here
        // *********************************************************
        for (int32_t j = 0; j < *(pMatrixNumPerCol + i); j++) {
            addrOffset = (*pTempAddr++) >> 1;
            x2 = cycle_bit_shift_p(x1, *pTempMatrix++, zcSize);
            x3 = _mm256_loadu_si256((__m256i*)(pTempOut + addrOffset));
            x4 = _mm256_xor_si256(x2, x3);
            _mm256_storeu_si256((__m256i*)(pTempOut + addrOffset), x4);
        }
    }

    // Row Transform to resolve the small 4x4 parity matrix
    // lambdas
    x1 = _mm256_loadu_si256((__m256i*)pTempOut);
    x2 = _mm256_loadu_si256((__m256i*)(pTempOut + kProcBytes));
    x3 = _mm256_loadu_si256((__m256i*)(pTempOut + 2 * kProcBytes));
    x4 = _mm256_loadu_si256((__m256i*)(pTempOut + 3 * kProcBytes));

    // first 384
    // x5 is p_a1
    x5 = _mm256_xor_si256(x1, x2);
    x5 = _mm256_xor_si256(x5, x3);
    x5 = _mm256_xor_si256(x5, x4);

    // Special case for the circulant
    if ((i_LS == 3) || (i_LS == 7))
        _mm256_storeu_si256((__m256i*)pDataOut, x5);
    else {
        x5 = cycle_bit_shift_p(x5, (zcSize - 1), zcSize);
        _mm256_storeu_si256((__m256i*)pDataOut, x5);
    }

    // second 384
    // x7 is p_a2
    if ((i_LS == 3) || (i_LS == 7))
        x6 = cycle_bit_shift_p(x5, 1, zcSize);
    else
        x6 = x5;

    x7 = _mm256_xor_si256(x1, x6);
    _mm256_storeu_si256((__m256i*)(pDataOut + kProcBytes), x7);

    // third 384 - c2(x2)+w2(x7)=w3(x8)
    // p_a3
    x8 = _mm256_xor_si256(x2, x7);
    _mm256_storeu_si256((__m256i*)(pDataOut + 2 * kProcBytes), x8);

    // fourth 384 - c4(x4)+w1_0(x6)=w4(x9)
    // pa_4
    x9 = _mm256_xor_si256(x4, x6);
    _mm256_storeu_si256((__m256i*)(pDataOut + 3 * kProcBytes), x9);

    // Rest of parity based on identity matrix
    // p_c's
    for (; i < 4 + BG2_COL_INF_NUM; i++) {
        x1 = _mm256_loadu_si256(
            (__m256i*)(pDataOut + (i - BG2_COL_INF_NUM) * kProcBytes));
        for (int32_t j = 0; j < *(pMatrixNumPerCol + i); j++) {
            addrOffset = (*pTempAddr++) >> 1;
            x2 = cycle_bit_shift_p(x1, *pTempMatrix++, zcSize);
            x3 = _mm256_loadu_si256((__m256i*)(pTempOut + addrOffset));
            x4 = _mm256_xor_si256(x2, x3);
            _mm256_storeu_si256((__m256i*)(pTempOut + addrOffset), x4);
        }
    }
}

int32_t bblib_ldpc_encoder_5gnr(struct bblib_ldpc_encoder_5gnr_request* request,
    struct bblib_ldpc_encoder_5gnr_response* response)
{
    // input -----------------------------------------------------------
    // these values depend on the application
    uint16_t Zc = request->Zc;
    if (Zc > ZC_MAX) {
        std::fprintf(stderr, "Error: This AVX2 encoder supports only Zc <= %d\n",
            ZC_MAX);
        std::exit(-1);
    }

    int numberCodeblocks = request->numberCodeblocks;
    uint16_t Bg = request->baseGraph;

    // int nRows = (Bg == 1) ? BG1_ROW_TOTAL : BG2_ROW_TOTAL;
    uint32_t cbEncLen = request->nRows * Zc;
    uint32_t cbLen = (Bg == 1) ? Zc * BG1_COL_INF_NUM : Zc * BG2_COL_INF_NUM;

    int8_t* input[numberCodeblocks];
    int8_t* parity[numberCodeblocks];
    for (int i = 0; i < numberCodeblocks; i++) {
        input[i] = request->input[i];
        parity[i] = response->output[i];
    }

    // i_Ls decides the base matrix entries
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

    const int16_t* pShiftMatrix;
    const int16_t* pMatrixNumPerCol;
    const int16_t* pAddr;
    if (Bg == 1) {
        pShiftMatrix = Bg1HShiftMatrix + i_LS * BG1_NONZERO_NUM;
        pMatrixNumPerCol = Bg1MatrixNumPerCol;
        pAddr = Bg1Address;
    } else {
        pShiftMatrix = Bg2HShiftMatrix + i_LS * BG2_NONZERO_NUM;
        pMatrixNumPerCol = Bg2MatrixNumPerCol;
        pAddr = Bg2Address;
    }

    __attribute__((aligned(64)))
    int8_t input_internal_buffer[BG1_COL_TOTAL * avx2enc::kProcBytes]
        = { 0 };
    __attribute__((aligned(64)))
    int8_t parity_internal_buffer[BG1_ROW_TOTAL * avx2enc::kProcBytes]
        = { 0 };

    avx2enc::LDPC_ADAPTER_P ldpc_adapter_func
        = avx2enc::ldpc_select_adapter_func(Zc);
    auto ldpc_encoder_func
        = (Bg == 1 ? avx2enc::ldpc_encoder_bg1 : avx2enc::ldpc_encoder_bg2);

    for (int n = 0; n < numberCodeblocks; n++) {
        // Scatter Zc-bit chunks of the input into kProcBytes-sized chunks
        // of input_internal_buffer
        ldpc_adapter_func(input[n], input_internal_buffer, Zc, cbLen, 1);

        // Encode into parity_internal_buffer
        ldpc_encoder_func(input_internal_buffer, parity_internal_buffer,
            pMatrixNumPerCol, pAddr, pShiftMatrix, (int16_t)Zc, i_LS);

        // Gather parity bits from kProcBytes-sized chunks of
        // parity_internal_buffer
        ldpc_adapter_func(parity[n], parity_internal_buffer, Zc, cbEncLen, 0);
    }

    return 0;
}
} // namespace avx2enc