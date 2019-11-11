#include "encoder.hpp"
#include "cyclic_shift.hpp"


void ldpc_encoder_bg1(int8_t *pDataIn, int8_t *pDataOut, const int16_t *pMatrixNumPerCol,
        const int16_t *pAddr, const int16_t *pShiftMatrix, int16_t zcSize, uint8_t i_LS)
{
    const int16_t *pTempAddr,  *pTempMatrix;
    int8_t *pTempIn,*pTempOut;
    int16_t addrOffset = 0;
    int32_t i = 0;
    __m256i x1,x2,x3,x4,x5,x6,x7,x8,x9;
    CYCLIC_BIT_SHIFT cycle_bit_shift_p = ldpc_select_shift_func(zcSize);

    for(int32_t j = 0; j < BG1_ROW_TOTAL; j++){
        _mm256_storeu_si256 ((__m256i*) (pDataOut + j*PROC_BYTES), _mm256_set1_epi8(0));
    }

    pTempAddr = pAddr;
    pTempMatrix = pShiftMatrix;
    pTempIn = pDataIn;
    pTempOut = pDataOut;

    x1 = _mm256_loadu_si256 ((__m256i*)(pTempIn + i * PROC_BYTES));

    // getting lambdas
    for(int32_t j = 0; j<*(pMatrixNumPerCol + i); j++) {
        x2 = cycle_bit_shift_p(x1, *pTempMatrix++, zcSize);
        addrOffset = (*pTempAddr++)>>1;
        _mm256_storeu_si256 ((__m256i *) (pTempOut + addrOffset), x2);
    }

    i = 1; 
    for(; i<BG1_COL_INF_NUM; i++) {
        x1 = _mm256_loadu_si256 ((__m256i*)(pTempIn + i * PROC_BYTES));
        for(int32_t j = 0; j<*(pMatrixNumPerCol + i); j++) {
            addrOffset = (*pTempAddr++)>>1;
            x2 = cycle_bit_shift_p(x1, *pTempMatrix++, zcSize);
            x3 = _mm256_loadu_si256 ((__m256i*) (pTempOut + addrOffset));
            x4 = _mm256_xor_si256 (x2, x3); 
            _mm256_storeu_si256 ((__m256i*) (pTempOut + addrOffset), x4);
        }
    }

    // printf("pTempOut: \n");
    // for (int i = 0; i < BG1_ROW_TOTAL * PROC_BYTES; i++) {
    //     printf("%i ", pTempOut[i]);
    // }
    // printf("\n");

    // Row Transform to resolve the small 4x4 parity matrix
    // lambdas
    x1 = _mm256_loadu_si256 ((__m256i*) pTempOut);
    x2 = _mm256_loadu_si256 ((__m256i*) (pTempOut + PROC_BYTES));
    x3 = _mm256_loadu_si256 ((__m256i*) (pTempOut + 2*PROC_BYTES));
    x4 = _mm256_loadu_si256 ((__m256i*) (pTempOut + 3*PROC_BYTES));

    //first 384
    // x5 is p_a1
    x5 = _mm256_xor_si256 (x1, x2);
    x5 = _mm256_xor_si256 (x5, x3);
    x5 = _mm256_xor_si256 (x5, x4);

    // Special case for the circulant
    if (i_LS == 6) {
        x5 = cycle_bit_shift_p(x5, 103, zcSize);
        _mm256_storeu_si256 ((__m256i*) pDataOut, x5);
    }
    else
        _mm256_storeu_si256 ((__m256i*) pDataOut, x5);

    //second 384
    // x7 is p_a2
    if (i_LS == 6)
        x6 = x5;
    else
        x6 = cycle_bit_shift_p(x5, 1, zcSize);

    x7 = _mm256_xor_si256 (x1, x6);
    _mm256_storeu_si256 ((__m256i*) (pDataOut + PROC_BYTES), x7);

    //third 384 - c2(x2)+w2(x7)=w3(x8)
    // p_a3
    x8 = _mm256_xor_si256 (x4, x6);
    _mm256_storeu_si256 ((__m256i*) (pDataOut + 3*PROC_BYTES), x8);

    //fourth 384 - c4(x4)+w1_0(x6)=w4(x9)
    // pa_4
    x9 = _mm256_xor_si256 (x3, x8);
    _mm256_storeu_si256 ((__m256i*) (pDataOut + 2*PROC_BYTES), x9);

    // Rest of parity based on identity matrix
    // p_c's
    for(; i < 4 + BG1_COL_INF_NUM; i++) {
        x1 = _mm256_loadu_si256 ((__m256i*)(pDataOut + (i - BG1_COL_INF_NUM) * PROC_BYTES));
        for(int32_t j = 0; j<*(pMatrixNumPerCol + i); j++) {
            addrOffset = (*pTempAddr++) >> 1;
            //addrOffset = *pTempAddr++;
            x2 = cycle_bit_shift_p(x1, *pTempMatrix++, zcSize);
            x3 = _mm256_loadu_si256 ((__m256i*) (pTempOut + addrOffset));
            x4 = _mm256_xor_si256 (x2, x3);
            _mm256_storeu_si256 ((__m256i*) (pTempOut + addrOffset), x4);
        }
    }
}

void ldpc_encoder_bg2(int8_t *pDataIn, int8_t *pDataOut, const int16_t *pMatrixNumPerCol,
        const int16_t *pAddr, const int16_t *pShiftMatrix, int16_t zcSize, uint8_t i_LS)
{
    const int16_t *pTempAddr,  *pTempMatrix;
    int8_t *pTempIn,*pTempOut;
    int16_t addrOffset = 0;
    int32_t i = 0;
    __m256i x1,x2,x3,x4,x5,x6,x7,x8,x9;
    CYCLIC_BIT_SHIFT cycle_bit_shift_p = ldpc_select_shift_func(zcSize);

    for(int32_t j = 0; j < BG2_ROW_TOTAL; j++){
        _mm256_storeu_si256 ((__m256i*) (pDataOut + j*PROC_BYTES), _mm256_set1_epi8(0));
    }

    pTempAddr = pAddr;
    pTempMatrix = pShiftMatrix;
    pTempIn = pDataIn;
    pTempOut = pDataOut;

    x1 = _mm256_loadu_si256 ((__m256i*)(pTempIn + i * PROC_BYTES));

    // getting lambdas
    for(int32_t j = 0; j<*(pMatrixNumPerCol + i); j++) {
        x2 = cycle_bit_shift_p(x1, *pTempMatrix++, zcSize);
        addrOffset = (*pTempAddr++)>>1;
        _mm256_storeu_si256 ((__m256i *) (pTempOut + addrOffset), x2);
    }
    i = 1; 
    for(; i<BG2_COL_INF_NUM; i++) {
        x1 = _mm256_loadu_si256 ((__m256i*)(pTempIn + i * PROC_BYTES));
        // can add threading here *********************************************************
        for(int32_t j = 0; j<*(pMatrixNumPerCol + i); j++) {
            addrOffset = (*pTempAddr++)>>1;
            x2 = cycle_bit_shift_p(x1, *pTempMatrix++, zcSize);
            x3 = _mm256_loadu_si256 ((__m256i*) (pTempOut + addrOffset));
            x4 = _mm256_xor_si256 (x2, x3); 
            _mm256_storeu_si256 ((__m256i*) (pTempOut + addrOffset), x4);
        }
    }

    // Row Transform to resolve the small 4x4 parity matrix
    // lambdas
    x1 = _mm256_loadu_si256 ((__m256i*) pTempOut);
    x2 = _mm256_loadu_si256 ((__m256i*) (pTempOut + PROC_BYTES));
    x3 = _mm256_loadu_si256 ((__m256i*) (pTempOut + 2*PROC_BYTES));
    x4 = _mm256_loadu_si256 ((__m256i*) (pTempOut + 3*PROC_BYTES));

    //first 384
    // x5 is p_a1
    x5 = _mm256_xor_si256 (x1, x2);
    x5 = _mm256_xor_si256 (x5, x3);
    x5 = _mm256_xor_si256 (x5, x4);

    // Special case for the circulant
    if ((i_LS == 3) || (i_LS == 7))
        _mm256_storeu_si256 ((__m256i*) pDataOut, x5);
    else {
        x5 = cycle_bit_shift_p(x5, (zcSize-1), zcSize);
        _mm256_storeu_si256 ((__m256i*) pDataOut, x5);
    }

    //second 384
    // x7 is p_a2
    if ((i_LS == 3) || (i_LS == 7))
        x6 = cycle_bit_shift_p(x5, 1, zcSize);
    else
        x6 = x5;

    x7 = _mm256_xor_si256 (x1, x6);
    _mm256_storeu_si256 ((__m256i*) (pDataOut + PROC_BYTES), x7);

    //third 384 - c2(x2)+w2(x7)=w3(x8)
    // p_a3
    x8 = _mm256_xor_si256 (x2, x7);
    _mm256_storeu_si256 ((__m256i*) (pDataOut + 2*PROC_BYTES), x8);

    //fourth 384 - c4(x4)+w1_0(x6)=w4(x9)
    // pa_4
    x9 = _mm256_xor_si256 (x4, x6);
    _mm256_storeu_si256 ((__m256i*) (pDataOut + 3*PROC_BYTES), x9);

    // Rest of parity based on identity matrix
    // p_c's
    for(; i < 4 + BG2_COL_INF_NUM; i++) {
        x1 = _mm256_loadu_si256 ((__m256i*)(pDataOut + (i - BG2_COL_INF_NUM) * PROC_BYTES));
        for(int32_t j = 0; j<*(pMatrixNumPerCol + i); j++) {
            addrOffset = (*pTempAddr++) >> 1;
            x2 = cycle_bit_shift_p(x1, *pTempMatrix++, zcSize);
            x3 = _mm256_loadu_si256 ((__m256i*) (pTempOut + addrOffset));
            x4 = _mm256_xor_si256 (x2, x3);
            _mm256_storeu_si256 ((__m256i*) (pTempOut + addrOffset), x4);
        }
    }
}

LDPC_ENCODER ldpc_select_encoder_func(uint16_t Bg)
{
    if (Bg == 1) 
        return ldpc_encoder_bg1;
    else 
        return ldpc_encoder_bg2;
}