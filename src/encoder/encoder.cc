/**
 * @file encoder.cc
 * @brief Implementations for Agora's AVX2-based LDPC encoder.
 *
 * We need an AVX2-based LDPC encoder because FlexRAN's LDPC encoder requires
 * AVX-512.
 */
#include "encoder.h"

#include "common_typedef_sdk.h"
#include "cyclic_shift.h"
#include "iobuffer.h"

namespace avx2enc {
void LdpcEncoderBg1(int8_t* pDataIn, int8_t* pDataOut,
                    const int16_t* pMatrixNumPerCol, const int16_t* pAddr,
                    const int16_t* pShiftMatrix, int16_t zcSize, uint8_t i_LS) {
  const int16_t* p_temp_addr;
  const int16_t* p_temp_matrix;
  int8_t* p_temp_in;
  int8_t* p_temp_out;
  int16_t addr_offset = 0;
  size_t i = 0;
  __m256i x1;
  __m256i x2;
  __m256i x3;
  __m256i x4;
  __m256i x5;
  __m256i x6;
  __m256i x7;
  __m256i x8;
  __m256i x9;
  CYCLIC_BIT_SHIFT cycle_bit_shift_p = LdpcSelectShiftFunc(zcSize);

  for (size_t j = 0; j < BG1_ROW_TOTAL; j++) {
    _mm256_storeu_si256((__m256i*)(pDataOut + j * kProcBytes),
                        _mm256_set1_epi8(0));
  }

  p_temp_addr = pAddr;
  p_temp_matrix = pShiftMatrix;
  p_temp_in = pDataIn;
  p_temp_out = pDataOut;

  x1 = _mm256_loadu_si256((__m256i*)(p_temp_in + i * kProcBytes));

  // getting lambdas
  for (int32_t j = 0; j < *(pMatrixNumPerCol + i); j++) {
    x2 = cycle_bit_shift_p(x1, *p_temp_matrix++, zcSize);
    addr_offset = (*p_temp_addr++) >> 1;
    _mm256_storeu_si256((__m256i*)(p_temp_out + addr_offset), x2);
  }

  i = 1;
  for (; i < BG1_COL_INF_NUM; i++) {
    x1 = _mm256_loadu_si256((__m256i*)(p_temp_in + i * kProcBytes));
    for (int32_t j = 0; j < *(pMatrixNumPerCol + i); j++) {
      addr_offset = (*p_temp_addr++) >> 1;
      x2 = cycle_bit_shift_p(x1, *p_temp_matrix++, zcSize);
      x3 = _mm256_loadu_si256((__m256i*)(p_temp_out + addr_offset));
      x4 = _mm256_xor_si256(x2, x3);
      _mm256_storeu_si256((__m256i*)(p_temp_out + addr_offset), x4);
    }
  }

  // std::printf("pTempOut: \n");
  // for (int i = 0; i < BG1_ROW_TOTAL * kProcBytes; i++) {
  //     std::printf("%i ", pTempOut[i]);
  // }
  // std::printf("\n");

  // Row Transform to resolve the small 4x4 parity matrix
  // lambdas
  x1 = _mm256_loadu_si256((__m256i*)p_temp_out);
  x2 = _mm256_loadu_si256((__m256i*)(p_temp_out + kProcBytes));
  x3 = _mm256_loadu_si256((__m256i*)(p_temp_out + 2 * kProcBytes));
  x4 = _mm256_loadu_si256((__m256i*)(p_temp_out + 3 * kProcBytes));

  // first 384
  // x5 is p_a1
  x5 = _mm256_xor_si256(x1, x2);
  x5 = _mm256_xor_si256(x5, x3);
  x5 = _mm256_xor_si256(x5, x4);

  // Special case for the circulant
  if (i_LS == 6) {
    x5 = cycle_bit_shift_p(x5, 103, zcSize);
    _mm256_storeu_si256((__m256i*)pDataOut, x5);
  } else {
    _mm256_storeu_si256((__m256i*)pDataOut, x5);
  }

  // second 384
  // x7 is p_a2
  if (i_LS == 6) {
    x6 = x5;
  } else {
    x6 = cycle_bit_shift_p(x5, 1, zcSize);
  }

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
      addr_offset = (*p_temp_addr++) >> 1;
      // addrOffset = *pTempAddr++;
      x2 = cycle_bit_shift_p(x1, *p_temp_matrix++, zcSize);
      x3 = _mm256_loadu_si256((__m256i*)(p_temp_out + addr_offset));
      x4 = _mm256_xor_si256(x2, x3);
      _mm256_storeu_si256((__m256i*)(p_temp_out + addr_offset), x4);
    }
  }
}

void LdpcEncoderBg2(int8_t* pDataIn, int8_t* pDataOut,
                    const int16_t* pMatrixNumPerCol, const int16_t* pAddr,
                    const int16_t* pShiftMatrix, int16_t zcSize, uint8_t i_LS) {
  const int16_t* p_temp_addr;
  const int16_t* p_temp_matrix;
  int8_t* p_temp_in;
  int8_t* p_temp_out;
  int16_t addr_offset = 0;
  size_t i = 0;
  __m256i x1;
  __m256i x2;
  __m256i x3;
  __m256i x4;
  __m256i x5;
  __m256i x6;
  __m256i x7;
  __m256i x8;
  __m256i x9;
  CYCLIC_BIT_SHIFT cycle_bit_shift_p = LdpcSelectShiftFunc(zcSize);

  for (size_t j = 0; j < BG2_ROW_TOTAL; j++) {
    _mm256_storeu_si256((__m256i*)(pDataOut + j * kProcBytes),
                        _mm256_set1_epi8(0));
  }

  p_temp_addr = pAddr;
  p_temp_matrix = pShiftMatrix;
  p_temp_in = pDataIn;
  p_temp_out = pDataOut;

  x1 = _mm256_loadu_si256((__m256i*)(p_temp_in + i * kProcBytes));

  // getting lambdas
  for (int32_t j = 0; j < *(pMatrixNumPerCol + i); j++) {
    x2 = cycle_bit_shift_p(x1, *p_temp_matrix++, zcSize);
    addr_offset = (*p_temp_addr++) >> 1;
    _mm256_storeu_si256((__m256i*)(p_temp_out + addr_offset), x2);
  }
  i = 1;
  for (; i < BG2_COL_INF_NUM; i++) {
    x1 = _mm256_loadu_si256((__m256i*)(p_temp_in + i * kProcBytes));
    // can add threading here
    // *********************************************************
    for (int32_t j = 0; j < *(pMatrixNumPerCol + i); j++) {
      addr_offset = (*p_temp_addr++) >> 1;
      x2 = cycle_bit_shift_p(x1, *p_temp_matrix++, zcSize);
      x3 = _mm256_loadu_si256((__m256i*)(p_temp_out + addr_offset));
      x4 = _mm256_xor_si256(x2, x3);
      _mm256_storeu_si256((__m256i*)(p_temp_out + addr_offset), x4);
    }
  }

  // Row Transform to resolve the small 4x4 parity matrix
  // lambdas
  x1 = _mm256_loadu_si256((__m256i*)p_temp_out);
  x2 = _mm256_loadu_si256((__m256i*)(p_temp_out + kProcBytes));
  x3 = _mm256_loadu_si256((__m256i*)(p_temp_out + 2 * kProcBytes));
  x4 = _mm256_loadu_si256((__m256i*)(p_temp_out + 3 * kProcBytes));

  // first 384
  // x5 is p_a1
  x5 = _mm256_xor_si256(x1, x2);
  x5 = _mm256_xor_si256(x5, x3);
  x5 = _mm256_xor_si256(x5, x4);

  // Special case for the circulant
  if ((i_LS == 3) || (i_LS == 7)) {
    _mm256_storeu_si256((__m256i*)pDataOut, x5);
  } else {
    x5 = cycle_bit_shift_p(x5, (zcSize - 1), zcSize);
    _mm256_storeu_si256((__m256i*)pDataOut, x5);
  }

  // second 384
  // x7 is p_a2
  if ((i_LS == 3) || (i_LS == 7)) {
    x6 = cycle_bit_shift_p(x5, 1, zcSize);
  } else {
    x6 = x5;
  }

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
      addr_offset = (*p_temp_addr++) >> 1;
      x2 = cycle_bit_shift_p(x1, *p_temp_matrix++, zcSize);
      x3 = _mm256_loadu_si256((__m256i*)(p_temp_out + addr_offset));
      x4 = _mm256_xor_si256(x2, x3);
      _mm256_storeu_si256((__m256i*)(p_temp_out + addr_offset), x4);
    }
  }
}

int32_t BblibLdpcEncoder5gnr(
    struct bblib_ldpc_encoder_5gnr_request* request,
    struct bblib_ldpc_encoder_5gnr_response* response) {
  // input -----------------------------------------------------------
  // these values depend on the application
  uint16_t zc = request->Zc;
  if (zc > ZC_MAX) {
    std::fprintf(stderr, "Error: This AVX2 encoder supports only Zc <= %d\n",
                 ZC_MAX);
    throw std::runtime_error("Encoder: This AVX2 encoder supports only Zc");
  }

  int number_codeblocks = request->numberCodeblocks;
  uint16_t bg = request->baseGraph;

  // int nRows = (Bg == 1) ? BG1_ROW_TOTAL : BG2_ROW_TOTAL;
  uint32_t cb_enc_len = request->nRows * zc;
  uint32_t cb_len = (bg == 1) ? zc * BG1_COL_INF_NUM : zc * BG2_COL_INF_NUM;

  int8_t* input[number_codeblocks];
  int8_t* parity[number_codeblocks];
  for (int i = 0; i < number_codeblocks; i++) {
    input[i] = request->input[i];
    parity[i] = response->output[i];
  }

  // i_Ls decides the base matrix entries
  uint8_t i_ls;
  if ((zc % 15) == 0) {
    i_ls = 7;
  } else if ((zc % 13) == 0) {
    i_ls = 6;
  } else if ((zc % 11) == 0) {
    i_ls = 5;
  } else if ((zc % 9) == 0) {
    i_ls = 4;
  } else if ((zc % 7) == 0) {
    i_ls = 3;
  } else if ((zc % 5) == 0) {
    i_ls = 2;
  } else if ((zc % 3) == 0) {
    i_ls = 1;
  } else {
    i_ls = 0;
  }

  const int16_t* p_shift_matrix;
  const int16_t* p_matrix_num_per_col;
  const int16_t* p_addr;
  if (bg == 1) {
    p_shift_matrix = kBg1HShiftMatrix + i_ls * BG1_NONZERO_NUM;
    p_matrix_num_per_col = kBg1MatrixNumPerCol;
    p_addr = kBg1Address;
  } else {
    p_shift_matrix = kBg2HShiftMatrix + i_ls * BG2_NONZERO_NUM;
    p_matrix_num_per_col = kBg2MatrixNumPerCol;
    p_addr = kBg2Address;
  }

  __attribute__((aligned(64)))
  int8_t input_internal_buffer[BG1_COL_TOTAL * avx2enc::kProcBytes] = {0};
  __attribute__((aligned(64)))
  int8_t parity_internal_buffer[BG1_ROW_TOTAL * avx2enc::kProcBytes] = {0};

  avx2enc::LDPC_ADAPTER_P ldpc_adapter_func =
      avx2enc::LdpcSelectAdapterFunc(zc);
  auto ldpc_encoder_func =
      (bg == 1 ? avx2enc::LdpcEncoderBg1 : avx2enc::LdpcEncoderBg2);

  for (int n = 0; n < number_codeblocks; n++) {
    // Scatter Zc-bit chunks of the input into kProcBytes-sized chunks
    // of input_internal_buffer
    ldpc_adapter_func(input[n], input_internal_buffer, zc, cb_len, 1);

    // Encode into parity_internal_buffer
    ldpc_encoder_func(input_internal_buffer, parity_internal_buffer,
                      p_matrix_num_per_col, p_addr, p_shift_matrix, (int16_t)zc,
                      i_ls);

    // Gather parity bits from kProcBytes-sized chunks of
    // parity_internal_buffer
    ldpc_adapter_func(parity[n], parity_internal_buffer, zc, cb_enc_len, 0);
  }

  return 0;
}
}  // namespace avx2enc