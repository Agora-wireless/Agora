#include "modulation.h"

void Print256Epi32(__m256i var) {
  auto* val = reinterpret_cast<int32_t*>(&var);
  std::printf("Numerical: %i %i %i %i %i %i %i %i \n", val[0], val[1], val[2],
              val[3], val[4], val[5], val[6], val[7]);
}

void Print256Epi16(__m256i var) {
  auto* val = reinterpret_cast<int16_t*>(&var);
  std::printf("Numerical: %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i\n",
              val[0], val[1], val[2], val[3], val[4], val[5], val[6], val[7],
              val[8], val[9], val[10], val[11], val[12], val[13], val[14],
              val[15]);
}

void Print256Epi8(__m256i var) {
  auto* val = reinterpret_cast<int8_t*>(&var);
  std::printf(
      "Numerical int8_t: %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i "
      "%i %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i \n",
      val[0], val[1], val[2], val[3], val[4], val[5], val[6], val[7], val[8],
      val[9], val[10], val[11], val[12], val[13], val[14], val[15], val[16],
      val[17], val[18], val[19], val[20], val[21], val[22], val[23], val[24],
      val[25], val[26], val[27], val[28], val[29], val[30], val[31]);
}

void Print128Epi8(__m128i var) {
  auto* val = reinterpret_cast<int8_t*>(&var);
  std::printf(
      "Numerical int8_t: %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i \n",
      val[0], val[1], val[2], val[3], val[4], val[5], val[6], val[7], val[8],
      val[9], val[10], val[11], val[12], val[13], val[14], val[15]);
}

/**
 ***********************************************************************************
 * Initialization functions
 ***********************************************************************************
 */

void InitModulationTable(Table<complex_float>& mod_table, size_t mod_order) {
  if (!mod_table.IsAllocated()) {
    mod_table.Malloc(1, pow(2, kMaxModType),
                     Agora_memory::Alignment_t::kAlign32);
  }
  // mod_table.malloc(pow(2, kMaxModType), 2, 32);
  switch (mod_order) {
    case 4:
      InitQpskTable(mod_table);
      break;
    case 16:
      InitQam16Table(mod_table);
      break;
    case 64:
      InitQam64Table(mod_table);
      break;
    case 256:
      InitQam256Table(mod_table);
      break;
    default: {
      std::printf("Modulation order not supported, use default value 4\n");
      InitQam16Table(mod_table);
    }
  }
}

/**
 * QPSK modulation
 *              Q
 *  01  |  11
 *---------------> I
 *  00  |  10
 */
void InitQpskTable(Table<complex_float>& qpsk_table) {
  float scale = 1 / sqrt(2);
  float mod_qpsk[2] = {-scale, scale};
  for (int i = 0; i < 4; i++) {
    qpsk_table[0][i] = {mod_qpsk[i / 2], mod_qpsk[i % 2]};
  }
}

// /**
//   * 16-QAM modulation
//   *              Q
//   *  0010  0110  |  1110  1010
//   *  0011  0111  |  1111  1011
//   *---------------------------------> I
//   *  0001  0101  |  1101  1001
//   *  0000  0100  |  1100  1000
//   */
// void init_qam16_table(float **qam16_table)
// {
//     qam16_table.malloc(16, 2, 32);
//     float scale = 1/sqrt(10);
//     float mod_16qam[4] = {-3*scale, -1*scale, 3*scale, scale};
//     for (int i = 0; i < 16; i++) {
//         qam16_table[i][0] = mod_16qam[i / 4];
//         qam16_table[i][1] = mod_16qam[i % 4];
//     }
// }

/**
 * 16-QAM modulation
 *              Q
 *  1011  1001  |  0001  0011
 *  1010  1000  |  0000  0010
 *---------------------------------> I
 *  1110  1100  |  0100  0110
 *  1111  1101  |  0101  0111
 */
void InitQam16Table(Table<complex_float>& qam16_table) {
  float scale = 1 / sqrt(10);
  float mod_16qam[4] = {1 * scale, 3 * scale, (-1) * scale, (-3) * scale};
  for (int i = 0; i < 16; i++) {
    /* get bit 2 and 0 */
    int imag_i = (((i >> 2) & 0x1) << 1) + (i & 0x1);
    /* get bit 3 and 1 */
    int real_i = (((i >> 3) & 0x1) << 1) + ((i >> 1) & 0x1);
    qam16_table[0][i] = {mod_16qam[real_i], mod_16qam[imag_i]};
    // std::printf("%d: (%.3f, %.3f)\n", i, qam16_table[i][0].re,
    // qam16_table[i][0].im);
  }
}

/**
 * 64-QAM modulation
 *              Q
 *  101111  101101  100101  101011  |  000111  000101  001101  001111
 *  101110  101100  100100  100110  |  000110  000100  001100  001110
 *  101010  101000  100000  100010  |  000010  000000  001000  001010
 *  101011  101001  100001  100011  |  000011  000001  001001  001011
 *------------------------------------------------------------------------> I
 *  111011  111001  110001  110011  |  010011  010001  011001  011010
 *  111010  111000  110000  110010  |  010010  010000  011000  011011
 *  111110  111100  110100  110110  |  010110  010100  011100  011110
 *  111111  111101  110101  110111  |  010111  010101  011101  011111
 */

void InitQam64Table(Table<complex_float>& qam64_table) {
  float scale = 1 / sqrt(42);
  float mod_64qam[8] = {3 * scale,    1 * scale,    5 * scale,    7 * scale,
                        (-3) * scale, (-1) * scale, (-5) * scale, (-7) * scale};
  for (int i = 0; i < 64; i++) {
    /* get bit 4, 2, 0 */
    int imag_i = (((i >> 4) & 0x1) << 2) + (((i >> 2) & 0x1) << 1) + (i & 0x1);
    /* get bit 5, 3, 1 */
    int real_i =
        (((i >> 5) & 0x1) << 2) + (((i >> 3) & 0x1) << 1) + ((i >> 1) & 0x1);
    qam64_table[0][i] = {mod_64qam[real_i], mod_64qam[imag_i]};
  }
}

/**
 * 256-QAM Modulation
 *  Q
 * 01000000 01000010 01001010 01001000 01101000 01101010 01100010 01100000 | 11100000 11100010 11101010 11101000 11001000 11001010 11000010 11000000 
 * 01000001 01000011 01001011 01001001 01101001 01101011 01100011 01100001 | 11100001 11100011 11101011 11101001 11001001 11001011 11000011 11000001 
 * 01000101 01000111 01001111 01001101 01101101 01101111 01100111 01100101 | 11100101 11100111 11101111 11101101 11001101 11001111 11000111 11000101 
 * 01000100 01000110 01001110 01001100 01101100 01101110 01100110 01100100 | 11100100 11100110 11101110 11101100 11001100 11001110 11000110 11000100 
 * 01010100 01010110 01011110 01011100 01111100 01111110 01110110 01110100 | 11110100 11110110 11111110 11111100 11011100 11011110 11010110 11010100 
 * 01010101 01010111 01011111 01011101 01111101 01111111 01110111 01110101 | 11110101 11110111 11111111 11111101 11011101 11011111 11010111 11010101 
 * 01010001 01010011 01011011 01011001 01111001 01111011 01110011 01110001 | 11110001 11110011 11111011 11111001 11011001 11011011 11010011 11010001 
 * 01010000 01010010 01011010 01011000 01111000 01111010 01110010 01110000 | 11110000 11110010 11111010 11111000 11011000 11011010 11010010 11010000 
 * ------------------------------------------------------------------------------------------------------------------------------------------------- I
 * 00010000 00010010 00011010 00011000 00111000 00111010 00110010 00110000 | 10110000 10110010 10111010 10111000 10011000 10011010 10010010 10010000 
 * 00010001 00010011 00011011 00011001 00111001 00111011 00110011 00110001 | 10110001 10110011 10111011 10111001 10011001 10011011 10010011 10010001 
 * 00010101 00010111 00011111 00011101 00111101 00111111 00110111 00110101 | 10110101 10110111 10111111 10111101 10011101 10011111 10010111 10010101 
 * 00010100 00010110 00011110 00011100 00111100 00111110 00110110 00110100 | 10110100 10110110 10111110 10111100 10011100 10011110 10010110 10010100 
 * 00000100 00000110 00001110 00001100 00101100 00101110 00100110 00100100 | 10100100 10100110 10101110 10101100 10001100 10001110 10000110 10000100 
 * 00000101 00000111 00001111 00001101 00101101 00101111 00100111 00100101 | 10100101 10100111 10101111 10101101 10001101 10001111 10000111 10000101 
 * 00000001 00000011 00001011 00001001 00101001 00101011 00100011 00100001 | 10100001 10100011 10101011 10101001 10001001 10001011 10000011 10000001 
 * 00000000 00000010 00001010 00001000 00101000 00101010 00100010 00100000 | 10100000 10100010 10101010 10101000 10001000 10001010 10000010 10000000
 */
void InitQam256Table(Table<complex_float>& qam256_table) {
  float scale = 1 / sqrt(170);
  uint8_t imag_i;
  uint8_t real_i;
  /**
   * To generate this table, first create a 4 bit gray code. Then, generate a
   * vector of the scales you want for your QAM table. Finally, use the gray
   * code values for each index (0-15) to select the scale value you assign to
   * that index in this array. This means that if you take the index of each
   * scale in this table, and order those indicies by the value of the scale
   * the index refers to, you will have a gray code
   */
  float mod_256qam[16] = {
      (-15) * scale, (-13) * scale, (-9) * scale, (-11) * scale,
      (-1) * scale,  (-3) * scale,  (-7) * scale, (-5) * scale,
      15 * scale,    13 * scale,    9 * scale,    11 * scale,
      1 * scale,     3 * scale,     7 * scale,    5 * scale};
  for (int i = 0; i < 256; i++) {
    // Get bits 6, 4, 2, 0 (and pack into 4 bit integer)
    imag_i = (((i & 0x40) >> 3)) + (((i & 0x10) >> 2)) + (((i & 0x4) >> 1)) +
             (i & 0x1);
    // Get bits 7, 5, 3, 1 (and pack into 4 bit integer)
    real_i = (((i & 0x80) >> 4)) + (((i & 0x20) >> 3)) + (((i & 0x8) >> 2)) +
             (((i & 0x2) >> 1));
    qam256_table[0][i] = {mod_256qam[real_i], mod_256qam[imag_i]};
  }
}

/**
 ***********************************************************************************
 * Modulation functions
 ***********************************************************************************
 */

complex_float ModSingle(int x, Table<complex_float>& mod_table) {
  return mod_table[0][x];
}

complex_float ModSingleUint8(uint8_t x, Table<complex_float>& mod_table) {
  return mod_table[0][x];
}

// TODO: test correctness
void ModSimd(uint8_t* in, complex_float*& out, size_t len,
             Table<complex_float>& mod_table) {
#ifdef __AVX512F__
  for (size_t i = 0; i < len / kSCsPerCacheline; i++) {
    __m512i index = _mm512_setr_epi64(in[0], in[1], in[2], in[3], in[4], in[5],
                                      in[6], in[7]);
    __m512d t = _mm512_i64gather_pd(index, (double*)(mod_table[0]), 8);
    _mm512_store_pd((double*)(out), t);
    in += kSCsPerCacheline;
    out += kSCsPerCacheline;
  }
#else
  size_t half_size = kSCsPerCacheline / 2;
  for (size_t i = 0; i < len / kSCsPerCacheline; i++) {
    __m256i index = _mm256_setr_epi64x(in[0], in[1], in[2], in[3]);
    __m256d t = _mm256_i64gather_pd((double*)(mod_table[0]), index, 8);
    _mm256_store_pd((double*)(out), t);
    in += half_size;
    out += half_size;
    index = _mm256_setr_epi64x(in[4], in[5], in[6], in[7]);
    t = _mm256_i64gather_pd((double*)(mod_table[0]), index, 8);
    _mm256_store_pd((double*)(out), t);
    in += half_size;
    out += half_size;
  }
#endif

  size_t remainder = len % kSCsPerCacheline;
  for (size_t i = 0; i < remainder; i++) {
    out[i] = ModSingleUint8(in[i], mod_table);
  }
}

/**
 ***********************************************************************************
 * Demodulation functions
 ***********************************************************************************
 */

/**
 * QPSK modulation
 *              Q
 *  01  |  11
 *---------------> I
 *  00  |  10
 */
void DemodQpskHardLoop(const float* vec_in, uint8_t* vec_out, int num) {
  for (int i = 0; i < num; i++) {
    float real_val = *(vec_in + i * 2);
    float imag_val = *(vec_in + i * 2 + 1);

    *(vec_out + i) = 0;
    if (real_val >= 0) {
      *(vec_out + i) |= 1UL << 1;
    }
    if (imag_val >= 0) {
      *(vec_out + i) |= 1UL;
    }
  }
}

// /**
//   * 16-QAM demodulation
//   *              Q
//   *  0010  0110  |  1110  1010
//   *  0011  0111  |  1111  1011
//   *---------------------------------> I
//   *  0001  0101  |  1101  1001
//   *  0000  0100  |  1100  1000
//   */
// void demod_16qam_hard_loop(float *vec_in, uint8_t *vec_out, int num)
// {
//     float float_val = QAM16_THRESHOLD;

//     for (int i = 0; i < num; i++) {
//         float real_val = *(vec_in + i * 2);
//         float imag_val = *(vec_in + i * 2 + 1);

//         *(vec_out + i) = 0;
//         if (real_val > 0)
//             *(vec_out + i) |= 1UL << 3;
//             //*(vec_out + i) += 8;
//         if (std::abs(real_val) < float_val)
//             *(vec_out + i) |= 1UL << 2;
//             //*(vec_out + i) += 4;
//         if (imag_val > 0)
//             *(vec_out + i) |= 1UL << 1;
//             //*(vec_out + i) += 2;
//         if (std::abs(imag_val) < float_val)
//             *(vec_out + i) |= 1UL ;
//             //*(vec_out + i) += 1;
//     }
// }

/**
 * 16-QAM demodulation
 *              Q
 *  1011  1001  |  0001  0011
 *  1010  1000  |  0000  0010
 *---------------------------------> I
 *  1110  1100  |  0100  0110
 *  1111  1101  |  0101  0111
 */

void Demod16qamHardLoop(const float* vec_in, uint8_t* vec_out, int num) {
  float float_val = QAM16_THRESHOLD;

  for (int i = 0; i < num; i++) {
    float real_val = *(vec_in + i * 2);
    float imag_val = *(vec_in + i * 2 + 1);

    *(vec_out + i) = 0;
    if (real_val <= 0) {
      *(vec_out + i) |= 1UL << 3;
    }
    if (std::abs(real_val) > float_val) {
      *(vec_out + i) |= 1UL << 1;
    }
    if (imag_val <= 0) {
      *(vec_out + i) |= 1UL << 2;
    }
    if (std::abs(imag_val) > float_val) {
      *(vec_out + i) |= 1UL;
    }
  }
}

void Demod16qamHardSse(float* vec_in, uint8_t* vec_out, int num) {
  float* symbols_ptr = vec_in;
  auto* result_ptr = reinterpret_cast<__m64*>(vec_out);
  __m128 symbol1;
  __m128 symbol2;
  __m128 symbol3;
  __m128 symbol4;
  __m128i symbol_i1;
  __m128i symbol_i2;
  __m128i symbol_i3;
  __m128i symbol_i4;
  __m128i symbol_12;
  __m128i symbol_34;
  __m128i symbol_abs_1;
  __m128i symbol_abs_2;
  __m128i symbol_gt_0_1;
  __m128i symbol_gt_threshold_1;
  __m128i symbol_gt_0_2;
  __m128i symbol_gt_threshold_2;
  __m128i bit0_1;
  __m128i bit1_1;
  __m128i bit2_1;
  __m128i bit3_1;
  __m128i bit0_2;
  __m128i bit1_2;
  __m128i bit2_2;
  __m128i bit3_2;
  __m128i bit0;
  __m128i bit1;
  __m128i bit2;
  __m128i bit3;
  __m128i result;
  __m128 scale_v = _mm_set1_ps(-SCALE_BYTE_CONV_QAM16);
  __m128i vec_zero = _mm_set1_epi16(0);
  __m128i vec_threshold = _mm_set1_epi16(2 * SCALE_BYTE_CONV_QAM16 / sqrt(10));
  __m128i vec_true_mask = _mm_set1_epi16(0x1);

  __m128i shuffle_real_1 = _mm_set_epi8(0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                                        0xff, 0xff, 13, 12, 9, 8, 5, 4, 1, 0);
  __m128i shuffle_real_2 = _mm_set_epi8(13, 12, 9, 8, 5, 4, 1, 0, 0xff, 0xff,
                                        0xff, 0xff, 0xff, 0xff, 0xff, 0xff);

  __m128i shuffle_imag_1 = _mm_set_epi8(0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                                        0xff, 0xff, 15, 14, 11, 10, 7, 6, 3, 2);
  __m128i shuffle_imag_2 = _mm_set_epi8(15, 14, 11, 10, 7, 6, 3, 2, 0xff, 0xff,
                                        0xff, 0xff, 0xff, 0xff, 0xff, 0xff);

  __m128i shuffle_16_to_8 = _mm_set_epi8(0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                                         0xff, 0xff, 14, 12, 10, 8, 6, 4, 2, 0);

  for (int i = 0; i < num / 8; i++) {
    symbol1 = _mm_load_ps(symbols_ptr);
    symbols_ptr += 4;
    symbol2 = _mm_load_ps(symbols_ptr);
    symbols_ptr += 4;
    symbol3 = _mm_load_ps(symbols_ptr);
    symbols_ptr += 4;
    symbol4 = _mm_load_ps(symbols_ptr);
    symbols_ptr += 4;

    symbol_i1 = _mm_cvtps_epi32(_mm_mul_ps(symbol1, scale_v));
    symbol_i2 = _mm_cvtps_epi32(_mm_mul_ps(symbol2, scale_v));
    symbol_i3 = _mm_cvtps_epi32(_mm_mul_ps(symbol3, scale_v));
    symbol_i4 = _mm_cvtps_epi32(_mm_mul_ps(symbol4, scale_v));
    symbol_12 = _mm_packs_epi32(symbol_i1, symbol_i2);
    symbol_34 = _mm_packs_epi32(symbol_i3, symbol_i4);

    symbol_abs_1 = _mm_abs_epi16(symbol_12);
    symbol_gt_0_1 = _mm_cmpgt_epi16(symbol_12, vec_zero);
    symbol_gt_threshold_1 = _mm_cmpgt_epi16(symbol_abs_1, vec_threshold);
    symbol_gt_0_1 = _mm_and_si128(symbol_gt_0_1, vec_true_mask);
    symbol_gt_threshold_1 = _mm_and_si128(symbol_gt_threshold_1, vec_true_mask);

    bit3_1 = _mm_shuffle_epi8(symbol_gt_0_1, shuffle_real_1);
    bit2_1 = _mm_shuffle_epi8(symbol_gt_0_1, shuffle_imag_1);
    bit1_1 = _mm_shuffle_epi8(symbol_gt_threshold_1, shuffle_real_1);
    bit0_1 = _mm_shuffle_epi8(symbol_gt_threshold_1, shuffle_imag_1);

    symbol_abs_2 = _mm_abs_epi16(symbol_34);
    symbol_gt_0_2 = _mm_cmpgt_epi16(symbol_34, vec_zero);
    symbol_gt_threshold_2 = _mm_cmpgt_epi16(symbol_abs_2, vec_threshold);
    symbol_gt_0_2 = _mm_and_si128(symbol_gt_0_2, vec_true_mask);
    symbol_gt_threshold_2 = _mm_and_si128(symbol_gt_threshold_2, vec_true_mask);

    bit3_2 = _mm_shuffle_epi8(symbol_gt_0_2, shuffle_real_2);
    bit2_2 = _mm_shuffle_epi8(symbol_gt_0_2, shuffle_imag_2);
    bit1_2 = _mm_shuffle_epi8(symbol_gt_threshold_2, shuffle_real_2);
    bit0_2 = _mm_shuffle_epi8(symbol_gt_threshold_2, shuffle_imag_2);

    bit3 = _mm_or_si128(bit3_1, bit3_2);
    bit2 = _mm_or_si128(bit2_1, bit2_2);
    bit1 = _mm_or_si128(bit1_1, bit1_2);
    bit0 = _mm_or_si128(bit0_1, bit0_2);

    bit1 = _mm_slli_epi16(bit1, 1);
    bit2 = _mm_slli_epi16(bit2, 2);
    bit3 = _mm_slli_epi16(bit3, 3);

    result = _mm_add_epi16(bit0, bit1);
    result = _mm_add_epi16(result, bit2);
    result = _mm_add_epi16(result, bit3);

    result = _mm_shuffle_epi8(result, shuffle_16_to_8);
    _mm_storel_pi(result_ptr, (__m128)result);
    result_ptr++;
  }
  // Demodulate last symbols
  for (int i = 8 * (num / 8); i < num; i++) {
    float real_val = *(vec_in + i * 2);
    float imag_val = *(vec_in + i * 2 + 1);

    *(vec_out + i) = 0;
    if (real_val <= 0) {
      *(vec_out + i) |= 1UL << 3;
    }
    if (std::abs(real_val) > QAM16_THRESHOLD) {
      *(vec_out + i) |= 1UL << 1;
    }
    if (imag_val <= 0) {
      *(vec_out + i) |= 1UL << 2;
    }
    if (std::abs(imag_val) > QAM16_THRESHOLD) {
      *(vec_out + i) |= 1UL;
    }
  }
}

void Demod16qamHardAvx2(float* vec_in, uint8_t* vec_out, int num) {
  float* symbols_ptr = vec_in;
  auto* result_ptr = reinterpret_cast<__m128i*>(vec_out);
  __m256 symbol1;
  __m256 symbol2;
  __m256 symbol3;
  __m256 symbol4;
  __m256i symbol_i1;
  __m256i symbol_i2;
  __m256i symbol_i3;
  __m256i symbol_i4;
  __m256i symbol_12;
  __m256i symbol_34;
  __m256i symbol_abs_1;
  __m256i symbol_abs_2;
  __m256i symbol_gt_0_1;
  __m256i symbol_gt_threshold_1;
  __m256i symbol_gt_0_2;
  __m256i symbol_gt_threshold_2;
  __m256i bit0_1;
  __m256i bit1_1;
  __m256i bit2_1;
  __m256i bit3_1;
  __m256i bit0_2;
  __m256i bit1_2;
  __m256i bit2_2;
  __m256i bit3_2;
  __m256i bit0;
  __m256i bit1;
  __m256i bit2;
  __m256i bit3;
  __m256i result;
  __m256 scale_v = _mm256_set1_ps(-SCALE_BYTE_CONV_QAM16);
  __m256i vec_zero = _mm256_set1_epi16(0);
  __m256i vec_threshold =
      _mm256_set1_epi16(2 * SCALE_BYTE_CONV_QAM16 / sqrt(10));
  __m256i vec_true_mask = _mm256_set1_epi16(0x1);

  __m256i shuffle_real = _mm256_set_epi8(
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 13, 12, 9, 8, 5, 4, 1, 0,
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 13, 12, 9, 8, 5, 4, 1, 0);

  __m256i shuffle_imag =
      _mm256_set_epi8(0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 15, 14,
                      11, 10, 7, 6, 3, 2, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                      0xff, 0xff, 15, 14, 11, 10, 7, 6, 3, 2);

  __m256i shuffle_16_to_8 =
      _mm256_set_epi8(0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 14, 12,
                      10, 8, 6, 4, 2, 0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                      0xff, 0xff, 14, 12, 10, 8, 6, 4, 2, 0);

  for (int i = 0; i < num / 16; i++) {
    symbol1 = _mm256_load_ps(symbols_ptr);
    symbols_ptr += 8;
    symbol2 = _mm256_load_ps(symbols_ptr);
    symbols_ptr += 8;
    symbol3 = _mm256_load_ps(symbols_ptr);
    symbols_ptr += 8;
    symbol4 = _mm256_load_ps(symbols_ptr);
    symbols_ptr += 8;

    symbol_i1 = _mm256_cvtps_epi32(_mm256_mul_ps(symbol1, scale_v));
    symbol_i2 = _mm256_cvtps_epi32(_mm256_mul_ps(symbol2, scale_v));
    symbol_i3 = _mm256_cvtps_epi32(_mm256_mul_ps(symbol3, scale_v));
    symbol_i4 = _mm256_cvtps_epi32(_mm256_mul_ps(symbol4, scale_v));
    symbol_12 = _mm256_packs_epi32(symbol_i1, symbol_i2);
    symbol_12 = _mm256_permute4x64_epi64(symbol_12, 0xd8);
    symbol_34 = _mm256_packs_epi32(symbol_i3, symbol_i4);
    symbol_34 = _mm256_permute4x64_epi64(symbol_34, 0xd8);

    symbol_abs_1 = _mm256_abs_epi16(symbol_12);
    symbol_gt_0_1 = _mm256_cmpgt_epi16(symbol_12, vec_zero);
    symbol_gt_threshold_1 = _mm256_cmpgt_epi16(symbol_abs_1, vec_threshold);
    symbol_gt_0_1 = _mm256_and_si256(symbol_gt_0_1, vec_true_mask);
    symbol_gt_threshold_1 =
        _mm256_and_si256(symbol_gt_threshold_1, vec_true_mask);

    bit3_1 = _mm256_shuffle_epi8(symbol_gt_0_1, shuffle_real);
    bit3_1 = _mm256_permute4x64_epi64(bit3_1, 0xd8);
    bit2_1 = _mm256_shuffle_epi8(symbol_gt_0_1, shuffle_imag);
    bit2_1 = _mm256_permute4x64_epi64(bit2_1, 0xd8);

    bit1_1 = _mm256_shuffle_epi8(symbol_gt_threshold_1, shuffle_real);
    bit1_1 = _mm256_permute4x64_epi64(bit1_1, 0xd8);
    bit0_1 = _mm256_shuffle_epi8(symbol_gt_threshold_1, shuffle_imag);
    bit0_1 = _mm256_permute4x64_epi64(bit0_1, 0xd8);

    symbol_abs_2 = _mm256_abs_epi16(symbol_34);
    symbol_gt_0_2 = _mm256_cmpgt_epi16(symbol_34, vec_zero);
    symbol_gt_threshold_2 = _mm256_cmpgt_epi16(symbol_abs_2, vec_threshold);
    symbol_gt_0_2 = _mm256_and_si256(symbol_gt_0_2, vec_true_mask);
    symbol_gt_threshold_2 =
        _mm256_and_si256(symbol_gt_threshold_2, vec_true_mask);

    bit3_2 = _mm256_shuffle_epi8(symbol_gt_0_2, shuffle_real);
    bit3_2 = _mm256_permute4x64_epi64(bit3_2, 0xd8);
    bit2_2 = _mm256_shuffle_epi8(symbol_gt_0_2, shuffle_imag);
    bit2_2 = _mm256_permute4x64_epi64(bit2_2, 0xd8);

    bit1_2 = _mm256_shuffle_epi8(symbol_gt_threshold_2, shuffle_real);
    bit1_2 = _mm256_permute4x64_epi64(bit1_2, 0xd8);
    bit0_2 = _mm256_shuffle_epi8(symbol_gt_threshold_2, shuffle_imag);
    bit0_2 = _mm256_permute4x64_epi64(bit0_2, 0xd8);

    bit3 = _mm256_permute2x128_si256(bit3_1, bit3_2, 0x20);
    bit2 = _mm256_permute2x128_si256(bit2_1, bit2_2, 0x20);
    bit1 = _mm256_permute2x128_si256(bit1_1, bit1_2, 0x20);
    bit0 = _mm256_permute2x128_si256(bit0_1, bit0_2, 0x20);

    bit1 = _mm256_slli_epi16(bit1, 1);
    bit2 = _mm256_slli_epi16(bit2, 2);
    bit3 = _mm256_slli_epi16(bit3, 3);

    result = _mm256_add_epi16(bit0, bit1);
    result = _mm256_add_epi16(result, bit2);
    result = _mm256_add_epi16(result, bit3);

    result = _mm256_shuffle_epi8(result, shuffle_16_to_8);
    result = _mm256_permute4x64_epi64(result, 0xd8);
    _mm_storeu_si128(result_ptr, _mm256_extracti128_si256(result, 0));
    result_ptr++;
  }
  // Demodulate last symbols
  int next_start = 16 * (num / 16);
  Demod16qamHardSse(vec_in + 2 * next_start, vec_out + next_start,
                    num - next_start);
}

void Demod16qamSoftAvx2(float* vec_in, int8_t* llr, int num) {
  float* symbols_ptr = vec_in;
  auto* result_ptr = reinterpret_cast<__m256i*>(llr);
  __m256 symbol1;
  __m256 symbol2;
  __m256 symbol3;
  __m256 symbol4;
  __m256i symbol_i1;
  __m256i symbol_i2;
  __m256i symbol_i3;
  __m256i symbol_i4;
  __m256i symbol_i;
  __m256i symbol_abs;
  __m256i symbol_12;
  __m256i symbol_34;
  __m256i offset = _mm256_set1_epi8(2 * SCALE_BYTE_CONV_QAM16 / sqrt(10));
  __m256i result1n;
  __m256i result1a;
  __m256i result2n;
  __m256i result2a;
  __m256i result1na;
  __m256i result2na;
  __m256 scale_v = _mm256_set1_ps(SCALE_BYTE_CONV_QAM16);

  __m256i shuffle_negated_1 = _mm256_set_epi8(
      0xff, 0xff, 7, 6, 0xff, 0xff, 5, 4, 0xff, 0xff, 3, 2, 0xff, 0xff, 1, 0,
      0xff, 0xff, 7, 6, 0xff, 0xff, 5, 4, 0xff, 0xff, 3, 2, 0xff, 0xff, 1, 0);
  __m256i shuffle_abs_1 = _mm256_set_epi8(
      7, 6, 0xff, 0xff, 5, 4, 0xff, 0xff, 3, 2, 0xff, 0xff, 1, 0, 0xff, 0xff, 7,
      6, 0xff, 0xff, 5, 4, 0xff, 0xff, 3, 2, 0xff, 0xff, 1, 0, 0xff, 0xff);

  __m256i shuffle_negated_2 =
      _mm256_set_epi8(0xff, 0xff, 15, 14, 0xff, 0xff, 13, 12, 0xff, 0xff, 11,
                      10, 0xff, 0xff, 9, 8, 0xff, 0xff, 15, 14, 0xff, 0xff, 13,
                      12, 0xff, 0xff, 11, 10, 0xff, 0xff, 9, 8);
  __m256i shuffle_abs_2 =
      _mm256_set_epi8(15, 14, 0xff, 0xff, 13, 12, 0xff, 0xff, 11, 10, 0xff,
                      0xff, 9, 8, 0xff, 0xff, 15, 14, 0xff, 0xff, 13, 12, 0xff,
                      0xff, 11, 10, 0xff, 0xff, 9, 8, 0xff, 0xff);

  for (int i = 0; i < num / 16; i++) {
    symbol1 = _mm256_load_ps(symbols_ptr);
    symbols_ptr += 8;
    symbol2 = _mm256_load_ps(symbols_ptr);
    symbols_ptr += 8;
    symbol3 = _mm256_load_ps(symbols_ptr);
    symbols_ptr += 8;
    symbol4 = _mm256_load_ps(symbols_ptr);
    symbols_ptr += 8;
    symbol_i1 = _mm256_cvtps_epi32(_mm256_mul_ps(symbol1, scale_v));
    symbol_i2 = _mm256_cvtps_epi32(_mm256_mul_ps(symbol2, scale_v));
    symbol_i3 = _mm256_cvtps_epi32(_mm256_mul_ps(symbol3, scale_v));
    symbol_i4 = _mm256_cvtps_epi32(_mm256_mul_ps(symbol4, scale_v));
    symbol_12 = _mm256_packs_epi32(symbol_i1, symbol_i2);
    symbol_12 = _mm256_permute4x64_epi64(symbol_12, 0xd8);
    symbol_34 = _mm256_packs_epi32(symbol_i3, symbol_i4);
    symbol_34 = _mm256_permute4x64_epi64(symbol_34, 0xd8);
    symbol_i = _mm256_packs_epi16(symbol_12, symbol_34);
    symbol_i = _mm256_permute4x64_epi64(symbol_i, 0xd8);

    symbol_abs = _mm256_abs_epi8(symbol_i);
    symbol_abs = _mm256_sub_epi8(offset, symbol_abs);

    result1n = _mm256_shuffle_epi8(symbol_i, shuffle_negated_1);
    result1a = _mm256_shuffle_epi8(symbol_abs, shuffle_abs_1);

    result2n = _mm256_shuffle_epi8(symbol_i, shuffle_negated_2);
    result2a = _mm256_shuffle_epi8(symbol_abs, shuffle_abs_2);

    result1na = _mm256_or_si256(result1n, result1a);
    result2na = _mm256_or_si256(result2n, result2a);

    _mm256_store_si256(result_ptr,
                       _mm256_permute2x128_si256(result1na, result2na, 0x20));
    result_ptr++;
    _mm256_store_si256(result_ptr,
                       _mm256_permute2x128_si256(result1na, result2na, 0x31));
    result_ptr++;
  }
  // Demodulate last symbols
  int next_start = 16 * (num / 16);
  Demod16qamSoftSse(vec_in + 2 * next_start, llr + next_start * 4,
                    num - next_start);
}

/**
 * 64-QAM modulation
 *              Q
 *  101111  101101  100101  101011  |  000111  000101  001101  001111
 *  101110  101100  100100  100110  |  000110  000100  001100  001110
 *  101010  101000  100000  100010  |  000010  000000  001000  001010
 *  101011  101001  100001  100011  |  000011  000001  001001  001011
 *------------------------------------------------------------------------> I
 *  111011  111001  110001  110011  |  010011  010001  011001  011010
 *  111010  111000  110000  110010  |  010010  010000  011000  011011
 *  111110  111100  110100  110110  |  010110  010100  011100  011110
 *  111111  111101  110101  110111  |  010111  010101  011101  011111
 */
void Demod64qamHardLoop(const float* vec_in, uint8_t* vec_out, int num) {
  for (int i = 0; i < num; i++) {
    float real_val = *(vec_in + i * 2);
    float imag_val = *(vec_in + i * 2 + 1);

    *(vec_out + i) = 0;

    if (real_val <= 0) {
      *(vec_out + i) |= 1UL << 5;
    }
    if (std::abs(real_val) > QAM64_THRESHOLD_3) {
      *(vec_out + i) |= 1UL << 3;
      *(vec_out + i) |= 1UL << 1;
    } else if (std::abs(real_val) > QAM64_THRESHOLD_2) {
      *(vec_out + i) |= 1UL << 3;
    } else if (std::abs(real_val) <= QAM64_THRESHOLD_1) {
      *(vec_out + i) |= 1UL << 1;
    }

    if (imag_val <= 0) {
      *(vec_out + i) |= 1UL << 4;
    }
    if (std::abs(imag_val) > QAM64_THRESHOLD_3) {
      *(vec_out + i) |= 1UL << 2;
      *(vec_out + i) |= 1UL;
    } else if (std::abs(imag_val) > QAM64_THRESHOLD_2) {
      *(vec_out + i) |= 1UL << 2;
    } else if (std::abs(imag_val) <= QAM64_THRESHOLD_1) {
      *(vec_out + i) |= 1UL;
    }
  }
}

void Demod64qamHardSse(float* vec_in, uint8_t* vec_out, int num) {
  float* symbols_ptr = vec_in;
  auto* result_ptr = reinterpret_cast<__m64*>(vec_out);
  __m128 symbol1;
  __m128 symbol2;
  __m128 symbol3;
  __m128 symbol4;
  __m128i symbol_i1;
  __m128i symbol_i2;
  __m128i symbol_i3;
  __m128i symbol_i4;
  __m128i symbol_12;
  __m128i symbol_34;
  __m128i symbol_abs_1;
  __m128i symbol_abs_2;
  __m128i symbol_gt_0_1;
  __m128i symbol_gt_0_2;
  __m128i symbol_lt_threshold1_1;
  __m128i symbol_lt_threshold1_2;
  __m128i symbol_gt_threshold2_1;
  __m128i symbol_gt_threshold2_2;
  __m128i symbol_gt_threshold3_1;
  __m128i symbol_gt_threshold3_2;
  __m128i bit01_1;
  __m128i bit23_1;
  __m128i bit45_1;
  __m128i bit01_2;
  __m128i bit23_2;
  __m128i bit45_2;
  __m128i bit0_1;
  __m128i bit1_1;
  __m128i bit2_1;
  __m128i bit3_1;
  __m128i bit4_1;
  __m128i bit5_1;
  __m128i bit0_2;
  __m128i bit1_2;
  __m128i bit2_2;
  __m128i bit3_2;
  __m128i bit4_2;
  __m128i bit5_2;
  __m128i bit0;
  __m128i bit1;
  __m128i bit2;
  __m128i bit3;
  __m128i bit4;
  __m128i bit5;
  __m128i result;
  __m128 scale_v = _mm_set1_ps(-SCALE_BYTE_CONV_QAM16);
  __m128i vec_zero = _mm_set1_epi16(0);
  __m128i offset1 = _mm_set1_epi16(2 * SCALE_BYTE_CONV_QAM64 / sqrt(42));
  __m128i offset2 = _mm_set1_epi16(4 * SCALE_BYTE_CONV_QAM64 / sqrt(42));
  __m128i offset3 = _mm_set1_epi16(6 * SCALE_BYTE_CONV_QAM64 / sqrt(42));
  __m128i vec_true_mask = _mm_set1_epi16(0x1);

  __m128i shuffle_real_1 = _mm_set_epi8(0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                                        0xff, 0xff, 13, 12, 9, 8, 5, 4, 1, 0);
  __m128i shuffle_real_2 = _mm_set_epi8(13, 12, 9, 8, 5, 4, 1, 0, 0xff, 0xff,
                                        0xff, 0xff, 0xff, 0xff, 0xff, 0xff);

  __m128i shuffle_imag_1 = _mm_set_epi8(0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                                        0xff, 0xff, 15, 14, 11, 10, 7, 6, 3, 2);
  __m128i shuffle_imag_2 = _mm_set_epi8(15, 14, 11, 10, 7, 6, 3, 2, 0xff, 0xff,
                                        0xff, 0xff, 0xff, 0xff, 0xff, 0xff);

  __m128i shuffle_16_to_8 = _mm_set_epi8(0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                                         0xff, 0xff, 14, 12, 10, 8, 6, 4, 2, 0);

  for (int i = 0; i < num / 8; i++) {
    symbol1 = _mm_load_ps(symbols_ptr);
    symbols_ptr += 4;
    symbol2 = _mm_load_ps(symbols_ptr);
    symbols_ptr += 4;
    symbol3 = _mm_load_ps(symbols_ptr);
    symbols_ptr += 4;
    symbol4 = _mm_load_ps(symbols_ptr);
    symbols_ptr += 4;

    symbol_i1 = _mm_cvtps_epi32(_mm_mul_ps(symbol1, scale_v));
    symbol_i2 = _mm_cvtps_epi32(_mm_mul_ps(symbol2, scale_v));
    symbol_i3 = _mm_cvtps_epi32(_mm_mul_ps(symbol3, scale_v));
    symbol_i4 = _mm_cvtps_epi32(_mm_mul_ps(symbol4, scale_v));
    symbol_12 = _mm_packs_epi32(symbol_i1, symbol_i2);
    symbol_34 = _mm_packs_epi32(symbol_i3, symbol_i4);

    symbol_abs_1 = _mm_abs_epi16(symbol_12);
    symbol_gt_0_1 = _mm_cmpgt_epi16(symbol_12, vec_zero);
    symbol_lt_threshold1_1 = _mm_cmpgt_epi16(offset1, symbol_abs_1);
    symbol_gt_threshold2_1 = _mm_cmpgt_epi16(symbol_abs_1, offset2);
    symbol_gt_threshold3_1 = _mm_cmpgt_epi16(symbol_abs_1, offset3);
    symbol_gt_0_1 = _mm_and_si128(symbol_gt_0_1, vec_true_mask);

    symbol_lt_threshold1_1 =
        _mm_and_si128(symbol_lt_threshold1_1, vec_true_mask);
    symbol_gt_threshold2_1 =
        _mm_and_si128(symbol_gt_threshold2_1, vec_true_mask);
    symbol_gt_threshold3_1 =
        _mm_and_si128(symbol_gt_threshold3_1, vec_true_mask);
    bit01_1 = _mm_or_si128(symbol_lt_threshold1_1, symbol_gt_threshold3_1);
    bit23_1 = symbol_gt_threshold2_1;
    bit45_1 = symbol_gt_0_1;

    bit5_1 = _mm_shuffle_epi8(bit45_1, shuffle_real_1);
    bit4_1 = _mm_shuffle_epi8(bit45_1, shuffle_imag_1);
    bit3_1 = _mm_shuffle_epi8(bit23_1, shuffle_real_1);
    bit2_1 = _mm_shuffle_epi8(bit23_1, shuffle_imag_1);
    bit1_1 = _mm_shuffle_epi8(bit01_1, shuffle_real_1);
    bit0_1 = _mm_shuffle_epi8(bit01_1, shuffle_imag_1);

    symbol_abs_2 = _mm_abs_epi16(symbol_34);
    symbol_gt_0_2 = _mm_cmpgt_epi16(symbol_34, vec_zero);
    symbol_lt_threshold1_2 = _mm_cmpgt_epi16(offset1, symbol_abs_2);
    symbol_gt_threshold2_2 = _mm_cmpgt_epi16(symbol_abs_2, offset2);
    symbol_gt_threshold3_2 = _mm_cmpgt_epi16(symbol_abs_2, offset3);
    symbol_gt_0_2 = _mm_and_si128(symbol_gt_0_2, vec_true_mask);

    symbol_lt_threshold1_2 =
        _mm_and_si128(symbol_lt_threshold1_2, vec_true_mask);
    symbol_gt_threshold2_2 =
        _mm_and_si128(symbol_gt_threshold2_2, vec_true_mask);
    symbol_gt_threshold3_2 =
        _mm_and_si128(symbol_gt_threshold3_2, vec_true_mask);
    bit01_2 = _mm_or_si128(symbol_lt_threshold1_2, symbol_gt_threshold3_2);
    bit23_2 = symbol_gt_threshold2_2;
    bit45_2 = symbol_gt_0_2;

    bit5_2 = _mm_shuffle_epi8(bit45_2, shuffle_real_2);
    bit4_2 = _mm_shuffle_epi8(bit45_2, shuffle_imag_2);
    bit3_2 = _mm_shuffle_epi8(bit23_2, shuffle_real_2);
    bit2_2 = _mm_shuffle_epi8(bit23_2, shuffle_imag_2);
    bit1_2 = _mm_shuffle_epi8(bit01_2, shuffle_real_2);
    bit0_2 = _mm_shuffle_epi8(bit01_2, shuffle_imag_2);

    bit5 = _mm_or_si128(bit5_1, bit5_2);
    bit4 = _mm_or_si128(bit4_1, bit4_2);
    bit3 = _mm_or_si128(bit3_1, bit3_2);
    bit2 = _mm_or_si128(bit2_1, bit2_2);
    bit1 = _mm_or_si128(bit1_1, bit1_2);
    bit0 = _mm_or_si128(bit0_1, bit0_2);

    bit1 = _mm_slli_epi16(bit1, 1);
    bit2 = _mm_slli_epi16(bit2, 2);
    bit3 = _mm_slli_epi16(bit3, 3);
    bit4 = _mm_slli_epi16(bit4, 4);
    bit5 = _mm_slli_epi16(bit5, 5);

    result = _mm_add_epi16(bit0, bit1);
    result = _mm_add_epi16(result, bit2);
    result = _mm_add_epi16(result, bit3);
    result = _mm_add_epi16(result, bit4);
    result = _mm_add_epi16(result, bit5);

    result = _mm_shuffle_epi8(result, shuffle_16_to_8);
    _mm_storel_pi(result_ptr, (__m128)result);
    result_ptr++;
  }
  // Demodulate last symbols
  for (int i = 8 * (num / 8); i < num; i++) {
    float real_val = *(vec_in + i * 2);
    float imag_val = *(vec_in + i * 2 + 1);

    *(vec_out + i) = 0;

    if (real_val <= 0) {
      *(vec_out + i) |= 1UL << 5;
    }
    if (std::abs(real_val) > QAM64_THRESHOLD_3) {
      *(vec_out + i) |= 1UL << 3;
      *(vec_out + i) |= 1UL << 1;
    } else if (std::abs(real_val) > QAM64_THRESHOLD_2) {
      *(vec_out + i) |= 1UL << 3;
    } else if (std::abs(real_val) <= QAM64_THRESHOLD_1) {
      *(vec_out + i) |= 1UL << 1;
    }

    if (imag_val <= 0) {
      *(vec_out + i) |= 1UL << 4;
    }
    if (std::abs(imag_val) > QAM64_THRESHOLD_3) {
      *(vec_out + i) |= 1UL << 2;
      *(vec_out + i) |= 1UL;
    } else if (std::abs(imag_val) > QAM64_THRESHOLD_2) {
      *(vec_out + i) |= 1UL << 2;
    } else if (std::abs(imag_val) <= QAM64_THRESHOLD_1) {
      *(vec_out + i) |= 1UL;
    }
  }
}

void Demod64qamHardAvx2(float* vec_in, uint8_t* vec_out, int num) {
  float* symbols_ptr = vec_in;
  auto* result_ptr = reinterpret_cast<__m128i*>(vec_out);
  __m256 symbol1;
  __m256 symbol2;
  __m256 symbol3;
  __m256 symbol4;
  __m256i symbol_i1;
  __m256i symbol_i2;
  __m256i symbol_i3;
  __m256i symbol_i4;
  __m256i symbol_12;
  __m256i symbol_34;
  __m256i symbol_abs_1;
  __m256i symbol_abs_2;
  __m256i symbol_gt_0_1;
  __m256i symbol_gt_0_2;
  __m256i symbol_lt_threshold1_1;
  __m256i symbol_lt_threshold1_2;
  __m256i symbol_gt_threshold2_1;
  __m256i symbol_gt_threshold2_2;
  __m256i symbol_gt_threshold3_1;
  __m256i symbol_gt_threshold3_2;
  __m256i bit01_1;
  __m256i bit23_1;
  __m256i bit45_1;
  __m256i bit01_2;
  __m256i bit23_2;
  __m256i bit45_2;
  __m256i bit0_1;
  __m256i bit1_1;
  __m256i bit2_1;
  __m256i bit3_1;
  __m256i bit4_1;
  __m256i bit5_1;
  __m256i bit0_2;
  __m256i bit1_2;
  __m256i bit2_2;
  __m256i bit3_2;
  __m256i bit4_2;
  __m256i bit5_2;
  __m256i bit0;
  __m256i bit1;
  __m256i bit2;
  __m256i bit3;
  __m256i bit4;
  __m256i bit5;
  __m256i result;
  __m256 scale_v = _mm256_set1_ps(-SCALE_BYTE_CONV_QAM16);
  __m256i vec_zero = _mm256_set1_epi16(0);
  __m256i offset1 = _mm256_set1_epi16(2 * SCALE_BYTE_CONV_QAM64 / sqrt(42));
  __m256i offset2 = _mm256_set1_epi16(4 * SCALE_BYTE_CONV_QAM64 / sqrt(42));
  __m256i offset3 = _mm256_set1_epi16(6 * SCALE_BYTE_CONV_QAM64 / sqrt(42));
  __m256i vec_true_mask = _mm256_set1_epi16(0x1);

  __m256i shuffle_real = _mm256_set_epi8(
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 13, 12, 9, 8, 5, 4, 1, 0,
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 13, 12, 9, 8, 5, 4, 1, 0);

  __m256i shuffle_imag =
      _mm256_set_epi8(0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 15, 14,
                      11, 10, 7, 6, 3, 2, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                      0xff, 0xff, 15, 14, 11, 10, 7, 6, 3, 2);

  __m256i shuffle_16_to_8 =
      _mm256_set_epi8(0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 14, 12,
                      10, 8, 6, 4, 2, 0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                      0xff, 0xff, 14, 12, 10, 8, 6, 4, 2, 0);

  for (int i = 0; i < num / 16; i++) {
    symbol1 = _mm256_load_ps(symbols_ptr);
    symbols_ptr += 8;
    symbol2 = _mm256_load_ps(symbols_ptr);
    symbols_ptr += 8;
    symbol3 = _mm256_load_ps(symbols_ptr);
    symbols_ptr += 8;
    symbol4 = _mm256_load_ps(symbols_ptr);
    symbols_ptr += 8;

    symbol_i1 = _mm256_cvtps_epi32(_mm256_mul_ps(symbol1, scale_v));
    symbol_i2 = _mm256_cvtps_epi32(_mm256_mul_ps(symbol2, scale_v));
    symbol_i3 = _mm256_cvtps_epi32(_mm256_mul_ps(symbol3, scale_v));
    symbol_i4 = _mm256_cvtps_epi32(_mm256_mul_ps(symbol4, scale_v));
    symbol_12 = _mm256_packs_epi32(symbol_i1, symbol_i2);
    symbol_12 = _mm256_permute4x64_epi64(symbol_12, 0xd8);
    symbol_34 = _mm256_packs_epi32(symbol_i3, symbol_i4);
    symbol_34 = _mm256_permute4x64_epi64(symbol_34, 0xd8);

    symbol_abs_1 = _mm256_abs_epi16(symbol_12);
    symbol_gt_0_1 = _mm256_cmpgt_epi16(symbol_12, vec_zero);
    symbol_lt_threshold1_1 = _mm256_cmpgt_epi16(offset1, symbol_abs_1);
    symbol_gt_threshold2_1 = _mm256_cmpgt_epi16(symbol_abs_1, offset2);
    symbol_gt_threshold3_1 = _mm256_cmpgt_epi16(symbol_abs_1, offset3);
    symbol_gt_0_1 = _mm256_and_si256(symbol_gt_0_1, vec_true_mask);

    symbol_lt_threshold1_1 =
        _mm256_and_si256(symbol_lt_threshold1_1, vec_true_mask);
    symbol_gt_threshold2_1 =
        _mm256_and_si256(symbol_gt_threshold2_1, vec_true_mask);
    symbol_gt_threshold3_1 =
        _mm256_and_si256(symbol_gt_threshold3_1, vec_true_mask);
    bit01_1 = _mm256_or_si256(symbol_lt_threshold1_1, symbol_gt_threshold3_1);
    bit23_1 = symbol_gt_threshold2_1;
    bit45_1 = symbol_gt_0_1;

    bit5_1 = _mm256_shuffle_epi8(bit45_1, shuffle_real);
    bit5_1 = _mm256_permute4x64_epi64(bit5_1, 0xd8);
    bit4_1 = _mm256_shuffle_epi8(bit45_1, shuffle_imag);
    bit4_1 = _mm256_permute4x64_epi64(bit4_1, 0xd8);

    bit3_1 = _mm256_shuffle_epi8(bit23_1, shuffle_real);
    bit3_1 = _mm256_permute4x64_epi64(bit3_1, 0xd8);
    bit2_1 = _mm256_shuffle_epi8(bit23_1, shuffle_imag);
    bit2_1 = _mm256_permute4x64_epi64(bit2_1, 0xd8);

    bit1_1 = _mm256_shuffle_epi8(bit01_1, shuffle_real);
    bit1_1 = _mm256_permute4x64_epi64(bit1_1, 0xd8);
    bit0_1 = _mm256_shuffle_epi8(bit01_1, shuffle_imag);
    bit0_1 = _mm256_permute4x64_epi64(bit0_1, 0xd8);

    symbol_abs_2 = _mm256_abs_epi16(symbol_34);
    symbol_gt_0_2 = _mm256_cmpgt_epi16(symbol_34, vec_zero);
    symbol_lt_threshold1_2 = _mm256_cmpgt_epi16(offset1, symbol_abs_2);
    symbol_gt_threshold2_2 = _mm256_cmpgt_epi16(symbol_abs_2, offset2);
    symbol_gt_threshold3_2 = _mm256_cmpgt_epi16(symbol_abs_2, offset3);
    symbol_gt_0_2 = _mm256_and_si256(symbol_gt_0_2, vec_true_mask);

    symbol_lt_threshold1_2 =
        _mm256_and_si256(symbol_lt_threshold1_2, vec_true_mask);
    symbol_gt_threshold2_2 =
        _mm256_and_si256(symbol_gt_threshold2_2, vec_true_mask);
    symbol_gt_threshold3_2 =
        _mm256_and_si256(symbol_gt_threshold3_2, vec_true_mask);
    bit01_2 = _mm256_or_si256(symbol_lt_threshold1_2, symbol_gt_threshold3_2);
    bit23_2 = symbol_gt_threshold2_2;
    bit45_2 = symbol_gt_0_2;

    bit5_2 = _mm256_shuffle_epi8(bit45_2, shuffle_real);
    bit5_2 = _mm256_permute4x64_epi64(bit5_2, 0xd8);
    bit4_2 = _mm256_shuffle_epi8(bit45_2, shuffle_imag);
    bit4_2 = _mm256_permute4x64_epi64(bit4_2, 0xd8);

    bit3_2 = _mm256_shuffle_epi8(bit23_2, shuffle_real);
    bit3_2 = _mm256_permute4x64_epi64(bit3_2, 0xd8);
    bit2_2 = _mm256_shuffle_epi8(bit23_2, shuffle_imag);
    bit2_2 = _mm256_permute4x64_epi64(bit2_2, 0xd8);

    bit1_2 = _mm256_shuffle_epi8(bit01_2, shuffle_real);
    bit1_2 = _mm256_permute4x64_epi64(bit1_2, 0xd8);
    bit0_2 = _mm256_shuffle_epi8(bit01_2, shuffle_imag);
    bit0_2 = _mm256_permute4x64_epi64(bit0_2, 0xd8);

    bit0 = _mm256_permute2x128_si256(bit0_1, bit0_2, 0x20);
    bit1 = _mm256_permute2x128_si256(bit1_1, bit1_2, 0x20);
    bit2 = _mm256_permute2x128_si256(bit2_1, bit2_2, 0x20);
    bit3 = _mm256_permute2x128_si256(bit3_1, bit3_2, 0x20);
    bit4 = _mm256_permute2x128_si256(bit4_1, bit4_2, 0x20);
    bit5 = _mm256_permute2x128_si256(bit5_1, bit5_2, 0x20);

    bit1 = _mm256_slli_epi16(bit1, 1);
    bit2 = _mm256_slli_epi16(bit2, 2);
    bit3 = _mm256_slli_epi16(bit3, 3);
    bit4 = _mm256_slli_epi16(bit4, 4);
    bit5 = _mm256_slli_epi16(bit5, 5);

    result = _mm256_add_epi16(bit0, bit1);
    result = _mm256_add_epi16(result, bit2);
    result = _mm256_add_epi16(result, bit3);
    result = _mm256_add_epi16(result, bit4);
    result = _mm256_add_epi16(result, bit5);

    result = _mm256_shuffle_epi8(result, shuffle_16_to_8);
    result = _mm256_permute4x64_epi64(result, 0xd8);
    _mm_storeu_si128(result_ptr, _mm256_extracti128_si256(result, 0));
    result_ptr++;
  }
  // Demodulate last symbols
  int next_start = 16 * (num / 16);
  Demod64qamHardSse(vec_in + 2 * next_start, vec_out + next_start,
                    num - next_start);
}

void Demod64qamSoftAvx2(float* vec_in, int8_t* llr, int num) {
  auto* symbols_ptr = static_cast<float*>(vec_in);
  auto* result_ptr = reinterpret_cast<__m256i*>(llr);
  __m256 symbol1;
  __m256 symbol2;
  __m256 symbol3;
  __m256 symbol4;
  __m256i symbol_i1;
  __m256i symbol_i2;
  __m256i symbol_i3;
  __m256i symbol_i4;
  __m256i symbol_i;
  __m256i symbol_abs;
  __m256i symbol_abs2;
  __m256i symbol_12;
  __m256i symbol_34;
  __m256i offset1 = _mm256_set1_epi8(4 * SCALE_BYTE_CONV_QAM64 / sqrt(42));
  __m256i offset2 = _mm256_set1_epi8(2 * SCALE_BYTE_CONV_QAM64 / sqrt(42));
  __m256 scale_v = _mm256_set1_ps(SCALE_BYTE_CONV_QAM64);
  __m256i result11;
  __m256i result12;
  __m256i result13;
  __m256i result22;
  __m256i result21;
  __m256i result23;
  __m256i result31;
  __m256i result32;
  __m256i result33;
  __m256i result_final1;
  __m256i result_final2;
  __m256i result_final3;

  __m256i shuffle_negated_1 =
      _mm256_set_epi8(0xff, 0xff, 5, 4, 0xff, 0xff, 0xff, 0xff, 3, 2, 0xff,
                      0xff, 0xff, 0xff, 1, 0, 0xff, 0xff, 5, 4, 0xff, 0xff,
                      0xff, 0xff, 3, 2, 0xff, 0xff, 0xff, 0xff, 1, 0);
  __m256i shuffle_negated_2 =
      _mm256_set_epi8(11, 10, 0xff, 0xff, 0xff, 0xff, 9, 8, 0xff, 0xff, 0xff,
                      0xff, 7, 6, 0xff, 0xff, 11, 10, 0xff, 0xff, 0xff, 0xff, 9,
                      8, 0xff, 0xff, 0xff, 0xff, 7, 6, 0xff, 0xff);
  __m256i shuffle_negated_3 = _mm256_set_epi8(
      0xff, 0xff, 0xff, 0xff, 15, 14, 0xff, 0xff, 0xff, 0xff, 13, 12, 0xff,
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 15, 14, 0xff, 0xff, 0xff, 0xff,
      13, 12, 0xff, 0xff, 0xff, 0xff);

  __m256i shuffle_abs_1 =
      _mm256_set_epi8(5, 4, 0xff, 0xff, 0xff, 0xff, 3, 2, 0xff, 0xff, 0xff,
                      0xff, 1, 0, 0xff, 0xff, 5, 4, 0xff, 0xff, 0xff, 0xff, 3,
                      2, 0xff, 0xff, 0xff, 0xff, 1, 0, 0xff, 0xff);
  __m256i shuffle_abs_2 =
      _mm256_set_epi8(0xff, 0xff, 0xff, 0xff, 9, 8, 0xff, 0xff, 0xff, 0xff, 7,
                      6, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 9, 8,
                      0xff, 0xff, 0xff, 0xff, 7, 6, 0xff, 0xff, 0xff, 0xff);
  __m256i shuffle_abs_3 =
      _mm256_set_epi8(0xff, 0xff, 15, 14, 0xff, 0xff, 0xff, 0xff, 13, 12, 0xff,
                      0xff, 0xff, 0xff, 11, 10, 0xff, 0xff, 15, 14, 0xff, 0xff,
                      0xff, 0xff, 13, 12, 0xff, 0xff, 0xff, 0xff, 11, 10);

  __m256i shuffle_abs2_1 =
      _mm256_set_epi8(0xff, 0xff, 0xff, 0xff, 3, 2, 0xff, 0xff, 0xff, 0xff, 1,
                      0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 3, 2,
                      0xff, 0xff, 0xff, 0xff, 1, 0, 0xff, 0xff, 0xff, 0xff);
  __m256i shuffle_abs2_2 =
      _mm256_set_epi8(0xff, 0xff, 9, 8, 0xff, 0xff, 0xff, 0xff, 7, 6, 0xff,
                      0xff, 0xff, 0xff, 5, 4, 0xff, 0xff, 9, 8, 0xff, 0xff,
                      0xff, 0xff, 7, 6, 0xff, 0xff, 0xff, 0xff, 5, 4);
  __m256i shuffle_abs2_3 =
      _mm256_set_epi8(15, 14, 0xff, 0xff, 0xff, 0xff, 13, 12, 0xff, 0xff, 0xff,
                      0xff, 11, 10, 0xff, 0xff, 15, 14, 0xff, 0xff, 0xff, 0xff,
                      13, 12, 0xff, 0xff, 0xff, 0xff, 11, 10, 0xff, 0xff);

  for (int i = 0; i < num / 16; i++) {
    // Load symbols, 4 real and 4 imaginary values at a time
    symbol1 = _mm256_load_ps(symbols_ptr);
    symbols_ptr += 8;
    symbol2 = _mm256_load_ps(symbols_ptr);
    symbols_ptr += 8;
    symbol3 = _mm256_load_ps(symbols_ptr);
    symbols_ptr += 8;
    symbol4 = _mm256_load_ps(symbols_ptr);
    symbols_ptr += 8;
    // Cast symbols into integers
    symbol_i1 = _mm256_cvtps_epi32(_mm256_mul_ps(symbol1, scale_v));
    symbol_i2 = _mm256_cvtps_epi32(_mm256_mul_ps(symbol2, scale_v));
    symbol_i3 = _mm256_cvtps_epi32(_mm256_mul_ps(symbol3, scale_v));
    symbol_i4 = _mm256_cvtps_epi32(_mm256_mul_ps(symbol4, scale_v));
    // Pack symbols into 16 bit integers
    symbol_12 = _mm256_packs_epi32(symbol_i1, symbol_i2);
    symbol_12 = _mm256_permute4x64_epi64(symbol_12, 0xd8);
    symbol_34 = _mm256_packs_epi32(symbol_i3, symbol_i4);
    symbol_34 = _mm256_permute4x64_epi64(symbol_34, 0xd8);
    // Pack symbols into 8 bit integers (one 256 bit vector)
    symbol_i = _mm256_packs_epi16(symbol_12, symbol_34);
    symbol_i = _mm256_permute4x64_epi64(symbol_i, 0xd8);
    // first LLR is simply the symbol
    // this LLR corresponds to bit 5 and 4 (both flip over the I and Q axis)
    // LLR(b5,b4) = |x|
    symbol_abs = _mm256_abs_epi8(symbol_i);
    // Take distance between offset1 and symbols for second LLR
    // offset1 here divides the point where bit 3 and 2 flip (over 4d)
    // LLR(b3,b2) = 4d - |x|
    symbol_abs = _mm256_sub_epi8(offset1, symbol_abs);
    // third LLR is difference between offset2 and first distance
    // offset2 is 2d (lower point where bit 1 and 0 flip)
    // LLR(b1,b0) = 2d - |4d - |x||
    symbol_abs2 = _mm256_sub_epi8(offset2, _mm256_abs_epi8(symbol_abs));

    // Pack so that the LLRs for real and imaginary part of each modulated value
    // are distributed as follows:
    // real msb: imag msb: real: imag: real lsb: imag lsb
    result11 = _mm256_shuffle_epi8(symbol_i, shuffle_negated_1);
    result12 = _mm256_shuffle_epi8(symbol_abs, shuffle_abs_1);
    result13 = _mm256_shuffle_epi8(symbol_abs2, shuffle_abs2_1);

    result21 = _mm256_shuffle_epi8(symbol_i, shuffle_negated_2);
    result22 = _mm256_shuffle_epi8(symbol_abs, shuffle_abs_2);
    result23 = _mm256_shuffle_epi8(symbol_abs2, shuffle_abs2_2);

    result31 = _mm256_shuffle_epi8(symbol_i, shuffle_negated_3);
    result32 = _mm256_shuffle_epi8(symbol_abs, shuffle_abs_3);
    result33 = _mm256_shuffle_epi8(symbol_abs2, shuffle_abs2_3);

    // OR all results together
    result_final1 =
        _mm256_or_si256(_mm256_or_si256(result11, result12), result13);
    result_final2 =
        _mm256_or_si256(_mm256_or_si256(result21, result22), result23);
    result_final3 =
        _mm256_or_si256(_mm256_or_si256(result31, result32), result33);
    // Permute to string all results together
    _mm256_storeu_si256(result_ptr, _mm256_permute2x128_si256(
                                        result_final1, result_final2, 0x20));
    result_ptr++;
    _mm256_storeu_si256(result_ptr, _mm256_permute2x128_si256(
                                        result_final3, result_final1, 0x30));
    result_ptr++;
    _mm256_storeu_si256(result_ptr, _mm256_permute2x128_si256(
                                        result_final2, result_final3, 0x31));
    result_ptr++;
  }
  int next_start = 16 * (num / 16);
  Demod64qamSoftSse(vec_in + 2 * next_start, llr + next_start * 6,
                    num - next_start);
}

/**
 * 256-QAM Modulation
 *  Q
 * 01000000 01000010 01001010 01001000 01101000 01101010 01100010 01100000 | 11100000 11100010 11101010 11101000 11001000 11001010 11000010 11000000 
 * 01000001 01000011 01001011 01001001 01101001 01101011 01100011 01100001 | 11100001 11100011 11101011 11101001 11001001 11001011 11000011 11000001 
 * 01000101 01000111 01001111 01001101 01101101 01101111 01100111 01100101 | 11100101 11100111 11101111 11101101 11001101 11001111 11000111 11000101 
 * 01000100 01000110 01001110 01001100 01101100 01101110 01100110 01100100 | 11100100 11100110 11101110 11101100 11001100 11001110 11000110 11000100 
 * 01010100 01010110 01011110 01011100 01111100 01111110 01110110 01110100 | 11110100 11110110 11111110 11111100 11011100 11011110 11010110 11010100 
 * 01010101 01010111 01011111 01011101 01111101 01111111 01110111 01110101 | 11110101 11110111 11111111 11111101 11011101 11011111 11010111 11010101 
 * 01010001 01010011 01011011 01011001 01111001 01111011 01110011 01110001 | 11110001 11110011 11111011 11111001 11011001 11011011 11010011 11010001 
 * 01010000 01010010 01011010 01011000 01111000 01111010 01110010 01110000 | 11110000 11110010 11111010 11111000 11011000 11011010 11010010 11010000 
 * ------------------------------------------------------------------------------------------------------------------------------------------------- I
 * 00010000 00010010 00011010 00011000 00111000 00111010 00110010 00110000 | 10110000 10110010 10111010 10111000 10011000 10011010 10010010 10010000 
 * 00010001 00010011 00011011 00011001 00111001 00111011 00110011 00110001 | 10110001 10110011 10111011 10111001 10011001 10011011 10010011 10010001 
 * 00010101 00010111 00011111 00011101 00111101 00111111 00110111 00110101 | 10110101 10110111 10111111 10111101 10011101 10011111 10010111 10010101 
 * 00010100 00010110 00011110 00011100 00111100 00111110 00110110 00110100 | 10110100 10110110 10111110 10111100 10011100 10011110 10010110 10010100 
 * 00000100 00000110 00001110 00001100 00101100 00101110 00100110 00100100 | 10100100 10100110 10101110 10101100 10001100 10001110 10000110 10000100 
 * 00000101 00000111 00001111 00001101 00101101 00101111 00100111 00100101 | 10100101 10100111 10101111 10101101 10001101 10001111 10000111 10000101 
 * 00000001 00000011 00001011 00001001 00101001 00101011 00100011 00100001 | 10100001 10100011 10101011 10101001 10001001 10001011 10000011 10000001 
 * 00000000 00000010 00001010 00001000 00101000 00101010 00100010 00100000 | 10100000 10100010 10101010 10101000 10001000 10001010 10000010 10000000
 */
void Demod256qamHardLoop(const float* vec_in, uint8_t* vec_out, int num) {
  /**
   * bit 7: set if real is positive
   * bit 6: set if imag is positive
   * bit 5: set if real is under threshold 4
   * bit 4: set if imag is under threshold 4
   * bit 3: set if real is over threshold 2 and under threshold 6
   * bit 2: set if imag is over threshold 2 and under threshold 6
   * bit 1: set between real threshold 5-6 and 2-3
   * bit 0: set between imag threshold 5-6 and 2-3
   */
  for (int i = 0; i < num; i++) {
    float real_val = vec_in[i * 2];
    float imag_val = vec_in[(i * 2) + 1];

    // Initialize this output
    vec_out[i] = 0;

    // We can determine top 2 bits using the sign of the I and Q symbols
    if (real_val > 0) {
      vec_out[i] |= 0x80;
    }
    if (imag_val > 0) {
      vec_out[i] |= 0x40;
    }
    // Now, check each threshold to determine which bits should be set
    if (std::abs(real_val) < QAM256_THRESHOLD_1) {
      vec_out[i] |= 0x20;
    } else if (std::abs(real_val) < QAM256_THRESHOLD_2) {
      vec_out[i] |= 0x22;
    } else if (std::abs(real_val) < QAM256_THRESHOLD_3) {
      vec_out[i] |= 0x2A;
    } else if (std::abs(real_val) < QAM256_THRESHOLD_4) {
      vec_out[i] |= 0x28;
    } else if (std::abs(real_val) < QAM256_THRESHOLD_5) {
      vec_out[i] |= 0x08;
    } else if (std::abs(real_val) < QAM256_THRESHOLD_6) {
      vec_out[i] |= 0x0A;
    } else if (std::abs(real_val) < QAM256_THRESHOLD_7) {
      vec_out[i] |= 0x02;
    }
    // Check same thresholds for real value
    if (std::abs(imag_val) < QAM256_THRESHOLD_1) {
      vec_out[i] |= 0x10;
    } else if (std::abs(imag_val) < QAM256_THRESHOLD_2) {
      vec_out[i] |= 0x11;
    } else if (std::abs(imag_val) < QAM256_THRESHOLD_3) {
      vec_out[i] |= 0x15;
    } else if (std::abs(imag_val) < QAM256_THRESHOLD_4) {
      vec_out[i] |= 0x14;
    } else if (std::abs(imag_val) < QAM256_THRESHOLD_5) {
      vec_out[i] |= 0x04;
    } else if (std::abs(imag_val) < QAM256_THRESHOLD_6) {
      vec_out[i] |= 0x05;
    } else if (std::abs(imag_val) < QAM256_THRESHOLD_7) {
      vec_out[i] |= 0x01;
    }
  }
}

void Demod256qamHardSse(float* vec_in, uint8_t* vec_out, int num) {
  __m128 symbol1;
  __m128 symbol2;
  __m128 symbol3;
  __m128 symbol4;
  __m128i intsymbol1;
  __m128i intsymbol2;
  __m128i intsymbol3;
  __m128i intsymbol4;
  __m128i symbol12;
  __m128i symbol34;
  __m128i symbol_abs;
  __m128i symbol_gt_0;
  __m128i symbol_gt_threshold1;
  __m128i symbol_gt_threshold2;
  __m128i symbol_lt_threshold3;
  __m128i symbol_lt_threshold4;
  __m128i symbol_gt_threshold5;
  __m128i symbol_lt_threshold6;
  __m128i symbol_lt_threshold7;
  __m128i symbol12_bit0;
  __m128i symbol12_bit1;
  __m128i symbol12_bit2;
  __m128i symbol12_bit3;
  __m128i symbol12_bit4;
  __m128i symbol12_bit5;
  __m128i symbol12_bit6;
  __m128i symbol12_bit7;
  __m128i symbol34_bit0;
  __m128i symbol34_bit1;
  __m128i symbol34_bit2;
  __m128i symbol34_bit3;
  __m128i symbol34_bit4;
  __m128i symbol34_bit5;
  __m128i symbol34_bit6;
  __m128i symbol34_bit7;
  __m128i bit7;
  __m128i bit6;
  __m128i bit5;
  __m128i bit4;
  __m128i bit3;
  __m128i bit2;
  __m128i bit1;
  __m128i bit0;
  __m128i result;
  __m128i symbol_bit01;
  __m128i symbol_bit23;
  __m128i symbol_bit45;
  __m128i symbol_bit67 __attribute__((aligned(16)));
  auto* result_ptr = reinterpret_cast<__m64*>(vec_out);
  int remaining_symbols;
  /*
   * Note: SCALE_BYTE_CONV_QAM16 factor here is simply a way to force all
   * symbols to be in integer range so they can be packed into 16 bit integers
   */
  __m128 scale_factor = _mm_set1_ps(SCALE_BYTE_CONV_QAM256);
  __m128i vec_zero = _mm_set1_epi16(0);
  __m128i threshold1 =
      _mm_set1_epi16(QAM256_THRESHOLD_1 * SCALE_BYTE_CONV_QAM256);
  __m128i threshold2 =
      _mm_set1_epi16(QAM256_THRESHOLD_2 * SCALE_BYTE_CONV_QAM256);
  __m128i threshold3 =
      _mm_set1_epi16(QAM256_THRESHOLD_3 * SCALE_BYTE_CONV_QAM256);
  __m128i threshold4 =
      _mm_set1_epi16(QAM256_THRESHOLD_4 * SCALE_BYTE_CONV_QAM256);
  __m128i threshold5 =
      _mm_set1_epi16(QAM256_THRESHOLD_5 * SCALE_BYTE_CONV_QAM256);
  __m128i threshold6 =
      _mm_set1_epi16(QAM256_THRESHOLD_6 * SCALE_BYTE_CONV_QAM256);
  __m128i threshold7 =
      _mm_set1_epi16(QAM256_THRESHOLD_7 * SCALE_BYTE_CONV_QAM256);
  __m128i true_mask = _mm_set1_epi16(0x1);

  __m128i shuffle_real_1 = _mm_set_epi8(0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                                        0xff, 0xff, 13, 12, 9, 8, 5, 4, 1, 0);
  __m128i shuffle_real_2 = _mm_set_epi8(13, 12, 9, 8, 5, 4, 1, 0, 0xff, 0xff,
                                        0xff, 0xff, 0xff, 0xff, 0xff, 0xff);

  __m128i shuffle_imag_1 = _mm_set_epi8(0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                                        0xff, 0xff, 15, 14, 11, 10, 7, 6, 3, 2);
  __m128i shuffle_imag_2 = _mm_set_epi8(15, 14, 11, 10, 7, 6, 3, 2, 0xff, 0xff,
                                        0xff, 0xff, 0xff, 0xff, 0xff, 0xff);

  __m128i shuffle_16_to_8 = _mm_set_epi8(0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                                         0xff, 0xff, 14, 12, 10, 8, 6, 4, 2, 0);

  float* symbols_ptr = vec_in;
  for (int i = 0; i < num / 8; i++) {
    /*
     * Pack a total of 32 symbols across 4 vectors. 32 symbols means 16 pairs
     * of real and imaginary values, or 16 8 bit outputs (16 * 8 = 128)
     */
    symbol1 = _mm_load_ps(symbols_ptr);
    symbols_ptr += 4;
    symbol2 = _mm_load_ps(symbols_ptr);
    symbols_ptr += 4;
    symbol3 = _mm_load_ps(symbols_ptr);
    symbols_ptr += 4;
    symbol4 = _mm_load_ps(symbols_ptr);
    symbols_ptr += 4;
    // Scale each symbol, then pack them into 32 bit integer vectors
    intsymbol1 = _mm_cvtps_epi32(_mm_mul_ps(symbol1, scale_factor));
    intsymbol2 = _mm_cvtps_epi32(_mm_mul_ps(symbol2, scale_factor));
    intsymbol3 = _mm_cvtps_epi32(_mm_mul_ps(symbol3, scale_factor));
    intsymbol4 = _mm_cvtps_epi32(_mm_mul_ps(symbol4, scale_factor));
    // Pack the symbols into 16 bit integers. We now need two 128 bit vectors.
    symbol12 = _mm_packs_epi32(intsymbol1, intsymbol2);
    symbol34 = _mm_packs_epi32(intsymbol3, intsymbol4);

    /*
     * Process symbol vectors. We need to take the absolute value of the
     * symbols, then compare them to each threshold (including zero)
     */
    symbol_abs = _mm_abs_epi16(symbol12);
    symbol_gt_0 = _mm_cmpgt_epi16(symbol12, vec_zero);
    symbol_gt_threshold1 = _mm_cmpgt_epi16(symbol_abs, threshold1);
    symbol_gt_threshold2 = _mm_cmpgt_epi16(symbol_abs, threshold2);
    symbol_lt_threshold3 = _mm_cmpgt_epi16(threshold3, symbol_abs);
    symbol_lt_threshold4 = _mm_cmpgt_epi16(threshold4, symbol_abs);
    symbol_gt_threshold5 = _mm_cmpgt_epi16(symbol_abs, threshold5);
    symbol_lt_threshold6 = _mm_cmpgt_epi16(threshold6, symbol_abs);
    symbol_lt_threshold7 = _mm_cmpgt_epi16(threshold7, symbol_abs);
    /*
     * _mm_cmpgt_epi16 will place 0xFFFF in the 16 bit slot if the 16 bit
     * integer in vector a is greater than the corresponding one in vector b
     * for that offset. Mask with 16 values of 0x1 so that the those locations
     * where the comparision was true only hold 0x1, not 0xFFFF.
     */
    symbol_gt_0 = _mm_and_si128(symbol_gt_0, true_mask);
    symbol_gt_threshold1 = _mm_and_si128(symbol_gt_threshold1, true_mask);
    symbol_gt_threshold2 = _mm_and_si128(symbol_gt_threshold2, true_mask);
    symbol_lt_threshold3 = _mm_and_si128(symbol_lt_threshold3, true_mask);
    symbol_lt_threshold4 = _mm_and_si128(symbol_lt_threshold4, true_mask);
    symbol_gt_threshold5 = _mm_and_si128(symbol_gt_threshold5, true_mask);
    symbol_lt_threshold6 = _mm_and_si128(symbol_lt_threshold6, true_mask);
    symbol_lt_threshold7 = _mm_and_si128(symbol_lt_threshold7, true_mask);
    /*
     * Now determine the value of each bit. The value of each bit is based off
     * the values stored in each comparision output, and how those relate to
     * the actual QAM table, given in the docstring for Demod256qamHardLoop.
     * see that function for additional explanation of how each threshold
     * influences which bits should be set.
     */
    /*
     * First, we determine the value of bits together, since the imaginary and
     * real symbols have similar conditions
     */
    symbol_bit01 =
        _mm_or_si128(_mm_and_si128(symbol_lt_threshold7, symbol_gt_threshold5),
                     _mm_and_si128(symbol_lt_threshold3, symbol_gt_threshold1));
    symbol_bit23 = _mm_and_si128(symbol_lt_threshold6, symbol_gt_threshold2);
    symbol_bit45 = symbol_lt_threshold4;
    symbol_bit67 = symbol_gt_0;
    // Now, unpack these values into individual bits
    symbol12_bit7 = _mm_shuffle_epi8(symbol_bit67, shuffle_real_1);
    symbol12_bit6 = _mm_shuffle_epi8(symbol_bit67, shuffle_imag_1);
    symbol12_bit5 = _mm_shuffle_epi8(symbol_bit45, shuffle_real_1);
    symbol12_bit4 = _mm_shuffle_epi8(symbol_bit45, shuffle_imag_1);
    symbol12_bit3 = _mm_shuffle_epi8(symbol_bit23, shuffle_real_1);
    symbol12_bit2 = _mm_shuffle_epi8(symbol_bit23, shuffle_imag_1);
    symbol12_bit1 = _mm_shuffle_epi8(symbol_bit01, shuffle_real_1);
    symbol12_bit0 = _mm_shuffle_epi8(symbol_bit01, shuffle_imag_1);

    /*
     * We now need to do the same process for the second set of symbols.
     * The bits from these symbols will then be packed into the upper half of a
     * second vector, that they can be OR'ed to create the final result
     */

    symbol_abs = _mm_abs_epi16(symbol34);
    symbol_gt_0 = _mm_cmpgt_epi16(symbol34, vec_zero);
    symbol_gt_threshold1 = _mm_cmpgt_epi16(symbol_abs, threshold1);
    symbol_gt_threshold2 = _mm_cmpgt_epi16(symbol_abs, threshold2);
    symbol_lt_threshold3 = _mm_cmpgt_epi16(threshold3, symbol_abs);
    symbol_lt_threshold4 = _mm_cmpgt_epi16(threshold4, symbol_abs);
    symbol_gt_threshold5 = _mm_cmpgt_epi16(symbol_abs, threshold5);
    symbol_lt_threshold6 = _mm_cmpgt_epi16(threshold6, symbol_abs);
    symbol_lt_threshold7 = _mm_cmpgt_epi16(threshold7, symbol_abs);

    symbol_gt_0 = _mm_and_si128(symbol_gt_0, true_mask);
    symbol_gt_threshold1 = _mm_and_si128(symbol_gt_threshold1, true_mask);
    symbol_gt_threshold2 = _mm_and_si128(symbol_gt_threshold2, true_mask);
    symbol_lt_threshold3 = _mm_and_si128(symbol_lt_threshold3, true_mask);
    symbol_lt_threshold4 = _mm_and_si128(symbol_lt_threshold4, true_mask);
    symbol_gt_threshold5 = _mm_and_si128(symbol_gt_threshold5, true_mask);
    symbol_lt_threshold6 = _mm_and_si128(symbol_lt_threshold6, true_mask);
    symbol_lt_threshold7 = _mm_and_si128(symbol_lt_threshold7, true_mask);

    symbol_bit01 =
        _mm_or_si128(_mm_and_si128(symbol_lt_threshold7, symbol_gt_threshold5),
                     _mm_and_si128(symbol_lt_threshold3, symbol_gt_threshold1));
    symbol_bit23 = _mm_and_si128(symbol_lt_threshold6, symbol_gt_threshold2);
    symbol_bit45 = symbol_lt_threshold4;
    symbol_bit67 = symbol_gt_0;
    // Now, unpack these values into individual bits
    symbol34_bit7 = _mm_shuffle_epi8(symbol_bit67, shuffle_real_2);
    symbol34_bit6 = _mm_shuffle_epi8(symbol_bit67, shuffle_imag_2);
    symbol34_bit5 = _mm_shuffle_epi8(symbol_bit45, shuffle_real_2);
    symbol34_bit4 = _mm_shuffle_epi8(symbol_bit45, shuffle_imag_2);
    symbol34_bit3 = _mm_shuffle_epi8(symbol_bit23, shuffle_real_2);
    symbol34_bit2 = _mm_shuffle_epi8(symbol_bit23, shuffle_imag_2);
    symbol34_bit1 = _mm_shuffle_epi8(symbol_bit01, shuffle_real_2);
    symbol34_bit0 = _mm_shuffle_epi8(symbol_bit01, shuffle_imag_2);

    bit7 = _mm_or_si128(symbol12_bit7, symbol34_bit7);
    bit6 = _mm_or_si128(symbol12_bit6, symbol34_bit6);
    bit5 = _mm_or_si128(symbol12_bit5, symbol34_bit5);
    bit4 = _mm_or_si128(symbol12_bit4, symbol34_bit4);
    bit3 = _mm_or_si128(symbol12_bit3, symbol34_bit3);
    bit2 = _mm_or_si128(symbol12_bit2, symbol34_bit2);
    bit1 = _mm_or_si128(symbol12_bit1, symbol34_bit1);
    bit0 = _mm_or_si128(symbol12_bit0, symbol34_bit0);

    // Left shift each bit so that they can be combined into output
    bit1 = _mm_slli_epi16(bit1, 1);
    bit2 = _mm_slli_epi16(bit2, 2);
    bit3 = _mm_slli_epi16(bit3, 3);
    bit4 = _mm_slli_epi16(bit4, 4);
    bit5 = _mm_slli_epi16(bit5, 5);
    bit6 = _mm_slli_epi16(bit6, 6);
    bit7 = _mm_slli_epi16(bit7, 7);

    result = _mm_add_epi16(bit0, bit1);
    result = _mm_add_epi16(result, bit2);
    result = _mm_add_epi16(result, bit3);
    result = _mm_add_epi16(result, bit4);
    result = _mm_add_epi16(result, bit5);
    result = _mm_add_epi16(result, bit6);
    result = _mm_add_epi16(result, bit7);

    result = _mm_shuffle_epi8(result, shuffle_16_to_8);
    _mm_storel_pi(result_ptr, (__m128)result);
    result_ptr++;
  }
  // Demodulate symbols that don't fit in a multiple of 8
  remaining_symbols = num % 8;
  Demod256qamHardLoop(vec_in + 2 * (num - remaining_symbols),
                      vec_out + (num - remaining_symbols), remaining_symbols);
}

void Demod256qamHardAvx2(float* vec_in, uint8_t* vec_out, int num) {
  float* symbols_ptr = vec_in;
  auto* result_ptr = reinterpret_cast<__m128i*>(vec_out);
  __m256 symbol1;
  __m256 symbol2;
  __m256 symbol3;
  __m256 symbol4;
  __m256i intsymbol1;
  __m256i intsymbol2;
  __m256i intsymbol3;
  __m256i intsymbol4;
  __m256i symbol12;
  __m256i symbol34;
  __m256i symbol_abs;
  __m256i symbol_gt_0;
  __m256i symbol_gt_threshold1;
  __m256i symbol_gt_threshold2;
  __m256i symbol_lt_threshold3;
  __m256i symbol_lt_threshold4;
  __m256i symbol_gt_threshold5;
  __m256i symbol_lt_threshold6;
  __m256i symbol_lt_threshold7;
  __m256i symbol12_bit0;
  __m256i symbol12_bit1;
  __m256i symbol12_bit2;
  __m256i symbol12_bit3;
  __m256i symbol12_bit4;
  __m256i symbol12_bit5;
  __m256i symbol12_bit6;
  __m256i symbol12_bit7;
  __m256i symbol34_bit0;
  __m256i symbol34_bit1;
  __m256i symbol34_bit2;
  __m256i symbol34_bit3;
  __m256i symbol34_bit4;
  __m256i symbol34_bit5;
  __m256i symbol34_bit6;
  __m256i symbol34_bit7;
  __m256i bit7;
  __m256i bit6;
  __m256i bit5;
  __m256i bit4;
  __m256i bit3;
  __m256i bit2;
  __m256i bit1;
  __m256i bit0;
  __m256i result;
  __m256i symbol_bit01;
  __m256i symbol_bit23;
  __m256i symbol_bit45;
  __m256i symbol_bit67;
  int next_start;

  /*
   * Note: SCALE_BYTE_CONV_QAM16 factor here is simply a way to force all
   * symbols to be in integer range so they can be packed into 16 bit integers
   */
  __m256 scale_factor = _mm256_set1_ps(SCALE_BYTE_CONV_QAM256);
  __m256i vec_zero = _mm256_set1_epi16(0);
  __m256i threshold1 =
      _mm256_set1_epi16(QAM256_THRESHOLD_1 * SCALE_BYTE_CONV_QAM256);
  __m256i threshold2 =
      _mm256_set1_epi16(QAM256_THRESHOLD_2 * SCALE_BYTE_CONV_QAM256);
  __m256i threshold3 =
      _mm256_set1_epi16(QAM256_THRESHOLD_3 * SCALE_BYTE_CONV_QAM256);
  __m256i threshold4 =
      _mm256_set1_epi16(QAM256_THRESHOLD_4 * SCALE_BYTE_CONV_QAM256);
  __m256i threshold5 =
      _mm256_set1_epi16(QAM256_THRESHOLD_5 * SCALE_BYTE_CONV_QAM256);
  __m256i threshold6 =
      _mm256_set1_epi16(QAM256_THRESHOLD_6 * SCALE_BYTE_CONV_QAM256);
  __m256i threshold7 =
      _mm256_set1_epi16(QAM256_THRESHOLD_7 * SCALE_BYTE_CONV_QAM256);
  __m256i true_mask = _mm256_set1_epi16(0x1);

  __m256i shuffle_real = _mm256_set_epi8(
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 13, 12, 9, 8, 5, 4, 1, 0,
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 13, 12, 9, 8, 5, 4, 1, 0);

  __m256i shuffle_imag =
      _mm256_set_epi8(0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 15, 14,
                      11, 10, 7, 6, 3, 2, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                      0xff, 0xff, 15, 14, 11, 10, 7, 6, 3, 2);

  __m256i shuffle_16_to_8 =
      _mm256_set_epi8(0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 14, 12,
                      10, 8, 6, 4, 2, 0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                      0xff, 0xff, 14, 12, 10, 8, 6, 4, 2, 0);

  for (int i = 0; i < num / 16; i++) {
    // Load symbols, 8 per vector
    symbol1 = _mm256_load_ps(symbols_ptr);
    symbols_ptr += 8;
    symbol2 = _mm256_load_ps(symbols_ptr);
    symbols_ptr += 8;
    symbol3 = _mm256_load_ps(symbols_ptr);
    symbols_ptr += 8;
    symbol4 = _mm256_load_ps(symbols_ptr);
    symbols_ptr += 8;

    // Scale symbols by fixed value, and truncate into 32 bit integers
    intsymbol1 = _mm256_cvtps_epi32(_mm256_mul_ps(symbol1, scale_factor));
    intsymbol2 = _mm256_cvtps_epi32(_mm256_mul_ps(symbol2, scale_factor));
    intsymbol3 = _mm256_cvtps_epi32(_mm256_mul_ps(symbol3, scale_factor));
    intsymbol4 = _mm256_cvtps_epi32(_mm256_mul_ps(symbol4, scale_factor));

    // Pack symbols into 2 blocks of 16 bit integers
    symbol12 = _mm256_packs_epi32(intsymbol1, intsymbol2);
    /*
     * _mm256_packs_epi32 interleaves the values of both symbols,
     * so permute them back to be in order
     */
    symbol12 = _mm256_permute4x64_epi64(symbol12, 0xd8);
    symbol34 = _mm256_packs_epi32(intsymbol3, intsymbol4);
    symbol34 = _mm256_permute4x64_epi64(symbol34, 0xd8);

    /*
     * Process symbol vectors. We need to take the absolute value of the
     * symbols, then compare them to each threshold (including zero)
     */
    symbol_abs = _mm256_abs_epi16(symbol12);
    symbol_gt_0 = _mm256_cmpgt_epi16(symbol12, vec_zero);
    symbol_gt_threshold1 = _mm256_cmpgt_epi16(symbol_abs, threshold1);
    symbol_gt_threshold2 = _mm256_cmpgt_epi16(symbol_abs, threshold2);
    symbol_lt_threshold3 = _mm256_cmpgt_epi16(threshold3, symbol_abs);
    symbol_lt_threshold4 = _mm256_cmpgt_epi16(threshold4, symbol_abs);
    symbol_gt_threshold5 = _mm256_cmpgt_epi16(symbol_abs, threshold5);
    symbol_lt_threshold6 = _mm256_cmpgt_epi16(threshold6, symbol_abs);
    symbol_lt_threshold7 = _mm256_cmpgt_epi16(threshold7, symbol_abs);
    /*
     * _mm_cmpgt_epi16 will place 0xFFFF in the 16 bit slot if the 16 bit
     * integer in vector a is greater than the corresponding one in vector b
     * for that offset. Mask with 16 values of 0x1 so that the those locations
     * where the comparision was true only hold 0x1, not 0xFFFF.
     */
    symbol_gt_0 = _mm256_and_si256(symbol_gt_0, true_mask);
    symbol_gt_threshold1 = _mm256_and_si256(symbol_gt_threshold1, true_mask);
    symbol_gt_threshold2 = _mm256_and_si256(symbol_gt_threshold2, true_mask);
    symbol_lt_threshold3 = _mm256_and_si256(symbol_lt_threshold3, true_mask);
    symbol_lt_threshold4 = _mm256_and_si256(symbol_lt_threshold4, true_mask);
    symbol_gt_threshold5 = _mm256_and_si256(symbol_gt_threshold5, true_mask);
    symbol_lt_threshold6 = _mm256_and_si256(symbol_lt_threshold6, true_mask);
    symbol_lt_threshold7 = _mm256_and_si256(symbol_lt_threshold7, true_mask);
    /*
     * Now determine the value of each bit. The value of each bit is based off
     * the values stored in each comparision output, and how those relate to
     * the actual QAM table, given in the docstring for Demod256qamHardLoop.
     * see that function for additional explanation of how each threshold
     * influences which bits should be set.
     */
    /*
     * First, we determine the value of bits together, since the imaginary and
     * real symbols have similar conditions
     */
    symbol_bit01 = _mm256_or_si256(
        _mm256_and_si256(symbol_lt_threshold7, symbol_gt_threshold5),
        _mm256_and_si256(symbol_lt_threshold3, symbol_gt_threshold1));
    symbol_bit23 = _mm256_and_si256(symbol_lt_threshold6, symbol_gt_threshold2);
    symbol_bit45 = symbol_lt_threshold4;
    symbol_bit67 = symbol_gt_0;
    // Now, unpack these values into individual bits
    symbol12_bit7 = _mm256_shuffle_epi8(symbol_bit67, shuffle_real);
    /*
     * Due to the way that _mm256_shuffle arranges the 16 bit values, we must
     * now further shuffle each bit so that all selected bits are in the lower
     * 128 bits of the vector
     */
    symbol12_bit7 = _mm256_permute4x64_epi64(symbol12_bit7, 0xd8);
    symbol12_bit6 = _mm256_shuffle_epi8(symbol_bit67, shuffle_imag);
    symbol12_bit6 = _mm256_permute4x64_epi64(symbol12_bit6, 0xd8);
    symbol12_bit5 = _mm256_shuffle_epi8(symbol_bit45, shuffle_real);
    symbol12_bit5 = _mm256_permute4x64_epi64(symbol12_bit5, 0xd8);
    symbol12_bit4 = _mm256_shuffle_epi8(symbol_bit45, shuffle_imag);
    symbol12_bit4 = _mm256_permute4x64_epi64(symbol12_bit4, 0xd8);
    symbol12_bit3 = _mm256_shuffle_epi8(symbol_bit23, shuffle_real);
    symbol12_bit3 = _mm256_permute4x64_epi64(symbol12_bit3, 0xd8);
    symbol12_bit2 = _mm256_shuffle_epi8(symbol_bit23, shuffle_imag);
    symbol12_bit2 = _mm256_permute4x64_epi64(symbol12_bit2, 0xd8);
    symbol12_bit1 = _mm256_shuffle_epi8(symbol_bit01, shuffle_real);
    symbol12_bit1 = _mm256_permute4x64_epi64(symbol12_bit1, 0xd8);
    symbol12_bit0 = _mm256_shuffle_epi8(symbol_bit01, shuffle_imag);
    symbol12_bit0 = _mm256_permute4x64_epi64(symbol12_bit0, 0xd8);

    /*
     * We now repeat the above process for the symbol vectors 3 and 4. The bits
     * from these symbols will be shuffled with the bits from symbols 1 and 2 to
     * create one 256 bit vector per each bit
     */

    // Process symbol vectors
    symbol_abs = _mm256_abs_epi16(symbol34);
    symbol_gt_0 = _mm256_cmpgt_epi16(symbol34, vec_zero);
    symbol_gt_threshold1 = _mm256_cmpgt_epi16(symbol_abs, threshold1);
    symbol_gt_threshold2 = _mm256_cmpgt_epi16(symbol_abs, threshold2);
    symbol_lt_threshold3 = _mm256_cmpgt_epi16(threshold3, symbol_abs);
    symbol_lt_threshold4 = _mm256_cmpgt_epi16(threshold4, symbol_abs);
    symbol_gt_threshold5 = _mm256_cmpgt_epi16(symbol_abs, threshold5);
    symbol_lt_threshold6 = _mm256_cmpgt_epi16(threshold6, symbol_abs);
    symbol_lt_threshold7 = _mm256_cmpgt_epi16(threshold7, symbol_abs);

    // Mask the 0xFFFF values to 0x1
    symbol_gt_0 = _mm256_and_si256(symbol_gt_0, true_mask);
    symbol_gt_threshold1 = _mm256_and_si256(symbol_gt_threshold1, true_mask);
    symbol_gt_threshold2 = _mm256_and_si256(symbol_gt_threshold2, true_mask);
    symbol_lt_threshold3 = _mm256_and_si256(symbol_lt_threshold3, true_mask);
    symbol_lt_threshold4 = _mm256_and_si256(symbol_lt_threshold4, true_mask);
    symbol_gt_threshold5 = _mm256_and_si256(symbol_gt_threshold5, true_mask);
    symbol_lt_threshold6 = _mm256_and_si256(symbol_lt_threshold6, true_mask);
    symbol_lt_threshold7 = _mm256_and_si256(symbol_lt_threshold7, true_mask);

    // Compare thresholds to set bit values
    symbol_bit01 = _mm256_or_si256(
        _mm256_and_si256(symbol_lt_threshold7, symbol_gt_threshold5),
        _mm256_and_si256(symbol_lt_threshold3, symbol_gt_threshold1));
    symbol_bit23 = _mm256_and_si256(symbol_lt_threshold6, symbol_gt_threshold2);
    symbol_bit45 = symbol_lt_threshold4;
    symbol_bit67 = symbol_gt_0;
    // Now, unpack these values into individual bits
    symbol34_bit7 = _mm256_shuffle_epi8(symbol_bit67, shuffle_real);
    symbol34_bit7 = _mm256_permute4x64_epi64(symbol34_bit7, 0xd8);
    symbol34_bit6 = _mm256_shuffle_epi8(symbol_bit67, shuffle_imag);
    symbol34_bit6 = _mm256_permute4x64_epi64(symbol34_bit6, 0xd8);
    symbol34_bit5 = _mm256_shuffle_epi8(symbol_bit45, shuffle_real);
    symbol34_bit5 = _mm256_permute4x64_epi64(symbol34_bit5, 0xd8);
    symbol34_bit4 = _mm256_shuffle_epi8(symbol_bit45, shuffle_imag);
    symbol34_bit4 = _mm256_permute4x64_epi64(symbol34_bit4, 0xd8);
    symbol34_bit3 = _mm256_shuffle_epi8(symbol_bit23, shuffle_real);
    symbol34_bit3 = _mm256_permute4x64_epi64(symbol34_bit3, 0xd8);
    symbol34_bit2 = _mm256_shuffle_epi8(symbol_bit23, shuffle_imag);
    symbol34_bit2 = _mm256_permute4x64_epi64(symbol34_bit2, 0xd8);
    symbol34_bit1 = _mm256_shuffle_epi8(symbol_bit01, shuffle_real);
    symbol34_bit1 = _mm256_permute4x64_epi64(symbol34_bit1, 0xd8);
    symbol34_bit0 = _mm256_shuffle_epi8(symbol_bit01, shuffle_imag);
    symbol34_bit0 = _mm256_permute4x64_epi64(symbol34_bit0, 0xd8);

    /*
     * Finally, we need to repack the result vectors together to create one
     * 256 bit result. We pack the first and second symbol into the lower half
     * of the vector, and third and fourth into upper half
     */
    bit0 = _mm256_permute2x128_si256(symbol12_bit0, symbol34_bit0, 0x20);
    bit1 = _mm256_permute2x128_si256(symbol12_bit1, symbol34_bit1, 0x20);
    bit2 = _mm256_permute2x128_si256(symbol12_bit2, symbol34_bit2, 0x20);
    bit3 = _mm256_permute2x128_si256(symbol12_bit3, symbol34_bit3, 0x20);
    bit4 = _mm256_permute2x128_si256(symbol12_bit4, symbol34_bit4, 0x20);
    bit5 = _mm256_permute2x128_si256(symbol12_bit5, symbol34_bit5, 0x20);
    bit6 = _mm256_permute2x128_si256(symbol12_bit6, symbol34_bit6, 0x20);
    bit7 = _mm256_permute2x128_si256(symbol12_bit7, symbol34_bit7, 0x20);

    // Left shift each bit so we can combine them
    bit1 = _mm256_slli_epi16(bit1, 1);
    bit2 = _mm256_slli_epi16(bit2, 2);
    bit3 = _mm256_slli_epi16(bit3, 3);
    bit4 = _mm256_slli_epi16(bit4, 4);
    bit5 = _mm256_slli_epi16(bit5, 5);
    bit6 = _mm256_slli_epi16(bit6, 6);
    bit7 = _mm256_slli_epi16(bit7, 7);

    // Add each vector to combine them into a single result vector
    result = _mm256_add_epi16(bit0, bit1);
    result = _mm256_add_epi16(result, bit2);
    result = _mm256_add_epi16(result, bit3);
    result = _mm256_add_epi16(result, bit4);
    result = _mm256_add_epi16(result, bit5);
    result = _mm256_add_epi16(result, bit6);
    result = _mm256_add_epi16(result, bit7);

    /// Shuffle the result to pack the 16 bit values into 8 bit ones
    result = _mm256_shuffle_epi8(result, shuffle_16_to_8);
    result = _mm256_permute4x64_epi64(result, 0xd8);
    // Store the final result as a vector of length 128 into the output
    _mm_storeu_si128(result_ptr, _mm256_extracti128_si256(result, 0));
    result_ptr++;
  }
  /*
   * Demodulate symbols that did not fit into multiple of 16 using SSE
   * demodulation function
   */
  next_start = 16 * (num / 16);
  Demod256qamHardSse(vec_in + 2 * next_start, vec_out + next_start,
                     num - next_start);
}

#ifdef __AVX512F__

void Demod256qamHardAvx512(float* vec_in, uint8_t* vec_out, int num) {
  float* symbols_ptr = vec_in;
  auto* result_ptr = reinterpret_cast<__m256i*>(vec_out);
  __m512 symbol1;
  __m512 symbol2;
  __m512 symbol3;
  __m512 symbol4;
  __m512i intsymbol1;
  __m512i intsymbol2;
  __m512i intsymbol3;
  __m512i intsymbol4;
  __m512i symbol12;
  __m512i symbol34;
  __m512i symbol_abs;
  __m512i symbol_gt_0;
  __m512i symbol_gt_threshold1;
  __m512i symbol_gt_threshold2;
  __m512i symbol_lt_threshold3;
  __m512i symbol_lt_threshold4;
  __m512i symbol_gt_threshold5;
  __m512i symbol_lt_threshold6;
  __m512i symbol_lt_threshold7;
  __m512i symbol12_bit0;
  __m512i symbol12_bit1;
  __m512i symbol12_bit2;
  __m512i symbol12_bit3;
  __m512i symbol12_bit4;
  __m512i symbol12_bit5;
  __m512i symbol12_bit6;
  __m512i symbol12_bit7;
  __m512i symbol34_bit0;
  __m512i symbol34_bit1;
  __m512i symbol34_bit2;
  __m512i symbol34_bit3;
  __m512i symbol34_bit4;
  __m512i symbol34_bit5;
  __m512i symbol34_bit6;
  __m512i symbol34_bit7;
  __m512i bit7;
  __m512i bit6;
  __m512i bit5;
  __m512i bit4;
  __m512i bit3;
  __m512i bit2;
  __m512i bit1;
  __m512i bit0;
  __m512i result;
  __m512i symbol_bit01;
  __m512i symbol_bit23;
  __m512i symbol_bit45;
  __m512i symbol_bit67;
  int next_start;

  /*
   * Note: SCALE_BYTE_CONV_QAM16 factor here is simply a way to force all
   * symbols to be in integer range so they can be packed into 16 bit integers
   */
  __m512 scale_factor = _mm512_set1_ps(SCALE_BYTE_CONV_QAM256);
  __m512i vec_zero = _mm512_set1_epi16(0);
  __m512i threshold1 =
      _mm512_set1_epi16(QAM256_THRESHOLD_1 * SCALE_BYTE_CONV_QAM256);
  __m512i threshold2 =
      _mm512_set1_epi16(QAM256_THRESHOLD_2 * SCALE_BYTE_CONV_QAM256);
  __m512i threshold3 =
      _mm512_set1_epi16(QAM256_THRESHOLD_3 * SCALE_BYTE_CONV_QAM256);
  __m512i threshold4 =
      _mm512_set1_epi16(QAM256_THRESHOLD_4 * SCALE_BYTE_CONV_QAM256);
  __m512i threshold5 =
      _mm512_set1_epi16(QAM256_THRESHOLD_5 * SCALE_BYTE_CONV_QAM256);
  __m512i threshold6 =
      _mm512_set1_epi16(QAM256_THRESHOLD_6 * SCALE_BYTE_CONV_QAM256);
  __m512i threshold7 =
      _mm512_set1_epi16(QAM256_THRESHOLD_7 * SCALE_BYTE_CONV_QAM256);
  __m512i true_mask = _mm512_set1_epi16(0x1);

  /*
  __m512i shuffle_real = _mm512_set_epi16(
    30, 28, 26, 24, 22, 20, 18, 16, 14, 12, 10, 8, 6, 4, 2, 0,
    30, 28, 26, 24, 22, 20, 18, 16, 14, 12, 10, 8, 6, 4, 2, 0);

  __m512i shuffle_imag = _mm512_set_epi16(
    31, 29, 27, 25, 23, 21, 19, 17, 15, 14, 11, 10, 7, 6, 3, 2,
    31, 29, 27, 25, 23, 21, 19, 17, 15, 14, 11, 10, 7, 6, 3, 2);

  __m512i combine_v = _mm512_set_epi16(
    47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 32,
    15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0);

  __m512i shuffle_16_to_8 = _mm512_set_epi8(
    62, 60, 58, 56, 54, 52, 50, 48, 46, 44, 42, 40, 38, 36, 34, 32,
    30, 28, 26, 24, 22, 20, 18, 16, 14, 12, 10, 8, 6, 4, 2, 0,
    62, 60, 58, 56, 54, 52, 50, 48, 46, 44, 42, 40, 38, 36, 34, 32,
    30, 28, 26, 24, 22, 20, 18, 16, 14, 12, 10, 8, 6, 4, 2, 0);
  */

  /*
   * _mm512_set_epi16 and _mm512_set_epi8 are not available, so use this as
   * an alternative
   */
  __m512i shuffle_real = _mm512_set_epi32(
      (30 << 16) + 28, (26 << 16) + 24, (22 << 16) + 20, (18 << 16) + 16,
      (14 << 16) + 12, (10 << 16) + 8, (6 << 16) + 4, (2 << 16) + 0,
      (30 << 16) + 28, (26 << 16) + 24, (22 << 16) + 20, (18 << 16) + 16,
      (14 << 16) + 12, (10 << 16) + 8, (6 << 16) + 4, (2 << 16) + 0);

  __m512i shuffle_imag = _mm512_set_epi32(
      (31 << 16) + 29, (27 << 16) + 25, (23 << 16) + 21, (19 << 16) + 17,
      (15 << 16) + 13, (11 << 16) + 9, (7 << 16) + 5, (3 << 16) + 1,
      (31 << 16) + 29, (27 << 16) + 25, (23 << 16) + 21, (19 << 16) + 17,
      (15 << 16) + 13, (11 << 16) + 9, (7 << 16) + 5, (3 << 16) + 1);

  __m512i combine_v = _mm512_set_epi32(
      (47 << 16) + 46, (45 << 16) + 44, (43 << 16) + 42, (41 << 16) + 40,
      (39 << 16) + 38, (37 << 16) + 36, (35 << 16) + 34, (33 << 16) + 32,
      (15 << 16) + 14, (13 << 16) + 12, (11 << 16) + 10, (9 << 16) + 8,
      (7 << 16) + 6, (5 << 16) + 4, (3 << 16) + 2, (1 << 16) + 0);

  __m512i shuffle_16_to_8 = _mm512_set_epi32(
      0xffffffff, 0xffffffff, (14 << 24) + (12 << 16) + (10 << 8) + 8,
      (6 << 24) + (4 << 16) + (2 << 8) + 0, 0xffffffff, 0xffffffff,
      (14 << 24) + (12 << 16) + (10 << 8) + 8,
      (6 << 24) + (4 << 16) + (2 << 8) + 0, 0xffffffff, 0xffffffff,
      (14 << 24) + (12 << 16) + (10 << 8) + 8,
      (6 << 24) + (4 << 16) + (2 << 8) + 0, 0xffffffff, 0xffffffff,
      (14 << 24) + (12 << 16) + (10 << 8) + 8,
      (6 << 24) + (4 << 16) + (2 << 8) + 0);

  __m512i fix_pack = _mm512_set_epi64(7, 5, 3, 1, 6, 4, 2, 0);

  for (int i = 0; i < num / 32; i++) {
    // Load symbols, 8 per vector
    symbol1 = _mm512_load_ps(symbols_ptr);
    symbols_ptr += 16;
    symbol2 = _mm512_load_ps(symbols_ptr);
    symbols_ptr += 16;
    symbol3 = _mm512_load_ps(symbols_ptr);
    symbols_ptr += 16;
    symbol4 = _mm512_load_ps(symbols_ptr);
    symbols_ptr += 16;

    // Scale symbols by fixed value, and truncate into 32 bit integers
    intsymbol1 = _mm512_cvtps_epi32(_mm512_mul_ps(symbol1, scale_factor));
    intsymbol2 = _mm512_cvtps_epi32(_mm512_mul_ps(symbol2, scale_factor));
    intsymbol3 = _mm512_cvtps_epi32(_mm512_mul_ps(symbol3, scale_factor));
    intsymbol4 = _mm512_cvtps_epi32(_mm512_mul_ps(symbol4, scale_factor));

    // Pack symbols into 2 blocks of 16 bit integers
    symbol12 = _mm512_packs_epi32(intsymbol1, intsymbol2);
    /*
     * _mm512_packs_epi32 interleaves the values of both symbols,
     * so permute them back to be in order
     */
    symbol12 = _mm512_permutexvar_epi64(fix_pack, symbol12);
    symbol34 = _mm512_packs_epi32(intsymbol3, intsymbol4);
    symbol34 = _mm512_permutexvar_epi64(fix_pack, symbol34);

    /*
     * Process symbol vectors. We need to take the absolute value of the
     * symbols, then compare them to each threshold (including zero)
     * If the threshold value comparision is true, the 16 bit value will hold
     * 0x1
     */
    symbol_abs = _mm512_abs_epi16(symbol12);
    symbol_gt_0 = _mm512_maskz_abs_epi16(
        _mm512_cmpgt_epi16_mask(symbol12, vec_zero), true_mask);
    symbol_gt_threshold1 = _mm512_maskz_abs_epi16(
        _mm512_cmpgt_epi16_mask(symbol_abs, threshold1), true_mask);
    symbol_gt_threshold2 = _mm512_maskz_abs_epi16(
        _mm512_cmpgt_epi16_mask(symbol_abs, threshold2), true_mask);
    symbol_lt_threshold3 = _mm512_maskz_abs_epi16(
        _mm512_cmpgt_epi16_mask(threshold3, symbol_abs), true_mask);
    symbol_lt_threshold4 = _mm512_maskz_abs_epi16(
        _mm512_cmpgt_epi16_mask(threshold4, symbol_abs), true_mask);
    symbol_gt_threshold5 = _mm512_maskz_abs_epi16(
        _mm512_cmpgt_epi16_mask(symbol_abs, threshold5), true_mask);
    symbol_lt_threshold6 = _mm512_maskz_abs_epi16(
        _mm512_cmpgt_epi16_mask(threshold6, symbol_abs), true_mask);
    symbol_lt_threshold7 = _mm512_maskz_abs_epi16(
        _mm512_cmpgt_epi16_mask(threshold7, symbol_abs), true_mask);
    /*
     * Now determine the value of each bit. The value of each bit is based off
     * the values stored in each comparision output, and how those relate to
     * the actual QAM table, given in the docstring for Demod256qamHardLoop.
     * see that function for additional explanation of how each threshold
     * influences which bits should be set.
     */
    /*
     * First, we determine the value of bits together, since the imaginary and
     * real symbols have similar conditions
     */
    symbol_bit01 = _mm512_or_si512(
        _mm512_and_si512(symbol_lt_threshold7, symbol_gt_threshold5),
        _mm512_and_si512(symbol_lt_threshold3, symbol_gt_threshold1));
    symbol_bit23 = _mm512_and_si512(symbol_lt_threshold6, symbol_gt_threshold2);
    symbol_bit45 = symbol_lt_threshold4;
    symbol_bit67 = symbol_gt_0;
    /*
     * Now, unpack these values into individual bits
     * Note that the upper 256 bits of these vectors will be a copy of the
     * values in the lower 256 bits
     */
    symbol12_bit7 = _mm512_permutexvar_epi16(shuffle_real, symbol_bit67);
    symbol12_bit6 = _mm512_permutexvar_epi16(shuffle_imag, symbol_bit67);
    symbol12_bit5 = _mm512_permutexvar_epi16(shuffle_real, symbol_bit45);
    symbol12_bit4 = _mm512_permutexvar_epi16(shuffle_imag, symbol_bit45);
    symbol12_bit3 = _mm512_permutexvar_epi16(shuffle_real, symbol_bit23);
    symbol12_bit2 = _mm512_permutexvar_epi16(shuffle_imag, symbol_bit23);
    symbol12_bit1 = _mm512_permutexvar_epi16(shuffle_real, symbol_bit01);
    symbol12_bit0 = _mm512_permutexvar_epi16(shuffle_imag, symbol_bit01);

    /*
     * We now repeat the above process for the symbol vectors 3 and 4. The bits
     * from these symbols will be shuffled with the bits from symbols 1 and 2 to
     * create one 512 bit vector per each bit
     */

    // Process symbol vectors
    symbol_abs = _mm512_abs_epi16(symbol34);
    symbol_gt_0 = _mm512_maskz_abs_epi16(
        _mm512_cmpgt_epi16_mask(symbol34, vec_zero), true_mask);
    symbol_gt_threshold1 = _mm512_maskz_abs_epi16(
        _mm512_cmpgt_epi16_mask(symbol_abs, threshold1), true_mask);
    symbol_gt_threshold2 = _mm512_maskz_abs_epi16(
        _mm512_cmpgt_epi16_mask(symbol_abs, threshold2), true_mask);
    symbol_lt_threshold3 = _mm512_maskz_abs_epi16(
        _mm512_cmpgt_epi16_mask(threshold3, symbol_abs), true_mask);
    symbol_lt_threshold4 = _mm512_maskz_abs_epi16(
        _mm512_cmpgt_epi16_mask(threshold4, symbol_abs), true_mask);
    symbol_gt_threshold5 = _mm512_maskz_abs_epi16(
        _mm512_cmpgt_epi16_mask(symbol_abs, threshold5), true_mask);
    symbol_lt_threshold6 = _mm512_maskz_abs_epi16(
        _mm512_cmpgt_epi16_mask(threshold6, symbol_abs), true_mask);
    symbol_lt_threshold7 = _mm512_maskz_abs_epi16(
        _mm512_cmpgt_epi16_mask(threshold7, symbol_abs), true_mask);

    // Compare thresholds to set bit values
    symbol_bit01 = _mm512_or_si512(
        _mm512_and_si512(symbol_lt_threshold7, symbol_gt_threshold5),
        _mm512_and_si512(symbol_lt_threshold3, symbol_gt_threshold1));
    symbol_bit23 = _mm512_and_si512(symbol_lt_threshold6, symbol_gt_threshold2);
    symbol_bit45 = symbol_lt_threshold4;
    symbol_bit67 = symbol_gt_0;
    // Now, unpack these values into individual bits
    symbol34_bit7 = _mm512_permutexvar_epi16(shuffle_real, symbol_bit67);
    symbol34_bit6 = _mm512_permutexvar_epi16(shuffle_imag, symbol_bit67);
    symbol34_bit5 = _mm512_permutexvar_epi16(shuffle_real, symbol_bit45);
    symbol34_bit4 = _mm512_permutexvar_epi16(shuffle_imag, symbol_bit45);
    symbol34_bit3 = _mm512_permutexvar_epi16(shuffle_real, symbol_bit23);
    symbol34_bit2 = _mm512_permutexvar_epi16(shuffle_imag, symbol_bit23);
    symbol34_bit1 = _mm512_permutexvar_epi16(shuffle_real, symbol_bit01);
    symbol34_bit0 = _mm512_permutexvar_epi16(shuffle_imag, symbol_bit01);

    /*
     * Finally, we need to repack the result vectors together to create one
     * 256 bit result. We pack the first and second symbol into the lower half
     * of the vector, and third and fourth into upper half
     */
    bit0 = _mm512_permutex2var_epi16(symbol12_bit0, combine_v, symbol34_bit0);
    bit1 = _mm512_permutex2var_epi16(symbol12_bit1, combine_v, symbol34_bit1);
    bit2 = _mm512_permutex2var_epi16(symbol12_bit2, combine_v, symbol34_bit2);
    bit3 = _mm512_permutex2var_epi16(symbol12_bit3, combine_v, symbol34_bit3);
    bit4 = _mm512_permutex2var_epi16(symbol12_bit4, combine_v, symbol34_bit4);
    bit5 = _mm512_permutex2var_epi16(symbol12_bit5, combine_v, symbol34_bit5);
    bit6 = _mm512_permutex2var_epi16(symbol12_bit6, combine_v, symbol34_bit6);
    bit7 = _mm512_permutex2var_epi16(symbol12_bit7, combine_v, symbol34_bit7);

    // Left shift each bit so we can combine them
    bit1 = _mm512_slli_epi16(bit1, 1);
    bit2 = _mm512_slli_epi16(bit2, 2);
    bit3 = _mm512_slli_epi16(bit3, 3);
    bit4 = _mm512_slli_epi16(bit4, 4);
    bit5 = _mm512_slli_epi16(bit5, 5);
    bit6 = _mm512_slli_epi16(bit6, 6);
    bit7 = _mm512_slli_epi16(bit7, 7);

    // Add each vector to combine them into a single result vector
    result = _mm512_add_epi16(bit0, bit1);
    result = _mm512_add_epi16(result, bit2);
    result = _mm512_add_epi16(result, bit3);
    result = _mm512_add_epi16(result, bit4);
    result = _mm512_add_epi16(result, bit5);
    result = _mm512_add_epi16(result, bit6);
    result = _mm512_add_epi16(result, bit7);

    /// Shuffle the result to pack the 16 bit values into 8 bit ones
    result = _mm512_shuffle_epi8(result, shuffle_16_to_8);
    // Pack the result of the shuffle into the lower 256 bits
    result = _mm512_permutexvar_epi64(fix_pack, result);
    // Store the final result as a vector of length 128 into the output
    _mm256_storeu_si256(result_ptr, _mm512_extracti32x8_epi32(result, 0));
    result_ptr++;
  }
  /*
   * Demodulate symbols that did not fit into multiple of 16 using SSE
   * demodulation function
   */
  next_start = 32 * (num / 32);
  Demod256qamHardAvx2(vec_in + 2 * next_start, vec_out + next_start,
                      num - next_start);
}

#endif

void Demod256qamSoftLoop(const float* vec_in, int8_t* llr, int num) {
  /**
   * LLR algorithm derived from paper:
   * Q. Sun and W. Qi,
   * "Soft-demodulation algorithm for 64QAM and it's application in HSPA+,"
   * 2012 IEEE 11th International Conference on Signal Processing, 2012,
   * pp. 2309-2312, doi: 10.1109/ICoSP.2012.6492042.
   *
   * Equations 7, 8, and 9
   */
  int i;
  int8_t re;
  int8_t im;
  const uint8_t t1 = QAM256_THRESHOLD_4 * SCALE_BYTE_CONV_QAM256;
  const uint8_t t2 = QAM256_THRESHOLD_2 * SCALE_BYTE_CONV_QAM256;
  const uint8_t t3 = QAM256_THRESHOLD_1 * SCALE_BYTE_CONV_QAM256;
  for (i = 0; i < num; i++) {
    re = (int8_t)(SCALE_BYTE_CONV_QAM256 * (vec_in[2 * i]));
    im = (int8_t)(SCALE_BYTE_CONV_QAM256 * (vec_in[2 * i + 1]));

    // Upper two bits simply use real and imaginary values
    llr[8 * i + 0] = re;
    llr[8 * i + 1] = im;
    // Next two bits use the absolute value of the prior 2 and a threshold
    llr[8 * i + 2] = t1 - abs(re);
    llr[8 * i + 3] = t1 - abs(im);
    // Once again, calculate the LLR recursively using boundary judgement method
    llr[8 * i + 4] = t2 - abs(llr[8 * i + 2]);
    llr[8 * i + 5] = t2 - abs(llr[8 * i + 3]);
    /**
     * Same pattern for the final bits. Note that the threshold that is
     * subtracted from is based on the threshold across which the bits revelant
     * to the current LLR flip in the constellation
     *
     * In addition, the prior LLR that is used essentially gives the distance
     * from a given threshold. For example abs(llr[8 * i + 4]) gives the
     * distance the symbol is from either QAM256_THRESHOLD_2 or
     * QAM256_THRESHOLD_6, whichever is closest.
     */
    llr[8 * i + 6] = t3 - abs(llr[8 * i + 4]);
    llr[8 * i + 7] = t3 - abs(llr[8 * i + 5]);
  }
}

void Demod256qamSoftSse(const float* vec_in, int8_t* llr, int num) {
  float* symbols_ptr = (float*)vec_in;
  auto* result_ptr = reinterpret_cast<__m128i*>(llr);
  __m128 symbol1;
  __m128 symbol2;
  __m128 symbol3;
  __m128 symbol4;
  __m128i symbol_i1;
  __m128i symbol_i2;
  __m128i symbol_i3;
  __m128i symbol_i4;
  __m128i symbol_i;
  __m128i symbol_bit54;
  __m128i symbol_bit32;
  __m128i symbol_bit10;
  __m128i symbol_12;
  __m128i symbol_34;
  __m128i offset1 = _mm_set1_epi8(QAM256_THRESHOLD_1 * SCALE_BYTE_CONV_QAM256);
  __m128i offset2 = _mm_set1_epi8(QAM256_THRESHOLD_2 * SCALE_BYTE_CONV_QAM256);
  __m128i offset3 = _mm_set1_epi8(QAM256_THRESHOLD_4 * SCALE_BYTE_CONV_QAM256);
  __m128 scale_v = _mm_set1_ps(SCALE_BYTE_CONV_QAM256);
  __m128i result10;
  __m128i result32;
  __m128i result54;
  __m128i result76;

  __m128i shuffle_bit76_1 =
      _mm_set_epi8(0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 3, 2, 0xff, 0xff, 0xff,
                   0xff, 0xff, 0xff, 1, 0);
  __m128i shuffle_bit76_2 =
      _mm_set_epi8(0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 7, 6, 0xff, 0xff, 0xff,
                   0xff, 0xff, 0xff, 5, 4);
  __m128i shuffle_bit76_3 =
      _mm_set_epi8(0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 11, 10, 0xff, 0xff, 0xff,
                   0xff, 0xff, 0xff, 9, 8);
  __m128i shuffle_bit76_4 =
      _mm_set_epi8(0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 15, 14, 0xff, 0xff, 0xff,
                   0xff, 0xff, 0xff, 13, 12);

  __m128i shuffle_bit54_1 =
      _mm_set_epi8(0xff, 0xff, 0xff, 0xff, 3, 2, 0xff, 0xff, 0xff, 0xff, 0xff,
                   0xff, 1, 0, 0xff, 0xff);

  __m128i shuffle_bit54_2 =
      _mm_set_epi8(0xff, 0xff, 0xff, 0xff, 7, 6, 0xff, 0xff, 0xff, 0xff, 0xff,
                   0xff, 5, 4, 0xff, 0xff);

  __m128i shuffle_bit54_3 =
      _mm_set_epi8(0xff, 0xff, 0xff, 0xff, 11, 10, 0xff, 0xff, 0xff, 0xff, 0xff,
                   0xff, 9, 8, 0xff, 0xff);

  __m128i shuffle_bit54_4 =
      _mm_set_epi8(0xff, 0xff, 0xff, 0xff, 15, 14, 0xff, 0xff, 0xff, 0xff, 0xff,
                   0xff, 13, 12, 0xff, 0xff);

  __m128i shuffle_bit32_1 =
      _mm_set_epi8(0xff, 0xff, 3, 2, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 1, 0,
                   0xff, 0xff, 0xff, 0xff);

  __m128i shuffle_bit32_2 =
      _mm_set_epi8(0xff, 0xff, 7, 6, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 5, 4,
                   0xff, 0xff, 0xff, 0xff);

  __m128i shuffle_bit32_3 =
      _mm_set_epi8(0xff, 0xff, 11, 10, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 9, 8,
                   0xff, 0xff, 0xff, 0xff);

  __m128i shuffle_bit32_4 =
      _mm_set_epi8(0xff, 0xff, 15, 14, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 13,
                   12, 0xff, 0xff, 0xff, 0xff);

  __m128i shuffle_bit10_1 =
      _mm_set_epi8(3, 2, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 1, 0, 0xff, 0xff,
                   0xff, 0xff, 0xff, 0xff);

  __m128i shuffle_bit10_2 =
      _mm_set_epi8(7, 6, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 5, 4, 0xff, 0xff,
                   0xff, 0xff, 0xff, 0xff);

  __m128i shuffle_bit10_3 =
      _mm_set_epi8(11, 10, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 9, 8, 0xff, 0xff,
                   0xff, 0xff, 0xff, 0xff);

  __m128i shuffle_bit10_4 =
      _mm_set_epi8(15, 14, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 13, 12, 0xff,
                   0xff, 0xff, 0xff, 0xff, 0xff);

  for (int i = 0; i < num / 8; i++) {
    symbol1 = _mm_load_ps(symbols_ptr);
    symbols_ptr += 4;
    symbol2 = _mm_load_ps(symbols_ptr);
    symbols_ptr += 4;
    symbol3 = _mm_load_ps(symbols_ptr);
    symbols_ptr += 4;
    symbol4 = _mm_load_ps(symbols_ptr);
    symbols_ptr += 4;
    symbol_i1 = _mm_cvtps_epi32(_mm_mul_ps(symbol1, scale_v));
    symbol_i2 = _mm_cvtps_epi32(_mm_mul_ps(symbol2, scale_v));
    symbol_i3 = _mm_cvtps_epi32(_mm_mul_ps(symbol3, scale_v));
    symbol_i4 = _mm_cvtps_epi32(_mm_mul_ps(symbol4, scale_v));
    symbol_12 = _mm_packs_epi32(symbol_i1, symbol_i2);
    symbol_34 = _mm_packs_epi32(symbol_i3, symbol_i4);
    symbol_i = _mm_packs_epi16(symbol_12, symbol_34);

    /*
     * This math is where the LLR occurs. Note that we use the
     * absolute value of the prior vector for the next subtraction,
     * like in the traditional LLR
     */
    symbol_bit54 = _mm_sub_epi8(offset3, _mm_abs_epi8(symbol_i));
    symbol_bit32 = _mm_sub_epi8(offset2, _mm_abs_epi8(symbol_bit54));
    symbol_bit10 = _mm_sub_epi8(offset1, _mm_abs_epi8(symbol_bit32));

    /*
     * Now extract the result and store it. We must do this 4 times due
     * to the capacity of a 128 bit vector
     */
    result10 = _mm_shuffle_epi8(symbol_bit10, shuffle_bit10_1);
    result32 = _mm_shuffle_epi8(symbol_bit32, shuffle_bit32_1);
    result54 = _mm_shuffle_epi8(symbol_bit54, shuffle_bit54_1);
    result76 = _mm_shuffle_epi8(symbol_i, shuffle_bit76_1);
    // Store the first set of results.
    _mm_store_si128(
        result_ptr,
        _mm_or_si128(_mm_or_si128(_mm_or_si128(result10, result32), result54),
                     result76));
    result_ptr++;
    // Extract and store the second set of results.
    result10 = _mm_shuffle_epi8(symbol_bit10, shuffle_bit10_2);
    result32 = _mm_shuffle_epi8(symbol_bit32, shuffle_bit32_2);
    result54 = _mm_shuffle_epi8(symbol_bit54, shuffle_bit54_2);
    result76 = _mm_shuffle_epi8(symbol_i, shuffle_bit76_2);
    _mm_store_si128(
        result_ptr,
        _mm_or_si128(_mm_or_si128(_mm_or_si128(result10, result32), result54),
                     result76));
    result_ptr++;
    // Third set
    result10 = _mm_shuffle_epi8(symbol_bit10, shuffle_bit10_3);
    result32 = _mm_shuffle_epi8(symbol_bit32, shuffle_bit32_3);
    result54 = _mm_shuffle_epi8(symbol_bit54, shuffle_bit54_3);
    result76 = _mm_shuffle_epi8(symbol_i, shuffle_bit76_3);
    _mm_store_si128(
        result_ptr,
        _mm_or_si128(_mm_or_si128(_mm_or_si128(result10, result32), result54),
                     result76));
    result_ptr++;
    // Fourth and final set.
    result10 = _mm_shuffle_epi8(symbol_bit10, shuffle_bit10_4);
    result32 = _mm_shuffle_epi8(symbol_bit32, shuffle_bit32_4);
    result54 = _mm_shuffle_epi8(symbol_bit54, shuffle_bit54_4);
    result76 = _mm_shuffle_epi8(symbol_i, shuffle_bit76_4);
    _mm_store_si128(
        result_ptr,
        _mm_or_si128(_mm_or_si128(_mm_or_si128(result10, result32), result54),
                     result76));
    result_ptr++;
  }
  // Demodulate the last symbols
  int next_start = 8 * (num / 8);
  Demod256qamSoftLoop(vec_in + 2 * next_start, llr + next_start * 8,
                      num - next_start);
}

void Demod256qamSoftAvx2(const float* vec_in, int8_t* llr, int num) {
  float* symbols_ptr = (float*)vec_in;
  auto* result_ptr = reinterpret_cast<__m256i*>(llr);
  __m256 symbol1;
  __m256 symbol2;
  __m256 symbol3;
  __m256 symbol4;
  __m256i symbol_i1;
  __m256i symbol_i2;
  __m256i symbol_i3;
  __m256i symbol_i4;
  __m256i symbol_i;
  __m256i symbol_bit54;
  __m256i symbol_bit32;
  __m256i symbol_bit10;
  __m256i symbol_12;
  __m256i symbol_34;
  __m256i offset1 =
      _mm256_set1_epi8(QAM256_THRESHOLD_1 * SCALE_BYTE_CONV_QAM256);
  __m256i offset2 =
      _mm256_set1_epi8(QAM256_THRESHOLD_2 * SCALE_BYTE_CONV_QAM256);
  __m256i offset3 =
      _mm256_set1_epi8(QAM256_THRESHOLD_4 * SCALE_BYTE_CONV_QAM256);
  __m256 scale_v = _mm256_set1_ps(SCALE_BYTE_CONV_QAM256);
  __m256i result10;
  __m256i result32;
  __m256i result54;
  __m256i result76;
  __m256i result_final1;
  __m256i result_final2;
  __m256i result_final3;
  __m256i result_final4;

  __m256i shuffle_bit76_1 = _mm256_set_epi8(
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 3, 2, 0xff, 0xff, 0xff, 0xff, 0xff,
      0xff, 1, 0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 3, 2, 0xff, 0xff, 0xff,
      0xff, 0xff, 0xff, 1, 0);
  __m256i shuffle_bit76_2 = _mm256_set_epi8(
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 7, 6, 0xff, 0xff, 0xff, 0xff, 0xff,
      0xff, 5, 4, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 7, 6, 0xff, 0xff, 0xff,
      0xff, 0xff, 0xff, 5, 4);
  __m256i shuffle_bit76_3 = _mm256_set_epi8(
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 11, 10, 0xff, 0xff, 0xff, 0xff, 0xff,
      0xff, 9, 8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 11, 10, 0xff, 0xff, 0xff,
      0xff, 0xff, 0xff, 9, 8);
  __m256i shuffle_bit76_4 = _mm256_set_epi8(
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 15, 14, 0xff, 0xff, 0xff, 0xff, 0xff,
      0xff, 13, 12, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 15, 14, 0xff, 0xff,
      0xff, 0xff, 0xff, 0xff, 13, 12);

  __m256i shuffle_bit54_1 =
      _mm256_set_epi8(0xff, 0xff, 0xff, 0xff, 3, 2, 0xff, 0xff, 0xff, 0xff,
                      0xff, 0xff, 1, 0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 3,
                      2, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 1, 0, 0xff, 0xff);
  __m256i shuffle_bit54_2 =
      _mm256_set_epi8(0xff, 0xff, 0xff, 0xff, 7, 6, 0xff, 0xff, 0xff, 0xff,
                      0xff, 0xff, 5, 4, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 7,
                      6, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 5, 4, 0xff, 0xff);
  __m256i shuffle_bit54_3 =
      _mm256_set_epi8(0xff, 0xff, 0xff, 0xff, 11, 10, 0xff, 0xff, 0xff, 0xff,
                      0xff, 0xff, 9, 8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 11,
                      10, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 9, 8, 0xff, 0xff);
  __m256i shuffle_bit54_4 = _mm256_set_epi8(
      0xff, 0xff, 0xff, 0xff, 15, 14, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 13,
      12, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 15, 14, 0xff, 0xff, 0xff, 0xff,
      0xff, 0xff, 13, 12, 0xff, 0xff);

  __m256i shuffle_bit32_1 =
      _mm256_set_epi8(0xff, 0xff, 3, 2, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 1,
                      0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 3, 2, 0xff, 0xff,
                      0xff, 0xff, 0xff, 0xff, 1, 0, 0xff, 0xff, 0xff, 0xff);
  __m256i shuffle_bit32_2 =
      _mm256_set_epi8(0xff, 0xff, 7, 6, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 5,
                      4, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 7, 6, 0xff, 0xff,
                      0xff, 0xff, 0xff, 0xff, 5, 4, 0xff, 0xff, 0xff, 0xff);
  __m256i shuffle_bit32_3 =
      _mm256_set_epi8(0xff, 0xff, 11, 10, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 9,
                      8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 11, 10, 0xff, 0xff,
                      0xff, 0xff, 0xff, 0xff, 9, 8, 0xff, 0xff, 0xff, 0xff);
  __m256i shuffle_bit32_4 = _mm256_set_epi8(
      0xff, 0xff, 15, 14, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 13, 12, 0xff,
      0xff, 0xff, 0xff, 0xff, 0xff, 15, 14, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
      13, 12, 0xff, 0xff, 0xff, 0xff);

  __m256i shuffle_bit10_1 = _mm256_set_epi8(
      3, 2, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 1, 0, 0xff, 0xff, 0xff, 0xff,
      0xff, 0xff, 3, 2, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 1, 0, 0xff, 0xff,
      0xff, 0xff, 0xff, 0xff);
  __m256i shuffle_bit10_2 = _mm256_set_epi8(
      7, 6, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 5, 4, 0xff, 0xff, 0xff, 0xff,
      0xff, 0xff, 7, 6, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 5, 4, 0xff, 0xff,
      0xff, 0xff, 0xff, 0xff);
  __m256i shuffle_bit10_3 = _mm256_set_epi8(
      11, 10, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 9, 8, 0xff, 0xff, 0xff, 0xff,
      0xff, 0xff, 11, 10, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 9, 8, 0xff, 0xff,
      0xff, 0xff, 0xff, 0xff);
  __m256i shuffle_bit10_4 = _mm256_set_epi8(
      15, 14, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 13, 12, 0xff, 0xff, 0xff,
      0xff, 0xff, 0xff, 15, 14, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 13, 12,
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff);

  for (int i = 0; i < num / 16; i++) {
    symbol1 = _mm256_load_ps(symbols_ptr);
    symbols_ptr += 8;
    symbol2 = _mm256_load_ps(symbols_ptr);
    symbols_ptr += 8;
    symbol3 = _mm256_load_ps(symbols_ptr);
    symbols_ptr += 8;
    symbol4 = _mm256_load_ps(symbols_ptr);
    symbols_ptr += 8;
    symbol_i1 = _mm256_cvtps_epi32(_mm256_mul_ps(symbol1, scale_v));
    symbol_i2 = _mm256_cvtps_epi32(_mm256_mul_ps(symbol2, scale_v));
    symbol_i3 = _mm256_cvtps_epi32(_mm256_mul_ps(symbol3, scale_v));
    symbol_i4 = _mm256_cvtps_epi32(_mm256_mul_ps(symbol4, scale_v));
    // Pack symbols into 16 bit integers
    symbol_12 = _mm256_packs_epi32(symbol_i1, symbol_i2);
    // _packs intrinsic interleaves the two vectors, _permute fixes that
    symbol_12 = _mm256_permute4x64_epi64(symbol_12, 0xd8);
    symbol_34 = _mm256_packs_epi32(symbol_i3, symbol_i4);
    symbol_34 = _mm256_permute4x64_epi64(symbol_34, 0xd8);
    // Pack symbols into 8 bit integers (one 256 bit vector)
    symbol_i = _mm256_packs_epi16(symbol_12, symbol_34);
    symbol_i = _mm256_permute4x64_epi64(symbol_i, 0xd8);

    /*
     * This math is where the LLR occurs. Note that we use the
     * absolute value of the prior vector for the next subtraction,
     * like in the traditional LLR
     */
    symbol_bit54 = _mm256_sub_epi8(offset3, _mm256_abs_epi8(symbol_i));
    symbol_bit32 = _mm256_sub_epi8(offset2, _mm256_abs_epi8(symbol_bit54));
    symbol_bit10 = _mm256_sub_epi8(offset1, _mm256_abs_epi8(symbol_bit32));

    /*
     * Now extract the result and store it. We must do this 4 times due
     * to the capacity of a 256 bit vector
     */
    result10 = _mm256_shuffle_epi8(symbol_bit10, shuffle_bit10_1);
    result32 = _mm256_shuffle_epi8(symbol_bit32, shuffle_bit32_1);
    result54 = _mm256_shuffle_epi8(symbol_bit54, shuffle_bit54_1);
    result76 = _mm256_shuffle_epi8(symbol_i, shuffle_bit76_1);
    // Store the result
    result_final1 = _mm256_or_si256(
        _mm256_or_si256(_mm256_or_si256(result76, result54), result32),
        result10);

    // Extract and store the second set of results.
    result10 = _mm256_shuffle_epi8(symbol_bit10, shuffle_bit10_2);
    result32 = _mm256_shuffle_epi8(symbol_bit32, shuffle_bit32_2);
    result54 = _mm256_shuffle_epi8(symbol_bit54, shuffle_bit54_2);
    result76 = _mm256_shuffle_epi8(symbol_i, shuffle_bit76_2);
    // Store the result
    result_final2 = _mm256_or_si256(
        _mm256_or_si256(_mm256_or_si256(result76, result54), result32),
        result10);
    // Third set
    result10 = _mm256_shuffle_epi8(symbol_bit10, shuffle_bit10_3);
    result32 = _mm256_shuffle_epi8(symbol_bit32, shuffle_bit32_3);
    result54 = _mm256_shuffle_epi8(symbol_bit54, shuffle_bit54_3);
    result76 = _mm256_shuffle_epi8(symbol_i, shuffle_bit76_3);
    // Store the result
    result_final3 = _mm256_or_si256(
        _mm256_or_si256(_mm256_or_si256(result76, result54), result32),
        result10);
    // Fourth and final set.
    result10 = _mm256_shuffle_epi8(symbol_bit10, shuffle_bit10_4);
    result32 = _mm256_shuffle_epi8(symbol_bit32, shuffle_bit32_4);
    result54 = _mm256_shuffle_epi8(symbol_bit54, shuffle_bit54_4);
    result76 = _mm256_shuffle_epi8(symbol_i, shuffle_bit76_4);
    // Store the result
    result_final4 = _mm256_or_si256(
        _mm256_or_si256(_mm256_or_si256(result76, result54), result32),
        result10);
    /*
     * Now, unshuffle the result vectors, and store them to memory
     */
    _mm256_storeu_si256(result_ptr, _mm256_permute2x128_si256(
                                        result_final1, result_final2, 0x20));
    result_ptr++;
    _mm256_storeu_si256(result_ptr, _mm256_permute2x128_si256(
                                        result_final3, result_final4, 0x20));
    result_ptr++;
    _mm256_storeu_si256(result_ptr, _mm256_permute2x128_si256(
                                        result_final1, result_final2, 0x31));
    result_ptr++;
    _mm256_storeu_si256(result_ptr, _mm256_permute2x128_si256(
                                        result_final3, result_final4, 0x31));
    result_ptr++;
  }
  // Demodulate the last symbols
  int next_start = 16 * (num / 16);
  Demod256qamSoftSse(vec_in + 2 * next_start, llr + next_start * 8,
                     num - next_start);
}

#ifdef __AVX512F__
void Demod256qamSoftAvx512(const float* vec_in, int8_t* llr, int num) {
  float* symbols_ptr = (float*)vec_in;
  auto* result_ptr = reinterpret_cast<__m512i*>(llr);
  __m512 symbol1;
  __m512 symbol2;
  __m512 symbol3;
  __m512 symbol4;
  __m512i symbol_i1;
  __m512i symbol_i2;
  __m512i symbol_i3;
  __m512i symbol_i4;
  __m512i symbol_i;
  __m512i symbol_bit54;
  __m512i symbol_bit32;
  __m512i symbol_bit10;
  __m512i symbol_12;
  __m512i symbol_34;
  __m512i offset1 =
      _mm512_set1_epi8(QAM256_THRESHOLD_1 * SCALE_BYTE_CONV_QAM256);
  __m512i offset2 =
      _mm512_set1_epi8(QAM256_THRESHOLD_2 * SCALE_BYTE_CONV_QAM256);
  __m512i offset3 =
      _mm512_set1_epi8(QAM256_THRESHOLD_4 * SCALE_BYTE_CONV_QAM256);
  __m512 scale_v = _mm512_set1_ps(SCALE_BYTE_CONV_QAM256);
  __m512i result10;
  __m512i result32;
  __m512i result54;
  __m512i result76;
  __m512i result_final;
  __mmask32 bit76_mask = _mm512_movepi16_mask(_mm512_set_epi32(
      0x00000000, 0x0000FFFF, 0x00000000, 0x0000FFFF, 0x00000000, 0x0000FFFF,
      0x00000000, 0x0000FFFF, 0x00000000, 0x0000FFFF, 0x00000000, 0x0000FFFF,
      0x00000000, 0x0000FFFF, 0x00000000, 0x0000FFFF));
  __mmask32 bit54_mask = _mm512_movepi16_mask(_mm512_set_epi32(
      0x00000000, 0xFFFF0000, 0x00000000, 0xFFFF0000, 0x00000000, 0xFFFF0000,
      0x00000000, 0xFFFF0000, 0x00000000, 0xFFFF0000, 0x00000000, 0xFFFF0000,
      0x00000000, 0xFFFF0000, 0x00000000, 0xFFFF0000));
  __mmask32 bit32_mask = _mm512_movepi16_mask(_mm512_set_epi32(
      0x0000FFFF, 0x00000000, 0x0000FFFF, 0x00000000, 0x0000FFFF, 0x00000000,
      0x0000FFFF, 0x00000000, 0x0000FFFF, 0x00000000, 0x0000FFFF, 0x00000000,
      0x0000FFFF, 0x00000000, 0x0000FFFF, 0x00000000));
  __mmask32 bit10_mask = _mm512_movepi16_mask(_mm512_set_epi32(
      0xFFFF0000, 0x00000000, 0xFFFF0000, 0x00000000, 0xFFFF0000, 0x00000000,
      0xFFFF0000, 0x00000000, 0xFFFF0000, 0x00000000, 0xFFFF0000, 0x00000000,
      0xFFFF0000, 0x00000000, 0xFFFF0000, 0x00000000));

  __m512i shuffle_bit76_1 = _mm512_setr_epi64(0, 1, 2, 3, 4, 5, 6, 7);
  __m512i shuffle_bit76_2 = _mm512_setr_epi64(8, 9, 10, 11, 12, 13, 14, 15);
  __m512i shuffle_bit76_3 = _mm512_setr_epi64(16, 17, 18, 19, 20, 21, 22, 23);
  __m512i shuffle_bit76_4 = _mm512_setr_epi64(24, 25, 26, 27, 28, 29, 30, 31);
  __m512i shuffle_bit54_1 = _mm512_setr_epi64(
      0 << 16, 1 << 16, 2 << 16, 3 << 16, 4 << 16, 5 << 16, 6 << 16, 7 << 16);
  __m512i shuffle_bit54_2 =
      _mm512_setr_epi64(8 << 16, 9 << 16, 10 << 16, 11 << 16, 12 << 16,
                        13 << 16, 14 << 16, 15 << 16);
  __m512i shuffle_bit54_3 =
      _mm512_setr_epi64(16 << 16, 17 << 16, 18 << 16, 19 << 16, 20 << 16,
                        21 << 16, 22 << 16, 23 << 16);
  __m512i shuffle_bit54_4 =
      _mm512_setr_epi64(24 << 16, 25 << 16, 26 << 16, 27 << 16, 28 << 16,
                        29 << 16, 30 << 16, 31 << 16);
  __m512i shuffle_bit32_1 =
      _mm512_setr_epi64(0L << 32, 1L << 32, 2L << 32, 3L << 32, 4L << 32,
                        5L << 32, 6L << 32, 7L << 32);
  __m512i shuffle_bit32_2 =
      _mm512_setr_epi64(8L << 32, 9L << 32, 10L << 32, 11L << 32, 12L << 32,
                        13L << 32, 14L << 32, 15L << 32);
  __m512i shuffle_bit32_3 =
      _mm512_setr_epi64(16L << 32, 17L << 32, 18L << 32, 19L << 32, 20L << 32,
                        21L << 32, 22L << 32, 23L << 32);
  __m512i shuffle_bit32_4 =
      _mm512_setr_epi64(24L << 32, 25L << 32, 26L << 32, 27L << 32, 28L << 32,
                        29L << 32, 30L << 32, 31L << 32);
  __m512i shuffle_bit10_1 =
      _mm512_setr_epi64(0L << 48, 1L << 48, 2L << 48, 3L << 48, 4L << 48,
                        5L << 48, 6L << 48, 7L << 48);
  __m512i shuffle_bit10_2 =
      _mm512_setr_epi64(8L << 48, 9L << 48, 10L << 48, 11L << 48, 12L << 48,
                        13L << 48, 14L << 48, 15L << 48);
  __m512i shuffle_bit10_3 =
      _mm512_setr_epi64(16L << 48, 17L << 48, 18L << 48, 19L << 48, 20L << 48,
                        21L << 48, 22L << 48, 23L << 48);
  __m512i shuffle_bit10_4 =
      _mm512_setr_epi64(24L << 48, 25L << 48, 26L << 48, 27L << 48, 28L << 48,
                        29L << 48, 30L << 48, 31L << 48);
  __m512i fix_pack = _mm512_set_epi64(7, 5, 3, 1, 6, 4, 2, 0);
  __m512i zero_vec = _mm512_set_epi64(0, 0, 0, 0, 0, 0, 0, 0);

  for (int i = 0; i < num / 32; i++) {
    symbol1 = _mm512_load_ps(symbols_ptr);
    symbols_ptr += 16;
    symbol2 = _mm512_load_ps(symbols_ptr);
    symbols_ptr += 16;
    symbol3 = _mm512_load_ps(symbols_ptr);
    symbols_ptr += 16;
    symbol4 = _mm512_load_ps(symbols_ptr);
    symbols_ptr += 16;
    symbol_i1 = _mm512_cvtps_epi32(_mm512_mul_ps(symbol1, scale_v));
    symbol_i2 = _mm512_cvtps_epi32(_mm512_mul_ps(symbol2, scale_v));
    symbol_i3 = _mm512_cvtps_epi32(_mm512_mul_ps(symbol3, scale_v));
    symbol_i4 = _mm512_cvtps_epi32(_mm512_mul_ps(symbol4, scale_v));
    // Pack symbols into 16 bit integers
    symbol_12 = _mm512_packs_epi32(symbol_i1, symbol_i2);
    // _packs intrinsic interleaves the two vectors, _permute fixes that
    symbol_12 = _mm512_permutexvar_epi64(fix_pack, symbol_12);
    symbol_34 = _mm512_packs_epi32(symbol_i3, symbol_i4);
    symbol_34 = _mm512_permutexvar_epi64(fix_pack, symbol_34);
    // Pack symbols into 8 bit integers (one 512 bit vector)
    symbol_i = _mm512_packs_epi16(symbol_12, symbol_34);
    symbol_i = _mm512_permutexvar_epi64(fix_pack, symbol_i);

    /*
     * This math is where the LLR occurs. Note that we use the
     * absolute value of the prior vector for the next subtraction,
     * like in the traditional LLR
     */
    symbol_bit54 = _mm512_sub_epi8(offset3, _mm512_abs_epi8(symbol_i));
    symbol_bit32 = _mm512_sub_epi8(offset2, _mm512_abs_epi8(symbol_bit54));
    symbol_bit10 = _mm512_sub_epi8(offset1, _mm512_abs_epi8(symbol_bit32));

    /*
     * Now extract the result and store it. We must do this 4 times due
     * to the capacity of a 512 bit vector
     */
    result10 = _mm512_mask_permutexvar_epi16(zero_vec, bit10_mask,
                                             shuffle_bit10_1, symbol_bit10);
    result32 = _mm512_mask_permutexvar_epi16(zero_vec, bit32_mask,
                                             shuffle_bit32_1, symbol_bit32);
    result54 = _mm512_mask_permutexvar_epi16(zero_vec, bit54_mask,
                                             shuffle_bit54_1, symbol_bit54);
    result76 = _mm512_mask_permutexvar_epi16(zero_vec, bit76_mask,
                                             shuffle_bit76_1, symbol_i);
    // Store the result
    result_final = _mm512_or_si512(
        _mm512_or_si512(_mm512_or_si512(result76, result54), result32),
        result10);
    _mm512_storeu_si512(result_ptr, result_final);
    result_ptr++;

    // Extract and store the second set of results.
    result10 = _mm512_mask_permutexvar_epi16(zero_vec, bit10_mask,
                                             shuffle_bit10_2, symbol_bit10);
    result32 = _mm512_mask_permutexvar_epi16(zero_vec, bit32_mask,
                                             shuffle_bit32_2, symbol_bit32);
    result54 = _mm512_mask_permutexvar_epi16(zero_vec, bit54_mask,
                                             shuffle_bit54_2, symbol_bit54);
    result76 = _mm512_mask_permutexvar_epi16(zero_vec, bit76_mask,
                                             shuffle_bit76_2, symbol_i);
    // Store the result
    result_final = _mm512_or_si512(
        _mm512_or_si512(_mm512_or_si512(result76, result54), result32),
        result10);
    _mm512_storeu_si512(result_ptr, result_final);
    result_ptr++;
    // Third set
    result10 = _mm512_mask_permutexvar_epi16(zero_vec, bit10_mask,
                                             shuffle_bit10_3, symbol_bit10);
    result32 = _mm512_mask_permutexvar_epi16(zero_vec, bit32_mask,
                                             shuffle_bit32_3, symbol_bit32);
    result54 = _mm512_mask_permutexvar_epi16(zero_vec, bit54_mask,
                                             shuffle_bit54_3, symbol_bit54);
    result76 = _mm512_mask_permutexvar_epi16(zero_vec, bit76_mask,
                                             shuffle_bit76_3, symbol_i);
    // Store the result
    result_final = _mm512_or_si512(
        _mm512_or_si512(_mm512_or_si512(result76, result54), result32),
        result10);
    _mm512_storeu_si512(result_ptr, result_final);
    result_ptr++;
    //// Fourth and final set.
    result10 = _mm512_mask_permutexvar_epi16(zero_vec, bit10_mask,
                                             shuffle_bit10_4, symbol_bit10);
    result32 = _mm512_mask_permutexvar_epi16(zero_vec, bit32_mask,
                                             shuffle_bit32_4, symbol_bit32);
    result54 = _mm512_mask_permutexvar_epi16(zero_vec, bit54_mask,
                                             shuffle_bit54_4, symbol_bit54);
    result76 = _mm512_mask_permutexvar_epi16(zero_vec, bit76_mask,
                                             shuffle_bit76_4, symbol_i);
    // Store the result
    result_final = _mm512_or_si512(
        _mm512_or_si512(_mm512_or_si512(result76, result54), result32),
        result10);
    _mm512_storeu_si512(result_ptr, result_final);
    result_ptr++;
  }
  // Demodulate the last symbols
  int next_start = 32 * (num / 32);
  Demod256qamSoftAvx2(vec_in + 2 * next_start, llr + next_start * 8,
                      num - next_start);
}
#endif
