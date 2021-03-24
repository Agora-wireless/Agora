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
 * 10000000 10000001 10000101 10000100 10010100 10010101 10010001 10010000 | 11010000 11010001 11010101 11010100 11000100 11000101 11000001 11000000 
 * 10000010 10000011 10000111 10000110 10010110 10010111 10010011 10010010 | 11010010 11010011 11010111 11010110 11000110 11000111 11000011 11000010 
 * 10001010 10001011 10001111 10001110 10011110 10011111 10011011 10011010 | 11011010 11011011 11011111 11011110 11001110 11001111 11001011 11001010 
 * 10001000 10001001 10001101 10001100 10011100 10011101 10011001 10011000 | 11011000 11011001 11011101 11011100 11001100 11001101 11001001 11001000 
 * 10101000 10101001 10101101 10101100 10111100 10111101 10111001 10111000 | 11111000 11111001 11111101 11111100 11101100 11101101 11101001 11101000 
 * 10101010 10101011 10101111 10101110 10111110 10111111 10111011 10111010 | 11111010 11111011 11111111 11111110 11101110 11101111 11101011 11101010 
 * 10100010 10100011 10100111 10100110 10110110 10110111 10110011 10110010 | 11110010 11110011 11110111 11110110 11100110 11100111 11100011 11100010 
 * 10100000 10100001 10100101 10100100 10110100 10110101 10110001 10110000 | 11110000 11110001 11110101 11110100 11100100 11100101 11100001 11100000 
 * -----------------------------------------------------------------------------------------------------------------------------------------------    I
 * 00100000 00100001 00100101 00100100 00110100 00110101 00110001 00110000 | 01110000 01110001 01110101 01110100 01100100 01100101 01100001 01100000 
 * 00100010 00100011 00100111 00100110 00110110 00110111 00110011 00110010 | 01110010 01110011 01110111 01110110 01100110 01100111 01100011 01100010 
 * 00101010 00101011 00101111 00101110 00111110 00111111 00111011 00111010 | 01111010 01111011 01111111 01111110 01101110 01101111 01101011 01101010 
 * 00101000 00101001 00101101 00101100 00111100 00111101 00111001 00111000 | 01111000 01111001 01111101 01111100 01101100 01101101 01101001 01101000 
 * 00001000 00001001 00001101 00001100 00011100 00011101 00011001 00011000 | 01011000 01011001 01011101 01011100 01001100 01001101 01001001 01001000 
 * 00001010 00001011 00001111 00001110 00011110 00011111 00011011 00011010 | 01011010 01011011 01011111 01011110 01001110 01001111 01001011 01001010 
 * 00000010 00000011 00000111 00000110 00010110 00010111 00010011 00010010 | 01010010 01010011 01010111 01010110 01000110 01000111 01000011 01000010 
 * 00000000 00000001 00000101 00000100 00010100 00010101 00010001 00010000 | 01010000 01010001 01010101 01010100 01000100 01000101 01000001 01000000 
 */
void InitQam256Table(Table<complex_float>& qam256_table) {
  float scale = 1 / sqrt(170);
  uint8_t imag_i, real_i;
  /**
   * To generate this table, first create a 4 bit gray code. Then, generate a
   * vector of the scales you want for your QAM table. Finally, use the gray
   * code values for each index (0-15) to select the scale value you assign to
   * that index in this array. This means that if you take the index of each
   * scale in this table, and order those indicies by the value of the scale
   * the index refers to, you will have a gray code
   */
  float mod_256qam[16] = {(-15) * scale, (-13) * scale, (-9) * scale, 
                          (-11) * scale, (-1) * scale, (-3) * scale, 
                          (-7) * scale, (-5) * scale, 15 * scale, 
                          13 * scale, 9 * scale, 11 * scale, 1 * scale, 
                          3 * scale, 7 * scale, 5 * scale};
  for (int i = 0; i < 256; i++) {
    // Get bits 6, 4, 2, 0 (and pack into 4 bit integer)
    imag_i = (((i & 0x40) >> 3)) + (((i & 0x10) >> 2)) + 
              (((i & 0x4) >> 1)) + (i & 0x1);
    // Get bits 7, 5, 3, 1 (and pack into 4 bit integer)
    real_i = (((i & 0x80) >> 4)) + (((i & 0x20) >> 3)) +
              (((i & 0x8) >> 2)) + (((i & 0x2) >> 1));
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

void Demod256qamHardLoop(const float* vec_in, uint8_t* vec_out, int num) {
  
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
    symbol_abs = _mm256_sub_epi8(offset1, symbol_abs);
    symbol_abs2 = _mm256_sub_epi8(offset2, _mm256_abs_epi8(symbol_abs));

    result11 = _mm256_shuffle_epi8(symbol_i, shuffle_negated_1);
    result12 = _mm256_shuffle_epi8(symbol_abs, shuffle_abs_1);
    result13 = _mm256_shuffle_epi8(symbol_abs2, shuffle_abs2_1);

    result21 = _mm256_shuffle_epi8(symbol_i, shuffle_negated_2);
    result22 = _mm256_shuffle_epi8(symbol_abs, shuffle_abs_2);
    result23 = _mm256_shuffle_epi8(symbol_abs2, shuffle_abs2_2);

    result31 = _mm256_shuffle_epi8(symbol_i, shuffle_negated_3);
    result32 = _mm256_shuffle_epi8(symbol_abs, shuffle_abs_3);
    result33 = _mm256_shuffle_epi8(symbol_abs2, shuffle_abs2_3);

    result_final1 =
        _mm256_or_si256(_mm256_or_si256(result11, result12), result13);
    result_final2 =
        _mm256_or_si256(_mm256_or_si256(result21, result22), result23);
    result_final3 =
        _mm256_or_si256(_mm256_or_si256(result31, result32), result33);

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
