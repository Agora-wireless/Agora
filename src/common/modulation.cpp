/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 * 
 */
#include "modulation.hpp"



void print256_epi32(__m256i var)
{
    int32_t *val = (int32_t*) &var;
    printf("Numerical: %i %i %i %i %i %i %i %i \n", 
           val[0], val[1], val[2], val[3], val[4], val[5], 
           val[6], val[7]);
}

void print256_epi16(__m256i var)
{
    int16_t *val = (int16_t*) &var;
    printf("Numerical: %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i\n", 
           val[0], val[1], val[2], val[3], val[4], val[5], val[6], val[7],
           val[8], val[9], val[10], val[11], val[12], val[13], val[14], val[15]);
}

void print256_epi8(__m256i var)
{
    int8_t *val = (int8_t*) &var;
    printf("Numerical int8_t: %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i \n", 
           val[0], val[1], val[2], val[3], val[4], val[5], val[6], val[7], 
           val[8], val[9], val[10], val[11], val[12], val[13], val[14], val[15], 
           val[16], val[17], val[18], val[19], val[20], val[21], val[22], val[23], 
           val[24], val[25], val[26], val[27], val[28], val[29], val[30], val[31]
           );
}


void print128_epi8(__m128i var)
{
    int8_t *val = (int8_t*) &var;
    printf("Numerical int8_t: %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i \n", 
           val[0], val[1], val[2], val[3], val[4], val[5], val[6], val[7], 
           val[8], val[9], val[10], val[11], val[12], val[13], val[14], val[15]
           );
}


/**
  ***********************************************************************************
  * Initialization functions
  ***********************************************************************************
  */


float **init_modulation_table(size_t mod_order)
{
    float **mod_table;
    switch(mod_order) {
        case 2:
            mod_table = init_qpsk_table();
            break;
        case 4:
            mod_table = init_qam16_table();
            break;
        case 6:
            mod_table = init_qam64_table();
            break;
        default: {
            printf("Modulation order not supported, use default value 4\n");
            mod_table = init_qam16_table();
        }
    }
    return mod_table;
}


/**
  * QPSK modulation
  *              Q
  *  01  |  11
  *---------------> I
  *  00  |  10
  */
float **init_qpsk_table()
{
    float **qpsk_table;
    alloc_buffer_2d(&qpsk_table, 4, 2, 32, 1);
    float scale = 1/sqrt(2);
    float mod_qpsk[2] = {-scale, scale};
    for (int i = 0; i < 4; i++) {
        qpsk_table[i][1] = mod_qpsk[i / 2];
        qpsk_table[i][0] = mod_qpsk[i % 2];
    }
    return qpsk_table;
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
//     alloc_buffer_2d(&qam16_table, 16, 2, 32, 1);
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
float **init_qam16_table()
{
    float **qam16_table;
    alloc_buffer_2d(&qam16_table, 16, 2, 32, 1);
    float scale = 1/sqrt(10);
    float mod_16qam[4] = {1 * scale, 3 * scale, (-1) * scale, (-3) * scale};
    for (int i = 0; i < 16; i++) {
      /* get bit 2 and 0 */
      int imag_i = (((i >> 2) & 0x1) << 1) + (i & 0x1);
      /* get bit 3 and 1 */
      int real_i = (((i >> 3) & 0x1) << 1) + ((i >> 1) & 0x1);
      qam16_table[i][0] = mod_16qam[real_i];
      qam16_table[i][1] = mod_16qam[imag_i];
      printf("%d: (%.3f, %.3f)\n", i, qam16_table[i][0], qam16_table[i][1]);
    }
    return qam16_table;
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

float **init_qam64_table()
{
    float **qam64_table;
    alloc_buffer_2d(&qam64_table, 64, 2, 32, 1);
    float scale = 1/sqrt(42);
    float mod_64qam[8] = {3 * scale, 1 * scale, 5 * scale, 7 * scale, (-3) * scale, (-1) * scale, (-5) * scale, (-7) * scale};
    for (int i = 0; i < 64; i++) {
      /* get bit 4, 2, 0 */
      int imag_i = (((i >> 4) & 0x1) << 2) + (((i >> 2) & 0x1) << 1) + (i & 0x1);
      /* get bit 5, 3, 1 */
      int real_i = (((i >> 5) & 0x1) << 2) + (((i >> 3) & 0x1) << 1) + ((i >> 1) & 0x1);
      qam64_table[i][0] = mod_64qam[real_i];
      qam64_table[i][1] = mod_64qam[imag_i];
    }
    return qam64_table;
}



/**
  ***********************************************************************************
  * Modulation functions
  ***********************************************************************************
  */


complex_float mod_single(int x, float **mod_table) 
{
    complex_float re;
    re.re = mod_table[x][0];
    re.im = mod_table[x][1];
    return re;
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


// void demod_16qam_hard_avx2(float *vec_in, uint8_t *vec_out, int num, int num_simd256)
// {
//     // printf("input:");
//     // for (int i = 0; i < num; i++) {
//     //     printf("(%.2f, %.2f) ", *(vec_in + i * 2), *(vec_in + i * 2 + 1));
//     // }
//     // printf("\n");
//     float float_val = 0.6325;
//     __m256 vec_zero = _mm256_set1_ps(0);
//     __m256 vec_float = _mm256_set1_ps(0.6325);
//     __m256 vec_float_neg = _mm256_set1_ps(-0.6325);
//     // __m256i vec_true = _mm256_set1_epi32(0xFFFFFFFF);
//     __m256i vec_true_mask = _mm256_set1_epi32(0x1);
//     __m256i vec_GT_0 = _mm256_setr_epi32(8, 2, 8, 2, 8, 2, 8, 2);
//     __m256i vec_abs_LT_val = _mm256_setr_epi32(4, 1, 4, 1, 4, 1, 4, 1);
    
//     for (int i = 0; i < num_simd256 * double_num_in_simd256; i = i + double_num_in_simd256) {
//         __m256 raw_data = _mm256_load_ps((vec_in + i * 2));
//         __m256i ret1 = (__m256i)_mm256_cmp_ps(raw_data, vec_zero, _CMP_GT_OS);
//         ret1 = _mm256_and_si256(ret1, vec_true_mask);

//         __m256i ret2 = (__m256i)_mm256_cmp_ps(raw_data, vec_float, _CMP_LT_OS);
//         __m256i ret3 = (__m256i)_mm256_cmp_ps(raw_data, vec_float_neg, _CMP_GT_OS);
//         __m256i ret4 = _mm256_and_si256(ret2, ret3);
//         ret4 = _mm256_and_si256(ret4, vec_true_mask);

//         __m256i ret1_final = _mm256_mullo_epi32(vec_GT_0, ret1);
//         __m256i ret4_final = _mm256_mullo_epi32(ret4, vec_abs_LT_val);

//         __m256i ret_sum = _mm256_add_epi32(ret1_final, ret4_final);
//         // printf("real + real, imag + imag \n");
//         // print256_epi32(ret_sum);
//         // printf("real + imag \n");
//         ret_sum = _mm256_hadd_epi32(ret_sum, ret_sum);
//         // print256_epi32(ret_sum);
//         int8_t *ret_final = (int8_t*) &ret_sum;
//         // print256_epi8(ret_sum);
//         *(vec_out + i) = ret_final[0];
//         *(vec_out + i + 1) = ret_final[4];
//         *(vec_out + i + 2) = ret_final[16];
//         *(vec_out + i + 3) = ret_final[20];

//         // printf("final: %i %i %i %i \n", *(vec_out + i), *(vec_out + i + 1), *(vec_out + i + 2), *(vec_out + i + 3));
//     }

//     for (int i = num_simd256 * double_num_in_simd256; i < num; i++) {
//         float real_val = *(vec_in + i * 2);
//         float imag_val = *(vec_in + i * 2 + 1);
        
//         *(vec_out + i) = 0;
//         if (real_val > 0)
//             *(vec_out + i) |= 1UL << 3;
//         if (std::abs(real_val) < float_val)
//             *(vec_out + i) |= 1UL << 2;
//         if (imag_val > 0)
//             *(vec_out + i) |= 1UL << 1;
//         if (std::abs(imag_val) < float_val)
//             *(vec_out + i) |= 1UL ;
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
void demod_16qam_hard_loop(float *vec_in, uint8_t *vec_out, int num)
{
    float float_val = QAM16_THRESHOLD;

    for (int i = 0; i < num; i++) {
        float real_val = *(vec_in + i * 2);
        float imag_val = *(vec_in + i * 2 + 1);
        
        *(vec_out + i) = 0;
        if (real_val <= 0)
            *(vec_out + i) |= 1UL << 3;
        if (std::abs(real_val) > float_val)
            *(vec_out + i) |= 1UL << 1;
        if (imag_val <= 0)
            *(vec_out + i) |= 1UL << 2;
        if (std::abs(imag_val) > float_val)
            *(vec_out + i) |= 1UL ;
    }
}



void demod_16qam_hard_avx2(float *vec_in, uint8_t *vec_out, int num)
{
    float *symbolsPtr = vec_in;
    __m128i *resultPtr = (__m128i *)vec_out;
    __m256 symbol1, symbol2, symbol3, symbol4;
    __m256i symbol_i1, symbol_i2, symbol_i3, symbol_i4, symbol_12, symbol_34;
    __m256i symbol_abs_1, symbol_abs_2;
    __m256i symbol_gt_0_1, symbol_gt_threshold_1, symbol_gt_0_2, symbol_gt_threshold_2;
    __m256i bit0_1, bit1_1, bit2_1, bit3_1;
    __m256i bit0_2, bit1_2, bit2_2, bit3_2;
    __m256i bit0, bit1, bit2, bit3;
    __m256i result;
    __m256 scale_v = _mm256_set1_ps(-SCALE_BYTE_CONV_QAM16);
    __m256i vec_zero = _mm256_set1_epi16(0);
    __m256i vec_threshold = _mm256_set1_epi16(2 * SCALE_BYTE_CONV_QAM16 / sqrt(10));
    __m256i vec_true_mask = _mm256_set1_epi16(0x1);

    __m256i shuffle_real = _mm256_set_epi8(0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,13,12,9,8,5,4,1,0,
                                           0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,13,12,9,8,5,4,1,0);

    __m256i shuffle_imag = _mm256_set_epi8(0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,15,14,11,10,7,6,3,2,
                                           0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,15,14,11,10,7,6,3,2);

    __m256i shuffle_16_to_8 = _mm256_set_epi8(0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,14,12,10,8,6,4,2,0,
                                              0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,14,12,10,8,6,4,2,0);

    for (int i = 0; i < num / 16; i++) {
        symbol1   = _mm256_load_ps(symbolsPtr); symbolsPtr += 8;
        symbol2   = _mm256_load_ps(symbolsPtr); symbolsPtr += 8;
        symbol3   = _mm256_load_ps(symbolsPtr); symbolsPtr += 8;
        symbol4   = _mm256_load_ps(symbolsPtr); symbolsPtr += 8;
        
        symbol_i1 = _mm256_cvtps_epi32(_mm256_mul_ps(symbol1, scale_v));
        symbol_i2 = _mm256_cvtps_epi32(_mm256_mul_ps(symbol2, scale_v));
        symbol_i3 = _mm256_cvtps_epi32(_mm256_mul_ps(symbol3, scale_v));
        symbol_i4 = _mm256_cvtps_epi32(_mm256_mul_ps(symbol4, scale_v));
        symbol_12  = _mm256_packs_epi32(symbol_i1, symbol_i2);
        symbol_12 = _mm256_permute4x64_epi64(symbol_12, 0xd8);
        symbol_34  = _mm256_packs_epi32(symbol_i3, symbol_i4);
        symbol_34 = _mm256_permute4x64_epi64(symbol_34, 0xd8);

        symbol_abs_1  = _mm256_abs_epi16(symbol_12);
        symbol_gt_0_1 = _mm256_cmpgt_epi16(symbol_12, vec_zero);
        symbol_gt_threshold_1 = _mm256_cmpgt_epi16(symbol_abs_1, vec_threshold);
        symbol_gt_0_1 = _mm256_and_si256(symbol_gt_0_1, vec_true_mask);
        symbol_gt_threshold_1 = _mm256_and_si256(symbol_gt_threshold_1, vec_true_mask);

        bit3_1 = _mm256_shuffle_epi8(symbol_gt_0_1, shuffle_real);
        bit3_1 = _mm256_permute4x64_epi64(bit3_1, 0xd8);
        bit2_1 = _mm256_shuffle_epi8(symbol_gt_0_1, shuffle_imag);
        bit2_1 = _mm256_permute4x64_epi64(bit2_1, 0xd8);

        bit1_1 = _mm256_shuffle_epi8(symbol_gt_threshold_1, shuffle_real);
        bit1_1 = _mm256_permute4x64_epi64(bit1_1, 0xd8);
        bit0_1 = _mm256_shuffle_epi8(symbol_gt_threshold_1, shuffle_imag);
        bit0_1 = _mm256_permute4x64_epi64(bit0_1, 0xd8);

        symbol_abs_2  = _mm256_abs_epi16(symbol_34);
        symbol_gt_0_2 = _mm256_cmpgt_epi16(symbol_34, vec_zero);
        symbol_gt_threshold_2 = _mm256_cmpgt_epi16(symbol_abs_2, vec_threshold);
        symbol_gt_0_2 = _mm256_and_si256(symbol_gt_0_2, vec_true_mask);
        symbol_gt_threshold_2 = _mm256_and_si256(symbol_gt_threshold_2, vec_true_mask);

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
        _mm_store_si128(resultPtr, _mm256_extracti128_si256(result, 0)); resultPtr++;
    }
    // Demodulate last symbols
    for (int i = 16 * (num / 16); i < num; i++) {
        float real_val = *(vec_in + i * 2);
        float imag_val = *(vec_in + i * 2 + 1);
        
        *(vec_out + i) = 0;
        if (real_val <= 0)
            *(vec_out + i) |= 1UL << 3;
        if (std::abs(real_val) > QAM16_THRESHOLD)
            *(vec_out + i) |= 1UL << 1;
        if (imag_val <= 0)
            *(vec_out + i) |= 1UL << 2;
        if (std::abs(imag_val) > QAM16_THRESHOLD)
            *(vec_out + i) |= 1UL ;
    }
}
// void demod_16qam_hard_avx2(float *vec_in, uint8_t *vec_out, int num)
// {
//     __m256 vec_zero = _mm256_set1_ps(0);
//     __m256 vec_float = _mm256_set1_ps(QAM16_THRESHOLD);
//     __m256 vec_float_neg = _mm256_set1_ps(-QAM16_THRESHOLD);
//     __m256i vec_true_mask = _mm256_set1_epi32(0x1);
//     __m256i vec_LE_0 = _mm256_setr_epi32(8, 4, 8, 4, 8, 4, 8, 4);
//     __m256i vec_abs_GT_val = _mm256_setr_epi32(2, 1, 2, 1, 2, 1, 2, 1);

    
//     for (int i = 0; i < num / 4; i = i + 4) {
//         __m256 raw_data = _mm256_load_ps((vec_in + i * 2));
//         __m256i ret1 = (__m256i)_mm256_cmp_ps(raw_data, vec_zero, _CMP_LE_OS);
//         ret1 = _mm256_and_si256(ret1, vec_true_mask);

//         __m256i ret2 = (__m256i)_mm256_cmp_ps(raw_data, vec_float, _CMP_GT_OS);
//         __m256i ret3 = (__m256i)_mm256_cmp_ps(raw_data, vec_float_neg, _CMP_LT_OS);
//         __m256i ret4 = _mm256_and_si256(ret2, ret3);
//         ret4 = _mm256_and_si256(ret4, vec_true_mask);

//         __m256i ret1_final = _mm256_mullo_epi32(ret1, vec_LE_0);
//         __m256i ret4_final = _mm256_mullo_epi32(ret4, vec_abs_GT_val);

//         __m256i ret_sum = _mm256_add_epi32(ret1_final, ret4_final);
//         // printf("real + real, imag + imag \n");
//         // print256_epi32(ret_sum);
//         // printf("real + imag \n");
//         ret_sum = _mm256_hadd_epi32(ret_sum, ret_sum);
//         // print256_epi32(ret_sum);
//         int8_t *ret_final = (int8_t*) &ret_sum;
//         // print256_epi8(ret_sum);
//         *(vec_out + i) = ret_final[0];
//         *(vec_out + i + 1) = ret_final[4];
//         *(vec_out + i + 2) = ret_final[16];
//         *(vec_out + i + 3) = ret_final[20];

//         // printf("final: %i %i %i %i \n", *(vec_out + i), *(vec_out + i + 1), *(vec_out + i + 2), *(vec_out + i + 3));
//     }

//     for (int i = 4 * (num / 4); i < num; i++) {
//         float real_val = *(vec_in + i * 2);
//         float imag_val = *(vec_in + i * 2 + 1);
        
//         *(vec_out + i) = 0;
//         if (real_val > 0)
//             *(vec_out + i) |= 1UL << 3;
//         if (std::abs(real_val) < float_val)
//             *(vec_out + i) |= 1UL << 2;
//         if (imag_val > 0)
//             *(vec_out + i) |= 1UL << 1;
//         if (std::abs(imag_val) < float_val)
//             *(vec_out + i) |= 1UL ;
//     }
    
// }



/* code from srsLTE: 
    https://github.com/srsLTE/srsLTE/blob/6cde228bc165f921135c39c891ec41f2041651e0/lib/src/phy/modem/demod_soft.c
    */
void demod_16qam_soft_loop(float *vec_in, int8_t *llr, int num) {
    for (int i = 0; i < num; i++) {
        int8_t yre = (int8_t) (SCALE_BYTE_CONV_QAM16 * (vec_in[2 * i]));
        int8_t yim = (int8_t) (SCALE_BYTE_CONV_QAM16 * (vec_in[2 * i + 1]));

        llr[4*i+0] = -yre;
        llr[4*i+1] = -yim;
        llr[4*i+2] = abs(yre)-2*SCALE_BYTE_CONV_QAM16/sqrt(10);
        llr[4*i+3] = abs(yim)-2*SCALE_BYTE_CONV_QAM16/sqrt(10);
    }
}


void demod_16qam_soft_sse(float *vec_in, int8_t *llr, int num) {
    float *symbolsPtr = vec_in;
    __m128i *resultPtr = (__m128i*) llr;
    __m128 symbol1, symbol2, symbol3, symbol4;
    __m128i symbol_i1, symbol_i2, symbol_i3, symbol_i4, symbol_i, symbol_abs, symbol_12, symbol_34;
    __m128i offset = _mm_set1_epi8(2*SCALE_BYTE_CONV_QAM16/sqrt(10));
    __m128i result1n, result1a, result2n, result2a;
    __m128 scale_v = _mm_set1_ps(-SCALE_BYTE_CONV_QAM16);

    __m128i shuffle_negated_1 = _mm_set_epi8(0xff,0xff,7,6,0xff,0xff,5,4,0xff,0xff,3,2,0xff,0xff,1,0);
    __m128i shuffle_abs_1     = _mm_set_epi8(7,6,0xff,0xff,5,4,0xff,0xff,3,2,0xff,0xff,1,0,0xff,0xff);

    __m128i shuffle_negated_2 = _mm_set_epi8(0xff,0xff,15,14,0xff,0xff,13,12,0xff,0xff,11,10,0xff,0xff,9,8);
    __m128i shuffle_abs_2     = _mm_set_epi8(15,14,0xff,0xff,13,12,0xff,0xff,11,10,0xff,0xff,9,8,0xff,0xff);

    for (int i = 0; i < num / 8; i++) {
        symbol1   = _mm_load_ps(symbolsPtr); symbolsPtr+=4;
        symbol2   = _mm_load_ps(symbolsPtr); symbolsPtr+=4;
        symbol3   = _mm_load_ps(symbolsPtr); symbolsPtr+=4;
        symbol4   = _mm_load_ps(symbolsPtr); symbolsPtr+=4;
        symbol_i1 = _mm_cvtps_epi32(_mm_mul_ps(symbol1, scale_v));
        symbol_i2 = _mm_cvtps_epi32(_mm_mul_ps(symbol2, scale_v));
        symbol_i3 = _mm_cvtps_epi32(_mm_mul_ps(symbol3, scale_v));
        symbol_i4 = _mm_cvtps_epi32(_mm_mul_ps(symbol4, scale_v));
        symbol_12  = _mm_packs_epi32(symbol_i1, symbol_i2);
        symbol_34  = _mm_packs_epi32(symbol_i3, symbol_i4);
        symbol_i   = _mm_packs_epi16(symbol_12, symbol_34);

        symbol_abs  = _mm_abs_epi8(symbol_i);
        symbol_abs  = _mm_sub_epi8(symbol_abs, offset);

        result1n = _mm_shuffle_epi8(symbol_i, shuffle_negated_1);
        result1a = _mm_shuffle_epi8(symbol_abs, shuffle_abs_1);

        result2n = _mm_shuffle_epi8(symbol_i, shuffle_negated_2);
        result2a = _mm_shuffle_epi8(symbol_abs, shuffle_abs_2);

        _mm_store_si128(resultPtr, _mm_or_si128(result1n, result1a)); resultPtr++;
        _mm_store_si128(resultPtr, _mm_or_si128(result2n, result2a)); resultPtr++;

    }
    // Demodulate last symbols
    for (int i = 8 * (num / 8); i < num; i++) {
        int8_t yre = (int8_t) (SCALE_BYTE_CONV_QAM16 * (vec_in[2 * i]));
        int8_t yim = (int8_t) (SCALE_BYTE_CONV_QAM16 * (vec_in[2 * i + 1]));

        llr[4*i+0] = -yre;
        llr[4*i+1] = -yim;
        llr[4*i+2] = abs(yre)-2*SCALE_BYTE_CONV_QAM16/sqrt(10);
        llr[4*i+3] = abs(yim)-2*SCALE_BYTE_CONV_QAM16/sqrt(10);
    }

    // for (int i = 0; i < ue_num; i++) {
    //     printf("sse: in: %.2f, %.2f, out: %i %i %i %i\n", vec_in[2*i], vec_in[2*i+1], llr[4*i+0], llr[4*i+1], llr[4*i+2], llr[4*i+3]);
    // }
}


void demod_16qam_soft_avx2(float *vec_in, int8_t *llr, int num) {
    float *symbolsPtr = vec_in;
    __m256i *resultPtr = (__m256i*) llr;
    __m256 symbol1, symbol2, symbol3, symbol4;
    __m256i symbol_i1, symbol_i2, symbol_i3, symbol_i4, symbol_i, symbol_abs, symbol_12, symbol_34;
    __m256i offset = _mm256_set1_epi8(2*SCALE_BYTE_CONV_QAM16/sqrt(10));
    __m256i result1n, result1a, result2n, result2a, result1na, result2na;
    __m256 scale_v = _mm256_set1_ps(-SCALE_BYTE_CONV_QAM16);

    __m256i shuffle_negated_1 = _mm256_set_epi8(0xff,0xff,7,6,0xff,0xff,5,4,0xff,0xff,3,2,0xff,0xff,1,0,
                                                0xff,0xff,7,6,0xff,0xff,5,4,0xff,0xff,3,2,0xff,0xff,1,0);
    __m256i shuffle_abs_1     = _mm256_set_epi8(7,6,0xff,0xff,5,4,0xff,0xff,3,2,0xff,0xff,1,0,0xff,0xff,
                                                7,6,0xff,0xff,5,4,0xff,0xff,3,2,0xff,0xff,1,0,0xff,0xff);

    __m256i shuffle_negated_2 = _mm256_set_epi8(0xff,0xff,15,14,0xff,0xff,13,12,0xff,0xff,11,10,0xff,0xff,9,8,
                                                0xff,0xff,15,14,0xff,0xff,13,12,0xff,0xff,11,10,0xff,0xff,9,8);
    __m256i shuffle_abs_2     = _mm256_set_epi8(15,14,0xff,0xff,13,12,0xff,0xff,11,10,0xff,0xff,9,8,0xff,0xff,
                                                15,14,0xff,0xff,13,12,0xff,0xff,11,10,0xff,0xff,9,8,0xff,0xff);

    for (int i = 0; i < num / 16; i++) {
        symbol1   = _mm256_load_ps(symbolsPtr); symbolsPtr += 8;
        symbol2   = _mm256_load_ps(symbolsPtr); symbolsPtr += 8;
        symbol3   = _mm256_load_ps(symbolsPtr); symbolsPtr += 8;
        symbol4   = _mm256_load_ps(symbolsPtr); symbolsPtr += 8;
        symbol_i1 = _mm256_cvtps_epi32(_mm256_mul_ps(symbol1, scale_v));
        symbol_i2 = _mm256_cvtps_epi32(_mm256_mul_ps(symbol2, scale_v));
        symbol_i3 = _mm256_cvtps_epi32(_mm256_mul_ps(symbol3, scale_v));
        symbol_i4 = _mm256_cvtps_epi32(_mm256_mul_ps(symbol4, scale_v));
        symbol_12  = _mm256_packs_epi32(symbol_i1, symbol_i2);
        symbol_12 = _mm256_permute4x64_epi64(symbol_12, 0xd8);
        symbol_34  = _mm256_packs_epi32(symbol_i3, symbol_i4);
        symbol_34 = _mm256_permute4x64_epi64(symbol_34, 0xd8);
        symbol_i   = _mm256_packs_epi16(symbol_12, symbol_34);
        symbol_i = _mm256_permute4x64_epi64(symbol_i, 0xd8);

        symbol_abs  = _mm256_abs_epi8(symbol_i);
        symbol_abs  = _mm256_sub_epi8(symbol_abs, offset);

        result1n = _mm256_shuffle_epi8(symbol_i, shuffle_negated_1);
        result1a = _mm256_shuffle_epi8(symbol_abs, shuffle_abs_1);

        result2n = _mm256_shuffle_epi8(symbol_i, shuffle_negated_2);
        result2a = _mm256_shuffle_epi8(symbol_abs, shuffle_abs_2);

        result1na = _mm256_or_si256(result1n, result1a);
        result2na = _mm256_or_si256(result2n, result2a);
        

        _mm256_store_si256(resultPtr, _mm256_permute2x128_si256(result1na, result2na, 0x20)); resultPtr++;
        _mm256_store_si256(resultPtr, _mm256_permute2x128_si256(result1na, result2na, 0x31)); resultPtr++;

    }
    // Demodulate last symbols
    for (int i = 16 * (num / 16); i < num; i++) {
        int8_t yre = (int8_t) (SCALE_BYTE_CONV_QAM16 * (vec_in[2 * i]));
        int8_t yim = (int8_t) (SCALE_BYTE_CONV_QAM16 * (vec_in[2 * i + 1]));

        llr[4*i+0] = -yre;
        llr[4*i+1] = -yim;
        llr[4*i+2] = abs(yre)-2*SCALE_BYTE_CONV_QAM16/sqrt(10);
        llr[4*i+3] = abs(yim)-2*SCALE_BYTE_CONV_QAM16/sqrt(10);
    }

    // for (int i = 0; i < num; i++) {
    //     printf("avx2: in: %.2f, %.2f, out: %i %i %i %i\n", vec_in[2*i], vec_in[2*i+1], llr[4*i+0], llr[4*i+1], llr[4*i+2], llr[4*i+3]);
    // }
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
void demod_64qam_hard_loop(float *vec_in, uint8_t *vec_out, int num)
{

    for (int i = 0; i < num; i++) {
        float real_val = *(vec_in + i * 2);
        float imag_val = *(vec_in + i * 2 + 1);
        
        *(vec_out + i) = 0;

        if (real_val <= 0)
            *(vec_out + i) |= 1UL << 5;
        if (std::abs(real_val) > QAM64_THRESHOLD_3) {
            *(vec_out + i) |= 1UL << 3;
            *(vec_out + i) |= 1UL << 1;
        } else if (std::abs(real_val) > QAM64_THRESHOLD_2) {
            *(vec_out + i) |= 1UL << 3;
        } else if (std::abs(real_val) <= QAM64_THRESHOLD_1) {
            *(vec_out + i) |= 1UL << 1;
        }

        if (imag_val <= 0)
            *(vec_out + i) |= 1UL << 4;
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



void demod_64qam_hard_avx2(float *vec_in, uint8_t *vec_out, int num, int num_simd256)
{
    __m256 vec_zero = _mm256_set1_ps(0);
    __m256 vec_float = _mm256_set1_ps(QAM16_THRESHOLD);
    __m256 vec_float_neg = _mm256_set1_ps(-QAM16_THRESHOLD);
    __m256i vec_true_mask = _mm256_set1_epi32(0x1);
    __m256i vec_LE_0 = _mm256_setr_epi32(8, 4, 8, 4, 8, 4, 8, 4);
    __m256i vec_abs_GT_val = _mm256_setr_epi32(2, 1, 2, 1, 2, 1, 2, 1);
    
    for (int i = 0; i < num_simd256 * double_num_in_simd256; i = i + double_num_in_simd256) {
        __m256 raw_data = _mm256_load_ps((vec_in + i * 2));
        __m256i ret1 = (__m256i)_mm256_cmp_ps(raw_data, vec_zero, _CMP_LE_OS);
        ret1 = _mm256_and_si256(ret1, vec_true_mask);

        __m256i ret2 = (__m256i)_mm256_cmp_ps(raw_data, vec_float, _CMP_GT_OS);
        __m256i ret3 = (__m256i)_mm256_cmp_ps(raw_data, vec_float_neg, _CMP_LT_OS);
        __m256i ret4 = _mm256_and_si256(ret2, ret3);
        ret4 = _mm256_and_si256(ret4, vec_true_mask);

        __m256i ret1_final = _mm256_mullo_epi32(ret1, vec_LE_0);
        __m256i ret4_final = _mm256_mullo_epi32(ret4, vec_abs_GT_val);

        __m256i ret_sum = _mm256_add_epi32(ret1_final, ret4_final);
        // printf("real + real, imag + imag \n");
        // print256_epi32(ret_sum);
        // printf("real + imag \n");
        ret_sum = _mm256_hadd_epi32(ret_sum, ret_sum);
        // print256_epi32(ret_sum);
        int8_t *ret_final = (int8_t*) &ret_sum;
        // print256_epi8(ret_sum);
        *(vec_out + i) = ret_final[0];
        *(vec_out + i + 1) = ret_final[4];
        *(vec_out + i + 2) = ret_final[16];
        *(vec_out + i + 3) = ret_final[20];

        // printf("final: %i %i %i %i \n", *(vec_out + i), *(vec_out + i + 1), *(vec_out + i + 2), *(vec_out + i + 3));
    }

    for (int i = num_simd256 * double_num_in_simd256; i < num; i++) {
        float real_val = *(vec_in + i * 2);
        float imag_val = *(vec_in + i * 2 + 1);
        
        *(vec_out + i) = 0;

        if (real_val <= 0)
            *(vec_out + i) |= 1UL << 5;
        if (std::abs(real_val) > QAM64_THRESHOLD_3) {
            *(vec_out + i) |= 1UL << 3;
            *(vec_out + i) |= 1UL << 1;
        } else if (std::abs(real_val) > QAM64_THRESHOLD_2) {
            *(vec_out + i) |= 1UL << 3;
        } else if (std::abs(real_val) <= QAM64_THRESHOLD_1) {
            *(vec_out + i) |= 1UL << 1;
        }

        if (imag_val <= 0)
            *(vec_out + i) |= 1UL << 4;
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



void demod_64qam_hard_avx2(float *vec_in, uint8_t *vec_out, int num)
{
    float *symbolsPtr = vec_in;
    __m128i *resultPtr = (__m128i *)vec_out;
    __m256 symbol1, symbol2, symbol3, symbol4;
    __m256i symbol_i1, symbol_i2, symbol_i3, symbol_i4, symbol_12, symbol_34;
    __m256i symbol_abs_1, symbol_abs_2;
    __m256i symbol_gt_0_1, symbol_gt_0_2;
    __m256i symbol_lt_threshold1_1, symbol_lt_threshold1_2;
    __m256i symbol_gt_threshold2_1, symbol_gt_threshold2_2;
    __m256i symbol_gt_threshold3_1, symbol_gt_threshold3_2;
    __m256i bit01_1, bit23_1, bit45_1, bit01_2, bit23_2, bit45_2;
    __m256i bit0_1, bit1_1, bit2_1, bit3_1, bit4_1, bit5_1;
    __m256i bit0_2, bit1_2, bit2_2, bit3_2, bit4_2, bit5_2;
    __m256i bit0, bit1, bit2, bit3, bit4, bit5;
    __m256i result;
    __m256 scale_v = _mm256_set1_ps(-SCALE_BYTE_CONV_QAM16);
    __m256i vec_zero = _mm256_set1_epi16(0);
    __m256i offset1 = _mm256_set1_epi16(2*SCALE_BYTE_CONV_QAM64/sqrt(42));
    __m256i offset2 = _mm256_set1_epi16(4*SCALE_BYTE_CONV_QAM64/sqrt(42));
    __m256i offset3 = _mm256_set1_epi16(6*SCALE_BYTE_CONV_QAM64/sqrt(42));
    __m256i vec_true_mask = _mm256_set1_epi16(0x1);

    __m256i shuffle_real = _mm256_set_epi8(0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,13,12,9,8,5,4,1,0,
                                           0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,13,12,9,8,5,4,1,0);

    __m256i shuffle_imag = _mm256_set_epi8(0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,15,14,11,10,7,6,3,2,
                                           0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,15,14,11,10,7,6,3,2);

    __m256i shuffle_16_to_8 = _mm256_set_epi8(0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,14,12,10,8,6,4,2,0,
                                              0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,14,12,10,8,6,4,2,0);

    for (int i = 0; i < num / 16; i++) {
        symbol1   = _mm256_load_ps(symbolsPtr); symbolsPtr += 8;
        symbol2   = _mm256_load_ps(symbolsPtr); symbolsPtr += 8;
        symbol3   = _mm256_load_ps(symbolsPtr); symbolsPtr += 8;
        symbol4   = _mm256_load_ps(symbolsPtr); symbolsPtr += 8;
        
        symbol_i1 = _mm256_cvtps_epi32(_mm256_mul_ps(symbol1, scale_v));
        symbol_i2 = _mm256_cvtps_epi32(_mm256_mul_ps(symbol2, scale_v));
        symbol_i3 = _mm256_cvtps_epi32(_mm256_mul_ps(symbol3, scale_v));
        symbol_i4 = _mm256_cvtps_epi32(_mm256_mul_ps(symbol4, scale_v));
        symbol_12  = _mm256_packs_epi32(symbol_i1, symbol_i2);
        symbol_12 = _mm256_permute4x64_epi64(symbol_12, 0xd8);
        symbol_34  = _mm256_packs_epi32(symbol_i3, symbol_i4);
        symbol_34 = _mm256_permute4x64_epi64(symbol_34, 0xd8);

        symbol_abs_1  = _mm256_abs_epi16(symbol_12);
        symbol_gt_0_1 = _mm256_cmpgt_epi16(symbol_12, vec_zero);
        symbol_lt_threshold1_1 = _mm256_cmpgt_epi16(offset1, symbol_abs_1);
        symbol_gt_threshold2_1 = _mm256_cmpgt_epi16(symbol_abs_1, offset2);
        symbol_gt_threshold3_1 = _mm256_cmpgt_epi16(symbol_abs_1, offset3);
        symbol_gt_0_1 = _mm256_and_si256(symbol_gt_0_1, vec_true_mask);

        symbol_lt_threshold1_1 = _mm256_and_si256(symbol_lt_threshold1_1, vec_true_mask);
        symbol_gt_threshold2_1 = _mm256_and_si256(symbol_gt_threshold2_1, vec_true_mask);
        symbol_gt_threshold3_1 = _mm256_and_si256(symbol_gt_threshold3_1, vec_true_mask);
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


        symbol_abs_2  = _mm256_abs_epi16(symbol_34);
        symbol_gt_0_2 = _mm256_cmpgt_epi16(symbol_34, vec_zero);
        symbol_lt_threshold1_2 = _mm256_cmpgt_epi16(offset1, symbol_abs_2);
        symbol_gt_threshold2_2 = _mm256_cmpgt_epi16(symbol_abs_2, offset2);
        symbol_gt_threshold3_2 = _mm256_cmpgt_epi16(symbol_abs_2, offset3);
        symbol_gt_0_2 = _mm256_and_si256(symbol_gt_0_2, vec_true_mask);

        symbol_lt_threshold1_2 = _mm256_and_si256(symbol_lt_threshold1_2, vec_true_mask);
        symbol_gt_threshold2_2 = _mm256_and_si256(symbol_gt_threshold2_2, vec_true_mask);
        symbol_gt_threshold3_2 = _mm256_and_si256(symbol_gt_threshold3_2, vec_true_mask);
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
        _mm_store_si128(resultPtr, _mm256_extracti128_si256(result, 0)); resultPtr++;
    }
    // Demodulate last symbols
    for (int i = 16 * (num / 16); i < num; i++) {
        float real_val = *(vec_in + i * 2);
        float imag_val = *(vec_in + i * 2 + 1);
        
        *(vec_out + i) = 0;

        if (real_val <= 0)
            *(vec_out + i) |= 1UL << 5;
        if (std::abs(real_val) > QAM64_THRESHOLD_3) {
            *(vec_out + i) |= 1UL << 3;
            *(vec_out + i) |= 1UL << 1;
        } else if (std::abs(real_val) > QAM64_THRESHOLD_2) {
            *(vec_out + i) |= 1UL << 3;
        } else if (std::abs(real_val) <= QAM64_THRESHOLD_1) {
            *(vec_out + i) |= 1UL << 1;
        }

        if (imag_val <= 0)
            *(vec_out + i) |= 1UL << 4;
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






void demod_64qam_soft_loop(float *vec_in, int8_t *llr, int num) 
{
    for (int i = 0; i < num; i++) {
        float yre = (int8_t) (SCALE_BYTE_CONV_QAM64 * (vec_in[2 * i]));
        float yim = (int8_t) (SCALE_BYTE_CONV_QAM64 * (vec_in[2 * i + 1]));

        llr[6*i+0] = -yre;
        llr[6*i+1] = -yim;
        llr[6*i+2] = abs(yre)-4*SCALE_BYTE_CONV_QAM64/sqrt(42);
        llr[6*i+3] = abs(yim)-4*SCALE_BYTE_CONV_QAM64/sqrt(42);
        llr[6*i+4] = abs(llr[6*i+2])-2*SCALE_BYTE_CONV_QAM64/sqrt(42);
        llr[6*i+5] = abs(llr[6*i+3])-2*SCALE_BYTE_CONV_QAM64/sqrt(42);
    }
}


void demod_64qam_soft_sse(float *vec_in, int8_t *llr, int num)
{
    float *symbolsPtr = (float*) vec_in;
    __m128i *resultPtr = (__m128i*) llr;
    __m128 symbol1, symbol2, symbol3, symbol4;
    __m128i symbol_i1, symbol_i2, symbol_i3, symbol_i4, symbol_i, symbol_abs, symbol_abs2,symbol_12, symbol_34;
    __m128i offset1 = _mm_set1_epi8(4*SCALE_BYTE_CONV_QAM64/sqrt(42));
    __m128i offset2 = _mm_set1_epi8(2*SCALE_BYTE_CONV_QAM64/sqrt(42));
    __m128 scale_v = _mm_set1_ps(-SCALE_BYTE_CONV_QAM64);
    __m128i result11, result12, result13, result22, result21,result23, result31, result32, result33;

    __m128i shuffle_negated_1 = _mm_set_epi8(0xff,0xff,5,4,0xff,0xff,0xff,0xff,3,2,0xff,0xff,0xff,0xff,1,0);
    __m128i shuffle_negated_2 = _mm_set_epi8(11,10,0xff,0xff,0xff,0xff,9,8,0xff,0xff,0xff,0xff,7,6,0xff,0xff);
    __m128i shuffle_negated_3 = _mm_set_epi8(0xff,0xff,0xff,0xff,15,14,0xff,0xff,0xff,0xff,13,12,0xff,0xff,0xff,0xff);

    __m128i shuffle_abs_1 = _mm_set_epi8(5,4,0xff,0xff,0xff,0xff,3,2,0xff,0xff,0xff,0xff,1,0,0xff,0xff);
    __m128i shuffle_abs_2 = _mm_set_epi8(0xff,0xff,0xff,0xff,9,8,0xff,0xff,0xff,0xff,7,6,0xff,0xff,0xff,0xff);
    __m128i shuffle_abs_3 = _mm_set_epi8(0xff,0xff,15,14,0xff,0xff,0xff,0xff,13,12,0xff,0xff,0xff,0xff,11,10);

    __m128i shuffle_abs2_1 = _mm_set_epi8(0xff,0xff,0xff,0xff,3,2,0xff,0xff,0xff,0xff,1,0,0xff,0xff,0xff,0xff);
    __m128i shuffle_abs2_2 = _mm_set_epi8(0xff,0xff,9,8,0xff,0xff,0xff,0xff,7,6,0xff,0xff,0xff,0xff,5,4);
    __m128i shuffle_abs2_3 = _mm_set_epi8(15,14,0xff,0xff,0xff,0xff,13,12,0xff,0xff,0xff,0xff,11,10,0xff,0xff);

    for (int i = 0; i < num / 8; i++) {
        symbol1   = _mm_load_ps(symbolsPtr); symbolsPtr+=4;
        symbol2   = _mm_load_ps(symbolsPtr); symbolsPtr+=4;
        symbol3   = _mm_load_ps(symbolsPtr); symbolsPtr+=4;
        symbol4   = _mm_load_ps(symbolsPtr); symbolsPtr+=4;
        symbol_i1 = _mm_cvtps_epi32(_mm_mul_ps(symbol1, scale_v));
        symbol_i2 = _mm_cvtps_epi32(_mm_mul_ps(symbol2, scale_v));
        symbol_i3 = _mm_cvtps_epi32(_mm_mul_ps(symbol3, scale_v));
        symbol_i4 = _mm_cvtps_epi32(_mm_mul_ps(symbol4, scale_v));
        symbol_12  = _mm_packs_epi32(symbol_i1, symbol_i2);
        symbol_34  = _mm_packs_epi32(symbol_i3, symbol_i4);
        symbol_i   = _mm_packs_epi16(symbol_12, symbol_34);

        symbol_abs  = _mm_abs_epi8(symbol_i);
        symbol_abs  = _mm_sub_epi8(symbol_abs, offset1);
        symbol_abs2 = _mm_sub_epi8(_mm_abs_epi8(symbol_abs), offset2);

        result11 = _mm_shuffle_epi8(symbol_i, shuffle_negated_1);
        result12 = _mm_shuffle_epi8(symbol_abs, shuffle_abs_1);
        result13 = _mm_shuffle_epi8(symbol_abs2, shuffle_abs2_1);

        result21 = _mm_shuffle_epi8(symbol_i, shuffle_negated_2);
        result22 = _mm_shuffle_epi8(symbol_abs, shuffle_abs_2);
        result23 = _mm_shuffle_epi8(symbol_abs2, shuffle_abs2_2);

        result31 = _mm_shuffle_epi8(symbol_i, shuffle_negated_3);
        result32 = _mm_shuffle_epi8(symbol_abs, shuffle_abs_3);
        result33 = _mm_shuffle_epi8(symbol_abs2, shuffle_abs2_3);

        _mm_store_si128(resultPtr, _mm_or_si128(_mm_or_si128(result11, result12),result13)); resultPtr++;
        _mm_store_si128(resultPtr, _mm_or_si128(_mm_or_si128(result21, result22),result23)); resultPtr++;
        _mm_store_si128(resultPtr, _mm_or_si128(_mm_or_si128(result31, result32),result33)); resultPtr++;

    }
    for (int i = 8 * (num / 8); i < num; i++) {
        float yre = (int8_t) (SCALE_BYTE_CONV_QAM64 * (vec_in[2 * i]));
        float yim = (int8_t) (SCALE_BYTE_CONV_QAM64 * (vec_in[2 * i + 1]));

        llr[6*i+0] = -yre;
        llr[6*i+1] = -yim;
        llr[6*i+2] = abs(yre)-4*SCALE_BYTE_CONV_QAM64/sqrt(42);
        llr[6*i+3] = abs(yim)-4*SCALE_BYTE_CONV_QAM64/sqrt(42);
        llr[6*i+4] = abs(llr[6*i+2])-2*SCALE_BYTE_CONV_QAM64/sqrt(42);
        llr[6*i+5] = abs(llr[6*i+3])-2*SCALE_BYTE_CONV_QAM64/sqrt(42);
    }
}



void demod_64qam_soft_avx2(float *vec_in, int8_t *llr, int num)
{
    float *symbolsPtr = (float*) vec_in;
    __m256i *resultPtr = (__m256i*) llr;
    __m256 symbol1, symbol2, symbol3, symbol4;
    __m256i symbol_i1, symbol_i2, symbol_i3, symbol_i4, symbol_i, symbol_abs, symbol_abs2,symbol_12, symbol_34;
    __m256i offset1 = _mm256_set1_epi8(4*SCALE_BYTE_CONV_QAM64/sqrt(42));
    __m256i offset2 = _mm256_set1_epi8(2*SCALE_BYTE_CONV_QAM64/sqrt(42));
    __m256 scale_v = _mm256_set1_ps(-SCALE_BYTE_CONV_QAM64);
    __m256i result11, result12, result13, result22, result21,result23, result31, result32, result33;
    __m256i result_final1, result_final2, result_final3;

    __m256i shuffle_negated_1 = _mm256_set_epi8(0xff,0xff,5,4,0xff,0xff,0xff,0xff,3,2,0xff,0xff,0xff,0xff,1,0,
                                              0xff,0xff,5,4,0xff,0xff,0xff,0xff,3,2,0xff,0xff,0xff,0xff,1,0);
    __m256i shuffle_negated_2 = _mm256_set_epi8(11,10,0xff,0xff,0xff,0xff,9,8,0xff,0xff,0xff,0xff,7,6,0xff,0xff,
                                              11,10,0xff,0xff,0xff,0xff,9,8,0xff,0xff,0xff,0xff,7,6,0xff,0xff);
    __m256i shuffle_negated_3 = _mm256_set_epi8(0xff,0xff,0xff,0xff,15,14,0xff,0xff,0xff,0xff,13,12,0xff,0xff,0xff,0xff,
                                              0xff,0xff,0xff,0xff,15,14,0xff,0xff,0xff,0xff,13,12,0xff,0xff,0xff,0xff);

    __m256i shuffle_abs_1 = _mm256_set_epi8(5,4,0xff,0xff,0xff,0xff,3,2,0xff,0xff,0xff,0xff,1,0,0xff,0xff,
                                          5,4,0xff,0xff,0xff,0xff,3,2,0xff,0xff,0xff,0xff,1,0,0xff,0xff);
    __m256i shuffle_abs_2 = _mm256_set_epi8(0xff,0xff,0xff,0xff,9,8,0xff,0xff,0xff,0xff,7,6,0xff,0xff,0xff,0xff,
                                          0xff,0xff,0xff,0xff,9,8,0xff,0xff,0xff,0xff,7,6,0xff,0xff,0xff,0xff);
    __m256i shuffle_abs_3 = _mm256_set_epi8(0xff,0xff,15,14,0xff,0xff,0xff,0xff,13,12,0xff,0xff,0xff,0xff,11,10,
                                          0xff,0xff,15,14,0xff,0xff,0xff,0xff,13,12,0xff,0xff,0xff,0xff,11,10);

    __m256i shuffle_abs2_1 = _mm256_set_epi8(0xff,0xff,0xff,0xff,3,2,0xff,0xff,0xff,0xff,1,0,0xff,0xff,0xff,0xff,
                                           0xff,0xff,0xff,0xff,3,2,0xff,0xff,0xff,0xff,1,0,0xff,0xff,0xff,0xff);
    __m256i shuffle_abs2_2 = _mm256_set_epi8(0xff,0xff,9,8,0xff,0xff,0xff,0xff,7,6,0xff,0xff,0xff,0xff,5,4,
                                           0xff,0xff,9,8,0xff,0xff,0xff,0xff,7,6,0xff,0xff,0xff,0xff,5,4);
    __m256i shuffle_abs2_3 = _mm256_set_epi8(15,14,0xff,0xff,0xff,0xff,13,12,0xff,0xff,0xff,0xff,11,10,0xff,0xff,
                                           15,14,0xff,0xff,0xff,0xff,13,12,0xff,0xff,0xff,0xff,11,10,0xff,0xff);

    for (int i = 0; i < num / 16; i++) {
        symbol1   = _mm256_load_ps(symbolsPtr); symbolsPtr += 8;
        symbol2   = _mm256_load_ps(symbolsPtr); symbolsPtr += 8;
        symbol3   = _mm256_load_ps(symbolsPtr); symbolsPtr += 8;
        symbol4   = _mm256_load_ps(symbolsPtr); symbolsPtr += 8;
        symbol_i1 = _mm256_cvtps_epi32(_mm256_mul_ps(symbol1, scale_v));
        symbol_i2 = _mm256_cvtps_epi32(_mm256_mul_ps(symbol2, scale_v));
        symbol_i3 = _mm256_cvtps_epi32(_mm256_mul_ps(symbol3, scale_v));
        symbol_i4 = _mm256_cvtps_epi32(_mm256_mul_ps(symbol4, scale_v));
        symbol_12  = _mm256_packs_epi32(symbol_i1, symbol_i2);
        symbol_12 = _mm256_permute4x64_epi64(symbol_12, 0xd8);
        symbol_34  = _mm256_packs_epi32(symbol_i3, symbol_i4);
        symbol_34 = _mm256_permute4x64_epi64(symbol_34, 0xd8);
        symbol_i   = _mm256_packs_epi16(symbol_12, symbol_34);
        symbol_i = _mm256_permute4x64_epi64(symbol_i, 0xd8);

        symbol_abs  = _mm256_abs_epi8(symbol_i);
        symbol_abs  = _mm256_sub_epi8(symbol_abs, offset1);
        symbol_abs2 = _mm256_sub_epi8(_mm256_abs_epi8(symbol_abs), offset2);

        result11 = _mm256_shuffle_epi8(symbol_i, shuffle_negated_1);
        result12 = _mm256_shuffle_epi8(symbol_abs, shuffle_abs_1);
        result13 = _mm256_shuffle_epi8(symbol_abs2, shuffle_abs2_1);

        result21 = _mm256_shuffle_epi8(symbol_i, shuffle_negated_2);
        result22 = _mm256_shuffle_epi8(symbol_abs, shuffle_abs_2);
        result23 = _mm256_shuffle_epi8(symbol_abs2, shuffle_abs2_2);

        result31 = _mm256_shuffle_epi8(symbol_i, shuffle_negated_3);
        result32 = _mm256_shuffle_epi8(symbol_abs, shuffle_abs_3);
        result33 = _mm256_shuffle_epi8(symbol_abs2, shuffle_abs2_3);

        result_final1 = _mm256_or_si256(_mm256_or_si256(result11, result12),result13);
        result_final2 = _mm256_or_si256(_mm256_or_si256(result21, result22),result23);
        result_final3 = _mm256_or_si256(_mm256_or_si256(result31, result32),result33);

        _mm256_store_si256(resultPtr, _mm256_permute2x128_si256(result_final1, result_final2, 0x20)); resultPtr++;
        _mm256_store_si256(resultPtr, _mm256_permute2x128_si256(result_final3, result_final1, 0x30)); resultPtr++;
        _mm256_store_si256(resultPtr, _mm256_permute2x128_si256(result_final2, result_final3, 0x31)); resultPtr++;
    }
    for (int i = 8 * (num / 16); i < num; i++) {
        float yre = (int8_t) (SCALE_BYTE_CONV_QAM64 * (vec_in[2 * i]));
        float yim = (int8_t) (SCALE_BYTE_CONV_QAM64 * (vec_in[2 * i + 1]));

        llr[6*i+0] = -yre;
        llr[6*i+1] = -yim;
        llr[6*i+2] = abs(yre)-4*SCALE_BYTE_CONV_QAM64/sqrt(42);
        llr[6*i+3] = abs(yim)-4*SCALE_BYTE_CONV_QAM64/sqrt(42);
        llr[6*i+4] = abs(llr[6*i+2])-2*SCALE_BYTE_CONV_QAM64/sqrt(42);
        llr[6*i+5] = abs(llr[6*i+3])-2*SCALE_BYTE_CONV_QAM64/sqrt(42);
    }
}



