/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 * 
 */
#include "compute_common.hpp"

void init_qpsk_table(float **qpsk_table)
{
    float scale = 1/sqrt(2);
    float mod_qpsk[2] = {-scale, scale};
    for (int i = 0; i < 4; i++) {
        qpsk_table[0][i] = mod_qpsk[i / 2];
        qpsk_table[1][i] = mod_qpsk[i % 2];
    }
}

void init_qam16_table(float **qam16_table)
{
    float scale = 1/sqrt(10);
    float modvec_16qam[4] = {-3*scale, -1*scale, 3*scale, scale};
    for (int i = 0; i < 16; i++) {
        qam16_table[0][i] = modvec_16qam[i / 4];
        qam16_table[1][i] = modvec_16qam[i % 4];
    }
}

void init_qam64_table(float **qam64_table)
{
    float scale = 1/sqrt(42);
    float mod_64qam[8] = {-7*scale, -5*scale, -3*scale, -1*scale, scale, 3*scale, 5*scale, 7*scale};
    for (int i = 0; i < 64; i++) {
        qam64_table[0][i] = mod_64qam[i / 8];
        qam64_table[1][i] = mod_64qam[i % 8];
    }
}

complex_float divide(complex_float e1, complex_float e2)
{
    complex_float re;
    float module = e2.real * e2.real + e2.imag * e2.imag;
    re.real = (e1.real * e2.real + e1.imag * e2.imag) / module;
    re.imag = (e1.imag * e2.real - e1.real * e2.imag) / module;
    return re;
}


imat demod_16qam(cx_fmat x)
{
    imat re;
    mat zero_mat = zeros<mat>(size(x));
    // mat float_mat = 0.6325*ones<mat>(size(x));
    mat float_mat(size(x));
    float_mat.fill(0.6325);

    umat c1 = real(x)>zero_mat;
    imat c1_int = conv_to<imat>::from(c1);
    umat c2 = abs(real(x))<float_mat;
    imat c2_int = conv_to<imat>::from(c2);
    umat c3 = imag(x)>zero_mat;
    imat c3_int = conv_to<imat>::from(c3);
    umat c4 = abs(imag(x))<float_mat;
    imat c4_int = conv_to<imat>::from(c4);
    re = 8*c1_int+4*c2_int+2*c3_int+1*c4_int;
    // cout << "In demod_16qam: memory of x: " << x.memptr() << ",  memory of re: " << re.memptr() << endl;
    // cout << "x:" << endl;
    // cout << x.st() << endl;
    // cout <<imag(x).st() << endl;
    // cout << "Re:" << re.st() << endl;
    return re;
}


void demod_16qam_loop(float *vec_in, uint8_t *vec_out, int ue_num)
{
    float float_val = 0.6325;

    for (int i = 0; i < ue_num; i++) {
        float real_val = *(vec_in + i * 2);
        float imag_val = *(vec_in + i * 2 + 1);
        
        *(vec_out + i) = 0;
        if (real_val > 0)
            *(vec_out + i) |= 1UL << 3;
            //*(vec_out + i) += 8;
        if (std::abs(real_val) < float_val)
            *(vec_out + i) |= 1UL << 2;
            //*(vec_out + i) += 4;
        if (imag_val > 0)
            *(vec_out + i) |= 1UL << 1;
            //*(vec_out + i) += 2;
        if (std::abs(imag_val) < float_val)
            *(vec_out + i) |= 1UL ;
            //*(vec_out + i) += 1;

    }
}

void print256_epi32(__m256i var)
{
    int32_t *val = (int32_t*) &var;
    printf("Numerical: %i %i %i %i %i %i %i %i \n", 
           val[0], val[1], val[2], val[3], val[4], val[5], 
           val[6], val[7]);
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

void demod_16qam_loop_simd(float *vec_in, uint8_t *vec_out, int ue_num, int num_simd256)
{
    // printf("input:");
    // for (int i = 0; i < ue_num; i++) {
    //     printf("(%.2f, %.2f) ", *(vec_in + i * 2), *(vec_in + i * 2 + 1));
    // }
    // printf("\n");
    float float_val = 0.6325;
    __m256 vec_zero = _mm256_set1_ps(0);
    __m256 vec_float = _mm256_set1_ps(0.6325);
    __m256 vec_float_neg = _mm256_set1_ps(-0.6325);
    // __m256i vec_true = _mm256_set1_epi32(0xFFFFFFFF);
    __m256i vec_true_mask = _mm256_set1_epi32(0x1);
    __m256i vec_GT_0 = _mm256_setr_epi32(8, 2, 8, 2, 8, 2, 8, 2);
    __m256i vec_abs_LT_val = _mm256_setr_epi32(4, 1, 4, 1, 4, 1, 4, 1);
    
    for (int i = 0; i < num_simd256 * double_num_in_simd256; i = i + double_num_in_simd256) {
        __m256 raw_data = _mm256_load_ps((vec_in + i * 2));
        __m256i ret1 = (__m256i)_mm256_cmp_ps(raw_data, vec_zero, _CMP_GT_OS);
        ret1 = _mm256_and_si256(ret1, vec_true_mask);

        __m256i ret2 = (__m256i)_mm256_cmp_ps(raw_data, vec_float, _CMP_LT_OS);
        __m256i ret3 = (__m256i)_mm256_cmp_ps(raw_data, vec_float_neg, _CMP_GT_OS);
        __m256i ret4 = _mm256_and_si256(ret2, ret3);
        ret4 = _mm256_and_si256(ret4, vec_true_mask);

        __m256i ret1_final = _mm256_mullo_epi32(vec_GT_0, ret1);
        __m256i ret4_final = _mm256_mullo_epi32(ret4, vec_abs_LT_val);

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

    for (int i = num_simd256 * double_num_in_simd256; i < ue_num; i++) {
        float real_val = *(vec_in + i * 2);
        float imag_val = *(vec_in + i * 2 + 1);
        
        *(vec_out + i) = 0;
        if (real_val > 0)
            *(vec_out + i) |= 1UL << 3;
        if (std::abs(real_val) < float_val)
            *(vec_out + i) |= 1UL << 2;
        if (imag_val > 0)
            *(vec_out + i) |= 1UL << 1;
        if (std::abs(imag_val) < float_val)
            *(vec_out + i) |= 1UL ;
    }
    
}


// inline void CoMP::demod_16qam_soft(float *vec_in, float *vec_out, int length )
// {
//     unsigned char re, im;
//     for (int i = 0; i < length; i++) {
//         float re_float = *(vec_in + i * 2);
//         float im_float = *(vec_in + i * 2+1);
//         re = (unsigned char)((re_float+1) * 255);
//         im = (unsigned char)((im_float+1) * 255);
//         *(vec_out + 4 * i) = m_bpsk_lut[re];
//         *(vec_out + 4 * i + 1) = m_qam16_lut2[re];
//         *(vec_out + 4 * i + 2) = m_bpsk_lut[im];
//         *(vec_out + 4 * i + 3) = m_qam16_lut2[im];
//     }
// }


// inline cx_fmat mod_16qam(imat x)
// {
//     // cx_fmat re(size(x));
//     fmat real_re = conv_to<fmat>::from(x);
//     fmat imag_re = conv_to<fmat>::from(x);
//     // float scale = 1/sqrt(10);
//     // float modvec_16qam[4]  = {-3*scale, -1*scale, 3*scale, scale};

//     // real_re.for_each([&modvec_16qam](fmat::elem_type& val) { val = modvec_16qam[(int)val/4]; } );
//     // imag_re.for_each([&modvec_16qam](fmat::elem_type& val) { val = modvec_16qam[(int)val%4]; } );
//     real_re.for_each([this](fmat::elem_type& val) { val = qam16_table[0][(int)val]; } );
//     imag_re.for_each([this](fmat::elem_type& val) { val = qam16_table[1][(int)val]; } );
//     cx_fmat re(real_re, imag_re);
//     // re.set_real(real_re);
//     // re.set_imag(imag_re);
//     // cout << "In mod_16qam: memory of x: " << x.memptr() << ",  memory of re: " << re.memptr() << endl;
//     // cout << "x:" << endl;
//     // cout << x.st() << endl;
//     // cout << "Re:" << real(re).st() << endl;
//     return re;
// }


complex_float mod_16qam_single(int x, float **qam16_table) 
{
    complex_float re;
    re.real = qam16_table[0][x];
    re.imag = qam16_table[1][x];
    return re;
}

