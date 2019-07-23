#include "compute_common.hpp"

void init_qam16_table(float (*qam16_table)[16])
{
    float scale = 1/sqrt(10);
    float modvec_16qam[4] = {-3*scale, -1*scale, 3*scale, scale};
    for (int i = 0; i < 16; i++) {
        qam16_table[0][i] = modvec_16qam[i / 4];
        qam16_table[1][i] = modvec_16qam[i % 4];
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
    // __m128 vec_zero = _mm_set1_ps(0);
    // __m128 vec_float = _mm_set1_ps(0.6325);
    // for (int i = 0; i < ue_num; i += 4) {
    //     __m128 vec_in_cur = 
    //     __m128 vec_real_cmp = _mm_cmpgt_ps()
    // }
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


complex_float mod_16qam_single(int x, float (*qam16_table)[16]) 
{
    complex_float re;
    re.real = qam16_table[0][x];
    re.imag = qam16_table[1][x];
    return re;
}

