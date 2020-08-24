/*
 Select signal processing and communications blocks implemented using Intel AVX

find_beacon_avx: Correlation and Peak detection of a beacon with Gold code (2 repetitions)
---------------------------------------------------------------------
 Copyright (c) 2018-2020, Rice University 
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 Author(s): Rahman Doost-Mohamamdy: doost@rice.edu
---------------------------------------------------------------------
*/

#include "comms-lib.h"
#include <queue>
//#include <itpp/itbase.h>
#include <immintrin.h>
#include <iomanip>

#define USE_AVX
#define ALIGNMENT 32
#define AVX_PACKED_SP 8 //single-precision
#define AVX_PACKED_SI 16 // short int
#define AVX_PACKED_CS 8 // complex short int

int CommsLib::find_beacon_avx(const std::vector<std::complex<float>>& iq,
    const std::vector<std::complex<float>>& seq)
{
    std::queue<int> valid_peaks;

    // Original LTS sequence
    int seqLen = seq.size();
    struct timespec tv, tv2;
    clock_gettime(CLOCK_MONOTONIC, &tv);

    // correlate signal with beacon
    std::vector<std::complex<float>> gold_corr_avx
        = CommsLib::correlate_avx(iq, seq);
    clock_gettime(CLOCK_MONOTONIC, &tv2);
#ifdef TEST_BENCH
    double diff1
        = ((tv2.tv_sec - tv.tv_sec) * 1e9 + (tv2.tv_nsec - tv.tv_nsec)) / 1e3;
#endif

    clock_gettime(CLOCK_MONOTONIC, &tv);
    // multiply the corre result with its (gold seq length-) shifted copy
    std::vector<std::complex<float>> gold_auto_corr
        = CommsLib::auto_corr_mult_avx(gold_corr_avx, seqLen);
    // calculate the abs (use the result for peak detection)
    std::vector<float> gold_corr_avx_2 = CommsLib::abs2_avx(gold_auto_corr);
    clock_gettime(CLOCK_MONOTONIC, &tv2);
#ifdef TEST_BENCH
    double diff2
        = ((tv2.tv_sec - tv.tv_sec) * 1e9 + (tv2.tv_nsec - tv.tv_nsec)) / 1e3;
#endif

    // calculate the adaptive theshold
    std::vector<float> consts1(seqLen, 1);
    clock_gettime(CLOCK_MONOTONIC, &tv);
    // calculate the moving sum of the abs of corr result and use as threshold
    std::vector<float> corr_abs_avx = CommsLib::abs2_avx(gold_corr_avx);
    std::vector<float> thresh_avx
        = CommsLib::correlate_avx_s(corr_abs_avx, consts1);
    clock_gettime(CLOCK_MONOTONIC, &tv2);
#ifdef TEST_BENCH
    double diff3
        = ((tv2.tv_sec - tv.tv_sec) * 1e9 + (tv2.tv_nsec - tv.tv_nsec)) / 1e3;
#endif

    // perform thresholding and find peak
    clock_gettime(CLOCK_MONOTONIC, &tv);
    for (size_t i = 0; i < gold_corr_avx_2.size(); i++) {
        if (gold_corr_avx_2[i] > thresh_avx[i])
            valid_peaks.push(i);
    }
    clock_gettime(CLOCK_MONOTONIC, &tv2);
#ifdef TEST_BENCH
    double diff4
        = ((tv2.tv_sec - tv.tv_sec) * 1e9 + (tv2.tv_nsec - tv.tv_nsec)) / 1e3;
#endif

#ifdef TEST_BENCH
    std::cout << "Convolution AVX took " << diff1 << " usec" << std::endl;
    std::cout << "Corr Abs AVX took " << diff2 << " usec" << std::endl;
    std::cout << "Thresh calc AVX took " << diff3 << " usec" << std::endl;
    std::cout << "Peak Detect AVX took " << diff4 << " usec" << std::endl;
    printf("Saving Corr data\n");
    std::string filename = "corr_simd.bin";
    FILE* fc = fopen(filename.c_str(), "wb");
    float* cdata_ptr = (float*)gold_corr_avx_2.data();
    fwrite(cdata_ptr, gold_corr_avx_2.size(), sizeof(float), fc);
    fclose(fc);
    filename = "thresh_simd.bin";
    FILE* fp = fopen(filename.c_str(), "wb");
    float* tdata_ptr = (float*)thresh_avx.data();
    fwrite(tdata_ptr, thresh_avx.size(), sizeof(float), fp);
    fclose(fp);
    filename = "indata.bin";
    FILE* fi = fopen(filename.c_str(), "wb");
    float* idata_ptr = (float*)iq.data();
    fwrite(idata_ptr, iq.size() * 2, sizeof(float), fi);
    fclose(fi);
#endif

    if (valid_peaks.empty()) {
        valid_peaks.push(-1);
    }

    return valid_peaks.front();
}

static inline __m256i __m256_complex_cs16_mult(
    __m256i data1, __m256i data2, bool conj)
{
    const __m256i neg0 = _mm256_set1_epi32(0xFFFF0000);
    const __m256i neg1 = _mm256_set1_epi32(0x00010000);
    const __m256i mix = _mm256_set1_epi32(0x0000FFFF);

    __m256i temp0 = _mm256_xor_si256(data2, neg0);
    temp0 = _mm256_add_epi32(temp0, neg1);

    __m256i temp1 = _mm256_shufflehi_epi16(conj ? temp0 : data2, 0xb1);
    temp1 = _mm256_shufflelo_epi16(temp1, 0xb1);

    __m256i re = _mm256_madd_epi16(data1, conj ? data2 : temp0);
    __m256i im = _mm256_madd_epi16(data1, temp1);

    re = _mm256_srai_epi32(re, 15);
    im = _mm256_srai_epi32(im, 15);

    re = _mm256_and_si256(re, mix);
    im = _mm256_and_si256(im, mix);
    im = _mm256_slli_epi32(im, 0x10);

    return _mm256_or_si256(re, im);
}

std::vector<std::complex<int16_t>> CommsLib::complex_mult_avx(
    std::vector<std::complex<int16_t>> const& f,
    std::vector<std::complex<int16_t>> const& g, const bool conj)
{
    size_t length1 = g.size();
    size_t length0 = f.size();
    size_t res_len = std::min(length0, length1);

    __m256i* in0 = (__m256i*)(f.data());
    __m256i* in1 = (__m256i*)(g.data());
    std::vector<std::complex<int16_t>> out(res_len, 0);
    __m256i* outf = (__m256i*)out.data();

    __m256i data0 __attribute__((aligned(ALIGNMENT)));
    __m256i data1 __attribute__((aligned(ALIGNMENT)));

    __m256i res __attribute__((aligned(ALIGNMENT)));

    size_t vecSize = res_len / AVX_PACKED_CS;
    for (size_t i = 0; i < vecSize; i++) {
        data0 = _mm256_loadu_si256(in0 + i);
        data1 = _mm256_loadu_si256(in1 + i);
        res = __m256_complex_cs16_mult(data0, data1, conj);
        _mm256_storeu_si256(outf + i, res);
    }

    for (size_t i = vecSize * AVX_PACKED_CS; i < res_len; i++) {
        int16_t i0 = f[i].real();
        int16_t i1 = g[i].real();
        int16_t q0 = f[i].imag();
        int16_t q1 = g[i].imag();
        int16_t ires = (int16_t)((i0 * i1 + (conj ? 1 : -1) * q0 * q1) >> 15);
        int16_t qres = (int16_t)((i1 * q0 + (conj ? -1 : 1) * i0 * q1) >> 15);
        out[i] = std::complex<int16_t>(ires, qres);
    }

    return out;
}

inline __m256 CommsLib::__m256_complex_cf32_mult(
    __m256 data1, __m256 data2, bool conj)
{
    __m256 prod0 __attribute__((aligned(ALIGNMENT)));
    __m256 prod1 __attribute__((aligned(ALIGNMENT)));
    __m256 res __attribute__((aligned(ALIGNMENT)));

    // https://stackoverflow.com/questions/39509746
    const __m256 neg0
        = _mm256_setr_ps(1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0);
    const __m256 neg1
        = _mm256_set_ps(1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0);
    prod0 = _mm256_mul_ps(data1, data2); // q1*q2, i1*i2, ...

    /* Step 2: Negate the imaginary elements of vec2 */
    data2 = _mm256_mul_ps(data2, conj ? neg0 : neg1);

    /* Step 3: Switch the real and imaginary elements of vec2 */
    data2 = _mm256_permute_ps(data2, 0xb1);

    /* Step 4: Multiply vec1 and the modified vec2 */
    prod1 = _mm256_mul_ps(data1, data2); // i2*q1, -i1*q2, ...

    /* Horizontally add the elements in vec3 and vec4 */
    res = conj
        ? _mm256_hadd_ps(prod0, prod1)
        : _mm256_hsub_ps(prod0, prod1); // i2*q1+-i1*q2, i1*i2+-q1*q2, ...
    res = _mm256_permute_ps(res, 0xd8);

    return res;
}

std::vector<std::complex<float>> CommsLib::complex_mult_avx(
    std::vector<std::complex<float>> const& f,
    std::vector<std::complex<float>> const& g, const bool conj)
{
    size_t length1 = 2 * g.size();
    size_t length0 = 2 * f.size();
    size_t res_len = std::min(length0, length1);

    float* in0 = (float*)(f.data());
    float* in1 = (float*)(g.data());
    std::vector<std::complex<float>> out(res_len / 2, 0);
    float* outf = (float*)out.data();

    __m256 data0 __attribute__((aligned(ALIGNMENT)));
    __m256 data1 __attribute__((aligned(ALIGNMENT)));

    __m256 res __attribute__((aligned(ALIGNMENT)));

    size_t rem = res_len - (res_len % AVX_PACKED_SP);
    for (size_t i = 0; i < rem; i += AVX_PACKED_SP) {
        data0 = _mm256_loadu_ps(in0 + i);
        data1 = _mm256_loadu_ps(in1 + i);
        res = __m256_complex_cf32_mult(data0, data1, conj);
        _mm256_storeu_ps(outf + i, res);
    }

    for (size_t i = rem; i < res_len; i += 2)
        out[i / 2] = f[i / 2] * (conj ? std::conj(g[i / 2]) : g[i / 2]);

    return out;
}

std::vector<std::complex<float>> CommsLib::auto_corr_mult_avx(
    std::vector<std::complex<float>> const& f, const int dly, const bool conj)
{
#if 0
    size_t length = 2 * f.size();
    float* in = (float*)(f.data());
    std::vector<std::complex<float>> out(length / 2, 0);
    float* outf = (float*)(out.data());
    int d = 2 * dly;

    __m256 data1 __attribute__((aligned(ALIGNMENT)));
    __m256 data2 __attribute__((aligned(ALIGNMENT)));
    __m256 res __attribute__((aligned(ALIGNMENT)));

    size_t rem = length - (length % AVX_PACKED_SP);
    if (rem > 0) {
        for (size_t i = d; i < rem; i += AVX_PACKED_SP) {
            data1 = _mm256_loadu_ps(in + i);
            data2 = _mm256_loadu_ps(in + i - d);
            res = __m256_complex_cf32_mult(data1, data2, conj);
            _mm256_storeu_ps(outf + i, res);
        }
    }
    for (size_t i = rem; i < length; i += 2)
        out[i / 2] = f[i / 2] * (conj ? std::conj(f[i / 2 - dly]) : f[i / 2 - dly]);
    return out;
#else
    std::vector<std::complex<float>> g(f.begin(), f.end() - dly);
    std::vector<std::complex<float>> z(dly, 0);
    g.insert(g.begin(), z.begin(), z.end());
    return CommsLib::complex_mult_avx(f, g, conj);
#endif
}

std::vector<std::complex<int16_t>> CommsLib::auto_corr_mult_avx(
    std::vector<std::complex<int16_t>> const& f, const int dly, const bool conj)
{
    std::vector<std::complex<int16_t>> g(f.begin(), f.end() - dly);
    std::vector<std::complex<int16_t>> z(dly, 0);
    g.insert(g.begin(), z.begin(), z.end());
    return CommsLib::complex_mult_avx(f, g, conj);
}

std::vector<float> CommsLib::abs2_avx(std::vector<std::complex<float>> const& f)
{
    size_t length = 2 * f.size();
    float* in = (float*)(f.data());
    std::vector<float> out(length / 2, 0);
    float* outf = (float*)(out.data());

    __m256 data1 __attribute__((aligned(ALIGNMENT)));
    __m256 data2 __attribute__((aligned(ALIGNMENT)));
    __m256 prod0 __attribute__((aligned(ALIGNMENT)));
    __m256 prod1 __attribute__((aligned(ALIGNMENT)));
    __m256 res __attribute__((aligned(ALIGNMENT)));
    const __m256i perm0
        = _mm256_set_epi32(0x7, 0x6, 0x3, 0x2, 0x5, 0x4, 0x1, 0x0);

    size_t rem = length - (length % (2 * AVX_PACKED_SP));
    if (rem > 0) {
        for (size_t i = 0; i < rem; i += 2 * AVX_PACKED_SP) {
            data1 = _mm256_loadu_ps(in
                + i); // low index go to high bit of __m256 variable apparently
            data2 = _mm256_loadu_ps(in + i + AVX_PACKED_SP);

            prod0 = _mm256_mul_ps(data1, data1);
            prod1 = _mm256_mul_ps(data2, data2);

            res = _mm256_hadd_ps(prod0, prod1);
            res = _mm256_permutevar8x32_ps(res, perm0);
            _mm256_storeu_ps(outf + i / 2, res);
        }
    }

    for (size_t i = rem; i < length; i += 2)
        outf[i / 2] = in[i] * in[i] + in[i + 1] * in[i + 1];
    return out;
}

std::vector<int32_t> CommsLib::abs2_avx(
    std::vector<std::complex<int16_t>> const& f)
{
    size_t len = f.size();

    __m256i* in0 = (__m256i*)(f.data());
    std::vector<int32_t> out(len, 0);
    __m256i* outf = (__m256i*)out.data();

    size_t vecSize = len / AVX_PACKED_CS;
    for (size_t i = 0; i < vecSize; i++) {
        __m256i data0 = _mm256_loadu_si256(in0 + i);
        __m256i mag = _mm256_madd_epi16(data0, data0);
        _mm256_storeu_si256(outf + i, mag);
    }

    for (size_t i = vecSize * AVX_PACKED_CS; i < len; i++) {
        int16_t i0 = f[i].real();
        int16_t q0 = f[i].imag();
        out[i] = i0 * i0 + q0 * q0;
    }

    return out;
}

std::vector<std::complex<int16_t>> CommsLib::correlate_avx(
    std::vector<std::complex<int16_t>> const& f,
    std::vector<std::complex<int16_t>> const& g)
{
    // assuming length0 is larger or equal to length1
    size_t length1 = g.size();

    std::vector<std::complex<int16_t>> in(length1 - 1, 0);
    in.insert(in.end(), f.begin(), f.end());
    size_t length = in.size();

    int16_t* in1 = (int16_t*)(g.data());
    std::vector<std::complex<int16_t>> out(length, 0);

    size_t sz = sizeof(std::complex<int16_t>);

    __m256i seq_samp[length1] __attribute__((aligned(ALIGNMENT)));

    for (size_t i = 0; i < length1; i++) {
        __m256i samp_i = _mm256_set1_epi16(in1[i * 2]);
        __m256i samp_q = _mm256_set1_epi16(in1[i * 2 + 1]);
        seq_samp[i] = _mm256_unpackhi_epi16(samp_i, samp_q);
    }

    for (size_t i = 0; i < (length - length1); i += AVX_PACKED_SI) {
        __m256i accm = _mm256_set1_epi16(0x0);
        for (size_t j = 0; j < length1; j++) {
            __m256i* in_temp = (__m256i*)(in.data() + (i + j) * sz);
            __m256i data = _mm256_loadu_si256(in_temp);
            __m256i prod = __m256_complex_cs16_mult(data, seq_samp[j], true);
            accm = _mm256_add_epi16(prod, accm);
        }
        __m256i* out_temp = (__m256i*)(out.data() + i * sz);
        _mm256_storeu_si256(out_temp, accm);
    }

    return out;
}

std::vector<std::complex<float>> CommsLib::correlate_avx(
    std::vector<std::complex<float>> const& f,
    std::vector<std::complex<float>> const& g)
{
    // assuming length0 is larger or equal to length1
    size_t length0 = f.size();
    size_t length1 = g.size();

    std::vector<std::complex<float>> in(length0 + length1 - 1, 0);
    //std::copy(f.begin(), f.end(), std::back_inserter(in));
    //in.insert(in.end(), f.begin(), f.end());
    for (size_t i = length1 - 1; i < in.size(); i++) {
        size_t j = i - length1 + 1;
        in[i] = f[j];
    }
    size_t length = in.size();

    float* in0 = (float*)(in.data());
    float* in1 = (float*)(g.data());
    std::vector<std::complex<float>> out(length, 0);
    float* outf = (float*)out.data();

    __m256 seq_samp[length1] __attribute__((aligned(ALIGNMENT)));

    for (size_t i = 0; i < length1; i++) {
        __m256 samp_i = _mm256_broadcast_ss(&in1[i * 2]);
        __m256 samp_q = _mm256_broadcast_ss(&in1[i * 2 + 1]);
        seq_samp[i] = _mm256_shuffle_ps(samp_i, samp_q, 0x0);
        seq_samp[i] = _mm256_permute_ps(seq_samp[i], 0xd8);
    }

    for (size_t i = 0; i < 2 * (length - length1); i += AVX_PACKED_SP) {
        __m256 accm = _mm256_setzero_ps();
        for (size_t j = 0; j < length1; j++) {
            __m256 data = _mm256_loadu_ps(in0 + i + j * 2);
            __m256 prod = __m256_complex_cf32_mult(data, seq_samp[j], true);
            accm = _mm256_add_ps(prod, accm);
        }
        _mm256_storeu_ps(outf + i, accm);
    }
    return out;
}

std::vector<float> CommsLib::correlate_avx_s(
    std::vector<float> const& f, std::vector<float> const& g)
{
    // assuming length0 is larger or equal to length1
    size_t length0 = f.size();
    size_t length1 = g.size();

    std::vector<float> in(length0 + length1 - 1, 0);
    //in.insert(in.end(), f.begin(), f.end());
    for (size_t i = length1 - 1; i < in.size(); i++) {
        size_t j = i - length1 + 1;
        in[i] = f[j];
    }
    size_t length = in.size();

    float* in0 = (float*)(in.data());
    float* in1 = (float*)(g.data());
    std::vector<float> out(length, 0);
    float* outf = (float*)out.data();

    __m256 data __attribute__((aligned(ALIGNMENT)));
    __m256 prod __attribute__((aligned(ALIGNMENT)));
    __m256 accm __attribute__((aligned(ALIGNMENT)));

    __m256 seq_samp[length1] __attribute__((aligned(ALIGNMENT)));

    // Repeat the kernel across the vector
    for (size_t i = 0; i < length1; i++) {
        seq_samp[i] = _mm256_broadcast_ss(&in1[i]);
    }

    for (size_t i = 0; i < (length - length1); i += AVX_PACKED_SP) {
        accm = _mm256_setzero_ps();
        for (size_t j = 0; j < length1; j++) {
            data = _mm256_loadu_ps(in0 + i + j);
            prod = _mm256_mul_ps(data, seq_samp[j]);
            accm = _mm256_add_ps(prod, accm);
        }
        _mm256_storeu_ps(outf + i, accm);
    }
    return out;
}
