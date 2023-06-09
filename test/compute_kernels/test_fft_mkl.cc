/**
 * @file test_fft_mkl.cc
 * @brief Testing functions for fft using the mkl library
 */

#include <immintrin.h>

#include <cmath>
#include <iostream>

#include "cpu_attach.h"
#include "datatype_conversion.h"
#include "memory_manage.h"
#include "mkl_dfti.h"

static double fft_get_time(void) {
  struct timespec tv;
  clock_gettime(CLOCK_MONOTONIC, &tv);
  return tv.tv_sec + tv.tv_nsec / 1000000000.0;
}

int flushCache() {
  const size_t bigger_than_cachesize = 100 * 1024 * 1024;  // 100 * 1024 *
                                                           // 1024;
  long* p = new long[bigger_than_cachesize];
  // When you want to "flush" cache.
  for (long long i = 0; i < bigger_than_cachesize; i++) {
    p[i] = rand();
  }
  delete p;
}

int flushCacheRuntime(long* p, long long p_size) {
  long temp;
  for (int i = 0; i < p_size; i++) {
    temp = p[i];
  }
}

static double bench_fft_1d_mkl(unsigned N, unsigned iterations) {
  float _Complex* input =
      static_cast<float _Complex*>(Agora_memory::padded_aligned_alloc(
          Agora_memory::Alignment_t::kAlign64, N * sizeof(float _Complex)));
  float _Complex* output =
      static_cast<float _Complex*>(Agora_memory::padded_aligned_alloc(
          Agora_memory::Alignment_t::kAlign64, N * sizeof(float _Complex)));
  DFTI_DESCRIPTOR_HANDLE my_desc1_handle;
  MKL_LONG status;
  //...put input data into x[0],...,x[31]; y[0],...,y[31]
  status =
      DftiCreateDescriptor(&my_desc1_handle, DFTI_SINGLE, DFTI_COMPLEX, 1, N);
  status = DftiCommitDescriptor(my_desc1_handle);

  srand(0);
  for (unsigned i = 0; i < N; i++) {
    float real = (float)rand() / RAND_MAX - 0.5f;
    ;
    float imag = (float)rand() / RAND_MAX - 0.5f;
    ;
    input[i] = real + _Complex_I * imag;
  }

  double start_time = fft_get_time();
  for (unsigned i = 0; i < iterations; i++) {
    status = DftiComputeForward(my_desc1_handle, input);
  }
  double end_time = fft_get_time();

  status = DftiFreeDescriptor(&my_desc1_handle);

  return end_time - start_time;
}

static double bench_ifft_1d_mkl(unsigned N, unsigned iterations) {
  float _Complex* input =
      static_cast<float _Complex*>(Agora_memory::padded_aligned_alloc(
          Agora_memory::Alignment_t::kAlign64, N * sizeof(float _Complex)));
    float _Complex* output
        = static_cast<float _Complex*>(Agora_memory::padded_aligned_alloc(Agora_memory::Alignment_t::kAlign64, N * sizeof(float _Complex));
    DFTI_DESCRIPTOR_HANDLE my_desc1_handle;
    MKL_LONG status;
    //...put input data into x[0],...,x[31]; y[0],...,y[31]
    status = DftiCreateDescriptor(
        &my_desc1_handle, DFTI_SINGLE, DFTI_COMPLEX, 1, N);
    status = DftiCommitDescriptor(my_desc1_handle);

    srand(0);
    for (unsigned i = 0; i < N; i++) {
    float real = (float)rand() / RAND_MAX - 0.5f;
    ;
    float imag = (float)rand() / RAND_MAX - 0.5f;
    ;
    input[i] = real + _Complex_I * imag;
    }

    double start_time = fft_get_time();
    for (unsigned i = 0; i < iterations; i++) {
    status = DftiComputeBackward(my_desc1_handle, input);
    }
    double end_time = fft_get_time();

    status = DftiFreeDescriptor(&my_desc1_handle);

    return end_time - start_time;
}

static double bench_fft_1d_mkl_out(unsigned N, unsigned iterations) {
  float _Complex* input =
      static_cast<float _Complex*>(Agora_memory::padded_aligned_alloc(
          Agora_memory::Alignment_t::kAlign64, N * sizeof(float _Complex)));
  float _Complex* output =
      static_cast<float _Complex*>(Agora_memory::padded_aligned_alloc(
          Agora_memory::Alignment_t::kAlign64, N * sizeof(float _Complex)));
  DFTI_DESCRIPTOR_HANDLE my_desc1_handle;
  MKL_LONG status;
  //...put input data into x[0],...,x[31]; y[0],...,y[31]
  status =
      DftiCreateDescriptor(&my_desc1_handle, DFTI_SINGLE, DFTI_COMPLEX, 1, N);
  status = DftiSetValue(my_desc1_handle, DFTI_PLACEMENT, DFTI_NOT_INPLACE);
  status = DftiCommitDescriptor(my_desc1_handle);

  srand(0);
  for (unsigned i = 0; i < N; i++) {
    float real = (float)rand() / RAND_MAX - 0.5f;
    ;
    float imag = (float)rand() / RAND_MAX - 0.5f;
    ;
    input[i] = real + _Complex_I * imag;
  }

  double start_time = fft_get_time();
  for (unsigned i = 0; i < iterations; i++) {
    status = DftiComputeForward(my_desc1_handle, input, output);
  }
  double end_time = fft_get_time();

  status = DftiFreeDescriptor(&my_desc1_handle);

  return end_time - start_time;
}

static double bench_data_type_convert(unsigned N, unsigned iterations) {
  short* input_buffer = static_cast<short*>(Agora_memory::padded_aligned_alloc(
      Agora_memory::Alignment_t::kAlign64, 2 * N * sizeof(short) * 10000));
  float* input_buffer_float =
      static_cast<float*>(Agora_memory::padded_aligned_alloc(
          Agora_memory::Alignment_t::kAlign64, 2 * N * sizeof(float) * 10000));
  float* output_buffer = static_cast<float*>(Agora_memory::padded_aligned_alloc(
      Agora_memory::Alignment_t::kAlign64, 2 * N * sizeof(float) * 10000));

  long long bigger_than_cachesize = 1000 * 1024 * 1024;  // 100 * 1024 * 1024;
  long* p = new long[bigger_than_cachesize];

  float csi_format_offset = 1.0 / kShrtFltConvFactor;

  srand(0);
  for (unsigned i = 0; i < 2 * N * 10000; i++) {
    input_buffer[i] = (short)(((float)rand() / RAND_MAX - 0.5f) * 1000);
  }

  for (unsigned i = 0; i < 2 * N; i++) {
    input_buffer_float[i] = (float)rand() / RAND_MAX - 0.5f;
  }

  // flushCache();
  double duration = 0;
  double start_time = fft_get_time();
  for (unsigned i = 0; i < iterations; i++) {
    // flushCache();
    // flushCacheRuntime(p, bigger_than_cachesize);

    short* input = input_buffer;              // + i % 10000 * N;
    float* input_float = input_buffer_float;  // + i% 10000 * N;
    float* output = output_buffer;            // + i% 10000 * N;
    // start_time = fft_get_time();
    const __m256 magic =
        _mm256_set1_ps(float((1 << 23) + (1 << 15)) / kShrtFltConvFactor);
    const __m256i magic_i = _mm256_castps_si256(magic);
    for (int j = 0; j < N * 2; j += 16) {
      // get input:
      // __m128i val =
      // _mm_set1_epi16(100);//_mm_load_si128((__m128i*)(input + j)); //
      // port 2,3

      __m128i val = _mm_load_si128((__m128i*)(input + j));
      __m128i val1 = _mm_load_si128((__m128i*)(input + j + 8));
      // interleave with 0x0000
      __m256i val_unpacked = _mm256_cvtepu16_epi32(val);  // port 5
      /// convert by xor-ing and subtracting magic value:
      // VPXOR avoids port5 bottlenecks on Intel CPUs before SKL
      __m256i val_f_int =
          _mm256_xor_si256(val_unpacked, magic_i);     // port 0,1,5
      __m256 val_f = _mm256_castsi256_ps(val_f_int);   // no instruction
      __m256 converted = _mm256_sub_ps(val_f, magic);  // port 1,5 ?
      // store:
      // __m256 converted = _mm256_set1_ps(0);
      _mm256_store_ps(output + j, converted);  // port 2,3,4,7

      // __m128i val1 = _mm_load_si128((__m128i*)(input + j + 8));
      // interleave with 0x0000
      __m256i val_unpacked1 = _mm256_cvtepu16_epi32(val1);  // port 5
      /// convert by xor-ing and subtracting magic value:
      // VPXOR avoids port5 bottlenecks on Intel CPUs before SKL
      __m256i val_f_int1 =
          _mm256_xor_si256(val_unpacked1, magic_i);      // port 0,1,5
      __m256 val_f1 = _mm256_castsi256_ps(val_f_int1);   // no instruction
      __m256 converted1 = _mm256_sub_ps(val_f1, magic);  // port 1,5 ?
      // store:
      // __m256 converted = _mm256_set1_ps(0);
      _mm256_store_ps(output + j + 8, converted1);  // port 2,3,4,7
    }

    // __m256 format_offset = _mm256_set1_ps(csi_format_offset);
    // for (int j = 0; j < N * 2; j += 8)
    // {
    //     //  Load 8 16-bit shorts.
    //     //  vi = {a,b,c,d,e,f,g,h}
    //     __m128i vi = _mm_load_si128((__m128i*)(input + j));

    //     //  Convert to 32-bit integers
    //     __m256i vi0 = _mm256_cvtepi16_epi32(vi);

    //     //  Convert to float
    //     __m256 vf0 = _mm256_cvtepi32_ps(vi0);

    //     //  Multiply
    //     vf0 = _mm256_mul_ps(vf0, format_offset);

    //     //  Store
    //     _mm256_store_ps(output + j, vf0);

    // }

    // for(int j = 0; j < N * 2; j++)
    //     output[j] = input[j] * csi_format_offset;
    // duration += fft_get_time()-start_time;
  }
  double end_time = fft_get_time();

  return end_time - start_time;
}

static void demod_16qam_loop(float* vec_in, uint8_t* vec_out, int ue_num) {
  float float_val = 0.6325;
  for (int i = 0; i < ue_num; i++) {
    float real_val = *(vec_in + i * 2);
    float imag_val = *(vec_in + i * 2 + 1);

    *(vec_out + i) = 0;
    if (real_val > 0) *(vec_out + i) |= 1UL << 3;
    //*(vec_out + i) += 8;
    if (std::abs(real_val) < float_val) *(vec_out + i) |= 1UL << 2;
    //*(vec_out + i) += 4;
    if (imag_val > 0) *(vec_out + i) |= 1UL << 1;
    //*(vec_out + i) += 2;
    if (std::abs(imag_val) < float_val) *(vec_out + i) |= 1UL;
    //*(vec_out + i) += 1;
  }
}

static void demod_16qam_loop2(float* vec_in, uint8_t* vec_out, int ue_num) {
  float float_val = 0.6325;
  for (int i = 0; i < ue_num; i++) {
    float real_val = *(vec_in + i * 2);
    float imag_val = *(vec_in + i * 2 + 1);

    *(vec_out + i) = 0;
    if (real_val > 0) *(vec_out + i) |= 1UL << 3;
    //*(vec_out + i) += 8;
    if (std::abs(real_val) < float_val) *(vec_out + i) |= 1UL << 2;
    //*(vec_out + i) += 4;
    if (imag_val > 0) *(vec_out + i) |= 1UL << 1;
    //*(vec_out + i) += 2;
    if (std::abs(imag_val) < float_val) *(vec_out + i) |= 1UL;
    //*(vec_out + i) += 1;
  }
}

static double bench_demod(unsigned N, unsigned iterations) {
  float* input_buffer = static_cast<float*>(Agora_memory::padded_aligned_alloc(
      Agora_memory::Alignment_t::kAlign64, 2 * N * sizeof(float) * 10000));
  uint8_t* output_buffer = static_cast<uint8_t*>(
      Agora_memory::padded_aligned_alloc(Agora_memory::Alignment_t::kAlign64,
                                         2 * N * sizeof(uint8_t) * 10000));

  long long bigger_than_cachesize = 1000 * 1024 * 1024;  // 100 * 1024 * 1024;
  long* p = new long[bigger_than_cachesize];

  float csi_format_offset = 1.0 / kShrtFltConvFactor;

  srand(0);
  for (unsigned i = 0; i < 2 * N * 10000; i++) {
    input_buffer[i] = 2 * ((float)rand() / RAND_MAX - 0.5f);
  }

  float _Complex* input =
      static_cast<float _Complex*>(Agora_memory::padded_aligned_alloc(
          Agora_memory::Alignment_t::kAlign64, 2048 * sizeof(float _Complex)));
  float _Complex* output =
      static_cast<float _Complex*>(Agora_memory::padded_aligned_alloc(
          Agora_memory::Alignment_t::kAlign64, 2048 * sizeof(float _Complex)));

  DFTI_DESCRIPTOR_HANDLE my_desc1_handle;
  MKL_LONG status;
  //...put input data into x[0],...,x[31]; y[0],...,y[31]
  status =
      DftiCreateDescriptor(&my_desc1_handle, DFTI_SINGLE, DFTI_COMPLEX, 1, N);
  status = DftiCommitDescriptor(my_desc1_handle);

  srand(0);
  for (unsigned i = 0; i < N; i++) {
    float real = (float)rand() / RAND_MAX - 0.5f;
    ;
    float imag = (float)rand() / RAND_MAX - 0.5f;
    ;
    input[i] = real + _Complex_I * imag;
  }

  // flushCache();
  double duration = 0;
  double start_time = fft_get_time();
  start_time = fft_get_time();
  for (unsigned i = 0; i < iterations; i++) {
    // flushCache();
    // flushCacheRuntime(p, bigger_than_cachesize);

    float* input_ptr = input_buffer;  // + i % 10000 * N;
    uint8_t* output = output_buffer;  // + i% 10000 * 64;
    start_time = fft_get_time();
    // _mm_prefetch((char *)(output+64), _MM_HINT_T1);
    demod_16qam_loop(input_ptr, output, N);
    duration += fft_get_time() - start_time;
    // demod_16qam_loop2(input_ptr, output, N);
    // status = DftiComputeForward( my_desc1_handle, input);
    // bench_ifft_1d_mkl(2048, 10);
    // bench_fft_1d_mkl_out(2048, 10);
    // bench_data_type_convert(2048, 1);
  }
  // double end_time = fft_get_time();
  // duration = end_time - start_time;
  DftiFreeDescriptor(&my_desc1_handle);
  return duration;
}

static void run_benchmark_1d(unsigned N, unsigned iterations) {
  double flops = 5.0 * N * log2(N);  // Estimation
  double fft_time_fft1 = bench_fft_1d_mkl(N, iterations);
  double fft_time_fft2 = bench_fft_1d_mkl(N, iterations);
  double fft_time_fft3 = bench_fft_1d_mkl(N, iterations);
  double fft_time_fft4 = bench_fft_1d_mkl(N, iterations);
  double fft_time_ifft = bench_fft_1d_mkl(N, iterations);
  flops *= iterations;

  double fft_mflops_fft1 = flops / (1000000.0 * fft_time_fft1);
  double fft_mflops_fft2 = flops / (1000000.0 * fft_time_fft2);
  double fft_mflops_fft3 = flops / (1000000.0 * fft_time_fft3);
  double fft_mflops_fft4 = flops / (1000000.0 * fft_time_fft4);
  double fft_mflops_ifft = flops / (1000000.0 * fft_time_ifft);

  std::printf("FFT :              %06u %12.3f Mflops %12.3f us iteration\n", N,
              fft_mflops_fft1, 1000000.0 * fft_time_fft1 / iterations);
  std::printf("FFT :              %06u %12.3f Mflops %12.3f us iteration\n", N,
              fft_mflops_fft2, 1000000.0 * fft_time_fft2 / iterations);
  std::printf("FFT :              %06u %12.3f Mflops %12.3f us iteration\n", N,
              fft_mflops_fft3, 1000000.0 * fft_time_fft3 / iterations);
  std::printf("FFT :              %06u %12.3f Mflops %12.3f us iteration\n", N,
              fft_mflops_fft4, 1000000.0 * fft_time_fft4 / iterations);

  std::printf("IFFT :              %06u %12.3f Mflops %12.3f us iteration\n", N,
              fft_mflops_ifft, 1000000.0 * fft_time_ifft / iterations);
}

static void run_benchmark_1d_ifft(unsigned N, unsigned iterations) {
  double flops = 5.0 * N * log2(N);  // Estimation
  double fft_time_fft1 = bench_ifft_1d_mkl(N, iterations);
  double fft_time_fft2 = bench_ifft_1d_mkl(N, iterations);
  double fft_time_fft3 = bench_ifft_1d_mkl(N, iterations);
  double fft_time_fft4 = bench_ifft_1d_mkl(N, iterations);
  double fft_time_ifft = bench_ifft_1d_mkl(N, iterations);
  flops *= iterations;

  double fft_mflops_fft1 = flops / (1000000.0 * fft_time_fft1);
  double fft_mflops_fft2 = flops / (1000000.0 * fft_time_fft2);
  double fft_mflops_fft3 = flops / (1000000.0 * fft_time_fft3);
  double fft_mflops_fft4 = flops / (1000000.0 * fft_time_fft4);
  double fft_mflops_ifft = flops / (1000000.0 * fft_time_ifft);

  std::printf("IFFT :              %06u %12.3f Mflops %12.3f us iteration\n", N,
              fft_mflops_fft1, 1000000.0 * fft_time_fft1 / iterations);
  std::printf("IFFT :              %06u %12.3f Mflops %12.3f us iteration\n", N,
              fft_mflops_fft2, 1000000.0 * fft_time_fft2 / iterations);
  std::printf("IFFT :              %06u %12.3f Mflops %12.3f us iteration\n", N,
              fft_mflops_fft3, 1000000.0 * fft_time_fft3 / iterations);
  std::printf("IFFT :              %06u %12.3f Mflops %12.3f us iteration\n", N,
              fft_mflops_fft4, 1000000.0 * fft_time_fft4 / iterations);

  std::printf("IFFT :              %06u %12.3f Mflops %12.3f us iteration\n", N,
              fft_mflops_ifft, 1000000.0 * fft_time_ifft / iterations);
}

static void run_benchmark_data_type(unsigned N, unsigned iterations) {
  double flops = 5.0 * N * log2(N);  // Estimation
  double fft_time_fft1 = bench_data_type_convert(N, iterations);
  double fft_time_fft2 = bench_data_type_convert(N, iterations);
  double fft_time_fft3 = bench_data_type_convert(N, iterations);
  double fft_time_fft4 = bench_data_type_convert(N, iterations);
  double fft_time_ifft = bench_data_type_convert(N, iterations);
  flops *= iterations;

  double fft_mflops_fft1 = flops / (1000000.0 * fft_time_fft1);
  double fft_mflops_fft2 = flops / (1000000.0 * fft_time_fft2);
  double fft_mflops_fft3 = flops / (1000000.0 * fft_time_fft3);
  double fft_mflops_fft4 = flops / (1000000.0 * fft_time_fft4);
  double fft_mflops_ifft = flops / (1000000.0 * fft_time_ifft);

  std::printf(
      "data type convert :              %06u %12.3f Mflops %12.3f us "
      "iteration\n",
      N, fft_mflops_fft1, 1000000.0 * fft_time_fft1 / iterations);
  std::printf(
      "data type convert :              %06u %12.3f Mflops %12.3f us "
      "iteration\n",
      N, fft_mflops_fft2, 1000000.0 * fft_time_fft2 / iterations);
  std::printf(
      "data type convert :              %06u %12.3f Mflops %12.3f us "
      "iteration\n",
      N, fft_mflops_fft3, 1000000.0 * fft_time_fft3 / iterations);
  std::printf(
      "data type convert :              %06u %12.3f Mflops %12.3f us "
      "iteration\n",
      N, fft_mflops_fft4, 1000000.0 * fft_time_fft4 / iterations);

  std::printf(
      "data type convert :              %06u %12.3f Mflops %12.3f us "
      "iteration\n",
      N, fft_mflops_ifft, 1000000.0 * fft_time_ifft / iterations);
}

static void run_benchmark_demod(unsigned N, unsigned iterations) {
  double flops = 5.0 * N * log2(N);  // Estimation
  double fft_time_fft1 = bench_demod(N, iterations);
  double fft_time_fft2 = bench_demod(N, iterations);
  double fft_time_fft3 = bench_demod(N, iterations);
  double fft_time_fft4 = bench_demod(N, iterations);
  double fft_time_ifft = bench_demod(N, iterations);
  flops *= iterations;

  double fft_mflops_fft1 = flops / (1000000.0 * fft_time_fft1);
  double fft_mflops_fft2 = flops / (1000000.0 * fft_time_fft2);
  double fft_mflops_fft3 = flops / (1000000.0 * fft_time_fft3);
  double fft_mflops_fft4 = flops / (1000000.0 * fft_time_fft4);
  double fft_mflops_ifft = flops / (1000000.0 * fft_time_ifft);

  std::printf("demod :              %06u %12.3f Mflops %12.3f us iteration\n",
              N, fft_mflops_fft1, 1000000.0 * fft_time_fft1 / iterations);
  std::printf("demod :              %06u %12.3f Mflops %12.3f us iteration\n",
              N, fft_mflops_fft2, 1000000.0 * fft_time_fft2 / iterations);
  std::printf("demod :              %06u %12.3f Mflops %12.3f us iteration\n",
              N, fft_mflops_fft3, 1000000.0 * fft_time_fft3 / iterations);
  std::printf("demod :              %06u %12.3f Mflops %12.3f us iteration\n",
              N, fft_mflops_fft4, 1000000.0 * fft_time_fft4 / iterations);

  std::printf("demod :              %06u %12.3f Mflops %12.3f us iteration\n",
              N, fft_mflops_ifft, 1000000.0 * fft_time_ifft / iterations);
}

int main(int argc, char* argv[]) {
  // putenv("MKL_THREADING_LAYER=sequential");
  // putenv("MKL_ENABLE_INSTRUCTIONS=AVX2");
  // putenv("MKL_VERBOSE=1");
  if (argc != 4) {
    std::fprintf(stderr, "Usage: %s [iterations] [Nx] [mode]\n", argv[0]);
    return 1;
  }

  int main_core_id = 2;
  if (stick_this_thread_to_core(main_core_id) != 0) {
    std::printf("Main thread: stitch main thread to core %d failed\n",
                main_core_id);
    throw std::runtime_error(
        "FFT MKL Test: Main thread: stitch main thread to core ");
  } else {
    std::printf("Main thread: stitch main thread to core %d succeeded\n",
                main_core_id);
  }

  if (argc == 4) {
    unsigned iterations = strtoul(argv[1], NULL, 0);
    unsigned Nx = strtoul(argv[2], NULL, 0);
    unsigned mode = strtoul(argv[3], NULL, 0);
    if (mode == 0)
      run_benchmark_1d(Nx, iterations);
    else if (mode == 1)
      run_benchmark_1d_ifft(Nx, iterations);
    else if (mode == 2)
      run_benchmark_data_type(Nx, iterations);
    else if (mode == 3)
      run_benchmark_demod(Nx, iterations);
  }
}
