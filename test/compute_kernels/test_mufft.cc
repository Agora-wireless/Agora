/**
 * @file test_mufft.cc
 * @brief Testing functions for benchmarking mu fft routines
 */
#include <immintrin.h>

#include "cpu_attach.h"
#include "mufft/fft.h"

static double mufft_get_time(void) {
  struct timespec tv;
  clock_gettime(CLOCK_MONOTONIC, &tv);
  return tv.tv_sec + tv.tv_nsec / 1000000000.0;
}

int flushCache() {
  const size_t bigger_than_cachesize = 2 * 1024;  // 100 * 1024 * 1024;
  long *p = new long[bigger_than_cachesize];
  // When you want to "flush" cache.
  for (int i = 0; i < bigger_than_cachesize; i++) {
    p[i] = rand();
  }
  delete p;
}

static double bench_fft_1d(unsigned N, unsigned iterations, int direction) {
  complex float *input =
      (complex float *)mufft_alloc(N * sizeof(complex float));
  complex float *output =
      (complex float *)mufft_alloc(N * sizeof(complex float));

  srand(0);
  for (unsigned i = 0; i < N; i++) {
    float real = (float)rand() / RAND_MAX - 0.5f;
    float imag = (float)rand() / RAND_MAX - 0.5f;
    input[i] = real + _Complex_I * imag;
  }

  mufft_plan_1d *muplan =
      mufft_create_plan_1d_c2c(N, direction, MUFFT_FLAG_CPU_ANY);

  double start_time = mufft_get_time();
  for (unsigned i = 0; i < iterations; i++) {
    mufft_execute_plan_1d(muplan, output, input);
  }
  double end_time = mufft_get_time();

  mufft_free(input);
  mufft_free(output);
  mufft_free_plan_1d(muplan);

  return end_time - start_time;
}

static double bench_fft_1d_0(unsigned N, unsigned iterations, int direction) {
  complex float *input =
      (complex float *)mufft_alloc(N * sizeof(complex float));
  complex float *output =
      (complex float *)mufft_alloc(N * sizeof(complex float));

  srand(0);
  for (unsigned i = 0; i < N; i++) {
    float real = 0;
    float imag = 0;
    input[i] = real + _Complex_I * imag;
  }

  mufft_plan_1d *muplan =
      mufft_create_plan_1d_c2c(N, direction, MUFFT_FLAG_CPU_ANY);

  double start_time = mufft_get_time();
  for (unsigned i = 0; i < iterations; i++) {
    // flushCache();
    // for (int j = 0; j < 2048 * 2; j+=8) {
    //     // _mm256_load_ps((float *)input+j);
    //     __m256 converted = _mm256_set1_ps(0);
    //     _mm256_store_ps((float *)input + j, converted);
    // }
    mufft_execute_plan_1d(muplan, output, input);
  }
  double end_time = mufft_get_time();

  mufft_free(input);
  mufft_free(output);
  mufft_free_plan_1d(muplan);

  return end_time - start_time;
}

static void run_benchmark_1d(unsigned N, unsigned iterations) {
  double flops = 5.0 * N * log2(N);  // Estimation
  // double mufft_time_fft1 = bench_fft_1d(N, iterations, MUFFT_FORWARD);
  // double mufft_time_fft2 = bench_fft_1d(N, iterations, MUFFT_FORWARD);
  // double mufft_time_fft3 = bench_fft_1d(N, iterations, MUFFT_FORWARD);
  // double mufft_time_fft4 = bench_fft_1d(N, iterations, MUFFT_FORWARD);
  // double mufft_time_ifft = bench_fft_1d(N, iterations, MUFFT_FORWARD);
  double mufft_time_fft1 = bench_fft_1d_0(N, iterations, MUFFT_INVERSE);
  double mufft_time_fft2 = bench_fft_1d(N, iterations, MUFFT_INVERSE);
  double mufft_time_fft3 = bench_fft_1d(N, iterations, MUFFT_INVERSE);
  double mufft_time_fft4 = bench_fft_1d(N, iterations, MUFFT_INVERSE);
  double mufft_time_ifft = bench_fft_1d(N, iterations, MUFFT_INVERSE);
  flops *= iterations;

  double mufft_mflops_fft1 = flops / (1000000.0 * mufft_time_fft1);
  double mufft_mflops_fft2 = flops / (1000000.0 * mufft_time_fft2);
  double mufft_mflops_fft3 = flops / (1000000.0 * mufft_time_fft3);
  double mufft_mflops_fft4 = flops / (1000000.0 * mufft_time_fft4);
  double mufft_mflops_ifft = flops / (1000000.0 * mufft_time_ifft);

  std::printf("FFT :              %06u %12.3f Mflops %12.3f us iteration\n", N,
              mufft_mflops_fft1, 1000000.0 * mufft_time_fft1 / iterations);
  std::printf("FFT :              %06u %12.3f Mflops %12.3f us iteration\n", N,
              mufft_mflops_fft2, 1000000.0 * mufft_time_fft2 / iterations);
  std::printf("FFT :              %06u %12.3f Mflops %12.3f us iteration\n", N,
              mufft_mflops_fft3, 1000000.0 * mufft_time_fft3 / iterations);
  std::printf("FFT :              %06u %12.3f Mflops %12.3f us iteration\n", N,
              mufft_mflops_fft4, 1000000.0 * mufft_time_fft4 / iterations);

  std::printf("IFFT :              %06u %12.3f Mflops %12.3f us iteration\n", N,
              mufft_mflops_ifft, 1000000.0 * mufft_time_ifft / iterations);
}

int main(int argc, char *argv[]) {
  if (argc != 3) {
    std::fprintf(stderr, "Usage: %s [iterations] [Nx]\n", argv[0]);
    return 1;
  }

  // int main_core_id = 2;
  // if(stick_this_thread_to_core(main_core_id) != 0) {
  //     std::printf("Main thread: stitch main thread to core %d failed\n",
  //     main_core_id); std::exit(0);
  // }
  // else {
  //     std::printf("Main thread: stitch main thread to core %d succeeded\n",
  //     main_core_id);
  // }

  if (argc == 3) {
    unsigned iterations = strtoul(argv[1], NULL, 0);
    unsigned Nx = strtoul(argv[2], NULL, 0);
    run_benchmark_1d(Nx, iterations);
  }
}