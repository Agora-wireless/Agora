/**
 * @file test_matrix.cc
 * @brief Testing functions for zf matrix computations
 */

#include <sys/resource.h>
#include <sys/time.h>

#include "armadillo"
#include "cpu_attach.h"

static double test_get_time(void) {
  struct timespec tv;
  clock_gettime(CLOCK_MONOTONIC, &tv);
  return tv.tv_sec + tv.tv_nsec / 1000000000.0;
}

void flushCache() {
  const size_t bigger_than_cachesize = 100 * 1024 * 1024;
  auto* p = new long[bigger_than_cachesize];
  // When you want to "flush" cache.
  for (int i = 0; i < bigger_than_cachesize; i++) {
    p[i] = rand();
  }

  delete[] p;
}

static double bench_ZF_warmup(unsigned Nx, unsigned Ny, unsigned iterations) {
  srand(time(NULL));
  arma::fmat real_H = randn<arma::fmat>(Nx, Ny);
  arma::fmat imag_H = randn<arma::fmat>(Nx, Ny);
  arma::cx_fmat mat_input(real_H, imag_H);
  arma::cx_fmat mat_output(Ny, Nx);

  // __itt_resume();
  double start_time = test_get_time();
  for (unsigned i = 0; i < iterations; i++) {
    pinv(mat_output, mat_input, 1e-2, "dc");
  }
  double end_time = test_get_time();
  // __itt_pause();

  return end_time - start_time;
}

static double bench_ZF(unsigned Nx, unsigned Ny, unsigned iterations) {
  srand(time(NULL));
  arma::fmat real_H = randn<arma::fmat>(Nx, Ny);
  arma::fmat imag_H = randn<arma::fmat>(Nx, Ny);
  arma::cx_fmat mat_input(real_H, imag_H);
  arma::cx_fmat mat_output(Ny, Nx);

  // std::cout<<mat_input<<std::endl;

  // __itt_resume();
  double start_time = test_get_time();
  for (unsigned i = 0; i < iterations; i++) {
    pinv(mat_output, mat_input, 1e-2, "dc");
  }
  double end_time = test_get_time();
  // __itt_pause();

  return end_time - start_time;
}

static double bench_multiply_dim1(unsigned Nx, unsigned Ny,
                                  unsigned iterations) {
  srand(0);
  arma::fmat real_right = randn<arma::fmat>(Nx, Ny);
  arma::fmat imag_right = randn<arma::fmat>(Nx, Ny);
  arma::fmat real_left = randn<arma::fmat>(1, Nx);
  arma::fmat imag_left = randn<arma::fmat>(1, Nx);
  arma::cx_fmat mat_right(real_right, imag_right);
  arma::cx_fmat mat_left(real_left, imag_left);
  arma::cx_fmat result(1, Ny);

  double start_time = test_get_time();
  // __itt_resume();
  for (unsigned i = 0; i < iterations; i++) {
    result = mat_left * mat_right;
  }
  // __itt_pause();
  double end_time = test_get_time();

  return end_time - start_time;
}

static double bench_multiply_dim2(unsigned Nx, unsigned Ny,
                                  unsigned iterations) {
  srand(0);
  arma::fmat real_left = randn<arma::fmat>(Nx, Ny);
  arma::fmat imag_left = randn<arma::fmat>(Nx, Ny);
  arma::fmat real_right = randn<arma::fmat>(Ny, 1);
  arma::fmat imag_right = randn<arma::fmat>(Ny, 1);
  arma::cx_fmat mat_right(real_right, imag_right);
  arma::cx_fmat mat_left(real_left, imag_left);
  arma::cx_fmat result(Nx, 1);

  double start_time = test_get_time();
  // __itt_resume();
  for (unsigned i = 0; i < iterations; i++) {
    result = mat_left * mat_right;
  }
  // __itt_pause();
  double end_time = test_get_time();

  return end_time - start_time;
}

static double bench_multiply_transpose(unsigned Nx, unsigned Ny,
                                       unsigned iterations) {
  srand(0);
  arma::fmat real_left = randn<arma::fmat>(Nx, Ny);
  arma::fmat imag_left = randn<arma::fmat>(Nx, Ny);
  arma::fmat real_right = randn<arma::fmat>(Nx, 1);
  arma::fmat imag_right = randn<arma::fmat>(Nx, 1);
  arma::cx_fmat mat_right(real_right, imag_right);
  arma::cx_fmat mat_left(real_left, imag_left);
  arma::cx_fmat result(Nx, 1);

  double start_time = test_get_time();
  for (unsigned i = 0; i < iterations; i++) {
    result = mat_left.st() * mat_right;
  }
  double end_time = test_get_time();

  return end_time - start_time;
}

static void run_benchmark_ZF(unsigned Nx, unsigned Ny, unsigned iterations) {
  double zf_time1 = bench_ZF_warmup(Nx, Ny, iterations);
  // hpctoolkit_sampling_start();
  // __itt_pause();
  double zf_time2 = bench_ZF(Nx, Ny, iterations);
  double zf_time3 = bench_ZF(Nx, Ny, iterations);
  double zf_time4 = bench_ZF(Nx, Ny, iterations);
  double zf_time5 = bench_ZF(Nx, Ny, iterations);
  double zf_time6 = bench_ZF(Nx, Ny, iterations);
  double zf_time7 = bench_ZF(Nx, Ny, iterations);
  // __itt_pause();
  // hpctoolkit_sampling_start();

  std::printf("ZF:              %04u x %04u  %12.3f us per iteration\n", Nx, Ny,
              1000000.0 * zf_time1 / iterations);
  std::printf("ZF:              %04u x %04u  %12.3f us per iteration\n", Nx, Ny,
              1000000.0 * zf_time2 / iterations);
  std::printf("ZF:              %04u x %04u  %12.3f us per iteration\n", Nx, Ny,
              1000000.0 * zf_time3 / iterations);
  std::printf("ZF:              %04u x %04u  %12.3f us per iteration\n", Nx, Ny,
              1000000.0 * zf_time4 / iterations);
  std::printf("ZF:              %04u x %04u  %12.3f us per iteration\n", Nx, Ny,
              1000000.0 * zf_time5 / iterations);
  std::printf("ZF:              %04u x %04u  %12.3f us per iteration\n", Nx, Ny,
              1000000.0 * zf_time6 / iterations);
  std::printf("ZF:              %04u x %04u  %12.3f us per iteration\n", Nx, Ny,
              1000000.0 * zf_time7 / iterations);
}

static void run_benchmark_multiply(int dim, unsigned Nx, unsigned Ny,
                                   unsigned iterations) {
  // __itt_pause();
  double mul_time1 = (dim == 1) ? bench_multiply_dim1(Nx, Ny, iterations)
                                : bench_multiply_dim2(Nx, Ny, iterations);
  double mul_time2 = (dim == 1) ? bench_multiply_dim1(Nx, Ny, iterations)
                                : bench_multiply_dim2(Nx, Ny, iterations);
  double mul_time3 = (dim == 1) ? bench_multiply_dim1(Nx, Ny, iterations)
                                : bench_multiply_dim2(Nx, Ny, iterations);
  double mul_time4 = (dim == 1) ? bench_multiply_dim1(Nx, Ny, iterations)
                                : bench_multiply_dim2(Nx, Ny, iterations);
  double mul_time5 = (dim == 1) ? bench_multiply_dim1(Nx, Ny, iterations)
                                : bench_multiply_dim2(Nx, Ny, iterations);
  // __itt_pause();

  if (dim == 1) std::printf("(1 x %04u) * (%04u x %04u)\n", Nx, Nx, Ny);
  if (dim == 2) std::printf("(%04u x %04u) * (%04u x 1)\n", Nx, Ny, Ny);
  std::printf("Multiply:              %04u x %04u  %12.3f us per iteration\n",
              Nx, Ny, 1000000.0 * mul_time1 / iterations);
  std::printf("Multiply:              %04u x %04u  %12.3f us per iteration\n",
              Nx, Ny, 1000000.0 * mul_time2 / iterations);
  std::printf("Multiply:              %04u x %04u  %12.3f us per iteration\n",
              Nx, Ny, 1000000.0 * mul_time3 / iterations);
  std::printf("Multiply:              %04u x %04u  %12.3f us per iteration\n",
              Nx, Ny, 1000000.0 * mul_time4 / iterations);
  std::printf("Multiply:              %04u x %04u  %12.3f us per iteration\n",
              Nx, Ny, 1000000.0 * mul_time5 / iterations);
}

static void run_benchmark_precode(unsigned Nx, unsigned Ny,
                                  unsigned iterations) {
  double mul_time1_equal = bench_multiply_dim2(Nx, Ny, iterations);
  double mul_time2_equal = bench_multiply_dim2(Nx, Ny, iterations);
  double mul_time3_equal = bench_multiply_dim2(Nx, Ny, iterations);
  double mul_time4_equal = bench_multiply_dim2(Nx, Ny, iterations);
  double mul_time5_equal = bench_multiply_dim2(Nx, Ny, iterations);

  double mul_time1_precode = bench_multiply_transpose(Nx, Ny, iterations);
  double mul_time2_precode = bench_multiply_transpose(Nx, Ny, iterations);
  double mul_time3_precode = bench_multiply_transpose(Nx, Ny, iterations);
  double mul_time4_precode = bench_multiply_transpose(Nx, Ny, iterations);
  double mul_time5_precode = bench_multiply_transpose(Nx, Ny, iterations);

  std::printf("Equalization: (%04u x %04u) * (%04u x 1)\n", Nx, Ny, Ny);
  std::printf("Multiply:              %04u x %04u  %12.3f us per iteration\n",
              Nx, Ny, 1000000.0 * mul_time1_equal / iterations);
  std::printf("Multiply:              %04u x %04u  %12.3f us per iteration\n",
              Nx, Ny, 1000000.0 * mul_time2_equal / iterations);
  std::printf("Multiply:              %04u x %04u  %12.3f us per iteration\n",
              Nx, Ny, 1000000.0 * mul_time3_equal / iterations);
  std::printf("Multiply:              %04u x %04u  %12.3f us per iteration\n",
              Nx, Ny, 1000000.0 * mul_time4_equal / iterations);
  std::printf("Multiply:              %04u x %04u  %12.3f us per iteration\n",
              Nx, Ny, 1000000.0 * mul_time5_equal / iterations);

  std::printf("Precoding: (%04u x %04u) * (%04u x 1)\n", Ny, Nx, Nx);
  std::printf("Multiply:              %04u x %04u  %12.3f us per iteration\n",
              Nx, Ny, 1000000.0 * mul_time1_precode / iterations);
  std::printf("Multiply:              %04u x %04u  %12.3f us per iteration\n",
              Nx, Ny, 1000000.0 * mul_time2_precode / iterations);
  std::printf("Multiply:              %04u x %04u  %12.3f us per iteration\n",
              Nx, Ny, 1000000.0 * mul_time3_precode / iterations);
  std::printf("Multiply:              %04u x %04u  %12.3f us per iteration\n",
              Nx, Ny, 1000000.0 * mul_time4_precode / iterations);
  std::printf("Multiply:              %04u x %04u  %12.3f us per iteration\n",
              Nx, Ny, 1000000.0 * mul_time5_precode / iterations);
}

int main(int argc, char* argv[]) {
  // __itt_pause();
  // setenv("MKL_THREADING_LAYER", "sequential", true /* overwrite */);
  // std::cout << "MKL_THREADING_LAYER =  " << getenv("MKL_THREADING_LAYER")
  //           << std::endl;
  // putenv("MKL_VERBOSE=1");
  if (argc != 5) {
    std::fprintf(stderr, "Usage: %s [iterations] [Nx] [Ny] mode\n", argv[0]);
    return 1;
  }

  int main_core_id = 2;
  if (stick_this_thread_to_core(main_core_id) != 0) {
    std::printf("Main thread: stitch main thread to core %d failed\n",
                main_core_id);
    throw std::runtime_error(
        "Test Matrix: Main thread: stitch main thread to core ");
  } else {
    std::printf("Main thread: stitch main thread to core %d succeeded\n",
                main_core_id);
  }

  setpriority(PRIO_PROCESS, 0, -20);

  if (argc == 5) {
    unsigned iterations = strtoul(argv[1], NULL, 0);
    unsigned Nx = strtoul(argv[2], NULL, 0);
    unsigned Ny = strtoul(argv[3], NULL, 0);
    unsigned mode = strtoul(argv[4], NULL, 0);
    if (mode == 0)
      run_benchmark_ZF(Nx, Ny, iterations);
    else if (mode == 1)
      run_benchmark_multiply(1, Nx, Ny, iterations);
    else if (mode == 2)
      run_benchmark_multiply(2, Nx, Ny, iterations);
    else if (mode == 3)
      run_benchmark_precode(Nx, Ny, iterations);
    else
      std::printf("Mode is not supported!\n");
  }
}