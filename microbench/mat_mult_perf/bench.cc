#include <gflags/gflags.h>
#include <mkl.h>

#include <armadillo>
#include <iostream>

#include "timer.h"

double freq_ghz = -1.0;  // RDTSC frequency

#define clflushopt(addr) \
  asm volatile(".byte 0x66; clflush %0" : "+m"(*(volatile char*)(addr)));

DEFINE_uint64(n_iters, 10000, "Number of iterations of inversion");
DEFINE_uint64(n_rows, 8, "Number of matrix rows");
DEFINE_uint64(n_cols, 8, "Number of matrix columns");

float mat_mult_perf(std::vector<arma::cx_fmat>& test_matrices,
                    std::vector<arma::cx_fmat>& test_col_vectors) {
  arma::cx_fmat out(FLAGS_n_rows, 1);
  float ret = 0.0;

  size_t start_tsc = rdtsc();
  for (size_t iter = 0; iter < FLAGS_n_iters; iter++) {
    out = test_matrices[iter] * test_col_vectors[iter];
    ret += out[0].real();
  }
  size_t end_tsc = rdtsc();
  std::printf(
      "[No JIT] Average time to multiple [%llu x %llu] matrix by [%llu x %llu] "
      "vector = %.0f ns\n",
      test_matrices[0].n_rows, test_matrices[0].n_cols,
      test_col_vectors[0].n_rows, test_col_vectors[0].n_cols,
      to_nsec(end_tsc - start_tsc, freq_ghz) / FLAGS_n_iters);

  return ret;
}

float mat_mult_perf_jit(std::vector<arma::cx_fmat>& test_matrices,
                        std::vector<arma::cx_fmat>& test_col_vectors) {
  void* mkl_jitter;
  cgemm_jit_kernel_t mkl_jit_cgemm;
  MKL_Complex8 alpha = {1, 0};
  MKL_Complex8 beta = {0, 0};

  mkl_jit_status_t status = mkl_jit_create_cgemm(
      &mkl_jitter, MKL_COL_MAJOR, MKL_NOTRANS, MKL_NOTRANS, FLAGS_n_rows, 1,
      FLAGS_n_cols, &alpha, FLAGS_n_rows, FLAGS_n_cols, &beta, FLAGS_n_rows);
  if (status == MKL_JIT_ERROR) {
    std::fprintf(stderr, "Error: Failed to init MKL JIT");
    std::exit(-1);
  }
  mkl_jit_cgemm = mkl_jit_get_cgemm_ptr(mkl_jitter);

  arma::cx_fmat out(FLAGS_n_rows, 1);
  float ret = 0.0;

  size_t start_tsc = rdtsc();
  for (size_t iter = 0; iter < FLAGS_n_iters; iter++) {
    mkl_jit_cgemm(
        mkl_jitter,
        reinterpret_cast<MKL_Complex8*>(test_matrices[iter].memptr()),
        reinterpret_cast<MKL_Complex8*>(test_col_vectors[iter].memptr()),
        reinterpret_cast<MKL_Complex8*>(out.memptr()));
    ret += out[0].real();
  }
  size_t end_tsc = rdtsc();
  std::printf(
      "[JIT]: Average time to multiple [%llu x %llu] matrix by [%llu x %llu] "
      "vector = %.0f ns\n",
      test_matrices[0].n_rows, test_matrices[0].n_cols,
      test_col_vectors[0].n_rows, test_col_vectors[0].n_cols,
      to_nsec(end_tsc - start_tsc, freq_ghz) / FLAGS_n_iters);

  return ret;
}

void flush_cache_lines(std::vector<arma::cx_fmat>& test_matrices,
                       std::vector<arma::cx_fmat>& test_col_vectors) {
  for (auto& m : test_matrices) {
    auto* base = reinterpret_cast<uint8_t*>(m.memptr());
    size_t n_bytes = m.n_rows * m.n_cols * sizeof(arma::cx_float);

    for (size_t i = 0; i < n_bytes; i += 64) {
      clflushopt(base + i);
    }
  }

  for (auto& v : test_col_vectors) {
    auto* base = reinterpret_cast<uint8_t*>(v.memptr());
    size_t n_bytes = v.n_rows * v.n_cols * sizeof(arma::cx_float);

    for (size_t i = 0; i < n_bytes; i += 64) {
      clflushopt(base + i);
    }
  }
}

int main(int argc, char** argv) {
  mkl_set_num_threads(1);
  arma::arma_rng::set_seed_random();
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  freq_ghz = measure_rdtsc_freq();
  nano_sleep(100 * 1000 * 1000, freq_ghz);  // Trigger turbo for 100 ms

  std::vector<arma::cx_fmat> test_matrices;
  for (size_t i = 0; i < FLAGS_n_iters; i++) {
    test_matrices.push_back(
        arma::randn<arma::cx_fmat>(FLAGS_n_rows, FLAGS_n_cols));
  }

  std::vector<arma::cx_fmat> test_col_vectors;
  for (size_t i = 0; i < FLAGS_n_iters; i++) {
    test_col_vectors.push_back(arma::randn<arma::cx_fmat>(FLAGS_n_cols, 1));
  }

  std::printf("No cache line flushing, JIT\n");
  for (size_t i = 0; i < 4; i++) {
    float ret = mat_mult_perf_jit(test_matrices, test_col_vectors);
    std::fprintf(stderr, "Computation proof = %.2f\n", ret);
  }

  std::printf("\nNo cache line flushing, no JIT\n");
  for (size_t i = 0; i < 4; i++) {
    float ret = mat_mult_perf(test_matrices, test_col_vectors);
    std::fprintf(stderr, "Computation proof = %.2f\n", ret);
  }

  std::printf("\nCache line flushing, JIT\n");
  for (size_t i = 0; i < 4; i++) {
    flush_cache_lines(test_matrices, test_col_vectors);
    float ret = mat_mult_perf_jit(test_matrices, test_col_vectors);
    std::fprintf(stderr, "Computation proof = %.2f\n", ret);
  }

  std::printf("\nCache line flushing, no JIT\n");
  for (size_t i = 0; i < 4; i++) {
    flush_cache_lines(test_matrices, test_col_vectors);
    float ret = mat_mult_perf(test_matrices, test_col_vectors);
    std::fprintf(stderr, "Computation proof = %.2f\n", ret);
  }
}
