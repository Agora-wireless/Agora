#include <gflags/gflags.h>
#include <mkl.h>
#define ARMA_DONT_PRINT_ERRORS
#include <armadillo>
#include <iostream>
#include "timer.h"

DEFINE_uint64(n_iters, 10000, "Number of iterations of inversion");

// Print the condition number distribution for random matrices
static void condition_number_distribution(size_t n_rows, size_t n_cols) {
  std::vector<double> cond_vec;
  for (size_t i = 0; i < FLAGS_n_iters; i++) {
    cond_vec.push_back(arma::cond(arma::randn<arma::cx_fmat>(n_rows, n_cols)));
  }
  std::sort(cond_vec.begin(), cond_vec.end());
  std::printf("%zux%zu %.1f %.1f %.1f\n", n_rows, n_cols, mean(cond_vec),
         cond_vec[cond_vec.size() * 0.50], cond_vec.back());
}

int main(int argc, char** argv) {
  mkl_set_num_threads(8);
  arma::arma_rng::set_seed_random();
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  std::printf("Size average median max\n");  // Print the header
  for (size_t n_rows = 8; n_rows <= 64; n_rows += 8) {
    for (size_t n_cols = 8; n_cols <= n_rows; n_cols += 8) {
      condition_number_distribution(n_rows, n_cols);
    }
  }
}
