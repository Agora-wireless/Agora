#include <gflags/gflags.h>
#include <mkl.h>
#define ARMA_DONT_PRINT_ERRORS
#include <armadillo>
#include <iostream>
#include "timer.h"

double freq_ghz = -1.0;  // RDTSC frequency

// First 20% iterations are for warmup and not accounted for in timing
static constexpr double warmup_fraction = .2;

DEFINE_uint64(n_iters, 10000, "Number of iterations of inversion");
DEFINE_uint64(n_rows, 64, "Number of matrix rows");
DEFINE_uint64(n_cols, 32, "Number of matrix columns");

enum class PinvMode { kFormula, kSVD };

std::pair<std::vector<arma::cx_fmat>, double> arma_pseudo_inverses(
    const std::vector<arma::cx_fmat>& test_matrices, PinvMode mode) {
  TscTimer timer(FLAGS_n_iters, freq_ghz);
  std::vector<arma::cx_fmat> ret;

  for (size_t iter = 0; iter < FLAGS_n_iters; iter++) {
    const arma::cx_fmat& input = test_matrices[iter];
    arma::cx_fmat output;

    const bool take_measurement = (iter >= FLAGS_n_iters * warmup_fraction);
    if (take_measurement) timer.start();

    if (mode == PinvMode::kFormula) {
      try {
        output = arma::inv_sympd(input.t() * input) * input.t();
      } catch (std::runtime_error) {
        std::printf("Failed to invert A. Condition number of input = %.2f\n",
               arma::cond(input.t() * input));
        output = arma::pinv(input);
      }
    } else {
      output = pinv(input);
    }

    if (take_measurement) timer.stop();
    ret.push_back(output);
  }
  return std::pair<std::vector<arma::cx_fmat>, double>(ret, timer.avg_usec());
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

  std::pair<std::vector<arma::cx_fmat>, double> ret_formula =
      arma_pseudo_inverses(test_matrices, PinvMode::kFormula);
  std::pair<std::vector<arma::cx_fmat>, double> ret_svd =
      arma_pseudo_inverses(test_matrices, PinvMode::kSVD);

  // Header: "<matrix size> <Microseconds with formula> <Microseconds with SVD>
  // <Speedup with formula>"
  std::printf("%zux%zu %.1f %.1f %.1f\n", FLAGS_n_rows, FLAGS_n_cols,
         ret_formula.second, ret_svd.second,
         ret_svd.second / ret_formula.second);

  double norm_sum = 0.0;
  for (size_t i = 0; i < FLAGS_n_iters; i++) {
    norm_sum += arma::norm(ret_formula.first[i] - ret_svd.first[i]);
  }
  std::fprintf(stderr, "Computation proof = %.2f\n", norm_sum);
}
