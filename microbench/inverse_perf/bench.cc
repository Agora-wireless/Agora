#include <gflags/gflags.h>
#include <mkl.h>
#define ARMA_DONT_PRINT_ERRORS
#include <armadillo>
#include <iostream>
#include "timer.h"
//#include <eigen3/Eigen/Dense>

double freq_ghz = -1.0;

// First 20% iterations are for warmup and not accounted for in timing
static constexpr double warmup_fraction = .2;

DEFINE_uint64(n_iters, 10000, "Number of iterations of inversion");
DEFINE_uint64(n_rows, 64, "Number of matrix rows");
DEFINE_uint64(n_cols, 32, "Number of matrix columns");
DEFINE_uint64(check_condition, 0, "Check the square matrix's condition number");

enum class PinvMode { kFormula, kSVD };

std::pair<std::vector<arma::cx_fmat>, double> arma_inverses(
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
        printf("Failed to invert A. Condition number of input = %.2f\n",
               arma::cond(input.t() * input));
        output = arma::pinv(input);
      }
    } else {
      output = pinv(input);
    }

    /*
    if (FLAGS_check_condition == 1) {
      // Condition number(X) = norm(X) * norm(X's inverse)
      ret += arma::norm(input) * arma::norm(output);
    }
    */

    if (take_measurement) timer.stop();
    ret.push_back(output);
  }
  return std::pair<std::vector<arma::cx_fmat>, double>(ret, timer.avg_msec());
}

/*
std::complex<float> test_eigen(const std::complex<float>* _in_mat_arr,
                               PinvMode mode) {
  const size_t tot_size = FLAGS_n_rows * FLAGS_n_cols;
  auto* in_mat_arr = new std::complex<float>[tot_size];
  auto* out_mat_arr = new std::complex<float>[tot_size];
  for (size_t i = 0; i < tot_size; i++) in_mat_arr[i] = _in_mat_arr[i];

  Eigen::Map<Eigen::Matrix<std::complex<float>, Eigen::Dynamic,
Eigen::Dynamic, Eigen::ColMajor>> input(in_mat_arr, FLAGS_n_rows,
FLAGS_n_cols);

  Eigen::Map<Eigen::Matrix<std::complex<float>, Eigen::Dynamic,
Eigen::Dynamic, Eigen::ColMajor>> output(out_mat_arr, FLAGS_n_cols,
FLAGS_n_rows);

  std::complex<float> ret(0.0, 0.0);
  TscTimer timer(FLAGS_n_iters, freq_ghz);
  for (size_t iter = 0; iter < FLAGS_n_iters; iter++) {
    const bool take_measurement = (iter >= FLAGS_n_iters *
warmup_fraction); if (take_measurement) timer.start();

    if (mode == PinvMode::kFormula) {
      output = (input.adjoint() * input).inverse() * input.adjoint();
    } else {
      output = input.completeOrthogonalDecomposition().pseudoInverse();
    }
    if (take_measurement) timer.stop();

    ret += output.sum();
    in_mat_arr[0] = out_mat_arr[0];
  }

  printf(
      "Eigen: Average time for %s-based pseudo-inverse of "
      "%zux%zu matrix = {avg %.3f ms, stddev %.3f ms}\n",
      mode == PinvMode::kFormula ? "formula" : "SVD", FLAGS_n_rows,
      FLAGS_n_cols, timer.avg_msec(), timer.stddev_msec());
  return ret;
}
*/

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  freq_ghz = measure_rdtsc_freq();
  nano_sleep(100 * 1000 * 1000,
             freq_ghz);  // Spin 100 ms to trigger turbo
  mkl_set_num_threads(1);

  std::vector<arma::cx_fmat> test_matrices;
  for (size_t i = 0; i < FLAGS_n_iters; i++) {
    test_matrices.push_back(
        arma::randu<arma::cx_fmat>(FLAGS_n_rows, FLAGS_n_cols));
  }

  // Part 1: Test inverse speed
  std::pair<std::vector<arma::cx_fmat>, double> ret_1 =
      arma_inverses(test_matrices, PinvMode::kFormula);
  std::pair<std::vector<arma::cx_fmat>, double> ret_2 =
      arma_inverses(test_matrices, PinvMode::kSVD);

  // Header: "Matrix_size Formula SVD SVD/Formula"
  printf("%zux%zu %.3f %.3f %.2f\n", FLAGS_n_rows, FLAGS_n_cols, ret_1.second,
         ret_2.second, ret_2.second / ret_1.second);

  double norm_sum = 0.0;
  for (size_t i = 0; i < FLAGS_n_iters; i++) {
    norm_sum += arma::norm(ret_1.first[i] - ret_2.first[i]);
  }
  fprintf(stderr, "Computation proof = %.2f\n", norm_sum);
}
