#include <gflags/gflags.h>
#include <mkl.h>
#include <armadillo>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "timer.h"

double freq_ghz = -1.0;

// First 20% iterations are for warmup and not accounted for in timing
static constexpr double warmup_fraction = .2;

DEFINE_uint64(n_iters, 10000, "Number of iterations of inversion");
DEFINE_uint64(n_rows, 64, "Number of matrix rows");
DEFINE_uint64(n_cols, 32, "Number of matrix columns");
DEFINE_uint64(check_condition, 0, "Check the square matrix's condition number");

enum class PinvMode { kFormula, kSVD };

std::pair<arma::cx_float, double> test_arma(const arma::cx_float* _in_mat_arr,
                                            PinvMode mode) {
  const size_t tot_size = FLAGS_n_rows * FLAGS_n_cols;
  auto* in_mat_arr = new arma::cx_float[tot_size];
  auto* out_mat_arr = new arma::cx_float[tot_size];
  for (size_t i = 0; i < tot_size; i++) in_mat_arr[i] = _in_mat_arr[i];

  arma::cx_fmat input(in_mat_arr, FLAGS_n_rows, FLAGS_n_cols, false);
  arma::cx_fmat output(out_mat_arr, FLAGS_n_cols, FLAGS_n_rows, false);

  TscTimer timer(FLAGS_n_iters, freq_ghz);
  arma::cx_float ret(0.0, 0.0);
  for (size_t iter = 0; iter < FLAGS_n_iters; iter++) {
    const bool take_measurement = (iter >= FLAGS_n_iters * warmup_fraction);
    if (take_measurement) timer.start();

    if (mode == PinvMode::kFormula) {
      arma::cx_fmat A = input.t() * input;
      if (FLAGS_check_condition == 1) ret += rcond(A);
      output = A.i() * input.t();
    } else {
      output = pinv(input);
    }

    if (take_measurement) timer.stop();

    ret += arma::accu(output);
    in_mat_arr[0] = out_mat_arr[0];
  }
  return std::pair<arma::cx_float, double>(ret, timer.avg_msec());
}

std::complex<float> test_eigen(const std::complex<float>* _in_mat_arr,
                               PinvMode mode) {
  const size_t tot_size = FLAGS_n_rows * FLAGS_n_cols;
  auto* in_mat_arr = new std::complex<float>[tot_size];
  auto* out_mat_arr = new std::complex<float>[tot_size];
  for (size_t i = 0; i < tot_size; i++) in_mat_arr[i] = _in_mat_arr[i];

  Eigen::Map<Eigen::Matrix<std::complex<float>, Eigen::Dynamic, Eigen::Dynamic,
                           Eigen::ColMajor>>
      input(in_mat_arr, FLAGS_n_rows, FLAGS_n_cols);

  Eigen::Map<Eigen::Matrix<std::complex<float>, Eigen::Dynamic, Eigen::Dynamic,
                           Eigen::ColMajor>>
      output(out_mat_arr, FLAGS_n_cols, FLAGS_n_rows);

  std::complex<float> ret(0.0, 0.0);
  TscTimer timer(FLAGS_n_iters, freq_ghz);
  for (size_t iter = 0; iter < FLAGS_n_iters; iter++) {
    const bool take_measurement = (iter >= FLAGS_n_iters * warmup_fraction);
    if (take_measurement) timer.start();

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

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  freq_ghz = measure_rdtsc_freq();
  nano_sleep(100 * 1000 * 1000, freq_ghz);  // Spin 100 ms to trigger turbo
  mkl_set_num_threads(1);

  auto* in_base_mat_arr = new arma::cx_float[FLAGS_n_rows * FLAGS_n_cols];
  for (size_t i = 0; i < FLAGS_n_rows * FLAGS_n_cols; i++) {
    auto re = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    auto im = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    in_base_mat_arr[i] = arma::cx_float(re, im);
  }

  std::pair<arma::cx_float, double> ret_1 =
      test_arma(in_base_mat_arr, PinvMode::kFormula);
  std::pair<arma::cx_float, double> ret_2 =
      test_arma(in_base_mat_arr, PinvMode::kSVD);

  // Header: "Matrix_size Formula SVD SVD/Formula"
  printf("%zux%zu %.3f %.3f ms %.2f\n", FLAGS_n_rows, FLAGS_n_cols,
         ret_1.second, ret_2.second, ret_2.second / ret_1.second);
  fprintf(stderr, "Arma: Results = {%.5f, %.5f}, {%.5f, %.5f}\n",
          ret_1.first.real(), ret_1.first.imag(), ret_2.first.real(),
          ret_2.first.imag());
}
