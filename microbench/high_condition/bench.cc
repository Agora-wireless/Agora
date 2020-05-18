#include <gflags/gflags.h>
#include <mkl.h>
#include <armadillo>
#include <iostream>
#include "timer.h"

DEFINE_uint64(n_rows, 64, "Number of matrix rows");
DEFINE_uint64(n_cols, 32, "Number of matrix columns");

enum class PinvMode { kFormula, kSVD };

/// Check a condition at runtime. If the condition is false, throw exception.
/// This is faster than rt_assert(cond, std::string) as it avoids string
/// construction.
static inline void rt_assert(bool condition, const char* throw_str) {
  if (!condition) throw std::runtime_error(throw_str);
}

arma::cx_float test_arma(const arma::cx_float* _in_mat_arr, PinvMode mode) {
  const size_t tot_size = FLAGS_n_rows * FLAGS_n_cols;
  auto* in_mat_arr = new arma::cx_float[tot_size];
  auto* out_mat_arr = new arma::cx_float[tot_size];
  for (size_t i = 0; i < tot_size; i++) in_mat_arr[i] = _in_mat_arr[i];

  arma::cx_fmat input(in_mat_arr, FLAGS_n_rows, FLAGS_n_cols, false);
  arma::cx_fmat output(out_mat_arr, FLAGS_n_cols, FLAGS_n_rows, false);

  arma::cx_float ret(0.0, 0.0);

  if (mode == PinvMode::kFormula) {
    arma::cx_fmat A = input.t() * input;
    output = A.i() * input.t();
  } else {
    output = pinv(input);
  }

  ret += arma::accu(output);
  in_mat_arr[0] = out_mat_arr[0];

  return ret;
}

arma::cx_fmat gen_matrix_with_condition(double cond_num) {
  rt_assert(cond_num >= 1.0,
            "Condition number too small for gen_matrix_with_condition()");
  rt_assert(std::min(FLAGS_n_rows, FLAGS_n_cols) > 1,
            "> 1 singular values needed for ");

  auto mat = arma::randn<arma::cx_fmat>(FLAGS_n_rows, FLAGS_n_cols);
  arma::cx_fmat U;  // n_rows x n_rows
  arma::cx_fmat V;  // n_cols x n_cols
  arma::fvec s;     // min(n_rows, n_cols)
  arma::svd(U, s, V, mat);

  // Set the singular values to evenly spaced between {cond_num, ..., 1.0}
  for (size_t i = 0; i < s.size(); i++) {
    s[i] = cond_num - i * ((cond_num - 1) / (s.size() - 1));
  }

  // Turn the singular values into a n_rows x n_rows diagonal matrix
  auto s_mat = arma::cx_fmat(FLAGS_n_rows, FLAGS_n_cols, arma::fill::zeros);
  for (size_t i = 0; i < std::min(FLAGS_n_rows, FLAGS_n_cols); i++) {
    s_mat(i, i) = s[i];
  }

  return U * s_mat * V.t();
}

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  mkl_set_num_threads(1);
  arma::cx_fmat ret = gen_matrix_with_condition(10000000);

  printf("Result matrix {%lldx%lld}: cond number = %.3f\n", ret.n_rows,
         ret.n_cols, arma::cond(ret));
  ret.raw_print();

  /*

  for (size_t i = 0; i < FLAGS_n_rows * FLAGS_n_cols; i++) {
    auto re = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    auto im = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    in_base_mat_arr[i] = arma::cx_float(re, im);
  }

  arma::cx_float ret_1 = test_arma(in_base_mat_arr, PinvMode::kFormula);
  arma::cx_float ret_2 = test_arma(in_base_mat_arr, PinvMode::kSVD);
  printf("Arma: Results = {%.5f, %.5f}, {%.5f, %.5f}\n", ret_1.real(),
         ret_1.imag(), ret_2.real(), ret_2.imag());
         */
}
