#include <gflags/gflags.h>
#include <mkl.h>
#include <armadillo>
#include <iostream>
#include "timer.h"

DEFINE_uint64(n_rows, 64, "Number of matrix rows");
DEFINE_uint64(n_cols, 32, "Number of matrix columns");
DEFINE_double(condition, 1000, "Condition number of input matrix");

enum class PinvMode { kFormula, kSVD };

/// Check a condition at runtime. If the condition is false, throw exception.
/// This is faster than rt_assert(cond, std::string) as it avoids string
/// construction.
static inline void rt_assert(bool condition, const char* throw_str) {
  if (!condition) throw std::runtime_error(throw_str);
}

// Generate a matrix with condition number approxately equal to cond_num
arma::cx_fmat gen_matrix_with_condition(double cond_num) {
  rt_assert(cond_num >= 1.0,
            "Condition number too small for gen_matrix_with_condition()");
  rt_assert(std::min(FLAGS_n_rows, FLAGS_n_cols) > 1,
            "> 1 singular values needed for gen_matrix_with_condition()");

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

  // Without normalization, the return's condition number is very close to
  // cond_num
  return arma::normalise(U * s_mat * V.t());
}

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  mkl_set_num_threads(1);
  arma::cx_fmat input = gen_matrix_with_condition(FLAGS_condition);

  printf("Result matrix {%lldx%lld}: cond number = %.3f\n", input.n_rows,
         input.n_cols, arma::cond(input));
  input.raw_print();

  arma::cx_fmat output;

  // Formula mode
  arma::cx_fmat A = input.t() * input;
  output = A.i() * input.t();
  printf("\nResults from pseudoinverse from formula:\n");
  (input * output).raw_print();

  // SVD mode;
  printf("\nResults from pseudoinverse from formula:\n");
  output = pinv(input);
  (input * output).raw_print();
}
