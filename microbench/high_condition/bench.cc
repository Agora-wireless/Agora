#include <gflags/gflags.h>
#include <mkl.h>
#include <armadillo>
#include <iostream>
#include "timer.h"

DEFINE_uint64(n_ants, 64, "Number of matrix rows");
DEFINE_uint64(n_users, 32, "Number of matrix columns");
DEFINE_double(condition, 10, "Condition number of input matrix");  // 20 dB

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
  rt_assert(std::min(FLAGS_n_ants, FLAGS_n_users) > 1,
            "> 1 singular values needed for gen_matrix_with_condition()");

  auto mat = arma::randn<arma::cx_fmat>(FLAGS_n_ants, FLAGS_n_users);
  arma::cx_fmat U;  // n_ants x n_ants
  arma::cx_fmat V;  // n_users x n_users
  arma::fvec s;     // min(n_ants, n_users)
  arma::svd(U, s, V, mat);

  // Set the singular values to evenly spaced between {cond_num, ..., 1.0}
  for (size_t i = 0; i < s.size(); i++) {
    s[i] = cond_num - i * ((cond_num - 1) / (s.size() - 1));
  }

  // Turn the singular values into a n_ants x n_ants diagonal matrix
  auto s_mat = arma::cx_fmat(FLAGS_n_ants, FLAGS_n_users, arma::fill::zeros);
  for (size_t i = 0; i < std::min(FLAGS_n_ants, FLAGS_n_users); i++) {
    s_mat(i, i) = s[i];
  }

  // Without normalization, the return's condition number is very close to
  // cond_num
  return arma::normalise(U * s_mat * V.t());
}

int main(int argc, char** argv) {
  mkl_set_num_threads(1);
  arma::arma_rng::set_seed_random();

  gflags::ParseCommandLineFlags(&argc, &argv, true);
  rt_assert(FLAGS_n_ants >= FLAGS_n_users, "");
  const arma::cx_fmat A = gen_matrix_with_condition(FLAGS_condition);
  arma::cx_fmat A_lpinv;  // Left pseudo-inverse of A

  std::printf("Input matrix {%lldx%lld}: cond number = %.3f\n", A.n_rows, A.n_cols,
         arma::cond(A));

  auto id_mat = arma::cx_fmat(FLAGS_n_users, FLAGS_n_users, arma::fill::zeros);
  for (size_t i = 0; i < FLAGS_n_users; i++)
    id_mat(i, i) = arma::cx_float(1, 0);

  A_lpinv = arma::inv(A.t() * A) * A.t();
  std::printf(
      "Formula (without inv_sympd): Sum of absolute differences of "
      "(A_lpinv * A - I): %.6f\n",
      arma::accu(arma::abs(A_lpinv * A - id_mat)));

  A_lpinv = arma::inv_sympd(A.t() * A) * A.t();
  std::printf(
      "Formula (with inv_sympd): Sum of absolute differences of "
      "(A_lpinv * A - I): %.6f\n",
      arma::accu(arma::abs(A_lpinv * A - id_mat)));

  std::printf(
      "Formula (without inv_sympd): Sum of absolute differences of "
      "(A_pinv * A - I): %.6f\n",
      arma::accu(arma::abs(A_lpinv * A - id_mat)));

  A_lpinv = pinv(A);
  std::printf("SVD: Sum of absolute differences of (A_lpinv * A - I): %.6f\n",
         arma::accu(arma::abs(A_lpinv * A - id_mat)));
}
