#include <mkl.h>
#include <armadillo>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "timer.h"

static constexpr size_t kNumIters = 1000;
static constexpr size_t kSize = 64;
double freq_ghz;

enum class InverseMode { kInverse, kPseudoInverse };

float test_arma(const float *_in_mat_arr, InverseMode mode) {
  float in_mat_arr[kSize * kSize];
  float out_mat_arr[kSize * kSize];
  for (size_t i = 0; i < kSize * kSize; i++) in_mat_arr[i] = _in_mat_arr[i];

  TscTimer timer;
  timer.start();
  for (size_t iter = 0; iter < kNumIters; iter++) {
    arma::Mat<float> input(in_mat_arr, kSize, kSize, false);
    arma::Mat<float> output(out_mat_arr, kSize, kSize, false);
    if (mode == InverseMode::kPseudoInverse) {
      output = pinv(input);
    } else {
      output = input.i();
    }
    in_mat_arr[0] = out_mat_arr[0];
  }

  timer.stop();
  printf("Armadillo: Average time for %s of %zux%zu matrix = %.3f ms\n",
         mode == InverseMode::kInverse ? "inverse" : "pseudo-inverse", kSize,
         kSize, timer.avg_usec(freq_ghz) / (1000.0 * kNumIters));
  return out_mat_arr[0];
}

float test_eigen(const float *_in_mat_arr, InverseMode mode) {
  float in_mat_arr[kSize * kSize];
  float out_mat_arr[kSize * kSize];
  for (size_t i = 0; i < kSize * kSize; i++) in_mat_arr[i] = _in_mat_arr[i];

  TscTimer timer;
  timer.start();
  for (size_t iter = 0; iter < kNumIters; iter++) {
    Eigen::Map<Eigen::Matrix<float, kSize, kSize, Eigen::ColMajor>> input(
        in_mat_arr);
    Eigen::Map<Eigen::Matrix<float, kSize, kSize, Eigen::ColMajor>> output(
        out_mat_arr);
    if (mode == InverseMode::kPseudoInverse) {
      output = input.completeOrthogonalDecomposition().pseudoInverse();
    } else {
      output = input.inverse();
    }
    in_mat_arr[0] = out_mat_arr[0];
  }

  timer.stop();
  printf("Eigen: Average time for %s of %zux%zu matrix = %.3f ms\n",
         mode == InverseMode::kInverse ? "inverse" : "pseudo-inverse", kSize,
         kSize, timer.avg_usec(freq_ghz) / (1000.0 * kNumIters));
  return out_mat_arr[0];
}

int main() {
  freq_ghz = measure_rdtsc_freq();
  mkl_set_num_threads(1);

  float *in_base_mat_arr = new float[kSize * kSize];
  for (size_t i = 0; i < kSize * kSize; i++) {
    in_base_mat_arr[i] =
        static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
  }

  float ret_1 = test_arma(in_base_mat_arr, InverseMode::kInverse);
  float ret_2 = test_arma(in_base_mat_arr, InverseMode::kPseudoInverse);
  float ret_3 = test_eigen(in_base_mat_arr, InverseMode::kInverse);
  float ret_4 = test_eigen(in_base_mat_arr, InverseMode::kPseudoInverse);
  printf("%.5f, %.5f, %.5f, %.5f\n", ret_1, ret_2, ret_3, ret_4);
}
