/**
 * @file test_arma_cube_slice_2x2.cc
 * @brief Testing slice-wise multiplication of a 2x2xN cube with 2x1xN matrix
 *        used in equalization and phase tracking.
 */

#include <gtest/gtest.h>
// For some reason, gtest include order matters

#include "config.h"
#include "dodemul.h"
#include "gettime.h"

/*
 * Test 2x2xN cube slice-wise multiplication with 2x1xN matrix with a loop.
 */
double measure_slicewise_mm_2x2xN_loop(size_t vec_len,
                                       int dim, double freq_ghz) {
  size_t tsc_start, tsc_end;
  double duration_ms;
  
  arma::cx_cube cub_a(dim, dim, vec_len, arma::fill::randu);
  arma::cx_cube cub_b(dim, 1, vec_len, arma::fill::randu);
  arma::cx_cube cub_c(dim, 1, vec_len, arma::fill::zeros);

  tsc_start = GetTime::Rdtsc();
  for (size_t i = 0; i < vec_len; ++i) {
    cub_c.slice(i) = cub_a.slice(i) * cub_b.slice(i);
  }
  tsc_end = GetTime::Rdtsc();

  duration_ms = GetTime::CyclesToMs(tsc_end - tsc_start, freq_ghz);
  // printf("Time measured = %.2f ms\n", duration_ms);
  return duration_ms;
}

/*
 * Transform 2x2xN cube to 4 1x1xN vectors, 2x1xN matrix to 2 1x1xN vectors, and
 * perform element-wise multiplication to simulate slice-wise multiplication.
 */
double measure_slicewise_mm_2x2xN_vectors(size_t vec_len,
                                          int dim, double freq_ghz) {
  size_t tsc_start, tsc_end;
  double duration_ms;

  arma::cx_vec vec_a_1_1(vec_len, arma::fill::randu);
  arma::cx_vec vec_a_1_2(vec_len, arma::fill::randu);
  arma::cx_vec vec_a_2_1(vec_len, arma::fill::randu);
  arma::cx_vec vec_a_2_2(vec_len, arma::fill::randu);
  arma::cx_vec vec_b_1(vec_len, arma::fill::randu);
  arma::cx_vec vec_b_2(vec_len, arma::fill::randu);
  arma::cx_vec vec_c_1(vec_len, arma::fill::zeros);
  arma::cx_vec vec_c_2(vec_len, arma::fill::zeros);

  tsc_start = GetTime::Rdtsc();
  vec_c_1 = vec_a_1_1 % vec_b_1 + vec_a_1_2 % vec_b_2;
  vec_c_2 = vec_a_2_1 % vec_b_1 + vec_a_2_2 % vec_b_2;
  tsc_end = GetTime::Rdtsc();

  duration_ms = GetTime::CyclesToMs(tsc_end - tsc_start, freq_ghz);
  // printf("Time measured = %.2f ms\n", duration_ms);
  return duration_ms;
}

/*
 * Transform 2x2xN cube to 4 1x1xN vectors, 2x1xN matrix to 2 1x1xN vectors, and
 * perform element-wise multiplication to simulate slice-wise multiplication.
 * 
 * This version extracts the vectors from the cube.
 */
double measure_slicewise_mm_2x2xN_vectors_extract(size_t vec_len,
                                          int dim, double freq_ghz) {
  size_t tsc_start, tsc_end;
  double duration_ms;

  arma::cx_cube cub_a(dim, dim, vec_len, arma::fill::randu);
  arma::cx_cube cub_b(dim, 1, vec_len, arma::fill::randu);
  arma::cx_cube cub_c(dim, 1, vec_len, arma::fill::zeros);

  tsc_start = GetTime::Rdtsc();
  cub_c.tube(0, 0) =
    cub_a.tube(0, 0) % cub_b.tube(0, 0) + cub_a.tube(0, 1) % cub_b.tube(1, 0);
  cub_c.tube(1, 0) =
    cub_a.tube(1, 0) % cub_b.tube(0, 0) + cub_a.tube(1, 1) % cub_b.tube(1, 0);

  // arma::cx_vec vec_a_1_1 = cub_a.tube(0, 0);
  // arma::cx_vec vec_a_1_2 = cub_a.tube(0, 1);
  // arma::cx_vec vec_a_2_1 = cub_a.tube(1, 0);
  // arma::cx_vec vec_a_2_2 = cub_a.tube(1, 1);
  // arma::cx_vec vec_b_1 = cub_b.tube(0, 0);
  // arma::cx_vec vec_b_2 = cub_b.tube(1, 0);
  // arma::cx_vec vec_c_1 = vec_a_1_1 % vec_b_1 + vec_a_1_2 % vec_b_2;
  // arma::cx_vec vec_c_2 = vec_a_2_1 % vec_b_1 + vec_a_2_2 % vec_b_2;
  // cub_c.tube(0, 0) = vec_c_1;
  // cub_c.tube(1, 0) = vec_c_2;
  tsc_end = GetTime::Rdtsc();

  duration_ms = GetTime::CyclesToMs(tsc_end - tsc_start, freq_ghz);
  // printf("Time measured = %.2f ms\n", duration_ms);
  return duration_ms;
}

TEST(TestArma, CubeMMSlicewise) {
  int iter = 1000;
  size_t vec_len = 768; // number of subcarriers in this case
  int dim = 2;
  double time_ms_loop = 0.0;
  double time_ms_vectors = 0.0;
  double time_ms_vectors_extract = 0.0;
  double freq_ghz = GetTime::MeasureRdtscFreq();

  for (int i = 0; i < iter; ++i) {
    time_ms_loop += measure_slicewise_mm_2x2xN_loop(vec_len, dim, freq_ghz);
    time_ms_vectors +=
      measure_slicewise_mm_2x2xN_vectors(vec_len, dim, freq_ghz);
    time_ms_vectors_extract +=
      measure_slicewise_mm_2x2xN_vectors_extract(vec_len, dim, freq_ghz);
  }
  printf("Time for %d loop = %.2f ms\n", iter, time_ms_loop);
  printf("Time for %d vectors = %.2f ms\n", iter, time_ms_vectors);
  printf("Time for %d vectors extract = %.2f ms\n", iter,
         time_ms_vectors_extract);
}

int main(int argc, char** argv) {
  printf("Running main() from %s\n", __FILE__);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}