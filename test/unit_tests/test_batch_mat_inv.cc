/**
 * @file test_batch_mat_inv.cc
 * @brief Test the batch matrix inversion implementations for time consumption.
 */

#include <gtest/gtest.h>
// For some reason, gtest include order matters

#include "config.h"
#include "gettime.h"

/*
 * Test 2x2xN cube slice-wise matrix inversion with a loop.
 */
void batch_mat_inv_sympd_arma_loop_slices(size_t vec_len, int dim,
                                          double freq_ghz, double& duration_ms,
                                          arma::cx_fcube& cub_a,
                                          arma::cx_fcube& cub_b) {
  size_t tsc_start, tsc_end;
  
  // arma::cx_fcube cub_a(dim, dim, vec_len, arma::fill::randu);
  // arma::cx_fcube cub_b(dim, dim, vec_len, arma::fill::zeros);

  tsc_start = GetTime::Rdtsc();
  for (size_t i = 0; i < vec_len; ++i) {
    arma::cx_fmat mat_csi = cub_a.slice(i);
    cub_b.slice(i) = arma::inv_sympd(mat_csi.t() * mat_csi) * mat_csi.t();
  }
  tsc_end = GetTime::Rdtsc();

  duration_ms = GetTime::CyclesToMs(tsc_end - tsc_start, freq_ghz);
  // printf("Time measured = %.2f ms\n", duration_ms);
}

/*
 * Test 2x2xN cube slice-wise matrix inversion with vector decomposition.
 */
void batch_mat_inv_sympd_arma_decomp_vec(size_t vec_len, int dim,
                                         double freq_ghz, double& duration_ms,
                                         arma::cx_fcube& cub_a,
                                         arma::cx_fcube& cub_b) {
  size_t tsc_start, tsc_end;

  RtAssert(dim == 2, "Only support 2x2 matrix inversion");
  
  // A = [ a b ], B = [a''' b''']
  //     [ c d ]      [c''' d''']
  // arma::cx_fcube cub_a(dim, dim, vec_len, arma::fill::randu);
  // arma::cx_fcube cub_b(dim, dim, vec_len, arma::fill::zeros);

  // temporary storages
  // Product of A^T and A = A', A' = [a^2 + c^2, a*b + c*d] = [a' b']
  //                                 [a*b + c*d, b^2 + d^2]   [c' d']
  // Inversion of A' = A'' = [a'' b''] = [d'  -b'] / (a'*d' - b'*c')
  //                         [c'' d'']   [-c'  a']
  // Multiplication of A'' and A = [a''' b'''] = [a''*a + b''*c  a''*b + b''*d]
  //                               [c''' d''']   [c''*a + d''*c  c''*b + d''*d]
  arma::cx_fcube cub_a_prod(dim, dim, vec_len, arma::fill::zeros);
  arma::cx_fcube cub_a_inv(dim, dim, vec_len, arma::fill::zeros);
  arma::cx_fcube cub_a_det(1, 1, vec_len, arma::fill::zeros);

  tsc_start = GetTime::Rdtsc();
  // a' = a^2 + c^2, b' = a*b + c*d, c' = a*b + c*d, d' = b^2 + d^2
  cub_a_prod.tube(0, 0) =
    arma::square(cub_a.tube(0, 0)) + arma::square(cub_a.tube(0, 0));
  cub_a_prod.tube(0, 1) =
    cub_a.tube(0, 0) % cub_a.tube(0, 1) + cub_a.tube(1, 0) % cub_a.tube(1, 1);
  cub_a_prod.tube(1, 0) = cub_a_prod.tube(0, 1);
  cub_a_prod.tube(1, 1) =
    arma::square(cub_a.tube(0, 1)) + arma::square(cub_a.tube(1, 1));

  // a_det = a'*d' - b'*c'
  cub_a_det.tube(0, 0) =
    cub_a_prod.tube(0, 0) % cub_a_prod.tube(1, 1) -
    cub_a_prod.tube(0, 1) % cub_a_prod.tube(1, 0);
  // a'' = d' / a_det, b'' = -b' / a_det
  // c'' = -c' / a_det, d'' = a' / a_det
  cub_a_inv.tube(0, 0) =
    cub_a_prod.tube(1, 1) / cub_a_det.tube(0, 0);
  cub_a_inv.tube(0, 1) =
    -cub_a_prod.tube(0, 1) / cub_a_det.tube(0, 0);
  cub_a_inv.tube(1, 0) =
    -cub_a_prod.tube(1, 0) / cub_a_det.tube(0, 0);
  cub_a_inv.tube(1, 1) =
    cub_a_prod.tube(0, 0) / cub_a_det.tube(0, 0);

  // a''' = a''*a + b''*c, b''' = a''*b + b''*d
  // c''' = c''*a + d''*c, d''' = c''*b + d''*d
  cub_b.tube(0, 0) =
    cub_a_inv.tube(0, 0) % cub_a.tube(0, 0) +
    cub_a_inv.tube(0, 1) % cub_a.tube(1, 0);
  cub_b.tube(0, 1) =
    cub_a_inv.tube(0, 0) % cub_a.tube(0, 1) +
    cub_a_inv.tube(0, 1) % cub_a.tube(1, 1);
  cub_b.tube(1, 0) =
    cub_a_inv.tube(1, 0) % cub_a.tube(0, 0) +
    cub_a_inv.tube(1, 1) % cub_a.tube(1, 0);
  cub_b.tube(1, 1) =
    cub_a_inv.tube(1, 0) % cub_a.tube(0, 1) +
    cub_a_inv.tube(1, 1) % cub_a.tube(1, 1);
  tsc_end = GetTime::Rdtsc();

  duration_ms = GetTime::CyclesToMs(tsc_end - tsc_start, freq_ghz);
  // printf("Time measured = %.2f ms\n", duration_ms);
}

/*
 * Test 2x2xN cube slice-wise matrix inversion with vector decomposition.
 * Simplified version of the above function (but not as straitforward).
 */
void batch_mat_inv_sympd_arma_decomp_vec_simp(size_t vec_len, int dim,
                                              double freq_ghz,
                                              double& duration_ms,
                                              arma::cx_fcube& cub_a,
                                              arma::cx_fcube& cub_b) {
  size_t tsc_start, tsc_end;
  
  // A = [ a b ], B = [a''' b''']
  //     [ c d ]      [c''' d''']
  // arma::cx_fcube cub_a(dim, dim, vec_len, arma::fill::randu);
  // arma::cx_fcube cub_b(dim, dim, vec_len, arma::fill::zeros);

  // temporary storages
  // Product of A^T and A = A', A' = [a^2 + c^2, a*b + c*d] = [a' b']
  //                                 [a*b + c*d, b^2 + d^2]   [c' d']
  // Inversion of A' = A'' = [a'' b''] = [d'  -b'] / (a'*d' - b'*c')
  //                         [c'' d'']   [-c'  a']
  // Multiplication of A'' and A = [a''' b'''] = [a''*a + b''*c  a''*b + b''*d]
  //                               [c''' d''']   [c''*a + d''*c  c''*b + d''*d]
  arma::cx_fcube cub_a_prod(dim, dim, vec_len, arma::fill::zeros);
  arma::cx_fcube cub_a_inv(dim, dim, vec_len, arma::fill::zeros);
  arma::cx_fcube cub_a_det(1, 1, vec_len, arma::fill::zeros);

  tsc_start = GetTime::Rdtsc();
  // a' = a^2 + c^2, b' = a*b + c*d, c' = a*b + c*d, d' = b^2 + d^2
  cub_a_prod.tube(0, 0) =
    arma::square(cub_a.tube(0, 0)) + arma::square(cub_a.tube(0, 0));
  cub_a_prod.tube(0, 1) =
    cub_a.tube(0, 0) % cub_a.tube(0, 1) + cub_a.tube(1, 0) % cub_a.tube(1, 1);
  cub_a_prod.tube(1, 0) = cub_a_prod.tube(0, 1);
  cub_a_prod.tube(1, 1) =
    arma::square(cub_a.tube(0, 1)) + arma::square(cub_a.tube(1, 1));

  // basically move the scalar det to the end, and substitute inversed matrix
  // to the operand of the second multiplication

  // a_det = a'*d' - b'*c'
  cub_a_det.tube(0, 0) =
    cub_a_prod.tube(0, 0) % cub_a_prod.tube(1, 1) -
    cub_a_prod.tube(0, 1) % cub_a_prod.tube(1, 0);

  // a''' = a''*a + b''*c, b''' = a''*b + b''*d
  // c''' = c''*a + d''*c, d''' = c''*b + d''*d
  cub_b.tube(0, 0) =
    ( cub_a_prod.tube(1, 1) % cub_a.tube(0, 0) +
     -cub_a_prod.tube(0, 1) % cub_a.tube(1, 0)) / cub_a_det.tube(0, 0);
  cub_b.tube(0, 1) =
    ( cub_a_prod.tube(1, 1) % cub_a.tube(0, 1) +
     -cub_a_prod.tube(0, 1) % cub_a.tube(1, 1)) / cub_a_det.tube(0, 0);
  cub_b.tube(1, 0) =
    (-cub_a_prod.tube(1, 0) % cub_a.tube(0, 0) +
      cub_a_prod.tube(0, 0) % cub_a.tube(1, 0)) / cub_a_det.tube(0, 0);
  cub_b.tube(1, 1) =
    (-cub_a_prod.tube(1, 0) % cub_a.tube(0, 1) +
      cub_a_prod.tube(0, 0) % cub_a.tube(1, 1)) / cub_a_det.tube(0, 0);
  tsc_end = GetTime::Rdtsc();

  duration_ms = GetTime::CyclesToMs(tsc_end - tsc_start, freq_ghz);
  // printf("Time measured = %.2f ms\n", duration_ms);
}

TEST(TestBatchMatInv, TimingAnalysis) {
  int iter = 1000;
  size_t vec_len = 768; // number of subcarriers in this case
  int dim = 2;
  double time_ms_loop = 0.0;
  double time_ms_vectors = 0.0;
  double time_ms_vectors_simp = 0.0;
  double freq_ghz = GetTime::MeasureRdtscFreq();

  double time_ms_tmp = 0.0;

  arma::cx_fcube cub_src(dim, dim, vec_len, arma::fill::randu);
  arma::cx_fcube cub_res(dim, dim, vec_len, arma::fill::zeros);

  printf("Running %d times of %ld (%dx%d) matrix inversion...\n",
         iter, vec_len, dim, dim);

  for (int i = 0; i < iter; ++i) {
    batch_mat_inv_sympd_arma_loop_slices(vec_len, dim, freq_ghz, time_ms_tmp,
                                         cub_src, cub_res);
    time_ms_loop += time_ms_tmp;
    
    batch_mat_inv_sympd_arma_decomp_vec(vec_len, dim, freq_ghz, time_ms_tmp,
                                        cub_src, cub_res);
    time_ms_vectors += time_ms_tmp;

    batch_mat_inv_sympd_arma_decomp_vec_simp(vec_len, dim, freq_ghz, 
                                             time_ms_tmp, cub_src ,cub_res);
    time_ms_vectors_simp += time_ms_tmp;
  }
  printf("[arma] (sympd) Time for %dx loops of slices = %.2f ms\n",
         iter, time_ms_loop);
  printf("[arma] (sympd) Time for %dx vector decomposition (vec) = %.2f ms\n",
         iter, time_ms_vectors);
  printf("[arma] (sympd) Time for %dx vector decomposition (vec_simp) = %.2f ms\n",
         iter, time_ms_vectors_simp);
}

TEST(TestBatchMatInv, Correctness) {
  // Prepare the operands
  size_t vec_len = 768; // number of subcarriers in this case
  int dim = 2;
  double time_ms_dummy = 0.0;
  double freq_ghz = GetTime::MeasureRdtscFreq();

  arma::cx_fcube cub_a(dim, dim, vec_len, arma::fill::randu);

  // Method 1: Armadillo's inv_sympd + loop (default)
  arma::cx_fcube cub_b_1(dim, dim, vec_len, arma::fill::zeros);

  batch_mat_inv_sympd_arma_loop_slices(vec_len, dim, freq_ghz, time_ms_dummy,
                                       cub_a, cub_b_1);

  // Method 2: vector decomposition
  arma::cx_fcube cub_b_2(dim, dim, vec_len, arma::fill::zeros);

  batch_mat_inv_sympd_arma_decomp_vec(vec_len, dim, freq_ghz, time_ms_dummy,
                                      cub_a, cub_b_2);
  
  // Method 3: vector decomposition (simplified)
  arma::cx_fcube cub_b_3(dim, dim, vec_len, arma::fill::zeros);

  batch_mat_inv_sympd_arma_decomp_vec_simp(vec_len, dim, freq_ghz,
                                           time_ms_dummy, cub_a, cub_b_3);
  
  // Check the results
  EXPECT_TRUE(arma::approx_equal(cub_a, cub_b_1, "absdiff", 1e-3));
  EXPECT_TRUE(arma::approx_equal(cub_a, cub_b_2, "absdiff", 1e-3));
}

int main(int argc, char** argv) {
  printf("Running main() from %s\n", __FILE__);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}