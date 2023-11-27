/**
 * @file test_batch_mm.cc
 * @brief Testing slice-wise multiplication of a 2x2xN cube with 2x1xN matrix
 *        used in equalization and phase tracking.
 */

#include <gtest/gtest.h>
// For some reason, gtest include order matters

#include "config.h"
#include "dodemul.h"
#include "gettime.h"
#include "mkl.h"

/*
 * Test 2x2xN cube slice-wise multiplication with 2x1xN matrix with a loop.
 * OR
 * Test 4x4xN cube slice-wise multiplication with 4x1xN matrix with a loop.
 */
double time_batch_mm_arma_loop_slices(size_t vec_len, int dim,
                                            double freq_ghz) {
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

  // // Broadcast operation with subcarrier grouping
  // arma::cx_mat temp_b_mat(dim, 1, arma::fill::randu);
  // tsc_start = GetTime::Rdtsc();
  // cub_c = cub_a.each_slice() * temp_b_mat;
  // tsc_end = GetTime::Rdtsc();

  duration_ms = GetTime::CyclesToMs(tsc_end - tsc_start, freq_ghz);
  // printf("Time measured = %.2f ms\n", duration_ms);
  return duration_ms;
}

/*
 * Transform 2x2xN cube to 4 1x1xN vectors, 2x1xN matrix to 2 1x1xN vectors, and
 * perform element-wise multiplication to simulate slice-wise multiplication.
 * 
 * OR
 * Transform 4x4xN cube to 16 1x1xN vectors, 4x1xN matrix to 4 1x1xN vectors,
 * and perform element-wise multiplication to simulate slice-wise
 * multiplication.
 */
double time_batch_mm_arma_decomp_vec(size_t vec_len,
                                     int dim, double freq_ghz) {
  size_t tsc_start, tsc_end;
  double duration_ms;

  if (dim == 2) {
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
  } else if (dim == 4) {
    arma::cx_vec vec_a_1_1(vec_len, arma::fill::randu);
    arma::cx_vec vec_a_1_2(vec_len, arma::fill::randu);
    arma::cx_vec vec_a_1_3(vec_len, arma::fill::randu);
    arma::cx_vec vec_a_1_4(vec_len, arma::fill::randu);
    arma::cx_vec vec_a_2_1(vec_len, arma::fill::randu);
    arma::cx_vec vec_a_2_2(vec_len, arma::fill::randu);
    arma::cx_vec vec_a_2_3(vec_len, arma::fill::randu);
    arma::cx_vec vec_a_2_4(vec_len, arma::fill::randu);
    arma::cx_vec vec_a_3_1(vec_len, arma::fill::randu);
    arma::cx_vec vec_a_3_2(vec_len, arma::fill::randu);
    arma::cx_vec vec_a_3_3(vec_len, arma::fill::randu);
    arma::cx_vec vec_a_3_4(vec_len, arma::fill::randu);
    arma::cx_vec vec_a_4_1(vec_len, arma::fill::randu);
    arma::cx_vec vec_a_4_2(vec_len, arma::fill::randu);
    arma::cx_vec vec_a_4_3(vec_len, arma::fill::randu);
    arma::cx_vec vec_a_4_4(vec_len, arma::fill::randu);
    arma::cx_vec vec_b_1(vec_len, arma::fill::randu);
    arma::cx_vec vec_b_2(vec_len, arma::fill::randu);
    arma::cx_vec vec_b_3(vec_len, arma::fill::randu);
    arma::cx_vec vec_b_4(vec_len, arma::fill::randu);
    arma::cx_vec vec_c_1(vec_len, arma::fill::zeros);
    arma::cx_vec vec_c_2(vec_len, arma::fill::zeros);
    arma::cx_vec vec_c_3(vec_len, arma::fill::zeros);
    arma::cx_vec vec_c_4(vec_len, arma::fill::zeros);

    tsc_start = GetTime::Rdtsc();
    vec_c_1 = vec_a_1_1 % vec_b_1 + vec_a_1_2 % vec_b_2 +
              vec_a_1_3 % vec_b_3 + vec_a_1_4 % vec_b_4;
    vec_c_2 = vec_a_2_1 % vec_b_1 + vec_a_2_2 % vec_b_2 +
              vec_a_2_3 % vec_b_3 + vec_a_2_4 % vec_b_4;
    vec_c_3 = vec_a_3_1 % vec_b_1 + vec_a_3_2 % vec_b_2 +
              vec_a_3_3 % vec_b_3 + vec_a_3_4 % vec_b_4;
    vec_c_4 = vec_a_4_1 % vec_b_1 + vec_a_4_2 % vec_b_2 +
              vec_a_4_3 % vec_b_3 + vec_a_4_4 % vec_b_4;
    tsc_end = GetTime::Rdtsc();
  } else {
    printf("Error: dim = %d not supported! Expected value: 2 or 4.\n", dim);
    return -1;
  }

  duration_ms = GetTime::CyclesToMs(tsc_end - tsc_start, freq_ghz);
  // printf("Time measured = %.2f ms\n", duration_ms);
  return duration_ms;
}

/*
 * Transform 2x2xN cube to 4 1x1xN vectors, 2x1xN matrix to 2 1x1xN vectors, and
 * perform element-wise multiplication to simulate slice-wise multiplication.
 * 
 * This version extracts the vectors from the cube.
 * 
 * OR
 * Transform 4x4xN cube to 16 1x1xN vectors, 4x1xN matrix to 4 1x1xN vectors,
 * and perform element-wise multiplication to simulate slice-wise
 * multiplication.
 */
double time_batch_mm_arma_decomp_vec_from_cube(size_t vec_len, int dim,
                                               double freq_ghz) {
  size_t tsc_start, tsc_end;
  double duration_ms;

  arma::cx_cube cub_a(dim, dim, vec_len, arma::fill::randu);
  arma::cx_cube cub_b(dim, 1, vec_len, arma::fill::randu);
  arma::cx_cube cub_c(dim, 1, vec_len, arma::fill::zeros);

  if (dim == 2) {
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
  } else if (dim == 4) {
    tsc_start = GetTime::Rdtsc();
    cub_c.tube(0, 0) =
      cub_a.tube(0, 0) % cub_b.tube(0, 0) +
      cub_a.tube(0, 1) % cub_b.tube(1, 0) +
      cub_a.tube(0, 2) % cub_b.tube(2, 0) +
      cub_a.tube(0, 3) % cub_b.tube(3, 0);
    cub_c.tube(1, 0) =
      cub_a.tube(1, 0) % cub_b.tube(0, 0) +
      cub_a.tube(1, 1) % cub_b.tube(1, 0) +
      cub_a.tube(1, 2) % cub_b.tube(2, 0) +
      cub_a.tube(1, 3) % cub_b.tube(3, 0);
    cub_c.tube(2, 0) =
      cub_a.tube(2, 0) % cub_b.tube(0, 0) +
      cub_a.tube(2, 1) % cub_b.tube(1, 0) +
      cub_a.tube(2, 2) % cub_b.tube(2, 0) +
      cub_a.tube(2, 3) % cub_b.tube(3, 0);
    cub_c.tube(3, 0) =
      cub_a.tube(3, 0) % cub_b.tube(0, 0) +
      cub_a.tube(3, 1) % cub_b.tube(1, 0) +
      cub_a.tube(3, 2) % cub_b.tube(2, 0) +
      cub_a.tube(3, 3) % cub_b.tube(3, 0);

    // TODO: idea is interesting, but the syntax is not correct
    // arma::cx_mat temp_c_mat(dim, vec_len, arma::fill::zeros);
  
    // temp_c_mat += cub_a.col_as_mat(0).each_row() % cub_b.row_as_mat(0);
    // temp_c_mat += cub_a.col_as_mat(1).each_row() % cub_b.row_as_mat(1);
    // temp_c_mat += cub_a.col_as_mat(2).each_row() % cub_b.row_as_mat(2);
    // temp_c_mat += cub_a.col_as_mat(3).each_row() % cub_b.row_as_mat(3);

    // for (int i = 0; i < dim; ++i) {
    //   cub_c.tube(i, 0) = temp_c_mat.row(i);
    // }

    tsc_end = GetTime::Rdtsc();
  } else {
    printf("Error: dim = %d not supported! Expected value: 2 or 4.\n", dim);
    return -1;
  }

  duration_ms = GetTime::CyclesToMs(tsc_end - tsc_start, freq_ghz);
  // printf("Time measured = %.2f ms\n", duration_ms);
  return duration_ms;
}

/*
 * Test loop-based 2x2xN slice-wise multiplication with 2x1xN matrix with MKL.
 * OR
 * Test loop-based 4x4xN slice-wise multiplication with 4x1xN matrix with MKL.
 */
double time_batch_mm_mkl_cblas_cgemm_loop(size_t vec_len, int dim,
                                          double freq_ghz) {
  size_t tsc_start, tsc_end;
  double duration_ms;

  // arma::cx_cube cub_a(dim, dim, vec_len, arma::fill::randu);
  // arma::cx_cube cub_b(dim, 1, vec_len, arma::fill::randu);
  // arma::cx_cube cub_c(dim, 1, vec_len, arma::fill::zeros);

  // construct 3-d arrays for MKL
  MKL_Complex8 A_Array[vec_len][dim][dim];
  MKL_Complex8 B_Array[vec_len][dim][1];
  MKL_Complex8 C_Array[vec_len][dim][1];

  MKL_INT alpha_Array[vec_len];
  MKL_INT beta_Array[vec_len];

  MKL_INT alpha = 1;
  MKL_INT beta = 0;

  MKL_INT* K_Array = new MKL_INT[vec_len];
  MKL_INT* M_Array = new MKL_INT[vec_len];
  MKL_INT* N_Array = new MKL_INT[vec_len];

  CBLAS_TRANSPOSE* Trans_Array = new CBLAS_TRANSPOSE[vec_len];

  std::fill(alpha_Array, alpha_Array + vec_len, 1);
  std::fill(beta_Array, beta_Array + vec_len, 0);
  std::fill(K_Array, K_Array + vec_len, dim);
  std::fill(M_Array, M_Array + vec_len, dim);
  std::fill(N_Array, N_Array + vec_len, 1);
  std::fill(Trans_Array, Trans_Array + vec_len, CblasNoTrans);

  tsc_start = GetTime::Rdtsc();
  for (size_t i = 0; i < vec_len; ++i) {
    cblas_cgemm(CblasColMajor, CblasNoTrans, CblasNoTrans,
                dim, 1, dim,
                &alpha,
                &A_Array[i][0][0], dim,
                &B_Array[i][0][0], dim,
                &beta,
                &C_Array[i][0][0], dim);
    // cblas_cgemm(CblasColMajor, CblasNoTrans, CblasNoTrans,
    //             dim, 1, dim,
    //             &alpha_Array[i],
    //             cub_a.memptr(), dim,
    //             cub_b.memptr(), dim,
    //             &beta_Array[i],
    //             cub_c.memptr, dim);
  }
  tsc_end = GetTime::Rdtsc();

  duration_ms = GetTime::CyclesToMs(tsc_end - tsc_start, freq_ghz);
  // printf("Time measured = %.2f ms\n", duration_ms);
  return duration_ms;
}

/*
 * Test 2x2xN cube slice-wise multiplication with 2x1xN matrix with MKL
 * (clbas interface).
 * 
 * OR
 * Test 4x4xN cube slice-wise multiplication with 4x1xN matrix with MKL
 * (clbas interface).
 */
double time_batch_mm_mkl_cblas_cgemm_batch(size_t vec_len, int dim,
                                           double freq_ghz) {
  size_t tsc_start, tsc_end;
  double duration_ms;

  // construct 3-d arrays for MKL
  MKL_Complex8** A_Array;
  MKL_Complex8** B_Array;
  MKL_Complex8** C_Array;

  A_Array = new MKL_Complex8*[vec_len];
  B_Array = new MKL_Complex8*[vec_len];
  C_Array = new MKL_Complex8*[vec_len];

  for (size_t i = 0; i < vec_len; ++i) {
    A_Array[i] = new MKL_Complex8[dim * dim];
    B_Array[i] = new MKL_Complex8[dim * 1];
    C_Array[i] = new MKL_Complex8[dim * 1];

    for (int j = 0; j < dim; ++j) {
      for (int k = 0; k < dim; ++k) {
        A_Array[i][j * dim + k].real = arma::randu();
        A_Array[i][j * dim + k].imag = arma::randu();
      }
      B_Array[i][j].real = arma::randu();
      B_Array[i][j].imag = arma::randu();
    }
  }

  MKL_INT* M_Array = new MKL_INT[vec_len];
  MKL_INT* N_Array = new MKL_INT[vec_len];
  MKL_INT* K_Array = new MKL_INT[vec_len];

  CBLAS_TRANSPOSE* Trans_Array = new CBLAS_TRANSPOSE[vec_len];

  std::fill(M_Array, M_Array + vec_len, dim);
  std::fill(N_Array, N_Array + vec_len, 1);
  std::fill(K_Array, K_Array + vec_len, dim);
  std::fill(Trans_Array, Trans_Array + vec_len, CblasNoTrans);
  
  MKL_INT alpha = 1;
  MKL_INT beta = 0;
  MKL_INT group_size = vec_len;

  tsc_start = GetTime::Rdtsc();
  // Using cblas_cgemm3m_batch() instead of cblas_cgemm_batch() gives around
  // 50% performance improvement (time measured = 333.73 ms vs. 682.66 ms)
  // cblas_cgemm3m_batch(CblasColMajor, Trans_Array, Trans_Array,
  cblas_cgemm_batch(CblasColMajor, Trans_Array, Trans_Array,
                    M_Array, N_Array, K_Array,
                    &alpha,
                    (const void **) A_Array, M_Array,
                    (const void **) B_Array, K_Array,
                    &beta,
                    (void **) C_Array, M_Array,
                    1, &group_size);
  tsc_end = GetTime::Rdtsc();

  delete M_Array;
  delete N_Array;
  delete K_Array;
  delete Trans_Array;

  for (size_t i = 0; i < vec_len; ++i) {
    delete A_Array[i];
    delete B_Array[i];
    delete C_Array[i];
  }

  delete A_Array;
  delete B_Array;
  delete C_Array;

  duration_ms = GetTime::CyclesToMs(tsc_end - tsc_start, freq_ghz);
  // printf("Time measured = %.2f ms\n", duration_ms);
  return duration_ms;
}

TEST(TestBatchMatMult, TimingAnalysis) {
  int iter = 1000;
  size_t vec_len = 768; // number of subcarriers in this case
  int dim = 4;
  double time_ms_loop = 0.0;
  double time_ms_vectors = 0.0;
  double time_ms_vectors_extract = 0.0;
  double time_ms_MKL = 0.0;
  double time_ms_MKL_batch = 0.0;
  double freq_ghz = GetTime::MeasureRdtscFreq();

  printf("Running %d times of %ld (%dx%d) x (%dx1) matrix mult...\n",
         iter, vec_len, dim, dim, dim);

  for (int i = 0; i < iter; ++i) {
    time_ms_loop +=
      time_batch_mm_arma_loop_slices(vec_len, dim, freq_ghz);
    time_ms_vectors +=
      time_batch_mm_arma_decomp_vec(vec_len, dim, freq_ghz);
    time_ms_vectors_extract +=
      time_batch_mm_arma_decomp_vec_from_cube(vec_len, dim, freq_ghz);
    time_ms_MKL +=
      time_batch_mm_mkl_cblas_cgemm_loop(vec_len, dim, freq_ghz);
    time_ms_MKL_batch +=
      time_batch_mm_mkl_cblas_cgemm_batch(vec_len, dim, freq_ghz);
  }
  printf("[arma] Time for %dx loops of slices = %.2f ms\n", iter, time_ms_loop);
  printf("[arma] Time for %dx vector decomposition (vec) = %.2f ms\n",
         iter, time_ms_vectors);
  printf("[arma] Time for %dx vector decompositoin (tube) = %.2f ms\n",
         iter, time_ms_vectors_extract);
  printf("[mkl] Time for %dx loops of cblas_cgemm = %.2f ms\n",
         iter, time_ms_MKL);
  printf("[mkl] Time for %dx cblas_cgemm_batch = %.2f ms\n",
         iter, time_ms_MKL_batch);
}

int main(int argc, char** argv) {
  printf("Running main() from %s\n", __FILE__);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}