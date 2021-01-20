#include <gtest/gtest.h>
// For some reason, gtest include order matters
#include <armadillo>

#include "config.hpp"
#include "gettime.h"
#include "utils.h"

static constexpr size_t kMaxFrameNum = 10;

/// Test correctness of two-step recriprocal calibration
TEST(TestRecip, Correctness) {
  std::unique_ptr<Config> cfg(new Config("data/tddconfig-sim-ul.json"));
  cfg->GenData();

  double freq_ghz = measure_rdtsc_freq();

  Table<complex_float> calib_buffer, recip_buffer_0, recip_buffer_1;
  recip_buffer_0.calloc(kFrameWnd, cfg->ofdm_data_num() * cfg->bs_ant_num(),
                        Agora_memory::Alignment_t::k64Align);
  recip_buffer_1.calloc(kFrameWnd, cfg->ofdm_data_num() * cfg->bs_ant_num(),
                        Agora_memory::Alignment_t::k64Align);
  calib_buffer.rand_alloc_cx_float(kFrameWnd,
                                   cfg->ofdm_data_num() * cfg->bs_ant_num(),
                                   Agora_memory::Alignment_t::k64Align);

  std::printf("Reference antenna: %zu\n", cfg->ref_ant());

  size_t start_tsc = rdtsc();

  // Algorithm in reciprocity.cpp (use as ground truth)
  for (size_t i = 0; i < kMaxFrameNum; i++) {
    arma::cx_float* ptr_in =
        reinterpret_cast<arma::cx_float*>(calib_buffer[i % kFrameWnd]);
    arma::cx_fmat mat_input(ptr_in, cfg->ofdm_data_num(), cfg->bs_ant_num(),
                            false);
    arma::cx_fvec vec_calib_ref = mat_input.col(cfg->ref_ant());
    arma::cx_float* recip_buff =
        reinterpret_cast<arma::cx_float*>(recip_buffer_0[i % kFrameWnd]);
    arma::cx_fmat calib_mat = mat_input.each_col() / vec_calib_ref;

    arma::cx_fmat recip_mat(recip_buff, cfg->bs_ant_num(), cfg->ofdm_data_num(),
                            false);
    recip_mat = calib_mat.st();

    for (size_t sc_id = 0; sc_id < cfg->ofdm_data_num();
         sc_id += cfg->bs_ant_num()) {
      // TODO: interpolate instead of steps
      recip_mat
          .cols(sc_id, std::min(sc_id + cfg->bs_ant_num() - 1,
                                cfg->ofdm_data_num() - 1))
          .each_col() = recip_mat.col(sc_id);
    }
  }

  double ms0 = cycles_to_ms(rdtsc() - start_tsc, freq_ghz);

  start_tsc = rdtsc();

  // 2-step algorithm used in dofft and dozf
  // Partially tranpose is not tested here
  for (size_t i = 0; i < kMaxFrameNum; i++) {
    // In dofft
    for (size_t ant_id = 0; ant_id < cfg->bs_ant_num(); ant_id++) {
      auto* ptr_in =
          calib_buffer[i % kFrameWnd] + ant_id * cfg->ofdm_data_num();
      for (size_t sc_id = 0; sc_id < cfg->ofdm_data_num();
           sc_id += cfg->bs_ant_num()) {
        for (size_t j = 0; j < cfg->bs_ant_num(); j++)
          ptr_in[std::min(sc_id + j, cfg->ofdm_data_num() - 1)] = ptr_in[sc_id];
      }
    }
    // Transpose
    arma::cx_float* ptr_calib =
        reinterpret_cast<arma::cx_float*>(calib_buffer[i % kFrameWnd]);
    arma::cx_fmat calib_mat(ptr_calib, cfg->ofdm_data_num(), cfg->bs_ant_num(),
                            false);
    arma::cx_float* recip_buff =
        reinterpret_cast<arma::cx_float*>(recip_buffer_1[i % kFrameWnd]);
    arma::cx_fmat recip_mat(recip_buff, cfg->bs_ant_num(), cfg->ofdm_data_num(),
                            false);
    recip_mat = calib_mat.st();

    // In dozf
    for (size_t sc_id = 0; sc_id < cfg->ofdm_data_num(); sc_id++) {
      arma::cx_float* ptr_in = reinterpret_cast<arma::cx_float*>(
          recip_buffer_1[i % kFrameWnd] + sc_id * cfg->bs_ant_num());
      arma::cx_fvec recip_vec(ptr_in, cfg->bs_ant_num(), false);
      recip_vec = recip_vec / recip_vec(cfg->ref_ant());
    }
  }

  double ms1 = cycles_to_ms(rdtsc() - start_tsc, freq_ghz);

  std::printf("Time per frame (algorithm1, algorithm2) = (%.4f, %.4f) ms\n",
              ms0 / kMaxFrameNum, ms1 / kMaxFrameNum);

  // Check correctness
  constexpr float allowed_error = 1e-3;
  for (size_t i = 0; i < kMaxFrameNum; i++) {
    float* buf0 = (float*)recip_buffer_0[i % kFrameWnd];
    float* buf1 = (float*)recip_buffer_1[i % kFrameWnd];
    for (size_t j = 0; j < cfg->ofdm_data_num() * cfg->bs_ant_num(); j++) {
      ASSERT_LE(abs(buf0[j] - buf1[j]), allowed_error);
    }
  }

  calib_buffer.free();
  recip_buffer_0.free();
  recip_buffer_1.free();
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
