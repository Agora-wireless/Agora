#include <gtest/gtest.h>
// For some reason, gtest include order matters
#include <armadillo>

#include "config.h"
#include "gettime.h"
#include "utils.h"

static constexpr size_t kMaxFrameNum = 10;

/// Test correctness of two-step recriprocal calibration
TEST(TestRecip, Correctness) {
  auto* cfg = new Config("data/tddconfig-sim-ul.json");
  cfg->GenData();

  double freq_ghz = MeasureRdtscFreq();

  Table<complex_float> calib_buffer;
  Table<complex_float> recip_buffer_0;
  Table<complex_float> recip_buffer_1;
  recip_buffer_0.Calloc(kFrameWnd, cfg->ofdm_data_num_ * cfg->bs_ant_num_,
                        Agora_memory::Alignment_t::kK64Align);
  recip_buffer_1.Calloc(kFrameWnd, cfg->ofdm_data_num_ * cfg->bs_ant_num_,
                        Agora_memory::Alignment_t::kK64Align);
  calib_buffer.RandAllocCxFloat(kFrameWnd,
                                cfg->ofdm_data_num_ * cfg->bs_ant_num_,
                                Agora_memory::Alignment_t::kK64Align);

  std::printf("Reference antenna: %zu\n", cfg->ref_ant_);

  size_t start_tsc = Rdtsc();

  // Algorithm in reciprocity.cpp (use as ground truth)
  for (size_t i = 0; i < kMaxFrameNum; i++) {
    arma::cx_float* ptr_in =
        reinterpret_cast<arma::cx_float*>(calib_buffer[i % kFrameWnd]);
    arma::cx_fmat mat_input(ptr_in, cfg->ofdm_data_num_, cfg->bs_ant_num_,
                            false);
    arma::cx_fvec vec_calib_ref = mat_input.col(cfg->ref_ant_);
    arma::cx_float* recip_buff =
        reinterpret_cast<arma::cx_float*>(recip_buffer_0[i % kFrameWnd]);
    arma::cx_fmat calib_mat = mat_input.each_col() / vec_calib_ref;

    arma::cx_fmat recip_mat(recip_buff, cfg->bs_ant_num_, cfg->ofdm_data_num_,
                            false);
    recip_mat = calib_mat.st();

    for (size_t sc_id = 0; sc_id < cfg->ofdm_data_num_;
         sc_id += cfg->bs_ant_num_) {
      // TODO: interpolate instead of steps
      recip_mat
          .cols(sc_id,
                std::min(sc_id + cfg->bs_ant_num_ - 1, cfg->ofdm_data_num_ - 1))
          .each_col() = recip_mat.col(sc_id);
    }
  }

  double ms0 = CyclesToMs(Rdtsc() - start_tsc, freq_ghz);

  start_tsc = Rdtsc();

  // 2-step algorithm used in dofft and dozf
  // Partially tranpose is not tested here
  for (size_t i = 0; i < kMaxFrameNum; i++) {
    // In dofft
    for (size_t ant_id = 0; ant_id < cfg->bs_ant_num_; ant_id++) {
      auto* ptr_in = calib_buffer[i % kFrameWnd] + ant_id * cfg->ofdm_data_num_;
      for (size_t sc_id = 0; sc_id < cfg->ofdm_data_num_;
           sc_id += cfg->bs_ant_num_) {
        for (size_t j = 0; j < cfg->bs_ant_num_; j++) {
          ptr_in[std::min(sc_id + j, cfg->ofdm_data_num_ - 1)] = ptr_in[sc_id];
        }
      }
    }
    // Transpose
    arma::cx_float* ptr_calib =
        reinterpret_cast<arma::cx_float*>(calib_buffer[i % kFrameWnd]);
    arma::cx_fmat calib_mat(ptr_calib, cfg->ofdm_data_num_, cfg->bs_ant_num_,
                            false);
    arma::cx_float* recip_buff =
        reinterpret_cast<arma::cx_float*>(recip_buffer_1[i % kFrameWnd]);
    arma::cx_fmat recip_mat(recip_buff, cfg->bs_ant_num_, cfg->ofdm_data_num_,
                            false);
    recip_mat = calib_mat.st();

    // In dozf
    for (size_t sc_id = 0; sc_id < cfg->ofdm_data_num_; sc_id++) {
      arma::cx_float* ptr_in = reinterpret_cast<arma::cx_float*>(
          recip_buffer_1[i % kFrameWnd] + sc_id * cfg->bs_ant_num_);
      arma::cx_fvec recip_vec(ptr_in, cfg->bs_ant_num_, false);
      recip_vec = recip_vec / recip_vec(cfg->ref_ant_);
    }
  }

  double ms1 = CyclesToMs(Rdtsc() - start_tsc, freq_ghz);

  std::printf("Time per frame (algorithm1, algorithm2) = (%.4f, %.4f) ms\n",
              ms0 / kMaxFrameNum, ms1 / kMaxFrameNum);

  // Check correctness
  constexpr float kAllowedError = 1e-3;
  for (size_t i = 0; i < kMaxFrameNum; i++) {
    float* buf0 = (float*)recip_buffer_0[i % kFrameWnd];
    float* buf1 = (float*)recip_buffer_1[i % kFrameWnd];
    for (size_t j = 0; j < cfg->ofdm_data_num_ * cfg->bs_ant_num_; j++) {
      ASSERT_LE(abs(buf0[j] - buf1[j]), kAllowedError);
    }
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
