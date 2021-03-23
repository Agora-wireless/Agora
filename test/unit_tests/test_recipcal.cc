#include <gtest/gtest.h>
// For some reason, gtest include order matters
#include <armadillo>

#include "config.h"
#include "gettime.h"
#include "utils.h"

static constexpr size_t kMaxFrameNum = 10;

/// Test correctness of two-step recriprocal calibration
TEST(TestRecip, Correctness) {
  auto cfg = std::make_unique<Config>("data/tddconfig-sim-ul.json");
  cfg->GenData();

  double freq_ghz = GetTime::MeasureRdtscFreq();

  Table<complex_float> calib_buffer;
  Table<complex_float> recip_buffer_0;
  Table<complex_float> recip_buffer_1;
  recip_buffer_0.Calloc(kFrameWnd, cfg->OfdmDataNum() * cfg->BsAntNum(),
                        Agora_memory::Alignment_t::kAlign64);
  recip_buffer_1.Calloc(kFrameWnd, cfg->OfdmDataNum() * cfg->BsAntNum(),
                        Agora_memory::Alignment_t::kAlign64);
  calib_buffer.RandAllocCxFloat(kFrameWnd, cfg->OfdmDataNum() * cfg->BsAntNum(),
                                Agora_memory::Alignment_t::kAlign64);

  std::printf("Reference antenna: %zu\n", cfg->RefAnt());

  size_t start_tsc = GetTime::Rdtsc();

  // Algorithm in reciprocity.cpp (use as ground truth)
  for (size_t i = 0; i < kMaxFrameNum; i++) {
    auto* ptr_in =
        reinterpret_cast<arma::cx_float*>(calib_buffer[i % kFrameWnd]);
    arma::cx_fmat mat_input(ptr_in, cfg->OfdmDataNum(), cfg->BsAntNum(), false);
    arma::cx_fvec vec_calib_ref = mat_input.col(cfg->RefAnt());
    auto* recip_buff =
        reinterpret_cast<arma::cx_float*>(recip_buffer_0[i % kFrameWnd]);
    arma::cx_fmat calib_mat = mat_input.each_col() / vec_calib_ref;

    arma::cx_fmat recip_mat(recip_buff, cfg->BsAntNum(), cfg->OfdmDataNum(),
                            false);
    recip_mat = calib_mat.st();

    for (size_t sc_id = 0; sc_id < cfg->OfdmDataNum();
         sc_id += cfg->BsAntNum()) {
      // TODO: interpolate instead of steps
      recip_mat
          .cols(sc_id,
                std::min(sc_id + cfg->BsAntNum() - 1, cfg->OfdmDataNum() - 1))
          .each_col() = recip_mat.col(sc_id);
    }
  }

  double ms0 = GetTime::CyclesToMs(GetTime::Rdtsc() - start_tsc, freq_ghz);

  start_tsc = GetTime::Rdtsc();

  // 2-step algorithm used in dofft and dozf
  // Partially tranpose is not tested here
  for (size_t i = 0; i < kMaxFrameNum; i++) {
    // In dofft
    for (size_t ant_id = 0; ant_id < cfg->BsAntNum(); ant_id++) {
      auto* ptr_in = calib_buffer[i % kFrameWnd] + ant_id * cfg->OfdmDataNum();
      for (size_t sc_id = 0; sc_id < cfg->OfdmDataNum();
           sc_id += cfg->BsAntNum()) {
        for (size_t j = 0; j < cfg->BsAntNum(); j++) {
          ptr_in[std::min(sc_id + j, cfg->OfdmDataNum() - 1)] = ptr_in[sc_id];
        }
      }
    }
    // Transpose
    auto* ptr_calib =
        reinterpret_cast<arma::cx_float*>(calib_buffer[i % kFrameWnd]);
    arma::cx_fmat calib_mat(ptr_calib, cfg->OfdmDataNum(), cfg->BsAntNum(),
                            false);
    auto* recip_buff =
        reinterpret_cast<arma::cx_float*>(recip_buffer_1[i % kFrameWnd]);
    arma::cx_fmat recip_mat(recip_buff, cfg->BsAntNum(), cfg->OfdmDataNum(),
                            false);
    recip_mat = calib_mat.st();

    // In dozf
    for (size_t sc_id = 0; sc_id < cfg->OfdmDataNum(); sc_id++) {
      auto* ptr_in = reinterpret_cast<arma::cx_float*>(
          recip_buffer_1[i % kFrameWnd] + sc_id * cfg->BsAntNum());
      arma::cx_fvec recip_vec(ptr_in, cfg->BsAntNum(), false);
      recip_vec = recip_vec / recip_vec(cfg->RefAnt());
    }
  }

  double ms1 = GetTime::CyclesToMs(GetTime::Rdtsc() - start_tsc, freq_ghz);

  std::printf("Time per frame (algorithm1, algorithm2) = (%.4f, %.4f) ms\n",
              ms0 / kMaxFrameNum, ms1 / kMaxFrameNum);

  // Check correctness
  constexpr float kAllowedError = 1e-3;
  for (size_t i = 0; i < kMaxFrameNum; i++) {
    auto* buf0 = reinterpret_cast<float*>(recip_buffer_0[i % kFrameWnd]);
    auto* buf1 = reinterpret_cast<float*>(recip_buffer_1[i % kFrameWnd]);
    for (size_t j = 0; j < cfg->OfdmDataNum() * cfg->BsAntNum(); j++) {
      ASSERT_LE(abs(buf0[j] - buf1[j]), kAllowedError);
    }
  }

  calib_buffer.Free();
  recip_buffer_0.Free();
  recip_buffer_1.Free();
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
