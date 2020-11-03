#include <gtest/gtest.h>
// For some reason, gtest include order matters
#include "config.hpp"
#include "gettime.h"
#include "utils.h"
#include <armadillo>

static constexpr size_t kMaxFrameNum = 10;

/// Test correctness of two-step recriprocal calibration
TEST(TestRecip, Correctness)
{

    auto* cfg = new Config("data/tddconfig-sim-ul.json");
    cfg->genData();

    double freq_ghz = measure_rdtsc_freq();

    Table<complex_float> calib_buffer, recip_buffer_0, recip_buffer_1;
    recip_buffer_0.calloc(kFrameWnd, cfg->OFDM_DATA_NUM * cfg->BS_ANT_NUM, 64);
    recip_buffer_1.calloc(kFrameWnd, cfg->OFDM_DATA_NUM * cfg->BS_ANT_NUM, 64);
    calib_buffer.rand_alloc_cx_float(
        kFrameWnd, cfg->OFDM_DATA_NUM * cfg->BS_ANT_NUM, 64);

    printf("Reference antenna: %zu\n", cfg->ref_ant);

    size_t start_tsc = rdtsc();

    // Algorithm in reciprocity.cpp (use as ground truth)
    for (size_t i = 0; i < kMaxFrameNum; i++) {
        arma::cx_float* ptr_in
            = reinterpret_cast<arma::cx_float*>(calib_buffer[i % kFrameWnd]);
        arma::cx_fmat mat_input(
            ptr_in, cfg->OFDM_DATA_NUM, cfg->BS_ANT_NUM, false);
        arma::cx_fvec vec_calib_ref = mat_input.col(cfg->ref_ant);
        arma::cx_float* recip_buff
            = reinterpret_cast<arma::cx_float*>(recip_buffer_0[i % kFrameWnd]);
        arma::cx_fmat calib_mat = mat_input.each_col() / vec_calib_ref;

        arma::cx_fmat recip_mat(
            recip_buff, cfg->BS_ANT_NUM, cfg->OFDM_DATA_NUM, false);
        recip_mat = calib_mat.st();

        for (size_t sc_id = 0; sc_id < cfg->OFDM_DATA_NUM;
             sc_id += cfg->BS_ANT_NUM) {
            // TODO: interpolate instead of steps
            recip_mat
                .cols(sc_id,
                    std::min(
                        sc_id + cfg->BS_ANT_NUM - 1, cfg->OFDM_DATA_NUM - 1))
                .each_col()
                = recip_mat.col(sc_id);
        }
    }

    double ms0 = cycles_to_ms(rdtsc() - start_tsc, freq_ghz);

    start_tsc = rdtsc();

    // 2-step algorithm used in dofft and dozf
    // Partially tranpose is not tested here
    for (size_t i = 0; i < kMaxFrameNum; i++) {
        // In dofft
        for (size_t ant_id = 0; ant_id < cfg->BS_ANT_NUM; ant_id++) {
            auto* ptr_in
                = calib_buffer[i % kFrameWnd] + ant_id * cfg->OFDM_DATA_NUM;
            for (size_t sc_id = 0; sc_id < cfg->OFDM_DATA_NUM;
                 sc_id += cfg->BS_ANT_NUM) {
                for (size_t j = 0; j < cfg->BS_ANT_NUM; j++)
                    ptr_in[std::min(sc_id + j, cfg->OFDM_DATA_NUM - 1)]
                        = ptr_in[sc_id];
            }
        }
        // Transpose
        arma::cx_float* ptr_calib
            = reinterpret_cast<arma::cx_float*>(calib_buffer[i % kFrameWnd]);
        arma::cx_fmat calib_mat(
            ptr_calib, cfg->OFDM_DATA_NUM, cfg->BS_ANT_NUM, false);
        arma::cx_float* recip_buff
            = reinterpret_cast<arma::cx_float*>(recip_buffer_1[i % kFrameWnd]);
        arma::cx_fmat recip_mat(
            recip_buff, cfg->BS_ANT_NUM, cfg->OFDM_DATA_NUM, false);
        recip_mat = calib_mat.st();

        // In dozf
        for (size_t sc_id = 0; sc_id < cfg->OFDM_DATA_NUM; sc_id++) {
            arma::cx_float* ptr_in = reinterpret_cast<arma::cx_float*>(
                recip_buffer_1[i % kFrameWnd] + sc_id * cfg->BS_ANT_NUM);
            arma::cx_fvec recip_vec(ptr_in, cfg->BS_ANT_NUM, false);
            recip_vec = recip_vec / recip_vec(cfg->ref_ant);
        }
    }

    double ms1 = cycles_to_ms(rdtsc() - start_tsc, freq_ghz);

    printf("Time per frame (algorithm1, algorithm2) = (%.4f, %.4f) ms\n",
        ms0 / kMaxFrameNum, ms1 / kMaxFrameNum);

    // Check correctness
    constexpr float allowed_error = 1e-3;
    for (size_t i = 0; i < kMaxFrameNum; i++) {
        float* buf0 = (float*)recip_buffer_0[i % kFrameWnd];
        float* buf1 = (float*)recip_buffer_1[i % kFrameWnd];
        for (size_t j = 0; j < cfg->OFDM_DATA_NUM * cfg->BS_ANT_NUM; j++) {
            ASSERT_LE(abs(buf0[j] - buf1[j]), allowed_error);
        }
    }
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
