#include <gtest/gtest.h>
// For some reason, gtest include order matters
#include "config.hpp"
#include "gettime.h"
#include "txrx_client.hpp"
#include "utils.h"
#include <armadillo>

static constexpr size_t kMaxFrameNum = 1;

/// Test correctness of CFO estimation and correction
TEST(TestCFO, Correctness)
{

    auto* cfg = new Config("data/tddconfig-sim-ul.json");
    cfg->genData();

    double freq_ghz = measure_rdtsc_freq();

    std::unique_ptr<RadioTXRX> ru_;

    const std::complex<double> j(0.0, 1.0);
    const double cfo_sim = 10;

    // Generate signal
    std::vector<std::complex<float>> gold_cf32_cfo;
    std::vector<std::vector<double>> gold_ifft
        = CommsLib::getSequence(128, CommsLib::GOLD_IFFT);

    for (size_t i = 0; i < 128; i++) {
        std::complex<float> gold_cf32
            = std::complex<float>(gold_ifft[0][i], gold_ifft[1][i]);

        // Add CFO
        double tmp = 2 * M_PI * cfo_sim * i;
        std::complex<double> cfo_tmp = exp(tmp * -j);
        gold_cf32_cfo.push_back(gold_cf32 * cfo_tmp);
    }

    // CFO estimation
    size_t start_tsc = rdtsc();
    for (size_t fidx = 0; fidx < kMaxFrameNum; fidx++) {
        int sync_index = 0;
        ru_->cfo_estimation(sync_index, gold_cf32_cfo);
    }

    double ms0 = cycles_to_ms(rdtsc() - start_tsc, freq_ghz);

    complex_float* fft_buff = (complex_float*)gold_cf32_cfo[0];

    // CFO correction
    start_tsc = rdtsc();
    for (size_t fidx = 0; fidx < kMaxFrameNum; fidx++) {
        bool is_downlink = true;
        ru_->cfo_correction(
            is_downlink, (complex_float*)fft_buff, cfg->OFDM_CA_NUM);
    }
    double ms1 = cycles_to_ms(rdtsc() - start_tsc, freq_ghz);

    printf("Time per frame (estimation, correction) = (%.4f, %.4f) ms\n",
        ms0 / kMaxFrameNum, ms1 / kMaxFrameNum);

    // Check correctness
    /*
    constexpr float allowed_error = 1e-3;
    for (size_t i = 0; i < kMaxFrameNum; i++) {
        float* buf0 = (float*)recip_buffer_0[i % kFrameWnd];
        float* buf1 = (float*)recip_buffer_1[i % kFrameWnd];
        for (size_t j = 0; j < cfg->OFDM_DATA_NUM * cfg->BS_ANT_NUM; j++) {
            ASSERT_LE(abs(buf0[j] - buf1[j]), allowed_error);
        }
    }
    */
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
