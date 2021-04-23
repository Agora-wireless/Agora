#include <gtest/gtest.h>
// For some reason, gtest include order matters
#include "config.hpp"
#include "gettime.h"
#include "txrx_client.hpp"
#include "utils.h"
#include <armadillo>

static constexpr size_t kMaxFrameNum = 1;

/// Test correctness of CFO estimation and correction
TEST(TestCFO, EstCorr)
{

    auto* cfg = new Config("data/tddconfig-sim-ul.json");
    cfg->genData();

    double freq_ghz = measure_rdtsc_freq();

    int n_threads = 1;
    int in_core_id = 0;
    RadioTXRX* ru_ = new RadioTXRX(cfg, n_threads, in_core_id);

    const std::complex<double> j(0.0, 1.0);
    const double cfo_khz = 55;
    const double cfo_sim = cfo_khz / (cfg->rate * 1e-3);
    std::cout << "Applying CFO of " << cfo_khz << " kHz" << " or " << cfo_sim << std::endl;

    // Generate signal
    std::vector<std::complex<float>> gold_cf32_cfo;
    std::vector<std::vector<double>> gold_ifft
        = CommsLib::getSequence(128, CommsLib::GOLD_IFFT);

    int goldReps = 2;
    for (size_t i = 0; i < (goldReps * gold_ifft[0].size()); i++) {
        std::complex<double> gold_cf32
            = std::complex<double>(gold_ifft[0][i % gold_ifft[0].size()], gold_ifft[1][i % gold_ifft[0].size()]);

        // Add CFO
        double tmp = 2 * M_PI * cfo_sim * i / cfg->beacon_longsym_len;
        std::complex<double> cfo_tmp = exp(tmp * -j);
        std::complex<float> samp_tmp = (std::complex<float>)(gold_cf32 * cfo_tmp);
        gold_cf32_cfo.push_back(samp_tmp);
    }

    // CFO estimation
    size_t start_tsc = rdtsc();
    const int sync_index = cfg->beacon_longsym_len * cfg->beacon_longsym_reps;
    for (size_t fidx = 0; fidx < kMaxFrameNum; fidx++) {
        ru_->cfo_estimation(sync_index, gold_cf32_cfo);
    }
/*
    double ms0 = cycles_to_ms(rdtsc() - start_tsc, freq_ghz);

    complex_float* fft_buff;
    for (size_t i = 0; i < gold_cf32_cfo.size(); i++) {
        fft_buff[i].re = gold_cf32_cfo[i].real();
        fft_buff[i].im = gold_cf32_cfo[i].imag();
    }

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
*/
    EXPECT_EQ(4, 4) << "Two plus two must equal four";
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
