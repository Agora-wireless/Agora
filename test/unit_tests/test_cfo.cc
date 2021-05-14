#include <gtest/gtest.h>
// For some reason, gtest include order matters
#include "txrx_client.h"
#include "config.h"
#include "gettime.h"
#include "utils.h"
#include <armadillo>

static constexpr size_t kMaxFrameNum = 1;   // Only for checking timing

/// Test correctness of CFO estimation and correction
TEST(TestCFO, EstCorr)
{
    //auto cfg = new std::make_unique<Config>("data/tddconfig-sim-ul.json");
    auto* cfg = new Config("data/tddconfig-sim-ul.json");
    cfg->GenData();

    int n_threads = 1;
    int in_core_id = 0;
    double rate = cfg->Rate();
    double ms0, ms1;
    std::unique_ptr<RadioTxRx> ru_ = std::make_unique<RadioTxRx>(cfg, n_threads, in_core_id);

    const std::complex<double> j(0.0, 1.0);

    for (size_t cfoIdx = 0; cfoIdx < 100; cfoIdx++) {
        const double cfo_khz = 10 * cfoIdx;
        const double cfo_sim = cfo_khz / (rate * 1e-3);

        // Generate signal
        std::vector<std::complex<float>> gold_cf32_cfo;
        std::vector<std::vector<double>> gold_ifft
            = CommsLib::GetSequence(128, CommsLib::kGoldIfft);

        int goldReps = 2;
        for (size_t i = 0; i < (goldReps * gold_ifft[0].size()); i++) {
            std::complex<double> gold_cf32
                = std::complex<double>(gold_ifft[0][i % gold_ifft[0].size()], gold_ifft[1][i % gold_ifft[0].size()]);

            // Add CFO: CFO modeled as in "ML Estimation of Time and Frequency Offset in OFDM Systems"by Jan-Jaap van de Beek et.al.
            double tmp = 2 * M_PI * cfo_sim * i / cfg->BeaconLongSymLen();
            std::complex<double> cfo_tmp = exp(tmp * j);
            std::complex<float> samp_tmp = (std::complex<float>)(gold_cf32 * cfo_tmp);
            gold_cf32_cfo.push_back(samp_tmp);
        }

        // CFO estimation
        size_t start_tsc = GetTime::Rdtsc();
        const int sync_index = cfg->BeaconLongSymLen() * cfg->BeaconLongSymReps();
        for (size_t fidx = 0; fidx < kMaxFrameNum; fidx++) {
            ru_->CFOEstimation(sync_index, gold_cf32_cfo);
        }
        ms0 = GetTime::CyclesToMs(GetTime::Rdtsc() - start_tsc, cfg->FreqGhz());

        double cfo_est_khz = ru_->GetCFO() * rate * 1e-3;
        // printf("CFO Applied (kHz): %f ... CFO Estimated (kHz): %f ... DIFF: %f \n", cfo_khz, cfo_est_khz, cfo_khz - cfo_est_khz);

        complex_float* fft_buff;
        AllocBuffer1d(&fft_buff, gold_cf32_cfo.size(), Agora_memory::Alignment_t::kAlign64, 1);
        for (size_t i = 0; i < gold_cf32_cfo.size(); i++) {
            fft_buff[i].re = gold_cf32_cfo[i].real();
            fft_buff[i].im = gold_cf32_cfo[i].imag();
        }

        // CFO correction
        start_tsc = GetTime::Rdtsc();
        complex_float* corrected_vec;
        for (size_t fidx = 0; fidx < kMaxFrameNum; fidx++) {
            bool is_downlink = true;
            corrected_vec = ru_->CFOCorrection(
                is_downlink, (complex_float*)fft_buff, gold_cf32_cfo.size());
        }
        ms1 = GetTime::CyclesToMs(GetTime::Rdtsc() - start_tsc, cfg->FreqGhz());

        // Check if correction worked
        std::vector<std::complex<float>> verify_vec;
        for (size_t i = 0; i < gold_cf32_cfo.size(); i++) {
            std::complex<float> tmp(corrected_vec[i].re, corrected_vec[i].im);
            verify_vec.push_back(tmp);
        }
        ru_->CFOEstimation(sync_index, verify_vec);
        double cfo_after_corr = ru_->GetCFO();
        double cfo_residual_khz = (cfo_after_corr * rate * 1e-3);

        // Check correctness
        constexpr float allowed_error = 0.01;            // 10 Hz max error
        std::cout << "CFO APPLIED: " << cfo_khz << " CFO ESTIMATED: " << cfo_est_khz << " RESIDUAL ERR: " << cfo_residual_khz << std::endl;
        ASSERT_LE(abs(cfo_khz - cfo_est_khz), allowed_error);
        ASSERT_LE(abs(cfo_residual_khz - 0.0), allowed_error);
    }
    printf("Time per frame (estimation, correction) = (%.4f, %.4f) ms\n", ms0 / kMaxFrameNum, ms1 / kMaxFrameNum);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
