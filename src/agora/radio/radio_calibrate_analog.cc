/**
 * @file radio_calibrate_analog.cc
 * @brief Implementation file for the radio analog calibration functions
 * , i.e.  DC offset and IQ imbalance
 */
#include <chrono>
#include <thread>

#include "comms-lib.h"
#include "matplotlibcpp.h"
#include "radio_lib.h"
#include "radio_soapysdr.h"

namespace plt = matplotlibcpp;
static constexpr bool kIQImbalancePlot = false;
static constexpr double kToneTargetLevel = -10.0f;

static size_t CheckTone(size_t channel, std::vector<Radio*>& rx_devs,
                        std::vector<float> tone_levels, double fft_bin) {
  size_t completed = 0;
  const size_t n = 1024;
  const auto win = CommsLib::HannWindowFunction(n);
  const auto window_gain = CommsLib::WindowFunctionPower(win);

  for (size_t radio_idx = 0; radio_idx < rx_devs.size(); radio_idx++) {
    auto& tone_level = tone_levels.at(radio_idx);

    if (tone_level < kToneTargetLevel) {
      const auto samps = dynamic_cast<RadioSoapySdr*>(rx_devs.at(radio_idx))
                             ->SnoopSamples(channel, n);
      tone_level = CommsLib::MeasureTone(samps, win, window_gain, fft_bin, n);
      std::cout << "Radio ID " << rx_devs.at(radio_idx)->Id()
                << ": tone level =" << tone_level << std::endl;
    }

    if (tone_level >= kToneTargetLevel) {
      completed++;
    }
  }
  return completed;
}

static void PlotTones(size_t channel, std::vector<Radio*>& rx_devs,
                      std::vector<float> tone_levels, double fft_bin) {
  if (kIQImbalancePlot) {
    const size_t n = 1024;
    const auto win = CommsLib::HannWindowFunction(n);
    const auto window_gain = CommsLib::WindowFunctionPower(win);

    //Reread tone for each radio
    for (size_t radio_idx = 0; radio_idx < rx_devs.size(); radio_idx++) {
      const auto samps = dynamic_cast<RadioSoapySdr*>(rx_devs.at(radio_idx))
                             ->SnoopSamples(channel, n);
      auto tone_level =
          CommsLib::MeasureTone(samps, win, window_gain, fft_bin, n);

      std::cout << "Radio ID " << rx_devs.at(radio_idx)->Id()
                << ": old tone level =" << tone_levels.at(radio_idx)
                << ": new tone level =" << tone_level << std::endl;

      auto fft_mag = CommsLib::MagnitudeFft(samps, win, n);
      std::vector<double> mag_double(n);
      std::transform(fft_mag.begin(), fft_mag.end(), mag_double.begin(),
                     [](float cf) {
                       return 10 * std::max(std::log10((double)cf), -20.0);
                     });
      // std::vector<double> sampsDouble(N);
      // std::transform(samps.begin(), samps.end(), sampsDouble.begin(),
      //    [](std::complex<float> cf) {
      //        return cf.real();
      //    });
      plt::figure_size(1200, 780);
      // plt::plot(sampsDouble);
      plt::plot(mag_double);
      plt::xlim(0, (int)n);
      plt::ylim(-100, 100);
      // plt::ylim(-1, 1);
      plt::title("Spectrum figure After Gain Adjustment, FFT Window POWER " +
                 std::to_string(window_gain));
      plt::legend();
      plt::save("rx" + std::to_string(rx_devs.size()) + "_" +
                std::to_string(rx_devs.at(radio_idx)->Id()) + "_ch" +
                std::to_string(channel) + ".png");
    }
  }
}

static void AdjustRxGains(size_t channel, std::vector<Radio*>& rx_devs,
                          std::vector<float> tone_levels,
                          const std::string& gain_stage, double min_setting,
                          double max_setting) {
  for (size_t radio_idx = 0u; radio_idx < rx_devs.size(); radio_idx++) {
    auto& tone_level = tone_levels.at(radio_idx);
    if (tone_level < kToneTargetLevel) {
      const double new_gain_setting =
          std::min((kToneTargetLevel - tone_level) + min_setting, max_setting);
      dynamic_cast<RadioSoapySdr*>(rx_devs.at(radio_idx))
          ->SetRxGain(channel, gain_stage, new_gain_setting);
      std::cout << "Increasing RX gain level (" << gain_stage << ") to "
                << new_gain_setting << std::endl;
    }
  }
}

static void MeasureCorrection(RadioSoapySdr* ref, size_t channel,
                              size_t num_samples, size_t rx_center_tone,
                              size_t tx_center_tone,
                              const std::vector<float>& window,
                              double window_gain) {
  std::this_thread::sleep_for(std::chrono::milliseconds(SETTLE_TIME_MS));
  const auto samps = ref->SnoopSamples(channel, num_samples);
  const auto meas_dc_level = CommsLib::MeasureTone(samps, window, window_gain,
                                                   rx_center_tone, num_samples);
  const auto meas_imbalance_level = CommsLib::MeasureTone(
      samps, window, window_gain, rx_center_tone - tx_center_tone, num_samples);
  const auto desired_tone_level = CommsLib::MeasureTone(
      samps, window, window_gain, rx_center_tone + tx_center_tone, num_samples);
  std::cout << "DciqMinimize: dc level =" << meas_dc_level
            << " dB, im level=" << meas_imbalance_level
            << " dB, tone level=" << desired_tone_level << "dB" << std::endl;
}

void RadioConfig::AdjustCalibrationGains(std::vector<Radio*>& rx_devs,
                                         Radio* tx_dev, size_t channel,
                                         double fft_bin, bool plot) {
  const double target_level = -10.0f;
  const double attn_max = -18.0f;
  const double max_tx_gain = -6.0f;
  const size_t rx_devs_size = rx_devs.size();

  auto* tx_radio = dynamic_cast<RadioSoapySdr*>(tx_dev);

  tx_radio->ResetTxGains();
  for (size_t r = 0; r < rx_devs_size; r++) {
    dynamic_cast<RadioSoapySdr*>(rx_devs.at(r))->ResetRxGains();
  }

  tx_radio->SetTxCalGain(channel);
  std::vector<float> tone_levels(rx_devs_size, -200.0f);

  std::string gain_stage = "ATTN";
  double min_gain = attn_max;
  double max_gain = 0.0f;
  size_t adjusted = 0;
  bool first = true;
  while (adjusted < rx_devs_size) {
    std::this_thread::sleep_for(std::chrono::milliseconds(SETTLE_TIME_MS));
    adjusted = CheckTone(channel, rx_devs, tone_levels, fft_bin);

    // try bumping up tx gain first, if no rx succeeded
    if ((first == true) && (adjusted == 0)) {
      float max_tone_level =
          *max_element(tone_levels.begin(), tone_levels.end());

      const double gain_setting =
          std::min((target_level - max_tone_level) + attn_max, max_tx_gain);
      tx_radio->SetTxGain(channel, gain_stage, gain_setting);
      std::cout << "Increasing TX gain level (ATTN) to " << gain_setting
                << std::endl;
    } else {
      AdjustRxGains(channel, rx_devs, tone_levels, gain_stage, min_gain,
                    max_gain);
      //Finished
      if (gain_stage == "LNA") {
        break;
      } else {
        gain_stage = "LNA";
        min_gain = 0;
        max_gain = 30.0f;
      }
    }
    first = false;
  }
  if (plot) {
    PlotTones(channel, rx_devs, tone_levels, fft_bin);
  }
  std::cout << rx_devs_size - adjusted << " radios reached target level"
            << std::endl;
}

void RadioConfig::SetIqBalance(Radio* dev, int direction, size_t channel,
                               int gcorr, int iqcorr) {
  const auto gcorri = (gcorr < 0) ? 2047 - std::abs(gcorr) : 2047;
  const auto gcorrq = (gcorr > 0) ? 2047 - std::abs(gcorr) : 2047;
  const double gain_iq = double(gcorrq) / double(gcorri);
  const double phase_iq = 2 * std::atan(iqcorr / 2047.0);
  const std::complex<double> i_qcorr =
      gain_iq * std::exp(std::complex<double>(0, phase_iq));
  dynamic_cast<RadioSoapySdr*>(dev)->SetIQBalance(direction, channel, i_qcorr);
}

void RadioConfig::DciqMinimize(Radio* target_dev, Radio* ref_dev, int direction,
                               size_t channel, double rx_center_tone,
                               double tx_center_tone) {
  const size_t n = 1024;
  std::vector<float> win = CommsLib::HannWindowFunction(n);
  const auto window_gain = CommsLib::WindowFunctionPower(win);
  auto* target_radio = dynamic_cast<RadioSoapySdr*>(target_dev);
  auto* ref_radio = dynamic_cast<RadioSoapySdr*>(ref_dev);

  target_radio->SetIQBalance(direction, channel, 0.0);
  target_radio->SetDcOffset(direction, channel, {0.0f, 0.0f});

  // match the internal fixed point representation for DC correction
  const int fixed_scale = (direction == SOAPY_SDR_RX) ? 64 : 128;

  // measure initial
  MeasureCorrection(ref_radio, channel, n, rx_center_tone, tx_center_tone, win,
                    window_gain);

  // look through each correction arm twice
  float min_dc_level = 0.0f;
  std::complex<double> best_dc_corr(0.0, 0.0);
  constexpr size_t kMaxIterations = 4;
  for (size_t i = 0; i < kMaxIterations; i++) {
    int start = -fixed_scale;
    int stop = +fixed_scale;
    int step = 8;

    // narrow in for the final iteration set
    if (i > 1) {
      // restart with finer search
      if (i == 2) {
        min_dc_level = 0.0f;
      }

      const int center =
          int((((i % 2) == 0) ? best_dc_corr.imag() : best_dc_corr.real()) *
              fixed_scale);
      start = std::max<int>(start, center - 8);
      stop = std::min<int>(stop, center + 8);
      step = 1;
    }

    for (int j = start; j < stop; j += step) {
      // try I or Q arm based on iteration
      const auto dc_corr = ((i % 2) == 0)
                               ? std::complex<double>(best_dc_corr.real(),
                                                      double(j) / fixed_scale)
                               : std::complex<double>(double(j) / fixed_scale,
                                                      best_dc_corr.imag());
      target_radio->SetDcOffset(direction, channel, dc_corr);

      // measure the efficacy
      std::this_thread::sleep_for(std::chrono::milliseconds(SETTLE_TIME_MS));
      const auto samps = ref_radio->SnoopSamples(channel, n);
      const auto meas_dc_level =
          CommsLib::MeasureTone(samps, win, window_gain, rx_center_tone, n);

      // save desired results
      if (meas_dc_level < min_dc_level) {
        min_dc_level = meas_dc_level;
        best_dc_corr = dc_corr;
      }
    }
  }

  target_radio->SetDcOffset(direction, channel, best_dc_corr);
  if (direction == SOAPY_SDR_TX) {
    const long dccorri = std::lround(best_dc_corr.real() * fixed_scale);
    const long dccorrq = std::lround(best_dc_corr.imag() * fixed_scale);
    std::cout << "Optimized TX DC Offset: (" << dccorri << "," << dccorrq
              << ")\n";
  } else {
    long dcoffi = std::lround(best_dc_corr.real() * fixed_scale);
    if (dcoffi < 0) {
      dcoffi = (1 << 6) | std::abs(dcoffi);
    }

    long dcoffq = std::lround(best_dc_corr.imag() * fixed_scale);
    if (dcoffq < 0) {
      dcoffq = (1 << 6) | std::abs(dcoffq);
    }
    std::cout << "Optimized RX DC Offset: (" << dcoffi << "," << dcoffq
              << ")\n";
  }

  // correct IQ imbalance
  float min_imbalance_level = 0.0f;
  int bestgcorr = 0;
  int bestiqcorr = 0;
  for (size_t iter = 0; iter < kMaxIterations; iter++) {
    int start = -512;
    int stop = 512;
    int step = 8;

    if (iter > 1) {
      if (iter == 2) {
        min_imbalance_level = 0;  // restart with finer search
      }
      const int center = ((iter % 2) == 0) ? bestgcorr : bestiqcorr;
      start = std::max<int>(start, center - 8);
      stop = std::min<int>(stop, center + 8);
      step = 1;
    }
    // SoapySDR::logf(debugLogLevel, "start=%d, stop=%d, step=%d", start,
    // stop, step);
    for (int i = start; i < stop; i += step) {
      const int gcorr = ((iter % 2) == 0) ? i : bestgcorr;
      const int iqcorr = ((iter % 2) == 1) ? i : bestiqcorr;
      RadioConfig::SetIqBalance(target_dev, direction, channel, gcorr, iqcorr);

      // measure the efficacy
      std::this_thread::sleep_for(std::chrono::milliseconds(SETTLE_TIME_MS));
      const auto samps = ref_radio->SnoopSamples(channel, n);
      const auto meas_imbalance_level = CommsLib::MeasureTone(
          samps, win, window_gain, rx_center_tone - tx_center_tone, n);

      // save desired results
      if (meas_imbalance_level < min_imbalance_level) {
        min_imbalance_level = meas_imbalance_level;
        bestgcorr = gcorr;
        bestiqcorr = iqcorr;
      }
    }
  }

  // apply the ideal correction
  RadioConfig::SetIqBalance(target_dev, direction, channel, bestgcorr,
                            bestiqcorr);
  auto gcorri = (bestgcorr < 0) ? 2047 - std::abs(bestgcorr) : 2047;
  auto gcorrq = (bestgcorr > 0) ? 2047 - std::abs(bestgcorr) : 2047;
  std::cout << "Optimized IQ Imbalance Setting: GCorr (" << gcorri << ","
            << gcorrq << "), iqcorr=" << bestiqcorr << "\n";

  MeasureCorrection(ref_radio, channel, n, rx_center_tone, tx_center_tone, win,
                    window_gain);
}

void RadioConfig::DciqCalibrationProc(size_t channel) {
  std::cout << "****************************************************\n";
  std::cout << "   DC Offset and IQ Imbalance Calibration: Ch " << channel
            << std::endl;
  std::cout << "****************************************************\n";

  const double sample_rate = cfg_->Rate();
  const double center_rf_freq = cfg_->RadioRfFreq();
  const double tone_bb_freq = sample_rate / 7;
  const size_t total_radios = cfg_->NumRadios();

  size_t reference_radio = cfg_->RefRadio(0);
  auto* ref_dev =
      dynamic_cast<RadioSoapySdr*>(radios_.at(reference_radio).get());

  /*
   * Start with calibrating the rx paths on all radios using the reference
   * radio
   */
  std::cout << "Calibrating Rx Channels with Tx Reference Radio\n";
  ref_dev->InitRefTx(channel, center_rf_freq + tone_bb_freq);
  std::vector<Radio*> all_but_ref_devs;
  for (size_t r = 0; r < total_radios; r++) {
    if (r != reference_radio) {
      auto* cal_radio = dynamic_cast<RadioSoapySdr*>(radios_.at(r).get());
      cal_radio->InitCalRx(channel, center_rf_freq);
      all_but_ref_devs.push_back(radios_.at(r).get());
    }
  }
  //Can we move this code to InitRefTx????
  ref_dev->StartRefTx(channel);

  // Tune rx gains for calibration on all radios except reference radio
  // Tune tx gain on reference radio
  RadioConfig::AdjustCalibrationGains(all_but_ref_devs, ref_dev, channel,
                                      tone_bb_freq / sample_rate, true);

  // Minimize Rx DC offset and IQ Imbalance on all receiving radios
  // TODO: Parallelize this loop
  for (size_t r = 0; r < total_radios - 1; r++) {
    RadioConfig::DciqMinimize(all_but_ref_devs.at(r), all_but_ref_devs.at(r),
                              SOAPY_SDR_RX, channel, 0.0,
                              tone_bb_freq / sample_rate);
  }
  ref_dev->StopRefTx(channel);

  /*
   * Calibrate the rx path of the reference radio
   */
  std::cout << "Calibrating Rx Channel of the Reference Radio\n";
  std::vector<Radio*> ref_dev_container;
  ref_dev_container.push_back(ref_dev);
  auto* ref_ref_dev =
      dynamic_cast<RadioSoapySdr*>(all_but_ref_devs[reference_radio - 1]);

  ref_ref_dev->InitRefTx(channel, center_rf_freq + tone_bb_freq);
  // must set TX "RF" Freq to make sure, we continue using the same LO for rx
  // cal
  ref_dev->InitCalRx(channel, center_rf_freq);

  ref_ref_dev->StartRefTx(channel);

  // Tune rx gain for calibraion on reference radio
  // Tune tx gain on neighboring radio to reference radio
  RadioConfig::AdjustCalibrationGains(ref_dev_container, ref_ref_dev, channel,
                                      tone_bb_freq / sample_rate);
  RadioConfig::DciqMinimize(ref_dev, ref_dev, SOAPY_SDR_RX, channel, 0.0,
                            tone_bb_freq / sample_rate);

  ref_ref_dev->StopRefTx(channel);

  /*
   * Calibrate the tx path of the reference radio
   */
  std::cout << "Calibrating Tx Channels with Rx Reference Radio\n";
  const double tx_tone_bb_freq = sample_rate / 21;
  std::vector<Radio*> ref_ref_dev_container;
  ref_ref_dev_container.push_back(ref_ref_dev);

  // must set TX "RF" Freq to make sure, we continue using the same LO for rx
  // cal
  ref_ref_dev->SetFreqRf(channel, center_rf_freq);
  // Should this be nagative if we need
  // centerRfFreq-toneBBFreq at true center?
  ref_ref_dev->SetFreqBb(channel, -tone_bb_freq);
  ref_dev->SetFreqRf(channel, center_rf_freq);
  ref_dev->SetFreqBb(channel, tx_tone_bb_freq);
  ref_dev->StartRefTx(channel);

  // Tune tx gain for calibraion on reference antenna
  // Tune rx gain on neighboring radio to reference radio
  RadioConfig::AdjustCalibrationGains(
      ref_ref_dev_container, ref_dev, channel,
      (tone_bb_freq + tx_tone_bb_freq) / sample_rate);
  RadioConfig::DciqMinimize(ref_dev, ref_ref_dev, SOAPY_SDR_TX, channel,
                            tone_bb_freq / sample_rate,
                            tx_tone_bb_freq / sample_rate);

  // kill TX on ref at the end
  ref_dev->StopRefTx(channel);
  ref_dev->SetFreqBb(channel, 0.0);
  ref_ref_dev->SetFreqBb(channel, 0.0);

  /*
   * Now calibrate the tx paths on all other radios using the reference radio
   */
  std::cout << "Calibrating Tx Channel of the Reference Radio\n";
  // ref_dev->setFrequency(SOAPY_SDR_RX, channel, "RF", centerRfFreq);
  // Should this be nagative if we need
  // centerRfFreq-toneBBFreq at true center?
  ref_dev->SetFreqBb(channel, -tone_bb_freq);

  for (size_t r = 0; r < total_radios - 1; r++) {
    auto* radio = dynamic_cast<RadioSoapySdr*>(all_but_ref_devs.at(r));
    radio->SetFreqRf(channel, center_rf_freq);
    radio->SetFreqBb(channel, tx_tone_bb_freq);
    radio->StartRefTx(channel);
    // Tune tx gain for calibraion of the current radio
    // Tune rx gain on the reference radio
    RadioConfig::AdjustCalibrationGains(
        ref_dev_container, radio, channel,
        (tone_bb_freq + tx_tone_bb_freq) / sample_rate);
    RadioConfig::DciqMinimize(radio, ref_dev, SOAPY_SDR_TX, channel,
                              tone_bb_freq / sample_rate,
                              tx_tone_bb_freq / sample_rate);
    radio->StopRefTx(channel);
    radio->SetFreqBb(channel, 0.0);
  }
  std::cout << "****************************************************\n";
  std::cout << "   Ending DC Offset and IQ Imbalance Calibration\n";
  std::cout << "****************************************************\n";
}
