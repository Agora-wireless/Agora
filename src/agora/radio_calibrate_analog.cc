/**
 * @file radio_calibrate_analog.cc
 * @brief Implementation file for the radio analog calibration functions
 * , i.e.  DC offset and IQ imbalance
 */
#include "matplotlibcpp.h"
#include "radio_lib.h"

namespace plt = matplotlibcpp;
static constexpr bool kIQImbalancePlot = false;

void RadioConfig::AdjustCalibrationGains(
    std::vector<SoapySDR::Device*>& rx_devs, SoapySDR::Device* tx_dev,
    size_t channel, double fft_bin, bool plot) {
  using std::cout;
  using std::endl;
  const double target_level = -10;
  const double attn_max = -18;
  const size_t n = 1024;
  const size_t rx_devs_size = rx_devs.size();
  const auto win = CommsLib::HannWindowFunction(n);
  const auto window_gain = CommsLib::WindowFunctionPower(win);

  // reset all gains
  for (size_t ch = 0; ch < 2; ch++) {
    tx_dev->setGain(SOAPY_SDR_TX, ch, "PA2", 0);
    tx_dev->setGain(SOAPY_SDR_TX, ch, "PAD", 0);
    tx_dev->setGain(SOAPY_SDR_TX, ch, "IAMP", 0);
    tx_dev->setGain(SOAPY_SDR_TX, ch, "ATTN", attn_max);
  }

  for (size_t r = 0; r < rx_devs_size; r++) {
    for (size_t ch = 0; ch < 2; ch++) {
      rx_devs.at(r)->setGain(SOAPY_SDR_RX, ch, "LNA", 0);
      rx_devs.at(r)->setGain(SOAPY_SDR_RX, ch, "PGA", 0);
      rx_devs.at(r)->setGain(SOAPY_SDR_RX, ch, "TIA", 0);
      rx_devs.at(r)->setGain(SOAPY_SDR_RX, ch, "ATTN", attn_max);
      rx_devs.at(r)->setGain(SOAPY_SDR_RX, ch, "LNA2", 14.0);
    }
  }

  tx_dev->setGain(SOAPY_SDR_TX, channel, "PAD", 40);
  std::this_thread::sleep_for(std::chrono::milliseconds(SETTLE_TIME_MS));

  float max_tone_level = -200;
  std::vector<bool> adjusted_radios(rx_devs_size, false);
  std::vector<float> tone_levels(rx_devs_size, 0);
  size_t remaining_radios = adjusted_radios.size();

  for (size_t r = 0; r < rx_devs_size; r++) {
    const auto samps = RadioConfig::SnoopSamples(rx_devs.at(r), channel, n);
    auto tone_level =
        CommsLib::MeasureTone(samps, win, window_gain, fft_bin, n);
    if (tone_level >= target_level) {
      adjusted_radios.at(r) = true;
      remaining_radios--;
    }
    tone_levels.at(r) = tone_level;
    if (tone_level > max_tone_level) {
      max_tone_level = tone_level;
    }
    cout << "Node " << r << ": toneLevel0=" << tone_level << endl;
  }

  std::string next_gain_stage;
  if (remaining_radios == rx_devs_size) {
    // if all need adjustment, try bumping up tx gain first
    tx_dev->setGain(SOAPY_SDR_TX, channel, "ATTN",
                    std::min((target_level - max_tone_level) + attn_max, -6.0));
    cout << "Increasing TX gain level (ATTN) to "
         << std::min((target_level - max_tone_level) + attn_max, -6.0) << endl;
    next_gain_stage = "ATTN";
  } else {
    for (size_t r = 0; r < rx_devs_size; r++) {
      if (adjusted_radios.at(r) == false) {
        rx_devs.at(r)->setGain(
            SOAPY_SDR_RX, channel, "ATTN",
            std::min((target_level - tone_levels.at(r)) + attn_max, 0.0));
        cout << "Increasing RX gain level (ATTN) to "
             << std::min((target_level - max_tone_level) + attn_max, 0.0)
             << endl;
      }
    }
    next_gain_stage = "LNA";
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(SETTLE_TIME_MS));

  for (size_t r = 0; r < rx_devs_size; r++) {
    if (adjusted_radios.at(r) == false) {
      const auto samps = RadioConfig::SnoopSamples(rx_devs.at(r), channel, n);
      const float tone_level =
          CommsLib::MeasureTone(samps, win, window_gain, fft_bin, n);
      if (tone_level >= target_level) {
        adjusted_radios.at(r) = true;
        remaining_radios--;
      }
      tone_levels.at(r) = tone_level;
      cout << "Node " << r << ": toneLevel1=" << tone_level << endl;
    }
  }

  if (remaining_radios == 0) {
    return;
  }
  double min_gain = 0;
  double max_gain = 30;
  if (next_gain_stage == "ATTN") {
    min_gain = attn_max;
    max_gain = 0;
  }

  // adjust next gain stage
  for (size_t r = 0; r < rx_devs_size; r++) {
    if (adjusted_radios.at(r) == false) {
      rx_devs.at(r)->setGain(
          SOAPY_SDR_RX, channel, next_gain_stage,
          std::min((target_level - tone_levels.at(r)) + min_gain, max_gain));
      cout << "Increasing RX gain level (" << next_gain_stage << ") to "
           << std::min((target_level - tone_levels.at(r)) + min_gain, max_gain)
           << endl;
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(SETTLE_TIME_MS));

  for (size_t r = 0; r < rx_devs_size; r++) {
    if (adjusted_radios.at(r) == false) {
      const auto samps = RadioConfig::SnoopSamples(rx_devs.at(r), channel, n);
      auto tone_level =
          CommsLib::MeasureTone(samps, win, window_gain, fft_bin, n);
      if (tone_level > target_level) {
        adjusted_radios.at(r) = true;
        remaining_radios--;
      }
      tone_levels.at(r) = tone_level;
      cout << "Node " << r << ": toneLevel2=" << tone_level << endl;
    }
  }

  if (remaining_radios == 0 || next_gain_stage == "LNA") {
    return;
  }

  // adjust next gain stage
  min_gain = 0;
  max_gain = 30;
  for (size_t r = 0; r < rx_devs_size; r++) {
    if (adjusted_radios.at(r) == false) {
      rx_devs.at(r)->setGain(
          SOAPY_SDR_RX, channel, "LNA",
          std::min((target_level - tone_levels.at(r)), max_gain));
      cout << "Increasing RX gain level (LNA) to "
           << std::min((target_level - tone_levels.at(r)) + min_gain, max_gain)
           << endl;
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(SETTLE_TIME_MS));

  for (size_t r = 0; r < rx_devs_size; r++) {
    if (adjusted_radios.at(r)) {
      continue;
    }
    auto samps = RadioConfig::SnoopSamples(rx_devs.at(r), channel, n);
    float tone_level =
        CommsLib::MeasureTone(samps, win, window_gain, fft_bin, n);
    if (tone_level > target_level) {
      adjusted_radios.at(r) = true;
      remaining_radios--;
    }
    tone_levels.at(r) = tone_level;
    cout << "Node " << r << ": toneLevel3=" << tone_level << endl;
    if (kIQImbalancePlot && plot) {
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
      plt::save("rx" + std::to_string(rx_devs_size) + "_" + std::to_string(r) +
                "_ch" + std::to_string(channel) + ".png");
    }
  }

  std::cout << rx_devs_size - remaining_radios << " radios reached target level"
            << std::endl;
}

void RadioConfig::SetIqBalance(SoapySDR::Device* dev, int direction,
                               size_t channel, int gcorr, int iqcorr) {
  auto gcorri = (gcorr < 0) ? 2047 - std::abs(gcorr) : 2047;
  auto gcorrq = (gcorr > 0) ? 2047 - std::abs(gcorr) : 2047;
  double gain_iq = double(gcorrq) / double(gcorri);
  double phase_iq = 2 * std::atan(iqcorr / 2047.0);
  std::complex<double> i_qcorr =
      gain_iq * std::exp(std::complex<double>(0, phase_iq));
  dev->setIQBalance(direction, channel, i_qcorr);
}

void RadioConfig::DciqMinimize(SoapySDR::Device* targetDev,
                               SoapySDR::Device* refDev, int direction,
                               size_t channel, double rxCenterTone,
                               double txCenterTone) {
  size_t n = 1024;
  std::vector<float> win = CommsLib::HannWindowFunction(n);
  const auto window_gain = CommsLib::WindowFunctionPower(win);

  targetDev->setIQBalance(direction, channel, 0.0);
  targetDev->setDCOffset(direction, channel, 0.0);

  // match the internal fixed point representation for DC correction
  const int fixed_scale = (direction == SOAPY_SDR_RX) ? 64 : 128;

  // measure initial
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(SETTLE_TIME_MS));
    const auto samps = SnoopSamples(refDev, channel, n);
    const auto meas_dc_level =
        CommsLib::MeasureTone(samps, win, window_gain, rxCenterTone, n);
    const auto meas_imbalance_level = CommsLib::MeasureTone(
        samps, win, window_gain, rxCenterTone - txCenterTone, n);
    const auto desired_tone_level = CommsLib::MeasureTone(
        samps, win, window_gain, rxCenterTone + txCenterTone, n);
    std::cout << "dciqMinimize initial: dcLvl=" << meas_dc_level
              << " dB, imLvl=" << meas_imbalance_level
              << " dB, toneLevel=" << desired_tone_level << "dB" << std::endl;
  }

  // look through each correction arm twice
  float min_dc_level(0);
  std::complex<double> best_dc_corr(0.0, 0.0);
  for (size_t iter = 0; iter < 4; iter++) {
    int start = -fixed_scale;
    int stop = +fixed_scale;
    int step = 8;
    if (iter == 2) {
      min_dc_level = 0;  // restart with finer search
    }
    if (iter > 1)  // narrow in for the final iteration set
    {
      const int center =
          int((((iter % 2) == 0) ? best_dc_corr.imag() : best_dc_corr.real()) *
              fixed_scale);
      start = std::max<int>(start, center - 8),
      stop = std::min<int>(stop, center + 8), step = 1;
    }
    for (int i = start; i < stop; i += step) {
      // try I or Q arm based on iteration
      const auto dc_corr = ((iter % 2) == 0)
                               ? std::complex<double>(best_dc_corr.real(),
                                                      double(i) / fixed_scale)
                               : std::complex<double>(double(i) / fixed_scale,
                                                      best_dc_corr.imag());
      targetDev->setDCOffset(direction, channel, dc_corr);

      // measure the efficacy
      std::this_thread::sleep_for(std::chrono::milliseconds(SETTLE_TIME_MS));
      const auto samps = SnoopSamples(refDev, channel, n);
      const auto meas_dc_level =
          CommsLib::MeasureTone(samps, win, window_gain, rxCenterTone, n);

      // save desired results
      if (meas_dc_level < min_dc_level) {
        min_dc_level = meas_dc_level;
        best_dc_corr = dc_corr;
      }
    }
  }

  targetDev->setDCOffset(direction, channel, best_dc_corr);
  if (direction == SOAPY_SDR_TX) {
    long dccorri = std::lround(best_dc_corr.real() * 128);
    long dccorrq = std::lround(best_dc_corr.imag() * 128);
    std::cout << "Optimized TX DC Offset: (" << dccorri << "," << dccorrq
              << ")\n";
  } else {
    long dcoffi = std::lround(best_dc_corr.real() * 64);
    if (dcoffi < 0) {
      dcoffi = (1 << 6) | std::abs(dcoffi);
    }

    long dcoffq = std::lround(best_dc_corr.imag() * 64);
    if (dcoffq < 0) {
      dcoffq = (1 << 6) | std::abs(dcoffq);
    }
    std::cout << "Optimized RX DC Offset: (" << dcoffi << "," << dcoffq
              << ")\n";
  }

  // correct IQ imbalance
  float min_imbalance_level(0);
  int bestgcorr = 0;
  int bestiqcorr = 0;
  for (size_t iter = 0; iter < 4; iter++) {
    int start = -512;
    int stop = 512;
    int step = 8;
    if (iter == 2) {
      min_imbalance_level = 0;  // restart with finer search
    }
    if (iter > 1) {
      const int center = ((iter % 2) == 0) ? bestgcorr : bestiqcorr;
      start = std::max<int>(start, center - 8);
      stop = std::min<int>(stop, center + 8), step = 1;
    }
    // SoapySDR::logf(debugLogLevel, "start=%d, stop=%d, step=%d", start,
    // stop, step);
    for (int i = start; i < stop; i += step) {
      const int gcorr = ((iter % 2) == 0) ? i : bestgcorr;
      const int iqcorr = ((iter % 2) == 1) ? i : bestiqcorr;
      RadioConfig::SetIqBalance(targetDev, direction, channel, gcorr, iqcorr);

      // measure the efficacy
      std::this_thread::sleep_for(std::chrono::milliseconds(SETTLE_TIME_MS));
      const auto samps = SnoopSamples(refDev, channel, n);
      const auto meas_imbalance_level = CommsLib::MeasureTone(
          samps, win, window_gain, rxCenterTone - txCenterTone, n);

      // save desired results
      if (meas_imbalance_level < min_imbalance_level) {
        min_imbalance_level = meas_imbalance_level;
        bestgcorr = gcorr;
        bestiqcorr = iqcorr;
      }
    }
  }

  // apply the ideal correction
  RadioConfig::SetIqBalance(targetDev, direction, channel, bestgcorr,
                            bestiqcorr);
  auto gcorri = (bestgcorr < 0) ? 2047 - std::abs(bestgcorr) : 2047;
  auto gcorrq = (bestgcorr > 0) ? 2047 - std::abs(bestgcorr) : 2047;
  std::cout << "Optimized IQ Imbalance Setting: GCorr (" << gcorri << ","
            << gcorrq << "), iqcorr=" << bestiqcorr << "\n";

  // measure corrections
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(SETTLE_TIME_MS));
    const auto samps = SnoopSamples(refDev, channel, n);
    const auto meas_dc_level =
        CommsLib::MeasureTone(samps, win, window_gain, rxCenterTone, n);
    const auto meas_imbalance_level = CommsLib::MeasureTone(
        samps, win, window_gain, rxCenterTone - txCenterTone, n);
    const auto desired_tone_level = CommsLib::MeasureTone(
        samps, win, window_gain, rxCenterTone + txCenterTone, n);
    std::cout << "dciqMinimize final: dcLvl=" << meas_dc_level
              << " dB, imLvl=" << meas_imbalance_level
              << " dB, toneLevel=" << desired_tone_level << "dB" << std::endl;
  }
}

void RadioConfig::DciqCalibrationProc(size_t channel) {
  std::cout << "****************************************************\n";
  std::cout << "   DC Offset and IQ Imbalance Calibration: Ch " << channel
            << std::endl;
  std::cout << "****************************************************\n";
  double sample_rate = cfg_->Rate();
  double center_rf_freq = cfg_->RadioRfFreq();
  double tone_bb_freq = sample_rate / 7;
  size_t radio_size = cfg_->NumRadios();

  size_t reference_radio = cfg_->RefRadio(0);
  SoapySDR::Device* ref_dev = ba_stn_[reference_radio];

  /*
   * Start with calibrating the rx paths on all radios using the reference
   * radio
   */
  std::cout << "Calibrating Rx Channels with Tx Reference Radio\n";
  ref_dev->setFrequency(SOAPY_SDR_TX, channel, "RF",
                        center_rf_freq + tone_bb_freq);
  //ref_dev->setFrequency(SOAPY_SDR_RX, channel, "RF", center_rf_freq);
  ref_dev->setFrequency(SOAPY_SDR_TX, channel, "BB", 0);
  std::vector<SoapySDR::Device*> all_but_ref_devs;
  for (size_t r = 0; r < radio_size; r++) {
    if (r == reference_radio) {
      continue;
    }
    SoapySDR::Device* dev = ba_stn_[r];
    // must set TX "RF" Freq to make sure, we continue using the same LO for
    // rx cal
    dev->setFrequency(SOAPY_SDR_TX, channel, "RF", center_rf_freq);
    dev->setFrequency(SOAPY_SDR_RX, channel, "RF", center_rf_freq);
    dev->setFrequency(SOAPY_SDR_RX, channel, "BB", 0);
    dev->setDCOffsetMode(SOAPY_SDR_RX, channel, false);
    all_but_ref_devs.push_back(dev);
  }
  ref_dev->writeSetting(SOAPY_SDR_TX, channel, "TSP_TSG_CONST",
                        std::to_string(1 << 14));
  ref_dev->writeSetting(SOAPY_SDR_TX, channel, "TX_ENB_OVERRIDE", "true");

  // Tune rx gains for calibration on all radios except reference radio
  // Tune tx gain on reference radio
  RadioConfig::AdjustCalibrationGains(all_but_ref_devs, ref_dev, channel,
                                      tone_bb_freq / sample_rate, true);

  // Minimize Rx DC offset and IQ Imbalance on all receiving radios
  // TODO: Parallelize this loop
  for (size_t r = 0; r < radio_size - 1; r++) {
    RadioConfig::DciqMinimize(all_but_ref_devs[r], all_but_ref_devs[r],
                              SOAPY_SDR_RX, channel, 0.0,
                              tone_bb_freq / sample_rate);
  }

  ref_dev->writeSetting(SOAPY_SDR_TX, channel, "TSP_TSG_CONST", "NONE");
  ref_dev->writeSetting(SOAPY_SDR_TX, channel, "TX_ENB_OVERRIDE", "false");

  /*
   * Calibrate the rx path of the reference radio
   */
  std::cout << "Calibrating Rx Channel of the Reference Radio\n";
  std::vector<SoapySDR::Device*> ref_dev_container;
  ref_dev_container.push_back(ref_dev);
  SoapySDR::Device* ref_ref_dev = all_but_ref_devs[reference_radio - 1];

  ref_ref_dev->setFrequency(SOAPY_SDR_TX, channel, "RF",
                            center_rf_freq + tone_bb_freq);
  ref_ref_dev->setFrequency(SOAPY_SDR_TX, channel, "BB", 0);
  // must set TX "RF" Freq to make sure, we continue using the same LO for rx
  // cal
  ref_dev->setFrequency(SOAPY_SDR_TX, channel, "RF", center_rf_freq);
  ref_dev->setFrequency(SOAPY_SDR_RX, channel, "RF", center_rf_freq);
  ref_dev->setFrequency(SOAPY_SDR_RX, channel, "BB", 0);
  ref_dev->setDCOffsetMode(SOAPY_SDR_RX, channel, false);

  ref_ref_dev->writeSetting(SOAPY_SDR_TX, channel, "TSP_TSG_CONST",
                            std::to_string(1 << 14));
  ref_ref_dev->writeSetting(SOAPY_SDR_TX, channel, "TX_ENB_OVERRIDE", "true");

  // Tune rx gain for calibraion on reference radio
  // Tune tx gain on neighboring radio to reference radio
  RadioConfig::AdjustCalibrationGains(ref_dev_container, ref_ref_dev, channel,
                                      tone_bb_freq / sample_rate);
  RadioConfig::DciqMinimize(ref_dev, ref_dev, SOAPY_SDR_RX, channel, 0.0,
                            tone_bb_freq / sample_rate);

  ref_ref_dev->writeSetting(SOAPY_SDR_TX, channel, "TSP_TSG_CONST", "NONE");
  ref_ref_dev->writeSetting(SOAPY_SDR_TX, channel, "TX_ENB_OVERRIDE", "false");

  /*
   * Calibrate the tx path of the reference radio
   */
  std::cout << "Calibrating Tx Channels with Rx Reference Radio\n";
  double tx_tone_bb_freq = sample_rate / 21;
  std::vector<SoapySDR::Device*> ref_ref_dev_container;
  ref_ref_dev_container.push_back(ref_ref_dev);

  // must set TX "RF" Freq to make sure, we continue using the same LO for rx
  // cal
  ref_ref_dev->setFrequency(SOAPY_SDR_TX, channel, "RF", center_rf_freq);
  ref_ref_dev->setFrequency(SOAPY_SDR_RX, channel, "RF", center_rf_freq);
  ref_ref_dev->setFrequency(
      SOAPY_SDR_RX, channel, "BB",
      -tone_bb_freq);  // Should this be nagative if we need
  // centerRfFreq-toneBBFreq at true center?
  ref_dev->setFrequency(SOAPY_SDR_TX, channel, "RF", center_rf_freq);
  ref_dev->setFrequency(SOAPY_SDR_TX, channel, "BB", tx_tone_bb_freq);
  ref_dev->writeSetting(SOAPY_SDR_TX, channel, "TSP_TSG_CONST",
                        std::to_string(1 << 14));
  ref_dev->writeSetting(SOAPY_SDR_TX, channel, "TX_ENB_OVERRIDE", "true");

  // Tune tx gain for calibraion on reference antenna
  // Tune rx gain on neighboring radio to reference radio
  RadioConfig::AdjustCalibrationGains(
      ref_ref_dev_container, ref_dev, channel,
      (tone_bb_freq + tx_tone_bb_freq) / sample_rate);
  RadioConfig::DciqMinimize(ref_dev, ref_ref_dev, SOAPY_SDR_TX, channel,
                            tone_bb_freq / sample_rate,
                            tx_tone_bb_freq / sample_rate);

  // kill TX on ref at the end
  ref_dev->writeSetting(SOAPY_SDR_TX, channel, "TSP_TSG_CONST", "NONE");
  ref_dev->writeSetting(SOAPY_SDR_TX, channel, "TX_ENB_OVERRIDE", "false");
  ref_dev->setFrequency(SOAPY_SDR_TX, channel, "BB", 0);
  ref_ref_dev->setFrequency(SOAPY_SDR_RX, channel, "BB", 0);

  /*
   * Now calibrate the tx paths on all other radios using the reference radio
   */
  std::cout << "Calibrating Tx Channel of the Reference Radio\n";
  // refDev->setFrequency(SOAPY_SDR_RX, channel, "RF", centerRfFreq);
  ref_dev->setFrequency(SOAPY_SDR_RX, channel, "BB",
                        -tone_bb_freq);  // Should this be nagative if we need
  // centerRfFreq-toneBBFreq at true center?
  for (size_t r = 0; r < radio_size - 1; r++) {
    all_but_ref_devs[r]->setFrequency(SOAPY_SDR_TX, channel, "RF",
                                      center_rf_freq);
    all_but_ref_devs[r]->setFrequency(SOAPY_SDR_TX, channel, "BB",
                                      tx_tone_bb_freq);
    all_but_ref_devs[r]->writeSetting(SOAPY_SDR_TX, channel, "TSP_TSG_CONST",
                                      std::to_string(1 << 14));
    all_but_ref_devs[r]->writeSetting(SOAPY_SDR_TX, channel, "TX_ENB_OVERRIDE",
                                      "true");
    // Tune tx gain for calibraion of the current radio
    // Tune rx gain on the reference radio
    RadioConfig::AdjustCalibrationGains(
        ref_dev_container, all_but_ref_devs[r], channel,
        (tone_bb_freq + tx_tone_bb_freq) / sample_rate);
    RadioConfig::DciqMinimize(all_but_ref_devs[r], ref_dev, SOAPY_SDR_TX,
                              channel, tone_bb_freq / sample_rate,
                              tx_tone_bb_freq / sample_rate);
    all_but_ref_devs[r]->writeSetting(SOAPY_SDR_TX, channel, "TX_ENB_OVERRIDE",
                                      "false");
    all_but_ref_devs[r]->writeSetting(SOAPY_SDR_TX, channel, "TSP_TSG_CONST",
                                      "NONE");
    all_but_ref_devs[r]->setFrequency(SOAPY_SDR_TX, channel, "BB", 0);
  }
  std::cout << "****************************************************\n";
  std::cout << "   Ending DC Offset and IQ Imbalance Calibration\n";
  std::cout << "****************************************************\n";
}
