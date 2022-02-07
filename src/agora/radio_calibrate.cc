/**
 * @file radio_calibrate.cc
 * @brief Implementation file for the radio configuration calibration functions
 */
#include "matplotlibcpp.h"
#include "radio_lib.h"

namespace plt = matplotlibcpp;
static constexpr size_t kMaxArraySampleOffset = 10;
static constexpr bool kReciprocalCalibPlot = false;
static constexpr bool kIQImbalancePlot = false;
static constexpr bool kVerboseCalibration = false;

std::vector<std::complex<float>> RadioConfig::SnoopSamples(
    SoapySDR::Device* dev, size_t channel, size_t readSize) {
  std::vector<uint32_t> samps_int =
      dev->readRegisters("RX_SNOOPER", channel, readSize);
  std::vector<std::complex<float>> samps =
      Utils::Uint32tocfloat(samps_int, "IQ");
  return samps;
}

void RadioConfig::AdjustCalibrationGains(std::vector<SoapySDR::Device*> rxDevs,
                                         SoapySDR::Device* txDev,
                                         size_t channel, double fftBin,
                                         bool plot) {
  using std::cout;
  using std::endl;
  double target_level = -10;
  double attn_max = -18;
  size_t n = 1024;
  size_t rx_devs_size = rxDevs.size();
  auto win = CommsLib::HannWindowFunction(n);
  const auto window_gain = CommsLib::WindowFunctionPower(win);

  // reset all gains
  for (size_t ch = 0; ch < 2; ch++) {
    txDev->setGain(SOAPY_SDR_TX, ch, "PA2", 0);
    txDev->setGain(SOAPY_SDR_TX, ch, "PAD", 0);
    txDev->setGain(SOAPY_SDR_TX, ch, "IAMP", 0);
    txDev->setGain(SOAPY_SDR_TX, ch, "ATTN", attn_max);
  }

  for (size_t r = 0; r < rx_devs_size; r++) {
    for (size_t ch = 0; ch < 2; ch++) {
      rxDevs[r]->setGain(SOAPY_SDR_RX, ch, "LNA", 0);
      rxDevs[r]->setGain(SOAPY_SDR_RX, ch, "PGA", 0);
      rxDevs[r]->setGain(SOAPY_SDR_RX, ch, "TIA", 0);
      rxDevs[r]->setGain(SOAPY_SDR_RX, ch, "ATTN", attn_max);
      rxDevs[r]->setGain(SOAPY_SDR_RX, ch, "LNA2", 14.0);
    }
  }

  txDev->setGain(SOAPY_SDR_TX, channel, "PAD", 40);
  std::this_thread::sleep_for(std::chrono::milliseconds(SETTLE_TIME_MS));

  float max_tone_level = -200;
  std::vector<bool> adjusted_radios(rx_devs_size, false);
  std::vector<float> tone_levels(rx_devs_size, 0);
  size_t remaining_radios = adjusted_radios.size();
  for (size_t r = 0; r < rx_devs_size; r++) {
    const auto samps = RadioConfig::SnoopSamples(rxDevs[r], channel, n);
    auto tone_level = CommsLib::MeasureTone(samps, win, window_gain, fftBin, n);
    if (tone_level >= target_level) {
      adjusted_radios[r] = true;
      remaining_radios--;
    }
    tone_levels[r] = tone_level;
    if (tone_level > max_tone_level) {
      max_tone_level = tone_level;
    }
    cout << "Node " << r << ": toneLevel0=" << tone_level << endl;
  }

  std::string next_gain_stage;
  if (remaining_radios == rx_devs_size) {
    // if all need adjustment, try bumping up tx gain first
    txDev->setGain(SOAPY_SDR_TX, channel, "ATTN",
                   std::min((target_level - max_tone_level) + attn_max, -6.0));
    cout << "Increasing TX gain level (ATTN) to "
         << std::min((target_level - max_tone_level) + attn_max, -6.0) << endl;
    next_gain_stage = "ATTN";
  } else {
    for (size_t r = 0; r < rx_devs_size; r++) {
      if (adjusted_radios[r]) {
        continue;
      }
      rxDevs[r]->setGain(
          SOAPY_SDR_RX, channel, "ATTN",
          std::min((target_level - tone_levels[r]) + attn_max, 0.0));
      cout << "Increasing RX gain level (ATTN) to "
           << std::min((target_level - max_tone_level) + attn_max, 0.0) << endl;
    }
    next_gain_stage = "LNA";
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(SETTLE_TIME_MS));

  for (size_t r = 0; r < rx_devs_size; r++) {
    if (adjusted_radios[r]) {
      continue;
    }
    const auto samps = RadioConfig::SnoopSamples(rxDevs[r], channel, n);
    float tone_level =
        CommsLib::MeasureTone(samps, win, window_gain, fftBin, n);
    if (tone_level >= target_level) {
      adjusted_radios[r] = true;
      remaining_radios--;
    }
    tone_levels[r] = tone_level;
    cout << "Node " << r << ": toneLevel1=" << tone_level << endl;
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
    if (adjusted_radios[r]) {
      continue;
    }
    rxDevs[r]->setGain(
        SOAPY_SDR_RX, channel, next_gain_stage,
        std::min((target_level - tone_levels[r]) + min_gain, max_gain));
    cout << "Increasing RX gain level (" << next_gain_stage << ") to "
         << std::min((target_level - tone_levels[r]) + min_gain, max_gain)
         << endl;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(SETTLE_TIME_MS));

  for (size_t r = 0; r < rx_devs_size; r++) {
    if (adjusted_radios[r]) {
      continue;
    }
    const auto samps = RadioConfig::SnoopSamples(rxDevs[r], channel, n);
    auto tone_level = CommsLib::MeasureTone(samps, win, window_gain, fftBin, n);
    if (tone_level > target_level) {
      adjusted_radios[r] = true;
      remaining_radios--;
    }
    tone_levels[r] = tone_level;
    cout << "Node " << r << ": toneLevel2=" << tone_level << endl;
  }

  if (remaining_radios == 0 || next_gain_stage == "LNA") {
    return;
  }

  // adjust next gain stage
  min_gain = 0;
  max_gain = 30;
  for (size_t r = 0; r < rx_devs_size; r++) {
    if (adjusted_radios[r]) {
      continue;
    }
    rxDevs[r]->setGain(SOAPY_SDR_RX, channel, "LNA",
                       std::min((target_level - tone_levels[r]), max_gain));
    cout << "Increasing RX gain level (LNA) to "
         << std::min((target_level - tone_levels[r]) + min_gain, max_gain)
         << endl;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(SETTLE_TIME_MS));

  for (size_t r = 0; r < rx_devs_size; r++) {
    if (adjusted_radios[r]) {
      continue;
    }
    auto samps = RadioConfig::SnoopSamples(rxDevs[r], channel, n);
    float tone_level =
        CommsLib::MeasureTone(samps, win, window_gain, fftBin, n);
    if (tone_level > target_level) {
      adjusted_radios[r] = true;
      remaining_radios--;
    }
    tone_levels[r] = tone_level;
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
  ref_dev->setFrequency(SOAPY_SDR_RX, channel, "RF", center_rf_freq);
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

void RadioConfig::AdjustDelays(std::vector<int> offset) {
  // adjust all trigger delay for all radios
  // with respect to the first non-ref radio
  size_t ref_offset = cfg_->RefAnt(0) == 0 ? 1 : 0;
  for (size_t i = 0; i < offset.size(); i++) {
    if (i == cfg_->RefAnt(0)) {
      continue;
    }
    int delta = offset[ref_offset] - offset[i];
    std::cout << "sample_adjusting delay of node " << i << " by " << delta
              << std::endl;
    int iter = delta < 0 ? -delta : delta;
    for (int j = 0; j < iter; j++) {
      if (delta < 0) {
        ba_stn_[i]->writeSetting("ADJUST_DELAYS", "-1");
      } else {
        ba_stn_[i]->writeSetting("ADJUST_DELAYS", "1");
      }
    }
  }
}

bool RadioConfig::InitialCalib(bool sample_adjust) {
  static constexpr size_t kRxTimeoutUs = 1000000;
  static constexpr size_t kTxTimeoutUs = 1000000;
  // excludes zero padding
  size_t seq_len = cfg_->PilotCf32().size();
  size_t read_len = cfg_->PilotCi16().size();

  // Transmitting from only one chain, create a null vector for chainB
  std::vector<std::complex<int16_t>> dummy_ci16(read_len, 0);

  std::vector<void*> txbuff0(2);
  std::vector<void*> txbuff1(2);
  txbuff0[0] = cfg_->PilotCi16().data();
  if (cfg_->NumChannels() == 2) {
    std::vector<std::complex<int16_t>> zeros(read_len,
                                             std::complex<int16_t>(0, 0));
    txbuff0[1] = zeros.data();
    txbuff1[0] = zeros.data();
    txbuff1[1] = cfg_->PilotCi16().data();
  }

  std::vector<std::vector<std::complex<int16_t>>> buff;
  // int ant = cfg_->num_channels();
  size_t m = cfg_->BsAntNum();
  // TODO: Fix for multi-cell
  size_t r = cfg_->NumRadios();
  size_t ref = cfg_->RefRadio(0);
  // allocate for uplink and downlink directions
  buff.resize(2 * m);
  for (size_t i = 0; i < 2 * m; i++) {
    buff.at(i).resize(read_len);
  }

  std::vector<std::complex<int16_t>> dummybuff(read_len);
  // DrainBuffers();

  //for (size_t i = 0; i < r; i++) {
  //  for (size_t ch = 0; ch < cfg_->NumChannels(); ch++) {
  //    ba_stn_.at(i)->setGain(
  //        SOAPY_SDR_TX, ch, "PAD",
  //        ch != 0u ? cfg_->CalibTxGainB() : cfg_->CalibTxGainA());
  //  }
  //  ba_stn_.at(i)->writeSetting("TDD_CONFIG", "{\"tdd_enabled\":false}");
  //  ba_stn_.at(i)->writeSetting("TDD_MODE", "false");
  //  ba_stn_.at(i)->activateStream(this->tx_streams_.at(i));
  //}

  size_t good_csi_cnt = 0;
  size_t n = 0;
  const size_t max_retries = 3;
  // second condition is for when too many attemps fail
  while ((good_csi_cnt < calib_meas_num_) && (n < (6 * calib_meas_num_))) {
    long long tx_time(0);
    long long rx_time(0);
    bool good_csi = true;
    // Transmit from Beamforming Antennas to Ref Antenna (Down)
    for (size_t i = 0; i < r; i++) {
      if (good_csi == false) {
        break;
      }
      if (i == ref) {
        continue;
      }

      // Send a separate pilot from each antenna
      for (size_t ch = 0; ch < cfg_->NumChannels(); ch++) {
        int tx_flags = SOAPY_SDR_WAIT_TRIGGER | SOAPY_SDR_END_BURST;
        size_t retry = 0;
        bool bad_read = false;
        while (retry < max_retries) {
          bad_read = false;
          int ret = ba_stn_.at(i)->writeStream(
              this->tx_streams_.at(i), ch > 0 ? txbuff1.data() : txbuff0.data(),
              read_len, tx_flags, tx_time, kTxTimeoutUs);
          if (ret < (int)read_len) {
            std::cout << "bad write\n";
          }

          int rx_flags_activate = SOAPY_SDR_WAIT_TRIGGER | SOAPY_SDR_END_BURST;
          ret = ba_stn_.at(ref)->activateStream(
              this->rx_streams_.at(ref), rx_flags_activate, rx_time, read_len);

          Go();  // trigger

          int rx_flags = SOAPY_SDR_END_BURST;
          std::vector<void*> rxbuff0(2);
          rxbuff0.at(0) = buff.at(cfg_->NumChannels() * i + ch).data();
          if (cfg_->NumChannels() == 2) {
            rxbuff0.at(1) = dummybuff.data();
          }
          ret = ba_stn_.at(ref)->readStream(this->rx_streams_.at(ref),
                                            rxbuff0.data(), read_len, rx_flags,
                                            rx_time, kRxTimeoutUs);
          if (ret < (int)read_len) {
            std::cout << "bad read (" << ret << ") at node " << ref
                      << " from node " << i << std::endl;
            retry++;
          } else {
            break;
          }
        }
        if (bad_read) {
          good_csi = false;
          break;
        }
      }
    }
    // Transmit from Ref Antenna to Beamforming Antennas (Up)
    if (good_csi == true) {
      int tx_flags = SOAPY_SDR_WAIT_TRIGGER | SOAPY_SDR_END_BURST;
      size_t retry = 0;
      bool bad_read = false;
      while (retry < max_retries) {
        bad_read = false;
        int ret = ba_stn_.at(ref)->writeStream(this->tx_streams_.at(ref),
                                               txbuff0.data(), read_len,
                                               tx_flags, tx_time, kTxTimeoutUs);
        if (ret < (int)read_len) {
          std::cout << "bad write\n";
        }

        for (size_t i = 0; i < r; i++) {
          if (i != ref) {
            int rx_flags = SOAPY_SDR_WAIT_TRIGGER | SOAPY_SDR_END_BURST;
            ret = ba_stn_.at(i)->activateStream(this->rx_streams_.at(i),
                                                rx_flags, rx_time, read_len);
          }
        }

        Go();  // Trigger

        int rx_flags = SOAPY_SDR_END_BURST;
        for (size_t i = 0; i < r; i++) {
          if (good_csi == false) {
            break;
          }
          if (i == ref) {
            continue;
          }
          std::vector<void*> rxbuff(2);
          rxbuff.at(0) = buff.at(m + cfg_->NumChannels() * i).data();
          if (cfg_->NumChannels() == 2) {
            rxbuff.at(1) = buff.at(m + cfg_->NumChannels() * i + 1).data();
          }
          ret = ba_stn_.at(i)->readStream(this->rx_streams_.at(i),
                                          rxbuff.data(), read_len, rx_flags,
                                          rx_time, kRxTimeoutUs);
          if (ret < (int)read_len) {
            bad_read = true;
            std::cout << "Bad read (" << ret << ") at node " << i
                      << " from node " << ref << std::endl;
          }
        }
        if (bad_read) {
          retry++;
        } else {
          break;
        }
      }
      if (bad_read) {
        good_csi = false;
      }
    }

    std::vector<std::vector<std::complex<int16_t>>> noise_buff;
    if (good_csi == true) {
      noise_buff.resize(m);
      for (size_t i = 0; i < r; i++) {
        int rx_flags = SOAPY_SDR_END_BURST;
        int ret = ba_stn_.at(i)->activateStream(this->rx_streams_.at(i),
                                                rx_flags, rx_time, read_len);
        std::vector<void*> rxbuff(2);
        noise_buff.at(cfg_->NumChannels() * i).resize(read_len);
        rxbuff.at(0) = noise_buff.at(cfg_->NumChannels() * i).data();

        if (cfg_->NumChannels() == 2) {
          noise_buff.at(cfg_->NumChannels() * i + 1).resize(read_len);
          rxbuff.at(1) = noise_buff.at(cfg_->NumChannels() * i + 1).data();
        }
        ret = ba_stn_.at(i)->readStream(this->rx_streams_.at(i), rxbuff.data(),
                                        read_len, rx_flags, rx_time,
                                        kRxTimeoutUs);
        if (ret < (int)read_len) {
          good_csi = false;
          std::cout << "bad noise read (" << ret << ") at node " << i
                    << std::endl;
        }
        // ba_stn_.at(i)->deactivateStream(this->rx_streams_.at(i));
      }
    }

    std::vector<std::vector<std::complex<float>>> noise(m);
    std::vector<std::vector<std::complex<float>>> up(m);
    std::vector<std::vector<std::complex<float>>> dn(m);
    std::vector<size_t> start_up(m, 0);
    std::vector<size_t> start_dn(m, 0);
    std::vector<int> offset(r);
    if (good_csi == true) {
      for (size_t i = 0; i < m; i++) {
        noise[i].resize(read_len);
        std::transform(noise_buff.at(i).begin(), noise_buff.at(i).end(),
                       noise.at(i).begin(), [](std::complex<int16_t> ci) {
                         return std::complex<float>(ci.real() / 32768.0,
                                                    ci.imag() / 32768.0);
                       });
      }

      for (size_t i = 0; i < m; i++) {
        if (i / cfg_->NumChannels() == ref) {
          continue;
        }

        up[i].resize(read_len);
        dn[i].resize(read_len);
        std::transform(buff.at(m + i).begin(), buff.at(m + i).end(),
                       up.at(i).begin(), [](std::complex<int16_t> ci) {
                         return std::complex<float>(ci.real() / 32768.0,
                                                    ci.imag() / 32768.0);
                       });
        std::transform(buff.at(i).begin(), buff.at(i).end(), dn.at(i).begin(),
                       [](std::complex<int16_t> ci) {
                         return std::complex<float>(ci.real() / 32768.0,
                                                    ci.imag() / 32768.0);
                       });
      }

      std::stringstream ss0;
      ss0 << "SNR_dn" << n << " = [";
      std::stringstream ss1;
      ss1 << "SNR_up" << n << " = [";
      for (size_t i = 0; i < m; i++) {
        if (good_csi == false) {
          break;
        }
        if (i / cfg_->NumChannels() == ref) {
          continue;
        }
        if (i % cfg_->NumChannels() == 0) {
          size_t peak_up =
              CommsLib::FindPilotSeq(up.at(i), cfg_->PilotCf32(), seq_len);
          size_t peak_dn =
              CommsLib::FindPilotSeq(dn.at(i), cfg_->PilotCf32(), seq_len);
          start_up[i] =
              peak_up < seq_len ? 0 : peak_up - seq_len + cfg_->CpLen();
          start_dn[i] =
              peak_dn < seq_len ? 0 : peak_dn - seq_len + cfg_->CpLen();
          if (kVerboseCalibration) {
            std::cout << "receive starting position from/to node "
                      << i / cfg_->NumChannels() << ": " << peak_up << "/"
                      << peak_dn << std::endl;
          }
          if ((start_up.at(i) == 0) || (start_dn.at(i) == 0)) {
            good_csi = false;
            std::cout << "Potential bad pilots: At node " << i
                      << " uplink/downlink peak found at index " << peak_up
                      << "/" << peak_dn << std::endl;
            if (kVerboseCalibration) {
              std::cout << "dn(" << i << ")=";
              for (auto& s : dn.at(i)) {
                std::cout << s.real() << "+1j*" << s.imag() << " ";
              }
              std::cout << std::endl;
              std::cout << "up(" << i << ")=";
              for (auto& s : up.at(i)) {
                std::cout << s.real() << "+1j*" << s.imag() << " ";
              }
              std::cout << std::endl;
            }
            break;
          }
        } else {
          start_up.at(i) = start_up.at(i - 1);
          start_dn.at(i) = start_dn.at(i - 1);
        }

        float sig_up = 0;
        float noise_up = 0;
        if (cfg_->OfdmCaNum() + start_up.at(i) >= up.at(i).size()) {
          std::cout << "up(" << i << ")=";
          //for (size_t s = 0; s < up.at(i).size(); s++) {
          //  std::cout << up.at(i).at(s).real() << "+1j*"
          //            << up.at(i).at(s).imag() << " ";
          //}
          std::cout << std::endl;
          std::cout << "Uplink pilot offset (" << start_up.at(i)
                    << ") too large!" << std::endl;
          good_csi = false;
          break;
        }
        for (size_t q = 0; q < cfg_->OfdmCaNum(); q++) {
          sig_up += std::pow(std::abs(up.at(i).at(q + start_up.at(i))), 2);
          noise_up += std::pow(std::abs(noise.at(i).at(q + start_up.at(i))), 2);
        }
        ss1 << 10 * std::log10(sig_up / noise_up) << " ";

        float sig_dn = 0;
        float noise_dn = 0;
        if (cfg_->OfdmCaNum() + start_dn.at(i) >= dn.at(i).size()) {
          std::cout << "Downlink pilot offset (" << start_dn.at(i)
                    << ") too large!" << std::endl;
          std::cout << "dn(" << i << ")=";
          //for (size_t s = 0; s < dn.at(i).size(); s++) {
          //  std::cout << dn.at(i).at(s).real() << "+1j*"
          //            << dn.at(i).at(s).imag() << " ";
          //}
          std::cout << std::endl;
          good_csi = false;
          break;
        }
        for (size_t q = 0; q < cfg_->OfdmCaNum(); q++) {
          sig_dn += std::pow(std::abs(dn.at(i).at(q + start_dn.at(i))), 2);
          noise_dn += std::pow(
              std::abs(noise.at(cfg_->RefAnt(0)).at(q + start_dn.at(i))), 2);
        }
        ss0 << 10 * std::log10(sig_dn / noise_dn) << " ";
        if (kReciprocalCalibPlot) {
          std::vector<double> up_i(read_len);
          std::transform(up.at(i).begin(), up.at(i).end(), up_i.begin(),
                         [](std::complex<double> cd) { return cd.real(); });

          std::vector<double> dn_i(read_len);
          std::transform(dn.at(i).begin(), dn.at(i).end(), dn_i.begin(),
                         [](std::complex<double> cd) { return cd.real(); });

          plt::figure_size(1200, 780);
          plt::plot(up_i);
          // plt::xlim(0, read_len);
          plt::ylim(-1, 1);
          plt::title("ant " + std::to_string(cfg_->RefAnt(0)) +
                     " (ref) to ant " + std::to_string(i));
          plt::legend();
          plt::save("up_" + std::to_string(i) + ".png");

          plt::figure_size(1200, 780);
          plt::plot(dn_i);
          // plt::xlim(0, read_len);
          plt::ylim(-1, 1);
          plt::title("ant " + std::to_string(i) + " to ant (ref)" +
                     std::to_string(cfg_->RefAnt(0)));
          plt::legend();
          plt::save("dn_" + std::to_string(i) + ".png");
        }
        if (i % cfg_->NumChannels() == 0) {
          if ((i > 0) &&
              ((std::abs((int)start_up.at(i) -
                         (int)start_up.at(i - cfg_->NumChannels())) >
                static_cast<int>(kMaxArraySampleOffset)) ||
               (std::abs((int)start_dn.at(i) -
                         (int)start_dn.at(i - cfg_->NumChannels())) >
                static_cast<int>(
                    kMaxArraySampleOffset)))) {  // make sure offsets are not too
                                                 // different from each other
            std::cout << "Uplink and Downlink pilot offsets (" << start_up.at(i)
                      << "/" << start_up.at(i - cfg_->NumChannels()) << ", "
                      << start_dn.at(i) << "/"
                      << start_dn.at(i - cfg_->NumChannels())
                      << ") are far apart!" << std::endl;
            good_csi = false;
            break;
          }
          offset.at(i / cfg_->NumChannels()) = start_up.at(i);
        }
      }
      ss0 << "];\n";
      ss1 << "];\n";
      if (kVerboseCalibration) {
        std::cout << ss1.str();
        std::cout << ss0.str();
      }
    }

    n++;  // increment number of measurement attemps
    if (good_csi == false) {
      std::cout << "Attempt " << n << " failed. Retrying..." << std::endl;
      continue;
    } else {
      std::cout << "Attempt " << n << " Succeeded. Processing " << good_csi_cnt
                << "th calibration result..." << std::endl;

      for (size_t i = 0; i < m; i++) {
        size_t id = i;
        if (cfg_->ExternalRefNode(0) && i / cfg_->NumChannels() == ref) {
          continue;
        }
        if (cfg_->ExternalRefNode(0) && (i / cfg_->NumChannels() > ref)) {
          id = i - cfg_->NumChannels();
        }
        if (kVerboseCalibration) {  // print time-domain data
          std::cout << "up_t" << id << " = [";
          for (size_t j = 0; j < read_len; j++) {
            std::cout << buff.at(m + i).at(j).real() << "+1j*"
                      << buff.at(m + i).at(j).imag() << " ";
          }
          std::cout << "];" << std::endl;
          std::cout << "dn_t" << id << " = [";
          for (size_t j = 0; j < read_len; j++) {
            std::cout << buff.at(i).at(j).real() << "+1j*"
                      << buff.at(i).at(j).imag() << " ";
          }
          std::cout << "];" << std::endl;
        }
        // computing reciprocity calibration matrix
        auto first_up = up.at(i).begin() + start_up.at(i);
        auto last_up = up.at(i).begin() + start_up.at(i) + cfg_->OfdmCaNum();
        std::vector<std::complex<float>> up_ofdm(first_up, last_up);
        assert(up_ofdm.size() == cfg_->OfdmCaNum());

        auto first_dn = dn.at(i).begin() + start_dn.at(i);
        auto last_dn = dn.at(i).begin() + start_dn.at(i) + cfg_->OfdmCaNum();
        std::vector<std::complex<float>> dn_ofdm(first_dn, last_dn);
        assert(dn_ofdm.size() == cfg_->OfdmCaNum());

        auto dn_f = CommsLib::FFT(dn_ofdm, cfg_->OfdmCaNum());
        auto up_f = CommsLib::FFT(up_ofdm, cfg_->OfdmCaNum());
        if (cfg_->ExternalRefNode(0) == false &&
            i / cfg_->NumChannels() == ref) {
          for (size_t j = 0; j < cfg_->OfdmCaNum(); j++) {
            dn_f[j] = std::complex<float>(1, 0);
            up_f[j] = std::complex<float>(1, 0);
          }
        }
        arma::cx_fvec dn_vec(
            reinterpret_cast<arma::cx_float*>(&dn_f[cfg_->OfdmDataStart()]),
            cfg_->OfdmDataNum(), false);
        arma::cx_fvec calib_dl_vec(
            reinterpret_cast<arma::cx_float*>(
                &init_calib_dl_[good_csi_cnt][id * cfg_->OfdmDataNum()]),
            cfg_->OfdmDataNum(), false);
        calib_dl_vec = dn_vec;

        arma::cx_fvec up_vec(
            reinterpret_cast<arma::cx_float*>(&up_f[cfg_->OfdmDataStart()]),
            cfg_->OfdmDataNum(), false);
        arma::cx_fvec calib_ul_vec(
            reinterpret_cast<arma::cx_float*>(
                &init_calib_ul_[good_csi_cnt][id * cfg_->OfdmDataNum()]),
            cfg_->OfdmDataNum(), false);
        calib_ul_vec = up_vec;
        // Utils::print_vec(dn_vec / up_vec,
        //     "n" + std::to_string(good_csi_cnt) + "_ant" + std::to_string(i));
      }

      // sample_adjusting trigger delays based on lts peak index
      if (sample_adjust && good_csi_cnt == 0) {  // just do it once
        AdjustDelays(offset);
      }
      good_csi_cnt++;
    }
  }
  //for (size_t i = 0; i < r; i++) {
  //  ba_stn_.at(i)->deactivateStream(this->tx_streams_.at(i));
  //  ba_stn_.at(i)->deactivateStream(this->rx_streams_.at(i));
  //  for (size_t ch = 0; ch < cfg_->NumChannels(); ch++) {
  //    ba_stn_.at(i)->setGain(SOAPY_SDR_TX, ch, "PAD",
  //                           ch != 0u ? cfg_->TxGainB() : cfg_->TxGainA());
  //  }
  //}
  return good_csi_cnt == calib_meas_num_;
}
