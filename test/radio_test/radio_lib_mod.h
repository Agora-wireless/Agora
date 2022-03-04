/**
 * @file radio_lib_mod.h
 * @brief Declaration file for the RadioConfigNoRxStream class.
 */
#ifndef RADIO_LIB_MOD_H_
#define RADIO_LIB_MOD_H_

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Time.hpp>
#include <chrono>
#include <complex>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>

#include "config.h"

class RadioConfigNoRxStream {
 public:
  explicit RadioConfigNoRxStream(Config* cfg);
  ~RadioConfigNoRxStream();

  bool RadioStart();
  void RadioStop();
  void ReadSensors();
  void RadioTx(void** buffs);
  void RadioRx(void** buffs);
  int RadioTx(size_t radio_id, const void* const* buffs, int flags,
              long long& frameTime);
  int RadioTx(size_t radio_id,
              const std::vector<std::vector<std::complex<int16_t>>>& tx_data,
              int flags, long long& frameTime);
  int RadioRx(size_t radio_id, void** buffs, long long& rx_time_ns);
  int RadioRx(size_t radio_id,
              std::vector<std::vector<std::complex<int16_t>>>& rx_data,
              long long& rx_time_ns);
  bool DoCalib() const { return calib_; }
  void Go();
  arma::cx_float* GetCalibUl() { return init_calib_ul_processed_; }
  arma::cx_float* GetCalibDl() { return init_calib_dl_processed_; }

  // Thread functions
  void InitBsRadio(size_t tid);
  void ConfigureBsRadio(size_t tid);

 private:
  bool InitialCalib(bool /*sample_adjust*/);
  void DrainBuffers();
  void AdjustDelays(std::vector<int> /*offset*/);
  static void DciqMinimize(SoapySDR::Device* /*targetDev*/,
                           SoapySDR::Device* /*refDev*/, int /*direction*/,
                           size_t /*channel*/, double /*rxCenterTone*/,
                           double /*txCenterTone*/);
  static void SetIqBalance(SoapySDR::Device* /*dev*/, int /*direction*/,
                           size_t /*channel*/, int /*gcorr*/, int /*iqcorr*/);
  static void AdjustCalibrationGains(std::vector<SoapySDR::Device*> /*rxDevs*/,
                                     SoapySDR::Device* /*txDev*/,
                                     size_t /*channel*/, double /*fftBin*/,
                                     bool plot = false);
  static std::vector<std::complex<float>> SnoopSamples(
      SoapySDR::Device* /*dev*/, size_t /*channel*/, size_t /*readSize*/);
  void DciqCalibrationProc(size_t /*channel*/);
  Config* cfg_;
  std::vector<SoapySDR::Device*> hubs_;
  std::vector<SoapySDR::Device*> ba_stn_;
  SoapySDR::Stream* ref_rx_stream_;

  arma::cx_float* init_calib_ul_processed_;
  arma::cx_float* init_calib_dl_processed_;
  Table<arma::cx_float> init_calib_ul_;
  Table<arma::cx_float> init_calib_dl_;
  size_t radio_num_;
  size_t antenna_num_;
  bool calib_;
  size_t calib_meas_num_;

  std::atomic<size_t> num_radios_initialized_;
  std::atomic<size_t> num_radios_configured_;
};
#endif  // RADIO_LIB_MOD_H_
