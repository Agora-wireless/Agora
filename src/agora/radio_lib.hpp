#ifndef RADIO_LIB
#define RADIO_LIB

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

#include "config.hpp"

class RadioConfig {
 public:
  RadioConfig(Config* cfg);
  bool radioStart();
  void radioStop();
  void readSensors();
  void radioTx(void** buffs);
  void radioRx(void** buffs);
  int radioTx(size_t, void** buffs, int flags, long long& frameTime);
  int radioRx(size_t, void** buffs, long long& frameTime);
  bool doCalib() { return calib_; }
  void go();
  arma::cx_float* get_calib_ul() { return init_calib_ul_; }
  arma::cx_float* get_calib_dl() { return init_calib_dl_; }
  ~RadioConfig();

 private:
  struct RadioConfigContext {
    RadioConfig* brs_;
    size_t tid_;
  };
  static void* initBSRadio_launch(void* in_context);
  static void* configureBSRadio_launch(void* in_context);
  void initBSRadio(RadioConfigContext* context);
  void configureBSRadio(RadioConfigContext* context);
  bool initial_calib(bool);
  static void drain_rx_buffer(SoapySDR::Device* ibsSdrs,
                              SoapySDR::Stream* istream,
                              std::vector<void*> buffs, size_t symSamp);
  void drain_buffers();
  void adjustDelays(std::vector<int>);
  static void dciqMinimize(SoapySDR::Device*, SoapySDR::Device*, int, size_t,
                           double, double);
  static void setIQBalance(SoapySDR::Device*, int, size_t, int, int);
  static void adjustCalibrationGains(std::vector<SoapySDR::Device*>,
                                     SoapySDR::Device*, size_t, double,
                                     bool plot = false);
  static std::vector<std::complex<float>> snoopSamples(SoapySDR::Device*,
                                                       size_t, size_t);
  void dciqCalibrationProc(size_t);
  Config* cfg_;
  std::vector<SoapySDR::Device*> hubs_;
  std::vector<SoapySDR::Device*> ba_stn_;
  SoapySDR::Stream* ref_rx_stream_;
  std::vector<SoapySDR::Stream*> tx_streams_;
  std::vector<SoapySDR::Stream*> rx_streams_;
  arma::cx_float* init_calib_ul_;
  arma::cx_float* init_calib_dl_;
  size_t radio_num_;
  size_t antenna_num_;
  bool is_ue_;
  bool calib_;
};
#endif
