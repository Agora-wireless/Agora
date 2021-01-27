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
  explicit RadioConfig(Config* cfg);
  bool RadioStart();
  void RadioStop();
  void ReadSensors();
  void RadioTx(void** buffs);
  void RadioRx(void** buffs);
  int RadioTx(size_t, void** buffs, int flags, long long& frameTime);
  int RadioRx(size_t, void** buffs, long long& frameTime);
  bool DoCalib() { return calib_; }
  void Go();
  arma::cx_float* GetCalibUl() { return init_calib_ul_; }
  arma::cx_float* GetCalibDl() { return init_calib_dl_; }
  ~RadioConfig();

 private:
  struct RadioConfigContext {
    RadioConfig* brs_;
    size_t tid_;
  };
  static void* InitBsRadioLaunch(void* in_context);
  static void* ConfigureBsRadioLaunch(void* in_context);
  void InitBsRadio(RadioConfigContext* context);
  void ConfigureBsRadio(RadioConfigContext* context);
  bool InitialCalib(bool);
  static void DrainRxBuffer(SoapySDR::Device* ibsSdrs,
                            SoapySDR::Stream* istream, std::vector<void*> buffs,
                            size_t symSamp);
  void DrainBuffers();
  void AdjustDelays(std::vector<int>);
  static void DciqMinimize(SoapySDR::Device*, SoapySDR::Device*, int, size_t,
                           double, double);
  static void SetIqBalance(SoapySDR::Device*, int, size_t, int, int);
  static void AdjustCalibrationGains(std::vector<SoapySDR::Device*>,
                                     SoapySDR::Device*, size_t, double,
                                     bool plot = false);
  static std::vector<std::complex<float>> SnoopSamples(SoapySDR::Device*,
                                                       size_t, size_t);
  void DciqCalibrationProc(size_t);
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
