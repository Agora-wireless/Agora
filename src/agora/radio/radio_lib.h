/**
 * @file radio_lib.h
 * @brief Declaration file for the RadioConfig class.
 */
#ifndef RADIO_LIB_H_
#define RADIO_LIB_H_

#include <complex>
#include <cstdlib>
#include <vector>

#include "SoapySDR/Device.hpp"
#include "armadillo"
#include "config.h"
#include "memory_manage.h"

class RadioConfig {
 public:
  explicit RadioConfig(Config* cfg);
  ~RadioConfig();

  bool RadioStart();
  void RadioStop();
  void ReadSensors();
  int RadioTx(size_t radio_id, const void* const* buffs, int flags,
              long long& tx_time);
  int RadioTx(size_t radio_id,
              const std::vector<std::vector<std::complex<int16_t>>>& tx_data,
              int flags, long long& tx_time);
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
  void CalibrateSampleOffset();
  /* Transmit from each array atenna to ref antenna,
   * return the received signal vector at the ref antenna
  */
  auto TxArrayToRef(const std::vector<std::complex<int16_t>>& tx_vec);
  /* Transmit from ref antenna to the rest of the array
  * return the received signal vector at the array
  */
  auto TxRefToArray(const std::vector<std::complex<int16_t>>& tx_vec);
  bool FindTimeOffset(
      const std::vector<std::vector<std::complex<int16_t>>>& rx_mat,
      std::vector<int>& offset);
  bool InitialCalib();
  static void DrainRxBuffer(SoapySDR::Device* ibs_sdrs,
                            SoapySDR::Stream* istream, std::vector<void*> buffs,
                            size_t sym_samp);
  void DrainBuffers();
  void AdjustDelays(std::vector<int> offset);
  static void DciqMinimize(SoapySDR::Device* target_dev,
                           SoapySDR::Device* ref_dev, int direction,
                           size_t channel, double rx_center_tone,
                           double tx_center_tone);
  static void SetIqBalance(SoapySDR::Device* dev, int direction, size_t channel,
                           int gcorr, int iqcorr);
  static void AdjustCalibrationGains(std::vector<SoapySDR::Device*>& rx_devs,
                                     SoapySDR::Device* tx_dev, size_t channel,
                                     double fft_bin, bool plot = false);
  static std::vector<std::complex<float>> SnoopSamples(SoapySDR::Device* dev,
                                                       size_t channel,
                                                       size_t read_size);
  void DciqCalibrationProc(size_t channel);
  Config* cfg_;
  std::vector<SoapySDR::Device*> hubs_;
  std::vector<SoapySDR::Device*> ba_stn_;
  std::vector<SoapySDR::Stream*> tx_streams_;
  std::vector<SoapySDR::Stream*> rx_streams_;
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
#endif  // RADIO_LIB_H_
