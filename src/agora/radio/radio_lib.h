/**
 * @file radio_lib.h
 * @brief Declaration file for the BsRadioSet class.
 */
#ifndef RADIO_LIB_H_
#define RADIO_LIB_H_

#include <atomic>
#include <complex>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <vector>

#include "SoapySDR/Device.hpp"
#include "armadillo"
#include "config.h"
#include "memory_manage.h"
#include "radio.h"
#include "radio_set.h"

class BsRadioSet : public RadioSet {
 public:
  BsRadioSet(Config* cfg, Radio::RadioType radio_type);
  virtual ~BsRadioSet() final;

  virtual bool RadioStart() final;
  virtual void Go() final;

  virtual bool DoCalib() const final { return calib_; }

  virtual arma::cx_float* GetCalibUl() final {
    return init_calib_ul_processed_;
  }
  virtual arma::cx_float* GetCalibDl() final {
    return init_calib_dl_processed_;
  }

  // Thread functions
  virtual void InitRadio(size_t radio_id) final;
  virtual void ConfigureRadio(size_t radio_id) final;

 private:
  long long SyncArrayTime();

  void CalibrateSampleOffset();
  /* Transmit from each array antenna to ref antenna,
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
  void AdjustDelays(std::vector<int> offset);
  static void DciqMinimize(Radio* target_dev, Radio* ref_dev, int direction,
                           size_t channel, double rx_center_tone,
                           double tx_center_tone);
  static void SetIqBalance(Radio* dev, int direction, size_t channel, int gcorr,
                           int iqcorr);
  static void AdjustCalibrationGains(std::vector<Radio*>& rx_devs,
                                     Radio* tx_dev, size_t channel,
                                     double fft_bin, bool plot = false);
  void DciqCalibrationProc(size_t channel);
  Config* cfg_;
  std::vector<SoapySDR::Device*> hubs_;
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
