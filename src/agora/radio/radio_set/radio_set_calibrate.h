/**
 * @file radio_set_calibrate.h
 * @brief Declaration file for the RadioSetCalibrate class.
 */
#ifndef RADIO_SET_CALIBRATE_H_
#define RADIO_SET_CALIBRATE_H_

#include <atomic>
#include <complex>
#include <cstdint>
#include <cstdlib>
#include <vector>

#include "SoapySDR/Device.hpp"
#include "armadillo"
//#include "mat_logger.h"
#include "memory_manage.h"
#include "radio.h"

class RadioSetCalibrate {
 public:
  RadioSetCalibrate(Config* cfg, std::string calibration_type);
  virtual ~RadioSetCalibrate() final;

  void CalibrateSampleOffset();
  void DciqCalibrationProc(size_t channel);
  void WriteAnalogCalibData();
  void ReciprocityCalib();
  virtual arma::cx_float* GetCalibUl() final {
    return init_calib_ul_processed_;
  }
  virtual arma::cx_float* GetCalibDl() final {
    return init_calib_dl_processed_;
  }

 private:
  // Thread functions
  void InitRadio(size_t radio_id);

  // digital
  bool FindTimeOffset(
      const std::vector<std::vector<std::complex<int16_t>>>& rx_mat,
      std::vector<int>& offset);
  bool CalibrateSampleOffsetUplink(size_t max_attempts);
  bool CalibrateSampleOffsetDownlink(size_t max_attempts);
  bool InitialCalib();
  bool DoReciprocityCalib();
  void Go();

  /* Transmit from each array antenna to ref antenna,
   * return the received signal vector at the ref antenna
  */
  auto TxArrayToRef(const std::vector<std::complex<int16_t>>& tx_vec);
  /* Transmit from ref antenna to the rest of the array
  * return the received signal vector at the array
  */
  auto TxRefToArray(const std::vector<std::complex<int16_t>>& tx_vec);

  // analog
  static std::complex<double> FindArgMinDC(Radio* target_dev, Radio* ref_dev,
                                           int direction, size_t channel,
                                           double rx_center_tone,
                                           double tx_center_tone);
  static std::pair<int, int> FindArgMinIQ(Radio* target_dev, Radio* ref_dev,
                                          int direction, size_t channel,
                                          double rx_center_tone,
                                          double tx_center_tone);
  static void SetIqBalance(Radio* dev, int direction, size_t channel, int gcorr,
                           int iqcorr);
  /* Increase tx gain and then rx gain until good signal received
   */
  static void AdjustCalibrationGains(std::vector<Radio*>& rx_devs,
                                     Radio* tx_dev, size_t channel,
                                     double fft_bin, bool plot = false);

  Config* cfg_;
  std::vector<SoapySDR::Device*> hubs_;
  arma::cx_float* init_calib_ul_processed_;
  arma::cx_float* init_calib_dl_processed_;
  Table<arma::cx_float> init_calib_ul_;
  Table<arma::cx_float> init_calib_dl_;
  size_t calib_meas_num_;
  size_t radio_num_;
  size_t antenna_num_;
  std::string calibration_type_;
  std::atomic<size_t> num_radios_initialized_;
  std::array<std::vector<std::pair<int, int>>, kMaxChannels> best_rx_iq_sets_;
  std::array<std::vector<std::complex<double>>, kMaxChannels> best_rx_dc_sets_;
  std::array<std::vector<std::pair<int, int>>, kMaxChannels> best_tx_iq_sets_;
  std::array<std::vector<std::complex<double>>, kMaxChannels> best_tx_dc_sets_;
  //CsvLog::CsvLogger logger_offset_;
 protected:
  std::vector<std::unique_ptr<Radio>> radios_;
};
#endif  // RADIO_SET_CALIBRATE_H_
