/**
 * @file radio_lib_mod.h
 * @brief Declaration file for the RadioConfigNoRxStream class.
 */
#ifndef RADIO_LIB_MOD_H_
#define RADIO_LIB_MOD_H_

#include <atomic>
#include <complex>
#include <vector>

#include "SoapySDR/Device.hpp"
#include "config.h"
#include "radio_data_plane.h"

class RadioConfigNoRxStream {
 public:
  explicit RadioConfigNoRxStream(Config* cfg);
  ~RadioConfigNoRxStream();

  bool RadioStart();
  void RadioStop();
  void ReadSensors();
  int RadioTx(size_t radio_id, const void* const* buffs, int flags,
              long long& frameTime);
  int RadioTx(size_t radio_id,
              const std::vector<std::vector<std::complex<int16_t>>>& tx_data,
              int flags, long long& frameTime);
  int RadioRx(size_t radio_id,
              std::vector<std::vector<std::complex<int16_t>>>& rx_data,
              long long& rx_time_ns);
  void Go();

  // Thread functions
  void InitBsRadio(size_t radio);
  void ConfigureBsRadio(size_t radio);

 private:
  void DrainBuffers();
  bool ParseRxStream(const char* raw_rx_data, size_t num_rx_bytes,
                     void** samples_out, size_t& sample_offset,
                     long long& rx_time_ns);

  Config* cfg_;
  std::vector<SoapySDR::Device*> hubs_;
  std::vector<SoapySDR::Device*> ba_stn_;

  size_t radio_num_;
  size_t antenna_num_;
  bool calib_;
  size_t calib_meas_num_;

  std::atomic<size_t> num_radios_initialized_;
  std::atomic<size_t> num_radios_configured_;

  std::vector<RadioDataPlane> rx_data_plane_;
  std::vector<SoapySDR::Stream*> tx_streams_;
};
#endif  // RADIO_LIB_MOD_H_
