/**
 * @file radio_data_plane_soapy.h
 * @brief Declaration file for the RadioDataPlaneSoapy Class
 */
#ifndef RADIO_DATA_PLANE_SOAPY_H_
#define RADIO_DATA_PLANE_SOAPY_H_

#include <complex>
#include <vector>

#include "SoapySDR/Device.hpp"
#include "config.h"

class RadioDataPlaneSoapy {
  enum Mode { kModeUninit, kModeShutdown, kModeDeactive, kModeActive };

 public:
  RadioDataPlaneSoapy();
  //Allow move and disallow copy
  explicit RadioDataPlaneSoapy(RadioDataPlaneSoapy&&) = default;
  explicit RadioDataPlaneSoapy(const RadioDataPlaneSoapy&) = delete;

  RadioDataPlaneSoapy(const Config* cfg, SoapySDR::Device* device, size_t id);
  ~RadioDataPlaneSoapy();

  void Init(const Config* cfg, SoapySDR::Device* device, size_t id);
  void Setup();
  void Activate();
  void Deactivate();
  void Close();

  int Rx(std::vector<std::vector<std::complex<int16_t>>>& rx_data,
         long long& rx_time_ns);

  void Flush();

 private:
  size_t radio_id_;
  enum Mode mode_;
  const Config* cfg_;
  SoapySDR::Device* device_;
  SoapySDR::Stream* rx_stream_;
};
#endif  // RADIO_DATA_PLANE_SOAPY_H_
