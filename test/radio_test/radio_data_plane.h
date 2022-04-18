/**
 * @file radio_data_plane.h
 * @brief Declaration file for the RadioDataPlane Class
 */
#ifndef RADIO_DATA_PLANE_H_
#define RADIO_DATA_PLANE_H_

#include <complex>
#include <vector>

#include "SoapySDR/Device.hpp"
#include "config.h"

//Abstract class defination for the Radio data plane connection
class RadioDataPlane {
 public:
  enum Mode { kModeUninit, kModeShutdown, kModeDeactive, kModeActive };

  virtual ~RadioDataPlane();
  explicit RadioDataPlane(RadioDataPlane&&) = delete;
  explicit RadioDataPlane(const RadioDataPlane&) = delete;

  virtual void Init(const Config* cfg, SoapySDR::Device* device, size_t id) = 0;
  virtual void Setup() = 0;
  virtual void Activate() = 0;
  virtual void Deactivate() = 0;
  virtual void Close() = 0;

  virtual int Rx(std::vector<std::vector<std::complex<int16_t>>>& rx_data,
                 long long& rx_time_ns) = 0;

  virtual void Flush() = 0;
  inline const size_t& Id() const { return radio_id_; }

 protected:
  virtual void Setup(const SoapySDR::Kwargs& args);

  inline const Config* Configuration() const { return cfg_; }
  inline const Mode& CheckMode() const { return mode_; }
  RadioDataPlane(const Config* cfg, SoapySDR::Device* device, size_t id);
  SoapySDR::Device* device_;
  SoapySDR::Stream* rx_stream_;

 private:
  RadioDataPlane();

  enum Mode mode_;
  size_t radio_id_;
  const Config* cfg_;
};
#endif  // RADIO_DATA_PLANE_H_
