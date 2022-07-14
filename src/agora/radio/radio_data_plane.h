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
#include "radio.h"

//Abstract class defination for the Radio data plane connection
class RadioDataPlane {
 public:
  enum DataPlaneType { kSoapyStream, kLinuxSocket };
  static std::unique_ptr<RadioDataPlane> Create(DataPlaneType type);

  enum Mode { kModeUninit, kModeShutdown, kModeDeactive, kModeActive };

  virtual ~RadioDataPlane();
  RadioDataPlane(RadioDataPlane&&) noexcept = delete;
  explicit RadioDataPlane(const RadioDataPlane&) = delete;

  virtual void Init(Radio* radio, const Config* cfg, bool hw_framer) = 0;
  virtual void Setup() = 0;
  virtual void Activate(
      Radio::ActivationTypes type = Radio::ActivationTypes::kActivate,
      long long act_time_ns = 0, size_t samples = 0) = 0;
  virtual void Deactivate() = 0;
  virtual void Close() = 0;

  virtual int Rx(std::vector<std::vector<std::complex<int16_t>>>& rx_data,
                 size_t rx_size, Radio::RxFlags& out_flags,
                 long long& rx_time_ns) = 0;
  virtual int Rx(std::vector<std::vector<std::complex<int16_t>>*>& rx_buffs,
                 size_t rx_size, Radio::RxFlags& out_flags,
                 long long& rx_time_ns) = 0;
  virtual int Rx(std::vector<void*>& rx_locations, size_t rx_size,
                 Radio::RxFlags& out_flags, long long& rx_time_ns) = 0;

  virtual void Flush() = 0;

 protected:
  RadioDataPlane();

  virtual void Setup(const SoapySDR::Kwargs& args);
  inline const Config* Configuration() const { return cfg_; }
  inline const Mode& CheckMode() const { return mode_; }
  inline const bool& HwFramer() const { return hw_framer_; }

  Radio* radio_{nullptr};
  SoapySDR::Stream* remote_stream_{nullptr};

 private:
  Mode mode_{kModeUninit};
  //Should try to remove cfg_
  const Config* cfg_;
  bool hw_framer_;
};
#endif  // RADIO_DATA_PLANE_H_