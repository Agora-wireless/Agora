/**
 * @file radio_set_bs.h
 * @brief Declaration file for the RadioSetBs class.
 */
#ifndef RADIO_SET_BS_H_
#define RADIO_SET_BS_H_

#include <atomic>
#include <complex>
#include <cstdint>
#include <cstdlib>
#include <vector>

#include "SoapySDR/Device.hpp"
#include "armadillo"
#include "config.h"
#include "memory_manage.h"
#include "radio.h"
#include "radio_set.h"

class RadioSetBs : public RadioSet {
 public:
  RadioSetBs(Config* cfg, Radio::RadioType radio_type);
  virtual ~RadioSetBs() final;

  virtual bool RadioStart() final;
  virtual void Go() final;

  virtual bool DoCalib() const final { return calib_; }

 private:
  // Thread functions
  void InitRadio(size_t radio_id);

  long long SyncArrayTime();

  void AdjustDelays();
  void ApplyCalib();
  Config* cfg_;
  std::vector<SoapySDR::Device*> hubs_;
  size_t radio_num_;
  size_t antenna_num_;
  bool calib_;
  std::vector<int> trigger_offsets_;

  std::atomic<size_t> num_radios_initialized_;
  std::atomic<size_t> num_radios_configured_;
};
#endif  // RADIO_SET_BS_H_
