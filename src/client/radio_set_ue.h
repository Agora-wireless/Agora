/**
 * @file radio_set_ue.h
 * @brief Declaration file for the RadioSetUe class
 */
#ifndef RADIO_SET_UE_H_
#define RADIO_SET_UE_H_

#include <atomic>
#include <cstdlib>
#include <vector>

#include "config.h"
#include "radio.h"
#include "radio_set.h"

class RadioSetUe : public RadioSet {
 public:
  RadioSetUe(const Config* const cfg, Radio::RadioType radio_type);
  ~RadioSetUe() final = default;

  bool RadioStart() final;
  void Go() final;

  bool DoCalib() const final { return false; }
  arma::cx_float* GetCalibUl() final { return nullptr; }
  arma::cx_float* GetCalibDl() final { return nullptr; }

 private:
  // Thread functions used in constructor
  void InitRadio(size_t radio_id);
  void ConfigureRadio(size_t radio_id);

  const Config* const cfg_;
  size_t total_radios_;
  size_t total_antennas_;

  std::atomic<size_t> num_client_radios_initialized_;
};
#endif  // RADIO_SET_UE_H_
