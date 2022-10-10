/**
 * @file client_radio.h
 * @brief Declaration file for the client radio config class
 */
#ifndef CLIENT_RADIO_LIB_H_
#define CLIENT_RADIO_LIB_H_

#include <atomic>
#include <cstdlib>
#include <vector>

#include "config.h"
#include "radio.h"
#include "radio_set.h"

class UeRadioSet : public RadioSet {
 public:
  UeRadioSet(const Config* const cfg, Radio::RadioType radio_type);
  virtual ~UeRadioSet() final = default;

  virtual bool RadioStart() final;
  virtual void Go() final;

  virtual bool DoCalib() const final { return false; }
  virtual arma::cx_float* GetCalibUl() final { return nullptr; }
  virtual arma::cx_float* GetCalibDl() final { return nullptr; }

  // Thread functions
  virtual void InitRadio(size_t radio_id) final;
  virtual void ConfigureRadio(size_t radio_id) final {}

 private:
  const Config* const cfg_;
  size_t total_radios_;
  size_t total_antennas_;

  std::atomic<size_t> num_client_radios_initialized_;
};
#endif  // CLIENT_RADIO_LIB_H_
