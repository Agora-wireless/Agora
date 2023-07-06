/**
 * @file radio_set_uhd.h
 * @brief Declaration file for the RadioSetUhd class.
 */
#ifndef RADIO_SET_UHD_H_
#define RADIO_SET_UHD_H_

#include <complex>
#include <cstdlib>
#include <vector>

#include "armadillo"
#include "config.h"
#include "memory_manage.h"
#include "radio.h"
#include "radio_set.h"
#include "uhd/usrp/multi_usrp.hpp"

class RadioSetUhd : public RadioSet {
 public:
  RadioSetUhd(Config* cfg, Radio::RadioType radio_type);
  virtual ~RadioSetUhd() final = default;

  virtual bool RadioStart() final;
  virtual void Go() final;

 private:
  void InitRadio(size_t radio_id);
  void ConfigureRadio(size_t radio_id);

  Config* cfg_;

  size_t radio_num_;
  size_t antenna_num_;

  std::atomic<size_t> num_radios_initialized_;
  std::atomic<size_t> num_radios_configured_;
};
#endif  // RADIO_SET_UHD_H_
