/**
 * @file radio_set.h
 * @brief Declaration file for the RadioSet class.  Holds a set of Radios.
 */
#ifndef RADIO_SET_H_
#define RADIO_SET_H_

#include <complex>
#include <cstdint>
#include <memory>
#include <vector>

#include "armadillo"
#include "radio.h"

class RadioSet {
 public:
  RadioSet() {};
  explicit RadioSet(size_t samples_per_symbol);
  virtual ~RadioSet() = 0;

  virtual void Go() = 0;

  virtual bool RadioStart() = 0;
  virtual void RadioStop();
  virtual void ReadSensors();

  int RadioTx(size_t radio_id, const void* const* buffs, size_t num_samps,
              Radio::TxFlags flags, long long& tx_time);
  int RadioTx(size_t radio_id, const void* const* buffs, Radio::TxFlags flags,
              long long& tx_time);
  int RadioTx(size_t radio_id,
              const std::vector<std::vector<std::complex<int16_t>>>& tx_data,
              Radio::TxFlags flags, long long& tx_time_ns);
  int RadioRx(size_t radio_id,
              std::vector<std::vector<std::complex<int16_t>>>& rx_data,
              size_t rx_size, Radio::RxFlags& out_flags, long long& rx_time_ns);

  int RadioRx(size_t radio_id,
              std::vector<std::vector<std::complex<int16_t>>*>& rx_buffs,
              size_t rx_size, Radio::RxFlags& out_flags, long long& rx_time_ns);

  int RadioRx(size_t radio_id, std::vector<void*>& rx_locs, size_t rx_size,
              Radio::RxFlags& out_flags, long long& rx_time_ns);

  virtual bool DoCalib() const { return false; };
  virtual arma::cx_float* GetCalibUl() { return nullptr; }
  virtual arma::cx_float* GetCalibDl() { return nullptr; }

 protected:
  bool RadioStart(Radio::ActivationTypes start_type);

  std::vector<std::unique_ptr<Radio>> radios_;
  size_t samples_per_symbol_;
};
#endif  // RADIO_SET_H_
