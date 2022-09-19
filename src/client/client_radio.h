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

class ClientRadioConfig {
 public:
  ClientRadioConfig(const Config* const cfg, Radio::RadioType radio_type);
  ~ClientRadioConfig();

  bool RadioStart();
  void RadioStop();
  void ReadSensors();
  int RadioTx(size_t radio_id, void** buffs, size_t num_samps,
              Radio::TxFlags flags, long long& tx_time);

  int RadioRx(size_t radio_id,
              std::vector<std::vector<std::complex<int16_t>>>& rx_data,
              size_t rx_size, Radio::RxFlags& out_flags, long long& rx_time_ns);

  int RadioRx(size_t radio_id,
              std::vector<std::vector<std::complex<int16_t>>*>& rx_buffs,
              size_t rx_size, Radio::RxFlags& out_flags, long long& rx_time_ns);

  int RadioRx(size_t radio_id, std::vector<void*>& rx_locs, size_t rx_size,
              Radio::RxFlags& out_flags, long long& rx_time_ns);

  void Go() const;

 private:
  void InitClientRadio(size_t radio_id);

  const Config* const cfg_;
  std::vector<std::unique_ptr<Radio>> radios_;
  size_t total_radios_;
  size_t total_antennas_;

  std::atomic<size_t> num_client_radios_initialized_;
};
#endif  // CLIENT_RADIO_LIB_H_
