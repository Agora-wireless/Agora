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
  explicit ClientRadioConfig(const Config* const cfg);
  ~ClientRadioConfig();

  bool RadioStart();
  void RadioStop();
  void ReadSensors();
  int RadioTx(size_t radio_id, void** buffs, size_t num_samps, int flags,
              long long& tx_time);
  int RadioRx(size_t radio_id, void** buffs, size_t num_samps,
              long long& rx_time);
  void Go();

 private:
  void InitClientRadio(size_t radio_id);

  const Config* const cfg_;
  std::vector<std::unique_ptr<Radio>> radios_;
  size_t total_radios_;
  size_t total_antennas_;

  std::atomic<size_t> num_client_radios_initialized_;
};
#endif  // CLIENT_RADIO_LIB_H_
