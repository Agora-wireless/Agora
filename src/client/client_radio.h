/**
 * @file client_radio.h
 * @brief Declaration file for the client radio config class
 */
#ifndef CLIENT_RADIO_LIB_H_
#define CLIENT_RADIO_LIB_H_

#include <atomic>
#include <cstdlib>
#include <vector>

#include "SoapySDR/Device.hpp"
#include "config.h"

class ClientRadioConfig {
 public:
  explicit ClientRadioConfig(const Config* const cfg);
  bool RadioStart();
  void RadioStop();
  void ReadSensors();
  int RadioTx(size_t /*r*/, void** buffs, size_t num_samps, int flags,
              long long& frameTime);
  int RadioRx(size_t /*r*/, void** buffs, size_t num_samps,
              long long& frameTime);
  static void DrainRxBuffer(SoapySDR::Device* dev, SoapySDR::Stream* istream,
                            std::vector<void*> buffs, size_t symSamp);
  void DrainBuffers();
  void Go();
  int Triggers(int i);
  ~ClientRadioConfig();

 private:
  void InitClientRadio(size_t tid);

  const Config* const cfg_;
  std::vector<SoapySDR::Device*> cl_stn_;
  std::vector<SoapySDR::Stream*> tx_streams_;
  std::vector<SoapySDR::Stream*> rx_streams_;
  size_t radio_num_;
  size_t antenna_num_;

  std::atomic<size_t> num_client_radios_initialized_;
};
#endif  // CLIENT_RADIO_LIB_H_
