/**
 * @file radio_data_plane_socket.h
 * @brief Declaration file for the RadioDataPlaneSocket Class
 */
#ifndef RADIO_DATA_PLANE_SOCKET_H_
#define RADIO_DATA_PLANE_SOCKET_H_

#include <complex>
#include <vector>

#include "SoapySDR/Device.hpp"
#include "config.h"
#include "radio_socket.h"

class RadioDataPlaneSocket {
  enum Mode { kModeUninit, kModeShutdown, kModeDeactive, kModeActive };

 public:
  RadioDataPlaneSocket();
  //Allow move and disallow copy
  explicit RadioDataPlaneSocket(RadioDataPlaneSocket&&) = default;
  explicit RadioDataPlaneSocket(const RadioDataPlaneSocket&) = delete;

  RadioDataPlaneSocket(const Config* cfg, SoapySDR::Device* device, size_t id);
  ~RadioDataPlaneSocket();

  void Init(const Config* cfg, SoapySDR::Device* device, size_t id);
  void Setup();
  void Activate();
  void Deactivate();
  void Close();

  int Rx(std::vector<std::vector<std::complex<int16_t>>>& rx_data,
         long long& rx_time_ns);

  void Flush();

 private:
  size_t radio_id_;
  enum Mode mode_;
  const Config* cfg_;
  SoapySDR::Device* device_;
  SoapySDR::Stream* rx_stream_;
  RadioSocket socket_;
};
#endif  // RADIO_DATA_PLANE_SOCKET_H_
