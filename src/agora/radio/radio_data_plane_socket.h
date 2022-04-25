/**
 * @file radio_data_plane_socket.h
 * @brief Declaration file for the RadioDataPlaneSocket Class
 */
#ifndef RADIO_DATA_PLANE_SOCKET_H_
#define RADIO_DATA_PLANE_SOCKET_H_

#include "radio_data_plane.h"
#include "radio_socket.h"

class RadioDataPlaneSocket : public RadioDataPlane {
 public:
  RadioDataPlaneSocket();
  //Allow move and disallow copy
  explicit RadioDataPlaneSocket(RadioDataPlaneSocket&&) = default;
  explicit RadioDataPlaneSocket(const RadioDataPlaneSocket&) = delete;

  RadioDataPlaneSocket(const Config* cfg, SoapySDR::Device* device, size_t id);
  ~RadioDataPlaneSocket() final;

  void Init(const Config* cfg, SoapySDR::Device* device, size_t id) final;
  void Setup() final;
  void Activate() final;
  void Deactivate() final;
  void Close() final;

  int Rx(std::vector<std::vector<std::complex<int16_t>>>& rx_data,
         long long& rx_time_ns) final;

  void Flush() final;

 private:
  RadioSocket socket_;
};
#endif  // RADIO_DATA_PLANE_SOCKET_H_
