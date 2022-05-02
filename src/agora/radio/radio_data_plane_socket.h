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
  // Allow move and disallow copy
  RadioDataPlaneSocket(RadioDataPlaneSocket&&) noexcept = default;
  explicit RadioDataPlaneSocket(const RadioDataPlaneSocket&) = delete;
  ~RadioDataPlaneSocket() final;

  void Init(Radio* radio, const Config* cfg) final;
  void Setup() final;
  void Activate() final;
  void Deactivate() final;
  void Close() final;

  int Rx(std::vector<std::vector<std::complex<int16_t>>>& rx_data,
         size_t rx_size, Radio::RxFlags rx_flags, long long& rx_time_ns) final;

  int Rx(std::vector<std::vector<std::complex<int16_t>>*>& rx_buffs,
         size_t rx_size, Radio::RxFlags rx_flags, long long& rx_time_ns) final;

  int Rx(std::vector<void*>& rx_locations, size_t rx_size,
         Radio::RxFlags rx_flags, long long& rx_time_ns) final;

  void Flush() final;

 private:
  RadioSocket socket_;
};
#endif  // RADIO_DATA_PLANE_SOCKET_H_
