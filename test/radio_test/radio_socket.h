/**
 * @file radio_socket.h
 * @brief Declaration file for the RadioSocket class.
 */
#ifndef RADIO_SOCKET_H_
#define RADIO_SOCKET_H_

#include <complex>
#include <cstddef>
#include <vector>

#include "SoapyRPCSocket.hpp"

class RadioSocket {
 public:
  RadioSocket(size_t samples_per_symbol);
  ~RadioSocket() = default;

  void Create(const std::string& address, const std::string& port);
  inline const std::string& GetAddress() const { return address_; }
  inline const std::string& GetPort() const { return port_; };

  int RxSymbol(std::vector<std::vector<std::complex<int16_t>>>& out_data,
               long long& rx_time_ns);

 private:
  bool CheckSymbolComplete(const size_t& bytes);
  size_t ParseRxSymbol(
      std::vector<std::vector<std::complex<int16_t>>>& out_samples,
      long long& rx_time_ns);

  sklk_SoapyRPCSocket socket_;
  std::vector<std::byte> rx_buffer_;
  size_t rx_bytes_;
  size_t rx_samples_;

  std::string address_;
  std::string port_;

  const size_t samples_per_symbol_;
  const size_t bytes_per_element_;
};
#endif  // RADIO_SOCKET_H_
