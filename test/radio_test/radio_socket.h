/**
 * @file radio_socket.h
 * @brief Declaration file for the RadioSocket class.
 */
#ifndef RADIO_SOCKET_H_
#define RADIO_SOCKET_H_

#include <complex>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "udp_server_ipv6.h"

///Class to commicate with the Radios.  Including symbol parsing, packing and unpacking based on control plan MTU settings.
class RadioSocket {
 public:
  RadioSocket();
  ~RadioSocket() = default;
  //Allow move, and disallow copy
  explicit RadioSocket(RadioSocket&&) = default;
  explicit RadioSocket(const RadioSocket&) = delete;

  void Create(size_t samples_per_symbol, const std::string& local_addr,
              const std::string& remote_addr, const std::string& local_port,
              const std::string& remote_port);
  inline const std::string& Address() const { return socket_->Address(); }
  inline const std::string& Port() const { return socket_->Port(); };

  int RxSymbol(std::vector<std::vector<std::complex<int16_t>>>& out_data,
               long long& rx_time_ns);
  void Flush();

 private:
  bool CheckSymbolComplete(const std::byte* in_data, const int& in_count);
  size_t ParseRxSymbol(
      std::vector<std::vector<std::complex<int16_t>>>& out_samples,
      long long& rx_time_ns);

  std::unique_ptr<UDPServerIPv6> socket_;
  std::vector<std::byte> rx_buffer_;
  size_t rx_bytes_;
  size_t rx_samples_;

  size_t samples_per_symbol_;
  const size_t bytes_per_element_;
};
#endif  // RADIO_SOCKET_H_
