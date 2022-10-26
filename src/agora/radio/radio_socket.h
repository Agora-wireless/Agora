/**
 * @file radio_socket.h
 * @brief Declaration file for the RadioSocket class.
 */
#ifndef RADIO_SOCKET_H_
#define RADIO_SOCKET_H_

#include <complex>
#include <cstddef>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "udp_server.h"

///Class to commicate with the Radios.  Including symbol parsing, packing and unpacking based on control plan MTU settings.
class RadioSocket {
 public:
  RadioSocket();
  ~RadioSocket();
  //Allow move, and disallow copy
  RadioSocket(RadioSocket&&) noexcept = default;
  explicit RadioSocket(const RadioSocket&) = delete;

  void Create(size_t samples_per_symbol, const std::string& local_addr,
              const std::string& remote_addr, const std::string& local_port,
              const std::string& remote_port);
  inline const std::string& Address() const { return socket_->Address(); }
  inline const std::string& Port() const { return socket_->Port(); };

  int RxSamples(std::vector<void*>& out_data, long long& rx_time_ns,
                size_t req_samples_per_channel);
  void Flush();

 private:
  bool CheckSymbolComplete(const std::byte* in_data, const int& in_count);
  size_t InspectRx(const std::byte* in_data, size_t in_count,
                   long long& rx_time_ticks, size_t& burst_count) const;
  size_t UnpackSamples(std::vector<void*>& out_samples, size_t req_samples,
                       long long& rx_time);

  size_t LoadSamples(std::vector<void*>& out_samples,
                     const std::complex<int16_t>* in_samples,
                     size_t num_in_samples);

  size_t GetUnpackedSamples(std::vector<void*>& out_samples, long long& rx_time,
                            size_t req_total_samples);

  size_t GetPackedSamples(std::vector<void*>& out_samples, long long& rx_time,
                          size_t sample_offset, size_t req_samples);

  std::unique_ptr<UDPServer> socket_;
  std::vector<std::byte> rx_buffer_;
  std::queue<size_t> rx_pkt_byte_count_;

  //Buffer to place "extra" samples that come during a socket read
  std::vector<std::complex<int16_t>> sample_buffer_;
  long long rx_time_unpacked_{0};

  size_t rx_bytes_{0};
  size_t rx_samples_{0};

  size_t samples_per_symbol_{1};
  const size_t bytes_per_element_{3u};
};
#endif  // RADIO_SOCKET_H_
