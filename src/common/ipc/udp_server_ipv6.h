/**
 * @file udp_server_ipv6.h
 * @brief Declaration file for the UDPServerIPv6 class
 */
#ifndef UDP_SERVER_IPV6_H_
#define UDP_SERVER_IPV6_H_

#include <netdb.h>

#include <cstddef>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>

/// Basic UDP server class based on OS sockets that supports receiving messages
class UDPServerIPv6 {
 public:
  static constexpr bool kDebugPrintUdpServerInit = false;

  // Initialize a UDP server listening on this UDP port with socket buffer
  // size = rx_buffer_size
  explicit UDPServerIPv6(const std::string& local_address, uint16_t local_port,
                         size_t rx_buffer_size = 0);
  explicit UDPServerIPv6(std::string local_address, std::string local_port,
                         size_t rx_buffer_size = 0);

  UDPServerIPv6& operator=(const UDPServerIPv6&) = delete;
  UDPServerIPv6(const UDPServerIPv6&) = delete;
  ~UDPServerIPv6();

  /**
   * @brief The remote_address | remote_port is the only address to which datagrams are received.
   * 1:1 association remote<-me
   *
   * @param remote_address Hostname or IP address of the remote server
   * @param remote_port UDP port of the remote server
   */
  ssize_t Connect(const std::string& remote_address,
                  const std::string& remote_port);
  ssize_t Connect(const std::string& remote_address, uint16_t remote_port);

  /**
   * @brief Try to receive up to len bytes in buf by default this will not block
   *
   * @return Return the number of bytes received if non-zero bytes are
   * received. If no bytes are received, return zero. If there was an error
   * in receiving, return -1.
   */
  ssize_t Recv(std::byte* buf, size_t len) const;

  ssize_t RecvFrom(std::byte* buf, size_t len, const std::string& src_address,
                   uint16_t src_port);

  /**
   * @brief Configures the socket in blocking mode.  Any calls to recv / send
   * will now block
   */
  void MakeBlocking(size_t timeout_sec = 0) const;

  inline const std::string& Address() const { return address_; }
  inline const std::string& Port() const { return port_; }

 private:
  /**
   * @brief The UDP port to server is listening on
   */
  const std::string port_;
  const std::string address_;
  /**
   * @brief The raw socket file descriptor
   */
  int sock_fd_ = -1;

  /**
   * @brief A cache mapping hostname:udp_port to addrinfo
   * Used in RecvFrom
   */
  std::map<std::string, addrinfo*> addrinfo_map_;
  /**
   * @brief Variable to control write access to the non-thread safe data
   * structures
   */
  std::mutex map_insert_access_;
};

#endif  // UDP_SERVER_IPV6_H_