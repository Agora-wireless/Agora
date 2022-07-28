/**
 * @file udp_server.h
 * @brief Declaration file for the UDPServer class
 */
#ifndef UDP_SERVER_H_
#define UDP_SERVER_H_

#include <netdb.h>

#include <cstdint>
#include <map>
#include <mutex>
#include <string>

/// Basic UDP server class based on OS sockets that supports receiving messages
class UDPServer {
 public:
  static const bool kDebugPrintUdpServerInit = true;

  // Initialize a UDP server listening on this UDP port with socket buffer
  // size = rx_buffer_size
  explicit UDPServer(uint16_t port, size_t rx_buffer_size = 0);

  UDPServer& operator=(const UDPServer&) = delete;
  UDPServer(const UDPServer&) = delete;
  ~UDPServer();

  /**
   * @brief Try to receive up to len bytes in buf by default this will not block
   *
   * @return Return the number of bytes received if non-zero bytes are
   * received. If no bytes are received, return zero. If there was an error
   * in receiving, return -1.
   */
  ssize_t Recv(uint8_t* buf, size_t len) const;

  /**
   * @brief Try once to receive up to len bytes in buf
   *
   * @return Return the number of bytes received if non-zero bytes are
   * received. If no bytes are received, return zero. If there was an error
   * in receiving, return -1.
   */
  ssize_t RecvFrom(uint8_t* buf, size_t len, const std::string& src_address,
                   uint16_t src_port);

  /**
   * @brief Configures the socket in blocking mode.  Any calls to recv / send
   * will now block
   */
  void MakeBlocking(size_t timeout_sec = 0) const;

 private:
  /**
   * @brief The UDP port to server is listening on
   */
  uint16_t port_;
  /**
   * @brief The raw socket file descriptor
   */
  int sock_fd_ = -1;

  /**
   * @brief A cache mapping hostname:udp_port to addrinfo
   */
  std::map<std::string, addrinfo*> addrinfo_map_;
  /**
   * @brief Variable to control write access to the non-thread safe data
   * structures
   */
  std::mutex map_insert_access_;
};

#endif  // UDP_SERVER_H_