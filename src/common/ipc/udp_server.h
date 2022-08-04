/**
 * @file udp_server.h
 * @brief Provides the UDPServer functions from the UDPComm class.  Receiver only support
 */
#ifndef UDP_SERVER_H_
#define UDP_SERVER_H_

#include "udp_comm.h"

class UDPServer {
 public:
  // Initialize a UDP server listening on this UDP port with socket buffer
  // size = rx_buffer_size
  explicit UDPServer(uint16_t port, size_t rx_buffer_size = 0)
      : comm_object_(std::string(), port, rx_buffer_size, 0) {}

  UDPServer& operator=(const UDPServer&) = delete;
  UDPServer(const UDPServer&) = delete;
  ~UDPServer() = default;

  inline ssize_t Connect(const std::string& remote_address,
                         uint16_t remote_port) {
    return comm_object_.Connect(remote_address, remote_port);
  }

  /**
   * @brief Try to receive up to len bytes in buf by default this will not block
   *
   * @return Return the number of bytes received if non-zero bytes are
   * received. If no bytes are received, return zero. If there was an error
   * in receiving, return -1.
   */
  inline ssize_t Recv(uint8_t* buf, size_t len) const {
    return comm_object_.Recv(reinterpret_cast<std::byte*>(buf), len);
  }

  /**
   * @brief Try once to receive up to len bytes in buf
   *
   * @return Return the number of bytes received if non-zero bytes are
   * received. If no bytes are received, return zero. If there was an error
   * in receiving, return -1.
   */
  inline ssize_t RecvFrom(uint8_t* buf, size_t len,
                          const std::string& src_address, uint16_t src_port) {
    return comm_object_.Recv(src_address, src_port,
                             reinterpret_cast<std::byte*>(buf), len);
  }

  /**
   * @brief Configures the socket in blocking mode.  Any calls to recv / send
   * will now block
   */
  inline void MakeBlocking(size_t timeout_sec = 0) const {
    return comm_object_.MakeBlocking(timeout_sec);
  }

 private:
  UDPComm comm_object_;
};

#endif  // UDP_SERVER_H_