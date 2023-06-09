/**
 * @file udp_server.h
 * @brief Provides the UDPServer functions from the UDPComm class.  Receiver only support
 */
#ifndef UDP_SERVER_H_
#define UDP_SERVER_H_

#include <utility>

#include "udp_comm.h"

/// Basic UDP server class based on OS sockets that supports receiving messages
class UDPServer {
 public:
  // Initialize a UDP server listening on this UDP port with socket buffer
  // size = rx_buffer_size
  explicit UDPServer(std::string local_address, uint16_t local_port,
                     size_t rx_buffer_size = 0)
      : address_(std::move(local_address)),
        port_(std::to_string(local_port)),
        comm_object_(address_, port_, rx_buffer_size, 0) {}
  explicit UDPServer(std::string local_address, std::string local_port,
                     size_t rx_buffer_size = 0)
      : address_(std::move(local_address)),
        port_(std::move(local_port)),
        comm_object_(address_, port_, rx_buffer_size, 0) {}

  UDPServer& operator=(const UDPServer&) = delete;
  UDPServer(const UDPServer&) = delete;
  ~UDPServer() = default;

  /**
   * @brief The remote_address | remote_port is the only address to which datagrams are received.
   * 1:1 association remote<-me
   *
   * @param remote_address Hostname or IP address of the remote server
   * @param remote_port UDP port of the remote server
   */
  inline ssize_t Connect(const std::string& remote_address,
                         const std::string& remote_port) {
    return comm_object_.Connect(remote_address, remote_port);
  }
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
  inline ssize_t Recv(std::byte* buf, size_t len) const {
    return comm_object_.Recv(buf, len);
  }
  inline ssize_t Recv(const std::string& src_address, uint16_t src_port,
                      std::byte* buf, size_t len) {
    return comm_object_.Recv(src_address, src_port, buf, len);
  }

  /**
   * @brief Configures the socket in blocking mode.  Any calls to recv / send
   * will now block
   */
  inline void MakeBlocking(size_t timeout_sec = 0) const {
    return comm_object_.MakeBlocking(timeout_sec);
  }

  inline const std::string& Address() const { return address_; }
  inline const std::string& Port() const { return port_; }

 private:
  /**
   * @brief The UDP port to server is listening on
   */
  const std::string address_;
  const std::string port_;
  UDPComm comm_object_;
};

#endif  // UDP_SERVER_IPV6_H_