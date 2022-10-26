/**
 * @file udp_client.h
 * @brief Provides the UDPClient functions from the UDPComm class.  Sender only support
 */
#ifndef UDP_CLIENT_H_
#define UDP_CLIENT_H_

#include <utility>

#include "udp_comm.h"

// Basic UDP client class based on OS sockets that supports sending messages
class UDPClient {
 public:
  explicit UDPClient(const std::string& src_addr, uint16_t src_port,
                     size_t tx_buffer_size = 0)
      : comm_object_(src_addr, src_port, 0, tx_buffer_size) {}
  explicit UDPClient(uint16_t src_port)
      : comm_object_(std::string(), src_port, 0, 0) {}
  explicit UDPClient(const std::string& src_addr)
      : comm_object_(src_addr, 0, 0, 0) {}

  UDPClient& operator=(const UDPClient&) = delete;
  UDPClient(const UDPClient&) = delete;
  ~UDPClient() = default;

  /**
   * @brief The remote_address | remote_port is the address to which datagrams are sent.
   * 1:1 association me->remote
   *
   * @param remote_address Hostname or IP address of the remote server
   * @param remote_port UDP port that the remote server is listening on
   */
  inline ssize_t Connect(const std::string& remote_address,
                         uint16_t remote_port) {
    return comm_object_.Connect(remote_address, remote_port);
  };

  /**
   * @brief Send one UDP packet to a remote server. The client caches the
   * the remote server's addrinfo after resolving it for the first time. After
   * the first time, sending data does not require expensive addrinfo
   * resolution.
   *
   * @param rem_hostname Hostname or IP address of the remote server
   * @param rem_port UDP port that the remote server is listening on
   * @param msg Pointer to the message to send
   * @param len Length in bytes of the message to send
   */
  inline void Send(const std::string& rem_hostname, uint16_t rem_port,
                   const std::byte* msg, size_t len) {
    return comm_object_.Send(rem_hostname, rem_port, msg, len);
  }

  /**
   * @brief Send one UDP packet to the connected remote server.
   *
   * @param msg Pointer to the message to send
   * @param len Length in bytes of the message to send
   */
  inline void Send(const std::byte* msg, size_t len) {
    return comm_object_.Send(msg, len);
  }

  // Enable recording of all packets sent by this UDP client
  inline void EnableRecording() { return comm_object_.EnableRecording(); }

 private:
  UDPComm comm_object_;
};

#endif  // UDP_CLIENT_H_