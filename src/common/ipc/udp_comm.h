/**
 * @file udp_comm.h
 * @brief Declaration file for the UDPComm class.  This class is used to send messages and receive messages from a udp endpoint
 */
#ifndef UDP_COMM_H_
#define UDP_COMM_H_

#include <netdb.h>

#include <cstddef>
#include <map>
#include <mutex>
#include <string>
#include <vector>

// Basic UDP client class based on OS sockets that supports sending messages
// and caches remote addrinfo mappings
class UDPComm {
 public:
  static constexpr bool kDebugPrintUdpInit = false;
  static constexpr bool kDebugPrintUdpSend = false;
  static constexpr bool kDebugPrintUdpRecv = false;
  explicit UDPComm(std::string local_addr, uint16_t local_port,
                   size_t rx_buffer_size, size_t tx_buffer_size);

  explicit UDPComm(std::string local_addr, const std::string& local_port,
                   size_t rx_buffer_size, size_t tx_buffer_size);

  UDPComm& operator=(const UDPComm&) = delete;
  UDPComm(const UDPComm&) = delete;
  ~UDPComm();

  /**
   * @brief The remote_address | remote_port is the address to which datagrams are sent.
   * 1:1 association me<->remote
   *
   * @param remote_address Hostname or IP address of the endpoint
   * @param remote_port UDP port of the endpoint
   */
  ssize_t Connect(const std::string& remote_address,
                  uint16_t remote_port) const;
  ssize_t Connect(const std::string& remote_address,
                  const std::string& remote_port) const;

  /**
   * @brief Send one UDP packet to a remote server. The client caches the
   * the remote server's addrinfo after resolving it for the first time. After
   * the first time, sending data does not require expensive addrinfo
   * resolution.
   *
   * @param to_hostname Hostname or IP address of the remote server
   * @param to_port UDP port that the remote server is listening on
   * @param msg Pointer to the message to send
   * @param len Length in bytes of the message to send
   */
  void Send(const std::string& rem_hostname, uint16_t rem_port,
            const std::byte* msg, size_t len);

  /**
   * @brief Send one UDP packet to the connected remote server.
   *
   * @param msg Pointer to the message to send
   * @param len Length in bytes of the message to send
   */
  void Send(const std::byte* msg, size_t len);

  /**
   * @brief Try to receive up to len bytes in buf by default this will not block
   *
   * @return Return the number of bytes received if non-zero bytes are
   * received. If no bytes are received, return zero. If there was an error
   * in receiving, return -1.
   */
  ssize_t Recv(std::byte* buf, size_t len) const;
  ssize_t Recv(const std::string& src_address, uint16_t src_port,
               std::byte* buf, size_t len);

  // Enable recording of all packets sent by this UDPComm object
  inline void EnableRecording() { enable_recording_flag_ = true; }

  /**
   * @brief Configures the socket in blocking mode.  Any calls to recv / send
   * will now block
   */
  void MakeBlocking(size_t rx_timeout_sec = 0) const;

 private:
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

  /**
   * @brief All packets sent, maintained if recording is enabled
   */
  std::vector<std::vector<uint8_t>> sent_vec_;

  /**
   * @brief If set to ture, we record all sent packets, otherwise we dont
   */
  bool enable_recording_flag_ = false;
};

#endif  // UDP_CLIENT_H_