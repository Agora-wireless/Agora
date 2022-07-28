/**
 * @file udp_client.h
 * @brief Declaration and Defination file for the UDPClient class.  This class is used to send messages to a remote server
 */
#ifndef UDP_CLIENT_H_
#define UDP_CLIENT_H_

#include <netdb.h>

#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <vector>

// Basic UDP client class based on OS sockets that supports sending messages
// and caches remote addrinfo mappings
class UDPClient {
 public:
  static const bool kDebugPrintUdpClientInit = false;
  static const bool kDebugPrintUdpClientSend = false;
  explicit UDPClient(uint16_t src_port = 0);

  UDPClient(const UDPClient&) = delete;
  ~UDPClient();

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
  void Send(const std::string& rem_hostname, uint16_t rem_port,
            const uint8_t* msg, size_t len);

  // Enable recording of all packets sent by this UDP client
  inline void EnableRecording() { enable_recording_flag_ = true; }

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