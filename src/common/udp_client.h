//	Copyright 2018, Carnegie Mellon University
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
#ifndef UDP_CLIENT_H_
#define UDP_CLIENT_H_

#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring> /* std::strerror, std::memset, std::memcpy */
#include <map>
#include <mutex>
#include <stdexcept>
#include <vector>

#include "logger.h"

// Basic UDP client class based on OS sockets that supports sending messages
// and caches remote addrinfo mappings
class UDPClient {
 public:
  static const bool kDebugPrintUdpClientInit = false;
  static const bool kDebugPrintUdpClientSend = false;
  explicit UDPClient(uint16_t src_port = 0) {
    if (kDebugPrintUdpClientInit) {
      AGORA_LOG_INFO("Creating UDP Client socket sending from port %d\n",
                     src_port);
    }
    sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock_fd_ == -1) {
      throw std::runtime_error(
          "UDPClient: Failed to create local socket. errno = " +
          std::string(std::strerror(errno)));
    }

    // Configure source address (helpful for flow control)
    if (src_port != 0) {
      ::sockaddr_in local_address;
      local_address.sin_family = AF_INET;
      local_address.sin_addr.s_addr = htonl(INADDR_ANY);
      local_address.sin_port = htons(src_port);
      std::memset(local_address.sin_zero, 0u, sizeof(local_address.sin_zero));
      const int bind_result =
          ::bind(sock_fd_, reinterpret_cast<::sockaddr*>(&local_address),
                 sizeof(local_address));
      if (bind_result != 0) {
        throw std::runtime_error("UDPClient: Failed to bind local socket.");
      }
    }
  }

  UDPClient(const UDPClient&) = delete;

  ~UDPClient() {
    for (const auto& kv : addrinfo_map_) {
      ::freeaddrinfo(kv.second);
    }
    addrinfo_map_.clear();
    if (sock_fd_ != -1) {
      const int close_status = ::close(sock_fd_);
      if (close_status != 0) {
        AGORA_LOG_WARN(
            "WARNING - UDPClient failed while trying to close socket %d with "
            "status %d\n",
            sock_fd_, close_status);
      }
      sock_fd_ = -1;
    }

    if (kDebugPrintUdpClientInit) {
      AGORA_LOG_INFO("Destroying UDPClient\n");
    }
  }

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
            const uint8_t* msg, size_t len) {
    std::string remote_uri = rem_hostname + ":" + std::to_string(rem_port);
    ::addrinfo* rem_addrinfo = nullptr;

    if (kDebugPrintUdpClientSend) {
      AGORA_LOG_INFO("UDPClient sending message to %s:%d of size %zu\n",
                     rem_hostname.c_str(), rem_port, len);
    }

    const auto remote_itr = addrinfo_map_.find(remote_uri);
    if (remote_itr == addrinfo_map_.end()) {
      char port_str[16u];
      std::snprintf(port_str, sizeof(port_str), "%u", rem_port);

      ::addrinfo hints;
      std::memset(&hints, 0, sizeof(hints));
      hints.ai_family = AF_INET;
      hints.ai_socktype = SOCK_DGRAM;
      hints.ai_protocol = IPPROTO_UDP;

      int r =
          ::getaddrinfo(rem_hostname.c_str(), port_str, &hints, &rem_addrinfo);
      if ((r != 0) || (rem_addrinfo == nullptr)) {
        char issue_msg[1000u];
        std::sprintf(issue_msg, "Failed to resolve %s. getaddrinfo error = %s.",
                     remote_uri.c_str(), gai_strerror(r));
        throw std::runtime_error(issue_msg);
      }

      if (kDebugPrintUdpClientInit) {
        AGORA_LOG_INFO("%d Resolving: %s map size %zu\n", sock_fd_,
                       remote_uri.c_str(), addrinfo_map_.size());
      }

      std::pair<std::map<std::string, struct addrinfo*>::iterator, bool>
          map_insert_result;

      {  // Synchronize access to insert for thread safety
        std::scoped_lock map_access(map_insert_access_);
        map_insert_result = addrinfo_map_.insert(
            std::pair<std::string, struct addrinfo*>(remote_uri, rem_addrinfo));
      }

      if (map_insert_result.second == false) {
        ::freeaddrinfo(rem_addrinfo);
        rem_addrinfo = map_insert_result.first->second;
      }
    } else {
      rem_addrinfo = remote_itr->second;
    }

    ssize_t ret = ::sendto(sock_fd_, msg, len, 0, rem_addrinfo->ai_addr,
                           rem_addrinfo->ai_addrlen);
    if (ret != static_cast<ssize_t>(len)) {
      throw std::runtime_error("sendto() failed. errno = " +
                               std::string(std::strerror(errno)));
    }

    if (enable_recording_flag_) {
      std::scoped_lock map_access(map_insert_access_);
      sent_vec_.emplace_back(msg, msg + len);
    }
  }

  // Enable recording of all packets sent by this UDP client
  void EnableRecording() { enable_recording_flag_ = true; }

 private:
  /**
   * @brief The raw socket file descriptor
   */
  int sock_fd_ = -1;

  /**
   * @brief A cache mapping hostname:udp_port to addrinfo
   */
  std::map<std::string, struct addrinfo*> addrinfo_map_;
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