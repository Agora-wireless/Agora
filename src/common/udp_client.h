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
#include <stdexcept>
#include <vector>

// Basic UDP client class based on OS sockets that supports sending messages
// and caches remote addrinfo mappings
class UDPClient {
 public:
  static const bool kDebugPrintUdpClientInit = false;
  UDPClient() {
    if (kDebugPrintUdpClientInit) {
      std::printf("Creating UDP Client socket\n");
    }
    sock_fd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock_fd_ == -1) {
      throw std::runtime_error("UDPClient: Failed to create local socket.");
    }
  }

  UDPClient(const UDPClient&) = delete;

  ~UDPClient() {
    for (const auto& kv : addrinfo_map_) {
      freeaddrinfo(kv.second);
    }
    addrinfo_map_.clear();
    if (sock_fd_ != -1) {
      close(sock_fd_);
      sock_fd_ = -1;
    }

    if (kDebugPrintUdpClientInit) {
      std::printf("Destroying UDPClient\n");
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
    struct addrinfo* rem_addrinfo = nullptr;

    if (addrinfo_map_.count(remote_uri) != 0) {
      rem_addrinfo = addrinfo_map_.at(remote_uri);
    } else {
      char port_str[16];
      snprintf(port_str, sizeof(port_str), "%u", rem_port);

      struct addrinfo hints;
      std::memset(&hints, 0, sizeof(hints));
      hints.ai_family = AF_INET;
      hints.ai_socktype = SOCK_DGRAM;
      hints.ai_protocol = IPPROTO_UDP;

      int r =
          getaddrinfo(rem_hostname.c_str(), port_str, &hints, &rem_addrinfo);
      if (r != 0 || rem_addrinfo == nullptr) {
        char issue_msg[1000];
        sprintf(issue_msg, "Failed to resolve %s. getaddrinfo error = %s.",
                remote_uri.c_str(), gai_strerror(r));
        throw std::runtime_error(issue_msg);
      }

      addrinfo_map_[remote_uri] = rem_addrinfo;
    }

    ssize_t ret = sendto(sock_fd_, msg, len, 0, rem_addrinfo->ai_addr,
                         rem_addrinfo->ai_addrlen);
    if (ret != static_cast<ssize_t>(len)) {
      throw std::runtime_error("sendto() failed. errno = " +
                               std::string(std::strerror(errno)));
    }

    if (enable_recording_flag_) {
      sent_vec_.emplace_back(msg, msg + len);
    }
  }

  // Enable recording of all packets sent by this UDP client
  void EnableRecording() { enable_recording_flag_ = true; }

 private:
  int sock_fd_ = -1;

  // A cache mapping hostname:udp_port to addrinfo
  std::map<std::string, struct addrinfo*> addrinfo_map_;
  // The list of all packets sent, maintained if recording is enabled
  std::vector<std::vector<uint8_t>> sent_vec_;

  bool enable_recording_flag_ = false;  // If true, we record all sent packets
};

#endif  // UDP_CLIENT_H_