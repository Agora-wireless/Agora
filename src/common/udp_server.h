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

#ifndef UDP_SERVER_H_
#define UDP_SERVER_H_

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring> /* std::strerror, std::memset, std::memcpy */
#include <stdexcept>

/// Basic UDP server class based on OS sockets that supports receiving messages
class UDPServer {
 public:
  static const bool kDebugPrintUdpServerInit = true;

  // Initialize a UDP server listening on this UDP port with socket buffer
  // size = rx_buffer_size
  explicit UDPServer(uint16_t port, size_t rx_buffer_size = 0) : port_(port) {
    if (kDebugPrintUdpServerInit) {
      std::printf("Creating UDP server listening at port %d\n", port);
    }
    sock_fd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock_fd_ == -1) {
      throw std::runtime_error("UDPServer: Failed to create local socket.");
    }

    // Set the socket as non-blocking
    int flags = fcntl(sock_fd_, F_GETFL);
    if (flags == -1) {
      throw std::runtime_error("UDPServer: fcntl failed to get flags");
    }
    int ret = fcntl(sock_fd_, F_SETFL, flags | O_NONBLOCK);
    if (ret == -1) {
      throw std::runtime_error("UDPServer: fcntl failed to set nonblock");
    }

    // Set buffer size
    if (rx_buffer_size != 0) {
      ret = setsockopt(sock_fd_, SOL_SOCKET, SO_RCVBUF, &rx_buffer_size,
                       sizeof(rx_buffer_size));
      if (ret != 0) {
        throw std::runtime_error("UDPServer: Failed to set RX buffer size.");
      }
    }

    struct sockaddr_in serveraddr;
    serveraddr.sin_family = AF_INET;
    serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);
    serveraddr.sin_port = htons(static_cast<unsigned short>(port));
    std::memset(serveraddr.sin_zero, 0, sizeof(serveraddr.sin_zero));

    ret = bind(sock_fd_, reinterpret_cast<struct sockaddr*>(&serveraddr),
               sizeof(serveraddr));
    if (ret != 0) {
      throw std::runtime_error("UDPServer: Failed to bind socket to port " +
                               std::to_string(port) +
                               ". Error: " + std::strerror(errno));
    }
  }

  UDPServer& operator=(const UDPServer&) = delete;
  UDPServer(const UDPServer&) = delete;

  ~UDPServer() {
    for (const auto& kv : addrinfo_map_) {
      freeaddrinfo(kv.second);
    }
    addrinfo_map_.clear();

    if (sock_fd_ != -1) {
      close(sock_fd_);
      sock_fd_ = -1;
    }

    if (kDebugPrintUdpServerInit) {
      std::printf("Destroying UDPServer\n");
    }
  }

  /**
   * @brief Try to receive up to len bytes in buf by default this will not block
   *
   * @return Return the number of bytes received if non-zero bytes are
   * received. If no bytes are received, return zero. If there was an error
   * in receiving, return -1.
   */
  ssize_t Recv(uint8_t* buf, size_t len) const {
    ssize_t ret = recv(sock_fd_, static_cast<void*>(buf), len, 0);
    if (ret == -1) {
      if (errno == EAGAIN || ret == EWOULDBLOCK) {
        // These errors mean that there's no data to receive
        return 0;
      } else {
        std::fprintf(stderr,
                     "UDPServer: recv() failed with unexpected error %s\n",
                     std::strerror(errno));
        return ret;
      }
    }
    return ret;
  }

  /**
   * @brief Try once to receive up to len bytes in buf
   *
   * @return Return the number of bytes received if non-zero bytes are
   * received. If no bytes are received, return zero. If there was an error
   * in receiving, return -1.
   */
  ssize_t RecvFrom(uint8_t* buf, size_t len, const std::string& src_address,
                   uint16_t src_port) {
    std::string remote_uri = src_address + ":" + std::to_string(src_port);
    struct addrinfo* rem_addrinfo = nullptr;

    if (addrinfo_map_.count(remote_uri) != 0) {
      rem_addrinfo = addrinfo_map_.at(remote_uri);
    } else {
      char port_str[16u];
      snprintf(port_str, sizeof(port_str), "%u", src_port);

      struct addrinfo hints;
      std::memset(&hints, 0, sizeof(hints));
      hints.ai_family = AF_INET;
      hints.ai_socktype = SOCK_DGRAM;
      hints.ai_protocol = IPPROTO_UDP;

      int r = getaddrinfo(src_address.c_str(), port_str, &hints, &rem_addrinfo);
      if (r != 0 || rem_addrinfo == nullptr) {
        char issue_msg[1000];
        sprintf(issue_msg, "Failed to resolve %s. getaddrinfo error = %s.",
                remote_uri.c_str(), gai_strerror(r));
        throw std::runtime_error(issue_msg);
      }
      addrinfo_map_[remote_uri] = rem_addrinfo;
    }

    // struct sockaddr_in remote_addr;
    // remote_addr.sin_family = AF_INET;
    // remote_addr.sin_port = htons(src_port);
    // remote_addr.sin_addr.s_addr = inet_addr(src_address.c_str());
    // std::memset(remote_addr.sin_zero, 0, sizeof(remote_addr.sin_zero));
    // socklen_t addrlen = sizeof(remote_addr);

    socklen_t addrlen = rem_addrinfo->ai_addrlen;
    ssize_t ret = recvfrom(sock_fd_, static_cast<void*>(buf), len, 0,
                           rem_addrinfo->ai_addr, &addrlen);

    if (ret == -1) {
      if (errno == EAGAIN || ret == EWOULDBLOCK) {
        // These errors mean that there's no data to receive
        return 0;
      } else {
        std::fprintf(stderr,
                     "UDPServer: recvfrom() failed with unexpected error %s\n",
                     std::strerror(errno));
        return ret;
      }
    }
    return ret;
  }

  /**
   * @brief Configures the socket in blocking mode.  Any calls to recv / send
   * will now block
   */
  void MakeBlocking(size_t timeout_sec = 0) {
    int flags = fcntl(sock_fd_, F_GETFL);
    if (flags == -1) {
      throw std::runtime_error("UDPServer: fcntl failed to get flags");
    }
    flags = flags & (~O_NONBLOCK);
    int ret = fcntl(sock_fd_, F_SETFL, flags);
    if (ret == -1) {
      throw std::runtime_error("UDPServer: fcntl failed to set blocking");
    }

    // Set timeout
    if (timeout_sec != 0) {
      struct timeval tv;
      tv.tv_sec = timeout_sec;
      tv.tv_usec = 0;
      ret = setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
      if (ret != 0) {
        throw std::runtime_error("UDPServer: Failed to set timeout.");
      }
    }
  }

 private:
  // The UDP port to listen on
  uint16_t port_;
  int sock_fd_ = -1;

  // A cache mapping hostname:udp_port to addrinfo
  std::map<std::string, struct addrinfo*> addrinfo_map_;
};

#endif  // UDP_SERVER_H_
