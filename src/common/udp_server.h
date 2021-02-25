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

#include <fcntl.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring> /* std::strerror, std::memset, std::memcpy */
#include <stdexcept>

/// Basic UDP server class based on OS sockets that supports receiving messages
class UDPServer {
 public:
  // Initialize a UDP server listening on this UDP port with socket buffer
  // size = rx_buffer_size
  explicit UDPServer(uint16_t port, size_t rx_buffer_size = 0) : port_(port) {
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
    if (sock_fd_ != -1) {
      close(sock_fd_);
    }
  }

  /**
   * @brief Try once to receive up to len bytes in buf without blocking
   *
   * @return Return the number of bytes received if non-zero bytes are
   * received. If no bytes are received, return zero. If there was an error
   * in receiving, return -1.
   *
   * return 0.
   */
  ssize_t RecvNonblocking(uint8_t* buf, size_t len) const {
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

 private:
  uint16_t port_;  // The UDP port to listen on
  int sock_fd_ = -1;
};

#endif  // UDP_SERVER_H_
