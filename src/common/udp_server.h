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
#include <map>
#include <mutex>
#include <stdexcept>

#include "logger.h"

/// Basic UDP server class based on OS sockets that supports receiving messages
class UDPServer {
 public:
  static const bool kDebugPrintUdpServerInit = true;

  // Initialize a UDP server listening on this UDP port with socket buffer
  // size = rx_buffer_size
  explicit UDPServer(uint16_t port, size_t rx_buffer_size = 0) : port_(port) {
    if (kDebugPrintUdpServerInit) {
      AGORA_LOG_INFO("Creating UDP server listening at port %d\n", port);
    }
    sock_fd_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, IPPROTO_UDP);
    if (sock_fd_ == -1) {
      throw std::runtime_error(
          "UDPServer: Failed to create local socket. errno = " +
          std::string(std::strerror(errno)));
    }

    int ret = 0;

    // Set buffer size
    if (rx_buffer_size != 0) {
      const unsigned int desired_buf_size =
          static_cast<unsigned int>(rx_buffer_size);
      unsigned int actual_buf_size;
      socklen_t actual_buf_storage_size = sizeof(actual_buf_size);

      ret = ::getsockopt(sock_fd_, SOL_SOCKET, SO_RCVBUF, &actual_buf_size,
                         &actual_buf_storage_size);

      if (ret < 0 || (actual_buf_size != desired_buf_size)) {
        actual_buf_size = desired_buf_size;
        ret = ::setsockopt(sock_fd_, SOL_SOCKET, SO_RCVBUF, &actual_buf_size,
                           actual_buf_storage_size);

        if (ret != 0) {
          throw std::runtime_error("UDPServer: Failed to set RX buffer size.");
        }
      }

      ret = ::getsockopt(sock_fd_, SOL_SOCKET, SO_RCVBUF, &actual_buf_size,
                         &actual_buf_storage_size);

      // Linux likes to return 2* the buffer size
      if ((actual_buf_size != desired_buf_size) &&
          (actual_buf_size != (desired_buf_size * 2))) {
        AGORA_LOG_ERROR(
            "Error setting RX buffer size to %zu actual size %d with status "
            "%d\n",
            rx_buffer_size, actual_buf_size, ret);
      }
    }

    ::sockaddr_in serveraddr;
    serveraddr.sin_family = AF_INET;
    serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);
    serveraddr.sin_port = htons(static_cast<unsigned short>(port));
    std::memset(serveraddr.sin_zero, 0u, sizeof(serveraddr.sin_zero));

    ret = ::bind(sock_fd_, reinterpret_cast<::sockaddr*>(&serveraddr),
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
      ::freeaddrinfo(kv.second);
    }
    addrinfo_map_.clear();

    if (sock_fd_ != -1) {
      ::close(sock_fd_);
      sock_fd_ = -1;
    }

    if (kDebugPrintUdpServerInit) {
      AGORA_LOG_INFO("Destroying UDPServer\n");
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
    ssize_t ret = ::recv(sock_fd_, static_cast<void*>(buf), len, 0);

    if (ret == -1) {
      if ((errno == EAGAIN) || (errno == EWOULDBLOCK)) {
        // These errors mean that there's no data to receive
        ret = 0;
      } else {
        AGORA_LOG_ERROR("UDPServer: recv() failed with unexpected error %s\n",
                        std::strerror(errno));
      }
    } else if (ret == 0) {
      AGORA_LOG_ERROR("UDPServer: recv() failed with return of 0\n");
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
    addrinfo* rem_addrinfo = nullptr;

    const auto remote_itr = addrinfo_map_.find(remote_uri);
    if (remote_itr == addrinfo_map_.end()) {
      char port_str[16u];
      ::snprintf(port_str, sizeof(port_str), "%u", src_port);

      addrinfo hints;
      std::memset(&hints, 0, sizeof(hints));
      hints.ai_family = AF_INET;
      hints.ai_socktype = SOCK_DGRAM;
      hints.ai_protocol = IPPROTO_UDP;

      int r =
          ::getaddrinfo(src_address.c_str(), port_str, &hints, &rem_addrinfo);
      if ((r != 0) || (rem_addrinfo == nullptr)) {
        char issue_msg[1000u];
        AGORA_LOG_ERROR(issue_msg,
                        "Failed to resolve %s. getaddrinfo error = %s.",
                        remote_uri.c_str(), gai_strerror(r));
        throw std::runtime_error(issue_msg);
      }

      std::pair<std::map<std::string, addrinfo*>::iterator, bool>
          map_insert_result;
      {  // Synchronize access to insert for thread safety
        std::scoped_lock map_access(map_insert_access_);
        map_insert_result = addrinfo_map_.insert(
            std::pair<std::string, addrinfo*>(remote_uri, rem_addrinfo));
      }
    } else {
      rem_addrinfo = remote_itr->second;
    }

    socklen_t addrlen = rem_addrinfo->ai_addrlen;
    ssize_t ret = ::recvfrom(sock_fd_, static_cast<void*>(buf), len, 0,
                             rem_addrinfo->ai_addr, &addrlen);

    if (ret == -1) {
      if ((errno == EAGAIN) || (errno == EWOULDBLOCK)) {
        // These errors mean that there's no data to receive
        ret = 0;
      } else {
        AGORA_LOG_ERROR(
            "UDPServer: recvfrom() failed with unexpected error %s\n",
            std::strerror(errno));
      }
    } else if (ret == 0) {
      AGORA_LOG_ERROR("UDPServer: recv() failed with return of 0\n");
    }
    return ret;
  }

  /**
   * @brief Configures the socket in blocking mode.  Any calls to recv / send
   * will now block
   */
  void MakeBlocking(size_t timeout_sec = 0) const {
    int current_flags = ::fcntl(sock_fd_, F_GETFL);
    if (current_flags == -1) {
      throw std::runtime_error("UDPServer: fcntl failed to get flags");
    }
    int desired_flags = current_flags & (~O_NONBLOCK);

    if (desired_flags != current_flags) {
      int fcntl_status = ::fcntl(sock_fd_, F_SETFL, desired_flags);
      if (fcntl_status == -1) {
        throw std::runtime_error("UDPServer: fcntl failed to set blocking");
      }

      // Verify the flags were properly set
      current_flags = ::fcntl(sock_fd_, F_GETFL);
      if (current_flags == -1) {
        throw std::runtime_error("UDPServer: fcntl failed to get flags");
      } else if (current_flags != desired_flags) {
        throw std::runtime_error(
            "UDPServer: failed to set UDP socket to blocking");
      }
    }

    // Set timeout
    if (timeout_sec != 0) {
      timeval tv;
      tv.tv_sec = timeout_sec;
      tv.tv_usec = 0;
      int opt_status =
          ::setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
      if (opt_status != 0) {
        throw std::runtime_error("UDPServer: Failed to set timeout.");
      }
    }
  }

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