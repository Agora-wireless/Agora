/**
 * @file udp_server_ipv6.cc
 * 
 * @brief Defination file for the UDPServerIPv6 class
 */
#include "udp_server_ipv6.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring> /* std::strerror, std::memset, std::memcpy */
#include <stdexcept>

#include "logger.h"

// Initialize a UDP server listening on this UDP port with socket buffer
// size = rx_buffer_size
UDPServerIPv6::UDPServerIPv6(uint16_t port, size_t rx_buffer_size)
    : port_(port) {
  std::string local_port(std::to_string(port));

  addrinfo hints;
  std::memset(&hints, 0u, sizeof(hints));
  hints.ai_family = AF_UNSPEC;    /* Allow IPv4 or IPv6 */
  hints.ai_socktype = SOCK_DGRAM; /* Datagram socket */
  hints.ai_flags = AI_PASSIVE;    /* For wildcard IP address */
  hints.ai_protocol = 0;          /* Any protocol */
  hints.ai_canonname = NULL;
  hints.ai_addr = NULL;
  hints.ai_next = NULL;

  addrinfo* found_addresses = nullptr;
  //Set node to NULL for loopback
  int r = ::getaddrinfo(local_address.c_str(), local_port.c_str(), &hints,
                        &found_addresses);

  if (kDebugPrintUdpServerInit) {
    AGORA_LOG_INFO("Creating UDP server listening at port %d\n", port);
  }
  sock_fd_ = ::socket(AF_INET6, SOCK_DGRAM | SOCK_NONBLOCK, IPPROTO_UDP);
  if (sock_fd_ == -1) {
    throw std::runtime_error(
        "UDPServerIPv6: Failed to create local socket. errno = " +
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
        throw std::runtime_error(
            "UDPServerIPv6: Failed to set RX buffer size.");
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

  //Replace with getaddrinfo / getifaddrs
  //::sockaddr_in6 serveraddr;
  //std::memset(&serveraddr, 0u, sizeof(serveraddr));
  //serveraddr.sin6_family = AF_INET6;
  //serveraddr.sin6_addr = in6addr_any; /* :: */
  //serveraddr.sin6_port = htons(static_cast<unsigned short>(port));

  addrinfo hints;
  std::memset(&hints, 0u, sizeof(hints));
  hints.ai_family = AF_UNSPEC;    /* Allow IPv4 or IPv6 */
  hints.ai_socktype = SOCK_DGRAM; /* Datagram socket */
  hints.ai_flags = AI_PASSIVE;    /* For wildcard IP address */
  hints.ai_protocol = 0;          /* Any protocol */
  hints.ai_canonname = NULL;
  hints.ai_addr = NULL;
  hints.ai_next = NULL;

  addrinfo* found_addresses = nullptr;
  //Set node to NULL for loopback
  int r = ::getaddrinfo(local_address.c_str(), local_port.c_str(), &hints,
                        &found_addresses);

    ret = ::bind(sock_fd_, reinterpret_cast<::sockaddr*>(&serveraddr),
               sizeof(serveraddr));
  if (ret != 0) {
    throw std::runtime_error("UDPServerIPv6: Failed to bind socket to port " +
                             std::to_string(port) +
                             ". Error: " + std::strerror(errno));
  }
}

UDPServerIPv6::~UDPServerIPv6() {
  if (sock_fd_ != -1) {
    ::close(sock_fd_);
    sock_fd_ = -1;
  }

  if (socket_address_info_ != nullptr) {
    ::freeaddrinfo(socket_address_info_);
    socket_address_info_ = nullptr;
  }

  if (kDebugPrintUdpServerInit) {
    AGORA_LOG_INFO("Destroying UDPServerIPv6\n");
  }
}

/**
   * @brief Try to receive up to len bytes in buf by default this will not block
   *
   * @return Return the number of bytes received if non-zero bytes are
   * received. If no bytes are received, return zero. If there was an error
   * in receiving, return -1.
   */
ssize_t UDPServerIPv6::Recv(uint8_t* buf, size_t len) const {
  ssize_t ret = ::recv(sock_fd_, static_cast<void*>(buf), len, 0);

  if (ret == -1) {
    if ((errno == EAGAIN) || (errno == EWOULDBLOCK)) {
      // These errors mean that there's no data to receive
      ret = 0;
    } else {
      AGORA_LOG_ERROR("UDPServerIPv6: recv() failed with unexpected error %s\n",
                      std::strerror(errno));
    }
  } else if (ret == 0) {
    AGORA_LOG_ERROR("UDPServerIPv6: recv() failed with return of 0\n");
  }
  return ret;
}

/**
   * @brief Locate and connect to a remote address 
   *
   * @return Connect for DGRAM sockets just indicates a 1:1 socket
   */
ssize_t UDPServerIPv6::Connect(const std::string& src_address,
                               uint16_t src_port) {
  std::string remote_port(std::to_string(src_port));
  addrinfo* found_addresses = nullptr;

  addrinfo hints;
  std::memset(&hints, 0u, sizeof(hints));
  hints.ai_family = AF_UNSPEC;    /* Allow IPv4 or IPv6 */
  hints.ai_socktype = SOCK_DGRAM; /* Datagram socket */
  hints.ai_flags = AI_PASSIVE;    /* For wildcard IP address */
  hints.ai_protocol = 0;          /* Any protocol */
  hints.ai_canonname = NULL;
  hints.ai_addr = NULL;
  hints.ai_next = NULL;

  int r = ::getaddrinfo(src_address.c_str(), remote_port.c_str(), &hints,
                        &found_addresses);
  if ((r != 0) || (found_addresses == nullptr)) {
    AGORA_LOG_ERROR("Failed to resolve %s. getaddrinfo error = %s.",
                    src_address.c_str(), gai_strerror(r));
    throw std::runtime_error("Failed to result getaddrinfo");
  }

  addrinfo* rem_connect;
  // getaddrinfo() returns a list of address structures, find the correct one to connect
  for (rem_connect = found_addresses; rem_connect != nullptr;
       rem_connect = rem_connect->ai_next) {
    if ((rem_connect->ai_family == local_address_->ai_family) &&
        (rem_connect->ai_socktype == local_address_->ai_socktype) &&
        (rem_connect->ai_protocol == local_address_->ai_protocol)) {
      AGORA_LOG_INFO("Found remote with family %d, type %d, proto %d\n",
                     rem_connect->ai_family, rem_connect->ai_socktype,
                     rem_connect->ai_protocol);
      break;
    }
    AGORA_LOG_TRACE("Found remote with family %d, type %d, proto %d\n",
                    rem_connect->ai_family, rem_connect->ai_socktype,
                    rem_connect->ai_protocol);
  }

  if (rem_connect != nullptr) {
    AGORA_LOG_WARN("Could not find a compatible remote with address %s:%s\n",
                   src_address, remote_port);
  }
  r = ::connect(sock_fd_, rem_connect->ai_addr, rem_connect->ai_addrlen);
  ::freeaddrinfo(found_addresses);
  return r;
}

/**
   * @brief Configures the socket in blocking mode.  Any calls to recv / send
   * will now block
   */
void UDPServerIPv6::MakeBlocking(size_t timeout_sec) const {
  int current_flags = ::fcntl(sock_fd_, F_GETFL);
  if (current_flags == -1) {
    throw std::runtime_error("UDPServerIPv6: fcntl failed to get flags");
  }
  int desired_flags = current_flags & (~O_NONBLOCK);

  if (desired_flags != current_flags) {
    int fcntl_status = ::fcntl(sock_fd_, F_SETFL, desired_flags);
    if (fcntl_status == -1) {
      throw std::runtime_error("UDPServerIPv6: fcntl failed to set blocking");
    }

    // Verify the flags were properly set
    current_flags = ::fcntl(sock_fd_, F_GETFL);
    if (current_flags == -1) {
      throw std::runtime_error("UDPServerIPv6: fcntl failed to get flags");
    } else if (current_flags != desired_flags) {
      throw std::runtime_error(
          "UDPServerIPv6: failed to set UDP socket to blocking");
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
      throw std::runtime_error("UDPServerIPv6: Failed to set timeout.");
    }
  }
}