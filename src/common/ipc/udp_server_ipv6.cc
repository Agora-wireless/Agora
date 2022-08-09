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
#include <utility>

#include "logger.h"
#include "utils.h"

//Allow IPv4 or IPv6(AF_UNSPEC);
static const int kAllowedAiFamily = AF_INET6;

UDPServerIPv6::UDPServerIPv6(const std::string& local_address,
                             uint16_t local_port, size_t rx_buffer_size)
    : UDPServerIPv6(local_address, std::to_string(local_port), rx_buffer_size) {
}

// Initialize a UDP server listening on this UDP port with socket buffer
// size = rx_buffer_size
UDPServerIPv6::UDPServerIPv6(std::string local_address, std::string local_port,
                             size_t rx_buffer_size)
    : port_(std::move(local_port)),
      address_(std::move(local_address)),
      sock_fd_(-1),
      server_address_info_(nullptr),
      connected_address_info_(nullptr) {
  AGORA_LOG_TRACE("Creating UDP Server at address %s port %s\n",
                  local_address.c_str(), local_port.c_str());

  //Set node to nullptr for loopback
  //node = &address_buffer[0u];
  //auto pton_result =
  //    ::inet_pton(kAllowedAiFamily, local_address.c_str(), address_buffer);
  //std::printf("Pton result %d\n", pton_result);

  const char* node;
  addrinfo hints;
  std::memset(&hints, 0u, sizeof(hints));
  hints.ai_family = kAllowedAiFamily;
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_flags = AI_NUMERICSERV;
  if (address_.empty()) {
    /// Set node to NULL for loopback or all interfaces
    node = nullptr;
    /// wildcard
    hints.ai_flags |= AI_PASSIVE;
  } else {
    node = address_.c_str();
    hints.ai_flags |= AI_NUMERICHOST;
  }
  /// Any protocol
  hints.ai_protocol = 0;
  hints.ai_canonname = nullptr;
  hints.ai_addr = nullptr;
  hints.ai_next = nullptr;

  int ret = ::getaddrinfo(node, port_.c_str(), &hints, &server_address_info_);
  if (ret < 0) {
    AGORA_LOG_ERROR("getaddrinfo returned error - %s (%d)\n", gai_strerror(ret),
                    ret);
    throw std::runtime_error("getaddrinfo returned error");
  }

  for (addrinfo* rp = server_address_info_; rp != nullptr; rp = rp->ai_next) {
    const int family = rp->ai_family;
    AGORA_LOG_TRACE(
        "Found address with family : %s (%d) type %d and protocol %d\n",
        (family == AF_PACKET)  ? "AF_PACKET"
        : (family == AF_INET)  ? "AF_INET"
        : (family == AF_INET6) ? "AF_INET6"
                               : "???",
        rp->ai_family, rp->ai_socktype, rp->ai_protocol);
    if (family == AF_INET6) {
      [[maybe_unused]] auto* address_ptr =
          &((sockaddr_in6*)rp->ai_addr)->sin6_addr;
      [[maybe_unused]] char address_buffer[INET6_ADDRSTRLEN];
      AGORA_LOG_TRACE("Ipv6 Address:  %s \n",
                      ::inet_ntop(family, address_ptr, address_buffer,
                                  sizeof(address_buffer)));
    } else {
      AGORA_LOG_ERROR(
          "UDPServerIPv6: Found address with unsupported family %d\n", family);
    }
  }

  RtAssert((ret == 0) && (server_address_info_ != nullptr) &&
               (server_address_info_->ai_next == nullptr),
           "Found 0 or more than 1 acceptable address with return status " +
               std::to_string(ret));

  if (kDebugPrintUdpServerInit) {
    AGORA_LOG_INFO("Creating UDP server listening at port %s\n", port_.c_str());
  }
  sock_fd_ = ::socket(server_address_info_->ai_family,
                      server_address_info_->ai_socktype | SOCK_NONBLOCK,
                      server_address_info_->ai_protocol);
  if (sock_fd_ < 0) {
    throw std::runtime_error(
        "UDPServerIPv6: Failed to create local socket. errno = " +
        std::string(std::strerror(errno)));
  }

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

  //Listen on the found address
  ret = ::bind(sock_fd_, server_address_info_->ai_addr,
               server_address_info_->ai_addrlen);
  if (ret != 0) {
    throw std::runtime_error("UDPServerIPv6: Failed to bind socket to port " +
                             port_ + ". Error: " + std::strerror(errno));
  }
}

UDPServerIPv6::~UDPServerIPv6() {
  if (sock_fd_ > 0) {
    auto status = ::close(sock_fd_);
    sock_fd_ = -1;

    if (status < 0) {
      AGORA_LOG_ERROR("UDPServerIPv6: Error %d reported in socket close\n",
                      status);
    }

    if (kDebugPrintUdpServerInit) {
      AGORA_LOG_INFO("UDPServerIPv6: Closing the socket\n");
    }
  }

  if (server_address_info_ != nullptr) {
    ::freeaddrinfo(server_address_info_);
    server_address_info_ = nullptr;
  }

  if (connected_address_info_ != nullptr) {
    ::freeaddrinfo(connected_address_info_);
    connected_address_info_ = nullptr;
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
ssize_t UDPServerIPv6::Recv(std::byte* buf, size_t len) const {
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
ssize_t UDPServerIPv6::Connect(const std::string& remote_address,
                               const std::string& remote_port) {
  RtAssert(
      (connected_address_info_ == nullptr) && (server_address_info_ != nullptr),
      "Connected address must be null before connection");

  addrinfo hints;
  std::memset(&hints, 0u, sizeof(hints));
  hints.ai_family = server_address_info_->ai_family;
  hints.ai_socktype = server_address_info_->ai_socktype;
  hints.ai_flags = AI_NUMERICHOST | AI_NUMERICSERV;
  hints.ai_protocol = server_address_info_->ai_protocol;
  hints.ai_canonname = server_address_info_->ai_canonname;
  hints.ai_addr = nullptr;
  hints.ai_next = nullptr;

  int r = ::getaddrinfo(remote_address.c_str(), remote_port.c_str(), &hints,
                        &connected_address_info_);
  if ((r != 0) || (connected_address_info_ == nullptr)) {
    AGORA_LOG_ERROR("Failed to resolve %s. getaddrinfo error = %s.",
                    remote_address.c_str(), gai_strerror(r));
    throw std::runtime_error("Failed to result getaddrinfo");
  } else if (connected_address_info_->ai_next != nullptr) {
    AGORA_LOG_ERROR("Too many client sockets found for address %s. port %s.\n",
                    remote_address.c_str(), remote_port.c_str());
    throw std::runtime_error(
        "Connect(): Too many acceptable addresses returned from getaddrinfo");
  }

  // getaddrinfo() returns a list of address structures, find the correct one to connect
  for (addrinfo* rem_connect = connected_address_info_; rem_connect != nullptr;
       rem_connect = rem_connect->ai_next) {
    if ((rem_connect->ai_family == server_address_info_->ai_family) &&
        (rem_connect->ai_socktype == server_address_info_->ai_socktype) &&
        (rem_connect->ai_protocol == server_address_info_->ai_protocol)) {
      AGORA_LOG_TRACE("Found remote with family %d, type %d, proto %d\n",
                      rem_connect->ai_family, rem_connect->ai_socktype,
                      rem_connect->ai_protocol);
      break;
    }
    AGORA_LOG_INFO("Found remote with family %d, type %d, proto %d\n",
                   rem_connect->ai_family, rem_connect->ai_socktype,
                   rem_connect->ai_protocol);
  }
  // Need to change this to rem_connect if we allow more than 1.
  r = ::connect(sock_fd_, connected_address_info_->ai_addr,
                connected_address_info_->ai_addrlen);
  return r;
}

ssize_t UDPServerIPv6::Connect(const std::string& remote_address,
                               uint16_t remote_port) {
  return Connect(remote_address, std::to_string(remote_port));
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