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
#include "network_utils.h"
#include "utils.h"

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
      sock_fd_(-1) {
  if (kDebugPrintUdpServerInit) {
    AGORA_LOG_TRACE("Creating UDP Server at address %s port %s\n",
                    address_.c_str(), port_.c_str());
  }
  /// Find the local interface
  auto server_address_info = agora_comm::GetAddressInfo(address_, port_);
  if (kDebugPrintUdpServerInit) {
    agora_comm::PrintAddressInfo(server_address_info);
  }

  RtAssert((server_address_info != nullptr) &&
               (server_address_info->ai_next == nullptr),
           "Found 0 or more than 1 acceptable address");

  sock_fd_ = ::socket(server_address_info->ai_family,
                      server_address_info->ai_socktype | SOCK_NONBLOCK,
                      server_address_info->ai_protocol);
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
    ::socklen_t actual_buf_storage_size = sizeof(actual_buf_size);

    auto sock_ret = ::getsockopt(sock_fd_, SOL_SOCKET, SO_RCVBUF,
                                 &actual_buf_size, &actual_buf_storage_size);

    if (sock_ret < 0 || (actual_buf_size != desired_buf_size)) {
      actual_buf_size = desired_buf_size;
      sock_ret = ::setsockopt(sock_fd_, SOL_SOCKET, SO_RCVBUF, &actual_buf_size,
                              actual_buf_storage_size);

      if (sock_ret != 0) {
        throw std::runtime_error(
            "UDPServerIPv6: Failed to set RX buffer size.");
      }
    }

    sock_ret = ::getsockopt(sock_fd_, SOL_SOCKET, SO_RCVBUF, &actual_buf_size,
                            &actual_buf_storage_size);

    // Linux likes to return 2* the buffer size
    if ((actual_buf_size != desired_buf_size) &&
        (actual_buf_size != (desired_buf_size * 2))) {
      AGORA_LOG_WARN(
          "Error setting RX buffer size to %zu actual size %d with status "
          "%d\n",
          rx_buffer_size, actual_buf_size, sock_ret);
    }
  }

  /// Listen on the found address
  const auto bind_status = ::bind(sock_fd_, server_address_info->ai_addr,
                                  server_address_info->ai_addrlen);
  if (bind_status != 0) {
    throw std::runtime_error("UDPServerIPv6: Failed to bind socket to port " +
                             port_ + ". Error: " + std::strerror(errno));
  }
  ::freeaddrinfo(server_address_info);
}

UDPServerIPv6::~UDPServerIPv6() {
  for (const auto& kv : addrinfo_map_) {
    ::freeaddrinfo(kv.second);
  }
  addrinfo_map_.clear();

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
   * @brief Try once to receive up to len bytes in buf
   *
   * @return Return the number of bytes received if non-zero bytes are
   * received. If no bytes are received, return zero. If there was an error
   * in receiving, return -1.
   */
ssize_t UDPServerIPv6::Recv(const std::string& src_address, uint16_t src_port,
                            std::byte* buf, size_t len) {
  const std::string port_string = std::to_string(src_port);
  const std::string remote_uri = src_address + ":" + port_string;
  ::addrinfo* rem_addrinfo = nullptr;
  AGORA_LOG_TRACE("Attempting Recv from %s \n", remote_uri.c_str());

  const auto remote_itr = addrinfo_map_.find(remote_uri);
  if (remote_itr == addrinfo_map_.end()) {
    rem_addrinfo = agora_comm::GetAddressInfo(src_address, port_string);
    if (rem_addrinfo == nullptr) {
      char issue_msg[1000u];
      AGORA_LOG_ERROR(issue_msg, "Failed to resolve %s", remote_uri.c_str());
      throw std::runtime_error(issue_msg);
    }

    std::pair<std::map<std::string, ::addrinfo*>::iterator, bool>
        map_insert_result;
    {  // Synchronize access to insert for thread safety
      std::scoped_lock map_access(map_insert_access_);
      map_insert_result = addrinfo_map_.insert(
          std::pair<std::string, ::addrinfo*>(remote_uri, rem_addrinfo));
    }
  } else {
    rem_addrinfo = remote_itr->second;
  }

  auto addrlen = rem_addrinfo->ai_addrlen;
  ssize_t ret = ::recvfrom(sock_fd_, static_cast<void*>(buf), len, 0,
                           rem_addrinfo->ai_addr, &addrlen);

  if (ret == -1) {
    if ((errno == EAGAIN) || (errno == EWOULDBLOCK)) {
      // These errors mean that there's no data to receive
      ret = 0;
    } else {
      AGORA_LOG_ERROR("UDPServer: recvfrom() failed with unexpected error %s\n",
                      std::strerror(errno));
    }
  } else if (ret == 0) {
    AGORA_LOG_ERROR("UDPServer: recv() failed with return of 0\n");
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
  /// Find the local interface
  auto client_address_info =
      agora_comm::GetAddressInfo(remote_address, remote_port);
  if (kDebugPrintUdpServerInit) {
    agora_comm::PrintAddressInfo(client_address_info);
  }

  if (client_address_info == nullptr) {
    AGORA_LOG_ERROR("Failed to resolve %s", remote_address.c_str());
    throw std::runtime_error("Failed to resolve getaddrinfo()");
  } else if (client_address_info->ai_next != nullptr) {
    ::freeaddrinfo(client_address_info);
    AGORA_LOG_ERROR("Too many client sockets found for address %s. port %s.\n",
                    remote_address.c_str(), remote_port.c_str());
    throw std::runtime_error(
        "Connect(): Too many acceptable addresses returned from getaddrinfo");
  }

  // GetAddressInfo() returns a list of address structures, find the correct one to connect
  if (kDebugPrintUdpServerInit) {
    for (addrinfo* rem_connect = client_address_info; rem_connect != nullptr;
         rem_connect = rem_connect->ai_next) {
      AGORA_LOG_INFO("Found remote with family %d, type %d, proto %d\n",
                     rem_connect->ai_family, rem_connect->ai_socktype,
                     rem_connect->ai_protocol);
    }
  }
  auto r = ::connect(sock_fd_, client_address_info->ai_addr,
                     client_address_info->ai_addrlen);
  ::freeaddrinfo(client_address_info);
  if (r != 0) {
    AGORA_LOG_ERROR("Failed to connect %s", remote_address.c_str());
    throw std::runtime_error("Failed to connect()");
  }
  return r;
}

ssize_t UDPServerIPv6::Connect(const std::string& remote_address,
                               uint16_t remote_port) {
  std::string remote = std::to_string(remote_port);
  if (remote == "0") {
    remote.clear();
  }
  return Connect(remote_address, std::to_string(remote_port));
}

/**
   * @brief Configures the socket in blocking mode.  Any calls to recv / send
   * will now block
   */
void UDPServerIPv6::MakeBlocking(size_t rx_timeout_sec) const {
  int current_flags = ::fcntl(sock_fd_, F_GETFL);
  if (current_flags == -1) {
    throw std::runtime_error("UDPServerIPv6: fcntl failed to get flags");
  }
  const int desired_flags = current_flags & (~O_NONBLOCK);

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
  if (rx_timeout_sec != 0) {
    timeval tv;
    tv.tv_sec = rx_timeout_sec;
    tv.tv_usec = 0;
    int opt_status =
        ::setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    if (opt_status != 0) {
      throw std::runtime_error("UDPServerIPv6: Failed to set timeout.");
    }
  }
}