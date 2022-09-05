/**
 * @file udp_comm.cc
 * @brief Definition file for the UDPComm class.  This class is used to send/recv messages from a remote endpoint
 */
#include "udp_comm.h"

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

///Default to ipv4 for historic reasons, use ::1 if you want ipv6
static const std::string kDefaultAddress = "127.0.0.1";

UDPComm::UDPComm(std::string local_addr, uint16_t local_port,
                 size_t rx_buffer_size, size_t tx_buffer_size)
    : UDPComm(std::move(local_addr),
              (local_port == 0) ? std::string() : std::to_string(local_port),
              rx_buffer_size, tx_buffer_size) {}

/// getaddrinfo() -> socket -> bind (for local address/port assignment)
UDPComm::UDPComm(std::string local_addr, const std::string& local_port,
                 size_t rx_buffer_size, size_t tx_buffer_size) {
  std::string bound_port;

  //If no address or port defined, then use the default
  if (local_addr.empty() && (local_port.empty() || (local_port == "0"))) {
    local_addr = kDefaultAddress;
  }

  auto* local_info = agora_comm::GetAddressInfo(local_addr, local_port);
  if (kDebugPrintUdpInit) {
    agora_comm::PrintAddressInfo(local_info);
  }

  /// loop through all the results and bind to the first we can
  for (auto* check = local_info; check != nullptr; check = check->ai_next) {
    ///NON blocking by default
    sock_fd_ = ::socket(check->ai_family, check->ai_socktype | SOCK_NONBLOCK,
                        check->ai_protocol);
    if (sock_fd_ == -1) {
      AGORA_LOG_ERROR("UDPComm: Failed to create local socket. errno = %s\n",
                      std::strerror(errno));
    } else {
      const auto bind_status =
          ::bind(sock_fd_, check->ai_addr, check->ai_addrlen);
      if (bind_status != 0) {
        AGORA_LOG_ERROR("UDPComm: Failed to bind local socket %d. errno = %s\n",
                        sock_fd_, std::strerror(errno));
        ::close(sock_fd_);
        sock_fd_ = -1;
      } else {  //Success
        if (kDebugPrintUdpInit) {
          const auto family = check->ai_family;
          AGORA_LOG_INFO(
              "UDPComm: Using address with family : %s (%d) type %d, "
              "protocol %d, flags %d\n",
              (family == AF_PACKET)  ? "AF_PACKET"
              : (family == AF_INET)  ? "AF_INET"
              : (family == AF_INET6) ? "AF_INET6"
                                     : "???",
              check->ai_family, check->ai_socktype, check->ai_protocol,
              check->ai_flags);
          if (family == AF_INET) {
            [[maybe_unused]] auto* address_ptr =
                &((sockaddr_in*)check->ai_addr)->sin_addr;
            [[maybe_unused]] auto* port_ptr =
                &((sockaddr_in*)check->ai_addr)->sin_port;
            [[maybe_unused]] auto* family_ptr =
                &((sockaddr_in*)check->ai_addr)->sin_family;
            [[maybe_unused]] char address_buffer[INET_ADDRSTRLEN];
            AGORA_LOG_INFO("Ipv4 Address:  %s, Port %d, Family %d \n",
                           ::inet_ntop(family, address_ptr, address_buffer,
                                       sizeof(address_buffer)),
                           ntohs(*port_ptr), *family_ptr);
            bound_port = std::to_string(ntohs(*port_ptr));
          } else if (family == AF_INET6) {
            [[maybe_unused]] auto* address_ptr =
                &((sockaddr_in6*)check->ai_addr)->sin6_addr;
            [[maybe_unused]] auto* port_ptr =
                &((sockaddr_in6*)check->ai_addr)->sin6_port;
            [[maybe_unused]] auto* family_ptr =
                &((sockaddr_in6*)check->ai_addr)->sin6_family;
            [[maybe_unused]] char address_buffer[INET6_ADDRSTRLEN];
            AGORA_LOG_INFO("Ipv6 Address:  %s Port %d, Family %d \n",
                           ::inet_ntop(family, address_ptr, address_buffer,
                                       sizeof(address_buffer)),
                           ntohs(*port_ptr), *family_ptr);
            bound_port = std::to_string(ntohs(*port_ptr));
          } else {
            AGORA_LOG_ERROR(
                "UDPComm: Bound address with unsupported family %d\n", family);
          }
        }
        break;
      }
    }
  }
  ::freeaddrinfo(local_info);
  if (sock_fd_ == -1) {
    throw std::runtime_error(
        "UDPComm: Failed to create local socket. errno = " +
        std::string(std::strerror(errno)));
  }

  // Set rx buffer size
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
        throw std::runtime_error("UDPComm: Failed to set RX buffer size.");
      }
    }

    sock_ret = ::getsockopt(sock_fd_, SOL_SOCKET, SO_RCVBUF, &actual_buf_size,
                            &actual_buf_storage_size);

    // Linux likes to return 2* the buffer size
    if ((actual_buf_size != desired_buf_size) &&
        (actual_buf_size != (desired_buf_size * 2))) {
      AGORA_LOG_WARN(
          "UDPComm:  Error setting RX buffer size to %zu actual size %d with "
          "status %d\n",
          rx_buffer_size, actual_buf_size, sock_ret);
    }
  }

  // Set tx buffer size
  if (tx_buffer_size != 0) {
    const unsigned int desired_buf_size =
        static_cast<unsigned int>(tx_buffer_size);
    unsigned int actual_buf_size;
    ::socklen_t actual_buf_storage_size = sizeof(actual_buf_size);

    auto sock_ret = ::getsockopt(sock_fd_, SOL_SOCKET, SO_SNDBUF,
                                 &actual_buf_size, &actual_buf_storage_size);

    if (sock_ret < 0 || (actual_buf_size != desired_buf_size)) {
      actual_buf_size = desired_buf_size;
      sock_ret = ::setsockopt(sock_fd_, SOL_SOCKET, SO_SNDBUF, &actual_buf_size,
                              actual_buf_storage_size);

      if (sock_ret != 0) {
        throw std::runtime_error("UDPComm: Failed to set TX buffer size.");
      }
    }

    sock_ret = ::getsockopt(sock_fd_, SOL_SOCKET, SO_SNDBUF, &actual_buf_size,
                            &actual_buf_storage_size);

    // Linux likes to return 2* the buffer size
    if ((actual_buf_size != desired_buf_size) &&
        (actual_buf_size != (desired_buf_size * 2))) {
      AGORA_LOG_WARN(
          "UDPComm:  Error setting TX buffer size to %zu actual size %d with "
          "status %d\n",
          rx_buffer_size, actual_buf_size, sock_ret);
    }
  }

  AGORA_LOG_INFO("UDPComm socket %d created %s : %s requested port %s\n",
                 sock_fd_, local_addr.c_str(), bound_port.c_str(),
                 local_port.c_str());
}

UDPComm::~UDPComm() {
  for (const auto& kv : addrinfo_map_) {
    ::freeaddrinfo(kv.second);
  }
  addrinfo_map_.clear();
  if (sock_fd_ != -1) {
    const int close_status = ::close(sock_fd_);
    if (close_status != 0) {
      AGORA_LOG_WARN(
          "WARNING - UDPComm failed while trying to close socket %d with "
          "status %d\n",
          sock_fd_, close_status);
    }
    sock_fd_ = -1;
  }

  if (kDebugPrintUdpInit) {
    AGORA_LOG_INFO("Destroying UDPComm\n");
  }
}

/**
   * @brief Locate and connect to a remote address 
   *
   * @return Connect for DGRAM sockets just indicates a 1:1 socket
   */
ssize_t UDPComm::Connect(const std::string& remote_address,
                         const std::string& remote_port) const {
  /// Find the local interface
  auto* remote_address_info =
      agora_comm::GetAddressInfo(remote_address, remote_port);
  if (kDebugPrintUdpInit) {
    agora_comm::PrintAddressInfo(remote_address_info);
  }

  if (remote_address_info == nullptr) {
    AGORA_LOG_ERROR("Failed to resolve %s", remote_address.c_str());
    throw std::runtime_error("Failed to resolve getaddrinfo()");
  } else if (remote_address_info->ai_next != nullptr) {
    ::freeaddrinfo(remote_address_info);
    AGORA_LOG_ERROR("Too many client sockets found for address %s. port %s.\n",
                    remote_address.c_str(), remote_port.c_str());
    throw std::runtime_error(
        "Connect(): Too many acceptable addresses returned from getaddrinfo");
  }

  // GetAddressInfo() returns a list of address structures, find the correct one to connect
  if (kDebugPrintUdpInit) {
    for (addrinfo* rem_connect = remote_address_info; rem_connect != nullptr;
         rem_connect = rem_connect->ai_next) {
      AGORA_LOG_INFO("Found remote with family %d, type %d, proto %d\n",
                     rem_connect->ai_family, rem_connect->ai_socktype,
                     rem_connect->ai_protocol);
    }
  }
  auto r = ::connect(sock_fd_, remote_address_info->ai_addr,
                     remote_address_info->ai_addrlen);
  ::freeaddrinfo(remote_address_info);
  if (r != 0) {
    AGORA_LOG_ERROR("Failed to connect %s", remote_address.c_str());
    throw std::runtime_error("Failed to connect()");
  }
  return r;
}

ssize_t UDPComm::Connect(const std::string& remote_address,
                         uint16_t remote_port) const {
  std::string remote = std::to_string(remote_port);
  if (remote == "0") {
    remote.clear();
  }
  return Connect(remote_address, std::to_string(remote_port));
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
void UDPComm::Send(const std::string& rem_hostname, uint16_t rem_port,
                   const std::byte* msg, size_t len) {
  const std::string port_str = std::to_string(rem_port);
  const std::string remote_uri = rem_hostname + ":" + port_str;
  ::addrinfo* rem_addrinfo = nullptr;

  if (kDebugPrintUdpSend) {
    AGORA_LOG_INFO("UDPComm sending message to %s:%d of size %zu\n",
                   rem_hostname.c_str(), rem_port, len);
  }

  const auto remote_itr = addrinfo_map_.find(remote_uri);
  if (remote_itr == addrinfo_map_.end()) {
    ::addrinfo hints;
    std::memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;

    const int r = ::getaddrinfo(rem_hostname.c_str(), port_str.c_str(), &hints,
                                &rem_addrinfo);
    if (kDebugPrintUdpSend) {
      agora_comm::PrintAddressInfo(rem_addrinfo);
    }

    if ((r != 0) || (rem_addrinfo == nullptr)) {
      char issue_msg[1000u];
      AGORA_LOG_ERROR(issue_msg,
                      "Send() failed to resolve %s. getaddrinfo error = %s.",
                      remote_uri.c_str(), gai_strerror(r));
      throw std::runtime_error(issue_msg);
    }

    if (kDebugPrintUdpSend) {
      AGORA_LOG_INFO("Send() Fd: %d - Resolving: %s map size %zu\n", sock_fd_,
                     remote_uri.c_str(), addrinfo_map_.size());
    }

    std::pair<std::map<std::string, ::addrinfo*>::iterator, bool>
        map_insert_result;

    {  // Synchronize access to insert for thread safety
      std::scoped_lock map_access(map_insert_access_);
      map_insert_result = addrinfo_map_.insert(
          std::pair<std::string, ::addrinfo*>(remote_uri, rem_addrinfo));
    }

    if (map_insert_result.second == false) {
      ::freeaddrinfo(rem_addrinfo);
      rem_addrinfo = map_insert_result.first->second;
    }
  } else {
    rem_addrinfo = remote_itr->second;
  }
  const ssize_t ret = ::sendto(sock_fd_, msg, len, 0, rem_addrinfo->ai_addr,
                               rem_addrinfo->ai_addrlen);
  if (ret != static_cast<ssize_t>(len)) {
    throw std::runtime_error("sendto() failed. errno = " +
                             std::string(std::strerror(errno)));
  }

  if (enable_recording_flag_) {
    std::scoped_lock map_access(map_insert_access_);
    sent_vec_.emplace_back(reinterpret_cast<const uint8_t*>(msg),
                           reinterpret_cast<const uint8_t*>(msg) + len);
  }
}

/**
   * @brief Send one UDP packet to the connected remote server.
   *
   * @param msg Pointer to the message to send
   * @param len Length in bytes of the message to send
   */
void UDPComm::Send(const std::byte* msg, size_t len) {
  if (kDebugPrintUdpSend) {
    AGORA_LOG_INFO("UDPComm sending message of size %zu\n", len);
  }

  const ssize_t ret = ::send(sock_fd_, msg, len, 0);
  if (ret != static_cast<ssize_t>(len)) {
    AGORA_LOG_ERROR("UDPComm send failed with code %d message %s\n", errno,
                    std::strerror(errno));
    throw std::runtime_error(
        "UDPComm::send() failed. Do you have a connection? " +
        std::string(std::strerror(errno)));
  }

  if (enable_recording_flag_) {
    std::scoped_lock map_access(map_insert_access_);
    sent_vec_.emplace_back(reinterpret_cast<const uint8_t*>(msg),
                           reinterpret_cast<const uint8_t*>(msg) + len);
  }
}

/**
   * @brief Try to receive up to len bytes in buf by default this will not block
   *
   * @return Return the number of bytes received if non-zero bytes are
   * received. If no bytes are received, return zero. If there was an error
   * in receiving, return -1.
   */
ssize_t UDPComm::Recv(std::byte* buf, size_t len) const {
  ssize_t ret = ::recv(sock_fd_, static_cast<void*>(buf), len, 0);

  if (ret == -1) {
    if ((errno == EAGAIN) || (errno == EWOULDBLOCK) ||
        (errno == ECONNREFUSED)) {
      // These errors mean that there's no data to receive
      ret = 0;
    } else {
      AGORA_LOG_ERROR("UDPComm: recv() failed with unexpected error %s(%d)\n",
                      std::strerror(errno), errno);
    }
  } else if (ret == 0) {
    AGORA_LOG_ERROR("UDPComm: recv() failed with return of 0\n");
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
ssize_t UDPComm::Recv(const std::string& src_address, uint16_t src_port,
                      std::byte* buf, size_t len) {
  const std::string port_string = std::to_string(src_port);
  const std::string remote_uri = src_address + ":" + port_string;
  ::addrinfo* rem_addrinfo = nullptr;
  AGORA_LOG_TRACE("Attempting Recv from %s \n", remote_uri.c_str());

  const auto remote_itr = addrinfo_map_.find(remote_uri);
  if (remote_itr == addrinfo_map_.end()) {
    rem_addrinfo = agora_comm::GetAddressInfo(src_address, port_string);
    if (kDebugPrintUdpRecv) {
      agora_comm::PrintAddressInfo(rem_addrinfo);
    }
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
      AGORA_LOG_ERROR("UDPComm: recvfrom() failed with unexpected error %s\n",
                      std::strerror(errno));
    }
  } else if (ret == 0) {
    AGORA_LOG_ERROR("UDPComm: recv() failed with return of 0\n");
  }
  return ret;
}

/**
   * @brief Configures the socket in blocking mode.  Any calls to recv / send
   * will now block
   */
void UDPComm::MakeBlocking(size_t rx_timeout_sec) const {
  int current_flags = ::fcntl(sock_fd_, F_GETFL);
  if (current_flags == -1) {
    throw std::runtime_error("UDPComm: fcntl failed to get flags");
  }
  const int desired_flags = current_flags & (~O_NONBLOCK);

  if (desired_flags != current_flags) {
    int fcntl_status = ::fcntl(sock_fd_, F_SETFL, desired_flags);
    if (fcntl_status == -1) {
      throw std::runtime_error("UDPComm: fcntl failed to set blocking");
    }

    // Verify the flags were properly set
    current_flags = ::fcntl(sock_fd_, F_GETFL);
    if (current_flags == -1) {
      throw std::runtime_error("UDPComm: fcntl failed to get flags");
    } else if (current_flags != desired_flags) {
      throw std::runtime_error("UDPComm: failed to set UDP socket to blocking");
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
      throw std::runtime_error("UDPComm: Failed to set timeout.");
    }
  }
}