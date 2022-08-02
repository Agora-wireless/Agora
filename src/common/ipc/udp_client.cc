/**
 * @file udp_client.cc
 * @brief Defination file for the UDPClient class.  This class is used to send messages to a remote server
 */
#include "udp_client.h"

#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring> /* std::strerror, std::memset, std::memcpy */
#include <stdexcept>

#include "logger.h"
#include "network_utils.h"

/// Allow either - AF_INET6 | AF_INET
static const int kDefaultAiFamily = AF_INET;

UDPClient::UDPClient(uint16_t src_port) : UDPClient(std::string(), src_port) {}

/// getaddrinfo() -> socket -> bind (for local address/port assignment)
UDPClient::UDPClient(std::string src_addr, uint16_t src_port) {
  std::string port_string = "";
  ///0 indicates that we are not assigning a port
  if (src_port != 0) {
    port_string = std::to_string(src_port);
  }

  if (src_addr.empty() && port_string.empty()) {
    ::addrinfo my_info;
    ///Default to IPV4, don't bind
    std::memset(&my_info, 0, sizeof(my_info));
    my_info.ai_family = kDefaultAiFamily;
    my_info.ai_socktype = SOCK_DGRAM;
    my_info.ai_protocol = IPPROTO_UDP;
    my_info.ai_next = nullptr;

    if (kDebugPrintUdpClientInit) {
      AGORA_LOG_INFO("UDPClient: Init to default family and port\n");
    }

    sock_fd_ = ::socket(my_info.ai_family, my_info.ai_socktype | SOCK_NONBLOCK,
                        my_info.ai_protocol);
  } else {
    auto local_info = agora_comm::GetAddressInfo(src_addr, port_string);
    if (kDebugPrintUdpClientInit) {
      agora_comm::PrintAddressInfo(local_info);
    }

    /// loop through all the results and bind to the first we can
    for (auto* check = local_info; check != nullptr; check = check->ai_next) {
      ///NON blocking by default
      sock_fd_ = ::socket(check->ai_family, check->ai_socktype | SOCK_NONBLOCK,
                          check->ai_protocol);
      if (sock_fd_ == -1) {
        AGORA_LOG_ERROR(
            "UDPClient: Failed to create local socket. errno = %s\n",
            std::strerror(errno));
      } else {
        const auto bind_status =
            ::bind(sock_fd_, check->ai_addr, check->ai_addrlen);
        if (bind_status != 0) {
          AGORA_LOG_ERROR(
              "UDPClient: Failed to bind local socket %d. errno = %s\n",
              sock_fd_, std::strerror(errno));
          ::close(sock_fd_);
          sock_fd_ = -1;
        } else {  //Success
          break;
        }
      }
    }
    ::freeaddrinfo(local_info);
  }
  if (sock_fd_ == -1) {
    throw std::runtime_error(
        "UDPClient: Failed to create local socket. errno = " +
        std::string(std::strerror(errno)));
  }

  if (kDebugPrintUdpClientInit) {
    AGORA_LOG_INFO("UDP Client socket %d created %s : %s\n", sock_fd_,
                   src_addr.c_str(), port_string.c_str());
  }
}

UDPClient::~UDPClient() {
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
void UDPClient::Send(const std::string& rem_hostname, uint16_t rem_port,
                     const uint8_t* msg, size_t len) {
  const std::string port_str = std::to_string(rem_port);
  const std::string remote_uri = rem_hostname + ":" + port_str;
  ::addrinfo* rem_addrinfo = nullptr;

  if (kDebugPrintUdpClientSend) {
    AGORA_LOG_INFO("UDPClient sending message to %s:%d of size %zu\n",
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
    agora_comm::PrintAddressInfo(rem_addrinfo);

    if ((r != 0) || (rem_addrinfo == nullptr)) {
      char issue_msg[1000u];
      AGORA_LOG_ERROR(issue_msg,
                      "Client failed to resolve %s. getaddrinfo error = %s.",
                      remote_uri.c_str(), gai_strerror(r));
      throw std::runtime_error(issue_msg);
    }

    if (kDebugPrintUdpClientInit) {
      AGORA_LOG_INFO("Fd: %d - Resolving: %s map size %zu\n", sock_fd_,
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
    sent_vec_.emplace_back(msg, msg + len);
  }
}

/**
   * @brief Send one UDP packet to the connected remote server.
   *
   * @param msg Pointer to the message to send
   * @param len Length in bytes of the message to send
   */
void UDPClient::Send(const uint8_t* msg, size_t len) {
  if (kDebugPrintUdpClientSend) {
    AGORA_LOG_INFO("UDPClient sending message of size %zu\n", len);
  }

  const ssize_t ret = ::send(sock_fd_, msg, len, 0);
  if (ret != static_cast<ssize_t>(len)) {
    AGORA_LOG_ERROR("UDPClient send failed with code %d message %s\n", errno,
                    std::strerror(errno));
    throw std::runtime_error(
        "UDPClient::send() failed. Do you have a connection? " +
        std::string(std::strerror(errno)));
  }

  if (enable_recording_flag_) {
    std::scoped_lock map_access(map_insert_access_);
    sent_vec_.emplace_back(msg, msg + len);
  }
}

/**
   * @brief Locate and connect to a remote address 
   *
   * @return Connect for DGRAM sockets just indicates a 1:1 socket
   * 0 for success -1 for failure
   */
ssize_t UDPClient::Connect(const std::string& remote_address,
                           uint16_t remote_port) {
  /// Find the pairing remote
  auto info =
      agora_comm::GetAddressInfo(remote_address, std::to_string(remote_port));

  /// Use the first 1 by default
  const auto connect_status =
      ::connect(sock_fd_, info->ai_addr, info->ai_addrlen);
  ::freeaddrinfo(info);
  if (connect_status != 0) {
    AGORA_LOG_INFO("Error while connecting with status %d - %s\n",
                   connect_status, strerror(errno));
  }
  return connect_status;
}