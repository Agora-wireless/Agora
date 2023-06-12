/**
 * @file network_utils.cc
 * @brief Helper definations for networking
 */
#include "network_utils.h"

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <linux/if_link.h>
#include <net/if.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <stdexcept>

#include "logger.h"

namespace agora_comm {

static constexpr bool kDebugPrint = false;
/// Allow either - AF_INET6 | AF_INET
static constexpr int kAllowedAiFamily = AF_UNSPEC;

int ListLocalInterfaces() {
  ifaddrs *interfaces = nullptr;
  char address_buffer[INET6_ADDRSTRLEN];

  int status = ::getifaddrs(&interfaces);
  if (status == 0) {
    for (const auto *current_if_address = interfaces;
         current_if_address != nullptr;
         current_if_address = current_if_address->ifa_next) {
      const auto family = current_if_address->ifa_addr->sa_family;

      std::printf("Interface Name  :  %s \n", current_if_address->ifa_name);
      std::printf("Address Family  :  %s (%d)\n",
                  (family == AF_PACKET)  ? "AF_PACKET"
                  : (family == AF_INET)  ? "AF_INET"
                  : (family == AF_INET6) ? "AF_INET6"
                                         : "???",
                  family);

      if ((family == AF_INET) || (family == AF_INET6)) {
        void *address_ptr;
        if (family == AF_INET) {
          address_ptr =
              &((sockaddr_in *)current_if_address->ifa_addr)->sin_addr;
        } else {
          address_ptr =
              &((sockaddr_in6 *)current_if_address->ifa_addr)->sin6_addr;
          auto scope_id =
              ((sockaddr_in6 *)current_if_address->ifa_addr)->sin6_scope_id;

          std::printf("Scope Id        :  %d\n", scope_id);
        }

        std::printf(
            "Internet Address:  %s \n",
            ::inet_ntop(current_if_address->ifa_addr->sa_family, address_ptr,
                        address_buffer, sizeof(address_buffer)));

        if (current_if_address->ifa_netmask != nullptr) {
          const auto netmask_family =
              current_if_address->ifa_netmask->sa_family;

          if ((netmask_family != family) ||
              ((netmask_family != AF_INET) && (netmask_family != AF_INET6))) {
            throw std::runtime_error("Unexpected ifa address family");
          }

          if (netmask_family == AF_INET) {
            address_ptr =
                &((sockaddr_in *)current_if_address->ifa_netmask)->sin_addr;
          } else if (netmask_family == AF_INET6) {
            address_ptr =
                &((sockaddr_in6 *)current_if_address->ifa_netmask)->sin6_addr;
          }
          std::printf("Netmask         :  %s \n",
                      ::inet_ntop(netmask_family, address_ptr, address_buffer,
                                  sizeof(address_buffer)));
        }

        if (current_if_address->ifa_ifu.ifu_broadaddr != nullptr) {
          /* If the ifa_flags field indicates that this is a P2P interface */
          sa_family_t broad_family;
          if ((current_if_address->ifa_flags & IFF_POINTOPOINT) ==
              IFF_POINTOPOINT) {
            broad_family = current_if_address->ifa_ifu.ifu_dstaddr->sa_family;
            std::printf("Destination Addr:  ");
            if (broad_family == AF_INET) {
              address_ptr =
                  &((sockaddr_in *)current_if_address->ifa_ifu.ifu_dstaddr)
                       ->sin_addr;
            } else if (broad_family == AF_INET6) {
              address_ptr =
                  &((sockaddr_in6 *)current_if_address->ifa_ifu.ifu_dstaddr)
                       ->sin6_addr;
            }
          } else {
            broad_family = current_if_address->ifa_ifu.ifu_broadaddr->sa_family;
            std::printf("Broadcast Addr  :  ");
            if (broad_family == AF_INET) {
              address_ptr =
                  &((sockaddr_in *)current_if_address->ifa_ifu.ifu_broadaddr)
                       ->sin_addr;
            } else if (broad_family == AF_INET6) {
              address_ptr =
                  &((sockaddr_in6 *)current_if_address->ifa_ifu.ifu_broadaddr)
                       ->sin6_addr;
            }
          }
          std::printf("%s \n",
                      ::inet_ntop(broad_family, address_ptr, address_buffer,
                                  sizeof(address_buffer)));
        }
      } else if ((family == AF_PACKET) &&
                 (current_if_address->ifa_data != nullptr)) {
        rtnl_link_stats *stats =
            (rtnl_link_stats *)current_if_address->ifa_data;

        std::printf(
            "\t\t\ttx_packets = %10u; rx_packets = %10u\n"
            "\t\t\ttx_bytes   = %10u; rx_bytes   = %10u\n",
            stats->tx_packets, stats->rx_packets, stats->tx_bytes,
            stats->rx_bytes);
      }
      std::printf("\n");
    }

    ::freeifaddrs(interfaces);
    interfaces = nullptr;
  } else {
    std::printf("getifaddrs() failed with errno =  %d %s \n", errno,
                std::strerror(errno));
  }
  return status;
}

std::string GetLocalAddressFromScope(size_t scope_id) {
  ifaddrs *interfaces = nullptr;
  char address_buffer[INET6_ADDRSTRLEN];
  std::string local_address = "";

  int status = ::getifaddrs(&interfaces);
  if (status == 0) {
    for (const auto *current_if_address = interfaces;
         current_if_address != nullptr;
         current_if_address = current_if_address->ifa_next) {
      const auto family = current_if_address->ifa_addr->sa_family;

      if (family == AF_INET6) {
        auto *address_ptr =
            &((sockaddr_in6 *)current_if_address->ifa_addr)->sin6_addr;
        auto found_scope_id =
            ((sockaddr_in6 *)current_if_address->ifa_addr)->sin6_scope_id;

        if (scope_id == found_scope_id) {
          inet_ntop(AF_INET6, address_ptr, address_buffer, INET6_ADDRSTRLEN);
          local_address = address_buffer;
          if (kDebugPrint) {
            std::printf(
                "GetLocalAddressFromScope: Found local address %s --- %d\n",
                local_address.c_str(), found_scope_id);
          }
        }
      }
    }
    ::freeifaddrs(interfaces);
    interfaces = nullptr;
  } else {
    std::printf("getifaddrs() failed with errno =  %d %s \n", errno,
                std::strerror(errno));
  }
  return local_address;
}

::addrinfo *GetAddressInfo(const std::string &address,
                           const std::string &port) {
  ::addrinfo *ret_info = nullptr;
  const char *node;
  const char *service;
  ::addrinfo hints;
  std::memset(&hints, 0u, sizeof(hints));
  hints.ai_family = kAllowedAiFamily;
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_flags = 0;
  if (address.empty()) {
    /// Set node to NULL for loopback or all interfaces
    node = nullptr;
    /// wildcard
    hints.ai_flags |= AI_PASSIVE;
  } else {
    node = address.c_str();
    hints.ai_flags |= AI_NUMERICHOST;
  }

  if (port.empty() || (port == "0")) {
    service = nullptr;
  } else {
    service = port.c_str();
    hints.ai_flags |= AI_NUMERICSERV;
  }
  AGORA_LOG_TRACE("node: %s service: %s\n", node, service);

  if ((service == nullptr) && (node == nullptr)) {
    throw std::runtime_error(
        "GetAddressInfo(): Node and Service cannot both be null");
  }

  /// Any protocol
  hints.ai_protocol = 0;
  hints.ai_canonname = nullptr;
  hints.ai_addr = nullptr;
  hints.ai_next = nullptr;

  const int status = ::getaddrinfo(node, service, &hints, &ret_info);
  if (status < 0) {
    AGORA_LOG_ERROR("getaddrinfo returned error - %s (%d)\n",
                    gai_strerror(status), status);
    throw std::runtime_error("getaddrinfo returned error");
  }
  return ret_info;
}

void PrintAddressInfo(const ::addrinfo *print_info) {
  for (const ::addrinfo *rp = print_info; rp != nullptr; rp = rp->ai_next) {
    const int family = rp->ai_family;
    AGORA_LOG_TRACE(
        "PrintAddressInfo: Found address with family : %s (%d) type %d, "
        "protocol %d, flags %d\n",
        (family == AF_PACKET)  ? "AF_PACKET"
        : (family == AF_INET)  ? "AF_INET"
        : (family == AF_INET6) ? "AF_INET6"
                               : "???",
        rp->ai_family, rp->ai_socktype, rp->ai_protocol, rp->ai_flags);
    if (family == AF_INET) {
      [[maybe_unused]] auto *address_ptr =
          &((sockaddr_in *)rp->ai_addr)->sin_addr;
      [[maybe_unused]] auto *port_ptr = &((sockaddr_in *)rp->ai_addr)->sin_port;
      [[maybe_unused]] auto *family_ptr =
          &((sockaddr_in *)rp->ai_addr)->sin_family;
      [[maybe_unused]] char address_buffer[INET_ADDRSTRLEN];
      AGORA_LOG_TRACE("Ipv4 Address:  %s, Port %d, Family %d \n",
                      ::inet_ntop(family, address_ptr, address_buffer,
                                  sizeof(address_buffer)),
                      ntohs(*port_ptr), *family_ptr);
    } else if (family == AF_INET6) {
      [[maybe_unused]] auto *address_ptr =
          &((sockaddr_in6 *)rp->ai_addr)->sin6_addr;
      [[maybe_unused]] auto *port_ptr =
          &((sockaddr_in6 *)rp->ai_addr)->sin6_port;
      [[maybe_unused]] auto *family_ptr =
          &((sockaddr_in6 *)rp->ai_addr)->sin6_family;
      [[maybe_unused]] char address_buffer[INET6_ADDRSTRLEN];
      AGORA_LOG_TRACE("Ipv6 Address:  %s Port %d, Family %d \n",
                      ::inet_ntop(family, address_ptr, address_buffer,
                                  sizeof(address_buffer)),
                      ntohs(*port_ptr), *family_ptr);
    } else {
      AGORA_LOG_ERROR(
          "PrintAddressInfo: Found address with unsupported family %d\n",
          family);
    }
  }
}

}  // namespace agora_comm