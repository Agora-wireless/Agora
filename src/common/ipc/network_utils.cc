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

namespace agora_comm {

static constexpr bool kDebugPrint = false;

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

}  // namespace agora_comm