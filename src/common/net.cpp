#include "net.hpp"

#include <cerrno>
#include <cstdio>  /* std::printf */
#include <cstdlib> /* std::exit */
#include <cstring> /* std::strerror, std::memset, std::memcpy */

void SetSocketBufSize(int socket_local, int sock_buf_size) {
  // use SO_REUSEPORT option, so that multiple sockets could receive packets
  // simultaneously, though the load is not balance
  int optval = 1;
  setsockopt(socket_local, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));

  // sock_buf_size = 1024*1024*64*8-1;
  if (setsockopt(socket_local, SOL_SOCKET, SO_RCVBUF, &sock_buf_size,
                 sizeof(sock_buf_size)) < 0) {
    std::printf("Error setting buffer size to %d\n", sock_buf_size);
    std::exit(-1);
  }
}

int SetupSocketIpv4(int port_id, bool set_sock_size, int sock_buf_size) {
  struct sockaddr_in local_addr;
  local_addr.sin_family = AF_INET;
  local_addr.sin_port = htons(port_id);
  local_addr.sin_addr.s_addr = INADDR_ANY;

  int socket_local;
  if ((socket_local = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {  // UDP socket
    std::printf("ERROR: cannot create IPV4 socket\n");
    std::exit(0);
  }

  if (set_sock_size) SetSocketBufSize(socket_local, sock_buf_size);

  if (bind(socket_local, (struct sockaddr*)&local_addr, sizeof(local_addr)) !=
      0) {
    std::fprintf(stderr, "Socket bind failed: %s", std::strerror(errno));
    std::exit(0);
  }
  return socket_local;
}

int SetupSocketIpv6(int port_id, bool set_sock_size, int sock_buf_size) {
  struct sockaddr_in6 local_addr;
  local_addr.sin6_family = AF_INET6;
  local_addr.sin6_port = htons(port_id);
  local_addr.sin6_addr = in6addr_any;

  int socket_local;
  if ((socket_local = socket(AF_INET6, SOCK_DGRAM, 0)) < 0) {  // UDP socket
    std::printf("ERROR: cannot create IPV6 socket\n");
    std::exit(0);
  } else {
    std::printf("Created IPV6 socket on port %d\n", port_id);
  }
  if (set_sock_size) SetSocketBufSize(socket_local, sock_buf_size);

  if (bind(socket_local, (struct sockaddr*)&local_addr, sizeof(local_addr)) !=
      0) {
    std::fprintf(stderr, "Socket bind failed: %s", std::strerror(errno));
    std::exit(0);
  }
  return socket_local;
}

void SetupSockaddrLocalIpv4(struct sockaddr_in* local_addr, int port_id) {
  (*local_addr).sin_family = AF_INET;
  (*local_addr).sin_port = htons(port_id);
  (*local_addr).sin_addr.s_addr = INADDR_ANY;
  std::memset((*local_addr).sin_zero, 0, sizeof((*local_addr).sin_zero));
}

void SetupSockaddrLocalIpv6(struct sockaddr_in6* local_addr, int port_id) {
  (*local_addr).sin6_family = AF_INET6;
  (*local_addr).sin6_port = htons(port_id);
  (*local_addr).sin6_addr = in6addr_any;
}

void SetupSockaddrRemoteIpv4(struct sockaddr_in* remote_addr, int port_id,
                             const char* remote_inet_addr) {
  (*remote_addr).sin_family = AF_INET;
  (*remote_addr).sin_port = htons(port_id);
  (*remote_addr).sin_addr.s_addr = inet_addr(remote_inet_addr);
  std::memset((*remote_addr).sin_zero, 0, sizeof((*remote_addr).sin_zero));
}

void SetupSockaddrRemoteIpv6(struct sockaddr_in6* remote_addr, int port_id,
                             const char* remote_inet_addr) {
  (*remote_addr).sin6_family = AF_INET6;
  (*remote_addr).sin6_port = htons(port_id);
  inet_pton(AF_INET6, remote_inet_addr, &((*remote_addr).sin6_addr));
}
