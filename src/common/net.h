#ifndef NET_H_
#define NET_H_

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>

int SetupSocketIpv4(int port_id, bool set_sock_size, int sock_buf_size);
int SetupSocketIpv6(int port_id, bool set_sock_size, int sock_buf_size);
void SetupSockaddrLocalIpv4(struct sockaddr_in* local_addr, int port_id);
void SetupSockaddrLocalIpv6(struct sockaddr_in6* local_addr, int port_id);
void SetupSockaddrRemoteIpv4(struct sockaddr_in* remote_addr, int port_id,
                             const char* remote_inet_addr);
void SetupSockaddrRemoteIpv6(struct sockaddr_in6* remote_addr, int port_id,
                             const char* remote_inet_addr);

#endif  // NET_H_