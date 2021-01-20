#ifndef __NET_H
#define __NET_H

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>

int setup_socket_ipv4(int port_id, bool set_sock_size, int sock_buf_size);
int setup_socket_ipv6(int port_id, bool set_sock_size, int sock_buf_size);
void setup_sockaddr_local_ipv4(struct sockaddr_in* local_addr, int port_id);
void setup_sockaddr_local_ipv6(struct sockaddr_in6* local_addr, int port_id);
void setup_sockaddr_remote_ipv4(struct sockaddr_in* remote_addr, int port_id,
                                const char* remote_inet_addr);
void setup_sockaddr_remote_ipv6(struct sockaddr_in6* remote_addr, int port_id,
                                const char* remote_inet_addr);

#endif