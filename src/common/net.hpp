/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 * 
 */
#ifndef __NET_H
#define __NET_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>

int setup_socket_ipv4(int port_id, int sock_buf_size);
int setup_socket_ipv6(int port_id, int sock_buf_size);
void setup_sockaddr_local_ipv4(struct sockaddr_in *local_addr, int port_id);
void setup_sockaddr_local_ipv6(struct sockaddr_in6 *local_addr, int port_id);
void setup_sockaddr_remote_ipv4(struct sockaddr_in *remote_addr, int port_id, const char *remote_inet_addr);
void setup_sockaddr_remote_ipv6(struct sockaddr_in6 *remote_addr, int port_id, const char *remote_inet_addr);

#endif 