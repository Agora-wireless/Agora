// Copyright (c) 2015-2017 Josh Blum
// SPDX-License-Identifier: BSL-1.0

// ** This header should be included first, to avoid compile errors.
// ** At least in the case of the windows header files.

// This header helps to abstract network differences between platforms.
// Including the correct headers for various network APIs.
// And providing various typedefs and definitions when missing.

#pragma once

/***********************************************************************
 * unix socket headers
 **********************************************************************/

#include <unistd.h>  //close
#define closesocket close
#include <arpa/inet.h>  //inet_ntop
#include <fcntl.h>      //fcntl and constants
#include <ifaddrs.h>    //getifaddrs
#include <net/if.h>     //if_nametoindex
#include <netdb.h>      //addrinfo
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/types.h>

/***********************************************************************
 * htonll and ntohll for GCC
 **********************************************************************/
#if defined(__GNUC__) && !defined(htonll)
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define htonll(x) __builtin_bswap64(x)
#else  //big endian
#define htonll(x) (x)
#endif  //little endian
#endif  //__GNUC__ and not htonll

#if defined(__GNUC__) && !defined(ntohll)
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define ntohll(x) __builtin_bswap64(x)
#else  //big endian
#define ntohll(x) (x)
#endif  //little endian
#endif  //__GNUC__ and not ntohll

/***********************************************************************
 * socket type definitions
 **********************************************************************/
#ifndef INVALID_SOCKET
#define INVALID_SOCKET -1
#endif  //INVALID_SOCKET

/***********************************************************************
 * socket errno
 **********************************************************************/
#define SOCKET_ERRNO errno
#define SOCKET_EINPROGRESS EINPROGRESS
#define SOCKET_ETIMEDOUT ETIMEDOUT

/***********************************************************************
 * OSX compatibility
 **********************************************************************/
#if !defined(IPV6_ADD_MEMBERSHIP) && defined(IPV6_JOIN_GROUP)
#define IPV6_ADD_MEMBERSHIP IPV6_JOIN_GROUP
#endif

#if !defined(IPV6_DROP_MEMBERSHIP) && defined(IPV6_LEAVE_GROUP)
#define IPV6_DROP_MEMBERSHIP IPV6_LEAVE_GROUP
#endif
