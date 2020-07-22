//	Copyright 2018, Carnegie Mellon University
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#pragma once

#include <netdb.h>
#include <stdexcept>
#include <sys/socket.h>
#include <unistd.h>

/// Basic UDP server class that supports receiving messages
class UDPServer {
public:
    // Initialize a UDP server listening on this UDP port with socket buffer
    // size = rx_buffer_size
    UDPServer(uint16_t port, size_t rx_buffer_size = 0)
        : port(port)
    {
        sock_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock_fd == -1) {
            throw std::runtime_error(
                "UDPServer: Failed to create local socket.");
        }

        // Set buffer size
        if (rx_buffer_size != 0) {
            int ret = setsockopt(sock_fd, SOL_SOCKET, SO_RCVBUF,
                &rx_buffer_size, sizeof(rx_buffer_size));
            if (ret != 0) {
                throw std::runtime_error(
                    "UDPServer: Failed to set RX buffer size.");
            }
        }

        struct sockaddr_in serveraddr;
        serveraddr.sin_family = AF_INET;
        serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);
        serveraddr.sin_port = htons(static_cast<unsigned short>(port));

        int r = bind(sock_fd, reinterpret_cast<struct sockaddr*>(&serveraddr),
            sizeof(serveraddr));
        if (r != 0) {
            throw std::runtime_error("UDPServer: Failed to bind socket to port "
                + std::to_string(port));
        }
    }

    UDPServer() {}
    UDPServer(const UDPServer&) = delete;

    ~UDPServer()
    {
        if (sock_fd != -1)
            close(sock_fd);
    }

    /**
     * @brief Try to receive up to len bytes in buf
     * @return The return value of the underlying recv() syscall
     */
    ssize_t recv_nonblocking(uint8_t* buf, size_t len)
    {
        return recv(sock_fd, static_cast<void*>(buf), len, 0);
    }

private:
    uint16_t port; // The UDP port to listen on
    int sock_fd = -1;
};
