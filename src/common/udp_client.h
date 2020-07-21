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

#include <map>
#include <netdb.h>
#include <stdexcept>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

/// Basic UDP client class that supports sending messages and caches remote
/// addrinfo mappings
class UDPClient {
public:
    UDPClient()
    {
        sock_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock_fd == -1) {
            throw std::runtime_error(
                "UDPClient: Failed to create local socket.");
        }
    }

    UDPClient(const UDPClient&) = delete;

    ~UDPClient()
    {
        for (auto kv : addrinfo_map)
            freeaddrinfo(kv.second);
        if (sock_fd != -1)
            close(sock_fd);
    }

    /**
     * @brief Send one UDP packet to a remote server
     *
     * @param rem_hostname Hostname or IP address of the remote server
     * @param rem_port UDP port that the remote server is listening on
     * @param msg Pointer to the message to send
     * @param len Length in bytes of the message to send
     */
    void send(const std::string rem_hostname, uint16_t rem_port,
        const uint8_t* msg, size_t len)
    {
        std::string remote_uri = rem_hostname + ":" + std::to_string(rem_port);
        struct addrinfo* rem_addrinfo = nullptr;

        if (addrinfo_map.count(remote_uri) != 0) {
            rem_addrinfo = addrinfo_map.at(remote_uri);
        } else {
            char port_str[16];
            snprintf(port_str, sizeof(port_str), "%u", rem_port);

            struct addrinfo hints;
            memset(&hints, 0, sizeof(hints));
            hints.ai_family = AF_INET;
            hints.ai_socktype = SOCK_DGRAM;
            hints.ai_protocol = IPPROTO_UDP;

            int r = getaddrinfo(
                rem_hostname.c_str(), port_str, &hints, &rem_addrinfo);
            if (r != 0 || rem_addrinfo == nullptr) {
                char issue_msg[1000];
                sprintf(issue_msg,
                    "Failed to resolve %s. getaddrinfo error = %s.",
                    remote_uri.c_str(), gai_strerror(r));
                throw std::runtime_error(issue_msg);
            }

            addrinfo_map[remote_uri] = rem_addrinfo;
        }

        ssize_t ret = sendto(sock_fd, msg, len, 0, rem_addrinfo->ai_addr,
            rem_addrinfo->ai_addrlen);
        if (ret != static_cast<ssize_t>(len)) {
            throw std::runtime_error(
                "sendto() failed. errno = " + std::string(strerror(errno)));
        }

        if (enable_recording_flag) {
            sent_vec.push_back(std::vector<uint8_t>(msg, msg + len));
        }
    }

    // Enable recording of all packets sent by this UDP client
    void enable_recording() { enable_recording_flag = true; }

private:
    int sock_fd = -1;

    // A cache mapping hostname:udp_port to addrinfo
    std::map<std::string, struct addrinfo*> addrinfo_map;

    // The list of all packets sent, maintained if recording is enabled
    std::vector<std::vector<uint8_t>> sent_vec;

    bool enable_recording_flag = false; // If true, we record all sent packets
};
