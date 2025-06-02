/****************************************************************************
 *
 * rtsp_image_transport
 * Copyright © 2021-2025 Fraunhofer FKIE
 * Author: Timo Röhling
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/
#include "host_override.h"

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstring>

namespace rtsp_image_transport
{

int create_host_override_socket(const char* node, bool numeric)
{
    if (!node)
        return -1;
    struct addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = 0;
    hints.ai_flags = AI_ADDRCONFIG;
    if (numeric)
        hints.ai_flags |= AI_NUMERICHOST;
    struct addrinfo* results;
    int sock = -1;
    if (getaddrinfo(node, nullptr, &hints, &results) == 0)
    {
        for (struct addrinfo* cur = results; cur != nullptr; cur = cur->ai_next)
        {
            sock = socket(cur->ai_family, cur->ai_socktype, cur->ai_protocol);
            if (sock < 0)
                continue;
            if (bind(sock, cur->ai_addr, cur->ai_addrlen) >= 0)
                break;
            close(sock);
            sock = -1;
        }
        freeaddrinfo(results);
    }
    return sock;
}

const void* addressBits(const struct sockaddr* sa)
{
    switch (sa->sa_family)
    {
        case AF_INET:
            return &reinterpret_cast<const struct sockaddr_in*>(sa)->sin_addr;
        case AF_INET6:
            return &reinterpret_cast<const struct sockaddr_in6*>(sa)->sin6_addr;
    }
    return nullptr;
}

std::string socket_bound_address(int sock)
{
    struct sockaddr_storage ss;
    struct sockaddr* sa = reinterpret_cast<struct sockaddr*>(&ss);
    socklen_t slen = sizeof(sockaddr_storage);
    getsockname(sock, sa, &slen);
    char strbuf[128];
    const char* ip_addr = inet_ntop(sa->sa_family, addressBits(sa), strbuf, sizeof(strbuf));
    return ip_addr ? std::string(ip_addr) : std::string();
}

}  // namespace rtsp_image_transport
