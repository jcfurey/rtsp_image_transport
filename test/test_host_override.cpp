/****************************************************************************
 *
 * rtsp_image_transport
 * Copyright © 2021-2025 Fraunhofer FKIE
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

#include <gtest/gtest.h>

#include <unistd.h>

using namespace rtsp_image_transport;

TEST(HostOverride, BindsToANumericAddress)
{
    int sock = create_host_override_socket("127.0.0.1", /*numeric=*/true);
    ASSERT_GE(sock, 0) << "could not bind to the loopback address";
    EXPECT_EQ(socket_bound_address(sock), "127.0.0.1");
    close(sock);
}

TEST(HostOverride, ResolvesAHostName)
{
    int sock = create_host_override_socket("localhost");
    ASSERT_GE(sock, 0) << "could not resolve localhost";
    const std::string address = socket_bound_address(sock);
    EXPECT_TRUE(address == "127.0.0.1" || address == "::1") << "unexpected address " << address;
    close(sock);
}

TEST(HostOverride, RejectsNullAndUnusableNames)
{
    EXPECT_LT(create_host_override_socket(nullptr), 0);
    EXPECT_LT(create_host_override_socket("host.invalid.example."), 0);
    /* A host name where only a numeric address is accepted must not resolve */
    EXPECT_LT(create_host_override_socket("localhost", /*numeric=*/true), 0);
}

TEST(HostOverride, RejectsAddressNotOnThisMachine)
{
    /* Documentation range from RFC 5737; binding to it must fail rather than
       silently give the RTSP server a bogus address to advertise. */
    EXPECT_LT(create_host_override_socket("192.0.2.1", true), 0);
}
