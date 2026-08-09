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

/* What the subscriber does when a session ends, and how long it waits before
   trying again. The policy is compared with >=, so the whole thing rests on the
   enum's numeric order; an inverted comparison or a renumbered value would
   reconnect when the user asked for never, or give up when they asked for
   always, and neither shows up without a camera that goes away. */

#include "reconnect_policy.h"

#include <gtest/gtest.h>

#include <chrono>

using namespace rtsp_image_transport;
using namespace std::chrono_literals;

/* The full truth table, written out rather than derived, so that changing a
   comparison in the header has to change this file too. */
TEST(ReconnectPolicy, DecidesPerEventAndPolicy)
{
    struct Expectation
    {
        ReconnectPolicy policy;
        bool on_timeout, on_failure, on_normal_end;
    };
    const Expectation table[] = {
        {ReconnectNever, false, false, false},
        {ReconnectOnTimeout, true, false, false},
        {ReconnectOnFailure, true, true, false},
        {ReconnectAlways, true, true, true},
    };

    for (const Expectation& e : table)
    {
        SCOPED_TRACE(static_cast<int>(e.policy));
        EXPECT_EQ(reconnectAfterTimeout(e.policy), e.on_timeout);
        EXPECT_EQ(reconnectAfterFailure(e.policy), e.on_failure);
        EXPECT_EQ(reconnectAfterNormalEnd(e.policy), e.on_normal_end);
    }
}

/* The numbering is the reconnect_policy parameter's value, so a launch file
   saying 2 has to keep meaning "on failure". */
TEST(ReconnectPolicy, ParameterValuesAreStable)
{
    EXPECT_EQ(static_cast<int>(ReconnectNever), 0);
    EXPECT_EQ(static_cast<int>(ReconnectOnTimeout), 1);
    EXPECT_EQ(static_cast<int>(ReconnectOnFailure), 2);
    EXPECT_EQ(static_cast<int>(ReconnectAlways), 3);
}

/* Each level has to be at least as eager as the one below it, which is what
   makes the >= comparisons in the plugin correct. */
TEST(ReconnectPolicy, EagernessIsMonotonic)
{
    const ReconnectPolicy ordered[] = {ReconnectNever, ReconnectOnTimeout, ReconnectOnFailure, ReconnectAlways};
    for (std::size_t i = 1; i < std::size(ordered); ++i)
    {
        SCOPED_TRACE(i);
        EXPECT_GE(reconnectAfterTimeout(ordered[i]), reconnectAfterTimeout(ordered[i - 1]));
        EXPECT_GE(reconnectAfterFailure(ordered[i]), reconnectAfterFailure(ordered[i - 1]));
        EXPECT_GE(reconnectAfterNormalEnd(ordered[i]), reconnectAfterNormalEnd(ordered[i - 1]));
    }
}

TEST(ReconnectBackoff, DoublesUntilItReachesTheCeiling)
{
    const auto maxwait = 30s;
    auto wait = std::chrono::milliseconds(100);
    const std::chrono::milliseconds expected[] = {200ms, 400ms, 800ms, 1600ms, 3200ms, 6400ms, 12800ms, 25600ms};
    for (const auto& want : expected)
    {
        wait = nextReconnectCooldown(wait, maxwait);
        EXPECT_EQ(wait, want);
    }
    /* 51200ms would be over the ceiling, so it clamps instead */
    wait = nextReconnectCooldown(wait, maxwait);
    EXPECT_EQ(wait, std::chrono::milliseconds(maxwait));
}

TEST(ReconnectBackoff, StaysAtTheCeilingOnceThere)
{
    const auto maxwait = 5s;
    auto wait = std::chrono::milliseconds(maxwait);
    for (int i = 0; i < 10; ++i)
    {
        wait = nextReconnectCooldown(wait, maxwait);
        ASSERT_EQ(wait, std::chrono::milliseconds(maxwait)) << "iteration " << i;
    }
}

/* A start above the ceiling must come down to it rather than keep growing. */
TEST(ReconnectBackoff, ClampsAStartLargerThanTheCeiling)
{
    EXPECT_EQ(nextReconnectCooldown(60s, 5s), std::chrono::milliseconds(5s));
}

/* reconnect_minwait comes from a double parameter, so 0 is reachable from a
   launch file. Doubling zero stays zero, which would spin the retry loop as
   fast as the executor allows. */
TEST(ReconnectBackoff, NeverReturnsZeroForANonZeroCeiling)
{
    EXPECT_GT(nextReconnectCooldown(0ms, 30s).count(), 0);
    EXPECT_GT(nextReconnectCooldown(std::chrono::milliseconds(-5), 30s).count(), 0);
    /* And it still converges on the ceiling from there */
    auto wait = nextReconnectCooldown(0ms, 30s);
    for (int i = 0; i < 40; ++i)
        wait = nextReconnectCooldown(wait, 30s);
    EXPECT_EQ(wait, std::chrono::milliseconds(30s));
}

/* A zero ceiling means "do not wait", and must not become an infinite loop of
   ever-doubling waits or a negative one. */
TEST(ReconnectBackoff, HandlesAZeroCeiling)
{
    EXPECT_EQ(nextReconnectCooldown(100ms, 0ms).count(), 0);
    EXPECT_EQ(nextReconnectCooldown(0ms, 0ms).count(), 0);
}

/* The result is a delay handed to a wall timer, so it must never go backwards
   or turn negative through overflow. */
TEST(ReconnectBackoff, IsMonotonicAndNeverNegative)
{
    const std::chrono::milliseconds ceilings[] = {1ms, 100ms, 30s, std::chrono::milliseconds::max() / 2};
    for (const auto& ceiling : ceilings)
    {
        SCOPED_TRACE(ceiling.count());
        auto wait = std::chrono::milliseconds(1);
        for (int i = 0; i < 80; ++i)
        {
            const auto next = nextReconnectCooldown(wait, ceiling);
            ASSERT_GE(next.count(), 0) << "iteration " << i;
            ASSERT_GE(next, wait) << "backoff went backwards at iteration " << i;
            ASSERT_LE(next, ceiling) << "backoff overshot the ceiling at iteration " << i;
            wait = next;
        }
    }
}
