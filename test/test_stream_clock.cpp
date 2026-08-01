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
#include "stream_clock.h"

#include <gtest/gtest.h>

#include <vector>

using namespace rtsp_image_transport;

namespace
{
constexpr std::int64_t WALL_NOW = 1700000000000000000LL;
constexpr std::int64_t FRAME = 33333333LL;
}  // namespace

TEST(StreamClock, PassesWallClockStampsThroughUnchanged)
{
    /* A node running on real time already produces usable presentation times,
       and the camera's own capture instants are worth preserving exactly. */
    StreamClock clock;
    for (int i = 0; i < 10; ++i)
    {
        rclcpp::Time stamp(WALL_NOW + i * FRAME);
        rclcpp::Time out = clock.toWallClock(stamp, rclcpp::Time(WALL_NOW + i * FRAME));
        EXPECT_EQ(out.nanoseconds(), stamp.nanoseconds()) << "frame " << i;
    }
    EXPECT_EQ(clock.reanchorCount(), 0u);
}

TEST(StreamClock, LiftsSimulatedTimeOntoRealTime)
{
    /* Simulated time usually starts near zero. Handing that to Live555 as a
       presentation time dates the RTCP sender report to 1970. */
    StreamClock clock;
    rclcpp::Time out = clock.toWallClock(rclcpp::Time(0), rclcpp::Time(WALL_NOW));
    EXPECT_EQ(out.nanoseconds(), WALL_NOW);

    /* Spacing is preserved once anchored */
    for (int i = 1; i < 10; ++i)
    {
        rclcpp::Time next = clock.toWallClock(rclcpp::Time(i * FRAME), rclcpp::Time(WALL_NOW + i * FRAME));
        EXPECT_EQ(next.nanoseconds(), WALL_NOW + i * FRAME) << "frame " << i;
    }
    EXPECT_EQ(clock.reanchorCount(), 0u) << "the initial anchor should not count as a jump";
}

TEST(StreamClock, KeepsMovingForwardWhenABagLoops)
{
    StreamClock clock;
    std::vector<std::int64_t> out;
    /* First pass through the bag */
    for (int i = 0; i < 5; ++i)
    {
        const rclcpp::Time t(WALL_NOW + i * FRAME);
        out.push_back(clock.toWallClock(t, t).nanoseconds());
    }
    /* The bag loops: stamps jump ten seconds back */
    for (int i = 0; i < 5; ++i)
    {
        const rclcpp::Time wall(WALL_NOW + (5 + i) * FRAME);
        const rclcpp::Time looped(WALL_NOW - 10000000000LL + i * FRAME);
        out.push_back(clock.toWallClock(looped, wall).nanoseconds());
    }
    for (std::size_t i = 1; i < out.size(); ++i)
        EXPECT_GT(out[i], out[i - 1]) << "presentation time went backwards at " << i;
    EXPECT_EQ(clock.reanchorCount(), 1u) << "the loop should re-anchor exactly once";

    /* And the original spacing resumes after the jump */
    EXPECT_EQ(out[7] - out[6], FRAME);
    EXPECT_EQ(out[8] - out[7], FRAME);
}

TEST(StreamClock, RepeatedStampsStillAdvance)
{
    /* RTP time stamps must be strictly increasing; two images sharing a stamp
       would otherwise stall the timeline. */
    StreamClock clock;
    std::int64_t previous = clock.toWallClock(rclcpp::Time(WALL_NOW), rclcpp::Time(WALL_NOW)).nanoseconds();
    for (int i = 0; i < 5; ++i)
    {
        std::int64_t out = clock.toWallClock(rclcpp::Time(WALL_NOW), rclcpp::Time(WALL_NOW)).nanoseconds();
        EXPECT_GT(out, previous);
        previous = out;
    }
}

TEST(StreamClock, RebasesStampsFromAnOldRecording)
{
    /* Replaying a bag recorded last week with use_sim_time gives stamps that
       are days old; the RTP timeline has to be about now instead. */
    StreamClock clock;
    const std::int64_t week = 7LL * 24 * 3600 * 1000000000LL;
    rclcpp::Time out = clock.toWallClock(rclcpp::Time(WALL_NOW - week), rclcpp::Time(WALL_NOW));
    EXPECT_EQ(out.nanoseconds(), WALL_NOW);
    rclcpp::Time next = clock.toWallClock(rclcpp::Time(WALL_NOW - week + FRAME), rclcpp::Time(WALL_NOW + FRAME));
    EXPECT_EQ(next.nanoseconds(), WALL_NOW + FRAME) << "spacing lost after rebasing";
}

TEST(StreamClock, ToleratesAModestlyWrongCameraClock)
{
    /* A camera a few minutes out of sync keeps its own timing rather than
       being dragged onto the local clock every frame. */
    StreamClock clock;
    const std::int64_t skew = 120LL * 1000000000LL;
    for (int i = 0; i < 10; ++i)
    {
        rclcpp::Time stamp(WALL_NOW + skew + i * FRAME);
        rclcpp::Time out = clock.toWallClock(stamp, rclcpp::Time(WALL_NOW + i * FRAME));
        EXPECT_EQ(out.nanoseconds(), stamp.nanoseconds());
    }
    EXPECT_EQ(clock.reanchorCount(), 0u);
}

TEST(StreamClock, ResetForgetsTheAnchor)
{
    StreamClock clock;
    clock.toWallClock(rclcpp::Time(0), rclcpp::Time(WALL_NOW));
    EXPECT_TRUE(clock.anchored());
    clock.reset();
    EXPECT_FALSE(clock.anchored());
    /* A restarted encoder starts from a clean pass-through again */
    rclcpp::Time out = clock.toWallClock(rclcpp::Time(WALL_NOW), rclcpp::Time(WALL_NOW));
    EXPECT_EQ(out.nanoseconds(), WALL_NOW);
}

TEST(StreamClock, OutputIsAlwaysStrictlyIncreasing)
{
    /* Whatever the input does, the RTP timeline may never go backwards. */
    StreamClock clock;
    const std::int64_t inputs[] = {WALL_NOW,          WALL_NOW + FRAME, 0,
                                   FRAME,             WALL_NOW,         WALL_NOW - 5000000000LL,
                                   WALL_NOW + FRAME,  WALL_NOW + FRAME, 12345,
                                   WALL_NOW + 2 * FRAME};
    std::int64_t previous = std::numeric_limits<std::int64_t>::min();
    std::int64_t wall = WALL_NOW;
    for (std::int64_t in : inputs)
    {
        std::int64_t out = clock.toWallClock(rclcpp::Time(in), rclcpp::Time(wall)).nanoseconds();
        EXPECT_GT(out, previous) << "input " << in;
        previous = out;
        wall += FRAME;
    }
}
