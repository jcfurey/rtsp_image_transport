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
#ifndef RTSP_IMAGE_TRANSPORT_STREAM_CLOCK_H_
#define RTSP_IMAGE_TRANSPORT_STREAM_CLOCK_H_

#include <rclcpp/time.hpp>

#include <cstdint>

namespace rtsp_image_transport
{

/* Maps ROS image time stamps onto the strictly increasing wall clock timeline
 * that RTP and RTCP require.
 *
 * ROS time is not wall clock time. Under simulation it usually starts near
 * zero, a bag replays whatever was recorded, and looping a bag or resetting the
 * simulation sends it backwards. Feeding any of that to Live555 as a
 * presentation time produces RTCP sender reports dated 1970 and RTP time stamps
 * that jump backwards, which receivers cannot follow.
 *
 * The mapping is an offset that is re-anchored whenever the input goes
 * backwards or lands implausibly far from real time. Between re-anchors the
 * spacing of the stamps is preserved exactly, so genuine frame timing survives.
 */
class StreamClock
{
public:
    /* Re-anchor when the mapped stamp differs from real time by more than this.
       Generous enough that a camera whose clock is merely a few minutes off
       keeps its own timing. */
    static constexpr std::int64_t MAX_WALL_CLOCK_OFFSET_NS = 3600LL * 1000000000LL;
    /* Minimum spacing inserted when stamps repeat or move backwards */
    static constexpr std::int64_t MIN_STEP_NS = 1000000LL;

    /* `now` is the current wall clock, passed in so the caller controls the
       clock source and tests stay deterministic. */
    rclcpp::Time toWallClock(const rclcpp::Time& stamp, const rclcpp::Time& now) noexcept;
    /* Forgets the anchor, e.g. when the encoder is restarted */
    void reset() noexcept;
    bool anchored() const noexcept
    {
        return anchored_;
    }
    /* How often the timeline had to be re-anchored; a growing count means the
       incoming stamps keep jumping. */
    std::uint64_t reanchorCount() const noexcept
    {
        return reanchor_count_;
    }

private:
    bool anchored_ = false;
    std::int64_t offset_ns_ = 0;
    std::int64_t last_output_ns_ = 0;
    std::uint64_t reanchor_count_ = 0;
};

}  // namespace rtsp_image_transport

#endif
