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
#include "stream_clock.h"

#include <cstdlib>

namespace rtsp_image_transport
{

void StreamClock::reset() noexcept
{
    anchored_ = false;
    offset_ns_ = 0;
    last_output_ns_ = 0;
}

rclcpp::Time StreamClock::toWallClock(const rclcpp::Time& stamp, const rclcpp::Time& now) noexcept
{
    const std::int64_t stamp_ns = stamp.nanoseconds();
    const std::int64_t now_ns = now.nanoseconds();

    if (!anchored_)
    {
        /* Start out as a pass-through, so a node running on wall clock time
           keeps the exact capture times its camera reported. */
        offset_ns_ = 0;
        anchored_ = true;
        last_output_ns_ = stamp_ns;
        if (std::llabs(stamp_ns - now_ns) > MAX_WALL_CLOCK_OFFSET_NS)
        {
            offset_ns_ = now_ns - stamp_ns;
            last_output_ns_ = now_ns;
        }
        return rclcpp::Time(last_output_ns_);
    }

    std::int64_t output_ns = stamp_ns + offset_ns_;
    if (output_ns <= last_output_ns_ || std::llabs(output_ns - now_ns) > MAX_WALL_CLOCK_OFFSET_NS)
    {
        /* The ROS clock jumped: a bag looped, the simulation was reset, or the
           publisher was restarted. Re-anchor on real time and carry on from
           there rather than stalling the timeline. */
        const std::int64_t candidate = now_ns > last_output_ns_ ? now_ns : last_output_ns_ + MIN_STEP_NS;
        offset_ns_ = candidate - stamp_ns;
        output_ns = candidate;
        ++reanchor_count_;
    }
    last_output_ns_ = output_ns;
    return rclcpp::Time(output_ns);
}

}  // namespace rtsp_image_transport
