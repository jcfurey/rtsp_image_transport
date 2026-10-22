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

#include <algorithm>
#include <cstdlib>
#include <limits>

namespace rtsp_image_transport
{

void StreamClock::reset() noexcept
{
    anchored_ = false;
    last_input_ns_ = 0;
    last_wall_ns_ = 0;
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
        anchored_ = true;
        last_input_ns_ = stamp_ns;
        last_wall_ns_ = now_ns;
        last_output_ns_ = stamp_ns;
        if (std::llabs(stamp_ns - now_ns) > MAX_WALL_CLOCK_OFFSET_NS)
        {
            last_output_ns_ = now_ns;
        }
        return rclcpp::Time(last_output_ns_);
    }

    const std::int64_t step = stamp_ns - last_input_ns_;
    const std::int64_t wall_step = std::max<std::int64_t>(0, now_ns - last_wall_ns_);
    const bool jumped_forward = step > wall_step && step - wall_step > MAX_FRAME_GAP_NS;
    // Adding an offset to an arbitrary source stamp could overflow int64_t.
    const bool can_advance = step > 0 && step <= std::numeric_limits<std::int64_t>::max() - last_output_ns_;
    std::int64_t output_ns = can_advance ? last_output_ns_ + step : last_output_ns_;
    if (!can_advance || jumped_forward || std::llabs(output_ns - now_ns) > MAX_WALL_CLOCK_OFFSET_NS)
    {
        /* The ROS clock jumped: a bag looped, the simulation was reset, or the
           publisher was restarted. Re-anchor on real time and carry on from
           there rather than stalling the timeline. */
        const std::int64_t candidate =
            now_ns > last_output_ns_
                ? now_ns
                : last_output_ns_ +
                      std::min(MIN_STEP_NS, std::numeric_limits<std::int64_t>::max() - last_output_ns_);
        output_ns = candidate;
        ++reanchor_count_;
    }
    last_output_ns_ = output_ns;
    last_input_ns_ = stamp_ns;
    last_wall_ns_ = now_ns;
    return rclcpp::Time(output_ns);
}

}  // namespace rtsp_image_transport
