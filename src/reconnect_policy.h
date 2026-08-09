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
#ifndef RTSP_IMAGE_TRANSPORT_RECONNECT_POLICY_H_
#define RTSP_IMAGE_TRANSPORT_RECONNECT_POLICY_H_

#include <algorithm>
#include <chrono>

namespace rtsp_image_transport
{

/* What the subscriber does when a session ends. The values are ordered by how
   eager they are and are compared with >=, so the numbering is part of the
   contract: each level does everything the level below it does. They are also
   the values of the reconnect_policy parameter, so they cannot be renumbered
   without changing what a launch file means. */
enum ReconnectPolicy
{
    ReconnectNever = 0,
    ReconnectOnTimeout = 1,
    ReconnectOnFailure = 2,
    ReconnectAlways = 3,
};

/* The stream stopped producing data but the session never failed. */
inline bool reconnectAfterTimeout(ReconnectPolicy policy) noexcept
{
    return policy >= ReconnectOnTimeout;
}

/* The session failed, or could not be established in the first place. */
inline bool reconnectAfterFailure(ReconnectPolicy policy) noexcept
{
    return policy >= ReconnectOnFailure;
}

/* The server ended the stream cleanly, which for a camera means it is done
   sending — only the most eager policy tries again. */
inline bool reconnectAfterNormalEnd(ReconnectPolicy policy) noexcept
{
    return policy >= ReconnectAlways;
}

/* Doubling backoff, so a camera that is down does not get hammered, clamped so
   the wait between attempts stays bounded. Returns the wait to use for the
   attempt after the one that just used `current`. */
inline std::chrono::milliseconds nextReconnectCooldown(std::chrono::milliseconds current,
                                                       std::chrono::milliseconds maxwait) noexcept
{
    if (maxwait.count() <= 0)
        return std::chrono::milliseconds::zero();
    /* A zero or negative starting point would never grow, leaving the retry
       loop spinning as fast as the executor allows. */
    if (current.count() <= 0)
        return std::min(std::chrono::milliseconds(1), maxwait);
    if (current >= maxwait)
        return maxwait;
    /* Doubling can overflow when maxwait is enormous; clamping first keeps the
       result monotonic rather than wrapping to a negative wait. */
    if (current > maxwait / 2)
        return maxwait;
    return current * 2;
}

}  // namespace rtsp_image_transport

#endif
