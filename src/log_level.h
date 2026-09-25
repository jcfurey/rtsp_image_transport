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
#ifndef RTSP_IMAGE_TRANSPORT_LOG_LEVEL_H_
#define RTSP_IMAGE_TRANSPORT_LOG_LEVEL_H_

#include <climits>

extern "C"
{
#include <libavutil/log.h>
}

namespace rtsp_image_transport
{

/* The most verbose FFmpeg level the calling thread currently forwards to ROS.
   Per thread rather than av_log_set_level(): that is global, and two encoders
   or decoders probing at once in one process would restore each other's saved
   level out of order, leaving FFmpeg silenced for good. It also kept the
   errors of every other stream quiet while one of them probed. */
inline thread_local int thread_av_log_cap = INT_MAX;

/* Quietens FFmpeg on this thread for the lifetime of the object. Probing runs
   on the calling thread, which is where its expected failures are logged. */
class TemporaryAvLogLevel
{
public:
    explicit TemporaryAvLogLevel(int level) : old_cap_(thread_av_log_cap)
    {
        thread_av_log_cap = level < old_cap_ ? level : old_cap_;
    }
    ~TemporaryAvLogLevel()
    {
        thread_av_log_cap = old_cap_;
    }
    TemporaryAvLogLevel(const TemporaryAvLogLevel&) = delete;
    TemporaryAvLogLevel& operator=(const TemporaryAvLogLevel&) = delete;

private:
    int old_cap_;
};

}  // namespace rtsp_image_transport

#endif