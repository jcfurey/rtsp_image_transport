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
#ifndef RTSP_IMAGE_TRANSPORT_FRAME_INJECTOR_H_
#define RTSP_IMAGE_TRANSPORT_FRAME_INJECTOR_H_

#include "frame_data.h"

#include <FramedSource.hh>

#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>

namespace rtsp_image_transport
{

/* Hands encoded NAL units from a ROS executor thread to the Live555 loop.
 *
 * One of these exists per attached client, so a client that cannot keep up
 * backs up its own queue and nobody else's. The queue is bounded: Live555
 * drains it only as fast as it can put packets on that client's link, and
 * without a bound a client slower than the encoder makes the queue — and with
 * it the latency of everything behind it — grow for as long as it stays
 * connected. Over the bound the oldest access units are dropped, which costs
 * the client a visible glitch until the next key frame but keeps it live
 * rather than falling ever further behind.
 */
class FrameInjector : public FramedSource
{
public:
    /* Latency budget for one client's send queue. Live555 asks for the next
       NAL unit as soon as it has sent the last, so anything queued beyond this
       is time the client is behind real time. */
    static constexpr std::int64_t MAX_QUEUE_SPAN_NS = 200000000LL;  // 200 ms
    /* Backstop for a stream whose stamps do not advance (a still image, a
       source with a stuck clock), where the span above never triggers. Sized
       well past any sane number of NAL units per picture. */
    static constexpr std::size_t MAX_QUEUE_LENGTH = 4096;

    static FrameInjector* createNew(UsageEnvironment& env);
    ~FrameInjector();
    void shutdown();
    void injectFrame(const FrameDataPtr& frame);
    /* How many NAL units have been dropped to keep the queue inside its
       bounds, for the publisher to report */
    std::size_t droppedFrames() const;

private:
    FrameInjector(UsageEnvironment& env);
    void deliverFrame();
    void doGetNextFrame() override;
    static void deliverFrameEvent(void* instance);
    /* Drops whole access units from the front until the queue is back inside
       its bounds. Call with frame_queue_mutex_ held. */
    void trimQueue();

    bool is_shutdown_;
    EventTriggerId deliver_frame_trigger_;
    mutable std::mutex frame_queue_mutex_;
    std::deque<FrameDataPtr> frame_queue_;
    std::size_t dropped_ = 0;
};

}  // namespace rtsp_image_transport

#endif
