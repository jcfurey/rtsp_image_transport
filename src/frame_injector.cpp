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
#include "frame_injector.h"

#include <algorithm>

namespace rtsp_image_transport
{

FrameInjector* FrameInjector::createNew(UsageEnvironment& env)
{
    return new FrameInjector(env);
}

FrameInjector::FrameInjector(UsageEnvironment& env)
    : FramedSource(env), is_shutdown_(false),
      deliver_frame_trigger_(envir().taskScheduler().createEventTrigger(
          FrameInjector::deliverFrameEvent))
{
}

FrameInjector::~FrameInjector()
{
    shutdown();
    envir().taskScheduler().deleteEventTrigger(deliver_frame_trigger_);
}

void FrameInjector::shutdown()
{
    std::lock_guard<std::mutex> lock{frame_queue_mutex_};
    is_shutdown_ = true;
    frame_queue_.clear();
    queued_nals_ = 0;
}

void FrameInjector::injectAccessUnit(const std::vector<FrameDataPtr>& frames)
{
    if (frames.empty())
        return;
    std::lock_guard<std::mutex> lock{frame_queue_mutex_};
    if (is_shutdown_)
        return;
    frame_queue_.push_back(AccessUnit{frames});
    queued_nals_ += frames.size();
    trimQueue();
    envir().taskScheduler().triggerEvent(deliver_frame_trigger_, this);
}

/* Drop only access units that Live555 has not started delivering. If the front
   unit is in progress, preserve it and discard complete units behind it; the
   newest unit is retained so the client catches up as soon as the active one
   finishes. */
void FrameInjector::trimQueue()
{
    while (frame_queue_.size() > 1)
    {
        const std::size_t first_droppable = frame_queue_.front().next == 0 ? 0 : 1;
        if (first_droppable >= frame_queue_.size() - 1)
            break;
        const bool too_many = queued_nals_ > MAX_QUEUE_LENGTH;
        const bool too_old =
            (frame_queue_.back().stamp() - frame_queue_[first_droppable].stamp()).nanoseconds()
            > MAX_QUEUE_SPAN_NS;
        if (!too_many && !too_old)
            break;

        const std::size_t count = frame_queue_[first_droppable].remaining();
        queued_nals_ -= count;
        dropped_ += count;
        frame_queue_.erase(frame_queue_.begin() + static_cast<std::ptrdiff_t>(first_droppable));
    }
}

std::size_t FrameInjector::droppedFrames() const
{
    std::lock_guard<std::mutex> lock{frame_queue_mutex_};
    return dropped_;
}

/* Written and read on the Live555 loop thread: afterGetting() asks the framer
   for this value synchronously after deliverFrame() sets it. */
bool FrameInjector::lastDeliveryEndedAccessUnit() const noexcept
{
    return last_ended_access_unit_;
}

void FrameInjector::doGetNextFrame()
{
    if (!is_shutdown_)
    {
        deliverFrame();
    }
    else
    {
        handleClosure(this);
    }
}

void FrameInjector::deliverFrame()
{
    if (!isCurrentlyAwaitingData())
        return;
    FrameDataPtr frame;
    {
        std::lock_guard<std::mutex> lock{frame_queue_mutex_};
        if (frame_queue_.empty())
            return;
        AccessUnit& access_unit = frame_queue_.front();
        frame = access_unit.frames[access_unit.next++];
        --queued_nals_;
        last_ended_access_unit_ = access_unit.next == access_unit.frames.size();
        if (last_ended_access_unit_)
            frame_queue_.pop_front();
    }
    if (frame->length() <= fMaxSize)
    {
        fFrameSize = frame->length();
    }
    else
    {
        fFrameSize = fMaxSize;
        fNumTruncatedBytes = frame->length() - fMaxSize;
    }
    uint64_t nsec = frame->stamp().nanoseconds();
    fPresentationTime.tv_sec = nsec / 1000000000ull;
    fPresentationTime.tv_usec = (nsec % 1000000000ull) / 1000ull;
    std::copy_n(frame->data(), fFrameSize, fTo);
    FramedSource::afterGetting(this);
}

void FrameInjector::deliverFrameEvent(void* instance)
{
    static_cast<FrameInjector*>(instance)->deliverFrame();
}

}  // namespace rtsp_image_transport
