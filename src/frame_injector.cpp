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
}

void FrameInjector::injectFrame(const FrameDataPtr& frame)
{
    std::lock_guard<std::mutex> lock{frame_queue_mutex_};
    if (is_shutdown_)
        return;
    frame_queue_.push_back(frame);
    trimQueue();
    envir().taskScheduler().triggerEvent(deliver_frame_trigger_, this);
}

/* All the NAL units of one picture carry that picture's stamp, so dropping
   every unit that shares the front stamp drops a whole access unit. Anything
   less would hand Live555 half a picture. */
void FrameInjector::trimQueue()
{
    while (frame_queue_.size() > MAX_QUEUE_LENGTH
           || (frame_queue_.size() > 1
               && (frame_queue_.back()->stamp() - frame_queue_.front()->stamp()).nanoseconds()
                      > MAX_QUEUE_SPAN_NS))
    {
        const rclcpp::Time oldest = frame_queue_.front()->stamp();
        do
        {
            frame_queue_.pop_front();
            ++dropped_;
        } while (!frame_queue_.empty() && frame_queue_.front()->stamp() == oldest);
        /* A single access unit larger than the whole budget would otherwise
           spin here with nothing left to drop. */
        if (frame_queue_.size() <= 1)
            break;
    }
}

std::size_t FrameInjector::droppedFrames() const
{
    std::lock_guard<std::mutex> lock{frame_queue_mutex_};
    return dropped_;
}

/* Deliberately unlocked. deliverFrame() sets this while holding the queue
   mutex and then calls FramedSource::afterGetting(), from which live555 asks
   the framer whether the NAL unit ended an access unit — so this runs inside
   that same call, on the same thread, with the mutex still held. Taking it
   again would deadlock on the first frame. */
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
    std::lock_guard<std::mutex> lock{frame_queue_mutex_};
    if (frame_queue_.empty())
        return;
    FrameDataPtr frame = frame_queue_.front();
    frame_queue_.pop_front();
    /* The picture is finished when nothing behind it shares its stamp. An
       empty queue counts as finished: the publisher enqueues a whole picture
       in one go, so having drained it means the last slice has gone. */
    last_ended_access_unit_ = frame_queue_.empty() || frame_queue_.front()->stamp() != frame->stamp();
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