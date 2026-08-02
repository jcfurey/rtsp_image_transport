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
#ifndef RTSP_IMAGE_TRANSPORT_EVENT_LOOP_H_
#define RTSP_IMAGE_TRANSPORT_EVENT_LOOP_H_

#include <BasicUsageEnvironment.hh>
#include <liveMedia.hh>

#include <chrono>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

namespace rtsp_image_transport
{

/* Live555 only notices events posted from other threads when its select() call
   returns, which it forces at this interval. The default of 10 ms would add up
   to that much latency to every frame handed over by a ROS thread. */
constexpr unsigned SCHEDULER_GRANULARITY_US = 1000;

/* A Live555 task scheduler together with the thread that runs it.
 *
 * The thread holds a reference to the loop itself, so the loop stays alive
 * until it has actually finished. That matters because Live555 callbacks
 * resurrect a shared_ptr to their owner: if the owner's last external
 * reference is released while a callback is running, the owner is destroyed on
 * this very thread. A plain std::thread member would then be joined from
 * inside itself, which terminates the process.
 */
class EventLoop
{
public:
    static std::shared_ptr<EventLoop> create(unsigned scheduler_granularity_us = SCHEDULER_GRANULARITY_US);
    ~EventLoop();
    EventLoop(const EventLoop&) = delete;
    EventLoop(EventLoop&&) = delete;
    EventLoop& operator=(const EventLoop&) = delete;
    EventLoop& operator=(EventLoop&&) = delete;

    UsageEnvironment& env() noexcept;
    /* True when the caller is running inside this event loop */
    bool isCurrentThread() const noexcept;
    /* Asks the loop to finish and waits for it to do so. Returns immediately
       when called from the loop thread, where waiting could never succeed. */
    void stop() noexcept;
    /* Runs fn on the loop thread and waits for it to finish; an exception fn
       throws propagates to the caller. Runs inline when called from the loop
       thread itself (waiting could never succeed there) or once the loop has
       stopped (nothing dispatches concurrently then). Built on triggerEvent,
       the one Live555 entry point that is safe from foreign threads —
       everything else (sending commands, closing Media, task scheduling)
       must happen on this thread. */
    void post(const std::function<void()>& fn);

private:
    struct PostedTask;
    explicit EventLoop(unsigned scheduler_granularity_us);
    void run();
    static void dispatchPosted(void* data);
    void drainPosted() noexcept;

    /* The order matters: the environment has to be reclaimed before the
       scheduler it refers to is destroyed. */
    std::shared_ptr<TaskScheduler> scheduler_;
    std::shared_ptr<UsageEnvironment> env_;
    EventLoopWatchVariable quit_flag_;
    EventTriggerId post_trigger_;
    mutable std::mutex mutex_;
    std::condition_variable finished_;
    bool running_;
    std::thread::id thread_id_;
    std::deque<std::shared_ptr<PostedTask>> posted_;
};

}  // namespace rtsp_image_transport

#endif
