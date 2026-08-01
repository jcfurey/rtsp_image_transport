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
#include "event_loop.h"

namespace rtsp_image_transport
{

namespace
{

void reclaim_env(UsageEnvironment* env)
{
    if (env)
        env->reclaim();
}

/* How long stop() waits for the loop to come back before giving up. Reaching
   this means Live555 is wedged in a socket handler; carrying on and leaking the
   thread beats blocking a ROS executor forever. */
constexpr std::chrono::seconds STOP_TIMEOUT{5};

}  // namespace

std::shared_ptr<EventLoop> EventLoop::create(unsigned scheduler_granularity_us)
{
    std::shared_ptr<EventLoop> loop(new EventLoop(scheduler_granularity_us));
    std::thread thread([loop]() { loop->run(); });
    {
        std::lock_guard<std::mutex> lock{loop->mutex_};
        loop->thread_id_ = thread.get_id();
    }
    /* Detached on purpose: nothing ever joins this thread, which is what makes
       it safe for the loop to outlive its last external owner. */
    thread.detach();
    return loop;
}

EventLoop::EventLoop(unsigned scheduler_granularity_us)
    : scheduler_(BasicTaskScheduler::createNew(scheduler_granularity_us)),
      env_(BasicUsageEnvironment::createNew(*scheduler_), reclaim_env), quit_flag_(0), running_(true)
{
}

EventLoop::~EventLoop()
{
    /* If the loop is still going, this is the destructor running on its own
       thread; setting the flag lets it return once the current callback ends. */
    quit_flag_ = 1;
}

void EventLoop::run()
{
    env_->taskScheduler().doEventLoop(&quit_flag_);
    std::lock_guard<std::mutex> lock{mutex_};
    running_ = false;
    finished_.notify_all();
}

UsageEnvironment& EventLoop::env() noexcept
{
    return *env_;
}

bool EventLoop::isCurrentThread() const noexcept
{
    std::lock_guard<std::mutex> lock{mutex_};
    return thread_id_ == std::this_thread::get_id();
}

void EventLoop::stop() noexcept
{
    quit_flag_ = 1;
    std::unique_lock<std::mutex> lock{mutex_};
    if (thread_id_ == std::this_thread::get_id())
        return;
    finished_.wait_for(lock, STOP_TIMEOUT, [this] { return !running_; });
}

}  // namespace rtsp_image_transport
