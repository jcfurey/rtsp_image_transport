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
#include "event_loop.h"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>

using namespace rtsp_image_transport;
using namespace std::chrono_literals;

namespace
{

std::atomic_int task_calls{0};
std::atomic_bool release_ran{false};

void countingTask(void*)
{
    task_calls++;
}

void releaseOwnerTask(void* opaque)
{
    /* Mirrors what happens when the owner of the loop is destroyed from inside
       a Live555 callback: it asks the loop to finish and then releases it, all
       from the loop thread itself. */
    auto* holder = static_cast<std::shared_ptr<EventLoop>*>(opaque);
    (*holder)->stop();
    holder->reset();
    delete holder;
    release_ran = true;
}

}  // namespace

TEST(EventLoop, RunsScheduledTasks)
{
    task_calls = 0;
    std::shared_ptr<EventLoop> loop = EventLoop::create();
    loop->env().taskScheduler().scheduleDelayedTask(0, countingTask, nullptr);
    for (int i = 0; i < 100 && task_calls.load() == 0; ++i)
        std::this_thread::sleep_for(10ms);
    EXPECT_GT(task_calls.load(), 0);
    loop->stop();
}

TEST(EventLoop, StopReturnsPromptlyFromAnotherThread)
{
    std::shared_ptr<EventLoop> loop = EventLoop::create();
    const auto start = std::chrono::steady_clock::now();
    loop->stop();
    const auto elapsed = std::chrono::steady_clock::now() - start;
    EXPECT_LT(elapsed, 3s) << "stop() waited far longer than the loop needs to notice the flag";
    EXPECT_FALSE(loop->isCurrentThread());
}

TEST(EventLoop, SeesCrossThreadEventsQuickly)
{
    /* Frames handed over by a ROS thread are delivered through triggerEvent,
       and the scheduler only notices between select() calls. A slow granularity
       here would show up directly as publishing latency. */
    task_calls = 0;
    std::shared_ptr<EventLoop> loop = EventLoop::create();
    EventTriggerId trigger = loop->env().taskScheduler().createEventTrigger(countingTask);
    ASSERT_NE(trigger, 0u);
    std::this_thread::sleep_for(50ms);

    const auto start = std::chrono::steady_clock::now();
    loop->env().taskScheduler().triggerEvent(trigger, nullptr);
    while (task_calls.load() == 0 && std::chrono::steady_clock::now() - start < 2s)
        std::this_thread::sleep_for(1ms);
    const auto latency = std::chrono::steady_clock::now() - start;

    EXPECT_GT(task_calls.load(), 0) << "the event was never picked up";
    EXPECT_LT(latency, 50ms) << "cross-thread events take too long to be noticed";
    loop->env().taskScheduler().deleteEventTrigger(trigger);
    loop->stop();
}

TEST(EventLoop, SurvivesLosingItsLastReferenceInsideItsOwnThread)
{
    /* Regression guard. Live555 callbacks resurrect a shared_ptr to their
       owner; when the outside world drops its reference at the same moment,
       the owner is destroyed on the loop thread. Joining the loop thread from
       within itself terminates the process, so the loop has to keep itself
       alive instead. */
    release_ran = false;
    std::shared_ptr<EventLoop> loop = EventLoop::create();
    auto* holder = new std::shared_ptr<EventLoop>(loop);
    loop->env().taskScheduler().scheduleDelayedTask(1000, releaseOwnerTask, holder);
    /* The scheduled task now holds the only reference outside the loop thread,
       and releases it from in there. */
    loop.reset();

    for (int i = 0; i < 200 && !release_ran.load(); ++i)
        std::this_thread::sleep_for(10ms);
    EXPECT_TRUE(release_ran.load()) << "the loop thread never got to release the owner";
    /* Give the thread a moment to unwind; reaching this point without an abort
       is the actual assertion. */
    std::this_thread::sleep_for(200ms);
    SUCCEED();
}

TEST(EventLoop, ManyLoopsCanCoexistAndShutDown)
{
    std::vector<std::shared_ptr<EventLoop>> loops;
    for (int i = 0; i < 8; ++i)
        loops.push_back(EventLoop::create());
    for (const auto& loop : loops)
        ASSERT_TRUE(loop);
    for (const auto& loop : loops)
        loop->stop();
    loops.clear();
    SUCCEED();
}
