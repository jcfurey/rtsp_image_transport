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
#include "frame_injector.h"

#include <gtest/gtest.h>

#include <memory>
#include <vector>

using namespace rtsp_image_transport;

namespace
{

constexpr std::int64_t BASE_NS = 1700000000000000000LL;
constexpr std::int64_t FRAME_NS = 33333333LL;

/* Nothing here starts Live555 playing, so the injector never delivers and the
   queue behaves exactly as it does for a client too slow to drain it. */
class FrameInjectorTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        loop_ = EventLoop::create();
        injector_ = FrameInjector::createNew(loop_->env());
    }

    void TearDown() override
    {
        if (injector_)
            injector_->shutdown();
        Medium::close(injector_);
        injector_ = nullptr;
        loop_.reset();
    }

    /* One picture's worth of NAL units, all sharing the picture's stamp */
    void injectPicture(std::int64_t stamp_ns, std::size_t nal_units = 1, std::size_t bytes = 64)
    {
        std::vector<unsigned char> payload(bytes, 0x42);
        for (std::size_t i = 0; i < nal_units; ++i)
        {
            injector_->injectFrame(
                std::make_shared<FrameData>(payload.data(), payload.size(), rclcpp::Time(stamp_ns)));
        }
    }

    std::shared_ptr<EventLoop> loop_;
    FrameInjector* injector_ = nullptr;
};

}  // namespace

TEST_F(FrameInjectorTest, KeepsEverythingWhileTheQueueIsInsideItsBudget)
{
    /* Well under MAX_QUEUE_SPAN_NS: a client keeping up must lose nothing. */
    for (int i = 0; i < 5; ++i)
        injectPicture(BASE_NS + i * FRAME_NS);
    EXPECT_EQ(injector_->droppedFrames(), 0u);
}

TEST_F(FrameInjectorTest, DropsOldestPicturesOnceTheQueueSpansTooMuchTime)
{
    /* A client that never drains used to back up without limit, so the frames
       behind it grew steadily older for as long as it stayed connected. */
    const int frames = 120;  // four seconds at 30 Hz, far past the 200 ms budget
    for (int i = 0; i < frames; ++i)
        injectPicture(BASE_NS + i * static_cast<std::int64_t>(FRAME_NS));
    EXPECT_GT(injector_->droppedFrames(), 0u) << "the queue grew without bound";
    /* What is left has to be the newest end of the stream, within the budget */
    const std::size_t kept = frames - injector_->droppedFrames();
    const std::int64_t span = static_cast<std::int64_t>(kept) * FRAME_NS;
    EXPECT_LE(span, FrameInjector::MAX_QUEUE_SPAN_NS + FRAME_NS)
        << "kept " << kept << " frames, spanning " << span / 1000000 << " ms";
}

TEST_F(FrameInjectorTest, DropsWholeAccessUnitsRatherThanPartOfAPicture)
{
    /* Every NAL unit of a picture carries that picture's stamp. Handing
       Live555 the tail of a picture whose first slices were dropped would
       packetise a fragment of a frame as if it were a whole one. */
    constexpr std::size_t NALS_PER_PICTURE = 8;
    const int frames = 60;
    for (int i = 0; i < frames; ++i)
        injectPicture(BASE_NS + i * static_cast<std::int64_t>(FRAME_NS), NALS_PER_PICTURE);
    const std::size_t dropped = injector_->droppedFrames();
    ASSERT_GT(dropped, 0u);
    EXPECT_EQ(dropped % NALS_PER_PICTURE, 0u)
        << "dropped " << dropped << " NAL units, which is not a whole number of pictures";
}

TEST_F(FrameInjectorTest, BoundsTheQueueEvenWhenStampsNeverAdvance)
{
    /* A source with a stuck clock never trips the span check, so the length
       backstop is the only thing standing between it and unbounded memory. */
    for (std::size_t i = 0; i < FrameInjector::MAX_QUEUE_LENGTH + 500; ++i)
        injectPicture(BASE_NS);
    EXPECT_GE(injector_->droppedFrames(), 500u);
}

TEST_F(FrameInjectorTest, ShutdownDiscardsWhatIsQueued)
{
    for (int i = 0; i < 5; ++i)
        injectPicture(BASE_NS + i * FRAME_NS);
    injector_->shutdown();
    /* Injecting after shutdown is a no-op rather than a crash or a leak */
    injectPicture(BASE_NS + 10 * FRAME_NS);
    SUCCEED();
}
