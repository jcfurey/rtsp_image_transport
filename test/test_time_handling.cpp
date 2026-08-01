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
#include "stream_decoder.h"
#include "stream_encoder.h"
#include "streaming_error.h"
#include "test_helpers.h"

#include <gtest/gtest.h>

#include <memory>
#include <vector>

using namespace rtsp_image_transport;
using namespace rtsp_image_transport::test;

namespace
{

/* The encoder time base is 1/300, so a 30 Hz stream advances 10 ticks a frame */
constexpr std::int64_t TICKS_PER_FRAME_AT_30HZ = 10;

std::unique_ptr<StreamEncoder> makeEncoder()
{
    try
    {
        auto encoder = std::make_unique<StreamEncoder>(VideoCodec::H264, /*use_hw_encoder=*/false);
        encoder->setBitrate(1000000);
        encoder->setFramerate(30);
        return encoder;
    }
    catch (const StreamingError&)
    {
        return nullptr;
    }
}

/* Feeds one image with the given stamp and returns the presentation time stamp
   the encoder assigned to it. */
std::int64_t feed(StreamEncoder& encoder, unsigned seq, std::int64_t stamp_ns)
{
    sensor_msgs::msg::Image img = makeTestImage(160, 120, seq);
    img.header.stamp = rclcpp::Time(stamp_ns);
    encoder.encodeVideo(img);
    while (encoder.nextPacket())
    {
    }
    return encoder.lastPresentationTimestamp();
}

}  // namespace

TEST(TimeHandling, EncoderPacesFramesByTheGapBetweenThem)
{
    auto encoder = makeEncoder();
    if (!encoder)
        GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
    std::vector<std::int64_t> pts;
    for (int i = 0; i < 6; ++i)
        pts.push_back(feed(*encoder, i, BASE_STAMP_NS + i * FRAME_INTERVAL_NS));
    for (std::size_t i = 1; i < pts.size(); ++i)
        EXPECT_EQ(pts[i] - pts[i - 1], TICKS_PER_FRAME_AT_30HZ) << "frame " << i;
}

TEST(TimeHandling, EncoderKeepsItsFrameRateWhenABagLoops)
{
    /* Regression guard. Presentation times used to be measured from the first
       stamp the encoder ever saw. Once a bag looped and the stamps went
       backwards, every later frame was clamped to one tick after its
       predecessor, so the encoder believed the stream ran at 300 fps and its
       rate control collapsed. */
    auto encoder = makeEncoder();
    if (!encoder)
        GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";

    for (int i = 0; i < 6; ++i)
        feed(*encoder, i, BASE_STAMP_NS + i * FRAME_INTERVAL_NS);

    /* The bag loops back ten seconds */
    std::vector<std::int64_t> after;
    for (int i = 0; i < 6; ++i)
        after.push_back(feed(*encoder, 6 + i, BASE_STAMP_NS - 10000000000LL + i * FRAME_INTERVAL_NS));

    for (std::size_t i = 1; i < after.size(); ++i)
    {
        EXPECT_EQ(after[i] - after[i - 1], TICKS_PER_FRAME_AT_30HZ)
            << "frame pacing lost after the loop at frame " << i;
    }
}

TEST(TimeHandling, EncoderPresentationTimesAreStrictlyIncreasing)
{
    auto encoder = makeEncoder();
    if (!encoder)
        GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
    const std::int64_t stamps[] = {BASE_STAMP_NS,
                                   BASE_STAMP_NS,                          // duplicate
                                   BASE_STAMP_NS + FRAME_INTERVAL_NS,
                                   BASE_STAMP_NS - FRAME_INTERVAL_NS,      // backwards
                                   BASE_STAMP_NS + 2 * FRAME_INTERVAL_NS,
                                   0,                                      // simulated time reset
                                   FRAME_INTERVAL_NS,
                                   BASE_STAMP_NS + 3 * FRAME_INTERVAL_NS};
    std::int64_t previous = -1;
    for (std::size_t i = 0; i < std::size(stamps); ++i)
    {
        std::int64_t pts = 0;
        ASSERT_NO_THROW(pts = feed(*encoder, static_cast<unsigned>(i), stamps[i])) << "frame " << i;
        EXPECT_GT(pts, previous) << "frame " << i << " did not advance the presentation time";
        previous = pts;
    }
}

TEST(TimeHandling, EncoderTreatsALongGapAsOneFrame)
{
    /* A publisher that stops while nobody is subscribed and resumes minutes
       later must not insert a minutes-long hole into the RTP timeline. */
    auto encoder = makeEncoder();
    if (!encoder)
        GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
    std::int64_t first = feed(*encoder, 0, BASE_STAMP_NS);
    std::int64_t second = feed(*encoder, 1, BASE_STAMP_NS + 300000000000LL);  // five minutes later
    EXPECT_EQ(second - first, TICKS_PER_FRAME_AT_30HZ);
}

TEST(TimeHandling, DecodedImagesKeepTheTimeBaseOfTheirInput)
{
    /* The subscriber decides whether packets carry the sender's clock or the
       node clock; whichever it picks has to survive the decoder. */
    auto packets = encodeTestStream(VideoCodec::H264, 320, 240, 10);
    if (packets.empty())
        GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";

    StreamDecoder::Options options;
    options.use_hw_decoder = false;
    options.hw_device = "none";
    StreamDecoder decoder(VideoCodec::H264, options);

    /* Stamps from a simulated clock that starts at 5 s, nowhere near wall clock */
    const std::int64_t sim_base = 5000000000LL;
    std::vector<std::int64_t> out;
    for (std::size_t i = 0; i < packets.size(); ++i)
    {
        rclcpp::Time stamp(sim_base + static_cast<std::int64_t>(i) * FRAME_INTERVAL_NS);
        FrameDataPtr data = std::make_shared<FrameData>(packets[i].data(), packets[i].size(), stamp);
        decoder.decodeVideo(data);
        while (sensor_msgs::msg::Image::UniquePtr img = decoder.nextFrame())
            out.push_back(rclcpp::Time(img->header.stamp).nanoseconds());
    }
    ASSERT_FALSE(out.empty());
    EXPECT_EQ(out.front(), sim_base) << "the decoder replaced the simulated time base";
    for (std::size_t i = 0; i < out.size(); ++i)
    {
        EXPECT_GE(out[i], sim_base) << "frame " << i;
        EXPECT_LE(out[i], sim_base + static_cast<std::int64_t>(packets.size()) * FRAME_INTERVAL_NS);
    }
}
