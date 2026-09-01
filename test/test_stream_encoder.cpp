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
#include "stream_encoder.h"

#include "streaming_error.h"
#include "test_helpers.h"

#include <gtest/gtest.h>

#include <sensor_msgs/image_encodings.hpp>

#include <vector>

using namespace rtsp_image_transport;
using namespace rtsp_image_transport::test;

namespace
{

std::unique_ptr<StreamEncoder> makeEncoder(VideoCodec codec = VideoCodec::H264)
{
    try
    {
        auto encoder = std::make_unique<StreamEncoder>(codec, /*use_hw_encoder=*/false);
        encoder->setBitrate(1000000);
        encoder->setFramerate(30);
        return encoder;
    }
    catch (const StreamingError&)
    {
        return nullptr;
    }
}

std::size_t encodeClip(StreamEncoder& encoder, unsigned width, unsigned height, unsigned frames,
                       std::vector<FrameDataPtr>* out = nullptr)
{
    std::size_t total = 0;
    for (unsigned i = 0; i < frames; ++i)
    {
        sensor_msgs::msg::Image img = makeTestImage(width, height, i);
        total += encoder.encodeVideo(img);
        while (FrameDataPtr packet = encoder.nextPacket())
        {
            if (out)
                out->push_back(packet);
        }
    }
    return total;
}

}  // namespace

TEST(StreamEncoder, ProducesPacketsForBgr8Input)
{
    auto encoder = makeEncoder();
    if (!encoder)
        GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
    std::vector<FrameDataPtr> packets;
    std::size_t count = encodeClip(*encoder, 320, 240, 10, &packets);
    EXPECT_GT(count, 0u);
    EXPECT_EQ(packets.size(), count);
    for (const FrameDataPtr& packet : packets)
        EXPECT_GT(packet->length(), 0u);
}

TEST(StreamEncoder, SplitsAnnexBStreamIntoIndividualNalUnits)
{
    /* Live555 expects one NAL unit per frame from the source, without the
       Annex B start code. A packet that still contains a start code would be
       packetised as garbage. */
    auto encoder = makeEncoder();
    if (!encoder)
        GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
    std::vector<FrameDataPtr> packets;
    encodeClip(*encoder, 320, 240, 10, &packets);
    ASSERT_FALSE(packets.empty());

    for (const FrameDataPtr& packet : packets)
    {
        const unsigned char* d = packet->data();
        ASSERT_GE(packet->length(), 1u);
        for (std::size_t i = 0; i + 3 < packet->length(); ++i)
        {
            const bool start_code = d[i] == 0 && d[i + 1] == 0 && (d[i + 2] == 1 || (d[i + 2] == 0 && d[i + 3] == 1));
            EXPECT_FALSE(start_code) << "NAL unit still contains a start code at offset " << i;
        }
    }
}

TEST(StreamEncoder, PropagatesImageStampToPackets)
{
    auto encoder = makeEncoder();
    if (!encoder)
        GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
    std::vector<FrameDataPtr> packets;
    encodeClip(*encoder, 320, 240, 6, &packets);
    ASSERT_FALSE(packets.empty());
    for (const FrameDataPtr& packet : packets)
    {
        EXPECT_GE(packet->stamp().nanoseconds(), BASE_STAMP_NS);
        EXPECT_LE(packet->stamp().nanoseconds(), BASE_STAMP_NS + 10 * FRAME_INTERVAL_NS);
    }
}

TEST(StreamEncoder, AcceptsTheCommonRosImageEncodings)
{
    for (const std::string& encoding : {sensor_msgs::image_encodings::BGR8, sensor_msgs::image_encodings::RGB8,
                                        sensor_msgs::image_encodings::MONO8, sensor_msgs::image_encodings::RGBA8,
                                        sensor_msgs::image_encodings::BGRA8})
    {
        auto encoder = makeEncoder();
        if (!encoder)
            GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
        SCOPED_TRACE(encoding);
        std::size_t produced = 0;
        for (unsigned i = 0; i < 4; ++i)
        {
            sensor_msgs::msg::Image img = makeTestImage(160, 120, i, encoding);
            EXPECT_NO_THROW(produced += encoder->encodeVideo(img));
            while (encoder->nextPacket())
            {
            }
        }
        EXPECT_GT(produced, 0u);
    }
}

TEST(StreamEncoder, RejectsUnsupportedImageEncoding)
{
    auto encoder = makeEncoder();
    if (!encoder)
        GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
    sensor_msgs::msg::Image img = makeTestImage(160, 120, 0);
    img.encoding = "totally_made_up";
    EXPECT_THROW(encoder->encodeVideo(img), StreamingError);
}

TEST(StreamEncoder, RejectsImageSizeChange)
{
    auto encoder = makeEncoder();
    if (!encoder)
        GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
    sensor_msgs::msg::Image first = makeTestImage(160, 120, 0);
    ASSERT_NO_THROW(encoder->encodeVideo(first));
    while (encoder->nextPacket())
    {
    }
    sensor_msgs::msg::Image second = makeTestImage(320, 240, 1);
    EXPECT_THROW(encoder->encodeVideo(second), StreamingError);
}

TEST(StreamEncoder, SettingsAreLockedOnceEncodingStarted)
{
    auto encoder = makeEncoder();
    if (!encoder)
        GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
    sensor_msgs::msg::Image img = makeTestImage(160, 120, 0);
    ASSERT_NO_THROW(encoder->encodeVideo(img));
    EXPECT_THROW(encoder->setBitrate(2000000), StreamingError);
    EXPECT_THROW(encoder->setFramerate(60), StreamingError);
    EXPECT_THROW(encoder->setPackageSizeHint(1000), StreamingError);
}

TEST(StreamEncoder, VbvBufferTracksTheConfiguredBitrate)
{
    /* Regression guard. setBitrate() used to change bit_rate and rc_max_rate
       but not rc_buffer_size, which stayed at twice the 1 Mbit/s default the
       context was built with. The VBV was therefore 2 Mbit whatever the stream
       was configured for: ten seconds of buffer at 200 kbit/s. The VBV bounds
       how far one picture may overshoot its share of the bitrate, and so how
       long a single frame can occupy the link. */
    for (unsigned long bitrate : {200000UL, 1000000UL, 8000000UL, 25000000UL})
    {
        auto encoder = makeEncoder();
        if (!encoder)
            GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
        SCOPED_TRACE(bitrate);
        encoder->setBitrate(bitrate);
        ASSERT_NE(encoder->context(), nullptr);
        EXPECT_EQ(encoder->context()->bit_rate, static_cast<std::int64_t>(bitrate));
        EXPECT_EQ(encoder->context()->rc_max_rate, static_cast<std::int64_t>(bitrate));
        /* A quarter second of buffer, so the number scales with the stream */
        EXPECT_EQ(encoder->context()->rc_buffer_size, static_cast<int>(bitrate / 4))
            << "the VBV does not follow the configured bitrate";
    }
}

TEST(StreamEncoder, ReportsUnsupportedCodec)
{
    /* Decode-only codecs have no encoder table entry and must say so rather
       than fail obscurely later. */
    EXPECT_THROW(StreamEncoder(VideoCodec::MJPEG, false), StreamingError);
    EXPECT_THROW(StreamEncoder(VideoCodec::MPEG2, false), StreamingError);
    EXPECT_THROW(StreamEncoder(VideoCodec::H263, false), StreamingError);
    EXPECT_THROW(StreamEncoder(VideoCodec::Unknown, false), StreamingError);
}

TEST(StreamEncoder, ExposesContextAndCodec)
{
    auto encoder = makeEncoder();
    if (!encoder)
        GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
    EXPECT_EQ(encoder->codec(), VideoCodec::H264);
    ASSERT_NE(encoder->context(), nullptr);
    ASSERT_NE(encoder->context()->codec, nullptr);
    EXPECT_NE(encoder->context()->codec->name, nullptr);
}
