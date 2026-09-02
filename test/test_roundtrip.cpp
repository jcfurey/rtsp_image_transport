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

#include <sensor_msgs/image_encodings.hpp>

#include <vector>

using namespace rtsp_image_transport;
using namespace rtsp_image_transport::test;

namespace
{

std::vector<FrameDataPtr> takePackets(StreamEncoder& encoder)
{
    std::vector<FrameDataPtr> packets;
    while (FrameDataPtr packet = encoder.nextPacket())
        packets.push_back(std::move(packet));
    return packets;
}

/* Reassembles one Annex B access unit the way the RTSP path does: the encoder
   hands out bare NAL units, the server marks the final one, and FrameExtractor
   joins all NAL units through that marker before feeding the decoder. */
FrameDataPtr toAnnexB(const std::vector<FrameDataPtr>& nals)
{
    static const unsigned char start_code[] = {0x00, 0x00, 0x00, 0x01};
    std::size_t size = nals.size() * sizeof(start_code);
    for (const FrameDataPtr& nal : nals)
        size += nal->length();

    std::vector<unsigned char> buffer;
    buffer.reserve(size);
    for (const FrameDataPtr& nal : nals)
    {
        buffer.insert(buffer.end(), start_code, start_code + sizeof(start_code));
        buffer.insert(buffer.end(), nal->data(), nal->data() + nal->length());
    }
    return std::make_shared<FrameData>(buffer.data(), buffer.size(), nals.front()->stamp());
}

}  // namespace

class RoundTripTest : public ::testing::TestWithParam<VideoCodec>
{
};

TEST_P(RoundTripTest, EncodedImagesComeBackRecognisable)
{
    const VideoCodec codec = GetParam();
    const unsigned width = 320, height = 240, frames = 12;

    std::unique_ptr<StreamEncoder> encoder;
    try
    {
        encoder = std::make_unique<StreamEncoder>(codec, /*use_hw_encoder=*/false);
    }
    catch (const StreamingError&)
    {
        GTEST_SKIP() << "no encoder for " << videoCodecName(codec) << " in this FFmpeg build";
    }
    encoder->setBitrate(4000000);
    encoder->setFramerate(30);

    StreamDecoder::Options options;
    options.use_hw_decoder = false;
    options.hw_device = "none";
    StreamDecoder decoder(codec, options);

    const bool annex_b = codec == VideoCodec::H264 || codec == VideoCodec::H265;
    std::vector<sensor_msgs::msg::Image> sent;
    std::vector<sensor_msgs::msg::Image::UniquePtr> received;

    for (unsigned i = 0; i < frames; ++i)
    {
        sensor_msgs::msg::Image img = makeTestImage(width, height, i);
        sent.push_back(img);
        std::size_t encoded = 0;
        ASSERT_NO_THROW(encoded = encoder->encodeVideo(img));
        if (encoded == 0)
            continue;

        std::vector<FrameDataPtr> packets = takePackets(*encoder);
        ASSERT_EQ(packets.size(), encoded);
        if (annex_b)
        {
            ASSERT_NO_THROW(decoder.decodeVideo(toAnnexB(packets)));
            while (sensor_msgs::msg::Image::UniquePtr out = decoder.nextFrame())
                received.push_back(std::move(out));
        }
        else
        {
            for (const FrameDataPtr& packet : packets)
            {
                ASSERT_NO_THROW(decoder.decodeVideo(packet));
                while (sensor_msgs::msg::Image::UniquePtr out = decoder.nextFrame())
                    received.push_back(std::move(out));
            }
        }
    }

    ASSERT_GE(received.size(), frames / 2) << "round trip lost most frames";
    EXPECT_EQ(received.front()->width, width);
    EXPECT_EQ(received.front()->height, height);
    EXPECT_EQ(received.front()->encoding, sensor_msgs::image_encodings::BGR8);

    /* Lossy compression at this bitrate should stay well inside a mean error
       of 24/255 per channel; anything worse means the pipeline scrambled the
       image rather than merely compressing it. */
    double difference = meanAbsoluteDifference(sent.front(), *received.front());
    EXPECT_LT(difference, 24.0) << "first decoded frame does not resemble the input";
}

INSTANTIATE_TEST_SUITE_P(Codecs, RoundTripTest,
                         ::testing::Values(VideoCodec::H264, VideoCodec::H265, VideoCodec::MPEG4, VideoCodec::VP8,
                                           VideoCodec::VP9),
                         [](const testing::TestParamInfo<VideoCodec>& info)
                         {
                             std::string name = videoCodecName(info.param);
                             for (char& c : name)
                             {
                                 if (!std::isalnum(static_cast<unsigned char>(c)))
                                     c = '_';
                             }
                             return name;
                         });

TEST(RoundTrip, GrayscaleInputSurvivesTheTrip)
{
    std::unique_ptr<StreamEncoder> encoder;
    try
    {
        encoder = std::make_unique<StreamEncoder>(VideoCodec::H264, false);
    }
    catch (const StreamingError&)
    {
        GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
    }
    encoder->setBitrate(4000000);
    encoder->setFramerate(30);

    StreamDecoder::Options options;
    options.use_hw_decoder = false;
    options.hw_device = "none";
    StreamDecoder decoder(VideoCodec::H264, options);

    std::vector<sensor_msgs::msg::Image::UniquePtr> received;
    for (unsigned i = 0; i < 8; ++i)
    {
        sensor_msgs::msg::Image img = makeTestImage(160, 120, i, sensor_msgs::image_encodings::MONO8);
        std::size_t encoded = 0;
        ASSERT_NO_THROW(encoded = encoder->encodeVideo(img));
        if (encoded == 0)
            continue;

        std::vector<FrameDataPtr> packets = takePackets(*encoder);
        ASSERT_EQ(packets.size(), encoded);
        ASSERT_NO_THROW(decoder.decodeVideo(toAnnexB(packets)));
        while (sensor_msgs::msg::Image::UniquePtr out = decoder.nextFrame())
            received.push_back(std::move(out));
    }
    ASSERT_FALSE(received.empty());
    /* Mono input still comes back as BGR8, with the three channels near equal */
    EXPECT_EQ(received.front()->encoding, sensor_msgs::image_encodings::BGR8);
    const auto& data = received.front()->data;
    for (std::size_t i = 0; i + 2 < data.size(); i += 3 * 97)
    {
        EXPECT_NEAR(data[i], data[i + 1], 24);
        EXPECT_NEAR(data[i + 1], data[i + 2], 24);
    }
}
