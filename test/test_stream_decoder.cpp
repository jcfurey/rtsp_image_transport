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

#include "streaming_error.h"
#include "test_helpers.h"

#include <gtest/gtest.h>

#include <sensor_msgs/image_encodings.hpp>

#include <vector>

using namespace rtsp_image_transport;
using namespace rtsp_image_transport::test;

namespace
{

struct DecodeResult
{
    std::vector<sensor_msgs::msg::Image::UniquePtr> images;
    std::size_t reported = 0;
};

/* Feeds a whole encoded clip through the decoder the way the RTSP client does,
   one packet at a time with an increasing presentation time. */
DecodeResult decodeAll(StreamDecoder& decoder, const std::vector<std::vector<std::uint8_t>>& packets)
{
    DecodeResult result;
    for (std::size_t i = 0; i < packets.size(); ++i)
    {
        rclcpp::Time stamp(BASE_STAMP_NS + static_cast<std::int64_t>(i) * FRAME_INTERVAL_NS);
        FrameDataPtr data = std::make_shared<FrameData>(packets[i].data(), packets[i].size(), stamp);
        result.reported += decoder.decodeVideo(data);
        while (sensor_msgs::msg::Image::UniquePtr img = decoder.nextFrame())
            result.images.push_back(std::move(img));
    }
    return result;
}

StreamDecoder::Options softwareOptions()
{
    StreamDecoder::Options options;
    options.use_hw_decoder = false;
    options.hw_device = "none";
    return options;
}

}  // namespace

class StreamDecoderCodecTest : public ::testing::TestWithParam<VideoCodec>
{
};

TEST_P(StreamDecoderCodecTest, DecodesToWellFormedBgr8Images)
{
    const VideoCodec codec = GetParam();
    const int width = 320, height = 240;
    auto packets = encodeTestStream(codec, width, height, 12);
    if (packets.empty())
        GTEST_SKIP() << "no encoder for " << videoCodecName(codec) << " in this FFmpeg build";

    StreamDecoder decoder(codec, softwareOptions());
    DecodeResult result = decodeAll(decoder, packets);

    ASSERT_GE(result.images.size(), 8u) << "decoded too few frames for " << videoCodecName(codec);
    EXPECT_EQ(result.reported, result.images.size());
    for (const auto& img : result.images)
    {
        EXPECT_EQ(img->encoding, sensor_msgs::image_encodings::BGR8);
        EXPECT_EQ(img->width, static_cast<unsigned>(width));
        EXPECT_EQ(img->height, static_cast<unsigned>(height));
        EXPECT_EQ(img->step, static_cast<unsigned>(3 * width));
        EXPECT_FALSE(img->is_bigendian);
        ASSERT_EQ(img->data.size(), static_cast<std::size_t>(3) * width * height);
        bool all_black = true;
        for (std::size_t i = 0; i < img->data.size(); i += 101)
        {
            if (img->data[i] != 0)
            {
                all_black = false;
                break;
            }
        }
        EXPECT_FALSE(all_black) << "decoded frame is entirely black";
    }
}

INSTANTIATE_TEST_SUITE_P(AllCodecs, StreamDecoderCodecTest,
                         ::testing::Values(VideoCodec::H264, VideoCodec::H265, VideoCodec::MPEG4, VideoCodec::MPEG2,
                                           VideoCodec::H263, VideoCodec::VP8, VideoCodec::VP9, VideoCodec::AV1,
                                           VideoCodec::MJPEG),
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

TEST(StreamDecoder, RejectsCodecWithoutDecoderSupport)
{
    EXPECT_THROW(StreamDecoder(VideoCodec::Unknown, softwareOptions()), StreamingError);
}

TEST(StreamDecoder, ReportsDecoderInUse)
{
    StreamDecoder decoder(VideoCodec::H264, softwareOptions());
    EXPECT_FALSE(decoder.description().empty());
    EXPECT_NE(decoder.description().find("h264"), std::string::npos);
    EXPECT_FALSE(decoder.isHardwareAccelerated());
    EXPECT_EQ(decoder.codec(), VideoCodec::H264);
    ASSERT_NE(decoder.context(), nullptr);
}

TEST(StreamDecoder, StampsFramesFromTheirOwnAccessUnit)
{
    /* Regression guard: the stamp must follow the access unit through the
       decoder rather than being taken from whichever packet happened to flush
       the frame out. */
    auto packets = encodeTestStream(VideoCodec::H264, 320, 240, 12);
    ASSERT_FALSE(packets.empty());

    StreamDecoder decoder(VideoCodec::H264, softwareOptions());
    DecodeResult result = decodeAll(decoder, packets);
    ASSERT_GE(result.images.size(), 8u);

    std::int64_t previous = -1;
    for (std::size_t i = 0; i < result.images.size(); ++i)
    {
        std::int64_t stamp = rclcpp::Time(result.images[i]->header.stamp).nanoseconds();
        EXPECT_GE(stamp, BASE_STAMP_NS) << "frame " << i << " has a stamp before the stream started";
        EXPECT_LE(stamp, BASE_STAMP_NS + static_cast<std::int64_t>(packets.size()) * FRAME_INTERVAL_NS);
        EXPECT_GT(stamp, previous) << "frame " << i << " is not strictly newer than its predecessor";
        previous = stamp;
    }
    /* The first frame carries the time of the first access unit, not of a
       later packet. */
    EXPECT_EQ(rclcpp::Time(result.images.front()->header.stamp).nanoseconds(), BASE_STAMP_NS);
}

TEST(StreamDecoder, LowLatencyAndBufferedModesBothDecodeEveryFrame)
{
    auto packets = encodeTestStream(VideoCodec::H264, 320, 240, 15);
    ASSERT_FALSE(packets.empty());

    std::size_t counts[2];
    for (int i = 0; i < 2; ++i)
    {
        StreamDecoder::Options options = softwareOptions();
        options.low_latency = (i == 0);
        StreamDecoder decoder(VideoCodec::H264, options);
        counts[i] = decodeAll(decoder, packets).images.size();
    }
    EXPECT_GE(counts[0], 10u);
    EXPECT_EQ(counts[0], counts[1]) << "low_latency changed how many frames come out";
}

TEST(StreamDecoder, SkipFrameLevelsReduceOutput)
{
    /* The subscriber leans on these levels to catch up when it falls behind,
       so each step must actually drop more than the previous one. */
    auto packets = encodeTestStream(VideoCodec::H264, 320, 240, 20);
    ASSERT_FALSE(packets.empty());

    auto countWith = [&](StreamDecoder::DecodeFrames level)
    {
        StreamDecoder decoder(VideoCodec::H264, softwareOptions());
        decoder.setDecodeFrames(level);
        return decodeAll(decoder, packets).images.size();
    };

    const std::size_t all = countWith(StreamDecoder::DecodeFrames::All);
    const std::size_t key = countWith(StreamDecoder::DecodeFrames::Key);
    const std::size_t none = countWith(StreamDecoder::DecodeFrames::None);

    EXPECT_GT(all, 0u);
    EXPECT_LT(key, all) << "key-frames-only did not drop anything";
    EXPECT_EQ(none, 0u) << "discard-everything still produced frames";
}

TEST(StreamDecoder, SurvivesGarbageInput)
{
    StreamDecoder decoder(VideoCodec::H264, softwareOptions());
    std::vector<std::uint8_t> garbage(512);
    for (std::size_t i = 0; i < garbage.size(); ++i)
        garbage[i] = static_cast<std::uint8_t>(i * 37 + 11);
    FrameDataPtr data = std::make_shared<FrameData>(garbage.data(), garbage.size(), rclcpp::Time(BASE_STAMP_NS));
    /* Either it quietly produces nothing or it reports a DecodingError, but it
       must not take the process down. */
    try
    {
        EXPECT_EQ(decoder.decodeVideo(data), 0u);
    }
    catch (const DecodingError&)
    {
    }
}

TEST(StreamDecoder, HandlesResolutionChangeMidStream)
{
    auto small = encodeTestStream(VideoCodec::H264, 320, 240, 8);
    auto large = encodeTestStream(VideoCodec::H264, 640, 480, 8);
    ASSERT_FALSE(small.empty());
    ASSERT_FALSE(large.empty());

    StreamDecoder decoder(VideoCodec::H264, softwareOptions());
    DecodeResult first = decodeAll(decoder, small);
    DecodeResult second = decodeAll(decoder, large);

    ASSERT_FALSE(first.images.empty());
    ASSERT_FALSE(second.images.empty());
    EXPECT_EQ(first.images.front()->width, 320u);
    EXPECT_EQ(second.images.back()->width, 640u);
    EXPECT_EQ(second.images.back()->height, 480u);
    EXPECT_EQ(second.images.back()->data.size(), static_cast<std::size_t>(3) * 640 * 480);
}

TEST(StreamDecoder, OddDimensionsAreHandled)
{
    /* Chroma subsampling makes odd sizes a classic source of buffer overruns
       in the colour conversion. */
    auto packets = encodeTestStream(VideoCodec::MJPEG, 321, 243, 5);
    if (packets.empty())
        GTEST_SKIP() << "no MJPEG encoder";
    StreamDecoder decoder(VideoCodec::MJPEG, softwareOptions());
    DecodeResult result = decodeAll(decoder, packets);
    ASSERT_FALSE(result.images.empty());
    EXPECT_EQ(result.images.front()->width, 321u);
    EXPECT_EQ(result.images.front()->height, 243u);
    EXPECT_EQ(result.images.front()->data.size(), static_cast<std::size_t>(3) * 321 * 243);
}

/* --- hardware decoding configuration ------------------------------------- */

TEST(StreamDecoderHardware, AutoFallsBackToSoftwareWhenNoGpuIsPresent)
{
    StreamDecoder::Options options;
    options.use_hw_decoder = true;
    options.hw_device = "auto";
    auto packets = encodeTestStream(VideoCodec::H264, 320, 240, 10);
    ASSERT_FALSE(packets.empty());

    StreamDecoder decoder(VideoCodec::H264, options);
    DecodeResult result = decodeAll(decoder, packets);
    /* Whether this machine has a GPU or not, every frame must come out. */
    EXPECT_GE(result.images.size(), 8u) << "frames were lost while selecting a decoder";
    EXPECT_FALSE(decoder.description().empty());
}

TEST(StreamDecoderHardware, NoneDisablesHardwareEntirely)
{
    StreamDecoder::Options options;
    options.use_hw_decoder = true;
    options.hw_device = "none";
    StreamDecoder decoder(VideoCodec::H264, options);
    EXPECT_FALSE(decoder.isHardwareAccelerated());
    EXPECT_NE(decoder.description().find("software"), std::string::npos);
}

TEST(StreamDecoderHardware, UseHwDecoderFalseDisablesHardwareEvenOnAuto)
{
    StreamDecoder::Options options;
    options.use_hw_decoder = false;
    options.hw_device = "auto";
    StreamDecoder decoder(VideoCodec::H265, options);
    EXPECT_FALSE(decoder.isHardwareAccelerated());
}

TEST(StreamDecoderHardware, UnknownDeviceNameFallsBackToSoftware)
{
    StreamDecoder::Options options;
    options.use_hw_decoder = true;
    options.hw_device = "definitely-not-a-device";
    StreamDecoder decoder(VideoCodec::H264, options);
    EXPECT_FALSE(decoder.isHardwareAccelerated());

    auto packets = encodeTestStream(VideoCodec::H264, 320, 240, 6);
    ASSERT_FALSE(packets.empty());
    EXPECT_FALSE(decodeAll(decoder, packets).images.empty());
}

TEST(StreamDecoderHardware, NamedDeviceIsAcceptedOrCleanlyRefused)
{
    /* Runs on machines with and without the device: it either gets used or the
       decoder quietly ends up in software, but never throws or loses frames. */
    for (const char* device : {"vaapi", "cuda", "qsv", "vdpau"})
    {
        StreamDecoder::Options options;
        options.use_hw_decoder = true;
        options.hw_device = device;
        StreamDecoder decoder(VideoCodec::H264, options);
        SCOPED_TRACE(device);
        if (decoder.isHardwareAccelerated())
            EXPECT_NE(decoder.description().find(device), std::string::npos);
        auto packets = encodeTestStream(VideoCodec::H264, 320, 240, 6);
        ASSERT_FALSE(packets.empty());
        EXPECT_GE(decodeAll(decoder, packets).images.size(), 4u);
    }
}

TEST(StreamDecoderHardware, PinnedDecoderIsUsedOrReportedAsUnavailable)
{
    StreamDecoder::Options options;
    options.decoder = "h264";
    StreamDecoder decoder(VideoCodec::H264, options);
    EXPECT_NE(decoder.description().find("h264"), std::string::npos);

    /* A decoder that does not exist must be reported rather than silently
       replaced by something else. */
    StreamDecoder::Options missing;
    missing.decoder = "no_such_decoder_exists";
    EXPECT_THROW(StreamDecoder(VideoCodec::H264, missing), StreamingError);
}

TEST(StreamDecoderHardware, AvailableDevicesIsStableAndNamesAreValid)
{
    /* The probe is cached, so repeated calls must agree; whatever it reports
       has to be a device type FFmpeg knows. */
    std::vector<std::string> first = StreamDecoder::availableHwDevices();
    std::vector<std::string> second = StreamDecoder::availableHwDevices();
    EXPECT_EQ(first, second);
    for (const std::string& name : first)
        EXPECT_NE(av_hwdevice_find_type_by_name(name.c_str()), AV_HWDEVICE_TYPE_NONE) << name;
}

TEST(StreamDecoderHardware, IntelIgpuPathIsOfferedWhenTheDeviceExists)
{
    /* On an Intel machine the render node is what VAAPI and Quick Sync both
       run on. Where it exists, asking for VAAPI must actually produce a
       hardware decoder for the codecs an iGPU handles. */
    std::vector<std::string> devices = StreamDecoder::availableHwDevices();
    const bool has_vaapi = std::find(devices.begin(), devices.end(), "vaapi") != devices.end();
    if (!has_vaapi)
        GTEST_SKIP() << "no VAAPI device on this machine";

    for (VideoCodec codec : {VideoCodec::H264, VideoCodec::H265, VideoCodec::VP9, VideoCodec::MJPEG})
    {
        StreamDecoder::Options options;
        options.hw_device = "vaapi";
        StreamDecoder decoder(codec, options);
        SCOPED_TRACE(videoCodecName(codec));
        EXPECT_TRUE(decoder.isHardwareAccelerated()) << "VAAPI is present but was not used";

        auto packets = encodeTestStream(codec, 320, 240, 8);
        if (packets.empty())
            continue;
        DecodeResult result = decodeAll(decoder, packets);
        EXPECT_GE(result.images.size(), 4u);
        for (const auto& img : result.images)
        {
            EXPECT_EQ(img->encoding, sensor_msgs::image_encodings::BGR8);
            EXPECT_EQ(img->data.size(), static_cast<std::size_t>(3) * 320 * 240);
        }
    }
}
