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

#include <memory>
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

namespace
{

/* decodeAll(), but a damaged stream is allowed to make decodeVideo() throw. */
std::size_t decodeSurvivingImages(StreamDecoder& decoder, const std::vector<std::vector<std::uint8_t>>& packets)
{
    std::size_t images = 0;
    for (std::size_t i = 0; i < packets.size(); ++i)
    {
        rclcpp::Time stamp(BASE_STAMP_NS + static_cast<std::int64_t>(i) * FRAME_INTERVAL_NS);
        FrameDataPtr data = std::make_shared<FrameData>(packets[i].data(), packets[i].size(), stamp);
        try
        {
            decoder.decodeVideo(data);
        }
        catch (const DecodingError&)
        {
            /* A damaged packet may be rejected outright, but frames decoded
               before it still have to be collected. */
        }
        while (decoder.nextFrame())
            images++;
    }
    return images;
}

/* Byte offsets of every Annex B start code in an elementary stream packet. A
   four byte code is found at its trailing three bytes, which leaves one zero on
   the end of the previous unit — harmless trailing padding to a decoder. */
std::vector<std::size_t> nalOffsets(const std::vector<std::uint8_t>& packet)
{
    std::vector<std::size_t> offsets;
    for (std::size_t i = 0; i + 2 < packet.size();)
    {
        if (packet[i] == 0 && packet[i + 1] == 0 && packet[i + 2] == 1)
        {
            offsets.push_back(i);
            i += 3;
        }
        else
        {
            ++i;
        }
    }
    return offsets;
}

/* Whether the NAL unit starting at this offset carries coded picture data.
   H.264 puts the type in the low five bits of a one byte header; H.265 uses a
   two byte header with the type in bits 1 to 6, and calls everything below 32 a
   video coding layer unit. */
bool isSliceNal(const std::vector<std::uint8_t>& packet, std::size_t begin, VideoCodec codec)
{
    if (begin + 3 >= packet.size())
        return false;
    const std::uint8_t header = packet[begin + 3];
    if (codec == VideoCodec::H265)
        return ((header >> 1) & 0x3f) <= 31;
    const std::uint8_t nal_type = header & 0x1f;
    return nal_type == 1 || nal_type == 5;
}

/* Rebuilds a packet with the nth coded slice removed, which is what losing the
   RTP packets carrying that slice looks like: the surviving slices decode, and
   the macroblocks the missing one covered are never written at all. Parameter
   sets and SEI are left alone, as they would be on a real stream. */
std::vector<std::uint8_t> withoutSlice(const std::vector<std::uint8_t>& packet, std::size_t slice_to_drop,
                                       VideoCodec codec = VideoCodec::H264)
{
    const std::vector<std::size_t> offsets = nalOffsets(packet);
    std::vector<std::uint8_t> out;
    std::size_t slice_index = 0;
    for (std::size_t k = 0; k < offsets.size(); ++k)
    {
        const std::size_t begin = offsets[k];
        const std::size_t end = k + 1 < offsets.size() ? offsets[k + 1] : packet.size();
        if (isSliceNal(packet, begin, codec) && slice_index++ == slice_to_drop)
            continue;
        out.insert(out.end(), packet.begin() + begin, packet.begin() + end);
    }
    return out;
}

std::size_t countSlices(const std::vector<std::uint8_t>& packet, VideoCodec codec = VideoCodec::H264)
{
    std::size_t slices = 0;
    for (std::size_t begin : nalOffsets(packet))
    {
        if (isSliceNal(packet, begin, codec))
            slices++;
    }
    return slices;
}

/* Feeds the decoder one NAL unit per call, which is how FrameExtractor drives
   it: live555 hands over a single NAL at a time and the extractor forwards each
   one on its own. Nothing else in this suite exercises that path. */
std::vector<sensor_msgs::msg::Image::UniquePtr> decodeNalAtATime(
    StreamDecoder& decoder, const std::vector<std::vector<std::uint8_t>>& packets)
{
    std::vector<sensor_msgs::msg::Image::UniquePtr> images;
    for (std::size_t i = 0; i < packets.size(); ++i)
    {
        rclcpp::Time stamp(BASE_STAMP_NS + static_cast<std::int64_t>(i) * FRAME_INTERVAL_NS);
        const std::vector<std::size_t> offsets = nalOffsets(packets[i]);
        for (std::size_t k = 0; k < offsets.size(); ++k)
        {
            const std::size_t end = k + 1 < offsets.size() ? offsets[k + 1] : packets[i].size();
            const std::vector<std::uint8_t> nal(packets[i].begin() + offsets[k], packets[i].begin() + end);
            try
            {
                decoder.decodeVideo(std::make_shared<FrameData>(nal.data(), nal.size(), stamp));
            }
            catch (const DecodingError&)
            {
            }
            while (sensor_msgs::msg::Image::UniquePtr img = decoder.nextFrame())
                images.push_back(std::move(img));
        }
    }
    return images;
}

/* Fraction of pixels that are exactly BGR (0, 135, 0) — what an all-zero YUV
   macroblock converts to, and therefore the signature of image area the decoder
   never wrote. */
double decoderGreenFraction(const sensor_msgs::msg::Image& img)
{
    std::size_t green = 0;
    for (unsigned y = 0; y < img.height; ++y)
    {
        for (unsigned x = 0; x < img.width; ++x)
        {
            const std::uint8_t* p = &img.data[static_cast<std::size_t>(y) * img.step + 3 * x];
            if (p[0] == 0 && p[1] == 135 && p[2] == 0)
                green++;
        }
    }
    return static_cast<double>(green) / (static_cast<double>(img.width) * img.height);
}

/* A stream that lost one slice out of every inter coded picture. Key frames are
   left whole so the sequence stays anchored, exactly as it would be on a link
   that drops the occasional RTP packet. */
std::vector<std::vector<std::uint8_t>> withSliceLoss(const std::vector<std::vector<std::uint8_t>>& packets,
                                                     std::size_t gop_size, std::size_t slice_to_drop,
                                                     VideoCodec codec = VideoCodec::H264)
{
    std::vector<std::vector<std::uint8_t>> lossy = packets;
    for (std::size_t i = 0; i < lossy.size(); ++i)
    {
        if (i % gop_size != 0)
            lossy[i] = withoutSlice(lossy[i], slice_to_drop, codec);
    }
    return lossy;
}

}  // namespace

/* The failure this whole path exists for: a multi-slice picture that lost one of
   its slices, which is what a dropped RTP packet leaves behind. The macroblocks
   the missing slice covered are never written, and an all-zero YUV block
   converts to flat green — the green bands seen on a lossy camera link. */
constexpr std::size_t SLICE_LOSS_GOP = 10;
constexpr std::size_t SLICE_LOSS_INDEX = 1;

TEST(StreamDecoder, DecodesOneNalAtATimeLikeTheRtspSource)
{
    /* AV_CODEC_FLAG2_CHUNKS is what lets the decoder assemble a picture across
       several calls, because that is how frames actually arrive: one NAL unit
       per call. A picture emitted before all of its slices had been handed over
       would leave part of the image unwritten, and unwritten YUV is green. At
       full HD, where a picture is most likely to be split up. */
    for (VideoCodec codec : {VideoCodec::H264, VideoCodec::H265})
    {
        auto packets = encodeTestStream(codec, 1920, 1080, 6);
        if (packets.empty())
            continue;

        StreamDecoder whole(codec, softwareOptions());
        const std::size_t whole_count = decodeAll(whole, packets).images.size();

        StreamDecoder split(codec, softwareOptions());
        const auto images = decodeNalAtATime(split, packets);

        ASSERT_FALSE(images.empty()) << videoCodecName(codec) << ": NAL-at-a-time produced no images";
        EXPECT_EQ(images.size(), whole_count)
            << videoCodecName(codec) << ": feeding one NAL at a time changed the frame count";
        for (const auto& img : images)
        {
            EXPECT_EQ(decoderGreenFraction(*img), 0.0)
                << videoCodecName(codec) << ": part of the picture was never written";
        }
    }
}

TEST(StreamDecoder, ConcealsSliceLossInsteadOfPublishingHoles)
{
    auto packets = encodeTestStream(VideoCodec::H264, 320, 240, 30, 4);
    if (packets.size() < 20 || countSlices(packets[1]) < 3)
        GTEST_SKIP() << "this FFmpeg build did not produce a multi-slice H.264 stream";

    StreamDecoder intact(VideoCodec::H264, softwareOptions());
    const std::size_t baseline = decodeSurvivingImages(intact, packets);
    ASSERT_GT(baseline, 0u);

    StreamDecoder decoder(VideoCodec::H264, softwareOptions());
    const std::size_t concealed =
        decodeSurvivingImages(decoder, withSliceLoss(packets, SLICE_LOSS_GOP, SLICE_LOSS_INDEX));

    /* Concealment reconstructs the missing macroblocks from the reference frame,
       so the video keeps flowing rather than stalling on every damaged picture.
       Measured at the full 30 of 30 on FFmpeg 7.1; asserted with slack so a
       different libavcodec's heuristics cannot fail the run. */
    EXPECT_GE(concealed, baseline * 3 / 4)
        << "slice loss cost far more frames than concealment should: " << concealed << " of " << baseline;
}

TEST(StreamDecoder, DropsIncompletelyDecodedFramesWhenAsked)
{
    auto packets = encodeTestStream(VideoCodec::H264, 320, 240, 30, 4);
    if (packets.size() < 20 || countSlices(packets[1]) < 3)
        GTEST_SKIP() << "this FFmpeg build did not produce a multi-slice H.264 stream";
    const auto lossy = withSliceLoss(packets, SLICE_LOSS_GOP, SLICE_LOSS_INDEX);

    StreamDecoder concealing(VideoCodec::H264, softwareOptions());
    const std::size_t concealed = decodeSurvivingImages(concealing, lossy);

    StreamDecoder::Options options = softwareOptions();
    options.drop_corrupt_frames = true;
    StreamDecoder dropping(VideoCodec::H264, options);
    const std::size_t kept = decodeSurvivingImages(dropping, lossy);

    /* Every picture that lost a slice has to be recognisable as damaged. If this
       fails, error concealment has most likely been turned off again: it is
       ff_er_frame_end() that records the damage in decode_error_flags, and with
       concealment disabled it returns early and flags nothing. */
    EXPECT_LT(kept, concealed / 2) << "damaged frames were not flagged: kept " << kept << " of " << concealed;
    /* The undamaged key frames still have to get through. */
    EXPECT_GT(kept, 0u) << "the guard dropped even the intact key frames";
}

/* H.265 is where the camera actually streams from, and it is the case the two
   tests above cannot cover: libavcodec has no error resilience for HEVC, so a
   picture that lost a slice arrives with decode_error_flags clear and
   AV_FRAME_FLAG_CORRUPT unset. Nothing libavcodec reports distinguishes it from
   a clean picture; the only evidence is the part of the frame buffer the
   decoder never wrote into. */
TEST(StreamDecoder, HevcSliceLossIsNotPublishedAsGreenBands)
{
    auto packets = encodeTestStream(VideoCodec::H265, 640, 480, 30, 4);
    if (packets.size() < 20 || countSlices(packets[1], VideoCodec::H265) < 3)
        GTEST_SKIP() << "this FFmpeg build did not produce a multi-slice H.265 stream";
    const auto lossy = withSliceLoss(packets, SLICE_LOSS_GOP, SLICE_LOSS_INDEX, VideoCodec::H265);

    StreamDecoder decoder(VideoCodec::H265, softwareOptions());
    DecodeResult result = decodeAll(decoder, lossy);
    ASSERT_FALSE(result.images.empty());

    /* An all-zero YUV macroblock converts to BGR(0,135,0). Before the decoder
       pre-filled its frames, a quarter of every damaged picture came out in
       exactly that colour. */
    for (const auto& img : result.images)
    {
        EXPECT_EQ(decoderGreenFraction(*img), 0.0)
            << "an unwritten region reached the subscriber as flat green";
    }
}

TEST(StreamDecoder, HevcDamageIsDetectedWithoutLibavcodecFlaggingIt)
{
    auto packets = encodeTestStream(VideoCodec::H265, 640, 480, 30, 4);
    if (packets.size() < 20 || countSlices(packets[1], VideoCodec::H265) < 3)
        GTEST_SKIP() << "this FFmpeg build did not produce a multi-slice H.265 stream";
    const auto lossy = withSliceLoss(packets, SLICE_LOSS_GOP, SLICE_LOSS_INDEX, VideoCodec::H265);

    StreamDecoder publishing(VideoCodec::H265, softwareOptions());
    const std::size_t published = decodeSurvivingImages(publishing, lossy);
    ASSERT_GT(published, 0u);

    StreamDecoder::Options options = softwareOptions();
    options.drop_corrupt_frames = true;
    StreamDecoder dropping(VideoCodec::H265, options);
    const std::size_t kept = decodeSurvivingImages(dropping, lossy);

    EXPECT_LT(kept, published / 2) << "damaged H.265 frames were not recognised: kept " << kept << " of "
                                   << published;
    EXPECT_GT(kept, 0u) << "the guard dropped even the intact key frames";
}

/* The detector must not fire on ordinary picture content. A luma value of 1
   does occur naturally; what marks a region as never written is that value
   together with perfectly neutral chroma across a large part of the frame. */
TEST(StreamDecoder, DropCorruptFramesKeepsACleanHevcStreamIntact)
{
    auto packets = encodeTestStream(VideoCodec::H265, 640, 480, 20);
    if (packets.size() < 10)
        GTEST_SKIP() << "no H.265 encoder in this FFmpeg build";

    StreamDecoder plain(VideoCodec::H265, softwareOptions());
    const std::size_t without_guard = decodeSurvivingImages(plain, packets);

    StreamDecoder::Options options = softwareOptions();
    options.drop_corrupt_frames = true;
    StreamDecoder guarded(VideoCodec::H265, options);
    const std::size_t with_guard = decodeSurvivingImages(guarded, packets);

    EXPECT_EQ(with_guard, without_guard) << "the damage detector fired on an undamaged H.265 stream";
}

TEST(StreamDecoder, DropCorruptFramesKeepsACleanStreamIntact)
{
    /* The guard must only fire on real damage: an undamaged stream has to decode
       identically whether or not it is armed. */
    auto packets = encodeTestStream(VideoCodec::H264, 320, 240, 20);
    if (packets.size() < 10)
        GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";

    StreamDecoder plain(VideoCodec::H264, softwareOptions());
    const std::size_t baseline = decodeAll(plain, packets).images.size();

    StreamDecoder::Options options = softwareOptions();
    options.drop_corrupt_frames = true;
    StreamDecoder guarded(VideoCodec::H264, options);
    const std::size_t guarded_count = decodeAll(guarded, packets).images.size();

    EXPECT_GT(baseline, 0u);
    EXPECT_EQ(guarded_count, baseline) << "the corrupt-frame guard dropped frames from a clean stream";
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
        /* Decoding may still have moved to another candidate part way through,
           if the device turned out not to handle this profile. Whatever it
           settled on, what it reports has to match: claiming acceleration while
           running on the CPU is the failure this guards against, because it
           looks exactly like a working GPU until someone measures the load. */
        EXPECT_EQ(decoder.isHardwareAccelerated(),
                  decoder.description().find("software") == std::string::npos)
            << "decoder reports " << decoder.description() << " but isHardwareAccelerated() says "
            << decoder.isHardwareAccelerated();
    }
}

/* A hardware decoder that opens but cannot handle the stream must hand it on
   rather than quietly decode on the CPU under a hardware label. The device
   probe decides which half of this runs, so it is meaningful either way. */
TEST(StreamDecoderHardware, ReportedAccelerationMatchesRealityForEveryDevice)
{
    for (const char* device : {"auto", "vaapi", "cuda", "qsv", "vdpau"})
    {
        for (VideoCodec codec : {VideoCodec::H264, VideoCodec::H265})
        {
            StreamDecoder::Options options;
            options.use_hw_decoder = true;
            options.hw_device = device;
            SCOPED_TRACE(std::string(device) + " / " + videoCodecName(codec));

            std::unique_ptr<StreamDecoder> decoder;
            ASSERT_NO_THROW(decoder = std::make_unique<StreamDecoder>(codec, options));

            auto packets = encodeTestStream(codec, 320, 240, 8);
            if (packets.empty())
                continue;
            DecodeResult result = decodeAll(*decoder, packets);
            /* Whichever candidate it ends on, the stream still has to decode. */
            EXPECT_GE(result.images.size(), 4u) << "frames were lost while settling on a decoder";
            EXPECT_EQ(decoder->isHardwareAccelerated(),
                      decoder->description().find("software") == std::string::npos)
                << "reported " << decoder->description();
        }
    }
}
