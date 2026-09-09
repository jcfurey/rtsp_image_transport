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

#include "stream_decoder.h"
#include "streaming_error.h"
#include "test_helpers.h"

#include <gtest/gtest.h>

#include <sensor_msgs/image_encodings.hpp>

#include <cstring>
#include <iterator>
#include <limits>
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

sensor_msgs::msg::Image makeSemiplanarImage(const std::string& encoding, unsigned padding, unsigned sequence)
{
    auto image = makeTestImage(160, 120, sequence);
    image.encoding = encoding;
    image.step = image.width + padding;
    const std::size_t luma_bytes = image.step * image.height;
    const std::size_t chroma_stride = encoding == "nv24" ? 2 * image.step : image.step;
    const unsigned chroma_height = encoding == "nv24" ? image.height : image.height / 2;
    image.data.assign(luma_bytes + chroma_stride * chroma_height, 0);
    for (unsigned y = 0; y < image.height; ++y)
        std::fill_n(image.data.begin() + y * image.step, image.width, 81);
    // BT.601 limited-range red: Y=81, U=90, V=240. Padding deliberately
    // differs from the pixels so treating padded rows as packed corrupts color.
    const unsigned chroma_width = encoding == "nv24" ? 2 * image.width : image.width;
    for (unsigned y = 0; y < chroma_height; ++y)
        for (unsigned x = 0; x < chroma_width; x += 2)
        {
            image.data[luma_bytes + y * chroma_stride + x] = encoding == "nv21" ? 240 : 90;
            image.data[luma_bytes + y * chroma_stride + x + 1] = encoding == "nv21" ? 90 : 240;
        }
    return image;
}

sensor_msgs::msg::Image::UniquePtr decodePendingImage(StreamEncoder& encoder, StreamDecoder& decoder,
                                                     const sensor_msgs::msg::Image& input)
{
    std::vector<unsigned char> annex_b;
    while (auto packet = encoder.nextPacket())
    {
        EXPECT_EQ(packet->stamp(), rclcpp::Time(input.header.stamp));
        annex_b.insert(annex_b.end(), {0, 0, 0, 1});
        annex_b.insert(annex_b.end(), packet->data(), packet->data() + packet->length());
    }
    if (annex_b.empty())
    {
        ADD_FAILURE() << "the encoder lost the pending image";
        return nullptr;
    }
    EXPECT_GT(decoder.decodeVideo(
                  std::make_shared<FrameData>(annex_b.data(), annex_b.size(), input.header.stamp)),
              0u);
    return decoder.nextFrame();
}

void expectSolidBgr(const sensor_msgs::msg::Image& image, int blue, int green, int red)
{
    ASSERT_EQ(image.encoding, "bgr8");
    ASSERT_EQ(image.width, 160u);
    ASSERT_EQ(image.height, 120u);
    ASSERT_GE(image.data.size(), static_cast<std::size_t>(image.step) * image.height);
    const int expected[] = {blue, green, red};
    for (const unsigned y : {5u, image.height / 2, image.height - 6})
        for (const unsigned x : {5u, image.width / 2, image.width - 6})
            for (unsigned channel = 0; channel < 3; ++channel)
                EXPECT_NEAR(image.data[y * image.step + x * 3 + channel], expected[channel], 15)
                    << "pixel " << x << "," << y << " channel " << channel;
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

TEST(StreamEncoder, KeyframeIntervalOverridesTheFrameRateDefault)
{
    auto encoder = makeEncoder();
    if (!encoder)
        GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
    /* setFramerate() derives one key frame per second, which is the default */
    encoder->setFramerate(25);
    EXPECT_EQ(encoder->context()->gop_size, 25);
    encoder->setKeyframeInterval(8);
    EXPECT_EQ(encoder->context()->gop_size, 8);
    /* Zero means "follow the frame rate again" rather than "never" — a gop of
       zero would ask the encoder for an unbounded interval. */
    encoder->setKeyframeInterval(0);
    EXPECT_EQ(encoder->context()->gop_size, 25);
}

TEST(StreamEncoder, KeyframeIntervalIsLockedOnceEncodingStarted)
{
    auto encoder = makeEncoder();
    if (!encoder)
        GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
    ASSERT_NO_THROW(encoder->encodeVideo(makeTestImage(160, 120, 0)));
    EXPECT_THROW(encoder->setKeyframeInterval(10), StreamingError);
}

TEST(StreamEncoder, ShorterKeyframeIntervalProducesMoreKeyFrames)
{
    /* The parameter has to reach the bitstream, not just the context. Counting
       IDR NAL units is the check: type 5 in the H.264 NAL header. */
    auto count_idr = [](unsigned interval)
    {
        auto encoder = makeEncoder();
        if (!encoder)
            return std::size_t{0};
        encoder->setFramerate(30);
        encoder->setKeyframeInterval(interval);
        std::size_t idr = 0;
        for (unsigned i = 0; i < 60; ++i)
        {
            encoder->encodeVideo(makeTestImage(160, 120, i));
            while (FrameDataPtr packet = encoder->nextPacket())
            {
                if (packet->length() > 0 && (packet->data()[0] & 0x1F) == 5)
                    idr++;
            }
        }
        return idr;
    };
    const std::size_t sparse = count_idr(30);
    const std::size_t dense = count_idr(6);
    if (sparse == 0 && dense == 0)
        GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
    EXPECT_GT(dense, sparse) << "a shorter key frame interval produced no extra key frames";
}

TEST(StreamEncoder, IntraRefreshReplacesPeriodicKeyFrames)
{
    /* The setting has to reach the bitstream, and x264 makes that easy to get
       wrong: it refuses intra refresh when more than one reference frame is
       allowed and says so only in a log line, carrying on without it. Counting
       IDR NAL units is what catches that — with a rolling refresh there should
       be the one at the start of the stream and no more. */
    auto count_idr = [](bool intra_refresh, bool* supported)
    {
        auto encoder = makeEncoder();
        if (!encoder)
            return std::size_t{0};
        encoder->setBitrate(2000000);
        encoder->setFramerate(30);
        if (intra_refresh)
            *supported = encoder->setIntraRefresh(true);
        encoder->setPackageSizeHint(1330);
        std::size_t idr = 0;
        for (unsigned i = 0; i < 90; ++i)
        {
            encoder->encodeVideo(makeTestImage(320, 240, i));
            while (FrameDataPtr packet = encoder->nextPacket())
            {
                if (packet->length() > 0 && (packet->data()[0] & 0x1F) == 5)
                    idr++;
            }
        }
        return idr;
    };
    bool supported = true, unused = true;
    const std::size_t periodic = count_idr(false, &unused);
    if (periodic == 0)
        GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
    const std::size_t rolling = count_idr(true, &supported);
    if (!supported)
        GTEST_SKIP() << "this encoder has no intra refresh";
    EXPECT_LT(rolling, periodic)
        << "intra refresh produced " << rolling << " IDR NAL units against " << periodic
        << " for periodic key frames; the setting did not reach the encoder";
}

TEST(StreamEncoder, IntraRefreshIsLockedOnceEncodingStarted)
{
    auto encoder = makeEncoder();
    if (!encoder)
        GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
    ASSERT_NO_THROW(encoder->encodeVideo(makeTestImage(160, 120, 0)));
    EXPECT_THROW(encoder->setIntraRefresh(true), StreamingError);
}

TEST(StreamEncoder, IntraRefreshStreamStillDecodes)
{
    /* Join after the only IDR. The subscriber deliberately withholds startup
       pictures while waiting for a key frame, but its bounded fallback must
       eventually let a rolling-refresh stream through. This used to claim to
       test decoding while only counting encoder packets. */
    auto encoder = makeEncoder();
    if (!encoder)
        GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
    encoder->setBitrate(2000000);
    encoder->setFramerate(30);
    if (!encoder->setIntraRefresh(true))
        GTEST_SKIP() << "this encoder has no intra refresh";

    StreamDecoder::Options options;
    options.use_hw_decoder = false;
    options.hw_device = "none";
    StreamDecoder decoder(VideoCodec::H264, options);
    std::vector<std::uint8_t> parameter_sets;
    std::size_t images = 0;
    constexpr unsigned JOIN_AT = 10;
    /* Allow one partial sweep to be discarded before FFmpeg begins emitting
       pictures, plus the decoder's 120-picture key-frame safety bound. */
    constexpr unsigned FRAMES = 300;
    const std::uint8_t start_code[] = {0, 0, 0, 1};

    for (unsigned i = 0; i < FRAMES; ++i)
    {
        encoder->encodeVideo(makeTestImage(160, 120, i));
        std::vector<std::uint8_t> access_unit;
        while (FrameDataPtr packet = encoder->nextPacket())
        {
            const std::uint8_t type = packet->length() > 0 ? packet->data()[0] & 0x1f : 0;
            if (i == 0 && (type == 7 || type == 8))
            {
                parameter_sets.insert(parameter_sets.end(), std::begin(start_code), std::end(start_code));
                parameter_sets.insert(parameter_sets.end(), packet->data(), packet->data() + packet->length());
            }
            if (i >= JOIN_AT)
            {
                access_unit.insert(access_unit.end(), std::begin(start_code), std::end(start_code));
                access_unit.insert(access_unit.end(), packet->data(), packet->data() + packet->length());
            }
        }
        if (i < JOIN_AT || access_unit.empty())
            continue;
        if (i == JOIN_AT)
            access_unit.insert(access_unit.begin(), parameter_sets.begin(), parameter_sets.end());
        try
        {
            decoder.decodeVideo(std::make_shared<FrameData>(
                access_unit.data(), access_unit.size(), rclcpp::Time(BASE_STAMP_NS + i * FRAME_INTERVAL_NS)));
        }
        catch (const DecodingError&)
        {
            /* Missing pre-join references are expected until the refresh sweep
               has replaced them. */
        }
        while (decoder.nextFrame())
            ++images;
    }

    ASSERT_FALSE(parameter_sets.empty()) << "the initial access unit carried no SPS/PPS";
    EXPECT_GT(images, 0u) << "a late decoder never recovered from the intra-refresh sweep";
    EXPECT_FALSE(decoder.awaitingKeyframe());
}

TEST(StreamEncoder, X265IntraRefreshUsesItsParameterDictionary)
{
    auto encoder = makeEncoder(VideoCodec::H265);
    if (!encoder)
        GTEST_SKIP() << "no H.265 encoder in this FFmpeg build";
    if (!strstr(encoder->context()->codec->name, "x265"))
        GTEST_SKIP() << "the selected software H.265 encoder is not x265";

    EXPECT_TRUE(encoder->setIntraRefresh(true));
    EXPECT_GT(encodeClip(*encoder, 160, 120, 5), 0u);
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

TEST(StreamEncoder, RejectsMalformedBuffersWithoutPoisoningTheEncoder)
{
    auto encoder = makeEncoder();
    if (!encoder)
        GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
    const auto valid = makeTestImage(160, 120, 0);
    for (int fault = 0; fault < 7; ++fault)
    {
        auto image = valid;
        switch (fault)
        {
        case 0:
            image.data.clear();
            break;
        case 1:
            image.data.pop_back();
            break;
        case 2:
            image.step = image.width * 3 - 1;
            break;
        case 3:
            image.step = UINT32_MAX;
            break;
        case 4:
            image.width = UINT32_MAX;
            break;
        case 5:
            image.height = 0;
            break;
        case 6:
            image.encoding = "invalid";
            break;
        }
        SCOPED_TRACE(fault);
        EXPECT_THROW(encoder->encodeVideo(image), InvalidImageError);
        EXPECT_GT(encoder->encodeVideo(valid), 0u);
        while (encoder->nextPacket())
        {
        }
    }
}

TEST(StreamEncoder, SemiplanarYuvUsesChromaAndPaddedRows)
{
    for (const std::string encoding : {"nv12", "nv21", "nv24"})
    {
        SCOPED_TRACE(encoding);
        auto encoder = makeEncoder();
        if (!encoder)
            GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
        auto image = makeSemiplanarImage(encoding, 16, 0);
        auto truncated = image;
        truncated.data.pop_back();
        EXPECT_THROW(encoder->encodeVideo(truncated), InvalidImageError);
        ASSERT_GT(encoder->encodeVideo(image), 0u);
        std::vector<unsigned char> annex_b;
        while (auto packet = encoder->nextPacket())
        {
            annex_b.insert(annex_b.end(), {0, 0, 0, 1});
            annex_b.insert(annex_b.end(), packet->data(), packet->data() + packet->length());
        }
        StreamDecoder::Options options;
        options.use_hw_decoder = false;
        StreamDecoder decoder(VideoCodec::H264, options);
        ASSERT_GT(
            decoder.decodeVideo(std::make_shared<FrameData>(annex_b.data(), annex_b.size(), image.header.stamp)),
            0u);
        const auto decoded = decoder.nextFrame();
        ASSERT_TRUE(decoded);
        const std::size_t center = (60 * decoded->width + 80) * 3;
        EXPECT_LT(decoded->data[center], 15);
        EXPECT_LT(decoded->data[center + 1], 15);
        EXPECT_GT(decoded->data[center + 2], 240);
    }
}

class SemiplanarInput : public ::testing::TestWithParam<std::string>
{
};

TEST_P(SemiplanarInput, RejectsMalformedFramesBeforeAndAfterStartupAndRecovers)
{
    for (const bool started : {false, true})
        for (unsigned fault = 0; fault < 5; ++fault)
        {
            SCOPED_TRACE(::testing::Message() << "started=" << started << " fault=" << fault);
            auto encoder = makeEncoder();
            if (!encoder)
                GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
            StreamDecoder::Options options;
            options.use_hw_decoder = false;
            StreamDecoder decoder(VideoCodec::H264, options);
            if (started)
            {
                const auto first = makeSemiplanarImage(GetParam(), 16, 0);
                ASSERT_GT(encoder->encodeVideo(first), 0u);
                ASSERT_TRUE(decodePendingImage(*encoder, decoder, first));
            }
            auto malformed = makeSemiplanarImage(GetParam(), 16, 1);
            switch (fault)
            {
            case 0:  // The luma plane alone is not a complete camera frame.
                malformed.data.resize(malformed.step * malformed.height);
                break;
            case 1:
                malformed.data.pop_back();
                break;
            case 2:
                malformed.step = malformed.width - 1;
                break;
            case 3:
                malformed.step = UINT32_MAX;
                break;
            case 4:
                if (GetParam() == "nv24")
                    malformed.step = static_cast<unsigned>(std::numeric_limits<int>::max()) / 2 + 1;
                else
                {
                    // Odd-width NV12/NV21 need an extra byte in the chroma row.
                    malformed.width = 161;
                    malformed.step = 161;
                }
                break;
            }
            EXPECT_THROW(encoder->encodeVideo(malformed), InvalidImageError);
            EXPECT_FALSE(encoder->nextPacket());
            const auto good = makeSemiplanarImage(GetParam(), 16, 2);
            ASSERT_GT(encoder->encodeVideo(good), 0u);
            const auto decoded = decodePendingImage(*encoder, decoder, good);
            ASSERT_TRUE(decoded);
            expectSolidBgr(*decoded, 0, 0, 255);
        }
}

TEST_P(SemiplanarInput, ChangingRowPaddingPreservesChromaWithoutRestarting)
{
    auto encoder = makeEncoder();
    if (!encoder)
        GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
    StreamDecoder::Options options;
    options.use_hw_decoder = false;
    StreamDecoder decoder(VideoCodec::H264, options);
    unsigned sequence = 0;
    for (const unsigned padding : {0u, 16u, 2u, 0u})
    {
        SCOPED_TRACE(padding);
        const auto image = makeSemiplanarImage(GetParam(), padding, sequence++);
        ASSERT_GT(encoder->encodeVideo(image), 0u);
        const auto decoded = decodePendingImage(*encoder, decoder, image);
        ASSERT_TRUE(decoded);
        expectSolidBgr(*decoded, 0, 0, 255);
    }
}

TEST_P(SemiplanarInput, RejectingAnImagePreservesUnreadPacketsFromThePreviousImage)
{
    auto encoder = makeEncoder();
    if (!encoder)
        GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
    const auto first = makeSemiplanarImage(GetParam(), 0, 0);
    ASSERT_GT(encoder->encodeVideo(first), 0u);
    auto bad = makeSemiplanarImage(GetParam(), 16, 1);
    bad.data.pop_back();
    EXPECT_THROW(encoder->encodeVideo(bad), InvalidImageError);
    StreamDecoder::Options options;
    options.use_hw_decoder = false;
    StreamDecoder decoder(VideoCodec::H264, options);
    const auto decoded = decodePendingImage(*encoder, decoder, first);
    ASSERT_TRUE(decoded);
    expectSolidBgr(*decoded, 0, 0, 255);
    const auto next = makeSemiplanarImage(GetParam(), 16, 2);
    ASSERT_GT(encoder->encodeVideo(next), 0u);
    ASSERT_TRUE(decodePendingImage(*encoder, decoder, next));
}

// Use wire strings so these regressions also compile with Jazzy headers,
// where sensor_msgs::image_encodings::NV12 is absent.
INSTANTIATE_TEST_SUITE_P(Encodings, SemiplanarInput, ::testing::Values("nv12", "nv21", "nv24"),
                         [](const ::testing::TestParamInfo<std::string>& info) { return info.param; });

TEST(StreamEncoder, ChangingInputFormatPreservesColorWithinOneStream)
{
    auto encoder = makeEncoder();
    if (!encoder)
        GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
    StreamDecoder::Options options;
    options.use_hw_decoder = false;
    StreamDecoder decoder(VideoCodec::H264, options);
    unsigned sequence = 0;
    for (const std::string encoding : {"nv12", "nv21", "nv24", "bgr8", "nv21", "nv12"})
    {
        SCOPED_TRACE(encoding);
        const bool blue = encoding == "bgr8";
        auto image = blue ? makeTestImage(160, 120, sequence++)
                          : makeSemiplanarImage(encoding, 16, sequence++);
        if (blue)
        {
            std::fill(image.data.begin(), image.data.end(), 0);
            for (std::size_t offset = 0; offset < image.data.size(); offset += 3)
                image.data[offset] = 255;
        }
        ASSERT_GT(encoder->encodeVideo(image), 0u);
        const auto decoded = decodePendingImage(*encoder, decoder, image);
        ASSERT_TRUE(decoded);
        expectSolidBgr(*decoded, blue ? 255 : 0, 0, blue ? 0 : 255);
    }
}

TEST(StreamEncoder, ChangingRgb16ByteOrderPreservesColorWithinOneStream)
{
    auto encoder = makeEncoder();
    if (!encoder)
        GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
    StreamDecoder::Options options;
    options.use_hw_decoder = false;
    StreamDecoder decoder(VideoCodec::H264, options);
    unsigned sequence = 0;
    for (const bool big_endian : {false, true, false})
    {
        SCOPED_TRACE(big_endian);
        auto image = makeTestImage(160, 120, sequence++);
        image.encoding = "rgb16";
        image.is_bigendian = big_endian;
        image.step = image.width * 6 + 16;
        image.data.assign(image.step * image.height, 0);
        // Unequal bytes expose swapped endianness; RGB primaries at 0 or 65535 do not.
        const std::uint16_t rgb[] = {0xd123, 0x6345, 0x2356};
        for (unsigned y = 0; y < image.height; ++y)
            for (unsigned x = 0; x < image.width; ++x)
                for (unsigned channel = 0; channel < 3; ++channel)
                {
                    const auto offset = y * image.step + x * 6 + channel * 2;
                    image.data[offset + (big_endian ? 0 : 1)] = rgb[channel] >> 8;
                    image.data[offset + (big_endian ? 1 : 0)] = rgb[channel] & 0xff;
                }
        ASSERT_GT(encoder->encodeVideo(image), 0u);
        const auto decoded = decodePendingImage(*encoder, decoder, image);
        ASSERT_TRUE(decoded);
        expectSolidBgr(*decoded, 35, 99, 209);
    }
}
