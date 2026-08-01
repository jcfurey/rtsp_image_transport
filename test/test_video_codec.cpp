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
#include "video_codec.h"

#include <gtest/gtest.h>

using namespace rtsp_image_transport;

TEST(VideoCodec, MapsRtspPayloadNames)
{
    EXPECT_EQ(fromRTSPCodecName("H264"), VideoCodec::H264);
    EXPECT_EQ(fromRTSPCodecName("H265"), VideoCodec::H265);
    EXPECT_EQ(fromRTSPCodecName("MP4V-ES"), VideoCodec::MPEG4);
    EXPECT_EQ(fromRTSPCodecName("VP8"), VideoCodec::VP8);
    EXPECT_EQ(fromRTSPCodecName("VP9"), VideoCodec::VP9);
    EXPECT_EQ(fromRTSPCodecName("AV1"), VideoCodec::AV1);
    EXPECT_EQ(fromRTSPCodecName("JPEG"), VideoCodec::MJPEG);
}

TEST(VideoCodec, MapsLegacyPayloadNames)
{
    /* RFC 2190, RFC 2429 and RFC 4629 spell H.263 differently and cameras use
       all three. RFC 2250 names an elementary MPEG-1/2 stream MPV. */
    EXPECT_EQ(fromRTSPCodecName("H263"), VideoCodec::H263);
    EXPECT_EQ(fromRTSPCodecName("H263-1998"), VideoCodec::H263);
    EXPECT_EQ(fromRTSPCodecName("H263-2000"), VideoCodec::H263);
    EXPECT_EQ(fromRTSPCodecName("MPV"), VideoCodec::MPEG2);
}

TEST(VideoCodec, RejectsUnknownAndAudioPayloadNames)
{
    EXPECT_EQ(fromRTSPCodecName(""), VideoCodec::Unknown);
    EXPECT_EQ(fromRTSPCodecName("PCMU"), VideoCodec::Unknown);
    EXPECT_EQ(fromRTSPCodecName("MPEG4-GENERIC"), VideoCodec::Unknown);
    EXPECT_EQ(fromRTSPCodecName("OPUS"), VideoCodec::Unknown);
    EXPECT_EQ(fromRTSPCodecName("SOMETHING-ELSE"), VideoCodec::Unknown);
}

TEST(VideoCodec, PayloadNameMatchingIsCaseSensitive)
{
    /* SDP payload names are upper case; accepting other spellings would risk
       matching an unrelated payload type. */
    EXPECT_EQ(fromRTSPCodecName("h264"), VideoCodec::Unknown);
    EXPECT_EQ(fromRTSPCodecName("Jpeg"), VideoCodec::Unknown);
}

TEST(VideoCodec, EveryCodecHasADistinctDisplayName)
{
    const VideoCodec all[] = {VideoCodec::H264,  VideoCodec::H265,  VideoCodec::MPEG4,
                              VideoCodec::VP8,   VideoCodec::VP9,   VideoCodec::AV1,
                              VideoCodec::MJPEG, VideoCodec::MPEG2, VideoCodec::H263};
    std::set<std::string> names;
    for (VideoCodec codec : all)
    {
        std::string name = videoCodecName(codec);
        EXPECT_FALSE(name.empty());
        EXPECT_NE(name, "unsupported codec") << "missing display name for codec " << static_cast<int>(codec);
        EXPECT_TRUE(names.insert(name).second) << "duplicate display name " << name;
    }
    EXPECT_EQ(videoCodecName(VideoCodec::Unknown), "unsupported codec");
}

TEST(VideoCodec, PayloadNamesRoundTripToDistinctCodecs)
{
    /* No two payload names may collapse onto the same codec unless they are
       deliberate aliases, and no payload name may map to Unknown by accident. */
    const std::pair<const char*, VideoCodec> pairs[] = {
        {"H264", VideoCodec::H264},   {"H265", VideoCodec::H265},   {"MP4V-ES", VideoCodec::MPEG4},
        {"MPV", VideoCodec::MPEG2},   {"H263", VideoCodec::H263},   {"VP8", VideoCodec::VP8},
        {"VP9", VideoCodec::VP9},     {"AV1", VideoCodec::AV1},     {"JPEG", VideoCodec::MJPEG}};
    std::set<VideoCodec> seen;
    for (const auto& [name, codec] : pairs)
    {
        EXPECT_EQ(fromRTSPCodecName(name), codec) << name;
        EXPECT_TRUE(seen.insert(codec).second) << "two payload names map to " << videoCodecName(codec);
    }
}
