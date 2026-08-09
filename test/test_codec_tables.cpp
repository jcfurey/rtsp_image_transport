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

/* The encoder and decoder candidate lists are plain tables of FFmpeg names, and
   a name in the wrong row is invisible until it runs: the codec opens happily
   and emits a bitstream nobody downstream can read. That is what mjpeg_vaapi
   and mjpeg_qsv did under MPEG-4 — an iGPU has both, so the publisher produced
   MJPEG and announced it as MPEG-4 Part 2, while a headless builder skipped
   them for lack of a device and never noticed.

   Checking the tables against libavcodec costs nothing and needs no hardware:
   avcodec_find_encoder_by_name() reports the codec ID whether or not a device
   for it exists. */

#include "stream_decoder.h"
#include "stream_encoder.h"
#include "video_codec.h"

#include <gtest/gtest.h>

extern "C"
{
#include <libavcodec/avcodec.h>
}

#include <map>
#include <string>
#include <vector>

using namespace rtsp_image_transport;

namespace
{

/* What libavcodec calls each of the transport's codecs. */
AVCodecID expectedCodecId(VideoCodec codec)
{
    switch (codec)
    {
        case VideoCodec::H264:
            return AV_CODEC_ID_H264;
        case VideoCodec::H265:
            return AV_CODEC_ID_HEVC;
        case VideoCodec::MPEG4:
            return AV_CODEC_ID_MPEG4;
        case VideoCodec::MPEG2:
            return AV_CODEC_ID_MPEG2VIDEO;
        case VideoCodec::H263:
            return AV_CODEC_ID_H263;
        case VideoCodec::VP8:
            return AV_CODEC_ID_VP8;
        case VideoCodec::VP9:
            return AV_CODEC_ID_VP9;
        case VideoCodec::AV1:
            return AV_CODEC_ID_AV1;
        case VideoCodec::MJPEG:
            return AV_CODEC_ID_MJPEG;
        default:
            return AV_CODEC_ID_NONE;
    }
}

const VideoCodec ALL_CODECS[] = {VideoCodec::H264, VideoCodec::H265,  VideoCodec::MPEG4, VideoCodec::MPEG2,
                                 VideoCodec::H263, VideoCodec::VP8,   VideoCodec::VP9,   VideoCodec::AV1,
                                 VideoCodec::MJPEG};

}  // namespace

TEST(CodecTables, EveryEncoderEncodesTheCodecItIsListedUnder)
{
    bool checked_any = false;
    for (VideoCodec codec : ALL_CODECS)
    {
        const AVCodecID want = expectedCodecId(codec);
        ASSERT_NE(want, AV_CODEC_ID_NONE) << videoCodecName(codec);
        for (const std::string& name : StreamEncoder::candidateEncoderNames(codec))
        {
            const AVCodec* encoder = avcodec_find_encoder_by_name(name.c_str());
            if (!encoder)
                continue;  // not in this FFmpeg build, so it can never be picked
            checked_any = true;
            EXPECT_EQ(encoder->id, want)
                << name << " is listed as an encoder for " << videoCodecName(codec) << " but encodes "
                << avcodec_get_name(encoder->id);
        }
    }
    EXPECT_TRUE(checked_any) << "no encoder from any table exists in this FFmpeg build";
}

TEST(CodecTables, EveryDecoderDecodesTheCodecItIsListedUnder)
{
    bool checked_any = false;
    for (VideoCodec codec : ALL_CODECS)
    {
        const AVCodecID want = expectedCodecId(codec);
        ASSERT_NE(want, AV_CODEC_ID_NONE) << videoCodecName(codec);
        for (const std::string& name : StreamDecoder::candidateDecoderNames(codec))
        {
            const AVCodec* decoder = avcodec_find_decoder_by_name(name.c_str());
            if (!decoder)
                continue;
            checked_any = true;
            EXPECT_EQ(decoder->id, want)
                << name << " is listed as a decoder for " << videoCodecName(codec) << " but decodes "
                << avcodec_get_name(decoder->id);
        }
    }
    EXPECT_TRUE(checked_any) << "no decoder from any table exists in this FFmpeg build";
}

/* A name that appears twice would silently be tried twice, and a codec with no
   entry at all cannot be served even in software. */
TEST(CodecTables, TablesHaveNoDuplicatesAndCoverEveryDecodableCodec)
{
    for (VideoCodec codec : ALL_CODECS)
    {
        SCOPED_TRACE(videoCodecName(codec));
        const std::vector<std::string> decoders = StreamDecoder::candidateDecoderNames(codec);
        EXPECT_FALSE(decoders.empty()) << "no decoder is offered for a codec the transport accepts";

        std::map<std::string, int> seen;
        for (const std::string& name : decoders)
            seen[name]++;
        for (const auto& [name, count] : seen)
            EXPECT_EQ(count, 1) << name << " appears " << count << " times";

        std::map<std::string, int> seen_encoders;
        for (const std::string& name : StreamEncoder::candidateEncoderNames(codec))
            seen_encoders[name]++;
        for (const auto& [name, count] : seen_encoders)
            EXPECT_EQ(count, 1) << name << " appears " << count << " times";
    }
}

/* Whatever the tables say, constructing a software encoder for a codec this
   build can encode must land on an encoder for that codec. */
TEST(CodecTables, SoftwareEncoderSelectionMatchesTheRequestedCodec)
{
    for (VideoCodec codec : ALL_CODECS)
    {
        SCOPED_TRACE(videoCodecName(codec));
        std::unique_ptr<StreamEncoder> encoder;
        try
        {
            encoder = std::make_unique<StreamEncoder>(codec, /*use_hw_encoder=*/false);
        }
        catch (const std::exception&)
        {
            continue;  // this build cannot encode it at all
        }
        ASSERT_NE(encoder->context(), nullptr);
        ASSERT_NE(encoder->context()->codec, nullptr);
        EXPECT_EQ(encoder->context()->codec->id, expectedCodecId(codec))
            << "selected " << encoder->context()->codec->name;
    }
}
