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
#ifndef RTSP_IMAGE_TRANSPORT_TEST_HELPERS_H_
#define RTSP_IMAGE_TRANSPORT_TEST_HELPERS_H_

#include "video_codec.h"

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cstdint>
#include <string>
#include <vector>

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavutil/opt.h>
}

namespace rtsp_image_transport
{
namespace test
{

/* Arbitrary but fixed instant, so expected time stamps can be written out */
constexpr std::int64_t BASE_STAMP_NS = 1700000000000000000LL;
constexpr std::int64_t FRAME_INTERVAL_NS = 33333333LL;

/* A smoothly varying pattern. Smooth matters: lossy codecs mangle noise badly
   enough that comparing decoded output against the input becomes meaningless. */
inline sensor_msgs::msg::Image makeTestImage(unsigned width, unsigned height, unsigned seq,
                                             const std::string& encoding = sensor_msgs::image_encodings::BGR8)
{
    sensor_msgs::msg::Image img;
    img.width = width;
    img.height = height;
    img.encoding = encoding;
    img.is_bigendian = false;
    unsigned channels = encoding == sensor_msgs::image_encodings::MONO8 ? 1 : 3;
    if (encoding == sensor_msgs::image_encodings::RGBA8 || encoding == sensor_msgs::image_encodings::BGRA8)
        channels = 4;
    img.step = channels * width;
    img.data.resize(static_cast<std::size_t>(img.step) * height);
    for (unsigned y = 0; y < height; ++y)
    {
        for (unsigned x = 0; x < width; ++x)
        {
            std::uint8_t* px = img.data.data() + static_cast<std::size_t>(y) * img.step + x * channels;
            for (unsigned c = 0; c < channels; ++c)
                px[c] = static_cast<std::uint8_t>((x / 4 + y / 4 + seq * 3 + c * 40) & 0xff);
        }
    }
    img.header.stamp = rclcpp::Time(BASE_STAMP_NS + static_cast<std::int64_t>(seq) * FRAME_INTERVAL_NS);
    return img;
}

/* FFmpeg encoder name usable to produce a test stream for the codec, or nullptr
   when this build cannot create one. */
inline const char* testEncoderFor(VideoCodec codec)
{
    static const struct
    {
        VideoCodec codec;
        const char* names[4];
    } table[] = {{VideoCodec::H264, {"libx264", "h264", nullptr, nullptr}},
                 {VideoCodec::H265, {"libx265", "hevc", nullptr, nullptr}},
                 {VideoCodec::MPEG4, {"mpeg4", "libxvid", nullptr, nullptr}},
                 {VideoCodec::MPEG2, {"mpeg2video", nullptr, nullptr, nullptr}},
                 {VideoCodec::H263, {"h263", nullptr, nullptr, nullptr}},
                 {VideoCodec::VP8, {"libvpx", "vp8", nullptr, nullptr}},
                 {VideoCodec::VP9, {"libvpx-vp9", "vp9", nullptr, nullptr}},
                 {VideoCodec::AV1, {"libaom-av1", "librav1e", nullptr, nullptr}},
                 {VideoCodec::MJPEG, {"mjpeg", nullptr, nullptr, nullptr}}};
    for (const auto& entry : table)
    {
        if (entry.codec != codec)
            continue;
        for (const char* name : entry.names)
        {
            if (name && avcodec_find_encoder_by_name(name))
                return name;
        }
    }
    return nullptr;
}

/* Encodes a short synthetic clip and returns the elementary stream packets,
   which is what the RTSP client would hand to the decoder. Empty if this
   FFmpeg build has no encoder for the codec. */
inline std::vector<std::vector<std::uint8_t>> encodeTestStream(VideoCodec codec, int width, int height,
                                                               int frame_count)
{
    std::vector<std::vector<std::uint8_t>> packets;
    const char* encoder_name = testEncoderFor(codec);
    if (!encoder_name)
        return packets;
    const AVCodec* encoder = avcodec_find_encoder_by_name(encoder_name);
    AVCodecContext* ctx = avcodec_alloc_context3(encoder);
    if (!ctx)
        return packets;
    ctx->width = width;
    ctx->height = height;
    ctx->pix_fmt = encoder->id == AV_CODEC_ID_MJPEG ? AV_PIX_FMT_YUVJ420P : AV_PIX_FMT_YUV420P;
    ctx->time_base = AVRational{1, 30};
    ctx->framerate = AVRational{30, 1};
    ctx->gop_size = 10;
    ctx->max_b_frames = 0;
    ctx->bit_rate = 4000000;
    if (std::string(encoder_name).find("x26") != std::string::npos)
    {
        av_opt_set(ctx->priv_data, "preset", "ultrafast", 0);
        av_opt_set(ctx->priv_data, "tune", "zerolatency", 0);
    }
    if (std::string(encoder_name).find("vpx") != std::string::npos)
        av_opt_set(ctx->priv_data, "deadline", "realtime", 0);
    if (std::string(encoder_name).find("aom") != std::string::npos)
    {
        av_opt_set(ctx->priv_data, "cpu-used", "8", 0);
        av_opt_set(ctx->priv_data, "usage", "realtime", 0);
    }
    if (avcodec_open2(ctx, nullptr, nullptr) < 0)
    {
        avcodec_free_context(&ctx);
        return packets;
    }
    AVFrame* frame = av_frame_alloc();
    frame->format = ctx->pix_fmt;
    frame->width = width;
    frame->height = height;
    av_frame_get_buffer(frame, 0);
    AVPacket* packet = av_packet_alloc();
    for (int i = 0; i < frame_count; ++i)
    {
        av_frame_make_writable(frame);
        for (int y = 0; y < height; ++y)
            for (int x = 0; x < width; ++x)
                frame->data[0][y * frame->linesize[0] + x] = static_cast<std::uint8_t>((x / 4 + y / 4 + i * 3) & 0xff);
        for (int plane = 1; plane < 3; ++plane)
            for (int y = 0; y < (height + 1) / 2; ++y)
                for (int x = 0; x < (width + 1) / 2; ++x)
                    frame->data[plane][y * frame->linesize[plane] + x] = static_cast<std::uint8_t>(96 + i * 2);
        frame->pts = i;
        if (avcodec_send_frame(ctx, frame) < 0)
            break;
        while (avcodec_receive_packet(ctx, packet) == 0)
        {
            packets.emplace_back(packet->data, packet->data + packet->size);
            av_packet_unref(packet);
        }
    }
    avcodec_send_frame(ctx, nullptr);
    while (avcodec_receive_packet(ctx, packet) == 0)
    {
        packets.emplace_back(packet->data, packet->data + packet->size);
        av_packet_unref(packet);
    }
    av_packet_free(&packet);
    av_frame_free(&frame);
    avcodec_free_context(&ctx);
    return packets;
}

/* Mean absolute difference per channel between two BGR8 images of equal size */
inline double meanAbsoluteDifference(const sensor_msgs::msg::Image& a, const sensor_msgs::msg::Image& b)
{
    if (a.data.size() != b.data.size() || a.data.empty())
        return 255.0;
    double total = 0.0;
    for (std::size_t i = 0; i < a.data.size(); ++i)
        total += std::abs(static_cast<int>(a.data[i]) - static_cast<int>(b.data[i]));
    return total / static_cast<double>(a.data.size());
}

}  // namespace test
}  // namespace rtsp_image_transport

#endif
