/****************************************************************************
 *
 * rtsp_image_transport
 * Copyright © 2021-2025 Fraunhofer FKIE
 * Author: Timo Röhling
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

#include <sensor_msgs/image_encodings.hpp>

#include <algorithm>
#include <cstdint>
#include <format>
#include <map>
#include <thread>
#include <vector>

extern "C"
{
#include <libavformat/avformat.h>
#include <libavutil/buffer.h>
#include <libavutil/opt.h>
}

#include <rclcpp/logging.hpp>

namespace rtsp_image_transport
{

namespace
{

/* Presentation time stamps are passed through the decoder as nanoseconds since
   epoch, so decoded frames can be labelled with the time stamp of the access
   unit they actually belong to, and not with the time stamp of whichever packet
   happened to flush them out of the decoder. */
constexpr AVRational NANOSECOND_TIME_BASE{1, 1000000000};

void set_codec_option(std::shared_ptr<AVCodecContext> ctx, const std::string& option, const std::string& value,
                      const rclcpp::Logger& logger = rclcpp::get_logger("ffmpeg"))
{
    int result = av_opt_set(ctx->priv_data, option.c_str(), value.c_str(), 0);
    if (result != 0)
        RCLCPP_WARN(logger, "[%s] cannot set codec option %s=\"%s\"", ctx->codec->name, option.c_str(), value.c_str());
}

void set_codec_option(std::shared_ptr<AVCodecContext> ctx, const std::string& option, int value,
                      const rclcpp::Logger& logger = rclcpp::get_logger("ffmpeg"))
{
    int result = av_opt_set_int(ctx->priv_data, option.c_str(), value, 0);
    if (result != 0)
        RCLCPP_WARN(logger, "[%s] cannot set codec option %s=%d", ctx->codec->name, option.c_str(), value);
}

void free_context(AVCodecContext* ctx)
{
    avcodec_free_context(&ctx);
}

void free_frame(AVFrame* frame)
{
    av_frame_free(&frame);
}

void free_packet(AVPacket* packet)
{
    av_packet_free(&packet);
}

#ifdef FFMPEG_HAS_SWS_THREADS
/* The destination frame borrows the memory of the ROS image message, so there
   is nothing to release when libswscale drops its reference. */
void keep_buffer(void*, unsigned char*) {}
#endif

/* Colour space conversion is the most expensive step of the decoding pipeline
   for large frames, so let libswscale spread it over a few threads if the
   installed version supports it. Sets `threaded` when the returned context
   requires the sws_scale_frame() calling convention; falls back to a plain
   single threaded context otherwise. */
std::shared_ptr<SwsContext> allocScaler(int width, int height, AVPixelFormat src_format, AVPixelFormat dst_format,
                                        bool& threaded)
{
    threaded = false;
#ifdef FFMPEG_HAS_SWS_THREADS
    int threads = static_cast<int>(std::clamp<unsigned>(std::thread::hardware_concurrency(), 1u, 4u));
    if (threads > 1)
    {
        SwsContext* ctx = sws_alloc_context();
        if (ctx)
        {
            av_opt_set_int(ctx, "srcw", width, 0);
            av_opt_set_int(ctx, "srch", height, 0);
            av_opt_set_int(ctx, "src_format", src_format, 0);
            av_opt_set_int(ctx, "dstw", width, 0);
            av_opt_set_int(ctx, "dsth", height, 0);
            av_opt_set_int(ctx, "dst_format", dst_format, 0);
            av_opt_set_int(ctx, "sws_flags", SWS_FAST_BILINEAR, 0);
            av_opt_set_int(ctx, "threads", threads, 0);
            if (sws_init_context(ctx, nullptr, nullptr) >= 0)
            {
                threaded = true;
                return std::shared_ptr<SwsContext>(ctx, sws_freeContext);
            }
            sws_freeContext(ctx);
        }
    }
#endif
    return std::shared_ptr<SwsContext>(sws_getContext(width, height, src_format, width, height, dst_format,
                                                      SWS_FAST_BILINEAR, nullptr, nullptr, nullptr),
                                       sws_freeContext);
}

const std::map<VideoCodec, std::vector<std::string>> FFMPEG_DECODERS{
    {VideoCodec::H264, {"h264_qsv", "h264_cuvid", "h264"}},
    {VideoCodec::H265, {"hevc_qsv", "hevc_cuvid", "hevc", "h265"}},
    {VideoCodec::MPEG4, {"mpeg4_cuvid", "mpeg4"}},
    {VideoCodec::VP8, {"vp8_qsv", "vp8_cuvid", "vp8"}},
    {VideoCodec::VP9, {"vp9_qsv", "vp9_cuvid", "vp9"}},
    {VideoCodec::AV1, {"av1_qsv", "av1_cuvid", "libdav1d", "av1"}},
    {VideoCodec::MJPEG, {"mjpeg_qsv", "mjpeg_cuvid", "mjpeg"}}};

}  // namespace

StreamDecoder::StreamDecoder(VideoCodec codec, bool use_hw_decoder, bool low_latency, const rclcpp::Logger& logger)
    : logger_(logger), codec_(codec), initialized_(false), low_latency_(low_latency), sws_threaded_(false), width_(0),
      height_(0), last_pixel_format_(AV_PIX_FMT_NONE)
{
    auto decoders = FFMPEG_DECODERS.find(codec);
    if (decoders == FFMPEG_DECODERS.end())
        throw StreamingError(std::format("no decoder support available for {}", videoCodecName(codec)));
    for (const std::string& codec_name : decoders->second)
    {
        if (!use_hw_decoder && codec_name.find("_") != std::string::npos)
            continue;
        const AVCodec* decoder = nullptr;
        try
        {
            decoder = avcodec_find_decoder_by_name(codec_name.c_str());
            if (decoder)
            {
                RCLCPP_DEBUG(logger_, "[%s] attempting to initialize decoder", codec_name.c_str());
                setupDecoder(decoder);
                break;
            }
            else
            {
                RCLCPP_DEBUG(logger_, "[%s] not available in your FFmpeg library", codec_name.c_str());
            }
        }
        catch (const std::exception& e)
        {
            ctx_.reset();
            RCLCPP_DEBUG(logger_, "[%s] %s", decoder ? decoder->name : "(nullptr)", e.what());
        }
    }
    if (!ctx_)
        throw StreamingError(std::format("no usable decoder available for {}", videoCodecName(codec)));
}

void StreamDecoder::setupDecoder(const AVCodec* decoder)
{
    ctx_.reset(avcodec_alloc_context3(decoder), free_context);
    if (!ctx_)
        throw StreamingError("failed to initialize decoder context");
    if (strstr(decoder->name, "qsv"))
    {
        set_codec_option(ctx_, "async_depth", 1, logger_);
    }
    ctx_->log_level_offset = 8;  // Turn errors into warnings
    ctx_->pkt_timebase = NANOSECOND_TIME_BASE;
    if (codec_ == VideoCodec::H264 || codec_ == VideoCodec::H265)
    {
        ctx_->flags2 |= AV_CODEC_FLAG2_CHUNKS;
        // The error concealment spams the log with spurious errors
        ctx_->error_concealment = 0;
    }
    if (low_latency_)
    {
        /* Frame level multithreading buffers up to thread_count frames before
           the first one is handed back, which adds several hundred milliseconds
           of latency on a typical multi-core machine. Slice threading keeps the
           speedup without the pipeline delay, and AV_CODEC_FLAG_LOW_DELAY tells
           the decoder to emit every frame as soon as it is complete. */
        ctx_->thread_type = FF_THREAD_SLICE;
        ctx_->flags |= AV_CODEC_FLAG_LOW_DELAY;
    }
    if (avcodec_open2(ctx_.get(), nullptr, 0) < 0)
        throw StreamingError("failed to open decoder");
}

void StreamDecoder::setDecodeFrames(DecodeFrames which) noexcept
{
    switch (which)
    {
        case DecodeFrames::All:
            ctx_->skip_frame = AVDISCARD_DEFAULT;
            break;
        case DecodeFrames::Intra:
            ctx_->skip_frame = AVDISCARD_NONINTRA;
            break;
        case DecodeFrames::Key:
            ctx_->skip_frame = AVDISCARD_NONKEY;
            break;
        case DecodeFrames::None:
            ctx_->skip_frame = AVDISCARD_ALL;
            break;
    }
}

std::size_t StreamDecoder::decodeVideo(const FrameDataPtr& data)
{
    if (!initialized_)
    {
        pkt_.reset(av_packet_alloc(), free_packet);
        frm_.reset(av_frame_alloc(), free_frame);
        bgr_frm_.reset(av_frame_alloc(), free_frame);
        if (!pkt_ || !frm_ || !bgr_frm_)
            throw DecodingError("failed to allocate decoder working buffers");
        frames_.clear();
        initialized_ = true;
    }
    pkt_->data = const_cast<unsigned char*>(data->data());
    pkt_->size = data->length();
    pkt_->pts = data->stamp().nanoseconds();
    pkt_->dts = pkt_->pts;
    int result;
    if ((result = avcodec_send_packet(ctx_.get(), pkt_.get())) != 0)
    {
        char errbuf[80];
        throw DecodingError(std::format("failed to send bitstream packet to decoder: {}",
                                        av_make_error_string(errbuf, sizeof(errbuf), result)));
    }
    std::size_t count = 0;
    while ((result = avcodec_receive_frame(ctx_.get(), frm_.get())) == 0)
    {
        if (!sws_ || frm_->width != width_ || frm_->height != height_ || frm_->format != last_pixel_format_)
        {
            width_ = frm_->width;
            height_ = frm_->height;
            last_pixel_format_ = static_cast<AVPixelFormat>(frm_->format);
            sws_ = allocScaler(width_, height_, last_pixel_format_, AV_PIX_FMT_BGR24, sws_threaded_);
            if (!sws_)
                throw DecodingError(std::format("cannot convert {}x{} frames in pixel format {} to BGR8", width_,
                                                height_, static_cast<int>(last_pixel_format_)));
        }
        /* Prefer the time stamp the decoder propagated with this frame; it
           belongs to the access unit that produced it. */
        std::int64_t frame_pts = frm_->best_effort_timestamp;
        if (frame_pts == AV_NOPTS_VALUE)
            frame_pts = frm_->pts;
        sensor_msgs::msg::Image::UniquePtr img = std::make_unique<sensor_msgs::msg::Image>();
        img->encoding = sensor_msgs::image_encodings::BGR8;
        img->header.stamp = frame_pts != AV_NOPTS_VALUE ? rclcpp::Time(frame_pts) : data->stamp();
        img->width = width_;
        img->height = height_;
        img->is_bigendian = false;
        img->step = 3 * width_;
        img->data.resize(static_cast<std::size_t>(3) * width_ * height_);
        convertToBGR(*img);
        count++;
        frames_.push_back(std::move(img));
    }
    av_frame_unref(frm_.get());
    if (result != AVERROR(EAGAIN))
    {
        char errbuf[80];
        throw DecodingError(std::format("failed to receive frames from decoder: {}",
                                        av_make_error_string(errbuf, sizeof(errbuf), result)));
    }
    return count;
}

void StreamDecoder::convertToBGR(sensor_msgs::msg::Image& img)
{
#ifdef FFMPEG_HAS_SWS_THREADS
    if (sws_threaded_)
    {
        /* sws_scale() only ever runs on the calling thread; the work is spread
           over the worker threads by sws_scale_frame(). It writes into the
           image message directly as long as the destination frame carries a
           reference counted buffer. */
        av_frame_unref(bgr_frm_.get());
        bgr_frm_->format = AV_PIX_FMT_BGR24;
        bgr_frm_->width = width_;
        bgr_frm_->height = height_;
        bgr_frm_->data[0] = img.data.data();
        bgr_frm_->linesize[0] = 3 * width_;
        bgr_frm_->buf[0] = av_buffer_create(img.data.data(), img.data.size(), keep_buffer, nullptr, 0);
        if (!bgr_frm_->buf[0])
            throw DecodingError("failed to reference image buffer for color conversion");
        int result = sws_scale_frame(sws_.get(), bgr_frm_.get(), frm_.get());
        bool wrote_in_place = bgr_frm_->data[0] == img.data.data();
        av_frame_unref(bgr_frm_.get());
        if (result >= 0 && wrote_in_place)
            return;
        /* Should not happen, but rather than fail every frame from here on,
           fall back to the single threaded conversion for the rest of the
           session. The context stays usable either way. */
        char errbuf[80];
        RCLCPP_WARN(logger_, "threaded color conversion unavailable (%s), falling back",
                    result < 0 ? av_make_error_string(errbuf, sizeof(errbuf), result) : "buffer was reallocated");
        sws_threaded_ = false;
    }
#endif
    unsigned char* bgr_data[] = {img.data.data()};
    int bgr_linesize[] = {3 * width_};
    sws_scale(sws_.get(), frm_->data, frm_->linesize, 0, height_, bgr_data, bgr_linesize);
}

sensor_msgs::msg::Image::UniquePtr StreamDecoder::nextFrame() noexcept
{
    if (frames_.empty())
        return sensor_msgs::msg::Image::UniquePtr();
    sensor_msgs::msg::Image::UniquePtr img = std::move(frames_.front());
    frames_.pop_front();
    return img;
}

VideoCodec StreamDecoder::codec() const noexcept
{
    return codec_;
}

AVCodecContext* StreamDecoder::context() noexcept
{
    return ctx_.get();
}

}  // namespace rtsp_image_transport
