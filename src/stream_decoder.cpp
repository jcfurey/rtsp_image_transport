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

#include "log_level.h"
#include "streaming_error.h"

#include <sensor_msgs/image_encodings.hpp>

#include <algorithm>
#include <cstdint>
#include <format>
#include <map>
#include <mutex>
#include <thread>
#include <vector>

extern "C"
{
#include <libavformat/avformat.h>
#include <libavutil/buffer.h>
#include <libavutil/hwcontext.h>
#include <libavutil/opt.h>
#include <libavutil/pixdesc.h>
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

void free_buffer(AVBufferRef* buffer)
{
    av_buffer_unref(&buffer);
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

/* Standalone hardware decoders. They open the GPU themselves and hand back
   frames in system memory, so they need no hardware device context from us.
   Names that this FFmpeg build does not know are skipped silently. */
const std::map<VideoCodec, std::vector<std::string>> STANDALONE_HW_DECODERS{
    {VideoCodec::H264, {"h264_cuvid", "h264_qsv", "h264_rkmpp", "h264_v4l2m2m"}},
    {VideoCodec::H265, {"hevc_cuvid", "hevc_qsv", "hevc_rkmpp", "hevc_v4l2m2m"}},
    {VideoCodec::MPEG4, {"mpeg4_cuvid", "mpeg4_rkmpp", "mpeg4_v4l2m2m"}},
    {VideoCodec::MPEG2, {"mpeg2_cuvid", "mpeg2_qsv", "mpeg2_v4l2m2m"}},
    {VideoCodec::H263, {"h263_v4l2m2m"}},
    {VideoCodec::VP8, {"vp8_cuvid", "vp8_qsv", "vp8_rkmpp", "vp8_v4l2m2m"}},
    {VideoCodec::VP9, {"vp9_cuvid", "vp9_qsv", "vp9_rkmpp", "vp9_v4l2m2m"}},
    {VideoCodec::AV1, {"av1_cuvid", "av1_qsv", "av1_rkmpp", "av1_vaapi"}},
    {VideoCodec::MJPEG, {"mjpeg_cuvid", "mjpeg_qsv"}}};

/* Native decoders. Each is tried first with every hardware device it advertises
   support for (NVDEC, VAAPI, ...) and finally as the software fallback. */
const std::map<VideoCodec, std::vector<std::string>> NATIVE_DECODERS{
    {VideoCodec::H264, {"h264"}},   {VideoCodec::H265, {"hevc"}},
    {VideoCodec::MPEG4, {"mpeg4"}}, {VideoCodec::MPEG2, {"mpeg2video"}},
    {VideoCodec::H263, {"h263"}},   {VideoCodec::VP8, {"vp8"}},
    {VideoCodec::VP9, {"vp9"}},     {VideoCodec::AV1, {"libdav1d", "av1"}},
    {VideoCodec::MJPEG, {"mjpeg"}}};

/* Order in which hardware devices are tried when hw_device is "auto" */
constexpr AVHWDeviceType HW_DEVICE_PREFERENCE[] = {
    AV_HWDEVICE_TYPE_CUDA,  AV_HWDEVICE_TYPE_VAAPI,         AV_HWDEVICE_TYPE_QSV,     AV_HWDEVICE_TYPE_VDPAU,
    AV_HWDEVICE_TYPE_DRM,   AV_HWDEVICE_TYPE_VIDEOTOOLBOX,  AV_HWDEVICE_TYPE_D3D11VA, AV_HWDEVICE_TYPE_VULKAN};

std::string hwDeviceName(AVHWDeviceType type)
{
    const char* name = av_hwdevice_get_type_name(type);
    return name ? name : "unknown";
}

/* Hardware device contexts are shared process wide: opening one per stream
   would create a separate GPU context per camera. The map also caches negative
   results so that an unavailable device is probed only once. Keyed by device
   path as well, so that machines with more than one GPU can address each. */
std::shared_ptr<AVBufferRef> sharedHwDevice(AVHWDeviceType type, const std::string& device_path,
                                            const rclcpp::Logger& logger)
{
    /* Recursive because creating a QSV device asks for a VAAPI device below */
    static std::recursive_mutex mutex;
    static std::map<std::pair<AVHWDeviceType, std::string>, AVBufferRef*> cache;

    std::lock_guard<std::recursive_mutex> lock{mutex};
    const std::pair<AVHWDeviceType, std::string> key{type, device_path};
    auto entry = cache.find(key);
    if (entry == cache.end())
    {
        AVBufferRef* device = nullptr;
        int result = AVERROR(ENOSYS);
        const char* path = device_path.empty() ? nullptr : device_path.c_str();
        {
            /* A missing GPU is an expected outcome here, not something to
               report to the user as an FFmpeg error. */
            TemporaryAvLogLevel quiet(AV_LOG_PANIC);
            /* On Linux, Quick Sync runs on top of VAAPI. Deriving the QSV
               device from a VAAPI one reuses the context we may already have
               open instead of opening the render node a second time, and keeps
               both on the same adapter when a machine has several. */
            if (type == AV_HWDEVICE_TYPE_QSV)
            {
                std::shared_ptr<AVBufferRef> vaapi = sharedHwDevice(AV_HWDEVICE_TYPE_VAAPI, device_path, logger);
                if (vaapi)
                    result = av_hwdevice_ctx_create_derived(&device, type, vaapi.get(), 0);
            }
            if (!device)
                result = av_hwdevice_ctx_create(&device, type, path, nullptr, 0);
        }
        if (result < 0 || !device)
        {
            char errbuf[80];
            RCLCPP_DEBUG(logger, "%s hardware device%s%s is unavailable: %s", hwDeviceName(type).c_str(),
                         path ? " " : "", path ? path : "",
                         av_make_error_string(errbuf, sizeof(errbuf), result));
            device = nullptr;
        }
        else
        {
            RCLCPP_INFO(logger, "opened %s device%s%s for hardware accelerated video decoding",
                        hwDeviceName(type).c_str(), path ? " " : "", path ? path : "");
        }
        entry = cache.emplace(key, device).first;
    }
    if (!entry->second)
        return std::shared_ptr<AVBufferRef>();
    return std::shared_ptr<AVBufferRef>(av_buffer_ref(entry->second), free_buffer);
}

/* Reports whether the decoder can be driven by the given hardware device, and
   which pixel format its frames will then have. */
bool decoderSupportsHwDevice(const AVCodec* decoder, AVHWDeviceType type, AVPixelFormat& hw_pixel_format)
{
    for (int i = 0;; ++i)
    {
        const AVCodecHWConfig* config = avcodec_get_hw_config(decoder, i);
        if (!config)
            return false;
        if (config->device_type == type && (config->methods & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX) != 0)
        {
            hw_pixel_format = config->pix_fmt;
            return true;
        }
    }
}

bool isHardwarePixelFormat(AVPixelFormat format)
{
    const AVPixFmtDescriptor* desc = av_pix_fmt_desc_get(format);
    return desc && (desc->flags & AV_PIX_FMT_FLAG_HWACCEL) != 0;
}

/* Standalone hardware decoders open the GPU themselves, but only when the
   vendor runtime is actually there. avcodec_open2() does not always notice that
   it is missing: h264_qsv, for instance, opens happily on a machine without
   Quick Sync and only fails on the first packet, by which time frames have
   already been lost. Checking the matching device up front avoids that. */
AVHWDeviceType standaloneDeviceRequirement(const std::string& decoder_name)
{
    if (decoder_name.ends_with("_cuvid"))
        return AV_HWDEVICE_TYPE_CUDA;
    if (decoder_name.ends_with("_qsv"))
        return AV_HWDEVICE_TYPE_QSV;
    if (decoder_name.ends_with("_vaapi"))
        return AV_HWDEVICE_TYPE_VAAPI;
    /* v4l2m2m and rkmpp have no AVHWDeviceType; they fail cleanly on open */
    return AV_HWDEVICE_TYPE_NONE;
}

}  // namespace

std::vector<std::string> StreamDecoder::availableHwDevices(const std::string& hw_device_path)
{
    rclcpp::Logger logger = rclcpp::get_logger("StreamDecoder");
    std::vector<std::string> available;
    for (AVHWDeviceType type : HW_DEVICE_PREFERENCE)
    {
        if (sharedHwDevice(type, hw_device_path, logger))
            available.push_back(hwDeviceName(type));
    }
    return available;
}

StreamDecoder::StreamDecoder(VideoCodec codec, const Options& options, const rclcpp::Logger& logger)
    : logger_(logger), codec_(codec), options_(options), initialized_(false), sws_threaded_(false), hardware_(false),
      width_(0), height_(0), last_pixel_format_(AV_PIX_FMT_NONE), hw_pixel_format_(AV_PIX_FMT_NONE),
      candidate_index_(0)
{
    candidates_ = buildCandidates();
    if (candidates_.empty())
    {
        if (!options_.decoder.empty())
            throw StreamingError(std::format("decoder \"{}\" is not available in your FFmpeg library",
                                             options_.decoder));
        throw StreamingError(std::format("no decoder support available for {}", videoCodecName(codec)));
    }

    std::string last_error;
    for (candidate_index_ = 0; candidate_index_ < candidates_.size(); ++candidate_index_)
    {
        const DecoderCandidate& candidate = candidates_[candidate_index_];
        try
        {
            openCandidate(candidate);
            RCLCPP_DEBUG(logger_, "using decoder %s", description_.c_str());
            return;
        }
        catch (const std::exception& e)
        {
            resetWorkingBuffers();
            last_error = e.what();
            RCLCPP_DEBUG(logger_, "[%s] %s", candidate.decoder_name.c_str(), e.what());
        }
    }
    throw StreamingError(std::format("no usable decoder available for {}{}", videoCodecName(codec),
                                     last_error.empty() ? "" : std::format(" ({})", last_error)));
}

/* Assembles the ordered list of ways this stream could be decoded. Hardware
   accelerated native decoders come first because they share one device context
   across all streams, then the standalone hardware decoders, then software. */
std::vector<DecoderCandidate> StreamDecoder::buildCandidates() const
{
    std::vector<DecoderCandidate> candidates;

    auto native = NATIVE_DECODERS.find(codec_);
    auto standalone = STANDALONE_HW_DECODERS.find(codec_);
    const std::vector<std::string> no_decoders;
    const std::vector<std::string>& native_names = native != NATIVE_DECODERS.end() ? native->second : no_decoders;
    const std::vector<std::string>& standalone_names =
        standalone != STANDALONE_HW_DECODERS.end() ? standalone->second : no_decoders;

    /* An explicitly requested decoder is the only one we try */
    if (!options_.decoder.empty())
    {
        const AVCodec* decoder = avcodec_find_decoder_by_name(options_.decoder.c_str());
        if (!decoder)
        {
            RCLCPP_ERROR(logger_, "decoder \"%s\" is not available in your FFmpeg library", options_.decoder.c_str());
            return candidates;
        }
        DecoderCandidate candidate;
        candidate.decoder_name = options_.decoder;
        bool standalone_hw = std::find(standalone_names.begin(), standalone_names.end(), options_.decoder)
                             != standalone_names.end();
        if (!standalone_hw)
        {
            /* An explicitly named decoder is used even if the device probe
               fails, so the resulting error is about what was asked for. */
            for (AVHWDeviceType type : hwDevicePreference())
            {
                AVPixelFormat hw_format = AV_PIX_FMT_NONE;
                if (decoderSupportsHwDevice(decoder, type, hw_format))
                {
                    DecoderCandidate accelerated = candidate;
                    accelerated.hw_device_type = type;
                    accelerated.hw_pixel_format = hw_format;
                    accelerated.hardware = true;
                    candidates.push_back(accelerated);
                }
            }
        }
        candidate.hardware = standalone_hw;
        candidates.push_back(candidate);
        return candidates;
    }

    const std::vector<AVHWDeviceType> devices = hwDevicePreference();
    if (!devices.empty())
    {
        for (const std::string& name : native_names)
        {
            const AVCodec* decoder = avcodec_find_decoder_by_name(name.c_str());
            if (!decoder)
                continue;
            for (AVHWDeviceType type : devices)
            {
                AVPixelFormat hw_format = AV_PIX_FMT_NONE;
                if (!decoderSupportsHwDevice(decoder, type, hw_format))
                    continue;
                DecoderCandidate candidate;
                candidate.decoder_name = name;
                candidate.hw_device_type = type;
                candidate.hw_pixel_format = hw_format;
                candidate.hardware = true;
                candidates.push_back(candidate);
            }
        }
        for (const std::string& name : standalone_names)
        {
            if (!avcodec_find_decoder_by_name(name.c_str()))
                continue;
            AVHWDeviceType required = standaloneDeviceRequirement(name);
            if (required != AV_HWDEVICE_TYPE_NONE)
            {
                if (std::find(devices.begin(), devices.end(), required) == devices.end())
                    continue;
                if (!sharedHwDevice(required, options_.hw_device_path, logger_))
                    continue;
            }
            else if (options_.hw_device != "auto")
            {
                /* No device type maps to these, so only offer them when we are
                   free to try anything. */
                continue;
            }
            DecoderCandidate candidate;
            candidate.decoder_name = name;
            candidate.hardware = true;
            candidates.push_back(candidate);
        }
    }
    for (const std::string& name : native_names)
    {
        if (!avcodec_find_decoder_by_name(name.c_str()))
            continue;
        DecoderCandidate candidate;
        candidate.decoder_name = name;
        candidates.push_back(candidate);
    }
    return candidates;
}

/* The hardware devices to consider, in order, for the configured hw_device */
std::vector<AVHWDeviceType> StreamDecoder::hwDevicePreference() const
{
    if (!options_.use_hw_decoder || options_.hw_device.empty() || options_.hw_device == "none")
        return {};
    if (options_.hw_device != "auto")
    {
        AVHWDeviceType type = av_hwdevice_find_type_by_name(options_.hw_device.c_str());
        if (type == AV_HWDEVICE_TYPE_NONE)
        {
            std::string known;
            for (AVHWDeviceType t = av_hwdevice_iterate_types(AV_HWDEVICE_TYPE_NONE); t != AV_HWDEVICE_TYPE_NONE;
                 t = av_hwdevice_iterate_types(t))
            {
                if (!known.empty())
                    known += ", ";
                known += hwDeviceName(t);
            }
            RCLCPP_ERROR(logger_, "unknown hardware device \"%s\"; this FFmpeg build supports: %s",
                         options_.hw_device.c_str(), known.empty() ? "(none)" : known.c_str());
            return {};
        }
        return {type};
    }
    return std::vector<AVHWDeviceType>(std::begin(HW_DEVICE_PREFERENCE), std::end(HW_DEVICE_PREFERENCE));
}

void StreamDecoder::openCandidate(const DecoderCandidate& candidate)
{
    const AVCodec* decoder = avcodec_find_decoder_by_name(candidate.decoder_name.c_str());
    if (!decoder)
        throw StreamingError("decoder is not available in your FFmpeg library");

    hw_device_.reset();
    hw_pixel_format_ = AV_PIX_FMT_NONE;
    hardware_ = candidate.hardware;
    ctx_.reset(avcodec_alloc_context3(decoder), free_context);
    if (!ctx_)
        throw StreamingError("failed to initialize decoder context");

    if (candidate.hw_device_type != AV_HWDEVICE_TYPE_NONE)
    {
        hw_device_ = sharedHwDevice(candidate.hw_device_type, options_.hw_device_path, logger_);
        if (!hw_device_)
            throw StreamingError(
                std::format("{} device is not available", hwDeviceName(candidate.hw_device_type)));
        hw_pixel_format_ = candidate.hw_pixel_format;
        ctx_->hw_device_ctx = av_buffer_ref(hw_device_.get());
        if (!ctx_->hw_device_ctx)
            throw StreamingError("failed to reference hardware device context");
        ctx_->opaque = this;
        ctx_->get_format = selectPixelFormat;
        description_ = std::format("{} + {}", decoder->name, hwDeviceName(candidate.hw_device_type));
    }
    else
    {
        description_ = candidate.hardware ? std::format("{} (hardware)", decoder->name)
                                          : std::format("{} (software)", decoder->name);
    }

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
    if (options_.low_latency)
    {
        /* Frame level multithreading buffers up to thread_count frames before
           the first one is handed back, which adds several hundred milliseconds
           of latency on a typical multi-core machine. Slice threading keeps the
           speedup without the pipeline delay, and AV_CODEC_FLAG_LOW_DELAY tells
           the decoder to emit every frame as soon as it is complete. */
        ctx_->thread_type = FF_THREAD_SLICE;
        ctx_->flags |= AV_CODEC_FLAG_LOW_DELAY;
    }
    int result = avcodec_open2(ctx_.get(), nullptr, 0);
    if (result < 0)
    {
        char errbuf[80];
        throw StreamingError(
            std::format("failed to open decoder: {}", av_make_error_string(errbuf, sizeof(errbuf), result)));
    }
}

/* Picks the hardware pixel format the decoder was set up for. Called by FFmpeg
   once the stream parameters are known. */
AVPixelFormat StreamDecoder::selectPixelFormat(AVCodecContext* ctx, const AVPixelFormat* formats)
{
    const StreamDecoder* self = static_cast<const StreamDecoder*>(ctx->opaque);
    if (self)
    {
        for (const AVPixelFormat* p = formats; *p != AV_PIX_FMT_NONE; ++p)
        {
            if (*p == self->hw_pixel_format_)
                return *p;
        }
    }
    /* The hardware cannot handle this stream (an unsupported profile, for
       instance). Decode in software rather than failing; picking a hardware
       format we did not prepare frame buffers for would abort the decode. */
    for (const AVPixelFormat* p = formats; *p != AV_PIX_FMT_NONE; ++p)
    {
        if (!isHardwarePixelFormat(*p))
            return *p;
    }
    return formats[0];
}

/* Moves to the next way of decoding this stream after a hardware path turned
   out not to work. Returns false when the list is exhausted. */
bool StreamDecoder::fallBackToNextCandidate(const std::string& reason)
{
    while (++candidate_index_ < candidates_.size())
    {
        const DecoderCandidate& candidate = candidates_[candidate_index_];
        try
        {
            openCandidate(candidate);
            RCLCPP_WARN(logger_, "%s; falling back to %s", reason.c_str(), description_.c_str());
            resetWorkingBuffers();
            return true;
        }
        catch (const std::exception& e)
        {
            RCLCPP_DEBUG(logger_, "[%s] %s", candidate.decoder_name.c_str(), e.what());
        }
    }
    resetWorkingBuffers();
    return false;
}

void StreamDecoder::resetWorkingBuffers()
{
    initialized_ = false;
    sws_.reset();
    frames_.clear();
    width_ = 0;
    height_ = 0;
    last_pixel_format_ = AV_PIX_FMT_NONE;
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
        sw_frm_.reset(av_frame_alloc(), free_frame);
        bgr_frm_.reset(av_frame_alloc(), free_frame);
        if (!pkt_ || !frm_ || !sw_frm_ || !bgr_frm_)
            throw DecodingError("failed to allocate decoder working buffers");
        frames_.clear();
        initialized_ = true;
    }
    pkt_->data = const_cast<unsigned char*>(data->data());
    pkt_->size = data->length();
    pkt_->pts = data->stamp().nanoseconds();
    pkt_->dts = pkt_->pts;
    char errbuf[80];
    int result;
    if ((result = avcodec_send_packet(ctx_.get(), pkt_.get())) != 0)
    {
        std::string message = std::format("failed to send bitstream packet to decoder: {}",
                                          av_make_error_string(errbuf, sizeof(errbuf), result));
        if (hardware_ && fallBackToNextCandidate(message))
            return 0;
        throw DecodingError(message);
    }
    std::size_t count = 0;
    while ((result = avcodec_receive_frame(ctx_.get(), frm_.get())) == 0)
    {
        AVFrame* source;
        try
        {
            source = downloadIfHardwareFrame(frm_.get());
        }
        catch (const DecodingError& e)
        {
            av_frame_unref(frm_.get());
            if (hardware_ && fallBackToNextCandidate(e.what()))
                return count;
            throw;
        }
        AVPixelFormat source_format = static_cast<AVPixelFormat>(source->format);
        if (!sws_ || source->width != width_ || source->height != height_ || source_format != last_pixel_format_)
        {
            width_ = source->width;
            height_ = source->height;
            last_pixel_format_ = source_format;
            sws_ = allocScaler(width_, height_, last_pixel_format_, AV_PIX_FMT_BGR24, sws_threaded_);
            if (!sws_)
            {
                const char* name = av_get_pix_fmt_name(last_pixel_format_);
                throw DecodingError(std::format("cannot convert {}x{} {} frames to BGR8", width_, height_,
                                                name ? name : "unknown"));
            }
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
        convertToBGR(*img, source);
        count++;
        frames_.push_back(std::move(img));
    }
    av_frame_unref(frm_.get());
    if (result != AVERROR(EAGAIN))
    {
        std::string message = std::format("failed to receive frames from decoder: {}",
                                          av_make_error_string(errbuf, sizeof(errbuf), result));
        if (hardware_ && fallBackToNextCandidate(message))
            return count;
        throw DecodingError(message);
    }
    return count;
}

/* Frames decoded on the GPU live in device memory; the ROS image message needs
   them in system memory. Frames that are already in system memory (software
   decoders and the standalone hardware decoders) are passed through. */
AVFrame* StreamDecoder::downloadIfHardwareFrame(AVFrame* frame)
{
    if (!frame->hw_frames_ctx && !isHardwarePixelFormat(static_cast<AVPixelFormat>(frame->format)))
        return frame;
    av_frame_unref(sw_frm_.get());
    /* Leaving the format unset lets FFmpeg pick the cheapest transfer format
       the device supports, usually NV12 or P010 for 10 bit streams. */
    int result = av_hwframe_transfer_data(sw_frm_.get(), frame, 0);
    if (result < 0)
    {
        char errbuf[80];
        throw DecodingError(std::format("failed to transfer decoded frame from device memory: {}",
                                        av_make_error_string(errbuf, sizeof(errbuf), result)));
    }
    av_frame_copy_props(sw_frm_.get(), frame);
    return sw_frm_.get();
}

void StreamDecoder::convertToBGR(sensor_msgs::msg::Image& img, AVFrame* source)
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
        int result = sws_scale_frame(sws_.get(), bgr_frm_.get(), source);
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
    sws_scale(sws_.get(), source->data, source->linesize, 0, height_, bgr_data, bgr_linesize);
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

const std::string& StreamDecoder::description() const noexcept
{
    return description_;
}

bool StreamDecoder::isHardwareAccelerated() const noexcept
{
    return hardware_;
}

}  // namespace rtsp_image_transport
