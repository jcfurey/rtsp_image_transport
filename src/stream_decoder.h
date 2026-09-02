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
#ifndef RTSP_IMAGE_TRANSPORT_STREAM_DECODER_H_
#define RTSP_IMAGE_TRANSPORT_STREAM_DECODER_H_

#include "frame_data.h"
#include "video_codec.h"

extern "C"
{
#include <libavutil/buffer.h>
#include <libavutil/hwcontext.h>
#include <libswscale/swscale.h>
}

#include <rclcpp/logger.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cstddef>
#include <deque>
#include <memory>
#include <string>
#include <vector>

namespace rtsp_image_transport
{

class StreamingError;
class DecodingError;

/* One way of decoding the stream: an FFmpeg decoder, optionally combined with a
   hardware device that provides the acceleration. */
struct DecoderCandidate
{
    std::string decoder_name;
    /* AV_HWDEVICE_TYPE_NONE means the decoder needs no device context of its
       own, which covers both software decoders and the standalone hardware
       decoders such as h264_cuvid or hevc_qsv. */
    AVHWDeviceType hw_device_type = AV_HWDEVICE_TYPE_NONE;
    AVPixelFormat hw_pixel_format = AV_PIX_FMT_NONE;
    bool hardware = false;
};

class StreamDecoder
{
public:
    enum class DecodeFrames
    {
        All,
        Intra,
        Key,
        None
    };
    struct Options
    {
        bool use_hw_decoder = true;
        bool low_latency = true;
        /* "auto" picks the first working device, "none" forces software
           decoding, anything else names an FFmpeg hardware device type such as
           "cuda", "vaapi", "qsv", "vdpau" or "vulkan". */
        std::string hw_device = "auto";
        /* Which adapter to use when a machine has more than one: a DRM render
           node such as "/dev/dri/renderD128" for VAAPI and QSV, or a device
           index for CUDA. Empty lets FFmpeg choose. */
        std::string hw_device_path;
        /* Forces a specific FFmpeg decoder (e.g. "hevc_cuvid") when not empty */
        std::string decoder;
        /* Drop frames the decoder could not fully reconstruct instead of
           publishing the concealed result. Off by default: concealment repairs
           the damage from the reference frame, which keeps the video smooth,
           whereas dropping trades artefacts for stutter. Turn it on for
           consumers that would rather see nothing than see wrong pixels. */
        bool drop_corrupt_frames = false;
        /* Worker threads for the YUV to BGR conversion. The conversion is
           bandwidth-bound and scales well past this on a workstation -- 4K
           measures about 2.8x faster at 20 threads than at 4 -- but this driver
           shares a CPU with everything else on the vehicle, and a camera per
           stream taking that many threads is not a good neighbour. The default
           stays where it has always been; raise it per deployment when the core
           budget actually allows. 0 uses hardware_concurrency() unclamped. */
        int sws_threads = 4;
    };

    StreamDecoder(VideoCodec codec, const Options& options,
                  const rclcpp::Logger& logger = rclcpp::get_logger("StreamDecoder"));
    VideoCodec codec() const noexcept;
    std::size_t decodeVideo(const FrameDataPtr& data);
    sensor_msgs::msg::Image::UniquePtr nextFrame() noexcept;
    AVCodecContext* context() noexcept;
    /* Human readable description of the decoder in use, e.g. "hevc + CUDA" */
    const std::string& description() const noexcept;
    bool isHardwareAccelerated() const noexcept;
    void setDecodeFrames(DecodeFrames which) noexcept;
    /* Throws away the decoder's reference frames and starts again at the next
       key frame.
     *
     * Packet loss can leave an H.264 decoder rejecting every slice that
     * follows — libavcodec reports "decode_slice_header error" and no picture
     * comes out — and it does not always find its way back on its own, because
     * a slice it will not decode is also a slice whose parameter sets it never
     * reaches. RTP keeps arriving throughout, so the session watchdog sees a
     * perfectly healthy stream. This is the way out of that: cheaper than a
     * reconnect, and unlike building a new decoder it keeps the extradata
     * parsed from the SDP, which is all a camera that never repeats its
     * parameter sets in band will ever send. */
    void flush() noexcept;

    /* Hardware device types this build of FFmpeg can instantiate right now.
       Probed once and cached; intended for diagnostics. */
    static std::vector<std::string> availableHwDevices(const std::string& hw_device_path = std::string());
    /* Every FFmpeg decoder this codec would consider, native and standalone
       hardware alike. Same reason as the encoder side: a name filed under the
       wrong codec opens and then decodes garbage. */
    static std::vector<std::string> candidateDecoderNames(VideoCodec codec);

private:
    static AVPixelFormat selectPixelFormat(AVCodecContext* ctx, const AVPixelFormat* formats);
    static int prepareFrameBuffer(AVCodecContext* ctx, AVFrame* frame, int flags);

    std::vector<DecoderCandidate> buildCandidates() const;
    std::vector<AVHWDeviceType> hwDevicePreference() const;
    void openCandidate(const DecoderCandidate& candidate);
    bool fallBackToNextCandidate(const std::string& reason);
    AVFrame* downloadIfHardwareFrame(AVFrame* frame);
    void convertToBGR(sensor_msgs::msg::Image& img, AVFrame* source);
    bool discardFrame(const AVFrame* frame) noexcept;
    /* Fraction of the picture libavcodec never wrote, or 0 when this decoder
       does not pre-fill its frames. Sparsely sampled; see the implementation. */
    double unwrittenFraction(const AVFrame* frame) const noexcept;
    void resetWorkingBuffers();

    rclcpp::Logger logger_;
    VideoCodec codec_;
    Options options_;
    bool initialized_, sws_threaded_, hardware_;
    /* Set when the decoder pre-fills freshly allocated frames, which is how a
       picture that lost a slice is told apart from one that decoded cleanly for
       the codecs libavcodec has no error concealment for. */
    bool fills_frames_;
    /* Set until a key frame has been decoded, so the partial pictures produced
       by joining a live stream mid-GOP are not published. */
    bool awaiting_keyframe_;
    std::size_t frames_before_keyframe_, corrupt_frames_;
    int width_, height_;
    AVPixelFormat last_pixel_format_, hw_pixel_format_;
    std::string description_;
    std::vector<DecoderCandidate> candidates_;
    std::size_t candidate_index_;
    std::shared_ptr<AVBufferRef> hw_device_;
    std::shared_ptr<AVCodecContext> ctx_;
    std::shared_ptr<AVPacket> pkt_;
    std::shared_ptr<AVFrame> frm_, sw_frm_, bgr_frm_;
    std::shared_ptr<SwsContext> sws_;
    std::deque<sensor_msgs::msg::Image::UniquePtr> frames_;
    std::vector<FrameDataPtr> hardware_probe_packets_;
    bool replaying_probe_packets_ = false;
};

}  // namespace rtsp_image_transport

#endif
