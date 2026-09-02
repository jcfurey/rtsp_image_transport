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
#ifndef RTSP_IMAGE_TRANSPORT_STREAM_ENCODER_H_
#define RTSP_IMAGE_TRANSPORT_STREAM_ENCODER_H_

#include "frame_data.h"
#include "video_codec.h"

#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cstdint>
#include <deque>
#include <memory>

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
}

namespace rtsp_image_transport
{

class StreamingError;
class EncodingError;

class StreamEncoder
{
public:
    StreamEncoder(VideoCodec codec, bool use_hw_encoder = true,
                  const rclcpp::Logger& logger = rclcpp::get_logger("StreamEncoder"));
    /* The FFmpeg encoders that will be tried for this codec, best first.
       Exposed so a test can check that each one really does encode the codec it
       is listed under: a name in the wrong list opens fine and produces a
       bitstream the receiver cannot make sense of. */
    static std::vector<std::string> candidateEncoderNames(VideoCodec codec);
    void setBitrate(unsigned long bit_rate);
    void setFramerate(unsigned fps);
    void setPackageSizeHint(unsigned size);
    /* Frames between key frames. Sets how long a receiver stays damaged after
       losing a packet: everything referencing the lost picture is wrong until
       the next key frame arrives, so this is the recovery time of a lossy
       stream, not merely a bandwidth setting. Call after setFramerate(), which
       otherwise derives the interval from the frame rate. 0 restores that
       default of one key frame per second. */
    void setKeyframeInterval(unsigned frames);
    /* Replaces periodic key frames with a band of intra macroblocks that
       sweeps across the picture, so recovery from packet loss is continuous
       instead of arriving all at once.
     *
     * The point for a lossy link is that there is no longer one large, wholly
       indispensable picture to lose: a key frame spans many packets, is
       therefore likely to be damaged, and a damaged one poisons everything
       referencing it. It also removes the bitrate spike a key frame causes.
     *
     * The cost is that a client joining mid-stream has no key frame to start
       from and must wait out a full sweep. Returns false when this encoder
       does not support it, leaving periodic key frames in place. */
    bool setIntraRefresh(bool enable);
    bool hwAccel() const noexcept;
    VideoCodec codec() const noexcept;
    std::size_t encodeVideo(const sensor_msgs::msg::Image& image);
    FrameDataPtr nextPacket() noexcept;
    AVCodecContext* context() noexcept;
    /* Presentation time stamp of the most recent frame, in the encoder time
       base. Exposed so tests can check that frame pacing survives a ROS clock
       that jumps. */
    std::int64_t lastPresentationTimestamp() const noexcept;

private:
    void setupEncoder(const AVCodec* encoder, bool silent);
    void openEncoder(int width, int height);

    rclcpp::Logger logger_;
    VideoCodec codec_;
    bool initialized_, is_vaapi_;
    std::shared_ptr<AVCodecContext> ctx_;
#ifdef FFMPEG_HAS_HWFRAME_SUPPORT
    std::shared_ptr<AVBufferRef> hw_device_, hw_frames_;
    AVHWDeviceContext* hw_device_ctx_;
    AVHWFramesContext* hw_frames_ctx_;
    std::shared_ptr<AVFrame> hw_frm_;
#endif
    std::shared_ptr<AVFrame> sw_frm_;
    std::shared_ptr<AVPacket> pkt_;
    AVPixelFormat last_pixel_format_;
    std::shared_ptr<SwsContext> sws_;
    rclcpp::Time last_ts_;
    std::int64_t last_pts_;
    std::deque<FrameDataPtr> packets_;
};

}  // namespace rtsp_image_transport

#endif
