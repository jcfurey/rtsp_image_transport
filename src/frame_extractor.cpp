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
#include "frame_extractor.h"

#include <cstring>

#include "frame_data.h"
#include "stream_client.h"
#include "streaming_error.h"

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <format>
#include <memory>

namespace rtsp_image_transport
{

namespace
{

const unsigned char MPEG_START_CODE[] = {0x00, 0x00, 0x00, 0x01};

/* Enough for 1080p key frames at sane bitrates; grown on demand for larger
   frames up to the maximum. */
constexpr std::size_t INITIAL_FRAME_BUFFER_SIZE = 262144;
constexpr std::size_t MAXIMUM_FRAME_BUFFER_SIZE = 16777216;

}

FrameExtractor* FrameExtractor::createNew(const std::weak_ptr<StreamClient>& stream_client, UsageEnvironment& env,
                                          MediaSubsession* subsession)
{
    return new FrameExtractor(stream_client, env, subsession);
}

FrameExtractor::FrameExtractor(const std::weak_ptr<StreamClient>& stream_client, UsageEnvironment& env,
                               MediaSubsession* subsession)
    : MediaSink(env), stream_client_(stream_client), subsession_(subsession),
      codec_(fromRTSPCodecName(subsession_->codecName())), buffer_(INITIAL_FRAME_BUFFER_SIZE), buffer_length_(0),
      warned_at_limit_(false)
{
    if (codec_ == VideoCodec::Unknown)
        throw StreamingError(std::format("unsupported video codec {}", subsession->codecName()));
    if (codec_ == VideoCodec::H264 || codec_ == VideoCodec::H265)
    {
        /* Pass the out-of-band parameter-set NAL units before media data.
         * H.264 carries SPS/PPS together in sprop-parameter-sets, while
         * RFC 7798 gives H.265 distinct sprop-vps/sps/pps attributes. */
        auto append_parameter_sets = [this](char const* sprops)
        {
            if (!sprops || !*sprops)
                return;
            unsigned num_nals = 0;
            std::unique_ptr<SPropRecord[]> nals(parseSPropParameterSets(sprops, num_nals));
            for (unsigned i = 0; i < num_nals; ++i)
            {
                parameter_sets_.insert(parameter_sets_.end(), MPEG_START_CODE,
                                       MPEG_START_CODE + sizeof(MPEG_START_CODE));
                parameter_sets_.insert(parameter_sets_.end(), nals[i].sPropBytes,
                                       nals[i].sPropBytes + nals[i].sPropLength);
            }
        };

        if (codec_ == VideoCodec::H264)
        {
            append_parameter_sets(subsession->fmtp_spropparametersets());
        }
        else
        {
            append_parameter_sets(subsession->fmtp_spropvps());
            append_parameter_sets(subsession->fmtp_spropsps());
            append_parameter_sets(subsession->fmtp_sproppps());
        }
        if (parameter_sets_.size() > buffer_.size())
            throw StreamingError("SDP parameter sets do not fit the frame buffer");
        seedParameterSets();
    }
}

/* The parameter sets have to reach the decoder ahead of the first slice, and
   they are only announced once, in the SDP. Whenever the buffer is emptied
   without having been delivered — which a NAL unit too large for it does — they
   have to go back in, or a camera that never repeats them in band leaves the
   decoder unable to start. Re-sending them is harmless: a decoder that already
   has them ignores a repeat. */
void FrameExtractor::seedParameterSets()
{
    if (parameter_sets_.empty() || buffer_length_ != 0 || parameter_sets_.size() > buffer_.size())
        return;
    std::copy_n(parameter_sets_.data(), parameter_sets_.size(), buffer_.data());
    buffer_length_ = parameter_sets_.size();
}

VideoCodec FrameExtractor::codec() const
{
    return codec_;
}

Boolean FrameExtractor::continuePlaying()
{
    if (codec_ == VideoCodec::H264 || codec_ == VideoCodec::H265)
    {
        if (buffer_.size() - sizeof(MPEG_START_CODE) >= buffer_length_)
        {
            std::copy_n(MPEG_START_CODE, sizeof(MPEG_START_CODE), buffer_.data() + buffer_length_);
            buffer_length_ += sizeof(MPEG_START_CODE);
        }
    }
    fSource->getNextFrame(buffer_.data() + buffer_length_, buffer_.size() - buffer_length_, newFrameCallback, this,
                          sourceClosed, this);
    return True;
}

/* The last access unit of a stream has nothing after it to mark its end, so
   without this it would sit in the buffer while the session shut down. */
void FrameExtractor::flushPending()
{
    std::shared_ptr<StreamClient> sc = stream_client_.lock();
    if (!sc || buffer_length_ == 0)
        return;
    /* continuePlaying() leaves a trailing start code waiting for a NAL unit
       that will now never arrive; it is not part of the picture. */
    const std::size_t length =
        buffer_length_ >= sizeof(MPEG_START_CODE) ? buffer_length_ - sizeof(MPEG_START_CODE) : buffer_length_;
    if (length == 0)
        return;
    const rclcpp::Time ts(buffer_time_.tv_sec, 1000ull * buffer_time_.tv_usec);
    sc->receiveStreamData(codec_, subsession_, std::make_shared<FrameData>(buffer_.data(), length, ts));
    buffer_length_ = 0;
    have_buffer_time_ = false;
}

void FrameExtractor::sourceClosed(void* obj)
{
    static_cast<FrameExtractor*>(obj)->flushPending();
    MediaSink::onSourceClosure(obj);
}

void FrameExtractor::deliverFrame(unsigned frameSize, unsigned numTruncatedBytes, struct timeval presentationTime,
                                  unsigned durationInMicroseconds)
{
    std::shared_ptr<StreamClient> sc = stream_client_.lock();
    if (sc)
    {
        if (numTruncatedBytes > 0)
        {
            /* The NAL unit did not fit. Enlarge the buffer so that the following
               frames do fit, and drop the truncated data instead of feeding a
               mutilated NAL unit to the decoder. */
            std::size_t required = buffer_length_ + frameSize + numTruncatedBytes;
            if (buffer_.size() < MAXIMUM_FRAME_BUFFER_SIZE)
            {
                std::size_t new_size = std::min(MAXIMUM_FRAME_BUFFER_SIZE, std::max(2 * buffer_.size(), required));
                RCLCPP_INFO(sc->logger_, "[%s] frame buffer too small by %u bytes, growing it to %zu bytes",
                            sc->topicName().c_str(), numTruncatedBytes, new_size);
                buffer_.resize(new_size);
            }
            else if (!warned_at_limit_)
            {
                warned_at_limit_ = true;
                RCLCPP_WARN(sc->logger_, "[%s] dropping video frames larger than the maximum buffer size of %zu bytes",
                            sc->topicName().c_str(), MAXIMUM_FRAME_BUFFER_SIZE);
            }
            buffer_length_ = 0;
            seedParameterSets();
            continuePlaying();
            return;
        }
        /* Hand the decoder whole access units rather than individual NAL units.
           Fed one NAL unit at a time it has to infer where each picture begins,
           from first_mb_in_slice, and a lost slice takes that boundary with it:
           pictures then merge instead of degrading, and a multi-slice stream
           losing packets decodes a small fraction of its frames.

           Two things close an access unit. The RTP marker bit says so directly
           (RFC 6184), which costs nothing in latency. A change of presentation
           time says so too, but only once the next picture has started, so it
           arrives a frame late; it is the fallback for a sender whose marker
           bit is wrong — as ours was until it stopped letting live555 guess. */
        const std::size_t prefix_length =
            buffer_length_ >= sizeof(MPEG_START_CODE) ? buffer_length_ - sizeof(MPEG_START_CODE) : buffer_length_;
        const bool time_changed = have_buffer_time_
                                  && (presentationTime.tv_sec != buffer_time_.tv_sec
                                      || presentationTime.tv_usec != buffer_time_.tv_usec);
        if (time_changed && prefix_length > 0)
        {
            /* Everything sharing the old stamp has arrived; send it, then start
               the new picture with the NAL unit that just came in. */
            const rclcpp::Time ts(buffer_time_.tv_sec, 1000ull * buffer_time_.tv_usec);
            sc->receiveStreamData(codec_, subsession_,
                                  std::make_shared<FrameData>(buffer_.data(), prefix_length, ts));
            const std::size_t carried = sizeof(MPEG_START_CODE) + frameSize;
            std::memmove(buffer_.data(), buffer_.data() + prefix_length, carried);
            buffer_length_ = carried;
        }
        else
        {
            buffer_length_ += frameSize;
        }
        buffer_time_ = presentationTime;
        have_buffer_time_ = true;

        RTPSource* rtp = subsession_ ? subsession_->rtpSource() : nullptr;
        const bool marker = rtp && rtp->curPacketMarkerBit();
        /* A buffer with no room for another NAL unit has to go now whatever the
           boundaries say, or the next getNextFrame() has nowhere to write. */
        const bool buffer_full = buffer_length_ + sizeof(MPEG_START_CODE) >= buffer_.size();
        if ((marker || buffer_full) && buffer_length_ > 0)
        {
            const rclcpp::Time ts(buffer_time_.tv_sec, 1000ull * buffer_time_.tv_usec);
            sc->receiveStreamData(codec_, subsession_,
                                  std::make_shared<FrameData>(buffer_.data(), buffer_length_, ts));
            buffer_length_ = 0;
            have_buffer_time_ = false;
        }
        continuePlaying();
    }
}

void FrameExtractor::newFrameCallback(void* obj, unsigned frameSize, unsigned numTruncatedBytes,
                                      struct timeval presentationTime, unsigned durationInMicroseconds)
{
    FrameExtractor* self = static_cast<FrameExtractor*>(obj);
    self->deliverFrame(frameSize, numTruncatedBytes, presentationTime, durationInMicroseconds);
}

}  // namespace rtsp_image_transport
