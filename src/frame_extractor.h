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
#ifndef RTSP_IMAGE_TRANSPORT_FRAME_EXTRACTOR_H_
#define RTSP_IMAGE_TRANSPORT_FRAME_EXTRACTOR_H_

#include "video_codec.h"

#include <liveMedia.hh>

#include <cstddef>
#include <memory>
#include <vector>

namespace rtsp_image_transport
{

class StreamClient;
class StreamingError;

class FrameExtractor : public MediaSink
{
public:
    static FrameExtractor* createNew(const std::weak_ptr<StreamClient>& stream_client, UsageEnvironment& env,
                                     MediaSubsession* subsession);
    VideoCodec codec() const;

protected:
    Boolean continuePlaying();

private:
    FrameExtractor(const std::weak_ptr<StreamClient>& stream_client, UsageEnvironment& env,
                   MediaSubsession* subsession);

    static void newFrameCallback(void* self, unsigned frameSize, unsigned numTruncatedBytes,
                                 struct timeval presentationTime, unsigned durationInMicroseconds);
    void deliverFrame(unsigned frameSize, unsigned numTruncatedBytes, struct timeval presentationTime,
                      unsigned durationInMicroseconds);

    /* Copies the out-of-band parameter sets to the front of an empty buffer */
    void seedParameterSets();

    std::weak_ptr<StreamClient> stream_client_;
    MediaSubsession* subsession_;
    VideoCodec codec_;
    /* Grows on demand: a 4K key frame easily exceeds the initial size, and a
       truncated NAL unit is worthless to the decoder. */
    std::vector<unsigned char> buffer_;
    std::size_t buffer_length_;
    /* The VPS/SPS/PPS NAL units the SDP announced, already in Annex B form.
       Kept so they can be prepended again after the buffer is discarded: a
       camera that only sends them out of band would otherwise never get them to
       the decoder at all. */
    std::vector<unsigned char> parameter_sets_;
    bool warned_at_limit_;
};

}  // namespace rtsp_image_transport

#endif