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

/* The Annex B assembly the RTSP client feeds the decoder.
 *
 * This is the piece that turns one-NAL-at-a-time delivery from Live555 into the
 * byte stream libavcodec sees, and it carries the out-of-band parameter sets a
 * camera announces only in its SDP. A MediaSubsession can be built from an SDP
 * string without a server, and a FramedSource can be scripted, so the whole
 * path runs here with no network.
 */

#include "frame_extractor.h"

#include "event_loop.h"
#include "frame_data.h"
#include "stream_client.h"
#include "test_helpers.h"
#include "video_codec.h"

#include <gtest/gtest.h>

#include <Base64.hh>
#include <liveMedia.hh>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

using namespace rtsp_image_transport;
using namespace rtsp_image_transport::test;
using namespace std::chrono_literals;

namespace
{

/* Hands out a scripted list of NAL units the way MultiFramedRTPSource does:
   one per getNextFrame(), reporting how much did not fit rather than splitting
   it. */
class ScriptedSource : public FramedSource
{
public:
    static ScriptedSource* createNew(UsageEnvironment& env, const std::vector<std::vector<std::uint8_t>>& nals,
                                     std::atomic<std::size_t>* consumed)
    {
        return new ScriptedSource(env, nals, consumed);
    }

private:
    ScriptedSource(UsageEnvironment& env, const std::vector<std::vector<std::uint8_t>>& nals,
                   std::atomic<std::size_t>* consumed)
        : FramedSource(env), nals_(nals), consumed_(consumed)
    {
    }

    void doGetNextFrame() override
    {
        /* Deferred: Live555 forbids calling afterGetting() from inside
           doGetNextFrame(), and a scripted source would otherwise recurse
           through the whole list on one stack. */
        envir().taskScheduler().scheduleDelayedTask(0, deliver, this);
    }

    static void deliver(void* self)
    {
        static_cast<ScriptedSource*>(self)->deliverNow();
    }

    void deliverNow()
    {
        if (index_ >= nals_.size())
        {
            handleClosure();
            return;
        }
        const std::vector<std::uint8_t>& nal = nals_[index_++];
        fFrameSize = static_cast<unsigned>(nal.size());
        fNumTruncatedBytes = 0;
        if (fFrameSize > fMaxSize)
        {
            fNumTruncatedBytes = fFrameSize - fMaxSize;
            fFrameSize = fMaxSize;
        }
        std::memcpy(fTo, nal.data(), fFrameSize);
        /* Give every scripted unit a deterministic access-unit timestamp so
           tests exercise the timestamp-change fallback without depending on
           scheduler timing or wall-clock resolution. */
        fPresentationTime.tv_sec = 1700000000;
        fPresentationTime.tv_usec = static_cast<suseconds_t>(1000 * index_);
        fDurationInMicroseconds = 0;
        /* Counted before handing over, so a test can wait for the whole script
           to have been consumed rather than for a delivery count it would have
           to predict. */
        if (consumed_)
            consumed_->fetch_add(1);
        FramedSource::afterGetting(this);
    }

    std::vector<std::vector<std::uint8_t>> nals_;
    std::atomic<std::size_t>* consumed_ = nullptr;
    std::size_t index_ = 0;
};

/* Annex B start code offsets, four byte form included. */
std::vector<std::size_t> nalOffsets(const std::vector<std::uint8_t>& p)
{
    std::vector<std::size_t> offsets;
    for (std::size_t i = 0; i + 2 < p.size();)
    {
        if (p[i] == 0 && p[i + 1] == 0 && p[i + 2] == 1)
        {
            offsets.push_back(i >= 1 && p[i - 1] == 0 ? i - 1 : i);
            i += 3;
        }
        else
        {
            ++i;
        }
    }
    return offsets;
}

/* The VPS, SPS and PPS of a real H.265 stream, so the SDP under test carries
   parameter sets libavcodec would actually accept. */
struct ParameterSets
{
    std::vector<std::uint8_t> vps, sps, pps;
    bool complete() const
    {
        return !vps.empty() && !sps.empty() && !pps.empty();
    }
};

ParameterSets extractH265ParameterSets()
{
    ParameterSets sets;
    const auto packets = encodeTestStream(VideoCodec::H265, 320, 240, 2);
    if (packets.empty())
        return sets;
    for (const auto& packet : packets)
    {
        const auto offsets = nalOffsets(packet);
        for (std::size_t k = 0; k < offsets.size(); ++k)
        {
            const std::size_t begin = offsets[k];
            const std::size_t end = k + 1 < offsets.size() ? offsets[k + 1] : packet.size();
            /* Skip the start code to get at the NAL unit itself */
            std::size_t body = begin;
            while (body + 2 < packet.size() && !(packet[body] == 0 && packet[body + 1] == 0 && packet[body + 2] == 1))
                ++body;
            body += 3;
            if (body >= end)
                continue;
            const int type = (packet[body] >> 1) & 0x3f;
            std::vector<std::uint8_t> nal(packet.begin() + body, packet.begin() + end);
            if (type == 32 && sets.vps.empty())
                sets.vps = nal;
            else if (type == 33 && sets.sps.empty())
                sets.sps = nal;
            else if (type == 34 && sets.pps.empty())
                sets.pps = nal;
        }
        if (sets.complete())
            break;
    }
    return sets;
}

std::string base64Of(const std::vector<std::uint8_t>& data)
{
    char* encoded = base64Encode(reinterpret_cast<const char*>(data.data()), static_cast<unsigned>(data.size()));
    std::string result = encoded ? encoded : "";
    delete[] encoded;
    return result;
}

std::string h265Sdp(const ParameterSets& sets)
{
    return "v=0\r\n"
           "o=- 1 1 IN IP4 127.0.0.1\r\n"
           "s=frame extractor test\r\n"
           "t=0 0\r\n"
           "m=video 0 RTP/AVP 96\r\n"
           "c=IN IP4 0.0.0.0\r\n"
           "a=rtpmap:96 H265/90000\r\n"
           "a=fmtp:96 sprop-vps="
           + base64Of(sets.vps) + ";sprop-sps=" + base64Of(sets.sps) + ";sprop-pps=" + base64Of(sets.pps)
           + "\r\n"
             "a=control:track1\r\n";
}

/* One NAL unit with a recognisable payload and the given size. */
std::vector<std::uint8_t> syntheticNal(std::uint8_t nal_type, std::size_t payload_bytes, std::uint8_t fill)
{
    std::vector<std::uint8_t> nal;
    nal.push_back(static_cast<std::uint8_t>(nal_type << 1));  // H.265 NAL header, first byte
    nal.push_back(0x01);                                      // second header byte
    nal.insert(nal.end(), payload_bytes, fill);
    return nal;
}

/* Drives a FrameExtractor over a scripted list of NAL units and collects
   everything it hands to the StreamClient. */
class ExtractorHarness
{
public:
    explicit ExtractorHarness(const std::string& sdp) : loop_(EventLoop::create())
    {
        session_ = MediaSession::createNew(loop_->env(), sdp.c_str());
        if (!session_)
            return;
        MediaSubsessionIterator it(*session_);
        subsession_ = it.next();
        client_ = StreamClient::create("frame_extractor_test", "rtsp://127.0.0.1/test");
        client_->setReceiveStreamDataHandler(
            [this](VideoCodec, MediaSubsession*, const FrameDataPtr& data)
            {
                std::lock_guard<std::mutex> lock{mutex_};
                delivered_.emplace_back(data->data(), data->data() + data->length());
            });
    }

    ~ExtractorHarness()
    {
        if (extractor_ || source_)
        {
            loop_->post(
                [this]
                {
                    if (extractor_)
                        Medium::close(extractor_);
                    if (source_)
                        Medium::close(source_);
                    extractor_ = nullptr;
                    source_ = nullptr;
                });
        }
        if (session_)
            loop_->post([this] { Medium::close(session_); });
        client_.reset();
        loop_->stop();
    }

    bool usable() const
    {
        return session_ != nullptr && subsession_ != nullptr && client_ != nullptr;
    }

    /* Runs the scripted NAL units through the extractor and returns everything
       it handed over.

       Waiting on a delivery count is not enough: the extractor holds a buffer
       back until it is worth sending, so which unit triggers which delivery is
       an implementation detail, and sampling mid-script sees an access unit
       that is still being assembled. This waits for the source to have handed
       over every unit in the script, and then for deliveries to stop arriving,
       so what comes back is the finished sequence. */
    std::vector<std::vector<std::uint8_t>> run(const std::vector<std::vector<std::uint8_t>>& nals)
    {
        const std::size_t total = nals.size();
        loop_->post(
            [this, &nals]
            {
                extractor_ = FrameExtractor::createNew(client_, loop_->env(), subsession_);
                source_ = ScriptedSource::createNew(loop_->env(), nals, &consumed_);
                extractor_->startPlaying(*source_, nullptr, nullptr);
            });

        const auto deadline = std::chrono::steady_clock::now() + 10s;
        while (consumed_.load() < total && std::chrono::steady_clock::now() < deadline)
            std::this_thread::sleep_for(5ms);

        /* Drain: the last unit consumed may still be on its way through
           deliverFrame. Settled once the count has not moved for a while. */
        std::size_t stable_for = 0, last_seen = 0;
        while (stable_for < 20 && std::chrono::steady_clock::now() < deadline)
        {
            std::size_t now = 0;
            {
                std::lock_guard<std::mutex> lock{mutex_};
                now = delivered_.size();
            }
            stable_for = (now == last_seen) ? stable_for + 1 : 0;
            last_seen = now;
            std::this_thread::sleep_for(5ms);
        }

        std::lock_guard<std::mutex> lock{mutex_};
        return delivered_;
    }

private:
    std::shared_ptr<EventLoop> loop_;
    MediaSession* session_ = nullptr;
    MediaSubsession* subsession_ = nullptr;
    std::shared_ptr<StreamClient> client_;
    FrameExtractor* extractor_ = nullptr;
    ScriptedSource* source_ = nullptr;
    mutable std::mutex mutex_;
    std::vector<std::vector<std::uint8_t>> delivered_;
    std::atomic<std::size_t> consumed_{0};
};

/* The Annex B form of the SDP parameter sets, which is what the extractor has
   to put in front of the media data. */
std::vector<std::uint8_t> annexB(const ParameterSets& sets)
{
    static const std::uint8_t start_code[] = {0, 0, 0, 1};
    std::vector<std::uint8_t> out;
    for (const std::vector<std::uint8_t>* nal : {&sets.vps, &sets.sps, &sets.pps})
    {
        out.insert(out.end(), start_code, start_code + 4);
        out.insert(out.end(), nal->begin(), nal->end());
    }
    return out;
}

bool startsWith(const std::vector<std::uint8_t>& haystack, const std::vector<std::uint8_t>& needle)
{
    return haystack.size() >= needle.size() && std::equal(needle.begin(), needle.end(), haystack.begin());
}

bool contains(const std::vector<std::uint8_t>& haystack, const std::vector<std::uint8_t>& needle)
{
    if (needle.empty() || haystack.size() < needle.size())
        return false;
    return std::search(haystack.begin(), haystack.end(), needle.begin(), needle.end()) != haystack.end();
}

class FrameExtractorTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        sets_ = extractH265ParameterSets();
        if (!sets_.complete())
            GTEST_SKIP() << "no H.265 encoder in this FFmpeg build to take parameter sets from";
    }
    ParameterSets sets_;
};

}  // namespace

/* The camera this fork exists for announces its parameter sets only in the SDP
   and never repeats them in band, so they have to reach the decoder ahead of
   the first slice. */
TEST_F(FrameExtractorTest, PrependsTheSdpParameterSetsToTheFirstFrame)
{
    ExtractorHarness harness(h265Sdp(sets_));
    ASSERT_TRUE(harness.usable()) << "the synthetic SDP did not parse into a video subsession";

    const auto delivered = harness.run({syntheticNal(1, 500, 0xAA)});
    ASSERT_FALSE(delivered.empty()) << "nothing reached the stream client";
    EXPECT_TRUE(startsWith(delivered.front(), annexB(sets_)))
        << "the first frame did not start with the SDP parameter sets";
}

/* A NAL unit larger than the assembly buffer makes the extractor grow it and
   start over, and the parameter sets were sitting in what it discarded. A 4K
   key frame is enough to trigger it on the initial 256 kB, and a camera that
   only announces them out of band then leaves the decoder unable to start for
   the rest of the session. */
TEST_F(FrameExtractorTest, PutsTheParameterSetsBackAfterTheBufferGrows)
{
    ExtractorHarness harness(h265Sdp(sets_));
    ASSERT_TRUE(harness.usable());

    /* Larger than INITIAL_FRAME_BUFFER_SIZE, so the first NAL is truncated and
       dropped; then an ordinary one that has to carry the parameter sets. */
    const auto delivered = harness.run({syntheticNal(1, 400000, 0xBB), syntheticNal(1, 500, 0xCC)});

    ASSERT_FALSE(delivered.empty()) << "nothing was delivered after the buffer grew";
    EXPECT_TRUE(contains(delivered.front(), annexB(sets_)))
        << "the parameter sets were lost when the oversized NAL unit reset the buffer";
}

/* The truncated unit itself must not reach the decoder: half a NAL unit is
   worse than none, because the decoder cannot tell it is incomplete. */
TEST_F(FrameExtractorTest, DropsATruncatedNalUnitRatherThanForwardingIt)
{
    ExtractorHarness harness(h265Sdp(sets_));
    ASSERT_TRUE(harness.usable());

    const auto delivered = harness.run({syntheticNal(1, 400000, 0xBB), syntheticNal(1, 500, 0xCC)});
    ASSERT_FALSE(delivered.empty());

    /* 0xBB is the oversized unit's payload and must appear nowhere. */
    for (const auto& frame : delivered)
    {
        EXPECT_EQ(std::count(frame.begin(), frame.end(), std::uint8_t{0xBB}), 0)
            << "part of the truncated NAL unit was forwarded";
    }
}

/* Live555 delivers one NAL unit per call, and the extractor only ships a buffer
   once there is enough in it to be worth shipping. Small units are therefore
   held and go out with whatever follows rather than being dropped — an access
   unit delimiter or an SEI on its own is well under the threshold.

   Which delivery each one lands in is not the point and is not fixed: the
   parameter sets already put the buffer over the threshold, so the first unit
   after them goes out on its own, and the next two travel together. What has to
   hold is that nothing is lost along the way. */
TEST_F(FrameExtractorTest, HoldsSmallNalUnitsRatherThanDroppingThem)
{
    ExtractorHarness harness(h265Sdp(sets_));
    ASSERT_TRUE(harness.usable());

    /* An access unit delimiter and an SEI, each far below the threshold on its
       own, followed by a slice big enough to push the buffer out. */
    const auto delivered =
        harness.run({syntheticNal(35, 4, 0xD1), syntheticNal(39, 6, 0xD2), syntheticNal(1, 400, 0xD3)});
    ASSERT_FALSE(delivered.empty());

    std::vector<std::uint8_t> everything;
    for (const auto& frame : delivered)
        everything.insert(everything.end(), frame.begin(), frame.end());

    EXPECT_GT(std::count(everything.begin(), everything.end(), std::uint8_t{0xD1}), 0)
        << "the access unit delimiter was dropped";
    EXPECT_GT(std::count(everything.begin(), everything.end(), std::uint8_t{0xD2}), 0)
        << "the SEI was dropped";
    EXPECT_GT(std::count(everything.begin(), everything.end(), std::uint8_t{0xD3}), 0)
        << "the slice was dropped";
}

/* H.264 takes its parameter sets from a single combined attribute; the codec
   specific branch has to pick the right one. */
TEST(FrameExtractorH264, PrependsCombinedSpropParameterSets)
{
    /* sprop-parameter-sets carries SPS and PPS together, comma separated. */
    const std::vector<std::uint8_t> sps{0x67, 0x42, 0x00, 0x1e, 0xab};
    const std::vector<std::uint8_t> pps{0x68, 0xce, 0x38, 0x80};
    const std::string sdp = "v=0\r\n"
                            "o=- 1 1 IN IP4 127.0.0.1\r\n"
                            "s=frame extractor test\r\n"
                            "t=0 0\r\n"
                            "m=video 0 RTP/AVP 96\r\n"
                            "c=IN IP4 0.0.0.0\r\n"
                            "a=rtpmap:96 H264/90000\r\n"
                            "a=fmtp:96 sprop-parameter-sets="
                            + base64Of(sps) + "," + base64Of(pps)
                            + "\r\n"
                              "a=control:track1\r\n";

    ExtractorHarness harness(sdp);
    ASSERT_TRUE(harness.usable());

    const auto delivered = harness.run({syntheticNal(1, 500, 0xEE)});
    ASSERT_FALSE(delivered.empty());

    static const std::uint8_t start_code[] = {0, 0, 0, 1};
    std::vector<std::uint8_t> expected;
    for (const std::vector<std::uint8_t>* nal : {&sps, &pps})
    {
        expected.insert(expected.end(), start_code, start_code + 4);
        expected.insert(expected.end(), nal->begin(), nal->end());
    }
    EXPECT_TRUE(startsWith(delivered.front(), expected))
        << "H.264 parameter sets were not taken from sprop-parameter-sets";
}

TEST(FrameExtractorMpeg4, ClosureDoesNotRemoveBytesFromAnUnframedPicture)
{
    /* MPEG-4, VP8/9, AV1 and JPEG are handed over without Annex B start
       codes. The source-closing fallback used to remove four bytes anyway,
       truncating the last picture of every markerless stream. */
    const std::string sdp = "v=0\r\n"
                            "o=- 1 1 IN IP4 127.0.0.1\r\n"
                            "s=frame extractor test\r\n"
                            "t=0 0\r\n"
                            "m=video 0 RTP/AVP 96\r\n"
                            "c=IN IP4 0.0.0.0\r\n"
                            "a=rtpmap:96 MP4V-ES/90000\r\n"
                            "a=control:track1\r\n";
    ExtractorHarness harness(sdp);
    ASSERT_TRUE(harness.usable());

    const std::vector<std::uint8_t> picture{0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0};
    const auto delivered = harness.run({picture});
    ASSERT_EQ(delivered.size(), 1u);
    EXPECT_EQ(delivered.front(), picture);
}

TEST(FrameExtractorMpeg4, TimestampFallbackPreservesAdjacentUnframedPictures)
{
    const std::string sdp = "v=0\r\n"
                            "o=- 1 1 IN IP4 127.0.0.1\r\n"
                            "s=frame extractor test\r\n"
                            "t=0 0\r\n"
                            "m=video 0 RTP/AVP 96\r\n"
                            "c=IN IP4 0.0.0.0\r\n"
                            "a=rtpmap:96 MP4V-ES/90000\r\n"
                            "a=control:track1\r\n";
    ExtractorHarness harness(sdp);
    ASSERT_TRUE(harness.usable());

    const std::vector<std::uint8_t> first{0x10, 0x11, 0x12, 0x13, 0x14, 0x15};
    const std::vector<std::uint8_t> second{0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26};
    const auto delivered = harness.run({first, second});
    ASSERT_EQ(delivered.size(), 2u);
    EXPECT_EQ(delivered[0], first);
    EXPECT_EQ(delivered[1], second);
}
