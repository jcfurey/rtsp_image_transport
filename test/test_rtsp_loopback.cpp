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
#include "stream_client.h"
#include "stream_decoder.h"
#include "stream_encoder.h"
#include "stream_server.h"
#include "streaming_error.h"
#include "test_helpers.h"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

using namespace rtsp_image_transport;
using namespace rtsp_image_transport::test;
using namespace std::chrono_literals;

namespace
{

/* Serves a synthetic video stream over a real RTSP session on the loopback
   interface, exactly as the publisher plugin does. */
class LoopbackServer
{
public:
    LoopbackServer(VideoCodec codec, unsigned width, unsigned height, bool use_multicast = false)
        : codec_(codec), width_(width), height_(height), stop_(false)
    {
        /* Port 0 lets the OS pick, so parallel test runs cannot collide */
        server_ = StreamServer::create("test_topic", 0, 1396 - 42);
        server_->start(codec_, use_multicast);
        encoder_ = std::make_unique<StreamEncoder>(codec_, /*use_hw_encoder=*/false);
        encoder_->setBitrate(2000000);
        encoder_->setFramerate(30);
        encoder_->setPackageSizeHint(server_->maxPacketSize() - 24);
        thread_ = std::thread([this] { run(); });
    }

    ~LoopbackServer()
    {
        stop_ = true;
        if (thread_.joinable())
            thread_.join();
        encoder_.reset();
        server_.reset();
    }

    std::string url() const
    {
        return server_->url();
    }

    unsigned framesPushed() const
    {
        return frames_pushed_.load();
    }

private:
    void run()
    {
        unsigned seq = 0;
        while (!stop_.load())
        {
            /* The server only encodes while a client is attached, which is what
               the publisher plugin does too. */
            if (server_->hasActiveStreams())
            {
                sensor_msgs::msg::Image img = makeTestImage(width_, height_, seq);
                try
                {
                    if (encoder_->encodeVideo(img) > 0)
                    {
                        while (FrameDataPtr packet = encoder_->nextPacket())
                            server_->sendFrame(packet);
                        frames_pushed_++;
                    }
                }
                catch (const std::exception&)
                {
                    return;
                }
                seq++;
            }
            std::this_thread::sleep_for(33ms);
        }
    }

    VideoCodec codec_;
    unsigned width_, height_;
    std::atomic_bool stop_;
    std::atomic_uint frames_pushed_{0};
    std::shared_ptr<StreamServer> server_;
    std::unique_ptr<StreamEncoder> encoder_;
    std::thread thread_;
};

/* Collects what a StreamClient reports, so a test can wait on it. */
class ClientObserver
{
public:
    void attach(const std::shared_ptr<StreamClient>& client)
    {
        client->setSubsessionStartedHandler(
            [this](VideoCodec codec, MediaSubsession*)
            {
                std::lock_guard<std::mutex> lock{mutex_};
                codec_ = codec;
                cv_.notify_all();
            });
        client->setSessionStartedHandler(
            [this]
            {
                std::lock_guard<std::mutex> lock{mutex_};
                started_ = true;
                cv_.notify_all();
            });
        client->setSessionFailedHandler(
            [this](int code, const std::string& message)
            {
                std::lock_guard<std::mutex> lock{mutex_};
                failed_ = true;
                failure_code_ = code;
                failure_message_ = message;
                cv_.notify_all();
            });
        client->setReceiveStreamDataHandler(
            [this](VideoCodec, MediaSubsession*, const FrameDataPtr& data)
            {
                std::lock_guard<std::mutex> lock{mutex_};
                nal_count_++;
                total_bytes_ += data->length();
                last_stamp_ = data->stamp();
                if (nal_count_ <= 400)
                    nals_.push_back(data);
                cv_.notify_all();
            });
    }

    template<typename Predicate>
    bool waitFor(Predicate predicate, std::chrono::milliseconds timeout = 20s)
    {
        std::unique_lock<std::mutex> lock{mutex_};
        return cv_.wait_for(lock, timeout, [&] { return predicate(); });
    }

    bool waitForNals(std::size_t count, std::chrono::milliseconds timeout = 20s)
    {
        return waitFor([&] { return nal_count_ >= count || failed_; }, timeout) && !failed_;
    }

    std::mutex mutex_;
    std::condition_variable cv_;
    bool started_ = false;
    bool failed_ = false;
    int failure_code_ = 0;
    std::string failure_message_;
    VideoCodec codec_ = VideoCodec::Unknown;
    std::size_t nal_count_ = 0;
    std::size_t total_bytes_ = 0;
    rclcpp::Time last_stamp_{0};
    std::vector<FrameDataPtr> nals_;
};

bool haveEncoderFor(VideoCodec codec)
{
    try
    {
        StreamEncoder encoder(codec, false);
        return true;
    }
    catch (const StreamingError&)
    {
        return false;
    }
}

}  // namespace

TEST(RtspLoopback, ServerPublishesAUsableUrl)
{
    if (!haveEncoderFor(VideoCodec::H264))
        GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
    LoopbackServer server(VideoCodec::H264, 320, 240);
    const std::string url = server.url();
    /* Each publisher runs its own RTSP server with a single unnamed session, so
       the URL is just the host and the port the OS handed out. */
    EXPECT_EQ(url.rfind("rtsp://", 0), 0u) << "url is " << url;
    const std::size_t host_start = std::strlen("rtsp://");
    const std::size_t colon = url.find(':', host_start);
    ASSERT_NE(colon, std::string::npos) << "url has no port: " << url;
    EXPECT_GT(colon, host_start) << "url has no host: " << url;
    const int port = std::atoi(url.c_str() + colon + 1);
    EXPECT_GT(port, 0) << "url is " << url;
    EXPECT_LE(port, 65535) << "url is " << url;
}

TEST(RtspLoopback, ClientReceivesVideoFromServer)
{
    if (!haveEncoderFor(VideoCodec::H264))
        GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
    LoopbackServer server(VideoCodec::H264, 320, 240);

    ClientObserver observer;
    std::shared_ptr<StreamClient> client = StreamClient::create("test_topic", server.url());
    observer.attach(client);
    ASSERT_NO_THROW(client->connect());

    ASSERT_TRUE(observer.waitFor([&] { return observer.started_ || observer.failed_; }))
        << "session never started";
    ASSERT_FALSE(observer.failed_) << "session failed: " << observer.failure_message_ << " ("
                                   << observer.failure_code_ << ")";
    EXPECT_EQ(observer.codec_, VideoCodec::H264);
    EXPECT_EQ(client->codec(), VideoCodec::H264);

    ASSERT_TRUE(observer.waitForNals(30)) << "no video data arrived over RTP";
    EXPECT_GT(observer.total_bytes_, 0u);
    client->disconnect();
}

TEST(RtspLoopback, ReceivedStreamDecodesBackToImages)
{
    /* The whole chain end to end: encode, packetise over RTP, receive,
       reassemble and decode. */
    if (!haveEncoderFor(VideoCodec::H264))
        GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
    const unsigned width = 320, height = 240;
    LoopbackServer server(VideoCodec::H264, width, height);

    ClientObserver observer;
    std::shared_ptr<StreamClient> client = StreamClient::create("test_topic", server.url());
    observer.attach(client);
    ASSERT_NO_THROW(client->connect());
    ASSERT_TRUE(observer.waitForNals(60)) << "not enough video data arrived";

    std::vector<FrameDataPtr> nals;
    {
        std::lock_guard<std::mutex> lock{observer.mutex_};
        nals = observer.nals_;
    }
    client->disconnect();

    StreamDecoder::Options options;
    options.use_hw_decoder = false;
    options.hw_device = "none";
    StreamDecoder decoder(VideoCodec::H264, options);
    std::size_t images = 0;
    for (const FrameDataPtr& nal : nals)
    {
        try
        {
            decoder.decodeVideo(nal);
        }
        catch (const DecodingError&)
        {
            continue;  // a partial first access unit is expected
        }
        while (sensor_msgs::msg::Image::UniquePtr img = decoder.nextFrame())
        {
            EXPECT_EQ(img->width, width);
            EXPECT_EQ(img->height, height);
            EXPECT_EQ(img->data.size(), static_cast<std::size_t>(3) * width * height);
            images++;
        }
    }
    EXPECT_GE(images, 3u) << "received stream did not decode into images";
}

TEST(RtspLoopback, H265SessionDecodesBackToImages)
{
    /* The deployed camera0 stream is HEVC, and the H.265 receive path —
       including the FrameExtractor's RFC 7798 sprop-vps/sps/pps SDP init —
       previously had zero session-level coverage: every session test was
       H264-only and the H265 decoder tests bypassed live555 entirely. */
    if (!haveEncoderFor(VideoCodec::H265))
        GTEST_SKIP() << "no H.265 encoder in this FFmpeg build";
    const unsigned width = 320, height = 240;
    std::unique_ptr<LoopbackServer> server;
    try
    {
        server = std::make_unique<LoopbackServer>(VideoCodec::H265, width, height);
    }
    catch (const std::exception& e)
    {
        GTEST_SKIP() << "live555 build lacks H.265 server support: " << e.what();
    }

    ClientObserver observer;
    std::shared_ptr<StreamClient> client = StreamClient::create("test_topic", server->url());
    observer.attach(client);
    /* Replace the observer's subsession handler with one that ALSO captures
       the SDP sprop attributes, so the log shows whether this run exercised
       the sprop init path or the in-band fallback. */
    std::string sprop_vps, sprop_sps, sprop_pps;
    client->setSubsessionStartedHandler(
        [&](VideoCodec codec, MediaSubsession* subsession)
        {
            std::lock_guard<std::mutex> lock{observer.mutex_};
            observer.codec_ = codec;
            if (subsession)
            {
                sprop_vps = subsession->attrVal_str("sprop-vps");
                sprop_sps = subsession->attrVal_str("sprop-sps");
                sprop_pps = subsession->attrVal_str("sprop-pps");
            }
            observer.cv_.notify_all();
        });
    ASSERT_NO_THROW(client->connect());
    ASSERT_TRUE(observer.waitForNals(60)) << "not enough video data arrived";
    EXPECT_EQ(observer.codec_, VideoCodec::H265);
    EXPECT_EQ(client->codec(), VideoCodec::H265);
    if (sprop_vps.empty() || sprop_sps.empty() || sprop_pps.empty())
        std::cerr << "[   NOTE   ] SDP carried no sprop parameter sets; this "
                     "run covered the in-band fallback, not the sprop init\n";

    std::vector<FrameDataPtr> nals;
    {
        std::lock_guard<std::mutex> lock{observer.mutex_};
        nals = observer.nals_;
    }
    client->disconnect();

    StreamDecoder::Options options;
    options.use_hw_decoder = false;
    options.hw_device = "none";
    StreamDecoder decoder(VideoCodec::H265, options);
    std::size_t images = 0;
    for (const FrameDataPtr& nal : nals)
    {
        try
        {
            decoder.decodeVideo(nal);
        }
        catch (const DecodingError&)
        {
            continue;  // a partial first access unit is expected
        }
        while (sensor_msgs::msg::Image::UniquePtr img = decoder.nextFrame())
        {
            EXPECT_EQ(img->width, width);
            EXPECT_EQ(img->height, height);
            images++;
        }
    }
    EXPECT_GE(images, 3u) << "received H.265 stream did not decode into images";
}

TEST(RtspLoopback, RtpRunsOverUdpByDefault)
{
    /* Latency is the thing being optimised, and TCP cannot drop. A lost
       segment stalls everything queued behind it until it is retransmitted,
       so on a lossy link an interleaved viewer falls progressively further
       behind live rather than losing a slice and carrying on. UDP is
       therefore what a fresh client asks for, with a receive buffer big
       enough that the kernel is not the thing dropping datagrams. */
    std::shared_ptr<StreamClient> client = StreamClient::create("test_topic", "rtsp://localhost:8554/test");
    EXPECT_FALSE(client->rtpOverTcp());
    EXPECT_EQ(client->rtpBufferSize(), DEFAULT_RTP_BUFFER_SIZE);
}

TEST(RtspLoopback, TcpInterleavedSessionDeliversVideo)
{
    /* Interleaving stays supported, and has to keep working, for lossy links
       where artefacts cost more than delay and for servers that refuse UDP. */
    if (!haveEncoderFor(VideoCodec::H264))
        GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
    LoopbackServer server(VideoCodec::H264, 320, 240);

    ClientObserver observer;
    std::shared_ptr<StreamClient> client = StreamClient::create("test_topic", server.url());
    client->setRtpOverTcp(true);
    observer.attach(client);
    ASSERT_NO_THROW(client->connect());

    ASSERT_TRUE(observer.waitFor([&] { return observer.started_ || observer.failed_; }))
        << "session never started";
    ASSERT_FALSE(observer.failed_) << "session failed: " << observer.failure_message_ << " ("
                                   << observer.failure_code_ << ")";
    ASSERT_TRUE(observer.waitForNals(30)) << "no video data arrived over interleaved TCP";
    EXPECT_GT(observer.total_bytes_, 0u);
    client->disconnect();
}

TEST(RtspLoopback, UdpSessionStillDeliversVideo)
{
    /* The default path: RTP on its own UDP sockets. */
    if (!haveEncoderFor(VideoCodec::H264))
        GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
    LoopbackServer server(VideoCodec::H264, 320, 240);

    ClientObserver observer;
    std::shared_ptr<StreamClient> client = StreamClient::create("test_topic", server.url());
    client->setRtpOverTcp(false);
    observer.attach(client);
    ASSERT_NO_THROW(client->connect());

    ASSERT_TRUE(observer.waitFor([&] { return observer.started_ || observer.failed_; }))
        << "session never started";
    ASSERT_FALSE(observer.failed_) << "session failed: " << observer.failure_message_ << " ("
                                   << observer.failure_code_ << ")";
    ASSERT_TRUE(observer.waitForNals(30)) << "no video data arrived over UDP";
    client->disconnect();
}

TEST(RtspLoopback, TcpInterleavedStreamDecodesBackToImages)
{
    if (!haveEncoderFor(VideoCodec::H264))
        GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
    const unsigned width = 320, height = 240;
    LoopbackServer server(VideoCodec::H264, width, height);

    ClientObserver observer;
    std::shared_ptr<StreamClient> client = StreamClient::create("test_topic", server.url());
    client->setRtpOverTcp(true);
    observer.attach(client);
    ASSERT_NO_THROW(client->connect());
    ASSERT_TRUE(observer.waitForNals(60)) << "not enough video data arrived";

    std::vector<FrameDataPtr> nals;
    {
        std::lock_guard<std::mutex> lock{observer.mutex_};
        nals = observer.nals_;
    }
    client->disconnect();

    StreamDecoder::Options options;
    options.use_hw_decoder = false;
    options.hw_device = "none";
    StreamDecoder decoder(VideoCodec::H264, options);
    std::size_t images = 0;
    for (const FrameDataPtr& nal : nals)
    {
        try
        {
            decoder.decodeVideo(nal);
        }
        catch (const DecodingError&)
        {
            continue;
        }
        while (sensor_msgs::msg::Image::UniquePtr img = decoder.nextFrame())
        {
            EXPECT_EQ(img->width, width);
            EXPECT_EQ(img->height, height);
            images++;
        }
    }
    EXPECT_GE(images, 3u) << "interleaved stream did not decode into images";
}

TEST(RtspLoopback, ClientReportsAvailableVideoSubsessions)
{
    if (!haveEncoderFor(VideoCodec::H264))
        GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
    LoopbackServer server(VideoCodec::H264, 320, 240);

    /* The server offers exactly one video subsession, so index 0 works and any
       higher index must fall back to it rather than failing the session. */
    for (std::size_t requested : {std::size_t(0), std::size_t(7)})
    {
        ClientObserver observer;
        std::shared_ptr<StreamClient> client = StreamClient::create("test_topic", server.url());
        client->setVideoSubsession(requested);
        EXPECT_EQ(client->videoSubsession(), requested);
        observer.attach(client);
        ASSERT_NO_THROW(client->connect());
        SCOPED_TRACE("video_subsession " + std::to_string(requested));
        ASSERT_TRUE(observer.waitFor([&] { return observer.started_ || observer.failed_; }));
        EXPECT_FALSE(observer.failed_) << observer.failure_message_;
        EXPECT_EQ(observer.codec_, VideoCodec::H264);
        client->disconnect();
    }
}

TEST(RtspLoopback, ReconnectAfterDisconnectWorks)
{
    if (!haveEncoderFor(VideoCodec::H264))
        GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
    LoopbackServer server(VideoCodec::H264, 320, 240);

    std::shared_ptr<StreamClient> client = StreamClient::create("test_topic", server.url());
    for (int attempt = 0; attempt < 3; ++attempt)
    {
        ClientObserver observer;
        observer.attach(client);
        SCOPED_TRACE("attempt " + std::to_string(attempt));
        ASSERT_NO_THROW(client->connect());
        ASSERT_TRUE(observer.waitFor([&] { return observer.started_ || observer.failed_; }));
        EXPECT_FALSE(observer.failed_) << observer.failure_message_;
        EXPECT_TRUE(observer.waitForNals(10));
        client->disconnect();
    }
}

TEST(RtspLoopback, ConnectingTwiceIsRejected)
{
    if (!haveEncoderFor(VideoCodec::H264))
        GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
    LoopbackServer server(VideoCodec::H264, 320, 240);
    std::shared_ptr<StreamClient> client = StreamClient::create("test_topic", server.url());
    ASSERT_NO_THROW(client->connect());
    EXPECT_THROW(client->connect(), StreamingError);
    client->disconnect();
}

TEST(RtspLoopback, UnreachableUrlFailsWithoutHanging)
{
    /* A typo in the URL used to deadlock the calling thread, because Live555
       reports this class of error by calling the handler from inside connect().
       Anything other than a prompt failure here is a regression. */
    ClientObserver observer;
    std::shared_ptr<StreamClient> client = StreamClient::create("test_topic", "rtsp://127.0.0.1:1/nothing-here");
    observer.attach(client);

    std::atomic_bool returned{false};
    std::thread caller(
        [&]
        {
            try
            {
                client->connect();
            }
            catch (const std::exception&)
            {
            }
            returned = true;
        });
    for (int i = 0; i < 300 && !returned.load(); ++i)
        std::this_thread::sleep_for(50ms);
    EXPECT_TRUE(returned.load()) << "connect() did not return; it is most likely deadlocked";
    caller.join();

    EXPECT_TRUE(observer.waitFor([&] { return observer.failed_; }, 15s)) << "no failure was reported";
    client->disconnect();
}

TEST(RtspLoopback, MalformedUrlIsRejectedPromptly)
{
    ClientObserver observer;
    std::shared_ptr<StreamClient> client = StreamClient::create("test_topic", "not-even-a-url");
    observer.attach(client);
    std::atomic_bool returned{false};
    std::thread caller(
        [&]
        {
            try
            {
                client->connect();
            }
            catch (const std::exception&)
            {
            }
            returned = true;
        });
    for (int i = 0; i < 200 && !returned.load(); ++i)
        std::this_thread::sleep_for(50ms);
    EXPECT_TRUE(returned.load()) << "connect() did not return for a malformed URL";
    caller.join();
    EXPECT_TRUE(observer.waitFor([&] { return observer.failed_; }, 10s));
    client->disconnect();
}

TEST(RtspLoopback, DestroyingAConnectedClientIsClean)
{
    /* Dropping the last reference while data is flowing can leave the Live555
       thread holding it, so the client ends up being destroyed from inside its
       own event loop. Repeated because it depends on that timing. */
    if (!haveEncoderFor(VideoCodec::H264))
        GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
    LoopbackServer server(VideoCodec::H264, 320, 240);
    for (int attempt = 0; attempt < 8; ++attempt)
    {
        ClientObserver observer;
        SCOPED_TRACE("attempt " + std::to_string(attempt));
        std::shared_ptr<StreamClient> client = StreamClient::create("test_topic", server.url());
        observer.attach(client);
        ASSERT_NO_THROW(client->connect());
        ASSERT_TRUE(observer.waitForNals(3));
        /* No explicit disconnect: the destructor has to tear the session down */
        client.reset();
    }
    SUCCEED();
}

TEST(RtspLoopback, ServerRestartsCleanlyWithADifferentCodec)
{
    if (!haveEncoderFor(VideoCodec::H264))
        GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
    std::shared_ptr<StreamServer> server = StreamServer::create("test_topic", 0, 1354);
    ASSERT_NO_THROW(server->start(VideoCodec::H264, false));
    const std::string first_url = server->url();
    EXPECT_FALSE(first_url.empty());

    /* Restarting releases the previous session's RTP objects; leaking them here
       used to grow the process on every reconfiguration. */
    for (int i = 0; i < 5; ++i)
    {
        ASSERT_NO_THROW(server->stop());
        EXPECT_TRUE(server->url().empty());
        ASSERT_NO_THROW(server->start(VideoCodec::H264, false));
        EXPECT_FALSE(server->url().empty());
    }
    EXPECT_FALSE(server->hasActiveStreams());
}

/* Multicast is a separate code path in the server: its own RTP and RTCP
   group sockets, its own sink, and a session that transmits whether or not a
   client is attached. Nothing exercised it, which is where the leaked sink,
   RTCP instance and framer went unnoticed across restarts. */
TEST(RtspLoopback, MulticastServerAdvertisesAUsableUrl)
{
    if (!haveEncoderFor(VideoCodec::H264))
        GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
    std::shared_ptr<StreamServer> server = StreamServer::create("test_topic", 0, 1354);
    ASSERT_NO_THROW(server->start(VideoCodec::H264, /*use_multicast=*/true));
    const std::string url = server->url();
    EXPECT_EQ(url.rfind("rtsp://", 0), 0u) << url;
    /* A multicast session is always transmitting, so it counts as active with
       no client attached — which is what stops the publisher from tearing the
       encoder down between subscribers. */
    EXPECT_TRUE(server->hasActiveStreams());
    EXPECT_NO_THROW(server->stop());
    EXPECT_TRUE(server->url().empty());
}

TEST(RtspLoopback, MulticastServerRestartsWithoutLeaking)
{
    if (!haveEncoderFor(VideoCodec::H264))
        GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
    std::shared_ptr<StreamServer> server = StreamServer::create("test_topic", 0, 1354);
    for (int i = 0; i < 5; ++i)
    {
        ASSERT_NO_THROW(server->start(VideoCodec::H264, /*use_multicast=*/true)) << "round " << i;
        EXPECT_FALSE(server->url().empty());
        ASSERT_NO_THROW(server->stop()) << "round " << i;
    }
}

/* Switching between the two transport modes on one server has to release
   whichever set of objects the previous mode allocated. */
TEST(RtspLoopback, ServerSwitchesBetweenUnicastAndMulticast)
{
    if (!haveEncoderFor(VideoCodec::H264))
        GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
    std::shared_ptr<StreamServer> server = StreamServer::create("test_topic", 0, 1354);
    for (int i = 0; i < 3; ++i)
    {
        ASSERT_NO_THROW(server->start(VideoCodec::H264, /*use_multicast=*/false));
        EXPECT_FALSE(server->hasActiveStreams()) << "unicast with no client is idle";
        ASSERT_NO_THROW(server->stop());
        ASSERT_NO_THROW(server->start(VideoCodec::H264, /*use_multicast=*/true));
        EXPECT_TRUE(server->hasActiveStreams()) << "multicast transmits regardless";
        ASSERT_NO_THROW(server->stop());
    }
}

/* Whether closing the multicast sink also closes the source differs by codec:
   the H.264 and H.265 sinks own it through Live555's fragmenter, the rest do
   not. Getting that wrong is a double free one way and a leak the other, and
   neither shows up without running every codec the server can serve. Under
   AddressSanitizer this is the test that pins the rule down. */
TEST(RtspLoopback, MulticastTeardownIsCleanForEveryCodec)
{
    for (VideoCodec codec : {VideoCodec::H264, VideoCodec::H265, VideoCodec::MPEG4, VideoCodec::VP8, VideoCodec::VP9,
                             VideoCodec::AV1})
    {
        SCOPED_TRACE(videoCodecName(codec));
        std::shared_ptr<StreamServer> server = StreamServer::create("test_topic", 0, 1354);
        try
        {
            server->start(codec, /*use_multicast=*/true);
        }
        catch (const StreamingError&)
        {
            /* This Live555 build cannot serve the codec at all */
            continue;
        }
        EXPECT_FALSE(server->url().empty());
        ASSERT_NO_THROW(server->stop());
        /* Twice, because the second round is what frees objects the first round
           left behind rather than the ones it created. */
        ASSERT_NO_THROW(server->start(codec, /*use_multicast=*/true));
        ASSERT_NO_THROW(server->stop());
    }
}

/* A multicast server still encodes and hands packets to the sink with nobody
   listening, which is the property the publisher relies on. */
TEST(RtspLoopback, MulticastServerAcceptsFrames)
{
    if (!haveEncoderFor(VideoCodec::H264))
        GTEST_SKIP() << "no H.264 encoder in this FFmpeg build";
    LoopbackServer server(VideoCodec::H264, 320, 240, /*use_multicast=*/true);
    EXPECT_FALSE(server.url().empty());

    const auto deadline = std::chrono::steady_clock::now() + 10s;
    while (server.framesPushed() < 5 && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(50ms);
    EXPECT_GE(server.framesPushed(), 5u) << "no frames reached the multicast sink";
}
