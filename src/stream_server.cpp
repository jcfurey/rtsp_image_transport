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
#include "stream_server.h"

#include "init.h"
#include "streaming_error.h"

#include <arpa/inet.h>
#include <rclcpp/logging.hpp>

#include <format>
#include <random>

namespace rtsp_image_transport
{

namespace
{

constexpr unsigned ESTIMATED_BITRATE = 500;

/* Key frames of high resolution streams are considerably larger than the
   Live555 default, and anything that does not fit is silently truncated. */
constexpr unsigned MAX_OUT_PACKET_BUFFER_SIZE = 524288;

uint16_t sockToPort(const in_addr& addr)
{
    return 16384 + (ntohl(addr.s_addr) & 0x7ffe);
}

/* Start of the port range Live555 probes for the RTP/RTCP pair of a unicast
   subsession. Randomized so that independently started publishers do not all
   collide on the same port first. */
portNumBits randomEphemeralPortBase()
{
    thread_local std::mt19937 rng{std::random_device{}()};
    return static_cast<portNumBits>(49152 + 2 * (rng() % 8192));
}

bool isSupported(VideoCodec codec)
{
    switch (codec)
    {
        case VideoCodec::H264:
#ifdef LIVE555_HAS_H265_SUPPORT
        case VideoCodec::H265:
#endif
        case VideoCodec::MPEG4:
#ifdef LIVE555_HAS_VPX_SUPPORT
        case VideoCodec::VP8:
        case VideoCodec::VP9:
#endif
#ifdef LIVE555_HAS_AV1_SUPPORT
        case VideoCodec::AV1:
#endif
            return true;
        default:
            return false;
    }
}

VideoRTPSink* createVideoRTPSink(VideoCodec codec, UsageEnvironment& env, Groupsock* rtpGroupsock,
                                 unsigned char rtpPayloadTypeIfDynamic)
{
    switch (codec)
    {
        case VideoCodec::H264:
            return H264VideoRTPSink::createNew(env, rtpGroupsock, rtpPayloadTypeIfDynamic);
#ifdef LIVE555_HAS_H265_SUPPORT
        case VideoCodec::H265:
            return H265VideoRTPSink::createNew(env, rtpGroupsock, rtpPayloadTypeIfDynamic);
#endif
        case VideoCodec::MPEG4:
            return MPEG4ESVideoRTPSink::createNew(env, rtpGroupsock, rtpPayloadTypeIfDynamic);
#ifdef LIVE555_HAS_VPX_SUPPORT
        case VideoCodec::VP8:
            return VP8VideoRTPSink::createNew(env, rtpGroupsock, rtpPayloadTypeIfDynamic);
        case VideoCodec::VP9:
            return VP9VideoRTPSink::createNew(env, rtpGroupsock, rtpPayloadTypeIfDynamic);
#endif
#ifdef LIVE555_HAS_AV1_SUPPORT
        case VideoCodec::AV1:
            return AV1VideoRTPSink::createNew(env, rtpGroupsock, rtpPayloadTypeIfDynamic);
#endif
        default:
            return nullptr;
    }
}

/* Live555 decides the RTP marker bit from nalUnitEndsAccessUnit(), whose base
   implementation assumes every VCL NAL unit ends a picture. Its own comment
   says that is wrong for more than one slice per picture, and invites exactly
   this subclass. Our streams do have several slices per picture whenever
   slice-max-size applies, so without this every slice is marked as a complete
   frame — a receiver following RFC 6184 is then told the picture ends 25 times
   per picture, and has no way to find the real boundaries.

   The injector knows the answer without guessing, because the publisher hands
   it each complete access unit atomically. */
template<class Framer>
class AccessUnitFramer : public Framer
{
public:
    static AccessUnitFramer* createNew(UsageEnvironment& env, FramedSource* source, FrameInjector* injector)
    {
        return new AccessUnitFramer(env, source, injector);
    }

protected:
    AccessUnitFramer(UsageEnvironment& env, FramedSource* source, FrameInjector* injector)
        : Framer(env, source, False, False), injector_(injector)
    {
    }

    Boolean nalUnitEndsAccessUnit(u_int8_t nal_unit_type) override
    {
        if (!injector_)
            return Framer::nalUnitEndsAccessUnit(nal_unit_type);
        /* The producer supplied the boundary explicitly. Do not filter it
           through the base class's VCL-only guess: suffix SEI and other
           non-VCL units are allowed to be the last NAL unit of an access unit. */
        return injector_->lastDeliveryEndedAccessUnit();
    }

private:
    FrameInjector* injector_;
};

FramedSource* createDiscreteFramer(VideoCodec codec, UsageEnvironment& env, FramedSource* source,
                                   FrameInjector* injector)
{
    switch (codec)
    {
        case VideoCodec::H264:
            return AccessUnitFramer<H264VideoStreamDiscreteFramer>::createNew(env, source, injector);
#ifdef LIVE555_HAS_H265_SUPPORT
        case VideoCodec::H265:
            return AccessUnitFramer<H265VideoStreamDiscreteFramer>::createNew(env, source, injector);
#endif
        case VideoCodec::MPEG4:
            return MPEG4VideoStreamDiscreteFramer::createNew(env, source);
        case VideoCodec::VP8:
            return source;
        case VideoCodec::VP9:
            return source;
        case VideoCodec::AV1:
            return source;
        default:
            return nullptr;
    }
}

void afterPlaying(void*) {}

}  // namespace

class MulticastServerMediaSubsession : public PassiveServerMediaSubsession
{
public:
    static MulticastServerMediaSubsession* createNew(RTPSink& sink, RTCPInstance* rtcp);

protected:
    MulticastServerMediaSubsession(RTPSink& sink, RTCPInstance* rtcp);
    const char* sdpLines(int addressFamily) override;
};

MulticastServerMediaSubsession* MulticastServerMediaSubsession::createNew(RTPSink& sink, RTCPInstance* rtcp)
{
    return new MulticastServerMediaSubsession(sink, rtcp);
}

MulticastServerMediaSubsession::MulticastServerMediaSubsession(RTPSink& sink, RTCPInstance* rtcp)
    : PassiveServerMediaSubsession(sink, rtcp)
{
}

const char* MulticastServerMediaSubsession::sdpLines(int addressFamily)
{
    delete[] fSDPLines;
    fSDPLines = nullptr;
    return PassiveServerMediaSubsession::sdpLines(addressFamily);
}

class UnicastServerMediaSubsession : public OnDemandServerMediaSubsession
{
public:
    static UnicastServerMediaSubsession* createNew(UsageEnvironment& env, std::weak_ptr<StreamServer> server,
                                                   portNumBits initialPortNum, Boolean multiplexRTCPWithRTP);

protected:
    UnicastServerMediaSubsession(UsageEnvironment& env, std::weak_ptr<StreamServer> server, portNumBits initialPortNum,
                                 Boolean multiplexRTCPWithRTP);
    const char* sdpLines(int addressFamily) override;
    FramedSource* createNewStreamSource(unsigned clientSessionId, unsigned& estBitrate) override;
    RTPSink* createNewRTPSink(Groupsock* rtpGroupsock, unsigned char rtpPayloadTypeIfDynamic,
                              FramedSource* inputSource) override;
    void closeStreamSource(FramedSource* inputSource) override;

private:
    std::weak_ptr<StreamServer> server_;
    bool dummy_session_;
};

UnicastServerMediaSubsession* UnicastServerMediaSubsession::createNew(UsageEnvironment& env,
                                                                      std::weak_ptr<StreamServer> server,
                                                                      portNumBits initialPortNum,
                                                                      Boolean multiplexRTCPWithRTP)
{
    return new UnicastServerMediaSubsession(env, server, initialPortNum, multiplexRTCPWithRTP);
}

UnicastServerMediaSubsession::UnicastServerMediaSubsession(UsageEnvironment& env, std::weak_ptr<StreamServer> server,
                                                           portNumBits initialPortNum, Boolean multiplexRTCPWithRTP)
    : OnDemandServerMediaSubsession(env, True, initialPortNum, multiplexRTCPWithRTP), server_(server),
      dummy_session_(false)
{
}

const char* UnicastServerMediaSubsession::sdpLines(int addressFamily)
{
    std::shared_ptr<StreamServer> s = server_.lock();
    if (s && s->sink_)
    {
        setSDPLinesFromRTPSink(s->sink_, nullptr, ESTIMATED_BITRATE);
    }
    dummy_session_ = true;
    const char* lines = OnDemandServerMediaSubsession::sdpLines(addressFamily);
    dummy_session_ = false;
    return lines;
}

RTPSink* UnicastServerMediaSubsession::createNewRTPSink(Groupsock* rtpGroupsock, unsigned char rtpPayloadTypeIfDynamic,
                                                        FramedSource* inputSource)
{
    VideoRTPSink* sink = nullptr;
    std::shared_ptr<StreamServer> s = server_.lock();
    if (s)
    {
        sink = createVideoRTPSink(s->codec(), envir(), rtpGroupsock, rtpPayloadTypeIfDynamic);
        if (!dummy_session_)
            s->sink_ = sink;
        if (sink)
        {
            sink->setPacketSizes(s->preferredPacketSize(), s->maxPacketSize());
        }
    }
    return sink;
}

FramedSource* UnicastServerMediaSubsession::createNewStreamSource(unsigned clientSessionId, unsigned& estBitrate)
{
    FramedSource* source = nullptr;
    std::shared_ptr<StreamServer> s = server_.lock();
    if (s)
    {
        estBitrate = ESTIMATED_BITRATE;
        FrameInjector* injector = FrameInjector::createNew(envir());
        source = createDiscreteFramer(s->codec(), envir(), injector, injector);
        if (source)
        {
            if (!dummy_session_)
                s->newStreamSource(source, injector);
        }
        else
        {
            injector->shutdown();
            RCLCPP_ERROR(s->logger_, "[%s] unable to create RTP sink for %s", s->topic_name_.c_str(),
                         videoCodecName(s->codec()).c_str());
        }
    }
    return source;
}

void UnicastServerMediaSubsession::closeStreamSource(FramedSource* inputSource)
{
    std::shared_ptr<StreamServer> s = server_.lock();
    if (s)
    {
        s->closeStreamSource(inputSource);
    }
    OnDemandServerMediaSubsession::closeStreamSource(inputSource);
}

std::shared_ptr<StreamServer> StreamServer::create(const std::string& topic_name, unsigned udp_port,
                                                   unsigned udp_packet_size, const rclcpp::Logger& logger)
{
    return std::shared_ptr<StreamServer>(new StreamServer(topic_name, udp_port, udp_packet_size, logger));
}

StreamServer::StreamServer(const std::string& topic_name, unsigned udp_port, unsigned udp_packet_size,
                           const rclcpp::Logger& logger)
    : logger_(logger), codec_(VideoCodec::Unknown), topic_name_(topic_name), udp_packet_size_(udp_packet_size),
      loop_(EventLoop::create()), rtsp_(nullptr), sms_(nullptr), sink_(nullptr), mcast_sink_(nullptr),
      mcast_rtcp_(nullptr), mcast_source_(nullptr)
{
    rtsp_ = RTSPServer::createNew(loop_->env(), udp_port);
    if (!rtsp_)
        throw StreamingError(udp_port == 0 ? "cannot create RTSP server"
                                           : std::format("cannot create RTSP server on port {}", udp_port));
}

/* Creating and tearing down RTP sinks, RTCP instances and media sessions all
   reach into objects the Live555 loop dispatches into, and in multicast mode
   the loop is transmitting from the moment the sink starts playing. Doing that
   from a ROS executor thread frees a groupsock out from under an in-flight
   sendto(). Both entry points therefore hand the work to the loop thread and
   wait for it; post() runs inline when the caller is already the loop thread,
   so start() calling stop() nests safely. */
void StreamServer::start(VideoCodec codec, bool use_multicast)
{
    loop_->post([this, codec, use_multicast] { startOnLoop(codec, use_multicast); });
}

void StreamServer::stop()
{
    loop_->post([this] { stopOnLoop(); });
}

void StreamServer::startOnLoop(VideoCodec codec, bool use_multicast)
{
    RTCPInstance* rtcp = nullptr;
    OutPacketBuffer::increaseMaxSizeTo(MAX_OUT_PACKET_BUFFER_SIZE);
    stop();
    if (!isSupported(codec))
        throw StreamingError(std::format("{} is not supported on your system", videoCodecName(codec)));
    codec_ = codec;
    if (use_multicast)
    {
        if (!rtp_mcast_ || !rtcp_mcast_)
        {
            struct sockaddr_storage sockStorage;
            struct sockaddr_in* sockAddr = reinterpret_cast<struct sockaddr_in*>(&sockStorage);
            sockStorage.ss_family = AF_INET;
            sockAddr->sin_addr.s_addr = chooseRandomIPv4SSMAddress(loop_->env());
            const uint16_t mcast_port = sockToPort(sockAddr->sin_addr);
            rtp_mcast_ = std::make_shared<Groupsock>(std::ref(loop_->env()), sockStorage, mcast_port, 255);
            rtp_mcast_->multicastSendOnly();
            rtcp_mcast_ = std::make_shared<Groupsock>(std::ref(loop_->env()), sockStorage, mcast_port + 1, 255);
            rtcp_mcast_->multicastSendOnly();
        }
        sink_ = createVideoRTPSink(codec_, loop_->env(), rtp_mcast_.get(), 96);
        if (!sink_)
            throw StreamingError(std::format("cannot instantiate VideoRTPSink for {}", videoCodecName(codec_)));
        sink_->setPacketSizes(preferredPacketSize(), maxPacketSize());
        char hostname[HOST_NAME_MAX + 1];
        if (gethostname(hostname, sizeof(hostname)) == 0)
        {
            hostname[HOST_NAME_MAX] = 0;
            rtcp = RTCPInstance::createNew(loop_->env(), rtcp_mcast_.get(), ESTIMATED_BITRATE,
                                           reinterpret_cast<unsigned char*>(hostname), sink_, NULL, True);
        }
        else
            throw StreamingError("missing or invalid hostname on this system");
        sms_ = ServerMediaSession::createNew(loop_->env(), "", "rtsp_image_transport", topic_name_.c_str(),
                                             /*multicast*/ True);
        sms_->addSubsession(MulticastServerMediaSubsession::createNew(*sink_, rtcp));
        rtsp_->addServerMediaSession(sms_);
        FrameInjector* injector = FrameInjector::createNew(loop_->env());
        FramedSource* source = createDiscreteFramer(codec_, loop_->env(), injector, injector);
        if (!source)
            throw StreamingError(std::format("cannot instantiate FramedSource for {}", videoCodecName(codec_)));
        /* Unlike the unicast case, Live555 does not take ownership of these
           objects, so stop() has to close them explicitly. */
        mcast_sink_ = sink_;
        mcast_rtcp_ = rtcp;
        mcast_source_ = source;
        newStreamSource(source, injector);
        sink_->startPlaying(*source, afterPlaying, this);
    }
    else
    {
        sms_ = ServerMediaSession::createNew(loop_->env(), "", "rtsp_image_transport", topic_name_.c_str(),
                                             /*multicast*/ False);
        sms_->addSubsession(UnicastServerMediaSubsession::createNew(loop_->env(), shared_from_this(),
                                                                    randomEphemeralPortBase(), False));
        rtsp_->addServerMediaSession(sms_);
    }
    std::shared_ptr<char> tmp(rtsp_->rtspURL(sms_, ros_interface_socket()), [](char* p) { delete[] p; });
    url_ = std::string(tmp.get());
    RCLCPP_INFO(logger_, "[%s] new RTSP session at %s", topic_name_.c_str(), url_.c_str());
}

void StreamServer::stopOnLoop()
{
    std::unique_lock<std::mutex> lock{streams_mutex_};
    if (!url_.empty())
        RCLCPP_DEBUG(logger_, "[%s] finished RTSP session at %s", topic_name_.c_str(), url_.c_str());
    ServerMediaSession* old_sms = sms_;
    VideoRTPSink* old_mcast_sink = mcast_sink_;
    RTCPInstance* old_mcast_rtcp = mcast_rtcp_;
    FramedSource* old_mcast_source = mcast_source_;
    sms_ = nullptr;
    sink_ = nullptr;
    mcast_sink_ = nullptr;
    mcast_rtcp_ = nullptr;
    mcast_source_ = nullptr;
    for (auto& stream : streams_)
    {
        if (stream.second)
            stream.second->shutdown();
    }
    streams_.clear();
    url_.clear();
    lock.unlock();

    if (old_sms && rtsp_)
    {
        rtsp_->deleteServerMediaSession(old_sms);
    }
    /* The multicast RTP objects are ours to close, and the order is not free.
       The source stays ours throughout: for H.264 and H.265 Live555 wraps it in
       an H264or5Fragmenter, whose destructor calls detachInputSource() precisely
       so that ~FramedFilter does not close what it was handed. So the sink never
       takes the source with it — but it does reach into it while being
       destroyed, because ~H264or5VideoRTPSink puts the fragmenter back in
       fSource and calls stopPlaying() one last time.

       Hence: sink first, then the source, which closes the framer and through it
       the FrameInjector. Closing the source first is a use-after-free inside the
       sink's destructor; not closing it at all leaks the whole chain. Both were
       measured under AddressSanitizer, which is what would catch a future
       Live555 changing its mind about this. */
    if (old_mcast_sink)
        old_mcast_sink->stopPlaying();
    if (old_mcast_rtcp)
        Medium::close(old_mcast_rtcp);
    if (old_mcast_sink)
        Medium::close(old_mcast_sink);
    if (old_mcast_source)
        Medium::close(old_mcast_source);
}

StreamServer::~StreamServer()
{
    /* Same reason as start()/stop(): the RTSP server object belongs to the loop
       thread. post() runs inline once the loop has stopped, and inline when
       this runs on the loop thread itself, so both teardown routes work. */
    loop_->post(
        [this]
        {
            stopOnLoop();
            if (rtsp_)
            {
                Medium::close(rtsp_);
                rtsp_ = nullptr;
            }
        });
    /* May run on the Live555 thread itself when a callback held the last
       reference; stop() knows not to wait for itself in that case. */
    loop_->stop();
}

bool StreamServer::hasActiveStreams() const noexcept
{
    std::lock_guard<std::mutex> lock{streams_mutex_};
    return !streams_.empty();
}

void StreamServer::sendAccessUnit(const std::vector<FrameDataPtr>& frames) noexcept
{
    if (frames.empty())
        return;
    std::lock_guard<std::mutex> lock{streams_mutex_};
    for (auto& stream : streams_)
    {
        stream.second->injectAccessUnit(frames);
    }
}

VideoCodec StreamServer::codec() const noexcept
{
    return codec_;
}

unsigned StreamServer::preferredPacketSize() const noexcept
{
    return udp_packet_size_ < 982 ? udp_packet_size_ : 982;
}

unsigned StreamServer::maxPacketSize() const noexcept
{
    return udp_packet_size_;
}

std::string StreamServer::url() const noexcept
{
    return url_;
}

void StreamServer::newStreamSource(FramedSource* source, FrameInjector* injector) noexcept
{
    std::lock_guard<std::mutex> lock{streams_mutex_};
    if (source && injector)
    {
        if (streams_.insert(std::make_pair(source, injector)).second)
            RCLCPP_DEBUG(logger_, "[%s] created media source %p", topic_name_.c_str(), source);
    }
}

void StreamServer::closeStreamSource(FramedSource* source) noexcept
{
    std::lock_guard<std::mutex> lock{streams_mutex_};
    StreamMapping::iterator it = streams_.find(source);
    if (it != streams_.end())
    {
        it->second->shutdown();
        RCLCPP_DEBUG(logger_, "[%s] destroyed media source %p", topic_name_.c_str(), source);
        streams_.erase(it);
    }
    if (streams_.empty())
        sink_ = nullptr;
}

}  // namespace rtsp_image_transport
