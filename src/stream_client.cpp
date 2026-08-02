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
#include "stream_client.h"

#include "frame_extractor.h"
#include "streaming_error.h"

#include <rclcpp/logging.hpp>

#include <format>
#include <string>
#include <vector>

namespace rtsp_image_transport
{

class Live555Client : public RTSPClient
{
public:
    std::shared_ptr<StreamClient> streamClient()
    {
        return stream_client_.lock();
    }

    static Live555Client* createNew(const std::weak_ptr<StreamClient>& stream_client, UsageEnvironment& env,
                                    char const* url, int verbosity = 0, char const* appName = 0,
                                    portNumBits tunnelOverHTTPPortNum = 0, int socketNumToServer = -1)
    {
        return new Live555Client(stream_client, env, url, verbosity, appName, tunnelOverHTTPPortNum, socketNumToServer);
    }
    void setSessionTimeout(const std::chrono::milliseconds& timeout) noexcept;
    void initiateSetup() noexcept;
    void teardown() noexcept;
    /* (Re)starts the watchdog clock. Called at handshake start and after
       every completed handshake step, so each of DESCRIBE/SETUP/PLAY gets a
       full timeout period rather than the whole handshake sharing one. */
    void rearmTimeout() noexcept;

private:
    Live555Client(const std::weak_ptr<StreamClient>& stream_client, UsageEnvironment& env, char const* url,
                  int verbosity, char const* appName, portNumBits tunnelOverHTTPPortNum, int socketNumToServer) noexcept
        : RTSPClient(env, url, verbosity, appName, tunnelOverHTTPPortNum, socketNumToServer),
          stream_client_(stream_client), session_(nullptr), subsession_(nullptr), has_video_(false),
          session_active_(false), handshake_started_(false), try_pos_(0), timeout_task_(nullptr),
          received_packets_(0)
    {
    }

    static void continueAfterDESCRIBE(RTSPClient* client, int resultCode, char* resultMsg) noexcept;
    static void continueAfterSETUP(RTSPClient* client, int resultCode, char* resultMsg) noexcept;
    static void continueAfterPLAY(RTSPClient* client, int resultCode, char* resultMsg) noexcept;
    static void continueAfterTEARDOWN(RTSPClient* client, int resultCode, char* resultMsg) noexcept;
    static bool collectVideoSubsessions(Live555Client* c, const std::shared_ptr<StreamClient>& sc) noexcept;
    static void setupNextSubsession(Live555Client* c) noexcept;
    static void subsessionAfterPlaying(void* obj) noexcept;
    static void checkTimeout(void* obj) noexcept;

    std::weak_ptr<StreamClient> stream_client_;
    std::mutex session_mutex_;
    std::shared_ptr<MediaSession> session_;
    MediaSubsession* subsession_;
    bool has_video_, session_active_, handshake_started_;
    /* All video subsessions of the current session in SDP order, and the order
       in which they are tried: the configured one first, then the others as
       fallbacks. */
    std::vector<MediaSubsession*> candidates_;
    std::vector<std::size_t> try_order_;
    std::size_t try_pos_;
    std::chrono::milliseconds timeout_;
    TaskToken timeout_task_;
    std::size_t received_packets_;
};

void Live555Client::initiateSetup() noexcept
{
    std::shared_ptr<StreamClient> sc = streamClient();
    if (sc)
    {
        RCLCPP_DEBUG(sc->logger(), "[%s] sending DESCRIBE command for RTSP stream", sc->topicName().c_str());
        /* Supervise the handshake, not just the established session:
           RTSPClient has no response timeout of its own, and the data
           watchdog used to arm only after PLAY — a server that accepted TCP
           but never answered DESCRIBE wedged the session forever, with the
           reconnect policy never consulted. */
        {
            std::lock_guard<std::mutex> lock{session_mutex_};
            handshake_started_ = true;
        }
        rearmTimeout();
        sendDescribeCommand(Live555Client::continueAfterDESCRIBE);
    }
}

void Live555Client::rearmTimeout() noexcept
{
    std::lock_guard<std::mutex> lock{session_mutex_};
    if (timeout_task_)
    {
        envir().taskScheduler().unscheduleDelayedTask(timeout_task_);
        timeout_task_ = nullptr;
    }
    if (timeout_.count() > 0)
        timeout_task_ = envir().taskScheduler().scheduleDelayedTask(1000 * timeout_.count(), checkTimeout, this);
}

void Live555Client::setSessionTimeout(const std::chrono::milliseconds& timeout) noexcept
{
    std::lock_guard<std::mutex> lock{session_mutex_};
    timeout_ = timeout;
    if (timeout_task_)
    {
        envir().taskScheduler().unscheduleDelayedTask(timeout_task_);
        timeout_task_ = nullptr;
    }
    if (timeout_.count() > 0 && (session_active_ || handshake_started_))
    {
        timeout_task_ = envir().taskScheduler().scheduleDelayedTask(1000 * timeout_.count(), checkTimeout, this);
    }
}

void Live555Client::teardown() noexcept
{
    std::lock_guard<std::mutex> lock{session_mutex_};
    session_active_ = false;
    handshake_started_ = false;
    subsession_ = nullptr;
    candidates_.clear();
    try_order_.clear();
    try_pos_ = 0;
    if (session_)
    {
        bool hadActiveSessions = false;
        MediaSubsessionIterator iter{*session_};
        while (MediaSubsession* subsession = iter.next())
        {
            if (subsession->sink)
            {
                Medium::close(subsession->sink);
            }
            if (subsession->rtcpInstance())
            {
                subsession->rtcpInstance()->setByeHandler(nullptr, nullptr);
            }
            hadActiveSessions = true;
        }
        if (hadActiveSessions)
            sendTeardownCommand(*session_, nullptr);
        session_.reset();
    }
    if (timeout_task_)
    {
        envir().taskScheduler().unscheduleDelayedTask(timeout_task_);
        timeout_task_ = nullptr;
    }
}

void Live555Client::continueAfterDESCRIBE(RTSPClient* client, int resultCode, char* resultString) noexcept
{
    Live555Client* c = static_cast<Live555Client*>(client);
    UsageEnvironment& env = c->envir();
    std::shared_ptr<char> sdpInfo(resultString, [](char* p) { delete[] p; });
    std::shared_ptr<StreamClient> sc = c->streamClient();
    if (sc)
    {
        RCLCPP_DEBUG(sc->logger(), "[%s] DESCRIBE command completed with result code %d", sc->topicName().c_str(),
                     resultCode);
        if (resultCode != 0)
        {
            sc->sessionFailed(resultCode, resultString ? resultString : env.getResultMsg());
            return;
        }
        RCLCPP_DEBUG(sc->logger(), "[%s] received SDP parameters:\n%s", sc->topicName().c_str(), sdpInfo.get());
        c->rearmTimeout();
        c->has_video_ = false;
        MediaSession* session = MediaSession::createNew(env, sdpInfo.get());
        if (!session)
        {
            sc->sessionFailed(500, "cannot create MediaSession");
            return;
        }
        c->session_.reset(session, static_cast<void (*)(Medium*)>(Medium::close));
        if (!c->session_->hasSubsessions())
        {
            c->teardown();
            sc->sessionFailed(415, "no media subsession");
            return;
        }
        if (!collectVideoSubsessions(c, sc))
        {
            c->teardown();
            sc->sessionFailed(415, "no supported video subsession");
            return;
        }
        setupNextSubsession(c);
    }
}

/* Enumerates all video subsessions the server offers and determines the order
   in which they are tried. Returns false if the stream carries no video this
   client can decode. */
bool Live555Client::collectVideoSubsessions(Live555Client* c, const std::shared_ptr<StreamClient>& sc) noexcept
{
    c->candidates_.clear();
    c->try_order_.clear();
    c->try_pos_ = 0;

    std::string summary;
    MediaSubsessionIterator iter{*c->session_};
    while (MediaSubsession* subsession = iter.next())
    {
        const char* codec_name = subsession->codecName();
        VideoCodec codec = codec_name ? fromRTSPCodecName(codec_name) : VideoCodec::Unknown;
        if (codec == VideoCodec::Unknown)
        {
            RCLCPP_DEBUG(sc->logger(), "[%s] ignoring RTSP media subsession with unsupported codec %s",
                         sc->topicName().c_str(), codec_name ? codec_name : "(none)");
            continue;
        }
        if (!summary.empty())
            summary += ", ";
        summary += std::format("#{} {}", c->candidates_.size(), videoCodecName(codec));
        if (subsession->videoWidth() > 0 && subsession->videoHeight() > 0)
            summary += std::format(" {}x{}", subsession->videoWidth(), subsession->videoHeight());
        if (subsession->videoFPS() > 0)
            summary += std::format(" @{}Hz", subsession->videoFPS());
        c->candidates_.push_back(subsession);
    }
    if (c->candidates_.empty())
        return false;

    RCLCPP_INFO(sc->logger(), "[%s] RTSP stream offers %zu video subsession(s): %s", sc->topicName().c_str(),
                c->candidates_.size(), summary.c_str());
    std::size_t selected = sc->videoSubsession();
    if (selected >= c->candidates_.size())
    {
        RCLCPP_WARN(sc->logger(), "[%s] video_subsession %zu does not exist, falling back to subsession 0",
                    sc->topicName().c_str(), selected);
        selected = 0;
    }
    c->try_order_.push_back(selected);
    for (std::size_t i = 0; i < c->candidates_.size(); ++i)
    {
        if (i != selected)
            c->try_order_.push_back(i);
    }
    return true;
}

void Live555Client::continueAfterSETUP(RTSPClient* client, int resultCode, char* resultString) noexcept
{
    Live555Client* c = static_cast<Live555Client*>(client);
    UsageEnvironment& env = c->envir();
    std::shared_ptr<char> tmp(resultString, [](char* p) { delete[] p; });
    std::shared_ptr<StreamClient> sc = c->streamClient();
    if (sc)
    {
        RCLCPP_DEBUG(sc->logger(), "[%s] SETUP command completed with result code %d", sc->topicName().c_str(),
                     resultCode);
        if (resultCode == 0)
        {
            try
            {
                VideoCodec codec = fromRTSPCodecName(c->subsession_->codecName());
                RCLCPP_DEBUG(sc->logger(), "[%s] found media subsession with %s from %s", sc->topicName().c_str(),
                             videoCodecName(codec).c_str(), sc->url().c_str());
                sc->subsessionStarted(codec, c->subsession_);
                FrameExtractor* sink = FrameExtractor::createNew(sc, env, c->subsession_);
                c->subsession_->sink = sink;
                c->subsession_->miscPtr = c;
                sink->startPlaying(*c->subsession_->readSource(), subsessionAfterPlaying, c->subsession_);
                RTCPInstance* rtcp = c->subsession_->rtcpInstance();
                if (rtcp)
                {
                    rtcp->setByeHandler(subsessionAfterPlaying, c->subsession_);
                }
                c->has_video_ = true;
                sc->codec_.store(codec, std::memory_order_relaxed);
            }
            catch (const std::exception& e)
            {
                RCLCPP_WARN(sc->logger(), "[%s] failed to play RTSP media subsession: %s", sc->topicName().c_str(),
                            e.what());
            }
        }
        else
        {
            RCLCPP_WARN(sc->logger(), "[%s] failed to setup RTSP media subsession: %s", sc->topicName().c_str(),
                        env.getResultMsg());
        }
        if (!c->has_video_)
        {
            /* Fall back to the next video subsession the server offers */
            setupNextSubsession(c);
            return;
        }
        sc->sessionReady();
        RCLCPP_DEBUG(sc->logger(), "[%s] sending PLAY command", sc->topicName().c_str());
        c->rearmTimeout();
        c->sendPlayCommand(*c->session_, continueAfterPLAY);
    }
}

void Live555Client::continueAfterPLAY(RTSPClient* client, int resultCode, char* resultString) noexcept
{
    Live555Client* c = static_cast<Live555Client*>(client);
    std::shared_ptr<char> tmp(resultString, [](char* p) { delete[] p; });
    std::shared_ptr<StreamClient> sc = c->streamClient();
    if (sc)
    {
        RCLCPP_DEBUG(sc->logger(), "[%s] PLAY command completed with result code %d", sc->topicName().c_str(),
                     resultCode);
        if (resultCode == 0)
        {
            std::unique_lock<std::mutex> lock{c->session_mutex_};
            c->session_active_ = true;
            /* Replace, not stack: the handshake watchdog may still be
               pending — two live tasks would double-fire checkTimeout. */
            if (c->timeout_task_)
            {
                c->envir().taskScheduler().unscheduleDelayedTask(c->timeout_task_);
                c->timeout_task_ = nullptr;
            }
            if (c->timeout_.count() > 0)
                c->timeout_task_ =
                    c->envir().taskScheduler().scheduleDelayedTask(1000 * c->timeout_.count(), checkTimeout, c);
            lock.unlock();
            sc->sessionStarted();
        }
        else
        {
            c->teardown();
            sc->sessionFailed(resultCode, resultString ? resultString : c->envir().getResultMsg());
        }
    }
}

void Live555Client::setupNextSubsession(Live555Client* c) noexcept
{
    UsageEnvironment& env = c->envir();
    std::shared_ptr<StreamClient> sc = c->streamClient();
    if (!sc)
        return;
    /* Only the subsession that is actually going to be played is initiated, so
       no RTP/RTCP sockets are allocated for the streams we are not interested
       in. */
    while (c->try_pos_ < c->try_order_.size())
    {
        const std::size_t index = c->try_order_[c->try_pos_++];
        c->subsession_ = c->candidates_[index];
        VideoCodec codec = fromRTSPCodecName(c->subsession_->codecName());
        if (c->subsession_->initiate())
        {
            RCLCPP_INFO(sc->logger(), "[%s] selected RTSP video subsession %zu with %s video",
                        sc->topicName().c_str(), index, videoCodecName(codec).c_str());
            RCLCPP_DEBUG(sc->logger(), "[%s] sending SETUP command for media subsession", sc->topicName().c_str());
            c->sendSetupCommand(*c->subsession_, continueAfterSETUP);
            return;
        }
        RCLCPP_WARN(sc->logger(), "[%s] failed to initiate RTSP video subsession %zu: %s", sc->topicName().c_str(),
                    index, env.getResultMsg());
    }
    c->subsession_ = nullptr;
    c->teardown();
    sc->sessionFailed(415, "no playable video subsession");
}

void Live555Client::subsessionAfterPlaying(void* obj) noexcept
{
    MediaSubsession* subsession = static_cast<MediaSubsession*>(obj);
    Live555Client* c = static_cast<Live555Client*>(subsession->miscPtr);
    std::shared_ptr<StreamClient> sc = c->streamClient();
    if (sc)
    {
        RCLCPP_DEBUG(sc->logger(), "[%s] media subsession finished playing", sc->topicName().c_str());
        sc->subsessionFinished(subsession);
    }
    Medium::close(subsession->sink);
    subsession->sink = nullptr;

    MediaSession& session = subsession->parentSession();
    MediaSubsessionIterator iter(session);
    while ((subsession = iter.next()))
    {
        if (subsession->sink)
            return;  // this subsession is still active
    }

    // All subsession streams have now been closed, so shutdown the client
    c->teardown();
    if (sc)
    {
        RCLCPP_DEBUG(sc->logger(), "[%s] media session finished", sc->topicName().c_str());
        sc->sessionFinished();
    }
}

void Live555Client::checkTimeout(void* obj) noexcept
{
    Live555Client* c = static_cast<Live555Client*>(obj);
    c->timeout_task_ = nullptr;
    std::shared_ptr<StreamClient> sc = c->streamClient();
    if (sc)
    {
        std::unique_lock<std::mutex> lock{c->session_mutex_};
        if (!c->session_active_)
        {
            /* Handshake watchdog fired: a full timeout period without a
               DESCRIBE/SETUP/PLAY step completing (each completed step
               re-arms). Report a session failure — not a timeout — so the
               reconnect path applies its exponential backoff. */
            lock.unlock();
            c->teardown();
            sc->sessionFailed(408, "RTSP session setup timed out");
            return;
        }
        std::size_t new_received_packets = 0;
        if (c->session_)
        {
            MediaSubsessionIterator iter(*c->session_);
            while (MediaSubsession* subsession = iter.next())
            {
                RTPSource* src = subsession->rtpSource();
                if (src)
                    new_received_packets += src->receptionStatsDB().totNumPacketsReceived();
            }
            if (new_received_packets != c->received_packets_)
            {
                c->received_packets_ = new_received_packets;
                c->timeout_task_ =
                    c->envir().taskScheduler().scheduleDelayedTask(1000 * c->timeout_.count(), checkTimeout, c);
            }
            else
            {
                lock.unlock();
                sc->sessionTimeout();
            }
        }
    }
}

std::shared_ptr<StreamClient> StreamClient::create(const std::string& topic_name, const std::string& url,
                                                   const rclcpp::Logger& logger) noexcept
{
    return std::shared_ptr<StreamClient>(new StreamClient(topic_name, url, logger));
}

StreamClient::StreamClient(const std::string& topic_name, const std::string& url, const rclcpp::Logger& logger) noexcept
    : topic_name_(topic_name), url_(url), logger_(logger), codec_(VideoCodec::Unknown), video_subsession_(0),
      retried_on_454_error_(false), timeout_(0), loop_(EventLoop::create()), client_(nullptr)
{
}

StreamClient::~StreamClient()
{
    /* Stop the dispatcher FIRST: teardown deletes the RTSPClient and its
       FrameExtractor sink, and a still-running loop may be inside
       deliverFrame on those very objects. With the loop stopped — or when
       this destructor already runs on the loop thread, where stop() returns
       immediately and the loop is us — the close below cannot race a
       handler. */
    loop_->stop();
    disconnect();
}

VideoCodec StreamClient::codec() const noexcept
{
    return codec_.load(std::memory_order_relaxed);
}

std::string StreamClient::topicName() const noexcept
{
    return topic_name_;
}

const rclcpp::Logger& StreamClient::logger() const noexcept
{
    return logger_;
}

std::size_t StreamClient::videoSubsession() const noexcept
{
    std::lock_guard<std::mutex> lock{client_mutex_};
    return video_subsession_;
}

void StreamClient::setVideoSubsession(std::size_t index) noexcept
{
    std::lock_guard<std::mutex> lock{client_mutex_};
    video_subsession_ = index;
}

std::string StreamClient::url() const noexcept
{
    return url_;
}

void StreamClient::connect()
{
    /* All Live555 object manipulation happens on the loop thread: RTSPClient
       registers with the task scheduler, whose socket/timer state a
       concurrently running loop mutates without locks. post() runs this
       inline when already on the loop thread (the 454 workaround path) and
       propagates the StreamingError to foreign callers. */
    loop_->post([this] {
        Live555Client* client;
        {
            std::lock_guard<std::mutex> lock{client_mutex_};
            if (client_)
                throw StreamingError("client is connected already");
            client_ = Live555Client::createNew(shared_from_this(), loop_->env(), url_.c_str(), 0, logger_.get_name());
            client_->setSessionTimeout(timeout_);
            client = client_;
        }
        RCLCPP_DEBUG(logger_, "[%s] connecting to %s", topic_name_.c_str(), url_.c_str());
        /* Live555 reports errors it detects right away (an unparsable URL, a
           host that cannot be resolved) by invoking the response handler from
           inside this call. The handler ends up in disconnect() or connect()
           again, so the mutex must not be held here. */
        client->initiateSetup();
    });
}

void StreamClient::disconnect()
{
    loop_->post([this] {
        std::lock_guard<std::mutex> lock{client_mutex_};
        if (client_)
        {
            RCLCPP_DEBUG(logger_, "[%s] disconnecting from %s", topic_name_.c_str(), url_.c_str());
            client_->teardown();
            Medium::close(client_);
            client_ = nullptr;
        }
    });
}

void StreamClient::sessionFailed(int code, const std::string& message)
{
    /* Workaround for certain ACTi boxes, which like to bungle the
       session setup on the first connection attempt */
    if (code == 454 && !retried_on_454_error_)
    {
        RCLCPP_DEBUG(logger_, "[%s] triggering workaround for transient 454 error with faulty IP cameras",
                     topic_name_.c_str());
        retried_on_454_error_ = true;
        disconnect();
        connect();
        return;
    }
    if (session_failed_handler_)
        session_failed_handler_(code, message);
}

void StreamClient::sessionReady()
{
    if (session_ready_handler_)
        session_ready_handler_();
}

void StreamClient::sessionStarted()
{
    retried_on_454_error_ = false;
    if (session_started_handler_)
        session_started_handler_();
}

void StreamClient::sessionFinished()
{
    if (session_finished_handler_)
        session_finished_handler_();
}

void StreamClient::sessionTimeout()
{
    if (session_timeout_handler_)
        session_timeout_handler_();
}

void StreamClient::subsessionStarted(VideoCodec codec, MediaSubsession* subsession)
{
    if (subsession_started_handler_)
        subsession_started_handler_(codec, subsession);
}

void StreamClient::subsessionFinished(MediaSubsession* subsession)
{
    if (subsession_finished_handler_)
        subsession_finished_handler_(subsession);
}

void StreamClient::receiveStreamData(VideoCodec codec, MediaSubsession* subsession, const FrameDataPtr& data)
{
    if (receive_stream_data_handler_)
        receive_stream_data_handler_(codec, subsession, data);
}

void StreamClient::setSessionTimeout(const std::chrono::milliseconds& timeout) noexcept
{
    /* Reaches into the task scheduler ((un)scheduleDelayedTask), which is
       loop-thread-only. */
    loop_->post([this, timeout] {
        std::lock_guard<std::mutex> lock{client_mutex_};
        timeout_ = timeout;
        if (client_)
            client_->setSessionTimeout(timeout_);
    });
}

void StreamClient::setSubsessionStartedHandler(SubsessionStartedHandler handler) noexcept
{
    subsession_started_handler_ = handler;
}

void StreamClient::setSubsessionFinishedHandler(SubsessionFinishedHandler handler) noexcept
{
    subsession_finished_handler_ = handler;
}

void StreamClient::setSessionFailedHandler(SessionFailedHandler handler) noexcept
{
    session_failed_handler_ = handler;
}

void StreamClient::setSessionReadyHandler(SessionReadyHandler handler) noexcept
{
    session_ready_handler_ = handler;
}

void StreamClient::setSessionStartedHandler(SessionStartedHandler handler) noexcept
{
    session_started_handler_ = handler;
}

void StreamClient::setSessionFinishedHandler(SessionFinishedHandler handler) noexcept
{
    session_finished_handler_ = handler;
}

void StreamClient::setSessionTimeoutHandler(SessionTimeoutHandler handler) noexcept
{
    session_timeout_handler_ = handler;
}

void StreamClient::setReceiveStreamDataHandler(ReceiveStreamDataHandler handler) noexcept
{
    receive_stream_data_handler_ = handler;
}

}  // namespace rtsp_image_transport
