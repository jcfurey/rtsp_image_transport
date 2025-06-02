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

namespace rtsp_image_transport
{

namespace
{

void reclaim_env(UsageEnvironment* env)
{
    if (env)
        env->reclaim();
}

}  // namespace

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

private:
    Live555Client(const std::weak_ptr<StreamClient>& stream_client, UsageEnvironment& env, char const* url,
                  int verbosity, char const* appName, portNumBits tunnelOverHTTPPortNum, int socketNumToServer) noexcept
        : RTSPClient(env, url, verbosity, appName, tunnelOverHTTPPortNum, socketNumToServer),
          stream_client_(stream_client), session_(nullptr), subsession_(nullptr), has_video_(false),
          session_active_(false), timeout_task_(nullptr), received_packets_(0)
    {
    }

    static void continueAfterDESCRIBE(RTSPClient* client, int resultCode, char* resultMsg) noexcept;
    static void continueAfterSETUP(RTSPClient* client, int resultCode, char* resultMsg) noexcept;
    static void continueAfterPLAY(RTSPClient* client, int resultCode, char* resultMsg) noexcept;
    static void continueAfterTEARDOWN(RTSPClient* client, int resultCode, char* resultMsg) noexcept;
    static void setupNextSubsession(Live555Client* c) noexcept;
    static void subsessionAfterPlaying(void* obj) noexcept;
    static void checkTimeout(void* obj) noexcept;

    std::weak_ptr<StreamClient> stream_client_;
    std::mutex session_mutex_;
    std::shared_ptr<MediaSession> session_;
    MediaSubsession* subsession_;
    bool has_video_, session_active_;
    std::unique_ptr<MediaSubsessionIterator> iter_;
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
        sendDescribeCommand(Live555Client::continueAfterDESCRIBE);
    }
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
    if (timeout_.count() > 0 && session_active_)
    {
        timeout_task_ = envir().taskScheduler().scheduleDelayedTask(1000 * timeout_.count(), checkTimeout, this);
    }
}

void Live555Client::teardown() noexcept
{
    std::lock_guard<std::mutex> lock{session_mutex_};
    session_active_ = false;
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
            sc->sessionFailed(resultCode, resultString);
            return;
        }
        RCLCPP_DEBUG(sc->logger(), "[%s] received SDP parameters:\n%s", sc->topicName().c_str(), sdpInfo.get());
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
        c->iter_ = std::make_unique<MediaSubsessionIterator>(*c->session_);
        setupNextSubsession(c);
    }
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
                sc->codec_ = codec;
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
        setupNextSubsession(c);
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
            if (c->timeout_.count() > 0)
                c->timeout_task_ =
                    c->envir().taskScheduler().scheduleDelayedTask(1000 * c->timeout_.count(), checkTimeout, c);
            lock.unlock();
            sc->sessionStarted();
        }
        else
        {
            c->teardown();
            sc->sessionFailed(resultCode, resultString);
        }
    }
}

void Live555Client::setupNextSubsession(Live555Client* c) noexcept
{
    UsageEnvironment& env = c->envir();
    std::shared_ptr<StreamClient> sc = c->streamClient();
    if (sc)
    {
        c->subsession_ = c->iter_->next();
        if (c->subsession_)
        {
            if (c->subsession_->initiate())
            {
                std::string codecName = c->subsession_->codecName();
                RCLCPP_DEBUG(sc->logger(), "[%s] initiated RTSP media subsession with %s stream",
                             sc->topicName().c_str(), codecName.c_str());
                VideoCodec codec = fromRTSPCodecName(codecName);
                if (codec != VideoCodec::Unknown)
                {
                    if (!c->has_video_)
                    {
                        RCLCPP_DEBUG(sc->logger(), "[%s] sending SETUP command for media subsession",
                                     sc->topicName().c_str());
                        c->sendSetupCommand(*c->subsession_, continueAfterSETUP);
                    }
                    else
                    {
                        RCLCPP_WARN(sc->logger(), "[%s] ignoring additional RTSP media subsession with %s video",
                                    sc->topicName().c_str(), videoCodecName(codec).c_str());
                        setupNextSubsession(c);
                    }
                }
                else
                {
                    RCLCPP_WARN(sc->logger(), "[%s] ignoring RTSP media subsession with unsupported codec %s",
                                sc->topicName().c_str(), codecName.c_str());
                    setupNextSubsession(c);
                }
            }
            else
            {
                RCLCPP_WARN(sc->logger(), "[%s] failed to initiate RTSP media subsession: %s", sc->topicName().c_str(),
                            env.getResultMsg());
                setupNextSubsession(c);
            }
            return;
        }
        c->iter_.reset();
        if (!c->has_video_)
        {
            c->teardown();
            sc->sessionFailed(415, "no playable media subsession");
            return;
        }
        sc->sessionReady();
        RCLCPP_DEBUG(sc->logger(), "[%s] sending PLAY command", sc->topicName().c_str());
        c->sendPlayCommand(*c->session_, continueAfterPLAY);
    }
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
    : topic_name_(topic_name), url_(url), logger_(logger), codec_(VideoCodec::Unknown), quit_flag_(0),
      retried_on_454_error_(false), timeout_(0), scheduler_(BasicTaskScheduler::createNew()),
      env_(BasicUsageEnvironment::createNew(*scheduler_), reclaim_env),
      event_loop_thread_([this]() { this->env_->taskScheduler().doEventLoop(&this->quit_flag_); }), client_(nullptr)
{
}

StreamClient::~StreamClient()
{
    disconnect();
    quit_flag_ = 1;
    event_loop_thread_.join();
}

VideoCodec StreamClient::codec() const noexcept
{
    return codec_;
}

std::string StreamClient::topicName() const noexcept
{
    return topic_name_;
}

const rclcpp::Logger& StreamClient::logger() const noexcept
{
    return logger_;
}

std::string StreamClient::url() const noexcept
{
    return url_;
}

void StreamClient::connect()
{
    std::lock_guard<std::mutex> lock{client_mutex_};
    if (client_)
        throw StreamingError("client is connected already");
    client_ = Live555Client::createNew(shared_from_this(), *env_, url_.c_str(), 0, logger_.get_name());
    client_->setSessionTimeout(timeout_);
    RCLCPP_DEBUG(logger_, "[%s] connecting to %s", topic_name_.c_str(), url_.c_str());
    client_->initiateSetup();
}

void StreamClient::disconnect()
{
    std::lock_guard<std::mutex> lock{client_mutex_};
    if (client_)
    {
        RCLCPP_DEBUG(logger_, "[%s] disconnecting from %s", topic_name_.c_str(), url_.c_str());
        client_->teardown();
        Medium::close(client_);
        client_ = nullptr;
    }
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
    std::lock_guard<std::mutex> lock{client_mutex_};
    timeout_ = timeout;
    if (client_)
        client_->setSessionTimeout(timeout_);
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
