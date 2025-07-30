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
#include "subscriber_plugin.h"

#include "init.h"
#include "stream_client.h"
#include "stream_decoder.h"
#include "streaming_error.h"

#include <rclcpp/waitable.hpp>

#include <functional>

namespace rtsp_image_transport
{

namespace
{

class ScheduledCB : public rclcpp::Waitable
{
public:
    using Signature = std::function<void()>;
    using SharedPtr = std::shared_ptr<ScheduledCB>;

    explicit ScheduledCB(const Signature& func);
    void trigger();

    std::size_t get_number_of_ready_guard_conditions() override;
    std::shared_ptr<void> take_data() override;
    std::shared_ptr<void> take_data_by_entity_id(std::size_t id) override;
    void set_on_ready_callback(std::function<void(std::size_t, int)>) override;
    void clear_on_ready_callback() override;
    void add_to_wait_set(rcl_wait_set_t& wait_set) override;
    bool is_ready(const rcl_wait_set_t&) override;
    void execute(const std::shared_ptr<void>&) override;
#if CURRENT_RCLCPP_VERSION >= FKIE_VERSION_TUPLE(29, 4, 0)
    std::vector<std::shared_ptr<rclcpp::TimerBase>> get_timers() const override;
#endif

private:
    Signature func_;
    rclcpp::GuardCondition cond_;
};

ScheduledCB::ScheduledCB(const Signature& func) : func_(func) {}

void ScheduledCB::trigger()
{
    cond_.trigger();
}

std::size_t ScheduledCB::get_number_of_ready_guard_conditions()
{
    return 1;
}

std::shared_ptr<void> ScheduledCB::take_data()
{
    return nullptr;
}

std::shared_ptr<void> ScheduledCB::take_data_by_entity_id(std::size_t id)
{
    return nullptr;
}

void ScheduledCB::set_on_ready_callback(std::function<void(std::size_t, int)>) {}

void ScheduledCB::clear_on_ready_callback() {}

void ScheduledCB::add_to_wait_set(rcl_wait_set_t& wait_set)
{
    cond_.add_to_wait_set(wait_set);
}

bool ScheduledCB::is_ready(const rcl_wait_set_t&)
{
    return true;
}

void ScheduledCB::execute(const std::shared_ptr<void>&)
{
    func_();
}

#if CURRENT_RCLCPP_VERSION >= FKIE_VERSION_TUPLE(29, 4, 0)
std::vector<std::shared_ptr<rclcpp::TimerBase>> ScheduledCB::get_timers() const
{
    return {};
}
#endif

}  // namespace

enum ReconnectPolicy
{
    ReconnectNever = 0,
    ReconnectOnTimeout = 1,
    ReconnectOnFailure = 2,
    ReconnectAlways = 3,
};

using namespace std::chrono_literals;

struct RTSP_IMAGE_TRANSPORT_NO_EXPORT SubscriberPlugin::Config
{
    bool use_hw_decoder = false;
    ReconnectPolicy reconnect_policy = ReconnectOnTimeout;
    std::chrono::milliseconds timeout = 2s;
    std::chrono::milliseconds reconnect_minwait = 100ms;
    std::chrono::milliseconds reconnect_maxwait = 30s;
};

using SuperClass = image_transport::SimpleSubscriberPlugin<std_msgs::msg::String>;

SubscriberPlugin::SubscriberPlugin()
    : SuperClass(), logger_(rclcpp::get_logger("rtsp_image_transport")), config_(std::make_unique<Config>()),
      old_lag_(0s)
{
    global_initialize();
}

void SubscriberPlugin::shutdown()
{
    cooldown_cb_group_.reset();
    cooldown_timer_.reset();
    if (client_)
        client_->disconnect();
    decoder_.reset();
    clearQueuedFrames();
    SuperClass::shutdown();
}

std::string SubscriberPlugin::getTransportName() const
{
    return "rtsp";
}

void SubscriberPlugin::subscribeImpl(rclcpp::Node* node, const std::string& base_topic, const Callback& callback,
                                     rmw_qos_profile_t custom_qos, rclcpp::SubscriptionOptions options)
{
    custom_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    custom_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    custom_qos.depth = 1;
    custom_qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    SuperClass::subscribeImpl(node, base_topic, callback, custom_qos, options);
    logger_ = node->get_logger();
    clock_ = node->get_clock();
    scheduled_cb_ = std::make_shared<ScheduledCB>(std::bind(&SubscriberPlugin::processFrame, this));
    scheduled_cb_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    node->get_node_waitables_interface()->add_waitable(scheduled_cb_, scheduled_cb_group_);
    topic_name_ = base_topic;
    node_base_ = rclcpp::node_interfaces::get_node_base_interface(node);
    node_timers_ = rclcpp::node_interfaces::get_node_timers_interface(node);
    node_param_ = rclcpp::node_interfaces::get_node_parameters_interface(node);
    failed_ = false;
    std::size_t len = node->get_effective_namespace().length();
    param_base_name_ = base_topic.substr(len);
    std::replace(param_base_name_.begin(), param_base_name_.end(), '/', '.');
    if (!param_base_name_.empty() && param_base_name_[0] == '.')
        param_base_name_ = param_base_name_.substr(1);
    if (!param_base_name_.empty())
        param_base_name_.push_back('.');
    param_base_name_ += getTransportName();
    setupParameters(node);
    param_cb_handle_ = node->add_post_set_parameters_callback([this](const std::vector<rclcpp::Parameter>&)
                                                              { this->updateParameters(); });
    updateParameters();
}

void SubscriberPlugin::internalCallback(const std_msgs::msg::String::ConstSharedPtr& msg, const Callback& callback)
{
    RCLCPP_DEBUG(logger_, "[%s] received updated RTSP URL: %s", topic_name_.c_str(), msg->data.c_str());
    failed_ = false;
    old_lag_ = 0s;
    callback_ = callback;
    cooldown_ = config_->reconnect_minwait;
    cooldown_cb_group_.reset();
    cooldown_timer_.reset();
    client_.reset();
    try
    {
        client_ = StreamClient::create(topic_name_, msg->data, logger_);
        client_->setSessionTimeout(config_->timeout);
        client_->setReceiveStreamDataHandler(std::bind(&SubscriberPlugin::receiveDataStream, this,
                                                       std::placeholders::_1, std::placeholders::_2,
                                                       std::placeholders::_3));
        client_->setSessionTimeoutHandler(std::bind(&SubscriberPlugin::sessionTimeout, this));
        client_->setSubsessionStartedHandler(
            std::bind(&SubscriberPlugin::subsessionStarted, this, std::placeholders::_1, std::placeholders::_2));
        client_->setSessionStartedHandler(std::bind(&SubscriberPlugin::sessionStarted, this));
        client_->setSessionFailedHandler(
            std::bind(&SubscriberPlugin::sessionFailed, this, std::placeholders::_1, std::placeholders::_2));
        client_->setSessionFinishedHandler(std::bind(&SubscriberPlugin::sessionFinished, this));
        client_->connect();
    }
    catch (std::exception& e)
    {
        RCLCPP_ERROR(logger_, "[%s] %s", topic_name_.c_str(), e.what());
        failed_ = true;
        clearQueuedFrames();
        if (config_->reconnect_policy >= ReconnectOnFailure)
        {
            reconnect();
        }
    }
}

void SubscriberPlugin::subsessionStarted(VideoCodec codec, MediaSubsession* subsession)
{
    old_lag_ = rclcpp::Duration(0, 0);
    RCLCPP_DEBUG(logger_, "[%s] setting up decoder for %s", topic_name_.c_str(), videoCodecName(codec).c_str());
    decoder_ = std::make_shared<StreamDecoder>(codec, config_->use_hw_decoder, logger_);
    RCLCPP_INFO(logger_, "[%s] start decoding (%s) from %s", topic_name_.c_str(), decoder_->context()->codec->name,
                client_->url().c_str());
}

void SubscriberPlugin::receiveDataStream(VideoCodec codec, MediaSubsession* subsession, const FrameDataPtr& data)
{
    RCLCPP_DEBUG_THROTTLE(logger_, *clock_, 30000, "[%s] receiving video frames from RTSP stream", topic_name_.c_str());
    pushFrame(data);
    dynamic_cast<ScheduledCB&>(*scheduled_cb_).trigger();
}

void SubscriberPlugin::setupParameters(rclcpp::Node* node)
{
    using rcl_interfaces::msg::ParameterDescriptor;
    if (!node->has_parameter(param_base_name_ + ".use_hw_decoder"))
        node->declare_parameter<bool>(
            param_base_name_ + ".use_hw_decoder", config_->use_hw_decoder,
            ParameterDescriptor().set__description("use NVDEC hardware acceleration if possible"));
    if (!node->has_parameter(param_base_name_ + ".reconnect_policy"))
        node->declare_parameter<int>(
            param_base_name_ + ".reconnect_policy", config_->reconnect_policy,
            ParameterDescriptor()
                .set__description("client reconnect policy (0 = never, 1 = on timeout, 2 = on failure, 3 = always)")
                .set__integer_range({rcl_interfaces::msg::IntegerRange().set__from_value(0).set__to_value(3)}));
    if (!node->has_parameter(param_base_name_ + ".timeout"))
        node->declare_parameter<double>(
            param_base_name_ + ".timeout", 1e-3 * config_->timeout.count(),
            ParameterDescriptor()
                .set__description("client session timeout [s] (0 = unlimited)")
                .set__floating_point_range(
                    {rcl_interfaces::msg::FloatingPointRange().set__from_value(0).set__to_value(60)}));
    if (!node->has_parameter(param_base_name_ + ".reconnect_minwait"))
        node->declare_parameter<double>(
            param_base_name_ + ".reconnect_minwait", 1e-3 * config_->reconnect_minwait.count(),
            ParameterDescriptor()
                .set__description("minimum delay between connection attempts [s]")
                .set__floating_point_range(
                    {rcl_interfaces::msg::FloatingPointRange().set__from_value(0).set__to_value(60)}));
    if (!node->has_parameter(param_base_name_ + ".reconnect_maxwait"))
        node->declare_parameter<double>(
            param_base_name_ + ".reconnect_maxwait", 1e-3 * config_->reconnect_maxwait.count(),
            ParameterDescriptor()
                .set__description("maximum delay between connection attempts [s]")
                .set__floating_point_range(
                    {rcl_interfaces::msg::FloatingPointRange().set__from_value(0).set__to_value(600)}));
}

void SubscriberPlugin::updateParameters()
{
    static constexpr int LVL_CODEC = 1;

    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr np = node_param_.lock();
    if (!np)
        return;
    Config new_config;
    new_config.use_hw_decoder = np->get_parameter(param_base_name_ + ".use_hw_decoder").as_bool();
    new_config.reconnect_policy =
        static_cast<ReconnectPolicy>(np->get_parameter(param_base_name_ + ".reconnect_policy").as_int());
    new_config.timeout = std::chrono::milliseconds(static_cast<std::chrono::milliseconds::rep>(
        1000 * np->get_parameter(param_base_name_ + ".timeout").as_double()));
    new_config.reconnect_minwait = std::chrono::milliseconds(static_cast<std::chrono::milliseconds::rep>(
        1000 * np->get_parameter(param_base_name_ + ".reconnect_minwait").as_double()));
    new_config.reconnect_maxwait = std::chrono::milliseconds(static_cast<std::chrono::milliseconds::rep>(
        1000 * np->get_parameter(param_base_name_ + ".reconnect_maxwait").as_double()));

    int changelevel = 0;
    if (new_config.use_hw_decoder != config_->use_hw_decoder)
        changelevel |= LVL_CODEC;

    *config_ = new_config;
    try
    {
        cooldown_ = config_->reconnect_minwait;
        if (config_->reconnect_maxwait < config_->reconnect_minwait)
            config_->reconnect_maxwait = config_->reconnect_minwait;
        if (client_)
        {
            client_->setSessionTimeout(config_->timeout);
            if (changelevel >= LVL_CODEC)
            {
                decoder_ = std::make_shared<StreamDecoder>(client_->codec(), config_->use_hw_decoder, logger_);
                RCLCPP_INFO(logger_, "[%s] start decoding (%s) from %s", topic_name_.c_str(),
                            decoder_->context()->codec->name, client_->url().c_str());
            }
        }
    }
    catch (std::exception& e)
    {
        RCLCPP_ERROR(logger_, "[%s] %s", topic_name_.c_str(), e.what());
        failed_ = true;
        clearQueuedFrames();
        if (config_->reconnect_policy >= ReconnectOnFailure)
        {
            reconnect();
        }
    }
}

void SubscriberPlugin::processFrame()
{
    using namespace std::chrono_literals;
    std::shared_ptr<StreamDecoder> decoder = decoder_;
    if (failed_ || !decoder)
        return;
    try
    {
        while (FrameDataPtr frame = popFrame())
        {
            rclcpp::Duration lag = frameLag();
            if (lag >= 2s)
            {
                if (old_lag_ < 2s)
                {
                    RCLCPP_WARN(logger_, "[%s] decoder is too slow; discarding all frames", topic_name_.c_str());
                    old_lag_ = lag;
                }
                decoder->setDecodeFrames(StreamDecoder::DecodeFrames::None);
            }
            else if (lag >= 1s)
            {
                if (old_lag_ < 1s)
                {
                    RCLCPP_WARN(logger_, "[%s] decoder is too slow; discarding non-key frames", topic_name_.c_str());
                    old_lag_ = lag;
                }
                decoder->setDecodeFrames(StreamDecoder::DecodeFrames::Key);
            }
            if (lag >= 500ms)
            {
                if (old_lag_ < 500ms)
                {
                    RCLCPP_WARN(logger_, "[%s] decoder is too slow; discarding non-intra frames", topic_name_.c_str());
                    old_lag_ = lag;
                }
                decoder->setDecodeFrames(StreamDecoder::DecodeFrames::Intra);
            }
            else
            {
                decoder->setDecodeFrames(StreamDecoder::DecodeFrames::All);
                if (lag == 0s)
                    old_lag_ = 0s;
            }
            if (decoder->decodeVideo(frame) > 0)
            {
                while (sensor_msgs::msg::Image::ConstSharedPtr img = decoder->nextFrame())
                {
                    callback_(img);
                }
            }
        }
    }
    catch (const DecodingError& e)
    {
        RCLCPP_WARN(logger_, "[%s] %s", topic_name_.c_str(), e.what());
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(logger_, "[%s] %s", topic_name_.c_str(), e.what());
        failed_ = true;
        clearQueuedFrames();
        if (config_->reconnect_policy >= ReconnectOnFailure)
        {
            reconnect();
        }
    }
}

void SubscriberPlugin::sessionStarted()
{
    cooldown_ = config_->reconnect_minwait;
    cooldown_timer_.reset();
}

void SubscriberPlugin::sessionTimeout()
{
    RCLCPP_ERROR(logger_, "[%s] session timeout for stream at %s", topic_name_.c_str(), client_->url().c_str());
    if (config_->reconnect_policy >= ReconnectOnTimeout)
    {
        reconnect();
    }
}

void SubscriberPlugin::sessionFailed(int code, const std::string& message)
{
    RCLCPP_ERROR(logger_, "[%s] %s failed. %s (%d)", topic_name_.c_str(), client_->url().c_str(), message.c_str(),
                 code);
    if (config_->reconnect_policy >= ReconnectOnFailure)
    {
        reconnect();
    }
}

void SubscriberPlugin::sessionFinished()
{
    RCLCPP_INFO(logger_, "[%s] end of video stream", topic_name_.c_str());
    if (config_->reconnect_policy >= ReconnectAlways)
    {
        reconnect();
    }
}

void SubscriberPlugin::reconnect()
{
    client_->disconnect();
    clearQueuedFrames();
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr nb = node_base_.lock();
    rclcpp::node_interfaces::NodeTimersInterface::SharedPtr nt = node_timers_.lock();
    if (nb && nt)
    {
        RCLCPP_INFO(logger_, "[%s] new connection attempt in %0.3lf seconds", topic_name_.c_str(),
                    1e-3 * cooldown_.count());
        cooldown_cb_group_ = nb->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cooldown_timer_ = std::make_shared<rclcpp::WallTimer<rclcpp::VoidCallbackType>>(
            cooldown_, std::bind(&SubscriberPlugin::cooldownTimerCallback, this), nb->get_context());
        nt->add_timer(cooldown_timer_, cooldown_cb_group_);
        cooldown_ *= 2;
        if (cooldown_ > config_->reconnect_maxwait)
            cooldown_ = config_->reconnect_maxwait;
    }
}

void SubscriberPlugin::cooldownTimerCallback()
{
    cooldown_timer_.reset();  // just in case
    clearQueuedFrames();
    client_->connect();
}

void SubscriberPlugin::pushFrame(const FrameDataPtr& frame)
{
    std::lock_guard<std::mutex> lock{queue_mutex_};
    queue_.push_back(frame);
}

FrameDataPtr SubscriberPlugin::popFrame()
{
    std::lock_guard<std::mutex> lock{queue_mutex_};
    if (queue_.empty())
        return FrameDataPtr();
    FrameDataPtr frame = queue_.front();
    queue_.pop_front();
    return frame;
}

rclcpp::Duration SubscriberPlugin::frameLag() const noexcept
{
    std::lock_guard<std::mutex> lock{queue_mutex_};
    if (queue_.empty())
        return rclcpp::Duration(0, 0);
    return queue_.back()->stamp() - queue_.front()->stamp();
}

void SubscriberPlugin::clearQueuedFrames()
{
    std::lock_guard<std::mutex> lock{queue_mutex_};
    queue_.clear();
}

}  // namespace rtsp_image_transport

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rtsp_image_transport::SubscriberPlugin, image_transport::SubscriberPlugin)
