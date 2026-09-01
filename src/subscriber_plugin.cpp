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
#include "reconnect_policy.h"
#include "stream_client.h"
#include "stream_decoder.h"
#include "streaming_error.h"
#include "topic_parameter.h"

#include <rclcpp/detail/add_guard_condition_to_rcl_wait_set.hpp>
#include <rclcpp/waitable.hpp>

#include <atomic>
#include <functional>

namespace rtsp_image_transport
{

/* From image_transport 6.4 the plugin entry points take node interfaces rather
   than a node, and the old ones are never called. RequiredInterfaces does not
   include the clock or waitables interfaces. The newer implementation therefore
   uses a short wall timer to process queued frames on the executor and falls
   back to a system clock for receive timestamps. See RTSP_IMAGE_TRANSPORT_* in
   init.h for the 6.4 and 7.0 boundaries. */

namespace
{

void declareParameter(
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& parameters, const std::string& name,
    const rclcpp::ParameterValue& value, const rcl_interfaces::msg::ParameterDescriptor& descriptor)
{
    if (!parameters->has_parameter(name))
        parameters->declare_parameter(name, value, descriptor);
}

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
    std::atomic<bool> pending_{false};
};

ScheduledCB::ScheduledCB(const Signature& func) : func_(func) {}

void ScheduledCB::trigger()
{
    pending_.store(true);
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
    /* Waitables are collected into the executor's raw rcl wait set. Using
       GuardCondition::add_to_wait_set() instead pins the condition to one
       particular wait-set object, which breaks callers such as spin_some()
       that legitimately construct a fresh executor for every call. */
    rclcpp::detail::add_guard_condition_to_rcl_wait_set(wait_set, cond_);
}

bool ScheduledCB::is_ready(const rcl_wait_set_t&)
{
    return pending_.load();
}

void ScheduledCB::execute(const std::shared_ptr<void>&)
{
    if (pending_.exchange(false))
        func_();
}

#if CURRENT_RCLCPP_VERSION >= FKIE_VERSION_TUPLE(29, 4, 0)
std::vector<std::shared_ptr<rclcpp::TimerBase>> ScheduledCB::get_timers() const
{
    return {};
}
#endif

}  // namespace

using namespace std::chrono_literals;

/* Where the time stamp on a published image comes from */
enum TimestampSource
{
    /* The sender's clock, taken from RTP/RTCP. Accurate when the camera is
       synchronised, but it is wall clock time and therefore meaningless to a
       node running on simulated time. */
    TimestampFromSender = 0,
    /* The node clock at the moment the frame arrives. Always in the node's own
       time base, at the cost of network jitter. */
    TimestampFromReceiver = 1,
    /* Sender time on a wall clock node, receive time when simulated time is in
       use. */
    TimestampAuto = 2,
};

struct RTSP_IMAGE_TRANSPORT_NO_EXPORT SubscriberPlugin::Config
{
    StreamDecoder::Options decoder;
    std::string frame_id;
    TimestampSource timestamp_source = TimestampAuto;
    int video_subsession = 0;
    bool rtp_over_tcp = true;
    int rtp_buffer_size = static_cast<int>(DEFAULT_RTP_BUFFER_SIZE);
    ReconnectPolicy reconnect_policy = ReconnectOnTimeout;
    std::chrono::milliseconds timeout = 2s;
    std::chrono::milliseconds reconnect_minwait = 100ms;
    std::chrono::milliseconds reconnect_maxwait = 30s;
    /* How far the decoder may fall behind before frames start being dropped.
       This is what bounds steady-state latency once the decoder cannot keep
       up: the queue settles at the rung that matches the shortfall, so the
       number is a latency budget, not a failure threshold. */
    std::chrono::milliseconds max_latency = 200ms;
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
    frame_timer_.reset();
    notify_frame_ = {};
    if (auto waitables = node_waitables_.lock(); waitables && scheduled_cb_)
        waitables->remove_waitable(scheduled_cb_, scheduled_cb_group_);
    scheduled_cb_.reset();
    scheduled_cb_group_.reset();
    {
        std::lock_guard<std::mutex> lock{cooldown_mutex_};
        cooldown_cb_group_.reset();
        cooldown_timer_.reset();
    }
    /* Destroy, don't just disconnect: ~StreamClient stops the Live555 loop
       BEFORE tearing objects down, whereas a plain disconnect() from this
       executor thread raced the loop's in-flight frame delivery. */
    client_.reset();
    decoder_.reset();
    clearQueuedFrames();
    SuperClass::shutdown();
}

std::string SubscriberPlugin::getTransportName() const
{
    return "rtsp";
}

/* True when images should be stamped with the time they arrived rather than
   with the sender's clock. Re-evaluated per frame because use_sim_time can be
   switched at runtime. */
bool SubscriberPlugin::useReceiveTimestamps() const
{
    switch (config_->timestamp_source)
    {
        case TimestampFromSender:
            return false;
        case TimestampFromReceiver:
            return true;
        default:
            /* ros_time_is_active() throws when called on a system or steady
               clock in newer rclcpp releases. The node-interface plugin API
               has no clock interface and deliberately installs a system clock,
               so check its type before asking about a ROS-time override. */
            return clock_ && clock_->get_clock_type() == RCL_ROS_TIME && clock_->ros_time_is_active();
    }
}

#if RTSP_IMAGE_TRANSPORT_HAS_LEGACY_PLUGIN_API
void SubscriberPlugin::subscribeImpl(rclcpp::Node* node, const std::string& base_topic, const Callback& callback,
                                     rmw_qos_profile_t custom_qos, rclcpp::SubscriptionOptions options)
{
    /* The transport topic carries a latched URL rather than image data, so a
       late joining subscriber has to be able to fetch the current value. That
       fixes reliability, durability and history; every other policy the caller
       asked for is left untouched. */
    custom_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    custom_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    custom_qos.depth = 1;
    custom_qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    /* An incompatible publisher otherwise just means no video, with nothing in
       the log to explain it. */
    options.event_callbacks.incompatible_qos_callback =
        [this](rclcpp::QOSRequestedIncompatibleQoSInfo& info)
    {
        RCLCPP_ERROR(logger_,
                     "[%s] the RTSP URL publisher offers an incompatible QoS policy (%d); this transport needs "
                     "RELIABLE and TRANSIENT_LOCAL, so no video will arrive",
                     topic_name_.c_str(), static_cast<int>(info.last_policy_kind));
    };
    SuperClass::subscribeImpl(node, base_topic, callback, custom_qos, options);
    logger_ = node->get_logger();
    clock_ = node->get_clock();
    ScheduledCB::SharedPtr scheduled_cb =
        std::make_shared<ScheduledCB>(std::bind(&SubscriberPlugin::processFrame, this));
    scheduled_cb_ = scheduled_cb;
    /* Cached so the hot receive path does not need a dynamic_cast per NAL unit */
    notify_frame_ = [scheduled_cb]() { scheduled_cb->trigger(); };
    scheduled_cb_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    node_waitables_ = node->get_node_waitables_interface();
    node->get_node_waitables_interface()->add_waitable(scheduled_cb_, scheduled_cb_group_);
    topic_name_ = base_topic;
    node_base_ = rclcpp::node_interfaces::get_node_base_interface(node);
    node_timers_ = rclcpp::node_interfaces::get_node_timers_interface(node);
    node_param_ = rclcpp::node_interfaces::get_node_parameters_interface(node);
    failed_ = false;
    param_base_name_ = topicParameterBase(*node, base_topic, getTransportName());
    setupParameters(node_param_.lock());
    param_cb_handle_ = node_param_.lock()->add_post_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>&) { this->updateParameters(); });
    updateParameters();
}
#endif

#if RTSP_IMAGE_TRANSPORT_USES_NODE_INTERFACES
void SubscriberPlugin::subscribeImpl(image_transport::RequiredInterfaces node_interfaces,
                                     const std::string& base_topic, const Callback& callback,
                                     rclcpp::QoS custom_qos, rclcpp::SubscriptionOptions options)
{
    custom_qos.reliable().keep_last(1).transient_local();
    options.event_callbacks.incompatible_qos_callback =
        [this](rclcpp::QOSRequestedIncompatibleQoSInfo& info)
    {
        RCLCPP_ERROR(logger_,
                     "[%s] the RTSP URL publisher offers an incompatible QoS policy (%d); this transport needs "
                     "RELIABLE and TRANSIENT_LOCAL, so no video will arrive",
                     topic_name_.c_str(), static_cast<int>(info.last_policy_kind));
    };
    SuperClass::subscribeImpl(node_interfaces, base_topic, callback, custom_qos, options);

    auto node_base = node_interfaces.get_node_base_interface();
    auto node_timers = node_interfaces.get_node_timers_interface();
    auto node_parameters = node_interfaces.get_node_parameters_interface();
    logger_ = node_interfaces.get_node_logging_interface()->get_logger();
    clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    topic_name_ = base_topic;
    node_base_ = node_base;
    node_timers_ = node_timers;
    node_param_ = node_parameters;
    failed_ = false;

    param_base_name_ = topicParameterBase(node_base->get_namespace(), base_topic, getTransportName());

    setupParameters(node_parameters);
    param_cb_handle_ = node_parameters->add_post_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>&) { this->updateParameters(); });
    updateParameters();

    scheduled_cb_group_ = node_base->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    frame_timer_ = std::make_shared<rclcpp::WallTimer<rclcpp::VoidCallbackType>>(
        2ms, std::bind(&SubscriberPlugin::processFrame, this), node_base->get_context());
    node_timers->add_timer(frame_timer_, scheduled_cb_group_);
    RCLCPP_INFO(logger_, "[%s] using image_transport node-interface API with executor timer frame delivery",
                topic_name_.c_str());

    /* RequiredInterfaces carries no clock interface, so clock_ above is a plain
       system clock and automatic timestamp_source picks sender time. Nothing
       on this path can produce a simulated stamp at all. Say so rather than
       letting every published image carry a wall clock time in a simulated time
       base. */
    if (node_parameters->has_parameter("use_sim_time")
        && node_parameters->get_parameter("use_sim_time").get_value<bool>())
    {
        RCLCPP_ERROR(logger_,
                     "[%s] use_sim_time is set, but image_transport %d.%d gives this transport no clock "
                     "interface, so published images will be stamped from the wall clock and not from /clock. "
                     "Use image_transport 6.3 or earlier if the stamps have to follow simulated time",
                     topic_name_.c_str(), CURRENT_IMAGE_TRANSPORT_VERSION >> 16,
                     (CURRENT_IMAGE_TRANSPORT_VERSION >> 8) & 0xff);
    }
}
#endif

void SubscriberPlugin::internalCallback(const std_msgs::msg::String::ConstSharedPtr& msg, const Callback& callback)
{
    RCLCPP_DEBUG(logger_, "[%s] received updated RTSP URL: %s", topic_name_.c_str(), msg->data.c_str());
    failed_ = false;
    old_lag_ = 0s;
    callback_ = callback;
    {
        std::lock_guard<std::mutex> lock{cooldown_mutex_};
        cooldown_ = config_->reconnect_minwait;
        cooldown_cb_group_.reset();
        cooldown_timer_.reset();
    }
    client_.reset();
    try
    {
        client_ = StreamClient::create(topic_name_, msg->data, logger_);
        client_->setVideoSubsession(static_cast<std::size_t>(config_->video_subsession));
        client_->setRtpOverTcp(config_->rtp_over_tcp);
        client_->setRtpBufferSize(static_cast<unsigned>(config_->rtp_buffer_size));
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
        if (reconnectAfterFailure(config_->reconnect_policy))
        {
            reconnect();
        }
    }
}

void SubscriberPlugin::subsessionStarted(VideoCodec codec, MediaSubsession* subsession)
{
    old_lag_ = rclcpp::Duration(0, 0);
    RCLCPP_DEBUG(logger_, "[%s] setting up decoder for %s", topic_name_.c_str(), videoCodecName(codec).c_str());
    decoder_ = std::make_shared<StreamDecoder>(codec, config_->decoder, logger_);
    RCLCPP_INFO(logger_, "[%s] start decoding %s with %s from %s", topic_name_.c_str(),
                videoCodecName(codec).c_str(), decoder_->description().c_str(), client_->url().c_str());
    reportMissingHwDecoder();
}

/* Hardware decoding was asked for but we ended up in software. The device probe
   has already run at this point, so listing what the machine can do is free and
   turns a silent performance cliff into an actionable message. */
void SubscriberPlugin::reportMissingHwDecoder()
{
    if (!decoder_ || decoder_->isHardwareAccelerated())
        return;
    if (!config_->decoder.use_hw_decoder || config_->decoder.hw_device == "none")
        return;
    std::string available;
    for (const std::string& name : StreamDecoder::availableHwDevices(config_->decoder.hw_device_path))
    {
        if (!available.empty())
            available += ", ";
        available += name;
    }
    RCLCPP_INFO(logger_, "[%s] no hardware decoder available, decoding in software (hardware devices on this "
                         "machine: %s)",
                topic_name_.c_str(), available.empty() ? "none" : available.c_str());
}

void SubscriberPlugin::receiveDataStream(VideoCodec codec, MediaSubsession* subsession, const FrameDataPtr& data)
{
    RCLCPP_DEBUG_THROTTLE(logger_, *clock_, 30000, "[%s] receiving video frames from RTSP stream", topic_name_.c_str());
    if (useReceiveTimestamps())
    {
        /* The RTP presentation time is the sender's wall clock, which says
           nothing about the node's time base when that is simulated. */
        data->setStamp(clock_->now());
    }
    pushFrame(data);
    if (notify_frame_)
        notify_frame_();
}

void SubscriberPlugin::setupParameters(
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_parameters)
{
    using rcl_interfaces::msg::ParameterDescriptor;
    declareParameter(
        node_parameters, param_base_name_ + ".frame_id", rclcpp::ParameterValue(config_->frame_id),
        ParameterDescriptor().set__description(
            "frame_id assigned to decoded images (RTSP video carries no ROS header)"));
    declareParameter(
        node_parameters, param_base_name_ + ".use_hw_decoder",
        rclcpp::ParameterValue(config_->decoder.use_hw_decoder),
        ParameterDescriptor().set__description("use GPU accelerated video decoding if possible"));
    declareParameter(
        node_parameters, param_base_name_ + ".hw_device", rclcpp::ParameterValue(config_->decoder.hw_device),
        ParameterDescriptor().set__description(
            "hardware device for video decoding: auto, none, or a specific FFmpeg device "
            "(cuda, vaapi, qsv, vdpau, drm, vulkan, videotoolbox, d3d11va)"));
    declareParameter(
        node_parameters, param_base_name_ + ".hw_device_path",
        rclcpp::ParameterValue(config_->decoder.hw_device_path),
        ParameterDescriptor().set__description(
            "which GPU to decode on when the machine has several: a DRM render node such as "
            "/dev/dri/renderD128 for VAAPI and Quick Sync, or a device index for CUDA "
            "(empty = let FFmpeg choose)"));
    declareParameter(
        node_parameters, param_base_name_ + ".decoder", rclcpp::ParameterValue(config_->decoder.decoder),
        ParameterDescriptor().set__description(
            "force a specific FFmpeg decoder, e.g. hevc_cuvid or h264_qsv (empty = choose automatically)"));
    declareParameter(
        node_parameters, param_base_name_ + ".low_latency", rclcpp::ParameterValue(config_->decoder.low_latency),
        ParameterDescriptor().set__description("decode with minimal buffering; disable for streams with B-frames"));
    declareParameter(
        node_parameters, param_base_name_ + ".timestamp_source",
        rclcpp::ParameterValue(static_cast<int>(config_->timestamp_source)),
        ParameterDescriptor()
            .set__description("image time stamp source (0 = sender clock via RTCP, 1 = time of reception, "
                              "2 = automatic: sender clock unless simulated time is in use)")
            .set__integer_range({rcl_interfaces::msg::IntegerRange().set__from_value(0).set__to_value(2)}));
    declareParameter(
        node_parameters, param_base_name_ + ".video_subsession",
        rclcpp::ParameterValue(config_->video_subsession),
        ParameterDescriptor()
            .set__description("zero-based supported video subsession to decode")
            .set__integer_range({rcl_interfaces::msg::IntegerRange().set__from_value(0).set__to_value(31)}));
    declareParameter(
        node_parameters, param_base_name_ + ".rtp_over_tcp", rclcpp::ParameterValue(config_->rtp_over_tcp),
        ParameterDescriptor().set__description(
            "carry RTP interleaved over the RTSP TCP connection instead of separate UDP sockets; "
            "avoids the packet loss that shows up as green bands in the decoded image"));
    declareParameter(
        node_parameters, param_base_name_ + ".rtp_buffer_size",
        rclcpp::ParameterValue(config_->rtp_buffer_size),
        ParameterDescriptor()
            .set__description("receive buffer requested for the RTP socket [bytes], UDP only "
                              "(0 = keep the system default)")
            .set__integer_range({rcl_interfaces::msg::IntegerRange().set__from_value(0).set__to_value(64 << 20)}));
    declareParameter(
        node_parameters, param_base_name_ + ".drop_corrupt_frames",
        rclcpp::ParameterValue(config_->decoder.drop_corrupt_frames),
        ParameterDescriptor().set__description(
            "drop frames the decoder could not fully reconstruct instead of publishing the concealed "
            "result; trades artefacts for stutter on a lossy stream"));
    declareParameter(
        node_parameters, param_base_name_ + ".sws_threads",
        rclcpp::ParameterValue(config_->decoder.sws_threads),
        ParameterDescriptor()
            .set__description("worker threads for the YUV to BGR conversion (0 = one per hardware thread); "
                              "the default shares the CPU with the rest of the vehicle rather than taking "
                              "everything the machine has")
            .set__integer_range({rcl_interfaces::msg::IntegerRange().set__from_value(0).set__to_value(64)}));
    declareParameter(
        node_parameters, param_base_name_ + ".max_latency",
        rclcpp::ParameterValue(1e-3 * config_->max_latency.count()),
        ParameterDescriptor()
            .set__description("how far the decoder may fall behind before frames are dropped [s]; this is the "
                              "latency budget the queue settles at when the decoder cannot keep up. Non-intra "
                              "frames go first, non-key frames at twice this, everything at four times "
                              "(0 = never drop, and let latency grow instead)")
            .set__floating_point_range(
                {rcl_interfaces::msg::FloatingPointRange().set__from_value(0).set__to_value(10)}));
    declareParameter(
        node_parameters, param_base_name_ + ".reconnect_policy",
        rclcpp::ParameterValue(static_cast<int>(config_->reconnect_policy)),
        ParameterDescriptor()
            .set__description("client reconnect policy (0 = never, 1 = on timeout, 2 = on failure, 3 = always)")
            .set__integer_range({rcl_interfaces::msg::IntegerRange().set__from_value(0).set__to_value(3)}));
    declareParameter(
        node_parameters, param_base_name_ + ".timeout", rclcpp::ParameterValue(1e-3 * config_->timeout.count()),
        ParameterDescriptor()
            .set__description("client session timeout [s] (0 = unlimited)")
            .set__floating_point_range(
                {rcl_interfaces::msg::FloatingPointRange().set__from_value(0).set__to_value(60)}));
    declareParameter(
        node_parameters, param_base_name_ + ".reconnect_minwait",
        rclcpp::ParameterValue(1e-3 * config_->reconnect_minwait.count()),
        ParameterDescriptor()
            .set__description("minimum delay between connection attempts [s]")
            .set__floating_point_range(
                {rcl_interfaces::msg::FloatingPointRange().set__from_value(0).set__to_value(60)}));
    declareParameter(
        node_parameters, param_base_name_ + ".reconnect_maxwait",
        rclcpp::ParameterValue(1e-3 * config_->reconnect_maxwait.count()),
        ParameterDescriptor()
            .set__description("maximum delay between connection attempts [s]")
            .set__floating_point_range(
                {rcl_interfaces::msg::FloatingPointRange().set__from_value(0).set__to_value(600)}));
}

void SubscriberPlugin::updateParameters()
{
    static constexpr int LVL_CODEC = 1;
    static constexpr int LVL_CONNECTION = 2;

    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr np = node_param_.lock();
    if (!np)
        return;
    Config new_config;
    new_config.frame_id = np->get_parameter(param_base_name_ + ".frame_id").as_string();
    new_config.decoder.use_hw_decoder = np->get_parameter(param_base_name_ + ".use_hw_decoder").as_bool();
    new_config.decoder.hw_device = np->get_parameter(param_base_name_ + ".hw_device").as_string();
    new_config.decoder.hw_device_path = np->get_parameter(param_base_name_ + ".hw_device_path").as_string();
    new_config.decoder.decoder = np->get_parameter(param_base_name_ + ".decoder").as_string();
    new_config.decoder.low_latency = np->get_parameter(param_base_name_ + ".low_latency").as_bool();
    new_config.timestamp_source =
        static_cast<TimestampSource>(np->get_parameter(param_base_name_ + ".timestamp_source").as_int());
    new_config.video_subsession =
        static_cast<int>(np->get_parameter(param_base_name_ + ".video_subsession").as_int());
    new_config.rtp_over_tcp = np->get_parameter(param_base_name_ + ".rtp_over_tcp").as_bool();
    new_config.rtp_buffer_size =
        static_cast<int>(np->get_parameter(param_base_name_ + ".rtp_buffer_size").as_int());
    new_config.decoder.drop_corrupt_frames =
        np->get_parameter(param_base_name_ + ".drop_corrupt_frames").as_bool();
    new_config.decoder.sws_threads =
        static_cast<int>(np->get_parameter(param_base_name_ + ".sws_threads").as_int());
    new_config.max_latency = std::chrono::milliseconds(static_cast<std::chrono::milliseconds::rep>(
        1000 * np->get_parameter(param_base_name_ + ".max_latency").as_double()));
    new_config.reconnect_policy =
        static_cast<ReconnectPolicy>(np->get_parameter(param_base_name_ + ".reconnect_policy").as_int());
    new_config.timeout = std::chrono::milliseconds(static_cast<std::chrono::milliseconds::rep>(
        1000 * np->get_parameter(param_base_name_ + ".timeout").as_double()));
    new_config.reconnect_minwait = std::chrono::milliseconds(static_cast<std::chrono::milliseconds::rep>(
        1000 * np->get_parameter(param_base_name_ + ".reconnect_minwait").as_double()));
    new_config.reconnect_maxwait = std::chrono::milliseconds(static_cast<std::chrono::milliseconds::rep>(
        1000 * np->get_parameter(param_base_name_ + ".reconnect_maxwait").as_double()));

    int changelevel = 0;
    if (new_config.decoder.use_hw_decoder != config_->decoder.use_hw_decoder
        || new_config.decoder.low_latency != config_->decoder.low_latency
        || new_config.decoder.hw_device != config_->decoder.hw_device
        || new_config.decoder.hw_device_path != config_->decoder.hw_device_path
        || new_config.decoder.decoder != config_->decoder.decoder
        || new_config.decoder.drop_corrupt_frames != config_->decoder.drop_corrupt_frames)
        changelevel |= LVL_CODEC;
    /* The transport is chosen in the SETUP request and the RTP socket is sized
       right before it, so either change needs a new session. */
    if (new_config.video_subsession != config_->video_subsession
        || new_config.rtp_over_tcp != config_->rtp_over_tcp
        || new_config.rtp_buffer_size != config_->rtp_buffer_size)
        changelevel |= LVL_CONNECTION;

    *config_ = new_config;
    try
    {
        {
            std::lock_guard<std::mutex> lock{cooldown_mutex_};
            cooldown_ = config_->reconnect_minwait;
        }
        if (config_->reconnect_maxwait < config_->reconnect_minwait)
            config_->reconnect_maxwait = config_->reconnect_minwait;
        if (client_)
        {
            client_->setSessionTimeout(config_->timeout);
            if (changelevel & LVL_CONNECTION)
            {
                reconnect();
                return;
            }
            if ((changelevel & LVL_CODEC) && decoder_)
            {
                decoder_ = std::make_shared<StreamDecoder>(client_->codec(), config_->decoder, logger_);
                RCLCPP_INFO(logger_, "[%s] start decoding %s with %s from %s", topic_name_.c_str(),
                            videoCodecName(client_->codec()).c_str(), decoder_->description().c_str(),
                            client_->url().c_str());
                reportMissingHwDecoder();
            }
        }
    }
    catch (std::exception& e)
    {
        RCLCPP_ERROR(logger_, "[%s] %s", topic_name_.c_str(), e.what());
        failed_ = true;
        clearQueuedFrames();
        if (reconnectAfterFailure(config_->reconnect_policy))
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
        /* The rungs of the drop ladder, derived from the latency budget. The
           queue settles on whichever one matches how far the decoder is
           behind, so these thresholds are what steady-state latency converges
           to under sustained overload — not a rarely-reached safety net.
           max_latency of zero switches dropping off entirely. */
        const bool may_drop = config_->max_latency > 0ms;
        const rclcpp::Duration drop_non_intra{config_->max_latency};
        const rclcpp::Duration drop_non_key{2 * config_->max_latency};
        const rclcpp::Duration drop_all{4 * config_->max_latency};
        while (FrameDataPtr frame = popFrame())
        {
            rclcpp::Duration lag = frameLag();
            if (may_drop && lag >= drop_all)
            {
                if (old_lag_ < drop_all)
                {
                    RCLCPP_WARN(logger_, "[%s] decoder is %.0f ms behind; discarding all frames",
                                topic_name_.c_str(), 1e-6 * lag.nanoseconds());
                    old_lag_ = lag;
                }
                decoder->setDecodeFrames(StreamDecoder::DecodeFrames::None);
            }
            else if (may_drop && lag >= drop_non_key)
            {
                if (old_lag_ < drop_non_key)
                {
                    RCLCPP_WARN(logger_, "[%s] decoder is %.0f ms behind; discarding non-key frames",
                                topic_name_.c_str(), 1e-6 * lag.nanoseconds());
                    old_lag_ = lag;
                }
                decoder->setDecodeFrames(StreamDecoder::DecodeFrames::Key);
            }
            else if (may_drop && lag >= drop_non_intra)
            {
                if (old_lag_ < drop_non_intra)
                {
                    RCLCPP_WARN(logger_, "[%s] decoder is %.0f ms behind; discarding non-intra frames",
                                topic_name_.c_str(), 1e-6 * lag.nanoseconds());
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
                while (sensor_msgs::msg::Image::UniquePtr decoded = decoder->nextFrame())
                {
                    decoded->header.frame_id = config_->frame_id;
                    sensor_msgs::msg::Image::ConstSharedPtr img(std::move(decoded));
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
        if (reconnectAfterFailure(config_->reconnect_policy))
        {
            reconnect();
        }
    }
}

void SubscriberPlugin::sessionStarted()
{
    std::lock_guard<std::mutex> lock{cooldown_mutex_};
    cooldown_ = config_->reconnect_minwait;
    cooldown_timer_.reset();
}

void SubscriberPlugin::sessionTimeout()
{
    RCLCPP_ERROR(logger_, "[%s] session timeout for stream at %s", topic_name_.c_str(),
                 client_ ? client_->url().c_str() : "(unknown)");
    if (reconnectAfterTimeout(config_->reconnect_policy))
    {
        reconnect();
    }
}

void SubscriberPlugin::sessionFailed(int code, const std::string& message)
{
    RCLCPP_ERROR(logger_, "[%s] %s failed. %s (%d)", topic_name_.c_str(),
                 client_ ? client_->url().c_str() : "(unknown)", message.c_str(), code);
    if (reconnectAfterFailure(config_->reconnect_policy))
    {
        reconnect();
    }
}

void SubscriberPlugin::sessionFinished()
{
    RCLCPP_INFO(logger_, "[%s] end of video stream", topic_name_.c_str());
    if (reconnectAfterNormalEnd(config_->reconnect_policy))
    {
        reconnect();
    }
}

void SubscriberPlugin::reconnect()
{
    if (!client_)
        return;
    client_->disconnect();
    client_->setVideoSubsession(static_cast<std::size_t>(config_->video_subsession));
    client_->setRtpOverTcp(config_->rtp_over_tcp);
    client_->setRtpBufferSize(static_cast<unsigned>(config_->rtp_buffer_size));
    clearQueuedFrames();
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr nb = node_base_.lock();
    rclcpp::node_interfaces::NodeTimersInterface::SharedPtr nt = node_timers_.lock();
    if (nb && nt)
    {
        /* Runs on ROS executor threads AND the Live555 handler thread (via
           the session failed/timeout handlers) — the cooldown state needs
           the lock. */
        std::lock_guard<std::mutex> lock{cooldown_mutex_};
        RCLCPP_INFO(logger_, "[%s] new connection attempt in %0.3lf seconds", topic_name_.c_str(),
                    1e-3 * cooldown_.count());
        cooldown_cb_group_ = nb->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cooldown_timer_ = std::make_shared<rclcpp::WallTimer<rclcpp::VoidCallbackType>>(
            cooldown_, std::bind(&SubscriberPlugin::cooldownTimerCallback, this), nb->get_context());
        nt->add_timer(cooldown_timer_, cooldown_cb_group_);
        cooldown_ = nextReconnectCooldown(cooldown_, config_->reconnect_maxwait);
    }
}

void SubscriberPlugin::cooldownTimerCallback()
{
    {
        std::lock_guard<std::mutex> lock{cooldown_mutex_};
        cooldown_timer_.reset();  // just in case
    }
    clearQueuedFrames();
    if (!client_)
        return;
    try
    {
        client_->connect();
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(logger_, "[%s] %s", topic_name_.c_str(), e.what());
        if (reconnectAfterFailure(config_->reconnect_policy))
            reconnect();
    }
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
