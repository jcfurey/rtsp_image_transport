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
#include "publisher_plugin.h"

#include "init.h"
#include "stream_encoder.h"
#include "stream_server.h"
#include "topic_parameter.h"
#include "video_codec.h"

#include <rclcpp/clock.hpp>

namespace rtsp_image_transport
{

/* From image_transport 6.4 the plugin entry points take node interfaces rather
   than a node, and the old ones are never called. RequiredInterfaces has every
   interface needed to publish the URL, configure the encoder, and serve RTSP;
   it only lacks the graph interface used by an optional legacy optimization.
   The newer path therefore lets publish() detect departed RTSP clients through
   StreamServer::hasActiveStreams(). See RTSP_IMAGE_TRANSPORT_* in init.h. */

namespace
{

void declareParameter(
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& parameters, const std::string& name,
    const rclcpp::ParameterValue& value, const rcl_interfaces::msg::ParameterDescriptor& descriptor)
{
    if (!parameters->has_parameter(name))
        parameters->declare_parameter(name, value, descriptor);
}

template<typename Number>
std::string withSI(Number value)
{
    if (value < 10000)
        return std::format("{} ", value);
    if (value < 10000000)
        return std::format("{} k", value / 1000);
    return std::format("{} M", value / 1000000);
}

}  // namespace

struct RTSP_IMAGE_TRANSPORT_NO_EXPORT PublisherPlugin::Config
{
    VideoCodec codec = VideoCodec::Unknown;
    unsigned target_bitrate = 1000000;
    unsigned expected_framerate = 30;
    bool use_hw_encoder = true;
    unsigned udp_port = 0;
    unsigned udp_packet_size = 1396;
    bool use_ip_multicast = false;
};

const std::map<std::string, VideoCodec> CODEC_NAMES = {
    {"H264", VideoCodec::H264},   {"AVC", VideoCodec::H264}, {"H265", VideoCodec::H265}, {"HEVC", VideoCodec::H265},
    {"MPEG4", VideoCodec::MPEG4}, {"VP8", VideoCodec::VP8},  {"VP9", VideoCodec::VP9},   {"AV1", VideoCodec::AV1}};

using SuperClass = image_transport::SimplePublisherPlugin<std_msgs::msg::String>;

PublisherPlugin::PublisherPlugin()
    : SuperClass(), logger_(rclcpp::get_logger("rtsp_image_transport")), config_(std::make_unique<Config>()),
      system_clock_(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)),
      steady_clock_(std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME)), update_url_(false), failed_(false)
{
    global_initialize();
}

void PublisherPlugin::shutdown()
{
    demand_timer_.reset();
    demand_cb_group_.reset();
    {
        /* Wait out any callback already handed to the republisher, then make
           later timer or middleware callbacks harmless before destroying the
           underlying publisher. */
        std::lock_guard<std::mutex> callback_lock{demand_callback_mutex_};
        std::lock_guard<std::mutex> state_lock{demand_state_mutex_};
        source_demand_callback_ = {};
        ros_subscriber_count_ = 0;
        rtsp_client_active_ = false;
    }
    SuperClass::shutdown();
    if (graph_monitor_)
        graph_monitor_->removeListener(this);
    graph_monitor_.reset();
    server_.reset();
    encoder_.reset();
}

std::string PublisherPlugin::getTransportName() const
{
    return "rtsp";
}

#if RTSP_IMAGE_TRANSPORT_HAS_LEGACY_PLUGIN_API
void PublisherPlugin::advertiseImpl(rclcpp::Node* node, const std::string& base_topic, rmw_qos_profile_t custom_qos,
                                    rclcpp::PublisherOptions options)
{
    logger_ = node->get_logger();
    topic_name_ = base_topic;
    /* The transport topic carries a latched URL rather than image data, so a
       late joining subscriber has to be able to fetch the current value. That
       fixes reliability, durability and history; every other policy the caller
       asked for is left untouched. */
    custom_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    custom_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    custom_qos.depth = 1;
    custom_qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    /* An incompatible subscriber otherwise just never receives the URL, with
       nothing in the log to explain it. */
    options.event_callbacks.incompatible_qos_callback =
        [this](rclcpp::QOSOfferedIncompatibleQoSInfo& info)
    {
        RCLCPP_ERROR(logger_,
                     "[%s] a subscriber requests an incompatible QoS policy (%d); this transport publishes the "
                     "stream URL as RELIABLE and TRANSIENT_LOCAL, so that subscriber will not receive it",
                     topic_name_.c_str(), static_cast<int>(info.last_policy_kind));
    };
    setupMatchedCallback(options);
    SuperClass::advertiseImpl(node, base_topic, custom_qos, options);
    graph_monitor_ = GraphMonitor::instance(node, this);
    node_param_ = rclcpp::node_interfaces::get_node_parameters_interface(node);
    param_base_name_ = topicParameterBase(*node, base_topic, getTransportName());
    setupParameters(node_param_.lock());
    param_cb_handle_ = node->add_post_set_parameters_callback([this](const std::vector<rclcpp::Parameter>&)
                                                              { this->updateParameters(); });
    updateParameters();
    setupDemandMonitor(*node);
}
#endif

#if RTSP_IMAGE_TRANSPORT_USES_NODE_INTERFACES
void PublisherPlugin::advertiseImpl(image_transport::RequiredInterfaces node_interfaces,
                                    const std::string& base_topic, rclcpp::QoS custom_qos,
                                    rclcpp::PublisherOptions options)
{
    logger_ = node_interfaces.get_node_logging_interface()->get_logger();
    topic_name_ = base_topic;
    /* Match the legacy path above. The transport topic carries one latched URL,
       so these three QoS policies are part of the wire protocol. */
    custom_qos.reliable().keep_last(1).transient_local();
    options.event_callbacks.incompatible_qos_callback =
        [this](rclcpp::QOSOfferedIncompatibleQoSInfo& info)
    {
        RCLCPP_ERROR(logger_,
                     "[%s] a subscriber requests an incompatible QoS policy (%d); this transport publishes the "
                     "stream URL as RELIABLE and TRANSIENT_LOCAL, so that subscriber will not receive it",
                     topic_name_.c_str(), static_cast<int>(info.last_policy_kind));
    };
    setupMatchedCallback(options);
    SuperClass::advertiseImpl(node_interfaces, base_topic, custom_qos, options);

    auto node_base = node_interfaces.get_node_base_interface();
    auto node_parameters = node_interfaces.get_node_parameters_interface();
    node_param_ = node_parameters;
    param_base_name_ = topicParameterBase(node_base->get_namespace(), base_topic, getTransportName());
    setupParameters(node_parameters);
    param_cb_handle_ = node_parameters->add_post_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>&) { this->updateParameters(); });
    updateParameters();
    setupDemandMonitor(DemandInterfaces(node_interfaces));

    RCLCPP_INFO(logger_,
                "[%s] using image_transport node-interface API; RTSP client state will control encoder shutdown",
                topic_name_.c_str());
}
#endif

/* image_transport's republisher is lazy: its matched callback subscribes to
   the input only while the selected output transport has a ROS subscriber. A
   native RTSP client is not a ROS graph endpoint, so a direct
   ffplay/ffprobe/VLC connection used to leave the republisher unsubscribed from
   the source image topic forever.

   Preserve that callback and give it a virtual match while Live555 has an
   active unicast stream. Real ROS matches are folded into the same count, so a
   ROS subscriber disconnecting cannot stop input needed by an RTSP client (or
   vice versa). This only influences upstream demand; it does not loop URL
   messages or copy image data. The encoder remains independently client-lazy. */
void PublisherPlugin::setupMatchedCallback(rclcpp::PublisherOptions& options)
{
    {
        std::lock_guard<std::mutex> lock{demand_state_mutex_};
        source_demand_callback_ = std::move(options.event_callbacks.matched_callback);
        ros_subscriber_count_ = 0;
        rtsp_client_active_ = false;
    }
    if (source_demand_callback_)
        options.event_callbacks.matched_callback =
            [this](rclcpp::MatchedInfo& info) { this->onPublisherMatched(info); };
}

void PublisherPlugin::setupDemandMonitor(DemandInterfaces node_interfaces)
{
    /* Normal camera publishers do not install a matched callback and already
       receive images continuously, so they need neither this timer nor its
       100 Hz mutex check. image_transport republish does install one. */
    if (!source_demand_callback_)
        return;
    auto node_base = node_interfaces.get_node_base_interface();
    auto node_timers = node_interfaces.get_node_timers_interface();
    demand_cb_group_ = node_base->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    demand_timer_ = std::make_shared<rclcpp::WallTimer<rclcpp::VoidCallbackType>>(
        std::chrono::milliseconds(10), std::bind(&PublisherPlugin::updateDemand, this),
        node_base->get_context());
    node_timers->add_timer(demand_timer_, demand_cb_group_);
}

void PublisherPlugin::onPublisherMatched(rclcpp::MatchedInfo& info)
{
    {
        std::lock_guard<std::mutex> lock{demand_state_mutex_};
        ros_subscriber_count_ = info.current_count;
    }
    forwardDemand(info);
}

void PublisherPlugin::forwardDemand(rclcpp::MatchedInfo info)
{
    /* Serialize delivery and read the latest state after taking the delivery
       lock. Concurrent RTSP and ROS disconnects therefore cannot deliver a
       stale non-zero count after the final zero. */
    std::lock_guard<std::mutex> callback_lock{demand_callback_mutex_};
    rclcpp::PublisherMatchedCallbackType callback;
    {
        std::lock_guard<std::mutex> state_lock{demand_state_mutex_};
        callback = source_demand_callback_;
        info.current_count = ros_subscriber_count_ + (rtsp_client_active_ ? 1 : 0);
    }
    if (callback)
        callback(info);
}

void PublisherPlugin::updateDemand()
{
    bool rtsp_active = false;
    {
        std::lock_guard<std::mutex> lock{mutex_};
        rtsp_active = server_ && !failed_ && server_->hasActiveStreams();
    }
    const std::size_t ros_subscribers = getNumSubscribers();

    std::size_t old_combined = 0, new_combined = 0;
    {
        std::lock_guard<std::mutex> lock{demand_state_mutex_};
        if (rtsp_active == rtsp_client_active_ && ros_subscribers == ros_subscriber_count_)
            return;
        old_combined = ros_subscriber_count_ + (rtsp_client_active_ ? 1 : 0);
        rtsp_client_active_ = rtsp_active;
        ros_subscriber_count_ = ros_subscribers;
        new_combined = ros_subscriber_count_ + (rtsp_client_active_ ? 1 : 0);
    }

    rclcpp::MatchedInfo info{};
    info.current_count_change = static_cast<std::int32_t>(new_combined) - static_cast<std::int32_t>(old_combined);
    info.total_count_change = info.current_count_change > 0 ? info.current_count_change : 0;
    forwardDemand(info);

    if (rtsp_active)
    {
        RCLCPP_DEBUG(logger_, "[%s] RTSP client connected; requesting source images", topic_name_.c_str());
    }
    else
    {
        std::lock_guard<std::mutex> lock{mutex_};
        if (encoder_ && (!server_ || !server_->hasActiveStreams()))
        {
            RCLCPP_INFO(logger_, "[%s] stop encoding for disconnected RTSP clients", topic_name_.c_str());
            encoder_.reset();
        }
        RCLCPP_DEBUG(logger_, "[%s] last RTSP client disconnected; releasing source images", topic_name_.c_str());
    }
}

void PublisherPlugin::setupParameters(
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_parameters)
{
    using rcl_interfaces::msg::ParameterDescriptor;
    declareParameter(node_parameters, param_base_name_ + ".codec", rclcpp::ParameterValue("H264"),
                     ParameterDescriptor().set__description("video encoding format"));
    declareParameter(
        node_parameters, param_base_name_ + ".target_bitrate",
        rclcpp::ParameterValue(static_cast<int>(config_->target_bitrate)),
        ParameterDescriptor()
            .set__description("targeted encoding bitrate [bits/s]")
            .set__integer_range({rcl_interfaces::msg::IntegerRange().set__from_value(0).set__to_value(100000000)}));
    declareParameter(
        node_parameters, param_base_name_ + ".expected_framerate",
        rclcpp::ParameterValue(static_cast<int>(config_->expected_framerate)),
        ParameterDescriptor()
            .set__description("expected video frame rate [frames/s]")
            .set__integer_range({rcl_interfaces::msg::IntegerRange().set__from_value(1).set__to_value(100)}));
    declareParameter(
        node_parameters, param_base_name_ + ".use_hw_encoder", rclcpp::ParameterValue(config_->use_hw_encoder),
        ParameterDescriptor().set__description("use NVENC or VAAPI hardware acceleration if possible"));
    declareParameter(
        node_parameters, param_base_name_ + ".udp_port", rclcpp::ParameterValue(static_cast<int>(config_->udp_port)),
        ParameterDescriptor()
            .set__description("force UDP port for RTSP server (0 = auto select)")
            .set__integer_range({rcl_interfaces::msg::IntegerRange().set__from_value(0).set__to_value(65535)}));
    declareParameter(
        node_parameters, param_base_name_ + ".udp_packet_size",
        rclcpp::ParameterValue(static_cast<int>(config_->udp_packet_size)),
        ParameterDescriptor()
            .set__description("size limit for UDP packets [octets]")
            .set__integer_range({rcl_interfaces::msg::IntegerRange().set__from_value(576).set__to_value(9000)}));
    declareParameter(
        node_parameters, param_base_name_ + ".use_ip_multicast",
        rclcpp::ParameterValue(config_->use_ip_multicast),
        ParameterDescriptor().set__description("use IP multicast for RTP stream"));
}

void PublisherPlugin::updateParameters()
{
    constexpr int LVL_SERVER = 4;
    constexpr int LVL_SESSION = 2;
    constexpr int LVL_CODEC = 1;

    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr np = node_param_.lock();
    if (!np)
        return;
    std::lock_guard<std::mutex> lock{mutex_};
    Config new_config;
    std::string codec_str = np->get_parameter(param_base_name_ + ".codec").as_string();
    std::string codec_str_canon;
    for (char ch : codec_str)
    {
        if (ch >= 'a' && ch <= 'z')
            ch -= 32;
        if (ch >= 'A' && ch <= 'Z')
            codec_str_canon.push_back(ch);
        if (ch >= '0' && ch <= '9')
            codec_str_canon.push_back(ch);
    }
    auto codec_iter = CODEC_NAMES.find(codec_str_canon);
    new_config.codec = codec_iter != CODEC_NAMES.end() ? codec_iter->second : VideoCodec::Unknown;
    if (new_config.codec == VideoCodec::Unknown)
    {
        std::string known;
        for (const auto& entry : CODEC_NAMES)
        {
            if (!known.empty())
                known += ", ";
            known += entry.first;
        }
        RCLCPP_ERROR(logger_, "[%s] unknown codec \"%s\"; supported values are: %s", topic_name_.c_str(),
                     codec_str.c_str(), known.c_str());
    }
    new_config.target_bitrate = np->get_parameter(param_base_name_ + ".target_bitrate").as_int();
    new_config.expected_framerate = np->get_parameter(param_base_name_ + ".expected_framerate").as_int();
    new_config.use_hw_encoder = np->get_parameter(param_base_name_ + ".use_hw_encoder").as_bool();
    new_config.udp_port = np->get_parameter(param_base_name_ + ".udp_port").as_int();
    new_config.udp_packet_size = np->get_parameter(param_base_name_ + ".udp_packet_size").as_int();
    new_config.use_ip_multicast = np->get_parameter(param_base_name_ + ".use_ip_multicast").as_bool();

    int changelevel = 0;
    /* A previous setup failure is retried on every parameter update, so a
       transient problem (e.g. a UDP port that was still in use) does not
       disable the publisher until the node is restarted. */
    if (!server_ || failed_ || config_->udp_port != new_config.udp_port
        || config_->udp_packet_size != new_config.udp_packet_size)
        changelevel |= LVL_SERVER;
    if (config_->codec != new_config.codec || config_->use_ip_multicast != new_config.use_ip_multicast)
        changelevel |= LVL_SESSION;
    if (config_->target_bitrate != new_config.target_bitrate
        || config_->expected_framerate != new_config.expected_framerate
        || config_->use_hw_encoder != new_config.use_hw_encoder)
        changelevel |= LVL_CODEC;

    *config_ = new_config;
    failed_ = false;
    try
    {
        if (changelevel >= LVL_SERVER)
        {
            encoder_.reset();
            server_.reset();
            server_ = StreamServer::create(topic_name_, config_->udp_port, config_->udp_packet_size - 42, logger_);
        }
        if (server_ && changelevel >= LVL_SESSION)
        {
            if (changelevel < LVL_SERVER)
                server_->stop();
            server_->start(config_->codec, config_->use_ip_multicast);
            update_url_ = true;
        }
        if (changelevel >= LVL_CODEC)
            encoder_.reset();
    }
    catch (std::exception& e)
    {
        server_.reset();
        encoder_.reset();
        update_url_ = false;
        failed_ = true;
        RCLCPP_ERROR(logger_, "[%s] %s", topic_name_.c_str(), e.what());
    }
}

void PublisherPlugin::publish(const sensor_msgs::msg::Image& image, const PublisherT& publisher) const
{
    std::lock_guard<std::mutex> lock{mutex_};
    try
    {
        if (!server_ || failed_)
            return;
        if (update_url_)
        {
            std_msgs::msg::String::UniquePtr url = std::make_unique<std_msgs::msg::String>();
            url->data = server_->url();
            publisher->publish(std::move(url));
            update_url_ = false;
        }
        if (!server_->hasActiveStreams())
        {
            // We need to call publish_fn at least once more after
            // the last client has disconnected, or the ROS topic networking
            // code may not notice that all clients have vanished.
            // As a beneficial side effect, if the RTP streams have timed out
            // for a different reason, the new URL message will cause all
            // remaining active clients to reconnect to the RTSP server.
            //
            // Note that this block will never execute if the RTSP server
            // runs in IP multicast mode, as the corresponding media session
            // will always be active and transmitting.
            if (encoder_)
            {
                RCLCPP_INFO(logger_, "[%s] stop encoding for %s", topic_name_.c_str(), server_->url().c_str());
                encoder_.reset();
                std_msgs::msg::String::UniquePtr url = std::make_unique<std_msgs::msg::String>();
                url->data = server_->url();
                publisher->publish(std::move(url));
            }
            return;
        }
        if (!encoder_)
        {
            encoder_ = std::make_unique<StreamEncoder>(config_->codec, config_->use_hw_encoder, logger_);
            stream_clock_.reset();
            encoder_->setBitrate(config_->target_bitrate);
            encoder_->setFramerate(config_->expected_framerate);
            encoder_->setPackageSizeHint(server_->maxPacketSize() - 24);
            RCLCPP_INFO(logger_, "[%s] start encoding (%s; %sbit/s; %u fps) for %s", topic_name_.c_str(),
                        encoder_->context()->codec->name, withSI(config_->target_bitrate).c_str(),
                        config_->expected_framerate, server_->url().c_str());
        }
        if (image.header.stamp.sec == 0 && image.header.stamp.nanosec == 0)
            RCLCPP_WARN_THROTTLE(logger_, *steady_clock_, 10000,
                                 "[%s] image header time stamp is not set, expect broken RTSP stream",
                                 topic_name_.c_str());
        if (encoder_->encodeVideo(image) > 0)
        {
            /* RTP and RTCP need wall clock presentation times. ROS time is not
               that: under simulation it starts near zero, and a looping bag
               sends it backwards. */
            const rclcpp::Time wall_now = system_clock_->now();
            const std::uint64_t reanchors_before = stream_clock_.reanchorCount();
            while (FrameDataPtr data = encoder_->nextPacket())
            {
                data->setStamp(stream_clock_.toWallClock(data->stamp(), wall_now));
                server_->sendFrame(data);
            }
            if (stream_clock_.reanchorCount() != reanchors_before)
                RCLCPP_INFO(logger_, "[%s] image time stamps jumped, re-anchoring the RTP timeline",
                            topic_name_.c_str());
        }
    }
    catch (const std::exception& e)
    {
        encoder_.reset();
        failed_ = true;
        RCLCPP_ERROR(logger_, "[%s] %s", topic_name_.c_str(), e.what());
    }
}

void PublisherPlugin::onGraphChange()
{
    std::lock_guard<std::mutex> lock{mutex_};
    if (getNumSubscribers() == 0 && encoder_ && server_ && !server_->hasActiveStreams())
    {
        RCLCPP_INFO(logger_, "[%s] stop encoding for %s", topic_name_.c_str(), server_->url().c_str());
        encoder_.reset();
    }
}

}  // namespace rtsp_image_transport

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rtsp_image_transport::PublisherPlugin, image_transport::PublisherPlugin)
