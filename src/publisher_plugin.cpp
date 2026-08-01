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
#include "video_codec.h"

#include <rclcpp/clock.hpp>

namespace rtsp_image_transport
{

/* From image_transport 6.4 the plugin entry points take node interfaces rather
   than a node, and the old ones are never called. This transport cannot be
   ported to them yet: RequiredInterfaces carries neither the clock interface
   the subscriber needs to honour simulated time, nor the waitables interface it
   uses to hand decoding to the executor, nor the graph interface the publisher
   watches for departing subscribers. Overriding the new entry point at least
   turns a silent black screen into something the log explains. */
#define RTSP_IMAGE_TRANSPORT_USES_NODE_INTERFACES \
    (CURRENT_IMAGE_TRANSPORT_VERSION >= FKIE_VERSION_TUPLE(6, 4, 0))


namespace
{

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

void PublisherPlugin::advertiseImpl(rclcpp::Node* node, const std::string& base_topic, rmw_qos_profile_t custom_qos,
                                    rclcpp::PublisherOptions options)
{
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
    SuperClass::advertiseImpl(node, base_topic, custom_qos, options);
    graph_monitor_ = GraphMonitor::instance(node, this);
    logger_ = node->get_logger();
    topic_name_ = base_topic;
    node_param_ = rclcpp::node_interfaces::get_node_parameters_interface(node);
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

#if RTSP_IMAGE_TRANSPORT_USES_NODE_INTERFACES
void PublisherPlugin::advertiseImpl(image_transport::RequiredInterfaces node_interfaces,
                                    const std::string& base_topic, rclcpp::QoS custom_qos,
                                    rclcpp::PublisherOptions options)
{
    rclcpp::Logger logger = node_interfaces.get_node_logging_interface()->get_logger();
    RCLCPP_ERROR(logger,
                 "[%s] the rtsp transport is not supported with image_transport %d.%d: its plugin API no longer "
                 "provides the clock, waitables and graph interfaces this transport needs. The topic will exist "
                 "but no video will be served. Use image_transport 6.3 or earlier, or the raw transport.",
                 base_topic.c_str(), CURRENT_IMAGE_TRANSPORT_VERSION >> 16,
                 (CURRENT_IMAGE_TRANSPORT_VERSION >> 8) & 0xff);
    SuperClass::advertiseImpl(node_interfaces, base_topic, custom_qos, options);
}
#endif

void PublisherPlugin::setupParameters(rclcpp::Node* node)
{
    using rcl_interfaces::msg::ParameterDescriptor;
    if (!node->has_parameter(param_base_name_ + ".codec"))
        node->declare_parameter<std::string>(param_base_name_ + ".codec", "H264",
                                             ParameterDescriptor().set__description("video encoding format"));
    if (!node->has_parameter(param_base_name_ + ".target_bitrate"))
        node->declare_parameter<int>(
            param_base_name_ + ".target_bitrate", config_->target_bitrate,
            ParameterDescriptor()
                .set__description("targeted encoding bitrate [bits/s]")
                .set__integer_range({rcl_interfaces::msg::IntegerRange().set__from_value(0).set__to_value(100000000)}));
    if (!node->has_parameter(param_base_name_ + ".expected_framerate"))
        node->declare_parameter<int>(
            param_base_name_ + ".expected_framerate", config_->expected_framerate,
            ParameterDescriptor()
                .set__description("expected video frame rate [frames/s]")
                .set__integer_range({rcl_interfaces::msg::IntegerRange().set__from_value(1).set__to_value(100)}));
    if (!node->has_parameter(param_base_name_ + ".use_hw_encoder"))
        node->declare_parameter<bool>(
            param_base_name_ + ".use_hw_encoder", config_->use_hw_encoder,
            ParameterDescriptor().set__description("use NVENC or VAAPI hardware acceleration if possible"));
    if (!node->has_parameter(param_base_name_ + ".udp_port"))
        node->declare_parameter<int>(
            param_base_name_ + ".udp_port", config_->udp_port,
            ParameterDescriptor()
                .set__description("force UDP port for RTSP server (0 = auto select)")
                .set__integer_range({rcl_interfaces::msg::IntegerRange().set__from_value(0).set__to_value(65535)}));
    if (!node->has_parameter(param_base_name_ + ".udp_packet_size"))
        node->declare_parameter<int>(
            param_base_name_ + ".udp_packet_size", config_->udp_packet_size,
            ParameterDescriptor()
                .set__description("size limit for UDP packets [octets]")
                .set__integer_range({rcl_interfaces::msg::IntegerRange().set__from_value(576).set__to_value(9000)}));
    if (!node->has_parameter(param_base_name_ + ".use_ip_multicast"))
        node->declare_parameter<bool>(param_base_name_ + ".use_ip_multicast", config_->use_ip_multicast,
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
    if (getNumSubscribers() == 0 && encoder_ && server_)
    {
        RCLCPP_INFO(logger_, "[%s] stop encoding for %s", topic_name_.c_str(), server_->url().c_str());
        encoder_.reset();
    }
}

}  // namespace rtsp_image_transport

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rtsp_image_transport::PublisherPlugin, image_transport::PublisherPlugin)
