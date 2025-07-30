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

namespace rtsp_image_transport
{

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
      update_url_(false), failed_(false)
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
    custom_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    custom_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    custom_qos.depth = 1;
    custom_qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
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
    new_config.target_bitrate = np->get_parameter(param_base_name_ + ".target_bitrate").as_int();
    new_config.expected_framerate = np->get_parameter(param_base_name_ + ".expected_framerate").as_int();
    new_config.use_hw_encoder = np->get_parameter(param_base_name_ + ".use_hw_encoder").as_bool();
    new_config.udp_port = np->get_parameter(param_base_name_ + ".udp_port").as_int();
    new_config.udp_packet_size = np->get_parameter(param_base_name_ + ".udp_packet_size").as_int();
    new_config.use_ip_multicast = np->get_parameter(param_base_name_ + ".use_ip_multicast").as_bool();

    int changelevel = 0;
    if ((!server_ && !failed_) || config_->udp_port != new_config.udp_port
        || config_->udp_packet_size != config_->udp_packet_size)
        changelevel |= LVL_SERVER;
    if (config_->codec != new_config.codec || config_->use_ip_multicast != new_config.use_ip_multicast)
        changelevel |= LVL_SESSION;
    if (config_->target_bitrate != new_config.target_bitrate
        || config_->expected_framerate != new_config.expected_framerate
        || config_->use_hw_encoder != new_config.use_hw_encoder)
        changelevel |= LVL_CODEC;

    std::lock_guard<std::mutex> lock{mutex_};
    *config_ = new_config;
    try
    {
        if (changelevel >= LVL_SERVER)
        {
            server_.reset();
            server_ = StreamServer::create(topic_name_, config_->udp_port, config_->udp_packet_size - 42, logger_);
        }
        if (changelevel >= LVL_SESSION)
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
            encoder_->setBitrate(config_->target_bitrate);
            encoder_->setFramerate(config_->expected_framerate);
            encoder_->setPackageSizeHint(server_->maxPacketSize() - 24);
            RCLCPP_INFO(logger_, "[%s] start encoding (%s; %sbit/s; %u fps) for %s", topic_name_.c_str(),
                        encoder_->context()->codec->name, withSI(config_->target_bitrate).c_str(),
                        config_->expected_framerate, server_->url().c_str());
        }
        if (image.header.stamp.sec == 0 && image.header.stamp.nanosec == 0)
            RCLCPP_WARN(logger_, "[%s] image header time stamp is not set, expect broken RTSP stream",
                        topic_name_.c_str());
        if (encoder_->encodeVideo(image) > 0)
        {
            while (FrameDataPtr data = encoder_->nextPacket())
                server_->sendFrame(data);
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
