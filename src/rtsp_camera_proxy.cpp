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
#include "host_override.h"

#include <BasicUsageEnvironment.hh>
#include <cv_bridge/cv_bridge.hpp>
#include <liveMedia.hh>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#include <format>
#include <thread>

extern char _binary_rtsp_only_png_start;
extern char _binary_rtsp_only_png_end;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("rtsp_camera_proxy");
    rclcpp::Logger logger = node->get_logger();
    int ros_sock = -1;
    if (const char* var = getenv("ROS_HOSTNAME"))
    {
        ros_sock = rtsp_image_transport::create_host_override_socket(var);
    }
    else if (const char* var = getenv("ROS_IP"))
    {
        ros_sock = rtsp_image_transport::create_host_override_socket(var, true);
    }
    if (ros_sock >= 0)
    {
        RCLCPP_INFO(logger,
                    "ROS_HOSTNAME/ROS_IP override: RTSP proxy will advertise IP "
                    "address %s",
                    rtsp_image_transport::socket_bound_address(ros_sock).c_str());
    }
    int png_size = &_binary_rtsp_only_png_end - &_binary_rtsp_only_png_start;
    const cv::Mat rtsp_only_mat =
        cv::imdecode(cv::Mat(1, png_size, CV_8U, &_binary_rtsp_only_png_start), cv::IMREAD_COLOR);
    sensor_msgs::msg::Image::ConstSharedPtr rtsp_only_img =
        cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8, rtsp_only_mat).toImageMsg();
    OutPacketBuffer::maxSize = 500000;
    TaskScheduler* scheduler = BasicTaskScheduler::createNew();
    UsageEnvironment* env = BasicUsageEnvironment::createNew(*scheduler);

    using rcl_interfaces::msg::IntegerRange;
    using rcl_interfaces::msg::ParameterDescriptor;
    int server_port =
        node->declare_parameter<int>("server_port", 0,
                                     ParameterDescriptor()
                                         .set__description("override RTSP server port")
                                         .set__integer_range({IntegerRange().set__from_value(0).set__to_value(65535)})
                                         .set__read_only(true));
    bool tcp = node->declare_parameter<bool>(
        "tcp", false, ParameterDescriptor().set__description("transmit RTP stream over TCP").set__read_only(true));

    std::vector<std::string> cameras = node->declare_parameter<std::vector<std::string>>(
        "cameras", {},
        ParameterDescriptor()
            .set__description("list of RTSP camera URIs which are to be proxied")
            .set__read_only(true));

    if (cameras.empty())
    {
        RCLCPP_FATAL(logger, "No RTSP camera URIs specified. Exiting.");
        return 1;
    }
    RTSPServer* rtspServer = RTSPServer::createNew(*env, server_port);
    if (!rtspServer)
    {
        RCLCPP_FATAL(logger, "Failed to setup RTSP server instance: %s", env->getResultMsg());
        return 1;
    }
    std::size_t id = 0;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> camera_pub_raw;
    std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> camera_pub_rtsp;
    for (const std::string& camera_uri : cameras)
    {
        std::string camera_base_name = std::format("camera{}", id++);
        std::string camera_base_topic = node->get_node_topics_interface()->resolve_topic_name(camera_base_name);
        portNumBits http_tunnel_port = tcp ? 0xFFFF : 0;
        ProxyServerMediaSession* sms = ProxyServerMediaSession::createNew(
            *env, rtspServer, camera_uri.c_str(), camera_base_name.c_str(), nullptr, nullptr, http_tunnel_port);
        rtspServer->addServerMediaSession(sms);
        char* proxyURL = rtspServer->rtspURL(sms, ros_sock);
        std_msgs::msg::String url_msg;
        url_msg.data = proxyURL;
        delete[] proxyURL;
        RCLCPP_INFO(logger, "%s: %s via %s", camera_base_topic.c_str(), camera_uri.c_str(), url_msg.data.c_str());

        camera_pub_raw.push_back(node->create_publisher<sensor_msgs::msg::Image>(
            camera_base_topic, rclcpp::QoS(rclcpp::KeepLast(1)).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)));
        camera_pub_raw.back()->publish(*rtsp_only_img);
        camera_pub_rtsp.push_back(node->create_publisher<std_msgs::msg::String>(
            camera_base_topic + "/rtsp", rclcpp::QoS(rclcpp::KeepLast(1))
                                             .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
                                             .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)));
        camera_pub_rtsp.back()->publish(url_msg);
    }
    EventLoopWatchVariable shutdown = 0;
    std::thread t([&shutdown, &env]() { env->taskScheduler().doEventLoop(&shutdown); });
    rclcpp::spin(node);
    shutdown = 1;
    t.join();
    return 0;
}
