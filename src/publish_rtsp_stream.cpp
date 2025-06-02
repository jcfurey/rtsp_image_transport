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
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

extern char _binary_rtsp_only_png_start;
extern char _binary_rtsp_only_png_end;

int main(int argc, char** argv)
{
    std::string self = argv[0];
    std::string::size_type n = self.rfind('/');
    if (n != std::string::npos)
        self = self.substr(n + 1);

    rclcpp::init(argc, argv);
    std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>(self);
    rclcpp::Logger logger = node->get_logger();
    std::string topic = node->get_node_topics_interface()->resolve_topic_name("image");

    if (args.size() < 2)
    {
        RCLCPP_ERROR(logger,
                     "Missing stream URL. Typical command line usage:\n"
                     "  $ ros2 run " ROS_PACKAGE_NAME " %s rtsp://<url> --ros-args --remap image:=<image topic>",
                     self.c_str());
        return 2;
    }
    if (topic == "image")
    {
        RCLCPP_WARN(logger,
                    "Topic 'image' has not been remapped! Typical command line usage:\n"
                    "  $ ros2 run " ROS_PACKAGE_NAME " %s rtsp://<url> --ros-args --remap image:=<image topic>",
                    self.c_str());
    }
    if (args.size() > 2)
    {
        RCLCPP_WARN(logger, "Extra command line arguments ignored");
    }
    std_msgs::msg::String url;
    url.data = args[1];
    if (url.data.substr(0, 7) != "rtsp://")
    {
        RCLCPP_WARN(logger, "URL does not begin with rtsp://");
    }
    int png_size = &_binary_rtsp_only_png_end - &_binary_rtsp_only_png_start;
    const cv::Mat rtsp_only_mat =
        cv::imdecode(cv::Mat(1, png_size, CV_8U, &_binary_rtsp_only_png_start), cv::IMREAD_COLOR);
    sensor_msgs::msg::Image::SharedPtr rtsp_only_img =
        cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8, rtsp_only_mat).toImageMsg();
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub0 = node->create_publisher<sensor_msgs::msg::Image>(
        topic, rclcpp::QoS(rclcpp::KeepLast(1)).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL));
    pub0->publish(*rtsp_only_img);
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub = node->create_publisher<std_msgs::msg::String>(
        topic + "/rtsp", rclcpp::QoS(rclcpp::KeepLast(1)).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL));
    pub->publish(url);
    rclcpp::spin(node);
}
