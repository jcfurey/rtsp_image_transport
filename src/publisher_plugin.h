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
#ifndef RTSP_IMAGE_TRANSPORT_PUBLISHER_PLUGIN_H_
#define RTSP_IMAGE_TRANSPORT_PUBLISHER_PLUGIN_H_

#include "graph_monitor.h"
#include "rtsp_image_transport_export.h"

#include <image_transport/simple_publisher_plugin.hpp>
#include <rclcpp/graph_listener.hpp>
#include <std_msgs/msg/string.hpp>

namespace rtsp_image_transport
{

class StreamEncoder;
class StreamServer;
class GraphMonitor;

class RTSP_IMAGE_TRANSPORT_EXPORT PublisherPlugin
    : public image_transport::SimplePublisherPlugin<std_msgs::msg::String>,
      private GraphMonitorListener
{
public:
    PublisherPlugin();
    void shutdown() override;

    std::string getTransportName() const override;

protected:
    void advertiseImpl(rclcpp::Node* node, const std::string& base_topic, rmw_qos_profile_t custom_qos,
                       rclcpp::PublisherOptions options) override;
    void publish(const sensor_msgs::msg::Image& image, const PublisherT& publisher) const override;

private:
    void setupParameters(rclcpp::Node* node);
    void updateParameters();
    void onGraphChange() override;

    struct Config;
    rclcpp::Logger logger_;
    rclcpp::node_interfaces::NodeParametersInterface::WeakPtr node_param_;
    std::string topic_name_, param_base_name_;
    std::unique_ptr<Config> config_;
    rclcpp::Node::PostSetParametersCallbackHandle::SharedPtr param_cb_handle_;
    std::shared_ptr<StreamServer> server_;
    std::shared_ptr<GraphMonitor> graph_monitor_;
    mutable std::unique_ptr<StreamEncoder> encoder_;
    mutable std::mutex mutex_;
    mutable bool update_url_, failed_;
};

}  // namespace rtsp_image_transport

#endif
