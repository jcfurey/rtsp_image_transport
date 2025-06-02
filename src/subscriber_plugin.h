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
#ifndef RTSP_IMAGE_TRANSPORT_SUBSCRIBER_PLUGIN_H_
#define RTSP_IMAGE_TRANSPORT_SUBSCRIBER_PLUGIN_H_

#include "frame_data.h"
#include "rtsp_image_transport_export.h"
#include "video_codec.h"

#include <image_transport/simple_subscriber_plugin.hpp>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/string.hpp>

#include <deque>
#include <mutex>

class MediaSubsession;

namespace rtsp_image_transport
{

class StreamClient;
class StreamDecoder;

class RTSP_IMAGE_TRANSPORT_EXPORT SubscriberPlugin
    : public image_transport::SimpleSubscriberPlugin<std_msgs::msg::String>
{
public:
    SubscriberPlugin();
    void shutdown() override;
    std::string getTransportName() const override;

protected:
    void subscribeImpl(rclcpp::Node* node, const std::string& base_topic, const Callback& callback,
                       rmw_qos_profile_t custom_qos, rclcpp::SubscriptionOptions options) override;
    void internalCallback(const std_msgs::msg::String::ConstSharedPtr& message, const Callback& callback) override;

private:
    void receiveDataStream(VideoCodec codec, MediaSubsession* subsession, const FrameDataPtr& data);
    void subsessionStarted(VideoCodec codec, MediaSubsession* subsession);
    void sessionFailed(int code, const std::string& message);
    void sessionStarted();
    void sessionFinished();
    void sessionTimeout();
    void processFrame();
    void reconnect();
    void cooldownTimerCallback();
    void pushFrame(const FrameDataPtr& frame);
    FrameDataPtr popFrame();
    rclcpp::Duration frameLag() const noexcept;
    void clearQueuedFrames();
    void setupParameters(rclcpp::Node* node);
    void updateParameters();

    struct Config;
    rclcpp::Logger logger_;
    std::string topic_name_, param_base_name_;
    bool failed_;
    std::chrono::milliseconds cooldown_;
    std::unique_ptr<Config> config_;
    rclcpp::node_interfaces::NodeTimersInterface::WeakPtr node_timers_;
    rclcpp::node_interfaces::NodeBaseInterface::WeakPtr node_base_;
    rclcpp::node_interfaces::NodeParametersInterface::WeakPtr node_param_;
    rclcpp::Node::PostSetParametersCallbackHandle::SharedPtr param_cb_handle_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::CallbackGroup::SharedPtr cooldown_cb_group_, scheduled_cb_group_;
    rclcpp::Duration old_lag_;
    rclcpp::WallTimer<rclcpp::VoidCallbackType>::SharedPtr cooldown_timer_;
    rclcpp::Waitable::SharedPtr scheduled_cb_;
    Callback callback_;
    std::shared_ptr<StreamClient> client_;
    std::shared_ptr<StreamDecoder> decoder_;

    mutable std::mutex queue_mutex_;
    std::deque<FrameDataPtr> queue_;
};

}  // namespace rtsp_image_transport

#endif
