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

#include "init.h"
#include "frame_data.h"
#include "rtsp_image_transport_export.h"
#include "video_codec.h"

#include <image_transport/simple_subscriber_plugin.hpp>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/string.hpp>

#include <atomic>
#include <chrono>
#include <deque>
#include <functional>
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
    ~SubscriberPlugin() override;
    void shutdown() override;
    std::string getTransportName() const override;

protected:
#if RTSP_IMAGE_TRANSPORT_HAS_LEGACY_PLUGIN_API
    void subscribeImpl(rclcpp::Node* node, const std::string& base_topic, const Callback& callback,
                       rmw_qos_profile_t custom_qos, rclcpp::SubscriptionOptions options) override;
#endif
#if RTSP_IMAGE_TRANSPORT_USES_NODE_INTERFACES
    /* Newer image_transport releases pass only their required node interfaces.
       The implementation uses a short wall timer in place of a custom waitable
       and a system clock when the node clock interface is unavailable. */
    void subscribeImpl(image_transport::RequiredInterfaces node_interfaces, const std::string& base_topic,
                       const Callback& callback, rclcpp::QoS custom_qos,
                       rclcpp::SubscriptionOptions options) override;
#endif
    void internalCallback(const std_msgs::msg::String::ConstSharedPtr& message, const Callback& callback) override;

private:
    friend class SubscriberPluginTestPeer;
    struct QueuedFrame
    {
        FrameDataPtr data;
        std::chrono::steady_clock::time_point received_at;
    };
    void receiveDataStream(VideoCodec codec, MediaSubsession* subsession, const FrameDataPtr& data);
    void subsessionStarted(VideoCodec codec, MediaSubsession* subsession);
    void sessionFailed(int code, const std::string& message);
    void sessionStarted();
    void sessionFinished();
    void sessionTimeout();
    void processFrame();
    bool useReceiveTimestamps() const;
    void reportMissingHwDecoder(const StreamDecoder& decoder);
    void reconnect();
    void cooldownTimerCallback();
    void pushFrame(const FrameDataPtr& frame,
                   std::chrono::steady_clock::time_point received_at = std::chrono::steady_clock::now());
    QueuedFrame popFrame();
    void trimQueuedFrames();  // caller holds queue_mutex_
    void clearQueuedFrames();
    void setupParameters(const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_parameters);
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
    rclcpp::node_interfaces::NodeWaitablesInterface::WeakPtr node_waitables_;
    rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr param_cb_handle_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::CallbackGroup::SharedPtr cooldown_cb_group_, scheduled_cb_group_;
    rclcpp::Duration old_lag_;
    rclcpp::WallTimer<rclcpp::VoidCallbackType>::SharedPtr cooldown_timer_, frame_timer_;
    /* Guards cooldown_, cooldown_timer_, cooldown_cb_group_: reconnect
       bookkeeping is driven from ROS executor threads and from the Live555
       handler thread (session failed/timeout), which used to race. */
    mutable std::mutex cooldown_mutex_;
    rclcpp::Waitable::SharedPtr scheduled_cb_;
    std::function<void()> notify_frame_;
    Callback callback_;
    std::shared_ptr<StreamClient> client_;
    std::atomic<std::shared_ptr<StreamDecoder>> decoder_;
    /* Watchdog for a decoder that is being fed but produces nothing. The
       session timeout cannot see this: it watches for RTP arriving, and here
       RTP arrives normally while the decoder rejects every slice. Wall clock
       deliberately, like the reconnect timers: a stalled decoder is a real
       time problem whether or not a simulation is paused. */
    mutable std::mutex stall_mutex_;
    std::chrono::steady_clock::time_point last_image_out_{};
    std::size_t decoder_stalls_ = 0;

    mutable std::mutex queue_mutex_;
    std::deque<QueuedFrame> queue_;
    std::size_t queued_bytes_ = 0;
    bool bound_queue_ = true;
    // The first access unit may carry parameter sets supplied only in SDP.
    bool preserve_first_frame_ = true;
    static constexpr std::size_t MAX_QUEUED_FRAMES = 120;
    static constexpr std::size_t MAX_QUEUED_BYTES = 64u << 20;
};

}  // namespace rtsp_image_transport

#endif
