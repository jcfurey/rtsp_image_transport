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
#ifndef RTSP_IMAGE_TRANSPORT_GRAPH_MONITOR_H_
#define RTSP_IMAGE_TRANSPORT_GRAPH_MONITOR_H_

#include <rclcpp/node.hpp>

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <set>
#include <thread>

namespace rtsp_image_transport
{

class GraphMonitorListener
{
public:
    virtual void onGraphChange() = 0;
};

class GraphMonitor
{
public:
    using SharedPtr = std::shared_ptr<GraphMonitor>;
    void addListener(GraphMonitorListener* listener);
    void removeListener(GraphMonitorListener* listener);

    static SharedPtr instance(rclcpp::Node* node, GraphMonitorListener* listener = nullptr);

private:
    explicit GraphMonitor(rclcpp::Node* node);
    void eventLoop();

    /* Declared before node_graph_, so it is destroyed after it. NodeGraph keeps
       only a raw NodeBaseInterface*, and holding the graph interface alive past
       the node — which the detached thread below does by construction — leaves
       that pointer dangling and the node still registered with rclcpp's own
       GraphListener. That listener then triggers a notify guard condition whose
       owner is gone, throws out of its thread, and aborts the process. Keeping
       the base alive for exactly as long as the graph keeps both valid. */
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_;
    rclcpp::Event::SharedPtr event_;
    std::set<GraphMonitorListener*> listeners_;
    std::atomic_bool shutdown_flag_;
    std::thread thread_;

    static std::mutex mutex_;
    static std::condition_variable instance_released_;
    static SharedPtr instance_;
};

}  // namespace rtsp_image_transport

#endif