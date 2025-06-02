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

#include "graph_monitor.h"

namespace rtsp_image_transport
{

std::mutex GraphMonitor::mutex_;
GraphMonitor::SharedPtr GraphMonitor::instance_;

GraphMonitor::GraphMonitor(rclcpp::Node* node)
    : node_graph_(node->get_node_graph_interface()), event_(node_graph_->get_graph_event()), shutdown_flag_(false),
      thread_(std::bind(&GraphMonitor::eventLoop, this))
{
    thread_.detach();
}

void GraphMonitor::eventLoop()
{
    using namespace std::chrono_literals;
    while (!shutdown_flag_.load())
    {
        node_graph_->wait_for_graph_change(event_, 10s);
        if (event_->check_and_clear() && !shutdown_flag_.load())
        {
            std::lock_guard<std::mutex> lock{mutex_};
            for (GraphMonitorListener* listener : listeners_)
                listener->onGraphChange();
        }
    }
    instance_.reset();
}

void GraphMonitor::addListener(GraphMonitorListener* listener)
{
    if (listener != nullptr)
    {
        std::lock_guard<std::mutex> lock{mutex_};
        listeners_.insert(listener);
    }
}

void GraphMonitor::removeListener(GraphMonitorListener* listener)
{
    if (listener != nullptr)
    {
        std::lock_guard<std::mutex> lock{mutex_};
        if (listeners_.erase(listener))
        {
            if (listeners_.empty())
            {
                shutdown_flag_.store(true);
                node_graph_->notify_graph_change();
            }
        }
    }
}

GraphMonitor::SharedPtr GraphMonitor::instance(rclcpp::Node* node, GraphMonitorListener* listener)
{
    using namespace std::chrono_literals;
    std::lock_guard<std::mutex> lock{mutex_};
    if (instance_)
    {
        if (!instance_->shutdown_flag_.load())
            return instance_;
        while (instance_)
            std::this_thread::yield();
    }
    instance_.reset(new GraphMonitor(node));
    if (listener != nullptr)
        instance_->listeners_.insert(listener);
    return instance_;
}

}  // namespace rtsp_image_transport