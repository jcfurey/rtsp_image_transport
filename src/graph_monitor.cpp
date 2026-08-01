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

#include <functional>

namespace rtsp_image_transport
{

std::mutex GraphMonitor::mutex_;
std::condition_variable GraphMonitor::instance_released_;
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
    /* Hand the last reference over to a local variable so that instance() can
       see the singleton slot become free while this object is still alive. */
    SharedPtr self;
    {
        std::lock_guard<std::mutex> lock{mutex_};
        self.swap(instance_);
    }
    instance_released_.notify_all();
    /* `self` may destroy *this* when it goes out of scope; do not touch any
       member from here on. */
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
    std::unique_lock<std::mutex> lock{mutex_};
    if (instance_ && !instance_->shutdown_flag_.load())
    {
        if (listener != nullptr)
            instance_->listeners_.insert(listener);
        return instance_;
    }
    /* A previous monitor is shutting down; wait until its thread has released
       the singleton before a new one is started. */
    instance_released_.wait(lock, []() { return !instance_; });
    instance_.reset(new GraphMonitor(node));
    if (listener != nullptr)
        instance_->listeners_.insert(listener);
    return instance_;
}

}  // namespace rtsp_image_transport
