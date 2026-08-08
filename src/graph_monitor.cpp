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

#include <algorithm>
#include <functional>
#include <utility>
#include <vector>

namespace rtsp_image_transport
{

std::mutex GraphMonitor::mutex_;
std::condition_variable GraphMonitor::instance_released_;
GraphMonitor::SharedPtr GraphMonitor::instance_;

namespace
{

/* Takes one graph event per node and never lets go of it.
 *
 * rclcpp registers a node with its GraphListener only on the first
 * get_graph_event() call — should_add_to_graph_listener_ is a one shot — and
 * the listener skips any node whose graph user count has fallen back to zero
 * when it rebuilds its wait set. Dropping the last event therefore takes the
 * node out of that wait set for good: a later get_graph_event() raises the
 * count again but never interrupts the listener, so the node's graph guard
 * condition is not waited on and notify_graph_change() is never called for it.
 *
 * A monitor that shuts down when its last listener leaves releases exactly that
 * last event, so the next monitor for the same node saw no graph changes at all
 * and every published image went unnoticed until something else happened to
 * wake the listener. Keeping one event alive holds the count above zero and
 * costs nothing beyond the event itself.
 */
void retainGraphEvent(const rclcpp::node_interfaces::NodeGraphInterface::SharedPtr& graph)
{
    using Entry = std::pair<std::weak_ptr<rclcpp::node_interfaces::NodeGraphInterface>, rclcpp::Event::SharedPtr>;
    static std::mutex retained_mutex;
    static std::vector<Entry> retained;

    std::lock_guard<std::mutex> lock{retained_mutex};
    std::erase_if(retained, [](const Entry& entry) { return entry.first.expired(); });
    for (const Entry& entry : retained)
    {
        if (entry.first.lock() == graph)
            return;
    }
    retained.emplace_back(graph, graph->get_graph_event());
}

}  // namespace

GraphMonitor::GraphMonitor(rclcpp::Node* node)
    : node_base_(node->get_node_base_interface()), node_graph_(node->get_node_graph_interface()),
      event_(node_graph_->get_graph_event()), shutdown_flag_(false),
      thread_(std::bind(&GraphMonitor::eventLoop, this))
{
    retainGraphEvent(node_graph_);
    thread_.detach();
}

void GraphMonitor::eventLoop()
{
    using namespace std::chrono_literals;
    while (!shutdown_flag_.load())
    {
        /* This thread is detached, so an escaping exception would terminate the
           process. rclcpp throws here once the context is shut down, which
           happens routinely while a publisher is still advertised. */
        try
        {
            node_graph_->wait_for_graph_change(event_, 10s);
            if (event_->check_and_clear() && !shutdown_flag_.load())
            {
                std::lock_guard<std::mutex> lock{mutex_};
                for (GraphMonitorListener* listener : listeners_)
                    listener->onGraphChange();
            }
        }
        catch (const std::exception&)
        {
            /* Nothing left to watch */
            break;
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
                /* Only to wake the wait below. Once the context is down there
                   is nothing to wake and nothing valid to poke: rclcpp throws
                   out of this rather than ignoring it, and the loop is about to
                   notice the flag on its own timeout anyway. */
                try
                {
                    if (node_base_ && node_base_->get_context() && node_base_->get_context()->is_valid())
                        node_graph_->notify_graph_change();
                }
                catch (const std::exception&)
                {
                }
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
