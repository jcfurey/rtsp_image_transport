/****************************************************************************
 *
 * rtsp_image_transport
 * Copyright © 2021-2025 Fraunhofer FKIE
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

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <atomic>
#include <chrono>
#include <thread>

using namespace rtsp_image_transport;
using namespace std::chrono_literals;

namespace
{

class CountingListener : public GraphMonitorListener
{
public:
    void onGraphChange() override
    {
        calls++;
    }
    std::atomic_uint calls{0};
};

class GraphMonitorTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        node_ = std::make_shared<rclcpp::Node>("graph_monitor_test");
    }
    void TearDown() override
    {
        node_.reset();
    }
    rclcpp::Node::SharedPtr node_;
};

}  // namespace

TEST_F(GraphMonitorTest, SharesOneInstanceBetweenListeners)
{
    CountingListener a, b;
    GraphMonitor::SharedPtr first = GraphMonitor::instance(node_.get(), &a);
    GraphMonitor::SharedPtr second = GraphMonitor::instance(node_.get(), &b);
    ASSERT_TRUE(first);
    EXPECT_EQ(first.get(), second.get()) << "each publisher started its own monitor thread";
    first->removeListener(&a);
    second->removeListener(&b);
}

TEST_F(GraphMonitorTest, NotifiesListenersOnGraphChange)
{
    CountingListener listener;
    GraphMonitor::SharedPtr monitor = GraphMonitor::instance(node_.get(), &listener);
    ASSERT_TRUE(monitor);

    /* Creating a publisher changes the graph, which the monitor must notice */
    auto publisher = node_->create_publisher<std_msgs::msg::String>("graph_monitor_test_topic", 1);
    for (int i = 0; i < 200 && listener.calls.load() == 0; ++i)
        std::this_thread::sleep_for(50ms);
    EXPECT_GT(listener.calls.load(), 0u) << "graph change was never reported";

    monitor->removeListener(&listener);
}

TEST_F(GraphMonitorTest, StopsNotifyingAfterRemoval)
{
    CountingListener listener;
    GraphMonitor::SharedPtr monitor = GraphMonitor::instance(node_.get(), &listener);
    monitor->removeListener(&listener);
    monitor.reset();

    const unsigned before = listener.calls.load();
    auto publisher = node_->create_publisher<std_msgs::msg::String>("graph_monitor_removed_topic", 1);
    std::this_thread::sleep_for(500ms);
    EXPECT_EQ(listener.calls.load(), before) << "a removed listener was still called";
}

TEST_F(GraphMonitorTest, CanBeRestartedAfterTheLastListenerLeaves)
{
    /* The monitor thread releases the singleton when the last listener goes
       away. Asking for a new one used to race with that hand-off; this must
       simply produce a fresh working monitor. */
    for (int round = 0; round < 5; ++round)
    {
        CountingListener listener;
        GraphMonitor::SharedPtr monitor = GraphMonitor::instance(node_.get(), &listener);
        ASSERT_TRUE(monitor) << "round " << round;
        monitor->removeListener(&listener);
        monitor.reset();
    }
    CountingListener listener;
    GraphMonitor::SharedPtr monitor = GraphMonitor::instance(node_.get(), &listener);
    ASSERT_TRUE(monitor);
    auto publisher = node_->create_publisher<std_msgs::msg::String>("graph_monitor_restart_topic", 1);
    for (int i = 0; i < 200 && listener.calls.load() == 0; ++i)
        std::this_thread::sleep_for(50ms);
    EXPECT_GT(listener.calls.load(), 0u) << "the restarted monitor does not report changes";
    monitor->removeListener(&listener);
}

TEST_F(GraphMonitorTest, IgnoresNullListeners)
{
    GraphMonitor::SharedPtr monitor = GraphMonitor::instance(node_.get(), nullptr);
    ASSERT_TRUE(monitor);
    EXPECT_NO_THROW(monitor->addListener(nullptr));
    EXPECT_NO_THROW(monitor->removeListener(nullptr));
    /* Still has no listeners, so add one and take it away to shut it down */
    CountingListener listener;
    monitor->addListener(&listener);
    monitor->removeListener(&listener);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
