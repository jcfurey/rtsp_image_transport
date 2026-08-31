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

/* The two command line programs, driven the way a user drives them. Both are
   documented entry points — publish_rtsp_stream is how the camera adapters
   announce a stream — and neither had any test at all. */

#include "stream_client.h"
#include "test_helpers.h"

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <csignal>
#include <condition_variable>
#include <cstdlib>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <sys/wait.h>
#include <unistd.h>

using namespace std::chrono_literals;
using namespace rtsp_image_transport;
using namespace rtsp_image_transport::test;

namespace
{

/* A child process that is always reaped, however the test leaves the scope. */
class Child
{
public:
    Child(const std::string& program, const std::vector<std::string>& args)
    {
        std::vector<char*> argv;
        argv.push_back(const_cast<char*>(program.c_str()));
        for (const std::string& a : args)
            argv.push_back(const_cast<char*>(a.c_str()));
        argv.push_back(nullptr);

        pid_ = fork();
        if (pid_ == 0)
        {
            /* Its own process group, so a stuck child can be killed as a unit
               without signalling the test runner. */
            setpgid(0, 0);
            execv(program.c_str(), argv.data());
            _exit(127);
        }
    }

    ~Child()
    {
        if (pid_ > 0)
        {
            ::kill(-pid_, SIGKILL);
            int status = 0;
            waitpid(pid_, &status, 0);
        }
    }

    Child(const Child&) = delete;
    Child& operator=(const Child&) = delete;

    bool started() const
    {
        return pid_ > 0;
    }

    /* Waits for the child to exit on its own. Returns false on timeout. */
    bool waitFor(std::chrono::milliseconds budget, int& exit_code)
    {
        const auto deadline = std::chrono::steady_clock::now() + budget;
        while (std::chrono::steady_clock::now() < deadline)
        {
            int status = 0;
            const pid_t r = waitpid(pid_, &status, WNOHANG);
            if (r == pid_)
            {
                pid_ = -1;
                exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
                return true;
            }
            std::this_thread::sleep_for(20ms);
        }
        return false;
    }

    void interrupt()
    {
        if (pid_ > 0)
            ::kill(pid_, SIGINT);
    }

private:
    pid_t pid_ = -1;
};

std::string binary(const char* name)
{
    return std::string(RTSP_BINARY_DIR) + "/" + name;
}

/* Waits for one latched message on a topic the child publishes. */
bool awaitUrl(const std::string& topic, std::string& out, std::chrono::seconds budget)
{
    auto node = std::make_shared<rclcpp::Node>("executable_test_listener");
    bool got = false;
    auto sub = node->create_subscription<std_msgs::msg::String>(
        topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
        [&out, &got](const std_msgs::msg::String::ConstSharedPtr& msg)
        {
            out = msg->data;
            got = true;
        });
    const auto deadline = std::chrono::steady_clock::now() + budget;
    while (!got && std::chrono::steady_clock::now() < deadline)
        rclcpp::spin_some(node);
    return got;
}

bool waitForSubscriptionCount(const rclcpp::PublisherBase::SharedPtr& publisher, std::size_t wanted,
                              std::chrono::seconds budget)
{
    const auto deadline = std::chrono::steady_clock::now() + budget;
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (publisher->get_subscription_count() == wanted)
            return true;
        std::this_thread::sleep_for(20ms);
    }
    return false;
}

class RtspObserver
{
public:
    void attach(const std::shared_ptr<StreamClient>& client)
    {
        client->setSessionFailedHandler(
            [this](int, const std::string& message)
            {
                std::lock_guard<std::mutex> lock{mutex_};
                failure_ = message;
                cv_.notify_all();
            });
        client->setReceiveStreamDataHandler(
            [this](VideoCodec, MediaSubsession*, const FrameDataPtr&)
            {
                std::lock_guard<std::mutex> lock{mutex_};
                ++nal_count_;
                cv_.notify_all();
            });
    }

    bool waitForNals(std::size_t wanted, std::chrono::seconds budget)
    {
        std::unique_lock<std::mutex> lock{mutex_};
        return cv_.wait_for(lock, budget, [&] { return nal_count_ >= wanted || !failure_.empty(); })
               && failure_.empty() && nal_count_ >= wanted;
    }

    std::string failure()
    {
        std::lock_guard<std::mutex> lock{mutex_};
        return failure_;
    }

private:
    std::mutex mutex_;
    std::condition_variable cv_;
    std::size_t nal_count_ = 0;
    std::string failure_;
};

}  // namespace

TEST(PublishRtspStream, RefusesToRunWithoutAUrl)
{
    Child child(binary("publish_rtsp_stream"), {});
    ASSERT_TRUE(child.started());
    int exit_code = 0;
    ASSERT_TRUE(child.waitFor(20s, exit_code)) << "the program kept running with no URL to publish";
    EXPECT_EQ(exit_code, 2) << "a missing URL has to be reported as a usage error";
}

TEST(PublishRtspStream, PublishesTheUrlOnTheTransportTopic)
{
    const std::string url = "rtsp://192.0.2.1:554/stream";
    Child child(binary("publish_rtsp_stream"),
                {url, "--ros-args", "-r", "image:=/executable_test/camera", "-r", "__node:=publish_under_test"});
    ASSERT_TRUE(child.started());

    std::string received;
    ASSERT_TRUE(awaitUrl("/executable_test/camera/rtsp", received, 30s))
        << "the URL never arrived on the transport topic";
    EXPECT_EQ(received, url);
}

/* The placeholder image exists so that consumers which only ever look at the
   raw topic still see something, and so the topic type is established. It is
   published exactly once, latched, and that is all the program promises.

   Whether a late joiner then receives it is up to the middleware, and it is not
   uniform: measured on Fast DDS, a single transient-local sensor_msgs/Image
   published at start-up is delivered on Jazzy and Kilted but not on Lyrical or
   Rolling, while the std_msgs/String URL from the same process is delivered on
   all four. Republishing to paper over that would put a stream of 1x1 frames on
   the user's image topic and skew anything measuring its rate, so the behaviour
   stands and the test asserts what is actually guaranteed: the publisher is
   there, with the right topic, and whatever it does deliver has the right
   shape. */
TEST(PublishRtspStream, AdvertisesThePlaceholderImageTopic)
{
    Child child(binary("publish_rtsp_stream"),
                {"rtsp://192.0.2.1:554/x", "--ros-args", "-r", "image:=/executable_test/placeholder", "-r",
                 "__node:=placeholder_under_test"});
    ASSERT_TRUE(child.started());

    auto node = std::make_shared<rclcpp::Node>("placeholder_listener");
    bool got = false;
    sensor_msgs::msg::Image image;
    auto sub = node->create_subscription<sensor_msgs::msg::Image>(
        "/executable_test/placeholder", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
        [&image, &got](const sensor_msgs::msg::Image::ConstSharedPtr& msg)
        {
            image = *msg;
            got = true;
        });

    const auto deadline = std::chrono::steady_clock::now() + 20s;
    while (node->count_publishers("/executable_test/placeholder") == 0
           && std::chrono::steady_clock::now() < deadline)
        rclcpp::spin_some(node);
    ASSERT_GT(node->count_publishers("/executable_test/placeholder"), 0u)
        << "the raw image topic was never advertised";

    /* A short grace period for the latched sample, since it does arrive on some
       middleware and its contents are worth checking when it does. */
    const auto sample_deadline = std::chrono::steady_clock::now() + 5s;
    while (!got && std::chrono::steady_clock::now() < sample_deadline)
        rclcpp::spin_some(node);
    if (!got)
        GTEST_SKIP() << "this middleware did not deliver the latched placeholder to a late joiner";

    EXPECT_EQ(image.width, 1u);
    EXPECT_EQ(image.height, 1u);
    EXPECT_EQ(image.encoding, "bgr8");
    EXPECT_EQ(image.data.size(), 3u);
}

TEST(PublishRtspStream, ExitsCleanlyOnInterrupt)
{
    Child child(binary("publish_rtsp_stream"),
                {"rtsp://192.0.2.1:554/y", "--ros-args", "-r", "image:=/executable_test/shutdown", "-r",
                 "__node:=shutdown_under_test"});
    ASSERT_TRUE(child.started());
    std::string received;
    ASSERT_TRUE(awaitUrl("/executable_test/shutdown/rtsp", received, 30s));

    child.interrupt();
    int exit_code = 0;
    ASSERT_TRUE(child.waitFor(20s, exit_code)) << "the program did not stop on SIGINT";
    EXPECT_EQ(exit_code, 0) << "shutdown was not clean";
}

TEST(RtspCameraProxy, RefusesToRunWithNoCameras)
{
    Child child(binary("rtsp_camera_proxy"), {"--ros-args", "-r", "__node:=proxy_no_cameras"});
    ASSERT_TRUE(child.started());
    int exit_code = 0;
    ASSERT_TRUE(child.waitFor(20s, exit_code)) << "the proxy kept running with nothing to proxy";
    EXPECT_EQ(exit_code, 1);
}

/* The proxy advertises its own RTSP URL for each camera before it has talked to
   any of them, so an unreachable address still exercises the whole setup path. */
TEST(RtspCameraProxy, AdvertisesAProxyUrlPerCamera)
{
    Child child(binary("rtsp_camera_proxy"),
                {"--ros-args", "-r", "__node:=proxy_under_test", "-r", "camera0:=/executable_test/proxy0", "-p",
                 "cameras:=[rtsp://192.0.2.1:554/one]"});
    ASSERT_TRUE(child.started());

    std::string received;
    ASSERT_TRUE(awaitUrl("/executable_test/proxy0/rtsp", received, 30s))
        << "the proxy never advertised a URL for its camera";
    EXPECT_EQ(received.rfind("rtsp://", 0), 0u) << "advertised URL was " << received;
    /* Its own server, not the upstream camera it proxies for. */
    EXPECT_EQ(received.find("192.0.2.1"), std::string::npos) << "the upstream URL was advertised: " << received;
}

TEST(ImageTransportRepublish, NativeRtspClientDrivesLazyInput)
{
    /* image_transport republish normally subscribes to `in` only while its
       output has a ROS subscriber. A native RTSP client is not in the ROS
       graph, so this is the regression test for the publisher plugin's virtual
       matched event: direct RTSP demand must start the raw subscription and
       leaving must release it again. */
    if (!testEncoderFor(VideoCodec::H264))
        GTEST_SKIP() << "no software H.264 encoder in this FFmpeg build";

    constexpr const char* input_topic = "/native_demand/input";
    constexpr const char* output_topic = "/native_demand/output/rtsp";
    auto node = std::make_shared<rclcpp::Node>("native_demand_test_source");
    auto publisher = node->create_publisher<sensor_msgs::msg::Image>(input_topic, rclcpp::QoS(1));

    std::jthread pump(
        [&](std::stop_token stop)
        {
            unsigned sequence = 0;
            while (!stop.stop_requested())
            {
                publisher->publish(makeTestImage(320, 240, sequence++));
                std::this_thread::sleep_for(33ms);
            }
        });

    Child relay("/usr/bin/env",
                {"ros2", "run", "image_transport", "republish", "--ros-args", "-p", "in_transport:=raw", "-p",
                 "out_transport:=rtsp", "-p", "out.rtsp.codec:=H264", "-p", "out.rtsp.use_hw_encoder:=false",
                 "-p", "out.rtsp.udp_port:=0", "-r", std::string("in:=") + input_topic, "-r",
                 std::string("out/rtsp:=") + output_topic, "-r", "__node:=native_demand_republisher"});
    ASSERT_TRUE(relay.started());

    /* A short-lived ROS subscriber gets the advertised URL. Once it is gone,
       prove the normal republisher match callback has made the input lazy
       before testing the native-client path. */
    std::string url;
    ASSERT_TRUE(awaitUrl(output_topic, url, 30s)) << "the relay never advertised its RTSP URL";
    ASSERT_TRUE(waitForSubscriptionCount(publisher, 0, 10s))
        << "the relay kept its raw input after the URL subscriber left";

    RtspObserver observer;
    std::shared_ptr<StreamClient> client = StreamClient::create("native_demand", url);
    observer.attach(client);
    ASSERT_NO_THROW(client->connect());

    ASSERT_TRUE(waitForSubscriptionCount(publisher, 1, 10s))
        << "a native RTSP client did not activate the raw image subscription";
    ASSERT_TRUE(observer.waitForNals(3, 30s)) << "no video reached the native RTSP client: " << observer.failure();

    client->disconnect();
    ASSERT_TRUE(waitForSubscriptionCount(publisher, 0, 10s))
        << "the relay kept its raw input after the native RTSP client left";
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
