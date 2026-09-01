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
#include "init.h"
#include "test_helpers.h"

#include <gtest/gtest.h>

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
using namespace rtsp_image_transport::test;

/* image_transport 6.4 replaced the plugin entry points with ones that take node
   interfaces. Encoding and transport work through that API, but it does not
   include the node clock interface the subscriber needs for simulated stamps. */
#define TRANSPORT_SUPPORTS_SIM_TIME (CURRENT_IMAGE_TRANSPORT_VERSION < FKIE_VERSION_TUPLE(6, 4, 0))
#define SKIP_IF_SIM_TIME_UNSUPPORTED()                                                              \
    do                                                                                              \
    {                                                                                               \
        if (!TRANSPORT_SUPPORTS_SIM_TIME)                                                           \
            GTEST_SKIP() << "image_transport " << (CURRENT_IMAGE_TRANSPORT_VERSION >> 16) << "."     \
                         << ((CURRENT_IMAGE_TRANSPORT_VERSION >> 8) & 0xff)                          \
                         << " does not give subscriber plugins the node clock interface needed "      \
                            "for simulated image stamps";                                            \
    } while (0)

namespace
{

/* Drives /clock the way a simulator or `ros2 bag play --clock` does */
class SimClock
{
public:
    explicit SimClock(const rclcpp::Node::SharedPtr& node)
        : publisher_(node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS())),
          node_(node)
    {
    }

    void set(std::int64_t nanoseconds)
    {
        now_ns_ = nanoseconds;
        rosgraph_msgs::msg::Clock msg;
        msg.clock = rclcpp::Time(nanoseconds);
        publisher_->publish(msg);
    }

    void advance(std::int64_t delta_ns)
    {
        set(now_ns_ + delta_ns);
    }

    std::int64_t nowNs() const
    {
        return now_ns_;
    }

    bool waitUntilAdopted(std::chrono::seconds timeout = 15s)
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline)
        {
            set(now_ns_);
            rclcpp::spin_some(node_);
            if (node_->get_clock()->now().nanoseconds() >= now_ns_ && now_ns_ > 0)
                return true;
            std::this_thread::sleep_for(10ms);
        }
        return false;
    }

private:
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr publisher_;
    rclcpp::Node::SharedPtr node_;
    std::int64_t now_ns_ = 0;
};

/* Runs a complete rtsp transport hop inside one process: an image_transport
   publisher on the rtsp transport, and an rtsp subscriber picking it back up. */
class TransportFixture
{
public:
    TransportFixture(const std::string& node_name, bool use_sim_time)
    {
        rclcpp::NodeOptions options;
        options.append_parameter_override("use_sim_time", use_sim_time);
        node_ = std::make_shared<rclcpp::Node>(node_name, options);
    }

    bool start(const std::string& topic = "camera/image")
    {
        auto collect = [this](const sensor_msgs::msg::Image::ConstSharedPtr& img)
        {
            std::lock_guard<std::mutex> lock{mutex_};
            received_.push_back(*img);
        };
        try
        {
#if RTSP_IMAGE_TRANSPORT_HAS_LEGACY_PLUGIN_API
            publisher_ = image_transport::create_publisher(node_.get(), topic);
            subscriber_ = image_transport::create_subscription(node_.get(), topic, collect, "rtsp");
#else
            /* image_transport 7.0 takes node interfaces, and its QoS argument
               has no default. */
            publisher_ = image_transport::create_publisher(*node_, topic, rclcpp::QoS(1));
            subscriber_ = image_transport::create_subscription(*node_, topic, collect, "rtsp", rclcpp::QoS(1));
#endif
        }
        catch (const std::exception& e)
        {
            failure_ = e.what();
            return false;
        }
        return true;
    }

    /* Publishes images until at least `wanted` come back, or the budget runs
       out. Returns how many arrived. */
    std::size_t pump(std::size_t wanted, std::chrono::seconds budget, SimClock* clock = nullptr,
                     std::int64_t stamp_start_ns = 0, std::int64_t stamp_step_ns = 33333333LL)
    {
        const auto deadline = std::chrono::steady_clock::now() + budget;
        unsigned seq = 0;
        std::int64_t stamp = stamp_start_ns;
        while (std::chrono::steady_clock::now() < deadline && count() < wanted)
        {
            sensor_msgs::msg::Image img = makeTestImage(320, 240, seq++);
            img.header.stamp = rclcpp::Time(stamp);
            img.header.frame_id = "camera";
            publisher_.publish(img);
            stamp += stamp_step_ns;
            if (clock)
                clock->set(stamp);
            for (int i = 0; i < 4; ++i)
            {
                rclcpp::spin_some(node_);
                std::this_thread::sleep_for(8ms);
            }
        }
        return count();
    }

    std::size_t count() const
    {
        std::lock_guard<std::mutex> lock{mutex_};
        return received_.size();
    }

    std::vector<sensor_msgs::msg::Image> received() const
    {
        std::lock_guard<std::mutex> lock{mutex_};
        return received_;
    }

    rclcpp::Node::SharedPtr node_;
    image_transport::Publisher publisher_;
    image_transport::Subscriber subscriber_;
    std::string failure_;

private:
    mutable std::mutex mutex_;
    std::vector<sensor_msgs::msg::Image> received_;
};

}  // namespace

TEST(Transport, DeliversImagesEndToEnd)
{
    /* The whole transport in one process: encode, serve over RTSP, discover the
       URL over the latched topic, connect, decode. Also proves the publisher
       and subscriber QoS settings actually match each other. */
    TransportFixture fixture("rtsp_transport_roundtrip", /*use_sim_time=*/false);
    if (!fixture.start())
        GTEST_SKIP() << "rtsp transport plugin not loadable: " << fixture.failure_;

    rclcpp::Clock wall(RCL_SYSTEM_TIME);
    const std::size_t got = fixture.pump(3, 60s, nullptr, wall.now().nanoseconds());
    ASSERT_GE(got, 1u) << "no image made it through the rtsp transport";

    for (const auto& img : fixture.received())
    {
        EXPECT_EQ(img.width, 320u);
        EXPECT_EQ(img.height, 240u);
        EXPECT_EQ(img.encoding, "bgr8");
        EXPECT_EQ(img.data.size(), static_cast<std::size_t>(3) * 320 * 240);
    }
}

TEST(Transport, PresentationTimesKeepTheSpacingOfTheImageStamps)
{
    /* Regression guard. A picture leaves the encoder as several NAL units —
       parameter sets ahead of every key frame, one per slice under x264's
       slice-max-size — and the publisher used to map each of them onto the
       RTP timeline separately. StreamClock takes a repeated stamp for a clock
       jump and steps a millisecond past it, so every extra NAL unit pushed the
       timeline that much ahead of real time. Players pace display by those
       time stamps and fell steadily behind the longer they watched: tens of
       seconds after a couple of minutes on a multi-slice software stream.

       The sender's time stamps come back on the decoded images (the default
       timestamp_source on a node running wall clock time), so the check is
       that consecutive images are a whole number of frame intervals apart.
       Dropped frames keep the spacing a multiple, a constant offset does not
       show up at all, and rounding through the 90 kHz RTP clock costs tens of
       microseconds. One discontinuity is allowed: the client re-synchronises
       its presentation clock on the first RTCP sender report after PLAY. */
    TransportFixture fixture("rtsp_transport_pacing", /*use_sim_time=*/false);
    if (!fixture.start("pacing/image"))
        GTEST_SKIP() << "rtsp transport plugin not loadable: " << fixture.failure_;

    rclcpp::Clock wall(RCL_SYSTEM_TIME);
    /* Enough images to span several key frames, whose parameter sets make the
       extra NAL units a certainty whatever encoder this machine picked */
    const std::size_t got = fixture.pump(100, 60s, nullptr, wall.now().nanoseconds(), FRAME_INTERVAL_NS);
    ASSERT_GE(got, 70u) << "too few images made it through the rtsp transport to judge their pacing";

    constexpr std::int64_t TOLERANCE_NS = 500000;  // 0.5 ms; the bug costs at least 1 ms per key frame
    const std::vector<sensor_msgs::msg::Image> received = fixture.received();
    unsigned discontinuities = 0;
    std::ostringstream details;
    for (std::size_t i = 1; i < received.size(); ++i)
    {
        const std::int64_t delta = rclcpp::Time(received[i].header.stamp).nanoseconds()
                                   - rclcpp::Time(received[i - 1].header.stamp).nanoseconds();
        const std::int64_t frames = std::llround(static_cast<double>(delta) / FRAME_INTERVAL_NS);
        const std::int64_t residual = std::llabs(delta - frames * FRAME_INTERVAL_NS);
        if (residual > TOLERANCE_NS)
        {
            ++discontinuities;
            details << " #" << i << ": " << delta / 1000 << " us";
        }
    }
    EXPECT_LE(discontinuities, 1u) << "the RTP timeline does not keep the spacing of the image stamps; "
                                   << discontinuities << " of " << received.size() - 1
                                   << " intervals are off by more than " << TOLERANCE_NS / 1000
                                   << " us:" << details.str();
}

TEST(Transport, PluginsAreActuallyDrivenByImageTransport)
{
    /* image_transport has more than one plugin entry point, and picking the
       wrong one to override compiles cleanly but leaves the plugin inert: the
       topic appears, no RTSP server is ever started, and no video flows. The
       parameters are the cheapest visible proof that our code ran. */
    TransportFixture fixture("rtsp_transport_alive", /*use_sim_time=*/false);
    if (!fixture.start("alive/image"))
        GTEST_SKIP() << "rtsp transport plugin not loadable: " << fixture.failure_;
    for (int i = 0; i < 50; ++i)
    {
        rclcpp::spin_some(fixture.node_);
        std::this_thread::sleep_for(20ms);
    }
    EXPECT_TRUE(fixture.node_->has_parameter("alive.image.rtsp.codec"))
        << "the publisher plugin never declared its correctly scoped parameters";
    EXPECT_TRUE(fixture.node_->has_parameter("alive.image.rtsp.use_hw_decoder"))
        << "the subscriber plugin never declared its correctly scoped parameters";
    EXPECT_TRUE(fixture.node_->has_parameter("alive.image.rtsp.timestamp_source"))
        << "the subscriber plugin never declared its correctly scoped parameters";
    EXPECT_TRUE(fixture.node_->has_parameter("alive.image.rtsp.frame_id"))
        << "the subscriber plugin did not expose decoded-image frame stamping";
    EXPECT_FALSE(fixture.node_->has_parameter("live.image.rtsp.use_hw_decoder"))
        << "the relative topic lost its first character while constructing parameter names";
}

TEST(Transport, SubscriberQosMatchesPublisher)
{
    /* Both ends force RELIABLE and TRANSIENT_LOCAL on the URL topic. If those
       ever drift apart the subscriber silently receives nothing, so check the
       endpoints agree rather than waiting for video to fail. */
    TransportFixture fixture("rtsp_transport_qos", /*use_sim_time=*/false);
    if (!fixture.start("qos/image"))
        GTEST_SKIP() << "rtsp transport plugin not loadable: " << fixture.failure_;

    const std::string url_topic = fixture.node_->get_effective_namespace() == "/"
                                      ? "/qos/image/rtsp"
                                      : fixture.node_->get_effective_namespace() + "/qos/image/rtsp";
    std::vector<rclcpp::TopicEndpointInfo> publishers, subscribers;
    for (int i = 0; i < 100 && (publishers.empty() || subscribers.empty()); ++i)
    {
        rclcpp::spin_some(fixture.node_);
        publishers = fixture.node_->get_publishers_info_by_topic(url_topic);
        subscribers = fixture.node_->get_subscriptions_info_by_topic(url_topic);
        std::this_thread::sleep_for(20ms);
    }
    ASSERT_FALSE(publishers.empty()) << "no publisher on " << url_topic;
    ASSERT_FALSE(subscribers.empty()) << "no subscriber on " << url_topic;

    for (const auto& endpoint : publishers)
    {
        EXPECT_EQ(endpoint.qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
        EXPECT_EQ(endpoint.qos_profile().durability(), rclcpp::DurabilityPolicy::TransientLocal);
    }
    for (const auto& endpoint : subscribers)
    {
        EXPECT_EQ(endpoint.qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
        EXPECT_EQ(endpoint.qos_profile().durability(), rclcpp::DurabilityPolicy::TransientLocal);
    }
}

TEST(Transport, DeliversImagesStampedInSimulatedTime)
{
    SKIP_IF_SIM_TIME_UNSUPPORTED();
    /* With use_sim_time the rest of the system works in the simulated time
       base. Images arriving stamped with the sender's wall clock would be
       hours or years away from it, and every downstream TF lookup or message
       filter would reject them. */
    TransportFixture fixture("rtsp_transport_sim_time", /*use_sim_time=*/true);
    SimClock clock(fixture.node_);
    const std::int64_t sim_start = 1234500000000LL;  // 1234.5 s, nowhere near wall clock
    clock.set(sim_start);
    ASSERT_TRUE(clock.waitUntilAdopted()) << "the node clock never picked up /clock";
    ASSERT_TRUE(fixture.node_->get_clock()->ros_time_is_active());

    if (!fixture.start("sim/image"))
        GTEST_SKIP() << "rtsp transport plugin not loadable: " << fixture.failure_;

    const std::size_t got = fixture.pump(3, 60s, &clock, sim_start);
    ASSERT_GE(got, 1u) << "no image made it through the rtsp transport under simulated time";

    rclcpp::Clock wall(RCL_SYSTEM_TIME);
    const std::int64_t wall_now = wall.now().nanoseconds();
    for (const auto& img : fixture.received())
    {
        const std::int64_t stamp = rclcpp::Time(img.header.stamp).nanoseconds();
        EXPECT_GE(stamp, sim_start) << "image predates the simulated clock";
        EXPECT_LT(stamp, sim_start + 600LL * 1000000000LL) << "image is far ahead of the simulated clock";
        EXPECT_GT(std::llabs(wall_now - stamp), 3600LL * 1000000000LL)
            << "image carries a wall clock stamp instead of simulated time";
    }
}

TEST(Transport, KeepsDeliveringWhenTheClockJumpsBackwards)
{
    SKIP_IF_SIM_TIME_UNSUPPORTED();
    /* `ros2 bag play --loop --clock` sends the clock back to the start of the
       recording. The stream has to carry on rather than stalling. */
    TransportFixture fixture("rtsp_transport_bag_loop", /*use_sim_time=*/true);
    SimClock clock(fixture.node_);
    const std::int64_t sim_start = 500000000000LL;
    clock.set(sim_start);
    ASSERT_TRUE(clock.waitUntilAdopted());

    if (!fixture.start("loop/image"))
        GTEST_SKIP() << "rtsp transport plugin not loadable: " << fixture.failure_;

    const std::size_t before = fixture.pump(2, 45s, &clock, sim_start);
    ASSERT_GE(before, 1u) << "nothing arrived before the loop";

    /* The bag restarts: the clock and the image stamps both jump back */
    clock.set(sim_start);
    for (int i = 0; i < 20; ++i)
    {
        rclcpp::spin_some(fixture.node_);
        std::this_thread::sleep_for(10ms);
    }
    const std::size_t after = fixture.pump(before + 2, 45s, &clock, sim_start);
    EXPECT_GT(after, before) << "the stream stopped delivering images after the clock jumped backwards";
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
