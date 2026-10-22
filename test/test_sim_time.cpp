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
#include <image_transport/subscriber_plugin.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#if __has_include(<rclcpp/experimental/executors/events_executor/events_executor.hpp>)
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#define RTSP_TEST_HAS_EVENTS_EXECUTOR 1
#endif

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <mutex>
#include <sstream>
#include <thread>
#include <string>
#include <utility>
#include <vector>

using namespace std::chrono_literals;
using namespace rtsp_image_transport::test;

namespace
{

void spinSome(const rclcpp::Node::SharedPtr& node)
{
    rclcpp::ExecutorOptions options;
    options.context = node->get_node_base_interface()->get_context();
    rclcpp::executors::SingleThreadedExecutor executor(options);
    executor.add_node(node);
    executor.spin_some();
}

template<class Plugin>
void checkWireMessageType(const Plugin& plugin)
{
    // Metadata discovery was added after the node-interface API. Keep this
    // regression usable with distributions whose base class predates it.
    if constexpr (requires { plugin.getMessageType(); })
        EXPECT_EQ(plugin.getMessageType(), "std_msgs/msg/String");
}

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
            spinSome(node_);
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
  TransportFixture(const std::string& node_name, bool use_sim_time,
                   const rclcpp::Context::SharedPtr& context = nullptr)
  {
      rclcpp::NodeOptions options;
      if (context)
          options.context(context);
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
#if !RTSP_IMAGE_TRANSPORT_USES_NODE_INTERFACES
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
                spinSome(node_);
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
    ASSERT_TRUE(fixture.start()) << fixture.failure_;

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
    ASSERT_TRUE(fixture.start("pacing/image")) << fixture.failure_;

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
    ASSERT_TRUE(fixture.start("alive/image")) << fixture.failure_;
    for (int i = 0; i < 50; ++i)
    {
        spinSome(fixture.node_);
        std::this_thread::sleep_for(20ms);
    }
    EXPECT_TRUE(fixture.node_->has_parameter("alive.image.rtsp.codec"))
        << "the publisher plugin never declared its correctly scoped parameters";
    EXPECT_TRUE(fixture.node_->has_parameter("alive.image.rtsp.keyframe_interval"))
        << "the publisher plugin did not expose the key frame interval";
    EXPECT_TRUE(fixture.node_->has_parameter("alive.image.rtsp.intra_refresh"))
        << "the publisher plugin did not expose intra refresh";
    EXPECT_TRUE(fixture.node_->has_parameter("alive.image.rtsp.use_hw_decoder"))
        << "the subscriber plugin never declared its correctly scoped parameters";
    EXPECT_TRUE(fixture.node_->has_parameter("alive.image.rtsp.timestamp_source"))
        << "the subscriber plugin never declared its correctly scoped parameters";
    EXPECT_TRUE(fixture.node_->has_parameter("alive.image.rtsp.frame_id"))
        << "the subscriber plugin did not expose decoded-image frame stamping";
    EXPECT_TRUE(fixture.node_->has_parameter("alive.image.rtsp.max_latency"))
        << "the subscriber plugin did not expose its latency budget";
    /* The drop ladder is what bounds latency once the decoder cannot keep up,
       so its budget has to be reachable from a launch file. */
    EXPECT_NEAR(fixture.node_->get_parameter("alive.image.rtsp.max_latency").as_double(), 0.2, 1e-9);
    /* RTP goes over UDP unless asked otherwise: TCP cannot drop, so a lost
       segment stalls the viewer behind a retransmit it has no use for. */
    EXPECT_FALSE(fixture.node_->get_parameter("alive.image.rtsp.rtp_over_tcp").as_bool());
    /* On UDP a lost packet can wedge the decoder while RTP keeps arriving, so
       the stall watchdog is what the transport default depends on. */
    EXPECT_TRUE(fixture.node_->has_parameter("alive.image.rtsp.decoder_stall_timeout"));
    EXPECT_GT(fixture.node_->get_parameter("alive.image.rtsp.decoder_stall_timeout").as_double(), 0.0);

    /* Declaring a parameter is not the same as honouring one. The launch files
       pass these in, and a name or type that does not match what the plugin
       declared would be accepted by launch and silently ignored here, so set
       each one the way a launch file does and read it back. */
    for (const auto& [name, value] :
         std::vector<std::pair<std::string, rclcpp::ParameterValue>>{
             {"alive.image.rtsp.decoder_stall_timeout", rclcpp::ParameterValue(3.5)},
             {"alive.image.rtsp.max_latency", rclcpp::ParameterValue(0.15)},
             {"alive.image.rtsp.rtp_over_tcp", rclcpp::ParameterValue(true)},
             {"alive.image.rtsp.video_subsession", rclcpp::ParameterValue(1)},
             {"alive.image.rtsp.rtp_buffer_size", rclcpp::ParameterValue(1048576)},
             {"alive.image.rtsp.keyframe_interval", rclcpp::ParameterValue(7)},
             {"alive.image.rtsp.intra_refresh", rclcpp::ParameterValue(true)},
             {"alive.image.rtsp.target_bitrate", rclcpp::ParameterValue(3000000)},
             {"alive.image.rtsp.expected_framerate", rclcpp::ParameterValue(25)}})
    {
        SCOPED_TRACE(name);
        ASSERT_TRUE(fixture.node_->has_parameter(name)) << "not declared, so a launch file setting it is ignored";
        ASSERT_NO_THROW(fixture.node_->set_parameter(rclcpp::Parameter(name, value)));
        EXPECT_EQ(fixture.node_->get_parameter(name).get_type(), value.get_type())
            << "the plugin declared a different type than a launch file would pass";
    }
    EXPECT_NEAR(fixture.node_->get_parameter("alive.image.rtsp.decoder_stall_timeout").as_double(), 3.5, 1e-9);
    EXPECT_TRUE(fixture.node_->get_parameter("alive.image.rtsp.intra_refresh").as_bool());
    EXPECT_EQ(fixture.node_->get_parameter("alive.image.rtsp.keyframe_interval").as_int(), 7);
    EXPECT_FALSE(fixture.node_->has_parameter("live.image.rtsp.use_hw_decoder"))
        << "the relative topic lost its first character while constructing parameter names";
}

TEST(Transport, PluginManifestReportsTheUrlWireType)
{
    pluginlib::ClassLoader<image_transport::SubscriberPlugin> loader("image_transport",
                                                                    "image_transport::SubscriberPlugin");
    auto plugin = loader.createUniqueInstance("image_transport/rtsp_sub");
    EXPECT_EQ(plugin->getTransportName(), "rtsp");
    checkWireMessageType(*plugin);
}

TEST(Transport, SubscriberQosMatchesPublisher)
{
    /* Both ends force RELIABLE and TRANSIENT_LOCAL on the URL topic. If those
       ever drift apart the subscriber silently receives nothing, so check the
       endpoints agree rather than waiting for video to fail. */
    TransportFixture fixture("rtsp_transport_qos", /*use_sim_time=*/false);
    ASSERT_TRUE(fixture.start("qos/image")) << fixture.failure_;

    const std::string url_topic = fixture.node_->get_effective_namespace() == "/"
                                      ? "/qos/image/rtsp"
                                      : fixture.node_->get_effective_namespace() + "/qos/image/rtsp";
    std::vector<rclcpp::TopicEndpointInfo> publishers, subscribers;
    for (int i = 0; i < 100 && (publishers.empty() || subscribers.empty()); ++i)
    {
        spinSome(fixture.node_);
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

    ASSERT_TRUE(fixture.start("sim/image")) << fixture.failure_;

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
    /* `ros2 bag play --loop --clock` sends the clock back to the start of the
       recording. The stream has to carry on rather than stalling. */
    TransportFixture fixture("rtsp_transport_bag_loop", /*use_sim_time=*/true);
    SimClock clock(fixture.node_);
    const std::int64_t sim_start = 500000000000LL;
    clock.set(sim_start);
    ASSERT_TRUE(clock.waitUntilAdopted());

    ASSERT_TRUE(fixture.start("loop/image")) << fixture.failure_;

    const std::size_t before = fixture.pump(2, 45s, &clock, sim_start);
    ASSERT_GE(before, 1u) << "nothing arrived before the loop";

    /* The bag restarts: the clock and the image stamps both jump back */
    clock.set(sim_start);
    for (int i = 0; i < 20; ++i)
    {
        spinSome(fixture.node_);
        std::this_thread::sleep_for(10ms);
    }
    const std::size_t after = fixture.pump(before + 2, 45s, &clock, sim_start);
    EXPECT_GT(after, before) << "the stream stopped delivering images after the clock jumped backwards";
}

TEST(Transport, SimulatedTimeCanBeEnabledAndPausedAtRuntime)
{
    TransportFixture fixture("rtsp_transport_clock_switch", false);
    ASSERT_TRUE(fixture.start("switch/image")) << fixture.failure_;
    ASSERT_GE(fixture.pump(2, 20s, nullptr, fixture.node_->now().nanoseconds()), 2u);

    ASSERT_TRUE(fixture.node_->set_parameter(rclcpp::Parameter("use_sim_time", true)).successful);
    SimClock clock(fixture.node_);
    constexpr std::int64_t sim_start = 1234000000000LL;
    clock.set(sim_start);
    ASSERT_TRUE(clock.waitUntilAdopted());
    const auto before = fixture.count();
    ASSERT_GE(fixture.pump(before + 3, 20s, &clock, sim_start), before + 3);
    EXPECT_LT(rclcpp::Time(fixture.received().back().header.stamp).nanoseconds(), sim_start + 60000000000LL);

    // No further /clock messages: the network and executor must keep working,
    // and receive stamps must remain at the paused ROS time.
    const auto paused = clock.nowNs();
    const auto count = fixture.count();
    ASSERT_GE(fixture.pump(count + 3, 20s, nullptr, paused, 0), count + 3);
    EXPECT_EQ(rclcpp::Time(fixture.received().back().header.stamp).nanoseconds(), paused);

    ASSERT_TRUE(fixture.node_->set_parameter(rclcpp::Parameter("use_sim_time", false)).successful);
    const auto resume_count = fixture.count();
    ASSERT_GE(fixture.pump(resume_count + 3, 20s, nullptr, fixture.node_->now().nanoseconds()),
              resume_count + 3);
    EXPECT_GT(rclcpp::Time(fixture.received().back().header.stamp).nanoseconds(), 1000000000000000000LL);
}

TEST(Transport, SupportsANonDefaultRosContext)
{
    auto context = std::make_shared<rclcpp::Context>();
    context->init(0, nullptr);
    {
        TransportFixture fixture("rtsp_transport_context", false, context);
        ASSERT_TRUE(fixture.start("context/image")) << fixture.failure_;
        EXPECT_GE(fixture.pump(3, 20s, nullptr, fixture.node_->now().nanoseconds()), 3u);
    }
    context->shutdown("explicit-context transport test complete");
}

#ifdef RTSP_TEST_HAS_EVENTS_EXECUTOR
TEST(Transport, DeliversFramesThroughEventsExecutor)
{
    TransportFixture fixture("rtsp_transport_events", false);
    ASSERT_TRUE(fixture.start("events/image")) << fixture.failure_;
    rclcpp::experimental::executors::EventsExecutor executor;
    executor.add_node(fixture.node_);
    const auto deadline = std::chrono::steady_clock::now() + 20s;
    unsigned sequence = 0;
    while (fixture.count() < 3 && std::chrono::steady_clock::now() < deadline)
    {
        auto image = makeTestImage(320, 240, sequence++);
        image.header.stamp = fixture.node_->now();
        fixture.publisher_.publish(image);
        executor.spin_some();
        std::this_thread::sleep_for(20ms);
    }
    EXPECT_GE(fixture.count(), 3u);
}
#endif

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
