// SPDX-License-Identifier: Apache-2.0
#include "publisher_plugin.h"
#include "subscriber_plugin.h"
#include "stream_encoder.h"
#include "stream_server.h"
#include "test_helpers.h"

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <chrono>
#include <thread>

using namespace rtsp_image_transport;
using namespace rtsp_image_transport::test;
using namespace std::chrono_literals;

namespace
{
class TransportHarness
{
public:
    explicit TransportHarness(bool tcp)
    {
        static std::atomic<unsigned> id{0};
        rclcpp::NodeOptions options;
        options.append_parameter_override("video.rtsp.use_hw_encoder", false);
        options.append_parameter_override("video.rtsp.use_hw_decoder", false);
        options.append_parameter_override("video.rtsp.rtp_over_tcp", tcp);
        options.append_parameter_override("video.rtsp.reconnect_minwait", 0.02);
        options.append_parameter_override("video.rtsp.reconnect_maxwait", 0.1);
        node = std::make_shared<rclcpp::Node>("robustness_" + std::to_string(id++), options);
        executor.add_node(node);
    }

    ~TransportHarness()
    {
        executor.cancel();
        if (thread.joinable())
            thread.join();
        subscriber.reset();
        publisher.reset();
    }

    void start(bool publish_images = true)
    {
        if (publish_images)
        {
            publisher = std::make_unique<PublisherPlugin>();
#if RTSP_IMAGE_TRANSPORT_HAS_LEGACY_PLUGIN_API
            publisher->advertise(node.get(), "video", rmw_qos_profile_default);
#else
            publisher->advertise(*node, "video", rclcpp::QoS(1));
#endif
        }
        subscriber = std::make_unique<SubscriberPlugin>();
        const auto collect = [this](const sensor_msgs::msg::Image::ConstSharedPtr& image)
        {
            width.store(image->width);
            ++images;
            if (on_image)
                on_image();
        };
#if RTSP_IMAGE_TRANSPORT_HAS_LEGACY_PLUGIN_API
        subscriber->subscribe(node.get(), "video", collect, rmw_qos_profile_default);
#else
        subscriber->subscribe(*node, "video", collect, rclcpp::QoS(1));
#endif
        thread = std::thread([this] { executor.spin(); });
    }

    template <class Action, class Predicate>
    bool until(Action action, Predicate predicate, std::chrono::milliseconds timeout = 8s)
    {
        const auto end = std::chrono::steady_clock::now() + timeout;
        do
        {
            action();
            if (predicate())
                return true;
            std::this_thread::sleep_for(20ms);
        } while (std::chrono::steady_clock::now() < end);
        return predicate();
    }

    void publish(unsigned w = 160, unsigned h = 120)
    {
        auto image = makeTestImage(w, h, sequence++);
        image.header.stamp = node->now();
        publisher->publish(image);
    }

    rclcpp::Node::SharedPtr node;
    std::unique_ptr<image_transport::PublisherPlugin> publisher;
    std::unique_ptr<image_transport::SubscriberPlugin> subscriber;
    std::atomic<unsigned> images{0}, width{0};
    rclcpp::executors::MultiThreadedExecutor executor{rclcpp::ExecutorOptions(), 3};
    std::thread thread;
    unsigned sequence = 0;
    std::function<void()> on_image;
};

class RobustTransport : public ::testing::TestWithParam<bool>
{
protected:
    void SetUp() override
    {
        try
        {
            StreamEncoder probe(VideoCodec::H264, false);
        }
        catch (const std::exception& e)
        {
            GTEST_SKIP() << "no usable software H.264 encoder: " << e.what();
        }
    }
};
} // namespace

TEST_P(RobustTransport, PublishingSurvivesMalformedImagesAndResolutionChanges)
{
    TransportHarness h(GetParam());
    h.start();
    ASSERT_TRUE(h.until([&] { h.publish(); }, [&] { return h.images >= 3; }));
    for (int fault = 0; fault < 3; ++fault)
    {
        auto invalid = makeTestImage(160, 120, 0);
        if (fault == 0)
            invalid.encoding = "unsupported";
        if (fault == 1)
            invalid.data.clear();
        if (fault == 2)
            invalid.step = 1;
        h.publisher->publish(invalid);
        const auto before = h.images.load();
        ASSERT_TRUE(h.until([&] { h.publish(); }, [&] { return h.images >= before + 3; }));
    }
    ASSERT_TRUE(h.until([&] { h.publish(320, 240); }, [&] { return h.width == 320; }));
}

TEST_P(RobustTransport, SubscriberRecoversWhenTheServerBecomesAvailable)
{
    TransportHarness h(GetParam());
    auto server = StreamServer::create("late_camera", 0, 1354);
    server->start(VideoCodec::H264, false);
    const auto url = server->url();
    server->stop(); // TCP accepts connections but DESCRIBE returns 404.
    auto url_publisher =
        h.node->create_publisher<std_msgs::msg::String>("video/rtsp", rclcpp::QoS(1).reliable().transient_local());
    h.start(false);
    std_msgs::msg::String message;
    message.data = url;
    url_publisher->publish(message);
    std::this_thread::sleep_for(300ms);
    EXPECT_EQ(h.images, 0u);
    server->start(VideoCodec::H264, false);
    StreamEncoder encoder(VideoCodec::H264, false);
    const auto send = [&]
    {
        auto image = makeTestImage(160, 120, h.sequence++);
        image.header.stamp = h.node->now();
        encoder.encodeVideo(image);
        std::vector<FrameDataPtr> packets;
        while (auto packet = encoder.nextPacket())
            packets.push_back(std::move(packet));
        server->sendAccessUnit(packets);
    };
    ASSERT_TRUE(h.until(send, [&] { return h.images >= 3; }));
    // Reconfiguration must obtain the SDP again, and a cancelled retry may
    // never reconnect an old session on top of the replacement.
    for (bool low_latency : {false, true})
    {
        ASSERT_TRUE(h.node->set_parameter(rclcpp::Parameter("video.rtsp.low_latency", low_latency)).successful);
        const auto before = h.images.load();
        ASSERT_TRUE(h.until(send, [&] { return h.images >= before + 3; }));
    }
    ASSERT_TRUE(h.node->set_parameter(rclcpp::Parameter("video.rtsp.decoder", "missing_decoder")).successful);
    std::this_thread::sleep_for(250ms);
    ASSERT_TRUE(h.node->set_parameter(rclcpp::Parameter("video.rtsp.decoder", "")).successful);
    const auto before = h.images.load();
    ASSERT_TRUE(h.until(send, [&] { return h.images >= before + 3; }));
}

TEST_P(RobustTransport, DestructionStopsCallbacksWhileTheExecutorKeepsRunning)
{
    TransportHarness h(GetParam());
    h.start();
    ASSERT_TRUE(h.until([&] { h.publish(); }, [&] { return h.images >= 3; }));
    // Deliberately use destruction, without an explicit shutdown() call.
    h.subscriber.reset();
    const auto final_count = h.images.load();
    for (int i = 0; i < 8; ++i)
    {
        h.publish();
        h.node->set_parameter(rclcpp::Parameter("video.rtsp.frame_id", "after_shutdown"));
        std::this_thread::sleep_for(10ms);
    }
    EXPECT_EQ(h.images, final_count);
    h.publisher.reset();
    h.node->set_parameter(rclcpp::Parameter("video.rtsp.target_bitrate", 2000000));
    std::this_thread::sleep_for(50ms);
}

TEST_P(RobustTransport, SubscriberCanBeDestroyedFromItsImageCallback)
{
    TransportHarness h(GetParam());
    std::atomic<bool> destroyed{false};
    h.on_image = [&]
    {
        h.subscriber.reset();
        destroyed.store(true);
    };
    h.start();
    ASSERT_TRUE(h.until([&] { h.publish(); }, [&] { return destroyed.load(); }));
    EXPECT_EQ(h.images, 1u);
}

TEST_P(RobustTransport, SubscriberCanBeDestroyedFromItsThrowingImageCallback)
{
    TransportHarness h(GetParam());
    std::atomic<bool> destroyed{false};
    h.on_image = [&]
    {
        h.subscriber.reset();
        destroyed.store(true);
        throw std::runtime_error("callback failed after releasing its subscriber");
    };
    h.start();
    ASSERT_TRUE(h.until([&] { h.publish(); }, [&] { return destroyed.load(); }));
    EXPECT_EQ(h.images, 1u);
}

TEST_P(RobustTransport, ANewUrlReplacesTheDecoderAndQueuedFrames)
{
    TransportHarness h(GetParam());
    auto first = StreamServer::create("first_camera", 0, 1354);
    auto second = StreamServer::create("second_camera", 0, 1354);
    first->start(VideoCodec::H264, false);
    second->start(VideoCodec::MPEG4, false);
    StreamEncoder first_encoder(VideoCodec::H264, false), second_encoder(VideoCodec::MPEG4, false);
    auto url_publisher =
        h.node->create_publisher<std_msgs::msg::String>("video/rtsp", rclcpp::QoS(1).reliable().transient_local());
    h.start(false);
    std_msgs::msg::String message;
    message.data = first->url();
    url_publisher->publish(message);
    const auto send = [&](StreamServer& server, StreamEncoder& encoder, unsigned width)
    {
        auto image = makeTestImage(width, 120, h.sequence++);
        image.header.stamp = h.node->now();
        encoder.encodeVideo(image);
        std::vector<FrameDataPtr> packets;
        while (auto packet = encoder.nextPacket())
            packets.push_back(std::move(packet));
        server.sendAccessUnit(packets);
    };
    ASSERT_TRUE(h.until([&] { send(*first, first_encoder, 160); }, [&] { return h.images >= 3; }));
    message.data = second->url();
    url_publisher->publish(message);
    ASSERT_TRUE(h.until(
        [&]
        {
            send(*first, first_encoder, 160);
            send(*second, second_encoder, 320);
        },
        [&] { return h.width == 320; }));
    const auto before = h.images.load();
    ASSERT_TRUE(h.until([&] { send(*second, second_encoder, 320); }, [&] { return h.images >= before + 3; }));
    EXPECT_EQ(h.width, 320u);
}

INSTANTIATE_TEST_SUITE_P(Bidirectional, RobustTransport, ::testing::Values(false, true),
                         [](const ::testing::TestParamInfo<bool>& info) { return info.param ? "Tcp" : "Udp"; });

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    const auto result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
