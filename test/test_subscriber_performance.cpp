// SPDX-License-Identifier: Apache-2.0
#include "stream_decoder.h"
#include "streaming_error.h"
#include "subscriber_plugin.h"
#include "test_helpers.h"

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <utility>
#include <vector>

using namespace std::chrono_literals;
using namespace rtsp_image_transport;
using namespace rtsp_image_transport::test;

namespace rtsp_image_transport
{

class StreamDecoderTestPeer
{
  public:
    // Exercise hardware-probe decisions deterministically on machines without
    // a GPU. The real codec still decodes valid access units in these tests.
    static void emulateHardwareCandidate(StreamDecoder& decoder)
    {
        decoder.hardware_ = true;
        decoder.candidates_ = {{"h264", AV_HWDEVICE_TYPE_NONE, AV_PIX_FMT_NONE, true},
                               {"h264", AV_HWDEVICE_TYPE_NONE, AV_PIX_FMT_NONE, false}};
        decoder.candidate_index_ = 0;
    }
    static bool proven(const StreamDecoder& decoder)
    {
        return decoder.hardware_candidate_proven_;
    }
    static std::size_t retainedPackets(const StreamDecoder& decoder)
    {
        return decoder.hardware_probe_packets_.size();
    }
    static std::size_t candidate(const StreamDecoder& decoder)
    {
        return decoder.candidate_index_;
    }
    static bool fallback(StreamDecoder& decoder)
    {
        return decoder.fallBackToNextCandidate("test device failure");
    }
    static bool hasScaler(const StreamDecoder& decoder)
    {
        return bool(decoder.sws_);
    }
    static int scalerThreads(const StreamDecoder& decoder)
    {
        std::int64_t threads = -1;
        if (decoder.sws_)
            av_opt_get_int(decoder.sws_.get(), "threads", 0, &threads);
        return static_cast<int>(threads);
    }
};

class SubscriberPluginTestPeer
{
  public:
    static void attach(SubscriberPlugin& plugin, const rclcpp::Node::SharedPtr& node,
                       const std::shared_ptr<StreamDecoder>& decoder,
                       const SubscriberPlugin::Callback& callback)
    {
        plugin.param_base_name_ = "input.rtsp";
        plugin.node_param_ = node->get_node_parameters_interface();
        plugin.setupParameters(node->get_node_parameters_interface());
        plugin.updateParameters();
        plugin.param_cb_handle_ = node->add_post_set_parameters_callback(
            [&plugin](const std::vector<rclcpp::Parameter>&) { plugin.updateParameters(); });
        plugin.decoder_.store(decoder);
        plugin.callback_ = callback;
    }
    static void push(SubscriberPlugin& plugin, const FrameDataPtr& frame,
                     std::chrono::steady_clock::time_point received_at = std::chrono::steady_clock::now())
    {
        plugin.pushFrame(frame, received_at);
    }
    static FrameDataPtr pop(SubscriberPlugin& plugin)
    {
        return plugin.popFrame().data;
    }
    static void process(SubscriberPlugin& plugin)
    {
        plugin.processFrame();
    }
    static void clear(SubscriberPlugin& plugin)
    {
        plugin.clearQueuedFrames();
    }
    static std::size_t size(const SubscriberPlugin& plugin)
    {
        return plugin.queue_.size();
    }
    static std::size_t bytes(const SubscriberPlugin& plugin)
    {
        return plugin.queued_bytes_;
    }
    static std::shared_ptr<StreamDecoder> decoder(const SubscriberPlugin& plugin)
    {
        return plugin.decoder_.load();
    }
    static void notify(SubscriberPlugin& plugin, std::function<void()> callback)
    {
        plugin.notify_frame_ = std::move(callback);
    }
    static void ageStallWindow(SubscriberPlugin& plugin)
    {
        plugin.last_image_out_ = std::chrono::steady_clock::now() - 10s;
    }
    static std::size_t stalls(const SubscriberPlugin& plugin)
    {
        return plugin.decoder_stalls_;
    }
};

} // namespace rtsp_image_transport

namespace
{

StreamDecoder::Options softwareOptions()
{
    StreamDecoder::Options options;
    options.use_hw_decoder = false;
    options.hw_device = "none";
    return options;
}

FrameDataPtr accessUnit(const std::vector<std::uint8_t>& packet, std::size_t index = 0)
{
    return std::make_shared<FrameData>(
        packet.data(), packet.size(),
        rclcpp::Time(BASE_STAMP_NS + static_cast<std::int64_t>(index) * FRAME_INTERVAL_NS));
}

std::size_t feed(StreamDecoder& decoder, const std::vector<std::vector<std::uint8_t>>& packets,
                 std::size_t from, std::size_t to)
{
    std::size_t count = 0;
    for (std::size_t i = from; i < to; ++i)
    {
        const auto reported = decoder.decodeVideo(accessUnit(packets.at(i), i));
        std::size_t received = 0;
        while (decoder.nextFrame())
            ++received;
        EXPECT_EQ(reported, received);
        count += received;
    }
    return count;
}

} // namespace

TEST(DecoderPerformance, IntentionalDropsDoNotProbeOrReplayAWorkingDevice)
{
    const auto packets = encodeTestStream(VideoCodec::H264, 320, 240, 60);
    ASSERT_EQ(packets.size(), 60u);
    StreamDecoder decoder(VideoCodec::H264, softwareOptions());
    StreamDecoderTestPeer::emulateHardwareCandidate(decoder);
    ASSERT_EQ(feed(decoder, packets, 0, 20), 20u);
    ASSERT_TRUE(StreamDecoderTestPeer::proven(decoder));
    const auto* context = decoder.context();

    decoder.setDecodeFrames(StreamDecoder::DecodeFrames::None);
    EXPECT_EQ(feed(decoder, packets, 20, 40), 0u);
    EXPECT_EQ(StreamDecoderTestPeer::candidate(decoder), 0u);
    EXPECT_EQ(StreamDecoderTestPeer::retainedPackets(decoder), 0u);
    EXPECT_EQ(decoder.context(), context);
    decoder.flush();
    EXPECT_TRUE(StreamDecoderTestPeer::proven(decoder));
    decoder.setDecodeFrames(StreamDecoder::DecodeFrames::All);
    EXPECT_EQ(feed(decoder, packets, 40, 60), 20u);
}

TEST(DecoderPerformance, IntentionalDropsBeforeStartupDoNotTriggerFallback)
{
    const auto packets = encodeTestStream(VideoCodec::H264, 320, 240, 40);
    ASSERT_EQ(packets.size(), 40u);
    StreamDecoder decoder(VideoCodec::H264, softwareOptions());
    StreamDecoderTestPeer::emulateHardwareCandidate(decoder);
    decoder.setDecodeFrames(StreamDecoder::DecodeFrames::None);
    EXPECT_EQ(feed(decoder, packets, 0, 20), 0u);
    EXPECT_EQ(StreamDecoderTestPeer::candidate(decoder), 0u);
    EXPECT_EQ(StreamDecoderTestPeer::retainedPackets(decoder), 0u);
    decoder.setDecodeFrames(StreamDecoder::DecodeFrames::All);
    EXPECT_EQ(feed(decoder, packets, 20, 40), 20u);
}

TEST(DecoderPerformance, DamagedPacketsDoNotReplaceAWorkingDevice)
{
    const auto packets = encodeTestStream(VideoCodec::H264, 320, 240, 40);
    ASSERT_EQ(packets.size(), 40u);
    StreamDecoder decoder(VideoCodec::H264, softwareOptions());
    StreamDecoderTestPeer::emulateHardwareCandidate(decoder);
    ASSERT_EQ(feed(decoder, packets, 0, 20), 20u);
    const auto* context = decoder.context();
    const std::vector<std::uint8_t> damaged{0, 0, 0, 1, 0x65};
    EXPECT_THROW(decoder.decodeVideo(accessUnit(damaged, 20)), DecodingError);
    EXPECT_EQ(StreamDecoderTestPeer::candidate(decoder), 0u);
    EXPECT_EQ(decoder.context(), context);
    EXPECT_EQ(StreamDecoderTestPeer::retainedPackets(decoder), 0u);
    EXPECT_EQ(feed(decoder, packets, 20, 40), 20u);
}

TEST(DecoderPerformance, SilentStartupStillFallsBackAndReplaysInitialization)
{
    const auto packets = encodeTestStream(VideoCodec::H264, 320, 240, 4);
    ASSERT_EQ(packets.size(), 4u);
    StreamDecoder decoder(VideoCodec::H264, softwareOptions());
    StreamDecoderTestPeer::emulateHardwareCandidate(decoder);
    // Model a device that accepts packets but produces nothing although All
    // was requested. The fallback must receive the first SPS/PPS and IDR.
    decoder.context()->skip_frame = AVDISCARD_ALL;
    EXPECT_EQ(feed(decoder, packets, 0, 4), 4u);
    EXPECT_EQ(StreamDecoderTestPeer::candidate(decoder), 1u);
    EXPECT_FALSE(decoder.isHardwareAccelerated());
}

TEST(DecoderPerformance, DeviceFallbackRetainsTheRequestedDropMode)
{
    const auto packets = encodeTestStream(VideoCodec::H264, 320, 240, 20);
    ASSERT_EQ(packets.size(), 20u);
    for (const auto mode : {StreamDecoder::DecodeFrames::None, StreamDecoder::DecodeFrames::Key,
                            StreamDecoder::DecodeFrames::Intra})
    {
        StreamDecoder decoder(VideoCodec::H264, softwareOptions());
        StreamDecoderTestPeer::emulateHardwareCandidate(decoder);
        decoder.setDecodeFrames(mode);
        const auto requested = decoder.context()->skip_frame;
        ASSERT_TRUE(StreamDecoderTestPeer::fallback(decoder));
        EXPECT_EQ(decoder.context()->skip_frame, requested);
        EXPECT_EQ(feed(decoder, packets, 0, 20), mode == StreamDecoder::DecodeFrames::None ? 0u : 2u);
    }
}

TEST(DecoderPerformance, DecoderIgnoringSkipFrameIsFilteredBeforeConversion)
{
    const auto packets = encodeTestStream(VideoCodec::H264, 320, 240, 20);
    ASSERT_EQ(packets.size(), 20u);
    for (const auto mode : {StreamDecoder::DecodeFrames::None, StreamDecoder::DecodeFrames::Key,
                            StreamDecoder::DecodeFrames::Intra})
    {
        StreamDecoder decoder(VideoCodec::H264, softwareOptions());
        StreamDecoderTestPeer::emulateHardwareCandidate(decoder);
        decoder.setDecodeFrames(mode);
        decoder.context()->skip_frame = AVDISCARD_DEFAULT;
        EXPECT_EQ(feed(decoder, packets, 0, 20), mode == StreamDecoder::DecodeFrames::None ? 0u : 2u);
        EXPECT_TRUE(StreamDecoderTestPeer::proven(decoder));
        EXPECT_EQ(StreamDecoderTestPeer::candidate(decoder), 0u);
        EXPECT_EQ(StreamDecoderTestPeer::retainedPackets(decoder), 0u);
        if (mode == StreamDecoder::DecodeFrames::None)
            EXPECT_FALSE(StreamDecoderTestPeer::hasScaler(decoder));
    }
}

class GpuDropTest : public ::testing::TestWithParam<std::pair<VideoCodec, bool>>
{
};

TEST_P(GpuDropTest, DropsWithoutSwitchingDecoderAndResumes)
{
    const auto devices = StreamDecoder::availableHwDevices();
    if (std::find(devices.begin(), devices.end(), "cuda") == devices.end())
        GTEST_SKIP() << "CUDA is unavailable";
    const auto [codec, cuvid] = GetParam();
    StreamDecoder::Options options;
    options.hw_device = "cuda";
    if (cuvid)
        options.decoder = codec == VideoCodec::H264 ? "h264_cuvid" : "hevc_cuvid";
    if (cuvid && !avcodec_find_decoder_by_name(options.decoder.c_str()))
        GTEST_SKIP() << "CUVID decoder is unavailable";
    const auto packets = encodeTestStream(codec, 320, 240, 80);
    ASSERT_EQ(packets.size(), 80u);
    StreamDecoder decoder(codec, options);
    ASSERT_GE(feed(decoder, packets, 0, 20), 18u);
    ASSERT_TRUE(decoder.isHardwareAccelerated());
    const auto candidate = StreamDecoderTestPeer::candidate(decoder);
    const auto description = decoder.description();

    decoder.setDecodeFrames(StreamDecoder::DecodeFrames::None);
    EXPECT_EQ(feed(decoder, packets, 20, 40), 0u);
    decoder.setDecodeFrames(StreamDecoder::DecodeFrames::Key);
    const auto key_frames = feed(decoder, packets, 40, 60);
    EXPECT_GE(key_frames, 1u);
    EXPECT_LE(key_frames, 2u);
    decoder.setDecodeFrames(StreamDecoder::DecodeFrames::All);
    EXPECT_GE(feed(decoder, packets, 60, 80), 18u);
    const std::vector<std::uint8_t> damaged = codec == VideoCodec::H264
        ? std::vector<std::uint8_t>{0, 0, 0, 1, 0x65}
        : std::vector<std::uint8_t>{0, 0, 0, 1, 0x26, 0x01};
    try
    {
        decoder.decodeVideo(accessUnit(damaged, 80));
    }
    catch (const DecodingError&)
    {
        // Vendors can reject the packet or silently discard it.
    }
    while (decoder.nextFrame()) {}
    EXPECT_GE(feed(decoder, packets, 60, 80), 18u);
    EXPECT_EQ(StreamDecoderTestPeer::candidate(decoder), candidate);
    EXPECT_EQ(decoder.description(), description);
    EXPECT_EQ(StreamDecoderTestPeer::retainedPackets(decoder), 0u);
}

INSTANTIATE_TEST_SUITE_P(CudaAndCuvid, GpuDropTest,
                         ::testing::Values(std::make_pair(VideoCodec::H264, false),
                                           std::make_pair(VideoCodec::H265, false),
                                           std::make_pair(VideoCodec::H264, true),
                                           std::make_pair(VideoCodec::H265, true)),
                         [](const testing::TestParamInfo<GpuDropTest::ParamType>& info)
                         {
                             return std::string(info.param.first == VideoCodec::H264 ? "H264" : "H265") +
                                    (info.param.second ? "Cuvid" : "Native");
                         });

class SubscriberPerformance : public ::testing::Test
{
  protected:
    static void SetUpTestSuite()
    {
        rclcpp::init(0, nullptr);
    }
    static void TearDownTestSuite()
    {
        rclcpp::shutdown();
    }
    void SetUp() override
    {
        node = std::make_shared<rclcpp::Node>("subscriber_performance_test");
        decoder = std::make_shared<StreamDecoder>(VideoCodec::H264, softwareOptions());
        plugin = std::make_unique<SubscriberPlugin>();
        SubscriberPluginTestPeer::attach(*plugin, node, decoder,
                                         [this](const sensor_msgs::msg::Image::ConstSharedPtr& image)
                                         { images.push_back(image); });
        SubscriberPluginTestPeer::notify(*plugin, [this] { ++notifications; });
    }
    void TearDown() override
    {
        plugin->shutdown();
        plugin.reset();
        node.reset();
    }
    void drain()
    {
        for (std::size_t attempts = 0; SubscriberPluginTestPeer::size(*plugin) && attempts < 1000; ++attempts)
            SubscriberPluginTestPeer::process(*plugin);
        EXPECT_EQ(SubscriberPluginTestPeer::size(*plugin), 0u);
    }
    rclcpp::Node::SharedPtr node;
    std::shared_ptr<StreamDecoder> decoder;
    std::unique_ptr<SubscriberPlugin> plugin;
    std::vector<sensor_msgs::msg::Image::ConstSharedPtr> images;
    std::size_t notifications = 0;
};

TEST_F(SubscriberPerformance, LiveScalerThreadsChangeWithoutLosingReferenceFrames)
{
    const auto packets = encodeTestStream(VideoCodec::H264, 320, 240, 10);
    ASSERT_EQ(packets.size(), 10u);
    // Only the first packet has SPS/PPS and an IDR; a decoder restart here
    // cannot decode the remaining P pictures.
    for (std::size_t i = 0; i < 3; ++i)
        SubscriberPluginTestPeer::push(*plugin, accessUnit(packets[i], i));
    drain();
    ASSERT_EQ(images.size(), 3u);
    const auto* context = decoder->context();
#ifdef FFMPEG_HAS_SWS_THREADS
    EXPECT_EQ(StreamDecoderTestPeer::scalerThreads(*decoder), 4);
#endif
    ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("input.rtsp.sws_threads", 1)).successful);
    EXPECT_EQ(SubscriberPluginTestPeer::decoder(*plugin), decoder);
    for (std::size_t i = 3; i < packets.size(); ++i)
        SubscriberPluginTestPeer::push(*plugin, accessUnit(packets[i], i));
    drain();
    ASSERT_EQ(images.size(), packets.size());
    EXPECT_EQ(decoder->context(), context);
    EXPECT_FALSE(decoder->awaitingKeyframe());
#ifdef FFMPEG_HAS_SWS_THREADS
    EXPECT_EQ(StreamDecoderTestPeer::scalerThreads(*decoder), 1);
#endif
    StreamDecoder reference(VideoCodec::H264, softwareOptions());
    for (std::size_t i = 0; i < packets.size(); ++i)
    {
        ASSERT_EQ(reference.decodeVideo(accessUnit(packets[i], i)), 1u);
        auto expected = reference.nextFrame();
        ASSERT_TRUE(expected);
        EXPECT_EQ(images[i]->data, expected->data);
        EXPECT_EQ(images[i]->header.stamp, expected->header.stamp);
    }
}

TEST_F(SubscriberPerformance, PausedStampsDoNotHideBacklogOrTriggerTheStallWatchdog)
{
    const auto packets = encodeTestStream(VideoCodec::H264, 320, 240, 50);
    ASSERT_EQ(packets.size(), 50u);
    for (std::size_t i = 0; i < 20; ++i)
        SubscriberPluginTestPeer::push(*plugin, accessUnit(packets[i]));
    drain();
    ASSERT_EQ(images.size(), 20u);
    SubscriberPluginTestPeer::ageStallWindow(*plugin);
    for (std::size_t i = 20; i < 40; ++i)
        SubscriberPluginTestPeer::push(*plugin, accessUnit(packets[i]),
                                       std::chrono::steady_clock::now() - 8s);
    drain();
    EXPECT_EQ(images.size(), 20u);
    EXPECT_EQ(SubscriberPluginTestPeer::stalls(*plugin), 0u);
    for (std::size_t i = 40; i < 50; ++i)
        SubscriberPluginTestPeer::push(*plugin, accessUnit(packets[i]));
    drain();
    ASSERT_EQ(images.size(), 30u);
    for (const auto& image : images)
        EXPECT_EQ(rclcpp::Time(image->header.stamp).nanoseconds(), BASE_STAMP_NS);
}

TEST_F(SubscriberPerformance, AStaleSingleFrameIsDropped)
{
    const auto packets = encodeTestStream(VideoCodec::H264, 320, 240, 1);
    ASSERT_EQ(packets.size(), 1u);
    SubscriberPluginTestPeer::push(*plugin, accessUnit(packets[0]), std::chrono::steady_clock::now() - 8s);
    drain();
    EXPECT_TRUE(images.empty());
    EXPECT_FALSE(StreamDecoderTestPeer::hasScaler(*decoder));
}

TEST_F(SubscriberPerformance, QueueBoundsKeepInitializationAndTheNewestWork)
{
    const std::vector<std::uint8_t> packet(100, 0);
    for (std::size_t i = 0; i < 240; ++i)
        SubscriberPluginTestPeer::push(*plugin, accessUnit(packet, i));
    EXPECT_EQ(SubscriberPluginTestPeer::size(*plugin), 120u);
    EXPECT_EQ(SubscriberPluginTestPeer::bytes(*plugin), 12000u);
    EXPECT_EQ(SubscriberPluginTestPeer::pop(*plugin)->stamp().nanoseconds(), BASE_STAMP_NS);
    SubscriberPluginTestPeer::push(*plugin, accessUnit(packet, 240));
    SubscriberPluginTestPeer::push(*plugin, accessUnit(packet, 241));
    EXPECT_EQ(SubscriberPluginTestPeer::pop(*plugin)->stamp().nanoseconds(),
              BASE_STAMP_NS + 122 * FRAME_INTERVAL_NS);
    SubscriberPluginTestPeer::clear(*plugin);
    EXPECT_EQ(SubscriberPluginTestPeer::bytes(*plugin), 0u);
}

TEST_F(SubscriberPerformance, QueueAlsoBoundsEncodedBytes)
{
    const auto frame = accessUnit(std::vector<std::uint8_t>(4u << 20, 0));
    for (int i = 0; i < 30; ++i)
        SubscriberPluginTestPeer::push(*plugin, frame);
    EXPECT_EQ(SubscriberPluginTestPeer::size(*plugin), 16u);
    EXPECT_EQ(SubscriberPluginTestPeer::bytes(*plugin), 64u << 20);
    SubscriberPluginTestPeer::pop(*plugin);
    EXPECT_EQ(SubscriberPluginTestPeer::bytes(*plugin), 60u << 20);
}

TEST_F(SubscriberPerformance, DisablingDroppingDisablesQueueTrimming)
{
    ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("input.rtsp.max_latency", 0.0)).successful);
    const auto frame = accessUnit(std::vector<std::uint8_t>(100, 0));
    for (int i = 0; i < 240; ++i)
        SubscriberPluginTestPeer::push(*plugin, frame);
    EXPECT_EQ(SubscriberPluginTestPeer::size(*plugin), 240u);
    ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("input.rtsp.max_latency", 0.2)).successful);
    EXPECT_EQ(SubscriberPluginTestPeer::size(*plugin), 120u);
}

TEST_F(SubscriberPerformance, NoDropModeYieldsAndReschedulesUntilTheQueueIsDrained)
{
    ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("input.rtsp.max_latency", 0.0)).successful);
    const auto packets = encodeTestStream(VideoCodec::H264, 320, 240, 40);
    ASSERT_EQ(packets.size(), 40u);
    for (std::size_t i = 0; i < packets.size(); ++i)
        SubscriberPluginTestPeer::push(*plugin, accessUnit(packets[i], i),
                                       std::chrono::steady_clock::now() - 8s);
    SubscriberPluginTestPeer::process(*plugin);
    EXPECT_GE(SubscriberPluginTestPeer::size(*plugin), 24u);
    EXPECT_GT(images.size(), 0u);
    EXPECT_EQ(notifications, 1u);
    drain();
    EXPECT_EQ(images.size(), 40u);
    EXPECT_GE(notifications, 2u);
}
