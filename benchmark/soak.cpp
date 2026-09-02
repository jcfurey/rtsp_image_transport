/* Soak both RTP transports on a lossy link and check what comes out.
 *
 * The earlier bench stopped at NAL arrival. This one runs the receive path the
 * way the subscriber does — queue on the Live555 thread, decode on a worker —
 * so the question it answers is whether usable pictures keep coming out over a
 * sustained run, per transport and per codec, and whether latency stays put or
 * walks.
 */
#include "frame_data.h"
#include "stream_client.h"
#include "stream_decoder.h"
#include "stream_encoder.h"
#include "stream_server.h"
#include "streaming_error.h"
#include "test_helpers.h"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace rtsp_image_transport;
using namespace rtsp_image_transport::test;
using namespace std::chrono_literals;

namespace
{

double pct(const std::vector<double>& sorted, double q)
{
    if (sorted.empty())
        return 0;
    return sorted[std::min(sorted.size() - 1, static_cast<std::size_t>(sorted.size() * q))];
}

}  // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    const bool tcp = argc > 1 && std::string(argv[1]) == "tcp";
    const VideoCodec codec = (argc > 2 && std::string(argv[2]) == "h265") ? VideoCodec::H265 : VideoCodec::H264;
    const int seconds = argc > 3 ? std::atoi(argv[3]) : 30;
    const char* label = argc > 4 ? argv[4] : "";
    const unsigned width = 640, height = 480;

    std::shared_ptr<StreamServer> server = StreamServer::create("soak", 0, 1396 - 42);
    std::unique_ptr<StreamEncoder> encoder;
    try
    {
        server->start(codec, false);
        encoder = std::make_unique<StreamEncoder>(codec, false);
        encoder->setBitrate(2000000);
        encoder->setFramerate(30);
        if (!std::getenv("RTSP_BENCH_NO_SLICE"))
            encoder->setPackageSizeHint(server->maxPacketSize() - 24);
    }
    catch (const std::exception& e)
    {
        std::printf("%-18s %-4s %-5s SKIP: %s\n", label, tcp ? "TCP" : "UDP",
                    codec == VideoCodec::H265 ? "H265" : "H264", e.what());
        rclcpp::shutdown();
        return 0;
    }

    std::mutex mutex;
    std::condition_variable cv;
    bool started = false, failed = false;
    std::string failure;
    std::deque<FrameDataPtr> queue;
    std::atomic_bool stop{false};
    std::size_t nals = 0;
    std::vector<double> image_latency;
    std::size_t images = 0, decode_errors = 0, decoder_resets = 0;
    rclcpp::Clock wall(RCL_SYSTEM_TIME);

    /* Decode off the Live555 thread, as the subscriber plugin does; decoding
       inline would charge the transport for the decoder's CPU time. */
    std::thread worker(
        [&]
        {
            StreamDecoder::Options options;
            options.use_hw_decoder = false;
            options.hw_device = "none";
            options.low_latency = true;
            std::unique_ptr<StreamDecoder> decoder;
            try
            {
                decoder = std::make_unique<StreamDecoder>(codec, options);
            }
            catch (const std::exception&)
            {
                return;
            }
            auto last_image = std::chrono::steady_clock::now();
            std::size_t resets = 0;
            const bool watchdog = std::getenv("RTSP_BENCH_DECODER_WATCHDOG") != nullptr;
            while (!stop.load())
            {
                if (watchdog && std::chrono::steady_clock::now() - last_image > 1s)
                {
                    try
                    {
                        decoder = std::make_unique<StreamDecoder>(codec, options);
                        resets++;
                        std::lock_guard<std::mutex> lock{mutex};
                        decoder_resets = resets;
                    }
                    catch (const std::exception&)
                    {
                    }
                    last_image = std::chrono::steady_clock::now();
                }
                FrameDataPtr frame;
                {
                    std::unique_lock<std::mutex> lock{mutex};
                    if (queue.empty())
                    {
                        lock.unlock();
                        std::this_thread::sleep_for(2ms);
                        continue;
                    }
                    frame = queue.front();
                    queue.pop_front();
                }
                try
                {
                    decoder->decodeVideo(frame);
                }
                catch (const std::exception&)
                {
                    std::lock_guard<std::mutex> lock{mutex};
                    decode_errors++;
                    continue;
                }
                while (sensor_msgs::msg::Image::UniquePtr img = decoder->nextFrame())
                {
                    const double ms =
                        (wall.now().nanoseconds() - rclcpp::Time(img->header.stamp).nanoseconds()) / 1e6;
                    last_image = std::chrono::steady_clock::now();
                    std::lock_guard<std::mutex> lock{mutex};
                    images++;
                    image_latency.push_back(ms);
                }
            }
        });

    std::shared_ptr<StreamClient> client = StreamClient::create("soak", server->url());
    client->setRtpOverTcp(tcp);
    client->setRtpBufferSize(2u * 1024u * 1024u);
    client->setSessionTimeout(0ms);
    client->setSessionStartedHandler(
        [&]
        {
            std::lock_guard<std::mutex> lock{mutex};
            started = true;
            cv.notify_all();
        });
    client->setSessionFailedHandler(
        [&](int code, const std::string& message)
        {
            std::lock_guard<std::mutex> lock{mutex};
            failed = true;
            failure = message + " (" + std::to_string(code) + ")";
            cv.notify_all();
        });
    client->setReceiveStreamDataHandler(
        [&](VideoCodec, MediaSubsession*, const FrameDataPtr& data)
        {
            std::lock_guard<std::mutex> lock{mutex};
            nals++;
            queue.push_back(data);
        });
    client->connect();
    {
        std::unique_lock<std::mutex> lock{mutex};
        if (!cv.wait_for(lock, 30s, [&] { return started || failed; }) || failed)
        {
            stop = true;
            worker.join();
            std::printf("%-18s %-4s %-5s FAILED: %s\n", label, tcp ? "TCP" : "UDP",
                        codec == VideoCodec::H265 ? "H265" : "H264",
                        failed ? failure.c_str() : "session never started");
            rclcpp::shutdown();
            return 1;
        }
    }

    const auto interval = 33333333ns;
    auto next = std::chrono::steady_clock::now();
    const auto deadline = next + std::chrono::seconds(seconds);
    unsigned seq = 0, sent = 0;
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (server->hasActiveStreams())
        {
            sensor_msgs::msg::Image img = makeTestImage(width, height, seq++);
            img.header.stamp = wall.now();
            try
            {
                if (encoder->encodeVideo(img) > 0)
                {
                    const rclcpp::Time stamp(img.header.stamp);
                    while (FrameDataPtr packet = encoder->nextPacket())
                    {
                        packet->setStamp(stamp);
                        server->sendFrame(packet);
                    }
                    sent++;
                }
            }
            catch (const std::exception&)
            {
                break;
            }
        }
        next += interval;
        std::this_thread::sleep_until(next);
    }
    std::this_thread::sleep_for(2s);
    client->disconnect();
    stop = true;
    worker.join();

    std::vector<double> lat;
    std::size_t got_images, errors, got_nals;
    {
        std::lock_guard<std::mutex> lock{mutex};
        lat = image_latency;
        got_images = images;
        errors = decode_errors;
        got_nals = nals;
    }
    /* Skip the first second: RTSP setup, and the pictures before the first key
       frame are held back by design. */
    if (lat.size() > 30)
        lat.erase(lat.begin(), lat.begin() + 30);
    double first_q = 0, last_q = 0;
    if (lat.size() >= 8)
    {
        const std::size_t q = lat.size() / 4;
        for (std::size_t i = 0; i < q; ++i)
            first_q += lat[i];
        for (std::size_t i = lat.size() - q; i < lat.size(); ++i)
            last_q += lat[i];
        first_q /= q;
        last_q /= q;
    }
    std::vector<double> sorted = lat;
    std::sort(sorted.begin(), sorted.end());

    std::printf("%-18s %-4s %-5s sent %4u  nals %5zu  images %4zu (%5.1f%%)  errs %3zu | "
                "p50 %6.1f p90 %6.1f max %7.1f ms | drift %+7.1f ms | resets %zu\n",
                label, tcp ? "TCP" : "UDP", codec == VideoCodec::H265 ? "H265" : "H264", sent, got_nals,
                got_images, sent ? 100.0 * got_images / sent : 0.0, errors, pct(sorted, 0.50),
                pct(sorted, 0.90), sorted.empty() ? 0.0 : sorted.back(), last_q - first_q, decoder_resets);
    rclcpp::shutdown();
    return 0;
}
