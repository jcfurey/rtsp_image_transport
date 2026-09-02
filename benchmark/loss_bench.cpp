/* RTP over UDP vs interleaved over TCP, on a link that loses packets.
 *
 * No ROS graph: StreamServer and StreamClient talk to each other directly, so
 * every packet crossing lo is RTSP control or RTP and the loss injected with
 * iptables hits the stream and nothing else.
 *
 * Latency is measured at the transport, as now() minus the presentation stamp
 * the sender put on the access unit, so it is the delay the transport added
 * and not the decoder competing for CPU. The whole point of the comparison is
 * what each transport does when a packet goes missing: UDP hands over what
 * arrived and keeps going, TCP holds everything behind the gap until the
 * retransmission lands.
 */
#include "frame_data.h"
#include "stream_client.h"
#include "stream_encoder.h"
#include "stream_server.h"
#include "streaming_error.h"
#include "test_helpers.h"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace rtsp_image_transport;
using namespace rtsp_image_transport::test;
using namespace std::chrono_literals;

namespace
{

struct Result
{
    std::size_t nals = 0;
    double p50 = 0, p90 = 0, p99 = 0, max = 0;
    double first_quarter = 0, last_quarter = 0;
    bool failed = false;
    std::string failure;
};

double pct(const std::vector<double>& sorted, double q)
{
    if (sorted.empty())
        return 0;
    return sorted[std::min(sorted.size() - 1, static_cast<std::size_t>(sorted.size() * q))];
}

Result run(bool rtp_over_tcp, unsigned width, unsigned height, unsigned bitrate, int seconds)
{
    Result r;
    std::shared_ptr<StreamServer> server = StreamServer::create("bench", 0, 1396 - 42);
    server->start(VideoCodec::H264, false);

    std::unique_ptr<StreamEncoder> encoder;
    try
    {
        encoder = std::make_unique<StreamEncoder>(VideoCodec::H264, false);
        encoder->setBitrate(bitrate);
        encoder->setFramerate(30);
        encoder->setPackageSizeHint(server->maxPacketSize() - 24);
    }
    catch (const StreamingError& e)
    {
        r.failed = true;
        r.failure = e.what();
        return r;
    }

    std::mutex mutex;
    std::condition_variable cv;
    bool started = false, failed = false;
    std::string failure;
    std::vector<double> latencies;
    rclcpp::Clock wall(RCL_SYSTEM_TIME);

    std::shared_ptr<StreamClient> client = StreamClient::create("bench", server->url());
    client->setRtpOverTcp(rtp_over_tcp);
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
            const double ms = (wall.now().nanoseconds() - data->stamp().nanoseconds()) / 1e6;
            std::lock_guard<std::mutex> lock{mutex};
            latencies.push_back(ms);
        });
    client->connect();
    {
        std::unique_lock<std::mutex> lock{mutex};
        if (!cv.wait_for(lock, 30s, [&] { return started || failed; }))
        {
            r.failed = true;
            r.failure = "session never started";
            return r;
        }
        if (failed)
        {
            r.failed = true;
            r.failure = failure;
            return r;
        }
    }

    /* Feed the encoder at 30 Hz with wall clock stamps, exactly as the
       publisher plugin does for a camera running on real time. */
    const auto interval = 33333333ns;
    auto next = std::chrono::steady_clock::now();
    const auto deadline = next + std::chrono::seconds(seconds);
    unsigned seq = 0;
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
    std::this_thread::sleep_for(1s);
    client->disconnect();

    std::vector<double> all;
    {
        std::lock_guard<std::mutex> lock{mutex};
        all = latencies;
    }
    /* Drop the first second: RTSP setup and the client anchoring its clock */
    if (all.size() > 60)
        all.erase(all.begin(), all.begin() + 60);
    r.nals = all.size();
    if (all.size() >= 8)
    {
        const std::size_t q = all.size() / 4;
        for (std::size_t i = 0; i < q; ++i)
            r.first_quarter += all[i];
        for (std::size_t i = all.size() - q; i < all.size(); ++i)
            r.last_quarter += all[i];
        r.first_quarter /= q;
        r.last_quarter /= q;
    }
    std::sort(all.begin(), all.end());
    r.p50 = pct(all, 0.50);
    r.p90 = pct(all, 0.90);
    r.p99 = pct(all, 0.99);
    r.max = all.empty() ? 0 : all.back();
    return r;
}

}  // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    const bool tcp = argc > 1 && std::string(argv[1]) == "tcp";
    const int seconds = argc > 2 ? std::atoi(argv[2]) : 20;
    const unsigned width = argc > 3 ? std::atoi(argv[3]) : 640;
    const unsigned height = argc > 4 ? std::atoi(argv[4]) : 480;
    const unsigned bitrate = argc > 5 ? std::atoi(argv[5]) : 2000000;
    const char* label = argc > 6 ? argv[6] : "";

    const Result r = run(tcp, width, height, bitrate, seconds);
    if (r.failed)
    {
        std::printf("%-22s %-4s FAILED: %s\n", label, tcp ? "TCP" : "UDP", r.failure.c_str());
        rclcpp::shutdown();
        return 1;
    }
    std::printf("%-22s %-4s nals %5zu | p50 %7.1f  p90 %7.1f  p99 %7.1f  max %8.1f ms"
                " | first quarter %7.1f -> last %7.1f (%+.1f)\n",
                label, tcp ? "TCP" : "UDP", r.nals, r.p50, r.p90, r.p99, r.max, r.first_quarter,
                r.last_quarter, r.last_quarter - r.first_quarter);
    rclcpp::shutdown();
    return 0;
}
