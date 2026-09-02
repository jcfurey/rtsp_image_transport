/* End-to-end latency of one rtsp transport hop, in process.
 *
 * Publishes images stamped with the wall clock and measures how long each one
 * takes to come back out of the subscriber. On a wall-clock node the default
 * timestamp_source is the sender's clock and StreamClock is a pass-through, so
 * the stamp on a decoded image is the instant the image was published and
 * now() - stamp is the whole encode -> RTSP -> decode path.
 */
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

namespace
{

sensor_msgs::msg::Image makeImage(unsigned width, unsigned height, unsigned seq)
{
    sensor_msgs::msg::Image img;
    img.width = width;
    img.height = height;
    img.encoding = sensor_msgs::image_encodings::BGR8;
    img.is_bigendian = false;
    img.step = 3 * width;
    img.data.resize(static_cast<std::size_t>(img.step) * height);
    /* Moving detail, so the encoder has real work to do rather than emitting
       skip macroblocks for a static scene */
    for (unsigned y = 0; y < height; ++y)
    {
        std::uint8_t* row = img.data.data() + static_cast<std::size_t>(y) * img.step;
        for (unsigned x = 0; x < width; ++x)
        {
            const unsigned v = (x * 7 + y * 3 + seq * 11) ^ ((x >> 2) * (y >> 2) + seq * 5);
            row[x * 3 + 0] = static_cast<std::uint8_t>(v);
            row[x * 3 + 1] = static_cast<std::uint8_t>(v >> 2);
            row[x * 3 + 2] = static_cast<std::uint8_t>(v >> 4);
        }
    }
    return img;
}

struct Stats
{
    double p50 = 0, p90 = 0, p99 = 0, mean = 0, min = 0, max = 0;
    std::size_t n = 0;
};

Stats summarize(std::vector<double> v)
{
    Stats s;
    if (v.empty())
        return s;
    std::sort(v.begin(), v.end());
    s.n = v.size();
    s.min = v.front();
    s.max = v.back();
    s.p50 = v[v.size() / 2];
    s.p90 = v[std::min(v.size() - 1, v.size() * 90 / 100)];
    s.p99 = v[std::min(v.size() - 1, v.size() * 99 / 100)];
    for (double d : v)
        s.mean += d;
    s.mean /= v.size();
    return s;
}

}  // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    const unsigned width = argc > 1 ? std::atoi(argv[1]) : 1280;
    const unsigned height = argc > 2 ? std::atoi(argv[2]) : 720;
    const unsigned fps = argc > 3 ? std::atoi(argv[3]) : 30;
    const unsigned frames = argc > 4 ? std::atoi(argv[4]) : 300;
    const char* label = argc > 5 ? argv[5] : "baseline";

    rclcpp::NodeOptions options;
    auto node = std::make_shared<rclcpp::Node>("rtsp_latency_bench", options);

    std::mutex mutex;
    std::vector<double> latencies_ms;
    std::vector<std::int64_t> arrival_ns;
    rclcpp::Clock wall(RCL_SYSTEM_TIME);

    auto collect = [&](const sensor_msgs::msg::Image::ConstSharedPtr& img)
    {
        const std::int64_t now = wall.now().nanoseconds();
        const std::int64_t stamp = rclcpp::Time(img->header.stamp).nanoseconds();
        std::lock_guard<std::mutex> lock{mutex};
        latencies_ms.push_back((now - stamp) / 1e6);
        arrival_ns.push_back(now);
    };

    image_transport::Publisher pub;
    image_transport::Subscriber sub;
#if defined(RTSP_BENCH_LEGACY_API)
    pub = image_transport::create_publisher(node.get(), "bench/image");
    sub = image_transport::create_subscription(node.get(), "bench/image", collect, "rtsp");
#else
    pub = image_transport::create_publisher(node.get(), "bench/image");
    sub = image_transport::create_subscription(node.get(), "bench/image", collect, "rtsp");
#endif

    const auto interval = std::chrono::nanoseconds(1000000000ull / fps);
    auto next = std::chrono::steady_clock::now();
    for (unsigned i = 0; i < frames; ++i)
    {
        sensor_msgs::msg::Image img = makeImage(width, height, i);
        img.header.frame_id = "camera";
        img.header.stamp = wall.now();
        pub.publish(img);
        next += interval;
        while (std::chrono::steady_clock::now() < next)
        {
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(1ms);
        }
    }
    /* Let the tail drain */
    const auto drain = std::chrono::steady_clock::now() + 3s;
    while (std::chrono::steady_clock::now() < drain)
    {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(2ms);
    }

    std::vector<double> all;
    std::size_t received;
    {
        std::lock_guard<std::mutex> lock{mutex};
        all = latencies_ms;
        received = latencies_ms.size();
    }
    /* The first frames include RTSP setup and the client's clock anchoring */
    std::vector<double> warm(all.begin() + std::min<std::size_t>(all.size(), 30), all.end());
    const Stats s = summarize(warm);
    /* Drift: does latency grow over the run? Compare first and last quarter. */
    double first_q = 0, last_q = 0;
    if (warm.size() >= 8)
    {
        const std::size_t q = warm.size() / 4;
        for (std::size_t i = 0; i < q; ++i)
            first_q += warm[i];
        for (std::size_t i = warm.size() - q; i < warm.size(); ++i)
            last_q += warm[i];
        first_q /= q;
        last_q /= q;
    }

    std::printf("%-22s %ux%u@%u  sent %u  recv %zu | p50 %6.1f  p90 %6.1f  p99 %6.1f  max %7.1f ms"
                " | first quarter %6.1f -> last %6.1f (growth %+.1f ms)\n",
                label, width, height, fps, frames, received, s.p50, s.p90, s.p99, s.max, first_q, last_q,
                last_q - first_q);
    rclcpp::shutdown();
    return 0;
}
