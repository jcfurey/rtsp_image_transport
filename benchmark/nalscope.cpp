/* What actually reaches the decoder after packet loss wedges it.
 *
 * Prints a per-second timeline: which H.264 NAL types the FrameExtractor
 * delivered, how many pictures came out, and how many NAL units were
 * suspiciously short. The question it answers is whether IDR NAL units keep
 * arriving after the stream goes dead — in which case the decoder is refusing
 * a perfectly good key frame — or whether they stop arriving intact, in which
 * case the fault is upstream of libavcodec.
 */
#include "frame_data.h"
#include "stream_client.h"
#include "stream_decoder.h"
#include "stream_encoder.h"
#include "stream_server.h"
#include "streaming_error.h"
#include "test_helpers.h"

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace rtsp_image_transport;
using namespace rtsp_image_transport::test;
using namespace std::chrono_literals;

namespace
{

const char* nalName(unsigned type)
{
    switch (type)
    {
        case 1: return "P";    // non-IDR slice
        case 5: return "IDR";  // key frame slice
        case 6: return "SEI";
        case 7: return "SPS";
        case 8: return "PPS";
        case 9: return "AUD";
        default: return "?";
    }
}

/* Walks the Annex B buffer the extractor delivered and reports every NAL in
   it, so a buffer carrying more than one shows up as such. */
void scanNals(const unsigned char* data, std::size_t len, std::map<std::string, unsigned>& hist,
              unsigned& nal_count, unsigned& short_nals)
{
    std::size_t i = 0;
    while (i + 4 < len)
    {
        if (data[i] == 0 && data[i + 1] == 0 && ((data[i + 2] == 1) || (data[i + 2] == 0 && data[i + 3] == 1)))
        {
            const std::size_t sc = (data[i + 2] == 1) ? 3 : 4;
            const std::size_t nal_start = i + sc;
            if (nal_start >= len)
                break;
            const unsigned type = data[nal_start] & 0x1F;
            /* Find the next start code to size this NAL */
            std::size_t j = nal_start + 1;
            while (j + 3 < len
                   && !(data[j] == 0 && data[j + 1] == 0
                        && ((data[j + 2] == 1) || (data[j + 2] == 0 && data[j + 3] == 1))))
                j++;
            const std::size_t nal_len = (j + 3 < len ? j : len) - nal_start;
            hist[nalName(type)]++;
            nal_count++;
            /* A slice this small is almost certainly a fragment that lost its
               tail rather than a real picture. */
            if ((type == 1 || type == 5) && nal_len < 32)
                short_nals++;
            i = nal_start;
        }
        else
        {
            i++;
        }
    }
}

struct Second
{
    std::map<std::string, unsigned> hist;
    unsigned nals = 0, short_nals = 0, images = 0;
};

}  // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    const int seconds = argc > 1 ? std::atoi(argv[1]) : 25;
    const unsigned width = 640, height = 480;

    std::shared_ptr<StreamServer> server = StreamServer::create("scope", 0, 1396 - 42);
    std::unique_ptr<StreamEncoder> encoder;
    try
    {
        server->start(VideoCodec::H264, false);
        encoder = std::make_unique<StreamEncoder>(VideoCodec::H264, false);
        encoder->setBitrate(2000000);
        encoder->setFramerate(30);
        encoder->setPackageSizeHint(server->maxPacketSize() - 24);
    }
    catch (const std::exception& e)
    {
        std::printf("SKIP: %s\n", e.what());
        rclcpp::shutdown();
        return 0;
    }

    std::mutex mutex;
    std::condition_variable cv;
    bool started = false, failed = false;
    std::deque<FrameDataPtr> queue;
    std::atomic_bool stop{false};
    std::vector<Second> timeline(seconds + 4);
    std::vector<std::vector<unsigned char>> captured;
    const auto t0 = std::chrono::steady_clock::now();
    auto bucket = [&]() -> std::size_t
    {
        const auto s = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - t0).count();
        return std::min<std::size_t>(timeline.size() - 1, static_cast<std::size_t>(s < 0 ? 0 : s));
    };

    std::thread worker(
        [&]
        {
            StreamDecoder::Options options;
            options.use_hw_decoder = false;
            options.hw_device = "none";
            std::unique_ptr<StreamDecoder> decoder;
            try
            {
                decoder = std::make_unique<StreamDecoder>(VideoCodec::H264, options);
            }
            catch (const std::exception&)
            {
                return;
            }
            while (!stop.load())
            {
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
                    continue;
                }
                while (decoder->nextFrame())
                {
                    std::lock_guard<std::mutex> lock{mutex};
                    timeline[bucket()].images++;
                }
            }
        });

    std::shared_ptr<StreamClient> client = StreamClient::create("scope", server->url());
    client->setRtpOverTcp(false);
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
        [&](int, const std::string&)
        {
            std::lock_guard<std::mutex> lock{mutex};
            failed = true;
            cv.notify_all();
        });
    client->setReceiveStreamDataHandler(
        [&](VideoCodec, MediaSubsession*, const FrameDataPtr& data)
        {
            std::lock_guard<std::mutex> lock{mutex};
            Second& s = timeline[bucket()];
            scanNals(data->data(), data->length(), s.hist, s.nals, s.short_nals);
            captured.emplace_back(data->data(), data->data() + data->length());
            queue.push_back(data);
        });
    client->connect();
    {
        std::unique_lock<std::mutex> lock{mutex};
        if (!cv.wait_for(lock, 30s, [&] { return started || failed; }) || failed)
        {
            stop = true;
            worker.join();
            std::printf("session did not start\n");
            rclcpp::shutdown();
            return 1;
        }
    }

    const auto interval = 33333333ns;
    auto next = std::chrono::steady_clock::now();
    const auto deadline = next + std::chrono::seconds(seconds);
    unsigned seq = 0;
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (server->hasActiveStreams())
        {
            sensor_msgs::msg::Image img = makeTestImage(width, height, seq++);
            img.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
            try
            {
                if (encoder->encodeVideo(img) > 0)
                {
                    const rclcpp::Time stamp(img.header.stamp);
                    std::vector<FrameDataPtr> access_unit;
                    while (FrameDataPtr packet = encoder->nextPacket())
                    {
                        packet->setStamp(stamp);
                        access_unit.push_back(std::move(packet));
                    }
                    server->sendAccessUnit(access_unit);
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

    std::printf("%4s %7s %7s %7s | %s\n", "sec", "images", "nals", "short", "NAL types delivered");
    std::lock_guard<std::mutex> lock{mutex};
    for (std::size_t i = 0; i < timeline.size(); ++i)
    {
        const Second& s = timeline[i];
        if (s.nals == 0 && s.images == 0)
            continue;
        std::string types;
        for (const auto& [name, count] : s.hist)
            types += name + std::string("=") + std::to_string(count) + " ";
        std::printf("%4zu %7u %7u %7u | %s\n", i, s.images, s.nals, s.short_nals, types.c_str());
    }
    /* Replay the exact bytes the live decoder was fed, into a decoder that has
       never seen anything. Same data, no accumulated state: if this produces
       pictures the live one did not, the bytes were always decodable and the
       fault is the decoder's state rather than the stream. */
    unsigned live_images = 0;
    for (const Second& s : timeline)
        live_images += s.images;
    unsigned replay_images = 0;
    {
        StreamDecoder::Options options;
        options.use_hw_decoder = false;
        options.hw_device = "none";
        StreamDecoder fresh(VideoCodec::H264, options);
        std::size_t idx = 0;
        for (const auto& buf : captured)
        {
            FrameDataPtr d = std::make_shared<FrameData>(buf.data(), buf.size(),
                                                         rclcpp::Time(BASE_STAMP_NS + 33333333LL * (idx++ / 4)));
            try
            {
                fresh.decodeVideo(d);
            }
            catch (const std::exception&)
            {
                continue;
            }
            while (fresh.nextFrame())
                replay_images++;
        }
    }
    std::printf("\ncaptured %zu delivered buffers\n", captured.size());
    std::printf("live decoder produced   %u pictures\n", live_images);
    std::printf("replay into fresh decoder produced %u pictures\n", replay_images);
    rclcpp::shutdown();
    return 0;
}
