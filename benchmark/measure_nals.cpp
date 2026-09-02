/* Counts NAL units per picture at the settings ros_to_rtsp_h264.launch.yaml
   uses, which is what the old per-NAL-unit clock mapping turned into drift:
   each NAL unit beyond the first cost MIN_STEP_NS (1 ms) of extra timeline. */
#include "stream_encoder.h"
#include "streaming_error.h"
#include "test_helpers.h"

#include <rclcpp/rclcpp.hpp>

#include <cstdio>
#include <vector>

using namespace rtsp_image_transport;
using namespace rtsp_image_transport::test;

namespace
{
constexpr std::int64_t MIN_STEP_MS = 1;  // StreamClock::MIN_STEP_NS
constexpr double FRAME_MS = 1000.0 / 30.0;

void run(const char* label, unsigned w, unsigned h, unsigned bitrate, unsigned packet_hint)
{
    std::unique_ptr<StreamEncoder> encoder;
    try
    {
        encoder = std::make_unique<StreamEncoder>(VideoCodec::H264, /*use_hw_encoder=*/false);
        encoder->setBitrate(bitrate);
        encoder->setFramerate(30);
        if (packet_hint)
            encoder->setPackageSizeHint(packet_hint);
    }
    catch (const StreamingError& e)
    {
        std::printf("%-28s skipped: %s\n", label, e.what());
        return;
    }
    std::size_t total_nals = 0, frames = 0, max_nals = 0;
    for (unsigned i = 0; i < 60; ++i)
    {
        std::size_t n = encoder->encodeVideo(makeTestImage(w, h, i));
        while (encoder->nextPacket())
        {
        }
        if (n == 0)
            continue;
        total_nals += n;
        max_nals = std::max(max_nals, n);
        frames++;
    }
    const double per_frame = static_cast<double>(total_nals) / frames;
    /* Every NAL unit after the first advanced the old timeline by 1 ms */
    const double drift_ms_per_frame = (per_frame - 1) * MIN_STEP_MS;
    const double drift_ratio = drift_ms_per_frame / FRAME_MS;
    std::printf("%-28s %5.1f NAL/frame (max %3zu)  drift %5.1f ms/frame = %5.1f%% of real time"
                "  -> %5.1f s adrift after 2 min\n",
                label, per_frame, max_nals, drift_ms_per_frame, 100.0 * drift_ratio, 120.0 * drift_ratio);
}
}  // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    /* udp_packet_size 1396 -> maxPacketSize 1354 -> hint 1330, as the plugin does */
    std::printf("H.264 software encoder, 30 fps, slice-max-size from udp_packet_size=1396\n\n");
    run("1080p 8 Mbit/s (launch)", 1920, 1080, 8000000, 1330);
    run("1080p 8 Mbit/s, no hint", 1920, 1080, 8000000, 0);
    run("720p 4 Mbit/s", 1280, 720, 4000000, 1330);
    run("320x240 1 Mbit/s (test)", 320, 240, 1000000, 1330);
    rclcpp::shutdown();
    return 0;
}
