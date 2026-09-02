/* Deterministic reproduction of the decoder wedge, with no network involved.
 *
 * Encodes a clip with the publisher's own encoder settings, hands the NAL
 * units to the decoder the way FrameExtractor does — one per call, Annex B
 * framed — and drops exactly one of them. Then it counts the pictures that
 * come out afterwards.
 *
 * If dropping a single NAL unit is enough to stop the decoder permanently,
 * with key frames still arriving every 30 frames, the fault is in how the
 * stream is fed to libavcodec and not in anything the network did.
 */
#include "frame_data.h"
#include "stream_decoder.h"
#include "stream_encoder.h"
#include "streaming_error.h"
#include "test_helpers.h"

#include <rclcpp/rclcpp.hpp>

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

using namespace rtsp_image_transport;
using namespace rtsp_image_transport::test;

namespace
{

constexpr unsigned char START_CODE[4] = {0x00, 0x00, 0x00, 0x01};

struct Nal
{
    std::vector<unsigned char> annexb;  // start code + payload, as delivered
    unsigned type = 0;
    unsigned frame = 0;
};

const char* nalName(unsigned t)
{
    switch (t)
    {
        case 1: return "P";
        case 5: return "IDR";
        case 6: return "SEI";
        case 7: return "SPS";
        case 8: return "PPS";
        case 9: return "AUD";
        default: return "?";
    }
}

/* The exact stream the publisher would put on the wire: same encoder, same
   slice-max-size derived from udp_packet_size. */
std::vector<Nal> encodeStream(unsigned frames)
{
    std::vector<Nal> out;
    auto encoder = std::make_unique<StreamEncoder>(VideoCodec::H264, /*use_hw_encoder=*/false);
    encoder->setBitrate(2000000);
    encoder->setFramerate(30);
    encoder->setPackageSizeHint(1354 - 24);
    for (unsigned i = 0; i < frames; ++i)
    {
        sensor_msgs::msg::Image img = makeTestImage(640, 480, i);
        if (encoder->encodeVideo(img) == 0)
            continue;
        while (FrameDataPtr packet = encoder->nextPacket())
        {
            if (packet->length() == 0)
                continue;
            Nal nal;
            nal.annexb.insert(nal.annexb.end(), START_CODE, START_CODE + 4);
            nal.annexb.insert(nal.annexb.end(), packet->data(), packet->data() + packet->length());
            nal.type = packet->data()[0] & 0x1F;
            nal.frame = i;
            out.push_back(std::move(nal));
        }
    }
    return out;
}

/* Feeds the stream with one NAL unit removed. `before` counts pictures decoded
   up to the drop; `tail` counts those decoded over the final quarter of the
   stream, which is the part that says whether the decoder ever came back —
   several key frames after the damage, with no dependence on how much stream
   happened to be left. */
void runWithDrop(const std::vector<Nal>& stream, long drop_index, unsigned& before, unsigned& tail,
                 bool truncate_instead = false)
{
    StreamDecoder::Options options;
    options.use_hw_decoder = false;
    options.hw_device = "none";
    StreamDecoder decoder(VideoCodec::H264, options);
    before = tail = 0;
    const std::size_t tail_start = stream.size() * 3 / 4;
    for (std::size_t i = 0; i < stream.size(); ++i)
    {
        if (static_cast<long>(i) == drop_index && !truncate_instead)
            continue;
        /* Truncation is what an RTP fragment loss actually produces: live555
           reassembles an FU-A from several packets, and if one goes missing the
           NAL unit that reaches the extractor is short. Dropping a whole NAL
           unit models the single-packet case; this models the fragmented one. */
        std::vector<unsigned char> damaged;
        const unsigned char* bytes = stream[i].annexb.data();
        std::size_t length = stream[i].annexb.size();
        if (static_cast<long>(i) == drop_index && truncate_instead)
        {
            if (length < 16)
                continue;
            damaged.assign(stream[i].annexb.begin(), stream[i].annexb.begin() + length / 2);
            bytes = damaged.data();
            length = damaged.size();
        }
        FrameDataPtr data = std::make_shared<FrameData>(bytes, length,
                                                        rclcpp::Time(BASE_STAMP_NS + 33333333LL * stream[i].frame));
        try
        {
            decoder.decodeVideo(data);
        }
        catch (const std::exception&)
        {
            continue;
        }
        while (decoder.nextFrame())
        {
            if (i >= tail_start)
                tail++;
            else if (drop_index < 0 || static_cast<long>(i) <= drop_index)
                before++;
        }
    }
}

}  // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    const unsigned frames = argc > 1 ? std::atoi(argv[1]) : 120;

    std::vector<Nal> stream;
    try
    {
        stream = encodeStream(frames);
    }
    catch (const StreamingError& e)
    {
        std::printf("SKIP: %s\n", e.what());
        rclcpp::shutdown();
        return 0;
    }
    if (stream.empty())
    {
        std::printf("SKIP: encoder produced nothing\n");
        rclcpp::shutdown();
        return 0;
    }

    unsigned base_before = 0, base_tail = 0;
    runWithDrop(stream, -1, base_before, base_tail);
    std::printf("%u frames -> %zu NAL units; undamaged run decodes %u pictures in the final quarter\n\n", frames,
                stream.size(), base_tail);
    if (base_tail == 0)
    {
        std::printf("baseline produced nothing; cannot judge recovery\n");
        rclcpp::shutdown();
        return 1;
    }
    std::printf("Dropping one NAL unit in the first half; the final quarter of the stream is several key\n"
                "frames later, so anything still empty there never recovered.\n\n");

    const long half = static_cast<long>(stream.size()) / 2;
    const long step = std::max<long>(1, half / 40);
    for (bool truncate : {false, true})
    {
        std::printf("=== damage: %s ===\n", truncate ? "NAL unit truncated to half its length (lost RTP fragment)"
                                                     : "whole NAL unit missing (lost RTP packet)");
        unsigned fatal = 0, tested = 0;
        std::map<std::string, std::pair<unsigned, unsigned>> by_type;
        for (long i = 0; i < half; i += step)
        {
            unsigned before = 0, tail = 0;
            runWithDrop(stream, i, before, tail, truncate);
            tested++;
            const bool dead = tail * 4 < base_tail;  // under a quarter of what it should have
            auto& counts = by_type[nalName(stream[i].type)];
            counts.second++;
            if (dead)
            {
                fatal++;
                counts.first++;
            }
        }
        std::printf("%u of %u killed the stream permanently\n", fatal, tested);
        for (const auto& [name, counts] : by_type)
            std::printf("   %-4s %u of %u fatal\n", name.c_str(), counts.first, counts.second);
        std::printf("\n");
    }
    rclcpp::shutdown();
    return 0;
}
