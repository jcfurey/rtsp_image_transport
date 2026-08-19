/****************************************************************************
 *
 * rtsp_image_transport
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

/* Contract tests for the two halves of H.265 slice-loss detection.
 *
 * libavcodec has no error resilience for H.265, so a picture that loses a slice
 * is neither repaired nor reported. prefillPlanes() marks every freshly
 * allocated buffer, and unwrittenFraction() measures how much of that marker
 * survived decoding; above 5% the caller drops the frame instead of publishing
 * a green band. Both are hand-vectorised, so their edge cases -- unaligned
 * heads, short rows, odd chroma subsampling, planes that must not be touched at
 * all -- are where the danger lives, and are what these sweeps cover.
 *
 * unwrittenFraction() is checked against a deliberately plain model written
 * below for clarity rather than speed. prefillPlanes() is checked in both
 * directions: every byte that must be written, and a poisoned guard band behind
 * every plane that must survive untouched.
 */
#include "frame_fill.h"

#include <gtest/gtest.h>

#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

extern "C"
{
#include <libavutil/imgutils.h>
}

namespace rtsp_image_transport
{
namespace
{

constexpr unsigned char POISON = 0xAB;
constexpr std::size_t GUARD = 256;

int planeHeight(const AVPixFmtDescriptor* desc, int height, int plane)
{
    return plane == 0 ? height : AV_CEIL_RSHIFT(height, desc->log2_chroma_h);
}

/* Deterministic, so a failure reproduces exactly. */
struct Rng
{
    std::uint64_t s = 0x9E3779B97F4A7C15ull;
    unsigned char byte()
    {
        s ^= s << 13;
        s ^= s >> 7;
        s ^= s << 17;
        return static_cast<unsigned char>(s >> 24);
    }
};

/* What unwrittenFraction() is specified to compute. Written to be obviously
   correct; the shipping implementation is the fast one. */
double modelUnwrittenFraction(const AVFrame* frame)
{
    if (!frame->data[0] || frame->width <= 0 || frame->height <= 0)
        return 0.0;
    const AVPixFmtDescriptor* desc = av_pix_fmt_desc_get(static_cast<AVPixelFormat>(frame->format));
    if (!desc || (desc->flags & AV_PIX_FMT_FLAG_HWACCEL) != 0 || frame->linesize[0] <= 0)
        return 0.0;
    const bool planar_chroma = frame->data[1] && frame->data[2] && desc->nb_components >= 3
                               && frame->linesize[1] > 0 && frame->linesize[2] > 0;
    std::size_t sampled = 0, unwritten = 0;
    for (int y = 0; y < frame->height; y += 8)
    {
        for (int x = 0; x < frame->width; x += 8)
        {
            ++sampled;
            if (frame->data[0][static_cast<std::ptrdiff_t>(y) * frame->linesize[0] + x]
                != frame_fill::FRAME_FILL_LUMA)
                continue;
            if (planar_chroma)
            {
                const int cx = x >> desc->log2_chroma_w;
                const int cy = y >> desc->log2_chroma_h;
                if (frame->data[1][static_cast<std::ptrdiff_t>(cy) * frame->linesize[1] + cx]
                        != frame_fill::FRAME_FILL_CHROMA
                    || frame->data[2][static_cast<std::ptrdiff_t>(cy) * frame->linesize[2] + cx]
                           != frame_fill::FRAME_FILL_CHROMA)
                    continue;
            }
            ++unwritten;
        }
    }
    return sampled > 0 ? static_cast<double>(unwritten) / static_cast<double>(sampled) : 0.0;
}

/* A frame whose planes are allocated separately, each followed by a guard band,
   so an overrun lands somewhere we can see rather than in a neighbour. */
class TestFrame
{
public:
    AVFrame frame{};

    bool build(AVPixelFormat format, int width, int height, int align, int extra_pad)
    {
        const AVPixFmtDescriptor* desc = av_pix_fmt_desc_get(format);
        const int planes = av_pix_fmt_count_planes(format);
        if (!desc || planes <= 0)
            return false;
        frame.format = format;
        frame.width = width;
        frame.height = height;
        for (int p = 0; p < planes; ++p)
        {
            const int minimum = av_image_get_linesize(format, width, p);
            if (minimum <= 0)
                return false;
            const int stride = ((minimum + align - 1) / align) * align + extra_pad;
            const std::size_t fill =
                static_cast<std::size_t>(stride) * planeHeight(desc, height, p);
            blocks_.emplace_back(fill + GUARD, POISON);
            fill_bytes_.push_back(fill);
            frame.data[p] = blocks_.back().data();
            frame.linesize[p] = stride;
        }
        return true;
    }

    void poison()
    {
        for (auto& b : blocks_)
            std::fill(b.begin(), b.end(), POISON);
    }

    std::size_t planes() const { return blocks_.size(); }
    std::size_t fillBytes(std::size_t p) const { return fill_bytes_[p]; }
    const std::vector<unsigned char>& block(std::size_t p) const { return blocks_[p]; }

    /* Plane p entirely set to `value`, without disturbing the guard band. */
    void setPlane(std::size_t p, unsigned char value)
    {
        std::memset(blocks_[p].data(), value, fill_bytes_[p]);
    }

    void fillMarked() { for (std::size_t p = 0; p < planes(); ++p) setPlane(p, p == 0 ? 1 : 128); }

    /* Legal-range picture content, which can never equal the luma marker. */
    void fillPicture(Rng& rng)
    {
        for (std::size_t p = 0; p < planes(); ++p)
            for (std::size_t i = 0; i < fill_bytes_[p]; ++i)
                blocks_[p][i] = static_cast<unsigned char>(16 + (rng.byte() % 220));
    }

private:
    std::vector<std::vector<unsigned char>> blocks_;
    std::vector<std::size_t> fill_bytes_;
};

const AVPixelFormat PLANAR[] = {AV_PIX_FMT_YUV420P, AV_PIX_FMT_YUV422P, AV_PIX_FMT_YUV444P,
                                AV_PIX_FMT_YUV410P, AV_PIX_FMT_YUV440P};

/* Sizes chosen around the 8-pixel sampling grid and the 32-byte vector step:
   exact multiples, one either side, primes, and frames smaller than one step. */
const int DIMS[][2] = {{1, 1},    {3, 5},     {7, 7},     {8, 8},   {9, 9},   {16, 16}, {17, 23},
                       {31, 29},  {64, 48},   {65, 49},   {320, 240}, {641, 481}, {1920, 1080}};

std::string label(AVPixelFormat f, int w, int h, int align, int pad)
{
    return std::string(av_get_pix_fmt_name(f)) + " " + std::to_string(w) + "x" + std::to_string(h)
           + " align=" + std::to_string(align) + " pad=" + std::to_string(pad);
}

/* ------------------------------------------------------- unwrittenFraction -- */

TEST(UnwrittenFraction, MatchesTheModelAcrossFormatsAndGeometries)
{
    Rng rng;
    for (AVPixelFormat format : PLANAR)
    {
        for (const auto& d : DIMS)
        {
            for (int align : {1, 32})
            {
                TestFrame tf;
                if (!tf.build(format, d[0], d[1], align, 0))
                    continue;
                const std::string what = label(format, d[0], d[1], align, 0);

                tf.fillMarked();
                EXPECT_DOUBLE_EQ(frame_fill::unwrittenFraction(&tf.frame), modelUnwrittenFraction(&tf.frame))
                    << what << ", nothing decoded";
                EXPECT_DOUBLE_EQ(frame_fill::unwrittenFraction(&tf.frame), 1.0) << what;

                tf.fillPicture(rng);
                EXPECT_DOUBLE_EQ(frame_fill::unwrittenFraction(&tf.frame), modelUnwrittenFraction(&tf.frame))
                    << what << ", fully decoded";
                EXPECT_DOUBLE_EQ(frame_fill::unwrittenFraction(&tf.frame), 0.0) << what;

                /* Mixed content, which is what exercises the counting itself. */
                for (std::size_t p = 0; p < tf.planes(); ++p)
                    for (std::size_t i = 0; i < tf.fillBytes(p); ++i)
                    {
                        const unsigned char r = rng.byte();
                        const_cast<unsigned char*>(tf.block(p).data())[i] =
                            r < 96 ? (p == 0 ? 1 : 128) : static_cast<unsigned char>(16 + (r % 220));
                    }
                EXPECT_DOUBLE_EQ(frame_fill::unwrittenFraction(&tf.frame), modelUnwrittenFraction(&tf.frame))
                    << what << ", partially decoded";
            }
        }
    }
}

/* Luma 1 does occur in real picture content. Only luma 1 *under neutral chroma*
   means "never written", and getting this wrong is how a vectorised scan starts
   dropping healthy frames. */
TEST(UnwrittenFraction, DoesNotCountLumaOneWithoutNeutralChroma)
{
    for (AVPixelFormat format : PLANAR)
    {
        TestFrame tf;
        ASSERT_TRUE(tf.build(format, 640, 480, 32, 0));
        tf.setPlane(0, frame_fill::FRAME_FILL_LUMA);
        for (std::size_t p = 1; p < tf.planes(); ++p)
            tf.setPlane(p, 100);
        EXPECT_DOUBLE_EQ(frame_fill::unwrittenFraction(&tf.frame), 0.0)
            << av_get_pix_fmt_name(format) << ": luma 1 over non-neutral chroma is picture content";
    }
}

/* One component, or chroma that is not two separate planes: the chroma
   confirmation must be skipped rather than reaching for data[2]. */
TEST(UnwrittenFraction, ComparesLumaOnlyWhereThereIsNoPlanarChroma)
{
    for (AVPixelFormat format : {AV_PIX_FMT_GRAY8, AV_PIX_FMT_NV12, AV_PIX_FMT_NV21})
    {
        TestFrame tf;
        ASSERT_TRUE(tf.build(format, 640, 480, 32, 0));
        tf.setPlane(0, frame_fill::FRAME_FILL_LUMA);
        EXPECT_DOUBLE_EQ(frame_fill::unwrittenFraction(&tf.frame), 1.0) << av_get_pix_fmt_name(format);
        EXPECT_DOUBLE_EQ(frame_fill::unwrittenFraction(&tf.frame), modelUnwrittenFraction(&tf.frame));
    }
}

/* Planes are compared as bytes, so a 10-bit sample is judged on its low byte.
   That is what ships; pinning it keeps a future change from quietly altering
   which frames get dropped. */
TEST(UnwrittenFraction, ComparesHighBitDepthPlanesAsBytes)
{
    Rng rng;
    for (AVPixelFormat format : {AV_PIX_FMT_YUV420P10LE, AV_PIX_FMT_YUV422P10LE})
    {
        TestFrame tf;
        ASSERT_TRUE(tf.build(format, 320, 240, 32, 0));
        tf.fillMarked();
        EXPECT_DOUBLE_EQ(frame_fill::unwrittenFraction(&tf.frame), modelUnwrittenFraction(&tf.frame))
            << av_get_pix_fmt_name(format);
        tf.fillPicture(rng);
        EXPECT_DOUBLE_EQ(frame_fill::unwrittenFraction(&tf.frame), modelUnwrittenFraction(&tf.frame))
            << av_get_pix_fmt_name(format);
    }
}

/* data[0] can be a GPU device handle. None of these may be dereferenced. */
TEST(UnwrittenFraction, ReturnsZeroForFramesItMustNotRead)
{
    AVFrame stub{};
    stub.format = AV_PIX_FMT_YUV420P;
    stub.width = 64;
    stub.height = 64;
    EXPECT_DOUBLE_EQ(frame_fill::unwrittenFraction(&stub), 0.0) << "null data[0]";

    TestFrame tf;
    ASSERT_TRUE(tf.build(AV_PIX_FMT_YUV420P, 64, 64, 32, 0));
    tf.fillMarked();
    const int width = tf.frame.width, height = tf.frame.height, stride = tf.frame.linesize[0];

    tf.frame.width = 0;
    EXPECT_DOUBLE_EQ(frame_fill::unwrittenFraction(&tf.frame), 0.0) << "zero width";
    tf.frame.width = width;

    tf.frame.height = 0;
    EXPECT_DOUBLE_EQ(frame_fill::unwrittenFraction(&tf.frame), 0.0) << "zero height";
    tf.frame.height = height;

    tf.frame.width = -4;
    EXPECT_DOUBLE_EQ(frame_fill::unwrittenFraction(&tf.frame), 0.0) << "negative width";
    tf.frame.width = width;

    tf.frame.linesize[0] = 0;
    EXPECT_DOUBLE_EQ(frame_fill::unwrittenFraction(&tf.frame), 0.0) << "zero stride";
    tf.frame.linesize[0] = -stride;
    EXPECT_DOUBLE_EQ(frame_fill::unwrittenFraction(&tf.frame), 0.0) << "negative stride";
    tf.frame.linesize[0] = stride;

    tf.frame.format = AV_PIX_FMT_NONE;
    EXPECT_DOUBLE_EQ(frame_fill::unwrittenFraction(&tf.frame), 0.0) << "no pixel descriptor";

    AVFrame hw{};
    hw.format = AV_PIX_FMT_VAAPI;
    hw.width = 1920;
    hw.height = 1080;
    unsigned char handle[GUARD] = {0};
    hw.data[0] = handle;
    hw.linesize[0] = 1920;
    EXPECT_DOUBLE_EQ(frame_fill::unwrittenFraction(&hw), 0.0) << "hardware frame";
}

/* ----------------------------------------------------------- prefillPlanes -- */

/* The whole point of the marker: what comes back must be exactly the fill
   value, over exactly the bytes the stride describes, and nothing beyond. */
TEST(PrefillPlanes, FillsEveryPlaneExactlyAndNothingBeyondIt)
{
    for (AVPixelFormat format : {AV_PIX_FMT_YUV420P, AV_PIX_FMT_YUV422P, AV_PIX_FMT_YUV444P,
                                 AV_PIX_FMT_YUV410P, AV_PIX_FMT_YUV440P, AV_PIX_FMT_GRAY8,
                                 AV_PIX_FMT_YUV420P10LE, AV_PIX_FMT_YUV422P10LE})
    {
        for (const auto& d : DIMS)
        {
            for (int align : {1, 32, 64})
            {
                for (int pad : {0, 17})
                {
                    TestFrame tf;
                    if (!tf.build(format, d[0], d[1], align, pad))
                        continue;
                    tf.poison();
                    frame_fill::prefillPlanes(&tf.frame);

                    const std::string what = label(format, d[0], d[1], align, pad);
                    for (std::size_t p = 0; p < tf.planes(); ++p)
                    {
                        const unsigned char want =
                            p == 0 ? frame_fill::FRAME_FILL_LUMA : frame_fill::FRAME_FILL_CHROMA;
                        const auto& b = tf.block(p);
                        const std::size_t fill = tf.fillBytes(p);
                        std::size_t bad_fill = fill, bad_guard = b.size();
                        for (std::size_t i = 0; i < fill; ++i)
                            if (b[i] != want) { bad_fill = i; break; }
                        for (std::size_t i = fill; i < b.size(); ++i)
                            if (b[i] != POISON) { bad_guard = i; break; }
                        EXPECT_EQ(bad_fill, fill)
                            << what << ", plane " << p << ": byte " << bad_fill << " was not filled";
                        EXPECT_EQ(bad_guard, b.size())
                            << what << ", plane " << p << ": wrote "
                            << (bad_guard - fill + 1) << " byte(s) past the end of the plane";
                    }
                }
            }
        }
    }
}

/* A plane whose stride is negative runs backwards from data[]; filling it
   forwards would run off the allocation. */
TEST(PrefillPlanes, SkipsAPlaneWithANegativeStride)
{
    TestFrame tf;
    ASSERT_TRUE(tf.build(AV_PIX_FMT_YUV420P, 320, 240, 32, 0));
    tf.poison();
    tf.frame.linesize[1] = -tf.frame.linesize[1];
    frame_fill::prefillPlanes(&tf.frame);
    for (unsigned char b : tf.block(1))
        ASSERT_EQ(b, POISON) << "filled a plane with a negative stride";
}

TEST(PrefillPlanes, WritesNothingForAFrameOfZeroHeight)
{
    TestFrame tf;
    ASSERT_TRUE(tf.build(AV_PIX_FMT_YUV420P, 320, 240, 32, 0));
    tf.poison();
    tf.frame.height = 0;
    frame_fill::prefillPlanes(&tf.frame);
    for (std::size_t p = 0; p < tf.planes(); ++p)
        for (unsigned char b : tf.block(p))
            ASSERT_EQ(b, POISON) << "wrote to a frame of zero height";
}

/* data[0] is a device handle for a hardware frame, and unknown for a frame with
   no descriptor. The sentinel is sized to absorb everything a careless
   implementation would write, so the failure is reported rather than corrupting
   the heap and taking the whole test binary with it. */
TEST(PrefillPlanes, LeavesFramesItMustNotTouchAlone)
{
    struct Case { const char* name; AVPixelFormat format; int width, height, stride; };
    for (const Case& c : {Case{"hardware frame", AV_PIX_FMT_VAAPI, 1920, 1080, 1920},
                          Case{"no pixel descriptor", AV_PIX_FMT_NONE, 64, 64, 64}})
    {
        std::vector<unsigned char> sentinel(
            static_cast<std::size_t>(c.stride) * c.height + GUARD, POISON);
        AVFrame frame{};
        frame.format = c.format;
        frame.width = c.width;
        frame.height = c.height;
        frame.data[0] = sentinel.data();
        frame.linesize[0] = c.stride;
        frame_fill::prefillPlanes(&frame);
        std::size_t touched = sentinel.size();
        for (std::size_t i = 0; i < sentinel.size(); ++i)
            if (sentinel[i] != POISON) { touched = i; break; }
        EXPECT_EQ(touched, sentinel.size()) << c.name << ": wrote at byte " << touched;
    }
}

/* The two halves have to agree: a freshly marked frame reads as entirely
   unwritten, and one the decoder fully overwrote reads as not damaged at all. */
TEST(FrameFill, PrefillAndMeasurementAgree)
{
    Rng rng;
    for (AVPixelFormat format : PLANAR)
    {
        TestFrame tf;
        ASSERT_TRUE(tf.build(format, 1920, 1080, 32, 0));
        tf.poison();
        frame_fill::prefillPlanes(&tf.frame);
        EXPECT_DOUBLE_EQ(frame_fill::unwrittenFraction(&tf.frame), 1.0)
            << av_get_pix_fmt_name(format) << ": a marked frame is entirely unwritten";
        tf.fillPicture(rng);
        EXPECT_DOUBLE_EQ(frame_fill::unwrittenFraction(&tf.frame), 0.0)
            << av_get_pix_fmt_name(format) << ": a decoded frame carries no marker";
    }
}

}  // namespace
}  // namespace rtsp_image_transport
