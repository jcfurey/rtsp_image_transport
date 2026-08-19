/****************************************************************************
 *
 * rtsp_image_transport
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/
#include "frame_fill.h"

#include <cstddef>
#include <cstdint>
#include <cstring>

#if defined(__x86_64__) || defined(__i386__)
#  define UF_X86 1
#  include <immintrin.h>
#endif
/* Streaming stores are x86-64 only here. Restricting to __x86_64__ (rather than
 * any i386) guarantees SSE2 is baseline, so _mm_sfence() needs no target gate.
 * The AVX2 body itself is compiled with a target attribute because the build is
 * plain -O2 with no -march, and entered only after __builtin_cpu_supports. */
#if defined(__x86_64__) && defined(__GNUC__)
#  define PREFILL_HAVE_X86_STREAM 1
#  include <immintrin.h>
#else
#  define PREFILL_HAVE_X86_STREAM 0
#endif

namespace rtsp_image_transport
{
namespace frame_fill
{

/* Fills freshly allocated frames so that macroblocks the decoder never writes
   are recognisable afterwards, and read as black rather than as bright green.
   Installed only for the codecs libavcodec cannot conceal slice loss for. */
namespace {

/* Below this the non-temporal path is not worth it: the write-combining
 * buffers need a long run to amortise, and a plane this small may still be
 * usefully cache-resident for whatever touches it next. Small planes -- and
 * every degenerate case in the contract sweep -- stay on plain memset. */
constexpr std::size_t kStreamThreshold = 128u * 1024u;

#if PREFILL_HAVE_X86_STREAM

/* Fills exactly n bytes at dst. Head and tail go through memset so the
 * streaming body only ever sees 32-byte-aligned addresses; the head clamp is
 * what stops a short plane from stepping into the poisoned guard band. */
__attribute__((target("avx2")))
void fill_stream_avx2(unsigned char* dst, unsigned char value, std::size_t n) noexcept
{
    std::size_t head = (32u - (reinterpret_cast<std::uintptr_t>(dst) & 31u)) & 31u;
    if (head > n)
        head = n;
    if (head != 0)
    {
        std::memset(dst, value, head);
        dst += head;
        n -= head;
    }

    const __m256i v = _mm256_set1_epi8(static_cast<char>(value));
    while (n >= 128)
    {
        _mm256_stream_si256(reinterpret_cast<__m256i*>(dst + 0), v);
        _mm256_stream_si256(reinterpret_cast<__m256i*>(dst + 32), v);
        _mm256_stream_si256(reinterpret_cast<__m256i*>(dst + 64), v);
        _mm256_stream_si256(reinterpret_cast<__m256i*>(dst + 96), v);
        dst += 128;
        n -= 128;
    }
    while (n >= 32)
    {
        _mm256_stream_si256(reinterpret_cast<__m256i*>(dst), v);
        dst += 32;
        n -= 32;
    }
    if (n != 0)
        std::memset(dst, value, n);
}

bool cpu_has_avx2() noexcept
{
    static const bool ok = __builtin_cpu_supports("avx2") != 0;
    return ok;
}

#endif // PREFILL_HAVE_X86_STREAM

// Returns true when non-temporal stores were used, i.e. the caller owes a fence.
bool fill_plane(void* p, unsigned char value, std::size_t n) noexcept
{
#if PREFILL_HAVE_X86_STREAM
    if (n >= kStreamThreshold && cpu_has_avx2())
    {
        fill_stream_avx2(reinterpret_cast<unsigned char*>(p), value, n);
        return true;
    }
#endif
    std::memset(p, value, n);
    return false;
}

} // namespace

void prefillPlanes(AVFrame* frame) noexcept
{

    const AVPixFmtDescriptor* desc = av_pix_fmt_desc_get(static_cast<AVPixelFormat>(frame->format));
    if (!desc || (desc->flags & AV_PIX_FMT_FLAG_HWACCEL) != 0)
        return;

    bool streamed = false;
    for (int plane = 0; plane < AV_NUM_DATA_POINTERS && frame->data[plane]; ++plane)
    {
        const int height = plane == 0 ? frame->height : AV_CEIL_RSHIFT(frame->height, desc->log2_chroma_h);
        const int stride = frame->linesize[plane];
        const unsigned char value = plane == 0 ? FRAME_FILL_LUMA : FRAME_FILL_CHROMA;
        if (stride > 0 && height > 0)
        {
            const std::size_t bytes = static_cast<std::size_t>(stride) * static_cast<std::size_t>(height);
            streamed = fill_plane(frame->data[plane], value, bytes) || streamed;
        }
    }

#if PREFILL_HAVE_X86_STREAM
    /* Once per frame, not once per plane. Non-temporal stores are weakly
     * ordered and libavcodec hands this buffer straight to its worker threads;
     * without this they are not guaranteed to observe the fill. */
    if (streamed)
        _mm_sfence();
#else
    (void)streamed;
#endif
}

/* How much of the picture still carries the fill value, sampled every eighth
   pixel in both directions. A lost slice covers whole rows of macroblocks, so
   the sparse grid finds it at a sixty-fourth of the cost of a full scan.
   Chroma has to match as well: a luma value of 1 does turn up in ordinary
   picture content, but not with perfectly neutral chroma on top of it. */
namespace
{

// Confirm luma hits found by the vector scan.  `urow` is null for formats that
// compare luma only (GRAY8, NV12, ...), which keeps this branch predictable and
// guarantees we never touch a chroma plane the contract says is absent.
inline std::size_t confirm_hits(unsigned mask, int base, const unsigned char* urow,
                                const unsigned char* vrow, int log2cw) noexcept
{
    std::size_t n = 0;
    while (mask)
    {
        const int b = __builtin_ctz(mask);
        mask &= mask - 1;
        if (urow)
        {
            const int cx = (base + b) >> log2cw;
            if (urow[cx] != FRAME_FILL_CHROMA || vrow[cx] != FRAME_FILL_CHROMA)
                continue;
        }
        ++n;
    }
    return n;
}

inline std::size_t scan_row_scalar(const unsigned char* luma, int x, int width,
                                   const unsigned char* urow, const unsigned char* vrow,
                                   int log2cw) noexcept
{
    std::size_t n = 0;
    for (; x < width; x += 8)
    {
        if (luma[x] != FRAME_FILL_LUMA)
            continue;
        if (urow)
        {
            const int cx = x >> log2cw;
            if (urow[cx] != FRAME_FILL_CHROMA || vrow[cx] != FRAME_FILL_CHROMA)
                continue;
        }
        ++n;
    }
    return n;
}

#if UF_X86
const bool g_has_avx2 = __builtin_cpu_supports("avx2") != 0;

// Loads are bounded by x + 32 <= width, so the highest byte read is width-1:
// the same address range the scalar reference touches, no padding assumed.
__attribute__((target("avx2"))) std::size_t
scan_row_avx2(const unsigned char* luma, int width, const unsigned char* urow,
              const unsigned char* vrow, int log2cw) noexcept
{
    const __m256i fill = _mm256_set1_epi8(static_cast<char>(FRAME_FILL_LUMA));
    const unsigned kSampled = 0x01010101u;  // movemask bits for byte offsets 0,8,16,24
    std::size_t n = 0;
    int x = 0;
    for (; x + 128 <= width; x += 128)
    {
        const __m256i c0 = _mm256_cmpeq_epi8(
            _mm256_loadu_si256(reinterpret_cast<const __m256i*>(luma + x)), fill);
        const __m256i c1 = _mm256_cmpeq_epi8(
            _mm256_loadu_si256(reinterpret_cast<const __m256i*>(luma + x + 32)), fill);
        const __m256i c2 = _mm256_cmpeq_epi8(
            _mm256_loadu_si256(reinterpret_cast<const __m256i*>(luma + x + 64)), fill);
        const __m256i c3 = _mm256_cmpeq_epi8(
            _mm256_loadu_si256(reinterpret_cast<const __m256i*>(luma + x + 96)), fill);
        const __m256i any = _mm256_or_si256(_mm256_or_si256(c0, c1), _mm256_or_si256(c2, c3));
        if ((static_cast<unsigned>(_mm256_movemask_epi8(any)) & kSampled) == 0)
            continue;  // common case: 16 samples cleared by four loads and one test
        n += confirm_hits(static_cast<unsigned>(_mm256_movemask_epi8(c0)) & kSampled, x,
                          urow, vrow, log2cw);
        n += confirm_hits(static_cast<unsigned>(_mm256_movemask_epi8(c1)) & kSampled, x + 32,
                          urow, vrow, log2cw);
        n += confirm_hits(static_cast<unsigned>(_mm256_movemask_epi8(c2)) & kSampled, x + 64,
                          urow, vrow, log2cw);
        n += confirm_hits(static_cast<unsigned>(_mm256_movemask_epi8(c3)) & kSampled, x + 96,
                          urow, vrow, log2cw);
    }
    for (; x + 32 <= width; x += 32)
    {
        const unsigned m =
            static_cast<unsigned>(_mm256_movemask_epi8(_mm256_cmpeq_epi8(
                _mm256_loadu_si256(reinterpret_cast<const __m256i*>(luma + x)), fill)))
            & kSampled;
        if (m)
            n += confirm_hits(m, x, urow, vrow, log2cw);
    }
    return n + scan_row_scalar(luma, x, width, urow, vrow, log2cw);
}
#endif

}  // namespace

double unwrittenFraction(const AVFrame* frame) noexcept
{

    if (!frame->data[0] || frame->width <= 0 || frame->height <= 0)
        return 0.0;
    const AVPixFmtDescriptor* desc = av_pix_fmt_desc_get(static_cast<AVPixelFormat>(frame->format));
    if (!desc || (desc->flags & AV_PIX_FMT_FLAG_HWACCEL) != 0 || frame->linesize[0] <= 0)
        return 0.0;
    const bool planar_chroma =
        frame->data[1] && frame->data[2] && desc->nb_components >= 3 && frame->linesize[1] > 0
        && frame->linesize[2] > 0;
    const int log2cw = desc->log2_chroma_w;
    const int log2ch = desc->log2_chroma_h;
    const int ls0 = frame->linesize[0];
    // Closed form for the trip counts of the two sampling loops: identical to
    // counting ++sampled per visited grid point, but off the hot path.
    const std::size_t sampled = static_cast<std::size_t>((frame->width + 7) / 8)
                                * static_cast<std::size_t>((frame->height + 7) / 8);
    std::size_t unwritten = 0;
    for (int y = 0; y < frame->height; y += 8)
    {
        const unsigned char* luma = frame->data[0] + static_cast<std::ptrdiff_t>(y) * ls0;
        const unsigned char* urow = nullptr;
        const unsigned char* vrow = nullptr;
        if (planar_chroma)
        {
            const int cy = y >> log2ch;
            urow = frame->data[1] + static_cast<std::ptrdiff_t>(cy) * frame->linesize[1];
            vrow = frame->data[2] + static_cast<std::ptrdiff_t>(cy) * frame->linesize[2];
        }
        if (y + 8 < frame->height)
        {
            // The 8-row stride crosses pages, so the streamer restarts cold at
            // every row; seed it by hand two lines into the next sampled row.
            const unsigned char* nxt = luma + static_cast<std::ptrdiff_t>(8) * ls0;
            __builtin_prefetch(nxt, 0, 0);
            __builtin_prefetch(nxt + 64, 0, 0);
        }
#if UF_X86
        unwritten += g_has_avx2 ? scan_row_avx2(luma, frame->width, urow, vrow, log2cw)
                                : scan_row_scalar(luma, 0, frame->width, urow, vrow, log2cw);
#else
        unwritten += scan_row_scalar(luma, 0, frame->width, urow, vrow, log2cw);
#endif
    }
    return sampled > 0 ? static_cast<double>(unwritten) / static_cast<double>(sampled) : 0.0;
}

}  // namespace frame_fill
}  // namespace rtsp_image_transport
