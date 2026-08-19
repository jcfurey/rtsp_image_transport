/****************************************************************************
 *
 * rtsp_image_transport
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/
#ifndef RTSP_IMAGE_TRANSPORT_FRAME_FILL_H_
#define RTSP_IMAGE_TRANSPORT_FRAME_FILL_H_

extern "C"
{
#include <libavutil/frame.h>
#include <libavutil/pixdesc.h>
}

/* The two halves of the slice-loss detection libavcodec does not provide for
   H.265: mark a freshly allocated frame, then measure what the decoder did not
   write over. They are pure functions of the frame, hold no decoder state, and
   live here rather than inside StreamDecoder so their contracts can be tested
   directly -- which matters more than usual, because both are hand-vectorised
   and their edge cases are where the danger is. */
namespace rtsp_image_transport
{
namespace frame_fill
{

/* Value written into every freshly allocated luma plane, and into the chroma
   planes, before the decoder fills a picture in. Y=1 is below the legal TV range
   floor of 16 and pairs with neutral chroma, so anything left over reads as
   black rather than as the flat green an all-zero YUV block converts to, and is
   rare enough in real picture content to be recognisable as "never written".

   The alternative, zero, is what produces the bright green bands: YUV(0,0,0) is
   BGR(0,135,0). */
constexpr unsigned char FRAME_FILL_LUMA = 1;
constexpr unsigned char FRAME_FILL_CHROMA = 128;

/* Fills every plane of a freshly allocated frame with the marker above.
   Does nothing for hardware frames or a frame with no pixel descriptor, whose
   data[0] is a device handle rather than memory, and skips a plane whose stride
   is negative. Writes exactly linesize * plane_height bytes per plane and not
   one byte more. */
void prefillPlanes(AVFrame* frame) noexcept;

/* The fraction of the picture still carrying the marker, sampled every eighth
   pixel in both directions. Returns 0.0 for frames prefillPlanes() would have
   left alone. */
double unwrittenFraction(const AVFrame* frame) noexcept;

}  // namespace frame_fill
}  // namespace rtsp_image_transport

#endif
