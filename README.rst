****************************
RTSP Image Transport for ROS
****************************

Overview
========

This package allow ROS nodes to publish and subscribe image topics using the
`Real Time Streaming Protocol (RTSP)`_. Unlike regular `image_transport`_
plugins, the ``rtsp_image_transport`` does not transmit image data in-band. The
publisher merely publishes a latched `std_msgs/String`_ message with the URL of
an :RFC:`2326` compliant RTSP video stream server, from where the actual image
data is served. The subscriber will listen for URLs on the ROS topic and
automatically (re-)connect to the corresponding location.

Use Cases
=========

``rtsp_image_transport`` is intended for streaming live videos to one or more
users. The RTSP server supports multicast transmission, so it will potentially
save a huge amount of bandwidth if many clients are viewing the same video
stream.

The ``rtsp_image_transport`` plugin is compatible with many commercially
available IP cameras. You can use the ``publish_rtsp_stream`` node to create an
image topic with RTSP transport that reuses the existing RTSP stream.

Multiple Video Subsessions
==========================

This Nautilus fork adds the subscriber parameter ``video_subsession``. It is a
zero-based index over supported video subsessions in SDP order and defaults to
``0`` for upstream-compatible behavior. For an ``image_transport republish``
input named ``in``, select the second video subsession with
``-p in.rtsp.video_subsession:=1``.

You do not have to guess the index. Whenever the subscriber connects, it logs
every video subsession the server offers, together with codec and (if the SDP
announces them) resolution and frame rate::

  [/camera0] RTSP stream offers 3 video subsession(s):
      #0 H.265 1920x1080 @30Hz, #1 H.264 1024x600, #2 H.264 640x360

Only the selected subsession is set up, so no RTP sockets are allocated and no
bandwidth is consumed for the streams you are not watching. If the requested
index does not exist, or the server refuses it, the subscriber warns and falls
back to the remaining subsessions in SDP order instead of failing outright.

Changing ``video_subsession`` at runtime reconnects the client to the newly
selected stream.

The fork also handles H.265 SDP initialization correctly by prepending the
separate ``sprop-vps``, ``sprop-sps``, and ``sprop-pps`` parameter sets defined
by RFC 7798. Upstream treated H.265 like H.264 and looked only for the combined
``sprop-parameter-sets`` attribute, leaving some HEVC decoders without the
configuration needed to decode the first keyframe.

Hardware Accelerated Decoding
=============================

The subscriber decodes on the GPU whenever it can. At connect time it works
through the ways this machine could decode the stream and uses the first that
works:

1. the native FFmpeg decoder driven by a hardware device (NVDEC/CUDA, VAAPI,
   Quick Sync, VDPAU, Vulkan, VideoToolbox, D3D11VA), which is preferred
   because all streams then share a single GPU context;
2. a standalone hardware decoder (``h264_cuvid``, ``hevc_qsv``, ``*_rkmpp``,
   ``*_v4l2m2m``);
3. the software decoder.

Which one it settled on is logged when decoding starts::

  [/camera0] start decoding H.265 with hevc + cuda from rtsp://...

The device list is probed once per process and logged when the subscriber comes
up, so an installation problem is visible without turning on debug output::

  [/camera0] hardware video decoding available via: cuda, vaapi

Decoded frames are copied out of GPU memory into the ROS image message; the
colour conversion that follows is multi-threaded (see `Latency`_ below).

Parameters
----------

``use_hw_decoder`` (bool, default ``true``)
    Master switch. Set to ``false`` to decode in software.

``hw_device`` (string, default ``auto``)
    ``auto`` tries every device in the order above. ``none`` is equivalent to
    ``use_hw_decoder:=false``. Any other value pins a specific FFmpeg device
    type: ``cuda``, ``vaapi``, ``qsv``, ``vdpau``, ``drm``, ``vulkan``,
    ``videotoolbox`` or ``d3d11va``. An unusable value is reported along with
    the list your FFmpeg build supports.

``hw_device_path`` (string, default empty)
    Which adapter to use when the machine has more than one. For VAAPI and
    Quick Sync this is a DRM render node such as ``/dev/dri/renderD128``; for
    CUDA it is a device index. Empty lets FFmpeg pick.

``decoder`` (string, default empty)
    Pins one FFmpeg decoder by name, e.g. ``hevc_cuvid`` or ``h264_qsv``.
    Mostly useful for debugging: unlike the automatic selection, a pinned
    decoder is *not* silently replaced if it fails.

If a hardware decoder opens successfully but then fails to decode — an
unsupported profile, a busy or reset GPU — the subscriber logs a warning and
continues with the next option rather than dropping the stream::

  [/camera0] failed to send bitstream packet to decoder: ...; falling back to hevc (software)

Supported codecs are listed under `Supported Formats`_. Availability of a
hardware path for any given one depends on your GPU and FFmpeg build; the
subscriber only offers combinations that FFmpeg reports as supported, so an
unavailable one costs nothing at runtime.

Intel integrated graphics
-------------------------

Intel iGPUs are reached either through VAAPI or through Quick Sync, and the
stock Ubuntu FFmpeg build supports both for H.264, H.265, VP8, VP9, AV1,
MPEG-2, MPEG-4 and MJPEG. Install the driver and, for Quick Sync, the runtime::

  sudo apt install intel-media-va-driver-non-free libmfx-gen1.2

``vainfo`` should then list the profiles your GPU can decode. Nothing else is
needed: ``hw_device:=auto`` finds the render node on its own, and VAAPI is
tried before Quick Sync because it needs no extra runtime.

On Linux, Quick Sync is layered on VAAPI, so the subscriber derives the Quick
Sync device from the VAAPI one it already has rather than opening the GPU a
second time. If a machine has both an Intel iGPU and a discrete card, point
``hw_device_path`` at the render node you want::

  -p in.rtsp.hw_device:=vaapi -p in.rtsp.hw_device_path:=/dev/dri/renderD128

Note that a container needs the render node passed through
(``docker run --device /dev/dri``) and the user must be in the ``render``
group; otherwise the device probe fails and decoding quietly stays in
software, which the log will say.

Latency
=======

The subscriber decodes with minimal buffering by default. The ``low_latency``
parameter controls this: it makes the decoder emit each frame as soon as it is
complete rather than holding it in the reordering buffer, and it keeps FFmpeg
from using frame-level multithreading, which delays output by roughly one frame
per worker thread.

Set ``-p in.rtsp.low_latency:=false`` if your source encodes B-frames and you
see frames arriving out of order; you trade a few frames of latency for correct
presentation order. Live IP camera streams almost never need this.

When the decoder cannot keep up, the subscriber progressively drops frames
before decoding them: non-intra frames once the queue spans 0.5 s, non-key
frames at 1 s, and everything at 2 s, so the pipeline catches up instead of
drifting further behind.


Tests
=====

The package ships a GoogleTest suite covering codec selection, the decoder
(every supported codec, hardware selection and fallback, time stamps, frame
dropping), the encoder, an encode/decode round trip, the Live555 event loop,
the graph monitor, and a loopback integration test that serves a real RTSP
session on localhost and decodes what comes back::

  colcon build --packages-select rtsp_image_transport --cmake-args -DBUILD_TESTING=ON
  colcon test --packages-select rtsp_image_transport
  colcon test-result --all

Tests skip themselves when the FFmpeg build lacks a needed encoder, and the
hardware decoding tests adapt to whatever the machine actually has, so the
suite passes on a headless build server and exercises the GPU paths where one
is present.

Limitations
===========

If the image source is not a continuous video stream with roughly constant
frame rate, you may experience problems such as session timeouts or degraded
image quality.

``rtsp_image_transport`` is not suitable for image data that is to be consumed
by image processing algorithms. The lossy compression introduces artifacts
which may not be visible to the human eye but interfere with many algorithms
nevertheless.

You cannot use `rosbag2`_ to record data from ``rtsp_image_transport``; you will
just end up with a bunch of useless URL string messages.

Supported Formats
=================

``rtsp_image_transport`` uses the `Live555`_ library for its RTSP server and
client implementation and the `FFmpeg`_ library for video compression. Starting
with FFmpeg 4, some codecs are hardware acceleratable. The following table
summarizes the options, subject to availability in your FFmpeg version and
compatible hardware:

+------------+----------+----------+-----------+----------+----------+
|            | Software | NVIDIA   | QSV [2]_  | VAAPI    | OMX      |
+============+==========+==========+===========+==========+==========+
| H.264      | Yes      | Yes      | Yes       | Yes      | Encoding |
+------------+----------+----------+-----------+----------+----------+
| H.265      | Yes      | Yes      | Yes       | Yes      | No       |
+------------+----------+----------+-----------+----------+----------+
| MPEG-4     | Yes      | Decoding | No        | Encoding | No       |
+------------+----------+----------+-----------+----------+----------+
| MPEG-2 [1]_| Decoding | Decoding | Decoding  | Decoding | No       |
+------------+----------+----------+-----------+----------+----------+
| H.263 [1]_ | Decoding | No       | No        | No       | No       |
+------------+----------+----------+-----------+----------+----------+
| VP8        | Yes      | Decoding | Decoding  | Yes      | No       |
+------------+----------+----------+-----------+----------+----------+
| VP9        | Yes      | Decoding | Yes       | Yes      | No       |
+------------+----------+----------+-----------+----------+----------+
| AV1        | Yes      | Decoding | Yes       | Yes      | No       |
+------------+----------+----------+-----------+----------+----------+
| MJPEG [1]_ | Decoding | Decoding | Decoding  | Decoding | No       |
+------------+----------+----------+-----------+----------+----------+

Hardware decoding is also available through V4L2 stateful decoders
(``*_v4l2m2m``) and Rockchip MPP (``*_rkmpp``) where your FFmpeg build provides
them, which covers most ARM SoCs. The subscriber selects among whatever is
actually present; see `Hardware Accelerated Decoding`_.

The RTSP payload names recognised by the subscriber are ``H264``, ``H265``,
``MP4V-ES``, ``MPV`` (MPEG-1/2 elementary stream, :RFC:`2250`), ``H263``,
``H263-1998``, ``H263-2000``, ``VP8``, ``VP9``, ``AV1`` and ``JPEG``.

.. [1] ``rtsp_image_transport`` cannot create Motion JPEG (MJPEG), MPEG-2 or
    H.263 streams, only receive them for backwards compatibility with older
    IP cameras. If you really want to have independently compressed JPEG
    frames for your video stream, you can use the
    `compressed_image_transport`_ with JPEG compression instead.

.. [2] You need to install the Ubuntu package ``libmfx-gen1.2`` for QSV support.

.. _Real Time Streaming Protocol (RTSP): https://en.wikipedia.org/wiki/Real_Time_Streaming_Protocol

.. _image_transport: https://index.ros.org/p/image_transport/

.. _compressed_image_transport: https://index.ros.org/p/compressed_image_transport/

.. _std_msgs/String: https://docs.ros.org/en/rolling/p/std_msgs/msg/String.html

.. _Live555: http://www.live555.com/liveMedia

.. _FFmpeg: https://ffmpeg.org/

.. _rosbag2: https://index.ros.org/p/rosbag2/
