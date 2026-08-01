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
| H.264      | Yes      | Yes      | Yes       | Encoding | Encoding |
+------------+----------+----------+-----------+----------+----------+
| H.265      | Yes      | Yes      | Yes       | Encoding | No       |
+------------+----------+----------+-----------+----------+----------+
| MPEG-4     | Yes      | Decoding | No        | Encoding | No       |
+------------+----------+----------+-----------+----------+----------+
| VP8        | Yes      | Decoding | Decoding  | Encoding | No       |
+------------+----------+----------+-----------+----------+----------+
| VP9        | Yes      | Decoding | Yes       | Encoding | No       |
+------------+----------+----------+-----------+----------+----------+
| MJPEG [1]_ | Decoding | Decoding | Decoding  | No       | No       |
+------------+----------+----------+-----------+----------+----------+

.. [1] ``rtsp_image_transport`` cannot create Motion JPEG (MJPEG) streams,
    only receive them for backwards compatibility with some ancient IP
    cameras. If you really want to have independently compressed JPEG
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
