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

ROS Image Topic to RTSP
=======================

The package includes ready-to-run Jazzy launch YAMLs for the other direction:
encoding a regular ``sensor_msgs/Image`` topic and serving it to native RTSP
clients. H.264 is the broadly compatible choice::

  ros2 launch rtsp_image_transport ros_to_rtsp_h264.launch.yaml \
      input_topic:=/camera/image_raw \
      output_topic:=/camera/image_h264/rtsp

The HEVC example uses less bandwidth at comparable quality::

  ros2 launch rtsp_image_transport ros_to_rtsp_h265.launch.yaml \
      input_topic:=/camera/image_raw \
      output_topic:=/camera/image_h265/rtsp

Those topic arguments are normally the only values to change. The H.264 and
H.265 examples default to ports 8555 and 8556, respectively, and also accept
``rtsp_port``, ``bitrate`` and ``framerate`` launch arguments. Both request a
hardware encoder and transparently fall back to software when the machine has
no usable one. On NVIDIA hardware the selected encoder is logged as
``h264_nvenc`` or ``hevc_nvenc``.

The output ROS topic carries the URL, not the video. Read it once, then hand
the value to ffplay, VLC, GStreamer, or another RTSP client::

  ros2 topic echo --once /camera/image_h264/rtsp
  ffplay -rtsp_transport tcp -fflags nobuffer -flags low_delay \
      rtsp://HOST:8555/

The examples request best-effort input QoS with depth 1. That accepts both the
usual ``SensorDataQoS`` camera publisher and a reliable publisher, while
preventing old images from queuing ahead of the encoder. The encoder itself is
configured for no B-frames, zero look-ahead and the hardware encoder's
low-latency preset.

The relay is demand-driven. While no native client is playing the stream,
``image_transport republish`` releases its raw input subscription and the
encoder does no work. A native RTSP PLAY request activates the subscription in
about 10 ms; disconnecting the last client releases it again. Merely reading
the URL once therefore does not leave a high-bandwidth raw subscription or GPU
encoder running.

If starting the relay by hand instead of using the launch YAML, remap the full
``out/rtsp`` topic. In ROS 2, remapping ``out`` alone does not also remap that
child transport topic::

  ros2 run image_transport republish --ros-args \
      -p in_transport:=raw -p out_transport:=rtsp \
      -p out.rtsp.codec:=H264 -p out.rtsp.use_hw_encoder:=true \
      -r in:=/camera/image_raw \
      -r out/rtsp:=/camera/image_h264/rtsp

RTSP to ROS Image Topic
=======================

The receiving counterpart, decoding a stream back onto a
``sensor_msgs/Image`` topic::

  ros2 launch rtsp_image_transport rtsp_to_ros.launch.yaml \
      input_topic:=/camera/image_h264/rtsp \
      output_topic:=/camera/image_raw

It is configured for latency throughout: RTP over UDP with a 2 MB receive
buffer, minimal decoder buffering, and best-effort depth-1 output QoS, since a
consumer that has fallen behind on live video wants the newest frame rather
than a replay of the one it missed. ``max_latency`` and ``rtp_over_tcp`` are
launch arguments; see `Latency`_ and `RTP transport and packet loss`_.

The input is the URL topic, so it works equally with a stream advertised by
``publish_rtsp_stream`` from an IP camera.

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

Devices are probed once per process, only when hardware decoding is actually
wanted, and the result is shared by every stream. If none of them work the
subscriber says so and lists what the machine does have, so a missing driver
does not turn into a silent performance cliff::

  [/camera0] no hardware decoder available, decoding in software
             (hardware devices on this machine: vaapi)

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

The same applies when a device opens but turns out not to implement the
stream's profile, which is the usual outcome for 10 bit or 4:2:2 video on an
iGPU. That case used to decode on the CPU while still reporting the stream as
hardware accelerated, so the only symptom was a busy core. It now moves to the
next candidate — often a different GPU path that does support the profile, and
only then software::

  [/camera0] hevc + vaapi cannot decode this stream (it offers yuv420p10le, not vaapi); trying the next decoder

Whatever it settles on, the decoder named in the ``start decoding`` line is the
one actually running.

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
before decoding them, so the pipeline catches up instead of drifting further
behind. ``max_latency`` (float, default ``0.2`` s) is the budget that ladder
works to: non-intra frames are dropped once the queue spans ``max_latency``,
non-key frames at twice that, and everything at four times.

Because the queue settles on whichever rung matches the shortfall, this
parameter is what steady-state latency converges to whenever the decoder is
short of CPU — it is a latency budget, not a rarely-reached safety net. Lower
it for a tighter bound at the cost of more dropped frames, raise it to favour
smooth video over freshness, or set it to ``0`` to never drop and let latency
grow instead::

  ros2 run image_transport republish --ros-args \
      -p in_transport:=rtsp -p out_transport:=raw \
      -p in.rtsp.max_latency:=0.1 \
      -r in/rtsp:=/camera/image_h264/rtsp -r out:=/camera/image_raw

The publisher side is bounded the same way, per connected client: a client
whose link cannot carry the stream backs up its own send queue, and once that
queue spans more than 200 ms the oldest whole pictures are dropped rather than
letting that client fall ever further behind. Other clients are unaffected,
since each has its own queue.

The encoder's VBV buffer is a quarter of the target bitrate, which bounds how
far a single picture may overshoot its share of the link and therefore how
long one frame can occupy it.


RTP transport and packet loss
=============================

The subscriber runs RTP on its own UDP sockets by default (``rtp_over_tcp``,
off), because for live video a late frame is worth less than a missing one.
TCP cannot drop: a lost segment stalls every byte queued behind it until the
retransmission arrives, and the viewer waits through that for data it is
already too late to display. Loss on UDP costs a slice of one picture and the
stream carries on at once, so latency stays bounded by the link rather than by
its worst recent loss.

The subscriber asks for a 2 MB receive buffer on the RTP socket
(``rtp_buffer_size``), since the kernel default holds well under one HD frame
and would otherwise be the thing dropping datagrams. The kernel silently caps
the request at ``net.core.rmem_max``, and the log says so when it does::

    sysctl -w net.core.rmem_max=4194304

Measured on a 640x480 H.264 stream at 2 Mbit/s with packet loss injected below
TCP, latency from the sender's stamp to arrival:

===========  ==========  ========  =========  =========
Packet loss  Transport   p50 [ms]  p90 [ms]   max [ms]
===========  ==========  ========  =========  =========
1%           UDP              4.2      38-41    138-141
1%           TCP              4.5     80-112    253-256
5%           UDP              5.4        105        140
5%           TCP            47-75    211-228    423-616
===========  ==========  ========  =========  =========

UDP's worst case barely moves between 1% and 5% loss; what changes is how much
it drops (1% and 4.5% of NAL units). TCP delivers every byte at both rates and
pays entirely in time — at 5% even the median frame is tens of milliseconds
late, and the tail runs past half a second.

Set ``-p in.rtsp.rtp_over_tcp:=true`` to interleave RTP over the RTSP
connection instead. That is the right choice when artefacts cost more than
delay — a link losing enough packets to make the picture unusable, or a
recording — and for servers that will not serve RTP over UDP at all.

A lost datagram costs a slice. With ``AV_CODEC_FLAG2_CHUNKS`` set, the decoder
emits the picture anyway, built from the slices that did arrive; the
macroblocks the missing slice covered are never written. An all-zero YUV block
converts to BGR (0, 135, 0), so undamaged-looking loss would show up as flat
green bands across part of the image — see below for what is done about that.

What happens to residual loss depends on the codec, because libavcodec's error
concealment only covers some of them. For H.264, MPEG-2, MPEG-4 and H.263 the
decoder reconstructs the missing macroblocks from the reference frame, and the
picture is published repaired.

H.265, VP8, VP9 and AV1 have no concealment in libavcodec at all, and no way to
report that a picture came out incomplete. For those the subscriber pre-fills
each frame before decoding, so anything the decoder never writes reads as black
rather than as a bright green band, and the leftover fill is what identifies the
damaged region afterwards. Costs about 1.6% of decoding time.

Set ``-p in.rtsp.drop_corrupt_frames:=true`` to discard damaged pictures instead
of publishing them, which trades artefacts for stutter — useful for consumers
that would rather see nothing than see wrong pixels. It works for every codec
listed above, concealed or not.

Frames decoded before the first key frame of a session are never published.
Connecting to a live stream lands mid-GOP, so those pictures reference data that
was never received and would otherwise be the first thing on screen.


Quality of Service
==================

The transport topic ``<base topic>/rtsp`` carries a latched URL, not image
data, so the plugins pin three policies on it regardless of what is requested:
``RELIABLE``, ``TRANSIENT_LOCAL`` and ``KEEP_LAST`` with depth 1. A subscriber
that joins after the publisher started would otherwise never learn the stream
URL. Every other policy — deadline, lifespan, liveliness — is passed through
untouched.

Because those three are fixed, a peer that insists on ``BEST_EFFORT`` or
``VOLATILE`` is incompatible and simply receives nothing. Both plugins now
report that instead of leaving you with a silent black screen::

  [/camera0] the RTSP URL publisher offers an incompatible QoS policy (3);
             this transport needs RELIABLE and TRANSIENT_LOCAL, so no video will arrive

The QoS of the *image* topic your subscriber callback is fed from is yours to
choose as usual; it is unrelated to the URL topic.

ROS Time and Simulated Time
===========================

RTSP timing is wall clock timing: RTP and RTCP carry the sender's real time,
and ROS time is something else entirely under ``use_sim_time``. The plugins
translate between the two rather than assuming they are the same.

Subscriber
----------

``timestamp_source`` (int, default ``2``)
    Where the stamp on a decoded image comes from.

    ``0`` — the sender's clock, recovered from RTCP. Best when the camera is
    NTP synchronised, since it is the actual capture time.

    ``1`` — the node clock when the frame arrives. Always in the node's own
    time base, at the cost of network jitter.

    ``2`` (automatic) — sender clock on a node running on wall clock time,
    time of reception when simulated time is in use.

The default matters: with ``use_sim_time`` the rest of the system works in
simulated time, and images stamped with a camera's wall clock would be years
away from it. Every TF lookup and message filter downstream would reject them.

Publisher
---------

Image stamps are mapped onto a strictly increasing wall clock timeline before
they reach RTP, so simulated time starting near zero no longer produces RTCP
sender reports dated 1970. When the ROS clock jumps — a bag looping, a
simulation reset — the timeline is re-anchored and the stream carries on::

  [/camera0] image time stamps jumped, re-anchoring the RTP timeline

Frame pacing is taken from the gap between consecutive images rather than from
a fixed origin, so a clock that jumps backwards does not convince the encoder
that the stream suddenly runs at 300 fps.

Reconnect back-off and the session timeout deliberately use wall clock timers:
a network retry should not stall because a simulation is paused.

Recording and Playback
======================

Recording ``<base topic>/rtsp`` captures the stream URL, not the video, and
replaying it points the subscriber at a server that is no longer there. To put
the actual images in a bag, decode them first::

  ros2 run image_transport republish rtsp raw \
      --ros-args --remap in/rtsp:=/camera/image/rtsp --remap out:=/camera/image/raw
  ros2 bag record /camera/image/raw

The raw topic then plays back like any other, and the ROS time handling above
is what makes that work in a system running on ``--clock``.

If you do end up replaying a recorded URL topic, set
``-p in.rtsp.reconnect_policy:=0`` so the subscriber does not spend the session
retrying a stream that no longer exists.

Supported image_transport versions
==================================

The transport supports both the legacy plugin API used by Jazzy and Kilted and
the node-interface API introduced in ``image_transport`` 6.4.

The newer API does not provide the node clock interface. Consequently, on
6.4+ a subscriber plugin cannot stamp decoded images in simulated ROS time;
select ``timestamp_source:=1`` for receive timestamps, which use the system
clock on that API. Encoding, native RTSP serving, lazy ROS-to-RTSP republishing,
QoS handling, and wall-clock operation are supported normally.

Building
========

The package uses `ament_cmake_auto`_, so the dependencies are declared once in
``package.xml``. Besides those it needs `live555_vendor`_ and the FFmpeg
development libraries::

  sudo apt install libavcodec-dev libavformat-dev libavutil-dev libswscale-dev
  colcon build --packages-select rtsp_image_transport

The two helper nodes install to ``lib/rtsp_image_transport``, so they can be
started with ``ros2 run``::

  ros2 run rtsp_image_transport publish_rtsp_stream rtsp://<url> \
      --ros-args --remap image:=<image topic>

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

You cannot use `rosbag2`_ to record data from ``rtsp_image_transport``
directly; you will just end up with a bunch of useless URL string messages.
See `Recording and Playback`_ for the way around that.

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

.. _ament_cmake_auto: https://index.ros.org/p/ament_cmake_auto/

.. _live555_vendor: https://github.com/fkie/live555_vendor
