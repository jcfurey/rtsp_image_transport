Low-latency playback for RTSP cameras
=====================================

The playback settings in this guide apply to RTSP camera streams and ROS image
streams encoded by this package. The `ZED example <zed_rtsp.rst>`_ adds one
camera-specific capture configuration; the player does not require a ZED.
The `camera profile guide <camera_profiles.rst>`_ also provides RealSense
color-only and Lucid GigE launch profiles with optional vendor drivers/SDKs.

Native viewer
-------------

After building and sourcing the package overlay, open a stream with::

  ros2 run rtsp_image_transport play_rtsp rtsp://HOST:8555/PATH

The URL defaults to ``rtsp://127.0.0.1:8555/`` when omitted. The helper executes
ffplay in the foreground with these settings::

  SDL_RENDER_VSYNC=0 ffplay -rtsp_transport udp -buffer_size 2097152 \
      -fflags nobuffer -flags low_delay -threads 1 \
      -max_delay 0 -reorder_queue_size 0 \
      -framedrop -sync video -noinfbuf -an rtsp://HOST:8555/PATH

.. list-table:: What the profile controls
   :header-rows: 1
   :widths: 36 64

   * - Setting
     - Purpose
   * - ``-threads 1 -flags low_delay``
     - Avoid frame-thread buffering in the decoder.
   * - ``-sync video``
     - Avoid the external clock's deliberate slowdown while its packet queue fills.
   * - ``-fflags nobuffer -noinfbuf``
     - Reduce optional buffering and disable ffplay's unlimited input-buffer mode.
   * - ``-framedrop``
     - Let the player drop late frames instead of accumulating delay.
   * - ``-rtsp_transport udp``
     - Receive RTP without TCP retransmission stalls.
   * - ``-max_delay 0 -reorder_queue_size 0``
     - Disable packet-reordering waits for local playback.
   * - ``-buffer_size 2097152``
     - Give UDP packet bursts room in the receive socket; this is capacity, not a target delay.
   * - ``SDL_RENDER_VSYNC=0``
     - Avoid the long presentation waits observed for background test windows.
   * - ``-an``
     - Select video-only playback.

These are explicit viewer settings. They do not change a camera's encoder,
capture mode, ROS decoder parameters, system-wide display configuration, or
ROS discovery configuration.

Override the defaults
---------------------

Put the URL first and additional ffplay options afterward. Later options take
precedence over the helper defaults. For a remote UDP connection that needs
packet reordering, for example::

  ros2 run rtsp_image_transport play_rtsp rtsp://HOST:8555/PATH \
      -max_delay 50000 -reorder_queue_size 32

For a server or network that requires interleaved TCP::

  ros2 run rtsp_image_transport play_rtsp rtsp://HOST:8555/PATH \
      -rtsp_transport tcp

UDP without a reorder queue targets local viewing. On other links, packet
reordering and loss can require different settings. TCP preserves delivery but
can add latency during retransmissions. ``low_delay`` is intended for streams
encoded without frame reordering; sources with B-frames may need different
decoder settings. Explicitly set SDL values are preserved::

  SDL_RENDER_VSYNC=1 ros2 run rtsp_image_transport play_rtsp rtsp://HOST:8555/PATH
  ros2 run rtsp_image_transport play_rtsp --help

ROS image playback
------------------

When a ROS consumer needs decoded images, use the package's receiving launch
file rather than the native viewer. Its input is a ``std_msgs/String`` stream
URL topic::

  ros2 launch rtsp_image_transport rtsp_to_ros.launch.yaml \
      input_topic:=/camera/image_h264/rtsp \
      output_topic:=/camera/image_decoded

This path already defaults to hardware decoding with software fallback, UDP
with a 2 MiB receive buffer, minimal decoder buffering, and best-effort depth-1
output QoS. The relevant existing controls include:

* ``low_latency:=true`` avoids decoder frame reordering and frame-thread
  buffering. Disable it for sources that require B-frame reordering.
* ``max_latency`` is the decoder queue's latency budget in seconds, default
  ``0.2``. The drop ladder starts discarding non-intra frames at this age,
  non-key frames at twice it, and all frames at four times it. Lowering the
  budget trades continuity for freshness during overload; it is not a strict
  end-to-end limit. ``0`` disables dropping and allows delay to grow.
* ``rtp_over_tcp:=true`` selects TCP when required by the camera or network.
* ``timestamp_source`` selects sender, receiver, or automatic timestamps.
  Receiver timestamps are useful when camera and ROS clocks are not aligned,
  but they do not measure exposure-to-display latency.

For example, an application that prefers fresh frames under decoder overload
can request a 50 ms queue budget::

  ros2 launch rtsp_image_transport rtsp_to_ros.launch.yaml \
      input_topic:=/camera/image_h264/rtsp \
      output_topic:=/camera/image_decoded max_latency:=0.05

See the main README's hardware-decoding and latency sections for the decoder
selection, colour-conversion thread, reconnect, and corruption controls.

When joining an already-running H.264/H.265 stream, the decoder can receive
dependent pictures before the next random-access picture. The hardware
startup probe now waits for that picture before counting four packets without
output as a failed device. It retains the initial parameter sets and bounds
the wait and replay storage. This avoids abandoning a working GPU while it
is waiting for references; genuine startup failures still fall back. H.264
and H.265 late-join regressions cover this behavior.

Large decoded ROS images can still exceed a particular host/consumer's
processing capacity. Lowering the queue budget drops more pictures; it does
not increase throughput. Prefer the native viewer when ROS images are not
needed, or reduce camera ROI/resolution when the application permits it.

Capture and encoding
--------------------

For camera sources that support them, native image formats avoid unnecessary
colour conversions. Composing a camera and the encoder into the same process
with intra-process communication avoids serializing large raw images. Queue
depth 1 prevents raw or decoded ROS images from accumulating ahead of live
consumers. A higher capture frame rate reduces frame waiting at the cost of
more capture, encoding, and link bandwidth. These choices depend on the
camera driver and belong in its launch configuration.

The generic H.264/H.265 encoder already uses low-latency presets. On NVENC,
the fork also sets ``delay=0``: ``zerolatency`` alone leaves a separate async
output queue enabled. This fix applies to streams from any supported camera,
not only the ZED example.

Validate the complete path
--------------------------

Measure from the original capture time through presentation. A fast decoder
can still display old frames after capture, encoder, transport, or player
queueing. The `ZED measurement guide <zed_rtsp.rst#encoder-fix-and-measured-latency>`_
describes the per-frame timestamp test and its physical-display limits. Use
an optical camera-to-screen measurement when display/compositor delay must
also be included. Camera clocks must be aligned before comparing timestamps
between machines.
