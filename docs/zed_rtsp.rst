USB ZED capture and low-latency playback
========================================

This package includes the complete USB ZED example: capture, H.264 encoding
through ``rtsp_image_transport``, optional decoding back to ROS, and a native
playback helper. It does not require the Nautilus workspace or ``settings_erdc``.
The `camera profiles <camera_profiles.rst>`_ share the encoder integration
with RealSense and Lucid examples; the defaults and device controls differ
by driver.

Requirements
------------

The example was validated on Linux with a USB 3 ZED 2, ROS 2 Lyrical, FFmpeg
8.0.1, and NVIDIA encoding. Install ``v4l2_camera`` and ``v4l-utils`` for USB
capture; these are optional example dependencies, so the generic transport
does not require a camera driver or its OpenCV dependencies. On Ubuntu, after
sourcing the ROS environment::

  sudo apt install "ros-${ROS_DISTRO}-v4l2-camera" v4l-utils

Build this package in a colcon workspace and source its install overlay.
The launch file, playback helper, this guide, and the measurement summary are
installed with the package. Composed mode uses
``component_container --executor-type multi-threaded``, as provided by the
validated ROS distribution. ``composed:=false`` uses separate processes.

Start capture and playback
--------------------------

On the machine with the USB camera::

  ros2 launch rtsp_image_transport zed_rtsp.launch.py \
      composed:=true output_encoding:=yuv422_yuy2 \
      framerate:=60 bitrate:=12000000

In another terminal with the same overlay sourced::

  ros2 run rtsp_image_transport play_rtsp

The helper defaults to ``rtsp://127.0.0.1:8555/``. It starts only the viewer;
leave the capture launch running. Press Ctrl-C in the capture terminal to
release the camera, and close the viewer or press ``q`` to stop playback.

The default device is
``/dev/v4l/by-id/usb-Technologies__Inc._ZED_2-video-index0``. Override
``video_device`` for another device path. ``video-index1`` is metadata and
cannot supply images.

The image is side-by-side, unrectified stereo at 2560x720: each eye occupies
1280x720. UVC capture does not use the ZED SDK, compute depth, or publish
calibrated per-eye CameraInfo. A missing camera calibration file does not
prevent this video path from working.

Launch options and ROS endpoints
--------------------------------

With no arguments the launch retains the original 30 fps, RGB,
separate-process configuration. The command above selects the measured
low-latency profile explicitly. Inspect all arguments without opening a
camera::

  ros2 launch rtsp_image_transport zed_rtsp.launch.py --show-args

Blank profile-dependent argument defaults in the shared launch interface
resolve to the ZED defaults listed below.

.. list-table:: Principal options
   :header-rows: 1
   :widths: 24 20 56

   * - Argument
     - Default
     - Effect
   * - ``composed``
     - ``false``
     - Run camera and encoder in one process with intra-process image delivery.
   * - ``output_encoding``
     - ``rgb8``
     - ``yuv422_yuy2`` skips the camera driver's RGB conversion.
   * - ``width``, ``height``
     - ``2560``, ``720``
     - Full stereo image dimensions, not per-eye dimensions.
   * - ``framerate``
     - ``30``
     - Configure V4L2 capture and encoder frame rate; 60 fps was validated.
   * - ``bitrate``
     - ``8000000``
     - H.264 target bits per second; the 60 fps profile uses 12000000.
   * - ``rtsp_port``
     - ``8555``
     - RTSP listening port; change the playback URL to match.
   * - ``decode``
     - ``true``
     - Also offer decoded ROS images on demand.
   * - ``camera_namespace``
     - ``/zed``
     - Namespace for raw images, the stream URL, and decoded images.
   * - ``rtp_over_tcp``
     - ``false``
     - Select TCP instead of UDP for the optional ROS decoder.

The default namespace exposes:

* ``/zed/image_raw``: unrectified stereo images, YUYV in the optimized profile
  and RGB with the launch defaults. Raw ROS consumers must accept that encoding.
* ``/zed/image_h264/rtsp``: a reliable, transient-local ``std_msgs/String``
  containing the advertised RTSP URL.
* ``/zed/image_decoded``: decoded stereo BGR images with receive timestamps.

Encoding and ROS decoding are demand driven. A native RTSP viewer can start
encoding without a ROS image subscriber. The camera driver publishes only
the raw image transport; the dedicated encoder component owns the RTSP stream.
The shared process avoids serializing raw images between those two components.

The launch uses the caller's ROS domain, middleware, discovery configuration,
and log directory. It does not impose Nautilus-specific environment settings.
The RTSP listener accepts connections on host network interfaces; remote
clients use the advertised host address instead of ``127.0.0.1``.

Playback options
----------------

``play_rtsp`` is camera independent. The `general playback guide
<low_latency_playback.rst>`_ covers buffering, sync, transport overrides, SDL
configuration, and the ROS decoder's existing latency controls. The tested
profile expands to::

  SDL_RENDER_VSYNC=0 ffplay -rtsp_transport udp -buffer_size 2097152 \
      -fflags nobuffer -flags low_delay -threads 1 \
      -max_delay 0 -reorder_queue_size 0 \
      -framedrop -sync video -noinfbuf -an rtsp://127.0.0.1:8555/

Single-threaded decoding avoids frame-thread buffering. Video clock sync
avoids the external clock's deliberate slowdown while its packet queue fills.
The SDL setting avoids the long presentation waits observed when a test
window was in the background. The socket buffer absorbs packet bursts; its
capacity is not a fixed playback delay.

Supply another URL first, followed by additional ffplay options. Additional
options take precedence over the helper defaults::

  ros2 run rtsp_image_transport play_rtsp rtsp://HOST:8555/
  ros2 run rtsp_image_transport play_rtsp rtsp://HOST:8555/ -rtsp_transport tcp
  ros2 run rtsp_image_transport play_rtsp rtsp://HOST:8555/ \
      -max_delay 50000 -reorder_queue_size 32

UDP with no reorder queue targets local playback. A remote link may need a
reorder queue or TCP; TCP can add delay during retransmission. The helper
preserves an explicitly set ``SDL_RENDER_VSYNC`` value::

  SDL_RENDER_VSYNC=1 ros2 run rtsp_image_transport play_rtsp
  ros2 run rtsp_image_transport play_rtsp --help

Encoder fix and measured latency
--------------------------------

The fork explicitly sets NVENC's ``delay=0`` alongside ``zerolatency=1``.
Zero reordering alone leaves FFmpeg's separate asynchronous output queue
enabled, so the first encoded image waits for future images. The hardware
regression ``StreamEncoder.NvencProducesFirstImageWithoutPrimingFrames`` failed
before the fix and passed afterward. All 31 encoder tests passed on the
validation host.

Measurements on 2026-11-09 used a real ZED 2 at 2560x720 stereo. A temporary
diagnostic wrote each V4L2 monotonic frame timestamp into image pixels before
RGB conversion or ROS publication. The player recovered that timestamp from
decoded pixels and compared it with the completion of ``SDL_RenderPresent``.
The first five displayed seconds were excluded. Each range below contains the
medians from two runs.

.. list-table:: Capture timestamp to renderer on the validation host
   :header-rows: 1
   :widths: 67 10 23

   * - Configuration
     - FPS
     - Median age
   * - RGB, separate processes, external clock, original NVENC settings
     - 30
     - 391 ms
   * - Native YUYV, separate processes, video sync, original NVENC settings
     - 30
     - 175-177 ms
   * - Native YUYV, composed, video sync, original NVENC settings
     - 30
     - 156-161 ms
   * - Native YUYV, composed, video sync, NVENC output delay disabled
     - 30
     - 93-97 ms
   * - Same corrected pipeline, 12 Mbit/s
     - 60
     - 62-67 ms

All rows use ``SDL_RENDER_VSYNC=0``; the test viewport was 1280x360 while the
decoded images remained 2560x720. The final two runs displayed 2451 frames with
valid timestamp patterns on every frame and sustained 60 fps. Their
95th-percentile ages were 63 and 68 ms. The portable measurement summary is
`benchmark/zed_rtsp_latency.json <../benchmark/zed_rtsp_latency.json>`_.

These measurements include capture/USB queueing, ROS delivery, encoding, RTSP,
decoding, and player buffering. They exclude compositor and physical display
scanout, and sensor time outside the V4L2 timestamp. They are measurements on
the validation host, not a latency guarantee for other cameras or machines.
The pixel overlay was temporary test instrumentation and is not included in
normal capture or playback.

If video stops while the RTSP server still responds, check the raw camera
topic and kernel USB messages. During validation, repeated USB errors ``-71``
stopped capture on the original connection; replacing the port/cable allowed
the sustained comparisons to complete. Restart the capture launch after
recovering the USB device.
