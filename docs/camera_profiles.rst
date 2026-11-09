Camera capture profiles for RTSP playback
=========================================

The package provides ZED, RealSense color, and Lucid GigE entry points around
one shared ``camera_rtsp.launch.py`` pipeline. Each profile connects its
driver's raw image to the same H.264 RTSP encoder and optional ROS decoder.
The installed ``play_rtsp`` helper works with every profile.

.. list-table:: Available profiles
   :header-rows: 1
   :widths: 20 40 40

   * - Entry point
     - Driver
     - Profile defaults
   * - ``zed_rtsp.launch.py``
     - ``v4l2_camera``
     - 2560x720 stereo, 30 fps, RGB, separate processes for compatibility.
   * - ``realsense_rtsp.launch.py``
     - ``realsense2_camera``
     - Color-only 1280x720 at 30 fps, RGB8, composed capture and encoding.
   * - ``lucid_rtsp.launch.py``
     - ``lucid_camera_driver`` / Arena SDK
     - Device-native format/ROI, 15 fps, composed capture and encoding.

These drivers and their SDKs are optional. Building the RTSP transport and
inspecting launch arguments does not require a RealSense or Lucid driver.
Actual capture requires the matching driver, SDK, camera, and supported mode.
The core transport does not acquire their OpenCV or proprietary SDK dependencies.

RealSense color playback
------------------------

Install a ``realsense2_camera`` driver and matching librealsense SDK, source
that overlay, and start::

  ros2 launch rtsp_image_transport realsense_rtsp.launch.py
  # In another terminal:
  ros2 run rtsp_image_transport play_rtsp

For a specific device and a supported native YUYV mode::

  ros2 launch rtsp_image_transport realsense_rtsp.launch.py \
      serial_number:=123456789 color_format:=YUYV \
      width:=1280 height:=720 framerate:=30

The profile uses ``realsense2_camera::RealSenseNodeFactory`` and remaps its
private ``~/color/image_raw`` topic to ``/realsense/image_raw``. The serial
number is passed as a string. ``rgb_camera.color_profile`` and
``rgb_camera.color_format`` request the selected mode, and ``color_qos`` uses
``SENSOR_DATA``. The driver must support the requested profile and format;
inspect its logged profiles or parameter descriptions on the actual device.
RGB8 is the compatible default; YUYV can avoid SDK RGB conversion where the
device exposes that format.

The connected D435i was also verified in this supported 60 fps mode::

  ros2 launch rtsp_image_transport realsense_rtsp.launch.py \
      color_format:=YUYV width:=848 height:=480 framerate:=60
  # In another terminal:
  ros2 run rtsp_image_transport play_rtsp

This trades resolution for a shorter frame interval. On the tested D435i,
1280x720 supports 30 fps, while 848x480 supports 60 fps. Check the particular
camera with ``rs-enumerate-devices`` and confirm the driver's logged
``Open profile`` matches the requested format, dimensions, and frame rate.
Do not request 1280x720 at 60 fps on that device. Other RealSense models may
expose different modes.

The playback profile enables color and disables depth, infrared, IMU,
RGB-D publication, point clouds, alignment, and cross-stream synchronization.
This avoids starting work that a color RTSP viewer does not consume. It is
intended for devices with a color sensor. Devices whose color stream is exposed
through another sensor module need a matching driver configuration.

For an application that also needs depth or other streams, provide a ROS
parameter file. That file is applied after the profile parameters and can
override them::

  ros2 launch rtsp_image_transport realsense_rtsp.launch.py \
      driver_params_file:=/path/to/realsense_driver.yaml

For example, the driver YAML can contain::

  /**:
    ros__parameters:
      enable_depth: true
      depth_module.depth_profile: "640,480,30"

The RTSP encoder still consumes only color. Metric depth images are not
silently converted to lossy H.264 video. Keep the application's native depth
transport for depth data.

An SDK's frame-retention limit is not automatically a playback queue target.
This profile does not impose a universal ``frames_queue_size`` value; SDK
option support and frame-drop behavior require validation on the actual model.

Lucid GigE playback
-------------------

Install the ERDC ``lucid_camera_driver`` exposing
``Lucid_Driver::Lucid_Camera``, its message package, and the Arena SDK. The
profile was checked against the local ``driver-hardening`` source API::

  ros2 launch rtsp_image_transport lucid_rtsp.launch.py serial_number:=220903154
  # In another terminal:
  ros2 run rtsp_image_transport play_rtsp

Live playback was also checked with a TRI050S1-Q at 10.2.2.36, serial
233100494. With that camera's existing native format/ROI, this command
preserved ``bayer_rggb8`` at 2448x2048 and delivered 15 fps::

  ros2 launch rtsp_image_transport lucid_rtsp.launch.py serial_number:=233100494

The serial is passed as a string, including an all-numeric serial. This driver
selects by serial only when it has the expected nine-character form. Its image
topic becomes ``/lucid/image_raw``.

``image_encoding`` is empty by default, preserving the camera's current PFNC
format and avoiding an unconditional switch to on-camera RGB output. The driver
reads the actual PFNC format to label each ROS image. Its supported playback
formats are 8-bit mono, RGB/BGR, or Bayer; packed and higher-bit-depth formats
need a supported 8-bit device mode. For a color camera supporting BayerRG8::

  ros2 launch rtsp_image_transport lucid_rtsp.launch.py \
      serial_number:=220903154 image_encoding:=bayer_rggb8 framerate:=30

For a monochrome camera supporting Mono8, use ``image_encoding:=mono8``.
Select a format and frame rate the specific camera supports. The RTSP encoder
accepts supported native Bayer/mono encodings and performs the required video
conversion; a separate ROS RGB conversion is not required.

The checked driver already selects Arena ``StreamBufferHandlingMode=NewestOnly``.
The profile does not invent a ROS parameter for that SDK setting. It also does
not change the driver's hardcoded raw publisher depth of 10. The encoder's raw
subscription is best-effort with depth 1, and composition avoids the raw-image
serialization hop between driver and encoder.

For a standard-MTU network the profile disables packet auto-negotiation and
requests 1400-byte GigE packets, including IP/UDP headers. This avoids the
observed case where a camera persisted for jumbo packets fails negotiation on
a standard-MTU host. Change ``stream_packet_size`` and
``stream_auto_negotiate_packet_size`` only to match the actual camera/NIC/path;
the launch does not modify host MTU, network interfaces, or system buffers.

Use ``configuration_file`` for the driver's GenICam ``lucid_params`` YAML,
including supported ROI, exposure, and throughput controls::

  ros2 launch rtsp_image_transport lucid_rtsp.launch.py \
      configuration_file:=/path/to/lucid_genicam.yaml

This is distinct from ``driver_params_file``, which contains ROS parameters.
Lucid ROI is configured through GenICam rather than the generic ``width`` and
``height`` arguments. The GenICam file is applied after the driver's initial
settings; keep its acquisition rate consistent with ``framerate``. Exposure
and throughput limits depend on lighting, sensor mode, link capacity, and
other cameras sharing that link; the profile does not force universal values.

``enable_ptp:=true use_camera_time:=true`` is available for an appropriately
configured synchronized-camera setup. Camera timestamps must be in the
intended clock domain before using them for frame-age comparisons. PTP does
not itself remove capture, encoding, or playback queues.

Common transport settings
-------------------------

All profiles accept ``composed``, ``framerate``, ``bitrate``, ``rtsp_port``,
``camera_namespace``, ``camera_name``, ``decode``, ``rtp_over_tcp``,
``low_latency``, ``max_latency``, ``timestamp_source``, ``frame_id``, and
``driver_params_file``. ``max_latency`` controls the optional ROS decoder's
queue budget; it defaults to 0.2 seconds and is not an end-to-end limit. For
example, ``max_latency:=0.05`` requests a 50 ms budget. Blank profile-dependent arguments
select the defaults in the table above. Inspect the full interface with::

  ros2 launch rtsp_image_transport camera_rtsp.launch.py --show-args

The direct shared entry point also accepts
``camera_type:=zed|realsense|lucid``. Each named entry point selects that type.
``composed:=false`` starts the driver and encoder as separate processes.

Raw images, the stream URL, and decoded images use
``/<camera_type>/image_raw``, ``/<camera_type>/image_h264/rtsp``, and
``/<camera_type>/image_decoded`` by default. Override ``camera_namespace`` to
change the common prefix. Each simultaneously running RTSP server needs its
own port::

  ros2 launch rtsp_image_transport realsense_rtsp.launch.py rtsp_port:=8556
  ros2 run rtsp_image_transport play_rtsp rtsp://127.0.0.1:8556/

The core NVENC ``delay=0`` fix, encoder low-delay settings, depth-1 encoder
subscription, native-viewer profile, and ROS decoder's existing latency controls
apply across camera types. See `general playback <low_latency_playback.rst>`_
for transport and viewer overrides, and `ZED playback <zed_rtsp.rst>`_ for its
measured 30/60 fps profiles.

Existing camera applications and other drivers
----------------------------------------------

If another application already owns the camera, keep its driver running and
attach the generic encoder to its existing raw image topic::

  ros2 launch rtsp_image_transport ros_to_rtsp_h264.launch.yaml \
      input_topic:=/existing_camera/image_raw \
      output_topic:=/existing_camera/image_h264/rtsp
  ros2 run rtsp_image_transport play_rtsp

This works with other supported ROS image sources as well. To avoid a raw
serialization hop in an existing component application, load its camera driver
and ``image_transport::Republisher`` into the same container with intra-process
communication enabled, as demonstrated by the shared camera launch. Vendor
device configuration remains in the camera's driver; the RTSP package owns the
encoder, decoder, viewer, and reusable launch integration.

Validation scope
----------------

The ZED settings have real-camera capture-to-renderer measurements. RealSense
profile controls and topic naming were checked against the local ROS wrapper
source and the source matching the installed 4.58.4 driver. A connected D435i
was then tested on 2026-11-09 with firmware 5.13.0.50, librealsense 2.58.4,
ROS wrapper 4.58.4, ROS 2 Lyrical, and USB 3.2:

* RGB8 at 1280x720/30 fps: capture, H.264 NVENC encoding, and 60 decoded
  RTSP frames verified.
* YUYV at 1280x720/30 fps: serial-based device selection, native
  ``yuv422_yuy2`` ROS publication, and decoded ``bgr8`` ROS frames verified.
* YUYV at 848x480/60 fps: composed capture/encoding, native playback,
  180 decoded RTSP frames from an additional client, and sustained raw and
  decoded ROS delivery verified. A roughly ten-second check received 596 raw
  images and 556 decoded images, including decoder startup; the observed raw
  rate was 59.53 fps.

These checks validate the capture and playback paths, rather than establish
an optical camera-to-screen latency guarantee. A preliminary timing probe
added visible timestamp blocks to the upper-left pixels; that diagnostic is
not part of the package or its launch commands. Normal playback was restored
and verified without the probe. The ZED latency numbers are not measurements
of RealSense or Lucid cameras.

Lucid controls were checked against the local ``driver-hardening`` source.
Its driver and message package were then built in a separate overlay against
an existing local x64 Arena SDK. The TRI050S1-Q, firmware 1.141.0.0, was
tested on 2026-11-09 through this package's composed launch:

* Native 2448x2048 ``bayer_rggb8`` capture delivered 15 fps with 1400-byte
  GigE packets and packet auto-negotiation disabled.
* H.264 NVENC encoding, the native viewer, an additional RTSP client decoding
  60 frames, and decoded ``bgr8`` ROS images were verified.
* The test exposed premature GPU rejection when a ROS decoder joined between
  key frames. The core fix kept the live decoder on CUDA; H.264/H.265
  late-join regressions and the decoder/RTSP suites passed.
* Full-resolution decoded ROS delivery remained around 10.6 fps in the
  simultaneous raw/decoded subscription check. Increasing the existing
  conversion-thread setting from 4 to 16 did not improve it, nor did receiving
  serialized messages instead of deserializing images. No universal thread
  increase is applied. Use a supported smaller ROI when full-rate ROS images
  matter more than the full field of view. Native RTSP playback is a separate
  path, and no optical latency figure was measured for this camera.

Hardware-free tests check typed serials/parameters, color-only RealSense
routing, native Lucid configuration, matching camera/encoder topics, optional
dependencies, and both composed and separate launch construction. The driver
and SDK source trees were not modified for these live checks.

Reference material
------------------

The local checkout of `video_pipeline_ws
<https://public.git.erdc.dren.mil/JCFurey/video_pipeline_ws.git>`_ matched its
remote HEAD ``99cfd8c9f24e41621da4f4dec5c917384437d081`` when reviewed. Relevant
source paths in that revision include:

* ``results/lucid_driver_hardening.md``: live TRI028S-C bring-up, packet-size
  negotiation failures, serial typing, stale acquisition recovery, and
  throughput tuning. Its throughput cap was a mitigation for observed burst
  instability, not a proven universal latency improvement or confirmed PoE
  root cause.
* ``src/video_pipeline_bringup/config/cameras/lucid.yaml`` and
  ``src_optional/lucid_camera_driver``: the camera and driver controls used by
  this profile. The profile uses the existing optional driver API; it does not
  copy or alter the vendor SDK.
* ``src_optional/realsense-ros/realsense2_camera``: color-profile parameters,
  private image topic naming, component support, and native YUYV ROS encoding.
* ``docs/measurement-methodology.md`` and
  ``docs/algorithms/frame-age-calculation.md``: timestamp origins and why
  subscriber callback age alone is not exposure-to-display latency.

The upstream RealSense source for version 4.58.4 was also checked against the
installed wrapper's topic naming and QoS parameter registration.
