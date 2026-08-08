# Nautilus fork notes

Remote: [`jcfurey/rtsp_image_transport`](https://github.com/jcfurey/rtsp_image_transport),
branch `cam_wip`.

This change is based on upstream commit
`d815a176b0634cda8d5f4290f33173b82fab7a6c`. Nautilus adds a zero-based
`video_subsession` subscriber parameter. Upstream 2.0.2 always selects the
first supported video subsession, but the Deep Trekker 4K camera advertises
HEVC first and two H.264 streams after it. The parameter made subsession 1 a
working 1024x600 H.264 fallback during initial integration.

Subsession selection now enumerates every video subsession the server offers
and logs codec, resolution, and frame rate at connect time, so the right index
does not have to be guessed. Only the selected subsession is initiated, and an
out-of-range or unusable selection falls back to the remaining subsessions
instead of failing the session.

The fork now also initializes H.265 from its RFC 7798 SDP attributes. Upstream
looked for H.264's combined `sprop-parameter-sets` value for both codecs; this
camera instead supplies H.265 parameter sets separately as `sprop-vps`,
`sprop-sps`, and `sprop-pps`. Camera0 targets subsession 0, the camera's
highest-quality advertised LAN track (1920x1080 HEVC at 30 Hz).

The fork also replaces the embedded placeholder PNG with a 1x1 BGR ROS image
and removes the otherwise unnecessary `cv_bridge`/OpenCV dependency. This keeps
the transport independent of the CUDA-enabled OpenCV build in the Nautilus
runtime image.

## Hardware decoding

Upstream listed a couple of standalone hardware decoders (`h264_cuvid`,
`h264_qsv`, ...) and picked the first one FFmpeg happened to know about. There
was no hwaccel device path at all, so VAAPI — the standard on Intel and AMD
Linux — was never used, and a hardware decoder that returned frames in device
memory would have crashed the colour conversion.

The subscriber now builds an ordered list of ways to decode the stream and uses
the first that works:

1. native decoder + hardware device context (CUDA/NVDEC, VAAPI, QSV, VDPAU,
   Vulkan, VideoToolbox, D3D11VA), derived from `avcodec_get_hw_config()` so
   only combinations FFmpeg actually supports are offered;
2. standalone hardware decoders, including `*_rkmpp` and `*_v4l2m2m` for ARM
   SoCs;
3. software.

Device contexts are created once per process and shared, so N cameras use one
GPU context rather than N. Frames that come back in device memory are pulled
into system memory with `av_hwframe_transfer_data()` before conversion. If a
hardware decoder opens but then fails to decode, the subscriber logs a warning
and continues with the next candidate instead of dropping the stream.

Two traps found while testing this: `h264_qsv` opens successfully on a machine
with no Quick Sync and only fails on the first packet, and `hw_device:=none`
still reached the standalone hardware decoders. Both are handled — the
standalone decoders are now gated on their vendor device being creatable.

New subscriber parameters `hw_device` (auto/none/cuda/vaapi/qsv/...) and
`decoder` (pin one FFmpeg decoder by name). `use_hw_decoder` keeps its meaning.

Decode-only support for MPEG-2 (`MPV`) and H.263 (`H263`, `H263-1998`,
`H263-2000`) was added alongside, since those RTSP payload names still turn up
on older cameras.

## RTP transport and lossy links

Upstream never passes `streamUsingTCP` to `sendSetupCommand()`, so Live555's
default applies and RTP runs over its own UDP sockets. Nothing calls
`increaseReceiveBufferTo()` either, which leaves the RTP socket on the kernel
default receive buffer — well under one HD frame. On the Nautilus tether this
showed up as flat green bands over part of the decoded image, and the system UDP
counters attributed every input error to receive-buffer overflow.

The green is the tell. A lost datagram costs a slice; `AV_CODEC_FLAG2_CHUNKS`
makes the decoder emit the picture from the slices that did arrive, and the
macroblocks the missing one covered keep the value their buffer was allocated
with. Zero-filled YUV converts to BGR (0, 135, 0).

Three changes:

- New `rtp_over_tcp` subscriber parameter, **on by default**. RTP is interleaved
  over the RTSP connection, which removes the loss rather than tuning around it.
  For a camera on one Ethernet link this is close to strictly better: no loss, no
  reordering, no buffer sizing.
- New `rtp_buffer_size` parameter (2 MB default), applied to the RTP socket
  between `initiate()` and SETUP when UDP is selected. The kernel caps the
  request at `net.core.rmem_max` and the subscriber logs a warning when it does.
- Error concealment is enabled again. The fork had set `error_concealment = 0`
  to quieten the log, but `ff_er_frame_end()` returns early when concealment is
  off, so a partially decoded frame was neither repaired from the reference
  frame nor recorded in `decode_error_flags` — it simply reached the subscriber
  with holes in it. The log noise it was silencing is already demoted to
  warnings by `log_level_offset`.

On top of that the decoder no longer publishes pictures it could not fully
reconstruct at session start: frames are held until the first key frame, bounded
at 120 frames so a stream that never flags one still produces video. New
`drop_corrupt_frames` parameter (off by default) discards damaged frames outright
instead of publishing the concealed result.

Regression tested by reconstructing the real failure: a stream encoded with four
slices per picture, with one slice NAL removed from every inter frame. Concealing
delivers all 30 of 30 frames; `drop_corrupt_frames` keeps only the 3 undamaged key
frames, which is what confirms the damage is being detected at all.

## QoS, ROS time and bags

- The URL topic keeps its required RELIABLE / TRANSIENT_LOCAL / KEEP_LAST(1)
  policies, but an incompatible peer is now reported instead of silently
  producing no video. Everything else the caller asks for is passed through.
- New subscriber parameter `timestamp_source` (sender / receive / auto).
  Automatic is the default and switches to receive time when `use_sim_time` is
  active, because a camera's wall clock stamp is meaningless in a simulated
  time base and every downstream TF lookup and message filter would reject it.
- The publisher maps ROS image stamps onto a strictly increasing wall clock
  timeline before they reach RTP, re-anchoring when the clock jumps. Simulated
  time starting near zero no longer produces RTCP sender reports dated 1970,
  and a looping bag no longer sends RTP time stamps backwards.
- The encoder paces frames by the gap between them rather than from a fixed
  origin. A bag loop used to clamp every subsequent frame to one tick after its
  predecessor, so the encoder concluded the stream ran at 300 fps and its rate
  control collapsed.
- README documents how to get video into a bag at all (`image_transport
  republish rtsp raw`, then record the raw topic), since recording the URL
  topic captures nothing useful.

## image_transport 6.4 and later

image_transport 6.4 replaced the plugin entry points with ones taking node
interfaces and no longer calls the old ones. `RequiredInterfaces` is
`NodeInterfaces<Base, Parameters, Logging, Timers, Topics>`, which lacks the
clock, waitables, and graph interfaces used by the original plugins.

The subscriber now supports this API. It drains the decoder queue with a 2 ms
wall timer registered through the provided base/timers interfaces instead of a
custom waitable, and it declares and monitors parameters directly through the
parameters interface. Because the node clock is not provided, this path uses a
system clock for receive timestamps; callers on 6.4+ should explicitly select
`timestamp_source:=1` when receiver timestamps are wanted. The FLIR A700 launch
does this by default.

The image_transport publisher plugin remains unavailable on 6.4+ because its
subscriber-graph monitoring cannot be recreated from the provided interfaces.
This does not affect camera reception through `publish_rtsp_stream`, which
publishes the RTSP URL directly and is the pattern used by the Deep Trekker and
FLIR adapters. End-to-end tests that require the publisher plugin still skip on
these versions.

6.4 and 7.0 are separate boundaries, and conflating them is what kept Rolling
from compiling. 6.4 added the node-interface entry points and stopped calling
the old ones, but still declares them, so an override of the old signature
compiles and is simply never invoked. 7.0 removed them, and an override of a
method that no longer exists is an error — which is what both plugins were,
since each declared the legacy entry point unconditionally. The two boundaries
now have their own macros in `init.h`:
`RTSP_IMAGE_TRANSPORT_USES_NODE_INTERFACES` (>= 6.4) decides which entry point
does the work, and `RTSP_IMAGE_TRANSPORT_HAS_LEGACY_PLUGIN_API` (< 7.0) decides
whether the legacy one may be declared at all. Rolling now builds and passes
the suite, with the subscriber on its node-interface path.

## Build system

Converted to `ament_cmake_auto`. package.xml is now the single place the ROS
dependencies are listed; the executables and test binaries pick them up
automatically, and `ament_auto_package()` handles installation and export.
Two things still need naming explicitly: `live555_vendor` exports its CMake
package as `live555`, and ffmpeg comes from pkg-config.

The plugin module is deliberately *not* an `ament_auto_add_library` target.
CMake refuses to link a MODULE library into anything, so registering it would
break every ament_auto target declared after it, including the tests, and
exporting a pluginlib module as a linkable library would be wrong anyway.

Two portability problems fell out of the conversion, both of which also affect
the unmodified upstream package:

- `ros2 run rtsp_image_transport publish_rtsp_stream` never worked, because
  the executables were installed to `bin/` while `ros2 run` looks in
  `lib/<package>`. Both the README and the program's own usage message tell
  you to use `ros2 run`. The ament_auto default install location fixes it.
- Kilted and later no longer provide the `sensor_msgs::sensor_msgs` style
  alias, so upstream does not configure there at all. Message packages are now
  linked through `${sensor_msgs_TARGETS}`, and `ament_target_dependencies()` is
  avoided because Lyrical removed it.

Builds on Jazzy, Kilted, Lyrical and Rolling. Lyrical and Rolling use the
node-interface subscriber path used by the FLIR adapter; the legacy entry
points remain for image_transport 6.3 and earlier and are compiled out from 7.0.

## Tests

`test/` holds a GoogleTest suite (104 cases) run with `colcon test`. It covers
codec name mapping, `FrameData`, the decoder across every supported codec plus
hardware selection and fallback, the encoder, an encode/decode round trip, the
Live555 event loop, the graph monitor, and a loopback integration test that
runs a real RTSP session over localhost. Cases skip themselves when an encoder
or a GPU is absent, so the same suite is meaningful on a headless builder and
on a machine with an iGPU.

Building the sources into a static `rtsp_image_transport_core` library made
this possible: the plugins stay in the loadable module, everything else can be
linked into a test binary.

Verified on ROS 2 Jazzy and Lyrical in the official `ros:*-ros-base` images.
Three of the bugs listed below were found by these tests, not by reading.

## Latency and throughput work

- The subscriber's frame-dropping ladder never escalated past "discard
  non-intra frames" because of a missing `else`, so a decoder that fell far
  behind stayed behind. It now escalates to key-frames-only and drop-everything
  as designed.
- New `low_latency` subscriber parameter (default on) sets
  `AV_CODEC_FLAG_LOW_DELAY` and restricts FFmpeg to slice threading, removing
  the decoder's reorder and frame-threading delay.
- Decoded frames carry the presentation time of the access unit they came
  from, propagated through the decoder as a nanosecond PTS, instead of the time
  stamp of whichever packet happened to flush them out.
- Colour conversion runs through `sws_scale_frame()` on up to four threads
  where libswscale supports it, writing straight into the ROS image message.
  Measured 3.6 ms -> 1.2 ms per 4K frame, bit-identical output.
- The RTSP server's Live555 scheduler runs at 1 ms granularity instead of
  10 ms, which is what bounds how long a frame handed over by the ROS publisher
  thread waits before it is sent.
- The client stops iterating subsessions once one is playing, so PLAY is sent
  a round trip earlier.

## Bugs the test suite found

- `scanForStartCode` skipped a three byte Annex B start code whenever the
  preceding byte was not zero, because the scan advanced four bytes past it.
  Two NAL units were then merged into one packet and handed to Live555, which
  packetises it as a single malformed NAL. Any multi-slice stream is affected,
  which is what the `slice-max-size` hint produces for x264. Rewritten and
  checked against a brute force reference over 200k random buffers.
- Destroying a `StreamClient` or `StreamServer` could terminate the process.
  Live555 callbacks resurrect a `shared_ptr` to their owner, so when the last
  external reference is dropped concurrently the owner is destroyed on the
  Live555 thread, and its destructor joined that thread from inside itself
  ("Resource deadlock avoided"). The scheduler and its thread now live in a
  shared `EventLoop` that the thread keeps alive itself, so nothing is ever
  joined. Reproduced on Lyrical.
- `GraphMonitor`'s detached thread let exceptions escape. `wait_for_graph_change`
  throws once the context is shut down, which happens routinely while a
  publisher is still advertised, and an exception leaving a detached thread
  terminates the process.
- libvpx looks ahead 25 frames by default for VP9, so nothing reached the wire
  for most of a second. `lag-in-frames` is now zero for VP8/VP9, and the AV1
  encoders get their real low latency options instead of a `speed` setting
  libaom does not have.

## Bug fixes on top of upstream

- `connect()` held the client mutex while Live555 could invoke the failure
  handler synchronously (unparsable URL, unresolvable host), which re-entered
  `disconnect()` and deadlocked the executor thread.
- The frame extractor's fixed 128 KiB buffer silently truncated large key
  frames; it now grows on demand up to 16 MiB and drops truncated NAL units
  rather than feeding them to the decoder.
- `udp_packet_size` was compared against itself, so changing it at runtime did
  nothing.
- The publisher dereferenced a null `StreamServer` if a codec change followed a
  setup failure, and a failure was permanent until the node restarted.
- Multicast RTP sink, RTCP instance, and framer were leaked on every session
  restart.
- `GraphMonitor` reset its singleton from a detached thread without holding the
  lock, and spun on it under that same lock from `instance()`.
- The unicast RTP port base came from an unseeded `random()`, so every process
  probed the same port first.
- Null-pointer and buffer-underflow guards: `sws_getContext` failures, empty
  Live555 result strings, `client_` use after a failed setup, and
  `scanForStartCode` on packets shorter than four bytes.
- Missing `<algorithm>`, `<format>`, and `<functional>` includes that only
  resolved through transitive ROS headers.

Keep upstream's Apache-2.0 license and copyright notices when rebasing. The
intended upstreamable changes are configurable video subsession selection,
codec-correct H.265 SDP parameter-set handling, the latency work, and the bug
fixes listed above.
