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

A third one behaved worse than either, because it looked like success. When a
device opened but could not handle the particular stream — a 10 bit or 4:2:2
profile the iGPU does not implement, most often — `get_format` was handed a
list without the hardware pixel format in it and picked a software format
instead. libavcodec then decoded the whole stream on the CPU while the
transport went on reporting `hevc + vaapi`, so the only symptom was a busy
core. That case now declines the format, which drops the stream to the next
candidate: usually another GPU path that does support the profile, and only
then software. Either way `description()` and `isHardwareAccelerated()` agree
with what is actually running, and the reason is in the log.

New subscriber parameters `hw_device` (auto/none/cuda/vaapi/qsv/...) and
`decoder` (pin one FFmpeg decoder by name). `use_hw_decoder` keeps its meaning.

Decode-only support for MPEG-2 (`MPV`) and H.263 (`H263`, `H263-1998`,
`H263-2000`) was added alongside, since those RTSP payload names still turn up
on older cameras.

## H.265 slice loss

The green banding work assumed libavcodec would repair a picture that lost a
slice, and report it when it could not. That holds for H.264 and the other
mpegvideo-family decoders, which is what it was measured on. It does not hold
for H.265, and H.265 is what the cameras stream.

libavcodec has no error resilience for HEVC. `ff_er_frame_end()` is never
reached, so a picture missing a slice is neither repaired from the reference
frame nor recorded anywhere: `decode_error_flags` stays zero and
`AV_FRAME_FLAG_CORRUPT` stays clear. Measured on a four-slice 640x480 stream
missing one slice from every inter frame:

| | H.264 | H.265 (before) | H.265 (after) |
| --- | --- | --- | --- |
| worst flat-green area of a published frame | 0% | **27%** | 0% |
| frames kept with `drop_corrupt_frames` | 3 of 30 | **30 of 30** | 3 of 30 |

So the codec that needed the guard was the one it did not cover, and the
`drop_corrupt_frames` escape hatch silently did nothing.

The decoder now pre-fills every frame it allocates, for the codecs libavcodec
will not conceal for (H.265, VP8/VP9, AV1), with luma 1 and neutral chroma
instead of leaving the buffer at zero. Two things follow. Anything the decoder
never writes reads as black rather than as the BGR(0,135,0) that an all-zero
YUV block converts to, so a lost slice is no longer a fluorescent green band.
And what is left of the fill afterwards is exactly the region that was never
decoded, which is the signal `drop_corrupt_frames` needs and libavcodec does
not provide. It is sampled on every eighth pixel in both directions, and
requires neutral chroma as well as the luma value, because luma 1 does occur in
real picture content while the pair does not.

Costs one `memset` per decoded frame: 0.10 ms at 1080p and 0.42 ms at 4K,
against 6.4 ms and 26.9 ms of decoding, so about 1.6% either way. Hardware
frames are left alone — they live in device memory, and a GPU decoder handles
slice loss itself.

## The "you forgot to remap" warning never fired

`publish_rtsp_stream` checks whether its image topic was remapped and warns if
not, because the URL is useless on the default name. The check compared
`resolve_topic_name("image")` against the string `"image"` — and resolution
always returns an absolute name, `/image` at the root or `/robot/image` under a
namespace. It was never equal, so the warning has never appeared for anyone, in
this fork or upstream. The comparison now goes against the name the topic
resolves to when nothing remaps it.

Found by measuring coverage rather than by reading: both command line programs
had no test at all, and they are the documented entry points — the camera
adapters announce their streams through `publish_rtsp_stream`. They now get
run as child processes and driven the way a user drives them, which also turned
up that `publish_rtsp_stream` was the one target still built at the compiler's
default C++ standard rather than C++20, and that it never called
`rclcpp::shutdown()`.

One thing running them turned up is a middleware difference rather than a bug
here, worth knowing because it looks like one. The placeholder image on the raw
topic is published exactly once, latched. On Fast DDS that single
transient-local `sensor_msgs/Image` reaches a late joiner on Jazzy and Kilted
but not on Lyrical or Rolling; the `std_msgs/String` URL published moments later
by the same process reaches it on all four. Measured directly: republishing the
same image at 5 Hz makes it arrive on Lyrical, with either durability.

The behaviour is left alone. Republishing to force it would put a stream of 1x1
frames on the user's image topic and skew anything measuring the rate there, and
republishing the *URL* would be worse — the subscriber tears down and rebuilds
its RTSP session on every URL message, so a periodic republish would restart the
stream once a period. The test asserts what the program guarantees, which is
that the topic is advertised and the sample, if the middleware delivers it, has
the right shape.

## SDP parameter sets survive a buffer growth

The VPS/SPS/PPS from the SDP are copied to the front of the frame buffer once,
in the extractor's constructor, and travel to the decoder in front of the first
NAL unit. A NAL unit too large for the buffer resets `buffer_length_` to zero
before growing it, which threw them away — and the camera this fork exists for
is exactly the kind that announces them out of band and never repeats them.

The initial buffer is 256 kB and Live555 hands over one NAL unit at a time, so
a single 4K key-frame slice is enough to trigger it. The decoder then has no
parameter sets for the rest of the session and cannot start at all; only a
reconnect recovers.

The parameter sets are now retained in Annex B form and put back whenever the
buffer is emptied without having been delivered. Re-sending them is harmless —
a decoder that already has them ignores a repeat.

This now has a direct test, which it did not when the fix was written. A
`MediaSubsession` does not need a server behind it — `MediaSession::createNew()`
parses an SDP string — and a `FramedSource` can be scripted, so the whole path
runs with no network: a synthetic SDP carrying the VPS, SPS and PPS of a real
H.265 stream, then a NAL unit larger than the buffer, then an ordinary one. The
test fails on the unfixed code every time.

## MPEG-4 hardware encoding produced MJPEG

The encoder candidate list for MPEG-4 began `mjpeg_vaapi`, `mjpeg_qsv`. Those
are MJPEG encoders — `AV_CODEC_ID_MJPEG`, not `AV_CODEC_ID_MPEG4` — and the
stock Ubuntu FFmpeg has both. With `use_hw_encoder` on a machine with an iGPU
they open, succeed, and encode every frame, so the publisher emitted MJPEG and
announced it over RTP as MPEG-4 Part 2. Nothing downstream can decode that.

It stayed invisible because it needs the hardware to reproduce: a builder with
no render node skips both names for want of a device and quietly lands on
`libxvid`. Neither VAAPI nor Quick Sync encodes MPEG-4 Part 2 at all, so the
entries are simply gone rather than replaced.

The tables are now checked against libavcodec: every name in every encoder and
decoder list must report the codec ID of the row it sits in.
`avcodec_find_encoder_by_name()` answers that whether or not a device exists,
so the check runs on any machine — which is the point, since this is precisely
the class of mistake that only bites where the hardware is.

## Multicast crashed on teardown, and nothing ran it

`use_multicast` had no test at all. Writing one crashed on the first call, and
the fix took two attempts because the ownership rule is the opposite of what it
looks like.

Live555's H.264 and H.265 sinks put an `H264or5Fragmenter` between themselves
and the source. A `FramedFilter` normally closes its input source when
destroyed, so the obvious reading is that the sink owns the chain — but
`~H264or5Fragmenter` calls `detachInputSource()` for exactly this reason, so it
does not. The source stays ours. What the sink *does* do is reach back into the
fragmenter while being destroyed: `~H264or5VideoRTPSink` puts it back in
`fSource` and calls `stopPlaying()` once more.

So the source has to be closed, and it has to be closed after the sink.
Closing it first — which is what the code did — is a use-after-free inside the
sink's destructor. Skipping it, which was the first fix, leaks the framer and
the `FrameInjector` behind it. AddressSanitizer caught the leak that
sequence-of-events reasoning had missed; the tests now run every codec the
server can serve, twice each, so a future Live555 changing its mind shows up.

Underneath that was a second one. `StreamServer::start()` and `stop()` built and
destroyed Live555 objects on whichever thread called them, while the event loop
dispatched into the same objects. Unicast hides it, because nothing transmits
until a client attaches; a multicast sink transmits from the moment it starts
playing, so teardown freed a groupsock out from under an in-flight `sendto()`.
Both entry points and the destructor now hand their work to the loop thread, the
same way `StreamClient` already did.

## The graph monitor aborted the process on node destruction

This is the intermittent test failure that had been written off as a flake. It
reproduces 10 times in 25 on Rolling once you run the binary on its own:

    [ERROR] [rclcpp]: caught std::exception exception in GraphListener thread:
            failed to trigger notify guard condition because it is invalid
    terminate called after throwing an instance of 'std::runtime_error'

`GraphMonitor` held the node's `NodeGraphInterface` in a shared pointer and
watched it from a detached thread, so the graph interface routinely outlived
the node. `NodeGraph` keeps only a raw `NodeBaseInterface*`, and its destructor
is what unregisters the node from rclcpp's own `GraphListener` — deferring that
destructor past the node leaves the listener polling a node whose base is gone.
It then throws out of its own thread, which is fatal.

Holding the base interface alive alongside the graph interface fixes the abort.
`notify_graph_change()` is also no longer called once the context has been shut
down, where it throws rather than being ignored.

Worth stating plainly because the failure was not confined to tests: the
publisher plugin is what uses `GraphMonitor`, and a node shutting down while a
publisher was still advertised is exactly the reproducing sequence.

### A second fault underneath it

Removing the abort left a rarer failure with no crash at all: after a monitor
had been shut down and restarted, the new one never reported a graph change
again. Instrumenting showed it sitting in `wait_for_graph_change` for the full
timeout with its flag clear, while a manual `notify_graph_change()` from
`removeListener` woke it instantly — so the node's own graph changes were not
reaching it.

The reason is in rclcpp. `NodeGraph::get_graph_event()` registers the node with
the `GraphListener` only on the first call:

    if (should_add_to_graph_listener_.exchange(false)) {
      graph_listener_->add_node(this);
      graph_listener_->start_if_not_started();
    }

and `GraphListener::run_loop()` leaves out any node that currently has no
graph users when it builds its wait set:

    if (node_ptr->count_graph_users() == 0) { continue; }

A monitor that shuts down when its last listener leaves releases the node's
last graph event, so the count falls to zero and the node drops out of the wait
set. The next monitor's `get_graph_event()` raises the count again but the
one-shot has already fired, so the listener is never interrupted and never
re-adds the node. Its graph guard condition is simply not waited on any more.

Holding one graph event per node for the life of the process keeps the count
above zero. 0 failures in 200 runs of the restart case and 100 of the whole
binary, against a first failure inside 20 runs before.

This one is not test-only either. It is the publisher's "last subscriber left,
then a new one arrived" path: the second publisher would never notice
subscribers appearing or leaving.

## Event loop thread identity

`EventLoop::create()` recorded the loop thread's id from `thread.get_id()`
after starting it, leaving a window in which the loop thread did not recognise
itself. A `post()` issued from a Live555 callback inside that window would take
the "not on the loop thread" branch, queue itself, and then block waiting for
the only thread that could drain the queue — itself. The thread now records its
own id as its first action, before `doEventLoop()`, so no callback can run
before it is set.

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

- New `rtp_over_tcp` subscriber parameter. It shipped **on**, interleaving RTP
  over the RTSP connection, which removes the loss rather than tuning around
  it. It is now **off** — see "UDP by default" below; the reasoning that
  originally justified TCP weighed artefacts above latency, and for a live
  viewer that is the wrong way round.
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

Newer rclcpp throws if `ros_time_is_active()` is queried on that system clock;
the automatic timestamp path now checks the clock type first. This matters even
without simulated time: the first decoded frame otherwise terminated a 6.4+
subscriber process after the newly enabled publisher completed its RTSP hop.

Simulated time cannot be honoured on that path at all, and used to fail
quietly: the automatic `timestamp_source` resolves to sender time on a system
clock whatever the node is configured for, so every image carried a wall clock
stamp in a simulated time base. The subscriber now checks `use_sim_time`
through the parameters interface — which `RequiredInterfaces` does provide —
and says outright that stamps will not follow `/clock` on this image_transport
version.

The publisher now supports the node-interface API too. It can configure and
serve the stream from the base, parameter, logging, timer and topic interfaces
that `RequiredInterfaces` does provide. Its former graph monitor was only an
encoder-shutdown optimization; the server's own active-stream state provides a
more accurate signal and works without a graph interface. Non-simulated
end-to-end transport tests therefore run on 6.4+ instead of skipping.

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

## ROS image topic to native RTSP clients

`image_transport republish` is lazy: its matched callback subscribes to the raw
input only while the selected output transport has a ROS subscriber. A native
ffplay, ffprobe, VLC, or GStreamer RTSP client is not a ROS graph endpoint. The
client could complete RTSP SETUP and PLAY while the republisher remained
unsubscribed forever, so the server never received a frame to encode.

The publisher plugin now preserves the republisher's matched callback and adds
one virtual match while Live555 has an active native stream. It combines that
with the real ROS subscriber count, so either kind of client keeps the input
alive and only the last departure releases it. A 10 ms wall timer performs the
check only when a caller supplied a matched callback (the republisher case),
and also provides a fallback for RMW implementations that do not deliver
publisher-matched events reliably; ordinary camera publishers create no timer.
Idle relays have no raw image subscription, no colour conversion, and no
encoder allocation.

The regression test starts the real `image_transport republish` executable,
uses a temporary ROS subscriber only to retrieve its URL, proves the raw input
was released, then connects a `StreamClient` directly. That native connection
must re-create the raw subscription, receive H.264 NAL units, and release the
subscription again on disconnect.

Two installable launch YAMLs make the path usable without reconstructing the
command line: `ros_to_rtsp_h264.launch.yaml` and
`ros_to_rtsp_h265.launch.yaml`. Their raw input is best-effort depth 1 (which is
also compatible with reliable publishers), hardware encoding is preferred,
and the output remap names `out/rtsp` exactly. Only the input and output topic
arguments normally need changing.

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

`test/` holds a GoogleTest suite run with `colcon test`. It covers codec name
mapping, `FrameData`, the decoder across every supported codec plus hardware
selection and fallback, slice loss and damage detection per codec, the encoder,
an encode/decode round trip, the Live555 event loop, the graph monitor, and a
loopback integration test that runs a real RTSP session over localhost. Cases
skip themselves when an encoder or a GPU is absent, so the same suite is
meaningful on a headless builder and on a machine with an iGPU.

Building the sources into a static `rtsp_image_transport_core` library made
this possible: the plugins stay in the loadable module, everything else can be
linked into a test binary.

Verified on ROS 2 Jazzy, Kilted, Lyrical and Rolling in the official
`ros:*-ros-base` images. Several of the bugs listed below were found by these
tests, not by reading — including the H.265 one above, which had a passing
H.264 test sitting next to it the whole time.

Coverage is measured with gcov rather than guessed at, and it is what pointed
at the last round of bugs: the two command line programs had no test at all,
and `stream_server.cpp` sat at 67% because nothing ever set `use_multicast`.
Writing the missing tests found a crash in each.

The pieces that had no coverage at all now do. The frame extractor is driven
over a scripted `FramedSource` against a `MediaSubsession` parsed from an SDP
string; the reconnect policy moved into `reconnect_policy.h` so the truth table
and the backoff can be exercised directly; and the ffmpeg-to-ROS log bridge is
driven through `av_log()` with a capturing rcutils handler, which is what pins
down the severity mapping and the behaviour around its 256 byte formatting
buffer.

Moving the backoff out also fixed it. `cooldown_ *= 2` never grows from zero,
and `reconnect_minwait` is a double parameter, so a launch file setting it to 0
turned the retry into a spin. `nextReconnectCooldown()` starts from something
non-zero and clamps before doubling rather than after, so it cannot overflow
into a negative wait either.

Running all four distributions is worth the wall clock. The multi-slice H.265
stream the damage tests need takes `x265-params`, and x265 4.x segfaults inside
`NALList::serializeSubstreams()` when `maxSlices` is above one and it has
declined to allocate a thread pool — which it does in a container that cannot
call `set_mempolicy`. x265 3.5 on Jazzy and Kilted allocates the pool and is
fine, so the crash appeared only on Lyrical and Rolling, and it took the whole
`test_stream_decoder` binary with it: 32 cases silently stopped running rather
than failing. Asking for the pool explicitly (`pools=`) avoids it everywhere.

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
- Every NAL unit of a picture now carries the same RTP presentation time. The
  publisher mapped each NAL unit onto the wall clock timeline on its own, and
  `StreamClock` takes a repeated stamp for a clock jump and steps
  `MIN_STEP_NS` (1 ms) past it. A picture is several NAL units — the parameter
  sets ahead of every key frame, and under the `slice-max-size` the publisher
  derives from `udp_packet_size` one per slice — so every NAL unit after the
  first added a millisecond that no real time had passed for.

  A receiver anchors the RTP timeline to its own clock once and then displays
  each frame at that offset, so a sender clock running fast is latency that
  accumulates for as long as the session lasts. Measured end to end at
  320x240 and 1 Mbit/s, decoded images arrived 36.5-37.5 ms apart where the
  source stamped them 33.3 ms apart: 10% adrift, about 12 s after two minutes,
  with "image time stamps jumped, re-anchoring the RTP timeline" in the log on
  every single frame. Counting NAL units per picture at the 1080p 8 Mbit/s
  settings of the launch file gives 6.2 on average and 67 on a key frame,
  i.e. 15.6% — and that is on a synthetic gradient, which compresses far
  better than camera video and so understates the slice count real content
  produces.

  Hardware encoders emit one slice per picture, so there it was only the
  parameter sets on each key frame and the drift stayed small. Guarded by a
  transport test that checks decoded images arrive a whole number of frame
  intervals apart; it fails on every one of 99 intervals before the fix.
- The subscriber's drop ladder is now a budget rather than three hardcoded
  numbers. It stood at 0.5 s / 1 s / 2 s, and since the queue settles on
  whichever rung matches how far the decoder is behind, those numbers *were*
  the steady-state latency of any stream the machine could not quite keep up
  with. New `max_latency` parameter (default 0.2 s) sets the first rung, with
  the other two at two and four times it; 0 disables dropping. Measured in
  process on a four-core container encoding 720p30 in software — a deliberate
  overload — end-to-end latency went from p50 766 ms / max 1115 ms to p50
  366 ms / max 555 ms, and stopped growing over the run (+146 ms across the
  run before, -37 ms after).
- `FrameInjector`'s queue is bounded. Live555 pulls NAL units only as fast as
  it can put them on one client's link, so a client slower than the encoder
  used to back up a queue that nothing ever trimmed: unbounded memory, and
  latency that grew for as long as it stayed connected. Past 200 ms of queued
  stream time (or 4096 NAL units, for a source whose stamps do not advance)
  the oldest *whole access units* are dropped — every NAL unit of a picture
  carries that picture's stamp, so dropping by stamp cannot hand Live555 the
  tail of a picture whose first slices are gone. Each client has its own
  injector, so a slow one degrades only itself.
- **UDP by default.** `rtp_over_tcp` now defaults to off, reversing the choice
  made in "RTP transport and lossy links" above. That section optimised for
  the picture being intact; this one optimises for it being current, and the
  two want opposite transports.

  TCP cannot drop. A lost segment stalls every byte queued behind it until the
  retransmission arrives, and a live viewer spends that wait on data it is
  already too late to display — so on a link that keeps losing packets an
  interleaved session does not degrade, it falls progressively further behind
  live and stays there. UDP loses a slice and continues, which bounds latency
  by the link rather than by its worst recent loss.

  The loss handling that was built for the UDP path is what makes this
  affordable: concealment for H.264 and the mpegvideo family, the frame
  pre-fill and damage detection for H.265/VP8/VP9/AV1 so loss reads as black
  rather than fluorescent green, and the 2 MB RTP receive buffer so the kernel
  is not the thing dropping datagrams. `rtp_over_tcp:=true` is still there for
  a link lossy enough that artefacts outweigh delay, and for servers that will
  not serve UDP.

  A third launch file, `rtsp_to_ros.launch.yaml`, is the receiving counterpart
  of the two `ros_to_rtsp_*` ones and carries these defaults: UDP, the 2 MB
  receive buffer, `low_latency` decoding, the `max_latency` budget, and
  best-effort depth-1 QoS on the decoded image topic, because a consumer
  behind on live video wants the newest frame and not a replay of the one it
  missed. Verified as a round trip — synthetic 30 Hz source, the H.264 relay,
  and this receiver — at 29.94 Hz output with 1.1 ms of jitter and no loss.

  Measured on a lossy link, which is the case the change exists for. Loss was
  injected below TCP with netfilter — `iptables -A INPUT -i lo -m statistic
  --mode random --probability P -j DROP` — so TCP genuinely retransmits and
  UDP genuinely loses datagrams, rather than either being simulated above the
  socket. `StreamServer` and `StreamClient` were driven directly, with no ROS
  graph, so every packet crossing the interface belonged to the stream.
  640x480 H.264 at 2 Mbit/s, 30 Hz, 20 s per run, latency taken as arrival
  time minus the sender's presentation stamp; two repetitions each.

  | loss | transport | p50 | p90 | p99 | max | NAL units |
  | --- | --- | --- | --- | --- | --- | --- |
  | 0% | UDP | 4.3 | 5.6 | 7.2 | 56 | 1879 |
  | 0% | TCP | 4.3 | 46 | 48 | 48 | 1879 |
  | 1% | UDP | 4.2-4.3 | 38-41 | 137 | 138-141 | 2498-2505 |
  | 1% | TCP | 4.5 | 80-112 | 214-216 | 253-256 | 2528 |
  | 5% | UDP | 5.4 | 105 | 138-139 | 140 | 2413-2418 |
  | 5% | TCP | 47-75 | 211-228 | 323-483 | 423-616 | 2528 |

  The shape is the point rather than any single number. UDP's worst case is
  flat at about 140 ms whether the link loses 1% or 5%, and it pays for that
  by not delivering 1% and 4.5% of NAL units respectively. TCP delivers every
  byte at both loss rates — 2528 of 2528, always — and the cost lands entirely
  on time: at 5% even the *median* access unit is 47-75 ms late and the worst
  runs past half a second. That is head-of-line blocking, and it gets worse
  with loss while UDP's ceiling does not move.

  These figures understate the TCP penalty. Loopback RTT is effectively zero,
  so retransmission happens as fast as the stack can manage — fast retransmit,
  or the 200 ms RTO floor visible in the 1% tail. A real tether with real RTT
  recovers more slowly than this.

## The decoder stall that made UDP look unusable

Running the same loss rig all the way through to decoded pictures, rather than
stopping at NAL arrival, turned up something the transport measurements could
not see. H.264 over UDP delivered 3-11% of frames under loss while H.265
delivered 96-99%, and the H.264 figure was not a function of the loss rate:
3.3% at 0.05% loss, 97.9% at 0.2%, 93.9% at 0.5%, 11.1% at 1%. Bimodal, not a
cliff — the stream either worked or it was dead, and higher loss only made
dying sooner more likely.

It is a latching failure. One badly timed lost packet leaves libavcodec
rejecting every slice that follows — 1842 `decode_slice_header error` out of
3808 NAL units, with `Frame num change from 7 to ...` for every value of N —
and it does not find its way back, because a slice it will not decode is also
a slice whose parameter sets it never reaches. H.265 escapes it by emitting
one slice per picture, where a lost packet costs that picture and nothing
after it.

Two things made this hard to see. The session timeout cannot detect it: RTP
keeps arriving at full rate, and only the pictures have stopped, so the
transport looks perfectly healthy. And an early measurement of it was wrong in
the other direction — the bench set `setSessionTimeout(0ms)`, disabling the one
recovery path the product does have, which made the failure look permanent
where the real subscriber sometimes reconnected out of it.

`AV_CODEC_FLAG_OUTPUT_CORRUPT` and `AV_EF_IGNORE_ERR` were tried first, on the
theory that finished pictures were being discarded. They made no difference
(6.5% against 9.3%, inside the run-to-run spread), which is what established
that the slices genuinely cannot be decoded rather than being decoded and
dropped.

What works is throwing the reference chain away. New `StreamDecoder::flush()`
calls `avcodec_flush_buffers()` and re-arms the key-frame gate;
`avcodec_flush_buffers()` rather than a new decoder because it keeps the
extradata parsed from the SDP, which is every parameter set a camera that
never repeats them in band will ever send — precisely the camera this fork
exists for. The subscriber drives it from a new `decoder_stall_timeout`
parameter (default 2 s, 0 disables): fed for that long with no picture out,
flush and resume at the next key frame. The watchdog starts after the decoder's
bounded startup key-frame wait; flushing during that wait would keep a late
join to an intra-refresh stream from ever reaching its fallback.

Measured through the real plugin stack, frames delivered over a 13 s run:

| packet loss | without watchdog | with watchdog |
| --- | --- | --- |
| 1% | 18% | 70% |
| 3% | 16% | 38% |

3% sustained uniform loss is a thoroughly broken link, and 38% of frames is
still bad video; the point is that it recovers rather than staying dead.

### What the stall actually is

The watchdog was built before the mechanism was understood, and the mechanism
turns out not to be what its name suggests. Four measurements, in the order
they settled it:

1. `nalscope` prints the NAL types delivered against the pictures produced.
   Through 22 seconds of a dead stream it shows `SPS=1 PPS=1 IDR=13-15` every
   single second — a complete key frame, parameter sets and all, arriving once
   a second while nothing comes out. So key frames are not the missing piece.
2. It then replays the captured buffers into a decoder that has never seen
   anything. Live 62 pictures, replay 62. Identical. Whatever is wrong is in
   the *bytes*, not in accumulated decoder state — which means "the decoder is
   wedged" was the wrong description all along.
3. Counting frames out of `avcodec_receive_frame` directly: libavcodec itself
   produces about as many pictures as reach the subscriber. The key-frame gate
   and the corrupt-frame check discard almost nothing, so the loss is not in
   this package's own filtering either.
4. `dropbisect` damages exactly one NAL unit offline, with no network. Neither
   a missing NAL unit nor one truncated to half its length is fatal: 0 of 41
   for each, reproducibly, over a 150 frame clip.

Which leaves the framing. `FrameExtractor` hands the decoder its buffer
whenever it exceeds 48 bytes — a size threshold, with no relation to picture
boundaries — so a multi-slice picture arrives as several separate calls and
libavcodec has to infer where each picture starts, from `first_mb_in_slice`.
Lose the slice that carries the boundary and it cannot: pictures merge, which
is exactly what `Frame num change from 7 to ...` reports for every value of N.
3176 delivered buffers, roughly 750 pictures' worth of slices, decoded into
78 pictures. H.265 is untouched by this because x265 emits one slice per
picture, so every NAL unit is its own access unit already.

### Two attempted fixes, neither shipped

Grouping NAL units into access units by RTP timestamp works, mechanically —
deliveries drop from ~3200 to 749, one per picture — and lifts H.264 over UDP
at 1% loss from 30% to 84.7% of frames. It costs a frame of latency, because a
picture can only be closed when the next one starts: p50 went 11-12 ms to
39-46 ms. Against an explicit low-latency goal, for a fix that still leaves 2%
loss at 1.2%, that is the wrong trade.

Closing the access unit on the RTP marker bit instead should avoid the delay.
It does not group at all here — deliveries stay at 3239, and the robustness
gain goes with them — so live555 is not surfacing the bit the way this
assumed.

### The marker bit was ours to get wrong

Reading live555 rather than guessing at it answered the question immediately.
`H264or5VideoStreamDiscreteFramer::nalUnitEndsAccessUnit()` returns
`isVCL(nal_unit_type)`, above this comment:

    // This will be wrong if you are streaming multiple 'slices' per picture.
    // In that case, you can define a subclass that reimplements this virtual
    // function to do the right thing.

That return value is what `H264or5VideoRTPSink` uses to decide the RTP marker
bit. Our publisher splits every picture into slices via `slice-max-size`, so
the server was setting the marker on *every slice* — telling every receiver,
RFC 6184 conformant or not, that each slice was a complete picture. The client
experiment that "proved the marker bit unusable" had been measuring our own
malformed output.

Both ends are fixed:

- `AccessUnitFramer` subclasses the discrete framer and answers
  `nalUnitEndsAccessUnit()` from the injector rather than by guessing. The
  publisher enqueues each picture's NAL units as one atomic access unit, so the
  injector knows its final NAL even when its consumer runs concurrently.
- `FrameExtractor` assembles access units, closing one on the marker bit and
  falling back to a change of presentation time for senders that get the
  marker wrong. The fallback is a frame late by construction; the marker is
  not.
- The last access unit of a stream has nothing after it to close it, so the
  source-closure handler flushes what is buffered. The frame extractor tests
  caught that omission — they script a few NAL units and nothing came out at
  all.

This belongs upstream in `rtsp_image_transport`, not in live555. Live555
already documents this multi-slice limitation and supplies the virtual hook
for a source that knows its access-unit boundaries; the transport publisher is
the layer that has that knowledge and must override the default guess.

On a clean link, reproducibly: deliveries went from 3239 to 600 for 600
frames, exactly one access unit each; delivery stays at 100%; and p50 latency
halved, 11.2 ms to **5.8 ms**, because the decoder now receives one packet per
picture instead of four to fifteen. That is the solid result, and it needs no
loss to demonstrate.

**A number reported earlier for the lossy case does not survive scrutiny.**
"1% loss: 30% -> 71% of frames delivered" was one run of each, and the
run-to-run spread turns out to be enormous. Six runs of one unchanged
configuration at 1% loss delivered 2.5%, 8.0%, 16.8%, 43.7%, 54.8% and 61.8%
of frames — a 25-fold range. The failure is bimodal: whether a run dies early
or late depends on when an unlucky packet lands, so a single 20 second run
samples a heavy tail. Any single-run comparison across that, in either
direction, is noise. Treat the lossy figures in earlier notes as
order-of-magnitude only.

What is unambiguous is that H.264 degrades badly above about 1% loss and H.265
does not (89-99% throughout), because a lost packet costs H.265 one picture
and nothing after it. The stall watchdog stays as a backstop, though it no
longer fires on this path: the failure is dropped frames rather than a stopped
decoder.

### Shortening the GOP does not help, measurably

The obvious lever for the loss case is the key frame interval, on the
reasoning that a damaged picture corrupts everything referencing it until the
next key frame. `keyframe_interval` now exposes it (0 keeps one key frame per
second, derived from `expected_framerate`), because it is a control worth
having for bandwidth and for recovery time in principle.

It does not measurably improve delivery under uniform random loss. Six runs
each at 1% loss:

| key frame interval | frames delivered per run |
| --- | --- |
| 30 (default) | 2.5% 8.0% 16.8% 43.7% 54.8% 61.8% — median ~30% |
| 5 | 2.3% 3.0% 6.3% 10.2% 14.3% 52.3% — median ~8% |

Shorter is, if anything, worse, and the distributions overlap far too much to
claim either way. The explanation that fits: a key frame is many packets and
therefore likely to be damaged, and a damaged key frame poisons the whole GOP
that follows it. Six times as many key frames is six times as many chances to
lose the picture everything else depends on. Whatever the mechanism, the
honest statement is that this lever was tried and did not pay.

### Intra refresh does, and by a lot

If the problem is that one large picture is indispensable, the answer is not
to send that picture more often but to stop having one. `intra_refresh` sweeps
a band of intra macroblocks across the frame instead, so recovery is
continuous and no single loss is unrecoverable.

Ten runs per arm at 1% loss, eight at 2%, frames delivered:

| loss | intra refresh | runs | median |
| --- | --- | --- | --- |
| 1% | off | 7.1 14.7 21.6 22.7 23.3 24.7 26.2 28.9 42.9 100.0 | 24% |
| 1% | **on** | 39.1 63.8 66.0 71.6 78.7 78.7 78.7 85.8 92.9 92.9 | **79%** |
| 2% | off | 3.1 6.4 10.4 17.8 18.4 23.3 32.2 88.4 | 18% |
| 2% | **on** | 52.7 54.7 56.7 57.3 57.3 64.2 64.4 71.6 | **57%** |

The medians are the smaller half of it. What matters more is the spread: at 2%
loss the runs without it range from 3% to 88%, and with it from 53% to 72%.
The bimodal failure — a run either survives or dies depending on where an
unlucky packet lands — is what disappears. Every single run with intra refresh
beat six of the eight without it.

Two things were needed to get this working, and both are the kind that fail
silently:

- x264 refuses intra refresh when more than one reference frame is allowed,
  and reports it only as a log line while carrying on without it. The first
  measurement showed no effect at all for exactly this reason. `refs` is now
  set to 1 alongside it, which is what a rolling refresh wants anyway: the
  guarantee only holds if nothing references back further than the sweep.
- `AV_CODEC_FLAG_CLOSED_GOP` is cleared with it, a closed GOP being a
  statement about boundaries that a rolling refresh does not have.

It is **off by default**, for one reason: it costs bitrate for the same
quality — 100 kB against 156 kB over 120 frames of the synthetic clip — and on
a bandwidth-limited link more bits mean more lost packets, which is the
problem it exists to solve. Turn it on when the link loses packets and has
headroom.

There is a second cost. With no periodic key frame, a client joining a stream
that is already running has nothing to start from and must wait out a full
sweep before its picture is complete. Encoding begins when the first client
connects, so that client still receives a key frame and is unaffected — which
is precisely why every loss measurement above missed this: they all used a
single viewer. A decoder-level regression now joins after the initial IDR and
verifies eventual image output, including the bounded no-key-frame fallback.
The still-unmeasured cases are a second simultaneous RTSP viewer and any
multicast receiver. Both are worth checking before intra refresh is relied on
for a deployment that has more than one person watching: connect a second
`StreamClient` to a running session and time how long its first complete
picture takes.

- `setBitrate()` did not touch `rc_buffer_size`, which kept twice the 1 Mbit/s
  the context was built with. The VBV was 2 Mbit whatever the stream was
  configured for — ten seconds of buffer at 200 kbit/s, and a quarter second
  rather than the intended two at 8 Mbit/s. It is now a quarter of the actual
  bitrate, which is the usual low-latency compromise and, unlike the old
  value, scales with the stream. This one is reasoned rather than measured:
  the effect it bounds is transmission time on a bandwidth-limited link, which
  an in-process loopback cannot exhibit.
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
