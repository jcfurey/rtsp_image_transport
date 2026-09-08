# Bidirectional transport audit — 2026-09-08

Audited the ROS image → RTSP publisher and RTSP → ROS image subscriber,
including UDP RTP, interleaved TCP RTP, session recovery, malformed input,
multiple viewers, and shutdown with an executor still running. The starting
revision was `58eeb81`. The changes are local to this package.

## Findings and fixes

| Area | Failure found | Resulting behavior |
| --- | --- | --- |
| Plugin lifetime | Removing a ROS callback did not revoke work already taken by the executor. Destruction could leave callbacks referring to the plugin. Resetting a client pointer also did not wait for callbacks holding another reference. | Both plugins shut down explicitly. Queued ROS callbacks retain a shared lifetime gate. Subscriber shutdown waits for network callbacks before releasing decoder and delivery state. |
| Server lifetime | During server destruction, a subsession's weak owner reference could expire before its source cleanup ran. The source remained in the server map after Live555 freed it. AddressSanitizer reproduced a heap use-after-free. | Subsessions borrow the server until its synchronous RTSP teardown completes, so every source removal reaches the owning map. Construction and destruction run on the Live555 loop. Failed construction also stops its loop. |
| Subscriber concurrency | Network callbacks changed session and decoder state concurrently with executor callbacks and parameter updates. A cancelled retry could reconnect a replacement session. | Network callbacks enqueue session events and frames. The executor serializes session changes, decoding, and ROS callbacks. Retry timers carry a connection generation. |
| Recovery | The default only retried timeouts. A camera absent at startup, a failed decoder selection, or an ended session could leave a live feed permanently stopped. Decoder reconfiguration lost configuration supplied only in SDP. | The default is now `reconnect_policy=3`; it retries failures and normal endings as well as media timeouts. Reconfiguration retrieves fresh SDP. Successful decoder setup clears the failed state. URL changes discard old decoder state and queued pictures. |
| Image input | Pixel conversion trusted dimensions, stride, and buffer size. NV21/NV24 passed no chroma plane; NV12 was not accepted. A malformed image or changed resolution could disable publishing. | Input is validated before FFmpeg reads pixels. Padded NV12/NV21/NV24 planes are supplied correctly. Malformed images are dropped, valid resolution changes restart encoding, and other encoding failures retry after one second. Runtime hardware failures allow a software retry. |
| Multiple viewers | Live555 reused the first source despite the documented independent queues. The SDP path could retain a sink belonging to a departing viewer. | Each unicast viewer gets its own source and queue. SDP uses a live sink, and disconnecting a viewer removes its sink mapping. Tests verify continuing delivery and a subsequent third viewer. |
| Frame boundaries and memory | A full receive buffer could be emitted as a complete picture. After truncation, the remaining slices could become a second incomplete picture. Frozen timestamps defeated a time-only byte bound on send queues. | Receive buffers grow to 16 MiB, discard a truncated picture through its boundary, and reseed SDP configuration. Send queues have byte and NAL budgets and drop whole pending pictures. |
| Codec initialization | MPEG-4 cameras supplying VOL configuration only in SDP had no decoder bootstrap data. Empty decoder input could be treated as end-of-stream. | MPEG-4 SDP configuration precedes the first picture. Empty, null, and excessively large decoder packets are rejected. FFmpeg conversion and allocation failures are checked. |
| Event loop shutdown | Marking the loop stopped before draining queued tasks allowed concurrent foreign-thread cleanup during the final drain. | The loop continues to accept queued work until the final drain finishes. |

## Operational defaults and bounds

- `reconnect_policy=3`, handshake/media `timeout=2.0` seconds, exponential
  retry delay from 0.1 to 30 seconds. Set policy `0` for finite recordings that
  should end once. The receive launch file exposes policy and timeout.
- The legacy parameter `udp_port` is the **RTSP TCP listening port**. Its name
  remains compatible; its description now identifies the correct protocol.
- Each publisher queue has a 200 ms timestamp-span budget, a 4096-NAL budget,
  and a 64 MiB payload budget. One access unit may contain at most 4096 NALs
  and 16 MiB. Trimming retains the picture already being transmitted and the
  newest picture; those two can exceed the time or NAL budget together.
  Budgets exclude FFmpeg, Live555 packet buffers, and container overhead.
- The subscriber's existing defaults bound pending input to 120 access units
  or 64 MiB while latency dropping is enabled. `max_latency=0` explicitly
  disables these receive-queue bounds.
- Per-viewer queues share the encoder and Live555 socket dispatcher. They
  do not establish complete isolation from a blocked TCP socket or unlimited
  viewer capacity.

## Validation

Built in separate temporary build/install directories against ROS Lyrical,
image_transport 6.4.10, FFmpeg libavcodec 62.11.100, and the installed Live555
with H.265 support. The workspace's existing build/install was not replaced.

The original revision passed all 21 CTest executables. The final expanded
suite passed **22/22 CTest executables: 218 cases passed, four skipped, zero
failures**. It adds plugin recovery and lifetime regressions; existing tests also cover
codec round trips, timestamps, lazy native RTSP demand, the command-line
programs, and multicast setup/restart/teardown. Socket tests ran with actual
local network access in dedicated ROS domains.

AddressSanitizer, UndefinedBehaviorSanitizer, and leak detection cover the
encoder, extractor, injector, event loop, RTSP loopback, and plugin robustness
tests. All six sanitizer targets passed without reported sanitizer errors.
The final 12-case plugin lifecycle/recovery target also passed three consecutive
runs under all three checks, including destruction followed by a callback
exception. The earlier sanitizer run reproduced the server use-after-free
before its fix.

The plugin regressions exercise both UDP and TCP:

- invalid images followed by valid images and resolution changes;
- initial RTSP 404 followed by the server becoming available;
- repeated decoder reconfiguration, including an invalid decoder corrected
  while the executor remains active;
- destruction without explicit shutdown, with subsequent publishing and
  parameter updates;
- a user image callback releasing its own subscriber, including a callback
  that then throws;
- replacing an H.264 URL with an MPEG-4 URL and different image dimensions.

Additional tests cover a silent DESCRIBE peer, independent unicast sources,
large queues with frozen timestamps, oversized access units, padded YUV
chroma, truncated picture tails, and MPEG-4 SDP configuration.

Independent FFmpeg clients (`ffprobe -count_frames`) decoded the server's
H.264 and H.265 over both UDP and TCP at 640×480. In those two 12-second runs,
the package's continuously connected receiver decoded 360/360 pictures per
codec without decode errors. The H.265 TCP late join reported missing
references for the current GOP, then recovered at the next keyframe and
decoded 68 pictures within its three-second observation interval. A native
viewer can therefore still need the next keyframe at startup; this audit
does not promise immediate output for a client joining between keyframes.

The updated receive launch file also parsed successfully with
`ros2 launch rtsp_image_transport rtsp_to_ros.launch.yaml --show-args`.

## Throughput observations

The full bridge benchmark sends four independent 640×480 inputs at 30 Hz,
with alternating software H.264/H.265 encoders at 8 Mbit/s. It launches four
ROS → RTSP republishers and four RTSP → raw ROS republishers, checks every
route's pixel marker and URL, and pauses input 0 to check continued delivery
on the other routes. Both raw ROS legs use best-effort, depth-one QoS.

The original revision on default Fast DDS delivered 15.47/15.93 fps on the
two H.264 routes and 12.07/13.93 fps on H.265 in a 15-second measurement.
Updated-package Fast DDS measurements also fell below 30 fps and varied
considerably. A 4 MiB Fast DDS socket-buffer profile alone did not restore
30 fps. Switching the updated package to Cyclone DDS delivered
29.93/29.67 fps on H.264 and 26.20/26.33 fps on H.265 in an initial trial.
All of those trials retained distinct outputs and continued the other routes
when input 0 paused.

A final 30-second run with Cyclone DDS and a 4 MiB receive-buffer request,
with builds and sanitizer tests finished, measured:

| Route | Codec | Delivered / sent | fps | Median latency | p95 latency |
| --- | --- | ---: | ---: | ---: | ---: |
| 0 | H.264 | 900 / 900 | 30.00 | 13.15 ms | 15.43 ms |
| 1 | H.265 | 839 / 900 | 27.97 | 39.69 ms | 57.61 ms |
| 2 | H.264 | 900 / 900 | 30.00 | 13.33 ms | 15.56 ms |
| 3 | H.265 | 838 / 900 | 27.93 | 39.38 ms | 51.87 ms |

All routes had zero incorrect pixel markers and unique URLs. While input 0
paused, routes 1–3 continued at 28.33, 30.00, and 28.00 fps. Combined CPU use
was 6.269 cores for encoding relays, 0.825 for decoding relays, and 0.163 for
the harness. These are host-specific software measurements at 640×480.

This comparison points to raw ROS middleware delivery as an important limit
on this host; it does not isolate every source of frame loss or establish a
codec throughput guarantee. Best-effort delivery of fragmented large samples
can lose a whole image when a fragment is lost. See the
[ROS DDS tuning guide](https://docs.ros.org/en/rolling/How-To-Guides/DDS-tuning.html)
and [Fast DDS large-data guidance](https://fast-dds.docs.eprosima.com/en/latest/fastdds/use_cases/large_data/large_data.html).

Reproduce the complete route after building with `BUILD_BENCHMARKS=ON` and
sourcing the resulting install:

```bash
CYCLONEDDS_URI='<CycloneDDS><Domain><Internal><SocketReceiveBufferSize min="4MiB"/></Internal></Domain></CycloneDDS>' \
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=161 \
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_IP=127.0.0.1 \
  build/rtsp_image_transport/multi_stream_bench 4 640 480 30 mixed software ros
```

The buffer request requires a sufficient host socket-buffer limit. This audit
used per-process settings and did not change kernel or global ROS settings.
Further benchmark modes and packet-loss experiments are described in
[benchmark/README.md](benchmark/README.md).

## Remaining validation limits

- Physical cameras, the vehicle network, sustained packet loss, and long
  deployments were not exercised in this audit. Local tests cannot certify
  behavior across those conditions.
- Actual GPU encode/decode device paths were not validated; software and
  device-selection/fallback logic were tested. The Intel device test skips
  when no applicable device is exposed.
- The image_transport node-interface API does not provide the node clock.
  Simulated receive timestamps on that API remain an existing limitation;
  two tests explicitly skip. The H.263 decoding case also skips on this
  FFmpeg configuration.
- Multicast resource lifetime is covered; multiple real multicast receivers
  and second-viewer acquisition with `intra_refresh=true` remain unverified.
- The installed Live555 build has no AV1 server support. Codec availability
  remains dependent on the installed FFmpeg/Live555 build.
- Subprocess republishers emit a class_loader shutdown warning about live
  plugin objects on both the original and updated revision. Their tested
  runs exit successfully; this audit does not claim to fix that upstream
  unloading behavior.
