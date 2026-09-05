# Benchmarks and diagnostics

Measurement programs for the transport and latency work. They are not tests:
they take minutes, several need a lossy link, and their output is numbers to
read rather than assertions to pass. Nothing here runs in CI.

Build them with:

    colcon build --packages-select rtsp_image_transport \
        --cmake-args -DBUILD_BENCHMARKS=ON

The binaries land in `build/rtsp_image_transport/`.

## Injecting packet loss

`tc netem` is the usual tool and is unavailable on some kernels — a container
kernel built without `CONFIG_NET_SCH_NETEM` cannot load it at all. Netfilter
works instead, and for comparing transports it is arguably the better
instrument: it drops *below* TCP, so TCP genuinely retransmits and UDP
genuinely loses datagrams.

    iptables -A INPUT -i lo -m statistic --mode random --probability 0.02 -j DROP

This needs `CONFIG_NETFILTER_XT_MATCH_STATISTIC` (usually built in), the
`iptables` userspace extensions (`libxt_statistic.so`, in the `iptables`
package), and `NET_ADMIN`. Run it in a network namespace of its own so the
rule cannot touch anything else on the machine:

    docker run --rm --cap-add=NET_ADMIN --cap-add=NET_RAW <image> bash -c \
        'iptables -A INPUT -i lo -m statistic --mode random --probability 0.02 -j DROP
         /path/to/soak udp h264 25 "loss=2%"'

Note what this cannot do: netfilter drops, but it cannot delay, jitter or rate
limit. On loopback the round trip is effectively zero, so TCP recovers as fast
as it possibly can — fast retransmit, or the 200 ms RTO floor. Any TCP penalty
measured this way is a **lower bound** on what a real link would show.

To restrict the loss to video and leave RTSP control and ROS discovery alone,
match on size:

    iptables -A INPUT -i lo -p udp -m length --length 400:1600 \
        -m statistic --mode random --probability 0.02 -j DROP

## The programs

### `latency_bench WIDTH HEIGHT FPS FRAMES LABEL`

End-to-end latency through the **real plugins**: an `image_transport`
publisher on the rtsp transport and an rtsp subscriber, in one process.
Reports p50/p90/p99/max and whether latency grew over the run. This is the one
that exercises `max_latency`, the reconnect policy and the decoder stall
watchdog, because it goes through `SubscriberPlugin` rather than driving the
client directly.

### `multi_stream_bench STREAMS WIDTH HEIGHT SECONDS {h264|h265|mixed} [hardware|software] [direct|ros]`

Runs one to four independent raw ROS image topics through separate
`image_transport republish` processes, each serving its own RTSP output.
Subscribers decode every output through the real RTSP plugin. `h264` and
`h265` use the selected codec on every route; `mixed`
alternates H.264 and H.265; the default uses hardware encoding and CUDA
decoding when available. `software` disables both. The default `direct` output
decodes in the harness. `ros` starts a second set of independent republish
processes (`in_transport:=rtsp`, `out_transport:=raw`), then verifies each raw
ROS output topic. This covers both bridge directions, including publication
and delivery of decoded images over ROS.

Both raw ROS legs explicitly use best-effort, keep-last, depth-one QoS on
publishers and subscribers. The advertised RTSP URL topics remain reliable
and transient-local. At 1080p, each decoded BGR image is about 6 MiB, so DDS
delivery and receive-buffer limits are part of the `ros` measurement.

For example, after building with `BUILD_BENCHMARKS=ON`:

    ROS_DOMAIN_ID=231 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_IP=127.0.0.1 \
        build/rtsp_image_transport/multi_stream_bench 4 1920 1080 10 h264

    ROS_DOMAIN_ID=231 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_IP=127.0.0.1 \
        build/rtsp_image_transport/multi_stream_bench 4 1920 1080 30 mixed hardware ros

The benchmark sends each input at 30 Hz, warms up for five seconds, measures
for `SECONDS`, then pauses input 0 for three seconds. It reports each output's
frame rate, latency percentiles, pixel identity, and frame rate while input 0
is paused. It reports CPU use separately for ROS-to-RTSP relays, RTSP-to-ROS
relays, and the harness, in CPU cores (1.0 means one fully occupied core).
In `direct` mode the harness includes decoding; in `ros` mode it includes
receipt of raw ROS images. Latencies cover the complete route. The source
precomputes eight frames per input; reported harness memory includes them.

A distinct pixel marker and RTSP URL identify each route. Missing streams,
crossed outputs, or loss of another stream during the pause return a nonzero
exit status. Frame rate and latency are measurements without pass/fail limits.
Each invocation uses unique topic names, ephemeral RTSP ports, and stops its
own relay processes when it finishes. Run scaling trials sequentially in an
unused ROS domain so they do not compete with each other or a running vehicle.

The audit also tested Cyclone DDS with a per-process 4 MiB receive-buffer
request. To reproduce that configuration on a host whose socket-buffer limit
allows it:

    CYCLONEDDS_URI='<CycloneDDS><Domain><Internal><SocketReceiveBufferSize min="4MiB"/></Internal></Domain></CycloneDDS>' \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=231 \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_IP=127.0.0.1 \
        build/rtsp_image_transport/multi_stream_bench 4 1920 1080 30 h265 hardware ros

Four raw 1080p outputs still showed UDP receive-buffer drops with this setting
on the audit host. Keep raw ROS delivery measurements separate from direct
RTSP output measurements when assessing capacity.

### `soak {udp|tcp} {h264|h265} SECONDS LABEL`

`StreamServer` and `StreamClient` directly, with no ROS graph, decoding on a
worker thread the way the subscriber does. Reports how many pictures came out
of how many sent, plus latency and drift. This is the program that shows what
a transport does to *pictures* rather than to packets.

`RTSP_BENCH_DECODER_WATCHDOG=1` recreates the decoder when it goes quiet,
which is how the `decoder_stall_timeout` idea was validated before being
built into the subscriber. `RTSP_BENCH_NO_SLICE=1` skips the
`setPackageSizeHint()` call.

### `loss_bench {udp|tcp} SECONDS WIDTH HEIGHT BITRATE LABEL`

The transport comparison at the packet level: latency from the sender's
presentation stamp to NAL arrival, without the decoder in the way. Used for
the UDP-versus-TCP table in the fork notes.

### `nalscope SECONDS`

Per-second timeline of which H.264 NAL types the extractor delivered against
how many pictures came out, then replays the captured buffers into a fresh
decoder. The replay is the important part: if the fresh decoder produces the
same count as the live one, the bytes were undecodable and no amount of
resetting decoder state will help.

### `dropbisect FRAMES`

No network at all. Encodes a clip with the publisher's settings, feeds the NAL
units to the decoder the way the extractor does, and damages exactly one of
them — dropped or truncated — reporting whether the stream ever recovers.
Deterministic, so it is the right place to start on any "loss kills the
stream" question before reaching for the network.

Give it enough frames. It judges recovery by the final quarter of the stream,
so a short run puts the damage close to the window and reports failures that
are only the metric running out of room: 60 frames shows 5 of 38 truncations
as fatal, 150 frames reproducibly shows none. Use 150 or more, and treat a
result near the length threshold as an artifact until a longer run agrees.

### `measure_nals`

How many NAL units a picture becomes at various resolutions and bitrates.
Small, but it is what turns "slice-max-size splits the picture" into a number.

## What these established

See `NAUTILUS_FORK.md` for the findings and their numbers. The short version:

- The RTP presentation timeline used to run fast, which is measurable end to
  end as decoded images arriving further apart than they were stamped.
- UDP bounds latency under loss where interleaved TCP does not, though TCP
  never drops a byte.
- H.264 over lossy UDP can stop producing pictures for tens of seconds while
  complete key frames keep arriving; `dropbisect` and `nalscope` between them
  place the fault in access-unit boundaries rather than in decoder state.
