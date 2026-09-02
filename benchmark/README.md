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
