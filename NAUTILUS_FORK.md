# Nautilus fork notes

Remote: [`jcfurey/rtsp_image_transport`](https://github.com/jcfurey/rtsp_image_transport),
branch `ros2`.

This change is based on upstream commit
`d815a176b0634cda8d5f4290f33173b82fab7a6c`. Nautilus adds a zero-based
`video_subsession` subscriber parameter. Upstream 2.0.2 always selects the
first supported video subsession, but the Deep Trekker 4K camera advertises
HEVC first and two H.264 streams after it. The parameter made subsession 1 a
working 1024x600 H.264 fallback during initial integration.

The fork now also initializes H.265 from its RFC 7798 SDP attributes. Upstream
looked for H.264's combined `sprop-parameter-sets` value for both codecs; this
camera instead supplies H.265 parameter sets separately as `sprop-vps`,
`sprop-sps`, and `sprop-pps`. Camera0 targets subsession 0, the camera's
highest-quality advertised LAN track (1920x1080 HEVC at 30 Hz).

The fork also replaces the embedded placeholder PNG with a 1x1 BGR ROS image
and removes the otherwise unnecessary `cv_bridge`/OpenCV dependency. This keeps
the transport independent of the CUDA-enabled OpenCV build in the Nautilus
runtime image.

Keep upstream's Apache-2.0 license and copyright notices when rebasing. The
intended upstreamable changes are configurable video subsession selection and
codec-correct H.265 SDP parameter-set handling.
