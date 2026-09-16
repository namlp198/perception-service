#!/usr/bin/env bash
set -euo pipefail

# The deployed target is a Jetson Orin Nano, which has no hardware video
# encoder. Validate the configured low-latency x264 path instead of requiring
# the decoder-only NVIDIA V4L2 plugin to expose nvv4l2h264enc.
unset LD_LIBRARY_PATH
unset GST_PLUGIN_PATH
unset GST_PLUGIN_SYSTEM_PATH
unset GST_PLUGIN_SYSTEM_PATH_1_0

inspect_output="$(gst-inspect-1.0 x264enc 2>&1)" || {
    echo "Required Orin Nano software encoder is unavailable: x264enc" >&2
    printf '%s\n' "${inspect_output}" >&2
    exit 1
}

if ! encode_output="$(timeout 20s gst-launch-1.0 -q \
    videotestsrc num-buffers=5 \
    ! video/x-raw,width=640,height=480,framerate=30/1 \
    ! videoconvert \
    ! video/x-raw,format=I420 \
    ! x264enc tune=zerolatency speed-preset=ultrafast bitrate=4000 \
        key-int-max=30 bframes=0 sliced-threads=true \
    ! h264parse \
    ! fakesink sync=false 2>&1)"; then
    echo "The production-compatible Orin Nano x264 encode probe failed." >&2
    printf '%s\n' "${encode_output}" >&2
    exit 1
fi

echo "RTSP encoder preflight passed: Orin Nano x264 low-latency path."
