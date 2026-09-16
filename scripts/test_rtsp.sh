#!/usr/bin/env bash
set -euo pipefail

jetson_host="${1:-127.0.0.1}"
stream_path="${2:-/camera/rgb}"
stream_url="rtsp://${jetson_host}:8554${stream_path}"
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
local_library_path="${project_root}/.runtime/sysroot/usr/lib/aarch64-linux-gnu"

if [[ -d "${local_library_path}" ]]; then
    export LD_LIBRARY_PATH="${local_library_path}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
    export GST_PLUGIN_PATH="${local_library_path}/gstreamer-1.0${GST_PLUGIN_PATH:+:${GST_PLUGIN_PATH}}"
fi

if command -v ffprobe >/dev/null 2>&1; then
    exec timeout 8s ffprobe -v error -rw_timeout 5000000 -rtsp_transport tcp -show_streams \
        "${stream_url}"
fi
if command -v gst-launch-1.0 >/dev/null 2>&1; then
    set +e
    timeout 5s gst-launch-1.0 -q rtspsrc location="${stream_url}" protocols=tcp latency=0 \
        drop-on-latency=true ! rtph264depay ! h264parse ! fakesink sync=false num-buffers=1
    status=$?
    set -e
    if [[ "${status}" -eq 0 ]]; then
        exit 0
    fi
    exit "${status}"
fi

echo "Neither ffprobe nor gst-launch-1.0 is available." >&2
exit 1
