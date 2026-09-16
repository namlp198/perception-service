#!/usr/bin/env bash
set -euo pipefail

# Usage: test_rtsp.sh [host] [path] [attempts]
# Succeeds only when the client receives real H.264 media. A shared gst-rtsp-server media is
# torn down after its last client leaves and prepared again for the next one, so a probe that
# reconnects right after a previous probe can hit that window; bounded retries with a short
# settle delay separate such a transient from a stream that is really down.
jetson_host="${1:-127.0.0.1}"
stream_path="${2:-/camera/rgb}"
attempts="${3:-${PERCEPTION_RTSP_PROBE_ATTEMPTS:-3}}"
retry_delay_s="${PERCEPTION_RTSP_PROBE_RETRY_DELAY_S:-2}"
stream_url="rtsp://${jetson_host}:8554${stream_path}"
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
local_library_path="${project_root}/.runtime/sysroot/usr/lib/aarch64-linux-gnu"

if [[ ! "${attempts}" =~ ^[1-9][0-9]*$ ]]; then
    echo "Probe attempts must be a positive integer: ${attempts}" >&2
    exit 2
fi

if [[ -d "${local_library_path}" ]]; then
    export LD_LIBRARY_PATH="/usr/lib/aarch64-linux-gnu:${local_library_path}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi
unset GST_PLUGIN_PATH

probe_once() {
    if command -v ffprobe >/dev/null 2>&1; then
        timeout 8s ffprobe -v error -rw_timeout 5000000 -rtsp_transport tcp -show_streams \
            "${stream_url}"
        return $?
    fi
    if command -v gst-launch-1.0 >/dev/null 2>&1; then
        timeout 5s gst-launch-1.0 -q rtspsrc location="${stream_url}" protocols=tcp latency=0 \
            drop-on-latency=true ! rtph264depay ! h264parse ! fakesink sync=false num-buffers=1
        return $?
    fi
    echo "Neither ffprobe nor gst-launch-1.0 is available." >&2
    return 127
}

status=1
for ((attempt = 1; attempt <= attempts; attempt++)); do
    set +e
    probe_once
    status=$?
    set -e
    if [[ "${status}" -eq 0 ]]; then
        if ((attempt > 1)); then
            echo "RTSP media received from ${stream_url} on attempt ${attempt}/${attempts}."
        fi
        exit 0
    fi
    if [[ "${status}" -eq 127 ]]; then
        exit 1
    fi
    if ((attempt < attempts)); then
        echo "No RTSP media from ${stream_url} (attempt ${attempt}/${attempts}, status ${status});" \
            "retrying in ${retry_delay_s}s..." >&2
        sleep "${retry_delay_s}"
    fi
done
echo "No RTSP media from ${stream_url} after ${attempts} attempts (last status ${status})." >&2
exit "${status}"
