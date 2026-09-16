#!/usr/bin/env bash
set -euo pipefail

service_binary="${1:-build/jetson-debug/perception-service}"
config_path="${2:-config/default.yaml}"
client_seconds="${RTSP_CLIENT_SECONDS:-4}"

if [[ ! -x "${service_binary}" ]]; then
    echo "service binary is not executable: ${service_binary}" >&2
    exit 1
fi
if ! command -v gst-launch-1.0 >/dev/null 2>&1; then
    echo "gst-launch-1.0 is required for the integration test" >&2
    exit 1
fi

"${service_binary}" "${config_path}" &
service_pid=$!

cleanup() {
    if kill -0 "${service_pid}" 2>/dev/null; then
        kill -TERM "${service_pid}"
        wait "${service_pid}" || true
    fi
}
trap cleanup EXIT INT TERM

sleep 6
if ! kill -0 "${service_pid}" 2>/dev/null; then
    echo "service exited before RTSP clients connected" >&2
    exit 1
fi

check_endpoint() {
    local endpoint="$1"
    local stream_url="rtsp://127.0.0.1:8554${endpoint}"
    echo "checking ${stream_url}"
    set +e
    timeout "${client_seconds}s" gst-launch-1.0 -q \
        rtspsrc location="${stream_url}" protocols=tcp latency=0 drop-on-latency=true \
        ! rtph264depay ! h264parse ! fakesink sync=false num-buffers=1
    local status=$?
    set -e
    if [[ "${status}" -ne 0 ]]; then
        echo "RTSP endpoint failed: ${stream_url}" >&2
        return 1
    fi
}

check_endpoint /camera/rgb
check_endpoint /camera/ir_left
check_endpoint /camera/ir_right

echo "all RTSP endpoints delivered H.264 over RTP/TCP"
