#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# The probe must use JetPack's system GStreamer stack. The project's staged
# runtime is for the application binary and can contain a second GStreamer ABI.
unset LD_LIBRARY_PATH
unset GST_PLUGIN_PATH
unset GST_PLUGIN_SYSTEM_PATH
unset GST_PLUGIN_SYSTEM_PATH_1_0

registry_path="${project_root}/.runtime/gstreamer-registry.bin"
mkdir -p -- "${project_root}/.runtime"
rm -f -- "${registry_path}"
export GST_REGISTRY_1_0="${registry_path}"

diagnose_plugin() {
    echo "NVIDIA GStreamer plugin diagnostics:" >&2
    find /usr/lib/aarch64-linux-gnu -maxdepth 4 -type f \
        \( -name 'libgstnvvideo4linux2.so' -o -name 'libgstnvvidconv.so' \) -print 2>/dev/null |
        while IFS= read -r plugin; do
            echo "--- ${plugin}" >&2
            ldd "${plugin}" >&2 || true
        done
    dpkg-query -W -f='${binary:Package} ${Version}\n' 'nvidia-l4t-gstreamer' \
        'nvidia-l4t-multimedia' 2>/dev/null >&2 || true
}

for element in nvvidconv nvv4l2h264enc; do
    inspect_output="$(gst-inspect-1.0 "${element}" 2>&1)" || {
        echo "Required Jetson GStreamer element is unavailable: ${element}" >&2
        printf '%s\n' "${inspect_output}" >&2
        diagnose_plugin
        exit 1
    }
done

if ! encode_output="$(timeout 20s gst-launch-1.0 -q \
    videotestsrc num-buffers=5 \
    ! video/x-raw,width=640,height=480,framerate=30/1 \
    ! videoconvert \
    ! video/x-raw,format=I420 \
    ! nvvidconv \
    ! 'video/x-raw(memory:NVMM),format=NV12' \
    ! nvv4l2h264enc maxperf-enable=true control-rate=1 bitrate=4000000 \
        iframeinterval=30 idrinterval=30 insert-sps-pps=true \
    ! h264parse \
    ! fakesink sync=false 2>&1)"; then
    echo "nvv4l2h264enc exists but the production-compatible encode probe failed." >&2
    printf '%s\n' "${encode_output}" >&2
    diagnose_plugin
    exit 1
fi

echo "NVIDIA H.264 encoder preflight passed: nvvidconv -> NVMM -> nvv4l2h264enc."
