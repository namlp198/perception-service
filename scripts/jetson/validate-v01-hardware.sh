#!/usr/bin/env bash
set -euo pipefail

if [[ "${1:-}" != "--confirm-service-interruption" ]]; then
    echo "Usage: $0 --confirm-service-interruption [preset]" >&2
    echo "This test temporarily stops perception-service-user.service and RTSP." >&2
    exit 2
fi

preset="${2:-jetson-local}"
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
service_name="perception-service-user.service"
config_path="${project_root}/config/default.yaml"
binary_dir="${project_root}/build/${preset}"
local_library_path="${project_root}/.runtime/sysroot/usr/lib/aarch64-linux-gnu"
service_was_active=false
service_restarted=false

if [[ -d "${local_library_path}" ]]; then
    export LD_LIBRARY_PATH="${local_library_path}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
    export GST_PLUGIN_PATH="${local_library_path}/gstreamer-1.0${GST_PLUGIN_PATH:+:${GST_PLUGIN_PATH}}"
fi

systemctl --user daemon-reload

if systemctl --user is-active --quiet "${service_name}"; then
    service_was_active=true
fi

restore_service() {
    if [[ "${service_was_active}" == true && "${service_restarted}" == false ]]; then
        echo "Restoring ${service_name} after validation..." >&2
        systemctl --user daemon-reload || true
        systemctl --user start "${service_name}" || true
        sleep 3
        systemctl --user --no-pager --full status "${service_name}" || true
        journalctl --user --unit "${service_name}" --lines 30 --no-pager || true
    fi
}
trap restore_service EXIT

for binary in camera-info imu-info perception-service; do
    if [[ ! -x "${binary_dir}/${binary}" ]]; then
        echo "Missing executable: ${binary_dir}/${binary}" >&2
        exit 1
    fi
done

echo "Stopping ${service_name}; RTSP will be temporarily unavailable..."
systemctl --user stop "${service_name}"

echo "Collecting D435i model, firmware, USB mode, profiles and intrinsics..."
"${binary_dir}/camera-info" "${config_path}"

echo "Sampling accelerometer and gyroscope for 10 seconds..."
"${binary_dir}/imu-info" "${config_path}" 10

echo "Starting ${service_name}..."
systemctl --user daemon-reload
systemctl --user start "${service_name}"
service_restarted=true
sleep 5
systemctl --user --no-pager --full status "${service_name}"

echo "Checking RGB and depth-visual RTSP endpoints..."
"${project_root}/scripts/test_rtsp.sh" 127.0.0.1 /camera/rgb
"${project_root}/scripts/test_rtsp.sh" 127.0.0.1 /camera/depth_visual

echo "V0.1 hardware acceptance completed successfully."
