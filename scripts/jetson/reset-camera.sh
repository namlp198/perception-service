#!/usr/bin/env bash
set -euo pipefail

# Operator-only recovery for a D435i whose Motion Module stays silent (service logs show
# `motion_callbacks=0` on every startup_timeout) until the camera re-enumerates over USB.
# It stops the service, issues a librealsense hardware reset (equivalent to a replug), waits for
# re-enumeration, restarts the service and runs the strict verification. RGB/depth are
# interrupted for the duration; never automate this from inside the service.
if [[ "${1:-}" != "--confirm-service-interruption" ]]; then
    echo "Usage: $0 --confirm-service-interruption [preset]" >&2
    exit 2
fi

preset="${2:-jetson-local}"
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
service_name="perception-service-user.service"
binary="${project_root}/build/${preset}/camera-info"
local_library_path="${project_root}/.runtime/sysroot/usr/lib/aarch64-linux-gnu"

if [[ -d "${local_library_path}" ]]; then
    export LD_LIBRARY_PATH="/usr/lib/aarch64-linux-gnu:${local_library_path}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi
if [[ ! -x "${binary}" ]]; then
    echo "Missing executable: ${binary}" >&2
    exit 1
fi

echo "Stopping ${service_name}; RTSP will be temporarily unavailable..."
systemctl --user stop "${service_name}"
sleep 2
echo "Issuing D435i hardware reset..."
"${binary}" "${project_root}/config/default.yaml" --hardware-reset | tail -n 1
echo "Waiting for USB re-enumeration..."
sleep 8
echo "Starting ${service_name}..."
systemctl --user start "${service_name}"
sleep 5
bash "${project_root}/scripts/jetson/verify-running-service.sh" full
