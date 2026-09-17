#!/usr/bin/env bash
set -euo pipefail

# Operator-only D435i IMU calibration wrapper around Intel's rs-imu-calibration.py. The camera has
# no factory IMU calibration (service logs "IMU Calibration is not available"); the tool records
# six static orientations and writes intrinsics to the camera EEPROM. It is interactive: the
# operator physically positions the camera as prompted. The service is stopped for the duration
# and restored afterwards, then the full verification runs.
if [[ "${1:-}" != "--confirm-service-interruption" ]]; then
    echo "Usage: $0 --confirm-service-interruption [librealsense_dir] [extra rs-imu-calibration args]" >&2
    echo "Example: $0 --confirm-service-interruption ~/librealsense -g" >&2
    exit 2
fi
shift
librealsense_dir="${1:-${HOME}/librealsense}"
if (($# > 0)); then
    shift
fi
tool="${librealsense_dir}/tools/rs-imu-calibration/rs-imu-calibration.py"
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
service_name="perception-service-user.service"

if [[ ! -f "${tool}" ]]; then
    echo "rs-imu-calibration.py not found at ${tool}" >&2
    exit 1
fi
if ! python3 -c 'import pyrealsense2, numpy' 2>/dev/null; then
    echo "python3 needs pyrealsense2 and numpy (librealsense built with -DBUILD_PYTHON_BINDINGS=ON," \
        "or 'pip3 install pyrealsense2 numpy')." >&2
    exit 1
fi

echo "Stopping ${service_name}; RTSP will be unavailable during calibration..."
systemctl --user stop "${service_name}"
restore_service() {
    echo "Restoring ${service_name}..." >&2
    systemctl --user start "${service_name}" || true
}
trap restore_service EXIT
sleep 2

echo "Follow the on-screen prompts: six orientations, hold still until 20 dots appear."
echo "Answer Y when asked to save the calibration to the camera. ESC aborts (not Ctrl-C)."
python3 "${tool}" "$@"

trap - EXIT
restore_service
sleep 5
bash "${project_root}/scripts/jetson/verify-running-service.sh" full
echo "Confirm the journal no longer reports 'IMU Calibration is not available':"
journalctl --user-unit="${service_name}" -n 40 --no-pager | grep -c 'IMU Calibration is not available' || true
