#!/usr/bin/env bash
set -euo pipefail

# Exit codes:
#   0  requested verification passed
#   1  service not running, encoder preflight failed, or RGB/depth RTSP media missing
#   3  (full mode only) RGB/depth are live but the mandatory IMU acceptance is unmet
service_name="perception-service-user.service"
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
mode="${1:-full}"

if [[ "${mode}" != "full" && "${mode}" != "streaming" ]]; then
    echo "Usage: $0 [full|streaming]" >&2
    exit 2
fi

if ! systemctl --user is-active --quiet "${service_name}"; then
    systemctl --user --no-pager --full status "${service_name}" || true
    exit 1
fi

main_pid="$(systemctl --user show "${service_name}" --property MainPID --value)"
if [[ ! "${main_pid}" =~ ^[1-9][0-9]*$ ]]; then
    echo "${service_name} has no valid MainPID: ${main_pid}" >&2
    exit 1
fi

echo "Waiting for fresh runtime evidence from PID ${main_pid} (${mode})..."
sleep 7
service_log="$(journalctl --user-unit="${service_name}" _PID="${main_pid}" --no-pager -n 60 \
    2>/dev/null || true)"
if [[ -z "${service_log}" || "${service_log}" == *"No journal files were found"* ]]; then
    service_log="$(systemctl --user --no-pager --full --lines=60 status "${service_name}" \
        2>&1 || true)"
fi
printf '%s\n' "${service_log}"

bash "${project_root}/scripts/jetson/validate-rtsp-encoder.sh"

# RGB/depth are verified first and on their own: they must be live regardless of IMU state.
for stream_path in /camera/rgb /camera/depth_visual; do
    echo "Checking RTSP media on ${stream_path}..."
    if ! bash "${project_root}/scripts/test_rtsp.sh" 127.0.0.1 "${stream_path}"; then
        echo "STREAMING FAILED: no media from rtsp://127.0.0.1:8554${stream_path}" >&2
        echo "Inspect camera.frames_received / stream.frames_published in the metrics above." >&2
        exit 1
    fi
done
echo "Streaming verification passed: RGB and depth_visual H.264 media are live."

metrics_line="$(grep 'camera\.frames_received=' <<<"${service_log}" | tail -n 1)"
imu_ok=false
if [[ -n "${metrics_line}" ]] &&
    grep -Eq 'imu\.ready=true ekf\.ready=true' <<<"${metrics_line}" &&
    grep -Eq 'imu\.accel_samples=[1-9][0-9]*' <<<"${metrics_line}" &&
    grep -Eq 'imu\.gyro_samples=[1-9][0-9]*' <<<"${metrics_line}"; then
    imu_ok=true
fi

if [[ "${mode}" == "full" ]]; then
    if [[ "${imu_ok}" != true ]]; then
        echo "IMU ACCEPTANCE FAILED: latest metrics do not prove fresh accelerometer and" \
            "gyroscope data (mandatory for EKF and full acceptance):" >&2
        printf '%s\n' "${metrics_line:-<no metrics line>}" >&2
        echo "RGB/depth remain live and independent; see docs/operations.md for the IMU checklist." >&2
        exit 3
    fi
    echo "Full verification passed: RGB, depth, accel, gyro and H.264 are live."
elif [[ "${imu_ok}" == true ]]; then
    echo "IMU/EKF readiness is also healthy."
else
    echo "IMU/EKF is not ready (mandatory for full acceptance, independent of streaming)." >&2
fi
