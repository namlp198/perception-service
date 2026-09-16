#!/usr/bin/env bash
set -euo pipefail

# Exit codes: 0 restarted and fully accepted; 1 restart or RGB/depth streaming failed;
# 3 restarted with RGB/depth live but the mandatory IMU acceptance unmet (service left running).
service_name="${1:-perception-service-user.service}"
if [[ ! "${service_name}" =~ ^[A-Za-z0-9@_.-]+\.service$ ]]; then
    echo "Unsafe systemd user service name: ${service_name}" >&2
    exit 2
fi

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
unit_source="${project_root}/deploy/systemd/perception-service-user.service"
unit_target="${HOME}/.config/systemd/user/perception-service-user.service"

bash "${project_root}/scripts/jetson/validate-rtsp-encoder.sh"
install -D -m 0644 "${unit_source}" "${unit_target}"
systemctl --user daemon-reload

systemctl --user restart "${service_name}"
sleep 5
systemctl --user --no-pager --full status "${service_name}"

set +e
bash "${project_root}/scripts/jetson/verify-running-service.sh" full
verify_status=$?
set -e
case "${verify_status}" in
    0)
        ;;
    3)
        echo "Service restarted; RGB/depth RTSP are live; IMU acceptance FAILED (mandatory)." >&2
        exit 3
        ;;
    *)
        echo "Service restart completed, but RGB/depth streaming verification failed." >&2
        exit 1
        ;;
esac
