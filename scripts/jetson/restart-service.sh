#!/usr/bin/env bash
set -euo pipefail

service_name="${1:-perception-service-user.service}"
if [[ ! "${service_name}" =~ ^[A-Za-z0-9@_.-]+\.service$ ]]; then
    echo "Unsafe systemd user service name: ${service_name}" >&2
    exit 2
fi

systemctl --user restart "${service_name}"
systemctl --user --no-pager --full status "${service_name}"
