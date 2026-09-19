#!/usr/bin/env bash
set -euo pipefail

# Records a replay dataset from the D435i. One process owns the camera, so the running service must
# release it first: running `dataset-record` while the service streams fails with
# `failed to claim usb interface, interface 0, is busy`. This stops the service, records, restarts
# it and runs the strict verification. RGB/depth are interrupted for the duration.
if [[ "${1:-}" != "--confirm-service-interruption" ]]; then
    echo "Usage: $0 --confirm-service-interruption <output-directory> [seconds] [preset]" >&2
    echo "A bare name records into the project's datasets/ directory; an absolute path is used" >&2
    echo "as given, for recording onto external storage." >&2
    echo "Example: $0 --confirm-service-interruption 2026-09-19-yard 30" >&2
    exit 2
fi

output_directory="${2:-}"
seconds="${3:-30}"
preset="${4:-jetson-local}"
if [[ -z "${output_directory}" ]]; then
    echo "An output directory is required." >&2
    exit 2
fi

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

# A bare name lands in the project's own datasets/ directory, which is already excluded from Git
# and from the rsync deploy. Absolute paths are used as given, for recording onto external storage.
if [[ "${output_directory}" != /* ]]; then
    output_directory="${project_root}/datasets/${output_directory}"
fi

if [[ -e "${output_directory}/index.jsonl" ]]; then
    # The recorder refuses to overwrite too; failing here keeps the service untouched.
    echo "A dataset already exists at ${output_directory}; choose another directory." >&2
    exit 2
fi

# Check writability before stopping the service: a permission error must not cost RTSP downtime.
parent_directory="$(dirname "${output_directory}")"
existing_ancestor="${parent_directory}"
while [[ ! -d "${existing_ancestor}" && "${existing_ancestor}" != "/" ]]; do
    existing_ancestor="$(dirname "${existing_ancestor}")"
done
if [[ ! -w "${existing_ancestor}" ]]; then
    echo "Cannot create ${output_directory}: the nearest existing parent ${existing_ancestor} is not writable." >&2
    echo "A bare name such as '2026-09-19-yard' records into ${project_root}/datasets/ instead." >&2
    exit 2
fi

# Measured on the Jetson at 640x480 RGB + depth, 30 fps: ~1.5 MB per frame set, ~45 MB/s. That
# fills a root filesystem in minutes, so check before stopping the service rather than after.
estimated_mb=$(( seconds * 45 * 12 / 10 ))
available_mb="$(df --output=avail -m "${existing_ancestor}" 2>/dev/null | tail -n 1 | tr -d ' ')"
if [[ "${available_mb}" =~ ^[0-9]+$ && "${available_mb}" -lt "${estimated_mb}" ]]; then
    echo "Not enough space on ${existing_ancestor}: ${available_mb} MB free, about ${estimated_mb} MB" >&2
    echo "needed for ${seconds}s (roughly 45 MB/s at 640x480 RGB + depth, 30 fps)." >&2
    exit 2
fi

service_name="perception-service-user.service"
binary="${project_root}/build/${preset}/dataset-record"
local_library_path="${project_root}/.runtime/sysroot/usr/lib/aarch64-linux-gnu"

if [[ -d "${local_library_path}" ]]; then
    export LD_LIBRARY_PATH="/usr/lib/aarch64-linux-gnu:${local_library_path}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi
if [[ ! -x "${binary}" ]]; then
    echo "Missing executable: ${binary}" >&2
    exit 1
fi

mkdir -p "$(dirname "${output_directory}")"

was_active=0
if systemctl --user is-active --quiet "${service_name}"; then
    was_active=1
fi

restore_service() {
    if [[ "${was_active}" -eq 1 ]]; then
        echo "Restarting ${service_name}..."
        systemctl --user start "${service_name}" || true
        sleep 5
        bash "${project_root}/scripts/jetson/verify-running-service.sh" || true
    fi
}
# Restore streaming even when the recording fails or the operator interrupts it.
trap restore_service EXIT

if [[ "${was_active}" -eq 1 ]]; then
    echo "Stopping ${service_name}; RTSP will be unavailable for about ${seconds}s..."
    systemctl --user stop "${service_name}"
    sleep 2
fi

echo "Recording ${seconds}s to ${output_directory}..."
record_status=0
"${binary}" "${project_root}/config/default.yaml" "${output_directory}" "${seconds}" || record_status=$?

if [[ "${record_status}" -ne 0 ]]; then
    echo "Recording failed with exit code ${record_status}." >&2
    exit "${record_status}"
fi

echo "Dataset written to ${output_directory}:"
ls -la "${output_directory}"
