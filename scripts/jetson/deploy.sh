#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/../.." && pwd)"

jetson_host="${PERCEPTION_JETSON_HOST:-192.168.1.220}"
jetson_user="${PERCEPTION_JETSON_USER:-percpc}"
remote_dir="${PERCEPTION_REMOTE_DIR:-/home/percpc/workspace/perception-service}"
ssh_port="${PERCEPTION_SSH_PORT:-22}"
identity_file="${PERCEPTION_SSH_IDENTITY:-}"
build_preset="${PERCEPTION_BUILD_PRESET:-jetson-local}"
delete_remote=false
dry_run=false
run_build=false
run_tests=false
restart_service=false
verbose=false
interactive_auth=false

usage() {
    cat <<'EOF'
Usage: ./scripts/jetson/deploy.sh [options]

Synchronize perception-service source to the Jetson over SSH. By default this
only updates source files; the existing remote build and .runtime directories
are preserved.

Options:
  --host HOST          Jetson host (default: 192.168.1.220)
  --user USER          SSH user (default: percpc)
  --remote-dir PATH    Remote project directory
  --port PORT          SSH port (default: 22)
  --identity PATH      SSH private key
  --preset PRESET      Remote CMake preset (default: jetson-local)
  --delete             Delete stale, non-excluded files on the remote target
  --dry-run            Show the rsync changes without transferring files
  --build              Build on the Jetson after synchronization
  --test               Run CTest after build (implies --build)
  --restart            Restart perception-service-user.service after success
  --verbose            Show detailed rsync statistics and transfer progress
  --interactive-auth   Allow an SSH password/passphrase prompt (one prompt)
  -h, --help           Show this help

Environment variables matching the defaults are also supported:
PERCEPTION_JETSON_HOST, PERCEPTION_JETSON_USER, PERCEPTION_REMOTE_DIR,
PERCEPTION_SSH_PORT, PERCEPTION_SSH_IDENTITY and PERCEPTION_BUILD_PRESET.
EOF
}

while (($# > 0)); do
    case "$1" in
        --host) jetson_host="${2:?--host requires a value}"; shift 2 ;;
        --user) jetson_user="${2:?--user requires a value}"; shift 2 ;;
        --remote-dir) remote_dir="${2:?--remote-dir requires a value}"; shift 2 ;;
        --port) ssh_port="${2:?--port requires a value}"; shift 2 ;;
        --identity) identity_file="${2:?--identity requires a value}"; shift 2 ;;
        --preset) build_preset="${2:?--preset requires a value}"; shift 2 ;;
        --delete) delete_remote=true; shift ;;
        --dry-run) dry_run=true; shift ;;
        --build) run_build=true; shift ;;
        --test) run_tests=true; run_build=true; shift ;;
        --restart) restart_service=true; shift ;;
        --verbose) verbose=true; shift ;;
        --interactive-auth) interactive_auth=true; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
done

for command_name in rsync ssh; do
    if ! command -v "${command_name}" >/dev/null 2>&1; then
        echo "Required command is not installed: ${command_name}" >&2
        exit 1
    fi
done

if [[ ! "${jetson_host}" =~ ^[A-Za-z0-9._:-]+$ ]]; then
    echo "Unsafe Jetson host: ${jetson_host}" >&2
    exit 2
fi
if [[ ! "${jetson_user}" =~ ^[A-Za-z_][A-Za-z0-9_-]*$ ]]; then
    echo "Unsafe SSH user: ${jetson_user}" >&2
    exit 2
fi
if [[ ! "${remote_dir}" =~ ^/[A-Za-z0-9._/-]+$ || "${remote_dir}" == "/" ]]; then
    echo "Remote directory must be a safe absolute path other than '/': ${remote_dir}" >&2
    exit 2
fi
if [[ ! "${ssh_port}" =~ ^[0-9]+$ ]] || ((ssh_port < 1 || ssh_port > 65535)); then
    echo "Invalid SSH port: ${ssh_port}" >&2
    exit 2
fi
if [[ ! "${build_preset}" =~ ^[A-Za-z0-9._-]+$ ]]; then
    echo "Unsafe CMake preset: ${build_preset}" >&2
    exit 2
fi
if [[ -n "${identity_file}" && ! -f "${identity_file}" ]]; then
    echo "SSH identity file does not exist: ${identity_file}" >&2
    exit 2
fi

ssh_options=(-p "${ssh_port}")
control_connection_open=false
if [[ "${interactive_auth}" == true ]]; then
    control_path="/tmp/perception-deploy-${UID}-$$-%C"
    ssh_options+=(
        -o BatchMode=no
        -o ControlMaster=auto
        -o ControlPersist=60
        -o "ControlPath=${control_path}"
    )
else
    ssh_options+=(-o BatchMode=yes)
fi
if [[ -n "${identity_file}" ]]; then
    ssh_options+=(-i "${identity_file}")
fi
ssh_target="${jetson_user}@${jetson_host}"

close_control_connection() {
    if [[ "${control_connection_open}" == true ]]; then
        ssh "${ssh_options[@]}" -O exit "${ssh_target}" >/dev/null 2>&1 || true
    fi
}
trap close_control_connection EXIT

echo "Deployment settings:"
echo "  source:      ${project_root}/"
echo "  destination: ${ssh_target}:${remote_dir}/"
echo "  SSH port:    ${ssh_port}"
echo "  preset:      ${build_preset}"
echo "  dry run:     ${dry_run}"
echo "  delete:      ${delete_remote}"
echo "  build:       ${run_build}"
echo "  test:        ${run_tests}"
echo "  restart:     ${restart_service}"
echo "  auth mode:   $([[ "${interactive_auth}" == true ]] && echo interactive || echo key-only)"

echo "Checking SSH access to ${ssh_target}:${ssh_port}..."
ssh "${ssh_options[@]}" "${ssh_target}" true
if [[ "${interactive_auth}" == true ]]; then
    control_connection_open=true
fi

if [[ "${dry_run}" == false ]]; then
    ssh "${ssh_options[@]}" "${ssh_target}" "mkdir -p '${remote_dir}'"
fi

printf -v rsync_shell 'ssh'
for option in "${ssh_options[@]}"; do
    printf -v quoted_option '%q' "${option}"
    rsync_shell+=" ${quoted_option}"
done

rsync_options=(
    --archive
    --compress
    --human-readable
    --itemize-changes
    --exclude-from="${script_dir}/rsync-exclude.txt"
)
if [[ "${verbose}" == true ]]; then
    rsync_options+=(--verbose --stats --info=progress2)
fi
if [[ "${delete_remote}" == true ]]; then
    rsync_options+=(--delete)
fi
if [[ "${dry_run}" == true ]]; then
    rsync_options+=(--dry-run)
fi

echo "Synchronizing ${project_root}/ -> ${ssh_target}:${remote_dir}/"
RSYNC_RSH="${rsync_shell}" rsync "${rsync_options[@]}" \
    "${project_root}/" "${ssh_target}:${remote_dir}/"

if [[ "${dry_run}" == true ]]; then
    echo "Dry run complete; build, test and restart steps were skipped."
    exit 0
fi

if [[ "${run_build}" == true ]]; then
    remote_build_args=(bash "${remote_dir}/scripts/jetson/build.sh" "${build_preset}")
    if [[ "${run_tests}" == true ]]; then
        remote_build_args+=(--test)
    fi
    printf -v remote_build_command '%q ' "${remote_build_args[@]}"
    echo "Building preset ${build_preset} on ${ssh_target}..."
    ssh "${ssh_options[@]}" "${ssh_target}" "${remote_build_command% }"
fi

if [[ "${restart_service}" == true ]]; then
    echo "Restarting perception-service-user.service on ${ssh_target}..."
    set +e
    ssh "${ssh_options[@]}" "${ssh_target}" \
        "bash '${remote_dir}/scripts/jetson/restart-service.sh'"
    restart_status=$?
    set -e
    if [[ "${restart_status}" -eq 3 ]]; then
        # Sync, build, restart and RGB/depth streaming succeeded; only the mandatory IMU
        # acceptance is unmet. Report it distinctly instead of as a generic failure.
        echo "Deployment completed with IMU ACCEPTANCE FAILED: service restarted, RGB/depth" \
            "RTSP live, accel/gyro data missing (mandatory before EKF/mission use)." >&2
        exit 3
    elif [[ "${restart_status}" -ne 0 ]]; then
        echo "Deployment FAILED: service restart or RGB/depth verification failed" \
            "(exit ${restart_status})." >&2
        exit "${restart_status}"
    fi
fi

echo "Deployment completed successfully."
