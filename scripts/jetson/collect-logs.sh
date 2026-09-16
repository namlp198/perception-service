#!/usr/bin/env bash
set -euo pipefail

line_count="${1:-200}"
if [[ ! "${line_count}" =~ ^[1-9][0-9]*$ ]] || ((line_count > 5000)); then
    echo "Log line count must be between 1 and 5000." >&2
    exit 2
fi

journalctl --user-unit=perception-service-user.service -n "${line_count}" --no-pager
