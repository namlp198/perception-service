#!/usr/bin/env bash
set -euo pipefail

preset="${1:-jetson-local}"
run_tests=false
if (($# > 2)); then
    echo "Usage: $0 [preset] [--test]" >&2
    exit 2
fi
if [[ "${2:-}" == "--test" ]]; then
    run_tests=true
elif (($# > 1)); then
    echo "Usage: $0 [preset] [--test]" >&2
    exit 2
fi

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${project_root}"

cmake --preset "${preset}"
cmake --build --preset "${preset}"

if [[ "${run_tests}" == true ]]; then
    ctest --preset "${preset}"
fi
