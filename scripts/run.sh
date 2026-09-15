#!/usr/bin/env bash
set -euo pipefail

config_path="${1:-config/default.yaml}"
build_preset="${PERCEPTION_BUILD_PRESET:-jetson-debug}"
exec "build/${build_preset}/perception-service" "${config_path}"
