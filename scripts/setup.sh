#!/usr/bin/env bash
set -euo pipefail

required_commands=(cmake c++ git pkg-config)
missing=0

for command_name in "${required_commands[@]}"; do
    if ! command -v "${command_name}" >/dev/null 2>&1; then
        echo "missing command: ${command_name}" >&2
        missing=1
    fi
done

required_modules=(opencv4 realsense2 gstreamer-1.0 gstreamer-app-1.0 gstreamer-rtsp-server-1.0)
for module_name in "${required_modules[@]}"; do
    if ! pkg-config --exists "${module_name}" 2>/dev/null; then
        echo "missing pkg-config module: ${module_name}" >&2
        missing=1
    fi
done

required_cmake_packages=(Eigen3 yaml-cpp spdlog GTest)
for package_name in "${required_cmake_packages[@]}"; do
    if ! cmake --find-package -DNAME="${package_name}" -DCOMPILER_ID=GNU -DLANGUAGE=CXX \
        -DMODE=EXIST >/dev/null 2>&1; then
        echo "missing CMake package: ${package_name}" >&2
        missing=1
    fi
done

if ! bash "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/jetson/validate-rtsp-encoder.sh"; then
    echo "Jetson Orin Nano H.264 encoder preflight failed" >&2
    missing=1
fi

if ((missing != 0)); then
    echo "Prerequisite check failed. See docs/setup.md; no packages were changed." >&2
    exit 1
fi

echo "All declared Jetson V0.1 prerequisites were found."
