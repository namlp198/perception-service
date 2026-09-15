#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
    echo "Run this script interactively with sudo: sudo ./scripts/install_dependencies.sh" >&2
    exit 1
fi

apt-get update
apt-get install -y \
    build-essential \
    cmake \
    pkg-config \
    libeigen3-dev \
    libgtest-dev \
    libopencv-dev \
    libspdlog-dev \
    libyaml-cpp-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstrtspserver-1.0-dev \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly

echo "System dependencies installed. librealsense2 and Jetson multimedia remain platform packages."
