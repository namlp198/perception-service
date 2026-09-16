# Setup and build

## Target

- NVIDIA Jetson Orin
- Ubuntu 22.04
- GCC 11 or a compatible C++20 compiler
- CMake 3.22 or newer
- Intel RealSense D435i

V0.1 production dependencies are librealsense2, OpenCV, Eigen3, yaml-cpp, spdlog, GoogleTest,
GStreamer and gst-rtsp-server. Jetson deployments also require NVIDIA GStreamer plugins containing
`nvvidconv` and `nvv4l2h264enc`.

Dependency installation varies with the JetPack/librealsense combination. `scripts/setup.sh` performs
a non-mutating prerequisite check and prints missing components; it intentionally does not add package
repositories or install software.

Install Ubuntu packages interactively with:

```bash
sudo ./scripts/install_dependencies.sh
```

The script deliberately leaves librealsense2 and NVIDIA multimedia to the JetPack-compatible packages
already installed on the target.

## Presets

- `debug`: portable host build, hardware backends disabled.
- `jetson-debug`: debug build with RealSense, GStreamer and OpenCV viewer enabled.
- `release`: optimized Jetson build with the same hardware features.
- `jetson-local`: fallback build using packages extracted under `.runtime/sysroot` when administrative
  package installation is unavailable. This is useful for validation, but system packages remain the
  preferred production deployment.

Build output is isolated under `build/<preset>`. No configure step downloads source code.

The default D435i profile enables accelerometer at 100 Hz and gyroscope at 200 Hz with bounded
512-sample queues. On Linux, the service user must have permission to open both the UVC video nodes
and the D435i motion/IIO interfaces. Use `camera-info` to list the exact profiles exposed by the
installed firmware/librealsense combination before changing these rates.

## Run

```bash
./scripts/build.sh debug
./scripts/test.sh debug
./scripts/run.sh config/default.yaml
```

The production service will fail fast until required backends and the remaining V0.1 milestones are
available. It never reports a missing camera or encoder as healthy.

## Development-PC deploy prerequisites

Linux and WSL deployment hosts need `rsync` and the OpenSSH client. On Ubuntu or WSL Ubuntu:

```bash
sudo apt update
sudo apt install rsync openssh-client
```

Configure SSH public-key authentication for `percpc@192.168.1.220`, then verify it without a
password prompt before using the non-interactive deploy script:

```bash
ssh -o BatchMode=yes percpc@192.168.1.220 true
```

Windows PowerShell uses `scripts/jetson/deploy.ps1`, which delegates to WSL so source transfer keeps
rsync's incremental update and exclude semantics.
