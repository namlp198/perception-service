# Setup and build

## Target

- NVIDIA Jetson Orin Nano
- Ubuntu 22.04
- GCC 11 or a compatible C++20 compiler
- CMake 3.22 or newer
- Intel RealSense D435i

V0.1 production dependencies are librealsense2, OpenCV, Eigen3, yaml-cpp, spdlog, GoogleTest,
GStreamer and gst-rtsp-server. The Orin Nano deployment requires `x264enc`; it does not expose
`nvv4l2h264enc` because the module has no hardware video encoder.

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
512-sample queues, a two-second startup proof deadline, a one-second runtime liveness deadline and a
30-second bounded Motion Module restart cadence (`restart_interval_ms`, `0` disables) that never
touches the RGB/depth pipeline.
On Linux, the service user must have permission to open both the UVC video nodes
and the D435i motion/IIO interfaces. Use `camera-info` to list the exact profiles exposed by the
installed firmware/librealsense combination before changing these rates.

The `jetson-local` user service searches `/usr/lib/aarch64-linux-gnu` before its extracted sysroot and
does not override GStreamer's system plugin path. Validate plugin loading and a real encode before
service startup:

```bash
./scripts/jetson/validate-rtsp-encoder.sh
```

## Run

```bash
./scripts/build.sh debug
./scripts/test.sh debug
./scripts/run.sh config/default.yaml
```

The production service will fail fast until required backends and the remaining V0.1 milestones are
available. It never reports a missing camera or encoder as healthy.

## Development-PC deploy prerequisites

## Building from Windows through WSL

The Windows MinGW toolchains shipped with IDEs have been unreliable here (CLion's GCC 13.1 cannot
spawn `cc1plus` in some shells), and they cannot compile the POSIX transport sockets at all. Building
in WSL is the supported way to get a full host build from a Windows development PC:

```bash
sudo apt-get install -y g++ cmake make libspdlog-dev libyaml-cpp-dev libgtest-dev libeigen3-dev
```

Install the optional dependencies too: without them the build silently drops the spdlog logging
paths, the YAML configuration loader and the GoogleTest harness, so a green build proves much less.
Then, from Windows PowerShell:

```powershell
wsl bash -lc "cd /mnt/d/<path>/perception-service && cmake -S . -B ~/psbuild -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON && cmake --build ~/psbuild -j4 && ctest --test-dir ~/psbuild --output-on-failure"
```

Build outside `/mnt` (as above) because WSL's `/tmp` is cleared when the distribution restarts and
`/mnt` paths are slow.

**A Debug build alone does not predict the Jetson build.** The Jetson preset is `RelWithDebInfo`, and
at `-O2` glibc enables `_FORTIFY_SOURCE`, which marks `read`/`write` and friends
`warn_unused_result`. A `(void)` cast does not satisfy that attribute, so calls that compile cleanly
at `-O0` become `-Werror=unused-result` failures on the Jetson. This cost one deploy round on
2026-09-19. Always repeat the build in the Jetson's configuration:

```bash
cmake -S . -B ~/psrel -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON && cmake --build ~/psrel -j4
```

Installing `g++-11` additionally lets the host match the Jetson's compiler generation. Distro
spdlog/yaml-cpp cannot be *linked* against it (they are built with the newer libstdc++), but
compiling every portable translation unit with `g++-11 -O2` and the project's warning flags
reproduces the Jetson's warning surface, which is where these failures appear.

Even then the host build does not prove the Jetson build: librealsense, GStreamer and OpenCV code
paths only compile there.

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
